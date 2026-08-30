// SPDX-License-Identifier: GPL-2.0-only
//
// Direct SystemVerilog descendant of MiSTer-devel/GBA_MiSTer's gba_cpu.vhd,
// commit 93790a023395bbd90e5eaf4dfb2cb5910afd55f5. This file preserves its
// ARM/Thumb register, ALU, shifter, multiplier, transfer, bank, and exception
// semantics while replacing the GBA bus and timing machinery.
//
// Copyright (C) 2016-2026 Robert Peip and GBA_MiSTer contributors
// Copyright (C) 2026 Jamie Blanks
//
// This program is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License version 2 as published by
// the Free Software Foundation.

module arm7tdmi_core
	import arm7tdmi_pkg::*;
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce,

	input  logic        irq_n,
	input  logic        fiq_n,

	input  logic        halt_req,
	output logic        halted,

	output logic        mem_req,
	input  logic        mem_ready,
	input  logic        mem_abort,
	output logic [31:0] mem_addr,
	output logic        mem_write,
	output logic [31:0] mem_wdata,
	input  logic [31:0] mem_rdata,
	output logic  [1:0] mem_size,
	output logic  [3:0] mem_wstrb,
	output logic        mem_seq,
	output logic        mem_fetch,
	output logic        mem_privileged,
	output logic        mem_lock,

	output logic        retire,

	input  logic        state_req,
	input  logic        state_write,
	input  logic  [5:0] state_index,
	input  logic [31:0] state_wdata,
	output logic [31:0] state_rdata,
	output logic        state_ready,
	input  logic        state_commit
);

	typedef enum logic [3:0] {
		RUN,
		MEM_ACCESS,
		SWP_READ,
		SWP_WRITE,
		BLOCK_ACCESS,
		MUL_WAIT,
		HALTED
	} core_state_e;

	// ARM data-processing opcodes. Thumb forms are decoded onto the same set so
	// one ALU serves both instruction sets.
	localparam logic [3:0] ALU_AND = 4'h0;
	localparam logic [3:0] ALU_EOR = 4'h1;
	localparam logic [3:0] ALU_SUB = 4'h2;
	localparam logic [3:0] ALU_RSB = 4'h3;
	localparam logic [3:0] ALU_ADD = 4'h4;
	localparam logic [3:0] ALU_ADC = 4'h5;
	localparam logic [3:0] ALU_SBC = 4'h6;
	localparam logic [3:0] ALU_RSC = 4'h7;
	localparam logic [3:0] ALU_TST = 4'h8;
	localparam logic [3:0] ALU_TEQ = 4'h9;
	localparam logic [3:0] ALU_CMP = 4'ha;
	localparam logic [3:0] ALU_CMN = 4'hb;
	localparam logic [3:0] ALU_ORR = 4'hc;
	localparam logic [3:0] ALU_MOV = 4'hd;
	localparam logic [3:0] ALU_BIC = 4'he;
	localparam logic [3:0] ALU_MVN = 4'hf;

	// Operand sources. Decode chooses a source; the mux that acts on the choice
	// is flat, so a forwarded value reaches the shifter through two levels
	// instead of the depth of the decode chain.
	localparam logic [3:0] SRC_ZERO     = 4'd0;
	localparam logic [3:0] SRC_IMM      = 4'd1;
	localparam logic [3:0] SRC_ARM_RN   = 4'd2;
	localparam logic [3:0] SRC_ARM_RD   = 4'd3;
	localparam logic [3:0] SRC_ARM_RM   = 4'd4;
	localparam logic [3:0] SRC_RM_SHIFT = 4'd5;
	localparam logic [3:0] SRC_T_R0     = 4'd6;
	localparam logic [3:0] SRC_T_R3     = 4'd7;
	localparam logic [3:0] SRC_T_R6     = 4'd8;
	localparam logic [3:0] SRC_T_R8     = 4'd9;
	localparam logic [3:0] SRC_T_HD     = 4'd10;
	localparam logic [3:0] SRC_T_HS     = 4'd11;
	localparam logic [3:0] SRC_SP       = 4'd12;
	localparam logic [3:0] SRC_LR       = 4'd13;
	localparam logic [3:0] SRC_PC_ALIGN = 4'd14;

	localparam logic [1:0] AMT_IMM   = 2'd0;
	localparam logic [1:0] AMT_ARM_RS = 2'd1;
	localparam logic [1:0] AMT_T_R3  = 2'd2;

	typedef enum logic [3:0] {
		EXEC_SIMPLE,
		EXEC_BRANCH,
		EXEC_MEMORY,
		EXEC_BLOCK,
		EXEC_MULTIPLY,
		EXEC_SWP,
		EXEC_SWI,
		EXEC_UNDEFINED
	} exec_kind_e;

	logic [31:0] regs [0:14];
	logic [31:0] usr_r8_14 [0:6];
	logic [31:0] fiq_r8_14 [0:6];
	logic [31:0] irq_r13_14 [0:1];
	logic [31:0] svc_r13_14 [0:1];
	logic [31:0] abt_r13_14 [0:1];
	logic [31:0] und_r13_14 [0:1];

	logic [31:0] spsr_fiq;
	logic [31:0] spsr_irq;
	logic [31:0] spsr_svc;
	logic [31:0] spsr_abt;
	logic [31:0] spsr_und;
	logic [31:0] cpsr;

	core_state_e state;
	logic        decode_valid;
	logic [31:0] decode_instruction;
	logic [31:0] decode_pc;
	logic        prefetch_valid;
	logic [31:0] prefetch_instruction;
	logic [31:0] prefetch_pc;
	logic        ahead_valid;
	logic [31:0] ahead_instruction;
	logic [31:0] ahead_pc;
	// synthesis translate_off
	/* verilator lint_off UNUSEDSIGNAL */
	logic [31:0] trace_retire_pc;
	logic [31:0] trace_retire_instruction;
	logic        trace_retire_thumb;
	logic        trace_retire_exception;
	logic        trace_retire_exception_return;
	/* verilator lint_on UNUSEDSIGNAL */
	// synthesis translate_on
	logic [31:0] next_fetch_pc;

	logic [31:0] halt_pc;
	logic [31:0] state_pc;
	logic [31:0] state_cpsr;
	logic        state_image_valid;
	logic        state_dirty;
	logic        halt_capture_pending;
	logic  [2:0] boundary_event;
	logic [31:0] boundary_pc;

	localparam logic [2:0] BOUNDARY_NONE = 3'd0;
	localparam logic [2:0] BOUNDARY_HALT = 3'd1;
	localparam logic [2:0] BOUNDARY_FIQ  = 3'd2;
	localparam logic [2:0] BOUNDARY_IRQ  = 3'd3;
	localparam logic [2:0] BOUNDARY_PABT = 3'd4;

	logic  [1:0] data_address;
	logic [31:0] data_writeback_value;
	logic  [3:0] data_rd;
	logic  [3:0] data_rn;
	logic  [1:0] data_size;
	logic        data_load;
	logic        data_signed;
	logic        data_writeback;
	logic        data_load_pc;
	logic [31:0] data_resume_pc;

	logic [31:0] swp_store_value;
	logic [31:0] swp_loaded_value;
	logic  [3:0] swp_rd;
	logic        swp_byte;
	logic [31:0] swp_resume_pc;

	logic [15:0] block_list;
	logic [31:0] block_address;
	logic [31:0] block_writeback_value;
	logic [31:0] block_resume_pc;
	logic  [3:0] block_rn;
	logic  [4:0] block_index;
	logic        block_load;
	logic        block_writeback;
	logic        block_user;
	logic        block_restore_cpsr;
	logic [31:0] block_pc_value;

	logic [63:0] multiply_result;
	logic [63:0] multiply_calculated_result;
	logic  [3:0] multiply_rd_lo;
	logic  [3:0] multiply_rd_hi;
	logic  [2:0] multiply_wait;
	logic        multiply_long;
	logic        multiply_set_flags;
	logic [31:0] multiply_resume_pc;
	logic        internal_cycle_pending;

	exec_kind_e dec_kind;
	logic        dec_condition;
	logic        dec_write_reg;
	logic  [3:0] dec_rd;
	logic [31:0] dec_result;
	logic        dec_write_flags;
	logic [31:0] dec_flag_mask;
	logic [31:0] dec_flag_value;
	logic        dec_restore_spsr;
	logic        dec_write_psr;
	logic        dec_write_spsr;
	logic [31:0] dec_psr_mask;
	logic [31:0] dec_psr_value;
	logic [31:0] dec_branch_target;
	logic        dec_branch_thumb;
	logic        dec_link;
	logic [31:0] dec_link_value;

	logic        dec_mem_preindex;
	logic [31:0] dec_mem_write_value;
	logic  [3:0] dec_mem_rd;
	logic  [3:0] dec_mem_rn;
	logic  [1:0] dec_mem_size;
	logic        dec_mem_load;
	logic        dec_mem_signed;
	logic        dec_mem_writeback;
	logic        dec_mem_load_pc;

	logic [15:0] dec_block_list;
	logic  [6:0] dec_block_addr_delta;
	logic        dec_block_addr_add;
	logic  [3:0] dec_block_rn;
	logic        dec_block_load;
	logic        dec_block_writeback;
	logic        dec_block_user;
	logic        dec_block_restore_cpsr;

	logic [31:0] dec_multiply_operand_a;
	logic [31:0] dec_multiply_operand_b;
	logic [63:0] dec_multiply_accumulator;
	logic  [3:0] dec_multiply_rd_lo;
	logic  [3:0] dec_multiply_rd_hi;
	logic  [1:0] dec_multiply_extra;
	logic        dec_multiply_long;
	logic        dec_multiply_signed;
	logic        dec_multiply_accumulate;
	logic        dec_multiply_set_flags;
	logic        dec_internal_cycle;
	logic        dec_dp;
	logic  [3:0] dec_op1_sel;
	logic  [3:0] dec_op2_sel;
	logic [31:0] dec_op2_imm;
	logic  [1:0] dec_amount_sel;
	logic  [7:0] dec_amount_imm;
	logic [31:0] dec_dp_op1;
	logic [31:0] dec_dp_op2;
	logic  [7:0] dec_dp_shift_amount;
	logic  [1:0] dec_dp_shift_type;
	logic        dec_dp_shift_enable;
	logic        dec_dp_shift_rrx;
	logic        dec_dp_shift_carry;
	logic  [3:0] dec_dp_opcode;
	logic [31:0] dec_dp_cpsr;
	logic [31:0] dec_dp_spsr;

	logic        dec_swp_byte;
	logic [31:0] dec_swp_store_value;

	exec_kind_e exec_kind;
	logic        exec_condition;
	logic        exec_write_reg;
	logic  [3:0] exec_rd;
	logic [31:0] exec_result;
	logic        exec_write_flags;
	logic [31:0] exec_flag_mask;
	logic        exec_restore_spsr;
	logic        exec_write_psr;
	logic        exec_write_spsr;
	logic [31:0] exec_psr_mask;
	logic [31:0] exec_psr_value;
	logic [31:0] exec_post_cpsr;
	logic [31:0] exec_branch_target;
	logic        exec_branch_thumb;
	logic        exec_link;
	logic [31:0] exec_link_value;

	logic        exec_mem_preindex;
	logic [31:0] exec_mem_write_value;
	logic  [3:0] exec_mem_rd;
	logic  [3:0] exec_mem_rn;
	logic  [1:0] exec_mem_size;
	logic        exec_mem_load;
	logic        exec_mem_signed;
	logic        exec_mem_writeback;
	logic        exec_mem_load_pc;

	logic [15:0] exec_block_list;
	logic  [6:0] exec_block_addr_delta;
	logic        exec_block_addr_add;
	logic  [3:0] exec_block_rn;
	logic  [4:0] exec_block_first_index;
	logic        exec_block_load;
	logic        exec_block_writeback;
	logic        exec_block_user;
	logic        exec_block_restore_cpsr;

	logic [31:0] exec_multiply_operand_a;
	logic [31:0] exec_multiply_operand_b;
	logic [63:0] exec_multiply_accumulator;
	logic  [3:0] exec_multiply_rd_lo;
	logic  [3:0] exec_multiply_rd_hi;
	logic  [1:0] exec_multiply_extra;
	logic        exec_multiply_long;
	logic        exec_multiply_signed;
	logic        exec_multiply_accumulate;
	logic        exec_multiply_set_flags;
	logic        exec_internal_cycle;
	logic        exec_dp;
	logic [31:0] exec_dp_op1;
	logic [31:0] exec_dp_op2;
	logic  [7:0] exec_dp_shift_amount;
	logic  [1:0] exec_dp_shift_type;
	logic        exec_dp_shift_enable;
	logic        exec_dp_shift_rrx;
	logic        exec_dp_shift_carry;
	logic  [3:0] exec_dp_opcode;
	logic [31:0] exec_dp_cpsr;
	logic [31:0] exec_dp_spsr;

	logic        exec_swp_byte;
	logic [31:0] exec_swp_store_value;

	logic [31:0] current_spsr;
	logic [31:0] visible_pc;
	logic        thumb;
	logic [31:0] decode_cpsr;
	logic [31:0] decode_spsr;
	logic [31:0] decode_visible_pc;
	logic [31:0] decode_visible_pc_p4;
	logic        decode_thumb;
	logic        decode_forward_0_valid;
	logic  [3:0] decode_forward_0_index;
	logic [31:0] decode_forward_0_value;
	logic        decode_forward_1_valid;
	logic  [3:0] decode_forward_1_index;
	logic [31:0] decode_forward_1_value;
	logic  [9:0] decode_forward_tags;
	logic [63:0] decode_forward_values;
	logic [31:0] dp_result;
	logic [31:0] dp_post_cpsr;
	logic [31:0] dp_mem_address;
	logic [31:0] dp_mem_updated_base;
	logic [31:0] dp_block_address;
	logic [31:0] dp_block_updated_base;
	logic        exec_simple_control;
	logic  [2:0] dp_multiply_wait;
	logic [31:0] exec_effective_result;
	logic [31:0] exec_effective_post_cpsr;
	logic [31:0] exec_effective_branch_target;
	logic [31:0] dec_post_cpsr;
	logic [15:0] block_remaining;
	logic  [4:0] block_next_index;
	logic  [4:0] dec_block_first_index;
	logic [31:0] data_loaded_value;
	logic [31:0] multiply_post_cpsr;
	logic  [2:0] state_user_index;
	logic  [2:0] state_fiq_index;

	assign thumb = |(cpsr & CPSR_T);
	assign visible_pc = decode_pc + (thumb ? 32'd4 : 32'd8);
	assign decode_thumb = |(decode_cpsr & CPSR_T);
	assign decode_visible_pc = prefetch_pc + (decode_thumb ? 32'd4 : 32'd8);
	assign decode_visible_pc_p4 = prefetch_pc + (decode_thumb ? 32'd8 : 32'd12);
	assign decode_forward_tags = {decode_forward_0_valid, decode_forward_0_index,
		decode_forward_1_valid, decode_forward_1_index};
	assign decode_forward_values = {decode_forward_0_value, decode_forward_1_value};

	function automatic logic [31:0] read_reg(input logic [3:0] index);
		begin
			case (index)
				4'd0: read_reg = regs[0];
				4'd1: read_reg = regs[1];
				4'd2: read_reg = regs[2];
				4'd3: read_reg = regs[3];
				4'd4: read_reg = regs[4];
				4'd5: read_reg = regs[5];
				4'd6: read_reg = regs[6];
				4'd7: read_reg = regs[7];
				4'd8: read_reg = regs[8];
				4'd9: read_reg = regs[9];
				4'd10: read_reg = regs[10];
				4'd11: read_reg = regs[11];
				4'd12: read_reg = regs[12];
				4'd13: read_reg = regs[13];
				4'd14: read_reg = regs[14];
				default: read_reg = visible_pc;
			endcase
		end
	endfunction

	function automatic logic [31:0] read_decode_reg(input logic [3:0] index);
		begin
			case (index)
				4'd0: read_decode_reg = regs[0];
				4'd1: read_decode_reg = regs[1];
				4'd2: read_decode_reg = regs[2];
				4'd3: read_decode_reg = regs[3];
				4'd4: read_decode_reg = regs[4];
				4'd5: read_decode_reg = regs[5];
				4'd6: read_decode_reg = regs[6];
				4'd7: read_decode_reg = regs[7];
				4'd8: read_decode_reg = regs[8];
				4'd9: read_decode_reg = regs[9];
				4'd10: read_decode_reg = regs[10];
				4'd11: read_decode_reg = regs[11];
				4'd12: read_decode_reg = regs[12];
				4'd13: read_decode_reg = regs[13];
				4'd14: read_decode_reg = regs[14];
				default: read_decode_reg = decode_visible_pc;
			endcase
		end
	endfunction

	// A value written by the instruction ahead has not reached the register file
	// yet. Each read port decides whether it is that register: its index is a
	// constant instruction field, so the comparison runs beside the register
	// read rather than behind the operand mux. The hit travels with the operand
	// and the value itself then needs a single mux, which matters because this
	// override sits on the loop a dependent instruction closes in one cycle.
	function automatic logic [1:0] forward_hits(
		input logic [3:0] index,
		input logic [9:0] tags
	);
		begin
			forward_hits[0] = (index < 15) && tags[9] && (tags[8:5] == index);
			forward_hits[1] = (index < 15) && tags[4] && (tags[3:0] == index);
		end
	endfunction

	// The second slot is written last in the source, so it wins.
	function automatic logic [31:0] forward_select(
		input logic  [1:0] hits,
		input logic [31:0] value,
		input logic [63:0] values
	);
		begin
			if (hits[1])      forward_select = values[31:0];
			else if (hits[0]) forward_select = values[63:32];
			else              forward_select = value;
		end
	endfunction

	// Quartus 17 requires the forwarding bus as an explicit function argument.
	`define ARM7_HITS(index) forward_hits(index, decode_forward_tags)
	`define ARM7_FORWARD(hits, value) forward_select(hits, value, decode_forward_values)

	// One physical read port per register field the two instruction sets use,
	// driven straight from the registered instruction. Decode then selects among
	// values that are already available instead of building its own read mux at
	// each of the fifty-one places it needs a register.
	logic [31:0] dec_instruction;
	logic [15:0] dec_tinsn;
	logic  [3:0] idx_arm_rn;
	logic  [3:0] idx_arm_rd;
	logic  [3:0] idx_arm_rs;
	logic  [3:0] idx_arm_rm;
	logic  [3:0] idx_thumb_r0;
	logic  [3:0] idx_thumb_r3;
	logic  [3:0] idx_thumb_r6;
	logic  [3:0] idx_thumb_r8;
	logic  [3:0] idx_thumb_hd;
	logic  [3:0] idx_thumb_hs;
	logic  [1:0] hit_arm_rn;
	logic  [1:0] hit_arm_rd;
	logic  [1:0] hit_arm_rs;
	logic  [1:0] hit_arm_rm;
	logic  [1:0] hit_thumb_r0;
	logic  [1:0] hit_thumb_r3;
	logic  [1:0] hit_thumb_r6;
	logic  [1:0] hit_thumb_r8;
	logic  [1:0] hit_thumb_hd;
	logic  [1:0] hit_thumb_hs;
	logic  [1:0] hit_sp;
	logic  [1:0] hit_lr;
	logic [31:0] raw_arm_rn;
	logic [31:0] raw_arm_rd;
	logic [31:0] raw_arm_rs;
	logic [31:0] raw_arm_rm;
	logic [31:0] raw_thumb_r0;
	logic [31:0] raw_thumb_r3;
	logic [31:0] raw_thumb_r6;
	logic [31:0] raw_thumb_r8;
	logic [31:0] raw_thumb_hd;
	logic [31:0] raw_thumb_hs;
	logic [31:0] raw_sp;
	logic [31:0] raw_lr;
	logic [31:0] reg_arm_rn;
	logic [31:0] reg_arm_rd;
	logic [31:0] reg_arm_rs;
	logic [31:0] reg_arm_rm;
	logic [31:0] reg_thumb_r0;
	logic [31:0] reg_thumb_r3;
	logic [31:0] reg_thumb_r8;
	logic [31:0] reg_thumb_hs;
	logic        arm_register_shift;

	assign dec_instruction = prefetch_instruction;
	assign dec_tinsn = prefetch_pc[1] ? prefetch_instruction[31:16] :
		prefetch_instruction[15:0];

	assign idx_arm_rn   = dec_instruction[19:16];
	assign idx_arm_rd   = dec_instruction[15:12];
	assign idx_arm_rs   = dec_instruction[11:8];
	assign idx_arm_rm   = dec_instruction[3:0];
	assign idx_thumb_r0 = {1'b0, dec_tinsn[2:0]};
	assign idx_thumb_r3 = {1'b0, dec_tinsn[5:3]};
	assign idx_thumb_r6 = {1'b0, dec_tinsn[8:6]};
	assign idx_thumb_r8 = {1'b0, dec_tinsn[10:8]};
	assign idx_thumb_hd = {dec_tinsn[7], dec_tinsn[2:0]};
	assign idx_thumb_hs = {dec_tinsn[6], dec_tinsn[5:3]};

	// A data-processing instruction whose shift amount comes from a register
	// spends an extra cycle before it reads its operands, so R15 reads one
	// instruction further on than usual. Bit 4 set with bit 7 clear picks that
	// form out - multiply, SWP and the halfword transfers all have bit 7 set -
	// once BX, MRS and MSR are excluded, which share the shape but sit in the
	// bits 27:23 = 00010 space with S clear.
	assign arm_register_shift = (dec_instruction[27:25] == 3'b000) &&
		dec_instruction[4] && !dec_instruction[7] &&
		!((dec_instruction[27:23] == 5'b00010) && !dec_instruction[20]);

	assign raw_arm_rn   = (arm_register_shift && (idx_arm_rn == 4'd15)) ?
		decode_visible_pc_p4 : read_decode_reg(idx_arm_rn);
	assign raw_arm_rd   = read_decode_reg(idx_arm_rd);
	assign raw_arm_rs   = (arm_register_shift && (idx_arm_rs == 4'd15)) ?
		decode_visible_pc_p4 : read_decode_reg(idx_arm_rs);
	assign raw_arm_rm   = (arm_register_shift && (idx_arm_rm == 4'd15)) ?
		decode_visible_pc_p4 : read_decode_reg(idx_arm_rm);
	assign raw_thumb_r0 = read_decode_reg(idx_thumb_r0);
	assign raw_thumb_r3 = read_decode_reg(idx_thumb_r3);
	assign raw_thumb_r6 = read_decode_reg(idx_thumb_r6);
	assign raw_thumb_r8 = read_decode_reg(idx_thumb_r8);
	assign raw_thumb_hd = read_decode_reg(idx_thumb_hd);
	assign raw_thumb_hs = read_decode_reg(idx_thumb_hs);
	assign raw_sp       = regs[13];
	assign raw_lr       = regs[14];

	assign hit_arm_rn   = `ARM7_HITS(idx_arm_rn);
	assign hit_arm_rd   = `ARM7_HITS(idx_arm_rd);
	assign hit_arm_rs   = `ARM7_HITS(idx_arm_rs);
	assign hit_arm_rm   = `ARM7_HITS(idx_arm_rm);
	assign hit_thumb_r0 = `ARM7_HITS(idx_thumb_r0);
	assign hit_thumb_r3 = `ARM7_HITS(idx_thumb_r3);
	assign hit_thumb_r6 = `ARM7_HITS(idx_thumb_r6);
	assign hit_thumb_r8 = `ARM7_HITS(idx_thumb_r8);
	assign hit_thumb_hd = `ARM7_HITS(idx_thumb_hd);
	assign hit_thumb_hs = `ARM7_HITS(idx_thumb_hs);
	assign hit_sp       = `ARM7_HITS(4'd13);
	assign hit_lr       = `ARM7_HITS(4'd14);

	assign reg_arm_rn   = `ARM7_FORWARD(hit_arm_rn, raw_arm_rn);
	assign reg_arm_rd   = `ARM7_FORWARD(hit_arm_rd, raw_arm_rd);
	assign reg_arm_rs   = `ARM7_FORWARD(hit_arm_rs, raw_arm_rs);
	assign reg_arm_rm   = `ARM7_FORWARD(hit_arm_rm, raw_arm_rm);
	assign reg_thumb_r0 = `ARM7_FORWARD(hit_thumb_r0, raw_thumb_r0);
	assign reg_thumb_r3 = `ARM7_FORWARD(hit_thumb_r3, raw_thumb_r3);
	assign reg_thumb_r8 = `ARM7_FORWARD(hit_thumb_r8, raw_thumb_r8);
	assign reg_thumb_hs = `ARM7_FORWARD(hit_thumb_hs, raw_thumb_hs);

	// True when the live register already is the user-mode one, so a user-bank
	// transfer must use it rather than the saved copy: R0-R7 always, R8-R12
	// outside FIQ because every other mode shares them, and R13/R14 only in
	// user and system mode.
	function automatic logic user_reg_is_live(input logic [3:0] index);
		user_reg_is_live = (index < 8) ||
			((index < 13) && (cpsr[4:0] != MODE_FIQ)) ||
			((index >= 13) && ((cpsr[4:0] == MODE_USER) || (cpsr[4:0] == MODE_SYS)));
	endfunction

	function automatic logic [31:0] read_user_reg(input logic [3:0] index);
		if (index >= 15)
			read_user_reg = visible_pc;
		else if (user_reg_is_live(index))
			read_user_reg = regs[index];
		else
			read_user_reg = usr_r8_14[index - 8];
	endfunction

	function automatic logic [31:0] spsr_for_mode(input logic [4:0] mode);
		case (mode)
			MODE_FIQ: spsr_for_mode = spsr_fiq;
			MODE_IRQ: spsr_for_mode = spsr_irq;
			MODE_SVC: spsr_for_mode = spsr_svc;
			MODE_ABT: spsr_for_mode = spsr_abt;
			MODE_UND: spsr_for_mode = spsr_und;
			default:  spsr_for_mode = cpsr;
		endcase
	endfunction

	assign current_spsr = spsr_for_mode(cpsr[4:0]);
	assign block_remaining = block_list & ~(16'h0001 << block_index[3:0]);
	assign block_next_index = first_set_register(block_remaining);
	assign dec_block_first_index = first_set_register(dec_block_list);
	assign data_loaded_value = load_lane(mem_rdata, data_address, data_size, data_signed);
	assign state_user_index = state_index[2:0];
	assign state_fiq_index = state_index[2:0] - 3'd1;
	always_comb begin
		multiply_calculated_result = exec_multiply_operand_a * exec_multiply_operand_b;
		if (exec_multiply_long && exec_multiply_signed && exec_multiply_operand_a[31])
			multiply_calculated_result = multiply_calculated_result -
				{exec_multiply_operand_b, 32'b0};
		if (exec_multiply_long && exec_multiply_signed && exec_multiply_operand_b[31])
			multiply_calculated_result = multiply_calculated_result -
				{exec_multiply_operand_a, 32'b0};
		if (exec_multiply_accumulate)
			multiply_calculated_result = multiply_calculated_result +
				exec_multiply_accumulator;
	end

	always_comb begin : decode_forwarding
		logic block_updates_active;

		decode_forward_0_valid = 1'b0;
		decode_forward_0_index = 4'b0;
		decode_forward_0_value = 32'b0;
		decode_forward_1_valid = 1'b0;
		decode_forward_1_index = 4'b0;
		decode_forward_1_value = 32'b0;
		block_updates_active = !block_user || user_reg_is_live(block_index[3:0]);

		case (state)
			RUN: begin
				if (decode_valid && exec_condition && (exec_kind == EXEC_SIMPLE) &&
					exec_write_reg && (exec_rd < 15)) begin
					decode_forward_0_valid = 1'b1;
					decode_forward_0_index = exec_rd;
					decode_forward_0_value = exec_effective_result;
				end
			end
			MEM_ACCESS: begin
				if (mem_req && mem_ready && !mem_abort) begin
					if (data_writeback && (data_rn < 15)) begin
						decode_forward_0_valid = 1'b1;
						decode_forward_0_index = data_rn;
						decode_forward_0_value = data_writeback_value;
					end
					if (data_load && !data_load_pc && (data_rd < 15)) begin
						decode_forward_1_valid = 1'b1;
						decode_forward_1_index = data_rd;
						decode_forward_1_value = data_loaded_value;
					end
				end
			end
			SWP_WRITE: begin
				if (mem_req && mem_ready && !mem_abort && (swp_rd < 15)) begin
					decode_forward_0_valid = 1'b1;
					decode_forward_0_index = swp_rd;
					decode_forward_0_value = swp_loaded_value;
				end
			end
			BLOCK_ACCESS: begin
				if (mem_req && mem_ready && !mem_abort && (block_next_index >= 16)) begin
					if (block_load && (block_index < 15) && block_updates_active) begin
						decode_forward_0_valid = 1'b1;
						decode_forward_0_index = block_index[3:0];
						decode_forward_0_value = mem_rdata;
					end
					if (block_writeback && (block_rn < 15)) begin
						decode_forward_1_valid = 1'b1;
						decode_forward_1_index = block_rn;
						decode_forward_1_value = block_writeback_value;
					end
				end
			end
			MUL_WAIT: begin
				if (multiply_wait <= 1) begin
					if (multiply_rd_lo < 15) begin
						decode_forward_0_valid = 1'b1;
						decode_forward_0_index = multiply_rd_lo;
						decode_forward_0_value = multiply_result[31:0];
					end
					if (multiply_long && (multiply_rd_hi < 15)) begin
						decode_forward_1_valid = 1'b1;
						decode_forward_1_index = multiply_rd_hi;
						decode_forward_1_value = multiply_result[63:32];
					end
				end
			end
			default: ;
		endcase
	end

	always_comb begin
		decode_cpsr = cpsr;
		if ((state == RUN) && decode_valid && exec_condition &&
			(exec_kind == EXEC_SIMPLE) &&
			(exec_write_flags || (exec_write_psr && !exec_write_spsr)))
			decode_cpsr = exec_effective_post_cpsr;
		else if ((state == MUL_WAIT) && (multiply_wait <= 1) &&
			multiply_set_flags)
			decode_cpsr = multiply_post_cpsr;
	end

	always_comb begin
		decode_spsr = current_spsr;
		if ((state == RUN) && decode_valid && exec_condition &&
			(exec_kind == EXEC_SIMPLE) && exec_write_psr && exec_write_spsr &&
			mode_has_spsr(cpsr[4:0]))
			decode_spsr = (current_spsr & ~exec_psr_mask) |
				(exec_psr_value & exec_psr_mask);
	end

	always_comb begin
		multiply_post_cpsr = cpsr;
		if (multiply_set_flags) begin
			multiply_post_cpsr[31] = multiply_long ? multiply_result[63] : multiply_result[31];
			multiply_post_cpsr[30] = multiply_long ? (multiply_result == 0) : (multiply_result[31:0] == 0);
		end
	end

	always_comb begin
		dec_post_cpsr = decode_cpsr;
		if (dec_write_flags)
			dec_post_cpsr = (dec_post_cpsr & ~dec_flag_mask) |
				(dec_flag_value & dec_flag_mask);
		if (dec_restore_spsr)
			dec_post_cpsr = decode_spsr;
		if (dec_write_psr && !dec_write_spsr)
			dec_post_cpsr = (dec_post_cpsr & ~dec_psr_mask) |
				(dec_psr_value & dec_psr_mask);
	end

	function automatic logic [31:0] align_pc(
		input logic [31:0] value,
		input logic target_thumb
	);
		align_pc = value & (target_thumb ? 32'hffff_fffe : 32'hffff_fffc);
	endfunction

	function automatic logic [32:0] add_with_carry(
		input logic [31:0] lhs,
		input logic [31:0] rhs,
		input logic        carry_in
	);
		begin
			// Bias bit zero so Quartus uses one carry chain for all three inputs.
			add_with_carry = 33'(({1'b0, lhs, 1'b1} +
				{1'b0, rhs, carry_in}) >> 1);
		end
	endfunction

	// The whole core's shifter and ALU. Decode hands over registered operands
	// and control; every class that needs arithmetic reads its answer here:
	// data-processing results and flags, transfer addresses and updated bases,
	// and branch targets computed from a register.
	always_comb begin : execute_datapath
		arm_shift_t shifted;
		logic [32:0] arithmetic;
		logic [31:0] flag_value;
		logic carry_out;
		logic overflow_out;

		if (!exec_dp_shift_enable) begin
			shifted.value = exec_dp_op2;
			shifted.carry = exec_dp_shift_carry;
		end else if (exec_dp_shift_rrx) begin
			shifted.value = {exec_dp_cpsr[29], exec_dp_op2[31:1]};
			shifted.carry = exec_dp_op2[0];
		end else begin
			shifted = shift_register(exec_dp_op2, exec_dp_shift_type,
				exec_dp_shift_amount, exec_dp_cpsr[29]);
		end

		dp_result = 32'b0;
		arithmetic = 33'b0;
		carry_out = shifted.carry;
		overflow_out = exec_dp_cpsr[28];
		case (exec_dp_opcode)
			ALU_AND: dp_result = exec_dp_op1 & shifted.value;
			ALU_EOR: dp_result = exec_dp_op1 ^ shifted.value;
			ALU_SUB: begin
				arithmetic = {1'b1, exec_dp_op1} - {1'b0, shifted.value};
				dp_result = arithmetic[31:0];
				carry_out = arithmetic[32];
				overflow_out = (exec_dp_op1[31] != shifted.value[31]) &&
					(dp_result[31] != exec_dp_op1[31]);
			end
			ALU_RSB: begin
				arithmetic = {1'b1, shifted.value} - {1'b0, exec_dp_op1};
				dp_result = arithmetic[31:0];
				carry_out = arithmetic[32];
				overflow_out = (shifted.value[31] != exec_dp_op1[31]) &&
					(dp_result[31] != shifted.value[31]);
			end
			ALU_ADD, ALU_CMN: begin
				arithmetic = {1'b0, exec_dp_op1} + {1'b0, shifted.value};
				dp_result = arithmetic[31:0];
				carry_out = arithmetic[32];
				overflow_out = (exec_dp_op1[31] == shifted.value[31]) &&
					(dp_result[31] != exec_dp_op1[31]);
			end
			ALU_ADC: begin
				arithmetic = add_with_carry(exec_dp_op1, shifted.value,
					exec_dp_cpsr[29]);
				dp_result = arithmetic[31:0];
				carry_out = arithmetic[32];
				overflow_out = (exec_dp_op1[31] == shifted.value[31]) &&
					(dp_result[31] != exec_dp_op1[31]);
			end
			ALU_SBC: begin
				arithmetic = add_with_carry(exec_dp_op1, ~shifted.value,
					exec_dp_cpsr[29]);
				dp_result = arithmetic[31:0];
				carry_out = arithmetic[32];
				overflow_out = (exec_dp_op1[31] != shifted.value[31]) &&
					(dp_result[31] != exec_dp_op1[31]);
			end
			ALU_RSC: begin
				arithmetic = add_with_carry(shifted.value, ~exec_dp_op1,
					exec_dp_cpsr[29]);
				dp_result = arithmetic[31:0];
				carry_out = arithmetic[32];
				overflow_out = (shifted.value[31] != exec_dp_op1[31]) &&
					(dp_result[31] != shifted.value[31]);
			end
			ALU_TST: dp_result = exec_dp_op1 & shifted.value;
			ALU_TEQ: dp_result = exec_dp_op1 ^ shifted.value;
			ALU_CMP: begin
				arithmetic = {1'b1, exec_dp_op1} - {1'b0, shifted.value};
				dp_result = arithmetic[31:0];
				carry_out = arithmetic[32];
				overflow_out = (exec_dp_op1[31] != shifted.value[31]) &&
					(dp_result[31] != exec_dp_op1[31]);
			end
			ALU_ORR: dp_result = exec_dp_op1 | shifted.value;
			ALU_MOV: dp_result = shifted.value;
			ALU_BIC: dp_result = exec_dp_op1 & ~shifted.value;
			default: dp_result = ~shifted.value;
		endcase

		flag_value = exec_dp_cpsr;
		flag_value[31] = dp_result[31];
		flag_value[30] = (dp_result == 0);
		flag_value[29] = carry_out;
		flag_value[28] = overflow_out;
		dp_post_cpsr = exec_dp_cpsr;
		if (exec_write_flags)
			dp_post_cpsr = (dp_post_cpsr & ~exec_flag_mask) |
				(flag_value & exec_flag_mask);
		if (exec_restore_spsr)
			dp_post_cpsr = exec_dp_spsr;
		if (exec_write_psr && !exec_write_spsr)
			dp_post_cpsr = (dp_post_cpsr & ~exec_psr_mask) |
				(exec_psr_value & exec_psr_mask);
	end

	// The ALU produces the updated base; pre-indexed transfers address with it
	// and post-indexed ones address with the unmodified base.
	assign dp_mem_updated_base = dp_result;
	assign dp_mem_address = exec_mem_preindex ? dp_result : exec_dp_op1;

	// A block transfer needs its first address and its updated base at once, so
	// the address gets its own adder rather than a second pass through the ALU.
	// An EXEC_SIMPLE that rewrites CPSR wholesale - MSR, or the compare group's
	// P form - flushes the pipeline, so it has to wait for a free bus.
	assign exec_simple_control = exec_restore_spsr ||
		(exec_write_psr && !exec_write_spsr && |(exec_psr_mask[7:0]));
	assign dp_block_updated_base = dp_result;
	assign dp_block_address = exec_block_addr_add ?
		(exec_dp_op1 + {25'b0, exec_block_addr_delta}) :
		(exec_dp_op1 - {25'b0, exec_block_addr_delta});

	// Early termination depends on the multiplier operand, so the count is
	// taken here rather than in front of the decode register.
	always_comb begin : multiply_cycle_count
		logic [2:0] cycles;
		if ((exec_multiply_operand_b[31:8] == 24'h000000) ||
			(exec_multiply_operand_b[31:8] == 24'hffffff))
			cycles = 3'd1;
		else if ((exec_multiply_operand_b[31:16] == 16'h0000) ||
			(exec_multiply_operand_b[31:16] == 16'hffff))
			cycles = 3'd2;
		else if ((exec_multiply_operand_b[31:24] == 8'h00) ||
			(exec_multiply_operand_b[31:24] == 8'hff))
			cycles = 3'd3;
		else
			cycles = 3'd4;
		dp_multiply_wait = cycles + {1'b0, exec_multiply_extra};
	end

	assign exec_effective_result = exec_dp ? dp_result : exec_result;
	assign exec_effective_post_cpsr = exec_dp ? dp_post_cpsr : exec_post_cpsr;
	assign exec_effective_branch_target = exec_dp ? dp_result : exec_branch_target;

	function automatic logic [31:0] load_lane(
		input logic [31:0] word,
		input logic  [1:0] address,
		input logic  [1:0] size,
		input logic        sign_extend
	);
		logic [7:0] byte_value;
		logic [15:0] half_value;
		begin
			case (address[1:0])
				2'd0: byte_value = word[7:0];
				2'd1: byte_value = word[15:8];
				2'd2: byte_value = word[23:16];
				default: byte_value = word[31:24];
			endcase
			half_value = address[1] ? word[31:16] : word[15:0];
			case (size)
				MEM_BYTE:
					load_lane = sign_extend ? {{24{byte_value[7]}}, byte_value} : {24'b0, byte_value};
				MEM_HALF:
					if (address[0])
						load_lane = sign_extend ?
							{{24{byte_value[7]}}, byte_value} :
							ror32({16'b0, half_value}, 5'd8);
					else
						load_lane = sign_extend ?
							{{16{half_value[15]}}, half_value} : {16'b0, half_value};
				default:
					load_lane = ror32(word, {address[1:0], 3'b000});
			endcase
		end
	endfunction

	function automatic logic [3:0] store_strobes(
		input logic  [1:0] address,
		input logic  [1:0] size
	);
		case (size)
			MEM_BYTE: store_strobes = 4'b0001 << address[1:0];
			MEM_HALF: store_strobes = address[1] ? 4'b1100 : 4'b0011;
			default:  store_strobes = 4'b1111;
		endcase
	endfunction

	function automatic logic [31:0] store_lanes(
		input logic [31:0] value,
		input logic  [1:0] size
	);
		case (size)
			MEM_BYTE: store_lanes = {4{value[7:0]}};
			MEM_HALF: store_lanes = {2{value[15:0]}};
			default:  store_lanes = value;
		endcase
	endfunction

	function automatic logic [4:0] first_set_register(input logic [15:0] list);
		integer i;
		begin
			first_set_register = 5'd16;
			for (i = 15; i >= 0; i = i - 1)
				if (list[i])
					first_set_register = i[4:0];
		end
	endfunction

	function automatic logic [4:0] count_registers(input logic [15:0] list);
		integer i;
		logic [4:0] count;
		begin
			count = 0;
			for (i = 0; i < 16; i = i + 1)
				count = count + list[i];
			count_registers = count;
		end
	endfunction

	// A block transfer writes its base register back in the same cycle a
	// banked save may run. Both are non-blocking, so a plain save would file
	// the old value and lose the writeback. This view substitutes it.
	function automatic logic [31:0] reg_with_writeback(input logic [3:0] index);
		begin
			if (block_writeback && (block_rn < 15) && (block_rn == index))
				reg_with_writeback = block_writeback_value;
			else
				reg_with_writeback = regs[index];
		end
	endfunction

	function automatic logic bank_switch_shared(input logic [4:0] from_mode,
		input logic [4:0] to_mode);
		bank_switch_shared = (from_mode == MODE_FIQ) || (to_mode == MODE_FIQ);
	endfunction

	// R8-R12 are banked only for FIQ; every other mode shares them. A switch
	// between two non-FIQ modes must leave them alone - the save and the
	// reload are both non-blocking, so the reload would put the stale copy
	// back and lose whatever the program had just written. `move_shared` is
	// set when FIQ is on one side of the switch, and when a whole image is
	// wanted, which is the halted-state capture and its commit.
	task automatic save_active_bank_wb(input logic [4:0] mode,
		input logic move_shared);
		integer i;
		begin
			i = 0;
			if (mode == MODE_FIQ) begin
				for (i = 0; i < 7; i = i + 1)
					fiq_r8_14[i] <= reg_with_writeback(4'(i + 8));
			end else begin
				if (move_shared)
					for (i = 0; i < 5; i = i + 1)
						usr_r8_14[i] <= reg_with_writeback(4'(i + 8));
				case (mode)
					MODE_USER, MODE_SYS: begin
						usr_r8_14[5] <= reg_with_writeback(4'(13));
						usr_r8_14[6] <= reg_with_writeback(4'(14));
					end
					MODE_IRQ: begin
						irq_r13_14[0] <= reg_with_writeback(4'(13));
						irq_r13_14[1] <= reg_with_writeback(4'(14));
					end
					MODE_SVC: begin
						svc_r13_14[0] <= reg_with_writeback(4'(13));
						svc_r13_14[1] <= reg_with_writeback(4'(14));
					end
					MODE_ABT: begin
						abt_r13_14[0] <= reg_with_writeback(4'(13));
						abt_r13_14[1] <= reg_with_writeback(4'(14));
					end
					MODE_UND: begin
						und_r13_14[0] <= reg_with_writeback(4'(13));
						und_r13_14[1] <= reg_with_writeback(4'(14));
					end
					default: ;
				endcase
			end
			i = 0;
		end
	endtask

	task automatic save_active_bank(input logic [4:0] mode,
		input logic move_shared);
		integer i;
		begin
			i = 0;
			if (mode == MODE_FIQ) begin
				for (i = 0; i < 7; i = i + 1)
					fiq_r8_14[i] <= regs[i + 8];
			end else begin
				if (move_shared)
					for (i = 0; i < 5; i = i + 1)
						usr_r8_14[i] <= regs[i + 8];
				case (mode)
					MODE_USER, MODE_SYS: begin
						usr_r8_14[5] <= regs[13];
						usr_r8_14[6] <= regs[14];
					end
					MODE_IRQ: begin
						irq_r13_14[0] <= regs[13];
						irq_r13_14[1] <= regs[14];
					end
					MODE_SVC: begin
						svc_r13_14[0] <= regs[13];
						svc_r13_14[1] <= regs[14];
					end
					MODE_ABT: begin
						abt_r13_14[0] <= regs[13];
						abt_r13_14[1] <= regs[14];
					end
					MODE_UND: begin
						und_r13_14[0] <= regs[13];
						und_r13_14[1] <= regs[14];
					end
					default: ;
				endcase
			end
			i = 0;
		end
	endtask

	task automatic load_active_bank(input logic [4:0] mode,
		input logic move_shared);
		integer i;
		begin
			i = 0;
			if (mode == MODE_FIQ) begin
				for (i = 0; i < 7; i = i + 1)
					regs[i + 8] <= fiq_r8_14[i];
			end else begin
				if (move_shared)
					for (i = 0; i < 5; i = i + 1)
						regs[i + 8] <= usr_r8_14[i];
				case (mode)
					MODE_USER, MODE_SYS: begin
						regs[13] <= usr_r8_14[5];
						regs[14] <= usr_r8_14[6];
					end
					MODE_IRQ: begin
						regs[13] <= irq_r13_14[0];
						regs[14] <= irq_r13_14[1];
					end
					MODE_SVC: begin
						regs[13] <= svc_r13_14[0];
						regs[14] <= svc_r13_14[1];
					end
					MODE_ABT: begin
						regs[13] <= abt_r13_14[0];
						regs[14] <= abt_r13_14[1];
					end
					MODE_UND: begin
						regs[13] <= und_r13_14[0];
						regs[14] <= und_r13_14[1];
					end
					default: ;
				endcase
			end
			i = 0;
		end
	endtask

	task automatic write_user_register(
		input logic  [3:0] index,
		input logic [31:0] value
	);
		begin
			if (index < 8) begin
				regs[index] <= value;
			end else if (index < 15) begin
				usr_r8_14[index - 8] <= value;
				if (user_reg_is_live(index))
					regs[index] <= value;
			end
		end
	endtask

	task automatic write_spsr_for_mode(
		input logic [4:0] mode,
		input logic [31:0] value
	);
		begin
			case (mode)
				MODE_FIQ: spsr_fiq <= value;
				MODE_IRQ: spsr_irq <= value;
				MODE_SVC: spsr_svc <= value;
				MODE_ABT: spsr_abt <= value;
				MODE_UND: spsr_und <= value;
				default: ;
			endcase
		end
	endtask

	task automatic issue_fetch(
		input logic [31:0] address,
		input logic        target_thumb,
		input logic  [4:0] target_mode,
		input logic        sequential
	);
		logic [31:0] aligned;
		begin
			aligned = align_pc(address, target_thumb);
			mem_req <= 1'b1;
			mem_addr <= aligned;
			mem_write <= 1'b0;
			mem_wdata <= 32'b0;
			mem_size <= target_thumb ? MEM_HALF : MEM_WORD;
			mem_wstrb <= 4'b0000;
			mem_seq <= sequential;
			mem_fetch <= 1'b1;
			mem_privileged <= (target_mode != MODE_USER);
			mem_lock <= 1'b0;
			next_fetch_pc <= aligned + (target_thumb ? 32'd2 : 32'd4);
		end
	endtask

	task automatic enter_exception(
		input logic  [4:0] target_mode,
		input logic [31:0] vector,
		input logic [31:0] link_value,
		input logic [31:0] saved_cpsr,
		input logic        disable_fiq
	);
		logic [31:0] next_cpsr;
		begin
			next_cpsr = saved_cpsr;
			next_cpsr[7] = 1'b1;
			if (disable_fiq)
				next_cpsr[6] = 1'b1;
			next_cpsr[5] = 1'b0;
			next_cpsr[4:0] = target_mode;
			// The banked registers of a mode are the same registers whether or
			// not the exception came from that mode, so entering a mode from
			// itself must leave them alone. Saving and reloading in the same
			// cycle would put the stale bank back, because both are
			// non-blocking.
			if (saved_cpsr[4:0] != target_mode) begin
				save_active_bank(saved_cpsr[4:0],
					bank_switch_shared(saved_cpsr[4:0], target_mode));
				load_active_bank(target_mode,
					bank_switch_shared(saved_cpsr[4:0], target_mode));
			end
			write_spsr_for_mode(target_mode, saved_cpsr);
			regs[14] <= link_value;
			cpsr <= next_cpsr;
			decode_valid <= 1'b0;
			prefetch_valid <= 1'b0;
			ahead_valid <= 1'b0;
			boundary_event <= BOUNDARY_NONE;
			state <= RUN;
			halted <= 1'b0;
			issue_fetch(vector, 1'b0, target_mode, 1'b0);
		end
	endtask

	task automatic request_boundary(
		input logic [31:0] next_pc_value,
		input logic  [7:6] boundary_mask
	);
		begin
			boundary_pc <= next_pc_value;
			decode_valid <= 1'b0;
			prefetch_valid <= 1'b0;
			ahead_valid <= 1'b0;
			if (!mem_req || mem_ready) begin
				mem_req <= 1'b0;
				mem_seq <= 1'b0;
				mem_fetch <= 1'b0;
				mem_lock <= 1'b0;
			end
			if (halt_req)
				boundary_event <= BOUNDARY_HALT;
			else if (!fiq_n && !boundary_mask[6])
				boundary_event <= BOUNDARY_FIQ;
			else if (!irq_n && !boundary_mask[7])
				boundary_event <= BOUNDARY_IRQ;
			else
				boundary_event <= BOUNDARY_NONE;
		end
	endtask

	task automatic retain_completed_fetch;
		begin
			if (mem_req && mem_ready && mem_fetch && !mem_abort) begin
				ahead_instruction <= mem_rdata;
				ahead_pc <= mem_addr;
				ahead_valid <= 1'b1;
			end else begin
				ahead_valid <= 1'b0;
			end
		end
	endtask

	task automatic capture_prefetch_decode;
		begin
			decode_instruction <= prefetch_instruction;
			decode_pc <= prefetch_pc;
			decode_valid <= 1'b1;
			exec_kind <= dec_kind;
			exec_condition <= dec_condition;
			exec_write_reg <= dec_write_reg;
			exec_rd <= dec_rd;
			exec_result <= dec_result;
			exec_write_flags <= dec_write_flags;
			exec_flag_mask <= dec_flag_mask;
			exec_restore_spsr <= dec_restore_spsr;
			exec_write_psr <= dec_write_psr;
			exec_write_spsr <= dec_write_spsr;
			exec_psr_mask <= dec_psr_mask;
			exec_psr_value <= dec_psr_value;
			exec_post_cpsr <= dec_post_cpsr;
			exec_branch_target <= dec_branch_target;
			exec_branch_thumb <= dec_branch_thumb;
			exec_link <= dec_link;
			exec_link_value <= dec_link_value;
			exec_mem_preindex <= dec_mem_preindex;
			exec_mem_write_value <= dec_mem_write_value;
			exec_mem_rd <= dec_mem_rd;
			exec_mem_rn <= dec_mem_rn;
			exec_mem_size <= dec_mem_size;
			exec_mem_load <= dec_mem_load;
			exec_mem_signed <= dec_mem_signed;
			exec_mem_writeback <= dec_mem_writeback;
			exec_mem_load_pc <= dec_mem_load_pc;
			exec_block_list <= dec_block_list;
			exec_block_addr_delta <= dec_block_addr_delta;
			exec_block_addr_add <= dec_block_addr_add;
			exec_block_rn <= dec_block_rn;
			exec_block_first_index <= dec_block_first_index;
			exec_block_load <= dec_block_load;
			exec_block_writeback <= dec_block_writeback;
			exec_block_user <= dec_block_user;
			exec_block_restore_cpsr <= dec_block_restore_cpsr;
			exec_multiply_operand_a <= dec_multiply_operand_a;
			exec_multiply_operand_b <= dec_multiply_operand_b;
			exec_multiply_accumulator <= dec_multiply_accumulator;
			exec_multiply_rd_lo <= dec_multiply_rd_lo;
			exec_multiply_rd_hi <= dec_multiply_rd_hi;
			exec_multiply_extra <= dec_multiply_extra;
			exec_multiply_long <= dec_multiply_long;
			exec_multiply_signed <= dec_multiply_signed;
			exec_multiply_accumulate <= dec_multiply_accumulate;
			exec_multiply_set_flags <= dec_multiply_set_flags;
			exec_internal_cycle <= dec_internal_cycle;
			exec_dp <= dec_dp;
			exec_dp_op1 <= dec_dp_op1;
			exec_dp_op2 <= dec_dp_op2;
			exec_dp_shift_amount <= dec_dp_shift_amount;
			exec_dp_shift_type <= dec_dp_shift_type;
			exec_dp_shift_enable <= dec_dp_shift_enable;
			exec_dp_shift_rrx <= dec_dp_shift_rrx;
			exec_dp_shift_carry <= dec_dp_shift_carry;
			exec_dp_opcode <= dec_dp_opcode;
			exec_dp_cpsr <= dec_dp_cpsr;
			exec_dp_spsr <= dec_dp_spsr;
			exec_swp_byte <= dec_swp_byte;
			exec_swp_store_value <= dec_swp_store_value;
		end
	endtask

	task automatic finish_sequential(
		input logic [31:0] resume_pc,
		input logic  [7:0] post_control
	);
		begin
			retire <= 1'b1;
			// synthesis translate_off
			trace_retire_pc <= decode_pc;
			trace_retire_instruction <= thumb ?
				{16'b0, (decode_pc[1] ? decode_instruction[31:16] : decode_instruction[15:0])} :
				decode_instruction;
			trace_retire_thumb <= thumb;
			trace_retire_exception <= 1'b0;
			trace_retire_exception_return <= 1'b0;
			// synthesis translate_on
			decode_valid <= 1'b0;
			if (mem_req && mem_ready && mem_fetch && mem_abort) begin
				boundary_event <= BOUNDARY_PABT;
				boundary_pc <= mem_addr;
				prefetch_valid <= 1'b0;
				ahead_valid <= 1'b0;
				mem_req <= 1'b0;
				mem_seq <= 1'b0;
				mem_fetch <= 1'b0;
				mem_lock <= 1'b0;
			end else if (halt_req || (!fiq_n && !post_control[6]) ||
				(!irq_n && !post_control[7])) begin
				request_boundary(resume_pc, post_control[7:6]);
			end else begin
				if (prefetch_valid) begin
					capture_prefetch_decode();
				end
				if (ahead_valid) begin
					prefetch_instruction <= ahead_instruction;
					prefetch_pc <= ahead_pc;
					prefetch_valid <= 1'b1;
					ahead_valid <= 1'b0;
					issue_fetch(next_fetch_pc, post_control[5], post_control[4:0], 1'b0);
				end else if (mem_req && mem_ready && mem_fetch) begin
					prefetch_instruction <= mem_rdata;
					prefetch_pc <= mem_addr;
					prefetch_valid <= 1'b1;
					issue_fetch(next_fetch_pc, post_control[5], post_control[4:0], 1'b1);
				end else begin
					prefetch_valid <= 1'b0;
					if (!mem_req || mem_ready)
						issue_fetch(resume_pc, post_control[5], post_control[4:0], 1'b0);
				end
			end
		end
	endtask

	task automatic finish_control(
		input logic [31:0] target_pc,
		input logic        target_thumb,
		input logic  [7:6] post_mask,
		input logic  [4:0] post_mode
	);
		begin
			retire <= 1'b1;
			// synthesis translate_off
			trace_retire_pc <= decode_pc;
			trace_retire_instruction <= thumb ?
				{16'b0, (decode_pc[1] ? decode_instruction[31:16] : decode_instruction[15:0])} :
				decode_instruction;
			trace_retire_thumb <= thumb;
			trace_retire_exception <= 1'b0;
			trace_retire_exception_return <= exec_restore_spsr || block_restore_cpsr;
			// synthesis translate_on
			decode_valid <= 1'b0;
			prefetch_valid <= 1'b0;
			ahead_valid <= 1'b0;
			if (halt_req || (!fiq_n && !post_mask[6]) ||
				(!irq_n && !post_mask[7])) begin
				request_boundary(align_pc(target_pc, target_thumb), post_mask);
			end else begin
				issue_fetch(target_pc, target_thumb, post_mode, 1'b0);
			end
		end
	endtask

	// Decode is combinational. The clocked block below is the only owner of
	// architectural state, matching the VHDL split between decode and execute.
	// An immediate shift encodes zero as thirty-two for LSR and ASR and as RRX
	// for ROR; the register-amount form treats zero as no shift. Normalising to
	// the register form lets one shifter serve both. Bit eight requests RRX.
	function automatic logic [8:0] immediate_shift_amount(
		input logic [1:0] shift_type,
		input logic [4:0] amount
	);
		begin
			if (amount != 0)
				immediate_shift_amount = {4'b0, amount};
			else
				case (shift_type)
					2'b00:   immediate_shift_amount = 9'd0;
					2'b11:   immediate_shift_amount = {1'b1, 8'd0};
					default: immediate_shift_amount = {1'b0, 8'd32};
				endcase
		end
	endfunction

	// Decode names an operand source; this mux acts on the name. Keeping it
	// flat and outside the decode chain is what holds the register file, the
	// forwarding path and the shifter two levels apart.
	always_comb begin : operand_select
		logic [31:0] op1_raw, op2_raw, amount_raw;
		logic  [1:0] op1_hit, op2_hit, amount_hit;

		case (dec_op1_sel)
			SRC_ARM_RN:   begin op1_raw = raw_arm_rn;   op1_hit = hit_arm_rn;   end
			SRC_T_R0:     begin op1_raw = raw_thumb_r0; op1_hit = hit_thumb_r0; end
			SRC_T_R3:     begin op1_raw = raw_thumb_r3; op1_hit = hit_thumb_r3; end
			SRC_T_R8:     begin op1_raw = raw_thumb_r8; op1_hit = hit_thumb_r8; end
			SRC_T_HD:     begin op1_raw = raw_thumb_hd; op1_hit = hit_thumb_hd; end
			SRC_SP:       begin op1_raw = raw_sp;       op1_hit = hit_sp;       end
			SRC_LR:       begin op1_raw = raw_lr;       op1_hit = hit_lr;       end
			SRC_PC_ALIGN: begin op1_raw = {decode_visible_pc[31:2], 2'b00};
			                    op1_hit = 2'b00; end
			default:      begin op1_raw = 32'b0;        op1_hit = 2'b00;        end
		endcase
		case (dec_op2_sel)
			SRC_IMM:      begin op2_raw = dec_op2_imm;  op2_hit = 2'b00;        end
			SRC_ARM_RD:   begin op2_raw = raw_arm_rd;   op2_hit = hit_arm_rd;   end
			SRC_ARM_RM:   begin op2_raw = raw_arm_rm;   op2_hit = hit_arm_rm;   end
			// R15 as the shifted operand of a register-amount shift reads four
			// bytes further ahead than the ordinary visible PC. R15 never
			// forwards, so the hit stays clear either way.
			SRC_RM_SHIFT: begin op2_raw = (idx_arm_rm == 4'd15) ?
			                        decode_visible_pc_p4 : raw_arm_rm;
			                    op2_hit = hit_arm_rm; end
			SRC_T_R0:     begin op2_raw = raw_thumb_r0; op2_hit = hit_thumb_r0; end
			SRC_T_R3:     begin op2_raw = raw_thumb_r3; op2_hit = hit_thumb_r3; end
			SRC_T_R6:     begin op2_raw = raw_thumb_r6; op2_hit = hit_thumb_r6; end
			SRC_T_HS:     begin op2_raw = raw_thumb_hs; op2_hit = hit_thumb_hs; end
			default:      begin op2_raw = 32'b0;        op2_hit = 2'b00;        end
		endcase
		case (dec_amount_sel)
			AMT_ARM_RS: begin amount_raw = raw_arm_rs;   amount_hit = hit_arm_rs;   end
			AMT_T_R3:   begin amount_raw = raw_thumb_r3; amount_hit = hit_thumb_r3; end
			default:    begin amount_raw = {24'b0, dec_amount_imm};
			                  amount_hit = 2'b00; end
		endcase

		dec_dp_op1 = `ARM7_FORWARD(op1_hit, op1_raw);
		dec_dp_op2 = `ARM7_FORWARD(op2_hit, op2_raw);
		dec_dp_shift_amount = 8'(`ARM7_FORWARD(amount_hit, amount_raw));
	end

	always_comb begin : decode
		logic [31:0] instruction;
		logic [15:0] tinsn;
		logic        logical_result;
		logic        set_flags;
		logic  [3:0] opcode;
		logic  [3:0] rd, rn;
		logic  [4:0] rotate_amount;
		logic [31:0] psr_source, psr_mask;
		logic  [4:0] register_count;
		logic  [3:0] thumb_rd;
		logic [31:0] multiply_a, multiply_b;

		instruction = dec_instruction;
		tinsn = dec_tinsn;
		multiply_a = decode_thumb ? reg_thumb_r0 : reg_arm_rm;
		multiply_b = decode_thumb ? reg_thumb_r3 : reg_arm_rs;

		dec_kind = EXEC_UNDEFINED;
		dec_condition = decode_thumb || condition_pass(instruction[31:28], decode_cpsr[31:28]);
		dec_write_reg = 1'b0;
		dec_rd = 4'b0;
		dec_result = 32'b0;
		dec_write_flags = 1'b0;
		dec_flag_mask = 32'b0;
		dec_flag_value = decode_cpsr;
		dec_restore_spsr = 1'b0;
		dec_write_psr = 1'b0;
		dec_write_spsr = 1'b0;
		dec_psr_mask = 32'b0;
		dec_psr_value = 32'b0;
		dec_branch_target = 32'b0;
		dec_branch_thumb = decode_thumb;
		dec_link = 1'b0;
		dec_link_value = 32'b0;

		dec_mem_preindex = 1'b0;
		dec_mem_write_value = 32'b0;
		dec_mem_rd = 4'b0;
		dec_mem_rn = 4'b0;
		dec_mem_size = MEM_WORD;
		dec_mem_load = 1'b0;
		dec_mem_signed = 1'b0;
		dec_mem_writeback = 1'b0;
		dec_mem_load_pc = 1'b0;

		dec_block_list = 16'b0;
		dec_block_addr_delta = 7'b0;
		dec_block_addr_add = 1'b1;
		dec_block_rn = 4'b0;
		dec_block_load = 1'b0;
		dec_block_writeback = 1'b0;
		dec_block_user = 1'b0;
		dec_block_restore_cpsr = 1'b0;

		dec_multiply_operand_a = 32'b0;
		dec_multiply_operand_b = 32'b0;
		dec_multiply_accumulator = 64'b0;
		dec_multiply_rd_lo = 4'b0;
		dec_multiply_rd_hi = 4'b0;
		dec_multiply_extra = 2'd0;
		dec_multiply_long = 1'b0;
		dec_multiply_signed = 1'b0;
		dec_multiply_accumulate = 1'b0;
		dec_multiply_set_flags = 1'b0;

		dec_internal_cycle = 1'b0;
		dec_dp = 1'b0;
		dec_op1_sel = SRC_ZERO;
		dec_op2_sel = SRC_ZERO;
		dec_op2_imm = 32'b0;
		dec_amount_sel = AMT_IMM;
		dec_amount_imm = 8'b0;
		dec_dp_shift_type = 2'b00;
		dec_dp_shift_enable = 1'b0;
		dec_dp_shift_rrx = 1'b0;
		dec_dp_shift_carry = decode_cpsr[29];
		dec_dp_opcode = ALU_MOV;
		dec_dp_cpsr = decode_cpsr;
		dec_dp_spsr = decode_spsr;

		dec_swp_byte = 1'b0;
		dec_swp_store_value = 32'b0;

		logical_result = 1'b0;
		set_flags = 1'b0;
		opcode = 4'b0;
		rd = 4'b0;
		rn = 4'b0;
		rotate_amount = 5'b0;
		psr_source = 32'b0;
		psr_mask = 32'b0;
		register_count = 5'b0;
		thumb_rd = 4'b0;

		if (!prefetch_valid) begin
			dec_kind = EXEC_SIMPLE;
			dec_condition = 1'b0;
		end else if (!decode_thumb) begin
			// BX
			if (instruction[27:4] == 24'h12fff1) begin
				dec_kind = EXEC_BRANCH;
				dec_branch_target = reg_arm_rm;
				dec_branch_thumb = dec_branch_target[0];
				dec_write_flags = 1'b1;
				dec_flag_mask = CPSR_T;
				dec_flag_value[5] = dec_branch_thumb;
			end
			// MRS
			else if ((instruction & 32'h0fbf_0fff) == 32'h010f_0000) begin
				dec_kind = EXEC_SIMPLE;
				dec_write_reg = 1'b1;
				dec_rd = instruction[15:12];
				dec_result = instruction[22] ? decode_spsr : decode_cpsr;
			end
			// MSR register or immediate
			else if (((instruction & 32'h0fb0_fff0) == 32'h0120_f000) ||
			         ((instruction & 32'h0fb0_f000) == 32'h0320_f000)) begin
				dec_kind = EXEC_SIMPLE;
				if (instruction[25]) begin
					rotate_amount = {instruction[11:8], 1'b0};
					psr_source = ror32({24'b0, instruction[7:0]}, rotate_amount);
				end else begin
					psr_source = reg_arm_rm;
				end
				if (instruction[16]) psr_mask[7:0] = 8'hff;
				if (instruction[17]) psr_mask[15:8] = 8'hff;
				if (instruction[18]) psr_mask[23:16] = 8'hff;
				if (instruction[19]) psr_mask[31:24] = 8'hff;
				if ((decode_cpsr[4:0] == MODE_USER) && !instruction[22])
					psr_mask = psr_mask & 32'hff00_0000;
				dec_write_psr = 1'b1;
				dec_write_spsr = instruction[22];
				dec_psr_mask = psr_mask;
				dec_psr_value = psr_source;
			end
			// Multiply long
			else if ((instruction & 32'h0f80_00f0) == 32'h0080_0090) begin
				dec_kind = EXEC_MULTIPLY;
				dec_multiply_operand_a = multiply_a;
				dec_multiply_operand_b = multiply_b;
				dec_multiply_accumulator = {reg_arm_rn, reg_arm_rd};
				dec_multiply_rd_lo = instruction[15:12];
				dec_multiply_rd_hi = instruction[19:16];
				dec_multiply_long = 1'b1;
				dec_multiply_signed = instruction[22];
				dec_multiply_accumulate = instruction[21];
				dec_multiply_set_flags = instruction[20];
				dec_multiply_extra = 2'd1 + {1'b0, instruction[21]};
			end
			// Multiply / multiply-accumulate
			else if ((instruction & 32'h0fc0_00f0) == 32'h0000_0090) begin
				dec_kind = EXEC_MULTIPLY;
				dec_multiply_operand_a = multiply_a;
				dec_multiply_operand_b = multiply_b;
				dec_multiply_accumulator = {32'b0, reg_arm_rd};
				dec_multiply_rd_lo = instruction[19:16];
				dec_multiply_long = 1'b0;
				dec_multiply_accumulate = instruction[21];
				dec_multiply_set_flags = instruction[20];
				dec_multiply_extra = {1'b0, instruction[21]};
			end
			// SWP / SWPB
			else if ((instruction & 32'h0fb0_0ff0) == 32'h0100_0090) begin
				dec_kind = EXEC_SWP;
				// Post-indexed with a zero offset: the address is Rn itself.
				dec_op1_sel = SRC_ARM_RN;
				dec_mem_rd = instruction[15:12];
				// A store of R15 writes it two instructions ahead, the same
				// rule the single and block stores already use.
				dec_swp_store_value = (instruction[3:0] == 4'd15) ?
					decode_visible_pc_p4 : reg_arm_rm;
				dec_swp_byte = instruction[22];
			end
			// Halfword and signed transfer
			else if ((instruction & 32'h0e00_0090) == 32'h0000_0090) begin
				rn = instruction[19:16];
				rd = instruction[15:12];
				dec_op1_sel = SRC_ARM_RN;
				if (instruction[22]) begin
					dec_op2_sel = SRC_IMM;
					dec_op2_imm = {24'b0, instruction[11:8], instruction[3:0]};
				end else begin
					dec_op2_sel = SRC_ARM_RM;
				end
				dec_dp_opcode = instruction[23] ? ALU_ADD : ALU_SUB;
				dec_mem_preindex = instruction[24];
				if (instruction[20] && (instruction[6:5] != 2'b00)) begin
					dec_kind = EXEC_MEMORY;
					dec_mem_load = 1'b1;
					dec_mem_size = instruction[5] ? MEM_HALF : MEM_BYTE;
					dec_mem_signed = instruction[6];
				end else if (!instruction[20] && (instruction[6:5] == 2'b01)) begin
					dec_kind = EXEC_MEMORY;
					dec_mem_load = 1'b0;
					dec_mem_size = MEM_HALF;
				end
				dec_mem_write_value = (rd == 15) ? prefetch_pc + 32'd12 : reg_arm_rd;
				dec_mem_rd = rd;
				dec_mem_rn = rn;
				dec_mem_writeback = instruction[21] || !instruction[24];
				dec_mem_load_pc = instruction[20] && (rd == 15);
			end
			// Data processing
			else if (instruction[27:26] == 2'b00) begin
				dec_kind = EXEC_SIMPLE;
				dec_dp = 1'b1;
				opcode = instruction[24:21];
				rn = instruction[19:16];
				rd = instruction[15:12];
				dec_dp_opcode = opcode;
				dec_op1_sel = SRC_ARM_RN;
				if (instruction[25]) begin
					rotate_amount = {instruction[11:8], 1'b0};
					dec_op2_sel = SRC_IMM;
					dec_op2_imm = ror32({24'b0, instruction[7:0]}, rotate_amount);
					dec_dp_shift_carry = (rotate_amount == 0) ? decode_cpsr[29] :
						dec_op2_imm[31];
				end else if (!instruction[4]) begin
					dec_op2_sel = SRC_ARM_RM;
					dec_dp_shift_enable = 1'b1;
					dec_dp_shift_type = instruction[6:5];
					{dec_dp_shift_rrx, dec_amount_imm} =
						immediate_shift_amount(instruction[6:5], instruction[11:7]);
				end else if (!instruction[7]) begin
					dec_internal_cycle = 1'b1;
					dec_op2_sel = SRC_RM_SHIFT;
					dec_dp_shift_enable = 1'b1;
					dec_dp_shift_type = instruction[6:5];
					dec_amount_sel = AMT_ARM_RS;
				end else begin
					dec_kind = EXEC_UNDEFINED;
				end

				set_flags = instruction[20] || (opcode[3:2] == 2'b10);
				dec_write_reg = !((opcode >= ALU_TST) && (opcode <= ALU_CMN));
				case (opcode)
					ALU_AND, ALU_EOR, ALU_TST, ALU_TEQ,
					ALU_ORR, ALU_MOV, ALU_BIC, ALU_MVN: logical_result = 1'b1;
					default: logical_result = 1'b0;
				endcase
				dec_rd = rd;
				dec_restore_spsr = instruction[20] && (rd == 15) &&
					mode_has_spsr(decode_cpsr[4:0]);
				if (set_flags && !dec_restore_spsr) begin
					dec_write_flags = 1'b1;
					dec_flag_mask = CPSR_N | CPSR_Z | CPSR_C;
					if (!logical_result)
						dec_flag_mask = dec_flag_mask | CPSR_V;
				end
				if (dec_write_reg && (rd == 15)) begin
					dec_kind = EXEC_BRANCH;
					dec_write_reg = 1'b0;
					dec_branch_thumb = dec_restore_spsr ? decode_spsr[5] : 1'b0;
				end
			end
			// Single data transfer
			else if (instruction[27:26] == 2'b01) begin
				dec_kind = EXEC_MEMORY;
				rn = instruction[19:16];
				rd = instruction[15:12];
				dec_op1_sel = SRC_ARM_RN;
				dec_dp_opcode = instruction[23] ? ALU_ADD : ALU_SUB;
				if (!instruction[25]) begin
					dec_op2_sel = SRC_IMM;
					dec_op2_imm = {20'b0, instruction[11:0]};
				end else if (!instruction[4]) begin
					dec_op2_sel = SRC_ARM_RM;
					dec_dp_shift_enable = 1'b1;
					dec_dp_shift_type = instruction[6:5];
					{dec_dp_shift_rrx, dec_amount_imm} =
						immediate_shift_amount(instruction[6:5], instruction[11:7]);
				end else begin
					dec_kind = EXEC_UNDEFINED;
				end
				dec_mem_preindex = instruction[24];
				dec_mem_write_value = (rd == 15) ? prefetch_pc + 32'd12 : reg_arm_rd;
				dec_mem_rd = rd;
				dec_mem_rn = rn;
				dec_mem_size = instruction[22] ? MEM_BYTE : MEM_WORD;
				dec_mem_load = instruction[20];
				dec_mem_writeback = instruction[21] || !instruction[24];
				dec_mem_load_pc = instruction[20] && (rd == 15);
			end
			// Block data transfer
			else if (instruction[27:25] == 3'b100) begin
				dec_kind = EXEC_BLOCK;
				rn = instruction[19:16];
				dec_op1_sel = SRC_ARM_RN;
				register_count = count_registers(instruction[15:0]);
				dec_block_list = instruction[15:0];
				if (register_count == 0) begin
					dec_block_list = 16'h8000;
					register_count = 5'd16;
				end
				// The ALU produces the updated base; the block address adder
				// takes the offset that belongs to this addressing mode.
				dec_op2_sel = SRC_IMM;
				dec_op2_imm = {25'b0, register_count, 2'b00};
				dec_dp_opcode = instruction[23] ? ALU_ADD : ALU_SUB;
				dec_block_addr_add = instruction[23];
				if (instruction[23])
					dec_block_addr_delta = instruction[24] ? 7'd4 : 7'd0;
				else
					dec_block_addr_delta = instruction[24] ?
						{register_count, 2'b00} : {(register_count - 1'b1), 2'b00};
				dec_block_rn = rn;
				dec_block_load = instruction[20];
				// A load of Rn wins when Rn is also in the transfer list.
				dec_block_writeback = instruction[21] &&
					!(instruction[20] && dec_block_list[rn]);
				dec_block_restore_cpsr = instruction[22] && instruction[20] && dec_block_list[15];
				dec_block_user = instruction[22] && !dec_block_restore_cpsr;
			end
			// Branch / branch with link
			else if (instruction[27:25] == 3'b101) begin
				dec_kind = EXEC_BRANCH;
				dec_branch_target = prefetch_pc + 32'd8 +
					{{6{instruction[23]}}, instruction[23:0], 2'b00};
				dec_branch_thumb = 1'b0;
				dec_link = instruction[24];
				dec_link_value = prefetch_pc + 32'd4;
			end
			else if (instruction[27:24] == 4'b1111) begin
				dec_kind = EXEC_SWI;
			end
			else begin
				dec_kind = EXEC_UNDEFINED;
			end
		end else begin
			// Thumb formats 1 and 2: shifts, add, and subtract.
			if (tinsn[15:13] == 3'b000) begin
				dec_kind = EXEC_SIMPLE;
				dec_dp = 1'b1;
				if (tinsn[12:11] != 2'b11) begin
					dec_dp_opcode = ALU_MOV;
					dec_op2_sel = SRC_T_R3;
					dec_dp_shift_enable = 1'b1;
					dec_dp_shift_type = tinsn[12:11];
					{dec_dp_shift_rrx, dec_amount_imm} =
						immediate_shift_amount(tinsn[12:11], tinsn[10:6]);
				end else begin
					dec_dp_opcode = tinsn[9] ? ALU_SUB : ALU_ADD;
					dec_op1_sel = SRC_T_R3;
					if (tinsn[10]) begin
						dec_op2_sel = SRC_IMM;
						dec_op2_imm = {29'b0, tinsn[8:6]};
					end else begin
						dec_op2_sel = SRC_T_R6;
					end
				end
				dec_write_reg = 1'b1;
				dec_rd = {1'b0, tinsn[2:0]};
				dec_write_flags = 1'b1;
				dec_flag_mask = CPSR_N | CPSR_Z | CPSR_C | ((tinsn[12:11] == 2'b11) ? CPSR_V : 32'b0);
			end
			// Immediate MOV/CMP/ADD/SUB.
			else if (tinsn[15:13] == 3'b001) begin
				dec_kind = EXEC_SIMPLE;
				dec_dp = 1'b1;
				dec_op1_sel = SRC_T_R8;
				dec_op2_sel = SRC_IMM;
				dec_op2_imm = {24'b0, tinsn[7:0]};
				case (tinsn[12:11])
					2'b00:   dec_dp_opcode = ALU_MOV;
					2'b01:   dec_dp_opcode = ALU_CMP;
					2'b10:   dec_dp_opcode = ALU_ADD;
					default: dec_dp_opcode = ALU_SUB;
				endcase
				dec_write_reg = (tinsn[12:11] != 2'b01);
				dec_rd = {1'b0, tinsn[10:8]};
				dec_write_flags = 1'b1;
				dec_flag_mask = CPSR_N | CPSR_Z;
				if (tinsn[12:11] != 2'b00)
					dec_flag_mask = dec_flag_mask | CPSR_C | CPSR_V;
			end
			// ALU operations.
			else if (tinsn[15:10] == 6'b010000) begin
				dec_kind = EXEC_SIMPLE;
				dec_dp = 1'b1;
				opcode = tinsn[9:6];
				rd = {1'b0, tinsn[2:0]};
				dec_op1_sel = SRC_T_R0;
				dec_op2_sel = SRC_T_R3;
				dec_write_reg = 1'b1;
				dec_write_flags = 1'b1;
				dec_flag_mask = CPSR_N | CPSR_Z;
				case (opcode)
					4'h0: dec_dp_opcode = ALU_AND;
					4'h1: dec_dp_opcode = ALU_EOR;
					// Shift by a register: Rd is shifted, Rs gives the amount.
					4'h2, 4'h3, 4'h4, 4'h7: begin
						dec_dp_opcode = ALU_MOV;
						dec_op2_sel = SRC_T_R0;
						dec_dp_shift_enable = 1'b1;
						dec_dp_shift_type = (opcode == 4'h2) ? 2'b00 :
							(opcode == 4'h3) ? 2'b01 : (opcode == 4'h4) ? 2'b10 : 2'b11;
						dec_amount_sel = AMT_T_R3;
						dec_flag_mask = dec_flag_mask | CPSR_C;
					end
					4'h5: begin
						dec_dp_opcode = ALU_ADC;
						dec_flag_mask = dec_flag_mask | CPSR_C | CPSR_V;
					end
					4'h6: begin
						dec_dp_opcode = ALU_SBC;
						dec_flag_mask = dec_flag_mask | CPSR_C | CPSR_V;
					end
					4'h8: begin
						dec_dp_opcode = ALU_AND;
						dec_write_reg = 1'b0;
					end
					// NEG is RSB with a zero first operand.
					4'h9: begin
						dec_dp_opcode = ALU_SUB;
						dec_op1_sel = SRC_ZERO;
						dec_flag_mask = dec_flag_mask | CPSR_C | CPSR_V;
					end
					4'ha: begin
						dec_dp_opcode = ALU_CMP;
						dec_write_reg = 1'b0;
						dec_flag_mask = dec_flag_mask | CPSR_C | CPSR_V;
					end
					4'hb: begin
						dec_dp_opcode = ALU_CMN;
						dec_write_reg = 1'b0;
						dec_flag_mask = dec_flag_mask | CPSR_C | CPSR_V;
					end
					4'hc: dec_dp_opcode = ALU_ORR;
					4'hd: begin
						dec_kind = EXEC_MULTIPLY;
						dec_dp = 1'b0;
						dec_write_reg = 1'b0;
						dec_write_flags = 1'b0;
						dec_multiply_operand_a = multiply_a;
						dec_multiply_operand_b = multiply_b;
						dec_multiply_rd_lo = rd;
						dec_multiply_set_flags = 1'b1;
					end
					4'he: dec_dp_opcode = ALU_BIC;
					default: dec_dp_opcode = ALU_MVN;
				endcase
				dec_rd = rd;
			end
			// High-register operations and BX.
			else if (tinsn[15:10] == 6'b010001) begin
				thumb_rd = {tinsn[7], tinsn[2:0]};
				dec_op1_sel = SRC_T_HD;
				dec_op2_sel = SRC_T_HS;
				case (tinsn[9:8])
					2'b00: begin
						dec_dp = 1'b1;
						dec_dp_opcode = ALU_ADD;
						if (thumb_rd == 15) begin
							dec_kind = EXEC_BRANCH; dec_branch_thumb = 1'b1;
						end else begin
							dec_kind = EXEC_SIMPLE; dec_write_reg = 1'b1; dec_rd = thumb_rd[3:0];
						end
					end
					2'b01: begin
						dec_kind = EXEC_SIMPLE;
						dec_dp = 1'b1;
						dec_dp_opcode = ALU_CMP;
						dec_write_flags = 1'b1;
						dec_flag_mask = CPSR_N | CPSR_Z | CPSR_C | CPSR_V;
					end
					2'b10: begin
						if (thumb_rd == 15) begin
							dec_kind = EXEC_BRANCH; dec_branch_target = reg_thumb_hs;
							dec_branch_thumb = 1'b1;
						end else begin
							dec_kind = EXEC_SIMPLE; dec_write_reg = 1'b1;
							dec_rd = thumb_rd[3:0]; dec_result = reg_thumb_hs;
						end
					end
					default: begin
						dec_kind = EXEC_BRANCH;
						dec_branch_target = reg_thumb_hs;
						dec_branch_thumb = reg_thumb_hs[0];
						dec_write_flags = 1'b1;
						dec_flag_mask = CPSR_T;
						dec_flag_value[5] = reg_thumb_hs[0];
					end
				endcase
			end
			// PC-relative load.
			else if (tinsn[15:11] == 5'b01001) begin
				dec_kind = EXEC_MEMORY;
				dec_op1_sel = SRC_PC_ALIGN;
				dec_op2_sel = SRC_IMM;
				dec_op2_imm = {22'b0, tinsn[7:0], 2'b00};
				dec_dp_opcode = ALU_ADD;
				dec_mem_preindex = 1'b1;
				dec_mem_rd = {1'b0, tinsn[10:8]};
				dec_mem_size = MEM_WORD;
				dec_mem_load = 1'b1;
			end
			// Register-offset and signed transfers.
			else if (tinsn[15:12] == 4'b0101) begin
				dec_kind = EXEC_MEMORY;
				dec_op1_sel = SRC_T_R3;
				dec_op2_sel = SRC_T_R6;
				dec_dp_opcode = ALU_ADD;
				dec_mem_preindex = 1'b1;
				dec_mem_rd = {1'b0, tinsn[2:0]};
				dec_mem_rn = {1'b0, tinsn[5:3]};
				dec_mem_write_value = reg_thumb_r0;
				if (!tinsn[9]) begin
					dec_mem_load = tinsn[11];
					dec_mem_size = tinsn[10] ? MEM_BYTE : MEM_WORD;
				end else begin
					case (tinsn[11:10])
						2'b00: begin dec_mem_load = 1'b0; dec_mem_size = MEM_HALF; end
						2'b01: begin dec_mem_load = 1'b1; dec_mem_size = MEM_BYTE; dec_mem_signed = 1'b1; end
						2'b10: begin dec_mem_load = 1'b1; dec_mem_size = MEM_HALF; end
						default: begin dec_mem_load = 1'b1; dec_mem_size = MEM_HALF; dec_mem_signed = 1'b1; end
					endcase
				end
			end
			// Immediate word/byte load and store.
			else if (tinsn[15:13] == 3'b011) begin
				dec_kind = EXEC_MEMORY;
				dec_mem_size = tinsn[12] ? MEM_BYTE : MEM_WORD;
				dec_op1_sel = SRC_T_R3;
				dec_op2_sel = SRC_IMM;
				dec_op2_imm = tinsn[12] ? {27'b0, tinsn[10:6]} : {25'b0, tinsn[10:6], 2'b00};
				dec_dp_opcode = ALU_ADD;
				dec_mem_preindex = 1'b1;
				dec_mem_rd = {1'b0, tinsn[2:0]};
				dec_mem_write_value = reg_thumb_r0;
				dec_mem_load = tinsn[11];
			end
			// Immediate halfword load and store.
			else if (tinsn[15:12] == 4'b1000) begin
				dec_kind = EXEC_MEMORY;
				dec_op1_sel = SRC_T_R3;
				dec_op2_sel = SRC_IMM;
				dec_op2_imm = {26'b0, tinsn[10:6], 1'b0};
				dec_dp_opcode = ALU_ADD;
				dec_mem_preindex = 1'b1;
				dec_mem_rd = {1'b0, tinsn[2:0]};
				dec_mem_write_value = reg_thumb_r0;
				dec_mem_size = MEM_HALF;
				dec_mem_load = tinsn[11];
			end
			// SP-relative load and store.
			else if (tinsn[15:12] == 4'b1001) begin
				dec_kind = EXEC_MEMORY;
				dec_op1_sel = SRC_SP;
				dec_op2_sel = SRC_IMM;
				dec_op2_imm = {22'b0, tinsn[7:0], 2'b00};
				dec_dp_opcode = ALU_ADD;
				dec_mem_preindex = 1'b1;
				dec_mem_rd = {1'b0, tinsn[10:8]};
				dec_mem_write_value = reg_thumb_r8;
				dec_mem_size = MEM_WORD;
				dec_mem_load = tinsn[11];
			end
			// Load address.
			else if (tinsn[15:12] == 4'b1010) begin
				dec_kind = EXEC_SIMPLE;
				dec_dp = 1'b1;
				dec_dp_opcode = ALU_ADD;
				dec_op1_sel = tinsn[11] ? SRC_SP : SRC_PC_ALIGN;
				dec_op2_sel = SRC_IMM;
				dec_op2_imm = {22'b0, tinsn[7:0], 2'b00};
				dec_write_reg = 1'b1;
				dec_rd = {1'b0, tinsn[10:8]};
			end
			// Add or subtract immediate from SP.
			else if (tinsn[15:8] == 8'b10110000) begin
				dec_kind = EXEC_SIMPLE;
				dec_dp = 1'b1;
				dec_dp_opcode = tinsn[7] ? ALU_SUB : ALU_ADD;
				dec_op1_sel = SRC_SP;
				dec_op2_sel = SRC_IMM;
				dec_op2_imm = {23'b0, tinsn[6:0], 2'b00};
				dec_write_reg = 1'b1;
				dec_rd = 4'd13;
			end
			// PUSH and POP.
			else if ((tinsn[15:9] == 7'b1011010) || (tinsn[15:9] == 7'b1011110)) begin
				dec_kind = EXEC_BLOCK;
				dec_block_load = tinsn[11];
				dec_block_rn = 4'd13;
				dec_block_writeback = 1'b1;
				dec_block_list = {tinsn[8] && tinsn[11],
					tinsn[8] && !tinsn[11], 6'b0, tinsn[7:0]};
				register_count = count_registers(dec_block_list);
				if (register_count == 0) begin
					dec_block_list = 16'h8000;
					register_count = 5'd16;
				end
				dec_op1_sel = SRC_SP;
				dec_op2_sel = SRC_IMM;
				dec_op2_imm = {25'b0, register_count, 2'b00};
				dec_dp_opcode = tinsn[11] ? ALU_ADD : ALU_SUB;
				dec_block_addr_add = tinsn[11];
				dec_block_addr_delta = tinsn[11] ? 7'd0 : {register_count, 2'b00};
			end
			// Multiple load and store.
			else if (tinsn[15:12] == 4'b1100) begin
				dec_kind = EXEC_BLOCK;
				dec_block_list = {8'b0, tinsn[7:0]};
				dec_block_rn = {1'b0, tinsn[10:8]};
				dec_block_load = tinsn[11];
				dec_block_writeback = !(tinsn[11] &&
					|(dec_block_list[7:0] & (8'b1 << tinsn[10:8])));
				register_count = count_registers(dec_block_list);
				if (register_count == 0) begin
					dec_block_list = 16'h8000;
					register_count = 5'd16;
				end
				dec_op1_sel = SRC_T_R8;
				dec_op2_sel = SRC_IMM;
				dec_op2_imm = {25'b0, register_count, 2'b00};
				dec_dp_opcode = ALU_ADD;
				dec_block_addr_add = 1'b1;
				dec_block_addr_delta = 7'd0;
			end
			// Conditional branch and SWI.
			else if (tinsn[15:12] == 4'b1101) begin
				if (tinsn[11:8] == 4'hf) begin
					dec_kind = EXEC_SWI;
				end else if (tinsn[11:8] == 4'he) begin
					dec_kind = EXEC_UNDEFINED;
				end else begin
					dec_kind = EXEC_BRANCH;
					dec_condition = condition_pass(tinsn[11:8], decode_cpsr[31:28]);
					dec_branch_target = prefetch_pc + 32'd4 +
						{{23{tinsn[7]}}, tinsn[7:0], 1'b0};
					dec_branch_thumb = 1'b1;
				end
			end
			// Unconditional branch.
			else if (tinsn[15:11] == 5'b11100) begin
				dec_kind = EXEC_BRANCH;
				dec_branch_target = prefetch_pc + 32'd4 +
					{{20{tinsn[10]}}, tinsn[10:0], 1'b0};
				dec_branch_thumb = 1'b1;
			end
			// Long branch with link, high half.
			else if (tinsn[15:11] == 5'b11110) begin
				dec_kind = EXEC_SIMPLE;
				dec_write_reg = 1'b1;
				dec_rd = 4'd14;
				dec_result = prefetch_pc + 32'd4 + {{9{tinsn[10]}}, tinsn[10:0], 12'b0};
			end
			// Long branch with link, low half.
			else if (tinsn[15:11] == 5'b11111) begin
				dec_kind = EXEC_BRANCH;
				dec_dp = 1'b1;
				dec_dp_opcode = ALU_ADD;
				dec_op1_sel = SRC_LR;
				dec_op2_sel = SRC_IMM;
				dec_op2_imm = {20'b0, tinsn[10:0], 1'b0};
				dec_branch_thumb = 1'b1;
				dec_link = 1'b1;
				dec_link_value = (prefetch_pc + 32'd2) | 32'd1;
			end
			else begin
				dec_kind = EXEC_UNDEFINED;
			end
		end
	end

	integer reset_index;
	always_ff @(posedge clk) begin : execute
		retire <= 1'b0;
		state_ready <= 1'b0;

		if (reset) begin
			for (reset_index = 0; reset_index < 15; reset_index = reset_index + 1)
				regs[reset_index] <= 32'b0;
			for (reset_index = 0; reset_index < 7; reset_index = reset_index + 1) begin
				usr_r8_14[reset_index] <= 32'b0;
				fiq_r8_14[reset_index] <= 32'b0;
			end
			for (reset_index = 0; reset_index < 2; reset_index = reset_index + 1) begin
				irq_r13_14[reset_index] <= 32'b0;
				svc_r13_14[reset_index] <= 32'b0;
				abt_r13_14[reset_index] <= 32'b0;
				und_r13_14[reset_index] <= 32'b0;
			end
			spsr_fiq <= 32'b0;
			spsr_irq <= 32'b0;
			spsr_svc <= 32'b0;
			spsr_abt <= 32'b0;
			spsr_und <= 32'b0;
			cpsr <= CPSR_I | CPSR_F | {27'b0, MODE_SVC};
			state <= RUN;
			decode_valid <= 1'b0;
			decode_instruction <= 32'b0;
			decode_pc <= 32'b0;
			prefetch_valid <= 1'b0;
			prefetch_instruction <= 32'b0;
			prefetch_pc <= 32'b0;
			ahead_valid <= 1'b0;
			ahead_instruction <= 32'b0;
			ahead_pc <= 32'b0;
			// synthesis translate_off
			trace_retire_pc <= 32'b0;
			trace_retire_instruction <= 32'b0;
			trace_retire_thumb <= 1'b0;
			trace_retire_exception <= 1'b0;
			trace_retire_exception_return <= 1'b0;
			// synthesis translate_on
			next_fetch_pc <= VECTOR_RESET;
			halted <= 1'b0;
			halt_pc <= VECTOR_RESET;
			state_pc <= VECTOR_RESET;
			state_cpsr <= CPSR_I | CPSR_F | {27'b0, MODE_SVC};
			state_image_valid <= 1'b0;
			state_dirty <= 1'b0;
			halt_capture_pending <= 1'b0;
			boundary_event <= BOUNDARY_NONE;
			boundary_pc <= VECTOR_RESET;
			mem_req <= 1'b0;
			mem_addr <= 32'b0;
			mem_write <= 1'b0;
			mem_wdata <= 32'b0;
			mem_size <= MEM_WORD;
			mem_wstrb <= 4'b0;
			mem_seq <= 1'b0;
			mem_fetch <= 1'b0;
			mem_privileged <= 1'b1;
			mem_lock <= 1'b0;
			state_rdata <= 32'b0;
			data_address <= 2'b0;
			data_writeback_value <= 32'b0;
			data_rd <= 4'b0;
			data_rn <= 4'b0;
			data_size <= MEM_WORD;
			data_load <= 1'b0;
			data_signed <= 1'b0;
			data_writeback <= 1'b0;
			data_load_pc <= 1'b0;
			data_resume_pc <= 32'b0;
			swp_store_value <= 32'b0;
			swp_loaded_value <= 32'b0;
			swp_rd <= 4'b0;
			swp_byte <= 1'b0;
			swp_resume_pc <= 32'b0;
			block_list <= 16'b0;
			block_address <= 32'b0;
			block_writeback_value <= 32'b0;
			block_resume_pc <= 32'b0;
			block_rn <= 4'b0;
			block_index <= 5'd16;
			block_load <= 1'b0;
			block_writeback <= 1'b0;
			block_user <= 1'b0;
			block_restore_cpsr <= 1'b0;
			block_pc_value <= 32'b0;
			multiply_result <= 64'b0;
			multiply_rd_lo <= 4'b0;
			multiply_rd_hi <= 4'b0;
			multiply_wait <= 3'b0;
			multiply_long <= 1'b0;
			multiply_set_flags <= 1'b0;
			multiply_resume_pc <= 32'b0;
			internal_cycle_pending <= 1'b0;
		end else if (ce) begin
			case (state)
				RUN: begin
					if (boundary_event != BOUNDARY_NONE) begin
						if (!mem_req || mem_ready) begin
							mem_req <= 1'b0;
							mem_seq <= 1'b0;
							mem_fetch <= 1'b0;
							mem_lock <= 1'b0;
							case (boundary_event)
								BOUNDARY_HALT: begin
									state <= HALTED;
									halted <= 1'b1;
									halt_pc <= boundary_pc;
									halt_capture_pending <= 1'b1;
									state_image_valid <= 1'b0;
									state_dirty <= 1'b0;
									boundary_event <= BOUNDARY_NONE;
								end
								BOUNDARY_FIQ:
									enter_exception(MODE_FIQ, VECTOR_FIQ, boundary_pc + 32'd4, cpsr, 1'b1);
								BOUNDARY_IRQ:
									enter_exception(MODE_IRQ, VECTOR_IRQ, boundary_pc + 32'd4, cpsr, 1'b0);
								BOUNDARY_PABT:
									enter_exception(MODE_ABT, VECTOR_PABT, boundary_pc + 32'd4, cpsr, 1'b0);
								default: boundary_event <= BOUNDARY_NONE;
							endcase
						end
					end else if (decode_valid && prefetch_valid) begin
						if (exec_condition && exec_internal_cycle &&
							!internal_cycle_pending) begin
							if (!mem_req || mem_ready) begin
								retain_completed_fetch();
								internal_cycle_pending <= 1'b1;
								mem_req <= 1'b0;
								mem_seq <= 1'b0;
								mem_fetch <= 1'b0;
								mem_lock <= 1'b0;
							end
						end else if (!exec_condition) begin
							internal_cycle_pending <= 1'b0;
							finish_sequential(decode_pc + (thumb ? 32'd2 : 32'd4), cpsr[7:0]);
						end else begin
							internal_cycle_pending <= 1'b0;
							case (exec_kind)
								EXEC_SIMPLE: begin
									if (!exec_simple_control || !mem_req || mem_ready) begin
										if (exec_write_reg && (exec_rd < 15))
											regs[exec_rd] <= exec_effective_result;
										if (exec_write_psr && exec_write_spsr && mode_has_spsr(cpsr[4:0]))
											write_spsr_for_mode(cpsr[4:0],
												(current_spsr & ~exec_psr_mask) |
												(exec_psr_value & exec_psr_mask));
										if (exec_restore_spsr) begin
											if (current_spsr[4:0] != cpsr[4:0]) begin
												save_active_bank(cpsr[4:0],
													bank_switch_shared(cpsr[4:0], current_spsr[4:0]));
												load_active_bank(current_spsr[4:0],
													bank_switch_shared(cpsr[4:0], current_spsr[4:0]));
											end
											cpsr <= current_spsr;
										end else if (exec_write_flags ||
											(exec_write_psr && !exec_write_spsr)) begin
											if (mode_valid(exec_effective_post_cpsr[4:0])) begin
												if (exec_effective_post_cpsr[4:0] != cpsr[4:0]) begin
													save_active_bank(cpsr[4:0], bank_switch_shared(
														cpsr[4:0], exec_effective_post_cpsr[4:0]));
													load_active_bank(exec_effective_post_cpsr[4:0],
														bank_switch_shared(cpsr[4:0],
															exec_effective_post_cpsr[4:0]));
												end
												cpsr <= exec_effective_post_cpsr;
											end
										end
										if (exec_restore_spsr) begin
											finish_control(decode_pc + (thumb ? 32'd2 : 32'd4),
												current_spsr[5], current_spsr[7:6],
												current_spsr[4:0]);
										end else if (exec_write_psr && !exec_write_spsr &&
											|(exec_psr_mask[7:0])) begin
											finish_control(decode_pc + 32'd4,
												exec_effective_post_cpsr[5],
												exec_effective_post_cpsr[7:6],
												exec_effective_post_cpsr[4:0]);
										end else begin
											finish_sequential(decode_pc + (thumb ? 32'd2 : 32'd4),
												exec_effective_post_cpsr[7:0]);
										end
									end
								end
								EXEC_BRANCH: begin
									if (!mem_req || mem_ready) begin
										if (exec_link)
											regs[14] <= exec_link_value;
										if (exec_restore_spsr) begin
											if (current_spsr[4:0] != cpsr[4:0]) begin
												save_active_bank(cpsr[4:0],
													bank_switch_shared(cpsr[4:0], current_spsr[4:0]));
												load_active_bank(current_spsr[4:0],
													bank_switch_shared(cpsr[4:0], current_spsr[4:0]));
											end
											cpsr <= current_spsr;
										end else if (exec_write_flags) begin
											cpsr <= exec_effective_post_cpsr;
										end
									finish_control(exec_effective_branch_target, exec_branch_thumb,
										exec_effective_post_cpsr[7:6],
										exec_effective_post_cpsr[4:0]);
									end
								end
								EXEC_MEMORY: begin
									if (!mem_req || mem_ready) begin
										retain_completed_fetch();
										data_address <= dp_mem_address[1:0];
										data_writeback_value <= dp_mem_updated_base;
										data_rd <= exec_mem_rd;
										data_rn <= exec_mem_rn;
										data_size <= exec_mem_size;
										data_load <= exec_mem_load;
										data_signed <= exec_mem_signed;
										data_writeback <= exec_mem_writeback;
										data_load_pc <= exec_mem_load_pc;
										data_resume_pc <= decode_pc + (thumb ? 32'd2 : 32'd4);
										decode_valid <= 1'b0;
										state <= MEM_ACCESS;
										mem_req <= 1'b1;
										mem_addr <= dp_mem_address;
										mem_write <= !exec_mem_load;
										mem_wdata <= store_lanes(exec_mem_write_value, exec_mem_size);
										mem_size <= exec_mem_size;
										mem_wstrb <= exec_mem_load ? 4'b0 :
											store_strobes(dp_mem_address[1:0], exec_mem_size);
										mem_seq <= 1'b0;
										mem_fetch <= 1'b0;
										mem_privileged <= (cpsr[4:0] != MODE_USER);
										mem_lock <= 1'b0;
									end
								end
								EXEC_BLOCK: begin
									if ((!mem_req || mem_ready) && (exec_block_first_index < 16)) begin
										retain_completed_fetch();
										block_list <= exec_block_list;
										block_address <= dp_block_address;
										block_writeback_value <= dp_block_updated_base;
										block_resume_pc <= decode_pc + (thumb ? 32'd2 : 32'd4);
										block_rn <= exec_block_rn;
										block_index <= exec_block_first_index;
										block_load <= exec_block_load;
										block_writeback <= exec_block_writeback;
										block_user <= exec_block_user;
										block_restore_cpsr <= exec_block_restore_cpsr;
										block_pc_value <= decode_pc + (thumb ? 32'd6 : 32'd12);
										decode_valid <= 1'b0;
										state <= BLOCK_ACCESS;
										mem_req <= 1'b1;
										mem_addr <= dp_block_address;
										mem_write <= !exec_block_load;
										if (exec_block_first_index == 15)
											mem_wdata <= decode_pc + (thumb ? 32'd6 : 32'd12);
										else if (exec_block_user)
											mem_wdata <= read_user_reg(exec_block_first_index[3:0]);
										else
											mem_wdata <= read_reg(exec_block_first_index[3:0]);
										mem_size <= MEM_WORD;
										mem_wstrb <= exec_block_load ? 4'b0 : 4'b1111;
										mem_seq <= 1'b0;
										mem_fetch <= 1'b0;
										mem_privileged <= (cpsr[4:0] != MODE_USER);
										mem_lock <= 1'b0;
									end
								end
								EXEC_MULTIPLY: begin
									if (!mem_req || mem_ready) begin
										retain_completed_fetch();
										multiply_result <= multiply_calculated_result;
										multiply_rd_lo <= exec_multiply_rd_lo;
										multiply_rd_hi <= exec_multiply_rd_hi;
										multiply_wait <= dp_multiply_wait;
										multiply_long <= exec_multiply_long;
										multiply_set_flags <= exec_multiply_set_flags;
										multiply_resume_pc <= decode_pc + (thumb ? 32'd2 : 32'd4);
										decode_valid <= 1'b0;
										mem_req <= 1'b0;
										mem_seq <= 1'b0;
										mem_fetch <= 1'b0;
										mem_lock <= 1'b0;
										state <= MUL_WAIT;
									end
								end
								EXEC_SWP: begin
									if (!mem_req || mem_ready) begin
										retain_completed_fetch();
										swp_store_value <= exec_swp_store_value;
										swp_rd <= exec_mem_rd;
										swp_byte <= exec_swp_byte;
										swp_resume_pc <= decode_pc + 32'd4;
										data_address <= dp_mem_address[1:0];
										decode_valid <= 1'b0;
										state <= SWP_READ;
										mem_req <= 1'b1;
										mem_addr <= dp_mem_address;
										mem_write <= 1'b0;
										mem_wdata <= 32'b0;
										mem_size <= exec_swp_byte ? MEM_BYTE : MEM_WORD;
										mem_wstrb <= 4'b0;
										mem_seq <= 1'b0;
										mem_fetch <= 1'b0;
										mem_privileged <= (cpsr[4:0] != MODE_USER);
										mem_lock <= 1'b1;
									end
								end
								EXEC_SWI: begin
									if (!mem_req || mem_ready) begin
										retire <= 1'b1;
										// synthesis translate_off
										trace_retire_pc <= decode_pc;
										trace_retire_instruction <= thumb ?
											{16'b0, (decode_pc[1] ? decode_instruction[31:16] : decode_instruction[15:0])} :
											decode_instruction;
										trace_retire_thumb <= thumb;
										trace_retire_exception <= 1'b1;
										trace_retire_exception_return <= 1'b0;
										// synthesis translate_on
										enter_exception(MODE_SVC, VECTOR_SWI,
											decode_pc + (thumb ? 32'd2 : 32'd4), cpsr, 1'b0);
									end
								end
								default: begin
									if (!mem_req || mem_ready) begin
										retire <= 1'b1;
										// synthesis translate_off
										trace_retire_pc <= decode_pc;
										trace_retire_instruction <= thumb ?
											{16'b0, (decode_pc[1] ? decode_instruction[31:16] : decode_instruction[15:0])} :
											decode_instruction;
										trace_retire_thumb <= thumb;
										trace_retire_exception <= 1'b1;
										trace_retire_exception_return <= 1'b0;
										// synthesis translate_on
										enter_exception(MODE_UND, VECTOR_UND,
											decode_pc + (thumb ? 32'd2 : 32'd4), cpsr, 1'b0);
									end
								end
							endcase
						end
					end else if (!mem_req) begin
						if (halt_req)
							request_boundary(next_fetch_pc, cpsr[7:6]);
						else if (!fiq_n && !cpsr[6])
							request_boundary(next_fetch_pc, cpsr[7:6]);
						else if (!irq_n && !cpsr[7])
							request_boundary(next_fetch_pc, cpsr[7:6]);
						else
							issue_fetch(next_fetch_pc, thumb, cpsr[4:0], 1'b0);
					end else if (mem_ready) begin
						if (mem_abort) begin
							boundary_event <= BOUNDARY_PABT;
							boundary_pc <= mem_addr;
							decode_valid <= 1'b0;
							prefetch_valid <= 1'b0;
							ahead_valid <= 1'b0;
							mem_req <= 1'b0;
							mem_seq <= 1'b0;
							mem_fetch <= 1'b0;
							mem_lock <= 1'b0;
						end else if (halt_req || (!fiq_n && !cpsr[6]) || (!irq_n && !cpsr[7])) begin
							if (decode_valid)
								request_boundary(decode_pc, cpsr[7:6]);
							else if (prefetch_valid)
								request_boundary(prefetch_pc, cpsr[7:6]);
							else
								request_boundary(mem_addr, cpsr[7:6]);
						end else if (decode_valid || !prefetch_valid) begin
							prefetch_instruction <= mem_rdata;
							prefetch_pc <= mem_addr;
							prefetch_valid <= 1'b1;
							issue_fetch(next_fetch_pc, thumb, cpsr[4:0], 1'b1);
						end else begin
							capture_prefetch_decode();
							prefetch_instruction <= mem_rdata;
							prefetch_pc <= mem_addr;
							prefetch_valid <= 1'b1;
							issue_fetch(next_fetch_pc, thumb, cpsr[4:0], 1'b1);
						end
					end
				end

				MEM_ACCESS: begin
					if (mem_req && mem_ready) begin
						if (mem_abort) begin
							enter_exception(MODE_ABT, VECTOR_DABT,
								data_resume_pc + (cpsr[5] ? 32'd6 : 32'd4), cpsr, 1'b0);
						end else begin
							if (data_writeback && (data_rn < 15))
								regs[data_rn] <= data_writeback_value;
							if (data_load && !data_load_pc)
								regs[data_rd] <= data_loaded_value;
							if (data_load_pc)
								finish_control(data_loaded_value, cpsr[5], cpsr[7:6], cpsr[4:0]);
							else
								finish_sequential(data_resume_pc, cpsr[7:0]);
							state <= RUN;
						end
					end
				end

				SWP_READ: begin
					if (mem_req && mem_ready) begin
						if (mem_abort) begin
							enter_exception(MODE_ABT, VECTOR_DABT, swp_resume_pc + 32'd4, cpsr, 1'b0);
						end else begin
							swp_loaded_value <= load_lane(mem_rdata, data_address,
								swp_byte ? MEM_BYTE : MEM_WORD, 1'b0);
							state <= SWP_WRITE;
							mem_req <= 1'b1;
							mem_write <= 1'b1;
							mem_wdata <= store_lanes(swp_store_value,
								swp_byte ? MEM_BYTE : MEM_WORD);
							mem_wstrb <= store_strobes(data_address,
								swp_byte ? MEM_BYTE : MEM_WORD);
							mem_seq <= 1'b0;
							mem_lock <= 1'b1;
						end
					end
				end

				SWP_WRITE: begin
					if (mem_req && mem_ready) begin
						mem_lock <= 1'b0;
						if (mem_abort) begin
							enter_exception(MODE_ABT, VECTOR_DABT, swp_resume_pc + 32'd4, cpsr, 1'b0);
						end else begin
							regs[swp_rd] <= swp_loaded_value;
							finish_sequential(swp_resume_pc, cpsr[7:0]);
							state <= RUN;
						end
					end
				end

				BLOCK_ACCESS: begin
					if (mem_req && mem_ready) begin
						if (mem_abort) begin
							enter_exception(MODE_ABT, VECTOR_DABT,
								block_resume_pc + (cpsr[5] ? 32'd6 : 32'd4), cpsr, 1'b0);
						end else begin
							if (block_load) begin
								if (block_index == 15) begin
									// Apply a loaded PC only after the final transfer.
								end
								else if (block_user)
									write_user_register(block_index[3:0], mem_rdata);
								else
									regs[block_index[3:0]] <= mem_rdata;
							end
							if (block_next_index < 16) begin
								block_list <= block_remaining;
								block_index <= block_next_index;
								block_address <= block_address + 32'd4;
								mem_addr <= block_address + 32'd4;
								mem_write <= !block_load;
								if (block_next_index == 15)
									mem_wdata <= block_pc_value;
								else if (block_user)
									mem_wdata <= read_user_reg(block_next_index[3:0]);
								else
									mem_wdata <= reg_with_writeback(block_next_index[3:0]);
								mem_wstrb <= block_load ? 4'b0 : 4'b1111;
								mem_seq <= 1'b1;
							end else begin
								if (block_writeback && (block_rn < 15))
									regs[block_rn] <= block_writeback_value;
								if (block_load && (block_index == 15)) begin
									if (block_restore_cpsr) begin
										if (current_spsr[4:0] != cpsr[4:0]) begin
											save_active_bank_wb(cpsr[4:0],
												bank_switch_shared(cpsr[4:0], current_spsr[4:0]));
											load_active_bank(current_spsr[4:0],
												bank_switch_shared(cpsr[4:0], current_spsr[4:0]));
										end
										cpsr <= current_spsr;
										finish_control(mem_rdata, current_spsr[5],
											current_spsr[7:6], current_spsr[4:0]);
									end else begin
										finish_control(mem_rdata, cpsr[5], cpsr[7:6], cpsr[4:0]);
									end
								end else begin
									finish_sequential(block_resume_pc, cpsr[7:0]);
								end
								state <= RUN;
							end
						end
					end
				end

				MUL_WAIT: begin
					if (multiply_wait > 1) begin
						multiply_wait <= multiply_wait - 1'b1;
					end else begin
						regs[multiply_rd_lo] <= multiply_result[31:0];
						if (multiply_long)
							regs[multiply_rd_hi] <= multiply_result[63:32];
						if (multiply_set_flags)
							cpsr <= multiply_post_cpsr;
						finish_sequential(multiply_resume_pc, multiply_post_cpsr[7:0]);
						state <= RUN;
					end
				end

				default: begin // HALTED
					mem_req <= 1'b0;
					mem_seq <= 1'b0;
					mem_fetch <= 1'b0;
					mem_lock <= 1'b0;
					if (halt_capture_pending) begin
						save_active_bank(cpsr[4:0], 1'b1);
						state_pc <= halt_pc;
						state_cpsr <= cpsr;
						state_image_valid <= 1'b1;
						state_dirty <= 1'b0;
						halt_capture_pending <= 1'b0;
					end else begin
						if (state_req && state_image_valid && (state_index <= STATE_UND_SPSR)) begin
							state_ready <= 1'b1;
							if (!state_write) begin
								if (state_index <= STATE_R7)
									state_rdata <= regs[state_index[3:0]];
								else if ((state_index >= STATE_USR_R8) && (state_index <= STATE_USR_R14))
									state_rdata <= usr_r8_14[state_user_index];
								else if (state_index == STATE_PC)
									state_rdata <= state_pc;
								else if (state_index == STATE_CPSR)
									state_rdata <= state_cpsr;
								else if ((state_index >= STATE_FIQ_R8) && (state_index <= STATE_FIQ_R14))
									state_rdata <= fiq_r8_14[state_fiq_index];
								else begin
									case (state_index)
										STATE_FIQ_SPSR: state_rdata <= spsr_fiq;
										STATE_IRQ_R13:  state_rdata <= irq_r13_14[0];
										STATE_IRQ_R14:  state_rdata <= irq_r13_14[1];
										STATE_IRQ_SPSR: state_rdata <= spsr_irq;
										STATE_SVC_R13:  state_rdata <= svc_r13_14[0];
										STATE_SVC_R14:  state_rdata <= svc_r13_14[1];
										STATE_SVC_SPSR: state_rdata <= spsr_svc;
										STATE_ABT_R13:  state_rdata <= abt_r13_14[0];
										STATE_ABT_R14:  state_rdata <= abt_r13_14[1];
										STATE_ABT_SPSR: state_rdata <= spsr_abt;
										STATE_UND_R13:  state_rdata <= und_r13_14[0];
										STATE_UND_R14:  state_rdata <= und_r13_14[1];
										default:         state_rdata <= spsr_und;
									endcase
								end
							end else begin
								state_dirty <= 1'b1;
								if (state_index <= STATE_R7)
									regs[state_index[3:0]] <= state_wdata;
								else if ((state_index >= STATE_USR_R8) && (state_index <= STATE_USR_R14))
									usr_r8_14[state_user_index] <= state_wdata;
								else if (state_index == STATE_PC)
									state_pc <= state_wdata;
								else if (state_index == STATE_CPSR)
									state_cpsr <= state_wdata;
								else if ((state_index >= STATE_FIQ_R8) && (state_index <= STATE_FIQ_R14))
									fiq_r8_14[state_fiq_index] <= state_wdata;
								else begin
									case (state_index)
										STATE_FIQ_SPSR: spsr_fiq <= state_wdata;
										STATE_IRQ_R13:  irq_r13_14[0] <= state_wdata;
										STATE_IRQ_R14:  irq_r13_14[1] <= state_wdata;
										STATE_IRQ_SPSR: spsr_irq <= state_wdata;
										STATE_SVC_R13:  svc_r13_14[0] <= state_wdata;
										STATE_SVC_R14:  svc_r13_14[1] <= state_wdata;
										STATE_SVC_SPSR: spsr_svc <= state_wdata;
										STATE_ABT_R13:  abt_r13_14[0] <= state_wdata;
										STATE_ABT_R14:  abt_r13_14[1] <= state_wdata;
										STATE_ABT_SPSR: spsr_abt <= state_wdata;
										STATE_UND_R13:  und_r13_14[0] <= state_wdata;
										STATE_UND_R14:  und_r13_14[1] <= state_wdata;
										default:         spsr_und <= state_wdata;
									endcase
								end
							end
						end
						if (state_commit && state_image_valid && mode_valid(state_cpsr[4:0]) &&
							((state_cpsr[5] && !state_pc[0]) || (!state_cpsr[5] && (state_pc[1:0] == 0)))) begin
							load_active_bank(state_cpsr[4:0], 1'b1);
							cpsr <= state_cpsr;
							halt_pc <= align_pc(state_pc, state_cpsr[5]);
							decode_valid <= 1'b0;
							prefetch_valid <= 1'b0;
							ahead_valid <= 1'b0;
							internal_cycle_pending <= 1'b0;
							state_dirty <= 1'b0;
						end
						if (!halt_req && state_image_valid && !state_dirty && !state_commit) begin
							halted <= 1'b0;
							state <= RUN;
							issue_fetch(halt_pc, cpsr[5], cpsr[4:0], 1'b0);
						end
					end
				end
			endcase
		end
	end
	`undef ARM7_READ_DECODE_REG
endmodule
