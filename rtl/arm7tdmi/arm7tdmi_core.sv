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
#(
	// A DELIBERATE CYCLE-TIMING INACCURACY, OFF BY DEFAULT.
	//
	// On, every multiply takes one more internal cycle than the real part:
	// DDI 0210C gives MUL, MLA, UMULL, UMLAL, SMULL and SMLAL as 1S+mI and this
	// makes them 1S+(m+1)I. Results, flags and register writes are untouched -
	// the only thing that moves is when the next instruction starts. The
	// cycle-count checks in the testbenches are expected to differ on the
	// multiply forms, by exactly one I cycle each and nothing else.
	//
	// It buys timing. `multiply_wait <= 1` is the retire decision, and unstaged
	// it gates the instruction issue, the forwarding network and the CPSR flag
	// read on one edge, from a single register with thousands of loads. Staging
	// it makes a multiply finish the way a load already does -
	// start_internal_completion into MEM_DONE, which issues the next
	// instruction from registers an edge later - and that deletes the MUL_WAIT
	// arms of decode_forwarding and decode_cpsr outright, because by then the
	// result is in `rf` and the flags are in `cpsr`.
	//
	// Turn it on only where the extra cycle is affordable, and measure rather
	// than assume it is. A host that stalls its bus master for the whole call
	// does not hide the cost - there the CPU's time is the caller's time, so
	// the multiply gets slower and so does the caller. A core that needs the
	// manual's numbers must leave it off, which is why off is the default.
	parameter bit MUL_RETIRE_STAGE = 1'b0
) (
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
		MEM_DONE,
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

	// One flat physical register file. A mode switch changes only the
	// index mapping - no data ever moves between banks, which deletes the
	// whole save/reload selector fabric the area histogram measured at a
	// third of the core. Layout: 0-7 R0-R7; 8-14 the user/system R8-R14
	// (shared with every mode but FIQ for R8-R12); 15-21 FIQ R8-R14;
	// 22-29 the R13/R14 pairs of IRQ, SVC, ABT, UND.
	logic [31:0] rf [0:29];
	localparam logic [4:0] RF_FIQ_OFF = 5'd7;
	localparam logic [4:0] RF_IRQ_R13 = 5'd22;
	localparam logic [4:0] RF_SVC_R13 = 5'd24;
	localparam logic [4:0] RF_ABT_R13 = 5'd26;
	localparam logic [4:0] RF_UND_R13 = 5'd28;

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
	// A prefetch abort travels with the word. The part flags the aborted
	// fetch and takes the exception only when that word reaches execute -
	// fetching continues meanwhile, and a branch that discards the word
	// discards the abort with it (DDI 0210C 9.3, Table 6-15 for the entry).
	logic        ahead_abort;
	logic        prefetch_abort;
	logic        exec_abort;
	// synthesis translate_off
	/* verilator lint_off UNUSEDSIGNAL */
	logic [31:0] trace_retire_pc;
	logic [31:0] trace_retire_instruction;
	logic        trace_retire_thumb;
	logic        trace_retire_exception;
	logic        trace_retire_exception_return;
	/* verilator lint_on UNUSEDSIGNAL */
	// synthesis translate_on
	logic [31:0] fetch_pc;
	logic [31:0] fetch_pc_next;

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

	// The registered rest-list: indices strictly after the one in flight.
	// The next index is its lowest set bit - one isolate and a shallow
	// encode from a register, replacing the decode-clear-priority chain
	// that a fit measured launching into the shifter.
	logic [15:0] block_rest;
	logic [31:0] block_address;
	logic [31:0] block_writeback_value;
	logic [31:0] block_resume_pc;
	logic  [3:0] block_rn;
	logic  [4:0] block_index;
	logic        block_load;
	logic        block_writeback;
	logic        block_user;
	logic        block_updates_live;
	logic        block_restore_cpsr;
	logic [31:0] block_pc_value;

	logic [63:0] multiply_result;
	logic  [3:0] multiply_rd_lo;
	logic  [3:0] multiply_rd_hi;
	logic  [2:0] multiply_wait;
	// `multiply_wait <= 1` is the multiply's retire decision, and one register
	// carrying it reaches the instruction issue, the forwarding network and the
	// register file at once - thousands of loads, and enough routing to reach
	// the first LUT to matter. The count is loaded at entry and only ever
	// decrements, so the decision is known an edge early: it is registered here
	// rather than recomputed from the counter at each read. One copy per
	// consumer group so each can sit with its loads - dont_merge keeps the
	// groups apart, maxfan lets the fitter replicate inside one.
	logic        mul_retire_seq   /* synthesis dont_merge maxfan = 32 */;
	logic        mul_retire_fwd   /* synthesis dont_merge maxfan = 32 */;
	logic        mul_retire_flags /* synthesis dont_merge maxfan = 32 */;
	logic [63:0] mul_acc;
	logic [31:0] mul_mplier;
	logic [31:0] mul_mcand;
	logic  [1:0] mul_group;
	logic  [2:0] mul_groups_left;
	logic [63:0] mul_acc_next;
	logic [63:0] multiply_final_result;
	logic [31:0] mul_mplier_full;
	logic [31:0] mc_accum;
	logic [31:0] mc_carry;
	logic [31:0] mc_accum_next;
	logic [31:0] mc_carry_next;
	logic        mul_full;
	logic        multiply_carry_final;
	logic        multiply_signed;
	logic [31:0] multiply_seed_hi;
	logic        multiply_long;
	logic        multiply_set_flags;
	logic [31:0] multiply_resume_pc;
	logic        internal_cycle_pending;
	logic [31:0] mem_done_pc;
	logic        mem_done_control;

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
	logic  [2:0] dp_multiply_groups;
	logic [63:0] multiply_seed;
	logic [31:0] exec_effective_result;
	logic [31:0] exec_effective_post_cpsr;
	logic [31:0] exec_effective_branch_target;
	logic [31:0] dec_post_cpsr;
	logic        block_last;
	logic  [4:0] block_next_index;
	logic  [4:0] dec_block_first_index;
	logic [31:0] data_loaded_value;
	logic [31:0] multiply_post_cpsr;
	logic  [4:0] state_rf_index;

	assign thumb = |(cpsr & CPSR_T);
	// The next sequential fetch address: the last issued fetch plus one
	// step. The add happens here, at use, from a register - not at issue
	// time, where it sat in series after the ALU on every branch (the
	// measured worst path ran shifter -> ALU -> align -> +step ->
	// fetch_pc_next). By any use of this wire, `thumb` already equals the
	// in-flight fetch's instruction set: everything that changes T issues
	// its target fetch on the same edge it writes CPSR.
	assign fetch_pc_next = fetch_pc + (thumb ? 32'd2 : 32'd4);
	assign visible_pc = decode_pc + (thumb ? 32'd4 : 32'd8);
	// The registered T bit, not the forwarded one, on purpose. decode's
	// outputs are only captured on `finish_sequential` paths, and nothing
	// that continues sequentially can change T or the mode: BX, exception
	// entry and return, and a control-byte MSR all flush the pipeline
	// through `finish_control` or `enter_exception`, and a flags-only
	// write's mask stops at bit 24. Reading `cpsr` here takes the whole
	// ARM/Thumb split of the decode chain - the port indexes, `dec_tinsn`,
	// every format test - off the ALU's forwarding cone.
	assign decode_thumb = thumb;
	assign decode_visible_pc = prefetch_pc + (decode_thumb ? 32'd4 : 32'd8);
	assign decode_visible_pc_p4 = prefetch_pc + (decode_thumb ? 32'd8 : 32'd12);
	assign decode_forward_tags = {decode_forward_0_valid, decode_forward_0_index,
		decode_forward_1_valid, decode_forward_1_index};
	assign decode_forward_values = {decode_forward_0_value, decode_forward_1_value};

	// Logical register to physical slot under a mode. Every input is a
	// registered value, so the translation is ready long before the read
	// data is needed.
	function automatic logic [4:0] rf_index(
		input logic [3:0] index,
		input logic [4:0] mode
	);
		begin
			if (index < 8)
				rf_index = {1'b0, index};
			else if (index < 13)
				rf_index = (mode == MODE_FIQ) ?
					({1'b0, index} + RF_FIQ_OFF) : {1'b0, index};
			else begin
				case (mode)
					MODE_FIQ: rf_index = {1'b0, index} + RF_FIQ_OFF;
					MODE_IRQ: rf_index = RF_IRQ_R13 + {4'b0, index[1]};
					MODE_SVC: rf_index = RF_SVC_R13 + {4'b0, index[1]};
					MODE_ABT: rf_index = RF_ABT_R13 + {4'b0, index[1]};
					MODE_UND: rf_index = RF_UND_R13 + {4'b0, index[1]};
					default:  rf_index = {1'b0, index};
				endcase
			end
		end
	endfunction

	function automatic logic [31:0] read_reg(input logic [3:0] index);
		begin
			read_reg = (index >= 15) ? visible_pc :
				rf[rf_index(index, cpsr[4:0])];
		end
	endfunction

	function automatic logic [31:0] read_decode_reg(input logic [3:0] index);
		begin
			read_decode_reg = (index >= 15) ? decode_visible_pc :
				rf[rf_index(index, cpsr[4:0])];
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
	logic        thumb_hi_form;
	logic  [3:0] idx_port_a, idx_port_b, idx_port_c, idx_port_d;
	logic [31:0] raw_port_a, raw_port_b, raw_port_c, raw_port_d;

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

	// The multiply encodings read every register operand after the PC has
	// stepped, so R15 anywhere in one reads PC+12 (oracle-verified). The
	// same override the register-shift form uses, on all four ports.
	logic arm_multiply_form;
	assign arm_multiply_form = ((dec_instruction[27:22] == 6'b000000) ||
		(dec_instruction[27:23] == 5'b00001)) &&
		(dec_instruction[7:4] == 4'b1001);

	// Four read ports, not ten. ARM and Thumb never decode at once, so their
	// field positions share ports; and the Thumb hi-register forms differ from
	// the low ones only in the extra index bit, so they share too. Ten 16-to-1
	// 32-bit muxes was 91% of this core's logic being multiplexing - and ten
	// readers on `regs` is ten fan-out cones through a design that decision
	// 0041 measured as interconnect-bound.
	//
	// The cost is one 4-bit mux ahead of each read, which is a single level on
	// a 4-bit signal, against six 32-bit read muxes removed.
	assign thumb_hi_form = (dec_tinsn[15:10] == 6'b010001);

	// The four port indexes are pre-decoded when the word is captured into
	// the prefetch slot and registered beside it, so the read ports launch
	// from flops rather than from a mux over the instruction fields - the
	// fit after the multiplier diet put that mux at the head of the
	// operand loop. `thumb` at capture time is the T the word will decode
	// under: nothing that changes T keeps a captured word.
	function automatic logic [15:0] port_indexes(
		input logic [31:0] word,
		input logic        upper_half,
		input logic        thumb_now
	);
		logic [15:0] t;
		logic        hi_form;
		begin
			t = upper_half ? word[31:16] : word[15:0];
			hi_form = (t[15:10] == 6'b010001);
			if (thumb_now)
				port_indexes = {{hi_form & t[7], t[2:0]},
					{hi_form & t[6], t[5:3]},
					{1'b0, t[8:6]}, {1'b0, t[10:8]}};
			else
				port_indexes = {word[19:16], word[15:12], word[11:8], word[3:0]};
		end
	endfunction
	logic [15:0] prefetch_ports;
	assign {idx_port_a, idx_port_b, idx_port_c, idx_port_d} = prefetch_ports;

	assign raw_port_a = read_decode_reg(idx_port_a);
	assign raw_port_b = read_decode_reg(idx_port_b);
	assign raw_port_c = read_decode_reg(idx_port_c);
	assign raw_port_d = read_decode_reg(idx_port_d);

	assign raw_arm_rn   = ((arm_register_shift || arm_multiply_form) &&
		(idx_arm_rn == 4'd15)) ? decode_visible_pc_p4 : raw_port_a;
	assign raw_arm_rd   = (arm_multiply_form && (idx_arm_rd == 4'd15)) ?
		decode_visible_pc_p4 : raw_port_b;
	// Rs=15 is UNPREDICTABLE; the part reads a shift amount before the PC
	// steps for the register-shift form, so it sees PC+8 where the Rn and
	// Rm reads of the same instruction see PC+12 - but a multiplier from
	// R15 is read after the step, PC+12 (both oracle-verified).
	assign raw_arm_rs   = (arm_multiply_form && (idx_arm_rs == 4'd15)) ?
		decode_visible_pc_p4 : raw_port_c;
	assign raw_arm_rm   = ((arm_register_shift || arm_multiply_form) &&
		(idx_arm_rm == 4'd15)) ? decode_visible_pc_p4 : raw_port_d;
	assign raw_thumb_r0 = raw_port_a;
	assign raw_thumb_r3 = raw_port_b;
	assign raw_thumb_r6 = raw_port_c;
	assign raw_thumb_r8 = raw_port_d;
	assign raw_thumb_hd = raw_port_a;
	assign raw_thumb_hs = raw_port_b;
	assign raw_sp       = rf[rf_index(4'd13, cpsr[4:0])];
	assign raw_lr       = rf[rf_index(4'd14, cpsr[4:0])];

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
		read_user_reg = (index >= 15) ? visible_pc :
			rf[rf_index(index, MODE_USER)];
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
	// The next list position is registered a cycle ahead - the fit that
	// followed the comb isolate put its carry chain on the worst path,
	// gating the register-file write enables. Every use below reads a
	// register; the isolate runs from `block_rest` into registers only.
	assign block_last = block_next_index[4];
	assign dec_block_first_index = first_set_register(dec_block_list);
	assign data_loaded_value = load_lane(mem_rdata, data_address, data_size, data_signed);
	// The halted-state map is the flat file plus PC, CPSR and the five
	// SPSRs. Registers 0-14 map straight through; each later bank sits one
	// SPSR slot further out of step with the physical layout.
	always_comb begin
		if (state_index <= STATE_USR_R14)
			state_rf_index = state_index[4:0];
		else if (state_index <= STATE_FIQ_R14)
			state_rf_index = state_index[4:0] - 5'd2;   // 17-23 -> 15-21
		else if (state_index <= STATE_IRQ_R14)
			state_rf_index = state_index[4:0] - 5'd3;   // 25-26 -> 22-23
		else if (state_index <= STATE_SVC_R14)
			state_rf_index = state_index[4:0] - 5'd4;   // 28-29 -> 24-25
		else if (state_index <= STATE_ABT_R14)
			state_rf_index = state_index[4:0] - 5'd5;   // 31-32 -> 26-27
		else
			state_rf_index = state_index[4:0] - 5'd6;   // 34-35 -> 28-29
	end
	// One 8-bit group of the multiplier per cycle, which is the rate the part
	// works at and the rate `multiply_cycle_count` already bills for. The
	// signed corrections do not depend on the iteration, so they are folded
	// into the accumulator when the multiply starts and never appear here:
	// what is left on this path is a 32x8 product and one 64-bit add.
	// One physical 32x8 product serves both the entry cycle (operands from
	// the execute registers) and the MUL_WAIT iterations (operands from the
	// iteration state) - they are never simultaneous.
	logic [31:0] mul_prod_a;
	logic  [7:0] mul_prod_b;
	logic [39:0] mul_prod;
	assign mul_prod_a = (state == MUL_WAIT) ? mul_mcand :
		exec_multiply_operand_a;
	assign mul_prod_b = (state == MUL_WAIT) ? mul_mplier[7:0] :
		exec_multiply_operand_b[7:0];
	assign mul_prod = mul_prod_a * mul_prod_b;

	always_comb begin : multiply_iteration
		logic [39:0] partial;
		logic [63:0] placed;
		partial = mul_prod;
		case (mul_group)
			2'd0: placed = {24'b0, partial};
			2'd1: placed = {16'b0, partial, 8'b0};
			2'd2: placed = {8'b0, partial, 16'b0};
			default: placed = {partial, 24'b0};
		endcase
		mul_acc_next = mul_acc + placed;
	end

	// The carry a multiply leaves in the C flag comes out of the array's
	// carry-save adder, so it is walked alongside the product: four Booth
	// iterations per cycle, over the same eight multiplier bits.
	//
	// The model is zaydlang and calc84maniac's, by way of NanoBoyAdvance,
	// under its zlib terms; this is an altered version, rewritten from the
	// software form and not to be mistaken for the original. Software carries
	// the pair as `sum` with `carry = sum - accum`; those stand for a
	// carry-save adder, which is what this is - an XOR and a majority per
	// step, no carry propagation anywhere.
	//
	// The addend is a Booth digit times the multiplicand, and the digit's
	// weight is fixed by the step's position, so it is a select over constant
	// shifts rather than a multiply. Setting bit 0 of the multiplicand is the
	// model's trick: it makes a negated addend invert the upper bits without
	// the low bit ever reaching the carry.
	// One group of the carry walk: four Booth iterations over eight
	// multiplier bits. A function, because the entry cycle walks group 0
	// from the execute registers while MUL_WAIT walks the later groups
	// from the iteration state - the retire cycle then reads only
	// registered values, keeping the multiplier off the forwarding loop.
	function automatic logic [63:0] csa_walk(
		input logic [31:0] accum_in,
		input logic [31:0] carry_in_w,
		input logic [31:0] mcand_in,
		input logic [31:0] mplier,
		input logic  [1:0] group
	);
		logic [31:0] accum, carry, addend, mcand, maj;
		logic  [2:0] triple;
		logic  [5:0] index;
		begin
			accum = accum_in;
			carry = carry_in_w;
			mcand = mcand_in | 32'd1;
			for (int i = 0; i < 4; i++) begin
				// The part's factors sit at odd positions: iteration k
				// recodes {b[2k+2], b[2k+1], b[2k]} at weight 2^(2k+1).
				// Early termination caps the index at 23, so index+1
				// never leaves the word.
				index = {1'b0, group, 3'b000} + 6'(i * 2) + 6'd1;
				triple = {mplier[index + 6'd1], mplier[index],
					mplier[index - 6'd1]};
				case (triple)
					3'b001, 3'b010: addend = mcand << index;
					3'b011:         addend = mcand << (index + 6'd1);
					3'b100:         addend = -(mcand << (index + 6'd1));
					3'b101, 3'b110: addend = -(mcand << index);
					default:        addend = 32'b0;
				endcase
				maj = (accum & carry) | (accum & addend) | (carry & addend);
				accum = accum ^ carry ^ addend;
				carry = {maj[30:0], 1'b0};
			end
			csa_walk = {accum, carry};
		end
	endfunction

	// The walk is shared the same way: entry feeds it the model preamble
	// and group 0, MUL_WAIT the registered state and the current group.
	assign {mc_accum_next, mc_carry_next} = (state == MUL_WAIT) ?
		csa_walk(mc_accum, mc_carry, mul_mcand, mul_mplier_full, mul_group) :
		csa_walk(exec_multiply_accumulate ?
				exec_multiply_accumulator[31:0] : 32'b0,
			exec_multiply_operand_b[0] ?
				-(exec_multiply_operand_a | 32'd1) : 32'b0,
			exec_multiply_operand_a, exec_multiply_operand_b, 2'd0);

	// When every group ran there is a closed form, because only the final
	// injected Booth carry survives: the last addend is negative exactly when
	// the multiplier's top two bits are 10.
	function automatic logic mul_carry_simple(input logic [31:0] multiplier);
		mul_carry_simple = (multiplier[31:30] == 2'b10);
	endfunction

	// The long forms take their carry from bit 63, where only the last three
	// Booth iterations reach. Both operands are scaled down so those land in
	// the top of a 32-bit value; the magic constants pre-place the carry and
	// accumulator bits the model needs at 63-60.
	// Signed 27x7 product, explicitly widened (assignment context), then
	// truncated to 31 bits: exact mod 2^31, which is all the masked bit
	// 30:28 extractions below ever read - and small enough to stay in LUTs.
	// The multiplicand arrives as 27 SIGNED bits: after the >>6 scaling a
	// signed value spans [-2^25, 2^25), so bit 26 is its sign, and the
	// unsigned path's values sit in the positive half.
	// The factor needs 8 signed bits: the unsigned path's scaled
	// multiplier runs to 63, so factor0 spans -47..79 (a 400k-vector
	// software differential pinned the 7-bit version's failures to
	// exactly that range).
	function automatic logic [30:0] booth_hi_prod(
		input logic [26:0] m,
		input logic  [7:0] f
	);
		logic signed [34:0] prod_s;
		begin
			prod_s = $signed(m) * $signed(f);
			booth_hi_prod = prod_s[30:0];
		end
	endfunction

	function automatic logic mul_carry_hi(
		input logic [31:0] multiplicand_in,
		input logic [31:0] multiplier_in,
		input logic [31:0] accum_hi,
		input logic        sign_extend
	);
		logic [31:0] mcand, mplier, carry, accum, sum, addend;
		logic [31:0] booth0, booth1, booth2, factor0, factor1, factor2;
		begin
			if (sign_extend) begin
				mcand = $unsigned($signed(multiplicand_in) >>> 6);
				mplier = $unsigned($signed(multiplier_in) >>> 26);
			end else begin
				mcand = multiplicand_in >> 6;
				mplier = multiplier_in >> 26;
			end
			mcand = mcand | 32'd1;
			carry = ~accum_hi & 32'h2000_0000;
			accum = accum_hi - 32'h0800_0000;
			booth0 = $unsigned($signed(mplier << 27) >>> 27);
			booth1 = $unsigned($signed(mplier << 29) >>> 29);
			booth2 = {32{mplier[0]}};
			factor0 = mplier - booth0;
			factor1 = booth0 - booth1;
			factor2 = booth1 - booth2;
			addend = {1'b0, booth_hi_prod(mcand[26:0], factor2[7:0])};
			accum = accum - (addend & 32'h1000_0000);
			addend = {1'b0, booth_hi_prod(mcand[26:0], factor1[7:0])};
			accum = accum - (addend & 32'h4000_0000);
			sum = accum + (addend & 32'h2000_0000);
			accum = accum - carry;
			addend = {1'b0, booth_hi_prod(mcand[26:0], factor0[7:0])};
			sum = sum + (addend & 32'h4000_0000);
			mul_carry_hi = (sum ^ accum) >> 31;
		end
	endfunction

	always_comb begin
		if (mul_full)
			multiply_carry_final = multiply_long ?
				mul_carry_hi(mul_mcand, mul_mplier_full,
					multiply_seed_hi, multiply_signed) :
				mul_carry_simple(mul_mplier_full);
		else
			// Group 0 was walked at entry and every later group a cycle
			// ahead of the count, so the retire cycle reads the walk from
			// a register - the CSA never touches the forwarding loop.
			multiply_carry_final = mc_carry[31];
	end

	// Group 0 is folded into the entry cycle and each later group lands a
	// cycle ahead of the count, so the retiring value - which the next
	// instruction may need forwarded into its operands - is always a
	// registered one. The 32x8 product and 64-bit add live on quiet
	// register-to-register paths inside MUL_WAIT, never on the loop.
	assign multiply_final_result = mul_acc;

	always_comb begin : decode_forwarding
		logic block_updates_active;

		decode_forward_0_valid = 1'b0;
		decode_forward_0_index = 4'b0;
		decode_forward_0_value = 32'b0;
		decode_forward_1_valid = 1'b0;
		decode_forward_1_index = 4'b0;
		decode_forward_1_value = 32'b0;
		// Registered a cycle ahead. It is a function of block_user, the index
		// and the CPSR mode - all registers - so computing it here would put
		// the mode field at the head of the forwarding cone, which is where
		// the worst path started: cpsr[4] -> this -> the forwarding hit ->
		// the operand mux -> the shifter, 15.09 ns.
		block_updates_active = block_updates_live;

		// No bus qualifiers below, on purpose. Whether the access has
		// completed does not matter to a forward: decode's outputs are only
		// captured by finish_sequential/finish_control, which run on the
		// completing cycle - on every other cycle the forwarded value is
		// computed and discarded. Gating on mem_req/mem_ready/mem_abort
		// put a bus input and a state compare at the head of the
		// forwarding cone (measured: mem_req -> tags -> shifter, worst
		// path of the registered-next-index fit).

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
				begin
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
				if (swp_rd < 15) begin
					decode_forward_0_valid = 1'b1;
					decode_forward_0_index = swp_rd;
					decode_forward_0_value = swp_loaded_value;
				end
			end
			BLOCK_ACCESS: begin
				if (block_last) begin
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
				// Nothing to forward when the retire is staged: MEM_DONE
				// issues the next instruction an edge after `rf` was written.
				if (!MUL_RETIRE_STAGE && mul_retire_fwd) begin
					if (multiply_rd_lo < 15) begin
						decode_forward_0_valid = 1'b1;
						decode_forward_0_index = multiply_rd_lo;
						decode_forward_0_value = multiply_final_result[31:0];
					end
					if (multiply_long && (multiply_rd_hi < 15)) begin
						decode_forward_1_valid = 1'b1;
						decode_forward_1_index = multiply_rd_hi;
						decode_forward_1_value = multiply_final_result[63:32];
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
		// Dead when the retire is staged: `cpsr` itself already holds
		// multiply_post_cpsr by the MEM_DONE edge that reads it.
		else if (!MUL_RETIRE_STAGE && (state == MUL_WAIT) && mul_retire_flags &&
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
			multiply_post_cpsr[31] = multiply_long ?
				multiply_final_result[63] : multiply_final_result[31];
			multiply_post_cpsr[30] = multiply_long ?
				(multiply_final_result == 0) : (multiply_final_result[31:0] == 0);
			multiply_post_cpsr[29] = multiply_carry_final;
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
		logic       ones_terminate;
		// The array stops early once the rest of the multiplier is all sign,
		// so an all-ones remainder only ends it when the operand is being read
		// as signed. An unsigned long multiply has to walk the whole 32 bits:
		// UMULL by 0xffffffff takes four cycles where SMULL by -1 takes one.
		// DDI 0210C's summary says "all zero or one" without qualification and
		// does not mention the long forms' signedness at all; this follows the
		// distinction NanoBoyAdvance fitted to the part.
		ones_terminate = !exec_multiply_long || exec_multiply_signed;
		if ((exec_multiply_operand_b[31:8] == 24'h000000) ||
			(ones_terminate && (exec_multiply_operand_b[31:8] == 24'hffffff)))
			cycles = 3'd1;
		else if ((exec_multiply_operand_b[31:16] == 16'h0000) ||
			(ones_terminate && (exec_multiply_operand_b[31:16] == 16'hffff)))
			cycles = 3'd2;
		else if ((exec_multiply_operand_b[31:24] == 8'h00) ||
			(ones_terminate && (exec_multiply_operand_b[31:24] == 8'hff)))
			cycles = 3'd3;
		else
			cycles = 3'd4;
		dp_multiply_wait = cycles + {1'b0, exec_multiply_extra};
		dp_multiply_groups = cycles;
	end

	// The two signed corrections turn an unsigned 32x32 into a signed one and
	// touch only the top half, so they are added once, up front, rather than
	// sitting on the iteration's path.
	always_comb begin : multiply_start
		logic [31:0] correction;
		logic [63:0] ones_tail;
		logic        early_ones;
		correction = 32'b0;
		if (exec_multiply_long && exec_multiply_signed && exec_multiply_operand_a[31])
			correction = correction - exec_multiply_operand_b;
		if (exec_multiply_long && exec_multiply_signed && exec_multiply_operand_b[31])
			correction = correction - exec_multiply_operand_a;

		// Stopping early on an all-ones remainder is only free inside a Booth
		// array, where the sign is already carried. This walks the multiplier
		// as plain unsigned groups, so the groups it skips still have to be
		// paid for: an all-ones tail above bit 8m means the operand is
		// (low 8m bits) - 2^8m, and the second term is subtracted here rather
		// than iterated over.
		// An all-ones tail above bit 8m means the unsigned operand is
		// (low 8m bits) + 2^32 - 2^8m, so both of the terms the iteration
		// skips are settled here: subtract A<<8m and add A<<32.
		ones_tail = 64'b0;
		early_ones = (dp_multiply_groups < 3'd4) && exec_multiply_operand_b[31];
		if (early_ones) begin
			case (dp_multiply_groups)
				3'd1: ones_tail = {24'b0, exec_multiply_operand_a, 8'b0};
				3'd2: ones_tail = {16'b0, exec_multiply_operand_a, 16'b0};
				default: ones_tail = {8'b0, exec_multiply_operand_a, 24'b0};
			endcase
		end

		multiply_seed = exec_multiply_accumulate ? exec_multiply_accumulator : 64'b0;
		multiply_seed[63:32] = multiply_seed[63:32] + correction;
		if (early_ones)
			multiply_seed[63:32] = multiply_seed[63:32] + exec_multiply_operand_a;
		multiply_seed = multiply_seed - ones_tail;
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

	// Binary index of a one-hot 16-bit value: four OR-trees, one level each.
	function automatic logic [3:0] oh_encode(input logic [15:0] oh);
		oh_encode[0] = |(oh & 16'haaaa);
		oh_encode[1] = |(oh & 16'hcccc);
		oh_encode[2] = |(oh & 16'hf0f0);
		oh_encode[3] = |(oh & 16'hff00);
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
				reg_with_writeback = rf[rf_index(index, cpsr[4:0])];
		end
	endfunction

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
			fetch_pc <= aligned;
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
			write_spsr_for_mode(target_mode, saved_cpsr);
			rf[rf_index(4'd14, target_mode)] <= link_value;
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

	// Hold the bus idle for one cycle, then retire. This is the internal cycle
	// a load spends writing its result back, which the manual counts as the
	// trailing +I of LDR, LDM and SWP.
	task automatic start_internal_completion(
		input logic [31:0] resume_pc,
		input logic        control_flow
	);
		begin
			mem_done_pc <= resume_pc;
			mem_done_control <= control_flow;
			mem_req <= 1'b0;
			mem_seq <= 1'b0;
			mem_fetch <= 1'b0;
			mem_lock <= 1'b0;
			state <= MEM_DONE;
		end
	endtask

	task automatic retain_completed_fetch;
		begin
			if (mem_req && mem_ready && mem_fetch) begin
				ahead_instruction <= mem_rdata;
				ahead_pc <= mem_addr;
				ahead_abort <= mem_abort;
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
			exec_abort <= prefetch_abort;
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

	// `refetch_seq` is the S/N the part announces for the opcode fetch
	// that follows this instruction when a retained word already sits in
	// `ahead`: S after a trailing internal cycle (loads, LDM, SWP and the
	// multiplies - their last row announces S) and after plain data
	// operations; N after a store, whose last data cycle announces N
	// (Tables 6-11 and 6-13).
	task automatic finish_sequential(
		input logic [31:0] resume_pc,
		input logic  [7:0] post_control,
		input logic        refetch_seq
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
			if (halt_req || (!fiq_n && !post_control[6]) ||
				(!irq_n && !post_control[7])) begin
				request_boundary(resume_pc, post_control[7:6]);
			end else begin
				if (prefetch_valid) begin
					capture_prefetch_decode();
				end
				if (ahead_valid) begin
					prefetch_instruction <= ahead_instruction;
					prefetch_ports <= port_indexes(ahead_instruction, ahead_pc[1],
						post_control[5]);
					prefetch_pc <= ahead_pc;
					prefetch_abort <= ahead_abort;
					prefetch_valid <= 1'b1;
					ahead_valid <= 1'b0;
					issue_fetch(fetch_pc_next, post_control[5], post_control[4:0],
						refetch_seq);
				end else if (mem_req && mem_ready && mem_fetch) begin
					prefetch_instruction <= mem_rdata;
					prefetch_ports <= port_indexes(mem_rdata, mem_addr[1],
						post_control[5]);
					prefetch_pc <= mem_addr;
					prefetch_abort <= mem_abort;
					prefetch_valid <= 1'b1;
					issue_fetch(fetch_pc_next, post_control[5], post_control[4:0], 1'b1);
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

	// Which of the sixteen conditions pass under the forwarded flags. Built
	// once, two levels deep, so the ALU's flag result reaches `dec_condition`
	// through one 16:1 select instead of entering the decode priority chain.
	logic [15:0] cond_pass_vec;
	always_comb begin : condition_vector
		logic n, z, c, v;
		{n, z, c, v} = decode_cpsr[31:28];
		cond_pass_vec[0]  = z;
		cond_pass_vec[1]  = !z;
		cond_pass_vec[2]  = c;
		cond_pass_vec[3]  = !c;
		cond_pass_vec[4]  = n;
		cond_pass_vec[5]  = !n;
		cond_pass_vec[6]  = v;
		cond_pass_vec[7]  = !v;
		cond_pass_vec[8]  = c && !z;
		cond_pass_vec[9]  = !c || z;
		cond_pass_vec[10] = (n == v);
		cond_pass_vec[11] = (n != v);
		cond_pass_vec[12] = !z && (n == v);
		cond_pass_vec[13] = z || (n != v);
		cond_pass_vec[14] = 1'b1;
		cond_pass_vec[15] = 1'b0;
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
		logic [31:0] arm_imm_ror;
		logic  [4:0] register_count;
		logic  [3:0] thumb_rd;
		logic [31:0] multiply_a, multiply_b;

		instruction = dec_instruction;
		tinsn = dec_tinsn;
		// The 8-bit immediate rotated by twice bits 11:8, shared by the MSR
		// and data-processing immediate forms rather than instantiated in
		// each arm of the chain.
		arm_imm_ror = ror32({24'b0, instruction[7:0]},
			{instruction[11:8], 1'b0});
		multiply_a = decode_thumb ? reg_thumb_r0 : reg_arm_rm;
		multiply_b = decode_thumb ? reg_thumb_r3 : reg_arm_rs;

		dec_kind = EXEC_UNDEFINED;
		dec_condition = decode_thumb || cond_pass_vec[instruction[31:28]];
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
				// Rd=15 is UNPREDICTABLE (DDI 0100I A4.1.32). The part writes
				// the PSR value into the fetch counter and keeps going without
				// a flush, so fetching continues from value+4 while the two
				// prefetched instructions still execute. A state snapshot
				// cannot hold a stale pipeline, so the observable the oracle
				// records - and the parent's save states can represent - is a
				// resume at value-4 with a refill. The retire itself costs a
				// branch's 2S+N here instead of the part's 1S; recorded as a
				// documented divergence, not an accident.
				if (instruction[15:12] == 4'd15) begin
					dec_kind = EXEC_BRANCH;
					dec_write_reg = 1'b0;
					dec_branch_target = dec_result - 32'd4;
					dec_branch_thumb = 1'b0;
				end
			end
			// MSR register or immediate
			else if (((instruction & 32'h0fb0_fff0) == 32'h0120_f000) ||
			         ((instruction & 32'h0fb0_f000) == 32'h0320_f000)) begin
				dec_kind = EXEC_SIMPLE;
				if (instruction[25]) begin
					psr_source = arm_imm_ror;
				end else begin
					psr_source = reg_arm_rm;
				end
				if (instruction[16]) psr_mask[7:0] = 8'hff;
				if (instruction[17]) psr_mask[15:8] = 8'hff;
				if (instruction[18]) psr_mask[23:16] = 8'hff;
				if (instruction[19]) psr_mask[31:24] = 8'hff;
				// Registered mode: a mode change never continues sequentially.
				if ((cpsr[4:0] == MODE_USER) && !instruction[22])
					psr_mask = psr_mask & 32'hff00_0000;
				// The part forces bit 4 of a value written to the CPSR to
				// one, so a control-byte MSR can never clear it - which is
				// also what keeps the written mode in the 1xxxx space
				// (oracle-verified; NBA's op |= 0x10).
				if (!instruction[22])
					psr_source[4] = 1'b1;
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
				// Rn=15 is UNPREDICTABLE; the part reads the base after the
				// PC has stepped, so the address is PC+12 (oracle-verified).
				if (instruction[19:16] == 4'd15) begin
					dec_mem_preindex = 1'b1;
					dec_op2_sel = SRC_IMM;
					dec_op2_imm = 32'd4;
					dec_dp_opcode = ALU_ADD;
				end
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
					dec_op2_sel = SRC_IMM;
					dec_op2_imm = arm_imm_ror;
					dec_dp_shift_carry = (instruction[11:8] == 4'b0) ?
						decode_cpsr[29] : dec_op2_imm[31];
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
					mode_has_spsr(cpsr[4:0]);
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
						// Thumb MUL multiplies Rd by Rs, and the part's
						// array walks Rd as the multiplier - it decides the
						// early termination and the leftover carry
						// (oracle-verified; the ARM form walks Rs).
						dec_multiply_operand_a = multiply_b;
						dec_multiply_operand_b = multiply_a;
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
					dec_condition = cond_pass_vec[tinsn[11:8]];
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

		// The part waits one internal cycle for a coprocessor to claim the
		// instruction before trapping, so DDI 0210C Table 6-21 gives Undefined
		// 2S+N+I where SWI's Table 6-15 gives 2S+N. Nothing here answers, but
		// the cycle is still spent.
		if (dec_kind == EXEC_UNDEFINED)
			dec_internal_cycle = 1'b1;
	end

	integer reset_index;
	always_ff @(posedge clk) begin : execute
		// Two write ports funnel every register-file write. Without them,
		// each of a dozen write sites grows its own selector tree into all
		// 960 file bits; with them, the file sees two decoded enables and
		// two data buses. Blocking-assigned, read only at the bottom of
		// this block. Port B is applied second, so it wins a same-address
		// conflict - the order the scattered writes had (a load over its
		// base writeback, the high word over the low).
		logic        rf_we_a, rf_we_b;
		logic  [4:0] rf_wa_a, rf_wa_b;
		logic [31:0] rf_wd_a, rf_wd_b;
		rf_we_a = 1'b0;
		rf_we_b = 1'b0;
		// The write address and data are don't-cares while the enable is low,
		// so pick both from the state here and let the enable alone carry
		// whether the access finished. Setting them inside those branches
		// instead put the bus's ready in front of the register file's address
		// decode and its data mux, when ready only ever needed the flop's
		// enable. Each state below has exactly one writer per port; RUN has two
		// and they split on the instruction kind, which is already decoded.
		case (state)
			RUN: begin
				rf_wa_a = (exec_kind == EXEC_BRANCH) ?
				          rf_index(4'd14, cpsr[4:0]) :
				          rf_index(exec_rd, cpsr[4:0]);
				rf_wd_a = (exec_kind == EXEC_BRANCH) ?
				          exec_link_value : exec_effective_result;
			end
			MEM_ACCESS: begin
				rf_wa_a = rf_index(data_rn, cpsr[4:0]);
				rf_wd_a = data_writeback_value;
			end
			SWP_WRITE: begin
				rf_wa_a = rf_index(swp_rd, cpsr[4:0]);
				rf_wd_a = swp_loaded_value;
			end
			BLOCK_ACCESS: begin
				rf_wa_a = rf_index(block_index[3:0],
				          block_user ? MODE_USER : cpsr[4:0]);
				rf_wd_a = mem_rdata;
			end
			MUL_WAIT: begin
				rf_wa_a = rf_index(multiply_rd_lo, cpsr[4:0]);
				rf_wd_a = multiply_final_result[31:0];
			end
			default: begin                            // HALTED
				rf_wa_a = state_rf_index;
				rf_wd_a = state_wdata;
			end
		endcase
		case (state)
			MEM_ACCESS: begin
				rf_wa_b = rf_index(data_rd, cpsr[4:0]);
				rf_wd_b = data_loaded_value;
			end
			BLOCK_ACCESS: begin
				rf_wa_b = rf_index(block_rn,
				          block_user ? MODE_USER : cpsr[4:0]);
				rf_wd_b = block_writeback_value;
			end
			MUL_WAIT: begin
				rf_wa_b = rf_index(multiply_rd_hi, cpsr[4:0]);
				rf_wd_b = multiply_final_result[63:32];
			end
			default: begin
				rf_wa_b = 5'd0;
				rf_wd_b = 32'b0;
			end
		endcase
		retire <= 1'b0;
		state_ready <= 1'b0;

		if (reset) begin
			for (reset_index = 0; reset_index < 30; reset_index = reset_index + 1)
				rf[reset_index] <= 32'b0;
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
			prefetch_ports <= 16'b0;
			prefetch_pc <= 32'b0;
			ahead_valid <= 1'b0;
			ahead_instruction <= 32'b0;
			ahead_pc <= 32'b0;
			ahead_abort <= 1'b0;
			prefetch_abort <= 1'b0;
			exec_abort <= 1'b0;
			// synthesis translate_off
			trace_retire_pc <= 32'b0;
			trace_retire_instruction <= 32'b0;
			trace_retire_thumb <= 1'b0;
			trace_retire_exception <= 1'b0;
			trace_retire_exception_return <= 1'b0;
			// synthesis translate_on
			// One step behind the reset vector, so the first sequential
			// issue of `fetch_pc_next` fetches the vector itself.
			fetch_pc <= VECTOR_RESET - 32'd4;
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
			block_rest <= 16'b0;
			block_next_index <= 5'd16;
			block_address <= 32'b0;
			block_writeback_value <= 32'b0;
			block_resume_pc <= 32'b0;
			block_rn <= 4'b0;
			block_index <= 5'd16;
			block_load <= 1'b0;
			block_writeback <= 1'b0;
			block_user <= 1'b0;
			block_updates_live <= 1'b1;
			block_restore_cpsr <= 1'b0;
			block_pc_value <= 32'b0;
			multiply_result <= 64'b0;
			multiply_rd_lo <= 4'b0;
			multiply_rd_hi <= 4'b0;
			multiply_wait <= 3'b0;
			mul_retire_seq <= 1'b0;
			mul_retire_fwd <= 1'b0;
			mul_retire_flags <= 1'b0;
			mul_acc <= 64'b0;
			mul_mplier <= 32'b0;
			mul_mcand <= 32'b0;
			mul_group <= 2'b0;
			mul_groups_left <= 3'b0;
			mul_mplier_full <= 32'b0;
			mul_full <= 1'b0;
			multiply_signed <= 1'b0;
			multiply_seed_hi <= 32'b0;
			mc_accum <= 32'b0;
			mc_carry <= 32'b0;
			multiply_long <= 1'b0;
			multiply_set_flags <= 1'b0;
			multiply_resume_pc <= 32'b0;
			internal_cycle_pending <= 1'b0;
			mem_done_pc <= 32'b0;
			mem_done_control <= 1'b0;
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
								default: boundary_event <= BOUNDARY_NONE;
							endcase
						end
					end else if (decode_valid && prefetch_valid) begin
						if (exec_abort) begin
							// The flagged word reached execute: prefetch
							// abort, taken regardless of its condition
							// field, R14_abt = its address + 4.
							if (!mem_req || mem_ready) begin
								retire <= 1'b1;
								// synthesis translate_off
								trace_retire_pc <= decode_pc;
								trace_retire_instruction <= decode_instruction;
								trace_retire_thumb <= thumb;
								trace_retire_exception <= 1'b1;
								trace_retire_exception_return <= 1'b0;
								// synthesis translate_on
								internal_cycle_pending <= 1'b0;
								enter_exception(MODE_ABT, VECTOR_PABT,
									decode_pc + 32'd4, cpsr, 1'b0);
							end
						end else if (exec_condition && exec_internal_cycle &&
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
							finish_sequential(decode_pc + (thumb ? 32'd2 : 32'd4), cpsr[7:0], 1'b1);
						end else begin
							internal_cycle_pending <= 1'b0;
							case (exec_kind)
								EXEC_SIMPLE: begin
									if (!exec_simple_control || !mem_req || mem_ready) begin
										if (exec_write_reg && (exec_rd < 15)) begin
											rf_we_a = 1'b1;
										end
										if (exec_write_psr && exec_write_spsr && mode_has_spsr(cpsr[4:0]))
											write_spsr_for_mode(cpsr[4:0],
												(current_spsr & ~exec_psr_mask) |
												(exec_psr_value & exec_psr_mask));
										if (exec_restore_spsr) begin
											cpsr <= current_spsr;
										end else if (exec_write_flags ||
											(exec_write_psr && !exec_write_spsr)) begin
											if (mode_valid(exec_effective_post_cpsr[4:0]))
												cpsr <= exec_effective_post_cpsr;
										end
										if (exec_restore_spsr) begin
											finish_control(decode_pc + (thumb ? 32'd2 : 32'd4),
												current_spsr[5], current_spsr[7:6],
												current_spsr[4:0]);
										end else if (exec_write_psr && !exec_write_spsr &&
											|(exec_psr_mask[7:0])) begin
											// An MSR that sets T keeps going in
											// Thumb without a flush on the part;
											// the snapshot observable is a
											// resume at +8, not +4
											// (oracle-verified).
											finish_control(decode_pc +
												(exec_effective_post_cpsr[5] ?
													32'd8 : 32'd4),
												exec_effective_post_cpsr[5],
												exec_effective_post_cpsr[7:6],
												exec_effective_post_cpsr[4:0]);
										end else begin
											finish_sequential(decode_pc + (thumb ? 32'd2 : 32'd4),
												exec_effective_post_cpsr[7:0], 1'b1);
										end
									end
								end
								EXEC_BRANCH: begin
									if (!mem_req || mem_ready) begin
										if (exec_link) begin
											rf_we_a = 1'b1;
										end
										if (exec_restore_spsr) begin
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
										begin : block_entry_next
											logic [15:0] rest0, nxt0;
											rest0 = exec_block_list &
												~(16'h0001 <<
												exec_block_first_index[3:0]);
											nxt0 = rest0 & (~rest0 + 16'd1);
											block_next_index <= (rest0 == 16'b0) ?
												5'd16 : {1'b0, oh_encode(nxt0)};
											block_rest <= rest0 & ~nxt0;
										end
										block_address <= dp_block_address;
										block_writeback_value <= dp_block_updated_base;
										block_resume_pc <= decode_pc + (thumb ? 32'd2 : 32'd4);
										block_rn <= exec_block_rn;
										block_index <= exec_block_first_index;
										block_updates_live <= !exec_block_user ||
											user_reg_is_live(exec_block_first_index[3:0]);
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
										mul_acc <= multiply_seed +
											{24'b0, mul_prod};
										mul_mplier <= {8'b0,
											exec_multiply_operand_b[31:8]};
										mul_mcand <= exec_multiply_operand_a;
										mul_group <= 2'd1;
										mul_groups_left <= dp_multiply_groups - 3'd1;
										mul_mplier_full <= exec_multiply_operand_b;
										mul_full <= (dp_multiply_groups == 3'd4);
										multiply_signed <= exec_multiply_signed;
										// The closed-form carry wants the raw
										// accumulator high word - the signed
										// corrections folded into the seed are
										// the iteration's business, not the
										// Booth array's.
										multiply_seed_hi <= exec_multiply_accumulate ?
											exec_multiply_accumulator[63:32] : 32'b0;
										// Group 0 of the carry walk from the
										// shared instance (entry-side inputs).
										{mc_accum, mc_carry} <=
											{mc_accum_next, mc_carry_next};
										multiply_rd_lo <= exec_multiply_rd_lo;
										multiply_rd_hi <= exec_multiply_rd_hi;
										multiply_wait <= dp_multiply_wait;
										mul_retire_seq <= (dp_multiply_wait <= 3'd1);
										mul_retire_fwd <= (dp_multiply_wait <= 3'd1);
										mul_retire_flags <= (dp_multiply_wait <= 3'd1);
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
							request_boundary(fetch_pc_next, cpsr[7:6]);
						else if (!fiq_n && !cpsr[6])
							request_boundary(fetch_pc_next, cpsr[7:6]);
						else if (!irq_n && !cpsr[7])
							request_boundary(fetch_pc_next, cpsr[7:6]);
						else
							issue_fetch(fetch_pc_next, thumb, cpsr[4:0], 1'b0);
					end else if (mem_ready) begin
						if (halt_req || (!fiq_n && !cpsr[6]) || (!irq_n && !cpsr[7])) begin
							if (decode_valid)
								request_boundary(decode_pc, cpsr[7:6]);
							else if (prefetch_valid)
								request_boundary(prefetch_pc, cpsr[7:6]);
							else
								request_boundary(mem_addr, cpsr[7:6]);
						end else if (decode_valid || !prefetch_valid) begin
							prefetch_instruction <= mem_rdata;
							prefetch_ports <= port_indexes(mem_rdata, mem_addr[1], thumb);
							prefetch_pc <= mem_addr;
							prefetch_abort <= mem_abort;
							prefetch_valid <= 1'b1;
							issue_fetch(fetch_pc_next, thumb, cpsr[4:0], 1'b1);
						end else begin
							capture_prefetch_decode();
							prefetch_instruction <= mem_rdata;
							prefetch_ports <= port_indexes(mem_rdata, mem_addr[1], thumb);
							prefetch_pc <= mem_addr;
							prefetch_abort <= mem_abort;
							prefetch_valid <= 1'b1;
							issue_fetch(fetch_pc_next, thumb, cpsr[4:0], 1'b1);
						end
					end
				end

				MEM_ACCESS: begin
					if (mem_req && mem_ready) begin
						if (mem_abort) begin
							enter_exception(MODE_ABT, VECTOR_DABT,
								data_resume_pc + (cpsr[5] ? 32'd6 : 32'd4), cpsr, 1'b0);
						end else begin
							if (data_writeback && (data_rn < 15)) begin
								rf_we_a = 1'b1;
							end
							if (data_load && !data_load_pc) begin
								rf_we_b = 1'b1;
							end
							// A load spends one internal cycle after its data
							// transfer - DDI 0210C Table 6-23 gives LDR as
							// S+N+I. A store has no such cycle, 2N, so it
							// resumes here.
							//
							// Base writeback to R15 is UNPREDICTABLE; the
							// oracle refills at the updated base, whose base
							// read happened at PC+12 where our ALU saw PC+8 -
							// hence the +4. A loaded PC wins over it.
							if (data_load)
								start_internal_completion(
									data_load_pc ? data_loaded_value :
									(data_writeback && (data_rn == 4'd15)) ?
										(data_writeback_value + 32'd4) :
										data_resume_pc,
									data_load_pc ||
									(data_writeback && (data_rn == 4'd15)));
							else if (data_writeback && (data_rn == 4'd15)) begin
								finish_control(data_writeback_value + 32'd4,
									cpsr[5], cpsr[7:6], cpsr[4:0]);
								state <= RUN;
							end else begin
								finish_sequential(data_resume_pc, cpsr[7:0], 1'b0);
								state <= RUN;
							end
						end
					end
				end

				MEM_DONE: begin
					if (mem_done_control)
						finish_control(mem_done_pc, cpsr[5], cpsr[7:6], cpsr[4:0]);
					else
						finish_sequential(mem_done_pc, cpsr[7:0], 1'b1);
					state <= RUN;
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
						end else if (swp_rd < 15) begin
							rf_we_a = 1'b1;
							// Table 6-23 gives SWP as S+2N+I.
							start_internal_completion(swp_resume_pc, 1'b0);
						end else begin
							// Rd=15 is UNPREDICTABLE; the loaded value goes
							// into the fetch counter (oracle-verified).
							start_internal_completion(swp_loaded_value, 1'b1);
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
								else begin
									rf_we_a = 1'b1;
								end
							end
							if (!block_last) begin : block_advance_next
								logic [15:0] nxt2;
								nxt2 = block_rest & (~block_rest + 16'd1);
								block_next_index <= (block_rest == 16'b0) ?
									5'd16 : {1'b0, oh_encode(nxt2)};
								block_rest <= block_rest & ~nxt2;
								block_index <= block_next_index;
								block_updates_live <= !block_user ||
									user_reg_is_live(block_next_index[3:0]);
								block_address <= block_address + 32'd4;
								mem_addr <= block_address + 32'd4;
								mem_write <= !block_load;
								if (block_next_index == 15)
									// R15 later in the list of an STM whose
									// base is R15 with writeback stores the
									// already-updated base, not PC+12: the
									// writeback landed in the fetch counter
									// before this position transferred
									// (oracle-verified).
									mem_wdata <= (block_writeback &&
										(block_rn == 4'd15)) ?
										block_writeback_value : block_pc_value;
								else if (block_user)
									mem_wdata <= (block_writeback &&
										(block_rn == block_next_index[3:0])) ?
										block_writeback_value :
										read_user_reg(block_next_index[3:0]);
								else
									mem_wdata <= reg_with_writeback(block_next_index[3:0]);
								mem_wstrb <= block_load ? 4'b0 : 4'b1111;
								mem_seq <= 1'b1;
							end else begin
								// An S-bit transfer's writeback goes to the
								// user bank - the part switches to user mode
								// for the transfer, so the base it updates is
								// the user one (oracle-verified).
								if (block_writeback && (block_rn < 15)) begin
									rf_we_b = 1'b1;
								end
								if (block_load && (block_index == 15)) begin
									if (block_restore_cpsr) begin
										cpsr <= current_spsr;
										start_internal_completion(mem_rdata, 1'b1);
									end else begin
										start_internal_completion(mem_rdata, 1'b1);
									end
								end else if (block_load) begin
									// Table 6-23 gives LDM as nS+N+I. STM is
									// (n-1)S+2N and has no internal cycle.
									start_internal_completion(
										(block_writeback && (block_rn == 4'd15)) ?
											block_writeback_value :
											block_resume_pc,
										block_writeback && (block_rn == 4'd15));
								end else if (block_writeback &&
									(block_rn == 4'd15)) begin
									// The block base was read at PC+8, same
									// as ours: the target is the writeback
									// value itself (oracle-verified).
									finish_control(block_writeback_value,
										cpsr[5], cpsr[7:6], cpsr[4:0]);
									state <= RUN;
								end else begin
									finish_sequential(block_resume_pc, cpsr[7:0], 1'b0);
									state <= RUN;
								end
							end
						end
					end
				end

				MUL_WAIT: begin
					if (mul_groups_left != 3'd0) begin
						mul_acc <= mul_acc_next;
						mul_mplier <= {8'b0, mul_mplier[31:8]};
						mul_group <= mul_group + 2'd1;
						mul_groups_left <= mul_groups_left - 3'd1;
						mc_accum <= mc_accum_next;
						mc_carry <= mc_carry_next;
					end
					if (!mul_retire_seq) begin
						multiply_wait <= multiply_wait - 1'b1;
						mul_retire_seq <= (multiply_wait <= 3'd2);
						mul_retire_fwd <= (multiply_wait <= 3'd2);
						mul_retire_flags <= (multiply_wait <= 3'd2);
					end else begin
						mul_retire_seq <= 1'b0;
						mul_retire_fwd <= 1'b0;
						mul_retire_flags <= 1'b0;
						if (multiply_rd_lo < 15) begin
							rf_we_a = 1'b1;
						end
						if (multiply_long && (multiply_rd_hi < 15)) begin
							rf_we_b = 1'b1;
						end
						if (multiply_set_flags)
							cpsr <= multiply_post_cpsr;
						// A destination of R15 is UNPREDICTABLE; the result
						// lands in the fetch counter, the high word winning
						// when both halves name it (oracle-verified).
						if (MUL_RETIRE_STAGE) begin
							// The deliberate extra cycle. The multiply
							// finishes here, but the next instruction does not
							// start until MEM_DONE - one edge later than the
							// real part would have started it.
							//
							// MEM_DONE reads the same values from registers:
							// `multiply_post_cpsr` differs from `cpsr` only in
							// bits 31:29, and `cpsr` has taken it by then, so
							// its cpsr[7:0]/cpsr[7:6]/cpsr[4:0]/cpsr[5]
							// arguments are the ones passed below. The bus
							// clear inside is a no-op - MUL entry already
							// retained the fetch and dropped mem_req.
							start_internal_completion(
								(multiply_long && (multiply_rd_hi == 4'd15)) ?
									multiply_final_result[63:32] :
								(multiply_rd_lo == 4'd15) ?
									multiply_final_result[31:0] :
									multiply_resume_pc,
								(multiply_rd_lo == 4'd15) ||
								(multiply_long && (multiply_rd_hi == 4'd15)));
						end else if ((multiply_rd_lo == 4'd15) ||
							(multiply_long && (multiply_rd_hi == 4'd15))) begin
							finish_control(
								(multiply_long && (multiply_rd_hi == 4'd15)) ?
									multiply_final_result[63:32] :
									multiply_final_result[31:0],
								cpsr[5], multiply_post_cpsr[7:6],
								multiply_post_cpsr[4:0]);
							state <= RUN;
						end else begin
							finish_sequential(multiply_resume_pc,
								multiply_post_cpsr[7:0], 1'b1);
							state <= RUN;
						end
					end
				end

				default: begin // HALTED
					mem_req <= 1'b0;
					mem_seq <= 1'b0;
					mem_fetch <= 1'b0;
					mem_lock <= 1'b0;
					if (halt_capture_pending) begin
						// The flat file already holds every banked register
						// in its state-map slot; only PC and CPSR need
						// capturing.
						state_pc <= halt_pc;
						state_cpsr <= cpsr;
						state_image_valid <= 1'b1;
						state_dirty <= 1'b0;
						halt_capture_pending <= 1'b0;
					end else begin
						if (state_req && state_image_valid && (state_index <= STATE_UND_SPSR)) begin
							state_ready <= 1'b1;
							if (!state_write) begin
								if (state_index == STATE_PC)
									state_rdata <= state_pc;
								else if (state_index == STATE_CPSR)
									state_rdata <= state_cpsr;
								else begin
									case (state_index)
										STATE_FIQ_SPSR: state_rdata <= spsr_fiq;
										STATE_IRQ_SPSR: state_rdata <= spsr_irq;
										STATE_SVC_SPSR: state_rdata <= spsr_svc;
										STATE_ABT_SPSR: state_rdata <= spsr_abt;
										STATE_UND_SPSR: state_rdata <= spsr_und;
										default:        state_rdata <=
											rf[state_rf_index];
									endcase
								end
							end else begin
								state_dirty <= 1'b1;
								if (state_index == STATE_PC)
									state_pc <= state_wdata;
								else if (state_index == STATE_CPSR)
									state_cpsr <= state_wdata;
								else begin
									case (state_index)
										STATE_FIQ_SPSR: spsr_fiq <= state_wdata;
										STATE_IRQ_SPSR: spsr_irq <= state_wdata;
										STATE_SVC_SPSR: spsr_svc <= state_wdata;
										STATE_ABT_SPSR: spsr_abt <= state_wdata;
										STATE_UND_SPSR: spsr_und <= state_wdata;
										default: begin
											rf_we_a = 1'b1;
										end
									endcase
								end
							end
						end
						if (state_commit && state_image_valid && mode_valid(state_cpsr[4:0]) &&
							((state_cpsr[5] && !state_pc[0]) || (!state_cpsr[5] && (state_pc[1:0] == 0)))) begin
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

		if (rf_we_a)
			rf[rf_wa_a] <= rf_wd_a;
		if (rf_we_b)
			rf[rf_wa_b] <= rf_wd_b;
	end
	`undef ARM7_READ_DECODE_REG
endmodule
