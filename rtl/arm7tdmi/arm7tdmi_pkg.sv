// Copyright (c) 2026 Jamie Blanks
// SPDX-License-Identifier: MIT

package arm7tdmi_pkg;

	typedef enum logic [4:0] {
		MODE_USER = 5'h10,
		MODE_FIQ  = 5'h11,
		MODE_IRQ  = 5'h12,
		MODE_SVC  = 5'h13,
		MODE_ABT  = 5'h17,
		MODE_UND  = 5'h1b,
		MODE_SYS  = 5'h1f
	} arm_mode_e;

	typedef enum logic [1:0] {
		MEM_BYTE = 2'b00,
		MEM_HALF = 2'b01,
		MEM_WORD = 2'b10
	} arm_mem_size_e;

	localparam logic [31:0] CPSR_N = 32'h8000_0000;
	localparam logic [31:0] CPSR_Z = 32'h4000_0000;
	localparam logic [31:0] CPSR_C = 32'h2000_0000;
	localparam logic [31:0] CPSR_V = 32'h1000_0000;
	localparam logic [31:0] CPSR_I = 32'h0000_0080;
	localparam logic [31:0] CPSR_F = 32'h0000_0040;
	localparam logic [31:0] CPSR_T = 32'h0000_0020;

	localparam logic [31:0] VECTOR_RESET = 32'h0000_0000;
	localparam logic [31:0] VECTOR_UND   = 32'h0000_0004;
	localparam logic [31:0] VECTOR_SWI   = 32'h0000_0008;
	localparam logic [31:0] VECTOR_PABT  = 32'h0000_000c;
	localparam logic [31:0] VECTOR_DABT  = 32'h0000_0010;
	localparam logic [31:0] VECTOR_IRQ   = 32'h0000_0018;
	localparam logic [31:0] VECTOR_FIQ   = 32'h0000_001c;

	// Halted-state port map. Values 37-63 are reserved and rejected.
	localparam logic [5:0] STATE_R0          = 6'd0;
	localparam logic [5:0] STATE_R7          = STATE_R0 + 6'd7;
	localparam logic [5:0] STATE_USR_R8      = 6'd8;
	localparam logic [5:0] STATE_USR_R14     = 6'd14;
	localparam logic [5:0] STATE_PC          = 6'd15;
	localparam logic [5:0] STATE_CPSR        = 6'd16;
	localparam logic [5:0] STATE_FIQ_R8      = 6'd17;
	localparam logic [5:0] STATE_FIQ_R14     = 6'd23;
	localparam logic [5:0] STATE_FIQ_SPSR    = 6'd24;
	localparam logic [5:0] STATE_IRQ_R13     = 6'd25;
	localparam logic [5:0] STATE_IRQ_R14     = 6'd26;
	localparam logic [5:0] STATE_IRQ_SPSR    = 6'd27;
	localparam logic [5:0] STATE_SVC_R13     = 6'd28;
	localparam logic [5:0] STATE_SVC_R14     = 6'd29;
	localparam logic [5:0] STATE_SVC_SPSR    = 6'd30;
	localparam logic [5:0] STATE_ABT_R13     = 6'd31;
	localparam logic [5:0] STATE_ABT_R14     = 6'd32;
	localparam logic [5:0] STATE_ABT_SPSR    = 6'd33;
	localparam logic [5:0] STATE_UND_R13     = 6'd34;
	localparam logic [5:0] STATE_UND_R14     = 6'd35;
	localparam logic [5:0] STATE_UND_SPSR    = 6'd36;

	typedef struct packed {
		logic [31:0] value;
		logic        carry;
	} arm_shift_t;

	function automatic logic mode_valid(input logic [4:0] mode);
		case (mode)
			MODE_USER, MODE_FIQ, MODE_IRQ, MODE_SVC,
			MODE_ABT, MODE_UND, MODE_SYS: mode_valid = 1'b1;
			default: mode_valid = 1'b0;
		endcase
	endfunction

	function automatic logic mode_has_spsr(input logic [4:0] mode);
		case (mode)
			MODE_FIQ, MODE_IRQ, MODE_SVC, MODE_ABT, MODE_UND:
				mode_has_spsr = 1'b1;
			default: mode_has_spsr = 1'b0;
		endcase
	endfunction

	function automatic logic condition_pass(
		input logic [3:0] condition,
		input logic [3:0] flags
	);
		logic n, z, c, v;
		begin
			n = flags[3];
			z = flags[2];
			c = flags[1];
			v = flags[0];
			case (condition)
				4'h0: condition_pass = z;
				4'h1: condition_pass = !z;
				4'h2: condition_pass = c;
				4'h3: condition_pass = !c;
				4'h4: condition_pass = n;
				4'h5: condition_pass = !n;
				4'h6: condition_pass = v;
				4'h7: condition_pass = !v;
				4'h8: condition_pass = c && !z;
				4'h9: condition_pass = !c || z;
				4'ha: condition_pass = (n == v);
				4'hb: condition_pass = (n != v);
				4'hc: condition_pass = !z && (n == v);
				4'hd: condition_pass = z || (n != v);
				4'he: condition_pass = 1'b1;
				default: condition_pass = 1'b0;
			endcase
		end
	endfunction

	function automatic logic [31:0] ror32(
		input logic [31:0] value,
		input logic [4:0] amount
	);
		if (amount == 0)
			ror32 = value;
		else
			ror32 = (value >> amount) | (value << (6'd32 - amount));
	endfunction

	function automatic arm_shift_t shift_immediate(
		input logic [31:0] value,
		input logic [1:0] shift_type,
		input logic [4:0] amount,
		input logic carry_in
	);
		arm_shift_t result;
		logic [4:0] carry_index;
		begin
			result.value = value;
			result.carry = carry_in;
			carry_index = 5'b0 - amount;
			case (shift_type)
				2'b00: begin // LSL
					if (amount != 0) begin
						result.value = value << amount;
						result.carry = value[carry_index];
					end
				end
				2'b01: begin // LSR, zero encodes 32
					if (amount == 0) begin
						result.value = 32'b0;
						result.carry = value[31];
					end else begin
						result.value = value >> amount;
						result.carry = value[amount - 1'b1];
					end
				end
				2'b10: begin // ASR, zero encodes 32
					if (amount == 0) begin
						result.value = {32{value[31]}};
						result.carry = value[31];
					end else begin
						result.value = $unsigned($signed(value) >>> amount);
						result.carry = value[amount - 1'b1];
					end
				end
				default: begin // ROR, zero encodes RRX
					if (amount == 0) begin
						result.value = {carry_in, value[31:1]};
						result.carry = value[0];
					end else begin
						result.value = ror32(value, amount);
						result.carry = value[amount - 1'b1];
					end
				end
			endcase
			shift_immediate = result;
		end
	endfunction

	function automatic arm_shift_t shift_register(
		input logic [31:0] value,
		input logic [1:0] shift_type,
		input logic [7:0] amount,
		input logic carry_in
	);
		arm_shift_t result;
		logic [4:0] rotate;
		logic [4:0] carry_index;
		begin
			result.value = value;
			result.carry = carry_in;
			rotate = amount[4:0];
			carry_index = 5'b0 - amount[4:0];
			if (amount != 0) begin
				case (shift_type)
					2'b00: begin
						if (amount < 32) begin
							result.value = value << amount;
							result.carry = value[carry_index];
						end else begin
							result.value = 32'b0;
							result.carry = (amount == 32) ? value[0] : 1'b0;
						end
					end
					2'b01: begin
						if (amount < 32) begin
							result.value = value >> amount;
							result.carry = value[amount[4:0] - 1'b1];
						end else begin
							result.value = 32'b0;
							result.carry = (amount == 32) ? value[31] : 1'b0;
						end
					end
					2'b10: begin
						if (amount < 32) begin
							result.value = $unsigned($signed(value) >>> amount);
							result.carry = value[amount[4:0] - 1'b1];
						end else begin
							result.value = {32{value[31]}};
							result.carry = value[31];
						end
					end
					default: begin
						result.value = ror32(value, rotate);
						result.carry = (rotate == 0) ? value[31] : value[rotate - 1'b1];
					end
				endcase
			end
			shift_register = result;
		end
	endfunction

endpackage
