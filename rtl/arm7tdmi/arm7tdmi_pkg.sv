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

	// One rotator serves all four shift types. LSL n is a rotate by 32-n
	// with the low n bits cleared, LSR n a rotate by n with the top n bits
	// cleared, ASR n the same rotate with the top n bits filled from the
	// sign, and ROR n the rotate itself - so the four parallel 32-bit
	// shifters this used to be collapse into one ror32 and a per-bit
	// select. Each result bit is a function of the rotated bit, the sign,
	// the type and two thermometer tests of the amount, which is two LUT
	// levels after the rotator instead of a fifth shifter-mux level, and a
	// third of the area. Amounts of 32 and more only arrive from register
	// shifts; their values and carries are the special cases DDI 0100I
	// tabulates.
	function automatic arm_shift_t shift_register(
		input logic [31:0] value,
		input logic [1:0] shift_type,
		input logic [7:0] amount,
		input logic carry_in
	);
		arm_shift_t result;
		logic [4:0] amt5, rot, carry_index;
		logic [31:0] rotated;
		logic ge32, eq32, is_left, sign, low_live, high_live;
		begin
			amt5 = amount[4:0];
			ge32 = |amount[7:5];
			eq32 = (amount == 8'd32);
			is_left = (shift_type == 2'b00);
			sign = value[31];
			rot = is_left ? (5'd0 - amt5) : amt5;
			rotated = ror32(value, rot);

			if (amount == 0) begin
				result.value = value;
				result.carry = carry_in;
			end else begin
				carry_index = is_left ? (5'd0 - amt5) : (amt5 - 5'd1);
				case (shift_type)
					2'b00: result.carry = eq32 ? value[0] :
						(ge32 ? 1'b0 : value[carry_index]);
					2'b01: result.carry = eq32 ? value[31] :
						(ge32 ? 1'b0 : value[carry_index]);
					2'b10: result.carry = ge32 ? value[31] : value[carry_index];
					default: result.carry = (amt5 == 0) ? value[31] :
						value[carry_index];
				endcase
				for (int i = 0; i < 32; i++) begin
					// Live below 32-n (right shifts keep the low bits) and
					// at or above n (a left shift keeps the high bits).
					low_live = ({1'b0, 5'(i)} + {1'b0, amt5}) < 6'd32;
					high_live = 5'(i) >= amt5;
					case (shift_type)
						2'b00: result.value[i] =
							(!ge32 && high_live) ? rotated[i] : 1'b0;
						2'b01: result.value[i] =
							(!ge32 && low_live) ? rotated[i] : 1'b0;
						2'b10: result.value[i] =
							(ge32 || !low_live) ? sign : rotated[i];
						default: result.value[i] = rotated[i];
					endcase
				end
			end
			shift_register = result;
		end
	endfunction

endpackage
