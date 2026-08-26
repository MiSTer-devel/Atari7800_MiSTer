// PS/2 scan codes to the Atari keyboard matrix that POKEY scans.
// Replaces the VHDL ps2_to_atari800 and its ps2_keyboard receiver. Only the
// direct path is kept: on MiSTer the HPS decodes PS/2 for us and hands over
// ps2_key, so the raw PS/2 clock and data receiver had no user.
//
// INPUT packs one key event from ps2_key:
//   [7:0] scan code   [12] extended prefix   [16] pressed (0 = released)
//
// POKEY drives KEYBOARD_SCAN and reads KEYBOARD_RESPONSE, which is active low:
//   bit 0  the scanned key is down
//   bit 1  the modifier for the current scan group is down

module ps2_to_atari800
(
	input  wire        CLK,
	input  wire        RESET_N,
	input  wire [31:0] INPUT,

	input  wire  [5:0] KEYBOARD_SCAN,
	output reg   [1:0] KEYBOARD_RESPONSE
);
	reg  [511:0] ps2_keys_reg;
	reg  [511:0] ps2_keys_next;

	wire   [7:0] key_value    = INPUT[7:0];
	wire         key_extended = INPUT[12];
	wire         key_up       = ~INPUT[16];

	reg   [63:0] atari_keyboard;
	reg          shift_pressed;
	reg          control_pressed;
	reg          break_pressed;

	// The direct path reports an event every clock, so the selected bit simply
	// tracks the current press state rather than pulsing once.
	always @(*) begin
		ps2_keys_next = ps2_keys_reg;
		ps2_keys_next[{key_extended, key_value}] = ~key_up;
	end

	always @(posedge CLK or negedge RESET_N) begin
		if (!RESET_N)
			ps2_keys_reg <= 512'd0;
		else
			ps2_keys_reg <= ps2_keys_next;
	end

	always @(*) begin
		atari_keyboard  = 64'd0;
		shift_pressed   = 1'b0;
		control_pressed = 1'b0;
		break_pressed   = 1'b0;

		atari_keyboard[63] = ps2_keys_reg[9'h01c];
		atari_keyboard[21] = ps2_keys_reg[9'h032];
		atari_keyboard[18] = ps2_keys_reg[9'h021];
		atari_keyboard[58] = ps2_keys_reg[9'h023];
		atari_keyboard[42] = ps2_keys_reg[9'h024];
		atari_keyboard[56] = ps2_keys_reg[9'h02b];
		atari_keyboard[61] = ps2_keys_reg[9'h034];
		atari_keyboard[57] = ps2_keys_reg[9'h033];
		atari_keyboard[13] = ps2_keys_reg[9'h043];
		atari_keyboard[1]  = ps2_keys_reg[9'h03b];
		atari_keyboard[5]  = ps2_keys_reg[9'h042];
		atari_keyboard[0]  = ps2_keys_reg[9'h04b];
		atari_keyboard[37] = ps2_keys_reg[9'h03a];
		atari_keyboard[35] = ps2_keys_reg[9'h031];
		atari_keyboard[8]  = ps2_keys_reg[9'h044];
		atari_keyboard[10] = ps2_keys_reg[9'h04d];
		atari_keyboard[47] = ps2_keys_reg[9'h015];
		atari_keyboard[40] = ps2_keys_reg[9'h02d];
		atari_keyboard[62] = ps2_keys_reg[9'h01b];
		atari_keyboard[45] = ps2_keys_reg[9'h02c];
		atari_keyboard[11] = ps2_keys_reg[9'h03c];
		atari_keyboard[16] = ps2_keys_reg[9'h02a];
		atari_keyboard[46] = ps2_keys_reg[9'h01d];
		atari_keyboard[22] = ps2_keys_reg[9'h022];
		atari_keyboard[43] = ps2_keys_reg[9'h035];
		atari_keyboard[23] = ps2_keys_reg[9'h01a];
		atari_keyboard[50] = ps2_keys_reg[9'h045];
		atari_keyboard[31] = ps2_keys_reg[9'h016];
		atari_keyboard[30] = ps2_keys_reg[9'h01e];
		atari_keyboard[26] = ps2_keys_reg[9'h026];
		atari_keyboard[24] = ps2_keys_reg[9'h025];
		atari_keyboard[29] = ps2_keys_reg[9'h02e];
		atari_keyboard[27] = ps2_keys_reg[9'h036];
		atari_keyboard[51] = ps2_keys_reg[9'h03d];
		atari_keyboard[53] = ps2_keys_reg[9'h03e];
		atari_keyboard[48] = ps2_keys_reg[9'h046];
		atari_keyboard[17] = ps2_keys_reg[9'h16c] | ps2_keys_reg[9'h003];
		atari_keyboard[52] = ps2_keys_reg[9'h066];
		atari_keyboard[28] = ps2_keys_reg[9'h076];
		atari_keyboard[39] = ps2_keys_reg[9'h111];
		atari_keyboard[60] = ps2_keys_reg[9'h058];
		atari_keyboard[44] = ps2_keys_reg[9'h00d];
		atari_keyboard[12] = ps2_keys_reg[9'h05a];
		atari_keyboard[33] = ps2_keys_reg[9'h029];
		atari_keyboard[54] = ps2_keys_reg[9'h04e];
		atari_keyboard[55] = ps2_keys_reg[9'h055];
		atari_keyboard[15] = ps2_keys_reg[9'h05b] | ps2_keys_reg[9'h172];  // ] -> DN
		atari_keyboard[14] = ps2_keys_reg[9'h054] | ps2_keys_reg[9'h175];  // [ -> UP
		atari_keyboard[6]  = ps2_keys_reg[9'h052] | ps2_keys_reg[9'h16b];  // ' -> Left
		atari_keyboard[7]  = ps2_keys_reg[9'h05d] | ps2_keys_reg[9'h174];  // \ -> Right
		atari_keyboard[38] = ps2_keys_reg[9'h04a];
		atari_keyboard[2]  = ps2_keys_reg[9'h04c];
		atari_keyboard[32] = ps2_keys_reg[9'h041];
		atari_keyboard[34] = ps2_keys_reg[9'h049];
		atari_keyboard[3]  = ps2_keys_reg[9'h005];
		atari_keyboard[4]  = ps2_keys_reg[9'h006];
		atari_keyboard[19] = ps2_keys_reg[9'h004];
		atari_keyboard[20] = ps2_keys_reg[9'h00c];
		shift_pressed      = ps2_keys_reg[9'h012] | ps2_keys_reg[9'h059];
		control_pressed    = ps2_keys_reg[9'h014] | ps2_keys_reg[9'h114] | ps2_keys_reg[9'h172] | ps2_keys_reg[9'h175] | ps2_keys_reg[9'h16b] | ps2_keys_reg[9'h174];
		break_pressed      = ps2_keys_reg[9'h077] | ps2_keys_reg[9'h00e];
	end

	// POKEY walks the matrix with an inverted scan code.
	always @(*) begin
		KEYBOARD_RESPONSE = 2'b11;

		if (atari_keyboard[~KEYBOARD_SCAN])
			KEYBOARD_RESPONSE[0] = 1'b0;

		case (KEYBOARD_SCAN[5:4])
			2'b00: if (break_pressed)   KEYBOARD_RESPONSE[1] = 1'b0;
			2'b10: if (shift_pressed)   KEYBOARD_RESPONSE[1] = 1'b0;
			2'b11: if (control_pressed) KEYBOARD_RESPONSE[1] = 1'b0;
			default: ;
		endcase
	end

endmodule
