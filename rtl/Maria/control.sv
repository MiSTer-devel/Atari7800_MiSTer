// k7800 (c) by Jamie Blanks
//
// Copyright (c) 2021-2026 Jamie Blanks
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

module control (
	input  logic             mclk0,
	input  logic             mclk1,
	input  logic             pclkp, // cpu clock phase
	input  logic             maria_en,
	input  logic [15:0]      AB,
	input  logic [7:0]       DB_in,
	output logic [7:0]       DB_out,
	output logic [7:0]       DB_out_oe,
	input  logic             RW,
	input  logic             ABEN,

	input  logic             drive_AB,

	output logic [7:0]       ctrl,
	output logic [24:0][7:0] color_map,
	input  logic [7:0]       status_read,
	output logic [7:0]       char_base,
	output logic [15:0]      ZP,
	input  logic             pal,

	// whether to slow pclk_0 for slow memory accesses
	output logic             sel_slow_clock,

	// when wait_sync is written to, ready is deasserted
	output logic             wsync,

	input  logic             clk_sys,
	input  logic             reset,
	input  logic             pclk0,
	input  logic             bypass_bios,
	output logic             cs_ram0,
	output logic             cs_ram1,
	output logic             cs_riot,
	output logic             cs_tia,
	output logic             cs_maria,
	output logic             cram_select
);

	// Internal Memory Mapped Registers
	logic [7:0]              ZPH, ZPL;
	assign sel_slow_clock = ~maria_en ? 1'b1 : (cs_tia || cs_riot);

	assign ZP = {ZPH, ZPL};

	always_comb begin
		{cs_ram0, cs_ram1, cs_riot, cs_tia, cs_maria} = 0;
		if (maria_en) casex (AB[15:5])
				// RIOT RAM: "Do Not Use" in 7800 mode.
				11'b0000_010x_1xx,
				11'b0000_001x_1xx: cs_riot = ~ABEN;

				// 1800-1FFF: 2K RAM.
				11'b0001_1xxx_xxx: cs_ram1 = 1;

				// 0040-00FF: Zero Page (Local variable space)
				// 0140-01FF: Stack
				11'b0000_000x_01x,
				11'b0000_000x_1xx,

				// 2000-27FF: 2K RAM. Zero Page and Stack mirrored from here.
				11'b0010_0xxx_xxx: cs_ram0 = 1;

				// TIA Registers:
				// 0000-001F, 0100-001F, 0200-021F, 0300-031F
				// All mirrors are ranges of the same registers
				11'b0000_00xx_000: cs_tia = ~ABEN;

				// MARIA Registers:
				// 0020-003F, 0120-003F, 0220-023F, 0320-033F
				// All ranges are mirrors of the same registers
				11'b0000_00xx_001: cs_maria = ~ABEN;
				default: ;

		endcase else casex (AB[15:5])
				11'bXXX0_XX0X_1XX,
				11'bxxx0_xx1x_1xx: cs_riot = 1;
				11'bxxx0_xxxx_0xx: cs_tia = 1;
				default: ;
		endcase
	end
	assign cram_select = cs_maria && ~RW && (pclkp && ~old_phase) && (|AB[1:0] || AB[4:0] == 5'h00);

	// CTRL is two stages deep. A write opens the first for phase 2, and phase
	// 1 opens the second, so a byte written to $3C reaches MARIA's control
	// signals half a cycle later - at the start of the phase 1 that follows
	// the write. Consecutive writes, as a read-modify-write on $3C makes,
	// each get their own phase 1.
	logic [7:0] ctrl_1;
	logic old_phase;
	wire [4:0] color_ram_index [32] = '{5'd0,
			5'd1,  5'd2,  5'd3,  5'd0, 5'd4,  5'd5,  5'd6,  5'd0,
			5'd7,  5'd8,  5'd9,  5'd0, 5'd10, 5'd11, 5'd12, 5'd0,
			5'd13, 5'd14, 5'd15, 5'd0, 5'd16, 5'd17, 5'd18, 5'd0,
			5'd19, 5'd20, 5'd21, 5'd0, 5'd22, 5'd23, 5'd24};
	always_ff @(posedge clk_sys) begin
		if (pclkp) begin
			wsync <= 1'b0;
			if (~RW && cs_maria) begin
				case(AB[5:0])
					6'h24: wsync <= 1'b1;
					6'h28: ; //vblank status read
					6'h2c: ZPH <= DB_in;
					6'h30: ZPL <= DB_in;
					6'h34: char_base <= DB_in;
					6'h38: ; // No register
					6'h3c: ctrl_1 <= DB_in;
					default: if (cram_select) color_map[color_ram_index[AB[4:0]]] <= DB_in;
				endcase
			end
		end else if (mclk0)
			ctrl <= ctrl_1; // second stage, open for phase 1

		if (mclk0) begin
			old_phase <= pclkp;
		end

		if (reset || ~maria_en) begin
			// '1 and the BIOS's $60 both hold DM = 11, so DMA stays off
			// either way; $60 is what the BIOS actually leaves behind.
			ctrl_1 <= bypass_bios ? 8'h60 : '1;
			ctrl <= bypass_bios ? 8'h60 : '1;
			color_map <= 200'b0; // FIXME: convert this to RAM?
			char_base <= 8'b0;
			// NTSC measured off the real BIOS at handoff; the PAL arm is
			// unmeasured and left as it was.
			{ZPH,ZPL} <= bypass_bios ? (pal ? {8'h27, 8'h30} : {8'h1F, 8'h84}) : 8'd0;
		end
	end

	// MARIA presents read data as a bus buffer, not a phase 2 register. The
	// CPU closes its data latch as phase 2 opens, so a value that only lands
	// part way through that phase reads one cycle stale - MSTAT's live VBLANK
	// bit is the one register where that shows. rtl/TIA.sv drives its pins the
	// same way for the same reason.
	//
	// MSTAT is one pass transistor from VBLANK onto D7, and it is the only
	// read driver MARIA has - the CTRL register and the colour map are
	// write-only cells with no path back to the bus. So D6-D0 of an MSTAT
	// read, and every other MARIA address, leave the bus holding what it
	// already had.
	always_comb begin
		DB_out = status_read;
		DB_out_oe = (RW && cs_maria && AB[5:0] == 6'h28) ? 8'h80 : 8'h00;
	end
endmodule
