// k7800 (c) by Jamie Blanks
//
// Copyright (c) 2026 Jamie Blanks
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

// POKEY audio: four channels, AUDCTL, STIMER, and the timer events that reach
// the interrupt logic.
//
// Source: PokeyReSchem-13.pdf page 5.
//
// ---------------------------------------------------------------------------
// AUDCTL, from the CO12294 spec
// ---------------------------------------------------------------------------
//   D7  17 bit poly becomes 9 bit
//   D6  channel 1 clocked at 1.79 MHz instead of the base clock
//   D5  channel 3 clocked at 1.79 MHz instead of the base clock
//   D4  channel 2 clocked by channel 1   (16 bit pair)
//   D3  channel 4 clocked by channel 3   (16 bit pair)
//   D2  high pass in channel 1, clocked by channel 3
//   D1  high pass in channel 2, clocked by channel 4
//   D0  base clock becomes 15 kHz instead of 64 kHz
//
// The cross wiring is worth stating plainly because it is easy to mirror by
// mistake: the 16 bit pairs are 1+2 and 3+4, but the high pass pairs are 1+3
// and 2+4. Only channels 1 and 2 have a filter at all.
//
// ---------------------------------------------------------------------------
// Timers
// ---------------------------------------------------------------------------
// Channels 1, 2 and 4 raise interrupts when their counter crosses zero.
// Channel 3 has no timer interrupt - it is the odd one out, and wiring it up
// would invent an interrupt source the chip does not have.
//
// STIMER reloads every counter and forces the outputs to a known but
// asymmetric state: high for channels 1 and 2, low for channels 3 and 4.

`default_nettype none

module pokey_audio (
	input  wire        clk,

	// Write strobes, one per register, from pokey_bus.
	input  wire        audf1_wr, audc1_wr,
	input  wire        audf2_wr, audc2_wr,
	input  wire        audf3_wr, audc3_wr,
	input  wire        audf4_wr, audc4_wr,
	input  wire        audctl_wr,
	input  wire        stimer,
	input  wire [7:0]  write_data,

	// Resynchronisations forced by the serial sheet.
	input  wire        twotone_reset, // channels 1 and 2
	input  wire        async_reset,   // channels 3 and 4

	// Clocks from pokey_clkdiv.
	input  wire        clk179_en,
	input  wire        ph2_en,
	input  wire        clk64_en,
	input  wire        clk15_en,

	// Polys from pokey_poly.
	input  wire        poly4,
	input  wire        poly5,
	input  wire        poly17,

	output wire        poly9_sel,     // AUDCTL bit 7, back to pokey_poly
	output wire        timer1,        // to the interrupt logic
	output wire        timer2,
	output wire        timer4,
	output wire [3:0]  dac1, dac2, dac3, dac4
);
	// -----------------------------------------------------------------------
	// AUDCTL
	// -----------------------------------------------------------------------
	// Page 5 draws AUDCTL as Cell 5 everywhere except bits 2 and 1, which are
	// Cell 12 and hand out the complement - those two are the high pass
	// enables, whose consumer wants them inverted.
	wire [7:0] audctl;
	wire [2:1] hp_en_n;

	pokey_cell5  #(.WIDTH(5)) u_audctl_hi (
		.clk, .ld(audctl_wr), .d(write_data[7:3]), .q(audctl[7:3]));
	pokey_cell12 #(.WIDTH(2)) u_audctl_hp (
		.clk, .ld(audctl_wr), .d(write_data[2:1]), .q_n(hp_en_n));
	pokey_cell5  #(.WIDTH(1)) u_audctl_lo (
		.clk, .ld(audctl_wr), .d(write_data[0]), .q(audctl[0]));

	assign audctl[2:1] = ~hp_en_n;

	assign poly9_sel = audctl[7];

	wire ch1_fast   = audctl[6];
	wire ch3_fast   = audctl[5];
	wire ch2_from_1 = audctl[4];
	wire ch4_from_3 = audctl[3];
	wire hp1_en     = audctl[2];
	wire hp2_en     = audctl[1];
	wire base_15k   = audctl[0];

	wire base_en = base_15k ? clk15_en : clk64_en;

	// -----------------------------------------------------------------------
	// Clock routing
	// -----------------------------------------------------------------------
	wire bor1, bor2, bor3, bor4;

	wire ch1_en = ch1_fast   ? clk179_en : base_en;
	wire ch2_en = ch2_from_1 ? bor1      : base_en;
	wire ch3_en = ch3_fast   ? clk179_en : base_en;
	wire ch4_en = ch4_from_3 ? bor3      : base_en;

	// Reload latency, in 1.79 MHz cycles, from Atari's modified formula:
	// M = 4 for an 8 bit counter and M = 7 for a 16 bit pair, against a normal
	// M = 1. So 3 extra cycles, or 6 when paired. Only visible when the counter
	// runs at 1.79 MHz; at 64 kHz or 15 kHz it hides inside one slow period,
	// which is exactly why the spec says to use the normal formula there.
	wire [2:0] delay1 = ch1_fast ? (ch2_from_1 ? 3'd6 : 3'd3) : 3'd0;
	wire [2:0] delay3 = ch3_fast ? (ch4_from_3 ? 3'd6 : 3'd3) : 3'd0;

	// -----------------------------------------------------------------------
	// Two-tone resynchronisation
	// -----------------------------------------------------------------------
	// The reset a two-tone toggle sends back to channels 1 and 2 does not
	// arrive at the toggle. It is two 1.79 MHz cycles late, which is why a fast
	// channel 1 in two-tone runs at AUDF + 6 where the same channel on its own
	// runs at AUDF + 4 - Altirra measures exactly that split
	// (TestEmu_PokeyTimers.cpp) and Watson spends a two stage delay
	// line on it (pokey.vhdl).
	//
	// The stages shift on 1.79 MHz, but the toggle can come from a channel
	// running slower than that, so the pulse is caught and held until the next
	// shift rather than sampled. Same structure as Watson's latch_delay_line.
	logic [1:0] tt_sr;
	logic       tt_hold;

	always_ff @(posedge clk) begin
		if (clk179_en) begin
			tt_sr   <= {tt_hold | twotone_reset, tt_sr[1]};
			tt_hold <= 1'b0;
		end else if (twotone_reset) begin
			tt_hold <= 1'b1;
		end
	end

	wire twotone_reset_d = tt_sr[0] & clk179_en;

	// Channels 3 and 4 take the asynchronous start reset with no such delay.
	wire ch12_resync = twotone_reset_d;
	wire ch34_resync = async_reset;

	// -----------------------------------------------------------------------
	// The four channels
	// -----------------------------------------------------------------------
	wire out1, out2, out3, out4;
	wire bor1_raw, bor2_raw, bor3_raw, bor4_raw;

	pokey_chan u_ch1 (
		.clk, .wr_en(audf1_wr), .audc_wr(audc1_wr), .write_data,
		.count_en(ch1_en), .clk179_en, .ph2_en,
		.wide_lo(ch2_from_1), .force_reload((ch2_from_1 & bor2_raw) | ch12_resync),
		.reload_delay(delay1), .pulse_mask(ch12_resync),
		.poly4, .poly5, .poly17,
		.stimer, .stimer_level(1'b1),
		.hp_clock(bor3), .hp_enable(hp1_en),
		.borrow(bor1), .borrow_raw(bor1_raw), .out(out1), .dac(dac1));

	pokey_chan u_ch2 (
		.clk, .wr_en(audf2_wr), .audc_wr(audc2_wr), .write_data,
		.count_en(ch2_en), .clk179_en, .ph2_en,
		.wide_lo(1'b0), .force_reload(ch12_resync), .reload_delay(3'd0),
		.pulse_mask(ch12_resync),
		.poly4, .poly5, .poly17,
		.stimer, .stimer_level(1'b1),
		.hp_clock(bor4), .hp_enable(hp2_en),
		.borrow(bor2), .borrow_raw(bor2_raw), .out(out2), .dac(dac2));

	pokey_chan u_ch3 (
		.clk, .wr_en(audf3_wr), .audc_wr(audc3_wr), .write_data,
		.count_en(ch3_en), .clk179_en, .ph2_en,
		.wide_lo(ch4_from_3), .force_reload((ch4_from_3 & bor4_raw) | ch34_resync),
		.reload_delay(delay3), .pulse_mask(ch34_resync),
		.poly4, .poly5, .poly17,
		.stimer, .stimer_level(1'b0),
		.hp_clock(1'b0), .hp_enable(1'b0),
		.borrow(bor3), .borrow_raw(bor3_raw), .out(out3), .dac(dac3));

	pokey_chan u_ch4 (
		.clk, .wr_en(audf4_wr), .audc_wr(audc4_wr), .write_data,
		.count_en(ch4_en), .clk179_en, .ph2_en,
		.wide_lo(1'b0), .force_reload(ch34_resync), .reload_delay(3'd0),
		.pulse_mask(ch34_resync),
		.poly4, .poly5, .poly17,
		.stimer, .stimer_level(1'b0),
		.hp_clock(1'b0), .hp_enable(1'b0),
		.borrow(bor4), .borrow_raw(bor4_raw), .out(out4), .dac(dac4));

	// Channel 3 deliberately has no timer output.
	assign timer1 = bor1;
	assign timer2 = bor2;
	assign timer4 = bor4;

`ifdef VERILATOR
	// A sink for Verilator's UNUSED check; Quartus warns 10036 on it instead.
	/* verilator lint_off UNUSED */
	wire _unused = &{out1, out2, out3, out4, bor1_raw, bor3_raw, 1'b0};
	/* verilator lint_on UNUSED */
`endif

endmodule

`default_nettype wire
