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

// POKEY C012294 - top level.
//
// Source: references/Pokey/schematics/PokeyReSchem-13.pdf, all sheets.
// Register semantics from references/Pokey/die/pokey.pdf (Atari CO12294 Rev B).
//
// ---------------------------------------------------------------------------
// Pin contract
// ---------------------------------------------------------------------------
// The port list is the real 40 pin part, not the convenience ports the VHDL
// this replaces exposed. Bidirectional and open drain pins are split into
// value and enable per AGENTS.md, because an FPGA boundary has no `z`:
//
//   D0-D7  ->  d_in, d_out, d_oe
//   IRQ    ->  irq_n_out, irq_oe     (open drain, only ever pulls low)
//   SOD    ->  sod_out, sod_oe
//   OCLK   ->  oclk_out, oclk_oe
//   BCLK   ->  bclk, bclk_out, bclk_oe
//
// Clocking: the real part has one PHI2 pin and derives two non-overlapping
// phases from it. This takes both phase enables instead - see pokey_clkgen.sv
// for why. They must never be high in the same clk cycle.
//
// AUD is analog on the real part: four Cell 11 DACs steer current into one pin.
// Both forms are exposed, per decision 0002 - the four per-channel drives, and
// a mixed value from pokey_mixer. The host picks.
//
// ---------------------------------------------------------------------------
// The read path
// ---------------------------------------------------------------------------
// There is no read mux here. Every readable register drives the precharged
// D0r..D7r bus through the tri-state cell its sheet gives it, and this file
// only brings the eight lines together - see the read bus section below.

`default_nettype none

module pokey (
	input  wire        clk,

	// The two phases of the target 1.79 MHz clock, one clk cycle each.
	input  wire        ph1_en,
	input  wire        ph2_en,

	// Bus pins.
	input  wire  [3:0] a,
	input  wire        cs0_n,      // pin 30, active low
	input  wire        cs1,        // pin 31, active high
	input  wire        rw,         // high = read
	input  wire  [7:0] d_in,
	output wire  [7:0] d_out,
	output wire        d_oe,

	// Keyboard pins. K0-K5 are the scan lines the chip DRIVES, walking 00..3F;
	// KR1 and KR2 are the two sense lines it reads back. The spec is explicit:
	// "six key scan lines (K0-K5), which holds a value from 00 to 3F. There are
	// two sense lines." KR1 senses the full decode, KR2 only CTRL, SHIFT and
	// BREAK. Both sense lines are active low.
	output wire  [5:0] k,
	input  wire        kr1,
	input  wire        kr2,

	// Pot pins.
	input  wire  [7:0] p,

	// Serial pins.
	input  wire        sid,        // serial input data
	input  wire        bclk,       // bi-directional clock in
	output wire        sod_out,
	output wire        sod_oe,
	output wire        oclk_out,   // transmit clock, always driven
	output wire        oclk_oe,
	output wire        bclk_out,   // bi-directional clock, driven in two modes
	output wire        bclk_oe,

	// Interrupt pin, open drain.
	output wire        irq_n_out,
	output wire        irq_oe,

	// Audio. Per-channel DAC drive, and the mixed AUD node.
	output wire  [3:0] dac1, dac2, dac3, dac4,
	output wire [15:0] aud
);
	// -----------------------------------------------------------------------
	// Clock phases
	// -----------------------------------------------------------------------
	wire phi1, phi2, phi2b, pre_s01;
	pokey_clkgen u_clkgen (
		.clk, .ph1_en, .ph2_en, .phi1(phi1), .phi2(phi2), .phi2b(phi2b), .pre_s01(pre_s01));

	// One 1.79 MHz tick per POKEY clock. o2 is the half the bus writes land in,
	// so the rest of the chip is stepped off o1 to keep the two apart.
	wire phi2_tick = ph1_en;

	// -----------------------------------------------------------------------
	// Bus
	// -----------------------------------------------------------------------
	wire  [7:0] write_data;
	wire [15:0] addr_rd, addr_wr, addr_wr_n;
	wire        stimer_strobe, skres_strobe, potgo_strobe;
	wire  [7:0] read_data;

	pokey_bus u_bus (
		.clk, .pre_s01, .phi2b,
		.a, .cs0_n, .cs1, .rw, .d_in,
		.read_bus_n(~read_data),
		.d_out, .d_oe, .write_data,
		.addr_rd, .addr_wr, .addr_wr_n,
		.stimer_strobe, .skres_strobe, .potgo_strobe);

	// -----------------------------------------------------------------------
	// Control registers and interrupts
	// -----------------------------------------------------------------------
	wire [7:0] irqst_bus, skstat_bus, skctl;
	wire       init_mode;
	wire       timer1, timer2, timer4;
	wire       break_key, other_key, kbd_overrun, shift_key, key_down;
	wire [7:0] kbcode_bus;
	wire       serin_ready, serout_needed, serout_done;
	wire       serin_busy, frame_error;
	wire [7:0] serin_bus;
	wire       twotone_reset, async_reset;

	pokey_irq u_irq (
		.clk,
		.irqen_wr (addr_wr[4'hE]),
		.skctl_wr (addr_wr[4'hF]),
		.irqst_rd (addr_rd[4'hE]),
		.skstat_rd(addr_rd[4'hF]),
		.skres    (skres_strobe),
		.write_data,
		.timer1, .timer2, .timer4,
		.break_key,
		.other_key,
		.serin_ready,
		.serout_needed,
		.serout_done,
		.frame_error,
		.kbd_overrun,
		.sid_pad       (sid),
		.shift_key,
		.key_down,
		.serin_busy,
		.irqst(irqst_bus), .skstat(skstat_bus), .skctl, .init_mode,
		.irq_n_out, .irq_oe);

	// -----------------------------------------------------------------------
	// Clock dividers
	// -----------------------------------------------------------------------
	wire clk179_en, clk64_en, clk15_en;
	pokey_clkdiv u_clkdiv (
		.clk, .phi2_en(phi2_tick), .init(init_mode),
		.clk179_en, .clk64_en, .clk15_en);

	// -----------------------------------------------------------------------
	// Polys
	// -----------------------------------------------------------------------
	wire       poly4, poly5, poly17, poly9_sel;
	wire [7:0] random_bus;

	pokey_poly u_poly (
		.clk, .poly_en(clk179_en), .init(init_mode), .poly9_sel,
		.random_rd(addr_rd[4'hA]),
		.poly4_out(poly4), .poly5_out(poly5), .poly17_out(poly17),
		.random(random_bus));

	// -----------------------------------------------------------------------
	// Audio
	// -----------------------------------------------------------------------
	pokey_audio u_audio (
		.clk,
		.audf1_wr (addr_wr[4'h0]), .audc1_wr (addr_wr[4'h1]),
		.audf2_wr (addr_wr[4'h2]), .audc2_wr (addr_wr[4'h3]),
		.audf3_wr (addr_wr[4'h4]), .audc3_wr (addr_wr[4'h5]),
		.audf4_wr (addr_wr[4'h6]), .audc4_wr (addr_wr[4'h7]),
		.audctl_wr(addr_wr[4'h8]),
		.stimer   (stimer_strobe),
		.write_data,
		.twotone_reset, .async_reset,
		.clk179_en, .clk64_en, .clk15_en, .ph2_en,
		.poly4, .poly5, .poly17, .poly9_sel,
		.timer1, .timer2, .timer4,
		.dac1, .dac2, .dac3, .dac4);

	pokey_mixer u_mixer (.dac1, .dac2, .dac3, .dac4, .aud);

	// -----------------------------------------------------------------------
	// Pots
	// -----------------------------------------------------------------------
	wire [7:0] pot_bus;
	wire       dump_n;

	// The stored counts and the ALLPOT byte are exposed by the sheet for
	// tracing and its unit test; the chip reads them through the bus.
	/* verilator lint_off PINCONNECTEMPTY */
	pokey_pots u_pots (
		.clk, .ph1_en, .ph2_en,
		.potgo(potgo_strobe), .init(init_mode), .fast_scan(skctl[2]),
		.clk15_en, .clk64_en,
		.pot_in(p),
		.pot_rd(addr_rd[7:0]), .allpot_rd(addr_rd[4'h8]),
		.pot_bus,
		.pot0(), .pot1(), .pot2(), .pot3(),
		.pot4(), .pot5(), .pot6(), .pot7(),
		.allpot(), .dump_n);
	/* verilator lint_on PINCONNECTEMPTY */

	// -----------------------------------------------------------------------
	// Keyboard
	// -----------------------------------------------------------------------
	// The scan clock comes off the 15 kHz divider, the same rate the pot scan
	// uses: one step per scan line.
	/* verilator lint_off PINCONNECTEMPTY */
	pokey_kbd u_kbd (
		.clk,
		.scan_en    (clk15_en),
		.kbd_enable (skctl[1]),
		.debounce_en(skctl[0]),
		.init       (init_mode),
		.kr1, .kr2,
		.kbcode_rd  (addr_rd[4'h9]),
		.k, .kbcode(), .kbcode_bus,
		.key_down, .shift_key, .break_key, .other_key, .kbd_overrun);
	/* verilator lint_on PINCONNECTEMPTY */

	// -----------------------------------------------------------------------
	// Serial
	// -----------------------------------------------------------------------
	/* verilator lint_off PINCONNECTEMPTY */
	pokey_serial u_serial (
		.clk,
		.skctl, .init(init_mode),
		.serout_wr (addr_wr[4'hD]),
		.serin_rd  (addr_rd[4'hD]),
		.write_data,
		.timer1, .timer2, .timer4, .bclk,
		.sid,
		.serin(), .serin_bus,
		.sod_out, .sod_oe, .oclk_out, .oclk_oe, .bclk_out, .bclk_oe,
		.twotone_reset, .async_reset,
		.serin_ready, .serout_needed, .serout_done,
		.serin_busy, .frame_error);
	/* verilator lint_on PINCONNECTEMPTY */

	// -----------------------------------------------------------------------
	// Read bus
	// -----------------------------------------------------------------------
	// D0r..D7r is one precharged line per bit, running across every sheet.
	// There is no mux: each readable register hangs a tri-state cell on the
	// line it belongs to and the cell can only pull it down, so the bus is a
	// wired-AND and an address nobody answers - B and C, or any address at all
	// while no read strobe is up - reads $FF because nothing pulls.
	//
	// Each sheet hands over its own pull already merged, all ones where its
	// drivers are off, so this is just the eight lines meeting.
	assign read_data = pot_bus & kbcode_bus & random_bus & serin_bus
	                 & irqst_bus & skstat_bus;

	// Addresses B and C have no read register, and the write strobe
	// complements are for sheets that do not use them.
`ifdef VERILATOR
	// A sink for Verilator's UNUSED check; Quartus warns 10036 on it instead.
	/* verilator lint_off UNUSED */
	wire _unused = &{1'b0, phi1, phi2, addr_wr_n, dump_n,
	                 addr_rd[12:11], addr_wr[13:9]};
	/* verilator lint_on UNUSED */
`endif

endmodule

`default_nettype wire
