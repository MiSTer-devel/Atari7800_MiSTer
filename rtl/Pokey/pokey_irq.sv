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

// POKEY interrupts and the two serial/keyboard control registers.
//
// Source: PokeyReSchem-13.pdf page 3, right side.
// Register semantics from the CO12294 spec.
//
// ---------------------------------------------------------------------------
// IRQEN (write $0E) and IRQST (read $0E)
// ---------------------------------------------------------------------------
//   D7 break key      D3 serial output transmission finished
//   D6 other key      D2 timer 4
//   D5 serial input data ready    D1 timer 2
//   D4 serial output data needed  D0 timer 1
//
// One bit of the sheet, drawn eight times across the top right:
//
//   IRQEN --Cell 12--> nEn --+-->|NOR 1|--o-- n1 --+--> Cell 9 --> D7r
//                            |      ^              |
//                            |      +--------------+---> wired NOR -> IRQ pad
//                            |      |
//                   setX --->|NOR 2 |<--------------+
//
//   n1 = ~(nEn | n2)      n2 = ~(setX | n1)
//
// Cell 12 hands out the complement, so nEn is low when the source is enabled.
// With the source disabled n1 is stuck low and n2 simply follows ~setX, so the
// latch does not remember anything; with it enabled, setX drops n2 and it
// stays down until IRQEN is written with a zero. That is why the spec says the
// only acknowledge is to disable the bit and enable it again.
//
// NOR 1 is drawn phase 1 and NOR 2 phase 2, with no X on either - a plain two
// phase cross coupled latch, so its state moves at most once per clock and
// every setX is sampled. It is modelled as the one register below.
//
// The set input is level sensitive, and that is a requirement on the sources:
// a source held high is re-set every cycle and survives the disable/enable
// that is the only acknowledge there is. Every set* on the sheet is a pulse
// for that reason - setBreak reaches this row through an edge detector, not
// from the break latch itself. See pokey_kbd.sv.
//
// n1 is "enabled and pending". It gates the Cell 9 that drives the read bus,
// which inverts, so IRQST is active low. It also gates one pull down on a
// wired NOR line that runs the width of the sheet; that line goes through one
// inverter into the gate of a single grounded transistor at the IRQ pad, so
// IRQ is open drain and pulls low when any source is asserting.
//
// ---------------------------------------------------------------------------
// SKCTL (write $0F) and SKSTAT (read $0F)
// ---------------------------------------------------------------------------
// SKCTL: D7 force break, D6..D4 serial mode, D3 two tone, D2 fast pot scan,
// D1..D0 keyboard scan and debounce enable. Eight Cell 5, and a NOR of bits 1
// and 0 makes Init.
//
// SKCTL[1:0] == 00 is init mode, and it is the reset this chip has instead of
// a reset pin: it parks the polys and holds the dividers. The 7800 BIOS and
// every Atari OS write $03 here to bring POKEY out of it.
//
// SKSTAT is a row of seven Cell 9, so bits 7..1 read inverted and bit 0, which
// has no driver at all, reads as the precharged one:
//   D7 frame error   D6 keyboard overrun   D5 serial input overrun
//   D4 SID pad, read directly    D3 kShift    D2 lkeyDown
//   D1 serial shift register busy   D0 no driver, always 1
//
// Bits 7, 6 and 5 come off NOR pair latches that are cleared only by a write
// to SKRES ($0A). Those pairs are phase 2 over phase 1, and SKRES arrives on
// the phase 1 gate's X input - uncoupled, a direct asynchronous clear. The
// clocked clear below, given priority over the set, is that at clk grain.
//
// The keyboard overrun is made here, as a NAND of setKey with the IRQST bit 6
// node - a new key accepted while the last one is still asserting - which is
// why pokey_kbd cannot form it. That gate carries no digit: it is static.

`default_nettype none

module pokey_irq (
	input  wire       clk,

	input  wire       irqen_wr,
	input  wire       skctl_wr,
	input  wire       irqst_rd,     // AddrEr, the read strobe at address $0E
	input  wire       skstat_rd,    // AddrFr, the read strobe at address $0F
	input  wire       skres,        // write strobe at address $0A
	input  wire [7:0] write_data,

	// Interrupt sources, each a single cycle pulse.
	input  wire       timer1,
	input  wire       timer2,
	input  wire       timer4,
	input  wire       break_key,
	input  wire       other_key,
	input  wire       serin_ready,
	input  wire       serout_needed,
	input  wire       serout_done,

	// Status sources.
	input  wire       frame_error,
	input  wire       kbd_overrun,
	input  wire       sid_pad,
	input  wire       shift_key,
	input  wire       key_down,
	input  wire       serin_busy,

	output wire [7:0] irqst,        // this sheet's pull on the read bus, $0E
	output wire [7:0] skstat,       // and at $0F
	output wire [7:0] skctl,
	output wire       init_mode,    // SKCTL[1:0] == 00
	output wire       irq_n_out,
	output wire       irq_oe
);
	// -----------------------------------------------------------------------
	// IRQEN. Cell 12 taps between the inverters, so the row leaves complement.
	// -----------------------------------------------------------------------
	wire [7:0] irqen_n;
	pokey_cell12 #(.WIDTH(8)) u_irqen (
		.clk, .ld(irqen_wr), .d(write_data), .q_n(irqen_n));

	// -----------------------------------------------------------------------
	// The eight NOR pair latches. n2 is the stored side; `pend` is ~n2.
	// -----------------------------------------------------------------------
	wire [7:0] set_src = {
		break_key, other_key, serin_ready, serout_needed,
		serout_done, timer4, timer2, timer1
	};

	logic [7:0] pend;

	always_ff @(posedge clk)
		pend <= set_src | (pend & ~irqen_n);

	// Bit 3, SEROC, is the exception and page 3 draws it that way: `sdoFinish`
	// reaches IRQST directly, without the cross-coupled pair the other seven
	// bits get. The spec says so in as many words - it is not a latch, IRQEN
	// cannot reset it, and it reads zero whenever the output shifter is empty.
	// So the live source replaces the stored bit here, which is what makes
	// polling SEROC work and what lets enabling IRQEN bit 3 after a
	// transmission has already finished assert the interrupt immediately.
	wire [7:0] pend_eff = {pend[7:4], set_src[3], pend[2:0]};

	wire [7:0] irq_assert = pend_eff & ~irqen_n;  // n1

	// -----------------------------------------------------------------------
	// IRQST readback: one Cell 9 per bit onto the precharged bus.
	// -----------------------------------------------------------------------
	wire [7:0] irqst_q, irqst_oe;

	genvar i;
	generate
		for (i = 0; i < 8; i = i + 1) begin : g_irqst
			pokey_cell9 u (.in(irq_assert[i]), .rd(irqst_rd),
				.q_n(irqst_q[i]), .q_oe(irqst_oe[i]));
		end
	endgenerate

	// A bit no driver pulls down stays at the precharged one.
	assign irqst = irqst_q | ~irqst_oe;

	// The wired NOR, its inverter, and the one grounded transistor at the pad.
	assign irq_n_out = 1'b0;
	assign irq_oe    = |irq_assert;

	// -----------------------------------------------------------------------
	// SKCTL
	// -----------------------------------------------------------------------
	wire [7:0] skctl_q;
	pokey_cell5 #(.WIDTH(8)) u_skctl (
		.clk, .ld(skctl_wr), .d(write_data), .q(skctl_q));

	assign skctl     = skctl_q;
	assign init_mode = ~(skctl_q[1] | skctl_q[0]);

	// -----------------------------------------------------------------------
	// SKSTAT. The three latching bits are held here in bus polarity: one is
	// "no error", which is what the Cell 9 has to invert onto the bus.
	// -----------------------------------------------------------------------
	// Both overruns are built here, and neither comes off a read strobe. Page 6
	// carries `AddrDr` exactly once, straight into the SERIN Cell 1 row's Rd
	// pins, so a read has no side effect and the serial overrun cannot be built
	// on that sheet - it is the same shape as the keyboard one: a byte
	// completing while the previous interrupt is still asserted. `pend` is
	// registered, so during the `serin_ready` pulse `irq_assert[5]` still shows
	// the previous byte's state, which is exactly the comparison wanted.
	wire key_ovrun = kbd_overrun | (other_key & irq_assert[6]);
	wire ser_ovrun = serin_ready & irq_assert[5];

	logic lat_frame, lat_kbd_ovr, lat_ser_ovr;

	always_ff @(posedge clk) begin
		if (skres) begin
			lat_frame   <= 1'b1;
			lat_kbd_ovr <= 1'b1;
			lat_ser_ovr <= 1'b1;
		end else begin
			if (frame_error)   lat_frame   <= 1'b0;
			if (key_ovrun)     lat_kbd_ovr <= 1'b0;
			if (ser_ovrun)     lat_ser_ovr <= 1'b0;
		end
	end

	wire [7:1] skstat_in = {
		~lat_frame,       // D7
		~lat_kbd_ovr,     // D6
		~lat_ser_ovr,     // D5
		~sid_pad,         // D4, the sheet calls it sdDelay
		shift_key,        // D3
		key_down,         // D2, lkeyDown
		serin_busy        // D1
	};

	wire [7:1] skstat_q, skstat_oe;

	generate
		for (i = 1; i < 8; i = i + 1) begin : g_skstat
			pokey_cell9 u (.in(skstat_in[i]), .rd(skstat_rd),
				.q_n(skstat_q[i]), .q_oe(skstat_oe[i]));
		end
	endgenerate

	assign skstat[7:1] = skstat_q | ~skstat_oe;
	assign skstat[0]   = 1'b1;        // no driver on the sheet

endmodule

`default_nettype wire
