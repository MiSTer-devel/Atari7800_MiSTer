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

// POKEY's internal clock dividers: 64 kHz, 15 kHz, and the keyboard scan clock.
//
// Source: references/Pokey/schematics/PokeyReSchem-13.pdf page 4, right side.
//
// The divide ratios are not guesses. The CO12294 spec gives the exact
// frequencies for NTSC:
//
//   Fin      1.78979 MHz
//   64 kHz   63.9210 kHz   ->  1.78979e6 / 63.9210e3 = 28.0000
//   15 kHz   15.6999 kHz   ->  1.78979e6 / 15.6999e3 = 114.0001
//
// So the dividers are exactly 28 and 114, and the familiar "64 kHz" and
// "15 kHz" names are both roughly 6% off what the chip actually produces.
// AUDCTL bit 0 picks between them for every audio channel, so an error here
// detunes the whole chip rather than just the pot scan.
//
// PAL differs; the spec says so without giving the numbers. Nothing here
// depends on the region because the ratios are counted off the incoming clock,
// whatever its frequency.
//
// ---------------------------------------------------------------------------
// Neither divider is a counter
// ---------------------------------------------------------------------------
// Page 4 draws each one as a chain of two-phase dynamic inverter stages,
// signal flowing right to left across the sheet, closed by a feedback gate,
// plus a comparator that force-feeds a one at a single state so the length
// comes out even:
//
//   64 kHz   5 bits   XOR  of bits 3 and 5   force at 00010   ->  28
//   15 kHz   7 bits   XNOR of bits 6 and 7   force at 1001001 -> 114
//
// The digit drawn inside each stage is Atari's coupler-phase mark, not an
// instance number: `references/Pokey/die/pokey.pdf` page 46 says "NUMBER
// INDICATES CLOCK PHASE OF INPUT COUPLERS". Reading the digits in signal
// order, right to left, gives an unbroken run of ("2","1") pairs, and the
// count settles the chain lengths without appeal to any other source:
//
//   15 kHz  NOR"2", NOR"1", then 12 inverters 2,1,2,1,...  =  7 pairs
//   64 kHz  NOR"2", inv"1", then  8 inverters 2,1,2,1,...  =  5 pairs
//
// (digit positions in PDF points: 15 kHz row y=482 x=1795..2956, 64 kHz row
// y=855 x=1863..2627). So a bit is one ("2","1") pair, the chain advances once
// per target clock, and the pair is one register bit here. The bit a tap sees
// is the settled o1 node, which is why `phi2_en` is wired from the o1 strobe
// in pokey.sv despite its name. Numbering and both force states agree with
// Altirra, references/Pokey/implementations/altirra/pokey.cpp:2864-2877, which
// is where the 28 and 114 came from originally.
//
// Reading the comparator. It is the horizontal wire drawn under each chain: a
// depletion pull-up at the far end and one pull-down transistor per tap, each
// drawn as an open circle where the tap crosses the wire. That is a NOR - the
// wire goes high only when every tap is low - so a tap drawn through a small
// inverter is a bit that must read 1 at the force state, and a direct tap is a
// bit that must read 0. Taken the other way round the 15 kHz chain has period
// 3, not 114, which is how the convention was settled.
//
//   15 kHz taps, newest first:  inverted, direct, direct, inverted,
//                               direct, direct, inverted        -> 1001001
//   64 kHz taps, newest first:  direct, inverted, direct,
//                               direct, direct                  -> 00010
//
// Init. Both feedback gates draw their Init input with an X through it. That
// mark is now settled: page 46's legend reads "X INDICATES NO COUPLER ON THAT
// INPUT", so Init is a real, direct, unclocked input to a gate whose other
// inputs are phase clocked - which is why it is applied combinationally here
// rather than through the shift enable. Checked at 600 dpi on the 64 kHz gate,
// -f 4 -x 21100 -y 6500 -W 2300 -H 1200: a three-input NOR marked "2" with the
// X on the Init leg. Init reaches the two chains at different points and
// that is what produces the lopsided reset states Altirra documents: on the
// 15 kHz side it is one input of the second NOR of the pair, the "1" one, so
// it feeds zeros in; on the 64 kHz side it is one input of a three-input "2"
// NOR whose output passes through the "1" inverter, so it feeds ones in.
// Either way it lands on the newest bit's own pair. Nothing else in the drawing
// explains why one chain resets to all zeros and the other to all ones, and the
// resulting reset phases - first match 19 shifts later for the 64 kHz, 78 for
// the 15 kHz - are Altirra's numbers as well. An unwired Init could not do that.

`default_nettype none

module pokey_clkdiv (
	input  wire  clk,
	input  wire  phi2_en,      // 1.79 MHz, one clk pulse per target cycle
	input  wire  init,         // SKCTL init holds the dividers in reset

	output wire  clk179_en,    // straight through, for AUDCTL bits 5 and 6
	output logic clk64_en,
	output logic clk15_en
);
	assign clk179_en = phi2_en;

	// Divide by 28.  m[0] is bit 1, the newest.
	// All zeros is the XOR chain's lock-up state, so both chains power up where
	// Init would leave them.  The chip does the same: SKCTL comes up zero, which
	// is Init.
	/* verilator lint_off PROCASSINIT */
	logic [4:0] m = 5'b11111;
	wire  cmp64 = (m == 5'b00010);
	wire  fb64  = m[2] ^ m[4];
	// Three-input NOR (Init, comparator, feedback) then the chain's first "1"
	// inverter, so Init feeds ones in and the reset state is all ones.
	wire  new64 = init | cmp64 | fb64;

	// Divide by 114.  n[0] is bit 1.
	logic [6:0] n = 7'b0000000;
	/* verilator lint_on PROCASSINIT */
	wire  cmp15 = (n == 7'b1001001);
	wire  fb15  = ~(n[5] ^ n[6]);
	// Two NORs in series here, Init on the second, so Init feeds zeros in.
	wire  new15 = ~init & (cmp15 | fb15);

	// The comparator drives a "2","1" inverter pair before it leaves the block,
	// which is one target clock; that is the pulse the rest of the chip sees.
	always_ff @(posedge clk) begin
		clk64_en <= 1'b0;
		clk15_en <= 1'b0;
		if (phi2_en) begin
			m <= {m[3:0], new64};
			n <= {n[5:0], new15};
			clk64_en <= cmp64;
			clk15_en <= cmp15;
		end
	end

endmodule

`default_nettype wire
