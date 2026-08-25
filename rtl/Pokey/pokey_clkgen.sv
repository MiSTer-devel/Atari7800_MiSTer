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

// POKEY clock phase generation.
//
// Source: references/Pokey/schematics/PokeyReSchem-13.pdf page 7, bottom centre.
// On the die the PHI2 pad feeds an inverter chain that produces two
// non-overlapping phases o1 and o2, plus the buffered Phi2B and the PreS01
// precharge strobe that the read bus uses.
//
//        PHI2 --->o->--+--->o->--- o1
//                      |
//                      +--->o->--- o2      (delayed so the two never overlap)
//                      |
//                      +--------- Phi2B
//                      +--------- PreS01
//
// ---------------------------------------------------------------------------
// Departure from the pin contract, deliberate
// ---------------------------------------------------------------------------
// The real part has one PHI2 pin and derives both phases inside. This module
// takes both phase enables from outside instead. The reason is that the host
// already has them: rtl/cart.sv runs POKEY from pclk0 and pclk1, which are the
// two halves of the same 1.79 MHz clock the real PHI2 pin carries. Rebuilding
// o1 and o2 from a single strobe would mean guessing a delay in clk_sys
// cycles and would add skew that the real chip does not have.
//
// The contract this module enforces instead: ph1_en and ph2_en are single
// clk_sys cycle strobes and are never high together. Everything downstream
// relies on that, so it is checked here rather than assumed.

`default_nettype none

module pokey_clkgen (
	input  wire clk,

	// The two phases of the target 1.79 MHz clock, one clk cycle each.
	input  wire ph1_en,
	input  wire ph2_en,

	output wire phi1,      // o1
	output wire phi2,      // o2
	output wire phi2b,     // buffered PHI2, high across the o2 half
	output wire pre_s01    // read bus precharge, ahead of a o1 read
);
	assign phi1  = ph1_en;
	assign phi2  = ph2_en;

	// Phi2B follows the o2 half of the target clock rather than pulsing, so it
	// is held between the o2 strobe and the next o1 strobe.
	logic phi2b_q;

	always_ff @(posedge clk)
		if (ph2_en)
			phi2b_q <= 1'b1;
		else if (ph1_en)
			phi2b_q <= 1'b0;

	assign phi2b = phi2b_q;

	// The read bus is precharged before it is driven, and PreS01 is what does
	// it. Page 7 takes it from an inverter off the raw PHI2 net ahead of the
	// first inverter, so it is a LEVEL covering the whole o1 half, not a pulse,
	// and `pokey_bus.sv` says the same. All three consumers are levels: the
	// A0-A3 pass gates, the pull-down that grounds every read strobe, and the
	// Cell 7 DISABLE. So the data pads drive for the o2 half and no longer.
	//
	// It was a one-cycle strobe until now, which left the pads live for the
	// rest of o1 as well. Nothing observable followed - reads have no side
	// effects anywhere - but it is not what the sheet draws, and three
	// testbenches had quietly come to depend on it by sampling the pads in the
	// gap between the phases. Their `bus_read` now samples after the o2 strobe,
	// which is where the die drives.
	logic pre_s01_q;

	always_ff @(posedge clk)
		if (ph1_en)
			pre_s01_q <= 1'b1;
		else if (ph2_en)
			pre_s01_q <= 1'b0;

	assign pre_s01 = pre_s01_q | ph1_en;

`ifdef SIMULATION
	// The whole library assumes the phases are exclusive. Say so loudly if a
	// caller ever wires them from the same strobe.
	always_ff @(posedge clk)
		if (ph1_en && ph2_en)
			$fatal(1, "pokey_clkgen: ph1_en and ph2_en high in the same clk cycle");
`endif

endmodule

`default_nettype wire
