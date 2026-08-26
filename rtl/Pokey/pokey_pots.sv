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

// POKEY pot ports: eight pin latches, the scan counter, and the eight pot
// value registers.
//
// Source: PokeyReSchem-13.pdf page 4, left half.
//
// The pot pins are Schmitt trigger inputs, and Atari's PDF page 23 quantifies
// what the spec only calls a "trigger voltage": V_T+ 1.9-2.6 V, V_T- 1.0-2.1 V,
// 0.3 V hysteresis. It is modelled here as a plain inverting threshold with no
// hysteresis, and in this core that is exact rather than approximate: the 7800
// does not wire POKEY's pot pins at all - cart.sv leaves `POT_IN`
// unconnected - so the pins never see an analog ramp and there is no
// band for hysteresis to act in. If they are ever driven from a real RC ramp,
// this is the line to revisit.
//
//   pin --+-- dump transistor to ground
//         |
//         +-> Schmitt inverter -> Cell 26 -> NOR -> PotnLd -> Cell 1 Ld
//                                             ^
//                                          freeze
//
//   PotClk -> Cell 23 bit 0 -BOR-> bit 1 -> bit 2 -> bit 3
//             Cell 23 bit 4 -BOR-> bit 5 -> bit 6 -> bit 7
//
// The two nibbles do not abut. Bit 4 takes a wired NOR look ahead on its own
// pulled up line, NOR(nPotClk, Q0..Q3), which is the same term the
// ripple would produce and is what "no borrow delay" in the cell name means.
//
// Cell 23 is a down counter preset to all ones, so Q counts 0,1,2,... and it
// is Q that leaves the row: every bit's Q pin drives the D row of the eight
// Cell 1 pot registers, which is why POTn reads the number of scan
// ticks since POTGO.
//
// The scan end detector is a second pulled up line, tapping Q for
// bits 2,5,6,7 and Q for bits 0,1,3,4. It goes high for exactly one counter
// value: Q = 8'hE4 = 228, the documented end of the scan.
//
// POTn is loaded continuously while its pin is still low, and stops when the
// pin goes high; the value left behind is the count at that moment. A pin
// that is already high when POTGO is written never loads at all and POTn
// keeps the previous scan's value, which is what Altirra documents.
//
// ---------------------------------------------------------------------------
// Where the pin crosses into clocked logic, read off the phase marks
// ---------------------------------------------------------------------------
// The digit inside a gate is Atari's coupler-phase mark (pokey.pdf page 46).
// This half of the sheet is annotated throughout, and the marks say:
//
//   pad -> Schmitt -> Cell 26 -----------> NOR "1" -> PotnLd -> Cell 1 Ld
//                     samples on O2                  (both inputs O1 coupled,
//                     holds on O1                     no X on either)
//
// Cell 26 draws its two pass gates with the pin labels O1 and O2 rather than a
// digit (page 7): IN through the O2 gate onto the storage node, the two-inverter loop closed by the O1 gate.
// So the asynchronous pin is captured on O2 and is already held when the O1
// PotnLd gates look at it - the handoff is one phase wide and the RTL's
// pokey_cell26 reproduces it exactly. The eight PotnLd NORs all carry "1".
//
// Two more phase readings on this sheet, both one target clock finer than what
// is modelled below and neither observable at the pins:
//
//   scan stop latch   cross coupled NOR "1" / NOR "2", a real two-phase
//                     set-reset pair, not the static latch the flop below
//                     stands in for.
//   dump              freeze -> inv "1" -> NOR "2", so the pins are grounded
//                     one target clock after freeze rises rather than with it.
//
// ALLPOT is read out of the pot registers themselves. Eight of the 64 cells
// are Cell 8 rather than Cell 1 and they sit on the diagonal, bit n of POTn,
// which is the whole reason Cell 8 exists in this chip - nothing else
// instantiates it. Cell 8's second read port does not read the bit it holds:
// its pull down hangs off the Ld stem, so Addr8r puts one "still loading" bit
// per pot on the bus. ALLPOT reads $FF right after POTGO and $00 once the scan
// has ended.

`default_nettype none

module pokey_pots (
	input  wire       clk,
	input  wire       ph1_en,
	input  wire       ph2_en,

	input  wire       potgo,       // AddrBw, the write strobe at address $0B
	input  wire       init,        // SKCTL init: not wired into this sheet
	input  wire       fast_scan,   // SKCTL bit 2, "Skctls_2" on the sheet

	input  wire       clk15_en,    // keybClk, one pulse per scan line
	input  wire       clk64_en,    // not used, see the pot clock below

	input  wire [7:0] pot_in,      // the eight pins

	input  wire [7:0] pot_rd,      // Addr0r..Addr7r, one per pot column
	input  wire       allpot_rd,   // Addr8r, the Cell 8 second read port

	output wire [7:0] pot_bus,     // this sheet's pull on the read bus
	output wire [7:0] pot0, pot1, pot2, pot3, pot4, pot5, pot6, pot7,
	output wire [7:0] allpot,
	output wire       dump_n       // low while the dump transistors are on
);
	// -----------------------------------------------------------------------
	// POTGO
	//
	// The cross coupled NOR pair takes its feedback
	// after the two inverters that drive P, so it cannot hold: potPreset is a
	// pulse that follows POTGO.
	// -----------------------------------------------------------------------
	wire pot_preset = potgo;

	// -----------------------------------------------------------------------
	// Scan stop latch
	//
	// NOR pair. The lower gate's output is the freeze line that crosses the
	// whole sheet and gates all eight
	// PotnLd gates; the upper gate holds its complement, which is the half
	// kept here so a register that powers up clear means "scan stopped" and
	// the pins stay grounded until the first POTGO.
	// -----------------------------------------------------------------------
	logic scan_run;
	wire  freeze = ~scan_run;

	// -----------------------------------------------------------------------
	// Pot line latches
	//
	// The dump transistor gates hang off a common rail and their drains
	// are on the pins, so they are ahead of the Schmitt inverter: while they
	// are on the latch cannot see a high whatever the outside world drives.
	// Fast scan holds them off, which is why fast scan reads are inaccurate.
	// -----------------------------------------------------------------------
	wire       dump = freeze & ~fast_scan;   // a NOR on the sheet
	wire [7:0] pin  = pot_in & {8{~dump}};

	wire [7:0] line;                          // Cell 26 Q, the sampled pin

	genvar i, c, b;
	generate
		for (i = 0; i < 8; i = i + 1) begin : g_line
			pokey_cell26 u (
				.clk, .ph1_en, .ph2_en,
				.in  (~pin[i]),               // through the Schmitt inverter
				.q_n (line[i]));
		end
	endgenerate

	// The eight NOR gates in the PotnLd row.
	wire [7:0] pot_ld = ~(line | {8{freeze}});

	// -----------------------------------------------------------------------
	// Scan counter: eight Cell 23 stages
	//
	// nPotClk = NOR(keybClk, Skctls_2), so CR is the inverse of that: one
	// pulse per scan line normally, and held asserted in fast scan, where a
	// two phase toggle cell steps once per machine cycle. Altirra measures the
	// same rates, 1/114 cycles and 1/cycle.
	// -----------------------------------------------------------------------

	wire [7:0] cnt_q, cnt_q_n, cnt_bor, cnt_cr;

	assign cnt_cr[0]   = pot_clk;
	assign cnt_cr[3:1] = cnt_bor[2:0];
	assign cnt_cr[4]   = pot_clk & ~cnt_q[0] & ~cnt_q[1]     // look-ahead line
	                             & ~cnt_q[2] & ~cnt_q[3];
	assign cnt_cr[7:5] = cnt_bor[6:4];

	generate
		for (i = 0; i < 8; i = i + 1) begin : g_count
			pokey_cell23 u (
				.clk,
				.p    (pot_preset),
				.cr   (cnt_cr[i]),
				.q    (cnt_q[i]),
				.q_n  (cnt_q_n[i]),
				.bor  (cnt_bor[i]),
				.bor_n());
		end
	endgenerate

	// The 228 detector. Written as the wired NOR the sheet draws
	// rather than as a compare, so the taps can be checked against it.
	wire scan_end = ~(cnt_q_n[0] | cnt_q_n[1] | cnt_q  [2] | cnt_q_n[3] |
	                  cnt_q_n[4] | cnt_q  [5] | cnt_q  [6] | cnt_q  [7]);

	wire pot_clk = fast_scan ? ph2_en : clk15_en;

	always_ff @(posedge clk)
		if (pot_preset)
			scan_run <= 1'b1;
		else if (scan_end)
			scan_run <= 1'b0;

	// KNOWN DIVERGENCE, not fixed. Real hardware stops at 228 on a slow scan
	// and **229** on a fast one: ASAP's pot test (pokey_pot.asx) saturates at
	// $E5 in fast mode and 228 in slow, and Altirra's pokey.cpp models the same
	// split. This stops at 228
	// in both, because the detector on the sheet is a plain 228 tap set either
	// way and nothing in the drawn freeze path distinguishes the modes.
	//
	// Two candidate mechanisms, neither traced:
	//   - the value latch is still transparent when the fast counter takes its
	//     next step, which a slow scan cannot show because the counter has not
	//     moved between the detector firing and the latch closing;
	//   - the freeze gate chain has a real propagation delay that crosses one
	//     counter step at a machine-cycle rate and not at a scan-line rate.
	//
	// Only a delay measured in TIME can produce the split at
	// all - one measured in counter steps gives 229 in both modes, because the
	// slow counter eventually takes that step too. And the time it needs is a
	// specific number:
	//
	//   freeze reaching the PotnLd gates one target clock late  -> 228, 228
	//   two target clocks late                                  -> 229, 228
	//
	// so reproducing the hardware means asserting that the freeze line carries
	// exactly two ("2","1") coupler pairs on its way to those gates. That is an
	// ordinary amount of delay for this chip - page 6's receive sequencer has
	// the same two pairs - but nothing has been read off the drawing to say so,
	// and picking two because one does not land is fitting to the answer.
	//
	// So this is where it stays until someone reads the coupler marks on the
	// freeze line at the PotnLd row, on Cwik page 4 or Atari's page 38.
	//
	// The same sheet has a second untraced hardware behaviour: reading POTn on
	// the cycle the counter increments returns count & (count + 1), from the
	// bits that fall being faster than the bits that rise (Altirra's
	// pokey.cpp). That one is analog and has no structure to copy.
	//
	// Exposure for both is nil: the 7800 leaves POKEY's pot pins unconnected.

	// -----------------------------------------------------------------------
	// Pot value registers: eight columns of eight Cell 1 latches
	// -----------------------------------------------------------------------
	// Eight columns of eight, one column per pot, drawn as a single block on
	// the sheet: Rd across the top from Addr0r..Addr7r, Ld and Ld along the
	// bottom from PotnLd, and the counter's Q on the D row down the right.
	// The cell on each column's own bit is a Cell 8; the rest are Cell 1. The
	// Cell 8's Rd pin sits on its own column's strobe like every other cell in
	// the column: Addr0r lands on both the Cell 1 beside it and the Cell 8.
	wire [7:0] pot [8];
	wire [7:0] cell_q  [8];          // what each cell puts on its bit line
	wire [7:0] cell_oe [8];

	generate
		for (c = 0; c < 8; c = c + 1) begin : g_pot
			for (b = 0; b < 8; b = b + 1) begin : g_bit
				if (b == c) begin : g_allpot
					pokey_cell8 u (
						.clk,
						.ld   (pot_ld[c]),
						.d    (cnt_q_n[b]),
						.rd   (pot_rd[c]),
						.rd_k (allpot_rd),
						.q    (cell_q[c][b]),
						.q_oe (cell_oe[c][b]),
						.state(pot[c][b]));
				end else begin : g_value
					pokey_cell1 u (
						.clk,
						.ld   (pot_ld[c]),
						.d    (cnt_q_n[b]),
						.rd   (pot_rd[c]),
						.q    (cell_q[c][b]),
						.q_oe (cell_oe[c][b]),
						.state(pot[c][b]));
				end
			end
		end
	endgenerate

	// All 64 cells hang on the same eight precharged lines, so a bit line is
	// the AND of what every column does to it and reads one when nobody pulls.
	generate
		for (b = 0; b < 8; b = b + 1) begin : g_bus
			wire [7:0] pull;
			for (c = 0; c < 8; c = c + 1) begin : g_col
				assign pull[c] = cell_q[c][b] | ~cell_oe[c][b];
			end
			assign pot_bus[b] = &pull;
		end
	endgenerate

	assign pot0 = pot[0];
	assign pot1 = pot[1];
	assign pot2 = pot[2];
	assign pot3 = pot[3];
	assign pot4 = pot[4];
	assign pot5 = pot[5];
	assign pot6 = pot[6];
	assign pot7 = pot[7];

	// What the Cell 8 RdK pull downs put on the bus under Addr8r, exposed
	// directly so a caller can see it without driving a read strobe.
	assign allpot = pot_ld;
	assign dump_n = ~dump;

	// Init reaches the dividers on this sheet, never the pot block, so a scan
	// in progress rides through it. clk64_en is not a pot clock either; both
	// ports are kept because pokey.sv wires them. Bit 4's Q pin is the one
	// Cell 23 output the sheet leaves unconnected, and the borrows out of
	// bit 3 and bit 7 go nowhere: the nibble carry is the look ahead above
	// and bit 7 ends the chain.
`ifdef VERILATOR
	// A sink for Verilator's UNUSED check; Quartus warns 10036 on it instead.
	wire _unused = &{1'b0, init, clk64_en, cnt_q[4], cnt_bor[7], cnt_bor[3]};
`endif

endmodule

`default_nettype wire
