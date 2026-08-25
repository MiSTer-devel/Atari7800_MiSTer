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

// POKEY keyboard scanner: the scan counter, the debounce comparator, KBCODE,
// the KEYBOARD CONTROL PLA and the two key interrupt sources.
//
// Source: references/Pokey/schematics/PokeyReSchem-13.pdf page 3, left side,
// with the PLA matrix off page 7 bottom left.
//
// ---------------------------------------------------------------------------
// The sheet, left to right
// ---------------------------------------------------------------------------
//
//   keybClk -> [Cell 3 | Cell 6 x6]  -> [Cell 4 x6] --Q--> [Cell 1 x8] -> D0-7r
//                 scan counter          comparator          KBCODE
//                      |                    |
//                      +--> K0-K5 pins      +--> C match line --> debComp
//                      |
//                      +--> 3 wired NOR columns: counter $00, $10, $30
//                             $00 -> Cell 2 -> KBCODE bit 7  (control)
//                             $10 -> Cell 2 -> kShift -> KBCODE bit 6, SKSTAT
//                             $30 -> static latch -> edge detect -> setBreak
//
//   iKR1, keyQ0, keyQ1, debComp -> KEYBOARD CONTROL PLA -> keyD0, keyD1,
//                                  nLdComp, nLdKbus
//
// The counter is a BINARY DECR row, so it counts DOWN, and three inverters sit
// between each stage's Q and its K pad, so the pins carry the complement of
// the code that lands in KBCODE. That is why the matrix in rtl/ps2_to_pokey.v
// indexes itself with ~KEYBOARD_SCAN.
//
// ---------------------------------------------------------------------------
// KEYBOARD CONTROL PLA, page 7 bottom left
// ---------------------------------------------------------------------------
// An NMOS NOR-NOR array. Every term row and every output column has a
// depletion pull up, so a term is the NOR of the input lines dotted on it and
// an output is the NOR of the terms dotted on it. Each input contributes a
// true column and, through an inverter, a complement column; a dot on the true
// column therefore requires that input to be 0.
//
// Reading the dots (columns 1-8 are iKR1, ~iKR1, keyQ0, ~keyQ0, keyQ1, ~keyQ1,
// debComp, ~debComp; then the four output columns):
//
//        iKR1 ~iKR1 keyQ0 ~keyQ0 keyQ1 ~keyQ1 debC ~debC | D0 D1 nLdC nLdK
//   T1     .    X     .     .      .     .      X    .   |  X  .   .    .
//   T2     .    .     X     .      X     .      .    X   |  X  .   .    .
//   T3     .    .     .     .      X     .      .    X   |  .  X   .    .
//   T4     .    .     X     .      X     .      .    .   |  .  X   .    .
//   T5     .    X     .     .      X     .      .    .   |  .  X   .    .
//   T6     .    X     X     .      .     X      X    .   |  .  X   .    X
//   T7     .    X     .     X      .     X      .    .   |  X  .   X    .
//   T8     X    .     X     .      .     .      .    X   |  X  .   .    .
//
// which is
//
//   T1 =  iKR1 & ~debComp                     T5 =  iKR1 & ~keyQ1
//   T2 = ~keyQ0 & ~keyQ1 &  debComp           T6 =  iKR1 & ~keyQ0 & keyQ1 & ~debComp
//   T3 = ~keyQ1 &  debComp                    T7 =  iKR1 &  keyQ0 &  keyQ1
//   T4 = ~keyQ0 & ~keyQ1                      T8 = ~iKR1 & ~keyQ0 &  debComp
//
//   keyD0 = ~(T1|T2|T7|T8)   keyD1 = ~(T3|T4|T5|T6)
//   nLdComp = ~T7            nLdKbus = ~T6
//
// keyQ1:keyQ0 is a four state scanner, held at 11 by kbScanDis:
//
//   11 idle          -- iKR1 -> load the comparator with this code, go to 10
//   10 waiting       -- the counter has to come all the way back round to the
//                       latched code with the key still down, then KBCODE is
//                       loaded and the state goes to 00
//   00 key is down   -- lkeyDown low; leaves when the counter reaches the code
//                       again and the key has gone
//   01 releasing     -- one more full sweep of confirmation, then back to 11
//
// debComp = SKCTL bit 0 AND "the counter does not match the latched code", so
// with debounce off debComp is stuck low and the machine needs iKR1 on two
// consecutive scan positions - which one key never gives. SKCTL $02 therefore
// reports no keys at all, and every Atari OS writes $03.

`default_nettype none

module pokey_kbd (
	input  wire       clk,

	input  wire       scan_en,      // keybClk, one pulse per scan step
	input  wire       kbd_enable,   // SKCTL bit 1
	input  wire       debounce_en,  // SKCTL bit 0
	input  wire       init,         // SKCTL[1:0] == 00

	input  wire       kr1,          // full decode sense, active low
	input  wire       kr2,          // CTRL / SHIFT / BREAK sense, active low

	input  wire       kbcode_rd,    // read strobe at address $09

	output wire [5:0] k,            // the scan lines
	output wire [7:0] kbcode,       // the stored code, for tracing and tests
	output wire [7:0] kbcode_bus,   // this sheet's pull on the read bus
	output wire       key_down,     // SKSTAT bit 2 source
	output wire       shift_key,    // SKSTAT bit 3 source
	output wire       break_key,    // IRQST bit 7 source
	output wire       other_key,    // IRQST bit 6 source
	output wire       kbd_overrun   // see below
);
	// kbScanDis holds the counter at zero and presets the scanner to idle.
	// The sheet drives it from SKCTL bit 1 alone; init is folded in because it
	// implies bit 1 is clear and keeps the parent's one reset in one place.
	wire kb_scan_dis = ~kbd_enable | init;

	// The sense pads invert. A closed key pulls the pin low, so these are the
	// active high "something is down at this scan position" signals.
	// Both pads also carry a feedback to positive power transistor. Its output
	// is disconnected on the die, so it is not modelled.
	wire ikr1 = ~kr1;
	wire ikr2 = ~kr2;

	// -----------------------------------------------------------------------
	// Scan counter: six BINARY DECR stages, Cell 3 and Cell 6 alternating.
	// Bit 0 is the top of the column, where keybClk arrives; each stage's T
	// output is the next stage's T input, so the row is a ripple down counter.
	// -----------------------------------------------------------------------
	wire [5:0] cnt, cnt_n;
	/* verilator lint_off UNUSED */
	wire [6:0] tchain;              // tchain[6] is bit 5's borrow, no consumer
	/* verilator lint_on UNUSED */

	assign tchain[0] = scan_en;

	genvar i;
	generate
		for (i = 0; i < 6; i = i + 1) begin : g_scan
			if (i % 2 == 0) begin : g_type1
				pokey_cell3 u (.clk, .t_in(tchain[i]), .r(kb_scan_dis),
					.q(cnt[i]), .q_n(cnt_n[i]), .t_out(tchain[i+1]));
			end else begin : g_type2
				pokey_cell6 u (.clk, .t_in(tchain[i]), .r(kb_scan_dis),
					.q(cnt[i]), .q_n(cnt_n[i]), .t_out(tchain[i+1]));
			end
		end
	endgenerate

	// Three inverters between each stage and its pad: the pins are the
	// complement of the internal code.
	assign k = cnt_n;

	// -----------------------------------------------------------------------
	// Comparator: Cell 4 holds the accepted code and compares it against the
	// live counter. Every bit that disagrees pulls the C line down, and C is
	// pulled up at the foot of the column, so C high means the whole word
	// matches. debComp is the NOR of ~SKCTL0 and C. That NOR carries no digit
	// and no X, so it is a plain static gate: the notation legend adds nothing
	// here and Atari7800_MiSTer-syc.16 stands as filed.
	// -----------------------------------------------------------------------
	wire kb_cmp_ld;
	wire [5:0] cmp_q, cmp_match;

	generate
		for (i = 0; i < 6; i = i + 1) begin : g_cmp
			pokey_cell4 u (.clk, .ld(kb_cmp_ld), .d(cnt[i]),
				.q(cmp_q[i]), .match(cmp_match[i]));
		end
	endgenerate

	wire c_match  = &cmp_match;
	wire deb_comp = ~(~debounce_en | c_match);

	// -----------------------------------------------------------------------
	// KEYBOARD CONTROL PLA and the two Cell 2 that hold the scanner state
	// -----------------------------------------------------------------------
	wire key_q0, key_q1;

	wire pt1 =  ikr1 & ~deb_comp;
	wire pt2 = ~key_q0 & ~key_q1 &  deb_comp;
	wire pt3 = ~key_q1 &  deb_comp;
	wire pt4 = ~key_q0 & ~key_q1;
	wire pt5 =  ikr1 & ~key_q1;
	wire pt6 =  ikr1 & ~key_q0 &  key_q1 & ~deb_comp;
	wire pt7 =  ikr1 &  key_q0 &  key_q1;
	wire pt8 = ~ikr1 & ~key_q0 &  deb_comp;

	wire key_d0     = ~(pt1 | pt2 | pt7 | pt8);
	wire key_d1     = ~(pt3 | pt4 | pt5 | pt6);
	wire n_ld_comp  = ~pt7;
	wire n_ld_kbus  = ~pt6;

	// keyQ0 and keyQ1 need nothing to move a cycle, even though the PLA runs
	// state -> PLA -> state, and the phase structure is not what saves it. The
	// sheet gives both Cell 2 their Ld through a plain unnumbered inverter off
	// keybClk and their nLd from keybClk directly, so they are transparent
	// while keybClk is LOW - the same half in which both load strobes are
	// active, since each is a NOR of its PLA output with keybClk. State update
	// and strobe share one window.
	//
	// So the self termination is the mechanism. nLdComp is ~(iKR1 & keyQ0 &
	// keyQ1); the moment the transparent latches take the new state, keyQ0
	// falls and the strobe it was holding ends. The pulse that loads Cell 4
	// therefore comes from the state as it was before the window, which is
	// exactly what `q` presents. The same argument covers nLdKbus, and it is
	// why the loop settles on the die instead of ringing.
	pokey_cell2 u_keyq0 (.clk, .ld(scan_en), .d(key_d0),
		.p(kb_scan_dis), .r(1'b0), .q(key_q0));
	pokey_cell2 u_keyq1 (.clk, .ld(scan_en), .d(key_d1),
		.p(kb_scan_dis), .r(1'b0), .q(key_q1));

	// Both load strobes are a NOR of their PLA output with keybClk - phase 2
	// couplers, no uncoupled input - so they fire on the same step that
	// produced them, in the keybClk LOW half, while the counter still shows the
	// code being captured. cnt reaches Cell 4 on that same edge, so the value
	// latched is the one the window opened on, not the stepped one. That is
	// what `scan_en` stands for here: the half step in which the strobes and
	// the three wired NOR columns below are live, not the keybClk high half.
	assign kb_cmp_ld = scan_en & ~n_ld_comp;

	// KBCODE loads one clk after the scan step, so the control and shift
	// latches and the Cell 4 comparator have settled - that is the die's
	// transparent window, per the house rule at the top of `pokey_cells.sv`.
	// The qualification has to be sampled WITH the window: `n_ld_kbus` comes
	// off the PLA, whose inputs move on the same step, so it has already
	// changed a cycle later.
	logic kbcode_ld;

	always_ff @(posedge clk)
		kbcode_ld <= scan_en & ~n_ld_kbus;

	// -----------------------------------------------------------------------
	// KBCODE: Cell 1 x8, loaded from the comparator latch, not from the
	// counter, so what the CPU reads is the debounced code.
	// -----------------------------------------------------------------------
	wire ctrl_q, shift_q;

	/* verilator lint_off PINCONNECTEMPTY */
	wire kbcode_oe;

	pokey_cell1 #(.WIDTH(8)) u_kbcode (
		.clk, .ld(kbcode_ld), .d({ctrl_q, shift_q, cmp_q}),
		.rd(kbcode_rd), .q(kbcode), .q_oe(kbcode_oe), .state());
	/* verilator lint_on PINCONNECTEMPTY */

	// Precharged bus: the cell only ever pulls it down.
	assign kbcode_bus = kbcode_oe ? kbcode : 8'hFF;

	// -----------------------------------------------------------------------
	// The KR2 group. Three wired NOR columns hang under the counter bus, each
	// pulled down by keybClk, by counter bits 0-3, and by bit 4 and bit 5 in
	// true or complement form, so a column is high only during a keybClk low
	// phase at one exact code:
	//
	//   counter $00 -> control    $10 -> shift    $30 -> break
	// -----------------------------------------------------------------------
	wire at_ctrl  = scan_en & (cnt == 6'h00);
	wire at_shift = scan_en & (cnt == 6'h10);
	wire at_break = scan_en & (cnt == 6'h30);

	// KBCODE can load in the same window when the accepted code is $00 or $10,
	// and on the die its transparent latch hands over the live sense line then.
	// Reproduced by the house rule rather than by a mux: these load on the scan
	// step and KBCODE reads them one clk later, by which point they have
	// settled. See `kbcode_ld` below.
	pokey_cell2 u_ctrl  (.clk, .ld(at_ctrl),  .d(ikr2),
		.p(1'b0), .r(1'b0), .q(ctrl_q));
	pokey_cell2 u_shift (.clk, .ld(at_shift), .d(ikr2),
		.p(1'b0), .r(1'b0), .q(shift_q));

	// preBreak is a two inverter loop closed by a pass gate on the $30 column,
	// so `break_n` holds the break sense between visits. An inverter sits
	// between the KR2 net and that pass gate, so the node carries the
	// COMPLEMENT: it is low while BREAK is held.
	//
	// The NOR after it is an EDGE DETECTOR, not a static gate. It takes
	// inverter 2's output on a numbered (phase coupled) input and the latch
	// node itself on an X (uncoupled) input, so it is
	//
	//     preBreak = ~(~node_previous_phase | node_now)
	//              =   node_previous_phase & ~node_now
	//
	// a falling edge of the node, which is the break key going down. Read as a
	// static gate it would be NOR(A, ~A) = 0, which is nonsense; Atari's legend
	// on references/Pokey/die/pokey.pdf page 46 is what decodes it. Traced at
	// 600 dpi on page 3, crop -x 4100 -y 12900 -W 2700 -H 1450.
	//
	//   BREAK pressed    node 1 -> 0    setBreak pulses for one clk
	//   BREAK held       node stays 0   setBreak stays low
	//   BREAK released   node 0 -> 1    nothing
	//
	// So setBreak is a PULSE, one per press, and not the level this used to
	// produce. It has to be: pokey_irq's IRQST bit is a NOR pair whose set
	// input is level sensitive, so a source held high is re-set every cycle and
	// survives the write of 0 to IRQEN that is POKEY's only acknowledge. A held
	// BREAK key would have pinned IRQ low with no way to clear it.
	// Atari7800_MiSTer-syc.17.
	//
	// The two inverters between preBreak and setBreak are a non-inverting
	// buffer. They are one coupler stage each, but nothing samples setBreak on
	// a phase boundary - pokey_irq looks at it every clk - so they are not
	// modelled as delay.
	logic break_n, break_n_q;

	always_ff @(posedge clk) begin
		break_n_q <= break_n;              // the NOR's phase coupled input
		if (at_break) break_n <= ~ikr2;    // the latch, transparent at $30
	end

	wire pre_break = break_n_q & ~break_n;

	// -----------------------------------------------------------------------
	// Outputs
	// -----------------------------------------------------------------------
	assign key_down  = ~key_q1;      // lkeyDown on the sheet
	assign shift_key = shift_q;
	assign break_key = pre_break;    // setBreak, a one clk pulse per press
	assign other_key = kbcode_ld;    // setKey is two inverters off kbcodeLd

	// The sheet forms the keyboard overrun on its right hand side, as a NAND
	// of setKey with the IRQST bit 6 node, which is not visible from here.
	// pokey_irq builds it; this port stays only because pokey.sv is not ours
	// to change.
	assign kbd_overrun = 1'b0;

endmodule

`default_nettype wire
