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

// POKEY standard cell library.
//
// Source: references/Pokey/schematics/PokeyReSchem-13.pdf page 7, Jorge Cwik's
// reconstruction from the die. Atari's own sheets 35-41 in
// references/Pokey/die/pokey.pdf use the same cell numbers, so the two can be
// diffed against each other and against this file.
//
// ---------------------------------------------------------------------------
// Atari's notation, from the original legend
// ---------------------------------------------------------------------------
// `references/Pokey/die/pokey.pdf` page 46 is Atari's own cell library sheet -
// the one Cwik redrew as his page 7 - and it carries the legend that decodes
// two marks this project spent a long time guessing at:
//
//     A. X INDICATES NO COUPLER ON THAT INPUT.
//     B. NUMBER INDICATES CLOCK PHASE OF INPUT COUPLERS.
//
// So an **X is not a no-connect and not an option**. It marks an input that
// bypasses the clock coupler: a direct, static, asynchronous input to a gate
// whose other inputs are phase clocked. The connection is real. Options are
// drawn separately and labelled in words - "OPTION 1 (ADD THIS LINE)" on Cell
// 22, "(Option 1 Remove this line)" on Cell 24.
//
// And the **number inside a gate is the clock phase of its input couplers**,
// not an instance number. An unnumbered gate is a plain static gate. Atari's
// own sheets write these as 1A/2A/1G/2G; Cwik kept the digit and dropped the
// letter, which is why they read as instance numbers. Two consequences worth
// keeping in mind while reading any sheet:
//
//   - the phase structure of the whole design is written on the drawing, and
//     counting numbered gates along a path counts coupler stages;
//   - a gate with one coupled and one direct input is doing something in time,
//     not just in logic. NOR(A coupled, ~A direct) is not a constant - it is an
//     edge detector, because the coupled input still holds the previous phase.
//
// ---------------------------------------------------------------------------
// Transparent latches
// ---------------------------------------------------------------------------
// Several cells here are latches on the die: while the enable is high the
// output follows the input, and the logic they feed settles inside that same
// window. Cell 2 is the clearest case; the shift cells' SET and R are another.
//
// `clk` runs many cycles per clock enable, and that is what makes this easy. A
// latch is a plain register clocked on `clk` and gated by the enable as a
// LEVEL:
//
//     always_ff @(posedge clk) if (ld) node <= d;
//
// While `ld` is high the register re-samples every cycle, so the output follows
// the input the way the die's latch does, and it keeps the last value when the
// level falls. Nothing is combinational from `d` to `q`, so a feedback path is
// not a loop: it iterates once per `clk` cycle and settles inside the window,
// which is how it settles on the die through gate delay.
//
// Two things follow, and both are requirements on the parent rather than on the
// cell:
//
//   1. `ld` must be the LEVEL the sheet draws, not a one-cycle strobe. A
//      one-cycle enable turns the cell back into an ordinary flop and hands
//      every same-window consumer the previous window's value. That was the
//      poly5 bug.
//   2. A consumer inside the window reads the cell one `clk` cycle after the
//      window opens. That is the die's propagation delay, and it stays well
//      inside the same target phase, so nothing at the pins moves.
//
// One trap, found the hard way. A register gated by the level captures its
// input as it stood during that cycle - the pre-update value - which is the
// right one. Anything else the die reads inside the same window has to be
// sampled WITH the window for the same reason, because the enable that opens
// the window is often the same enable that clocks what the consumer reads.
//
// And one hard limit, measured rather than reasoned. A window may cover at most
// ONE phase of the target clock, and a cell whose own output feeds back
// INVERTED - a toggle, `d = ~q` - must keep a single-cycle enable whatever the
// sheet draws. A settling loop converges inside a wide window; an inverting one
// never does. Six timer pulses through a toggle with a phase-wide enable gave
// eighteen flips and collapsed the divide by two. The die does not have this
// problem because a toggle there is master-slave, two latches on opposite
// non-overlapping phases, so it flips once per target clock however long each
// phase lasts. A single-cycle enable is that master-slave pair collapsed, and
// is the right model for those sites.
//
// Cell numbers are Atari's and are kept verbatim so a reviewer can hold the
// PDF next to the code.
//
// ---------------------------------------------------------------------------
// How the dynamic circuit maps onto an FPGA
// ---------------------------------------------------------------------------
// POKEY is two-phase dynamic logic. A storage node is a pass transistor into a
// pair of inverters, with a second pass transistor on the complement of the
// same control feeding the node back on itself:
//
//        D --| Ld |--o--->o->--->o->--+     write when Ld
//                    |                |
//                    +---| Ld_n |-----+     hold when Ld is low
//
// The complement input exists only to hold the node. A flip-flop already holds
// when its enable is low, so every `Ld_n`, `WR_n`, `CR_n` and `REC` that serves
// only as a keeper collapses into the enable being low. Where a complement
// does real work (the shift cells recirculate rather than merely hold) it is
// kept as a port.
//
// Every cell here is `always_ff @(posedge clk)` with an enable. No latches, no
// negedge, no generated clocks. Phase separation comes from ph1_en/ph2_en never
// being high in the same clk cycle - see pokey_clkgen.sv.
//
// ---------------------------------------------------------------------------
// Which of these are actually instantiated
// ---------------------------------------------------------------------------
// All 21, each where its sheet instantiates it:
//
//   1   pots, kbd, serial      12  audio, chan, irq
//   2   chan, kbd, serial      15  serial
//   3   kbd                    16  serial
//   4   kbd                    17  serial
//   5   audio, chan, irq,      20  chan
//       serial                 21  bus
//   6   kbd                    22  bus
//   7   bus                    23  pots
//   8   pots, the ALLPOT       24  chan
//       diagonal - its only    25  serial
//       use in the chip        26  pots
//   9   irq, poly
//   11  chan
//
// This block used to say eight were wired and the rest were carried as an
// unused transcription, which was true when it was written and had gone stale.
// Numbers 10, 13, 14, 18 and 19 are absent from Atari's own library, so they
// are not gaps here either.
//
// Read polarity is not uniform and matters. The internal read bus is precharged
// and drivers pull it down, so the cells that drive it invert. Cell 1, Cell 8,
// Cell 9 and Cell 12 output the complement; Cell 5 does not. That asymmetry is
// on the schematic, not a transcription slip.

`default_nettype none

// ---------------------------------------------------------------------------
// Cell 1 - TRI STATE LATCH
// ---------------------------------------------------------------------------
// Stores D on Ld and drives the read bus while Rd is high.
//
// The read tap sits between the two inverters, so it carries the complement of
// what was stored - but it drives the *gate* of a grounded pull down, and the
// Rd pass gate connects that pull down to the pin. The driver inverts again,
// so a precharged bus sees the stored value, and the pin is labelled plain Q
// on the drawing. Cell 9's pin is drawn with an overbar precisely because
// there the input reaches the gate directly and is inverted only once.
//
// An earlier version of this cell counted the tap and not the driver and
// handed out the complement, which would have inverted every byte read through
// it. Nothing was wired to it at the time.
module pokey_cell1 #(
	parameter int WIDTH = 1
) (
	input  wire              clk,
	input  wire              ld,      // load enable, already phase qualified
	input  wire [WIDTH-1:0]  d,
	input  wire              rd,      // read strobe
	output wire [WIDTH-1:0]  q,       // stored value, valid while rd
	output wire              q_oe,    // high while this cell drives the bus
	output wire [WIDTH-1:0]  state    // stored value, for use inside the chip
);
	logic [WIDTH-1:0] hold;

	always_ff @(posedge clk)
		if (ld)
			hold <= d;

	assign state = hold;
	assign q     = hold;
	assign q_oe  = rd;
endmodule

// ---------------------------------------------------------------------------
// Cell 2 - D LATCH FLIP FLOP
// ---------------------------------------------------------------------------
// Two NOR gates around the storage node give an asynchronous preset and reset:
//   nor2 = ~(node | p);  q = ~(nor2 | r)  =>  q = (node | p) & ~r
// so p forces q high, r forces q low, and r wins. Both are drawn with an X,
// which per Atari's own notation legend means the input has NO CLOCK COUPLER -
// it is a direct asynchronous input to an otherwise phase-clocked gate. The
// connection is real. See the note on the legend at the top of this file.
//
// The hold path recirculates Q, not the storage node. So a pulse on P or R is
// written back into the node on the next hold phase and survives the input
// going away - preset and reset are captured here, not transient.
//
// This is a LATCH: while Ld is high the cell is transparent and Q follows D.
// Three sheets have a consumer inside that same window - the poly5 gate on page
// 5, the scanner state bits on page 3, the bit rate toggles on page 6 - and a
// plain enabled flop hands every one of them the previous window's value.
//
// Drive `ld` with the level the sheet draws and the cell is that latch: see
// "Transparent latches" at the top of this file.
module pokey_cell2 (
	input  wire clk,
	input  wire ld,   // level, not a strobe
	input  wire d,
	input  wire p,    // preset, level
	input  wire r,    // reset, level, dominates
	output wire q
);
	logic node;

	always_ff @(posedge clk)
		if (ld)
			node <= d;                 // transparent while ld is high
		else
			node <= (node | p) & ~r;   // the hold path latches P and R

	assign q = (node | p) & ~r;
endmodule

// ---------------------------------------------------------------------------
// Cell 3 / Cell 6 - BINARY DECR. TYPE 1 and TYPE 2
// ---------------------------------------------------------------------------
// One stage of the keyboard scan counter. Structurally the two cells are
// identical, and identical to the counting core of Cell 20 and Cell 24: the
// storage node goes through a NOR and two inverters, with the third stage fed
// back through the T pass gate to toggle and the second through the T pass
// gate to hold. Only the phase labels differ, and they are swapped:
//
//     Cell 3 (TYPE 1)   takes T1, emits T2
//     Cell 6 (TYPE 2)   takes T2, emits T1
//
// so the stages alternate down the chain and a borrow cannot race through the
// whole counter inside one phase.
//
// There is no borrow input pin. The incoming phase IS the borrow - the cells
// abut with one stage's T output against the next stage's T input, exactly as
// the AUDF row abuts BOR to CR. The ripple output is one gate:
//
//     T_out = NOR(T_in, Q) = T_in & ~Q
//
// which carries the clock term, so it is a pulse and not a level. An earlier
// version split this into a separate `t1` enable and a `borrow_in`, and
// computed `borrow_in & ~state`, which dropped the clock term.
//
// The reset line reaches the NOR's second input through a crossed out
// connection, so the base cell has no reset at all and that NOR degenerates to
// an inverter. `r` is kept as the optioned form.
module pokey_cell3 (        // TYPE 1: clocked by T1, emits T2
	input  wire clk,
	input  wire t_in,       // borrow from the stage below, on this cell's phase
	input  wire r,          // reset, level, optioned in
	output wire q,
	output wire q_n,
	output wire t_out       // borrow to the stage above, on the other phase
);
	logic state;

	always_ff @(posedge clk)
		if (r)
			state <= 1'b0;
		else if (t_in)
			state <= ~state;

	assign q     = state;
	assign q_n   = ~state;
	assign t_out = t_in & ~state;   // 0 -> 1 wraps, so borrow out
endmodule

module pokey_cell6 (        // TYPE 2: clocked by T2, emits T1
	input  wire clk,
	input  wire t_in,
	input  wire r,
	output wire q,
	output wire q_n,
	output wire t_out
);
	logic state;

	always_ff @(posedge clk)
		if (r)
			state <= 1'b0;
		else if (t_in)
			state <= ~state;

	assign q     = state;
	assign q_n   = ~state;
	assign t_out = t_in & ~state;
endmodule

// ---------------------------------------------------------------------------
// Cell 4 - D LATCH WITH COMPARATOR
// ---------------------------------------------------------------------------
// A latch bit that compares its own D pin against what it stored, and reports
// on a shared match line.
//
// An XNOR takes D tapped ahead of the Ld pass gate and the node between the
// inverters, and its output gates a transistor sitting on the vertical C line.
// So each bit pulls C low when D differs from the stored bit, and C - pulled
// up elsewhere - is high only when every bit agrees. C is a wired-AND of
// "equal" across the word, active high, and is not a per-bit input.
//
// A wired-AND does not exist in the fabric, so the parent ANDs `match` across
// the bits. What this cell must not do is take the compared value as a
// separate input: it is D, the same pin the latch loads from, which is how the
// keyboard scanner compares the live scan code against the debounced one.
module pokey_cell4 (
	input  wire clk,
	input  wire ld,
	input  wire d,       // loaded on ld, and compared against the stored bit
	output wire q,
	output wire match    // parent ANDs these to form the C match line
);
	logic hold;

	always_ff @(posedge clk)
		if (ld)
			hold <= d;

	assign q     = hold;
	assign match = (d == hold);
endmodule

// ---------------------------------------------------------------------------
// Cell 5 - D LATCH, Q OUTPUT
// ---------------------------------------------------------------------------
// The plain register bit, and the one used for nearly every control register.
// Read tap is after the second inverter, so this one does not invert.
module pokey_cell5 #(
	parameter int WIDTH = 1
) (
	input  wire             clk,
	input  wire             ld,
	input  wire [WIDTH-1:0] d,
	output wire [WIDTH-1:0] q
);
	logic [WIDTH-1:0] hold;

	always_ff @(posedge clk)
		if (ld)
			hold <= d;

	assign q = hold;
endmodule

// ---------------------------------------------------------------------------
// Cell 7 - TRI-STATE BUS DRIVER
// ---------------------------------------------------------------------------
// The pad driver. Two NOR pairs drive a separate pull-up and pull-down so the
// pad can be released. Split into value and enable per AGENTS.md rather than
// modelled with a `z`, so the parent decides what an undriven pad reads as.
module pokey_cell7 #(
	parameter int WIDTH = 1
) (
	input  wire [WIDTH-1:0] in,
	input  wire             disable_n,   // the schematic's DISABLE, ACTIVE HIGH
	                                     // despite the _n; oe is its complement
	output wire [WIDTH-1:0] out,
	output wire             oe
);
	assign out = in;
	assign oe  = ~disable_n;
endmodule

// ---------------------------------------------------------------------------
// Cell 8 - TRI STATE LATCH with a second read port
// ---------------------------------------------------------------------------
// Cell 1 plus a second read strobe, RdK - but the second port does not read
// the stored data. Its pull down is gated off the Ld stem, which is what the
// drawing's title "(Rd Load Signal)" means: under RdK the cell puts Ld itself
// on the bus, the "this bit was written" status, not the code it holds.
//
// KBCODE uses it, so the CPU reads the key code through Rd and the key-down
// status through RdK.
//
// An earlier version modelled RdK as a duplicate of Rd over the same data.
module pokey_cell8 #(
	parameter int WIDTH = 1
) (
	input  wire             clk,
	input  wire             ld,
	input  wire [WIDTH-1:0] d,
	input  wire             rd,
	input  wire             rd_k,
	output wire [WIDTH-1:0] q,       // stored value under rd, Ld under rd_k
	output wire             q_oe,
	output wire [WIDTH-1:0] state
);
	logic [WIDTH-1:0] hold;

	always_ff @(posedge clk)
		if (ld)
			hold <= d;

	assign state = hold;
	assign q     = rd_k ? {WIDTH{ld}} : hold;
	assign q_oe  = rd | rd_k;
endmodule

// ---------------------------------------------------------------------------
// Cell 9 - TRI STATE DRIVER
// ---------------------------------------------------------------------------
// An open drain pulldown onto the precharged status bus: in and rd together
// pull the line low, otherwise it floats high. Status bits are read this way,
// which is why so many of them are documented as active low.
module pokey_cell9 (
	input  wire in,
	input  wire rd,
	output wire q_n,
	output wire q_oe
);
	assign q_n  = ~in;
	assign q_oe = rd;
endmodule

// ---------------------------------------------------------------------------
// Cell 11 - SOUND DAC
// ---------------------------------------------------------------------------
// Four binary weighted transistors steer current into the shared AUD node.
// From the CO12294 spec: "A logic zero audio input to this volume circuit
// always gives an open circuit (zero current) output", and volume-only mode
// forces the audio input true. So the channel contributes its volume when the
// waveform is high or when vol_only is set, and contributes nothing otherwise.
//
// This outputs the DAC's 4 bit drive, not a voltage. Summing the four channels
// is the AUD node's job - see pokey_mixer.sv.
module pokey_cell11 (
	input  wire [3:0] vol,      // AUDCx bits 3:0
	input  wire       vol_only, // AUDCx bit 4
	input  wire       in,       // channel waveform
	output wire [3:0] out
);
	assign out = (in | vol_only) ? vol : 4'd0;
endmodule

// ---------------------------------------------------------------------------
// Cell 12 - D LATCH, Q OUTPUT (complement)
// ---------------------------------------------------------------------------
// Cell 5 with the read tap moved one inverter earlier. Used where the consumer
// wants the complement, notably IRQEN feeding the IRQ tree.
module pokey_cell12 #(
	parameter int WIDTH = 1
) (
	input  wire             clk,
	input  wire             ld,
	input  wire [WIDTH-1:0] d,
	output wire [WIDTH-1:0] q_n
);
	logic [WIDTH-1:0] hold;

	always_ff @(posedge clk)
		if (ld)
			hold <= d;

	assign q_n = ~hold;
endmodule

// ---------------------------------------------------------------------------
// Cell 15 / 16 / 17 / 25 - master-slave shift cells
// ---------------------------------------------------------------------------
// The serial shift register bit. Master takes `d` on shift, or `in` on load;
// `transfer` moves master to slave; `rec` recirculates slave back into master
// so the register can hold across a phase without a new shift.
//
//   d ---|shift|--+                      +--|transfer|--+
//   in --|load |--+--> master --inv-->----+              +--> slave --> q
//                 ^                                      |
//                 +----------------|rec|------------------+
//
// The four variants differ only at the master node: 25 is the plain cell, 15
// adds a parallel load, 16 adds a reset to ground, 17 adds a set to Vdd.
module pokey_cell25 (
	input  wire clk,
	input  wire shift,
	input  wire d,
	input  wire transfer,
	input  wire rec,
	output wire q
);
	logic master, slave;

	always_ff @(posedge clk) begin
		if (shift)
			master <= d;
		else if (rec)
			master <= slave;

		if (transfer)
			slave <= master;
	end

	assign q = slave;
endmodule

module pokey_cell15 (
	input  wire clk,
	input  wire shift,
	input  wire d,
	input  wire load,
	input  wire in,
	input  wire transfer,
	input  wire rec,
	output wire q
);
	logic master, slave;

	always_ff @(posedge clk) begin
		if (load)                 // parallel load wins over the shift path
			master <= in;
		else if (shift)
			master <= d;
		else if (rec)
			master <= slave;

		if (transfer)
			slave <= master;
	end

	assign q = slave;
endmodule

module pokey_cell16 (
	input  wire clk,
	input  wire shift,
	input  wire d,
	input  wire r,        // pulls the master node to ground
	input  wire transfer,
	input  wire rec,
	output wire q
);
	logic master, slave;

	always_ff @(posedge clk) begin
		if (r)
			master <= 1'b0;
		else if (shift)
			master <= d;
		else if (rec)
			master <= slave;

		if (transfer)
			slave <= master;
	end

	assign q = slave;
endmodule

module pokey_cell17 (
	input  wire clk,
	input  wire shift,
	input  wire d,
	input  wire set,      // pulls the master node to Vdd
	input  wire transfer,
	input  wire rec,
	output wire q
);
	logic master, slave;

	always_ff @(posedge clk) begin
		if (set)
			master <= 1'b1;
		else if (shift)
			master <= d;
		else if (rec)
			master <= slave;

		if (transfer)
			slave <= master;
	end

	assign q = slave;
endmodule

// ---------------------------------------------------------------------------
// Cell 20 - BINARY DECRE WITH LOAD
// Cell 24 - BINARY DECRE WITH LOAD, NO BORROW DELAY
// ---------------------------------------------------------------------------
// One bit of an AUDF style counter. Read off PokeyReSchem-13.pdf page 7; both
// drawings were traced wire by wire, including gate input counts.
//
// Storage, identical in the two cells:
//
//     D --|WR|--> [inv inv, held closed by WR] --> reload
//                                                    |
//                                                  |Ld|
//                                                    v
//     count --> inv2 --> inv1 --> inv3 --> (no pin; see below)
//       ^                  |         |
//       +------|CR|--------+         |    two inversions: hold
//       +------|CR|------------------+    three inversions: toggle
//
// so `Ld` copies the written reload value into the count, and each `CR` pulse
// toggles the bit. CR is a clock, not a level. The cells abut with one cell's
// BOR driving the next cell's CR, which makes an AUDF counter a ripple clocked
// by borrows, not a synchronous chain with a shared enable.
//
// Neither cell has a Q pin. Its only outputs are the borrow pair, which is
// consistent with AUDF being write only - the count is never read back. `q` is
// exported here anyway so traces and tb_cells can see the count.
//
// Both cells compute the same borrow, as a NOR of three inputs:
//
//     BOR = NOR(Q, Ld, CR)  =  CR & ~Q & ~Ld
//
// Cell 24 reaches it in one gate, which is what "NO BORROW DELAY" means. Cell
// 20 takes three: the same NOR, then a NOR used as an inverter with its Ld
// input crossed out on the drawing, then NOR(Ld, BOR). Same function, longer
// path. Page 5 alternates the two along a row to balance the ripple, so in RTL
// they are one body under two names, kept apart so a reviewer can match the
// drawing box for box.
//
// The complement is where they genuinely differ:
//
//     Cell 20            BOR = ~BOR
//     Cell 24            BOR = NOR(BOR, Ld)
//     Cell 24 Option 1   BOR = ~BOR            "(Option 1: Remove this line)"
//
// Option 1 makes Cell 24's complement behave like Cell 20's during a load.
// Page 5 puts Option 1 at the top bit of every AUDF counter, which is the bit
// whose complement leaves the row.
//
// `~Ld` in the borrow is a guard, not arithmetic. On the die Ld comes back
// from the underflow through several gates and a delay element, so it lands
// well after the CR pulse that caused it and the two are never true together
// during a real borrow. It is written out because the drawing has three NOR
// inputs, and the parent keeps them apart in time the way the die does.
module pokey_cell20 (
	input  wire clk,
	input  wire wr,          // write the reload value
	input  wire d,
	input  wire ld,          // copy the reload value into the count
	input  wire cr,          // borrow from the bit below: one pulse, one toggle
	output wire q,
	output wire reload,
	output wire bor,         // drives the next bit's cr
	output wire bor_n
);
	logic reload_q, count_q;

	always_ff @(posedge clk) begin
		if (wr)
			reload_q <= d;

		if (ld)
			count_q <= reload_q;
		else if (cr)
			count_q <= ~count_q;
	end

	assign q      = count_q;
	assign reload = reload_q;
	assign bor    = cr & ~count_q & ~ld;
	assign bor_n  = ~bor;
endmodule

module pokey_cell24 #(
	parameter bit OPTION1 = 1'b0    // "Remove this line": drop Ld from BOR
) (
	input  wire clk,
	input  wire wr,
	input  wire d,
	input  wire ld,
	input  wire cr,
	output wire q,
	output wire reload,
	output wire bor,
	output wire bor_n
);
	logic reload_q, count_q;

	always_ff @(posedge clk) begin
		if (wr)
			reload_q <= d;

		if (ld)
			count_q <= reload_q;
		else if (cr)
			count_q <= ~count_q;
	end

	assign q      = count_q;
	assign reload = reload_q;
	assign bor    = cr & ~count_q & ~ld;
	assign bor_n  = OPTION1 ? ~bor : ~(bor | ld);
endmodule

// ---------------------------------------------------------------------------
// Cell 23 - BINARY DECRE, NO BORROW DELAY
// ---------------------------------------------------------------------------
// Cell 24's storage without the reload register: a preset input `P` in place
// of the written value, and a Q pin, since the pot counters are read back.
// The borrow loses the load term with it:
//
//     BOR = NOR(Q, CR) = CR & ~Q
//
// `P` sets the bit, it does not clear it. The chain is node -> inv -> NOR with
// P on its second input -> Q -> inv -> Q, so P high forces Q low and Q high,
// and the hold gate latches it. Read independently twice off the same drawing.
// P does not appear in the borrow: this is Cell 24's expression with the load
// term absent, not with P substituted for it.
module pokey_cell23 (
	input  wire clk,
	input  wire p,          // preset, level
	input  wire cr,
	output wire q,
	output wire q_n,
	output wire bor,
	output wire bor_n
);
	logic count_q;

	always_ff @(posedge clk)
		if (p)
			count_q <= 1'b1;
		else if (cr)
			count_q <= ~count_q;

	assign q     = count_q;
	assign q_n   = ~count_q;
	assign bor   = cr & ~count_q;
	assign bor_n = ~bor;
endmodule

// ---------------------------------------------------------------------------
// Cell 22 - WRITE ADDR DRIVER
// ---------------------------------------------------------------------------
// Turns a decoded address into the write strobe pair the register sheets
// consume, qualified by `c`. Both NORs take ~in on the upper input and c on
// the lower, and the middle NOR drives the output NOR's pull up, so
// `q = in & ~c` - the same gate as Cell 21, which is why the two agree.
// The schematic's "OPTION 1 (ADD THIS LINE)" makes the complement
// `~(q | c)` instead of `~q`; both are provided so the choice is a change at
// the instance rather than in this cell.
module pokey_cell22 #(
	parameter bit OPTION1 = 1'b0    // "add this line": gate the complement too
) (
	input  wire in,
	input  wire c,
	output wire q,
	output wire q_n
);
	assign q   = in & ~c;
	assign q_n = OPTION1 ? ~(q | c) : ~q;
endmodule

// ---------------------------------------------------------------------------
// Cell 21 - single output write strobe driver
// ---------------------------------------------------------------------------
// The single output sibling of Cell 22. Page 2 drives every write address
// through a Cell 22, which emits a level and its complement, except addresses
// 9 and A which use a Cell 21 with a Q and no complement. Those two are STIMER
// and SKRES, write-only registers whose consumers need only the one polarity.
//
//   in ---+---->o---+
//         |         +--NOR--> q
//   c  -------------+
//
// Read off PokeyReSchem-13.pdf page 7. In v1.0 this cell was drawn ambiguously
// enough that an earlier pass here read it as a rising edge one-shot; v1.3
// shows plainly that `in` crosses the `c` line without connecting and that
// there is no delay element, so there is no edge detector. It is combinational.
//
// SETTLED at 1200 dpi: `q = in & ~c`. The wire that could not be traced before
// is the cell's bounding box edge. The NOR's second input comes off a junction
// dot on the C line, and the wire from IN goes over the top and lands on the
// NOR *body*, which is the pull up drive rather than a logic input - the same
// convention Cell 7 uses. So q = NOR(~in, c) = in & ~c. Cell 22 is the same
// gate with a super buffered copy, so the two still agree with each other.
//
// Numbers 10, 13, 14, 18 and 19 are absent from Atari's library too. Not gaps
// in this transcription.
module pokey_cell21 (
	input  wire in,
	input  wire c,
	output wire q
);
	assign q = in & ~c;
endmodule

// ---------------------------------------------------------------------------
// Cell 26 - POT LINE LATCH
// ---------------------------------------------------------------------------
// Samples a pot pin on o2 and holds it through o1. The pot pins are slow
// analog ramps, so this is the only place an asynchronous input crosses into
// the chip's clocked logic; the parent is responsible for metastability.
// Output is the complement, matching the schematic.
module pokey_cell26 (
	input  wire clk,
	input  wire ph1_en,
	input  wire ph2_en,
	input  wire in,
	output wire q_n
);
	logic sampled;

	always_ff @(posedge clk)
		if (ph2_en)
			sampled <= in;
		else if (ph1_en)
			sampled <= sampled;

	assign q_n = ~sampled;
endmodule

`default_nettype wire
