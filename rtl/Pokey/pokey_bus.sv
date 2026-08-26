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

// POKEY bus interface: pads, address decode, read and write strobes.
//
// Source: PokeyReSchem-13.pdf page 2. It instantiates only two library cells
// - Cell 7 on each data pad and the Cell 21 / Cell 22 write drivers - and
// draws everything else as gates, pass transistors and two NOR planes.
//
// ---------------------------------------------------------------------------
// The sheet, left to right
// ---------------------------------------------------------------------------
//
//   D0..D7 pad -+-> Cell 7 -> pad      (in = D*r, DISABLE = PreS01 | ~csRd)
//               |
//               +-> inv inv --|Phi2B|-- inv - superbuffer -> D0w..D7w
//
//   A0..A3 pad --|PreS01|--|PreS01|--> latch --> bA3,nA3 .. bA0,nA0
//                                                     |
//                          +--------------------------+
//                          |
//        read plane, 14 rows (no B, no C)     write plane, 15 rows (no C)
//        row = depletion load, 4 taps         row = load, 4 taps + csWr
//              |                                    |
//        inverter pair -> Addr*r                 |Phi2B| -> Cell 21 / Cell 22
//              |                                    IN     C = Phi2B
//        pull down to gnd, gate = PreS01            -> Addr*w, nAddr*w
//
// Read and write are separate planes, not one decode plus a direction bit.
// Eleven of the sixteen addresses are a different register depending on
// direction, which is why the sheet emits strobe pairs:
//
//   addr   read     write      addr   read     write
//   0      POT0     AUDF1      8      ALLPOT   AUDCTL
//   1      POT1     AUDC1      9      KBCODE   STIMER
//   2      POT2     AUDF2      A      RANDOM   SKRES
//   3      POT3     AUDC2      B      -        POTGO
//   4      POT4     AUDF3      C      -        -
//   5      POT5     AUDC3      D      SERIN    SEROUT
//   6      POT6     AUDF4      E      IRQST    IRQEN
//   7      POT7     AUDC4      F      SKSTAT   SKCTL
//
// A plane row is a depletion load pulling the row high and four pull downs at
// the crossings the drawing marks with a small circle. Row r taps bAn where
// bit n of r is 0 and nAn where it is 1, so the row is left high only when
// every tapped line is low - an address match. Confirmed on the bA3 column:
// only the eight rows 7..0 mark it, which is exactly the addresses with
// A3 = 0.
//
// ---------------------------------------------------------------------------
// Phases: what qualifies a read and what qualifies a write
// ---------------------------------------------------------------------------
// Atari's coupler notation - the digit inside a gate giving its clock phase,
// the X marking an input with no coupler - is worth checking before reading
// any timing off these sheets. Page 2 carries NONE of it: no gate on the sheet
// holds a phase digit (the only two "1"s on the page are the words "Option 1")
// and the X detector finds no coupler mark anywhere, while pages 3 to 7 are
// annotated throughout. Cells 7, 21 and 22, the only library cells this sheet
// instantiates, are drawn unmarked as well.
//
// So everything below rests on the two clock rails and the pass gates sitting
// on them, not on a phase mark, and that is the whole evidence there is here.
// Atari's own bus sheet in pokey.pdf is the cross-check
// nobody has done yet.
//
// PreS01 is high across the o1 half (PHI2 low), Phi2B across the o2 half.
//
//   o1 half  address pads flow through the A latch; every Addr*r is held at
//            ground by its pull down; the pads are released.
//   o2 half  the A latch holds; the read plane drives Addr*r and a selected
//            read drives the pads; the write row and the write data are
//            sampled onto the Cell 21 / 22 IN nodes and the D*w nodes.
//   o1 half  Phi2B is low, so Q = IN & ~C fires the write strobes off those
//            nodes and the register sheets latch them.
//
// Both Phi2B pass gates are transparent latches, modelled as registers gated by
// the Phi2B level - see the note on write_data. So the write lands in the o1
// window that follows the phase which presented it, carrying the row and data
// as they stood when Phi2B fell, and nothing the pads do afterwards can reach
// it.
//
// Only the A0-A3 pads are sampled on PreS01; the write data pass gate is on
// Phi2B - the circle where each D*w line crosses the Phi2B column.
//
// ---------------------------------------------------------------------------
// Chip select
// ---------------------------------------------------------------------------
// Read off the sheet rather than taken from the pin list. Two three input
// NORs at the bottom centre:
//
//   csRd  = ~(~RW | CS0 | ~CS1) = RW & ~CS0 & CS1
//   csWr  = ~(~RW & ~CS0 & CS1)                     active low
//
// so the part is selected when CS0 is low and CS1 is high, as the pin list
// says. csWr is a fifth pull down on every write row; the read plane has no
// such column, so a read strobe is not qualified by chip select at all. What
// keeps a deselected read off the pins is the Cell 7 DISABLE, not the strobe.

`default_nettype none

module pokey_bus (
	input  wire       clk,

	// Phase strobes from pokey_clkgen.
	input  wire       pre_s01,   // o1 half: address sample, read strobes grounded
	input  wire       phi2b,     // o2 half: write plane and write data sampled

	// Pins.
	input  wire [3:0] a,
	input  wire       cs0_n,     // pin 30, active low
	input  wire       cs1,       // pin 31, active high
	input  wire       rw,        // high = read
	input  wire [7:0] d_in,

	// Read data gathered from the register sheets. The internal D0r..D7r bus
	// is precharged high and the register cells pull it down, so an address
	// nobody answers reads as all ones. The parent hands it over inverted.
	input  wire [7:0] read_bus_n,

	output wire [7:0] d_out,
	output wire       d_oe,

	// Latched write data for the register sheets: D0w..D7w.
	output wire [7:0] write_data,

	// Strobe pairs, one bit per address.
	output wire [15:0] addr_rd,
	output wire [15:0] addr_wr,
	output wire [15:0] addr_wr_n,

	// The three write-only strobes, taken off their own cells below.
	output wire       stimer_strobe,   // address 9, Cell 21
	output wire       skres_strobe,    // address A, Cell 21
	output wire       potgo_strobe     // address B, Cell 22
);
	// -----------------------------------------------------------------------
	// A0-A3. Two series pass gates on PreS01 into a pair of cross coupled
	// inverters, giving the true and complement lines the planes tap.
	// -----------------------------------------------------------------------
	logic [3:0] a_q;

	always_ff @(posedge clk)
		if (pre_s01)
			a_q <= a;

	wire [3:0] ba =  a_q;    // bA3..bA0
	wire [3:0] na = ~a_q;    // nA3..nA0

	// -----------------------------------------------------------------------
	// RW / CS0 / CS1. Each pad is buffered by an inverter pair, and the two
	// three input NORs pick the true or complement tap of each.
	// -----------------------------------------------------------------------
	wire cs_rd   = ~(~rw |  cs0_n | ~cs1);   // read and selected
	wire cs_wr_n = ~(~rw & ~cs0_n &  cs1);   // csWr, low when writing selected

	// -----------------------------------------------------------------------
	// The two NOR planes.
	// -----------------------------------------------------------------------
	logic [15:0] row_match;
	logic  [3:0] taps;
	logic  [3:0] rv;            // the row number, four bits wide

	// rv walks 0..F alongside r rather than being taken as r[3:0]. The select
	// would be shorter, but Quartus 17 will not parse the 4'(r) cast and Icarus
	// warns that it ignores a constant select in an always block, so neither
	// short form survives all three front ends.
	always_comb begin
		row_match = 16'd0;
		rv        = 4'd0;
		for (int r = 0; r < 16; r++) begin
			taps = (ba & ~rv) | (na & rv);
			row_match[r] = ~|taps;
			rv = rv + 4'd1;
		end
	end

	localparam logic [15:0] RD_ROWS = 16'hE7FF;   // no row for B or C
	localparam logic [15:0] WR_ROWS = 16'hEFFF;   // no row for C

	wire [15:0] row_r = row_match & RD_ROWS;
	wire [15:0] row_w = row_match & WR_ROWS & {16{~cs_wr_n}};

	// -----------------------------------------------------------------------
	// Read strobes. Each row is buffered out of the plane by an inverter pair
	// and its Addr*r line has a pull down to ground gated by PreS01, so the
	// strobe is live for the o2 half.
	//
	// Not qualified by chip select, and that is the sheet: the read plane has
	// no csWr style column and the ground device is gated by raw PreS01, so a
	// read strobe fires on whatever A0-A3 happen to be, selected or not, and
	// on a write too.
	//
	// Nothing observable comes of that. Reads have no side effects anywhere -
	// page 3's KBCODE strobe reaches nothing but a Cell 1 read enable, and page
	// 6 carries AddrDr exactly once, into the SERIN Cell 1 row's Rd pins - so
	// an unqualified strobe only enables a driver onto the internal bus, and
	// the Cell 7 DISABLE below keeps that bus off the pins.
	// -----------------------------------------------------------------------
	assign addr_rd = row_r & {16{~pre_s01}};

	// -----------------------------------------------------------------------
	// Data pads. Cell 7 per bit, driving the pad from the internal read bus.
	// DISABLE is the one gated signal on the sheet: NOR(PreS01, ~csRd) into a
	// super buffer, so the pads drive only for the o2 half of a selected read.
	// -----------------------------------------------------------------------
	wire [7:0] d_r = ~read_bus_n;    // D0r..D7r
	wire pad_disable = pre_s01 | ~cs_rd;
	wire [7:0] pad_oe;

	generate
		genvar b;
		for (b = 0; b < 8; b = b + 1) begin : g_pad
			pokey_cell7 u_pad (
				.in        (d_r[b]),
				.disable_n (pad_disable),
				.out       (d_out[b]),
				.oe        (pad_oe[b]));
		end
	endgenerate

	assign d_oe = &pad_oe;

	// -----------------------------------------------------------------------
	// Write data. Pad, two inverters, a pass gate on Phi2B, then an inverter
	// and a super buffer: four inversions, so D*w carries the pad value.
	//
	// The pass gate is a transparent latch, open across the o2 half and holding
	// the pad value into the o1 half where the write strobes fire. That is the
	// house pattern for this whole library - a plain register gated by the
	// level - not a fabric latch and not a flip flop clocked once a cycle.
	//
	// Leaving it combinational would satisfy this core's adapter, which holds
	// the pads for the whole POKEY clock, but not the pin contract this module
	// advertises: with the gate open across the o2 half, whatever the pads move
	// to afterwards would change or cancel the write. Same for the row pass
	// gates below.
	// -----------------------------------------------------------------------
	// Phi2B and PreS01 are drawn non-overlapping, but this model's Phi2B is
	// cleared BY ph1_en, so it is still high for the one clk that starts o1.
	// The pass gate has to close before the address latch reopens, or it would
	// capture the next cycle's address, so the window is Phi2B before o1.
	// That also puts a floor on the clock ratio for this sheet: clk has to give
	// at least three cycles per POKEY clock, or there is no clk between the two
	// levels and no write can be captured at all.
	wire wr_open = phi2b & ~pre_s01;

	logic [7:0] wdata_hold;

	always_ff @(posedge clk)
		if (wr_open)
			wdata_hold <= d_in;

	assign write_data = wdata_hold;

	// -----------------------------------------------------------------------
	// Write strobes. Each row reaches its cell's IN through a pass gate on
	// Phi2B and every C pin takes Phi2B, wired at the bottom cell and abutted
	// up the column. With Q = IN & ~C the strobe is the o1 half that follows.
	//
	// The pass gate holds the same way the write data one does. Q is forced to
	// zero for as long as C is high, so the one clk the register takes to
	// follow the row is invisible: nothing can read the node until Phi2B has
	// already fallen.
	//
	// Thirteen addresses get a Cell 22, which also emits the complement; 9 and
	// A get a Cell 21, which has only Q. F and 8 are the two sites that take
	// "OPTION 1 (ADD THIS LINE)". B has a Cell 22 whose complement pin is
	// drawn but left unlabelled, so nAddrBw goes nowhere.
	// -----------------------------------------------------------------------
	logic [15:0] row_w_hold;

	always_ff @(posedge clk)
		if (wr_open)
			row_w_hold <= row_w;

	logic [15:0] wr_q, wr_qn;

	pokey_cell22 #(.OPTION1(1'b1)) u_wF (
		.in(row_w_hold[4'hF]), .c(phi2b), .q(wr_q[4'hF]), .q_n(wr_qn[4'hF]));
	pokey_cell22 u_wE (
		.in(row_w_hold[4'hE]), .c(phi2b), .q(wr_q[4'hE]), .q_n(wr_qn[4'hE]));
	pokey_cell22 u_wD (
		.in(row_w_hold[4'hD]), .c(phi2b), .q(wr_q[4'hD]), .q_n(wr_qn[4'hD]));
	pokey_cell22 u_wB (
		.in(row_w_hold[4'hB]), .c(phi2b), .q(wr_q[4'hB]), .q_n(wr_qn[4'hB]));
	pokey_cell21 u_wA (
		.in(row_w_hold[4'hA]), .c(phi2b), .q(wr_q[4'hA]));
	pokey_cell21 u_w9 (
		.in(row_w_hold[4'h9]), .c(phi2b), .q(wr_q[4'h9]));
	pokey_cell22 #(.OPTION1(1'b1)) u_w8 (
		.in(row_w_hold[4'h8]), .c(phi2b), .q(wr_q[4'h8]), .q_n(wr_qn[4'h8]));
	pokey_cell22 u_w7 (
		.in(row_w_hold[4'h7]), .c(phi2b), .q(wr_q[4'h7]), .q_n(wr_qn[4'h7]));
	pokey_cell22 u_w6 (
		.in(row_w_hold[4'h6]), .c(phi2b), .q(wr_q[4'h6]), .q_n(wr_qn[4'h6]));
	pokey_cell22 u_w5 (
		.in(row_w_hold[4'h5]), .c(phi2b), .q(wr_q[4'h5]), .q_n(wr_qn[4'h5]));
	pokey_cell22 u_w4 (
		.in(row_w_hold[4'h4]), .c(phi2b), .q(wr_q[4'h4]), .q_n(wr_qn[4'h4]));
	pokey_cell22 u_w3 (
		.in(row_w_hold[4'h3]), .c(phi2b), .q(wr_q[4'h3]), .q_n(wr_qn[4'h3]));
	pokey_cell22 u_w2 (
		.in(row_w_hold[4'h2]), .c(phi2b), .q(wr_q[4'h2]), .q_n(wr_qn[4'h2]));
	pokey_cell22 u_w1 (
		.in(row_w_hold[4'h1]), .c(phi2b), .q(wr_q[4'h1]), .q_n(wr_qn[4'h1]));
	pokey_cell22 u_w0 (
		.in(row_w_hold[4'h0]), .c(phi2b), .q(wr_q[4'h0]), .q_n(wr_qn[4'h0]));

	// Addresses with no cell on the sheet, and the complements it never draws.
	// Row C exists in neither plane, so nothing consumes its match.
`ifdef VERILATOR
	// A sink for Verilator's UNUSED check; Quartus warns 10036 on it instead.
	wire _unused = &{1'b0, row_w[4'hC], row_w_hold[4'hC]};
`endif

	assign wr_q[4'hC]  = 1'b0;
	assign wr_qn[4'hC] = 1'b1;
	assign wr_qn[4'hA] = 1'b1;
	assign wr_qn[4'h9] = 1'b1;

	assign addr_wr   = wr_q;
	assign addr_wr_n = wr_qn;

	assign stimer_strobe = wr_q[4'h9];
	assign skres_strobe  = wr_q[4'hA];
	assign potgo_strobe  = wr_q[4'hB];

endmodule

`default_nettype wire
