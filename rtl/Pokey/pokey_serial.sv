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

// POKEY serial port: the two shift chains, their PLAs, and the bit rate clocks.
//
// Source: PokeyReSchem-13.pdf page 6, with the two
// PLA matrices on page 7 (both labelled "PAGE 5", Cwik's numbering for page 6).
// Frame format from the CO12294 spec.
//
// ---------------------------------------------------------------------------
// The two chains, as page 6 draws them left to right
// ---------------------------------------------------------------------------
// Receive, ten cells. SID reaches Cell 25's D through four inverters, so the
// chain holds the line true. Data moves left to right, one cell per bit.
//
//   SID -> Cell 25 -> Cell 17 -> ... -> Cell 17 -> Cell 17
//          ssiStopB   ssi_7             ssi_0      ssiStartBit
//                     `-- eight Cell 1 latches (SERIN) hang under these --'
//
// Only the nine Cell 17s have SET, and Cell 25 does not. That asymmetry is what
// makes the frame counter work: when the start bit lands in Cell 25 the PLA
// presets the other nine to one, leaving that single zero to walk the chain.
// Nine bit times later it reaches the far end, and by then Cell 25 holds the
// stop bit and the eight middle cells hold the byte.
//
// Transmit, ten cells, the mirror image:
//
//   0 -> Cell 17 -> Cell 15 -> ... -> Cell 15 -> Cell 16 -> SOD
//        stop bit   D7w               D0w        start bit
//                   `-- eight Cell 5 latches (SEROUT) feed these --'
//
// The load fires all three at once: Cell 16's R lays down the start bit, the
// Cell 15s take the byte from SEROUT, Cell 17's SET lays down the stop bit.
// Cell 17's D is grounded, so the chain fills with zeros behind the frame; the
// wired NOR along the top of the row watches those nine cells and calls the
// shifter empty when they are all zero. Shifting then stops with the stop bit
// still sitting in Cell 16, which is why the idle line reads high.
//
// ---------------------------------------------------------------------------
// Frame and bit rate
// ---------------------------------------------------------------------------
// "8 bits of serial data preceded by a logic zero start bit, and succeeded by a
// logic true stop bit. Input and output clocks are equal to the baud (bit)
// rate, not 16 times baud rate. Transmitted data changes when the output clock
// goes true. Received data is sampled when the input clock goes to zero."
//
// Page 6 puts a Cell 2 wired as a toggle on each of Timer4 and Timer2, so the
// bit rate is half the timer rate - the timer has to be programmed at twice the
// baud rate. Each clock then feeds a NOR against a three inverter delay,
// which turns the level into a narrow pulse on one edge; that is the shift
// pulse. OCLK is an open drain pull-down off that same node, so the pin rises
// as the pulse fires.
//
// ---------------------------------------------------------------------------
// SKCTL[6:4] mode chart, from the spec
// ---------------------------------------------------------------------------
//   D6 D5 D4   out rate   in rate    bi-dir clock
//   0  0  0    ext        ext        ext input   (also resets internal phase)
//   0  0  1    ext        chan 4 A   ext input
//   0  1  0    chan 4     chan 4     chan 4 output
//   0  1  1    chan 4 A   chan 4 A   input        "not useful"
//   1  0  0    chan 4     ext        ext input
//   1  0  1    chan 4 A   chan 4 A   input        "not useful"
//   1  1  0    chan 2     chan 4     chan 4 output
//   1  1  1    chan 2     chan 4 A   not used     (tri-state)
//
// The 110 in rate is the one entry Atari got wrong. Its own table says chan 2;
// the input rate is always chan 4 or external, and the Comments column of the
// same table agrees. Avery Lee's erratum says so too, and Watson's mode case
// reads chan 4.
//
// "A" marks the asynchronous receive modes, and they are exactly the four with
// bit 4 set, which is why bit 4 alone is the async term below. The rate mux on
// the sheet is a pass gate network off Skctls_4/5/6; it was not traced gate by
// gate, and this chart is the spec's, not the drawing's.
//
// ---------------------------------------------------------------------------
// Evidence standing
// ---------------------------------------------------------------------------
// No 7800 cartridge uses POKEY serial, so nothing here has been checked against
// hardware. The cells, the two PLAs and the frame mechanism come from the
// schematic. What does not: the gate between preSdoLoad and ssoLoad, the
// sdoDloaded latch, and the overrun rule - see the comments where each is used.

`default_nettype none

module pokey_serial (
	input  wire       clk,

	input  wire [7:0] skctl,
	input  wire       init,

	input  wire       serout_wr,   // write strobe at address $0D
	input  wire       serin_rd,    // read strobe at address $0D
	input  wire [7:0] write_data,

	// Bit rate sources, and channel 1 for the two-tone pair.
	input  wire       timer1,      // channel 1 divider
	input  wire       timer2,      // channel 2 divider
	input  wire       timer4,      // channel 4 divider
	input  wire       bclk,        // the external bi-directional clock pin

	input  wire       sid,         // serial input data pin

	output wire [7:0] serin,       // the stored byte, for tracing and tests
	output wire [7:0] serin_bus,   // this sheet's pull on the read bus
	output wire       sod_out,
	output wire       sod_oe,
	output wire       oclk_out,    // transmit clock, its own pin
	output wire       oclk_oe,
	output wire       bclk_out,    // bi-directional clock pin, driven half
	output wire       bclk_oe,

	// Back to the audio sheet: the two resynchronisations the serial logic
	// forces on the timers.
	output wire       twotone_reset, // channels 1 and 2
	output wire       async_reset,   // channels 3 and 4

	output wire       serin_ready,   // IRQST bit 5
	output wire       serout_needed, // IRQST bit 4
	output wire       serout_done,   // IRQST bit 3
	output wire       serin_busy,    // SKSTAT bit 1
	output wire       frame_error    // SKSTAT bit 7
);
	wire [2:0] mode        = skctl[6:4];
	wire       force_break = skctl[7];

	wire twotone_en = skctl[3];

	// Bits 2:0 belong to other sheets.
`ifdef VERILATOR
	// A sink for Verilator's UNUSED check; Quartus warns 10036 on it instead.
	/* verilator lint_off UNUSED */
	wire _unused_skctl = &{1'b0, skctl[2:0]};
	/* verilator lint_on UNUSED */
`endif

	// -----------------------------------------------------------------------
	// Bit rate: two Cell 2 toggles, then the pulse per bit
	// -----------------------------------------------------------------------
	// Each toggle feeds its own D from its own Q, inverted. On the die that is a
	// master-slave pair - two latches on opposite phases - so it flips once per
	// target clock however long a phase lasts. The one clk enable here is that
	// pair collapsed, and is the model, not a compromise: a single latch held
	// open on an inverting loop never settles, it flips every gate delay.
	logic timer4_q, timer2_q, timer1_q;

	always_ff @(posedge clk) begin
		timer4_q <= timer4;
		timer2_q <= timer2;
		timer1_q <= timer1;
	end

	wire timer4_rise = timer4 & ~timer4_q;
	wire timer2_rise = timer2 & ~timer2_q;
	wire timer1_rise = timer1 & ~timer1_q;   // two-tone only

	wire tog4_q, tog2_q;

	// The receive toggle is also held reset while an asynchronous receiver is
	// waiting for a start bit - see the async section below.
	pokey_cell2 u_tog4 (
		.clk, .ld(timer4_rise), .d(~tog4_q), .p(1'b0), .r(init | async_reset),
		.q(tog4_q));

	pokey_cell2 u_tog2 (
		.clk, .ld(timer2_rise), .d(~tog2_q), .p(1'b0), .r(init), .q(tog2_q));

	wire togd_tmr4 = ~tog4_q;      // the sheet takes both toggles inverted
	wire togd_tmr2 = ~tog2_q;

	logic out_clk, in_clk;

	always_comb begin
		unique case (mode)
			3'b000: begin out_clk = bclk;      in_clk = bclk;      end
			3'b001: begin out_clk = bclk;      in_clk = togd_tmr4; end
			3'b010: begin out_clk = togd_tmr4; in_clk = togd_tmr4; end
			3'b011: begin out_clk = togd_tmr4; in_clk = togd_tmr4; end
			3'b100: begin out_clk = togd_tmr4; in_clk = bclk;      end
			3'b101: begin out_clk = togd_tmr4; in_clk = togd_tmr4; end
			3'b110: begin out_clk = togd_tmr2; in_clk = togd_tmr4; end
			3'b111: begin out_clk = togd_tmr2; in_clk = togd_tmr4; end
		endcase
	end

	logic out_clk_q, in_clk_q;

	always_ff @(posedge clk) begin
		out_clk_q <= out_clk;
		in_clk_q  <= in_clk;
	end

	wire sdo_clock = out_clk & ~out_clk_q;   // transmit when the out clock goes true
	wire sdi_clock = in_clk_q & ~in_clk;     // sample when the in clock goes to zero

	// Two separate pins, and they are not the same net. OCLK is an open drain
	// output that always carries the transmit clock. BCLK is bidirectional:
	// an input except in modes 010 and 110, where the chip drives it with the
	// receive rate - channel 4 in both, which is what Watson drives on
	// SIO_CLOCKIN_OUT and what the corrected mode table says.
	assign oclk_out = out_clk;
	assign oclk_oe  = 1'b1;

	assign bclk_out = togd_tmr4;
	assign bclk_oe  = (mode == 3'b010) || (mode == 3'b110);

	// =======================================================================
	// Receive
	// =======================================================================
	// Phases within one received bit, read off the coupler-phase marks on page
	// 6. sdiClock climbs a column
	// of inverters marked 2,1 then 2,1 - two ("2","1") coupler pairs, so two
	// target clocks - and the three control lines are taken off it as:
	//
	//   ssiShft   = sdiClock delayed one pair
	//   ssiTransf = NOR "2" (ssiShft, sdiClock)
	//   ssiRec    = NOR "2" (shftD, ssiShft, sdiClock, ssiSet)
	//
	// Two things fall out. ssiTransf is a LEVEL, not a pulse: it is low only for
	// the two clocks sdiClock and ssiShft cover, and stands high from the
	// transfer clock onward, so the transfer gate is already open when SET
	// fires. And ssiSet reaches ssiRec on an X input - no coupler - so SET shuts
	// REC off in its own phase rather than the next one, which is what stops REC
	// from fighting it.
	//
	// pokey_cell17 moves master to slave on a clock edge, so an open gate cannot
	// be expressed as one enable; [3] is that open gate, one clk later. The
	// phase marks confirm this shape rather than replacing it.
	//
	//   [0] Sh   [1] Tr   [2] SET, from the state the phase opened with
	//   [3] Tr again: the gate SET saw open
	logic [3:0] rx_seq;

	always_ff @(posedge clk) begin
		if (init) begin
			rx_seq <= 4'b0000;
		end else begin
			rx_seq[0] <= sdi_clock;
			rx_seq[1] <= rx_seq[0];
			rx_seq[2] <= rx_seq[1];
			rx_seq[3] <= rx_seq[2];
		end
	end

	wire pre_sdi_set, sdi_compl, n_framerr, sd1_d1, no_sdi_err;

	// init runs the same three strobes so the chain comes up idle instead of
	// undefined: the Cell 17s take SET, Cell 25 takes the line.
	wire ssi_shft   = rx_seq[0] | init;
	wire ssi_transf = rx_seq[1] | rx_seq[3] | init;
	wire ssi_set    = (rx_seq[2] & ~pre_sdi_set) | init;
	// REC must stay off while Tr is open, or master and slave swap.
	wire ssi_rec    = ~ssi_shft & ~ssi_set & ~ssi_transf;

	// ssi[0] is Cell 25 and holds the stop bit at the end of a frame; ssi[1] is
	// the sheet's ssi_7 down to ssi[8] = ssi_0; ssi[9] is where the start bit
	// arrives and ends the frame.
	wire [9:0] ssi;

	pokey_cell25 u_ssi0 (
		.clk, .shift(ssi_shft), .d(sid), .transfer(ssi_transf), .rec(ssi_rec),
		.q(ssi[0]));

	genvar i;
	generate
		for (i = 1; i <= 9; i = i + 1) begin : g_ssi
			pokey_cell17 u (
				.clk, .shift(ssi_shft), .d(ssi[i-1]), .set(ssi_set),
				.transfer(ssi_transf), .rec(ssi_rec), .q(ssi[i]));
		end
	endgenerate

	// The two buffers the sheet puts in front of the PLA. sdinStartBit is the
	// inverted one - the frame is over when the start bit reaches the far end.
	wire sdi_stop_bit    = ssi[0];
	wire sdin_start_bit  = ~ssi[9];

	// -----------------------------------------------------------------------
	// SERIAL DATA IN PLA (page 7, bottom middle)
	// -----------------------------------------------------------------------
	// NOR-NOR array. A dot on an input's true column means the term wants that
	// input at 0, a dot on its complement column means 1; term = NOR of its
	// dotted columns, output = NOR of its dotted terms.
	//
	//        1 sdiStopBit   2 sdinStartBit   3 sdiQ1
	//   T1        0                -            1
	//   T2        -                0            0
	//   T3        1                1            0
	//   T4        0                1            0
	//   T5        1                -            1
	//
	//   1 nFramerr  = ~T4          low marks a bad stop bit
	//   2 sdiCompl  = ~(T3|T4)     low marks the end of a frame
	//   3 sdiBusy   = ~sdiQ1       the input inverter itself, no term column
	//   4 sd1D1     = ~(T1|T2)     back into the Cell 2 below the PLA
	//   5 preSdiSet = ~T1          low arms the preset of the nine Cell 17s
	//   6 noSdiErr  = ~(T3|T5)     low for a clean frame or a quiet line
	//
	// Reading it back out: while idle (sdiQ1 = 1) sd1D1 follows the stop bit
	// cell, so a zero arriving there drops sdiQ1 and starts the frame, and T1
	// fires the preset in the same phase. While busy, sd1D1 follows
	// sdinStartBit, so sdiQ1 returns to idle exactly when the start bit reaches
	// the last cell - and that is the phase T3 and T4 test the stop bit in.
	wire sdi_q1;

	wire sdi_ld = rx_seq[2];

	// The array reads the cell's stored side. sd1D1 is one of its own outputs and
	// goes straight back to this cell's D, and that loop does not always settle:
	// at term T4 - bad stop bit, start bit at the far end - sd1D1 inverts sdiQ1
	// forever. So the window is one clk, the master-slave pair collapsed, same
	// as the bit rate toggles.
	//
	// One iteration is also what the die's consumers see: SET and sdiCompl act on
	// the state the window opened with, which is what starts the frame and what
	// lands the byte. The new state reaches the next window.
	wire t1_in = ~sdi_stop_bit   &  sdi_q1;
	wire t2_in = ~sdin_start_bit & ~sdi_q1;
	wire t3_in =  sdi_stop_bit   &  sdin_start_bit & ~sdi_q1;
	wire t4_in = ~sdi_stop_bit   &  sdin_start_bit & ~sdi_q1;
	wire t5_in =  sdi_stop_bit   &  sdi_q1;

	assign n_framerr   = ~t4_in;
	assign sdi_compl   = ~(t3_in | t4_in);
	wire   sdi_busy    = ~sdi_q1;
	assign sd1_d1      = ~(t1_in | t2_in);
	assign pre_sdi_set = ~t1_in;
	assign no_sdi_err  = ~(t3_in | t5_in);

	pokey_cell2 u_sdi_q1 (
		.clk, .ld(sdi_ld), .d(sd1_d1), .p(init), .r(1'b0), .q(sdi_q1));

	// -----------------------------------------------------------------------
	// Asynchronous receive: the start bit resets channels 3 and 4
	// -----------------------------------------------------------------------
	// "In asynchronous modes, channels 3 and 4 are reset by each start bit at
	// the beginning of each serial data byte. This allows the serial data rate
	// to be slightly different from the rate set by channels 3 and 4."
	//
	// Taken as the edge the spec describes and not, as Watson does
	// (pokey.vhdl), as a level held for the whole idle stretch. The
	// difference matters here because this sheet has one toggle per timer, not
	// one per direction, so a held reset would freeze the transmit clock too -
	// and in modes 011 and 101, where both directions run off channel 4, that
	// deadlocks the transmitter with no way out.
	//
	// Resetting the toggle as well as the counters is what fixes the sampling
	// phase. Channel 4's first borrow then lands one timer period - half a bit
	// time - after the edge, and the toggle falls there, so SID is sampled in
	// the middle of the start bit and every half bit time after it.
	logic sid_q;

	always_ff @(posedge clk)
		sid_q <= sid;

	assign async_reset = skctl[4] & sdi_q1 & sid_q & ~sid;

	// SERIN: eight Cell 1 latches taken from the eight middle cells, loaded the
	// phase the frame completes.
	wire sdi_load = rx_seq[2] & ~sdi_compl;

	wire [7:0] serin_q, serin_oe;

	generate
		for (i = 0; i < 8; i = i + 1) begin : g_serin
			pokey_cell1 u (
				.clk, .ld(sdi_load), .d(ssi[8-i]), .rd(serin_rd),
				.q(serin_q[i]), .q_oe(serin_oe[i]), .state(serin[i]));
		end
	endgenerate

	// Precharged bus: the row only ever pulls it down.
	assign serin_bus = serin_q | ~serin_oe;

	// nFramerr goes low at the completing phase when the stop bit cell holds a
	// zero, which is the framing error.
	//
	// Overrun is not on page 6. AddrDr appears exactly once on the whole sheet
	// and reaches nothing but the Rd pin bussed along the SERIN Cell 1 row, so a
	// read of SERIN has no side effect and nothing here can count unread bytes.
	// It is built on the IRQ sheet instead, the way the keyboard one is: a byte
	// completing while the SERIN interrupt is still asserted. serin_ready is that
	// byte, so the IRQ sheet needs nothing else from here.
	assign serin_ready   = sdi_load;
	assign serin_busy    = sdi_busy;
	assign frame_error   = sdi_load & ~n_framerr;

	// =======================================================================
	// Transmit
	// =======================================================================
	// SEROUT: eight Cell 5 latches under the shift row.
	wire [7:0] serout_hold;

	generate
		for (i = 0; i < 8; i = i + 1) begin : g_serout
			pokey_cell5 u (.clk, .ld(serout_wr), .d(write_data[i]), .q(serout_hold[i]));
		end
	endgenerate

	// Same two phase shape as the receiver: decide, then transfer. A load takes
	// the place of a shift for that bit time, and puts the start bit on the pin.
	//
	// The transmit sequencer is drawn as the receiver's mirror image and carries
	// the same marks:
	// ssoTransfer = NOR "2" (ssoShft, the stage before it), again a level rather
	// than a pulse, and ssoLoad enters ssoRec on an X input - uncoupled, so a
	// load shuts REC off in its own phase, the way ssiSet does above.
	//
	// Two differences from the receiver that this code does NOT reproduce and
	// that nothing measured contradicts, so they are recorded rather than acted
	// on: the transmit chain puts an extra ("2","1") pair plus a static inverter
	// between the sdoClock gate and ssoShft, and ssoLoad is taken off the earlier
	// of the two stages, so on the die a load leads its shift by two target
	// clocks instead of replacing it in the same one.
	logic [1:0] tx_seq;

	wire sso_empty, sdon_shft_en, pre_sdo_load, sdo_finish, sdo_d1;
	wire sdo_dloaded;

	// preSdoLoad on its own is true whenever a byte is waiting, so the sheet
	// gates it with something before it reaches ssoLoad. That gate is a NOR on
	// page 6 whose second input was not resolved; what the frame needs, and what
	// this uses, is "a byte is waiting and the shifter has run dry".
	wire ssoload_req = sdo_dloaded & sso_empty & pre_sdo_load;

	always_ff @(posedge clk) begin
		if (init) begin
			tx_seq <= 2'b00;
		end else begin
			tx_seq[0] <= sdo_clock;
			tx_seq[1] <= tx_seq[0];
		end
	end

	// INIT is deliberately absent from these three. Page 6 draws it into the
	// output PLA Cell 2's P pin and the sdoDloaded latch and nowhere else on
	// this row, and Altirra's hardware-derived rule agrees: INIT resets the
	// serial state machines and the input shift register, not the output shift
	// register (pokey.cpp). So REC stands during INIT and the chain keeps
	// whatever it held, including the level on SOD.
	wire sso_load   = tx_seq[0] & ssoload_req;
	wire sso_shft   = tx_seq[0] & ~ssoload_req & sdon_shft_en;
	wire sso_transf = tx_seq[1];
	// SET is ssoLoad, and the sheet says so by abutment, not by a wire: the pin
	// labels along the row bottom read SET|Ld  Ld|Ld ... Ld|R  R, so the net
	// enters at Cell 16's right hand R, runs through every Cell 15's Ld and ends
	// at Cell 17's SET - the same bussed-through-the-row style as the AUDF chain.
	// Nothing joins SET from below: the inverter at the right of the row drives
	// a line that passes underneath without touching it.
	wire sso_set    = sso_load;          // Cell 17 lays down the stop bit
	wire sso_rec    = ~sso_shft & ~sso_load & ~sso_set & ~sso_transf;

	// sso[0] is the Cell 17 that holds the stop bit, sso[1]..sso[8] the eight
	// Cell 15s holding D7w..D0w, sso[9] the Cell 16 that faces the pin.
	wire [9:0] sso;

	pokey_cell17 u_sso0 (
		.clk, .shift(sso_shft), .d(1'b0), .set(sso_set),
		.transfer(sso_transf), .rec(sso_rec), .q(sso[0]));

	generate
		for (i = 1; i <= 8; i = i + 1) begin : g_sso
			pokey_cell15 u (
				.clk, .shift(sso_shft), .d(sso[i-1]), .load(sso_load),
				.in(serout_hold[8-i]), .transfer(sso_transf), .rec(sso_rec),
				.q(sso[i]));
		end
	endgenerate

	pokey_cell16 u_sso9 (
		.clk, .shift(sso_shft), .d(sso[8]), .r(sso_load),
		.transfer(sso_transf), .rec(sso_rec), .q(sso[9]));

	// The wired NOR along the top of the row. Cell 16 does not tap it, so it
	// reads empty with the stop bit still on the pin.
	assign sso_empty = ~|sso[8:0];

	// -----------------------------------------------------------------------
	// SERIAL DATA OUT PLA (page 7, bottom right)
	// -----------------------------------------------------------------------
	// Same array style as the input PLA. sdoDloaded has no complement column -
	// the sheet draws no inverter for it.
	//
	//        1 sdoQ1   2 sdoDloaded   3 ssoEmpty
	//   T1      1            0             -
	//   T2      0            -             1
	//   T3      -            0             0
	//
	//   1 sdoD1       = ~(T1|T2|T3)   back into the Cell 2 beside the PLA
	//   2 sdoFinish   = ~sdoQ1        the input inverter itself, no term column
	//   3 sdonShftEn  = ~T2           shift unless idle with the chain empty
	//   4 preSdoLoad  = ~(T1|T3)
	//
	// sdoQ1 tracks a byte written while a frame is still going out; sdoFinish is
	// its complement. sdonShftEn stops the chain the moment it empties, which is
	// what leaves the stop bit on the pin.
	wire sdo_q1;

	wire sdo_ld = tx_seq[1];

	// Stored side again: sdoD1 is this array's own output and this cell's D, so
	// one clk of window, as above.
	wire t1_out =  sdo_q1 & ~sdo_dloaded;
	wire t2_out = ~sdo_q1 &  sso_empty;
	wire t3_out = ~sdo_dloaded & ~sso_empty;

	assign sdo_d1       = ~(t1_out | t2_out | t3_out);
	assign sdo_finish   = ~sdo_q1;
	assign sdon_shft_en = ~t2_out;
	assign pre_sdo_load = ~(t1_out | t3_out);

	// INIT enters this cell's P, which is where page 6 draws it.
	pokey_cell2 u_sdo_q1 (
		.clk, .ld(sdo_ld), .d(sdo_d1), .p(init), .r(1'b0), .q(sdo_q1));

	// sdoDloaded: set by the write to SEROUT, cleared when the shifter takes the
	// byte. Page 6 builds it from a NOR pair off AddrDw and Init that was not
	// traced; this is the behaviour those gates have to produce.
	logic dloaded_r;

	always_ff @(posedge clk) begin
		if (init)           dloaded_r <= 1'b0;
		else if (serout_wr) dloaded_r <= 1'b1;
		else if (sso_load)  dloaded_r <= 1'b0;
	end

	assign sdo_dloaded = dloaded_r;

	// The inverter at the right of the transmit row takes Cell 16's Q and runs
	// left under the whole row without touching it, then down the left edge of
	// the block into a NOR against Skctls_7, SKCTL bit 7 - the force break gate:
	// ~(~sso9 | break) = sso9 & ~break. That drives the SOD pad, which is open
	// drain and inverts twice, so the pin follows Cell 16.
	wire sod_level = force_break ? 1'b0 : sso[9];

	// -----------------------------------------------------------------------
	// Two-tone, SKCTL bit 3
	// -----------------------------------------------------------------------
	// "In this mode audio channel 1 is transmitted in place of logic true, and
	// audio channel 2 is transmitted in place of logic false. Channel 2 must be
	// the lower tone of the tone pair." Break forces channel 2, because it
	// forces the data level to zero and the substitution follows that level.
	//
	// It is not a tone mux. One flip flop carries the output and both channels
	// toggle it, so the pin carries whichever tone is allowed to reach it:
	//
	//   channel 2 borrow  ->  always toggles
	//   channel 1 borrow  ->  toggles only while the data bit is true
	//
	// and every toggle reloads BOTH counters, so the shorter of the two wins
	// outright rather than beating against the other. Channel 1 is meant to be
	// the shorter period, so a true bit comes out as channel 1's tone and a
	// false bit as channel 2's.
	//
	// The missing gate on the channel 2 side is not an oversight in this code.
	// Atari's own block diagram draws a symmetric switch; Avery Lee's erratum
	// says the drawing is wrong twice - the select polarity is backwards AND
	// the hardware has no AND gate on the channel 2 side, so channel 2 pulses
	// always resync. Watson's pokey.vhdl reads the same. Do not "fix" the
	// asymmetry back.
	//
	// The reset is gated by bit 3 but the toggle is not; with the substitution
	// off the flip flop simply runs unobserved.
	// The edges, not the levels: a toggle re-samples on every clk its enable is
	// high, so a borrow held for a phase would flip it several times. Same
	// reason the bit rate toggles above take timerN_rise.
	wire tt_toggle = timer2_rise | (timer1_rise & sod_level);

	logic tt_q;

	always_ff @(posedge clk)
		if (init)           tt_q <= 1'b0;
		else if (tt_toggle) tt_q <= ~tt_q;

	assign twotone_reset = tt_toggle & twotone_en;

	assign sod_out = twotone_en ? tt_q : sod_level;
	assign sod_oe  = 1'b1;

	// The holding register is free the moment the shifter takes the byte.
	assign serout_needed = sso_load;

	// SEROC is a LEVEL, not an edge. Page 3 takes `sdoFinish` straight to IRQST
	// bit 3 without the cross-coupled pair every other IRQST bit gets, and the
	// spec is explicit: bit 3 is not a latch, IRQEN cannot reset it, and it
	// reads zero whenever the output shifter is empty. So hand the IRQ sheet
	// the live empty state and let it decide; an edge here would make SEROC
	// polling and enable-after-completion both wrong.
	assign serout_done   = sso_empty;

	// noSdiErr and sdoFinish are PLA outputs page 6 takes elsewhere: noSdiErr
	// into the SKSTAT error gates, sdoFinish out to the IRQ sheet. Kept so the
	// array reads as drawn.
`ifdef VERILATOR
	// A sink for Verilator's UNUSED check; Quartus warns 10036 on it instead.
	/* verilator lint_off UNUSED */
	wire _unused_pla = &{1'b0, no_sdi_err, sdo_finish};
	/* verilator lint_on UNUSED */
`endif

endmodule

`default_nettype wire
