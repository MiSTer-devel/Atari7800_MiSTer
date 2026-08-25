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

// One POKEY audio channel: AUDF divider, AUDC control, noise select, the
// optional high pass, and the Cell 11 volume DAC.
//
// Source: references/Pokey/schematics/PokeyReSchem-13.pdf page 5.
//
// ---------------------------------------------------------------------------
// The chain, as drawn
// ---------------------------------------------------------------------------
//
//   count_en --> AUDF down counter (Cell 24 x8) --> borrow = "Timer n"
//                                                      |
//        poly5 --> [Cell 2, Ld = Timer n] -------------+   sampled at the
//                                                      |   channel's own rate
//   AUDC[7] NOTPOLY5 ---- or ----------------------> gate
//                                                      |
//                    AUDC[5] PURE  -> toggle           |
//                    AUDC[6] POLY4 -> poly4            +--> audio flip flop
//                    otherwise     -> poly17 or poly9  |
//                                                      v
//                       high pass XOR (channels 1 and 2 only)
//                                                      v
//                              Cell 11 DAC --> AUD
//
// The polys run at the full 1.79 MHz and are sampled here, at this channel's
// divider rate. That is what the spec means by "each channel appears to
// contain separate poly counters clocked at its own frequency". Clocking the
// polys per channel instead is the classic way to get POKEY audio wrong.
//
// The volume path has two inversions that cancel. AUDC bits 3:0 are stored in
// Cell 12, whose outputs are complements, and each drives a NOR inside Cell 11.
// Complement into NOR is a plain AND, so the DAC steers current when the
// volume bit is set. `pokey_cell11` takes the true volume and the parent wires
// it straight through; the Cell 12 storage is an artefact of the die, not
// behaviour to reproduce.

`default_nettype none

module pokey_chan (
	input  wire       clk,

	input  wire       wr_en,      // AUDF write strobe for this channel
	input  wire       audc_wr,    // AUDC write strobe
	input  wire [7:0] write_data,

	input  wire       count_en,    // one pulse per divider tick, routed by AUDCTL
	input  wire       clk179_en,   // 1.79 MHz, for the reload delay below
	input  wire       ph2_en,      // the opposite target phase; see the high pass

	// 16 bit pairing. When wide_lo is set this channel is the low half: its own
	// underflow clocks the partner instead of reloading it, so the two behave
	// as one 16 bit counter rather than as a product of two dividers. The
	// partner asserts force_reload when IT underflows, which reloads both.
	input  wire       wide_lo,
	input  wire       force_reload,

	// The serial sheet resynchronises timers: two-tone on channels 1 and 2, an
	// asynchronous start bit on channels 3 and 4. While that resync is on the
	// way back the counter still reloads, but the pulse it produced must not
	// reach anything downstream - not the interrupt, not the waveform, not a
	// 16 bit partner. Watson makes exactly this split between audfN_pulse_raw
	// and audfN_pulse (pokey.vhdl:690-699).
	input  wire       pulse_mask,

	// Extra 1.79 MHz cycles the reload path costs. Atari's modified formula:
	//   Fout = Fin / (2 * (AUDF + M)),  M = 4 for 8 bit, M = 7 for 16 bit,
	// against the normal M = 1. The difference is reload latency, which is only
	// visible when the counter is clocked at 1.79 MHz - at 64 kHz or 15 kHz it
	// hides inside a single slow period, which is why the spec says to use the
	// normal formula there. So the parent passes 3 or 6 in fast mode and 0
	// otherwise, and the arithmetic falls out.
	input  wire [2:0] reload_delay,

	// Poly outputs, already running at 1.79 MHz.
	input  wire       poly4,
	input  wire       poly5,
	input  wire       poly17,

	input  wire       stimer,       // force the output to its known state
	input  wire       stimer_level, // high for channels 1 and 2, low for 3 and 4

	input  wire       hp_clock,     // paired channel's divider, for the high pass
	input  wire       hp_enable,    // AUDCTL bit 2 or 1

	output wire       borrow,       // "Timer n": the counter crossing zero
	output wire       borrow_raw,   // the same, before pulse_mask
	output wire       out,          // waveform ahead of the DAC
	output wire [3:0] dac           // Cell 11 drive
);
	// -----------------------------------------------------------------------
	// AUDC: eight bits of control. Only the volume half reaches the DAC.
	// -----------------------------------------------------------------------
	// Page 5 splits the register: bits 7..4 are Cell 5, bits 3..0 are Cell 12.
	// Cell 12 hands out the complement, which is what the NORs inside Cell 11
	// want. Taking it back to true here, and letting `pokey_cell11` take a true
	// volume, is the same pair of gates written the readable way round.
	wire [7:4] audc_ctl;
	wire [3:0] vol_n;

	pokey_cell5  #(.WIDTH(4)) u_audc_ctl (
		.clk, .ld(audc_wr), .d(write_data[7:4]), .q(audc_ctl));
	pokey_cell12 #(.WIDTH(4)) u_audc_vol (
		.clk, .ld(audc_wr), .d(write_data[3:0]), .q_n(vol_n));

	wire [3:0] vol       = ~vol_n;
	wire       vol_only  = audc_ctl[4];
	wire       pure_tone = audc_ctl[5];
	wire       use_poly4 = audc_ctl[6];
	wire       notpoly5  = audc_ctl[7];

	// -----------------------------------------------------------------------
	// AUDF: an eight bit down counter, built from the cells page 5 draws.
	//
	// The row runs against the write bus D7w..D0w left to right, so bit 7 is
	// the left hand box and bit 0 the right hand one:
	//
	//   bit     7     6     5     4     3     2     1     0
	//   cell   24*   24    24    20    24    24    24    20     * Option 1
	//
	// The boxes abut BOR to CR, so bit 0 takes the channel's clock and every
	// bit's borrow is the clock for the bit above it. This is a ripple, not a
	// synchronous chain with a shared enable. Bit 7's borrow is the underflow.
	// -----------------------------------------------------------------------
	localparam bit [7:0] CELL20 = 8'b0001_0001;   // Cell 20 sits at bits 4 and 0

	wire [7:0] count;
	wire [7:0] bor;
	wire [7:0] cr;

	// A divide by N counter reloads at underflow, and STIMER reloads it early.
	// Cell 20 and Cell 24 both give `ld` priority over `cr`. In 16 bit low
	// position the counter does NOT reload on its own underflow - it wraps to
	// $FF the way a real down counter does, and the partner's underflow
	// reloads it.
	wire reload_i;

	// Reload latency. While this is running the counter is held.
	logic [2:0] delay_q;
	wire        holding = (delay_q != 3'd0);

	genvar i;
	generate
		for (i = 0; i < 8; i = i + 1) begin : g_audf
			if (i == 0) begin : g_clk
				assign cr[i] = count_en & ~holding;   // bit 0 takes the channel clock
			end else begin : g_ripple
				assign cr[i] = bor[i-1];              // every other bit takes a borrow
			end

			if (CELL20[i]) begin : g_cell20
				pokey_cell20 u (
					.clk,
					.wr    (wr_en),
					.d     (write_data[i]),
					.ld    (reload_i),
					.cr    (cr[i]),
					.q     (count[i]),
					.reload(),
					.bor   (bor[i]),
					.bor_n ());
			end else begin : g_cell24
				pokey_cell24 #(.OPTION1(i == 7)) u (
					.clk,
					.wr    (wr_en),
					.d     (write_data[i]),
					.ld    (reload_i),
					.cr    (cr[i]),
					.q     (count[i]),
					.reload(),
					.bor   (bor[i]),
					.bor_n ());
			end
		end
	endgenerate

	// The counter value itself is never read back - AUDF is write only, and
	// the cells have no Q pin on the drawing - but keeping it named makes the
	// borrow chain readable and shows up in traces.
`ifdef VERILATOR
	// A sink for Verilator's UNUSED check; Quartus warns 10036 on it instead.
	/* verilator lint_off UNUSED */
	wire _unused_count = &{1'b0, count};
	/* verilator lint_on UNUSED */
`endif

	// The counter crossing zero is this channel's timer event and the tick the
	// waveform logic runs on.
	//
	// Page 5 puts two clocked gates between bit 7's nBOR and the Timer n net
	// (150 dpi crop 3550,250-5100,1500): a NOR marked "2", with reload12 on its
	// uncoupled X input, then an inverter marked "1" driving the keeper pair.
	// Atari's legend on references/Pokey/die/pokey.pdf page 46 says those digits
	// are coupler clock phases, so the pair is one whole target clock, o2 then
	// o1. It is collapsed here on purpose. The reload branch off that same node
	// is the only place the delay is observable, and it is already carried by
	// `reload_delay` - Atari's M = 4 / M = 7. Delaying `tick` instead would put
	// the reload one more 1.79 MHz cycle out and make the fast divisor AUDF + 5,
	// which the spec's own formula rules out.
	//
	// That NOR mixes a coupled input with a direct one, so the two terms are not
	// contemporary: nBOR is a phase stale and reload12 is live. The live half is
	// already here - `bor = cr & ~q & ~ld` inside Cell 20 and Cell 24 makes the
	// borrow fall the instant a reload asserts, so `bor[7] & ~reload_i` is
	// `bor[7]` identically and the gate would add nothing. Only the stale half
	// is collapsed, and no case has turned up where it shows.
	// The raw underflow drives the reload; the masked one is what leaves this
	// module and what the waveform runs on.
	wire tick_raw = bor[7];
	wire tick     = tick_raw & ~pulse_mask;

	assign borrow     = tick;
	assign borrow_raw = tick_raw;

	// Ld suppresses BOR inside every cell, so a reload wired straight back from
	// the underflow would cancel the underflow that caused it - and in 16 bit
	// mode the loop closes through the partner channel as well. The die does
	// not have that problem: Ld returns from the borrow through a NOR, an
	// inverter and an explicit delay element, so it lands a phase after the CR
	// pulse and the two are never true together. One register here stands in
	// for that delay. The counter therefore passes through $FF on its way to
	// the reload value rather than jumping straight to it, which is what a real
	// down counter does too.
	logic ld_q;

	always_ff @(posedge clk)
		ld_q <= stimer | force_reload | (tick_raw & ~wide_lo);

	assign reload_i = ld_q;

	always_ff @(posedge clk)
		if (stimer)
			delay_q <= 3'd0;
		else if (reload_i)
			delay_q <= reload_delay;
		else if (clk179_en && holding)
			delay_q <= delay_q - 3'd1;

	// -----------------------------------------------------------------------
	// poly5 gating - the Cell 2 on the sheet
	// -----------------------------------------------------------------------
	// Cell 2 is not a sampler. Its R pin is where AUDC bit 7, NOTPOLY5, enters,
	// its D is the 5 bit poly, its Ld is this channel's timer, and its Q has
	// exactly one load: the NOR below, which is itself qualified by the same
	// timer pulse. So the cell cannot be observed outside the tick that opens
	// it - there is no later phase that sees the held value - and the cell IS
	// the poly5 / NOTPOLY5 combination rather than a stage in front of it.
	//
	// The sheet's poly5Out is taken through one extra inverter that the XNOR
	// feedback does not go through, so it is the complement of what
	// `pokey_poly.sv` emits. Hence `.d(~poly5)`, and the whole thing reduces to
	// `allow = notpoly5 | poly5`.
	//
	// Ld is Timer n, which on the sheet is a level lasting the rest of the
	// phase, not a pulse. `tick_win` is that level: it opens on the borrow and
	// closes at the next phase boundary. The cell is a plain register gated by
	// it, per the house rule at the top of `pokey_cells.sv`, so it is
	// transparent across the window with no combinational path from `d`.
	//
	// The window makes no observable difference at this site today, because
	// poly5 only moves on `clk179_en` and so is constant across it - a one
	// cycle enable measures the same. It is written as the level anyway,
	// because that is what the sheet draws and because the difference would
	// appear the moment anything reads the cell later in the window.
	logic tick_win;

	always_ff @(posedge clk)
		if (tick)
			tick_win <= 1'b1;
		else if (clk179_en)
			tick_win <= 1'b0;

	wire poly5_gate;

	pokey_cell2 u_poly5_gate (
		.clk, .ld(tick | tick_win), .d(~poly5), .p(1'b0), .r(notpoly5),
		.q(poly5_gate));

	// poly5 gates whether a transition is allowed at all.
	wire allow = ~poly5_gate;

	// The consumer runs one clk after the window opens, which is the die's
	// propagation delay and stays inside the same phase. The two poly sources
	// have to be sampled WITH the window rather than read at that point:
	// `clk179_en` lands on the same cycle as the borrow and advances them at
	// its end, so a cycle later they have already moved.
	logic tick_d, poly4_s, poly17_s;

	always_ff @(posedge clk) begin
		tick_d <= tick;
		if (tick) begin
			poly4_s  <= poly4;
			poly17_s <= poly17;
		end
	end

	// -----------------------------------------------------------------------
	// The audio flip flop
	// -----------------------------------------------------------------------
	// `stimer_level` follows CO12294 - high for channels 1 and 2, low for 3 and
	// 4 - and not Cwik's sheet, which comes out the other way round. Each link
	// of that sheet's chain has now been read and none of them is the inversion:
	//
	//   rstAudPhase is active HIGH. Addr9w sets the STIMER NOR latch, whose
	//   output falls, and reaches rstAudPhase through inverter "2", inverter "1"
	//   and one more inverter "2" - an odd count (300 dpi crop 9100,17700 and
	//   600 dpi crop 19100,17700 on page 5).
	//   Cell 11's IN is active high: page 7 draws its bottom gate as
	//   NOR(VOL ONLY, IN) onto the line that gates every volume transistor
	//   (300 dpi crops 3500,5300 and 3500,6700 on page 7).
	//   Cell 2's P must set, or channels 1 and 2 would not differ from 3 and 4
	//   at all.
	//
	// So the sheet says 3 and 4 present a one and 1 and 2 a zero. Closing that
	// needs Atari's own sheets in references/Pokey/die/ or a 7800 measurement,
	// not page 5. See Atari7800_MiSTer-syc.21.
	logic wave_q;

	always_ff @(posedge clk)
		if (stimer)
			wave_q <= stimer_level;
		else if (tick_d && allow) begin
			if (pure_tone)
				wave_q <= ~wave_q;       // square wave at half the divider rate
			else if (use_poly4)
				wave_q <= poly4_s;
			else
				wave_q <= poly17_s;      // poly9 when AUDCTL bit 7 selects it
		end

	assign out = wave_q;

	// -----------------------------------------------------------------------
	// High pass. Only channels 1 and 2 have one; the parent ties hp_enable low
	// for 3 and 4. The flop samples this channel's output at the paired
	// channel's divider rate and the XOR cancels whatever both agree on, so a
	// signal slower than the sampling rate mostly disappears.
	// -----------------------------------------------------------------------
	// The two are on OPPOSITE target phases on the sheet, and that separation is
	// the point. The audio flip flop moves on this channel's tick; the high
	// pass Cell 2 latches on the other phase. Altirra states the hardware
	// result outright: the high pass update is always half a clock early or
	// late, never simultaneous.
	//
	// Channels 1 and 3, or 2 and 4, can borrow on the same tick. Sampling
	// `hp_clock` directly put the latch and the waveform one `clk` cycle apart
	// instead of half a 1.79 MHz clock - a pulse roughly sixteen times too
	// short, which a phase-related pair can ride into near-total pass or
	// cancellation that silicon cannot reach. So the borrow is held and the
	// latch runs on the next `ph2_en`.
	//
	// The check is tb_hpphase, which runs eight clk per target clock. The other
	// audio suites run two, where half a target clock and one clk are the same
	// duration and the property has nowhere to show - a monitor written there
	// counted zero against both versions. Reverting this block to latch
	// straight off hp_clock fails tb_hpphase on every offset, including its
	// "locked pair leaks more than it cancels" check.
	logic hp_q, hp_pend;

	always_ff @(posedge clk) begin
		if (hp_clock)
			hp_pend <= 1'b1;
		else if (ph2_en)
			hp_pend <= 1'b0;

		if (!hp_enable)
			hp_q <= 1'b0;            // held clear, so the XOR passes `out` through
		else if (hp_pend && ph2_en)
			hp_q <= wave_q;
	end

	wire filtered = wave_q ^ hp_q;

	// -----------------------------------------------------------------------
	// Volume DAC
	// -----------------------------------------------------------------------
	pokey_cell11 u_dac (.vol(vol), .vol_only(vol_only), .in(filtered), .out(dac));

endmodule

`default_nettype wire
