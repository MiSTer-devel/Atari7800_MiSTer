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

// Adapter presenting the old VHDL POKEY's port list on top of the pin-accurate
// `pokey` module, so rtl/cart.sv does not have to change when the swap happens.
//
// The VHDL this replaces exposed convenience ports rather than the chip's pins:
// a combined write-enable instead of CS0/CS1/R-W, four separate channel outputs
// instead of one AUD node, a reset line the die does not have, and a single
// clock enable instead of two phases. All of that translation lives here, and
// `pokey.sv` stays honest.
//
// ---------------------------------------------------------------------------
// The three translations worth knowing about
// ---------------------------------------------------------------------------
// 1. Two phases from one enable. The old port list carries only ENABLE_179.
//    o2 is generated here as the clk cycle after o1. The two-phase logic needs
//    the phases ordered and non-overlapping, which this gives; it does not need
//    them a real half period apart. clk_sys is far faster than 1.79 MHz so
//    there is always a cycle to spare.
//
// 2. WR_EN instead of a bus. The old interface has no read strobe - DATA_OUT
//    simply always reflects the addressed register - so the part is held
//    selected and R/W follows WR_EN.
//
// 3. RESET_N. The die has no reset pin; init mode, entered by writing $00 to
//    SKCTL, is the only reset POKEY has. Rather than bolt a fake reset onto
//    `pokey`, this drives that write through the real pins for as long as
//    RESET_N is low, which is exactly what an Atari OS does on boot.

`default_nettype none

module pokey_adapter (
	input  wire        CLK,
	input  wire        ENABLE_179,
	input  wire  [3:0] ADDR,
	input  wire  [7:0] DATA_IN,
	input  wire        WR_EN,
	input  wire        RESET_N,

	// Keyboard, as rtl/ps2_to_pokey.v drives it: both sense lines active low.
	input  wire        keyboard_scan_enable,
	output wire  [5:0] keyboard_scan,
	input  wire  [1:0] keyboard_response,

	input  wire  [7:0] POT_IN,

	// SIO. The old port list spread the serial pins across ten ports; the real
	// part has SID, SOD, BCLK and OCLK. Mapped through, with the spares tied.
	input  wire        SIO_IN1,
	input  wire        SIO_IN2,
	input  wire        SIO_IN3,
	output wire        SIO_OUT1,
	output wire        SIO_OUT2,
	output wire        SIO_OUT3,
	input  wire        SIO_CLOCKIN_IN,
	output wire        SIO_CLOCKIN_OUT,
	output wire        SIO_CLOCKIN_OE,
	output wire        SIO_CLOCKOUT,

	output wire  [7:0] DATA_OUT,
	output wire  [3:0] CHANNEL_0_OUT,
	output wire  [3:0] CHANNEL_1_OUT,
	output wire  [3:0] CHANNEL_2_OUT,
	output wire  [3:0] CHANNEL_3_OUT,
	// The single AUD node, with the measured DAC weighting and saturation.
	output wire [15:0] AUD,

	output wire        IRQ_N_OUT,
	output wire        POT_RESET
);
	// -----------------------------------------------------------------------
	// Phases
	// -----------------------------------------------------------------------
	logic ph2_q;

	always_ff @(posedge CLK)
		ph2_q <= ENABLE_179;

	wire ph1_en = ENABLE_179;
	wire ph2_en = ph2_q & ~ENABLE_179;   // never both in the same cycle

	// -----------------------------------------------------------------------
	// Bus. While RESET_N is low the adapter writes $00 to SKCTL, which is the
	// only reset the part has.
	// -----------------------------------------------------------------------
	wire        in_reset = ~RESET_N;

	wire  [3:0] a_i    = in_reset ? 4'hF    : ADDR;
	wire  [7:0] d_in_i = in_reset ? 8'h00   : DATA_IN;
	wire        rw_i   = in_reset ? 1'b0    : ~WR_EN;

	wire  [7:0] d_out_i;
	wire        d_oe_i;
	wire        irq_oe_i;
	wire        sod_out_i, sod_oe_i, oclk_out_i, oclk_oe_i;
	wire        bclk_out_i, bclk_oe_i;

	// rtl/cart.sv leaves every SIO port unconnected, because a 7800 cartridge
	// has no serial link to POKEY. An unconnected input has no defined level,
	// so the serial pins are driven to their idle state here rather than from
	// the ports. If a caller ever does wire SIO up, take these from SIO_IN1 and
	// SIO_CLOCKIN_IN instead - the serial block itself is already complete.
	wire        sid_i  = 1'b1;    // idle high, so no start bit is ever seen
	wire        bclk_i = 1'b0;

	pokey u_pokey (
		.clk    (CLK),
		.ph1_en (ph1_en),
		.ph2_en (ph2_en),

		.a      (a_i),
		.cs0_n  (1'b0),         // held selected: the old interface has no CS
		.cs1    (1'b1),
		.rw     (rw_i),
		.d_in   (d_in_i),
		.d_out  (d_out_i),
		.d_oe   (d_oe_i),

		.k      (keyboard_scan),
		.kr1    (keyboard_response[0]),
		.kr2    (keyboard_response[1]),

		.p      (POT_IN),

		.sid    (sid_i),
		.bclk   (bclk_i),
		.sod_out(sod_out_i),
		.sod_oe (sod_oe_i),
		.oclk_out(oclk_out_i),
		.oclk_oe(oclk_oe_i),
		.bclk_out(bclk_out_i),
		.bclk_oe(bclk_oe_i),

		.irq_n_out(),
		.irq_oe (irq_oe_i),

		.dac1   (CHANNEL_0_OUT),
		.dac2   (CHANNEL_1_OUT),
		.dac3   (CHANNEL_2_OUT),
		.dac4   (CHANNEL_3_OUT),
		.aud    (AUD));

	// The old interface has no read phase: DATA_OUT is expected to hold the
	// addressed register for the whole cycle, while the real pads drive only
	// the o2 half of a selected read. So the pad value is qualified by the
	// chip's read select - the same term csRd forms inside - rather than by
	// d_oe, and a bus nobody drives reads high the way the 7800's does. Without
	// this, a read taken while the adapter is holding SKCTL down for RESET_N
	// would show whatever the internal bus happened to carry.
	wire   read_sel = RESET_N & ~WR_EN;

	assign DATA_OUT = read_sel ? d_out_i : 8'hFF;

	// IRQ is open drain with an external pull-up, so a released pin reads high.
	assign IRQ_N_OUT = ~irq_oe_i;

	// The old port carried the dump transistor state; `pokey` keeps that
	// internal now, so report the pot scan as never externally reset.
	assign POT_RESET = 1'b0;

	assign SIO_OUT1        = sod_out_i;
	assign SIO_OUT2        = 1'b1;
	assign SIO_OUT3        = 1'b1;
	// BCLK and OCLK are two pins, not one. The old port list already had them
	// apart - SIO_CLOCKIN is the bi-directional pin, SIO_CLOCKOUT the transmit
	// clock - and the two were only tied together because `pokey` used to
	// expose a single clock output.
	assign SIO_CLOCKIN_OUT = bclk_out_i;
	assign SIO_CLOCKIN_OE  = bclk_oe_i;
	assign SIO_CLOCKOUT    = oclk_out_i;

`ifdef VERILATOR
	// A sink for Verilator's UNUSED check; Quartus warns 10036 on it instead.
	/* verilator lint_off UNUSED */
	wire _unused = &{1'b0, keyboard_scan_enable, d_oe_i,
	                 sod_oe_i, oclk_oe_i, SIO_IN1, SIO_IN2, SIO_IN3,
	                 SIO_CLOCKIN_IN};
	/* verilator lint_on UNUSED */
`endif

endmodule

`default_nettype wire
