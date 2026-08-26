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

// Adapter presenting the core's convenience port list on top of the
// pin-accurate `pokey` module, so cart.sv talks to a simple interface.
//
// What the convenience ports offer instead of the chip's pins: a combined
// write-enable instead of CS0/CS1/R-W, four separate channel outputs instead
// of one AUD node, a reset line the die does not have, and a single clock
// enable instead of two phases. All of that translation lives here, and
// `pokey.sv` stays honest.
//
// ---------------------------------------------------------------------------
// The three translations worth knowing about
// ---------------------------------------------------------------------------
// 1. Two phases, taken from the host rather than invented. POKEY's PHI2 pin is
//    wired to the CPU's phase 2 clock, so o2 is the CPU's phase 2 - the half
//    where a write's address and data are valid - and o1 is its phase 1. Both
//    enables come in.
//
//    Synthesizing o2 as the clk cycle after o1 does not work, even though the
//    two-phase logic only needs its phases ordered and non-overlapping. It is
//    the bus that cares: `pokey_bus.sv` samples the write row across the whole
//    o2 half and keeps the last value it saw, so a half that runs on past the
//    end of the CPU's cycle captures the next access instead. With o1 at pclk0
//    the halves straddle the cycle boundary and every register write is lost
//    or lands on the previous address.
//
// 2. WR_EN instead of a bus. This interface has no read strobe - DATA_OUT
//    simply always reflects the addressed register - so the part is held
//    selected and R/W follows WR_EN.
//
// 3. RESET_N. The die has no reset pin; init mode, entered by writing $00 to
//    SKCTL, is the only reset POKEY has. Rather than bolt a fake reset onto
//    `pokey`, this drives that write through the real pins for as long as
//    RESET_N is low, then writes $03 on the way out so the part is left
//    running. See the block comment on the bus below.

`default_nettype none

module pokey_adapter (
	input  wire        CLK,
	// The CPU's two phase enables, one clk cycle each: PHI1_EN is POKEY's o1
	// half, PHI2_EN its o2. cart.sv drives them from pclk1 and pclk0.
	input  wire        PHI1_EN,
	input  wire        PHI2_EN,
	input  wire  [3:0] ADDR,
	input  wire  [7:0] DATA_IN,
	input  wire        WR_EN,
	input  wire        RESET_N,

	// Keyboard, as ps2_to_pokey.v drives it: both sense lines active low.
	input  wire        keyboard_scan_enable,
	output wire  [5:0] keyboard_scan,
	input  wire  [1:0] keyboard_response,

	input  wire  [7:0] POT_IN,

	// SIO. The port list spreads the serial pins across ten ports; the real
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
	// Bus, and the boot sequence the adapter drives through it.
	//
	// While RESET_N is low the adapter writes $00 to SKCTL - init mode, the
	// only reset the part has. Then it writes $03 once on the way out, because
	// a part left in init makes no sound at all: init holds the 15 kHz and
	// 64 kHz dividers and all three polys, so every channel that is not
	// clocked straight off 1.79 MHz stops.
	//
	// An Atari 800 gets that second write from its OS. A 7800 cartridge has no
	// OS, and the die has no reset, so on real hardware POKEY simply comes up
	// in whatever state its latches settle to - "power-up state indeterminate"
	// in Atari's own documentation. Several released POKEY carts never write
	// SKCTL themselves (Cybernoid II, Beef Drop, the Galaxian sound demos) and
	// play correctly on hardware, so a part that comes up out of init is what
	// they were written against. Ending reset out of init is the deterministic
	// version of that.
	// -----------------------------------------------------------------------
	wire        in_reset = ~RESET_N;

	// Two POKEY cycles of the SKCTL $03 write once RESET_N releases - one to
	// carry the write through the bus's o1/o2 halves and one of margin. The
	// CPU is still fetching its reset vector, so nothing collides with it.
	logic [1:0] leaving;

	always_ff @(posedge CLK)
		if (in_reset)
			leaving <= 2'd2;
		else if (PHI1_EN && leaving != 2'd0)
			leaving <= leaving - 2'd1;

	wire        boot_wr = in_reset | (leaving != 2'd0);

	wire  [3:0] a_i    = boot_wr ? 4'hF : ADDR;
	wire  [7:0] d_in_i = boot_wr ? (in_reset ? 8'h00 : 8'h03) : DATA_IN;
	wire        rw_i   = boot_wr ? 1'b0 : ~WR_EN;

	wire  [7:0] d_out_i;
	wire        d_oe_i;
	wire        irq_oe_i;
	wire        sod_out_i, sod_oe_i, oclk_out_i, oclk_oe_i;
	wire        bclk_out_i, bclk_oe_i;

	// cart.sv leaves every SIO port unconnected, because a 7800 cartridge
	// has no serial link to POKEY. An unconnected input has no defined level,
	// so the serial pins are driven to their idle state here rather than from
	// the ports. If a caller ever does wire SIO up, take these from SIO_IN1 and
	// SIO_CLOCKIN_IN instead - the serial block itself is already complete.
	wire        sid_i  = 1'b1;    // idle high, so no start bit is ever seen
	wire        bclk_i = 1'b0;

	pokey u_pokey (
		.clk    (CLK),
		.ph1_en (PHI1_EN),
		.ph2_en (PHI2_EN),

		.a      (a_i),
		.cs0_n  (1'b0),         // held selected: this interface has no CS
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

	// This interface has no read phase: DATA_OUT is expected to hold the
	// addressed register for the whole cycle, while the real pads drive only
	// the o2 half of a selected read. So the pad value is qualified by the
	// chip's read select - the same term csRd forms inside - rather than by
	// d_oe, and a bus nobody drives reads high the way the 7800's does. Without
	// this, a read taken while the adapter is holding SKCTL down for RESET_N
	// would show whatever the internal bus happened to carry.
	wire   read_sel = ~boot_wr & ~WR_EN;

	assign DATA_OUT = read_sel ? d_out_i : 8'hFF;

	// IRQ is open drain with an external pull-up, so a released pin reads high.
	assign IRQ_N_OUT = ~irq_oe_i;

	// The port carries the dump transistor state; `pokey` keeps that internal,
	// so report the pot scan as never externally reset.
	assign POT_RESET = 1'b0;

	assign SIO_OUT1        = sod_out_i;
	assign SIO_OUT2        = 1'b1;
	assign SIO_OUT3        = 1'b1;
	// BCLK and OCLK are two pins, not one: SIO_CLOCKIN is the bi-directional
	// pin, SIO_CLOCKOUT the transmit clock.
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
