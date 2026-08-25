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

// POKEY polynomial counters: the 4 bit, the 5 bit, and the 17/9 bit.
//
// Source: references/Pokey/schematics/PokeyReSchem-13.pdf page 5, left side.
//
// ---------------------------------------------------------------------------
// What the schematic shows
// ---------------------------------------------------------------------------
// Each poly is a two-phase dynamic shift register. A bit is an inverter marked
// "2" followed by one marked "1", the o2 stage then the o1 stage, so a pair of
// triangles on the drawing is one bit. Feedback is an XNOR, drawn with the
// double curved input, not an XOR:
//
//   +--> [2][1] --> [2][1] --> [2][1] --> [2][1] --> poly4Out
//   |                              |          |
//   |                              +--XNOR----+
//   +------------------------------+
//
// The 4 bit taps stages 3 and 4. That reading is confirmed twice over: MAME's
// poly_init_4_5 XNORs exactly those two taps, and the AtariAge tap thread in
// references/Pokey/forum gives 3 and 5 for the 5 bit. Three independent
// sources agreeing is the strongest evidence position in this rewrite.
//
// The feedback polarity differs between the polys, and so does the state Init
// parks them in. Getting this backwards leaves a counter stuck forever.
//
//   4 and 5 bit   XNOR feedback, Init parks the register at zero
//   17/9 bit      XNOR feedback, Init seeds it and then holds bit 16 low so it
//                 drains to zero; the RANDOM drivers invert, hence $FF
//
// The spec's "the counter is set to all ones state, therefore the CPU will
// read $FF" is about the 17 bit, which RANDOM reads. It does not describe the
// 4 and 5 bit, and applying it to them wedges both.
//
// ---------------------------------------------------------------------------
// The 17 bit is not a plain shift register
// ---------------------------------------------------------------------------
// It shifts right, and the feedback is injected into the middle of the chain
// rather than at the end:
//
//   bit0 -> wraps to bit16
//   bit8 ^ bit13 -> inserted at bit7
//
// That mid-chain injection is why RANDOM's "high order 8 bits" is bits 15..8
// and not the last eight stages. The 9 bit variant is the same idea with the
// feedback at bit0 ^ bit5 landing at bit8.
//
// From the CO12294 spec, confirmed on the page image rather than through the
// OCR: out of init the counter is all ones and the CPU reads $FF from RANDOM.
//
// The polys are clocked at the full 1.79 MHz, always. The audio channels
// sample them at their own divider rate, which is what makes each channel
// appear to own a poly running at its frequency. Getting this backwards - and
// clocking the polys per channel - is the single most common POKEY error.

`default_nettype none

module pokey_poly (
	input  wire        clk,
	input  wire        poly_en,     // 1.79 MHz enable, one clk pulse per tick
	input  wire        init,        // SKCTL init: park the polys, see above
	input  wire        poly9_sel,   // AUDCTL bit 7: shorten the 17 bit to 9
	input  wire        random_rd,   // AddrAr, the read strobe at address $0A

	output wire        poly4_out,
	output wire        poly5_out,
	output wire        poly17_out,
	output wire [7:0]  random       // this sheet's pull on the read bus
);
	// -----------------------------------------------------------------------
	// 4 bit: shift left, XNOR of stages 3 and 4 into stage 1
	// -----------------------------------------------------------------------
	logic [3:0] poly4;

	always_ff @(posedge clk)
		if (init)
			poly4 <= 4'h0;     // all ones is this poly's lock-up state
		else if (poly_en)
			poly4 <= {poly4[2:0], ~(poly4[2] ^ poly4[3])};

	assign poly4_out = poly4[3];

	// -----------------------------------------------------------------------
	// 5 bit: shift left, XNOR of stages 3 and 5
	// -----------------------------------------------------------------------
	logic [4:0] poly5;

	always_ff @(posedge clk)
		if (init)
			poly5 <= 5'h00;    // likewise
		else if (poly_en)
			poly5 <= {poly5[3:0], ~(poly5[2] ^ poly5[4])};

	assign poly5_out = poly5[4];

	// -----------------------------------------------------------------------
	// 17 bit, collapsing to 9 bit under AUDCTL bit 7
	// -----------------------------------------------------------------------
	// One register serves both lengths: bit 16 is fed either from the feedback
	// (short loop, 9 bit) or from bit 0 (long loop, 17 bit). Feedback is XNOR
	// of bits 13 and 8, injected at bit 7, and the shift is to the right.
	//
	// The output is NOT the end of the chain. It is bit 9, held one enable in a
	// separate flop. The VHDL this replaces takes the same tap and annotates it
	// "from pokey schematics"; page 5 shows poly17Out coming off a node part
	// way along the chain through an inverter pair, not off the last stage.
	// Taking bit 0 instead leaves the noise stream a fixed phase out, which is
	// what the GHDL differential was showing.
	//
	// RANDOM is bits 15..8 through the driver row below, which inverts. With
	// XNOR feedback the register settles to all zeros while init holds bit 16
	// low, so RANDOM reads $FF - which is what the spec says the CPU sees out
	// of init.
	logic [16:0] shift;
	logic        cycle_delay;
	logic        sel9_del;

	wire feedback = ~(shift[13] ^ shift[8]);

	// Init is a level. Its leading edge seeds the register; while it stays high
	// the register keeps shifting with bit 16 held low, so it drains to zero.
	logic init_q;
	always_ff @(posedge clk)
		init_q <= init;

	always_ff @(posedge clk)
		if (init && !init_q) begin
			shift       <= 17'b0_1010_1010_1010_1010;
			cycle_delay <= 1'b0;
			sel9_del    <= 1'b0;
		end else if (poly_en) begin
			sel9_del     <= poly9_sel;
			shift[15:8]  <= shift[16:9];
			shift[7]     <= feedback;
			shift[6:0]   <= shift[7:1];
			shift[16]    <= ((feedback & sel9_del) | (shift[0] & ~poly9_sel)) & ~init;
			cycle_delay  <= shift[9];
		end

	assign poly17_out = cycle_delay;

	// -----------------------------------------------------------------------
	// RANDOM: a row of eight Cell 9 onto the precharged read bus
	// -----------------------------------------------------------------------
	// Page 5 draws AddrAr entering the row at the left as Rd and passing across
	// it, In coming down from the 17 bit register, and each box's Q driving
	// D7r..D0r (300 dpi -f 5 -x 400 -y 3980 -W 3200 -H 620). The cell is the
	// pull down, so it inverts on its own: feed it the true bit and the bus
	// carries the complement. Complementing here as well would read $00.
	wire [7:0] random_q, random_oe;

	genvar i;
	generate
		for (i = 0; i < 8; i = i + 1) begin : g_random
			pokey_cell9 u (
				.in  (shift[8+i]),
				.rd  (random_rd),
				.q_n (random_q[i]),
				.q_oe(random_oe[i]));
		end
	endgenerate

	// A bit no driver pulls down stays at the precharged one.
	assign random = random_q | ~random_oe;

endmodule

`default_nettype wire
