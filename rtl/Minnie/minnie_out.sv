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

// Minnie's output section: what the analog spec asked a subcontractor for.
//
// The 10 bit sample is held for a whole 27.965 kHz tick, so what leaves the
// chip is a staircase with images at 28 kHz and above. On the real cartridge
// those were dealt with by a first or second order pole in the output buffer,
// then the RF modulator, then the television's own audio bandwidth. None of
// that exists here and the framework resamples to 48 kHz, so the images would
// fold back audibly if the sample were simply widened and handed over.
//
//   +-----------+     +-------------+     +---------------+
//   | DC block  | --> | 1 pole low  | --> | scale and     |
//   | ~17 Hz    |     | pass 4.4kHz |     | bias          |
//   +-----------+     +-------------+     +---------------+
//
// DC block: the 3600 board "capacitively couples the incoming signal presented
// on the external audio line" through 1 uF into 6.8 kohms, which is a 23 Hz
// high pass. Modelling it matters for more than tidiness - waveform 4 outputs
// -1 rather than 0, and the square wave sits 64 counts low, so without it every
// volume change would step the DC level.
//
// Low pass: the specification asks for "approximately 4 KHz", first or second
// order. One pole at 4.4 kHz, run at the processor clock rather than the sample
// rate, because a reconstruction filter has to run faster than the staircase it
// is smoothing.
//
// Scale and bias: the core's mixer is unsigned, so the result is re-biased to
// sit mid range. That is not a fudge - the real output pin sourced current into
// an AC coupled load, and the analog spec says so: "allowing the output pin to
// be biased up to any level desired".
//
// Both filters are shift-and-accumulate, no multiplier and no divider, with
// fractional bits carried so the truncation cannot stall them.

`default_nettype none

module minnie_out #(
	parameter int DC_SHIFT  = 8,     // at 27.965 kHz -> ~17 Hz corner
	parameter int LP_SHIFT  = 6,     // at 1.79 MHz   -> ~4.4 kHz corner
	parameter int OUT_SHIFT = 5      // 10 bit sample -> +/-32768 of swing
) (
	input  wire               clk,
	input  wire               reset,
	input  wire               ph1_en,
	input  wire               sample_en,
	input  wire  signed [9:0] sample,

	output wire        [15:0] aud
);

	// 17 bits: the bias is 32768, which a signed 16 bit value cannot hold.
	localparam signed [16:0] OUT_BIAS = 17'sd32768;

	// --- DC block, at the sample rate ------------------------------------
	wire signed [17:0] samp_wide = {sample, {DC_SHIFT{1'b0}}};

	logic signed [17:0] dc_acc;

	// The difference needs one bit more than either operand: both span the full
	// +/-131072, and at 18 bits the subtract wraps as soon as the sample sits
	// more than half scale from its own running mean - a loud bass note does
	// it. The wrapped increment then carries the wrong sign and walks dc_acc to
	// the opposite rail, where it stays.
	wire signed [18:0] dc_diff = 19'($signed(samp_wide)) - 19'($signed(dc_acc));

	always_ff @(posedge clk) begin
		if (reset)
			dc_acc <= 18'sd0;
		else if (sample_en)
			dc_acc <= dc_acc + 18'(dc_diff >>> DC_SHIFT);
	end

	// +/-512 minus +/-512, so eleven bits and no wider. Both operands are cast
	// before the subtract: a concatenation is unsigned in Verilog, and one
	// unsigned operand would make the whole expression unsigned and mangle
	// every negative sample.
	wire signed [10:0] sample_ext = 11'($signed(sample));
	wire signed [10:0] dc_ext     = 11'($signed(dc_acc[17:DC_SHIFT]));
	wire signed [10:0] hp         = sample_ext - dc_ext;

	// --- One pole low pass, at the processor clock ------------------------
	wire signed [16:0] hp_wide = {hp, {LP_SHIFT{1'b0}}};

	logic signed [16:0] lp_acc;

	// Widened for the same reason as the DC blocker above.
	wire signed [17:0] lp_diff = 18'($signed(hp_wide)) - 18'($signed(lp_acc));

	always_ff @(posedge clk) begin
		if (reset)
			lp_acc <= 17'sd0;
		else if (ph1_en)
			lp_acc <= lp_acc + 17'(lp_diff >>> LP_SHIFT);
	end

	wire signed [10:0] lp_out = lp_acc[16:LP_SHIFT];

	// --- Scale and bias ---------------------------------------------------
	// hp is bounded at +/-1023, so this lands inside 32..65504: the whole of
	// the unsigned mixer's range, without underflowing it.
	//
	// That is as loud as the part can be. top.sv sums into 17 bits and
	// halves the result whenever an external audio source is present, and it
	// assumes one such source at a time - TIA's 32767 plus this 65504 is
	// 98271, inside the 131071 the sum holds. A cart running POKEY and Minnie
	// together would break that assumption, which is the same assumption
	// top.sv already documents for covox plus YM2151.
	wire signed [16:0] scaled = 17'($signed(lp_out) <<< OUT_SHIFT);

	assign aud = 16'(scaled + OUT_BIAS);

endmodule

`default_nettype wire
