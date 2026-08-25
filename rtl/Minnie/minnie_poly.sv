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

// Minnie's noise source: the "15 stage poly counter" of GCC-1730 section 2.1.3.
//
// The specification gives the length and nothing else - no taps, no clock rate,
// no seed. Those are invented here; see .agents/references/minnie.md.
//
//   taps    x^15 + x^14 + 1, maximal length, period 32767. Two gate feedback,
//           the same idiom POKEY uses for its 17 and 9 bit polys.
//   clock   once per microcode state, not once per sample. The three voices
//           read it at different states, so this is what decorrelates them.
//           Clocking per sample would give all three the same value.
//   seed    all ones. XOR feedback locks up at all zeros, so the seed must not
//           be zero and reset must not clear it.

`default_nettype none

module minnie_poly (
	input  wire        clk,
	input  wire        step_en,    // one pulse per microcode state
	input  wire        rst,        // reset, or CREG POLYRST

	output wire [14:0] poly
);

	logic [14:0] shifter;

	always_ff @(posedge clk) begin
		if (rst)
			shifter <= 15'h7FFF;
		else if (step_en)
			shifter <= {shifter[13:0], shifter[14] ^ shifter[13]};
	end

	assign poly = shifter;

endmodule

`default_nettype wire
