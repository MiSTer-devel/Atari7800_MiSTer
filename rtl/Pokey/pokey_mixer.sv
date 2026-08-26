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

// The AUD node: what the four Cell 11 DACs add up to.
//
// Source: PokeyReSchem-13.pdf page 5 for the
// structure, and Altirra's hardware reference manual Appendix E, "Analog Audio
// Model", for the numbers - which are measurements taken from real hardware
// rather than a model fitted to a datasheet.
//
// On the die there is no adder. Each channel's Cell 11 steers current into the
// single AUD pin through four binary weighted transistors, and the pin sums
// those currents in the analog domain. The spec calls it "a crude 4 bit digital
// to analog converter", and crude is the operative word.
//
// Two departures from a linear sum, both measured, both implemented here.
//
// ---------------------------------------------------------------------------
// 1. The volume bits are not a clean 1:2:4:8
// ---------------------------------------------------------------------------
// Steady-state levels from a single channel give voltage drops of 0.12, 0.26,
// 0.56 and 1.12 V, each within about 0.01 V. Nominal binary weighting would be
// 0.14, 0.28, 0.56, 1.12 against those. The gap between bit 1 and bit 2 is the
// most pronounced and is visible in volume ramps.
//
// Held here in hundredths of a volt, so a channel spans 0..206 and the four
// together 0..824. That 824 is Appendix E's own normalising constant,
// (0.12 + 0.26 + 0.56 + 1.12) x 4 = 8.24 V.
//
// ---------------------------------------------------------------------------
// 2. The sum saturates
// ---------------------------------------------------------------------------
// The output is roughly linear to a total level of about 12, then compresses;
// distortion is audible around 30 and above. Appendix E's hand-fitted curve,
// with input and output both normalised to 0..1:
//
//     y = 2.171 x                                          x <= 0.14
//     y = 2.171 (0.14 + (1 - e^(-2.85 (x - 0.14))) / 2.85) x >= 0.14
//
// **The effect applies to the sum, never to a channel on its own**, so it
// cannot be folded into the per-channel volumes: what it does depends on what
// the other three channels are doing at that instant. That is exactly why the
// linear sum this replaces could not be corrected by rescaling.
//
// Implemented as 26 linear segments over the 0..824 range with 5 bits of
// interpolation, which tracks the curve to 95 parts in 65535 - 0.14%, well
// under anything audible - and needs no ROM, no exponential and one small
// multiply. The breakpoints are the curve evaluated at every 32nd raw step.
//
// Note the sign. On the real pin all channels off is +5 V and rising output
// pulls the voltage down. The core's audio path is unsigned with silence at
// zero, so `aud` rises with output level; that is a whole-signal inversion, and
// it is inaudible because the path is AC coupled.

`default_nettype none

module pokey_mixer (
	input  wire [3:0]  dac1,
	input  wire [3:0]  dac2,
	input  wire [3:0]  dac3,
	input  wire [3:0]  dac4,
	output wire [15:0] aud
);
	// Measured drops, hundredths of a volt. Not 1:2:4:8.
	function automatic logic [7:0] weigh(input logic [3:0] d);
		weigh = (d[0] ? 8'd12  : 8'd0)
		      + (d[1] ? 8'd26  : 8'd0)
		      + (d[2] ? 8'd56  : 8'd0)
		      + (d[3] ? 8'd112 : 8'd0);
	endfunction

	// 0..824, so ten bits.
	wire [9:0] raw = {2'b00, weigh(dac1)} + {2'b00, weigh(dac2)}
	               + {2'b00, weigh(dac3)} + {2'b00, weigh(dac4)};

	// Appendix E's curve at every 32nd step of `raw`, scaled to 0..65535.
	// Written as a case rather than an unpacked-array localparam because
	// Icarus rejects the latter outright and Quartus 17.0.2's SystemVerilog
	// support is partial.
	function automatic logic [15:0] curve(input logic [4:0] i);
		case (i)
			5'd0 : curve = 16'd0;
			5'd1 : curve = 16'd5525;
			5'd2 : curve = 16'd11051;
			5'd3 : curve = 16'd16576;
			5'd4 : curve = 16'd22054;
			5'd5 : curve = 16'd27061;
			5'd6 : curve = 16'd31543;
			5'd7 : curve = 16'd35556;
			5'd8 : curve = 16'd39148;
			5'd9 : curve = 16'd42364;
			5'd10: curve = 16'd45242;
			5'd11: curve = 16'd47820;
			5'd12: curve = 16'd50127;
			5'd13: curve = 16'd52192;
			5'd14: curve = 16'd54041;
			5'd15: curve = 16'd55697;
			5'd16: curve = 16'd57179;
			5'd17: curve = 16'd58505;
			5'd18: curve = 16'd59693;
			5'd19: curve = 16'd60756;
			5'd20: curve = 16'd61708;
			5'd21: curve = 16'd62560;
			5'd22: curve = 16'd63323;
			5'd23: curve = 16'd64006;
			5'd24: curve = 16'd64617;
			5'd25: curve = 16'd65164;
			5'd26: curve = 16'd65535;
			default: curve = 16'd65535;
		endcase
	endfunction

	wire [4:0]  seg  = raw[9:5];         // at most 25, so seg + 1 is in range
	wire [4:0]  frac = raw[4:0];
	wire [15:0] lo   = curve(seg);
	wire [15:0] hi   = curve(seg + 5'd1);

	// The curve rises and no segment spans more than 5525, so the difference
	// fits in 13 bits and is never negative. Thirteen bits by five.
	wire [12:0] span = hi[12:0] - lo[12:0];
	wire [17:0] step = span * {8'd0, frac};

`ifdef VERILATOR
	// A sink for Verilator's UNUSED check; Quartus warns 10036 on it instead.
	/* verilator lint_off UNUSEDSIGNAL */
	wire _unused = &{1'b0, hi[15:13], step[4:0]};
	/* verilator lint_on UNUSEDSIGNAL */
`endif

	assign aud = lo + {3'd0, step[17:5]};

endmodule

`default_nettype wire
