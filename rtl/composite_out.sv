// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

// The 7800 and 2600 video output stage: colour and luminance codes in, baseband
// composite out, in volts.
//
// Neither chip has a composite pin. Each drives LUM lines, a SYNC line and a
// COLOR pin, and the board sums them (Atari drawing C025231-001, zone B/C-2):
//
//   MSYNC | TSYNC-  --U3--  R10  8.2K --+
//   MLUM3 | TLUM2   --U3--  R13   10K --+
//   MLUM2 | TLUM1   --U3--  R12   20K --+--o video node --- C7 47pF -- RF1
//   MLUM1 | TLUM0   --U3--  R11   39K --+  |                \-- C9 47pF, R3 1K
//   MLUM0           -------  R15   82K --+  R14 7.5K            to the 4.5 MHz
//                                          |                   sound oscillator
//   MCOL -- R16 4.3K --+                  gnd
//   TCOL -- R17 4.7K --+-- R2 470 to gnd, L1 47uH + C8 47pF series --o node
//
// U3 is a quad OR gate, so the two chips' luma bits and sync are OR'd and
// their chroma summed: the idle chip must rest its lines low, which MARIA does
// when disabled and the TIA does while blanked. This module takes one chip's
// codes at a time, selected upstream, which is the same node voltage under
// that condition.
//
// Colour itself is made inside MARIA: `tia-maria.pdf` page 47, COLOR DELAY
// LINE & LUM, is a delay chain feeding a crossbar with fifteen columns, one
// per colour code, out to the COLOR pin. Colour is a phase, not a level, which
// is why artifact colour exists at all. The line's delay is set by an on-chip
// RC reference, and an RC bias drifts with die temperature - the physical
// cause of the warm / cool / hot palettes.
//
// Sampled at 4x the subcarrier, which is clk_sys: four samples per 2600 pixel,
// two per MARIA pixel. That is enough for both, because the decoder's notch
// removes everything above the subcarrier anyway. Feed composite_decoder with
// SPC = 4.

module composite_out (
	input               clk,        // 4x subcarrier

	// Per-chip, because one instance serves both, as the board's one node does.
	input         [9:0] burst_on,   // burst window, samples after HSync falls
	input         [9:0] burst_off,
	input         [8:0] chroma_scale, // 256 = the tables as they are; TIA 236,
	                                  // being injected through R17 4.7K against
	                                  // MARIA's R16 4.3K
	input         [1:0] pix_samples,  // 2 for MARIA at 2*fsc, 0 (=4) for TIA
	input         [1:0] phase_adj,    // which of the four alignments the dividers
	                                  // allow: each turns artifact hue 90 degrees,
	                                  // and it is settled against a hardware capture
	input         [3:0] col,        // colour code; 0 is no chroma
	input         [3:0] lum,        // TIA passes {lum, 1'b0}: it has three bits, MARIA four
	input               blank,
	input               hsync,      // positive pulse; times the burst
	input               sync,       // the SYNC pin, asserted high here. MARIA's
	                                // is hsync ^ vsync, the TIA's hsync | vsync
	// High on the first sample of each source pixel. The carrier is aligned to
	// this rather than left to free-run, because where the pixel grid sits
	// against the subcarrier is a property of the chip's dividers - the pixel
	// clock is a whole multiple of fsc - not of where reset happened to fall.
	// The burst cannot supply it: the burst fixes the COLOUR reference, which
	// is why steady colours do not move with phase_adj, only artifacts do.
	input               pix_tick,
	input         [1:0] temp,       // 0 warm, 1 cool, 2 hot

	output logic signed [23:0] comp, // Q2.21 volts, 1.0 = 1.0V
	output logic        sample_tog  // toggles per sample, for the CDC
);

// The COLOR pin is a logic-level 3.58 MHz clock, phase-delayed per colour code
// by the on-chip delay line, and the burst is the same clock at zero delay
// during horizontal blank (GCC's MARIA specification; Colour Kill parks the
// pin at logic 1). So every colour has the burst's amplitude, and only the
// angle is the code's. The angles are NOT from any drawing: page 47 shows the
// delay line and the crossbar but not the tap spacing. They are the measured
// mean angle per code over the luma rows least affected by clipping, per
// palette, mirrored and turned 123.5 degrees into the broadcast frame - burst
// on the -sin axis, I and Q at their NTSC angles from it - from a fit that had
// been made against a decoder with its I axis opposite the burst. The angles'
// spacing is worth recording:
// Fitting a line to the fifteen hue angles gives 25.97, 26.95 and 27.93 degrees
// per colour code for the cool, warm and hot palettes, against the 25.7 / 26.7 /
// 27.7 that `rtl/video_mux.sv` documents - within a quarter degree on all
// three. Temperature moves the delay line's degrees per tap and nothing else,
// exactly as an RC-biased chain would. The nominal for fifteen taps closing one
// cycle would be 24.0 degrees; every measured palette says the real line runs
// fast.

// Burst and colour amplitude at the source, ahead of the node: the COLOR
// pin's square wave, 4.5 V, divided by R16 into R2 || R17 (0.0904) and taken
// at its fundamental (2/pi of the swing), then scaled with the luma so white
// is 0.714 V: 0.137 V. The pin's swing is not on any drawing; 4.5 V is what
// puts Tower Toppler's dithered brick at the 0.29 of a solid colour the
// hardware capture measures, and is a plausible NMOS high at this load.
localparam signed [23:0] BURST_AMP = 24'sd287090;

// Luma and sync are the board's ladder. Each leg's share of the node is its
// conductance over the node's total, R14 included: SYNC 0.2752, LUM3 0.2257,
// LUM2 0.1128, LUM1 0.0579, LUM0 0.0275, Thevenin 2.26K. The U3 legs swing
// a 74LS32's typical 3.4 V high to 0.25 V low at the fraction of a
// milliamp these resistors draw; MARIA's own LUM0 driver swings 1.18 times
// that, the ratio a two-swing fit to the palette greys gives (rms 0.022 V),
// since its level is not on the drawing. The node then runs about 1.35 V
// blank to white with sync 0.87 V below blank; everything is scaled so white
// is the broadcast 0.714 V, which leaves sync at 0.458 V, 0.64 of white where
// broadcast puts 0.40 - and that ratio stands whatever the TTL swing is,
// since sync and LUM3 come from the same gate. Blanking is all legs low with
// sync high, and is the zero here.
localparam signed [23:0] LUM_W3    = 24'sd787958;
localparam signed [23:0] LUM_W2    = 24'sd393979;
localparam signed [23:0] LUM_W1    = 24'sd202041;
localparam signed [23:0] LUM_W0    = 24'sd113389;
localparam signed [23:0] SYNC_LEVEL = -24'sd960924;

function automatic signed [23:0] ld(input [3:0] l);
	ld = (l[3] ? LUM_W3 : 24'sd0) + (l[2] ? LUM_W2 : 24'sd0) +
	     (l[1] ? LUM_W1 : 24'sd0) + (l[0] ? LUM_W0 : 24'sd0);
endfunction

// At four samples per cycle the carrier visits +sin, +cos, -sin, -cos and
// nothing else, so two constants per colour are the whole waveform.
function automatic signed [23:0] cs(input [5:0] a);
	case (a)
		6'd0 : cs = 24'sd0;
		6'd1 : cs = 24'sd98593;
		6'd2 : cs = 24'sd182352;
		6'd3 : cs = 24'sd247746;
		6'd4 : cs = 24'sd287020;
		6'd5 : cs = 24'sd248600;
		6'd6 : cs = 24'sd145666;
		6'd7 : cs = 24'sd2153;
		6'd8 : cs = -24'sd136853;
		6'd9 : cs = -24'sd218340;
		6'd10: cs = -24'sd267854;
		6'd11: cs = -24'sd286440;
		6'd12: cs = -24'sd225101;
		6'd13: cs = -24'sd95451;
		6'd14: cs = 24'sd41973;
		6'd15: cs = 24'sd144832;
		6'd16: cs = 24'sd0;
		6'd17: cs = 24'sd98593;
		6'd18: cs = 24'sd179386;
		6'd19: cs = 24'sd242950;
		6'd20: cs = 24'sd286143;
		6'd21: cs = 24'sd259906;
		6'd22: cs = 24'sd168110;
		6'd23: cs = 24'sd36190;
		6'd24: cs = -24'sd105739;
		6'd25: cs = -24'sd199359;
		6'd26: cs = -24'sd253874;
		6'd27: cs = -24'sd285197;
		6'd28: cs = -24'sd261514;
		6'd29: cs = -24'sd157235;
		6'd30: cs = -24'sd23299;
		6'd31: cs = 24'sd95745;
		6'd32: cs = 24'sd0;
		6'd33: cs = 24'sd98593;
		6'd34: cs = 24'sd184452;
		6'd35: cs = 24'sd252811;
		6'd36: cs = 24'sd286799;
		6'd37: cs = 24'sd235357;
		6'd38: cs = 24'sd119962;
		6'd39: cs = -24'sd35945;
		6'd40: cs = -24'sd165265;
		6'd41: cs = -24'sd236066;
		6'd42: cs = -24'sd279618;
		6'd43: cs = -24'sd274467;
		6'd44: cs = -24'sd172414;
		6'd45: cs = -24'sd27975;
		6'd46: cs = 24'sd101219;
		6'd47: cs = 24'sd185786;
		6'd48: cs = 24'sd0;
		6'd49: cs = 24'sd98593;
		6'd50: cs = 24'sd182352;
		6'd51: cs = 24'sd247746;
		6'd52: cs = 24'sd287020;
		6'd53: cs = 24'sd248600;
		6'd54: cs = 24'sd145666;
		6'd55: cs = 24'sd2153;
		6'd56: cs = -24'sd136853;
		6'd57: cs = -24'sd218340;
		6'd58: cs = -24'sd267854;
		6'd59: cs = -24'sd286440;
		6'd60: cs = -24'sd225101;
		6'd61: cs = -24'sd95451;
		6'd62: cs = 24'sd41973;
		6'd63: cs = 24'sd144832;
		default: cs = 24'sd0;
	endcase
endfunction

function automatic signed [23:0] cc(input [5:0] a);
	case (a)
		6'd0 : cc = 24'sd0;
		6'd1 : cc = -24'sd269630;
		6'd2 : cc = -24'sd221739;
		6'd3 : cc = -24'sd145060;
		6'd4 : cc = -24'sd6332;
		6'd5 : cc = 24'sd143593;
		6'd6 : cc = 24'sd247390;
		6'd7 : cc = 24'sd287082;
		6'd8 : cc = 24'sd252372;
		6'd9 : cc = 24'sd186409;
		6'd10: cc = 24'sd103320;
		6'd11: cc = -24'sd19304;
		6'd12: cc = -24'sd178186;
		6'd13: cc = -24'sd270758;
		6'd14: cc = -24'sd284005;
		6'd15: cc = -24'sd247880;
		6'd16: cc = 24'sd0;
		6'd17: cc = -24'sd269630;
		6'd18: cc = -24'sd224146;
		6'd19: cc = -24'sd152957;
		6'd20: cc = -24'sd23293;
		6'd21: cc = 24'sd121940;
		6'd22: cc = 24'sd232723;
		6'd23: cc = 24'sd284800;
		6'd24: cc = 24'sd266908;
		6'd25: cc = 24'sd206584;
		6'd26: cc = 24'sd134047;
		6'd27: cc = 24'sd32918;
		6'd28: cc = -24'sd118452;
		6'd29: cc = -24'sd240204;
		6'd30: cc = -24'sd286143;
		6'd31: cc = -24'sd270654;
		6'd32: cc = 24'sd0;
		6'd33: cc = -24'sd269630;
		6'd34: cc = -24'sd219996;
		6'd35: cc = -24'sd136041;
		6'd36: cc = 24'sd12928;
		6'd37: cc = 24'sd164401;
		6'd38: cc = 24'sd260825;
		6'd39: cc = 24'sd284831;
		6'd40: cc = 24'sd234751;
		6'd41: cc = 24'sd163381;
		6'd42: cc = 24'sd65073;
		6'd43: cc = -24'sd84195;
		6'd44: cc = -24'sd229552;
		6'd45: cc = -24'sd285724;
		6'd46: cc = -24'sd268655;
		6'd47: cc = -24'sd218870;
		6'd48: cc = 24'sd0;
		6'd49: cc = -24'sd269630;
		6'd50: cc = -24'sd221739;
		6'd51: cc = -24'sd145060;
		6'd52: cc = -24'sd6332;
		6'd53: cc = 24'sd143593;
		6'd54: cc = 24'sd247390;
		6'd55: cc = 24'sd287082;
		6'd56: cc = 24'sd252372;
		6'd57: cc = 24'sd186409;
		6'd58: cc = 24'sd103320;
		6'd59: cc = -24'sd19304;
		6'd60: cc = -24'sd178186;
		6'd61: cc = -24'sd270758;
		6'd62: cc = -24'sd284005;
		6'd63: cc = -24'sd247880;
		default: cc = 24'sd0;
	endcase
endfunction

// Free-running, never reset at hsync. Both chips' lines are a whole number of
// subcarrier cycles - 228 colour clocks on the 2600, 454 master clocks on MARIA
// - so the carrier arrives at the same phase every line and an artifact pattern
// holds one hue down the column. Two instances of this module in the same clock
// domain therefore stay in lockstep from power-up, which is what lets the two
// composites be selected between downstream.
logic [1:0] phase;
logic [1:0] pixph;              // carrier phase owed to the next pixel boundary
wire  [1:0] cphase = phase + phase_adj;
logic       hs_d;
logic [9:0] hcnt;

wire in_burst = (hcnt >= burst_on) && (hcnt < burst_off);

wire [5:0] idx = {temp, col};
/* verilator lint_off UNUSEDSIGNAL */
wire signed [32:0] cs_m = cs(idx) * $signed({1'b0, chroma_scale});
wire signed [32:0] cc_m = cc(idx) * $signed({1'b0, chroma_scale});
/* verilator lint_on UNUSEDSIGNAL */
wire signed [23:0] chroma_s = cs_m[31:8];
wire signed [23:0] chroma_c = cc_m[31:8];

logic signed [23:0] chroma;
logic signed [23:0] luma;      // the ladder's node: luma legs and the sync leg

always_comb begin
	// Burst rides at 180 degrees, the reference composite_decoder divides out.
	if (in_burst) begin
		luma = 24'sd0;
		case (cphase)
			2'd0:    chroma =  24'sd0;
			2'd1:    chroma = -BURST_AMP;
			2'd2:    chroma =  24'sd0;
			default: chroma =  BURST_AMP;
		endcase
	end
	else if (blank) begin
		luma   = 24'sd0;
		chroma = 24'sd0;
	end
	else begin
		luma = ld(lum);
		case (cphase)
			2'd0:    chroma =  chroma_s;
			2'd1:    chroma =  chroma_c;
			2'd2:    chroma = -chroma_s;
			default: chroma = -chroma_c;
		endcase
	end
	if (sync) luma = luma + SYNC_LEVEL;
end

// The video node, as one linear network. Two sources drive it: the ladder
// through its Thevenin 2.26K, and the COLOR pin through R16 into R2 and the
// series L1 47uH / C8 47pF. Hanging on it: the C9 47pF / R3 1K path to the
// sound oscillator and about 10 pF of stray. The modulator's own input, C7
// and whatever sits behind it, is left off: every capture this is checked
// against came from a console with the modulator out of circuit.
//
//   V(s) = [ Vl/Rth + Vc/Zlc(s) ] / Y(s)
//   Y(s) = 1/Rth + 1/Zlc(s) + 1/(R3 + 1/sC9) + sCs
//
// The LC branch is the point: at 3.4 MHz it is 390 ohms against the ladder's
// 2.26K, so luma is trapped to 0.13 at the subcarrier while chroma arrives at
// 0.74. A one-pixel 320-mode alternation therefore decodes at about 0.6 of a
// solid colour, and a two-pixel one carries almost none, which is the order
// hardware shows. Luma comes through at 1.0 at DC, 0.37 at 1.79 MHz.
//
// Bilinear transform at four samples a cycle, prewarped at the subcarrier so
// the trap and the chroma peak land exactly there. Both inputs share the
// poles, so each gets a five-tap numerator and the sum runs through two
// all-pole sections. Q14 coefficients, from .agents/tools/composite/node_design.py.
localparam signed [15:0] NL0 =  16'sd4537, NL1 =  16'sd3990, NL2 =  16'sd2527, NL3 =  16'sd2989, NL4 = -16'sd85;
localparam signed [15:0] NC0 =  16'sd4281, NC1 =  16'sd4163, NC2 = -16'sd4400, NC3 = -16'sd4163, NC4 =  16'sd119;
localparam signed [15:0] A1  =  16'sd4697, A2  =  16'sd6674;    // complex pair, 0.64 at 103 degrees
localparam signed [15:0] B1  =  16'sd386,  B2  = -16'sd8530;    // real poles, 0.71 and -0.73

logic signed [23:0] ld1, ld2, ld3, ld4;   // luma and chroma input histories
logic signed [23:0] cd1, cd2, cd3, cd4;
logic signed [26:0] u1, u2;               // first all-pole section
logic signed [26:0] v1, v2;               // second all-pole section

/* verilator lint_off UNUSEDSIGNAL */
wire signed [42:0] fir = luma * NL0 + ld1 * NL1 + ld2 * NL2 + ld3 * NL3 + ld4 * NL4 +
                         chroma * NC0 + cd1 * NC1 + cd2 * NC2 + cd3 * NC3 + cd4 * NC4;
wire signed [26:0] fir_q = fir[40:14];
wire signed [43:0] u_fb = u1 * A1 + u2 * A2;
wire signed [27:0] u_fq = u_fb[41:14];
wire signed [27:0] u_n  = fir_q - u_fq;
wire signed [26:0] u_c  = u_n[26:0];
wire signed [43:0] v_fb = v1 * B1 + v2 * B2;
wire signed [27:0] v_fq = v_fb[41:14];
wire signed [27:0] v_n  = u_c - v_fq;
/* verilator lint_on UNUSEDSIGNAL */
wire signed [23:0] node = (v_n > 28'sd8388607) ? 24'sd8388607 :
                          (v_n < -28'sd8388608) ? -24'sd8388608 : v_n[23:0];

always_ff @(posedge clk) begin
	ld1 <= luma;   ld2 <= ld1; ld3 <= ld2; ld4 <= ld3;
	cd1 <= chroma; cd2 <= cd1; cd3 <= cd2; cd4 <= cd3;
	u1 <= u_c; u2 <= u1;
	v1 <= v_n[26:0]; v2 <= v1;
	if (pix_tick) begin
		phase <= pixph;
		pixph <= pixph + pix_samples;
	end
	else phase <= phase + 2'd1;
	sample_tog <= ~sample_tog;
	comp       <= node;

	hs_d <= hsync;
	if (hs_d && ~hsync) hcnt <= 10'd0;
	else if (~&hcnt)    hcnt <= hcnt + 10'd1;
end

endmodule
