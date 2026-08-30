// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

// The 7800 and 2600 video output stage: colour and luminance codes in, baseband
// composite out, in volts.
//
// Neither chip has a composite pin. They drive LUM lines and a single COLOR pin,
// and a resistor network on the board sums them. `tia-maria.pdf` page 47,
// COLOR DELAY LINE & LUM, sheet 3 of 6, is that stage: a delay chain across the
// top feeding a crossbar with fifteen columns - one per colour code - out to
// PAD COLOR pin 43, alongside PAD LUM0-3 on pins 42/47/45/44 and SYNCB on 48.
//
// Three things that sheet settles:
//
//   Fifteen colour codes select fifteen taps of one delay line. Colour is a
//   phase, not a level, which is why artifact colour exists at all.
//
//   The line's delay is set by an on-chip RC reference - the sheet notes
//   R1 = 274 squares, R2 = 109.5, C1 = 11.8 pF, C2 = 4.5 pF against a 6915 sq
//   micron reference. An RC bias drifts with die temperature, which is the
//   physical cause of the warm / cool / hot palettes.
//
//   MARIA owns the stage for both chips. Its inputs are MC0-3 / ML0-3F beside
//   TC0-3 / TL0-3F, with MCB / TCB and MSYNC / TSYNC, gated by MENBLF. On a 7800
//   the 2600's colour never reaches a pin of its own; MARIA multiplexes the
//   codes and drives the one COLOR pin. So this module is instantiated once per
//   chip here only because they are separate modules in this core - the hardware
//   has a single one, and selecting between two composites downstream is the
//   same picture as selecting between two codes upstream.
//
// Sampled at 4x the subcarrier, which is clk_sys: four samples per 2600 pixel,
// two per MARIA pixel. That is enough for both, because the decoder's notch
// removes everything above the subcarrier anyway. Feed composite_decoder with
// SPC = 4.

module composite_out #(
	// Where the pixel grid sits against the carrier. Only four values exist at
	// four samples per cycle, and each turns the artifact hue by 90 degrees, so
	// this is what decides what a one-pixel dither decodes to. It is a property
	// of the chip - MARIA's pixel clock against the delay line - not a free
	// choice, and it is settled against a hardware capture.

	// Luma band limit, in samples averaged.
	//
	// 2, not 4. Four samples is one whole subcarrier period, so its null lands
	// exactly on fsc and every luma-derived artifact colour disappears - which
	// is not what the board does. Its luma path is L2 0.82-1.8uH against C5
	// 820pF around the Q1 follower, rolling off at 4 to 6 MHz, so the
	// subcarrier passes largely intact and artifact colour survives. Two
	// samples give 0.707 at fsc, which is the shape of that rolloff. 0
	// bypasses the filter entirely.
	parameter integer LUMA_AVG = 2,
	// How far a band-limited luma window is pulled from its mean toward its
	// peak, in 256ths. See the note by the filter. 0 leaves the plain mean.
	parameter [7:0] LUMA_PEAK = 8'd72,
	// The luma node's rolloff, as one pole: alpha = 1/2^LUMA_POLE.
	//
	// Not a trap. A 3.58 MHz trap belongs to the RECEIVER's luma channel - one
	// documented part has a 0.62 MHz -3dB width, so Q about 5.8 - and that is a
	// different box. What the console has is a slow node: the ladder's Thevenin
	// source, 2.26k with R14 7.5K in the reckoning, driving the node's
	// capacitance. 47 to 100 pF there puts the corner at 1.5 down to 0.7 MHz
	// and the gain at the subcarrier between 0.39 and 0.19, which is the
	// attenuation the measured output shows. One pole, because that is what the
	// circuit is.
	//
	// A boxcar cannot do this at all: four samples nulls fsc dead and takes
	// every artifact colour with it, two leaves 0.707 and a full-swing luma
	// alternation then decodes at twice the saturation of a real colour.
	parameter integer LUMA_POLE = 2,
	// Whether the chroma vector steps between colours instantly. On the chip it
	// cannot: the tap select changes at a pixel boundary but the line and the
	// driver take real time to follow. One sample of transition at this rate is
	// about 35ns. It matters most where the gate is fastest - the brick alters
	// every 139ns - and barely at all where it is slow, which is the direction
	// the hardware measurements go.
	parameter bit CHROMA_SLEW = 1'b1
)(
	input               clk,        // 4x subcarrier

	// Per-chip, because one instance serves both - as MARIA's stage does on the
	// board, taking TC/TL beside its own MC/ML and driving the one COLOR pin.
	input         [9:0] burst_on,   // burst window, samples after HSync falls
	input         [9:0] burst_off,
	input         [8:0] chroma_scale, // 256 = the tables as they are; TIA 236,
	                                  // being injected through R17 4.7K against
	                                  // MARIA's R16 4.3K
	input         [1:0] pix_samples,  // 2 for MARIA at 2*fsc, 0 (=4) for TIA
	input         [1:0] phase_adj,    // which of the alignments the dividers allow
	input         [3:0] col,        // colour code; 0 is no chroma
	input         [3:0] lum,        // TIA passes {lum, 1'b0}: it has three bits, MARIA four
	input               blank,
	input               hsync,      // positive pulse
	// High on the first sample of each source pixel. The carrier is aligned to
	// this rather than left to free-run, because where the pixel grid sits
	// against the subcarrier is a property of the chip's dividers - the pixel
	// clock is a whole multiple of fsc - not of where reset happened to fall.
	// The burst cannot supply it: the burst fixes the COLOUR reference, which
	// is why steady colours do not move with phase_adj, only artifacts do.
	input               pix_tick,
	input               hblank,
	input               vblank,
	input               vsync,
	input         [1:0] temp,       // 0 warm, 1 cool, 2 hot

	output logic signed [23:0] comp, // Q2.21 volts, 1.0 = 1.0V
	output logic        sample_tog, // toggles per sample, for the CDC

	// Carried alongside comp rather than taken from the normal video path, which
	// runs through video_mux and cofi and so arrives several pixels later. The
	// decoder finds the burst by counting from HSync, so its sync has to be the
	// one that matches this signal.
	output logic        hs_out,
	output logic        vs_out,
	output logic        hb_out,
	output logic        vb_out
);

// The ladder, the chroma amplitude and the hue angles are NOT from the drawing.
// Page 47 shows the delay line and the crossbar but not the tap spacing, and the
// board's resistor network is a different document. They are extracted from this
// project's palettes, which came from measured console output.
//
// Each colour's vector - angle AND magnitude - is the measured mean over the
// luma rows least affected by clipping, per palette. The magnitude is not equal
// across codes, even though one delay line at one amplitude is what the chip
// has; whatever that per-code variation is, it is in the real output, and
// flattening it moves the picture away from both measured references. Lifted by
// 1/0.940 for the resonator's loss at the carrier. The angles' spacing is worth
// recording:
// Fitting a line to the fifteen hue angles gives 25.97, 26.95 and 27.93 degrees
// per colour code for the cool, warm and hot palettes, against the 25.7 / 26.7 /
// 27.7 that `rtl/video_mux.sv` documents - within a quarter degree on all three,
// with the base phase unmoved at 316.5. Temperature moves the delay line's
// degrees per tap and nothing else, exactly as an RC-biased chain would. The
// nominal for fifteen taps closing one cycle would be 24.0 degrees; every
// measured palette says the real line runs fast.

// The burst is the same chroma, gated during the burst window and leaving on
// the same pin through the same divider - but not at full tap amplitude. It is
// 0.143V, 40 IRE peak to peak, where the palettes put chroma at 0.1774V mean.
// The decoder normalises chroma to the burst, so that difference lands straight
// on saturation: equal amplitudes cost every colour 19 percent, and the
// captured platforms and water are more saturated than that allows.
localparam signed [23:0] BURST_AMP = 24'sd299892;

function automatic signed [23:0] ld(input [3:0] l);
	case (l)
		4'd0 : ld = 24'sd0;
		4'd1 : ld = 24'sd76336;
		4'd2 : ld = 24'sd234881;
		4'd3 : ld = 24'sd364066;
		4'd4 : ld = 24'sd481506;
		4'd5 : ld = 24'sd593075;
		4'd6 : ld = 24'sd698771;
		4'd7 : ld = 24'sd798595;
		4'd8 : ld = 24'sd892548;
		4'd9 : ld = 24'sd986500;
		4'd10: ld = 24'sd1074581;
		4'd11: ld = 24'sd1162661;
		4'd12: ld = 24'sd1250741;
		4'd13: ld = 24'sd1332950;
		4'd14: ld = 24'sd1415158;
		4'd15: ld = 24'sd1497367;
		default: ld = 24'sd0;
	endcase
endfunction

// At four samples per cycle the carrier visits +sin, +cos, -sin, -cos and
// nothing else, so two constants per colour are the whole waveform.
function automatic signed [23:0] cs(input [5:0] a);
	case (a)
		6'd0 : cs = 24'sd0;
		6'd1 : cs = -24'sd246859;
		6'd2 : cs = -24'sd133480;
		6'd3 : cs = 24'sd23372;
		6'd4 : cs = 24'sd195020;
		6'd5 : cs = 24'sd305118;
		6'd6 : cs = 24'sd332160;
		6'd7 : cs = 24'sd278357;
		6'd8 : cs = 24'sd181065;
		6'd9 : cs = 24'sd55346;
		6'd10: cs = -24'sd96085;
		6'd11: cs = -24'sd242472;
		6'd12: cs = -24'sd355201;
		6'd13: cs = -24'sd357886;
		6'd14: cs = -24'sd291383;
		6'd15: cs = -24'sd193908;
		6'd16: cs = 24'sd0;
		6'd17: cs = -24'sd246859;
		6'd18: cs = -24'sd138802;
		6'd19: cs = 24'sd9787;
		6'd20: cs = 24'sd178773;
		6'd21: cs = 24'sd293292;
		6'd22: cs = 24'sd333673;
		6'd23: cs = 24'sd293864;
		6'd24: cs = 24'sd209533;
		6'd25: cs = 24'sd95395;
		6'd26: cs = -24'sd45290;
		6'd27: cs = -24'sd189668;
		6'd28: cs = -24'sd320206;
		6'd29: cs = -24'sd367714;
		6'd30: cs = -24'sd328946;
		6'd31: cs = -24'sd249389;
		6'd32: cs = 24'sd0;
		6'd33: cs = -24'sd246859;
		6'd34: cs = -24'sd129467;
		6'd35: cs = 24'sd38161;
		6'd36: cs = 24'sd212938;
		6'd37: cs = 24'sd316477;
		6'd38: cs = 24'sd326169;
		6'd39: cs = 24'sd257340;
		6'd40: cs = 24'sd148035;
		6'd41: cs = 24'sd9565;
		6'd42: cs = -24'sd150468;
		6'd43: cs = -24'sd297168;
		6'd44: cs = -24'sd368547;
		6'd45: cs = -24'sd331017;
		6'd46: cs = -24'sd243682;
		6'd47: cs = -24'sd126855;
		6'd48: cs = 24'sd0;
		6'd49: cs = -24'sd246859;
		6'd50: cs = -24'sd133480;
		6'd51: cs = 24'sd23372;
		6'd52: cs = 24'sd195020;
		6'd53: cs = 24'sd305118;
		6'd54: cs = 24'sd332160;
		6'd55: cs = 24'sd278357;
		6'd56: cs = 24'sd181065;
		6'd57: cs = 24'sd55346;
		6'd58: cs = -24'sd96085;
		6'd59: cs = -24'sd242472;
		6'd60: cs = -24'sd355201;
		6'd61: cs = -24'sd357886;
		6'd62: cs = -24'sd291383;
		6'd63: cs = -24'sd193908;
		default: cs = 24'sd0;
	endcase
endfunction

function automatic signed [23:0] cc(input [5:0] a);
	case (a)
		6'd0 : cc = 24'sd0;
		6'd1 : cc = 24'sd334654;
		6'd2 : cc = 24'sd434776;
		6'd3 : cc = 24'sd424674;
		6'd4 : cc = 24'sd309252;
		6'd5 : cc = 24'sd152053;
		6'd6 : cc = -24'sd17466;
		6'd7 : cc = -24'sd181253;
		6'd8 : cc = -24'sd340097;
		6'd9 : cc = -24'sd451450;
		6'd10: cc = -24'sd436775;
		6'd11: cc = -24'sd317650;
		6'd12: cc = -24'sd116341;
		6'd13: cc = 24'sd89767;
		6'd14: cc = 24'sd261507;
		6'd15: cc = 24'sd394020;
		6'd16: cc = 24'sd0;
		6'd17: cc = 24'sd334654;
		6'd18: cc = 24'sd431555;
		6'd19: cc = 24'sd429254;
		6'd20: cc = 24'sd324568;
		6'd21: cc = 24'sd178783;
		6'd22: cc = 24'sd13652;
		6'd23: cc = -24'sd144969;
		6'd24: cc = -24'sd300489;
		6'd25: cc = -24'sd429602;
		6'd26: cc = -24'sd456519;
		6'd27: cc = -24'sd373598;
		6'd28: cc = -24'sd201114;
		6'd29: cc = 24'sd1872;
		6'd30: cc = 24'sd181176;
		6'd31: cc = 24'sd330729;
		6'd32: cc = 24'sd0;
		6'd33: cc = 24'sd334654;
		6'd34: cc = 24'sd436445;
		6'd35: cc = 24'sd418126;
		6'd36: cc = 24'sd292213;
		6'd37: cc = 24'sd125079;
		6'd38: cc = -24'sd50498;
		6'd39: cc = -24'sd221290;
		6'd40: cc = -24'sd378630;
		6'd41: cc = -24'sd461572;
		6'd42: cc = -24'sd404615;
		6'd43: cc = -24'sd244498;
		6'd44: cc = -24'sd21959;
		6'd45: cc = 24'sd175325;
		6'd46: cc = 24'sd337185;
		6'd47: cc = 24'sd437380;
		6'd48: cc = 24'sd0;
		6'd49: cc = 24'sd334654;
		6'd50: cc = 24'sd434776;
		6'd51: cc = 24'sd424674;
		6'd52: cc = 24'sd309252;
		6'd53: cc = 24'sd152053;
		6'd54: cc = -24'sd17466;
		6'd55: cc = -24'sd181253;
		6'd56: cc = -24'sd340097;
		6'd57: cc = -24'sd451450;
		6'd58: cc = -24'sd436775;
		6'd59: cc = -24'sd317650;
		6'd60: cc = -24'sd116341;
		6'd61: cc = 24'sd89767;
		6'd62: cc = 24'sd261507;
		6'd63: cc = 24'sd394020;
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
wire signed [23:0] cs_now = cs_m[31:8];
wire signed [23:0] cc_now = cc_m[31:8];
logic signed [23:0] cs_d, cc_d;
/* verilator lint_off UNUSEDSIGNAL */
wire signed [24:0] cs_sum = cs_now + cs_d;
wire signed [24:0] cc_sum = cc_now + cc_d;
/* verilator lint_on UNUSEDSIGNAL */
wire signed [23:0] chroma_s = CHROMA_SLEW ? cs_sum[24:1] : cs_now;
wire signed [23:0] chroma_c = CHROMA_SLEW ? cc_sum[24:1] : cc_now;

logic signed [23:0] chroma;
logic signed [23:0] luma;

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
end

// The luma path is band-limited and the chroma path is not, which is the whole
// reason a one-pixel dither reads as a flat grey surface on hardware instead
// of as saturated artifact colour. The delay line's chroma is a continuous
// oscillation - it never has to slew - while luma steps between DAC levels
// every pixel and the board's video path cannot follow it at the subcarrier.
// Averaging one whole subcarrier period nulls exactly that component and
// leaves everything slower than it, which is what the ladder and the video
// amplifier do between them.
//
// A filter on the summed composite cannot do this job: real chroma sits at
// the same frequency as the artifact and would be attenuated with it.
// Chroma reaches the composite through the board's series LC - L1 47uH and C8
// 47pF against R2 470, resonant at 3.386 MHz with a Q near 2.13. That is not a
// detail: it is why hardware's response depends on how fast a pattern modulates,
// not just on its duty. Two 50-percent dithers of the same two colours measure
// 0.28 and 1.00 of full saturation on real hardware where a flat sum gives 0.50
// for both. As a biquad at this sample rate the response is 0.000 at DC, 0.107
// at 0.9 MHz, 0.940 at the subcarrier and 0.000 at twice it.
localparam signed [15:0] RES_B  =  16'sd3109;    // Q14
localparam signed [15:0] RES_A1 =  16'sd2249;
localparam signed [15:0] RES_A2 = -16'sd10167;

logic signed [23:0] cx1, cx2, cy1, cy2;
/* verilator lint_off UNUSEDSIGNAL */
wire signed [24:0] cdx  = chroma - cx2;
wire signed [40:0] cacc = cdx * RES_B + cy1 * RES_A1 + cy2 * RES_A2;
/* verilator lint_on UNUSEDSIGNAL */
wire signed [26:0] cres = cacc[40:14];
wire signed [23:0] chroma_ac = (cres >  27'sd8388607) ?  24'sd8388607 :
                               (cres < -27'sd8388608) ? -24'sd8388608 : cres[23:0];

logic signed [25:0] lsum;
logic [LUMA_AVG-1:0][23:0] lbuf;
wire signed [23:0] lold = (LUMA_AVG == 0) ? 24'sd0 : $signed(lbuf[LUMA_AVG-1]);
wire signed [25:0] lsum_n = lsum + {{2{luma[23]}}, luma} - {{2{lold[23]}}, lold};
wire signed [23:0] lmean = (LUMA_AVG == 4) ? lsum_n[25:2] :
                           (LUMA_AVG == 2) ? lsum_n[24:1] : luma;

// Averaging alone is too dark. A set still shows the alternation - its luma bandwidth does not fully reach the
// subcarrier - and the eye integrates light, not volts. Black against V is
// perceived at V*2^(-1/gamma), about 0.73 of it, where a plain mean gives
// 0.50; real hardware measures 0.64 on Tower Toppler's brick, between the
// two, because the set does part of the averaging electrically first. Once
// the alternation is filtered out here nothing downstream can integrate it,
// so the window is pulled toward its peak by that much instead.
//
// On a flat field the peak IS the mean, so this is identically zero there and
// cannot disturb flat colours.
wire signed [23:0] lw0 = luma;
wire signed [23:0] lw1 = $signed(lbuf[0]);
wire signed [23:0] lw2 = (LUMA_AVG >= 4) ? $signed(lbuf[1]) : lw1;
wire signed [23:0] lw3 = (LUMA_AVG >= 4) ? $signed(lbuf[2]) : lw1;
wire signed [23:0] lma  = (lw0 > lw1) ? lw0 : lw1;
wire signed [23:0] lmb  = (lw2 > lw3) ? lw2 : lw3;
wire signed [23:0] lmax = (lma > lmb) ? lma : lmb;
/* verilator lint_off UNUSEDSIGNAL */
wire signed [24:0] ldif = lmax - lmean;
wire signed [32:0] lpk  = ldif * $signed({1'b0, LUMA_PEAK});
/* verilator lint_on UNUSEDSIGNAL */
wire signed [23:0] luma_pk = (LUMA_AVG == 0) ? luma : (lmean + lpk[31:8]);

// One pole, y += (x - y) >> LUMA_POLE. At this sample rate that is 0.447, 0.200
// and 0.094 at the subcarrier for shifts of 1, 2 and 3.
logic signed [23:0] lp;
wire signed [24:0] lerr = luma_pk - lp;
/* verilator lint_off UNUSEDSIGNAL */
wire signed [24:0] lp_next = lp + (lerr >>> LUMA_POLE);
/* verilator lint_on UNUSEDSIGNAL */
wire signed [23:0] luma_bw = (LUMA_POLE == 0) ? luma_pk : lp;

always_ff @(posedge clk) begin
	cs_d <= cs_now;  cc_d <= cc_now;
	lp <= lp_next[23:0];
	cx1 <= chroma; cx2 <= cx1;
	cy1 <= chroma_ac; cy2 <= cy1;
	lbuf  <= {lbuf[LUMA_AVG-2:0], luma};
	lsum  <= lsum_n;
	if (pix_tick) begin
		phase <= pixph;
		pixph <= pixph + pix_samples;
	end
	else phase <= phase + 2'd1;
	sample_tog <= ~sample_tog;
	comp       <= luma_bw + chroma_ac;

	hs_out <= hsync;
	vs_out <= vsync;
	hb_out <= hblank;
	vb_out <= vblank;

	hs_d <= hsync;
	if (hs_d && ~hsync) hcnt <= 10'd0;
	else if (~&hcnt)    hcnt <= hcnt + 10'd1;
end

endmodule
