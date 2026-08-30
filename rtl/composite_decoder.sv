// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

// Portable NTSC composite decoder.
//
// Takes a baseband composite signal and produces RGB, the way a cheap
// receiver does: a half-subcarrier notch splits luma from chroma, the chroma is
// demodulated against a locally generated carrier, and the result is corrected
// against the colourburst measured in the signal itself.
//
// Nothing here is specific to any console. It decodes whatever composite it is
// given. See composite_decoder.md for the signal format, how to choose SPC, and
// what each config port does.
//
// Two deliberate departures from a real receiver:
//
//   Sync is not separated. hs/vs/hb/vb come in as flags. Every FPGA core
//   already has them, so recovering them from the signal would add a whole
//   lock-failure surface for no fidelity.
//
//   Nothing is clipped on the way in. Sources that leave broadcast range - the
//   NES most notably, whose brightest luma sits above white and whose $0D sits
//   below blanking - must reach the output stage intact, because reproducing
//   what a set does with them is the point. Clipping happens once, at the end.

module composite_decoder #(
	// Samples per subcarrier cycle. Must be EVEN, so that the notch delay
	// SPC/2 is a whole number of samples and lands exactly half a cycle back.
	// Pick a sample clock that is a whole multiple of the subcarrier.
	parameter SPC = 8
)(
	input                clk,
	input                ce,          // one composite sample

	// Signed Q2.21 volts: 1.0 = 1.0V. Blanking 0.000, white 0.714, sync -0.286.
	input  signed [23:0] comp,

	// Positive pulses, in the same sample domain as comp.
	input                hs_in,
	input                vs_in,
	input                hb_in,
	input                vb_in,

	// Burst window, in samples after HSync falls. burst_start should point a
	// few samples into the burst so its leading edge is not measured. The black
	// level is taken from the back porch after burst_start + burst_len, so
	// burst_len must cover the rest of the burst.
	input          [9:0] burst_start,
	input          [9:0] burst_len,

	input          [7:0] sat,         // 128 = unity
	input          [7:0] hue,         // 256 = one full cycle
	input          [3:0] smear,       // chroma trail length, 0 = off
	input          [3:0] luma_delay,  // samples; see the note on smear below

	// Black's level above blanking, signed Q2.13 volts. NTSC-M sets black 7.5
	// IRE above blanking, which is 0.0536V, or 439 here. NTSC-J and most
	// consoles have no setup at all, so 0. Getting it wrong lifts or crushes
	// blacks by 7.5 percent of the range.
	input  signed [15:0] setup,

	// Volts-to-white gain, Q16, applied after `setup` is removed: 65536*255 /
	// ((white - setup) * 8192). For 0.714V white that is 2857 with no setup and
	// 3089 with NTSC-M setup.
	input         [15:0] luma_gain,

	// Track the signal's level off the burst instead of trusting `luma_gain`
	// outright. NTSC gives the burst the same amplitude as sync, 40 IRE peak to
	// peak, so measuring it is the IRE scale - which is the information sync-tip
	// AGC uses, and the only route to it for a decoder that is handed sync as a
	// flag rather than in the signal. Falls back to `luma_gain` when there is no
	// burst to measure.
	input                agc_en,

	output logic         ce_out,
	output logic         hs_out,
	output logic         vs_out,
	output logic         hb_out,
	output logic         vb_out,
	output logic   [7:0] r_out,
	output logic   [7:0] g_out,
	output logic   [7:0] b_out
);

localparam HALFC     = SPC / 2;
localparam [31:0] PHASE_INC = 32'd16777216 / SPC;  // 2^24 per subcarrier cycle
localparam [31:0] BOX_RECIP = 32'd65536 / SPC;     // boxcar divide, avoids /SPC

localparam BURST_ACC  = 16;   // burst samples averaged, power of two
localparam BURST_SH   = 4;
// Back-porch samples averaged for black. Sixteen, not more: MARIA's burst
// ends only 24 samples before active video and the window has to fit.
localparam CLAMP_ACC  = 16;
localparam CLAMP_SH   = 4;

// Below this the burst is absent or unusable and colour is killed, which is
// what a receiver does with a monochrome signal.
localparam [31:0] MIN_MAG2 = 32'd16384;

// 40 IRE peak to peak, 0.286V, in Q2.13: what a nominal burst measures, and
// by NTSC the same amplitude as sync, which is what makes it usable as the
// level reference.
localparam [15:0]        BURST_PP    = 16'd2343;
localparam               BURST_SHIFT = 8;

// NTSC puts the burst 180 degrees from the axis the demodulator wants, so
// the measured burst vector is rotated by this before it becomes the
// reference. Encoders that emit burst at the standard phase decode with hue
// at zero meaning no shift.
localparam [7:0]         BURST_CONV  = 8'd128;

// Numerator of the once-per-line reciprocal, and with it the chroma gain: cg
// and sg both scale with it. 1.156 x 2^30, set against an exactly generated
// composite signal rather than against any one console's palette.
localparam [31:0]        DIV_NUM     = 32'h49F700A6;

function automatic signed [15:0] sat16(input signed [17:0] v);
	if      (v >  18'sd32767) sat16 =  16'sd32767;
	else if (v < -18'sd32768) sat16 = -16'sd32768;
	else                      sat16 =  v[15:0];
endfunction

// ---------------------------------------------------------------- line timing

logic       hs_d;
logic [9:0] hcnt;

wire [10:0] burst_end = {1'b0, burst_start} + BURST_ACC;
wire [10:0] clamp_beg = {1'b0, burst_start} + {1'b0, burst_len};
wire [10:0] clamp_end = clamp_beg + CLAMP_ACC;

wire in_burst = ({1'b0, hcnt} >= {1'b0, burst_start}) && ({1'b0, hcnt} <  burst_end);
wire in_clamp = ({1'b0, hcnt} >= clamp_beg)           && ({1'b0, hcnt} <  clamp_end) && hb_in;
wire burst_fin = ({1'b0, hcnt} == burst_end);
wire clamp_fin = ({1'b0, hcnt} == clamp_end);

always_ff @(posedge clk) if (ce) begin
	hs_d <= hs_in;
	if (hs_d && ~hs_in) hcnt <= 10'd0;      // trailing edge of sync starts the count
	else if (~&hcnt)    hcnt <= hcnt + 10'd1;
end

// ------------------------------------------------- clamp and requantise input
//
// Averaging the back porch gives the black level, so the decoder does not care
// what DC offset or scaling the source hands it. Q2.21 drops to Q2.13 here:
// past this point 122uV of resolution is far finer than any real source, and
// 16-bit operands keep every multiply inside one DSP.

logic signed [28:0] black_acc;
logic signed [23:0] black;
logic signed [15:0] c16;

wire signed [24:0] c_sub = comp - black;
wire signed [24:0] c_shf = c_sub >>> 8;

always_ff @(posedge clk) if (ce) begin
	if (hcnt == 10'd0)   black_acc <= 29'sd0;
	else if (in_clamp)   black_acc <= black_acc + {{5{comp[23]}}, comp};
	if (clamp_fin)       black <= black_acc[27:CLAMP_SH];

	// Scale into the working format, never clamp into it: the over-range is
	// the part that has to survive.
	if      (c_shf >  25'sd32767) c16 <=  16'sd32767;
	else if (c_shf < -25'sd32768) c16 <= -16'sd32768;
	else                          c16 <=  c_shf[15:0];
end

// --------------------------------------------------------- notch: luma/chroma
//
// SPC/2 samples back is half a subcarrier cycle, so the subcarrier arrives
// inverted: the sum cancels it and the difference keeps only it. Exact, and no
// coefficients. It is also the only separation that works on sources whose line
// is a whole number of subcarrier cycles, where a comb would cancel the wrong
// one of the two.

logic [HALFC-1:0][15:0] nd;
logic signed [15:0] yc, cc, yc_d, y_lp;

wire signed [15:0] c_del = $signed(nd[HALFC-1]);
/* verilator lint_off UNUSEDSIGNAL */
wire signed [16:0] n_sum = c16 + c_del;
wire signed [16:0] n_dif = c16 - c_del;
/* verilator lint_on UNUSEDSIGNAL */

// The notch takes the subcarrier out of luma but leaves what the pixel grid
// puts at twice it - a ripple every other sample. A receiver's luma bandwidth
// does not reach 2*fsc, so it never sees that; without this the output samples
// one phase of the ripple instead of the mean and a dither reads far too
// bright. Two taps null exactly Fs/2, which is where the ripple sits.
/* verilator lint_off UNUSEDSIGNAL */
wire signed [16:0] y_sum = yc + yc_d;
/* verilator lint_on UNUSEDSIGNAL */

always_ff @(posedge clk) if (ce) begin
	nd   <= {nd[HALFC-2:0], c16};
	yc   <= n_sum[16:1];
	cc   <= n_dif[16:1];
	yc_d <= yc;
	y_lp <= y_sum[16:1];
end

// ------------------------------------------------------------ carrier and mix
//
// The accumulator free-runs and is never reset at hsync. Sources whose line is
// a whole number of subcarrier cycles then hold a stable artifact hue down each
// column, and sources whose line is a half cycle get their dot crawl, both
// without being told which they are.

logic [23:0] phase;
always_ff @(posedge clk) if (ce) phase <= phase + PHASE_INC[23:0];

function automatic signed [10:0] qsin(input [6:0] a);
	case (a)
		7'd0 : qsin = 11'sd0;
		7'd1 : qsin = 11'sd13;
		7'd2 : qsin = 11'sd25;
		7'd3 : qsin = 11'sd38;
		7'd4 : qsin = 11'sd50;
		7'd5 : qsin = 11'sd63;
		7'd6 : qsin = 11'sd75;
		7'd7 : qsin = 11'sd87;
		7'd8 : qsin = 11'sd100;
		7'd9 : qsin = 11'sd112;
		7'd10: qsin = 11'sd124;
		7'd11: qsin = 11'sd136;
		7'd12: qsin = 11'sd148;
		7'd13: qsin = 11'sd160;
		7'd14: qsin = 11'sd172;
		7'd15: qsin = 11'sd184;
		7'd16: qsin = 11'sd196;
		7'd17: qsin = 11'sd207;
		7'd18: qsin = 11'sd218;
		7'd19: qsin = 11'sd230;
		7'd20: qsin = 11'sd241;
		7'd21: qsin = 11'sd252;
		7'd22: qsin = 11'sd263;
		7'd23: qsin = 11'sd273;
		7'd24: qsin = 11'sd284;
		7'd25: qsin = 11'sd294;
		7'd26: qsin = 11'sd304;
		7'd27: qsin = 11'sd314;
		7'd28: qsin = 11'sd324;
		7'd29: qsin = 11'sd334;
		7'd30: qsin = 11'sd343;
		7'd31: qsin = 11'sd352;
		7'd32: qsin = 11'sd361;
		7'd33: qsin = 11'sd370;
		7'd34: qsin = 11'sd379;
		7'd35: qsin = 11'sd387;
		7'd36: qsin = 11'sd395;
		7'd37: qsin = 11'sd403;
		7'd38: qsin = 11'sd410;
		7'd39: qsin = 11'sd418;
		7'd40: qsin = 11'sd425;
		7'd41: qsin = 11'sd432;
		7'd42: qsin = 11'sd438;
		7'd43: qsin = 11'sd445;
		7'd44: qsin = 11'sd451;
		7'd45: qsin = 11'sd456;
		7'd46: qsin = 11'sd462;
		7'd47: qsin = 11'sd467;
		7'd48: qsin = 11'sd472;
		7'd49: qsin = 11'sd477;
		7'd50: qsin = 11'sd481;
		7'd51: qsin = 11'sd485;
		7'd52: qsin = 11'sd489;
		7'd53: qsin = 11'sd492;
		7'd54: qsin = 11'sd496;
		7'd55: qsin = 11'sd499;
		7'd56: qsin = 11'sd501;
		7'd57: qsin = 11'sd503;
		7'd58: qsin = 11'sd505;
		7'd59: qsin = 11'sd507;
		7'd60: qsin = 11'sd509;
		7'd61: qsin = 11'sd510;
		7'd62: qsin = 11'sd510;
		7'd63: qsin = 11'sd511;
		7'd64: qsin = 11'sd511;
		default: qsin = 11'sd511;
	endcase
endfunction

function automatic signed [10:0] sine(input [7:0] p);
	logic [6:0] idx;
	logic signed [10:0] v;
	idx  = p[6] ? (7'd64 - {1'b0, p[5:0]}) : {1'b0, p[5:0]};
	v    = qsin(idx);
	sine = p[7] ? -v : v;
endfunction

wire [7:0] ph = phase[23:16];

logic signed [17:0] i_dem, q_dem;

/* verilator lint_off UNUSEDSIGNAL */
wire signed [26:0] i_mul = cc * sine(ph);
wire signed [26:0] q_mul = cc * sine(ph + 8'd64);
/* verilator lint_on UNUSEDSIGNAL */

always_ff @(posedge clk) if (ce) begin
	i_dem <= i_mul[26:9];                   // undo the table's 511 scale
	q_dem <= q_mul[26:9];
end

// ---------------------------------------------------------- chroma band limit
//
// A boxcar exactly one subcarrier cycle long nulls both the subcarrier and the
// 2*fsc image the mixer leaves behind, and its length is what makes a chroma
// dither one cycle wide average to the mean colour.

logic [SPC-1:0][17:0] ibuf, qbuf;
logic signed [21:0] iacc, qacc;
logic signed [17:0] ibox, qbox;

wire signed [17:0] i_old = $signed(ibuf[SPC-1]);
wire signed [17:0] q_old = $signed(qbuf[SPC-1]);
wire signed [21:0] iacc_n = iacc + {{4{i_dem[17]}}, i_dem} - {{4{i_old[17]}}, i_old};
wire signed [21:0] qacc_n = qacc + {{4{q_dem[17]}}, q_dem} - {{4{q_old[17]}}, q_old};
/* verilator lint_off UNUSEDSIGNAL */
wire signed [38:0] iacc_d = iacc_n * $signed({1'b0, BOX_RECIP[16:0]});
wire signed [38:0] qacc_d = qacc_n * $signed({1'b0, BOX_RECIP[16:0]});
/* verilator lint_on UNUSEDSIGNAL */

always_ff @(posedge clk) if (ce) begin
	ibuf <= {ibuf[SPC-2:0], i_dem};
	qbuf <= {qbuf[SPC-2:0], q_dem};
	iacc <= iacc_n;
	qacc <= qacc_n;
	ibox <= iacc_d[33:16];
	qbox <= qacc_d[33:16];
end

// ------------------------------------------------------------ burst lock
//
// Frequency is already known from SPC, so only phase and gain need recovering.
// Demodulating the burst gives a vector; dividing the chroma by that vector is
// simultaneously the rotation that corrects hue and the scaling that corrects
// saturation, so there is no angle to extract and no loop to settle. The
// division is one reciprocal per line, and a whole active line to do it in.

logic signed [15:0] bmax, bmin;
logic        [15:0] bpp;
logic signed [21:0] ib_acc, qb_acc;
logic signed [15:0] ib, qb;
logic signed [15:0] ibh, qbh;
logic signed [17:0] cg, sg;

wire [7:0] hue_ph = hue + BURST_CONV;
wire signed [10:0] hue_c = sine(hue_ph + 8'd64);
wire signed [10:0] hue_s = sine(hue_ph);
/* verilator lint_off UNUSEDSIGNAL */
wire signed [26:0] rot_i = ib * hue_c - qb * hue_s;
wire signed [26:0] rot_q = ib * hue_s + qb * hue_c;
/* verilator lint_on UNUSEDSIGNAL */

wire signed [31:0] ibh_x = {{16{ibh[15]}}, ibh};
wire signed [31:0] qbh_x = {{16{qbh[15]}}, qbh};
wire        [31:0] mag2  = $unsigned(ibh_x * ibh_x) + $unsigned(qbh_x * qbh_x);

// One restoring division per line: inv = 2^30 / mag2.
logic [31:0] div_rem;
/* verilator lint_off UNUSEDSIGNAL */
logic [31:0] div_num, div_quo, div_den;
/* verilator lint_on UNUSEDSIGNAL */
logic  [5:0] div_cnt;
logic        div_go;

wire [32:0] rem_sh = {div_rem, div_num[31]};
wire        rem_ge = (rem_sh >= {1'b0, div_den});
/* verilator lint_off UNUSEDSIGNAL */
wire [32:0] rem_nx = rem_ge ? (rem_sh - {1'b0, div_den}) : rem_sh;
/* verilator lint_on UNUSEDSIGNAL */

logic [15:0] inv;
logic        colour_ok;

// AGC: nominal burst over measured burst, Q14. A second short division, run
// once a line beside the chroma one, with a whole active line to finish in.
/* verilator lint_off UNUSEDSIGNAL */
logic [31:0] ag_rem, ag_num, ag_quo, ag_den;
/* verilator lint_on UNUSEDSIGNAL */
logic  [5:0] ag_cnt;
logic [15:0] ag_ratio;
/* verilator lint_off UNUSEDSIGNAL */
wire  [32:0] ag_sh = {ag_rem, ag_num[31]};
wire         ag_ge = (ag_sh >= {1'b0, ag_den});
wire  [32:0] ag_nx = ag_ge ? (ag_sh - {1'b0, ag_den}) : ag_sh;
/* verilator lint_on UNUSEDSIGNAL */
logic [16:0] lgain;
/* verilator lint_off UNUSEDSIGNAL */
// Both operands are 16 bits, so the product must be widened before the
// shift or SystemVerilog evaluates it at 16 bits and truncates.
wire  [31:0] lg_agc = ({16'd0, luma_gain} * {16'd0, ag_ratio}) >> 14;
/* verilator lint_on UNUSEDSIGNAL */

/* verilator lint_off UNUSEDSIGNAL */
wire signed [33:0] cg_mul = ibh * $signed({1'b0, inv});
wire signed [33:0] sg_mul = qbh * $signed({1'b0, inv});
wire signed [17:0] cg_raw =  cg_mul[BURST_SHIFT+17:BURST_SHIFT];
// Dividing by the burst vector is (i+jq)*conj(B)/|B|^2, so the imaginary part
// of the reference enters unnegated: the conjugate is already accounted for by
// the minus in Q's expression below. Negating here as well would turn the whole
// picture 180 degrees whenever the burst sits on the q axis, which is where a
// real one lands.
wire signed [17:0] sg_raw =  sg_mul[BURST_SHIFT+17:BURST_SHIFT];
wire signed [25:0] cg_sat = cg_raw * $signed({1'b0, sat});
wire signed [25:0] sg_sat = sg_raw * $signed({1'b0, sat});
/* verilator lint_on UNUSEDSIGNAL */

always_ff @(posedge clk) if (ce) begin
	if (hcnt == 10'd0) begin
		ib_acc <= 22'sd0;
		qb_acc <= 22'sd0;
		bmax   <= -16'sd32768;
		bmin   <=  16'sd32767;
	end
	else if (in_burst) begin
		ib_acc <= ib_acc + {{4{ibox[17]}}, ibox};
		qb_acc <= qb_acc + {{4{qbox[17]}}, qbox};
		if (c16 > bmax) bmax <= c16;
		if (c16 < bmin) bmin <= c16;
	end

	if (burst_fin) begin
		ib  <= sat16(ib_acc[21:BURST_SH]);
		qb  <= sat16(qb_acc[21:BURST_SH]);
		bpp <= (bmax > bmin) ? $unsigned(bmax - bmin) : 16'd0;
	end

	// Hue is applied to the burst vector, once, rather than to every sample.
	// Rotating the reference the other way is the same picture and costs four
	// multiplies a line instead of four a sample.
	if (burst_fin) div_go <= 1'b1;
	else           div_go <= 1'b0;

	if (div_go) begin
		ibh     <= rot_i[24:9];
		qbh     <= rot_q[24:9];
		div_cnt <= 6'd32;
		div_rem <= 32'd0;
		div_quo <= 32'd0;
		div_num <= DIV_NUM;
		div_den <= mag2;
		ag_cnt  <= 6'd32;
		ag_rem  <= 32'd0;
		ag_quo  <= 32'd0;
		ag_num  <= {2'd0, BURST_PP, 14'd0};
		ag_den  <= {16'd0, bpp};
	end
	else if (|div_cnt) begin
		div_cnt <= div_cnt - 6'd1;
		div_num <= {div_num[30:0], 1'b0};
		div_quo <= {div_quo[30:0], rem_ge};
		div_rem <= rem_nx[31:0];
		// On the last step the quotient's final bit is still only on the wire,
		// so it has to be taken with the register or the result comes out halved.
		if (div_cnt == 6'd1) begin
			colour_ok <= (div_den >= MIN_MAG2);
			inv       <= |div_quo[30:15] ? 16'hFFFF : {div_quo[14:0], rem_ge};
		end
	end

	if (|ag_cnt) begin
		ag_cnt <= ag_cnt - 6'd1;
		ag_num <= {ag_num[30:0], 1'b0};
		ag_quo <= {ag_quo[30:0], ag_ge};
		ag_rem <= ag_nx[31:0];
		if (ag_cnt == 6'd1)
			// Out of range means no usable burst: keep the fixed gain rather than
			// let a monochrome or broken line swing the brightness.
			ag_ratio <= (|ag_quo[30:15] || {ag_quo[14:0], ag_ge} < 16'd4096)
			              ? 16'd16384 : {ag_quo[14:0], ag_ge};
	end

	lgain <= agc_en ? lg_agc[16:0] : {1'b0, luma_gain};

	cg <= colour_ok ? cg_sat[24:7] : 18'sd0;
	sg <= colour_ok ? sg_sat[24:7] : 18'sd0;
end

// ------------------------------------------- burst correction and chroma trail
//
// The trail is asymmetric on purpose. A symmetric filter with the luma delay
// trimmed to match would bleed colour evenly both ways, which is not what
// composite looks like: a real set lowpasses demodulated chroma with a causal
// analogue filter, an exponential decay to the right, and runs chroma at a
// longer group delay than the luma path, which is only notched. Both push
// colour rightward. So the one-pole IIR supplies the tail, and luma_delay is
// meant to be set BELOW the chroma path's own delay so chroma lags.

logic signed [17:0] i_cor, q_cor, i_sm, q_sm;

/* verilator lint_off UNUSEDSIGNAL */
wire signed [36:0] icor_m = ibox * cg + qbox * sg;
wire signed [36:0] qcor_m = qbox * cg - ibox * sg;
/* verilator lint_on UNUSEDSIGNAL */
wire signed [17:0] i_err  = i_cor - i_sm;
wire signed [17:0] q_err  = q_cor - q_sm;

always_ff @(posedge clk) if (ce) begin
	i_cor <= icor_m[29:12];
	q_cor <= qcor_m[29:12];
	i_sm  <= |smear ? (i_sm + (i_err >>> smear)) : i_cor;
	q_sm  <= |smear ? (q_sm + (q_err >>> smear)) : q_cor;
end

// ------------------------------------------------------------- luma alignment

logic [15:0][15:0] ybuf;
logic signed [15:0] y_dly;

always_ff @(posedge clk) if (ce) begin
	ybuf  <= {ybuf[14:0], y_lp};
	y_dly <= $signed(ybuf[luma_delay]);
end

// ----------------------------------------------------------------- to RGB
//
// Fixed scale, no auto-gain: white is 0.714V and that is what becomes 255. A
// source that runs hotter than broadcast range is supposed to come out
// brighter, and to clip here rather than earlier.

logic signed [17:0] y8, i8, q8;

/* verilator lint_off UNUSEDSIGNAL */
// Black sits `setup` above blanking, and the clamp put blanking at zero, so
// setup comes off before the gain. Chroma is a difference and carries no
// setup, and it is already level-corrected by the burst division, so it takes
// the plain gain and not the AGC ratio.
wire signed [16:0] y_sub = y_dly - setup;
wire signed [18:0] lg    = $signed({2'b0, lgain});
wire signed [35:0] y8_m  = y_sub * lg;
wire signed [35:0] i8_m  = i_sm  * $signed({1'b0, luma_gain});
wire signed [35:0] q8_m  = q_sm  * $signed({1'b0, luma_gain});
/* verilator lint_on UNUSEDSIGNAL */

always_ff @(posedge clk) if (ce) begin
	y8 <= y8_m[33:16];
	i8 <= i8_m[33:16];
	q8 <= q8_m[33:16];
end

wire signed [27:0] y_sh  = {{2{y8[17]}}, y8, 8'd0};
wire signed [27:0] r_mix = y_sh + (i8 * 18'sd245 + q8 * 18'sd159);
wire signed [27:0] g_mix = y_sh - (i8 * 18'sd70  + q8 * 18'sd166);
wire signed [27:0] b_mix = y_sh - (i8 * 18'sd283 - q8 * 18'sd436);

/* verilator lint_off UNUSEDSIGNAL */
function automatic [7:0] clamp8(input signed [27:0] v);
	logic signed [19:0] s;
	s = v[27:8];
	if      (s > 20'sd255) clamp8 = 8'd255;
	else if (s < 20'sd0)   clamp8 = 8'd0;
	else                   clamp8 = s[7:0];
endfunction
/* verilator lint_on UNUSEDSIGNAL */

// --------------------------------------------------------------------- output

localparam LAT = 12;
logic [LAT-1:0] hs_p, vs_p, hb_p, vb_p;

always_ff @(posedge clk) begin
	ce_out <= ce;
	if (ce) begin
		hs_p <= {hs_p[LAT-2:0], hs_in};
		vs_p <= {vs_p[LAT-2:0], vs_in};
		hb_p <= {hb_p[LAT-2:0], hb_in};
		vb_p <= {vb_p[LAT-2:0], vb_in};

		hs_out <= hs_p[LAT-1];
		vs_out <= vs_p[LAT-1];
		hb_out <= hb_p[LAT-1];
		vb_out <= vb_p[LAT-1];

		r_out <= clamp8(r_mix);
		g_out <= clamp8(g_mix);
		b_out <= clamp8(b_mix);
	end
end

endmodule
