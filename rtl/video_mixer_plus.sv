//
//
// Copyright (c) 2017,2021 Alexey Melnikov
//
// This program is GPL Licensed. See COPYING for the full license.
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// video_mixer_plus: video_mixer with a composite-decode branch added beside the
// scandoubler. Everything outside that branch is unchanged from the framework's
// sys/video_mixer.sv; keep it that way so upstream updates stay easy to reapply.
//
// The branch is an alternative, not an addition. Selecting it bypasses the
// scandoubler, scanlines and hq2x, and skips gamma.
//

`timescale 1ns / 1ps

//
// LINE_LENGTH: Length of display line in pixels when HBlank = 0;
// HALF_DEPTH:  If =1 then color dept is 4 bits per component
//
// altera message_off 10720 
// altera message_off 12161

module video_mixer_plus
#(
	parameter LINE_LENGTH  = 768,
	parameter COMP_SPC     = 8,    // composite samples per subcarrier cycle, even
	parameter HALF_DEPTH   = 0,
	parameter GAMMA        = 0
)
(
	input            CLK_VIDEO, // should be multiple by (ce_pix*4)
	output reg       CE_PIXEL,  // output pixel clock enable

	input            ce_pix,    // input pixel clock or clock_enable

	input            scandoubler,
	input            hq2x, 	    // high quality 2x scaling

	// Composite decode branch. See rtl/composite_decoder.md for the signal
	// format and what each config port does.
	input            use_composite,
	input            ce_comp,           // one composite sample
	input     [23:0] composite,         // signed Q2.21 volts, 1.0 = 1.0V
	// The branch takes its own sync. The composite signal is generated before
	// the normal video path's delays, so HSync here would be several pixels
	// late and the decoder would look for the burst in the wrong place.
	input            comp_hs,
	input            comp_vs,
	input            comp_hb,
	input            comp_vb,
	input      [9:0] comp_burst_start,  // burst window, samples after HSync falls
	input      [9:0] comp_burst_len,
	input      [7:0] comp_sat,          // 128 = unity
	input      [7:0] comp_hue,          // signed, 256 = one full cycle
	input      [3:0] comp_smear,        // chroma trail length, 0 = off
	input      [3:0] comp_luma_delay,   // luma path delay in samples; set it below
	                                    // the chroma delay for the rightward trail
	input     [15:0] comp_setup,        // black above blanking, Q2.13 volts: 0 for
	                                    // NTSC-J and consoles, 439 for NTSC-M
	input     [15:0] comp_luma_gain,    // volts to white, Q16
	input            comp_agc,          // track level off the burst

	inout     [21:0] gamma_bus,

	// color
	input [DWIDTH:0] R,
	input [DWIDTH:0] G,
	input [DWIDTH:0] B,

	// Positive pulses.
	input            HSync,
	input            VSync,
	input            HBlank,
	input            VBlank,

	// Freeze engine
	// HDMI: displays last frame 
	// VGA:  black screen with HSync and VSync
	input            HDMI_FREEZE,
	output           freeze_sync,

	// video output signals
	output reg [7:0] VGA_R,
	output reg [7:0] VGA_G,
	output reg [7:0] VGA_B,
	output reg       VGA_VS,
	output reg       VGA_HS,
	output reg       VGA_DE
);

localparam DWIDTH = HALF_DEPTH ? 3 : 7;
localparam DWIDTH_SD = GAMMA ? 7 : DWIDTH;
localparam HALF_DEPTH_SD = GAMMA ? 0 : HALF_DEPTH;

wire frz_hs, frz_vs;
wire frz_hbl, frz_vbl;
video_freezer freezer
(
	.clk(CLK_VIDEO),
	.freeze(HDMI_FREEZE),
	.hs_in(HSync),
	.vs_in(VSync),
	.hbl_in(HBlank),
	.vbl_in(VBlank),
	.sync(freeze_sync),
	.hs_out(frz_hs),
	.vs_out(frz_vs),
	.hbl_out(frz_hbl),
	.vbl_out(frz_vbl)
);

reg frz;
always @(posedge CLK_VIDEO) begin
	reg frz1;
	
	frz1 <= HDMI_FREEZE;
	frz  <= frz1;
end

generate
	if(GAMMA && HALF_DEPTH) begin
		wire [7:0] R_in  = frz ? 8'd0 : {R,R};
		wire [7:0] G_in  = frz ? 8'd0 : {G,G};
		wire [7:0] B_in  = frz ? 8'd0 : {B,B};
	end else begin
		wire [DWIDTH:0] R_in = frz ? 1'd0 : R;
		wire [DWIDTH:0] G_in = frz ? 1'd0 : G;
		wire [DWIDTH:0] B_in = frz ? 1'd0 : B;
	end
endgenerate

wire hs_g, vs_g;
wire hb_g, vb_g;
wire [DWIDTH_SD:0] R_gamma, G_gamma, B_gamma;

generate
	if(GAMMA) begin
		assign gamma_bus[21] = 1;
		gamma_corr gamma(
			.clk_sys(gamma_bus[20]),
			.clk_vid(CLK_VIDEO),
			.ce_pix(ce_pix),

			.gamma_en(gamma_bus[19]),
			.gamma_wr(gamma_bus[18]),
			.gamma_wr_addr(gamma_bus[17:8]),
			.gamma_value(gamma_bus[7:0]),

			.HSync(frz_hs),
			.VSync(frz_vs),
			.HBlank(frz_hbl),
			.VBlank(frz_vbl),
			.RGB_in({R_in,G_in,B_in}),

			.HSync_out(hs_g),
			.VSync_out(vs_g),
			.HBlank_out(hb_g),
			.VBlank_out(vb_g),
			.RGB_out({R_gamma,G_gamma,B_gamma})
		);
	end else begin
		assign gamma_bus[21] = 0;
		assign {R_gamma,G_gamma,B_gamma} = {R_in,G_in,B_in};
		assign {hs_g, vs_g, hb_g, vb_g} = {frz_hs, frz_vs, frz_hbl, frz_vbl};
	end
endgenerate

wire [DWIDTH_SD:0] R_sd;
wire [DWIDTH_SD:0] G_sd;
wire [DWIDTH_SD:0] B_sd;
wire hs_sd, vs_sd, hb_sd, vb_sd, ce_pix_sd;

scandoubler #(.LENGTH(LINE_LENGTH), .HALF_DEPTH(HALF_DEPTH_SD)) sd
(
	.clk_vid(CLK_VIDEO),
	.hq2x(hq2x),

	.ce_pix(ce_pix),
	.hs_in(hs_g),
	.vs_in(vs_g),
	.hb_in(hb_g),
	.vb_in(vb_g),
	.r_in(R_gamma),
	.g_in(G_gamma),
	.b_in(B_gamma),

	.ce_pix_out(ce_pix_sd),
	.hs_out(hs_sd),
	.vs_out(vs_sd),
	.hb_out(hb_sd),
	.vb_out(vb_sd),
	.r_out(R_sd),
	.g_out(G_sd),
	.b_out(B_sd)
);

wire [7:0] R_comp, G_comp, B_comp;
wire hs_comp, vs_comp, hb_comp, vb_comp, ce_comp_out;

// A freeze blanks this branch to 0V rather than holding it: there is no frame
// store here, and the sync comes from the composite domain, not the freezer.
composite_decoder #(.SPC(COMP_SPC)) comp_dec
(
	.clk(CLK_VIDEO),
	.ce(ce_comp),
	.comp(frz ? 24'd0 : composite),

	.hs_in(comp_hs),
	.vs_in(comp_vs),
	.hb_in(comp_hb),
	.vb_in(comp_vb),

	.burst_start(comp_burst_start),
	.burst_len(comp_burst_len),
	.sat(comp_sat),
	.hue(comp_hue),
	.smear(comp_smear),
	.luma_delay(comp_luma_delay),
	.setup($signed(comp_setup)),
	.luma_gain(comp_luma_gain),
	.agc_en(comp_agc),

	.ce_out(ce_comp_out),
	.hs_out(hs_comp),
	.vs_out(vs_comp),
	.hb_out(hb_comp),
	.vb_out(vb_comp),
	.r_out(R_comp),
	.g_out(G_comp),
	.b_out(B_comp)
);

wire [DWIDTH_SD:0] rt = (scandoubler ? R_sd : R_gamma);
wire [DWIDTH_SD:0] gt = (scandoubler ? G_sd : G_gamma);
wire [DWIDTH_SD:0] bt = (scandoubler ? B_sd : B_gamma);

always @(posedge CLK_VIDEO) begin
	reg [7:0] r,g,b;
	reg hde,vde,hs,vs, old_vs;
	reg old_hde;
	reg old_ce;
	reg ce_osc, fs_osc;
	
	old_ce <= ce_pix;
	ce_osc <= ce_osc | (old_ce ^ ce_pix);

	old_vs <= vs;
	if(~old_vs & vs) begin
		fs_osc <= ce_osc;
		ce_osc <= 0;
	end

	CE_PIXEL <= use_composite ? ce_comp_out : scandoubler ? ce_pix_sd : fs_osc ? (~old_ce & ce_pix) : ce_pix;

	if(use_composite) begin
		r <= R_comp;
		g <= G_comp;
		b <= B_comp;
	end
	else if(!GAMMA && HALF_DEPTH) begin
		r <= {rt,rt};
		g <= {gt,gt};
		b <= {bt,bt};
	end
	else begin
		r <= rt;
		g <= gt;
		b <= bt;
	end

	hde <= use_composite ? ~hb_comp : scandoubler ? ~hb_sd : ~hb_g;
	vde <= use_composite ? ~vb_comp : scandoubler ? ~vb_sd : ~vb_g;
	vs  <= use_composite ?  vs_comp : scandoubler ?  vs_sd :  vs_g;
	hs  <= use_composite ?  hs_comp : scandoubler ?  hs_sd :  hs_g;

	if(CE_PIXEL) begin
		VGA_R <= r;
		VGA_G <= g;
		VGA_B <= b;

		VGA_VS <= vs;
		VGA_HS <= hs;

		old_hde <= hde;
		if(old_hde ^ hde) VGA_DE <= vde & hde;
	end
end

endmodule
