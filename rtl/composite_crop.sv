// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

// Overscan crop for the decoded composite picture: the same window the RGB
// path shows, applied by extending the receiver's blanking. Positions are in
// decoder pixels from HB falling and lines from VB falling, measured with the
// encoder-to-decoder chain test, so they move if the decoder's delays do.
//
//   MARIA  border shown   pixels 1-744    overscan shown   all lines
//          border hidden  pixels 51-690   overscan hidden  lines 6-229 (PAL 279)
//   TIA                   pixels 73-712                    all lines
//
// The TIA's own vertical blanking is not applied: a set shows whatever the
// program draws inside the receiver's window, black included.

module composite_crop (
	input  wire       clk,
	input  wire       pix,      // one output pixel
	input  wire       hb_in,
	input  wire       vb_in,
	input  wire       is_maria,
	input  wire       is_pal,
	input  wire       show_border,
	input  wire       show_overscan,
	output wire       hb_out,
	output wire       vb_out
);

reg [9:0] px;
reg [8:0] ln;
reg hb_d;

always @(posedge clk) if (pix) begin
	hb_d <= hb_in;
	px <= hb_in ? 10'd0 : px + 10'd1;
	if (vb_in) ln <= 9'd0;
	else if (hb_in && !hb_d) ln <= ln + 9'd1;
end

wire [9:0] left  = !is_maria ? 10'd73  : show_border ? 10'd1   : 10'd51;
wire [9:0] right = !is_maria ? 10'd712 : show_border ? 10'd744 : 10'd690;
wire [8:0] bottom = is_pal ? 9'd279 : 9'd229;

wire crop_h = (px < left) || (px > right);
wire crop_v = is_maria && !show_overscan && (ln < 9'd6 || ln > bottom);

assign hb_out = hb_in | crop_h;
assign vb_out = vb_in | crop_v;

endmodule
