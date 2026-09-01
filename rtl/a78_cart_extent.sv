// Cartridge extent for a downloaded image.
//
// Decision 0066: for any cartridge carrying an A78 header, the cartridge is the
// smaller of the header's declared ROM size (big-endian, bytes 49-52) and the
// number of payload bytes actually downloaded. The declared size may only
// shrink the cartridge, never grow it, so no header - corrupt, mis-stamped, or
// hostile - can make the core address a byte that was never written.
//
// A declared size of zero means undeclared, not empty, and selects the whole
// download. 2600 images carry no header and always use the download length.
//
// Bytes past the extent stay in memory but are not cartridge. Only a
// Souper cartridge with a valid ARSC block claims them; see
// .agents/formats/BUPCHIP_APPENDED_ROM_ARSC.md.

module a78_cart_extent
(
	input  logic        clk,
	input  logic        cart_download,
	input  logic        ioctl_wr,
	input  logic [24:0] ioctl_addr,
	input  logic  [7:0] ioctl_dout,
	input  logic        cart_is_7800,
	input  logic        tia_mode,

	output logic [31:0] cart_size
);

logic [31:0] cart_size_eof; // payload bytes downloaded so far
logic [31:0] hcart_size;    // header bytes 49-52; zero means undeclared

initial begin
	cart_size_eof = 32'h00008000;
	hcart_size = 32'd0;
end

always_ff @(posedge clk) begin
	if (cart_download && ioctl_wr) begin
		// A file shorter than its own header has no payload. Without this the
		// subtraction wraps and a 40-byte A78 reports a 4 GiB cartridge.
		if (cart_is_7800 && ioctl_addr < 25'd128)
			cart_size_eof <= 32'd0;
		else
			cart_size_eof <= (ioctl_addr - (cart_is_7800 ? 8'd128 : 1'b0)) + 1'd1;

		// A new cartridge never inherits the last one's header. Neither this
		// clear nor the tia_mode one below is observable today: cart_is_7800
		// gates the clamp, and any A78 long enough for the extent to matter
		// rewrites bytes 49-52 itself. They are kept so the invariant holds on
		// its own rather than as a consequence of the clamp's current shape,
		// and the mutation proof records that they cannot be killed.
		if (~|ioctl_addr)
			hcart_size <= 32'd0;
	end

	if (cart_download) begin
		if (tia_mode) begin
			hcart_size <= 32'd0;
		end else begin
			case (ioctl_addr)
				'd49: hcart_size[31:24] <= ioctl_dout;
				'd50: hcart_size[23:16] <= ioctl_dout;
				'd51: hcart_size[15:8]  <= ioctl_dout;
				'd52: hcart_size[7:0]   <= ioctl_dout;
				default: ;
			endcase
		end
	end
end

always_comb begin
	if (cart_is_7800 && |hcart_size && hcart_size < cart_size_eof)
		cart_size = hcart_size;
	else
		cart_size = cart_size_eof;
end

endmodule
