// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

// Adapts the FA2 byte handshake to the console-side port of the existing
// MiSTer save RAM. The block-file engine can hold requests off while loading.
module fa2_nvram_bridge
(
	input  logic       clk,
	input  logic       reset,
	input  logic       new_image,
	input  logic       backing_busy,
	input  logic       file_load_complete,
	input  logic       request,
	input  logic       write,
	input  logic [7:0] addr,
	input  logic [7:0] wdata,
	output logic       ready,
	output logic [7:0] rdata,
	input  logic [7:0] backing_rdata,
	output logic       backing_cs,
	output logic       backing_write,
	output logic [7:0] backing_addr,
	output logic [7:0] backing_wdata
);
	logic backing_valid;
	wire accept = request && !ready && !backing_busy;

	assign backing_cs = request;
	assign backing_write = accept && write;
	assign backing_addr = addr;
	assign backing_wdata = wdata;
	assign rdata = backing_valid ? backing_rdata : 8'h00;

	always_ff @(posedge clk) begin
		if (!request)
			ready <= 1'b0;
		else if (accept)
			ready <= 1'b1;

		if (new_image)
			backing_valid <= 1'b0;
		else if (file_load_complete || backing_write)
			backing_valid <= 1'b1;

		if (reset) begin
			ready <= 1'b0;
			backing_valid <= 1'b0;
		end
	end
endmodule
