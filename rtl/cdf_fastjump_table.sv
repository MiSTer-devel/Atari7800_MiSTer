// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

// One bit per 6507-visible CDF or BUS3 program byte. The real cartridge can
// inspect both JMP operands before replacing either; this table preserves that
// lookahead while the FPGA ROM remains on a byte-wide registered interface.
// The first operand is admitted as 0 or 1, which is the CDFJ mask; the mappers
// that want only 0 reject the odd case when they read the operand.
module cdf_fastjump_table
(
	input  logic        clk_sys,
	input  logic        load_start,
	input  logic [24:0] load_addr,
	input  logic        load_valid,
	input  logic  [7:0] load_data,
	input  logic [14:0] query_addr,
	output logic        query_valid
);
	logic [7:0] byte_minus_two;
	logic [7:0] byte_minus_one;
	logic       map_wren;
	logic [14:0] map_addr;
	logic       map_wdata;
	logic       map_q;

	always_comb begin
		map_wren = load_valid && load_addr < 25'd32768;
		map_addr = query_addr;
		map_wdata = 1'b0;

		if (map_wren) begin
			// Bytes zero and one clear the two entries that have no following
			// operands. Every remaining entry is overwritten at byte N+2.
			map_addr = load_addr < 25'd2 ?
				15'h7FFE + {14'b0, load_addr[0]} :
				load_addr[14:0] - 15'd2;
			map_wdata = load_addr >= 25'd2 && byte_minus_two == 8'h4C &&
				byte_minus_one[7:1] == 7'b0 && load_data == 8'b0;
		end
	end

	cache_ram #(
		.ADDR_WIDTH (15),
		.DATA_WIDTH (1)
	) map_ram (
		.clk_i   (clk_sys),
		.addr_i  (map_addr),
		.wren_i  (map_wren),
		.wdata_i (map_wdata),
		.q_o     (map_q)
	);

	assign query_valid = map_q;

	always_ff @(posedge clk_sys) begin
		if (load_start) begin
			byte_minus_two <= 8'b0;
			byte_minus_one <= load_valid ? load_data : 8'b0;
		end else if (load_valid) begin
			byte_minus_two <= byte_minus_one;
			byte_minus_one <= load_data;
		end
	end

	initial begin
		byte_minus_two = 8'b0;
		byte_minus_one = 8'b0;
	end
endmodule
