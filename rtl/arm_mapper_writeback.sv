// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

// Commit mapper-owned 32-bit table updates through the independent ARM RAM
// port. Each held payload remains stable until the RAM accepts its write.
module arm_mapper_writeback
(
	input  logic        clk_sys,
	input  logic        reset_sys,
	input  logic        pointer_write,
	input  logic [14:0] pointer_addr,
	input  logic [31:0] pointer_wdata,
	input  logic        map_write,
	input  logic [14:0] map_addr,
	input  logic [31:0] map_wdata,
	output logic        idle,

	input  logic        clk_arm,
	input  logic        reset_arm,
	output logic        ram_en,
	output logic        ram_write,
	output logic [14:0] ram_addr,
	output logic [31:0] ram_wdata,
	output logic  [3:0] ram_wstrb,
	input  logic        ram_accepted
);
	logic [14:0] pointer_addr_payload;
	logic [31:0] pointer_data_payload;
	logic pointer_toggle;
	logic pointer_ack_arm;
	logic pointer_ack_sync1;
	logic pointer_ack_sync2;
	logic [14:0] map_addr_payload;
	logic [31:0] map_data_payload;
	logic map_toggle;
	logic map_ack_arm;
	logic map_ack_sync1;
	logic map_ack_sync2;

	assign idle = pointer_ack_sync2 == pointer_toggle &&
		map_ack_sync2 == map_toggle;

	always @(posedge clk_sys) begin
		if (reset_sys) begin
			pointer_addr_payload <= '0;
			pointer_data_payload <= '0;
			pointer_toggle <= 1'b0;
			pointer_ack_sync1 <= 1'b0;
			pointer_ack_sync2 <= 1'b0;
			map_addr_payload <= '0;
			map_data_payload <= '0;
			map_toggle <= 1'b0;
			map_ack_sync1 <= 1'b0;
			map_ack_sync2 <= 1'b0;
		end else begin
			pointer_ack_sync1 <= pointer_ack_arm;
			pointer_ack_sync2 <= pointer_ack_sync1;
			map_ack_sync1 <= map_ack_arm;
			map_ack_sync2 <= map_ack_sync1;

			if (pointer_write && pointer_ack_sync2 == pointer_toggle) begin
				pointer_addr_payload <= pointer_addr;
				pointer_data_payload <= pointer_wdata;
				pointer_toggle <= ~pointer_toggle;
			end

			if (map_write && map_ack_sync2 == map_toggle) begin
				map_addr_payload <= map_addr;
				map_data_payload <= map_wdata;
				map_toggle <= ~map_toggle;
			end
		end
	end

	logic pointer_sync1;
	logic pointer_sync2;
	logic map_sync1;
	logic map_sync2;
	logic active;
	logic active_map;
	logic active_token;
	logic [14:0] active_addr;
	logic [31:0] active_data;

	assign ram_en = active;
	assign ram_write = active;
	assign ram_addr = active_addr;
	assign ram_wdata = active_data;
	assign ram_wstrb = 4'hF;

	always @(posedge clk_arm) begin
		if (reset_arm) begin
			pointer_sync1 <= 1'b0;
			pointer_sync2 <= 1'b0;
			pointer_ack_arm <= 1'b0;
			map_sync1 <= 1'b0;
			map_sync2 <= 1'b0;
			map_ack_arm <= 1'b0;
			active <= 1'b0;
			active_map <= 1'b0;
			active_token <= 1'b0;
			active_addr <= '0;
			active_data <= '0;
		end else begin
			pointer_sync1 <= pointer_toggle;
			pointer_sync2 <= pointer_sync1;
			map_sync1 <= map_toggle;
			map_sync2 <= map_sync1;

			if (active) begin
				if (ram_accepted) begin
					if (active_map)
						map_ack_arm <= active_token;
					else
						pointer_ack_arm <= active_token;
					active <= 1'b0;
				end
			end else if (pointer_sync2 != pointer_ack_arm) begin
				active <= 1'b1;
				active_map <= 1'b0;
				active_token <= pointer_sync2;
				active_addr <= pointer_addr_payload;
				active_data <= pointer_data_payload;
			end else if (map_sync2 != map_ack_arm) begin
				active <= 1'b1;
				active_map <= 1'b1;
				active_token <= map_sync2;
				active_addr <= map_addr_payload;
				active_data <= map_data_payload;
			end
		end
	end
endmodule
