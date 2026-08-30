// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

// The existing 128 KiB cartridge RAM reshaped as four byte lanes. Port A is
// the mapper's byte bus; port B is the ARM subsystem's independent word bus.
module cart_ram_tdp
(
	input  logic        clk_sys,
	input  logic        mapper_en,
	input  logic        mapper_write,
	input  logic [16:0] mapper_addr,
	input  logic  [7:0] mapper_wdata,
	output logic  [7:0] mapper_rdata,

	input  logic        clk_arm,
	input  logic        arm_en,
	input  logic        arm_write,
	input  logic [14:0] arm_addr,
	input  logic [31:0] arm_wdata,
	input  logic  [3:0] arm_wstrb,
	output logic [31:0] arm_rdata,
	output logic        arm_accepted,
	output logic [31:0] mapper_word_rdata
);
	logic [1:0] mapper_read_lane;
	wire [7:0] mapper_q [0:3];
	wire [7:0] arm_q [0:3];

	// clk_arm is exactly five times clk_sys off the same PLL, so one ARM edge
	// in five falls on the mapper's edge and the other four stand a whole ARM
	// period clear of it. Sitting that one edge out is all the arbitration the
	// two ports need, and it keeps the mapper's address - the far end of a 20 ns
	// decode out of cartridge ROM - away from this port's write enable.
	logic sys_toggle = 1'b0;
	logic sys_toggle_arm = 1'b0;
	logic sys_toggle_arm_d = 1'b0;
	logic [2:0] arm_phase = 3'd0;
	wire mapper_edge = arm_phase == 3'd4;
	wire arm_allow = arm_en && !mapper_edge;

	always @(posedge clk_sys)
		sys_toggle <= ~sys_toggle;

	// sys_toggle flips on the shared edge and reaches clk_arm one ARM cycle
	// later, so the two copies differ during phase 1. Counting on from there,
	// phase 4 is the ARM cycle whose own edge is the shared one.
	always @(posedge clk_arm) begin
		sys_toggle_arm <= sys_toggle;
		sys_toggle_arm_d <= sys_toggle_arm;
		if (sys_toggle_arm != sys_toggle_arm_d)
			arm_phase <= 3'd2;
		else
			arm_phase <= arm_phase == 3'd4 ? 3'd0 : arm_phase + 3'd1;
	end

	assign arm_accepted = arm_allow;
	assign mapper_rdata = mapper_q[mapper_read_lane];
	assign mapper_word_rdata = {mapper_q[3], mapper_q[2], mapper_q[1], mapper_q[0]};
	assign arm_rdata = {arm_q[3], arm_q[2], arm_q[1], arm_q[0]};

	always @(posedge clk_sys) begin
		if (mapper_en)
			mapper_read_lane <= mapper_addr[1:0];
	end

	genvar lane;
	generate
		for (lane = 0; lane < 4; lane = lane + 1) begin : ram_lane
			cache_ram_tdp_dc #(
				.ADDR_WIDTH (15),
				.DATA_WIDTH (8)
			) lane_ram (
				.clk_a_i   (clk_sys),
				.addr_a_i  (mapper_addr[16:2]),
				.wren_a_i  (mapper_en && mapper_write && mapper_addr[1:0] == lane[1:0]),
				.wdata_a_i (mapper_wdata),
				.q_a_o     (mapper_q[lane]),
				.clk_b_i   (clk_arm),
				.addr_b_i  (arm_addr),
				.wren_b_i  (arm_allow && arm_write && arm_wstrb[lane]),
				.wdata_b_i (arm_wdata[(lane*8)+:8]),
				.q_b_o     (arm_q[lane])
			);
		end
	endgenerate
endmodule
