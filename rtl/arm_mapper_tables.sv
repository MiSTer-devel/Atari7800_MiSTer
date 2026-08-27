// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

// Registered BUS/CDF stream tables. Shared cartridge RAM remains authoritative;
// these two M10Ks remove its pointer/increment reads from the 6507 deadline.
module arm_mapper_tables
(
	input  logic        clk_sys,
	input  logic  [1:0] family,
	input  logic  [2:0] revision,
	input  logic  [5:0] pointer_lookup_index,
	input  logic  [5:0] increment_lookup_index,
	output logic [31:0] pointer,
	output logic [31:0] increment,
	output logic [14:0] pointer_base,
	output logic [14:0] increment_base,
	output logic [14:0] map_base,
	output logic  [5:0] stream_count,

	input  logic        sys_pointer_write,
	input  logic  [5:0] sys_pointer_index,
	input  logic [31:0] sys_pointer_wdata,
	input  logic        sys_increment_write,
	input  logic  [5:0] sys_increment_index,
	input  logic [31:0] sys_increment_wdata,
	input  logic        sys_map_write,
	input  logic  [5:0] sys_map_index,
	input  logic [31:0] sys_map_wdata,

	input  logic        clk_arm,
	input  logic        arm_write,
	input  logic        arm_accepted,
	input  logic [14:0] arm_addr,
	input  logic [31:0] arm_wdata,
	input  logic  [3:0] arm_wstrb
);
	localparam logic [1:0] FAMILY_BUS = 2'd1;
	localparam logic [1:0] FAMILY_CDF = 2'd2;

	logic        arm_pointer_write;
	logic        arm_increment_write;
	logic        arm_map_write;
	logic  [5:0] arm_pointer_index;
	logic  [5:0] arm_increment_index;
	logic  [5:0] arm_map_index;
	logic  [5:0] pointer_addr_sys;
	logic  [5:0] increment_addr_sys;
	logic        sys_map_write_valid;
	wire [31:0] pointer_arm_unused;
	wire [31:0] increment_arm_unused;

	// The layout is fixed before the ARM runs, so the fast port works from its
	// own copy of the family and revision that choose it. Reading them live
	// put the cartridge-select decode - which starts two clock domains away in
	// clk_vid - on this port's write enable, where only 3.5 ns of the ARM
	// period is reachable.
	logic  [1:0] family_sys, family_arm;
	logic  [2:0] revision_sys, revision_arm;
	logic [14:0] pointer_base_arm, increment_base_arm, map_base_arm;
	logic  [5:0] stream_count_arm;

	always @(posedge clk_sys) begin
		family_sys <= family;
		revision_sys <= revision;
	end

	always @(posedge clk_arm) begin
		family_arm <= family_sys;
		revision_arm <= revision_sys;
	end

	// {pointer base, increment base, map base, stream count}
	function automatic [50:0] layout(input [1:0] f, input [2:0] r);
		logic [14:0] pb, ib, mb;
		logic  [5:0] sc;
	begin
		pb = 15'b0;
		ib = 15'b0;
		mb = 15'b0;
		sc = 6'b0;

		case (f)
			FAMILY_BUS: begin
				case (r)
					3'd0: begin
						pb = 15'h2B8;
						ib = 15'h2C8;
						sc = 6'd16;
					end
					3'd3: begin
						pb = 15'h1B6;
						ib = 15'h1C8;
						sc = 6'd18;
					end
					default: begin
						pb = 15'h1B8;
						ib = 15'h1C8;
						sc = 6'd16;
					end
				endcase
				mb = r == 3'd0 ? 15'h2D9 : 15'h1D8;
			end

			FAMILY_CDF: begin
				case (r)
					3'd0: begin
						pb = 15'h1B8;
						ib = 15'h1DA;
						sc = 6'd34;
					end
					3'd1: begin
						pb = 15'h028;
						ib = 15'h04A;
						sc = 6'd34;
					end
					default: begin
						pb = 15'h026;
						ib = 15'h049;
						sc = 6'd35;
					end
				endcase
			end

			default: ;
		endcase

		layout = {pb, ib, mb, sc};
	end
	endfunction

	always_comb begin
		{pointer_base, increment_base, map_base, stream_count} =
			layout(family, revision);
		{pointer_base_arm, increment_base_arm, map_base_arm, stream_count_arm} =
			layout(family_arm, revision_arm);
	end


	assign arm_pointer_index = arm_addr[5:0] - pointer_base_arm[5:0];
	assign arm_increment_index = arm_addr[5:0] - increment_base_arm[5:0];
	assign arm_map_index = 6'd16 + arm_addr[5:0] - map_base_arm[5:0];
	assign arm_pointer_write = arm_write && arm_accepted &&
		arm_addr >= pointer_base_arm &&
		arm_addr < pointer_base_arm + {9'b0, stream_count_arm};
	assign arm_increment_write = arm_write && arm_accepted &&
		arm_addr >= increment_base_arm &&
		arm_addr < increment_base_arm + {9'b0, stream_count_arm};
	assign arm_map_write = arm_write && arm_accepted &&
		family_arm == FAMILY_BUS &&
		arm_addr >= map_base_arm && arm_addr < map_base_arm + 15'd37;
	assign pointer_addr_sys = sys_pointer_write ?
		sys_pointer_index : pointer_lookup_index;
	assign sys_map_write_valid = sys_map_write && family == FAMILY_BUS;
	assign increment_addr_sys = sys_increment_write ? sys_increment_index :
		(sys_map_write_valid ? 6'd16 + sys_map_index : increment_lookup_index);

	cache_ram_tdp_dc_be #(.ADDR_WIDTH(6), .DATA_WIDTH(32)) pointer_ram (
		.clk_a_i     (clk_sys),
		.addr_a_i    (pointer_addr_sys),
		.wren_a_i    (sys_pointer_write),
		.byteena_a_i (4'hF),
		.wdata_a_i   (sys_pointer_wdata),
		.q_a_o       (pointer),
		.clk_b_i     (clk_arm),
		.addr_b_i    (arm_pointer_index),
		.wren_b_i    (arm_pointer_write),
		.byteena_b_i (arm_wstrb),
		.wdata_b_i   (arm_wdata),
		.q_b_o       (pointer_arm_unused)
	);

	cache_ram_tdp_dc_be #(.ADDR_WIDTH(6), .DATA_WIDTH(32)) increment_ram (
		.clk_a_i     (clk_sys),
		.addr_a_i    (increment_addr_sys),
		.wren_a_i    (sys_increment_write || sys_map_write_valid),
		.byteena_a_i (4'hF),
		.wdata_a_i   (sys_map_write_valid ? sys_map_wdata : sys_increment_wdata),
		.q_a_o       (increment),
		.clk_b_i     (clk_arm),
		.addr_b_i    (arm_map_write ? arm_map_index : arm_increment_index),
		.wren_b_i    (arm_increment_write || arm_map_write),
		.byteena_b_i (arm_wstrb),
		.wdata_b_i   (arm_wdata),
		.q_b_o       (increment_arm_unused)
	);
endmodule
