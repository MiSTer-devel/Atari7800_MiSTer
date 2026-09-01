// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

// The Souper BupChip player: firmware ROM, private RAM, the ARSC asset window,
// the $8007 command FIFO and the 48 kHz PCM FIFO, around the core's shared
// ARM7TDMI.
//
// The CPU itself is not here. Only its bus crosses in, the same way the 2600
// call mappers attach, so exactly one arm7tdmi_core is ever elaborated.
//
// The CPU is held in reset until the whole cartridge has landed and the asset
// window is published, so the firmware cannot read a half-written asset. It
// then boots from address zero, which is the firmware's reset vector - no state
// programming is needed for this profile.

module bupchip_subsystem #(
	// clk_arm is the PLL's outclk_3: 572.65625 MHz VCO / 8. rtl/pll/pll_0002.v
	// asks for 71.59091 MHz and Atari7800.sdc records what it gets. 57.272728
	// MHz is outclk_1, clk_vid - using it here drained the FIFO at 60 kHz.
	parameter int CLK_ARM_HZ = 71582031,
	parameter int PCM_HZ = 48000,
	parameter     ROM_MIF  = "bupchip.mif",
	parameter     ROM_INIT = "rtl/bupchip.hex"
) (
	input  logic        clk_sys,
	input  logic        clk_arm,
	input  logic        reset_arm,
	input  logic        enabled,		// Souper cartridge selected

	// Cartridge download, clk_sys.
	input  logic        load_start,
	input  logic [24:0] load_addr,
	input  logic        load_valid,
	input  logic  [7:0] load_data,
	input  logic        load_end,
	output logic [24:0] asset_start,

	// $8007 commands, clk_sys, one event per write pair.
	input  logic        cmd_valid_sys,
	input  logic  [7:0] cmd_data_sys,

	// ARM bus.
	input  logic        mem_req,
	input  logic [31:0] mem_addr,
	input  logic        mem_write,
	input  logic [31:0] mem_wdata,
	input  logic  [3:0] mem_wstrb,
	input  logic  [1:0] mem_size,
	output logic        mem_ready,
	output logic        mem_abort,
	output logic [31:0] mem_rdata,
	output logic        arm_hold,		// hold the CPU in reset
	output logic        load_wait,		// backpressure for the download stream

	// DDR3 channel (the core gives this profile its own).
	output logic [28:0] ddr_addr,
	output logic [63:0] ddr_din,
	output logic  [7:0] ddr_be,
	output logic  [7:0] ddr_len,
	output logic        ddr_req,
	output logic        ddr_rnw,
	input  logic        ddr_ack,
	input  logic [63:0] ddr_dout,
	input  logic        ddr_rvalid,
	input  logic        ddr_timeout,

	// Audio, clk_sys.
	// Signed PCM carried as plain bits; the mixer applies the offset.
	output logic [15:0] audio_l,
	output logic [15:0] audio_r
);
	logic        asset_rd_req, asset_rd_valid, asset_ready;
	logic [31:0] asset_rd_addr, asset_rd_data, asset_size;

	logic        reg_sel, reg_write;
	logic  [7:0] reg_addr;
	logic [31:0] reg_wdata, reg_rdata;

	logic        pcm_pop, pcm_available, pcm_enabled, muted;
	logic [31:0] pcm_frame;
	logic  [7:0] fault_code;

	// Boot only once the assets are all there.
	assign arm_hold = !(enabled && asset_ready);

	bupchip_asset_ddr assets (
		.clk_sys, .clk_arm, .reset_arm,
		.load_start, .load_addr, .load_valid, .load_data, .load_end,
		.asset_start,
		.rd_req(asset_rd_req), .rd_addr(asset_rd_addr),
		.rd_data(asset_rd_data), .rd_valid(asset_rd_valid),
		.asset_size, .asset_ready, .load_wait,
		.ddr_addr, .ddr_din, .ddr_be, .ddr_len, .ddr_req, .ddr_rnw,
		.ddr_ack, .ddr_dout, .ddr_rvalid, .ddr_timeout
	);

	bupchip_memory #(.ROM_MIF(ROM_MIF), .ROM_INIT(ROM_INIT)) memory (
		.clk(clk_arm), .reset(reset_arm || arm_hold),
		.mem_req(mem_req && !arm_hold), .mem_addr, .mem_write, .mem_wdata,
		.mem_wstrb, .mem_size, .mem_ready, .mem_abort, .mem_rdata,
		.asset_req(asset_rd_req), .asset_addr(asset_rd_addr),
		.asset_rdata(asset_rd_data), .asset_valid(asset_rd_valid), .asset_size,
		.reg_sel, .reg_addr, .reg_write, .reg_wdata, .reg_rdata
	);

	// Commands cross clk_sys to clk_arm on a toggle: they are rare - the game
	// sends at most one per frame - so a FIFO here would be all cost.
	logic       cmd_tog_sys, cmd_tog1, cmd_tog2, cmd_tog3;
	logic [7:0] cmd_byte_sys;
	logic       cmd_valid_arm;
	logic [7:0] cmd_data_arm;

	always_ff @(posedge clk_sys)
		if (cmd_valid_sys) begin
			cmd_byte_sys <= cmd_data_sys;
			cmd_tog_sys <= ~cmd_tog_sys;
		end

	always_ff @(posedge clk_arm) begin
		cmd_tog1 <= cmd_tog_sys;
		cmd_tog2 <= cmd_tog1;
		cmd_tog3 <= cmd_tog2;
		cmd_valid_arm <= cmd_tog2 ^ cmd_tog3;
		cmd_data_arm <= cmd_byte_sys;
	end

	bupchip_peripheral peripheral (
		.clk(clk_arm), .reset(reset_arm || arm_hold),
		.cmd_valid(cmd_valid_arm), .cmd_data(cmd_data_arm),
		.reg_sel, .reg_addr, .reg_write, .reg_wdata, .reg_rdata,
		.pcm_pop, .pcm_frame, .pcm_available, .pcm_enabled, .muted, .fault_code
	);

	// ---- 48 kHz drain, and the crossing back to the audio clock -------------
	localparam int POP_DIV = CLK_ARM_HZ / PCM_HZ;
	localparam int POP_AW = $clog2(POP_DIV);
	localparam logic [POP_AW-1:0] POP_COUNT_MAX = POP_AW'(POP_DIV - 1);

	logic [POP_AW-1:0] pop_count;
	logic [31:0] frame_arm;
	logic        frame_tog;

	always_ff @(posedge clk_arm) begin
		pcm_pop <= 1'b0;
		if (reset_arm || arm_hold) begin
			pop_count <= '0;
			frame_arm <= 32'b0;
		end else begin
			pop_count <= pop_count + 1'b1;
			if (pop_count == POP_COUNT_MAX) begin
				pop_count <= '0;
				// Only once the firmware has enabled output. Popping before
				// that drains a FIFO nobody has filled yet, which registers as
				// an underrun for every frame of the boot sequence and buries
				// the real ones.
				pcm_pop <= pcm_enabled;
				// An empty FIFO plays silence rather than the last frame:
				// holding a sample is a buzz, silence is a gap.
				frame_arm <= pcm_available ? pcm_frame : 32'b0;
				frame_tog <= ~frame_tog;
			end
		end
	end

	// frame_arm is stable between toggles, so a plain two-flop sync of the
	// toggle is enough to capture both channels without tearing.
	logic tog1, tog2;
	always_ff @(posedge clk_sys) begin
		tog1 <= frame_tog;
		tog2 <= tog1;
		if (tog1 ^ tog2) begin
			audio_l <= muted ? 16'd0 : frame_arm[15:0];
			audio_r <= muted ? 16'd0 : frame_arm[31:16];
		end
		if (!enabled) begin
			audio_l <= 16'd0;
			audio_r <= 16'd0;
		end
	end
endmodule
