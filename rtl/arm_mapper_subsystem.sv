// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

// One ARM service shared by the DPC+, BUS, and CDF mapper front ends.
module arm_mapper_subsystem
(
	input  logic        clk_sys,
	input  logic        reset_sys,
	input  logic        mapper_reset,
	input  logic        load_start,
	input  logic [24:0] load_addr,
	input  logic        load_valid,
	input  logic        load_end,
	input  logic [31:0] load_size,
	input  logic  [7:0] load_data,
	output logic        load_wait,

	input  logic        clk_arm,
	input  logic        reset_arm,
	output logic        shadow_ready,
	input  logic [15:0] mapper_ram_size,
	input  logic        dma_request,
	input  logic        dma_fill,
	input  logic [24:0] dma_source,
	input  logic [16:0] dma_dest,
	input  logic [17:0] dma_count,
	input  logic  [7:0] dma_value,
	output logic        dma_ready,
	output logic        dma_busy,
	output logic        dma_done,
	input  logic        sample_request,
	input  logic [24:0] sample_addr,
	output logic        sample_ready,
	output logic        sample_busy,
	output logic        sample_done,
	output logic  [7:0] sample_data,

	input  logic        call_request,
	input  logic [31:0] call_entry,
	input  logic [31:0] call_stack,
	input  logic        call_thumb,
	input  logic [31:0] audio_counter0,
	input  logic [31:0] audio_counter1,
	input  logic [31:0] audio_counter2,
	input  logic [31:0] audio_frequency0,
	input  logic [31:0] audio_frequency1,
	input  logic [31:0] audio_frequency2,
	output logic        call_ready,
	output logic        call_busy,
	output logic        call_done,
	output logic [31:0] audio_counter0_return,
	output logic [31:0] audio_counter1_return,
	output logic [31:0] audio_counter2_return,
	output logic [31:0] audio_frequency0_return,
	output logic [31:0] audio_frequency1_return,
	output logic [31:0] audio_frequency2_return,
	output logic        arm_halted,

	output logic        ram_en,
	output logic        ram_write,
	output logic [14:0] ram_addr,
	output logic [31:0] ram_wdata,
	output logic  [3:0] ram_wstrb,
	input  logic        ram_accepted,
	input  logic [31:0] ram_rdata,

	output logic        ddram_clk,
	output logic [28:0] ddram_addr,
	output logic  [7:0] ddram_burstcnt,
	input  logic        ddram_busy,
	input  logic [63:0] ddram_dout,
	input  logic        ddram_dout_ready,
	output logic        ddram_rd,
	output logic [63:0] ddram_din,
	output logic  [7:0] ddram_be,
	output logic        ddram_we
);
	logic mapper_reset_sync1;
	logic mapper_reset_arm;

	always @(posedge clk_arm) begin
		if (reset_arm) begin
			mapper_reset_sync1 <= 1'b1;
			mapper_reset_arm <= 1'b1;
		end else begin
			mapper_reset_sync1 <= mapper_reset;
			mapper_reset_arm <= mapper_reset_sync1;
		end
	end

	logic halt_req;
	logic mem_req;
	logic mem_ready;
	logic mem_abort;
	logic [31:0] mem_addr;
	logic mem_write;
	logic [31:0] mem_wdata;
	logic [31:0] mem_rdata;
	logic  [1:0] mem_size;
	logic  [3:0] mem_wstrb;
	logic mem_seq;
	logic mem_fetch;
	logic mem_privileged;
	logic mem_lock;
	logic return_fetch;
	logic retire;
	logic state_req;
	logic state_write;
	logic  [5:0] state_index;
	logic [31:0] state_wdata;
	logic [31:0] state_rdata;
	logic state_ready;
	logic state_commit;

	arm7tdmi_core arm_cpu (
		.clk            (clk_arm),
		.reset          (reset_arm),
		.ce             (1'b1),
		.irq_n          (1'b1),
		.fiq_n          (1'b1),
		.halt_req,
		.halted         (arm_halted),
		.mem_req,
		.mem_ready,
		.mem_abort,
		.mem_addr,
		.mem_write,
		.mem_wdata,
		.mem_rdata,
		.mem_size,
		.mem_wstrb,
		.mem_seq,
		.mem_fetch,
		.mem_privileged,
		.mem_lock,
		.retire,
		.state_req,
		.state_write,
		.state_index,
		.state_wdata,
		.state_rdata,
		.state_ready,
		.state_commit
	);

	arm_mapper_controller call_controller (
		.clk_sys,
		.reset_sys,
		.mapper_reset_sys (mapper_reset),
		.call_request,
		.call_entry,
		.call_stack,
		.call_thumb,
		.audio_counter0,
		.audio_counter1,
		.audio_counter2,
		.audio_frequency0,
		.audio_frequency1,
		.audio_frequency2,
		.call_ready,
		.call_busy,
		.call_done,
		.audio_counter0_return,
		.audio_counter1_return,
		.audio_counter2_return,
		.audio_frequency0_return,
		.audio_frequency1_return,
		.audio_frequency2_return,
		.clk_arm,
		.reset_arm,
		.mapper_reset_arm,
		.shadow_ready,
		.return_fetch    (return_fetch),
		.cpu_halted      (arm_halted),
		.halt_req,
		.state_req,
		.state_write,
		.state_index,
		.state_wdata,
		.state_ready,
		.state_rdata,
		.state_commit
	);

	arm_mapper_memory memory (
		.clk_sys,
		.reset_sys,
		.load_start,
		.load_addr,
		.load_valid,
		.load_end,
		.load_size,
		.load_data,
		.load_wait,
		.clk_arm,
		.reset_arm,
		.mapper_reset    (mapper_reset_arm),
		.shadow_ready,
		.mapper_ram_size,
		.dma_request,
		.dma_fill,
		.dma_source,
		.dma_dest,
		.dma_count,
		.dma_value,
		.dma_ready,
		.dma_busy,
		.dma_done,
		.sample_request,
		.sample_addr,
		.sample_ready,
		.sample_busy,
		.sample_done,
		.sample_data,
		.mem_ce          (1'b1),
		.mem_req,
		.mem_addr,
		.mem_write,
		.mem_wdata,
		.mem_size,
		.mem_wstrb,
		.mem_fetch,
		.mem_ready,
		.mem_abort,
		.mem_rdata,
		.return_fetch,
		.ram_en,
		.ram_write,
		.ram_addr,
		.ram_wdata,
		.ram_wstrb,
		.ram_accepted,
		.ram_rdata,
		.ddram_clk,
		.ddram_addr,
		.ddram_burstcnt,
		.ddram_busy,
		.ddram_dout,
		.ddram_dout_ready,
		.ddram_rd,
		.ddram_din,
		.ddram_be,
		.ddram_we
	);
endmodule
