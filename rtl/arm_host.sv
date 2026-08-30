// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

// The core's single ARM7TDMI and the seam where its clients attach.
//
// Exactly one CPU is shared between the Atari 2600 call mappers and the Souper
// BupChip player. Their address maps, storage, caches and
// peripherals are completely separate, so each lives with the cartridge path it
// serves and only this bus crosses. There is one client today; when the second
// arrives, the profile mux goes here rather than in either cartridge module.
module arm_host
(
	input  logic        clk_arm,
	input  logic        reset_arm,

	input  logic        halt_req,
	output logic        halted,

	output logic        mem_req,
	input  logic        mem_ready,
	input  logic        mem_abort,
	output logic [31:0] mem_addr,
	output logic        mem_write,
	output logic [31:0] mem_wdata,
	input  logic [31:0] mem_rdata,
	output logic  [1:0] mem_size,
	output logic  [3:0] mem_wstrb,
	output logic        mem_seq,
	output logic        mem_fetch,
	output logic        mem_privileged,
	output logic        mem_lock,
	output logic        retire,

	input  logic        state_req,
	input  logic        state_write,
	input  logic  [5:0] state_index,
	input  logic [31:0] state_wdata,
	output logic [31:0] state_rdata,
	output logic        state_ready,
	input  logic        state_commit
);
	arm7tdmi_core arm_cpu (
		.clk            (clk_arm),
		.reset          (reset_arm),
		.ce             (1'b1),
		.irq_n          (1'b1),
		.fiq_n          (1'b1),
		.halt_req,
		.halted,
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
endmodule
