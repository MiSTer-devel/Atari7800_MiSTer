// Copyright (c) 2026 Jamie Blanks
// SPDX-License-Identifier: MIT

// Processor-style wrapper for arm7tdmi_core. Signals change on MCLK positive
// edges. This is a protocol-level pin contract, not electrical or half-cycle
// compatibility with the ARM7TDMI macrocell.

module arm7tdmi_pin_wrapper (
	input  logic        MCLK,
	input  logic        nRESET,
	input  logic        nIRQ,
	input  logic        nFIQ,
	input  logic        nWAIT,
	input  logic        ABORT,

	output logic [31:0] A,
	input  logic [31:0] DIN,
	output logic [31:0] DOUT,
	output logic        DOUT_EN,
	output logic        nMREQ,
	output logic        SEQ,
	output logic        nRW,
	output logic  [1:0] MAS,
	output logic  [3:0] BYTE_EN,
	output logic        nOPC,
	output logic        nTRANS,
	output logic        LOCK,
	output logic        TBIT
);

	logic        mem_req;
	logic [31:0] mem_addr;
	logic        mem_write;
	logic [31:0] mem_wdata;
	logic  [1:0] mem_size;
	logic  [3:0] mem_wstrb;
	logic        mem_seq;
	logic        mem_fetch;
	logic        mem_privileged;
	logic        mem_lock;
	logic        tbit_latched;
	logic        unused_halted;
	logic        unused_retire;
	logic [31:0] unused_state_rdata;
	logic        unused_state_ready;

	always_ff @(posedge MCLK) begin
		if (!nRESET)
			tbit_latched <= 1'b0;
		else if (mem_req && mem_fetch)
			tbit_latched <= (mem_size == 2'b01);
	end

	assign A = mem_addr;
	assign DOUT = mem_wdata;
	assign DOUT_EN = mem_req && mem_write;
	assign nMREQ = !mem_req;
	assign SEQ = mem_seq;
	assign nRW = mem_write;
	assign MAS = mem_size;
	assign BYTE_EN = mem_wstrb;
	assign nOPC = !mem_fetch;
	assign nTRANS = mem_privileged;
	assign LOCK = mem_lock;
	assign TBIT = mem_fetch ? (mem_size == 2'b01) : tbit_latched;

	arm7tdmi_core core (
		.clk(MCLK),
		.reset(!nRESET),
		.ce(1'b1),
		.irq_n(nIRQ),
		.fiq_n(nFIQ),
		.halt_req(1'b0),
		.halted(unused_halted),
		.mem_req(mem_req),
		.mem_ready(nWAIT),
		.mem_abort(ABORT),
		.mem_addr(mem_addr),
		.mem_write(mem_write),
		.mem_wdata(mem_wdata),
		.mem_rdata(DIN),
		.mem_size(mem_size),
		.mem_wstrb(mem_wstrb),
		.mem_seq(mem_seq),
		.mem_fetch(mem_fetch),
		.mem_privileged(mem_privileged),
		.mem_lock(mem_lock),
		.retire(unused_retire),
		.state_req(1'b0),
		.state_write(1'b0),
		.state_index(6'b0),
		.state_wdata(32'b0),
		.state_rdata(unused_state_rdata),
		.state_ready(unused_state_ready),
		.state_commit(1'b0)
	);

endmodule
