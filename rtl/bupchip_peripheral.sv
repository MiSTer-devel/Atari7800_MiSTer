// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

// BupChip peripheral: the $8007 command FIFO and the 48 kHz PCM FIFO.
//
// Register map, widths and reset values are fixed by
// .agents/formats/BUPCHIP_FIRMWARE_CONTRACT.md section 4. The firmware asserts
// IDENT before touching anything else, so the map is a contract in both
// directions.
//
// The command side takes one event per $8007 *pair*, not per toggle: a game
// writes the register twice so that the open-drain request line emits exactly
// one complete pulse whatever level it started at. Delivering per toggle would
// duplicate every command. See
// .agents/evidence/bupchip_8007_command_encoding.md.

module bupchip_peripheral #(
	parameter int CMD_DEPTH = 32,
	// 4096 frames is 85 ms of audio. The MiSTer DDR3 window is shared with the
	// HPS, so a render can stall for a long and unpredictable time; the FIFO is
	// what turns that into "the ARM catches up" instead of "the audio drops
	// out". 512 frames was 10.7 ms, which a single HPS burst can exceed.
	parameter int PCM_DEPTH = 4096
) (
	input  logic        clk,
	input  logic        reset,

	// Command source, already in this clock domain.
	input  logic        cmd_valid,
	input  logic  [7:0] cmd_data,

	// MMIO, answered combinationally in the access cycle.
	input  logic        reg_sel,
	input  logic  [7:0] reg_addr,
	input  logic        reg_write,
	input  logic [31:0] reg_wdata,
	output logic [31:0] reg_rdata,

	// PCM sink: one stereo frame per pop.
	input  logic        pcm_pop,
	output logic [31:0] pcm_frame,
	output logic        pcm_available,

	// Output rather than an internal signal a parent reaches into: the mixer
	// must not drain a FIFO the firmware has not enabled, and a hierarchical
	// reference for that is not synthesizable.
	output logic        pcm_enabled,

	output logic        muted,
	output logic  [7:0] fault_code
);
	localparam int CMD_AW = $clog2(CMD_DEPTH);
	localparam int PCM_AW = $clog2(PCM_DEPTH);

	localparam logic [31:0] IDENT_VALUE = 32'h42555001; // "BUP" + revision 1

	// ---- command FIFO -------------------------------------------------------
	logic  [7:0] cmd_mem [0:CMD_DEPTH-1];
	logic [CMD_AW:0] cmd_wr, cmd_rd;
	logic        cmd_overflow;

	wire [CMD_AW:0] cmd_level = cmd_wr - cmd_rd;
	wire cmd_empty = cmd_level == '0;
	wire cmd_full  = cmd_level == CMD_DEPTH[CMD_AW:0];

	// This one is read asynchronously and that is deliberate: the ARM expects
	// CMD_POP to answer in its access cycle, and 32 x 8 is 256 registers and a
	// 32:1 mux - small enough to spend in fabric, unlike the PCM FIFO.
	//
	// A read of CMD_POP consumes one entry, so it must be a real access and not
	// a stray decode: reg_sel with a write would otherwise pop silently.
	wire cmd_pop = reg_sel && !reg_write && reg_addr == 8'h04 && !cmd_empty;

	// ---- PCM FIFO -----------------------------------------------------------
	// AGENTS.md: memories over 64 bytes use a provided block-RAM wrapper. This
	// one is 2 KiB and needs a write and a read port at once, which is
	// cache_ram_tdp_dc with both ports on the same clock.
	//
	// It must not be an inferred array with an asynchronous read: no block RAM
	// has an asynchronous read port, so that would put 16,384 registers and a
	// 512:1 mux in the fabric - most of the device - while every simulation
	// still passed.
	logic [31:0] pcm_head;
	logic [PCM_AW:0] pcm_wr, pcm_rd;
	logic        pcm_underflow, pcm_overflow;
	logic        pcm_enable;
	logic [PCM_AW:0] pcm_watermark;

	wire [PCM_AW:0] pcm_level = pcm_wr - pcm_rd;
	wire pcm_empty = pcm_level == '0;
	wire pcm_full  = pcm_level == PCM_DEPTH[PCM_AW:0];
	wire pcm_push  = reg_sel && reg_write && reg_addr == 8'h10;

	assign pcm_available = pcm_enable && !pcm_empty;
	assign pcm_enabled = pcm_enable;
	assign pcm_frame = pcm_head;

	// Port A writes, port B reads. The head is presented one cycle after the
	// pointer moves, which is invisible here: frames are consumed once every
	// clk_arm/48000 cycles, over a thousand apart.
	cache_ram_tdp_dc #(
		.ADDR_WIDTH (PCM_AW),
		.DATA_WIDTH (32)
	) pcm_fifo (
		.clk_a_i   (clk),
		.addr_a_i  (pcm_wr[PCM_AW-1:0]),
		.wren_a_i  (pcm_push && !pcm_full),
		.wdata_a_i (reg_wdata),
		.q_a_o     (),
		.clk_b_i   (clk),
		.addr_b_i  (pcm_rd[PCM_AW-1:0]),
		.wren_b_i  (1'b0),
		.wdata_b_i (32'b0),
		.q_b_o     (pcm_head)
	);

	always_ff @(posedge clk) begin
		if (reset) begin
			cmd_wr <= '0;
			cmd_rd <= '0;
			cmd_overflow <= 1'b0;
			pcm_wr <= '0;
			pcm_rd <= '0;
			pcm_underflow <= 1'b0;
			pcm_overflow <= 1'b0;
			pcm_enable <= 1'b0;
			pcm_watermark <= '0;
			muted <= 1'b0;
			fault_code <= 8'd0;
		end else begin
			// Command in. Overflow is a test failure, never a silent drop, so
			// the byte is discarded but the sticky bit records that it was.
			if (cmd_valid) begin
				if (cmd_full)
					cmd_overflow <= 1'b1;
				else begin
					cmd_mem[cmd_wr[CMD_AW-1:0]] <= cmd_data;
					cmd_wr <= cmd_wr + 1'b1;
				end
			end
			if (cmd_pop)
				cmd_rd <= cmd_rd + 1'b1;

			// PCM in.
			if (pcm_push) begin
				if (pcm_full)
					pcm_overflow <= 1'b1;
				else
					pcm_wr <= pcm_wr + 1'b1;
			end

			// PCM out. A pop with nothing to give is an underrun: the mixer
			// asked for a frame the firmware had not produced.
			if (pcm_pop) begin
				if (pcm_empty)
					pcm_underflow <= 1'b1;
				else
					pcm_rd <= pcm_rd + 1'b1;
			end

			if (reg_sel && reg_write) begin
				case (reg_addr)
					8'h0C: begin
						if (reg_wdata[0]) cmd_overflow <= 1'b0;
						if (reg_wdata[1]) cmd_rd <= cmd_wr;
					end
					8'h18: begin
						pcm_enable <= reg_wdata[0];
						pcm_watermark <= reg_wdata[16 +: PCM_AW+1];
						if (reg_wdata[1]) begin
							pcm_underflow <= 1'b0;
							pcm_overflow <= 1'b0;
						end
					end
					8'h1C: begin
						// Any FAULT write mutes; the firmware only writes it
						// when it has decided it cannot continue.
						muted <= 1'b1;
						fault_code <= reg_wdata[7:0];
					end
					default: ;
				endcase
			end
		end
	end

	always_comb begin
		case (reg_addr)
			8'h00: reg_rdata = IDENT_VALUE;
			8'h04: reg_rdata = cmd_empty ? 32'b0
				: {23'b0, 1'b1, cmd_mem[cmd_rd[CMD_AW-1:0]]};
			8'h08: reg_rdata = {15'b0, cmd_overflow, 6'b0, cmd_full, cmd_empty,
				8'b0} | {24'b0, 8'(cmd_level)};
			8'h14: reg_rdata = {6'b0, pcm_overflow, pcm_underflow, 5'b0,
				(pcm_level < pcm_watermark), pcm_full, pcm_empty,
				16'b0} | {16'b0, 16'(pcm_level)};
			default: reg_rdata = 32'b0;
		endcase
	end
endmodule
