// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

// Two-channel bridge onto the MiSTer DDR3 window.
//
// The framework port is one shared resource with an untagged response: beats
// come back in order with nothing saying who asked. The bridge owns that rule
// so its consumers do not have to. It records the requesting channel when the
// command is accepted, not when a beat returns, so a channel that is reset
// mid-transaction cannot have its beats delivered to the other one.
//
//   ch_req    ____/‾‾‾‾‾‾‾‾‾‾‾‾\________________   held until ack
//   ch_ack    _______________/‾\________________   command accepted
//   ch_rvalid ____________________/‾\___/‾\_____   one pulse per beat, in order
//
// A channel holds ch_req until it sees ch_ack, then counts ch_len beats on
// ch_rvalid for a read. A write returns no beats and is finished at ch_ack.
// One transaction is outstanding at a time.
module ddram
(
	input  logic        clk,
	input  logic        reset,

	// MiSTer DDR3 port
	output logic        DDRAM_CLK,
	input  logic        DDRAM_BUSY,
	output logic [7:0]  DDRAM_BURSTCNT,
	output logic [28:0] DDRAM_ADDR,
	input  logic [63:0] DDRAM_DOUT,
	input  logic        DDRAM_DOUT_READY,
	output logic        DDRAM_RD,
	output logic [63:0] DDRAM_DIN,
	output logic [7:0]  DDRAM_BE,
	output logic        DDRAM_WE,

	// Channel 1
	input  logic [28:0] ch1_addr,
	input  logic [63:0] ch1_din,
	input  logic  [7:0] ch1_be,
	input  logic  [7:0] ch1_len,
	input  logic        ch1_req,
	input  logic        ch1_rnw,
	output logic        ch1_ack,
	output logic [63:0] ch1_dout,
	output logic        ch1_rvalid,

	// Channel 2
	input  logic [28:0] ch2_addr,
	input  logic [63:0] ch2_din,
	input  logic  [7:0] ch2_be,
	input  logic  [7:0] ch2_len,
	input  logic        ch2_req,
	input  logic        ch2_rnw,
	output logic        ch2_ack,
	output logic [63:0] ch2_dout,
	output logic        ch2_rvalid
);
	assign DDRAM_CLK = clk;

	// Read data is broadcast unchanged; only the valid is routed. Data without
	// a valid is harmless, and this keeps a 64-bit mux off the return path.
	assign ch1_dout = DDRAM_DOUT;
	assign ch2_dout = DDRAM_DOUT;

	logic       presented;      // a command is on the wires, not yet taken
	logic       presented_rnw;
	logic [7:0] presented_len;
	logic       owner;          // channel the outstanding transaction belongs to
	logic [7:0] beats_left;

	// The port takes whatever is presented on the first cycle it is not busy.
	wire accept = presented && !DDRAM_BUSY;

	// Beats are not gated on DDRAM_BUSY: the port can raise busy for the next
	// command while it is still returning the previous one's data.
	wire beat = DDRAM_DOUT_READY && beats_left != 8'd0;

	// ch1 first. The profiles are exclusive today, so this only decides order
	// if that ever stops being true.
	wire pick2 = !ch1_req && ch2_req;

	assign ch1_ack    = accept && !owner;
	assign ch2_ack    = accept &&  owner;
	assign ch1_rvalid = beat   && !owner;
	assign ch2_rvalid = beat   &&  owner;

	always_ff @(posedge clk) begin
		if (reset) begin
			presented <= 1'b0;
			presented_rnw <= 1'b1;
			presented_len <= 8'd1;
			owner <= 1'b0;
			// Beats already in flight still arrive after a reset. Leaving the
			// count at zero drops them, so no channel is given an rvalid for a
			// transaction that no longer exists.
			beats_left <= 8'd0;
			DDRAM_RD <= 1'b0;
			DDRAM_WE <= 1'b0;
			DDRAM_ADDR <= '0;
			DDRAM_BURSTCNT <= 8'd1;
			DDRAM_DIN <= '0;
			DDRAM_BE <= '0;
		end else begin
			if (beat)
				beats_left <= beats_left - 8'd1;

			if (accept) begin
				presented <= 1'b0;
				DDRAM_RD <= 1'b0;
				DDRAM_WE <= 1'b0;
				if (presented_rnw)
					beats_left <= presented_len;
			end else if (!presented && beats_left == 8'd0 &&
				(ch1_req || ch2_req)) begin
				presented <= 1'b1;
				owner <= pick2;
				presented_rnw <= pick2 ? ch2_rnw : ch1_rnw;
				presented_len <= pick2 ? ch2_len : ch1_len;
				DDRAM_ADDR <= pick2 ? ch2_addr : ch1_addr;
				DDRAM_DIN <= pick2 ? ch2_din : ch1_din;
				DDRAM_BURSTCNT <= pick2 ? ch2_len : ch1_len;
				// Byte enables mean nothing on a read; the framework
				// convention is all lanes.
				DDRAM_BE <= (pick2 ? ch2_rnw : ch1_rnw) ? 8'hff :
					(pick2 ? ch2_be : ch1_be);
				DDRAM_RD <= pick2 ? ch2_rnw : ch1_rnw;
				DDRAM_WE <= pick2 ? !ch2_rnw : !ch1_rnw;
			end
		end
	end
endmodule
