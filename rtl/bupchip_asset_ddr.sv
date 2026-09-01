// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

// The ARSC asset window: the appended BupChip resources, captured out of the
// cartridge download and served to the ARM from DDR3.
//
// The block is 212 KiB for Rikki & Vikki, which is why it lives in DDR3 rather
// than M10K. It is read-only to the ARM and is published only after the whole
// download has landed, so the firmware can never read a half-written asset.
//
// Bytes arrive on clk_sys and are consumed on clk_arm, so the capture side
// packs a 64-bit word and hands it across with a toggle handshake - the same
// shape arm_mapper_memory uses for its ROM shadow, for the same reason: a
// byte-rate CDC would be both slower and harder to reason about.
//
// The read side is a small direct-mapped cache with 32-byte lines, filled by a
// four-beat burst. That shape is chosen by how CoreTone reads: sixteen voices
// each walk their own sample sequentially, so reads are sequential *within* a
// voice and scattered *across* them. A single line would be thrashed sixteen
// ways and every read would become a DDR3 round trip; sixteen lines keep a line
// per voice, and a 32-byte line turns roughly eight round trips into one.
//
// This matters more on hardware than in any model: the MiSTer DDR3 window is
// shared with the HPS, so its latency is both high and variable, and a render
// that misses the 240 Hz deadline empties the PCM FIFO and drops audio.

module bupchip_asset_ddr #(
	parameter logic [28:0] ASSET_BASE_WORD = 29'h07000000
) (
	input  logic        clk_sys,
	input  logic        clk_arm,
	input  logic        reset_arm,

	// Cartridge download, clk_sys. load_addr is the file offset, so it still
	// includes the 128-byte A78 header.
	input  logic        load_start,
	input  logic [24:0] load_addr,
	input  logic        load_valid,
	input  logic  [7:0] load_data,
	input  logic        load_end,
	// Reported for debug only. The block's start is taken from the A78 header
	// as it streams past, not from this port - see below.
	output logic [24:0] asset_start,

	// ARM side.
	input  logic        rd_req,
	input  logic [31:0] rd_addr,	// byte offset from the ARSC tag
	output logic [31:0] rd_data,
	output logic        rd_valid,
	output logic [31:0] asset_size,
	output logic        asset_ready,
	output logic        load_wait,

	// DDR3 channel.
	output logic [28:0] ddr_addr,
	output logic [63:0] ddr_din,
	output logic  [7:0] ddr_be,
	output logic  [7:0] ddr_len,
	output logic        ddr_req,
	output logic        ddr_rnw,
	input  logic        ddr_ack,
	input  logic [63:0] ddr_dout,
	input  logic        ddr_rvalid,
	// The bridge gave up on this channel's transaction. Nothing arrived and
	// nothing more will; re-issue.
	input  logic        ddr_timeout
);
	// ---- capture side (clk_sys) --------------------------------------------
	// Bytes accumulate into pack_*; a finished word is copied into pub_* and
	// announced with a toggle. The copy matters: publishing pack_* directly
	// would hand the ARM side the *next* word's index, because the same clock
	// edge that completes one word already starts the next.
	logic [63:0] pack_word, pub_word;
	logic  [7:0] pack_be, pub_be;
	logic [21:0] pack_index, pub_index;
	logic        pack_toggle;
	logic [24:0] size_sys;
	logic        end_toggle;
	logic        pub_ack_sync1, pub_ack_sync2;

	// The ARSC block begins at 128 + the header's declared ROM size. That has
	// to come from the header bytes as they stream past rather than from the
	// core's cart_size, because cart_size is still counting up while the
	// download runs: sampling it live would start capturing at whatever offset
	// the counter happened to hold, not at the block.
	logic [31:0] declared_size;
	always_ff @(posedge clk_sys) begin
		if (load_start)
			declared_size <= 32'b0;
		else if (load_valid) begin
			case (load_addr)
				25'd49: declared_size[31:24] <= load_data;
				25'd50: declared_size[23:16] <= load_data;
				25'd51: declared_size[15:8]  <= load_data;
				25'd52: declared_size[7:0]   <= load_data;
				default: ;
			endcase
		end
	end
	assign asset_start = declared_size[24:0] + 25'd128;

	wire in_block = load_valid && |declared_size && load_addr >= asset_start;
	wire [24:0] block_offset = load_addr - asset_start;

	always_ff @(posedge clk_sys) begin
		if (load_start) begin
			pack_word <= 64'b0;
			pack_be <= 8'b0;
			pack_index <= 22'b0;
			size_sys <= 25'b0;
		end else begin
			if (in_block) begin
				// Publish whenever the incoming byte belongs to a different
				// word than the one being packed. The download is sequential,
				// so that is the word boundary.
				if (pack_be != 8'b0 && block_offset[24:3] != pack_index) begin
					pub_word <= pack_word;
					pub_be <= pack_be;
					pub_index <= pack_index;
					pack_toggle <= ~pack_toggle;
					pack_word <= 64'b0;
					pack_be <= 8'b0;
				end
				pack_index <= block_offset[24:3];
				pack_word[{block_offset[2:0], 3'b000} +: 8] <= load_data;
				pack_be[block_offset[2:0]] <= 1'b1;
				size_sys <= block_offset + 25'd1;
			end
			if (load_end) begin
				// The tail word, which is usually partial.
				if (pack_be != 8'b0) begin
					pub_word <= pack_word;
					pub_be <= pack_be;
					pub_index <= pack_index;
					pack_toggle <= ~pack_toggle;
				end
				end_toggle <= ~end_toggle;
			end
		end
	end

	// Backpressure. A word is published every eight bytes and a DDR write takes
	// a few clk_arm cycles, so this almost never fires - but without it a slow
	// DDR3 window could have pub_* overwritten while it was still being
	// written, which would drop eight bytes of a sample with no other symptom.
	always_ff @(posedge clk_sys) begin
		pub_ack_sync1 <= pub_ack;
		pub_ack_sync2 <= pub_ack_sync1;
	end
	assign load_wait = pack_toggle != pub_ack_sync2;

	// ---- CDC ----------------------------------------------------------------
	logic pack_sync1, pack_sync2, pack_seen;
	logic end_sync1, end_sync2, end_seen;

	logic        pub_ack;

	// ---- read cache (clk_arm) ----------------------------------------------
	// Byte address layout: [1:0] byte in word, [2] word in beat,
	// [4:3] beat in line, [8:5] line, [24:9] tag.
	localparam int LINE_BEATS = 4;                 // 32-byte line
	localparam int LINES      = 16;
	localparam int LINE_AW    = 4;                 // $clog2(LINES)
	localparam int TAG_MSB    = 24;
	localparam int TAG_LSB    = 9;
	localparam int TAG_W      = TAG_MSB - TAG_LSB + 1;

	typedef enum logic [2:0] {
		D_IDLE, D_WRITE, D_MISS_CMD, D_MISS_FILL, D_PRESENT, D_ANSWER
	} dstate_e;
	dstate_e dstate;

	// 16 x 17 bits of tag is 34 bytes, under the threshold that would want a
	// block RAM, and it has to be compared combinationally to decide a hit.
	logic [TAG_W-1:0] tag [0:LINES-1];
	logic [LINES-1:0] tag_valid;

	logic [31:0] req_addr_q;
	logic  [1:0] fill_beat;
	logic        rd_served;

	wire [LINE_AW-1:0] req_line = req_addr_q[8:5];
	wire [TAG_W-1:0]   req_tag  = req_addr_q[TAG_MSB:TAG_LSB];
	wire hit = tag_valid[req_line] && tag[req_line] == req_tag;

	// Cache data: 16 lines x 4 beats of 64 bits = 512 bytes, which AGENTS.md
	// wants in a wrapper rather than inferred.
	logic  [5:0] cache_addr;
	logic        cache_wren;
	logic [63:0] cache_q;

	cache_ram #(
		.ADDR_WIDTH (6),
		.DATA_WIDTH (64)
	) cache_data (
		.clk_i   (clk_arm),
		.addr_i  (cache_addr),
		.wren_i  (cache_wren),
		.wdata_i (ddr_dout),
		.q_o     (cache_q)
	);

	assign cache_wren = dstate == D_MISS_FILL && ddr_rvalid;
	assign cache_addr = (dstate == D_MISS_FILL) ? {req_line, fill_beat}
	                                            : {req_line, req_addr_q[4:3]};

	assign rd_data = req_addr_q[2] ? cache_q[63:32] : cache_q[31:0];

	// Writes are single beat during capture; reads pull a whole line.
	assign ddr_len = (dstate == D_WRITE) ? 8'd1 : 8'(LINE_BEATS);
	assign ddr_din = pub_word;
	assign ddr_be  = pub_be;
	assign ddr_rnw = dstate != D_WRITE;
	assign ddr_req = dstate == D_WRITE || dstate == D_MISS_CMD;
	assign ddr_addr = (dstate == D_WRITE)
		? ASSET_BASE_WORD + {7'b0, pub_index}
		: ASSET_BASE_WORD + {7'b0, req_addr_q[24:5], 2'b00};

	always_ff @(posedge clk_arm) begin
		if (reset_arm) begin
			dstate <= D_IDLE;
			pack_sync1 <= 1'b0; pack_sync2 <= 1'b0; pack_seen <= 1'b0;
			pub_ack <= 1'b0;
			end_sync1 <= 1'b0; end_sync2 <= 1'b0; end_seen <= 1'b0;
			tag_valid <= '0;
			asset_ready <= 1'b0;
			asset_size <= 32'b0;
			rd_valid <= 1'b0;
			rd_served <= 1'b0;
			fill_beat <= 2'd0;
		end else begin
			pack_sync1 <= pack_toggle; pack_sync2 <= pack_sync1;
			end_sync1 <= end_toggle;  end_sync2 <= end_sync1;
			rd_valid <= 1'b0;

			// rd_req is a level, and bupchip_memory only drops it after it has
			// seen rd_valid. So D_ANSWER hands back to a D_IDLE that still sees
			// the request it just answered, and starts a second read of the
			// same address. That copy is a cache hit, finishes three cycles
			// later, and its rd_valid lands on the *next* request - which is
			// then answered with the previous word. One flag per request is
			// what keeps the level from being counted twice.
			if (!rd_req)
				rd_served <= 1'b0;

			case (dstate)
				D_IDLE: begin
					if (pack_sync2 != pack_seen) begin
						pack_seen <= pack_sync2;
						dstate <= D_WRITE;
					end else if (end_sync2 != end_seen) begin
						end_seen <= end_sync2;
						asset_size <= {7'b0, size_sys};
						asset_ready <= 1'b1;
						tag_valid <= '0;
					end else if (rd_req && asset_ready && !rd_served) begin
						req_addr_q <= rd_addr;
						dstate <= D_PRESENT;
					end
				end

				// Capture writes go straight through and invalidate the cache;
				// they only happen before the assets are published, so this
				// costs nothing during playback.
				D_WRITE: if (ddr_ack) begin
					tag_valid <= '0;
					pub_ack <= pack_seen;
					dstate <= D_IDLE;
				end

				// req_addr_q settled a cycle ago, so the tag compare is stable.
				D_PRESENT: dstate <= hit ? D_ANSWER : D_MISS_CMD;

				D_MISS_CMD: if (ddr_ack) begin
					fill_beat <= 2'd0;
					dstate <= D_MISS_FILL;
				end

				// A fill that loses a beat must not be left half-done: the
				// line is incomplete, so it stays invalid and the read is
				// asked for again. D_WRITE and D_MISS_CMD need no such arm -
				// ddr_req is driven by dstate, so staying put re-requests on
				// its own once the bridge is out of quarantine.
				D_MISS_FILL: if (ddr_timeout)
					dstate <= D_MISS_CMD;
				else if (ddr_rvalid) begin
					fill_beat <= fill_beat + 2'd1;
					if (fill_beat == 2'(LINE_BEATS - 1)) begin
						tag[req_line] <= req_tag;
						tag_valid[req_line] <= 1'b1;
						dstate <= D_PRESENT;
					end
				end

				// cache_q is valid the cycle after its address was presented.
				D_ANSWER: begin
					rd_valid <= 1'b1;
					rd_served <= 1'b1;
					dstate <= D_IDLE;
				end

				default: dstate <= D_IDLE;
			endcase
		end
	end
endmodule
