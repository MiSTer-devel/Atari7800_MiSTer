// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

// Mapper-owned ARM memory system. The HPS stream remains authoritative in
// external SDRAM; this block builds an immutable DDR3 shadow for ARM reads.
module arm_mapper_memory
#(
	parameter logic [28:0] SHADOW_BASE_WORD = 29'h06000000
)
(
	input  logic        clk_sys,
	input  logic        reset_sys,
	input  logic        load_start,
	input  logic [24:0] load_addr,
	input  logic        load_valid,
	input  logic        load_end,
	input  logic [31:0] load_size,
	input  logic  [7:0] load_data,
	output logic        load_wait,

	input  logic        clk_arm,
	input  logic        reset_arm,
	input  logic        mapper_reset,
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

	input  logic        mem_ce,
	input  logic        mem_req,
	input  logic [31:0] mem_addr,
	input  logic        mem_write,
	input  logic [31:0] mem_wdata,
	input  logic  [1:0] mem_size,
	input  logic  [3:0] mem_wstrb,
	input  logic        mem_fetch,
	output logic        mem_ready,
	output logic        mem_abort,
	output logic [31:0] mem_rdata,
	output logic        return_fetch,

	output logic        ram_en,
	output logic        ram_write,
	output logic [14:0] ram_addr,
	output logic [31:0] ram_wdata,
	output logic  [3:0] ram_wstrb,
	input  logic        ram_accepted,
	input  logic [31:0] ram_rdata,

	// DDR3 channel. Held request, one transaction at a time; see rtl/ddram.sv.
	output logic [28:0] ddr_addr,
	output logic [63:0] ddr_din,
	output logic  [7:0] ddr_be,
	output logic  [7:0] ddr_len,
	output logic        ddr_req,
	output logic        ddr_rnw,
	input  logic        ddr_ack,
	input  logic [63:0] ddr_dout,
	input  logic        ddr_rvalid
);
	function automatic logic [63:0] put_byte(
		input logic [63:0] old_word,
		input logic  [2:0] lane,
		input logic  [7:0] byte_value
	);
		begin
			put_byte = old_word;
			case (lane)
				3'd0: put_byte[7:0]   = byte_value;
				3'd1: put_byte[15:8]  = byte_value;
				3'd2: put_byte[23:16] = byte_value;
				3'd3: put_byte[31:24] = byte_value;
				3'd4: put_byte[39:32] = byte_value;
				3'd5: put_byte[47:40] = byte_value;
				3'd6: put_byte[55:48] = byte_value;
				default: put_byte[63:56] = byte_value;
			endcase
		end
	endfunction

	function automatic logic [7:0] put_enable(
		input logic [7:0] old_enable,
		input logic [2:0] lane
	);
		begin
			put_enable = old_enable;
			case (lane)
				3'd0: put_enable[0] = 1'b1;
				3'd1: put_enable[1] = 1'b1;
				3'd2: put_enable[2] = 1'b1;
				3'd3: put_enable[3] = 1'b1;
				3'd4: put_enable[4] = 1'b1;
				3'd5: put_enable[5] = 1'b1;
				3'd6: put_enable[6] = 1'b1;
				default: put_enable[7] = 1'b1;
			endcase
		end
	endfunction

	function automatic logic [7:0] get_byte(
		input logic [63:0] word_value,
		input logic  [2:0] lane
	);
		begin
			case (lane)
				3'd0: get_byte = word_value[7:0];
				3'd1: get_byte = word_value[15:8];
				3'd2: get_byte = word_value[23:16];
				3'd3: get_byte = word_value[31:24];
				3'd4: get_byte = word_value[39:32];
				3'd5: get_byte = word_value[47:40];
				3'd6: get_byte = word_value[55:48];
				default: get_byte = word_value[63:56];
			endcase
		end
	endfunction

	function automatic logic [255:0] put_beat(
		input logic [255:0] old_line,
		input logic   [1:0] beat,
		input logic  [63:0] beat_data
	);
		begin
			put_beat = old_line;
			case (beat)
				2'd0: put_beat[63:0]    = beat_data;
				2'd1: put_beat[127:64]  = beat_data;
				2'd2: put_beat[191:128] = beat_data;
				default: put_beat[255:192] = beat_data;
			endcase
		end
	endfunction

	function automatic logic [31:0] line_word(
		input logic [255:0] line,
		input logic   [2:0] word_index
	);
		begin
			case (word_index)
				3'd0: line_word = line[31:0];
				3'd1: line_word = line[63:32];
				3'd2: line_word = line[95:64];
				3'd3: line_word = line[127:96];
				3'd4: line_word = line[159:128];
				3'd5: line_word = line[191:160];
				3'd6: line_word = line[223:192];
				default: line_word = line[255:224];
			endcase
		end
	endfunction

	function automatic logic [31:0] apply_strobes(
		input logic [31:0] old_value,
		input logic [31:0] new_value,
		input logic  [3:0] strobes
	);
		begin
			apply_strobes = old_value;
			if (strobes[0]) apply_strobes[7:0] = new_value[7:0];
			if (strobes[1]) apply_strobes[15:8] = new_value[15:8];
			if (strobes[2]) apply_strobes[23:16] = new_value[23:16];
			if (strobes[3]) apply_strobes[31:24] = new_value[31:24];
		end
	endfunction

	// One held-payload mailbox carries packed words into clk_arm.
	logic [63:0] pack_data;
	logic  [7:0] pack_be;
	logic [21:0] pack_word_addr;
	logic [63:0] word_payload_data;
	logic  [7:0] word_payload_be;
	logic [21:0] word_payload_addr;
	logic        word_toggle;
	logic        word_pending;
	logic        word_ack_arm;
	logic        word_ack_sync1;
	logic        word_ack_sync2;
	logic        epoch_toggle;
	logic        end_toggle;
	logic        end_pending;
	logic [31:0] end_size_payload;

	// Held-payload mapper DMA command. The payload remains stable until the
	// clk_arm side completes the command and returns the same toggle token.
	logic        dma_toggle;
	logic        dma_complete_toggle;
	logic        dma_complete_sync1;
	logic        dma_complete_sync2;
	logic        shadow_ready_sync1;
	logic        shadow_ready_sync2;
	logic        dma_fill_payload;
	logic [24:0] dma_source_payload;
	logic [16:0] dma_dest_payload;
	logic [17:0] dma_count_payload;
	logic  [7:0] dma_value_payload;
	logic        sample_toggle;
	logic [24:0] sample_addr_payload;
	logic        sample_complete_toggle;
	logic        sample_complete_sync1;
	logic        sample_complete_sync2;
	logic  [7:0] sample_result;
	logic  [7:0] sample_result_sync1;
	logic  [7:0] sample_result_sync2;

	assign load_wait = word_pending || end_pending;
	assign dma_ready = shadow_ready_sync2 && !dma_busy;
	assign sample_ready = shadow_ready_sync2 && !sample_busy;

	always @(posedge clk_sys) begin : pack_download
		logic [63:0] next_data;
		logic  [7:0] next_be;

		if (reset_sys) begin
			pack_data <= '0;
			pack_be <= '0;
			pack_word_addr <= '0;
			word_payload_data <= '0;
			word_payload_be <= '0;
			word_payload_addr <= '0;
			word_toggle <= 1'b0;
			word_pending <= 1'b0;
			word_ack_sync1 <= 1'b0;
			word_ack_sync2 <= 1'b0;
			epoch_toggle <= 1'b0;
			end_toggle <= 1'b0;
			end_pending <= 1'b0;
			end_size_payload <= '0;
			dma_toggle <= 1'b0;
			dma_complete_sync1 <= 1'b0;
			dma_complete_sync2 <= 1'b0;
			shadow_ready_sync1 <= 1'b0;
			shadow_ready_sync2 <= 1'b0;
			dma_fill_payload <= 1'b0;
			dma_source_payload <= '0;
			dma_dest_payload <= '0;
			dma_count_payload <= '0;
			dma_value_payload <= '0;
			sample_toggle <= 1'b0;
			sample_addr_payload <= '0;
			sample_complete_sync1 <= 1'b0;
			sample_complete_sync2 <= 1'b0;
			sample_result_sync1 <= 8'b0;
			sample_result_sync2 <= 8'b0;
			sample_busy <= 1'b0;
			sample_done <= 1'b0;
			sample_data <= 8'b0;
			dma_busy <= 1'b0;
			dma_done <= 1'b0;
		end else begin
			word_ack_sync1 <= word_ack_arm;
			word_ack_sync2 <= word_ack_sync1;
			dma_complete_sync1 <= dma_complete_toggle;
			dma_complete_sync2 <= dma_complete_sync1;
			sample_complete_sync1 <= sample_complete_toggle;
			sample_complete_sync2 <= sample_complete_sync1;
			sample_result_sync1 <= sample_result;
			sample_result_sync2 <= sample_result_sync1;
			shadow_ready_sync1 <= shadow_ready;
			shadow_ready_sync2 <= shadow_ready_sync1;
			dma_done <= 1'b0;
			sample_done <= 1'b0;

			if (word_pending && word_ack_sync2 == word_toggle)
				word_pending <= 1'b0;

			if (load_start) begin
				pack_data <= '0;
				pack_be <= '0;
				pack_word_addr <= load_addr[24:3];
				end_pending <= 1'b0;
				epoch_toggle <= ~epoch_toggle;
			end

			if (load_valid && !word_pending && !end_pending) begin
				next_data = put_byte(load_start ? 64'b0 : pack_data,
					load_addr[2:0], load_data);
				next_be = put_enable(load_start ? 8'b0 : pack_be,
					load_addr[2:0]);
				pack_data <= next_data;
				pack_be <= next_be;
				if (load_addr[2:0] == 3'd0)
					pack_word_addr <= load_addr[24:3];
				if (load_addr[2:0] == 3'd7) begin
					word_payload_data <= next_data;
					word_payload_be <= next_be;
					word_payload_addr <= load_addr[24:3];
					word_toggle <= ~word_toggle;
					word_pending <= 1'b1;
					pack_data <= '0;
					pack_be <= '0;
				end
			end

			if (load_end)
				end_pending <= 1'b1;

			if (end_pending && !word_pending) begin
				if (pack_be != 8'b0) begin
					word_payload_data <= pack_data;
					word_payload_be <= pack_be;
					word_payload_addr <= pack_word_addr;
					word_toggle <= ~word_toggle;
					word_pending <= 1'b1;
					pack_data <= '0;
					pack_be <= '0;
				end else begin
					end_size_payload <= load_size;
					end_toggle <= ~end_toggle;
					end_pending <= 1'b0;
				end
			end

			if (dma_request && dma_ready) begin
				dma_fill_payload <= dma_fill;
				dma_source_payload <= dma_source;
				dma_dest_payload <= dma_dest;
				dma_count_payload <= dma_count;
				dma_value_payload <= dma_value;
				dma_toggle <= ~dma_toggle;
				dma_busy <= 1'b1;
			end

			if (dma_busy && dma_complete_sync2 == dma_toggle) begin
				dma_busy <= 1'b0;
				dma_done <= 1'b1;
			end

			if (sample_request && sample_ready) begin
				sample_addr_payload <= sample_addr;
				sample_toggle <= ~sample_toggle;
				sample_busy <= 1'b1;
			end

			if (sample_busy && sample_complete_sync2 == sample_toggle) begin
				sample_data <= sample_result_sync2;
				sample_busy <= 1'b0;
				sample_done <= 1'b1;
			end
		end
	end

	logic word_sync1;
	logic word_sync2;
	logic epoch_sync1;
	logic epoch_sync2;
	logic epoch_seen;
	logic end_sync1;
	logic end_sync2;
	logic end_seen;
	logic [31:0] rom_size;

	typedef enum logic [2:0] {
		DDR_IDLE,
		DDR_LOAD_WRITE,
		DDR_ROM_COMMAND,
		DDR_ROM_FILL,
		DDR_DMA_COMMAND,
		DDR_DMA_READ,
		DDR_SAMPLE_COMMAND,
		DDR_SAMPLE_READ
	} ddr_state_t;
	ddr_state_t ddr_state;
	logic [28:0] load_ddr_addr;
	logic [63:0] load_ddr_data;
	logic  [7:0] load_ddr_be;
	logic [28:0] fill_ddr_addr;
	logic  [1:0] fill_beat;
	logic [255:0] fill_line;
	logic [255:0] fill_done_data;
	logic fill_done;
	logic [28:0] dma_ddr_addr;
	logic [63:0] dma_read_data;
	logic [28:0] sample_ddr_addr;

	assign ddr_req = ddr_state == DDR_LOAD_WRITE ||
		ddr_state == DDR_ROM_COMMAND || ddr_state == DDR_DMA_COMMAND ||
		ddr_state == DDR_SAMPLE_COMMAND;
	assign ddr_rnw = ddr_state != DDR_LOAD_WRITE;
	assign ddr_addr = ddr_state == DDR_LOAD_WRITE ? load_ddr_addr :
		(ddr_state == DDR_DMA_COMMAND ? dma_ddr_addr :
		(ddr_state == DDR_SAMPLE_COMMAND ? sample_ddr_addr : fill_ddr_addr));
	// The ROM line is 256 bits: one burst of four, not four transactions.
	assign ddr_len = ddr_state == DDR_ROM_COMMAND ? 8'd4 : 8'd1;
	assign ddr_din = load_ddr_data;
	assign ddr_be = load_ddr_be;

	typedef enum logic [2:0] {
		BUS_IDLE,
		BUS_CACHE_CHECK,
		BUS_CACHE_WAIT,
		BUS_RAM_ISSUE,
		BUS_RAM_WAIT
	} bus_state_t;
	bus_state_t bus_state;
	logic [31:0] req_addr;
	logic        req_write;
	logic [31:0] req_wdata;
	logic  [1:0] req_size;
	logic  [3:0] req_wstrb;
	logic        req_fetch;

	logic [20:0] ic_tag [0:63];
	logic [20:0] dc_tag [0:63];
	logic [63:0] ic_valid;
	logic [63:0] dc_valid;
	logic [5:0] fill_index;
	logic [20:0] fill_tag;
	logic fill_fetch;
	logic [2:0] fill_word_index;
	wire [5:0] cache_addr = fill_done ? fill_index :
		(bus_state == BUS_IDLE ? mem_addr[10:5] : req_addr[10:5]);
	wire [255:0] ic_data;
	wire [255:0] dc_data;
	wire [255:0] selected_cache_data = req_fetch ? ic_data : dc_data;

	cache_ram #(.ADDR_WIDTH(6), .DATA_WIDTH(256)) instruction_cache (
		.clk_i   (clk_arm),
		.addr_i  (cache_addr),
		.wren_i  (fill_done && fill_fetch),
		.wdata_i (fill_done_data),
		.q_o     (ic_data)
	);

	cache_ram #(.ADDR_WIDTH(6), .DATA_WIDTH(256)) data_cache (
		.clk_i   (clk_arm),
		.addr_i  (cache_addr),
		.wren_i  (fill_done && !fill_fetch),
		.wdata_i (fill_done_data),
		.q_o     (dc_data)
	);

	logic [31:0] mamcr;
	logic [31:0] timer_control;
	logic [31:0] timer_count;
	// The Harmony/Melody's LPC2103 counts its timer at 70 MHz; clk_arm is
	// 71.590909 MHz (5 x clk_sys), so 44 of every 45 clk_arm ticks is 70 MHz
	// exactly. Titles that budget a frame against this timer - Draconian reads
	// T1TC and compares it with 1,171,987 - see a 2.3% overrun without it.
	localparam int TIMER_PERIOD = 45;
	localparam int TIMER_SKIP   = 45 - 44;
	logic  [5:0] timer_phase;
	wire         timer_tick = timer_phase >= TIMER_SKIP[5:0];

	typedef enum logic [1:0] {
		DMA_IDLE,
		DMA_COPY_COMMAND,
		DMA_COPY_WAIT,
		DMA_RAM_WRITE
	} dma_state_t;
	dma_state_t dma_state;
	logic dma_sync1;
	logic dma_sync2;
	logic dma_seen;
	logic dma_active_token;
	logic dma_active_fill;
	logic [24:0] dma_active_source;
	logic [16:0] dma_active_dest;
	logic [17:0] dma_remaining;
	logic  [7:0] dma_active_value;
	wire dma_ram_en = dma_state == DMA_RAM_WRITE;
	wire [7:0] dma_write_byte = dma_active_fill ? dma_active_value :
		get_byte(dma_read_data, dma_active_source[2:0]);

	typedef enum logic [2:0] {
		SAMPLE_IDLE,
		SAMPLE_CHECK,
		SAMPLE_COMMAND,
		SAMPLE_WAIT
	} sample_state_t;
	sample_state_t sample_state;
	logic sample_sync1;
	logic sample_sync2;
	logic sample_seen;
	logic sample_active_token;
	logic [24:0] sample_active_addr;
	logic sample_cache_valid;
	logic [21:0] sample_cache_tag;
	logic [63:0] sample_cache_data;

	// A completed fill is one cycle wide, so hold the answer for as long as
	// the requester takes it - mem_ce can stretch that.
	logic        fill_answer_held;

	// The real part overlaps its address and data phases, so a source that
	// already holds the answer returns it with no wait state. Nothing here is
	// registered on the way out: the address phase below decodes the request in
	// the cycle it appears, and each state that can answer drives mem_ready
	// directly. Registering the answer would add a response cycle to every
	// access - three clk_arm even for a cache hit, four for cartridge RAM. The
	// state machine is left only to sequence the sources that
	// genuinely take more than a cycle - a cache tag lookup, a DDR fill, and a
	// RAM read.
	//
	//   held-line fetch, RAM write, MMIO, abort   1 clk_arm
	//   cache hit across a line, RAM read         2 clk_arm
	//   cache miss                                DDR fill
	//
	// Sixteen sequential Thumb instructions live in one 256-bit line, but the
	// core asks for each halfword separately, so without the held line every
	// instruction paid a round trip for a halfword the previous fetch had
	// already read. Holding it is what the LPC2103's MAM does for the real
	// cartridge. Only instruction fetches take that path: ROM is immutable to
	// the ARM (a write below rom_size aborts), so the held line cannot go stale
	// while it is valid.
	logic [255:0] fetch_line_data;
	logic  [26:0] fetch_line_tag;
	logic         fetch_line_valid;

	// Address phase. Valid only while BUS_IDLE is offering to take a request.
	wire req_phase = mem_req && bus_state == BUS_IDLE && !mapper_reset;
	wire dec_sentinel = mem_fetch && mem_addr == 32'hF0000000;
	wire dec_bad = !shadow_ready || mem_size == 2'b11;
	wire dec_rom = mem_addr < rom_size;
	wire dec_ram = mem_addr >= 32'h40000000 &&
		mem_addr < 32'h40000000 + {16'b0, mapper_ram_size};
	// The whole APB peripheral window. Beside the two timer registers modelled
	// here the drivers also program the PLL, MEMMAP, MAM timing, both PINSELs
	// and TIMER0 - Draconian's PLL block at ROM $40, every CDFJ+ driver's
	// MAMTIM write at $94. None of those fault on the real part, so none may
	// fault here; the ones with no model read as zero and drop their writes.
	wire dec_mmio = mem_addr[31:21] == 11'h700;
	wire dec_ok = req_phase && !dec_sentinel && !dec_bad;

	wire hit_sentinel = req_phase && dec_sentinel;
	wire hit_bad      = req_phase && !dec_sentinel && dec_bad;
	wire hit_rom_wr   = dec_ok && dec_rom && mem_write;
	wire hit_line     = dec_ok && dec_rom && !mem_write && mem_fetch &&
		fetch_line_valid && fetch_line_tag == mem_addr[31:5];
	wire rom_lookup   = dec_ok && dec_rom && !mem_write && !hit_line;
	// The RAM port sits one ARM edge in five out to stay clear of the mapper
	// port, so a request landing on that edge is simply retried by the state
	// machine rather than answered here.
	wire ram_target   = dec_ok && !dec_rom && dec_ram;
	wire ram_phase    = ram_target && !dma_ram_en;
	wire hit_ram_wr   = ram_phase && mem_write && ram_accepted;
	wire hit_mmio     = dec_ok && !dec_rom && !dec_ram && dec_mmio;
	wire hit_none     = dec_ok && !dec_rom && !dec_ram && !dec_mmio;

	wire [31:0] mmio_rdata =
		mem_addr == 32'hE01FC000 ? mamcr :
		mem_addr == 32'hE0008004 ? timer_control :
		mem_addr == 32'hE0008008 ? timer_count : 32'b0;

	// Answers driven straight out of a state that already holds the data.
	wire cache_hit_now = bus_state == BUS_CACHE_CHECK &&
		((req_fetch && ic_valid[req_addr[10:5]] &&
			ic_tag[req_addr[10:5]] == req_addr[31:11]) ||
		(!req_fetch && dc_valid[req_addr[10:5]] &&
			dc_tag[req_addr[10:5]] == req_addr[31:11]));
	wire fill_hit_now = bus_state == BUS_CACHE_WAIT &&
		(fill_done || fill_answer_held);
	wire ram_wr_now   = bus_state == BUS_RAM_ISSUE && req_write &&
		ram_accepted && !dma_ram_en;
	wire ram_rd_now   = bus_state == BUS_RAM_WAIT;

	assign mem_ready = hit_sentinel || hit_bad || hit_rom_wr || hit_line ||
		hit_ram_wr || hit_mmio || hit_none ||
		cache_hit_now || fill_hit_now || ram_wr_now || ram_rd_now;
	assign mem_abort = hit_bad || hit_rom_wr || hit_none;
	assign mem_rdata =
		hit_line     ? line_word(fetch_line_data, mem_addr[4:2]) :
		hit_mmio     ? mmio_rdata :
		hit_sentinel ? 32'b0 :
		cache_hit_now ? line_word(selected_cache_data, req_addr[4:2]) :
		fill_hit_now ? line_word(fill_done_data, fill_word_index) :
		ram_rd_now   ? ram_rdata : 32'b0;
	assign return_fetch = hit_sentinel;

	// A RAM access drives the port in the cycle it is decoded; the state
	// machine only takes over when that first attempt was not accepted.
	assign ram_en = dma_ram_en || ram_phase || bus_state == BUS_RAM_ISSUE;
	assign ram_write = dma_ram_en || (ram_phase ? mem_write : req_write);
	assign ram_addr = dma_ram_en ? dma_active_dest[16:2] :
		(ram_phase ? mem_addr[16:2] : req_addr[16:2]);
	assign ram_wdata = dma_ram_en ? {4{dma_write_byte}} :
		(ram_phase ? mem_wdata : req_wdata);
	assign ram_wstrb = dma_ram_en ? (4'b0001 << dma_active_dest[1:0]) :
		(ram_phase ? mem_wstrb : req_wstrb);

	always @(posedge clk_arm) begin : arm_memory
		logic [255:0] next_fill_line;

		if (reset_arm) begin
			word_sync1 <= 1'b0;
			word_sync2 <= 1'b0;
			word_ack_arm <= 1'b0;
			epoch_sync1 <= 1'b0;
			epoch_sync2 <= 1'b0;
			epoch_seen <= 1'b0;
			end_sync1 <= 1'b0;
			end_sync2 <= 1'b0;
			end_seen <= 1'b0;
			shadow_ready <= 1'b0;
			rom_size <= '0;
			ddr_state <= DDR_IDLE;
			load_ddr_addr <= '0;
			load_ddr_data <= '0;
			load_ddr_be <= '0;
			fill_ddr_addr <= '0;
			fill_beat <= '0;
			fill_line <= '0;
			fill_done_data <= '0;
			fill_done <= 1'b0;
			dma_ddr_addr <= '0;
			dma_read_data <= '0;
			dma_sync1 <= 1'b0;
			dma_sync2 <= 1'b0;
			dma_seen <= 1'b0;
			dma_active_token <= 1'b0;
			dma_complete_toggle <= 1'b0;
			dma_active_fill <= 1'b0;
			dma_active_source <= '0;
			dma_active_dest <= '0;
			dma_remaining <= '0;
			dma_active_value <= '0;
			dma_state <= DMA_IDLE;
			sample_sync1 <= 1'b0;
			sample_sync2 <= 1'b0;
			sample_seen <= 1'b0;
			sample_active_token <= 1'b0;
			sample_active_addr <= '0;
			sample_complete_toggle <= 1'b0;
			sample_result <= 8'b0;
			sample_state <= SAMPLE_IDLE;
			sample_cache_valid <= 1'b0;
			sample_cache_tag <= '0;
			sample_cache_data <= '0;
			sample_ddr_addr <= '0;
			bus_state <= BUS_IDLE;
			req_addr <= '0;
			req_write <= 1'b0;
			req_wdata <= '0;
			req_size <= '0;
			req_wstrb <= '0;
			req_fetch <= 1'b0;
			fetch_line_data <= '0;
			fetch_line_tag <= '0;
			fetch_line_valid <= 1'b0;
			fill_answer_held <= 1'b0;
			ic_valid <= '0;
			dc_valid <= '0;
			fill_index <= '0;
			fill_tag <= '0;
			fill_fetch <= 1'b0;
			fill_word_index <= '0;
			mamcr <= '0;
			timer_control <= '0;
			timer_count <= '0;
			timer_phase <= '0;
		end else begin
			word_sync1 <= word_toggle;
			word_sync2 <= word_sync1;
			epoch_sync1 <= epoch_toggle;
			epoch_sync2 <= epoch_sync1;
			end_sync1 <= end_toggle;
			end_sync2 <= end_sync1;
			dma_sync1 <= dma_toggle;
			dma_sync2 <= dma_sync1;
			sample_sync1 <= sample_toggle;
			sample_sync2 <= sample_sync1;
			fill_done <= 1'b0;

			timer_phase <= (timer_phase == TIMER_PERIOD[5:0] - 6'd1) ?
				6'd0 : timer_phase + 6'd1;
			if (timer_control[0] && timer_tick)
				timer_count <= timer_count + 32'd1;

			if (epoch_sync2 != epoch_seen) begin
				epoch_seen <= epoch_sync2;
				shadow_ready <= 1'b0;
				ic_valid <= '0;
				fetch_line_valid <= 1'b0;
				dc_valid <= '0;
				sample_cache_valid <= 1'b0;
				mamcr <= '0;
				timer_control <= '0;
				timer_count <= '0;
			end

			if (mapper_reset) begin
				bus_state <= BUS_IDLE;
				fill_answer_held <= 1'b0;
				ic_valid <= '0;
				fetch_line_valid <= 1'b0;
				dc_valid <= '0;
				mamcr <= '0;
				timer_control <= '0;
				timer_count <= '0;
			end

			case (ddr_state)
				DDR_IDLE: begin
					if (word_sync2 != word_ack_arm) begin
						load_ddr_addr <= SHADOW_BASE_WORD +
							{7'b0, word_payload_addr};
						load_ddr_data <= word_payload_data;
						load_ddr_be <= word_payload_be;
						ddr_state <= DDR_LOAD_WRITE;
					end else if (end_sync2 != end_seen) begin
					end_seen <= end_sync2;
					rom_size <= end_size_payload;
					shadow_ready <= 1'b1;
					end else if (dma_state == DMA_COPY_COMMAND) begin
						dma_ddr_addr <= SHADOW_BASE_WORD +
							{7'b0, dma_active_source[24:3]};
						ddr_state <= DDR_DMA_COMMAND;
						dma_state <= DMA_COPY_WAIT;
					end else if (sample_state == SAMPLE_COMMAND) begin
						sample_ddr_addr <= SHADOW_BASE_WORD +
							{7'b0, sample_active_addr[24:3]};
						ddr_state <= DDR_SAMPLE_COMMAND;
						sample_state <= SAMPLE_WAIT;
					end
				end

				DDR_LOAD_WRITE: begin
					if (ddr_ack) begin
						word_ack_arm <= word_sync2;
						ddr_state <= DDR_IDLE;
					end
				end

				DDR_ROM_COMMAND: begin
					if (ddr_ack) begin
						fill_beat <= 2'd0;
						fill_line <= '0;
						ddr_state <= DDR_ROM_FILL;
					end
				end

				DDR_ROM_FILL: begin
					if (ddr_rvalid) begin
						next_fill_line = put_beat(fill_line, fill_beat, ddr_dout);
						fill_line <= next_fill_line;
						if (fill_beat == 2'd3) begin
							fill_done_data <= next_fill_line;
							fill_done <= 1'b1;
							ddr_state <= DDR_IDLE;
						end else begin
							fill_beat <= fill_beat + 2'd1;
						end
					end
				end

				DDR_DMA_COMMAND: begin
					if (ddr_ack)
						ddr_state <= DDR_DMA_READ;
				end

				DDR_DMA_READ: begin
					if (ddr_rvalid) begin
						dma_read_data <= ddr_dout;
						ddr_state <= DDR_IDLE;
						dma_state <= DMA_RAM_WRITE;
					end
				end

				DDR_SAMPLE_COMMAND: begin
					if (ddr_ack)
						ddr_state <= DDR_SAMPLE_READ;
				end

				default: begin // DDR_SAMPLE_READ
					if (ddr_rvalid) begin
						sample_cache_data <= ddr_dout;
						sample_cache_tag <= sample_active_addr[24:3];
						sample_cache_valid <= 1'b1;
						sample_result <= get_byte(ddr_dout,
							sample_active_addr[2:0]);
						sample_complete_toggle <= sample_active_token;
						sample_state <= SAMPLE_IDLE;
						ddr_state <= DDR_IDLE;
					end
				end
			endcase

			case (dma_state)
				DMA_IDLE: begin
					if (dma_sync2 != dma_seen) begin
						dma_seen <= dma_sync2;
						dma_active_token <= dma_sync2;
						dma_active_fill <= dma_fill_payload;
						dma_active_source <= dma_source_payload;
						dma_active_dest <= dma_dest_payload;
						dma_remaining <= dma_count_payload;
						dma_active_value <= dma_value_payload;
						if (dma_count_payload == 18'b0) begin
							dma_complete_toggle <= dma_sync2;
						end else if (dma_fill_payload) begin
							dma_state <= DMA_RAM_WRITE;
						end else begin
							dma_state <= DMA_COPY_COMMAND;
						end
					end
				end

				DMA_RAM_WRITE: begin
					if (ram_accepted) begin
						if (dma_remaining == 18'd1) begin
							dma_complete_toggle <= dma_active_token;
							dma_state <= DMA_IDLE;
						end else begin
							dma_active_dest <= dma_active_dest + 17'd1;
							dma_remaining <= dma_remaining - 18'd1;
							if (!dma_active_fill) begin
								dma_active_source <= dma_active_source + 25'd1;
								if (dma_active_source[2:0] == 3'd7)
									dma_state <= DMA_COPY_COMMAND;
							end
						end
					end
				end

				default: ;
			endcase

			case (sample_state)
				SAMPLE_IDLE: begin
					if (sample_sync2 != sample_seen) begin
						sample_seen <= sample_sync2;
						sample_active_token <= sample_sync2;
						sample_active_addr <= sample_addr_payload;
						sample_state <= SAMPLE_CHECK;
					end
				end

				SAMPLE_CHECK: begin
					if ({7'b0, sample_active_addr} >= rom_size) begin
						sample_result <= 8'b0;
						sample_complete_toggle <= sample_active_token;
						sample_state <= SAMPLE_IDLE;
					end else if (sample_cache_valid &&
						sample_cache_tag == sample_active_addr[24:3]) begin
						sample_result <= get_byte(sample_cache_data,
							sample_active_addr[2:0]);
						sample_complete_toggle <= sample_active_token;
						sample_state <= SAMPLE_IDLE;
					end else begin
						sample_state <= SAMPLE_COMMAND;
					end
				end

				default: ;
			endcase

			if (fill_done) begin
				if (fill_fetch) begin
					ic_tag[fill_index] <= fill_tag;
					ic_valid[fill_index] <= 1'b1;
				end else begin
					dc_tag[fill_index] <= fill_tag;
					dc_valid[fill_index] <= 1'b1;
				end
			end

			if (!mapper_reset) begin
				case (bus_state)
					BUS_IDLE: begin
						// The address phase above has already answered anything
						// that could be answered this cycle. What reaches the
						// state machine is only what takes longer: a cache tag
						// lookup, or a RAM access the port did not take.
						if (req_phase) begin
							req_addr <= mem_addr;
							req_write <= mem_write;
							req_wdata <= mem_wdata;
							req_size <= mem_size;
							req_wstrb <= mem_wstrb;
							req_fetch <= mem_fetch;

							if (hit_mmio && mem_write) begin
								case (mem_addr)
									32'hE01FC000:
										mamcr <= apply_strobes(mamcr, mem_wdata, mem_wstrb);
									32'hE0008004:
										timer_control <= apply_strobes(timer_control, mem_wdata, mem_wstrb);
									32'hE0008008:
										timer_count <= apply_strobes(timer_count, mem_wdata, mem_wstrb);
									default: ;
								endcase
							end

							if (rom_lookup)
								bus_state <= BUS_CACHE_CHECK;
							else if (ram_target && !hit_ram_wr)
								bus_state <= BUS_RAM_ISSUE;
						end
					end

					BUS_CACHE_CHECK: begin
						if (cache_hit_now) begin
							if (req_fetch) begin
								fetch_line_data <= selected_cache_data;
								fetch_line_tag <= req_addr[31:5];
								fetch_line_valid <= 1'b1;
							end
							if (mem_ce)
								bus_state <= BUS_IDLE;
						end else if (ddr_state == DDR_IDLE &&
							dma_state != DMA_COPY_COMMAND &&
							sample_state != SAMPLE_COMMAND) begin
							fill_index <= req_addr[10:5];
							fill_tag <= req_addr[31:11];
							fill_fetch <= req_fetch;
							fill_word_index <= req_addr[4:2];
							fill_ddr_addr <= SHADOW_BASE_WORD + {7'b0, req_addr[24:5], 2'b00};
							ddr_state <= DDR_ROM_COMMAND;
							bus_state <= BUS_CACHE_WAIT;
						end
					end

					BUS_CACHE_WAIT: begin
						if (fill_done) begin
							fill_answer_held <= 1'b1;
							if (req_fetch) begin
								fetch_line_data <= fill_done_data;
								fetch_line_tag <= req_addr[31:5];
								fetch_line_valid <= 1'b1;
							end
						end
						if ((fill_done || fill_answer_held) && mem_ce) begin
							fill_answer_held <= 1'b0;
							bus_state <= BUS_IDLE;
						end
					end

					BUS_RAM_ISSUE: begin
						if (ram_accepted && !dma_ram_en) begin
							if (req_write) begin
								// ram_wr_now answered it as the port took it.
								if (mem_ce)
									bus_state <= BUS_IDLE;
							end else begin
								bus_state <= BUS_RAM_WAIT;
							end
						end
					end

					default: begin // BUS_RAM_WAIT
						if (mem_ce)
							bus_state <= BUS_IDLE;
					end
				endcase
			end
		end
	end
endmodule
