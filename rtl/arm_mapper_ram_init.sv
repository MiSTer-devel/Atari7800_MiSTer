// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

// Restore mapper working RAM from the immutable DDR3 shadow, then preload the
// registered BUS/CDF stream tables before releasing the 6507 reset.
module arm_mapper_ram_init
(
	input  logic        clk_sys,
	input  logic        mapper_reset,
	input  logic        load_start,
	input  logic        load_end,
	input  logic  [1:0] family,
	input  logic  [2:0] revision,
	input  logic [15:0] mapper_ram_size,
	output logic        busy,

	output logic        dma_request,
	output logic        dma_fill,
	output logic [24:0] dma_source,
	output logic [16:0] dma_dest,
	output logic [17:0] dma_count,
	output logic  [7:0] dma_value,
	input  logic        dma_ready,
	input  logic        dma_done,

	output logic        ram_en,
	output logic [16:0] ram_addr,
	input  logic [31:0] ram_word_rdata,

	output logic        table_pointer_write,
	output logic  [5:0] table_pointer_index,
	output logic [31:0] table_pointer_wdata,
	output logic        table_increment_write,
	output logic  [5:0] table_increment_index,
	output logic [31:0] table_increment_wdata,
	output logic        table_map_write,
	output logic  [5:0] table_map_index,
	output logic [31:0] table_map_wdata
);
	localparam logic [1:0] FAMILY_NONE = 2'd0;
	localparam logic [1:0] FAMILY_DPC  = 2'd1;
	localparam logic [1:0] FAMILY_BUS  = 2'd2;
	localparam logic [1:0] FAMILY_CDF  = 2'd3;

	typedef enum logic [3:0] {
		INIT_IDLE,
		INIT_DMA_FIRST,
		INIT_DMA_FIRST_WAIT,
		INIT_DMA_SECOND,
		INIT_DMA_SECOND_WAIT,
		INIT_POINTER_READ,
		INIT_POINTER_WRITE,
		INIT_INCREMENT_READ,
		INIT_INCREMENT_WRITE,
		INIT_MAP_READ,
		INIT_MAP_WRITE
	} init_state_t;
	init_state_t state;

	logic loading;
	logic image_loaded;
	logic old_mapper_reset;
	logic [1:0] active_family;
	logic [2:0] active_revision;
	logic [15:0] active_ram_size;
	logic [14:0] pointer_base;
	logic [14:0] increment_base;
	logic [14:0] map_base;
	logic  [5:0] stream_count;
	logic  [5:0] table_index;
	logic [14:0] table_word_addr;
	logic has_second_dma;
	logic has_tables;

	assign busy = loading || state != INIT_IDLE;

	always_comb begin
		pointer_base = 15'b0;
		increment_base = 15'b0;
		map_base = 15'b0;
		stream_count = 6'b0;

		case (active_family)
			FAMILY_BUS: begin
				case (active_revision)
					3'd0: begin
						pointer_base = 15'h2B8;
						increment_base = 15'h2C8;
						stream_count = 6'd16;
					end
					3'd3: begin
						pointer_base = 15'h1B6;
						increment_base = 15'h1C8;
						stream_count = 6'd18;
					end
					default: begin
						pointer_base = 15'h1B8;
						increment_base = 15'h1C8;
						stream_count = 6'd16;
					end
				endcase
				map_base = active_revision == 3'd0 ? 15'h2D9 : 15'h1D8;
			end

			FAMILY_CDF: begin
				case (active_revision)
					3'd0: begin
						pointer_base = 15'h1B8;
						increment_base = 15'h1DA;
						stream_count = 6'd34;
					end
					3'd1: begin
						pointer_base = 15'h028;
						increment_base = 15'h04A;
						stream_count = 6'd34;
					end
					default: begin
						pointer_base = 15'h026;
						increment_base = 15'h049;
						stream_count = 6'd35;
					end
				endcase
			end

			default: ;
		endcase
	end

	assign has_second_dma = active_family != FAMILY_NONE;
	assign has_tables = active_family == FAMILY_BUS ||
		active_family == FAMILY_CDF;

	always_comb begin
		dma_request = state == INIT_DMA_FIRST || state == INIT_DMA_SECOND;
		dma_fill = 1'b1;
		dma_source = 25'b0;
		dma_dest = 17'b0;
		dma_count = 18'b0;
		dma_value = 8'b0;

		if (state == INIT_DMA_FIRST) begin
			case (active_family)
				FAMILY_DPC: dma_count = 18'h00C00;
				FAMILY_BUS: begin
					dma_fill = 1'b0;
					dma_count = active_revision == 3'd0 ?
						18'h00C00 : 18'h00800;
				end
				FAMILY_CDF: begin
					dma_fill = 1'b0;
					dma_count = 18'h00800;
				end
				default: dma_count = 18'd131072;
			endcase
		end else if (state == INIT_DMA_SECOND) begin
			case (active_family)
				FAMILY_DPC: begin
					dma_fill = 1'b0;
					dma_source = 25'h06C00;
					dma_dest = 17'h00C00;
					dma_count = 18'h01400;
				end
				FAMILY_BUS: begin
					dma_dest = active_revision == 3'd0 ?
						17'h00C00 : 17'h00800;
					dma_count = active_revision == 3'd0 ?
						18'h01400 : 18'h01800;
				end
				FAMILY_CDF: begin
					dma_dest = 17'h00800;
					dma_count = {2'b0, active_ram_size} - 18'h00800;
				end
				default: ;
			endcase
		end
	end

	always_comb begin
		table_word_addr = 15'b0;
		case (state)
			INIT_POINTER_READ: table_word_addr = pointer_base +
				{9'b0, table_index};
			INIT_INCREMENT_READ: table_word_addr = increment_base +
				{9'b0, table_index};
			INIT_MAP_READ: table_word_addr = map_base +
				{9'b0, table_index};
			default: ;
		endcase
	end

	assign ram_en = state == INIT_POINTER_READ ||
		state == INIT_INCREMENT_READ || state == INIT_MAP_READ;
	assign ram_addr = {table_word_addr, 2'b00};
	assign table_pointer_write = state == INIT_POINTER_WRITE;
	assign table_pointer_index = table_index;
	assign table_pointer_wdata = ram_word_rdata;
	assign table_increment_write = state == INIT_INCREMENT_WRITE;
	assign table_increment_index = table_index;
	assign table_increment_wdata = ram_word_rdata;
	assign table_map_write = state == INIT_MAP_WRITE;
	assign table_map_index = table_index;
	assign table_map_wdata = ram_word_rdata;

	always @(posedge clk_sys) begin
		old_mapper_reset <= mapper_reset;

		if (load_start) begin
			loading <= 1'b1;
			image_loaded <= 1'b0;
			state <= INIT_IDLE;
			table_index <= 6'b0;
		end else if (load_end) begin
			loading <= 1'b0;
			image_loaded <= family != FAMILY_NONE;
			active_family <= family;
			active_revision <= revision;
			active_ram_size <= mapper_ram_size;
			state <= family == FAMILY_NONE ? INIT_IDLE : INIT_DMA_FIRST;
			table_index <= 6'b0;
		end else begin
			case (state)
				INIT_IDLE: begin
					if (image_loaded && mapper_reset && !old_mapper_reset) begin
						state <= INIT_DMA_FIRST;
						table_index <= 6'b0;
					end
				end

				INIT_DMA_FIRST: begin
					if (dma_ready)
						state <= INIT_DMA_FIRST_WAIT;
				end

				INIT_DMA_FIRST_WAIT: begin
					if (dma_done) begin
						if (has_second_dma)
							state <= INIT_DMA_SECOND;
						else
							state <= INIT_IDLE;
					end
				end

				INIT_DMA_SECOND: begin
					if (dma_ready)
						state <= INIT_DMA_SECOND_WAIT;
				end

				INIT_DMA_SECOND_WAIT: begin
					if (dma_done) begin
						table_index <= 6'b0;
						state <= has_tables ? INIT_POINTER_READ : INIT_IDLE;
					end
				end

				INIT_POINTER_READ: state <= INIT_POINTER_WRITE;
				INIT_POINTER_WRITE: begin
					if (table_index == stream_count - 6'd1) begin
						table_index <= 6'b0;
						state <= INIT_INCREMENT_READ;
					end else begin
						table_index <= table_index + 6'd1;
						state <= INIT_POINTER_READ;
					end
				end

				INIT_INCREMENT_READ: state <= INIT_INCREMENT_WRITE;
				INIT_INCREMENT_WRITE: begin
					if (table_index == stream_count - 6'd1) begin
						table_index <= 6'b0;
						state <= active_family == FAMILY_BUS ?
							INIT_MAP_READ : INIT_IDLE;
					end else begin
						table_index <= table_index + 6'd1;
						state <= INIT_INCREMENT_READ;
					end
				end

				INIT_MAP_READ: state <= INIT_MAP_WRITE;
				default: begin // INIT_MAP_WRITE
					if (table_index == 6'd36) begin
						state <= INIT_IDLE;
					end else begin
						table_index <= table_index + 6'd1;
						state <= INIT_MAP_READ;
					end
				end
			endcase
		end
	end

	initial begin
		loading = 1'b0;
		image_loaded = 1'b0;
		old_mapper_reset = 1'b0;
		active_family = FAMILY_NONE;
		active_revision = 3'b0;
		active_ram_size = 16'd8192;
		table_index = 6'b0;
		state = INIT_IDLE;
	end
endmodule
