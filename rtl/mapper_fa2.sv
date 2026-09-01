// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

module mapper_fa2
#(
	parameter logic [18:0] READ_TICKS  = 19'd1790,
	parameter logic [18:0] WRITE_TICKS = 19'd361535
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce,
	input  logic        phi1,
	input  logic        a_change,
	input  logic [12:0] a_in,
	input  logic  [7:0] d_in,
	input  logic  [7:0] rom_data,
	input  logic [31:0] rom_size,
	output logic  [7:0] d_out,
	output logic [15:0] flags_out,
	output logic  [7:0] oe,
	output logic [18:0] rom_a,

	output logic        nvram_request,
	output logic        nvram_write,
	output logic  [7:0] nvram_addr,
	output logic  [7:0] nvram_wdata,
	input  logic  [7:0] nvram_rdata,
	input  logic        nvram_ready,
	output logic        nvram_dirty
);
	localparam logic [2:0] COPY_NONE      = 3'd0;
	localparam logic [2:0] COPY_LOAD_REQ  = 3'd1;
	localparam logic [2:0] COPY_LOAD_NEXT = 3'd2;
	localparam logic [2:0] COPY_SAVE_READ = 3'd3;
	localparam logic [2:0] COPY_SAVE_REQ  = 3'd4;

	logic [2:0] bank;
	logic [2:0] copy_state;
	logic [7:0] copy_addr;
	logic [18:0] timer;
	logic operation_active;
	logic command_pending;
	logic ready_response;
	logic [7:0] ram_addr;
	logic [7:0] ram_wdata;
	logic [7:0] ram_q;
	logic ram_wren;

	wire ram_write_port = a_in[12:8] == 5'h10;
	wire ram_read_port = a_in[12:8] == 5'h11;
	wire harmony_image = rom_size >= 32'd29696;
	wire padded_bank = rom_size == 32'd24576 && bank == 3'd6;
	wire flash_hotspot = a_in == 13'h1FF4;
	wire operation_done = operation_active && timer == 19'd0 &&
		copy_state == COPY_NONE;
	wire success_poll = operation_done && a_change && flash_hotspot;
	wire cpu_ram_write = copy_state == COPY_NONE && ram_write_port &&
		!phi1 && !a_change;

	always_comb begin
		ram_addr = a_in[7:0];
		ram_wdata = d_in;
		ram_wren = cpu_ram_write;

		if (copy_state != COPY_NONE)
			ram_addr = copy_addr;
		else if (command_pending || flash_hotspot)
			ram_addr = 8'hFF;

		if (copy_state == COPY_LOAD_REQ && nvram_ready) begin
			ram_wdata = nvram_rdata;
			ram_wren = 1'b1;
		end else if (success_poll) begin
			ram_addr = 8'hFF;
			ram_wdata = 8'h00;
			ram_wren = 1'b1;
		end
	end

	spram #(
		.addr_width (8),
		.data_width (8),
		.mem_name   ("FA2_WORK_RAM")
	) working_ram (
		.clock   (clk),
		.address (ram_addr),
		.data    (ram_wdata),
		.wren    (ram_wren),
		.q       (ram_q),
		.cs      (1'b1)
	);

	always_comb begin
		nvram_request = copy_state == COPY_LOAD_REQ ||
			copy_state == COPY_SAVE_REQ;
		nvram_write = copy_state == COPY_SAVE_REQ;
		nvram_addr = copy_addr;
		nvram_wdata = ram_q;
	end

	always_comb begin
		flags_out = 16'd0;
		d_out = 8'd0;
		oe = a_in[12] && !ram_write_port ? 8'hFF : 8'h00;

		if (ram_read_port) begin
			flags_out[0] = 1'b1;
			d_out = ram_q;
		end else if (flash_hotspot) begin
			flags_out[0] = 1'b1;
			if (ready_response || operation_done)
				d_out = rom_data & 8'hBF;
			else
				d_out = rom_data | 8'h40;
		end else if (padded_bank) begin
			flags_out[0] = 1'b1;
			d_out = 8'h00;
		end
	end

	always_comb begin
		rom_a = {4'd0, bank, a_in[11:0]};
		if (harmony_image)
			rom_a = rom_a + 19'd1024;
	end

	always_ff @(posedge clk) begin
		nvram_dirty <= 1'b0;

		if (timer != 19'd0 && ce)
			timer <= timer - 19'd1;

		if (ready_response && a_change && !flash_hotspot)
			ready_response <= 1'b0;

		if (!operation_active && !ready_response && a_change && flash_hotspot)
			command_pending <= 1'b1;

		if (command_pending) begin
			command_pending <= 1'b0;
			operation_active <= 1'b1;
			copy_addr <= 8'd0;
			case (ram_q)
				8'd1: begin
					timer <= READ_TICKS;
					copy_state <= COPY_LOAD_REQ;
				end
				8'd2: begin
					timer <= WRITE_TICKS;
					copy_state <= COPY_SAVE_READ;
				end
				default: begin
					timer <= 19'd0;
					copy_state <= COPY_NONE;
				end
			endcase
		end

		case (copy_state)
			COPY_LOAD_REQ: begin
				if (nvram_ready) begin
					if (copy_addr == 8'hFF)
						copy_state <= COPY_NONE;
					else begin
						copy_addr <= copy_addr + 8'd1;
						copy_state <= COPY_LOAD_NEXT;
					end
				end
			end

			COPY_LOAD_NEXT: copy_state <= COPY_LOAD_REQ;

			COPY_SAVE_READ: copy_state <= COPY_SAVE_REQ;

			COPY_SAVE_REQ: begin
				if (nvram_ready) begin
					if (copy_addr == 8'hFF) begin
						copy_state <= COPY_NONE;
						nvram_dirty <= 1'b1;
					end else begin
						copy_addr <= copy_addr + 8'd1;
						copy_state <= COPY_SAVE_READ;
					end
				end
			end

			default: ;
		endcase

		if (success_poll) begin
			operation_active <= 1'b0;
			ready_response <= 1'b1;
		end

		if (a_change) begin
			case (a_in)
				13'h1FF5: bank <= 3'd0;
				13'h1FF6: bank <= 3'd1;
				13'h1FF7: bank <= 3'd2;
				13'h1FF8: bank <= 3'd3;
				13'h1FF9: bank <= 3'd4;
				13'h1FFA: bank <= 3'd5;
				13'h1FFB: bank <= 3'd6;
				default: ;
			endcase
		end

		if (reset) begin
			bank <= 3'd0;
			copy_state <= COPY_NONE;
			copy_addr <= 8'd0;
			timer <= 19'd0;
			operation_active <= 1'b0;
			command_pending <= 1'b0;
			ready_response <= 1'b0;
			nvram_dirty <= 1'b0;
		end
	end
endmodule
