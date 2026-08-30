// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

// CDF cartridge front end. Pointer/increment tables are a separate mapper
// cache so a 6507 access never serializes four byte-wide RAM reads.
module mapper_cdf
(
	input  logic        clk,
	input  logic        reset,
	input  logic        access,
	input  logic        rw,
	input  logic [12:0] a_in,
	input  logic  [7:0] d_in,
	input  logic  [7:0] rom_data,
	input  logic  [1:0] revision,
	input  logic        enable_ldx,
	input  logic        enable_ldy,
	input  logic        fetch_offset_enable,
	input  logic  [7:0] fetch_offset,
	input  logic        fast_jump_valid,

	output logic  [7:0] d_out,
	output logic [15:0] flags_out,
	output logic  [7:0] oe,
	output logic [18:0] rom_a,

	output logic  [5:0] table_index,
	input  logic [31:0] table_pointer,
	input  logic [15:0] table_increment,
	output logic        pointer_update,
	output logic  [5:0] pointer_update_index,
	output logic [31:0] pointer_update_value,

	output logic        ram_en,
	output logic        ram_write,
	output logic [14:0] ram_addr,
	output logic  [7:0] ram_wdata,
	input  logic  [7:0] ram_rdata,
	input  logic  [7:0] amplitude,
	output logic        digital_audio,

	output logic        call_request,
	output logic [31:0] call_entry,
	output logic [31:0] call_stack,
	output logic        call_thumb,
	input  logic        call_ready,
	input  logic [31:0] cdfj_entry,
	input  logic [31:0] cdfj_stack
);
	logic [2:0] bank;
	logic [7:0] mode;
	logic fast_pending;
	logic [12:0] fast_expected_address;
	logic [1:0] jump_remaining;
	logic [12:0] expected_address;
	logic [5:0] jump_stream;
	logic call_pending;

	logic jplus;
	logic j_revision;
	logic fast_mode;
	logic [5:0] amplitude_stream;
	logic opcode_arms_fetch;
	logic operand_in_range;
	logic [7:0] normalized_operand;
	logic [8:0] fetch_limit;
	logic [6:0] amplitude_operand_sum;
	logic [5:0] amplitude_operand;
	logic fetch_substitute;
	logic jump_substitute;
	logic amplitude_fetch;
	logic jump_operand_valid;
	logic stream_substitute;
	logic [31:0] pointer_step;
	logic [14:0] display_address;

	assign jplus = revision == 2'd3;
	assign j_revision = revision >= 2'd2;
	assign fast_mode = mode[3:0] == 4'b0;
	assign digital_audio = mode[7:4] == 4'b0;
	assign amplitude_stream = j_revision ? 6'd35 : 6'd34;
	assign opcode_arms_fetch = rom_data == 8'hA9 ||
		(jplus && enable_ldx && rom_data == 8'hA2) ||
		(jplus && enable_ldy && rom_data == 8'hA0);
	// The bounds do not move with the fetched byte, so the byte meets nothing
	// but comparators. Same for the amplitude test: (x - offset) == amp in the
	// low six bits is the same question as x == (amp + offset), which keeps the
	// offset's subtract off the path from cartridge ROM to the 6507 bus.
	assign fetch_limit = {1'b0, fetch_offset} + {3'b0, amplitude_stream};
	assign amplitude_operand_sum = {1'b0, amplitude_stream} +
		{1'b0, fetch_offset[5:0]};
	assign amplitude_operand = fetch_offset_enable ?
		amplitude_operand_sum[5:0] : amplitude_stream;
	assign operand_in_range = fetch_offset_enable ?
		(rom_data >= fetch_offset && {1'b0, rom_data} <= fetch_limit) :
		rom_data <= {2'b0, amplitude_stream};
	assign normalized_operand = fetch_offset_enable ?
		rom_data - fetch_offset : rom_data;
	assign jump_operand_valid = (jump_remaining == 2'd2 &&
		(j_revision ? rom_data[7:1] == 7'b0 : rom_data == 8'b0)) ||
		(jump_remaining == 2'd1 && rom_data == 8'b0);
	assign jump_substitute = rw && a_in[12] && jump_remaining != 0 &&
		a_in == expected_address && jump_operand_valid;
	assign fetch_substitute = rw && a_in[12] && fast_mode && fast_pending &&
		a_in == fast_expected_address && operand_in_range;
	assign stream_substitute = jump_substitute || fetch_substitute;
	// Only a fetch substitution can land on the amplitude stream, and there the
	// index is the operand itself. Testing the shared table_index instead would
	// put the jump path's adder in front of ram_en, 2.5 ns into a budget that
	// starts back at the cartridge ROM.
	assign amplitude_fetch = fetch_substitute && !jump_substitute &&
		rom_data[5:0] == amplitude_operand;

	always_comb begin
		if (jump_substitute)
			table_index = jump_stream +
				((j_revision && jump_remaining == 2) ? {5'b0, rom_data[0]} : 6'b0);
		else if (fetch_substitute)
			table_index = normalized_operand[5:0];
		else
			table_index = 6'd32;

		pointer_step = jplus ?
			(table_pointer + {8'b0, table_increment, 8'b0}) :
			(table_pointer + {4'b0, table_increment, 12'b0});
		display_address = jplus ?
			(15'd2048 + table_pointer[30:16]) :
			(15'd2048 + {3'b0, table_pointer[31:20]});

		rom_a = (jplus ? 19'd2048 : 19'd4096) +
			{4'b0, bank, 12'b0} + {7'b0, a_in[11:0]};
		oe = a_in[12] ? 8'hFF : 8'h00;
		flags_out = 16'b0;
		d_out = 8'b0;
		ram_en = 1'b0;
		ram_write = 1'b0;
		ram_addr = display_address;
		ram_wdata = d_in;

		if (stream_substitute) begin
			flags_out[0] = 1'b1;
			if (amplitude_fetch)
				d_out = amplitude;
			else begin
				ram_en = 1'b1;
				d_out = ram_rdata;
			end
		end

		if (access && !rw && a_in == 13'h1FF0) begin
			table_index = 6'd32;
			ram_addr = jplus ? (15'd2048 + table_pointer[30:16]) :
				(15'd2048 + {3'b0, table_pointer[31:20]});
			ram_en = 1'b1;
			ram_write = 1'b1;
		end
	end

	assign call_request = call_pending && call_ready;
	assign call_entry = jplus ? cdfj_entry : 32'h00000808;
	assign call_stack = jplus ? cdfj_stack : 32'h40001FFC;
	assign call_thumb = 1'b1;

	always @(posedge clk) begin
		if (reset) begin
			bank <= revision == 2'd3 ? 3'd0 : 3'd6;
			mode <= 8'hFF;
			fast_pending <= 1'b0;
			fast_expected_address <= 13'b0;
			jump_remaining <= 2'd0;
			expected_address <= 13'b0;
			jump_stream <= 6'd33;
			call_pending <= 1'b0;
			pointer_update <= 1'b0;
			pointer_update_index <= 6'b0;
			pointer_update_value <= 32'b0;
		end else begin
			pointer_update <= 1'b0;
			if (call_pending && call_ready)
				call_pending <= 1'b0;

			if (access && a_in[12]) begin
				// A substituted read returns before Stella's hotspot switch.
				if (!stream_substitute &&
					a_in[11:0] >= 12'hFF4 && a_in[11:0] <= 12'hFFB) begin
					if (jplus)
						bank <= (a_in[11:0] == 12'hFF4 ||
							a_in[11:0] == 12'hFFB) ? 3'd0 : a_in[2:0] - 3'd4;
					else
						bank <= (a_in[11:0] == 12'hFF4 ||
							a_in[11:0] == 12'hFFB) ? 3'd6 : a_in[2:0] - 3'd5;
				end

				if (rw) begin
					if (stream_substitute) begin
						fast_pending <= 1'b0;
						if (table_index != amplitude_stream || jump_substitute) begin
							pointer_update <= 1'b1;
							pointer_update_index <= table_index;
							pointer_update_value <= jump_substitute ?
								(jplus ? table_pointer + 32'h00010000 :
									table_pointer + 32'h00100000) : pointer_step;
						end
						if (jump_substitute) begin
							if (j_revision && jump_remaining == 2)
								jump_stream <= 6'd33 + {5'b0, rom_data[0]};
							jump_remaining <= jump_remaining - 2'd1;
							expected_address <= expected_address + 13'd1;
						end
					end else begin
						fast_pending <= fast_mode && opcode_arms_fetch;
						if (fast_mode && opcode_arms_fetch)
							fast_expected_address <= a_in + 13'd1;
						if (jump_remaining != 0 && a_in == expected_address) begin
							jump_remaining <= 2'd0;
						end else if (fast_mode && rom_data == 8'h4C &&
							fast_jump_valid) begin
							jump_remaining <= 2'd2;
							expected_address <= a_in + 13'd1;
							jump_stream <= 6'd33;
						end else if (jump_remaining != 0 && a_in != expected_address) begin
							jump_remaining <= 2'd0;
						end
					end
				end else begin
					case (a_in[11:0])
						12'hFF0: begin
							pointer_update <= 1'b1;
							pointer_update_index <= 6'd32;
							pointer_update_value <= jplus ?
								table_pointer + 32'h00010000 :
								table_pointer + 32'h00100000;
						end
						12'hFF1: begin
							pointer_update <= 1'b1;
							pointer_update_index <= 6'd32;
							pointer_update_value <= jplus ?
								({table_pointer[23:0], 8'b0} & 32'hFF000000) |
									{8'b0, d_in, 16'b0} :
								({table_pointer[23:0], 8'b0} & 32'hF0000000) |
									{4'b0, d_in, 20'b0};
						end
						12'hFF2: mode <= d_in;
						12'hFF3: begin
							if ((d_in == 8'hFE || d_in == 8'hFF) && !call_pending)
								call_pending <= 1'b1;
						end
						default: ;
					endcase
				end
			end
		end
	end
endmodule
