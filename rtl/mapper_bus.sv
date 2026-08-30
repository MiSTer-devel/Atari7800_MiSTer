// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

// BUS1/2/3 cartridge front end. BUS0 stays disabled: its legacy increment
// register behaviour has no image to check it against.
module mapper_bus
(
	input  logic        clk,
	input  logic        reset,
	input  logic        access,
	input  logic        rw,
	input  logic [12:0] a_in,
	input  logic  [7:0] d_in,
	input  logic  [7:0] rom_data,
	input  logic  [1:0] revision,
	input  logic        fast_jump_valid,

	output logic        supported,
	output logic  [7:0] d_out,
	output logic [15:0] flags_out,
	output logic  [7:0] oe,
	output logic [18:0] rom_a,

	output logic  [5:0] pointer_lookup_index,
	output logic  [5:0] increment_lookup_index,
	input  logic [31:0] table_pointer,
	input  logic [31:0] table_increment,
	output logic        pointer_update,
	output logic  [5:0] pointer_update_index,
	output logic [31:0] pointer_update_value,
	output logic        map_update,
	output logic  [5:0] map_update_index,
	output logic [31:0] map_update_value,

	output logic        ram_en,
	output logic        ram_write,
	output logic [14:0] ram_addr,
	output logic  [7:0] ram_wdata,
	input  logic  [7:0] ram_rdata,
	input  logic  [7:0] amplitude,
	output logic        digital_audio,

	output logic        stuff_valid,
	output logic  [7:0] stuff_data,

	output logic        call_request,
	output logic [31:0] call_entry,
	output logic [31:0] call_stack,
	output logic        call_thumb,
	input  logic        call_ready
);
	logic [2:0] bank;
	logic [7:0] mode;
	logic call_pending;
	logic sty_pending;
	logic [12:0] sty_operand_address;
	logic stuff_target_valid;
	logic [7:0] stuff_target;
	logic [12:0] last_a;
	logic last_rw;
	logic [1:0] jump_remaining;
	logic [12:0] jump_operand_address;
	logic [31:0] stuff_map;
	logic [3:0] stuff_stream;

	typedef enum logic [2:0] {
		STUFF_IDLE,
		STUFF_MAP,
		STUFF_STREAM,
		STUFF_DATA,
		STUFF_READY
	} stuff_state_t;
	stuff_state_t stuff_state;

	logic bus_change;
	logic bus3;
	logic fast_mode;
	logic program_read;
	logic stream_read;
	logic amplitude_read;
	logic stream_write;
	logic pointer_write;
	logic [5:0] pointer_write_index;
	logic stuff_candidate;
	logic jump_substitute;
	logic [5:0] stream_index;
	logic [31:0] pointer_step;
	logic [14:0] display_address;

	assign supported = revision != 2'd0;
	assign bus3 = revision == 2'd3;
	assign fast_mode = mode[3:0] == 4'b0;
	assign digital_audio = mode[7:4] == 4'b0;
	assign bus_change = a_in != last_a || rw != last_rw;
	assign program_read = rw && a_in[12];
	assign stream_read = program_read &&
		((!bus3 && a_in[11:0] < 12'h010) ||
		 (bus3 && a_in[11:0] == 12'hFEF));
	assign amplitude_read = program_read &&
		((!bus3 && a_in[11:0] == 12'h018) ||
		 (bus3 && a_in[11:0] == 12'hFEE));
	assign stream_write = !rw && a_in[12] &&
		((!bus3 && a_in[11:0] >= 12'h010 && a_in[11:0] <= 12'h013) ||
		 (bus3 && a_in[11:0] == 12'hFF0));
	// DSxPTR aims one stream, so the lookup has to name that stream: the
	// stored value keeps the old pointer's top nibble.
	assign pointer_write = !rw && a_in[12] &&
		((!bus3 && a_in[11:0] >= 12'h014 && a_in[11:0] <= 12'h017) ||
		 (bus3 && a_in[11:0] == 12'hFF1));
	assign pointer_write_index = bus3 ? 6'd16 : {4'b0, a_in[1:0]};
	// Stella compares the whole address against a byte, so only $0000-$00FF
	// can ever stuff - a mirror such as $0284 must not.
	assign stuff_candidate = !rw && !a_in[12] && stuff_target_valid &&
		a_in[11:0] == {4'b0, stuff_target} && a_in[6:0] <= 7'h24;
	assign jump_substitute = bus3 && program_read && jump_remaining != 0 &&
		a_in == jump_operand_address && rom_data == 8'b0;

	always_comb begin
		if (jump_substitute)
			stream_index = 6'd17;
		else if (stream_read || stream_write)
			stream_index = bus3 ? 6'd16 : {2'b0, a_in[3:0]};
		else if (pointer_write)
			stream_index = pointer_write_index;
		else if (stuff_state >= STUFF_STREAM)
			stream_index = {2'b0, stuff_stream};
		else
			stream_index = 6'b0;

		pointer_lookup_index = stream_index;
		if ((stuff_state == STUFF_IDLE && stuff_candidate) ||
			stuff_state == STUFF_MAP)
			increment_lookup_index = 6'd16 + {1'b0, a_in[4:0]};
		else
			increment_lookup_index = stream_index;

		pointer_step = table_pointer + {4'b0, table_increment[15:0], 12'b0};
		display_address = 15'd2048 + {3'b0, table_pointer[31:20]};

		rom_a = (revision == 2'd0 ? 19'd3072 : 19'd4096) +
			{4'b0, bank, 12'b0} + {7'b0, a_in[11:0]};
		oe = a_in[12] ? 8'hFF : 8'h00;
		flags_out = 16'b0;
		d_out = 8'b0;
		ram_en = 1'b0;
		ram_write = 1'b0;
		ram_addr = display_address;
		ram_wdata = d_in;
		stuff_valid = stuff_state == STUFF_READY && stuff_candidate;
		stuff_data = ram_rdata;

		if (stream_read || jump_substitute) begin
			flags_out[0] = 1'b1;
			ram_en = 1'b1;
			d_out = ram_rdata;
		end else if (amplitude_read) begin
			flags_out[0] = 1'b1;
			d_out = amplitude;
		end

		if (stream_write) begin
			ram_en = 1'b1;
			ram_write = access;
		end else if (stuff_state == STUFF_DATA || stuff_state == STUFF_READY) begin
			ram_en = 1'b1;
		end
	end

	assign call_request = call_pending && call_ready;
	assign call_entry = 32'h00000808;
	assign call_stack = 32'h40001FFC;
	assign call_thumb = 1'b1;

	always @(posedge clk) begin
		last_a <= a_in;
		last_rw <= rw;

		if (reset) begin
			bank <= revision == 2'd0 ? 3'd5 : 3'd6;
			mode <= 8'hFF;
			call_pending <= 1'b0;
			sty_pending <= 1'b0;
			sty_operand_address <= 13'b0;
			stuff_target_valid <= 1'b0;
			stuff_target <= 8'b0;
			jump_remaining <= 2'b0;
			jump_operand_address <= 13'b0;
			stuff_map <= 32'b0;
			stuff_stream <= 4'b0;
			stuff_state <= STUFF_IDLE;
			pointer_update <= 1'b0;
			pointer_update_index <= 6'b0;
			pointer_update_value <= 32'b0;
			map_update <= 1'b0;
			map_update_index <= 6'b0;
			map_update_value <= 32'b0;
		end else begin
			pointer_update <= 1'b0;
			map_update <= 1'b0;
			if (call_pending && call_ready)
				call_pending <= 1'b0;

			case (stuff_state)
				STUFF_IDLE: begin
					if (bus_change && stuff_candidate)
						stuff_state <= STUFF_MAP;
				end
				STUFF_MAP: begin
					if (!stuff_candidate)
						stuff_state <= STUFF_IDLE;
					else begin
						stuff_map <= table_increment;
						stuff_stream <= table_increment[3:0];
						stuff_state <= STUFF_STREAM;
					end
				end
				STUFF_STREAM: stuff_state <= stuff_candidate ? STUFF_DATA : STUFF_IDLE;
				STUFF_DATA: stuff_state <= stuff_candidate ? STUFF_READY : STUFF_IDLE;
				default: begin
					if (!stuff_candidate)
						stuff_state <= STUFF_IDLE;
				end
			endcase

			if (access && supported) begin
				// A substituted read returns before Stella's hotspot switch.
				if (a_in[12] && !jump_substitute &&
					a_in[11:0] >= 12'hFF5 && a_in[11:0] <= 12'hFFB)
					bank <= a_in[2:0] - 3'd5;

				if (program_read) begin
					if (jump_remaining != 0) begin
						if (jump_substitute) begin
							pointer_update <= 1'b1;
							pointer_update_index <= 6'd17;
							pointer_update_value <= table_pointer + 32'h00100000;
							jump_remaining <= jump_remaining - 2'd1;
							jump_operand_address <= jump_operand_address + 13'd1;
						end else begin
							jump_remaining <= 2'b0;
						end
					end else if (bus3 && fast_mode && rom_data == 8'h4C &&
						fast_jump_valid) begin
						jump_remaining <= 2'd2;
						jump_operand_address <= a_in + 13'd1;
					end

					if (sty_pending) begin
						if (a_in == sty_operand_address) begin
							stuff_target <= rom_data;
							stuff_target_valid <= 1'b1;
						end
						sty_pending <= 1'b0;
					end else if (fast_mode && rom_data == 8'h84) begin
						sty_pending <= 1'b1;
						sty_operand_address <= a_in + 13'd1;
					end

					if (stream_read && !jump_substitute) begin
						pointer_update <= 1'b1;
						pointer_update_index <= stream_index;
						pointer_update_value <= pointer_step;
					end
				end else if (stream_write) begin
					pointer_update <= 1'b1;
					pointer_update_index <= stream_index;
					pointer_update_value <= table_pointer + 32'h00100000;
				end else if (!rw && a_in[12]) begin
					if (pointer_write) begin
						pointer_update <= 1'b1;
						pointer_update_index <= pointer_write_index;
						pointer_update_value <=
							({table_pointer[23:0], 8'b0} & 32'hF0000000) |
							{4'b0, d_in, 20'b0};
					end else if ((!bus3 && a_in[11:0] == 12'h019) ||
						(bus3 && a_in[11:0] == 12'hFF2)) begin
						// Only BUS3 keeps the byte; BUS1/2's STUFFMODE stores
						// nothing but "on" or "off".
						mode <= bus3 ? d_in :
							(d_in == 8'b0 ? 8'h00 : 8'h0F);
					end else if (((!bus3 && a_in[11:0] == 12'h01A) ||
						(bus3 && a_in[11:0] == 12'hFF3)) &&
						(d_in == 8'hFE || d_in == 8'hFF) && !call_pending) begin
						call_pending <= 1'b1;
					end
				end else if (stuff_state == STUFF_READY && stuff_candidate) begin
					pointer_update <= 1'b1;
					pointer_update_index <= {2'b0, stuff_stream};
					pointer_update_value <= pointer_step;
					map_update <= 1'b1;
					map_update_index <= {1'b0, a_in[4:0]};
					map_update_value <= {stuff_stream, stuff_map[31:4]};
					stuff_target_valid <= 1'b0;
					stuff_state <= STUFF_IDLE;
				end else if (!rw && !a_in[12] && !stuff_candidate) begin
					// One armed target, one write: Stella disarms on any poke
					// below $1000, including one it declined to stuff.
					stuff_target_valid <= 1'b0;
				end
			end
		end
	end
endmodule
