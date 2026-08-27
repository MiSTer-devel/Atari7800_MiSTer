// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

// DPC+ cartridge-side registers and fetchers. ROM and shared RAM remain in
// their existing external memories; this block only holds mapper state.
module mapper_dpcplus
(
	input  logic        clk,
	input  logic        reset,
	input  logic        access,
	input  logic        rw,
	input  logic [12:0] a_in,
	input  logic  [7:0] d_in,
	input  logic  [7:0] rom_data,
	input  logic        stable_fractional,

	output logic  [7:0] d_out,
	output logic [15:0] flags_out,
	output logic  [7:0] oe,
	output logic [18:0] rom_a,
	output logic        ram_sel,
	output logic        ram_rw,
	output logic [17:0] ram_a,
	input  logic  [7:0] ram_data,
	input  logic  [7:0] amplitude,
	output logic  [6:0] audio_waveform0,
	output logic  [6:0] audio_waveform1,
	output logic  [6:0] audio_waveform2,
	output logic        audio_note_write,
	output logic  [1:0] audio_note_voice,
	output logic  [7:0] audio_note_value,

	output logic        call_request,
	output logic [31:0] call_entry,
	output logic [31:0] call_stack,
	output logic        call_thumb,
	input  logic        call_ready,

	output logic        service_request,
	output logic        service_fill,
	output logic [18:0] service_source,
	output logic [14:0] service_dest,
	output logic  [7:0] service_count,
	output logic  [7:0] service_value,
	input  logic        service_ready
);
	logic [2:0] bank;
	logic [7:0] top [0:7];
	logic [7:0] bottom [0:7];
	logic [11:0] counter [0:7];
	logic [19:0] fractional [0:7];
	logic [7:0] increment [0:7];
	logic [31:0] random_number;
	logic fast_fetch;
	logic fast_pending;
	logic [7:0] params [0:7];
	logic [3:0] parameter_pointer;
	logic [6:0] waveform [0:2];
	logic call_pending;
	logic service_pending;

	logic [5:0] register_address;
	logic register_read;
	logic ram_register_read;
	logic [2:0] read_index;
	logic [2:0] read_function;
	logic [7:0] window_flag;
	logic [7:0] window_set;
	logic [31:0] random_next;
	logic [31:0] random_prior;
	logic [31:0] random_prior_xor;
	logic [11:0] ram_counter_address;
	logic [15:0] service_rom_offset;
	logic [12:0] service_dest_available;
	logic [16:0] service_source_available;
	logic  [7:0] service_fill_count;
	logic  [7:0] service_copy_count;

	assign random_next = ((random_number >> 11) | (random_number << 21)) ^
		(random_number[10] ? 32'h10ADAB1E : 32'b0);
	assign random_prior_xor = random_number[31] ?
		(random_number ^ 32'h10ADAB1E) : random_number;
	assign random_prior = (random_prior_xor << 11) |
		(random_prior_xor >> 21);
	assign service_rom_offset = {params[1], params[0]};
	assign service_dest_available = 13'h1000 -
		{1'b0, counter[params[2][2:0]]};
	assign service_source_available = 17'h07400 -
		{1'b0, service_rom_offset};

	always_comb begin
		service_fill_count = params[3];
		if (service_dest_available < {5'b0, params[3]})
			service_fill_count = service_dest_available[7:0];

		service_copy_count = service_fill_count;
		if (service_rom_offset >= 16'h7400)
			service_copy_count = 8'b0;
		else if (service_source_available < {9'b0, service_fill_count})
			service_copy_count = service_source_available[7:0];
	end

	// Each stream's window test on its own, so the fetched ROM byte only picks
	// one afterwards. Reading top/counter/bottom through read_index instead put
	// a mux, a subtract and a compare between the cartridge ROM and the 6507
	// data bus - about 3.5 ns of the 20 ns that path could not afford.
	always_comb
		for (int i = 0; i < 8; i++)
			window_set[i] = (top[i] - counter[i][7:0]) >
				(top[i] - bottom[i]);

	always_comb begin
		register_read = rw && a_in[12] &&
			((a_in[11:0] < 12'h028) ||
			(fast_fetch && fast_pending && rom_data < 8'h28));
		register_address = a_in[11:0] < 12'h028 ? a_in[5:0] :
			{1'b0, rom_data[4:0]};
		read_index = register_address[2:0];
		read_function = register_address[5:3];
		ram_register_read = register_read && read_function >= 3'd1 &&
			read_function <= 3'd3;
		window_flag = window_set[read_index] ? 8'hFF : 8'h00;

		rom_a = 19'd3072 + {4'b0, bank, 12'b0} +
			{7'b0, a_in[11:0]};
		oe = a_in[12] ? 8'hFF : 8'h00;
		flags_out = 16'b0;
		d_out = 8'b0;
		ram_sel = 1'b0;
		ram_rw = 1'b1;
		ram_a = 18'b0;
		ram_counter_address = counter[a_in[2:0]];

		if (ram_register_read) begin
			ram_sel = 1'b1;
			if (read_function < 3'd3)
				ram_counter_address = counter[read_index];
			else
				ram_counter_address = fractional[read_index][19:8];
			ram_a = 18'h00C00 + {6'b0, ram_counter_address};
		end else if (!rw && a_in[12] && (
			(a_in[11:0] >= 12'h060 && a_in[11:0] < 12'h068) ||
			(a_in[11:0] >= 12'h078 && a_in[11:0] < 12'h080))) begin
			// Only DFxPUSH ($060-$067, at counter-1) and DFxWRITE ($078-$07F,
			// at counter) reach display RAM. DFxHI ($068-$06F) and the random
			// number and note registers ($070-$077) sit between them and must
			// leave RAM alone.
			ram_sel = 1'b1;
			ram_rw = 1'b0;
			if (a_in[11:0] < 12'h068)
				ram_counter_address = counter[a_in[2:0]] - 12'd1;
			else
				ram_counter_address = counter[a_in[2:0]];
			ram_a = 18'h00C00 + {6'b0, ram_counter_address};
		end

		if (register_read) begin
			flags_out[0] = 1'b1;
			case (read_function)
				3'd0: begin
					case (read_index)
						3'd0: d_out = random_next[7:0];
						3'd1: d_out = random_prior[7:0];
						3'd2: d_out = random_number[15:8];
						3'd3: d_out = random_number[23:16];
						3'd4: d_out = random_number[31:24];
						3'd5: d_out = amplitude;
						default: d_out = 8'b0;
					endcase
				end
				3'd1: d_out = ram_data;
				3'd2: d_out = ram_data & window_flag;
				3'd3: d_out = ram_data;
				3'd4: d_out = read_index < 4 ? window_flag : 8'b0;
				default: d_out = 8'b0;
			endcase
		end
	end

	assign call_request = call_pending && call_ready;
	assign call_entry = 32'h00000C08;
	assign call_stack = 32'h40001FFC;
	assign call_thumb = 1'b1;
	assign service_request = service_pending && service_ready;
	assign audio_waveform0 = waveform[0];
	assign audio_waveform1 = waveform[1];
	assign audio_waveform2 = waveform[2];

	integer i;
	always @(posedge clk) begin
		if (reset) begin
			bank <= 3'd5;
			for (i = 0; i < 8; i = i + 1) begin
				top[i] <= 8'b0;
				bottom[i] <= 8'b0;
				counter[i] <= 12'b0;
				fractional[i] <= 20'b0;
				increment[i] <= 8'b0;
				params[i] <= 8'b0;
			end
			for (i = 0; i < 3; i = i + 1)
				waveform[i] <= 7'b0;
			random_number <= 32'h2B435044;
			fast_fetch <= 1'b0;
			fast_pending <= 1'b0;
			parameter_pointer <= 4'b0;
			call_pending <= 1'b0;
			service_pending <= 1'b0;
			service_fill <= 1'b0;
			service_source <= 19'b0;
			service_dest <= 15'b0;
			service_count <= 8'b0;
			service_value <= 8'b0;
			audio_note_write <= 1'b0;
			audio_note_voice <= 2'b0;
			audio_note_value <= 8'b0;
		end else begin
			audio_note_write <= 1'b0;
			if (call_pending && call_ready)
				call_pending <= 1'b0;
			if (service_pending && service_ready)
				service_pending <= 1'b0;

			if (access && a_in[12]) begin
				if (a_in[11:0] >= 12'hFF6 && a_in[11:0] <= 12'hFFB)
					bank <= a_in[2:0] - 3'd6;

				if (rw) begin
					if (register_read) begin
						fast_pending <= 1'b0;
						case (read_function)
							3'd0: begin
								if (read_index == 3'd0)
									random_number <= random_next;
								else if (read_index == 3'd1)
									random_number <= random_prior;
							end
							3'd1, 3'd2: counter[read_index] <=
								counter[read_index] + 12'd1;
							3'd3: fractional[read_index] <=
								fractional[read_index] + {12'b0, increment[read_index]};
							default: ;
						endcase
					end else begin
						fast_pending <= fast_fetch && rom_data == 8'hA9;
					end
				end else if (a_in[11:0] >= 12'h028 && a_in[11:0] < 12'h080) begin
					case ((a_in[11:0] - 12'h028) >> 3)
						4'd0: fractional[a_in[2:0]] <=
							(fractional[a_in[2:0]] &
							(stable_fractional ? 20'hF0000 : 20'hF00FF)) |
							{4'b0, d_in, 8'b0};
						4'd1: fractional[a_in[2:0]] <=
							{d_in[3:0], fractional[a_in[2:0]][15:0]};
						4'd2: begin
							increment[a_in[2:0]] <= d_in;
							fractional[a_in[2:0]] <=
								{fractional[a_in[2:0]][19:8], 8'b0};
						end
						4'd3: top[a_in[2:0]] <= d_in;
						4'd4: bottom[a_in[2:0]] <= d_in;
						4'd5: counter[a_in[2:0]][7:0] <= d_in;
						4'd6: begin
							case (a_in[2:0])
								3'd0: fast_fetch <= d_in == 8'b0;
								3'd1: begin
									if (parameter_pointer < 8) begin
										params[parameter_pointer[2:0]] <= d_in;
										parameter_pointer <= parameter_pointer + 4'd1;
									end
								end
								3'd2: begin
									if (d_in == 8'd0)
										parameter_pointer <= 4'b0;
									else if ((d_in == 8'd1 || d_in == 8'd2) &&
										!service_pending) begin
										service_fill <= d_in == 8'd2;
										service_source <= 19'd3072 +
											{3'b0, params[1], params[0]};
										service_dest <= 15'd3072 +
											{3'b0, counter[params[2][2:0]]};
										service_count <= d_in == 8'd2 ?
											service_fill_count : service_copy_count;
										service_value <= params[0];
										service_pending <= 1'b1;
										parameter_pointer <= 4'b0;
									end else if ((d_in == 8'hFE || d_in == 8'hFF) &&
										!call_pending)
										call_pending <= 1'b1;
								end
								3'd5, 3'd6, 3'd7:
									waveform[a_in[2:0] - 3'd5] <= d_in[6:0];
								default: ;
							endcase
						end
						4'd7: counter[a_in[2:0]] <= counter[a_in[2:0]] - 12'd1;
						4'd8: counter[a_in[2:0]][11:8] <= d_in[3:0];
						4'd9: begin
							case (a_in[2:0])
								3'd0: random_number <= 32'h2B435044;
								3'd1: random_number[7:0] <= d_in;
								3'd2: random_number[15:8] <= d_in;
								3'd3: random_number[23:16] <= d_in;
								3'd4: random_number[31:24] <= d_in;
								3'd5, 3'd6, 3'd7: begin
									audio_note_write <= 1'b1;
									audio_note_voice <= a_in[1:0] - 2'd1;
									audio_note_value <= d_in;
								end
								default: ;
							endcase
						end
						4'd10: counter[a_in[2:0]] <= counter[a_in[2:0]] + 12'd1;
						default: ;
					endcase
				end
			end
		end
	end
endmodule
