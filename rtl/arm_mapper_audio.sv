// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

// The cartridge oscillator runs in clk_sys. BUS/CDF note and counter state is
// exchanged with the native ARM helpers through banked FIQ R8-R13.
module arm_mapper_audio
#(
	parameter logic [23:0] CLK_RATE = 24'd14318182,
	parameter logic [23:0] AUDIO_RATE = 24'd20000
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic  [1:0] family,
	input  logic  [1:0] revision,
	input  logic [31:0] rom_size,
	input  logic [15:0] mapper_ram_size,
	input  logic [15:0] audio_size_addr,
	input  logic        bus_digital_audio,
	input  logic        cdf_digital_audio,

	input  logic  [6:0] dpc_waveform0,
	input  logic  [6:0] dpc_waveform1,
	input  logic  [6:0] dpc_waveform2,
	input  logic        dpc_note_write,
	input  logic  [1:0] dpc_note_voice,
	input  logic  [7:0] dpc_note_value,

	input  logic        call_launch,
	input  logic        call_done,
	input  logic [31:0] counter0_return,
	input  logic [31:0] counter1_return,
	input  logic [31:0] counter2_return,
	input  logic [31:0] frequency0_return,
	input  logic [31:0] frequency1_return,
	input  logic [31:0] frequency2_return,
	output logic [31:0] counter0,
	output logic [31:0] counter1,
	output logic [31:0] counter2,
	output logic [31:0] frequency0,
	output logic [31:0] frequency1,
	output logic [31:0] frequency2,

	output logic        ram_en,
	output logic [16:0] ram_addr,
	input  logic        ram_grant,
	input  logic  [7:0] ram_byte_data,
	input  logic [31:0] ram_word_data,

	output logic        rom_request,
	output logic [24:0] rom_addr,
	input  logic        rom_ready,
	input  logic        rom_done,
	input  logic  [7:0] rom_data,
	output logic  [7:0] amplitude
);
	localparam logic [23:0] TICK_THRESHOLD = CLK_RATE - AUDIO_RATE;

	typedef enum logic [3:0] {
		AUDIO_IDLE,
		AUDIO_NOTE_ISSUE,
		AUDIO_NOTE_CAPTURE,
		AUDIO_POINTER_ISSUE,
		AUDIO_POINTER_CAPTURE,
		AUDIO_SIZE_ISSUE,
		AUDIO_SIZE_CAPTURE,
		AUDIO_SAMPLE_ISSUE,
		AUDIO_SAMPLE_CAPTURE,
		AUDIO_DIGITAL_ROUTE,
		AUDIO_ROM_ISSUE,
		AUDIO_ROM_WAIT
	} audio_state_t;

	audio_state_t state;
	logic [23:0] tick_accum;
	wire audio_tick = tick_accum >= TICK_THRESHOLD;
	logic refresh_pending;
	logic note_pending;
	logic [1:0] note_voice;
	logic [7:0] note_value;
	logic [1:0] voice;
	logic [31:0] refresh_counter [0:2];
	logic [31:0] call_seed_counter [0:2];
	logic [31:0] waveform_pointer;
	logic [14:0] waveform_offset;
	logic [4:0] waveform_shift;
	logic [9:0] sample_sum;
	logic digital_sample;
	logic digital_low_nibble;
	logic [31:0] digital_address;
	logic [14:0] digital_ram_addr;
	logic [31:0] shifted_sample_index;
	logic [31:0] sample_offset_sum;
	logic [14:0] waveform_base;
	logic digital_mode;
	logic jplus_sample;
	logic [6:0] selected_dpc_waveform;

	always_comb begin
		waveform_base = 15'h07F4;
		if (family == 2'd3)
			waveform_base = revision == 2'd0 ? 15'h07F0 : 15'h01B0;

		// Only BUS3 has a sample register. BUS1/2 read their amplitude from
		// $1018, which always sums the three waveforms, so their SETMODE 0
		// must not divert this into the sample path.
		digital_mode = (family == 2'd2 && revision == 2'd3 &&
			bus_digital_audio) || (family == 2'd3 && cdf_digital_audio);
		// Only CDFJ+ moved the sample window. BUS3 keeps the 21/20 split even
		// though it shares the revision number.
		jplus_sample = family == 2'd3 && revision == 2'd3;

		case (voice)
			2'd0: begin
				shifted_sample_index = refresh_counter[0] >> waveform_shift;
				selected_dpc_waveform = dpc_waveform0;
			end
			2'd1: begin
				shifted_sample_index = refresh_counter[1] >> waveform_shift;
				selected_dpc_waveform = dpc_waveform1;
			end
			default: begin
				shifted_sample_index = refresh_counter[2] >> waveform_shift;
				selected_dpc_waveform = dpc_waveform2;
			end
		endcase

		sample_offset_sum = {17'b0, waveform_offset} + shifted_sample_index;
		ram_en = state == AUDIO_NOTE_ISSUE ||
			state == AUDIO_POINTER_ISSUE || state == AUDIO_SIZE_ISSUE ||
			state == AUDIO_SAMPLE_ISSUE;
		ram_addr = 17'b0;
		case (state)
			AUDIO_NOTE_ISSUE: ram_addr = 17'h01C00 +
				{7'b0, note_value, 2'b00};
			AUDIO_POINTER_ISSUE: ram_addr =
				{2'b0, waveform_base} + {13'b0, voice, 2'b00};
			AUDIO_SIZE_ISSUE: ram_addr =
				{1'b0, audio_size_addr} + {13'b0, voice, 2'b00};
			AUDIO_SAMPLE_ISSUE: begin
				if (digital_sample)
					ram_addr = {2'b0, digital_ram_addr};
				else if (family == 2'd1)
					ram_addr = 17'h00C00 +
						{5'b0, selected_dpc_waveform, 5'b0} +
						{12'b0, shifted_sample_index[4:0]};
				else if (family == 2'd3 && revision == 2'd3)
					ram_addr = (17'h00800 + {2'b0, sample_offset_sum[14:0]}) &
						({1'b0, mapper_ram_size} - 17'd1);
				else
					ram_addr = 17'h00800 + {5'b0, sample_offset_sum[11:0]};
			end
			default: ;
		endcase

		rom_request = state == AUDIO_ROM_ISSUE && rom_ready;
		rom_addr = digital_address[24:0];
	end

	always @(posedge clk) begin
		integer channel;

		if (reset) begin
			tick_accum <= 24'b0;
			refresh_pending <= 1'b0;
			note_pending <= 1'b0;
			note_voice <= 2'b0;
			note_value <= 8'b0;
			voice <= 2'b0;
			for (channel = 0; channel < 3; channel = channel + 1) begin
				refresh_counter[channel] <= 32'b0;
				call_seed_counter[channel] <= 32'b0;
			end
			counter0 <= 32'b0;
			counter1 <= 32'b0;
			counter2 <= 32'b0;
			frequency0 <= 32'b0;
			frequency1 <= 32'b0;
			frequency2 <= 32'b0;
			waveform_pointer <= 32'b0;
			waveform_offset <= 15'b0;
			waveform_shift <= 5'd27;
			sample_sum <= 10'b0;
			digital_sample <= 1'b0;
			digital_low_nibble <= 1'b0;
			digital_address <= 32'b0;
			digital_ram_addr <= 15'b0;
			amplitude <= 8'b0;
			state <= AUDIO_IDLE;
		end else begin
			if (audio_tick) begin
				tick_accum <= tick_accum + AUDIO_RATE - CLK_RATE;
				counter0 <= counter0 + frequency0;
				counter1 <= counter1 + frequency1;
				counter2 <= counter2 + frequency2;
				refresh_pending <= family != 2'd0;
			end else begin
				tick_accum <= tick_accum + AUDIO_RATE;
			end

			if (dpc_note_write) begin
				note_pending <= 1'b1;
				note_voice <= dpc_note_voice;
				note_value <= dpc_note_value;
			end

			if (call_launch && family >= 2'd2) begin
				call_seed_counter[0] <= counter0;
				call_seed_counter[1] <= counter1;
				call_seed_counter[2] <= counter2;
			end

			if (call_done && family >= 2'd2) begin
				if (counter0_return != call_seed_counter[0])
					counter0 <= counter0_return;
				if (counter1_return != call_seed_counter[1])
					counter1 <= counter1_return;
				if (counter2_return != call_seed_counter[2])
					counter2 <= counter2_return;
				frequency0 <= frequency0_return;
				frequency1 <= frequency1_return;
				frequency2 <= frequency2_return;
			end

			case (state)
				AUDIO_IDLE: begin
					if (note_pending && family == 2'd1) begin
						state <= AUDIO_NOTE_ISSUE;
					end else if (refresh_pending) begin
						refresh_pending <= audio_tick;
						refresh_counter[0] <= counter0;
						refresh_counter[1] <= counter1;
						refresh_counter[2] <= counter2;
						voice <= 2'd0;
						sample_sum <= 10'b0;
						digital_sample <= 1'b0;
						waveform_shift <= 5'd27;
						state <= family == 2'd1 ?
							AUDIO_SAMPLE_ISSUE : AUDIO_POINTER_ISSUE;
					end
				end

				AUDIO_NOTE_ISSUE: begin
					if (ram_grant)
						state <= AUDIO_NOTE_CAPTURE;
				end

				AUDIO_NOTE_CAPTURE: begin
					case (note_voice)
						2'd0: frequency0 <= ram_word_data;
						2'd1: frequency1 <= ram_word_data;
						default: frequency2 <= ram_word_data;
					endcase
					if (!dpc_note_write)
						note_pending <= 1'b0;
					state <= AUDIO_IDLE;
				end

				AUDIO_POINTER_ISSUE: begin
					if (ram_grant)
						state <= AUDIO_POINTER_CAPTURE;
				end

				AUDIO_POINTER_CAPTURE: begin
					waveform_pointer <= ram_word_data;
					if (digital_mode) begin
						digital_address <= ram_word_data +
							(refresh_counter[0] >>
							(jplus_sample ? 5'd13 : 5'd21));
						digital_low_nibble <= jplus_sample ?
							refresh_counter[0][12] : refresh_counter[0][20];
						state <= AUDIO_DIGITAL_ROUTE;
					end else begin
						if (family == 2'd2) begin
							if (ram_word_data < 32'h40000800 ||
								ram_word_data >= 32'h40001800)
								waveform_offset <= 15'b0;
							else
								waveform_offset <=
									ram_word_data[14:0] - 15'h0800;
						end else if (revision == 2'd3) begin
							if (ram_word_data < 32'h40000800 ||
								ram_word_data - 32'h40000800 >=
									{16'b0, mapper_ram_size - 16'h0800})
								waveform_offset <= 15'b0;
							else
								waveform_offset <=
									ram_word_data[14:0] - 15'h0800;
						end else begin
							waveform_offset <= {3'b0,
								ram_word_data[11:0] - 12'h800};
						end
						if (audio_size_addr == 16'b0) begin
							waveform_shift <= 5'd27;
							state <= AUDIO_SAMPLE_ISSUE;
						end else begin
							state <= AUDIO_SIZE_ISSUE;
						end
					end
				end

				AUDIO_SIZE_ISSUE: begin
					if (ram_grant)
						state <= AUDIO_SIZE_CAPTURE;
				end

				AUDIO_SIZE_CAPTURE: begin
					waveform_shift <= ram_word_data[11:7];
					state <= AUDIO_SAMPLE_ISSUE;
				end

				AUDIO_SAMPLE_ISSUE: begin
					if (ram_grant)
						state <= AUDIO_SAMPLE_CAPTURE;
				end

				AUDIO_SAMPLE_CAPTURE: begin
					if (digital_sample) begin
						amplitude <= digital_low_nibble ?
							{4'b0, ram_byte_data[3:0]} :
							{4'b0, ram_byte_data[7:4]};
						state <= AUDIO_IDLE;
					end else if (voice == 2'd2) begin
						amplitude <= sample_sum[7:0] + ram_byte_data;
						state <= AUDIO_IDLE;
					end else begin
						sample_sum <= sample_sum + {2'b0, ram_byte_data};
						voice <= voice + 2'd1;
						waveform_shift <= 5'd27;
						state <= family == 2'd1 ?
							AUDIO_SAMPLE_ISSUE : AUDIO_POINTER_ISSUE;
					end
				end

				AUDIO_DIGITAL_ROUTE: begin
					if (digital_address < rom_size) begin
						state <= AUDIO_ROM_ISSUE;
					end else if (digital_address >= 32'h40000000 &&
						digital_address - 32'h40000000 <
							{16'b0, mapper_ram_size}) begin
						digital_ram_addr <= digital_address[14:0];
						digital_sample <= 1'b1;
						state <= AUDIO_SAMPLE_ISSUE;
					end else begin
						amplitude <= 8'b0;
						state <= AUDIO_IDLE;
					end
				end

				AUDIO_ROM_ISSUE: begin
					if (rom_ready)
						state <= AUDIO_ROM_WAIT;
				end

				default: begin // AUDIO_ROM_WAIT
					if (rom_done) begin
						// Keep a refresh queued if its tick arrived during DDR wait.
						amplitude <= digital_low_nibble ?
							{4'b0, rom_data[3:0]} : {4'b0, rom_data[7:4]};
						state <= AUDIO_IDLE;
					end
				end
			endcase
		end
	end
endmodule
