// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

// Cross mapper calls into clk_arm and load the halted ARM state port. The
// mapper never exposes the 2600 bus to ARM; parameters live in shared cart RAM.
module arm_mapper_controller
	import arm7tdmi_pkg::*;
(
	input  logic        clk_sys,
	input  logic        reset_sys,
	input  logic        mapper_reset_sys,
	input  logic        call_request,
	input  logic [31:0] call_entry,
	input  logic [31:0] call_stack,
	input  logic        call_thumb,
	input  logic [31:0] audio_counter0,
	input  logic [31:0] audio_counter1,
	input  logic [31:0] audio_counter2,
	input  logic [31:0] audio_frequency0,
	input  logic [31:0] audio_frequency1,
	input  logic [31:0] audio_frequency2,
	output logic        call_ready,
	output logic        call_busy,
	output logic        call_done,
	output logic [31:0] audio_counter0_return,
	output logic [31:0] audio_counter1_return,
	output logic [31:0] audio_counter2_return,
	output logic [31:0] audio_frequency0_return,
	output logic [31:0] audio_frequency1_return,
	output logic [31:0] audio_frequency2_return,

	input  logic        clk_arm,
	input  logic        reset_arm,
	input  logic        mapper_reset_arm,
	input  logic        shadow_ready,
	input  logic        return_fetch,
	input  logic        cpu_halted,
	output logic        halt_req,

	output logic        state_req,
	output logic        state_write,
	output logic  [5:0] state_index,
	output logic [31:0] state_wdata,
	input  logic        state_ready,
	input  logic [31:0] state_rdata,
	output logic        state_commit
);
	localparam logic [31:0] RETURN_SENTINEL = 32'hF0000000;

	// Held-payload request mailbox, owned by clk_sys.
	logic [31:0] call_entry_payload;
	logic [31:0] call_stack_payload;
	logic        call_thumb_payload;
	logic [31:0] audio_counter_payload [0:2];
	logic [31:0] audio_frequency_payload [0:2];
	logic        call_toggle;
	logic        call_ack_arm;
	logic        call_ack_sync1;
	logic        call_ack_sync2;

	// Held-payload completion mailbox, owned by clk_arm.
	logic complete_toggle;
	logic complete_token;
	logic complete_sync1;
	logic complete_sync2;
	logic complete_seen;
	logic complete_token_sync1;
	logic complete_token_sync2;
	logic complete_ack_sys;
	logic complete_ack_sync1;
	logic complete_ack_sync2;
	logic [31:0] audio_counter_result [0:2];
	logic [31:0] audio_frequency_result [0:2];
	logic [31:0] audio_counter_sync1 [0:2];
	logic [31:0] audio_counter_sync2 [0:2];
	logic [31:0] audio_frequency_sync1 [0:2];
	logic [31:0] audio_frequency_sync2 [0:2];

	logic arm_online_sync1;
	logic arm_online_sync2;
	logic shadow_ready_sync1;
	logic shadow_ready_sync2;
	logic sys_online_sync1;
	logic sys_online_sync2;

	assign call_ready = arm_online_sync2 && shadow_ready_sync2 &&
		!mapper_reset_sys && !call_busy;

	always @(posedge clk_sys) begin
		integer audio_index;

		if (reset_sys) begin
			call_entry_payload <= '0;
			call_stack_payload <= '0;
			call_thumb_payload <= 1'b0;
			for (audio_index = 0; audio_index < 3; audio_index = audio_index + 1) begin
				audio_counter_payload[audio_index] <= 32'b0;
				audio_frequency_payload[audio_index] <= 32'b0;
				audio_counter_sync1[audio_index] <= 32'b0;
				audio_counter_sync2[audio_index] <= 32'b0;
				audio_frequency_sync1[audio_index] <= 32'b0;
				audio_frequency_sync2[audio_index] <= 32'b0;
			end
			audio_counter0_return <= 32'b0;
			audio_counter1_return <= 32'b0;
			audio_counter2_return <= 32'b0;
			audio_frequency0_return <= 32'b0;
			audio_frequency1_return <= 32'b0;
			audio_frequency2_return <= 32'b0;
			call_toggle <= 1'b0;
			call_ack_sync1 <= 1'b0;
			call_ack_sync2 <= 1'b0;
			complete_sync1 <= 1'b0;
			complete_sync2 <= 1'b0;
			complete_seen <= 1'b0;
			complete_token_sync1 <= 1'b0;
			complete_token_sync2 <= 1'b0;
			complete_ack_sys <= 1'b0;
			arm_online_sync1 <= 1'b0;
			arm_online_sync2 <= 1'b0;
			shadow_ready_sync1 <= 1'b0;
			shadow_ready_sync2 <= 1'b0;
			call_busy <= 1'b0;
			call_done <= 1'b0;
		end else begin
			for (audio_index = 0; audio_index < 3; audio_index = audio_index + 1) begin
				audio_counter_sync1[audio_index] <= audio_counter_result[audio_index];
				audio_counter_sync2[audio_index] <= audio_counter_sync1[audio_index];
				audio_frequency_sync1[audio_index] <= audio_frequency_result[audio_index];
				audio_frequency_sync2[audio_index] <= audio_frequency_sync1[audio_index];
			end
			call_ack_sync1 <= call_ack_arm;
			call_ack_sync2 <= call_ack_sync1;
			complete_sync1 <= complete_toggle;
			complete_sync2 <= complete_sync1;
			complete_token_sync1 <= complete_token;
			complete_token_sync2 <= complete_token_sync1;
			arm_online_sync1 <= !reset_arm;
			arm_online_sync2 <= arm_online_sync1;
			shadow_ready_sync1 <= shadow_ready;
			shadow_ready_sync2 <= shadow_ready_sync1;
			call_done <= 1'b0;

			if (mapper_reset_sys) begin
				complete_seen <= complete_sync2;
				complete_ack_sys <= complete_sync2;
				call_busy <= 1'b0;
			end else begin
				if (call_request && call_ready) begin
					call_entry_payload <= call_entry;
					call_stack_payload <= call_stack;
					call_thumb_payload <= call_thumb;
					audio_counter_payload[0] <= audio_counter0;
					audio_counter_payload[1] <= audio_counter1;
					audio_counter_payload[2] <= audio_counter2;
					audio_frequency_payload[0] <= audio_frequency0;
					audio_frequency_payload[1] <= audio_frequency1;
					audio_frequency_payload[2] <= audio_frequency2;
					call_toggle <= ~call_toggle;
					call_busy <= 1'b1;
				end

				if (complete_sync2 != complete_seen) begin
					complete_seen <= complete_sync2;
					complete_ack_sys <= complete_sync2;
					if (call_busy && call_ack_sync2 == call_toggle &&
						complete_token_sync2 == call_toggle) begin
						audio_counter0_return <= audio_counter_sync2[0];
						audio_counter1_return <= audio_counter_sync2[1];
						audio_counter2_return <= audio_counter_sync2[2];
						audio_frequency0_return <= audio_frequency_sync2[0];
						audio_frequency1_return <= audio_frequency_sync2[1];
						audio_frequency2_return <= audio_frequency_sync2[2];
						call_busy <= 1'b0;
						call_done <= 1'b1;
					end
				end
			end
		end
	end

	typedef enum logic [3:0] {
		CTRL_IDLE,
		CTRL_WAIT_HALT,
		CTRL_WRITE_STATE,
		CTRL_COMMIT,
		CTRL_RELEASE,
		CTRL_RUNNING,
		CTRL_RETURN_HALT,
		CTRL_READ_AUDIO,
		CTRL_CAPTURE_AUDIO
	} control_state_t;
	control_state_t control_state;
	logic call_sync1;
	logic call_sync2;
	logic call_seen;
	logic active_token;
	logic [31:0] active_entry;
	logic [31:0] active_stack;
	logic active_thumb;
	logic [31:0] active_audio_counter [0:2];
	logic [31:0] active_audio_frequency [0:2];
	logic [4:0] write_index;
	logic [2:0] audio_read_index;
	// state_index as a register, updated wherever write_index or
	// audio_read_index change, so the CPU's register-file index decode starts
	// from a flop instead of behind this adder and mux.
	logic [5:0] state_index_q;

	always_comb begin
		halt_req = control_state != CTRL_RUNNING;
		state_req = control_state == CTRL_WRITE_STATE ||
			control_state == CTRL_READ_AUDIO;
		state_write = control_state == CTRL_WRITE_STATE;
		state_index = state_index_q;
		state_wdata = 32'b0;
		state_commit = control_state == CTRL_COMMIT;

		case (write_index)
			5'd13: state_wdata = active_stack;
			5'd14: state_wdata = RETURN_SENTINEL;
			5'd15: state_wdata = active_entry;
			5'd16: state_wdata = (active_thumb ? CPSR_T : 32'b0) |
				{27'b0, MODE_SYS};
			5'd17: state_wdata = active_audio_counter[0];
			5'd18: state_wdata = active_audio_counter[1];
			5'd19: state_wdata = active_audio_counter[2];
			5'd20: state_wdata = active_audio_frequency[0];
			5'd21: state_wdata = active_audio_frequency[1];
			5'd22: state_wdata = active_audio_frequency[2];
			default: state_wdata = 32'b0;
		endcase
	end

	always @(posedge clk_arm) begin
		integer audio_index;

		if (reset_arm) begin
			call_sync1 <= 1'b0;
			call_sync2 <= 1'b0;
			call_seen <= 1'b0;
			call_ack_arm <= 1'b0;
			complete_toggle <= 1'b0;
			complete_token <= 1'b0;
			complete_ack_sync1 <= 1'b0;
			complete_ack_sync2 <= 1'b0;
			sys_online_sync1 <= 1'b0;
			sys_online_sync2 <= 1'b0;
			active_token <= 1'b0;
			active_entry <= '0;
			active_stack <= '0;
			active_thumb <= 1'b0;
			for (audio_index = 0; audio_index < 3; audio_index = audio_index + 1) begin
				active_audio_counter[audio_index] <= 32'b0;
				active_audio_frequency[audio_index] <= 32'b0;
				audio_counter_result[audio_index] <= 32'b0;
				audio_frequency_result[audio_index] <= 32'b0;
			end
			write_index <= '0;
			audio_read_index <= '0;
			state_index_q <= '0;
			control_state <= CTRL_WAIT_HALT;
		end else begin
			call_sync1 <= call_toggle;
			call_sync2 <= call_sync1;
			complete_ack_sync1 <= complete_ack_sys;
			complete_ack_sync2 <= complete_ack_sync1;
			sys_online_sync1 <= !reset_sys;
			sys_online_sync2 <= sys_online_sync1;

			if (mapper_reset_arm) begin
				call_seen <= call_sync2;
				call_ack_arm <= call_sync2;
				complete_toggle <= complete_ack_sync2;
				complete_token <= 1'b0;
				control_state <= CTRL_WAIT_HALT;
			end else begin
				case (control_state)
					CTRL_WAIT_HALT: begin
						if (cpu_halted)
							control_state <= CTRL_IDLE;
					end

					CTRL_IDLE: begin
						if (sys_online_sync2 && shadow_ready && cpu_halted &&
							complete_ack_sync2 == complete_toggle &&
							call_sync2 != call_seen) begin
							active_token <= call_sync2;
							active_entry <= call_entry_payload;
							active_stack <= call_stack_payload;
							active_thumb <= call_thumb_payload;
							for (audio_index = 0; audio_index < 3;
								audio_index = audio_index + 1) begin
								active_audio_counter[audio_index] <=
									audio_counter_payload[audio_index];
								active_audio_frequency[audio_index] <=
									audio_frequency_payload[audio_index];
							end
							call_seen <= call_sync2;
							call_ack_arm <= call_sync2;
							write_index <= 5'd0;
							state_index_q <= 6'd0;
							control_state <= CTRL_WRITE_STATE;
						end
					end

					CTRL_WRITE_STATE: begin
						if (state_ready) begin
							if (write_index == 5'd22)
								control_state <= CTRL_COMMIT;
							else
								write_index <= write_index + 5'd1;
								state_index_q <= {1'b0, write_index + 5'd1};
						end
					end

					CTRL_COMMIT: control_state <= CTRL_RELEASE;
					CTRL_RELEASE: control_state <= CTRL_RUNNING;

					CTRL_RUNNING: begin
						if (return_fetch)
							control_state <= CTRL_RETURN_HALT;
					end

					CTRL_RETURN_HALT: begin
						if (cpu_halted) begin
							audio_read_index <= 3'd0;
							state_index_q <= STATE_FIQ_R8;
							control_state <= CTRL_READ_AUDIO;
						end
					end

					CTRL_READ_AUDIO: begin
						if (state_ready)
							control_state <= CTRL_CAPTURE_AUDIO;
					end

					default: begin // CTRL_CAPTURE_AUDIO
						if (audio_read_index < 3'd3)
							audio_counter_result[audio_read_index[1:0]] <= state_rdata;
						else
							audio_frequency_result[
								audio_read_index[1:0] - 2'd3] <=
								state_rdata;

						if (audio_read_index == 3'd5) begin
							complete_token <= active_token;
							complete_toggle <= ~complete_toggle;
							control_state <= CTRL_IDLE;
						end else begin
							audio_read_index <= audio_read_index + 3'd1;
							state_index_q <= STATE_FIQ_R8 +
								{3'b0, audio_read_index + 3'd1};
							control_state <= CTRL_READ_AUDIO;
						end
					end
				endcase
			end
		end
	end
endmodule
