// SPDX-License-Identifier: MIT
// SN76489AN programmable tone/noise generator. Written from TI's manual and
// SMS Power's development page; no die shot or netlist of this part exists in
// references/. Decision 0085 records the choices below.
//
//   CLOCK ─┬─ /8 ─┬─ /2 ──> tone counters (3), noise counter, LFSR   N/16
//          │      └──────> write sequencer, four ticks per transfer  N/8
//   /CE ───┴─> READY low at once, high when the transfer completes
//
// Pin contract. The part has no reset; `reset` is FPGA housekeeping and wakes
// the chip silent (the real one wakes up random). `clk_en` is one tick per
// CLOCK cycle: 3.579545 MHz on the ACE board, clk_sys/4 in this core. `d[7]`
// is TI's "D0 (MSB)" pin, so bytes are the usual %1cctdddd form. READY is an
// open collector: `ready` is the pin level with its pull-up, `ready_oe` is
// high while the chip pulls it low.
//
// Timing. /CE's leading edge drops READY and queues a transfer. The transfer
// is four ticks of the N/8 clock (25 to 32 CLOCKs, "approximately 32" in the
// manual); on its last tick `d` and `we_n` are sampled and, if /WE is low,
// the register write lands and READY releases. The manual's only hold spec is
// "DATA w.r.t. READY, 0 ns", and every known host - Z80 /WAIT, TMS9900 READY,
// the ACE board's 74273 - holds the bus until READY, so a host that cannot
// must latch. A second /CE edge during a transfer queues one more.
//
// Registers: tone n gives a half period of n N/16 ticks, f = N / (32 n); n = 0
// counts 1024, as the TI parts do. Noise: 15-bit shift register, taps 0 and 1,
// shifted on the rising edge of its source (bits 4-6 of a free-running
// counter for N/512, N/1024, N/2048, or tone 2's flip-flop). A noise-control
// write loads a lone 1 in the top bit (decision 0086), so the first 1 reaches
// the output 14 shifts later.
// Output is the sum of four unipolar 2 dB-step levels, as the chip's summing
// amplifier does before the board's coupling capacitor.

`default_nettype none

module sn76489 (
	input  wire        clk,
	input  wire        clk_en,    // one tick per CLOCK pin cycle
	input  wire        reset,
	input  wire        ce_n,      // pin 6
	input  wire        we_n,      // pin 5
	input  wire  [7:0] d,         // d[7] is the latch/data flag
	output reg         ready,     // pin 4, level with pull-up
	output wire        ready_oe,  // 1 while pin 4 is pulled low
	output reg  [13:0] aud        // 0 to 4 x 4095
);

	// Clock chain. The tone counters see N/16, the sequencer N/8.
	reg [3:0] div;
	always_ff @(posedge clk) begin
		if (reset)       div <= 4'd0;
		else if (clk_en) div <= div + 4'd1;
	end
	wire en8  = clk_en && div[2:0] == 3'd7;
	wire en16 = clk_en && div == 4'd15;

	// Write sequencer.
	reg       ce_n_q;
	reg       pending;
	reg [1:0] seq;
	wire      apply = en8 && seq == 2'd3 && !we_n;

	assign ready_oe = ~ready;

	always_ff @(posedge clk) begin
		ce_n_q <= ce_n;
		if (reset) begin
			ce_n_q  <= 1'b1;
			pending <= 1'b0;
			seq     <= 2'd0;
			ready   <= 1'b1;
		end else begin
			if (en8) case (seq)
				2'd0: if (pending) begin
					seq     <= 2'd1;
					pending <= 1'b0;
				end
				2'd3: begin
					seq   <= 2'd0;
					ready <= 1'b1;
				end
				default: seq <= seq + 2'd1;
			endcase
			// The real pin responds asynchronously; one clk is the closest
			// a synchronous design gets, and it does not depend on clk rate.
			if (!ce_n && ce_n_q) begin
				pending <= 1'b1;
				ready   <= 1'b0;
			end
		end
	end

	// Control registers. A latch byte names the register and gives its low
	// four bits; a data byte reaches the same register with d[5:0].
	reg [9:0] tone [3];
	reg [3:0] vol  [4];
	reg [2:0] ctrl;
	reg [2:0] latch;
	wire [2:0] sel = d[7] ? d[6:4] : latch;

	function automatic logic [9:0] tone_write(input [9:0] cur, input is_latch, input [5:0] bits);
		tone_write = is_latch ? {cur[9:4], bits[3:0]} : {bits, cur[3:0]};
	endfunction

	always_ff @(posedge clk) begin
		if (reset) begin
			tone  <= '{10'd0, 10'd0, 10'd0};
			vol   <= '{4'hF, 4'hF, 4'hF, 4'hF};
			ctrl  <= 3'd0;
			latch <= 3'd0;
		end else if (apply) begin
			if (d[7]) latch <= d[6:4];
			case (sel)
				3'd0:    tone[0] <= tone_write(tone[0], d[7], d[5:0]);
				3'd2:    tone[1] <= tone_write(tone[1], d[7], d[5:0]);
				3'd4:    tone[2] <= tone_write(tone[2], d[7], d[5:0]);
				3'd6:    ctrl    <= d[2:0];
				default: vol[sel[2:1]] <= d[3:0];
			endcase
		end
	end

	// Tone generators: a 10-bit down counter whose borrow reloads it and
	// toggles the output. Reload is n - 1 so n ticks pass between toggles;
	// n = 0 wraps to 1023 and gives the 1024-tick period of the TI parts.
	reg  [9:0] tone_cnt [3];
	reg        tone_ff  [3];
	wire [2:0] borrow;
	assign borrow[0] = tone_cnt[0] == 10'd0;
	assign borrow[1] = tone_cnt[1] == 10'd0;
	assign borrow[2] = tone_cnt[2] == 10'd0;

	always_ff @(posedge clk) begin
		if (reset) begin
			tone_cnt <= '{10'd0, 10'd0, 10'd0};
			tone_ff  <= '{1'b0, 1'b0, 1'b0};
		end else if (en16) begin
			for (int i = 0; i < 3; i++) begin
				if (borrow[i]) begin
					tone_cnt[i] <= tone[i] - 10'd1;
					tone_ff[i]  <= ~tone_ff[i];
				end else begin
					tone_cnt[i] <= tone_cnt[i] - 10'd1;
				end
			end
		end
	end

	// Noise. The shift register is clocked by the rising edge of its source,
	// so the shift lands on the same tick as that edge.
	reg  [6:0]  ncnt;
	reg  [14:0] lfsr;
	wire [6:0]  ncnt_next = ncnt + 7'd1;
	wire        ff2_next  = tone_ff[2] ^ borrow[2];

	function automatic logic noise_source(input [1:0] rate, input [2:0] n_hi, input t2);
		case (rate)
			2'd0:    noise_source = n_hi[0];  // counter bit 4, N/512
			2'd1:    noise_source = n_hi[1];  // counter bit 5, N/1024
			2'd2:    noise_source = n_hi[2];  // counter bit 6, N/2048
			default: noise_source = t2;       // tone 2
		endcase
	endfunction

	wire src_now    = noise_source(ctrl[1:0], ncnt[6:4], tone_ff[2]);
	wire src_next   = noise_source(ctrl[1:0], ncnt_next[6:4], ff2_next);
	wire shift      = en16 && src_next && !src_now;
	wire noise_clear = apply && sel == 3'd6;
	wire feedback   = lfsr[0] ^ (ctrl[2] & lfsr[1]);

	always_ff @(posedge clk) begin
		if (reset) begin
			ncnt <= 7'd0;
			lfsr <= 15'h4000;
		end else begin
			if (en16) ncnt <= ncnt_next;
			if (noise_clear)  lfsr <= 15'h4000;
			else if (shift)   lfsr <= {feedback, lfsr[14:1]};
		end
	end

	// Attenuators and the summing amplifier. 2 dB a step, 1111 is off.
	function automatic logic [11:0] level(input [3:0] att);
		case (att)
			4'd0:  level = 12'd4095;
			4'd1:  level = 12'd3253;
			4'd2:  level = 12'd2584;
			4'd3:  level = 12'd2052;
			4'd4:  level = 12'd1630;
			4'd5:  level = 12'd1295;
			4'd6:  level = 12'd1029;
			4'd7:  level = 12'd817;
			4'd8:  level = 12'd649;
			4'd9:  level = 12'd516;
			4'd10: level = 12'd410;
			4'd11: level = 12'd325;
			4'd12: level = 12'd258;
			4'd13: level = 12'd205;
			4'd14: level = 12'd163;
			default: level = 12'd0;
		endcase
	endfunction

	reg [11:0] lvl [4];
	always_ff @(posedge clk) begin
		lvl[0] <= tone_ff[0] ? level(vol[0]) : 12'd0;
		lvl[1] <= tone_ff[1] ? level(vol[1]) : 12'd0;
		lvl[2] <= tone_ff[2] ? level(vol[2]) : 12'd0;
		lvl[3] <= lfsr[0]    ? level(vol[3]) : 12'd0;
		aud <= {2'd0, lvl[0]} + {2'd0, lvl[1]} + {2'd0, lvl[2]} + {2'd0, lvl[3]};
	end

endmodule

`default_nettype wire
