// k7800 (c) by Jamie Blanks
//
// Copyright (c) 2026 Jamie Blanks
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// Minnie's data path, GCC-1730 sections 2.1 through 2.1.3, and the microcode
// steps that drive it.
//
// ---------------------------------------------------------------------------
// One data path, three voices
// ---------------------------------------------------------------------------
// "There is only one data path, which is then time-division multiplexed among
// all three voices by the microcode." One 10 bit adder, one sign-extend
// shifter, one accumulator. The sequencer walks each voice through ten steps:
//
//   0  OSC_LO   index low  += freq low            keep the carry
//   1  OSC_HI   index high += freq high + carry
//   2  NZE_LO   index low  += noise low           keep the carry, latch noise
//   3  NZE_HI   index high += noise high + carry
//   4  FETCH    sample <- waveform logic          address is the post-noise index
//   5  MUL0     acc = VOL0 ? S : 0                mantissa, shift and add
//   6  MUL1     acc = acc/2 + (VOL1 ? S : 0)
//   7  MUL2     acc = acc/2 + (VOL2 ? S : 0)
//   8  MUL3     acc = acc/2 + S
//   9  SCALE    T += acc >> exponent              sign-extend shifter, then add
//
// Thirty states of work, of the 64 a sample tick allows. The index writeback
// rides along with each add - the block diagram routes the adder output back to
// the register array over DataFalse - so no separate writeback state is needed.
//
// Steps 5 to 8 compute S * (1 + VOL2/2 + VOL1/4 + VOL0/8), section 2.2.1.3.1,
// using the "fast feedback path" that can return the adder output shifted right
// by one. Working outward from the least significant mantissa bit:
//
//   A0 = VOL0*S
//   A1 = A0/2 + VOL1*S
//   A2 = A1/2 + VOL2*S
//   A3 = A2/2 + S      = S + VOL2*S/2 + VOL1*S/4 + VOL0*S/8
//
// The multiply runs at full 10 bit width and only then goes through the
// exponent shift, which is what the specification means by "in this way, the
// maximum resolution is preserved".
//
// ---------------------------------------------------------------------------
// One adder does both jobs
// ---------------------------------------------------------------------------
// Section 2.1.2 describes "logic allowing a carry to be detected from the 8th
// bit for multiple-byte adds". Feeding a byte in with the top two bits zeroed
// puts that carry at bit 8 of the sum, so the same 10 bit adder serves the
// 16 bit index adds and the 10 bit sample adds.
//
// ---------------------------------------------------------------------------
// Why the noise does not go through the sign-extend shifter
// ---------------------------------------------------------------------------
// Section 2.1.3 says it should: the shifter has an input multiplexer whose two
// sources are the waveform data path and the poly counter, a 3-bit shift
// control, and an output on the Ybus, and figure 2.0.1 draws the same thing.
// That route was built and measured, and it cannot work. The record is
// .agents/decisions/0020-minnie-noise-cannot-use-the-shared-shifter.md.
//
// The shifter hands over one byte. To reach the sixteen bits section 2.2.1.4.1
// names, that byte has to land in the high half of the index, which makes the
// smallest step 256 index counts. A two's complement value uniform over a
// 256-count step has a mean of -128 counts, not zero, and the index add is
// cumulative - so the mean is a fixed frequency offset of -54.6 Hz on every
// voice at every noise level above 7, regardless of the note. Measured at
// -43 Hz over the ladder. A 440 Hz note lands near 397; a 55 Hz bass note near
// 12. There is no shift amount that avoids it, because the problem is the step
// size, not the magnitude.
//
// So the noise is built here instead: the low NZE bits of the poly, sign
// extended to sixteen. Step one index count, mean half a count, magnitude
// doubling per level. Same ladder, none of the detune.
//
//   NZE 0    off
//   NZE k    +/-2^(k-1) index counts, uniform, step 1
//
// Read as unsigned rather than sign extended it would add a mean of about
// 16000 to the index at level 15 - roughly 7 kHz of pitch - which contradicts
// the "imperfection in frequency stability" section 2.2.1.4.1 describes at the
// bottom of its range. Signed is the only reading under which low levels are
// audible as instability rather than transposition.
//
`default_nettype none

module minnie_datapath (
	input  wire         clk,
	input  wire         rst,
	input  wire         step_en,        // one pulse per executed microcode state
	input  wire   [3:0] step,
	input  wire         latch_sample,

	// The current voice's registers. vol[7] is ZER and vol[0] is unused;
	// timbre[3] is unused. All three are accepted and ignored, per the sheet.
	input  wire   [7:0] freq_l,
	input  wire   [7:0] freq_h,
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire   [7:0] vol,
	input  wire   [7:0] timbre,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire   [7:0] idx_l,
	input  wire   [7:0] idx_h,

	// Index writeback.
	output wire         idx_wr,
	output wire         idx_hi,
	output wire   [7:0] idx_data,

	// Waveform logic.
	output wire   [2:0] wfm,
	input  wire   [7:0] wave_sample,

	// Noise source.
	input  wire  [14:0] poly,

	// T, the sample accumulator. Readable and writable from the bus, to no
	// useful effect - section 2.2.1.7 says so outright.
	input  wire         t_wr,
	input  wire   [7:0] t_din,
	output wire   [9:0] t_value,

	output logic  [9:0] sample_out,
	output logic        sample_en
);

	localparam [3:0] OSC_LO = 4'd0, OSC_HI = 4'd1,
	                 NZE_LO = 4'd2, NZE_HI = 4'd3,
	                 FETCH  = 4'd4,
	                 MUL0   = 4'd5, MUL1   = 4'd6,
	                 MUL2   = 4'd7, MUL3   = 4'd8,
	                 SCALE  = 4'd9;

	logic  [9:0] acc;
	logic  [9:0] t_reg;
	logic  [7:0] s_reg;
	logic        carry;
	/* verilator lint_off UNUSEDSIGNAL */
	logic [15:0] noise_lat;      // only the high byte is read back, at NZE_HI
	/* verilator lint_on UNUSEDSIGNAL */

	assign wfm     = timbre[2:0];
	assign t_value = t_reg;

	// Volume field, section 2.2.1.3. Bit 7 is ZER and bit 0 is unused, so the
	// six control bits sit in the middle: exponent in 6:4, mantissa in 3:1.
	// ZER has no effect here - the specification only promises "unexpected
	// things" without saying what, so the benign reading is the honest one.
	wire [2:0] vexp = vol[6:4];
	wire       m0   = vol[1];
	wire       m1   = vol[2];
	wire       m2   = vol[3];

	// Noise magnitude, section 2.2.1.4.1: the low NZE bits of the poly, sign
	// extended. Level 0 is exactly zero, and the step is one index count, which
	// is the property the whole scheme turns on - see the note above.
	wire  [3:0] nze      = timbre[7:4];
	wire [15:0] poly_ext = {1'b0, poly};
	wire        nsign    = poly_ext[(nze == 4'd0) ? 4'd0 : (nze - 4'd1)];

	logic [15:0] noise16;

	always_comb begin
		for (int i = 0; i < 16; i++)
			noise16[i] = (nze == 4'd0) ? 1'b0
			           : ((i[3:0] < nze) ? poly_ext[i] : nsign);
	end

	// The sample, sign extended into the 10 bit path, and the two shifter
	// outputs: the fast feedback path's right-by-one, and the exponent shift.
	wire  [9:0] s_ext   = {{2{s_reg[7]}}, s_reg};
	wire  [9:0] acc_sr1 = 10'($signed(acc) >>> 1);
	wire  [9:0] shifted = 10'($signed(acc) >>> vexp);

	logic [9:0] xbus, ybus;
	logic       cin;

	always_comb begin
		xbus = 10'd0;
		ybus = 10'd0;
		cin  = 1'b0;
		case (step)
			OSC_LO: begin xbus = {2'b00, idx_l}; ybus = {2'b00, freq_l};          end
			OSC_HI: begin xbus = {2'b00, idx_h}; ybus = {2'b00, freq_h}; cin = carry; end
			NZE_LO: begin xbus = {2'b00, idx_l}; ybus = {2'b00, noise16[7:0]};    end
			NZE_HI: begin xbus = {2'b00, idx_h}; ybus = {2'b00, noise_lat[15:8]}; cin = carry; end
			MUL0:   begin                        ybus = m0 ? s_ext : 10'd0;       end
			MUL1:   begin xbus = acc_sr1;        ybus = m1 ? s_ext : 10'd0;       end
			MUL2:   begin xbus = acc_sr1;        ybus = m2 ? s_ext : 10'd0;       end
			MUL3:   begin xbus = acc_sr1;        ybus = s_ext;                    end
			SCALE:  begin xbus = t_reg;          ybus = shifted;                  end
			default: ;
		endcase
	end

	// Eleven bits so the byte carry falls out at bit 8. Plain unsigned addition
	// produces the right two's complement bits either way.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [10:0] sum11 = {xbus[9], xbus} + {ybus[9], ybus} + {10'd0, cin};
	/* verilator lint_on UNUSEDSIGNAL */

	wire is_idx_step = (step == OSC_LO) || (step == OSC_HI)
	                || (step == NZE_LO) || (step == NZE_HI);

	assign idx_wr   = step_en && is_idx_step;
	assign idx_hi   = (step == OSC_HI) || (step == NZE_HI);
	assign idx_data = sum11[7:0];

	always_ff @(posedge clk) begin
		sample_en <= 1'b0;

		if (rst) begin
			acc        <= 10'd0;
			t_reg      <= 10'd0;
			s_reg      <= 8'd0;
			carry      <= 1'b0;
			noise_lat  <= 16'd0;
			sample_out <= 10'd0;
		end else begin
			if (step_en) begin
				case (step)
					OSC_LO: carry <= sum11[8];
					NZE_LO: begin
						carry     <= sum11[8];
						noise_lat <= noise16;
					end
					FETCH:  s_reg <= wave_sample;
					MUL0, MUL1, MUL2, MUL3:
					        acc   <= sum11[9:0];
					SCALE:  t_reg <= sum11[9:0];
					default: ;   // OSC_HI and NZE_HI only write the index
				endcase
			end

			// One sample per tick. T wraps rather than saturating: three loud
			// voices can reach +/-720 in a 10 bit accumulator and the real part
			// had nothing to stop them.
			if (latch_sample) begin
				sample_out <= t_reg;
				t_reg      <= 10'd0;
				sample_en  <= 1'b1;
			end

			// A bus write to T. Lands on the opposite phase from the microcode,
			// and is cleared again at the end of the tick regardless.
			if (t_wr)
				t_reg <= {t_reg[9:8], t_din};
		end
	end

endmodule

`default_nettype wire
