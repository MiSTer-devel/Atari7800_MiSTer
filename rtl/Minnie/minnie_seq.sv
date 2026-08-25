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

// Minnie's microcode sequencer. This is the FPGA stand-in for the control PLA.
//
// ---------------------------------------------------------------------------
// Two counters, and why they are separate
// ---------------------------------------------------------------------------
// "As the Minnie is operated directly from the 1.8 MHz clock, this allows for
// 64 internal microcode states for each complete sample calculation."
// 1.789772 MHz / 64 = 27.965 kHz.
//
//   tick    free running, 0..63, one step per processor clock. Never stalls.
//           Its wrap is the sample clock.
//   voice   0..2, and step 0..9 within a voice. Thirty states of real work.
//           A processor access holds these for one state.
//
// The separation is what produces the behaviour section 2.4.2 describes.
// Processor accesses steal microcode states but cannot steal ticks, so:
//
//   work finishes before the wrap    -> latch T, clear it, start the next pass
//   work has not finished at a wrap  -> the previous sample is held, and the
//                                       pass finishes into the following window
//
// which is exactly "cause the Minnie to repeat a sample on all its voices,
// essentially slipping one sample in time". The slip is uniform across voices
// because the pass is never abandoned partway, only delayed.
//
// The specification quotes a budget of 16 accesses per tick. This schedule
// leaves 34 idle states, so it tolerates 34 - one more than it needs to,
// because our thirty-state schedule is tighter than whatever GCC's was. The
// mechanism is the specified one; only the headroom differs.
//
// ---------------------------------------------------------------------------
// Stalling
// ---------------------------------------------------------------------------
// Processor accesses land on the bus phase and microcode states advance on the
// other, so they never share a clock. An access raises a flag that swallows the
// next state advance. "Bus access arbitration is made fairly simple by virtue
// of the fact that the processor always wins."

`default_nettype none

module minnie_seq (
	input  wire        clk,
	input  wire        reset,
	input  wire        ph1_en,        // advances one microcode state
	input  wire        bus_access,    // a processor read or write, on the bus phase

	output logic [1:0] voice,
	output logic [3:0] step,
	output wire        step_en,
	output wire        latch_sample,
	output wire        poly_en,
	output logic [5:0] tick
);

	localparam [3:0] LAST_STEP  = 4'd9;
	localparam [1:0] LAST_VOICE = 2'd2;

	logic busy;
	logic stall;

	// Reset gates the enables as well as the state. Without it the data path
	// would keep re-executing whatever step it is parked on - held at step 0,
	// that quietly accumulates the frequency constant into the index for the
	// whole time CREG RESET is asserted.
	wire  advance = ph1_en && !reset && !stall;
	wire  wrap    = ph1_en && !reset && (tick == 6'd63);

	assign step_en      = advance && busy;
	assign latch_sample = wrap && !busy;

	// The poly counter runs through idle states too - it is a free running
	// shift register on the real part, not something the microcode gates.
	assign poly_en      = advance;

	always_ff @(posedge clk) begin
		if (reset) begin
			tick  <= 6'd0;
			voice <= 2'd0;
			step  <= 4'd0;
			busy  <= 1'b1;
			stall <= 1'b0;
		end else begin
			if (bus_access)
				stall <= 1'b1;

			if (ph1_en) begin
				tick <= tick + 6'd1;

				if (stall) begin
					stall <= 1'b0;
				end else if (busy) begin
					if (step == LAST_STEP) begin
						step <= 4'd0;
						if (voice == LAST_VOICE) busy  <= 1'b0;
						else                     voice <= voice + 2'd1;
					end else begin
						step <= step + 4'd1;
					end
				end

				// Start the next pass only once the previous one has finished.
				// If it has not, this wrap passes without a latch and the held
				// sample repeats.
				if (tick == 6'd63 && !busy) begin
					voice <= 2'd0;
					step  <= 4'd0;
					busy  <= 1'b1;
				end
			end
		end
	end

endmodule

`default_nettype wire
