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

// Minnie's register array, GCC-1730 section 2.2, and the processor read path.
//
// ---------------------------------------------------------------------------
// The map
// ---------------------------------------------------------------------------
// 32 addresses, four blocks of eight. Only 20 bytes are real storage: each
// voice block holds six, offset 6 is unused, and all three "T" entries alias
// the one accumulator that lives in the datapath.
//
//   +0 FREQnL   +1 FREQnH   +2 VOLn   +3 TIMBREn
//   +4 INDEXnL  +5 INDEXnH  +6 -      +7 T (shared, cleared every sample)
//
//   $18 CREG      $19 WAVEADDR    $1A WAVEDATA    $1B -
//   $1C STROBE    $1D -           $1E -           $1F -
//
// WAVEADDR and WAVEDATA are ours, not GCC's - they are the upload port for the
// writable waveform memory, in two of the six Chip Control Block slots the
// specification left unused. Everything GCC defined keeps its address.
//
// ---------------------------------------------------------------------------
// Reads take two accesses
// ---------------------------------------------------------------------------
// Section 2.4.1: "in order to read a register, it is necessary to do a
// preliminary read of the location, followed by the real read... any data that
// shows up during this preliminary read is erroneous (it consists of the last
// byte of data read)". So a read moves the register into a pad latch and the
// bus sees whatever the latch held from the previous read. Reads exist for a
// wafer tester; the specification calls using them NOT RECOMMENDED and
// read-modify-writes RIGHT OUT.

`default_nettype none

module minnie_regs (
	input  wire        clk,
	input  wire        reset,

	// Processor side. bus_wr and bus_rd are one clk pulses on the bus phase.
	input  wire        bus_wr,
	input  wire        bus_rd,
	input  wire  [4:0] bus_addr,
	input  wire  [7:0] bus_din,
	output logic [7:0] bus_dout,

	// The voice the microcode is currently working on.
	input  wire  [1:0] voice,
	output wire  [7:0] freq_l,
	output wire  [7:0] freq_h,
	output wire  [7:0] vol,
	output wire  [7:0] timbre,
	output wire  [7:0] idx_l,
	output wire  [7:0] idx_h,

	// Index writeback from the datapath: the oscillator and noise adds land here.
	input  wire        idx_wr,
	input  wire        idx_hi,
	input  wire  [7:0] idx_data,

	// T lives in the datapath. These carry bus access to it. Only the low byte
	// is reachable through an 8 bit bus.
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire  [9:0] t_value,
	/* verilator lint_on UNUSEDSIGNAL */
	output wire        t_wr,

	output logic [7:0] creg,

	// Waveform upload port.
	output logic [6:0] wave_ptr,
	output wire        wave_wr,
	output wire  [7:0] wave_data
);

	logic [7:0] freq_lo [3];
	logic [7:0] freq_hi [3];
	logic [7:0] vol_r   [3];
	logic [7:0] timbre_r[3];
	logic [7:0] index_lo[3];
	logic [7:0] index_hi[3];

	// Address decode. The top two bits pick the block: 0-2 are voices, 3 is the
	// Chip Control Block.
	wire [1:0] blk      = bus_addr[4:3];
	wire [2:0] off      = bus_addr[2:0];
	wire       is_voice = (blk != 2'd3);

	assign t_wr      = bus_wr && is_voice && (off == 3'd7);
	assign wave_wr   = bus_wr && !is_voice && (off == 3'd2);
	assign wave_data = bus_din;

	assign freq_l = freq_lo [voice];
	assign freq_h = freq_hi [voice];
	assign vol    = vol_r   [voice];
	assign timbre = timbre_r[voice];
	assign idx_l  = index_lo[voice];
	assign idx_h  = index_hi[voice];

	// Read mux. Unused and write-only locations read back as zero; the pad latch
	// below is what actually reaches the bus, one access later.
	logic [7:0] read_val;

	always_comb begin
		read_val = 8'h00;
		if (is_voice) begin
			case (off)
				3'd0:    read_val = freq_lo [blk];
				3'd1:    read_val = freq_hi [blk];
				3'd2:    read_val = vol_r   [blk];
				3'd3:    read_val = timbre_r[blk];
				3'd4:    read_val = index_lo[blk];
				3'd5:    read_val = index_hi[blk];
				3'd7:    read_val = t_value[7:0];
				default: read_val = 8'h00;
			endcase
		end else begin
			case (off)
				3'd0:    read_val = creg;
				3'd1:    read_val = {1'b0, wave_ptr};
				default: read_val = 8'h00;
			endcase
		end
	end

	always_ff @(posedge clk) begin
		if (reset) begin
			for (int i = 0; i < 3; i++) begin
				freq_lo [i] <= 8'h00;
				freq_hi [i] <= 8'h00;
				vol_r   [i] <= 8'h00;
				// Waveform 4 is "off". Powering up there keeps the chip silent
				// until a program actually asks for a voice.
				timbre_r[i] <= 8'h04;
				index_lo[i] <= 8'h00;
				index_hi[i] <= 8'h00;
			end
			creg     <= 8'h00;
			wave_ptr <= 7'h00;
			bus_dout <= 8'h00;
		end else begin
			// The microcode's index writeback. A processor write in the same
			// sample tick wins, because the write stalls the microcode for that
			// state - the two never land on the same clock.
			if (idx_wr) begin
				if (idx_hi) index_hi[voice] <= idx_data;
				else        index_lo[voice] <= idx_data;
			end

			if (bus_wr) begin
				if (is_voice) begin
					case (off)
						3'd0: freq_lo [blk] <= bus_din;
						3'd1: freq_hi [blk] <= bus_din;
						3'd2: vol_r   [blk] <= bus_din;
						3'd3: timbre_r[blk] <= bus_din;
						3'd4: index_lo[blk] <= bus_din;
						3'd5: index_hi[blk] <= bus_din;
						default: ;   // offset 6 unused, offset 7 is T
					endcase
				end else begin
					case (off)
						3'd0: creg     <= bus_din;
						3'd1: wave_ptr <= bus_din[6:0];
						3'd2: wave_ptr <= wave_ptr + 7'd1;   // WAVEDATA auto-increments
						default: ;   // STROBE and the four spare slots do nothing
					endcase
				end
			end

			if (bus_rd)
				bus_dout <= read_val;
		end
	end

endmodule

`default_nettype wire
