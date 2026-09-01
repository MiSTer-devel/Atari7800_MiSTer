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

// Minnie waveform storage and the "free waveform" logic of GCC-1730 section 2.3.
//
// ---------------------------------------------------------------------------
// Storage is RAM here, not the mask ROM the specification describes
// ---------------------------------------------------------------------------
// GCC chose ROM on die area, not on technology - Namco's WSG had CPU writable
// wave RAM in 1980, and GCC built their business on Pac-Man hardware. A static
// cell is five to ten times the area of a mask ROM bit, and 1024 bits of it did
// not fit under a $1.00 cost ceiling. On an M10K the difference is a write port.
//
// The memory powers up holding the same two waveforms the mask would have, so a
// program that never touches WAVEADDR/WAVEDATA behaves exactly like the
// documented chip.
//
// ---------------------------------------------------------------------------
// Waveform select, section 2.2.1.4.2
// ---------------------------------------------------------------------------
//   0  stored waveform 0        6 index bits address one of 64 samples
//   1  stored waveform 1
//   2  not used                 blank
//   3  not used                 blank
//   4  off, "outputs a -1"
//   5  sawtooth                 the 8 index MSBs, unchanged
//   6  square                   the index MSB alone
//   7  triangle                 XOR fold of the index MSBs
//
// Codes 2, 3 and 4 all give $FF. That is not a coincidence we invented: in an
// NMOS NOR ROM an unprogrammed row reads as all ones, so a blank row IS -1.
// The specification documenting code 4 as "off (outputs a -1)" is describing
// what a blank row does, and codes 2 and 3 are simply blank rows nobody named.
//
// ---------------------------------------------------------------------------
// The triangle needs an inverter the specification does not mention
// ---------------------------------------------------------------------------
// Section 2.3 says the triangle "uses the MSB to exclusive-OR the 7 next most
// significant bits, shifted up by one. A zero is shifted up into the LSB."
// Taken literally that gives a clean triangle 0..254 read as UNSIGNED, which
// read as two's complement jumps by 129 mid cycle. Inverting the output MSB is
// the standard offset binary to two's complement fix and costs one inverter:
//
//   index[15:8]   0        64       127  128       192       255
//   raw fold      $00      $80      $FE  $FE       $7E       $00
//   MSB inverted  $80      $00      $7E  $7E       $FE       $80
//   as signed     -128 ->  0   ->  +126 +126  ->   -2   ->  -128     continuous
//
// The sawtooth needs no such fix - the top 8 index bits read as two's
// complement are already a correct ramp, half a period out of phase. The
// specification saying "unchanged" for the sawtooth while describing extra
// logic for the triangle is indirect support for reading it this way.

`default_nettype none

module minnie_wave (
	input  wire       clk,

	// Voice side. idx_h is the high byte of the current voice's 16 bit index,
	// so idx_h[7:2] is index[15:10], the 6 bits that pick one of 64 samples.
	input  wire [2:0] wfm,
	input  wire [7:0] idx_h,
	output wire [7:0] sample,

	// Processor side, through WAVEADDR/WAVEDATA. One clk pulse per write.
	input  wire       cpu_wr,
	input  wire [6:0] cpu_addr,
	input  wire [7:0] cpu_data
);

	// A write steals the port for a single clk cycle. It can only land on the
	// processor bus phase, and a sample is only latched on the opposite phase,
	// so the two never contend for the same cycle.
	wire [6:0] addr = cpu_wr ? cpu_addr : {wfm[0], idx_h[7:2]};

	wire [7:0] stored;

	spram #(
		.addr_width    (7),
		.data_width    (8),
		.mem_init_file ("rtl/Minnie/minnie_wave.mif"),
		.sim_init_file ("rtl/Minnie/minnie_wave.hex"),
		.mem_name      ("MINNIEWAVE")
	) u_wave (
		.clock   (clk),
		.address (addr),
		.data    (cpu_data),
		.wren    (cpu_wr),
		.q       (stored),
		.cs      (1'b1)
	);

	// Free waveforms, derived from the index by the logic described above.
	wire [7:0] saw = idx_h;
	wire [7:0] sqr = {idx_h[7], 7'b0};

	wire [7:0] tri_raw = {idx_h[6:0] ^ {7{idx_h[7]}}, 1'b0};
	wire [7:0] tri_out = {~tri_raw[7], tri_raw[6:0]};

	logic [7:0] selected;

	always_comb begin
		case (wfm)
			3'd0, 3'd1: selected = stored;
			3'd5:       selected = saw;
			3'd6:       selected = sqr;
			3'd7:       selected = tri_out;
			default:    selected = 8'hFF;   // codes 2, 3 and 4: a blank ROM row
		endcase
	end

	assign sample = selected;

endmodule

`default_nettype wire
