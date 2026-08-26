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

// Minnie - GCC 1730, the in-cart sound chip GCC designed for the 7800 in 1984
// and never shipped.
//
// Sources: GCC's own Minnie specifications, section numbers throughout these
// files refer to them.
//
// ---------------------------------------------------------------------------
// This chip never existed, so parts of it are invented
// ---------------------------------------------------------------------------
// The specifications leave the microcode ("FIX ME!!"), half of CREG, the base
// address ("XXXXX"), the waveform contents and the poly taps blank. Everything
// filled in here is flagged where it appears. There is no hardware oracle for
// any of it, so nothing in this file should ever be described as accurate -
// only as consistent with the specification.
//
// Two deliberate departures from the documented part: the waveform memory is
// writable rather than mask ROM (see minnie_wave.sv), and the noise added to
// the index is signed rather than unsigned (see minnie_datapath.sv).
//
// ---------------------------------------------------------------------------
// Pin contract
// ---------------------------------------------------------------------------
// The 1984 part is a 24 pin DIP: a clock, eight data lines, eleven address
// lines, R/W, an analog audio pin and the two supplies. It decodes its own
// address from A15..A10 and A4..A0, with A9..A5 unbonded, so its 32 registers
// would have aliased 32 times across a 1 kB window.
//
// Here the region decode is the cartridge's job and this takes a plain chip
// select, which is why the window can move without touching the chip. The
// bidirectional data bus is split into separate value and enable ports,
// because an FPGA boundary has no `z`.
//
// Clocking: the real part has one clock pin, PCK2, and makes two phases from
// it. This takes both phase enables instead, like pokey.sv.
// ph1_en advances the microcode; ph2_en is when the processor bus is valid.
// One microcode state per processor clock, so 64 states per sample - not one
// per phase, which would halve the sample rate.
//
//              ph1_en          ph2_en          ph1_en
//   state ------->|<-- work -->|<-- bus  -->|<-- work ...
//
// AUD is analog on the real part. Both forms are exposed: the raw 10 bit
// two's complement sample with its strobe, and a filtered unsigned value for
// the framework mixer. The host picks.

`default_nettype none

module minnie (
	input  wire        clk,

	// The two phases of the target 1.79 MHz clock, one clk cycle each.
	input  wire        ph1_en,
	input  wire        ph2_en,
	input  wire        reset,

	// Bus pins. The region decode happens outside; a is A4..A0.
	input  wire  [4:0] a,
	input  wire        cs,
	input  wire        rw,          // high = read
	input  wire  [7:0] d_in,
	output wire  [7:0] d_out,
	output wire        d_oe,

	// Audio. The DAC side, and the mixed value for the framework.
	output wire  [9:0] sample,
	output wire        sample_en,
	output wire [15:0] aud
);

	// Accesses land on the bus phase only. "The processor always wins": either
	// direction stalls the microcode for one state.
	wire bus_wr     = cs && !rw && ph2_en;
	wire bus_rd     = cs &&  rw && ph2_en;
	wire bus_access = bus_wr || bus_rd;

	assign d_oe = cs && rw;

	// CREG, section 2.2.2.1. Four bits are named on the sheet and four are
	// blank and hand-circled "Rethink". RESET and POLYRST are implemented
	// because they have an audible effect; TESTROM and FREERUN serve a wafer
	// tester that does not exist here and are accepted but do nothing, as are
	// the four GCC never named.
	//
	//   bit 7  6      5      4       3       2       1  0
	//          -      RESET  POLYRST TESTROM FREERUN -  -
	/* verilator lint_off UNUSEDSIGNAL */
	wire [7:0] creg;             // bits 7, 6, 3, 2, 1, 0 are accepted and ignored
	/* verilator lint_on UNUSEDSIGNAL */
	wire       creg_reset   = creg[5];
	wire       creg_polyrst = creg[4];

	wire synth_rst = reset || creg_reset;

	wire  [1:0] voice;
	wire  [3:0] step;
	wire        step_en, latch_sample, poly_en;
	/* verilator lint_off UNUSEDSIGNAL */
	wire  [5:0] tick;            // sample-tick position, for testbenches only
	/* verilator lint_on UNUSEDSIGNAL */

	minnie_seq u_seq (
		.clk,
		.reset       (synth_rst),
		.ph1_en,
		.bus_access,
		.voice,
		.step,
		.step_en,
		.latch_sample,
		.poly_en,
		.tick
	);

	wire [14:0] poly;

	minnie_poly u_poly (
		.clk,
		.step_en (poly_en),
		.rst     (reset || creg_polyrst),
		.poly
	);

	wire [7:0] freq_l, freq_h, vol, timbre, idx_l, idx_h;
	wire       idx_wr, idx_hi;
	wire [7:0] idx_data;
	wire [9:0] t_value;
	wire       t_wr;
	wire [6:0] wave_ptr;
	wire       wave_wr;
	wire [7:0] wave_data;

	minnie_regs u_regs (
		.clk,
		.reset,
		.bus_wr,
		.bus_rd,
		.bus_addr (a),
		.bus_din  (d_in),
		.bus_dout (d_out),
		.voice,
		.freq_l,
		.freq_h,
		.vol,
		.timbre,
		.idx_l,
		.idx_h,
		.idx_wr,
		.idx_hi,
		.idx_data,
		.t_value,
		.t_wr,
		.creg,
		.wave_ptr,
		.wave_wr,
		.wave_data
	);

	wire [2:0] wfm;
	wire [7:0] wave_sample;

	minnie_wave u_wave (
		.clk,
		.wfm,
		.idx_h,
		.sample   (wave_sample),
		.cpu_wr   (wave_wr),
		.cpu_addr (wave_ptr),
		.cpu_data (wave_data)
	);

	minnie_datapath u_dp (
		.clk,
		.rst     (synth_rst),
		.step_en,
		.step,
		.latch_sample,
		.freq_l,
		.freq_h,
		.vol,
		.timbre,
		.idx_l,
		.idx_h,
		.idx_wr,
		.idx_hi,
		.idx_data,
		.wfm,
		.wave_sample,
		.poly,
		.t_wr,
		.t_din   (d_in),
		.t_value,
		.sample_out (sample),
		.sample_en
	);

	minnie_out u_out (
		.clk,
		.reset,
		.ph1_en,
		.sample_en,
		.sample  ($signed(sample)),
		.aud
	);

endmodule

`default_nettype wire
