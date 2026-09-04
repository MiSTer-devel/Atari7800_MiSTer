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

//============================================================================
// SALLY - Atari's 6502C, the CPU in the 5200 and the 7800.
//
// A stock 6502 plus one pin: HALT, pin 35, active low, so MARIA can take the
// bus for display list DMA.
//
// WHERE THIS CIRCUIT CAME FROM
//
// Atari built this same function out of discrete parts on the 400/800 CPU
// board. HALT circuit.png, Figure 2-12: ANTIC's /HALT goes
// through a 74LS02 pair (Z301A/B) into a 7474 (Z302A/B) clocked off the phi
// pair, whose output enables two 74LS244 octal buffers (Z303, Z304) sitting
// between the CPU's address pins and the system bus - while the CPU itself is
// stalled through its ordinary RDY pin.
//
// For the 5200 and 7800 they folded all of that onto the CPU die. Tracing
// Schematic_Atari7800_NTSC_4000.jpg shows the 7800 board has
// none of those parts: SALLY's A0-A15 and D0-D7 go straight onto the bus,
// HALT arrives from MARIA pin 40 with only a 4k7 pull-up (R63), and MARIA
// drives RDY on a separate pin. So this module is on-die logic, not board
// logic - but it is the same circuit, which is why it is worth naming after
// the parts it replaced.
//
// TIMING
//
//              cycle N      N+1        N+2        N+3        N+4       N+5
//   phi2    __|~~~|_____|~~~|_____|~~~|_____|~~~|_____|~~~|_____|~~~|_____
//   halt_n  ~~~~\_____________________________________________/~~~~~~~~~~~
//                ^ MARIA moves it ~70 ns after phi2 falls
//   sample        s          s          s          s          s    <- phi2_en
//   addr_oe ~~~~~~~~~~~~~~~~~~~~~\____________________________________/~~~
//                                 ^ the bus is MARIA's from the start of N+2
//
// Measured from capture-range.sr (16 MHz, 19424 samples):
// the CPU clock is 558.8 ns (1.7896 MHz); /HALT is low for exactly 36 samples
// = 2.250 us = 4.03 CPU cycles on all three pulses in the capture; and every
// /HALT edge lands ~70 ns after phi2 falls, giving ~315 ns of setup before
// the next phi2. Sampling on phi2_en is therefore the far side of the cycle
// from MARIA's edge.
//
// The two cycles of latency in and out are NOT measured - the capture probed
// only SYNC, BLANK, /HALT and PCLK2, so it cannot see the bus change hands.
// Two cycles is what DMA.sv and souper.v independently assume, and moving
// that edge breaks both. Treat it as fitted, not established.
//
// What is pinned is that the two are equal, so the core loses exactly the
// cycles MARIA asked for. general_tests test 15 counts a fixed loop across
// the non vblank region and wants 556; an entry of two cycles against a
// release of one reads 561, one extra loop for every five scanlines.
//
// THE TWO PATHS
//
// On the 800, ANTIC drives two separate things: RDY into the CPU's own RDY
// pin, which stalls it, and HALT into the flip-flop pair, which tristates the
// buffers. SALLY cannot copy that split: MARIA has no RDY of its own to spare
// and RDY holds a read but never a write, so a read-modify-write straddling
// the boundary writes into a bus MARIA already owns. HALT here therefore takes
// the core's phase enables away instead - see the comment on `bus_off`
// below.
//
// The consequence is the handover phase. `halt_s`/`halt_bus` move on phi2_en,
// so the bus and the core's phase enables change hands together at the start
// of phase 2, never at the cycle boundary. A cycle whose phase 1 was SALLY's
// simply stops there and finishes its phase 2 when the bus comes back, so no
// half cycle of core activity ever happens off the bus. That coupling is the
// contract.
//
// Nothing states this outright for SALLY - there is no die shot, and the
// existing logic capture has no address, R/W or output-enable channel - so the
// handover edge is fitted, not measured.
//============================================================================

module sally (
	input  logic        clk_sys,
	input  logic        phi1_en,    // start of phase 1 (this repo: pclk1)
	input  logic        phi2_en,    // start of phase 2 (this repo: pclk0)

	// ---- pins a stock 6502 also has --------------------------------------
	input  logic        res_n,      // 40
	input  logic        rdy,        // 2   MARIA pin 41 wire-ANDed with TIA pin 3.
	                                //     A bare net on the board: no gate, no delay.
	input  logic        irq_n,      // 4
	input  logic        nmi_n,      // 6   from MARIA pin 2
	input  logic        so_n,       // 38
	input  logic  [7:0] data_in,    // 26-33
	output logic  [7:0] data_out,
	output logic        data_oe,
	output logic [15:0] addr_out,   // 9-20, 22-25
	output logic        rw_n,       // 34
	output logic        sync,       // 7
	output logic        phi1_out,   // 3
	output logic        phi2_out,   // 39

	// ---- the pin a stock 6502 does not have ------------------------------
	input  logic        halt_n,     // 35  from MARIA pin 40, 4k7 pull-up R63

	// ---- what the halt circuit turns off ---------------------------------
	output logic        addr_oe,    // 0 while halted: A0-A15 released
	output logic        rw_oe,      // 0 while halted: R/W released
	output logic        is_halted,  // for the system bus mux. Always ~addr_oe,
	                                // so the two cannot disagree.
	output logic        jammed,

	// Debug only, passed straight through from the core. Nothing on the board
	// has anything like these.
	output logic  [7:0] dbg_a, dbg_x, dbg_y, dbg_s, dbg_p, dbg_ir,
	output logic [15:0] dbg_pc
);

	// The 7474 pair. Two phase-2 samples in, two out; MARIA holds halt_n low
	// for a whole number of CPU cycles, so the release is symmetric with the
	// assertion.
	logic halt_s, halt_bus;

	always_ff @(posedge clk_sys) begin
		if (!res_n) begin
			halt_s   <= 1'b0;
			halt_bus <= 1'b0;
		end else if (phi2_en) begin
			halt_s   <= ~halt_n;
			halt_bus <= halt_s;
		end
	end

	logic core_data_oe;

	// HALT gives the core two cycles of grace and then stops it dead for as
	// long as MARIA asked for. The window is the halt shifted two cycles, not
	// the halt made shorter:
	//
	//   cycle     0   1   2   3  ...  m-1   m   m+1  m+2
	//   halt_n    \___________________________/
	//   bus_off           |=======================|      halt_bus
	//   held              |=======================|      the same window
	//
	// Cycles 0 and 1 are still ours, which is what the flip-flop pair is for:
	// a write already under way reaches the bus. MARIA takes over on cycle 2,
	// which is what DMA.sv means by "ultimately it takes 2 cpu cycles to start
	// up", and drops HALT on m; the core takes the same two cycles to come
	// back, so it loses exactly as many cycles as HALT was low. Over m+1 and
	// m+2 neither side drives and the address lines hold their charge, which
	// the top level already resolves.
	//
	// The count is what fixes the release edge. MARIA halts once per non
	// vblank scanline, 242 of them a frame, so a release one cycle early hands
	// the CPU 242 cycles a frame that hardware keeps. general_tests test 15
	// counts a 46 cycle loop across the non vblank region and wants 556; an
	// early release reads 561.
	//
	// RDY is not what stops the core here, and that is the one place this
	// departs from the 800 board it copies. RDY holds a read and never a
	// write, so a read-modify-write straddling the boundary runs both of its
	// writes: Galaga lost the second write of an `INC $67` into a bus MARIA
	// already owned and hung on the counter that never advanced. Two cycles
	// of grace cannot cover it either - the netlist's ready latch takes its
	// write term from the cycle itself, so a write also un-holds the cycle
	// after it, and the exposure is three cycles deep.
	//
	// On the 800 ANTIC drove RDY and HALT as separate pins and could leave
	// whatever margin it liked. MARIA has only HALT, so SALLY has to be the
	// thing that guarantees the core is not on the bus, and the only guarantee
	// that holds for a write is not running the cycle at all. The core is
	// stopped by taking its phase enables away, which is what the T65 wrapper
	// this replaces did, and it is why nothing here has to reason about what
	// DL captured while MARIA had the bus.
	logic bus_off;
	assign bus_off   = halt_bus;

	assign addr_oe   = ~bus_off;
	assign rw_oe     = ~bus_off;
	assign data_oe   = core_data_oe & ~bus_off;
	assign is_halted = bus_off;

	logic core_phi1_en, core_phi2_en;
	assign core_phi1_en = phi1_en & ~bus_off;
	assign core_phi2_en = phi2_en & ~bus_off;

	// Pin 39 is the system clock: it feeds MARIA pin 6 and everything after
	// it, so it keeps running through a halt even though the core inside does
	// not. Tracked from the ungated enables for that reason - the core's own
	// phase outputs freeze with it.
	logic phase2;
	always_ff @(posedge clk_sys) begin
		if      (phi1_en) phase2 <= 1'b0;
		else if (phi2_en) phase2 <= 1'b1;
	end

	assign phi1_out = ~phase2;
	assign phi2_out =  phase2;

	mos6502 #(.BCD_EN(1'b1)) core (
		.clk_sys  (clk_sys),
		.phi1_en  (core_phi1_en), // stopped for the halt, see above
		.phi2_en  (core_phi2_en),
		.res_n    (res_n),
		.rdy      (rdy),          // MARIA's RDY pin only: WSYNC, not the halt
		.irq_n    (irq_n),
		.nmi_n    (nmi_n),
		.so_n     (so_n),
		.data_in  (data_in),
		.data_out (data_out),
		.data_oe  (core_data_oe),
		.addr_out (addr_out),
		.rw_n     (rw_n),
		.sync     (sync),
		.phi1_out (),                 // the pins keep running; see phase2 above
		.phi2_out (),
		.jammed   (jammed),

		.dbg_a (dbg_a), .dbg_x (dbg_x), .dbg_y(dbg_y), .dbg_s(dbg_s),
		.dbg_p (dbg_p), .dbg_ir(dbg_ir), .dbg_pc(dbg_pc),
		.dbg_t (), .dbg_hold(), .dbg_int_active(), .dbg_take_int(),
		.dbg_res_active(), .dbg_tg()
	);

endmodule
