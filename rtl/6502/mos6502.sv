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
// NMOS 6502, 40-pin part, cycle accurate.
//
// Real pins only. Nothing Atari-specific lives here: the 7800's HALT is in
// sally.sv, and the NES 2A03's DMA stall arrives on `rdy` like any other RDY
// source. With BCD_EN=0 this is the 2A03/2A07 core.
//
// phi0 (pin 37) is replaced by the clk_sys + phi1_en/phi2_en trio, which is
// the one documented departure from the pin contract. Everything clocked runs
// off clk_sys and is gated by one of the two enables, so clk_sys can be as
// fast as the system needs without changing target timing.
//============================================================================

module mos6502
	import mos6502_pkg::*;
#(
	// 1 = NMOS 6502 / Sally decimal mode.
	// 0 = NES 2A03/2A07: ADC and SBC ignore the D flag. The flag itself still
	//     sets, clears, pushes and pulls normally.
	parameter bit BCD_EN = 1'b1
) (
	// ---- fabric clocking. Not chip pins; these replace phi0 -------------
	input  logic        clk_sys,   // free running, at least 2x the target phi0
	input  logic        phi1_en,   // one clk_sys pulse at the start of phase 1
	input  logic        phi2_en,   // one clk_sys pulse at the start of phase 2

	// ---- inputs ----------------------------------------------------------
	input  logic        res_n,     // 40  RES. Active low, level.
	input  logic        rdy,       // 2   RDY. Low stalls a read cycle, never a write.
	input  logic        irq_n,     // 4   IRQ. Active low, level, sampled in phase 2.
	input  logic        nmi_n,     // 6   NMI. Active low, negative edge, phase 2.
	input  logic        so_n,      // 38  S.O. Negative edge sets V. Tie high if unused.

	// ---- the data bus is the only thing on this part that floats ---------
	input  logic  [7:0] data_in,   // 26-33 read data, captured at the end of phase 2
	output logic  [7:0] data_out,  // 26-33 write data, held for the whole write cycle
	output logic        data_oe,   // 1 while driving. DBE is tied to phi2 on the die,
	                               // so this is (write cycle && phase 2).

	// ---- always-driven outputs. A stock 6502 never floats these ----------
	output logic [15:0] addr_out,  // 9-20, 22-25. A0-A15.
	output logic        rw_n,      // 34  R/W. 1 = read.
	output logic        sync,      // 7   SYNC. High for the whole opcode fetch cycle.
	output logic        phi1_out,  // 3   phase 1 level, for support chips
	output logic        phi2_out,  // 39  phase 2 level, for support chips

	// ---- not pins --------------------------------------------------------
	output logic        jammed,    // a KIL opcode has locked the part up

	// Architectural state, exported for verification and debug. Nothing inside
	// the core reads these back, so they cost nothing when left unconnected.
	output logic  [7:0] dbg_a, dbg_x, dbg_y, dbg_s, dbg_p, dbg_ir,
	output logic [15:0] dbg_pc,
	output logic  [3:0] dbg_t,
	output logic        dbg_hold, dbg_int_active, dbg_take_int, dbg_res_active,
	output logic [15:0] dbg_tg
);

	ctl_t       c;
	logic       sync_ctl;
	logic [7:0] ir;
	logic [7:0] a, x, y, s, p, dl, pcl, pch;
	logic       acr_now;

	mos6502_ctl #(.BCD_EN(BCD_EN)) ctl (
		.clk_sys(clk_sys), .phi1_en(phi1_en), .phi2_en(phi2_en),
		.res_n(res_n), .rdy(rdy), .irq_n(irq_n), .nmi_n(nmi_n), .so_n(so_n),
		.data_in(data_in), .p(p), .acr_now(acr_now), .dl(dl),
		.c(c), .ir(ir), .sync(sync_ctl), .jammed(jammed),
		.dbg_t(dbg_t), .dbg_hold(dbg_hold),
		.dbg_int_active(dbg_int_active), .dbg_take_int(dbg_take_int),
		.dbg_res_active(dbg_res_active), .dbg_tg(dbg_tg)
	);

	mos6502_dp #(.BCD_EN(BCD_EN)) dp (
		.clk_sys(clk_sys), .phi1_en(phi1_en), .phi2_en(phi2_en),
		.c(c), .ir(ir), .data_in(data_in),
		.a(a), .x(x), .y(y), .s(s), .p(p), .dl(dl),
		.addr_out(addr_out), .data_out(data_out),
		.acr_now(acr_now), .pcl(pcl), .pch(pch)
	);

	assign dbg_a  = a;   assign dbg_x = x;  assign dbg_y = y;
	assign dbg_s  = s;   assign dbg_p = p;  assign dbg_ir = ir;
	assign dbg_pc = {pch, pcl};

	// Which phase we are in, tracked from the enables so the level outputs
	// look like the real pins to anything downstream.
	logic phase2;
	always_ff @(posedge clk_sys) begin
		if      (phi1_en) phase2 <= 1'b0;
		else if (phi2_en) phase2 <= 1'b1;
	end

	assign phi1_out = ~phase2;
	assign phi2_out =  phase2;

	// R/W, SYNC and the data drive describe the cycle whose address is on the
	// pins, and they hold for the whole of it - the netlist shows all three
	// identical in both phases of every cycle, read-to-write transitions
	// included. The control word advances half a cycle earlier, at phi2_en,
	// so the three are retimed here to land with the address, which ABL/ABH
	// load at phi1_en for the same reason.
	//
	// RES needs no special case here: the control block already keeps SYNC down
	// and the control word quiet while a reset is recognised.
	logic wr_pin, sync_pin;
	always_ff @(posedge clk_sys) begin
		if (phi1_en) begin
			wr_pin   <= c.wr;
			sync_pin <= sync_ctl;
		end
	end

	assign rw_n = ~wr_pin;
	assign sync = sync_pin;

	// DBE is tied to phi2 on the die, so the part drives the data bus only in
	// the second half of a write cycle.
	assign data_oe = wr_pin & phase2;

endmodule
