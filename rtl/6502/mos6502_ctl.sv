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
// MOS 6502 control: predecode, the timing generator, and the per-cycle
// datapath steering for all 256 opcodes.
//
// THE ONE TIMING FACT THIS IS BUILT AROUND
//
// The random control logic latches its outputs on phase 2 and applies them on
// the following phase 1. So the
// control word driving cycle N was decided during phase 2 of cycle N-1. That
// is not a detail - it is what lets an instruction finish after IR has
// already been overwritten by the next opcode, which is how a 3-cycle
// LDA/ORA works at all. Everything here follows from it:
//
//   phase 1 of N : the latched control word is applied. Registers load, the
//                  address pins change.
//   phase 2 of N : memory data arrives. DL, ADD, PCL and PCH capture. The
//                  control word for cycle N+1 is computed and latched.
//
// T-STATES
//
// Cycle numbers here match the "#" column of the usual addressing-mode cycle
// tables. In the silicon's own naming, t==1 is T1
// (SYNC, and also T+ of the instruction just finishing) and the last cycle of
// an instruction is T0.
//
// WHY IR AND THE INCOMING BYTE ARE BOTH USED
//
// At phase 2 of the fetch cycle we are deciding the control for cycle 2 while
// IR still holds the *previous* opcode - which is exactly what we want, since
// that instruction's writeback happens on cycle 2's phase 1. The new opcode
// is on the data pins at that moment, so cycle 2's own sequencing comes from
// predecode reading those pins directly. That is what predecode is for on the
// real part, and it is why it can shorten a two-cycle instruction before the
// opcode has reached IR.
//============================================================================

module mos6502_ctl
	import mos6502_pkg::*;
#(
	parameter bit BCD_EN = 1'b1
) (
	input  logic       clk_sys,
	input  logic       phi1_en,    // closes the interrupt and reset recognition
	                               // latches - see the chain below
	input  logic       phi2_en,    // where the control word is built

	input  logic       res_n,
	input  logic       rdy,
	input  logic       irq_n,
	input  logic       nmi_n,
	input  logic       so_n,

	input  logic [7:0] data_in,
	input  logic [7:0] p,          // flags, for branches and for D and I
	input  logic       acr_now,    // the same carry, before the latch
	input  logic [7:0] dl,

	output ctl_t       c,
	output logic [7:0] ir,
	output logic       sync,
	output logic       jammed,

	// Sequencer state, for verification and debug only.
	output logic [3:0] dbg_t,
	output logic       dbg_hold,
	output logic       dbg_int_active,
	output logic       dbg_res_active,
	// The timing generator, exposed to diff against the netlist's own nodes.
	// Bits 0-5 T0/T1X/T2..T5, 6-7 the two extension latches, then
	// t_zero, end_x, res_p and the prefetch gate - the terms that decide them.
	output logic [15:0] dbg_tg,
	output logic       dbg_take_int
);

	// ---- state -------------------------------------------------------------
	/* verilator lint_off PROCASSINIT */
	logic [3:0] t = 4'd1;        // cycle within the instruction, 1 based
	/* verilator lint_on PROCASSINIT */
	ctl_t       c_reg;
	logic [3:0] nt;
	ctl_t       nc;

	decode_t    d;               // decode of IR, valid from cycle 2 onward
	mos6502_decode decode (.ir(ir), .d(d));

	// Interrupt and reset sequencing.
	logic int_g, res_g, int_armed;
	// RESG and INTG, the netlist's two "a break is in progress" nodes. RESG
	// alone suppresses the writes and picks the RES vector; either one forces
	// BRK into IR and pushes B clear.
	logic int_active, res_active;
	assign res_active = res_g;
	assign int_active = int_g | res_g;
	// Which vector the sequence fetches is not remembered anywhere. The die
	// reads the NMI request live as the address is formed - `pre_zero_adl[2]`
	// is a gate on `nmi_g`, not a latch - so a request arriving any time up to
	// the phase 2 that forms the vector address steals it, and a BRK with
	// nothing pending always gets $FFFE however the last sequence ended.
	// Declared up here because addr_vector below reads it; it is driven from
	// `nmig` beside the rest of the interrupt logic.
	logic nmi_pending;
	// RESG, but rising as soon as the pin is recognised. Two things read it at
	// the cycle rather than through the control word built a cycle earlier: the
	// vector select, because `zero_adl` is latched in the phase 2 that forms the
	// next address and RESG is already up there; and the write suppression,
	// which on the die is `mem_write_rdy = ... & ~res_g`. Reading only the
	// register makes both a cycle late. The fall stays on the register, because
	// the vector high fetch's address is latched while RESG is still up.
	logic res_now;
	assign res_now = res_g | res_c2;
	logic so_last;

	// SHA, SHX, SHY and TAS: the five stores whose value is ANDed with the
	// address high byte + 1, and whose address high byte becomes that value
	// when the index carried.
	logic sh_store;
	assign sh_store = (d.store == ST_SHA) || (d.store == ST_SHX) ||
	                  (d.store == ST_SHY) || (d.store == ST_TAS);

	// Branch and page-pgx bookkeeping.
	logic pgx;                 // the index add carried; a fix-up cycle is due
	logic idx_cross_q;         // the same carry, kept past the fix-up cycle
	logic br_take;               // the flag test says take it
	logic br_back;               // the offset was negative, so PCH goes down
	logic br_fix;                // PCH has to move as well, this cycle

	// Where the next opcode fetch takes its address from. Normally PC, but a
	// jump, a return and a taken branch all put the target straight onto the
	// address bus out of the ALU, and PC follows from there.
	typedef enum logic [1:0] { PC_PC, PC_EA, PC_ADD, PC_FIX } pcsrc_e;
	pcsrc_e n_pc_src;

	// SYNC is held through a stall: hold_sync = ~rdy & t_res_1_c1 in
	// klynch71's random_control_logic.v, so a held cycle keeps the SYNC the
	// cycle before it had.
	// RES suppresses it, because it is `pre_fetch_rdy` that carries `~res_p_c2`
	// on the die - and only that path, which is why a taken branch's short
	// circuit still fetches. The timing generator reads this, not the pin, so
	// the gate has to live here rather than on the output retiming.
	assign sync = (hold ? sync_q : (t == 4'd1)) & ~res_no_prefetch;
	// c is driven below, next to hold_mask, which it needs.
	assign jammed = (d.mode == M_JAM) && (t >= 4'd2);

	assign dbg_t = t;
	assign dbg_hold = hold;
	assign dbg_int_active = int_active;
	assign dbg_res_active = res_active;
	assign dbg_tg = {4'd0, res_no_prefetch, res_p, end_x_q, t_zero, ext_t6, ext_t5, tg};
	assign dbg_take_int = take_int;

	// ---- predecode ---------------------------------------------------------
	// Reads the opcode straight off the pins during the fetch cycle's phase 2.
	// The die's predecode has two jobs: suppress the PC
	// increment on a one-byte instruction's second fetch, and shorten a
	// two-cycle instruction. Only the first is needed here.
	//
	// The second is not, because of where this core makes the decision. The
	// die shortens the instruction during the fetch cycle, before the opcode
	// has reached IR, so it has to read the pins. Here the "is this the last
	// cycle" test is evaluated at the end of cycle 2, by which time IR holds
	// the opcode and `d.mode` answers the same question directly. Keeping a
	// second, bit-pattern copy of that answer would be a thing to get out of
	// step with the decode table for no gain.
	logic pd_implied;

	assign pd_implied = data_in[3] & ~data_in[2] & ~data_in[0];

	// Two-cycle opcodes: xxx010x1, 1xx000x0, and the implied ones except
	// 0xx01000. The timing generator needs this to raise T0 without t_zero.
	// predecode_logic.v.
	logic pd_two_cycle;
	always_comb begin
		logic imp, m_xxx010x1, m_1xx000x0, m_0xx01000;
		imp        =  ir[3] & ~ir[2] & ~ir[0];
		m_xxx010x1 = ~ir[4] &  ir[3] & ~ir[2] &  ir[0];
		m_1xx000x0 =  ir[7] & ~ir[4] & ~ir[3] & ~ir[2] & ~ir[0];
		m_0xx01000 = ~ir[7] & ~ir[4] &  ir[3] & ~ir[2] & ~ir[1] & ~ir[0];
		pd_two_cycle = m_xxx010x1 | m_1xx000x0 | (imp & ~m_0xx01000);
	end

	// ---- address steering helpers -----------------------------------------
	// Each returns the part of the control word that puts one address on the
	// pins. They are OR'd with the operation word, the way the random control
	// logic ORs its own terms.

	function automatic ctl_t addr_pc();          // address = PC
		ctl_t r = CTL_IDLE;
		r.pcl_adl = 1'b1; r.pch_adh = 1'b1;
		r.adl_abl = 1'b1; r.adh_abh = 1'b1;
		return r;
	endfunction

	function automatic ctl_t addr_zp();          // address = $00,DL
		ctl_t r = CTL_IDLE;
		r.dl_adl = 1'b1;
		r.zero_adh0 = 1'b1; r.zero_adh17 = 1'b1;
		r.adl_abl = 1'b1; r.adh_abh = 1'b1;
		return r;
	endfunction

	function automatic ctl_t addr_zp_add();      // address = $00,ADD  (indexed zp)
		ctl_t r = CTL_IDLE;
		r.add_adl = 1'b1;
		r.zero_adh0 = 1'b1; r.zero_adh17 = 1'b1;
		r.adl_abl = 1'b1; r.adh_abh = 1'b1;
		return r;
	endfunction

	function automatic ctl_t addr_stack();       // address = $01,S
		ctl_t r = CTL_IDLE;
		r.s_adl = 1'b1;
		r.zero_adh17 = 1'b1;      // ADH0 left precharged, so the page is $01
		r.adl_abl = 1'b1; r.adh_abh = 1'b1;
		return r;
	endfunction

	function automatic ctl_t addr_dl_add();      // address = {DL, ADD}
		ctl_t r = CTL_IDLE;
		r.add_adl = 1'b1; r.dl_adh = 1'b1;
		r.adl_abl = 1'b1; r.adh_abh = 1'b1;
		return r;
	endfunction

	function automatic ctl_t addr_fix_hi();      // keep ABL, ABH <= ADD
		ctl_t r = CTL_IDLE;
		r.add_sb06 = 1'b1; r.add_sb7 = 1'b1;
		r.sb_adh = 1'b1; r.adh_abh = 1'b1;
		return r;                  // adl_abl deliberately not asserted
	endfunction

	function automatic ctl_t addr_add_pch();   // address = {PCH, ADD}
		ctl_t r = CTL_IDLE;
		r.add_adl = 1'b1; r.pch_adh = 1'b1;
		r.adl_abl = 1'b1; r.adh_abh = 1'b1;
		return r;
	endfunction

	// The vector address is not held anywhere. ADH is left entirely
	// precharged, which is the $FF, and three ADL bits are pulled low to pick
	// which vector.
	//
	//   NMI  $FFFA / $FFFB   bit 2 low,          bit 0 low on the first
	//   RES  $FFFC / $FFFD   bit 1 low,          bit 0 low on the first
	//   IRQ  $FFFE / $FFFF   nothing but bit 0
	//
	// RES outranks a pending NMI: the die's bit 2 gate is
	// `~(nmi_g | vec_n | res_g)` with `nmi_g` active low, so RESG holds bit 2
	// high and the pair comes out $FFFC rather than $FFF8.
	// klynch71 interrupt_and_reset_control.v `pre_zero_adl`.
	function automatic ctl_t addr_vector(input logic hi);
		ctl_t r = CTL_IDLE;
		r.zero_adl0 = ~hi;
		r.zero_adl1 = res_now;
		r.zero_adl2 = nmi_pending & ~res_now;
		r.adl_abl = 1'b1; r.adh_abh = 1'b1;
		return r;
	endfunction

	// Load the ALU with (index + DL) so the next cycle can address with it.
	function automatic ctl_t alu_index(input logic use_x);
		ctl_t r = CTL_IDLE;
		if (use_x) r.x_sb = 1'b1; else r.y_sb = 1'b1;
		r.sb_add = 1'b1;                 // AI <= index
		r.dl_db  = 1'b1; r.db_add = 1'b1; // BI <= DL
		r.sums   = 1'b1;
		return r;
	endfunction

	// Load the ALU with DL + carry, to fix an address high byte.
	function automatic ctl_t alu_fix(input logic carry);
		ctl_t r = CTL_IDLE;
		r.zero_add = 1'b1;                // AI <= 0
		r.dl_db = 1'b1; r.db_add = 1'b1;  // BI <= DL
		r.sums = 1'b1;
		r.alucin = carry;
		return r;
	endfunction

	// ---- the operation of the instruction that is finishing ----------------
	// It runs across two cycles because ADD is a register: cycle A loads the
	// ALU, cycle B moves ADD into a register and sets the flags. A load needs
	// no ALU, so it finishes in cycle A alone. Cycle A is the next opcode
	// fetch (T+) and cycle B the one after, by which time IR has already been
	// overwritten - which is exactly why the control word is latched.

	function automatic ctl_t op_a(input decode_t o);
		ctl_t r = CTL_IDLE;
		logic combo;
		// SLO, RLA, SRE, RRA, ISC, DCP: the write-back cycle already loaded
		// BI with the modified byte, so the data half only has to bring the
		// accumulator in on the A side.
		combo = (o.rmw != RMW_NONE) && (o.data != D_NONE) && (o.mode != M_ACC);

		if (combo) begin
			unique case (o.data)
			D_ORA: begin r.ac_sb=1; r.sb_add=1; r.ors =1; end
			D_AND: begin r.ac_sb=1; r.sb_add=1; r.ands=1; end
			D_EOR: begin r.ac_sb=1; r.sb_add=1; r.eors=1; end
			D_ADC: begin r.ac_sb=1; r.sb_add=1; r.sums=1; r.alucin=p[0];
			             r.daa=BCD_EN & p[3]; end
			D_SBC: begin r.ac_sb=1; r.sb_add=1; r.sums=1; r.alucin=p[0];
			             r.dsa=BCD_EN & p[3]; end
			D_CMP: begin r.ac_sb=1; r.sb_add=1; r.sums=1; r.alucin=1; end
			default: ;
			endcase
			return r;
		end

		unique case (o.data)
		D_LDA: begin r.dl_db=1; r.sb_db=1; r.sb_ac=1; r.db7_n=1; r.dbz_z=1; end
		D_LDX: begin r.dl_db=1; r.sb_db=1; r.sb_x =1; r.db7_n=1; r.dbz_z=1; end
		D_LDY: begin r.dl_db=1; r.sb_db=1; r.sb_y =1; r.db7_n=1; r.dbz_z=1; end
		D_LAX: begin r.dl_db=1; r.sb_db=1; r.sb_ac=1; r.sb_x=1;
		             r.db7_n=1; r.dbz_z=1; end
		D_ORA: begin r.ac_sb=1; r.sb_add=1; r.dl_db=1; r.db_add=1; r.ors =1; end
		D_AND: begin r.ac_sb=1; r.sb_add=1; r.dl_db=1; r.db_add=1; r.ands=1; end
		// ANC, ALR and ARR take the whole operand, like AND does. The
		// netlist puts only DB bit 0 on the bus for these three and leaves
		// A alone; E X O depends on the community behaviour - `ANC #$07` at
		// $FA4A masks a display list's graphics page - and it runs on real
		// hardware, so Sally does the full AND. Decision 0071 supersedes 0010
		// for this case.
		D_ANC: begin r.ac_sb=1; r.sb_add=1; r.dl_db=1; r.db_add=1; r.ands=1; end
		D_ALR: begin r.ac_sb=1; r.sb_add=1; r.dl_db=1; r.db_add=1; r.srs=1; end
		D_ARR: begin r.ac_sb=1; r.sb_add=1; r.dl_db=1; r.db_add=1; r.srs=1;
		             r.arr_d = BCD_EN & p[3]; end
		// LAS also transfers S to X, off SB on this cycle - before ADD holds
		// the AND result. It does not write S.
		//
		// The page-crossing form is not modelled: the extra fix-up cycle
		// leaves the operand only partly on the bus, so A comes out as S
		// ANDed with something that is neither of the two bytes read.
		D_LAS: begin r.s_sb=1;  r.sb_add=1; r.sb_x=1;
		             r.dl_db=1; r.db_add=1; r.ands=1; end
		// ANE and LXA OR a "magic constant" into A before the AND. On the
		// visual6502 die that constant is $00, so the OR drops out. Other
		// parts are reported to use $EE or $FF, which is why these two are
		// called unstable.
		// ANE puts A and X on SB together and lets the bus wire-AND them,
		// the same trick SAX uses.
		D_ANE: begin r.ac_sb=1; r.x_sb=1; r.sb_add=1;
		             r.dl_db=1; r.db_add=1; r.ands=1; end
		D_LXA: begin r.ac_sb=1; r.sb_add=1; r.dl_db=1; r.db_add=1; r.ands=1; end
		D_EOR: begin r.ac_sb=1; r.sb_add=1; r.dl_db=1; r.db_add=1; r.eors=1; end
		D_ADC: begin r.ac_sb=1; r.sb_add=1; r.dl_db=1; r.db_add=1;
		             r.sums=1; r.alucin=p[0]; r.daa=BCD_EN & p[3]; end
		D_SBC: begin r.ac_sb=1; r.sb_add=1; r.dl_db=1; r.ndb_add=1;
		             r.sums=1; r.alucin=p[0]; r.dsa=BCD_EN & p[3]; end
		D_CMP: begin r.ac_sb=1; r.sb_add=1; r.dl_db=1; r.ndb_add=1;
		             r.sums=1; r.alucin=1; end
		D_CPX: begin r.x_sb =1; r.sb_add=1; r.dl_db=1; r.ndb_add=1;
		             r.sums=1; r.alucin=1; end
		D_CPY: begin r.y_sb =1; r.sb_add=1; r.dl_db=1; r.ndb_add=1;
		             r.sums=1; r.alucin=1; end
		// BIT takes N and V straight off the operand and only Z from the AND.
		D_BIT: begin r.ac_sb=1; r.sb_add=1; r.dl_db=1; r.db_add=1; r.ands=1;
		             r.db7_n=1; r.db6_v=1; end
		D_SBX: begin r.ac_sb=1; r.x_sb=1; r.sb_add=1; r.dl_db=1; r.ndb_add=1;
		             r.sums=1; r.alucin=1; end
		default: ;
		endcase

		// Accumulator-mode shifts. SRS is the only shift the ALU has, so ASL
		// and ROL are A+A with both sides of the adder holding A.
		if (o.mode == M_ACC) begin
			r.ac_sb = 1; r.sb_add = 1; r.ac_db = 1; r.db_add = 1;
			unique case (o.rmw)
			RMW_ASL: begin r.sums = 1; end
			RMW_ROL: begin r.sums = 1; r.alucin = p[0]; end
			RMW_LSR: begin r.srs  = 1; end
			RMW_ROR: begin r.srs  = 1; end
			default: ;
			endcase
		end

		if (o.mode == M_PULL) begin
			r.dl_db = 1;
			if (ir[6]) begin                     // PLA
				r.sb_db = 1; r.sb_ac = 1; r.db7_n = 1; r.dbz_z = 1;
			end else begin                       // PLP: B and bit 5 are dropped
				r.db_p = 1; r.db0_c = 1; r.db6_v = 1; r.db7_n = 1;
			end
		end

		if (o.mode == M_IMP) begin
			unique case (o.imp)
			I_TAX: begin r.ac_sb=1; r.sb_db=1; r.sb_x =1; r.db7_n=1; r.dbz_z=1; end
			I_TAY: begin r.ac_sb=1; r.sb_db=1; r.sb_y =1; r.db7_n=1; r.dbz_z=1; end
			I_TXA: begin r.x_sb =1; r.sb_db=1; r.sb_ac=1; r.db7_n=1; r.dbz_z=1; end
			I_TYA: begin r.y_sb =1; r.sb_db=1; r.sb_ac=1; r.db7_n=1; r.dbz_z=1; end
			I_TSX: begin r.s_sb =1; r.sb_db=1; r.sb_x =1; r.db7_n=1; r.dbz_z=1; end
			I_TXS: begin r.x_sb =1; r.sb_s =1; end   // the one transfer with no flags
			// DB precharges to $FF, so BI takes $FF for a decrement and its
			// inverse $00 with carry in for an increment. No constant needed.
			I_INX: begin r.x_sb=1; r.sb_add=1; r.ndb_add=1; r.sums=1; r.alucin=1; end
			I_INY: begin r.y_sb=1; r.sb_add=1; r.ndb_add=1; r.sums=1; r.alucin=1; end
			I_DEX: begin r.x_sb=1; r.sb_add=1; r.db_add =1; r.sums=1; end
			I_DEY: begin r.y_sb=1; r.sb_add=1; r.db_add =1; r.sums=1; end
			I_CLC, I_SEC: r.ir5_c = 1;
			I_CLI, I_SEI: r.ir5_i = 1;
			I_CLD, I_SED: r.ir5_d = 1;
			I_CLV:        r.zero_v = 1;
			default: ;
			endcase
		end
		return r;
	endfunction

	function automatic ctl_t op_b(input decode_t o);
		ctl_t r = CTL_IDLE;
		logic wrote;
		wrote = 1'b0;

		// Flags and the register write land together, both out of ADD.
		unique case (o.data)
		D_ORA, D_AND, D_EOR:
			begin r.sb_ac=1; r.db7_n=1; r.dbz_z=1; wrote=1; end
		D_ADC:
			begin r.sb_ac=1; r.db7_n=1; r.dbz_z=1; r.acr_c=1; r.avr_v=1;
			      r.daa=BCD_EN & p[3]; wrote=1; end
		D_SBC:
			begin r.sb_ac=1; r.db7_n=1; r.dbz_z=1; r.acr_c=1; r.avr_v=1;
			      r.dsa=BCD_EN & p[3]; wrote=1; end
		D_CMP, D_CPX, D_CPY:
			begin r.db7_n=1; r.dbz_z=1; r.acr_c=1; wrote=1; end
		// BIT keeps no result: N and V came straight off the operand a cycle
		// ago, and only Z comes from the AND.
		D_BIT:
			begin r.dbz_z=1; wrote=1; end
		// ANC is AND plus one extra: C is a copy of bit 7 of the result,
		// not an adder carry.
		D_ANC:
			begin r.sb_ac=1; r.db7_n=1; r.dbz_z=1; r.db7_c=1; wrote=1; end
		D_ALR:
			begin r.sb_ac=1; r.db7_n=1; r.dbz_z=1; r.acr_c=1; wrote=1; end
		// ARR takes its flags from ADC as much as from ROR: C is bit 6 of the
		// result and V is bit 6 against bit 5. In decimal mode a partial BCD
		// correction runs on the result while N, V and Z still come from the
		// value before it, and C comes from the high-nybble test instead of
		// bit 6. The nybbles tested are the AND result's; see mos6502_dp.sv.
		D_ARR:
			begin r.sb_ac=1; r.db7_n=1; r.dbz_z=1; r.arr_flags=1;
			      r.arr_daa = BCD_EN & p[3]; wrote=1; end
		// LAS writes A only. The netlist leaves X and S alone, against every
		// table including perfect6502's own.
		D_LAS:
			begin r.sb_ac=1; r.db7_n=1; r.dbz_z=1; wrote=1; end
		D_ANE:
			begin r.sb_ac=1; r.db7_n=1; r.dbz_z=1; wrote=1; end
		D_LXA:
			begin r.sb_ac=1; r.sb_x=1; r.db7_n=1; r.dbz_z=1; wrote=1; end
		D_SBX:
			begin r.sb_x=1; r.db7_n=1; r.dbz_z=1; r.acr_c=1; wrote=1; end
		default: ;
		endcase

		if (o.mode == M_ACC) begin
			r.sb_ac=1; r.db7_n=1; r.dbz_z=1; r.acr_c=1; wrote=1;
		end

		if (o.mode == M_IMP) begin
			unique case (o.imp)
			I_INX, I_DEX: begin r.sb_x=1; r.db7_n=1; r.dbz_z=1; wrote=1; end
			I_INY, I_DEY: begin r.sb_y=1; r.db7_n=1; r.dbz_z=1; wrote=1; end
			default: ;
			endcase
		end

		// Everything above reads its value out of ADD, over SB and onto DB so
		// the flag logic can see it. ROR is the exception the extra SB bit 7
		// line exists for: leaving it unasserted lets the precharge put a 1
		// there, which is how the carry rotates in.
		if (wrote) begin
			r.add_sb06 = 1;
			r.add_sb7  = ((o.mode == M_ACC && o.rmw == RMW_ROR) ||
			              (o.data == D_ARR)) ? ~p[0] : 1'b1;
			r.sb_db    = 1;
		end
		return r;
	endfunction

	// The same operation applied to a byte in memory, for the read-modify-
	// write cycles. AI and BI both take the operand, as the accumulator forms
	// do; DEC leans on the precharge for its $FF.
	function automatic ctl_t op_rmw(input rmwop_e k);
		ctl_t r = CTL_IDLE;
		r.dl_db = 1; r.db_add = 1;              // BI <= the operand
		unique case (k)
		RMW_ASL: begin r.sb_db=1; r.sb_add=1; r.sums=1; end
		RMW_ROL: begin r.sb_db=1; r.sb_add=1; r.sums=1; r.alucin=p[0]; end
		RMW_LSR: begin r.sb_db=1; r.sb_add=1; r.srs =1; end
		RMW_ROR: begin r.sb_db=1; r.sb_add=1; r.srs =1; end
		RMW_INC: begin r.zero_add=1;          r.sums=1; r.alucin=1; end
		RMW_DEC: begin r.sb_add=1;            r.sums=1; end  // AI <= $FF
		default: ;
		endcase
		return r;
	endfunction

	// Drive the finished read-modify-write value onto DB so DOR takes it.
	function automatic ctl_t rmw_out(input rmwop_e k, input dataop_e dop);
		ctl_t r = CTL_IDLE;
		r.add_sb06 = 1;
		r.add_sb7  = (k == RMW_ROR) ? ~p[0] : 1'b1;
		r.sb_db    = 1;
		r.db7_n    = 1; r.dbz_z = 1;
		if (k != RMW_INC && k != RMW_DEC) r.acr_c = 1;
		// Park the modified byte in BI for a combination opcode. The compare
		// and subtract halves want it inverted, since both are done as an add.
		if (dop == D_CMP || dop == D_SBC) r.ndb_add = 1;
		else if (dop != D_NONE)           r.db_add  = 1;
		return r;
	endfunction

	// What a store puts on DB.
	//
	// The last four are the H+1 group. Their write cycle is also the address
	// fix-up cycle, so the register and ADD - which holds the base address
	// high byte + 1 - are on SB together and the bus wire-ANDs them, exactly
	// the way ST_AX gets A & X. That merged value is what SB carries, which
	// is why the caller can send the same wire straight to ADH and get the
	// documented "value overwrites the address high byte on a page cross".
	function automatic ctl_t store_out(input storesrc_e k);
		ctl_t r = CTL_IDLE;
		unique case (k)
		ST_A:  begin r.ac_db = 1; end
		ST_X:  begin r.x_sb = 1; r.sb_db = 1; end
		ST_Y:  begin r.y_sb = 1; r.sb_db = 1; end
		ST_AX: begin r.ac_db = 1; r.x_sb = 1; r.sb_db = 1; end  // wire-AND: A & X
		ST_SHA, ST_TAS:
		       begin r.ac_db = 1; r.x_sb = 1; r.sb_db = 1;
		             r.add_sb06 = 1; r.add_sb7 = 1; end          // A & X & ADD
		ST_SHX: begin r.x_sb = 1; r.sb_db = 1;
		             r.add_sb06 = 1; r.add_sb7 = 1; end          // X & ADD
		ST_SHY: begin r.y_sb = 1; r.sb_db = 1;
		             r.add_sb06 = 1; r.add_sb7 = 1; end          // Y & ADD
		default: ;
		endcase
		return r;
	endfunction

	// The write cycle of an H+1 store. ADD already holds H+1 because the
	// fix-up cycle was told to carry in unconditionally. The address high
	// byte is only overwritten when the index actually carried; without a
	// carry the address is the ordinary base + index and ABH still holds H.
	// The write cycle of an H+1 store. ADD already holds H+1 because the
	// fix-up cycle was told to carry in unconditionally. The address high
	// byte is only overwritten when the index actually carried; without a
	// carry the address is the ordinary base + index and ABH still holds H.
	//
	// A stall on the fix-up cycle changes both halves on the die, and is not
	// modelled here.
	function automatic ctl_t store_sh(input storesrc_e k, input logic crossed);
		ctl_t r = store_out(k);
		r.wr = 1'b1;
		if (crossed) begin r.sb_adh = 1'b1; r.adh_abh = 1'b1; end
		return r;
	endfunction

	// ---- the addressing sequence, cycle by cycle ---------------------------
	// Cycle numbers are the "#" column of the addressing-mode cycle tables.

	function automatic ctl_t seq(input decode_t o, input logic [3:0] n,
	                             input logic idx_crossed);
		ctl_t r = CTL_IDLE;

		unique case (o.mode)

		// --- zero page ----------------------------------------------------
		M_ZP: unique case (n)
			4'd3: begin r = addr_zp();
			            if (o.access == A_WRITE) begin
			                r.wr = 1; r = r | store_out(o.store);
			            end
			      end
			4'd4: begin r.wr = 1; r.dl_db = 1; r = r | op_rmw(o.rmw); end
			4'd5: begin r.wr = 1; r = r | rmw_out(o.rmw, o.data); end
			default: ;
			endcase

		// --- zero page indexed. The pointer add wraps inside page 0 -------
		M_ZPX, M_ZPY: unique case (n)
			4'd3: begin r = addr_zp() | alu_index(o.mode == M_ZPX); end
			4'd4: begin r = addr_zp_add();
			            if (o.access == A_WRITE) begin
			                r.wr = 1; r = r | store_out(o.store);
			            end
			      end
			4'd5: begin r.wr = 1; r.dl_db = 1; r = r | op_rmw(o.rmw); end
			4'd6: begin r.wr = 1; r = r | rmw_out(o.rmw, o.data); end
			default: ;
			endcase

		// --- absolute. Cycle 3 parks the low byte in ADD ------------------
		M_ABS: unique case (n)
			4'd3: begin r = addr_pc(); r.pcl_pcl = 1; r.pch_pch = 1; r.ipc = 1;
			            r = r | alu_fix(1'b0);
			      end
			4'd4: begin r = addr_dl_add();
			            if (o.access == A_WRITE) begin
			                r.wr = 1; r = r | store_out(o.store);
			            end
			      end
			4'd5: begin r.wr = 1; r.dl_db = 1; r = r | op_rmw(o.rmw); end
			4'd6: begin r.wr = 1; r = r | rmw_out(o.rmw, o.data); end
			default: ;
			endcase

		// --- absolute indexed. The index goes onto the low byte only, so
		//     cycle 4 can address $100 short and cycle 5 repairs it.
		M_ABX, M_ABY: unique case (n)
			4'd3: begin r = addr_pc(); r.pcl_pcl = 1; r.pch_pch = 1; r.ipc = 1;
			            r = r | alu_index(o.mode == M_ABX);
			      end
			// An H+1 store carries into the fix-up unconditionally, so ADD
			// ends this cycle holding H+1 whether or not the index carried.
			// TAS's stable half goes here too: S takes A & X off SB now,
			// before ADD joins the bus and turns it into the store value.
			4'd4: begin r = addr_dl_add() | alu_fix(sh_store ? 1'b1 : acr_now);
			            if (o.store == ST_TAS) begin
			                r.ac_sb = 1; r.x_sb = 1; r.sb_s = 1;
			            end
			      end
			4'd5: begin if (sh_store) r = store_sh(o.store, idx_crossed);
			            else begin
			                r = addr_fix_hi();
			                if (o.access == A_WRITE) begin
			                    r.wr = 1; r = r | store_out(o.store);
			                end
			            end
			      end
			4'd6: begin r.wr = 1; r.dl_db = 1; r = r | op_rmw(o.rmw); end
			4'd7: begin r.wr = 1; r = r | rmw_out(o.rmw, o.data); end
			default: ;
			endcase

		// --- (zp,X). Cycle 3 is a dummy read at the un-indexed pointer ----
		M_IZX: unique case (n)
			4'd3: begin r = addr_zp() | alu_index(1'b1); end
			4'd4: begin r = addr_zp_add();
			            // ADD + 1, wrapping inside page 0
			            r.add_sb06 = 1; r.add_sb7 = 1; r.sb_add = 1;
			            r.ndb_add = 1; r.sums = 1; r.alucin = 1;
			      end
			4'd5: begin r = addr_zp_add() | alu_fix(1'b0); end
			4'd6: begin r = addr_dl_add();
			            if (o.access == A_WRITE) begin
			                r.wr = 1; r = r | store_out(o.store);
			            end
			      end
			4'd7: begin r.wr = 1; r.dl_db = 1; r = r | op_rmw(o.rmw); end
			4'd8: begin r.wr = 1; r = r | rmw_out(o.rmw, o.data); end
			default: ;
			endcase

		// --- (zp),Y -------------------------------------------------------
		M_IZY: unique case (n)
			4'd3: begin r = addr_zp();
			            r.zero_add = 1; r.dl_db = 1; r.db_add = 1;
			            r.sums = 1; r.alucin = 1;      // ADD <= pointer + 1
			      end
			4'd4: begin r = addr_zp_add() | alu_index(1'b0); end
			4'd5: begin r = addr_dl_add() | alu_fix(sh_store ? 1'b1 : acr_now); end
			4'd6: begin if (sh_store) r = store_sh(o.store, idx_crossed);
			            else begin
			                r = addr_fix_hi();
			                if (o.access == A_WRITE) begin
			                    r.wr = 1; r = r | store_out(o.store);
			                end
			            end
			      end
			4'd7: begin r.wr = 1; r.dl_db = 1; r = r | op_rmw(o.rmw); end
			4'd8: begin r.wr = 1; r = r | rmw_out(o.rmw, o.data); end
			default: ;
			endcase

		// --- branches. Cycle 3 adds the offset to PCL and is a dummy fetch
		//     at the old PC; cycle 4 only happens when PCH needs repairing.
		M_REL: unique case (n)
			4'd3: begin r = addr_pc();
			            r.dl_db = 1; r.sb_db = 1; r.sb_add = 1;   // AI <= offset
			            r.pcl_adl = 1; r.adl_add = 1;             // BI <= PCL
			            r.sums = 1;
			      end
			4'd4: begin r = addr_add_pch();
			            // Take the fixed low byte into PCL now, without
			            // incrementing: this fetch is a dummy at the wrong
			            // page and the real one comes next cycle.
			            r.adl_pcl = 1; r.pch_pch = 1;
			            r.pch_db = 1; r.db_add = 1;               // BI <= PCH
			            if (!br_back) begin r.zero_add = 1; r.alucin = 1; end
			            else            r.sb_add = 1;             // AI <= $FF
			            r.sums = 1;
			      end
			default: ;
			endcase

		// --- JMP absolute -------------------------------------------------
		M_JAB: unique case (n)
			4'd3: begin r = addr_pc() | alu_fix(1'b0); end
			default: ;
			endcase

		// --- JMP indirect. Only the pointer's low byte increments, which is
		//     the page-wrap bug; ABH is simply not reloaded on cycle 5.
		M_IND: unique case (n)
			4'd3: begin r = addr_pc(); r.pcl_pcl = 1; r.pch_pch = 1; r.ipc = 1;
			            r = r | alu_fix(1'b0);
			      end
			4'd4: begin r = addr_dl_add();
			            r.add_sb06 = 1; r.add_sb7 = 1; r.sb_add = 1;
			            r.ndb_add = 1; r.sums = 1; r.alucin = 1;
			      end
			4'd5: begin r.add_adl = 1; r.adl_abl = 1;   // ABH deliberately held
			            r = r | alu_fix(1'b0);
			      end
			default: ;
			endcase

		// --- stack --------------------------------------------------------
		// PHP is $08 and PHA $48, PLP $28 and PLA $68, so bit 6 of the
		// opcode is the only thing separating the register from the flags.
		M_PUSH: unique case (n)
			4'd3: begin r = addr_stack(); r.wr = 1; r.s_dec = 1;
			            if (ir[6]) r.ac_db = 1;                  // PHA
			            else begin r.p_db = 1; r.b_out = 1; end  // PHP, B set
			      end
			default: ;
			endcase

		M_PULL: unique case (n)
			4'd3: begin r = addr_stack(); r.s_inc = 1; end   // dummy read below the top
			4'd4: begin r = addr_stack(); end                // the real one
			default: ;
			endcase

		// --- JSR. The target's high byte is not fetched until the last
		//     cycle, after the push, which is why the address that gets
		//     stacked is the JSR's own last operand byte and RTS has to add
		//     one. The low byte waits in ADD across both pushes - it can,
		//     because S has its own incrementer and does not need the ALU.
		M_JSR: unique case (n)
			4'd3: begin r = addr_stack() | alu_fix(1'b0); end  // dummy read
			4'd4: begin r = addr_stack(); r.wr = 1; r.s_dec = 1; r.pch_db = 1; end
			4'd5: begin r = addr_stack(); r.wr = 1; r.s_dec = 1; r.pcl_db = 1; end
			4'd6: begin r = addr_pc(); r.pcl_pcl = 1; r.pch_pch = 1; end
			default: ;
			endcase

		// --- RTS. Cycle 6 is a dummy read at the un-incremented address,
		//     which is where the missing +1 gets added.
		M_RTS: unique case (n)
			4'd3: begin r = addr_stack(); r.s_inc = 1; end
			4'd4: begin r = addr_stack(); r.s_inc = 1; end
			4'd5: begin r = addr_stack() | alu_fix(1'b0); end  // ADD <= PCL
			4'd6: begin r = addr_dl_add();
			            r.adl_pcl = 1; r.adh_pch = 1; r.ipc = 1;
			      end
			default: ;
			endcase

		// --- RTI. Same shape as RTS with the status byte pulled first, and
		//     no final increment: the pushed address was already correct.
		M_RTI: unique case (n)
			4'd3: begin r = addr_stack(); r.s_inc = 1; end
			4'd4: begin r = addr_stack(); r.s_inc = 1; end
			4'd5: begin r = addr_stack(); r.s_inc = 1;
			            // the byte pulled last cycle is P; B and bit 5 drop
			            r.dl_db = 1; r.db_p = 1; r.db0_c = 1; r.db6_v = 1;
			            r.db7_n = 1;
			      end
			4'd6: begin r = addr_stack() | alu_fix(1'b0); end  // ADD <= PCL
			default: ;
			endcase

		// --- BRK, and the hardware interrupt and reset sequence -----------
		// One sequence for all four. A hardware interrupt forces $00 into IR
		// and holds PC still on the first two cycles, so the address that
		// gets stacked points at the instruction that did not run. Reset
		// walks the same path with its writes suppressed, which is why S
		// still ends up three lower without anything being stored.
		M_BRK: unique case (n)
			4'd3: begin r = addr_stack(); r.wr = ~res_active; r.s_dec = 1;
			            r.pch_db = 1;
			      end
			4'd4: begin r = addr_stack(); r.wr = ~res_active; r.s_dec = 1;
			            r.pcl_db = 1;
			      end
			4'd5: begin r = addr_stack(); r.wr = ~res_active; r.s_dec = 1;
			            r.p_db = 1; r.b_out = ~int_active;   // B set only for BRK
			      end
			4'd6: begin r = addr_vector(1'b0); r.one_i = 1; end
			4'd7: begin r = addr_vector(1'b1) | alu_fix(1'b0); end
			default: ;
			endcase

		default: ;
		endcase
		return r;
	endfunction

	// ---- branch test -------------------------------------------------------
	// Opcode bits 7:6 pick the flag and bit 5 the sense: xxy10000.
	//
	// Tested from IR on the branch's second cycle, not from the incoming
	// opcode on its first. The instruction before a branch is still retiring
	// during that first cycle - a DEX reaching zero sets Z a cycle after the
	// branch has been fetched - so testing that early reads the flag as it
	// was before the previous instruction finished. Klaus Dormann's very
	// first test is five DEX and a BEQ, which catches exactly this.
	logic br_flag;

	always_comb begin
		unique case (ir[7:6])
		2'b00:   br_flag = p[7];    // N
		2'b01:   br_flag = p[6];    // V
		2'b10:   br_flag = p[0];    // C
		default: br_flag = p[1];    // Z
		endcase
	end

	assign br_take = (br_flag == ir[5]);

	// A taken branch needs PCH moved when the offset's sign disagrees with
	// the carry out of the low byte. Decided and acted on in the same phase
	// 2, so it is combinational.
	assign br_fix = dl[7] ^ acr_now;

	// The offset's sign, straight off the latch that still holds it. Taking
	// it from the opcode fetch would sample the opcode, not the operand.
	assign br_back = dl[7];

	// The ready term, transcribed from the netlist rather than inferred.
	// klynch71's ready_control.v, net 1718:
	//
	//     rdy <= (READY | ~rw_n)      latched on phase 2
	//
	// Two things fall out of that one line, and both are why three inferred
	// rules all failed here. It is a REGISTER, so the stall of a cycle is
	// decided by READY sampled during the cycle before it. And the write term
	// is folded in on the same edge, so a write cycle sets it high and cannot
	// itself be held - which is the real content of "RDY does not stop a
	// write", and it also means the cycle right after a write cannot be held
	// either.
	//
	// RDY is read ONCE, at the phase 1 edge, and that reading stands for the
	// whole cycle. The pin itself is a level and MARIA moves it a clock or two
	// into phase 1, not on the edge; reading it live let phase 1 and phase 2 of
	// the same cycle disagree, and a cycle that formed its address held but
	// stepped its T-state anyway loses that address. Centipede lost the ADL
	// fetch of a JSR that way, every time a WSYNC stall ended on one.
	logic hold, wr_q, sync_q, rdy_q;
	wire  rdy_cy = phi1_en ? rdy : rdy_q;   // new value at the edge, held after
	assign hold = ~rdy_cy & ~wr_q;

	always_ff @(posedge clk_sys) begin
		if (!res_n)      rdy_q <= 1'b1;
		else if (phi1_en) rdy_q <= rdy;
	end

	// ACRL1 (net 572): the ALU carry the hold-carry path reads. Every page
	// fix-up carries out of the low byte except a backward branch's, which
	// borrowed instead, so only that one leaves ABH alone when it is held.
	logic br_back_q;             // the offset's sign, kept past DL's cycle
	logic adh_carry;
	assign adh_carry = ~((d.mode == M_REL) & br_back_q);

	// The write cycle of an H+1 store - the one RDY can hold, and the only
	// cycle in this core where a stall's effect on ADD is observable. On the
	// held repeat ABH loads from SB while ADD still has H+1, which is why the
	// address stays right; ADD is then cleared to $FF, so when the write
	// finally lands SB carries the plain register and the AND is gone. That
	// is the documented "sometimes AND (H+1) is dropped", and the netlist
	// shows a stall is what drops it.
	logic sh_write_cycle;
	assign sh_write_cycle = sh_store &&
	                        (((d.mode == M_ABX || d.mode == M_ABY) && t == 4'd5) ||
	                         (d.mode == M_IZY && t == 4'd6));

	// What a held cycle applies.
	//
	// A stall is not an idle cycle. The T-state stops, so the random control
	// logic builds the same control word again and it lands twice - once
	// during the held cycle, once when RDY comes back:
	//
	//     T4 ---- T5 ---- T0        no stall
	//     T4 -- T5 -- T5 -- T0      stalled at T5, same word both times
	//
	// Nothing double-commits because the RCL gates the stepping lines on rdy
	// itself. Transcribed from klynch71's random_control_logic.v:
	//
	//   ipc      inc_pc_c        = rdy & ...                       net 1275
	//   adl_abl  t4_abs_idx_or_t5_ind_y_n = ~(... | ~rdy)          net 46
	//   wr       mem_write_rdy   = ... & rdy                       net 187
	//   db_p     db_p            = ~(~rdy | ...)                   net 781
	//   pcl_db   pre_pcl_db      = ~(... | ~rdy)                   net 720
	//   ndb_add  pre_not_db_add_n= ~(rdy & ...)                    net 779
	//   sb_s     pre_sb_s_n      = ~(pla13 | jsr_rdy | stack_op_rdy) net 1358
	//
	// ABH is the one that is not simply dropped. Its usual enable
	// (using_adh_rdy, net 1343) goes away, but the hold-carry path
	// (net 933 -> 877) survives, and that path is alive only while the ALU
	// is the thing driving ADH. So a stall on an indexed access still loads
	// ABH from the ALU - which is why the netlist shows the page-fixed
	// address one cycle early there, and shows the unchanged address
	// everywhere else.
	//
	// The ALU is the one place this is not a literal transcription. The die
	// forces AI to 0, BI to ADL and the operation to OR (nets 1649, 604,
	// 1145), so ADD ends the held cycle holding ADL. Every cycle that can be
	// held here already has ADL == ADD, so simply leaving the ALU idle gives
	// the same ADD - and it also keeps ACR, which the die holds separately
	// through ACRL2, net 916.
	function automatic ctl_t hold_mask(input ctl_t x, input logic carry);
		ctl_t r = x;
		r.ipc     = 1'b0;
		r.adl_abl = 1'b0;
		r.wr      = 1'b0;
		r.db_p    = 1'b0;
		r.pcl_db  = 1'b0;
		r.ndb_add = 1'b0;
		r.sb_add  = 1'b0;  r.zero_add = 1'b0;  r.db_add = 1'b0;
		r.adl_add = 1'b0;
		r.sums = 1'b0; r.ands = 1'b0; r.eors = 1'b0; r.ors = 1'b0; r.srs = 1'b0;
		r.alucin = 1'b0;
		r.sb_s    = 1'b0;
		r.s_inc   = 1'b0;
		r.s_dec   = 1'b0;
		r.adh_abh = x.adh_abh & x.sb_adh & carry;
		r.add_ff  = sh_write_cycle;
		return r;
	endfunction

	// A held cycle is not an idle cycle: it applies its own control word.
	ctl_t c_pre;
	assign c_pre = hold ? hold_mask(c_reg, adh_carry) : c_reg;

	// S.O. does not go through the control word: it is sampled on phase 2 like
	// the other pins, but the V it sets lands on the next phase 1 as a
	// pulldown, so a PHP loading DOR on that same phase 1 pushes the new V.
	// See p_out in mos6502_dp.sv, which is where that half of it lives.
	always_comb begin
		c = c_pre;
		c.so_v = so_edge_q;
		c.wr   = c_pre.wr & ~res_now;   // a recognised RES suppresses every write
	end

	// A falling edge on S.O. sets V.
	logic so_edge, so_edge_q;
	assign so_edge = so_last & ~so_n;

	// ---- is the cycle we are in the last one? ------------------------------
	//
	// A function, because the timing generator needs the same question asked
	// about the cycle after this one: the die raises `end_x` when the next
	// cycle is T0, and `t_zero` follows from that.
	function automatic logic is_last(input logic [3:0] k, input logic crossed,
	                                 input logic taken, input logic pch_fix);
		logic r;
		r = 1'b0;
		if (k != 4'd1) begin
			unique case (d.mode)
			M_IMP, M_ACC, M_IMM: r = (k == 4'd2);
			M_JAM:               r = 1'b0;          // never finishes
			M_ZP:   r = (d.access == A_RMW) ? (k == 4'd5) : (k == 4'd3);
			M_ZPX,
			M_ZPY:  r = (d.access == A_RMW) ? (k == 4'd6) : (k == 4'd4);
			M_ABS:  r = (d.access == A_RMW) ? (k == 4'd6) : (k == 4'd4);
			M_ABX,
			M_ABY:  unique case (d.access)
			        A_READ:  r = (k == 4'd4 && !crossed) || (k == 4'd5);
			        A_WRITE: r = (k == 4'd5);
			        default: r = (k == 4'd7);
			        endcase
			M_IZX:  r = (d.access == A_RMW) ? (k == 4'd8) : (k == 4'd6);
			M_IZY:  unique case (d.access)
			        A_READ:  r = (k == 4'd5 && !crossed) || (k == 4'd6);
			        A_WRITE: r = (k == 4'd6);
			        default: r = (k == 4'd8);
			        endcase
			M_REL:  r = (k == 4'd2 && !taken)
			          | (k == 4'd3 && !pch_fix)
			          | (k == 4'd4);
			M_JAB:  r = (k == 4'd3);
			M_IND:  r = (k == 4'd5);
			M_JSR:  r = (k == 4'd6);
			M_RTS,
			M_RTI:  r = (k == 4'd6);
			M_BRK:  r = (k == 4'd7);
			M_PUSH: r = (k == 4'd3);
			M_PULL: r = (k == 4'd4);
			default: ;
			endcase
		end
		return r;
	endfunction

	logic last;
	assign last = is_last(t, pgx, br_take, br_fix);

	// ---- the die's timing generator ----------------------------------------
	//
	// Six stages, not a counter. `t` above is still what `seq` reads, but the
	// state that decides how RES lands is this chain, because it holds
	// combinations `t` cannot: T0 together with T2 on a two-cycle instruction,
	// T0 together with T1X while RES is recognised, and empty on the sixth
	// cycle of a seven-cycle one. Measured off the netlist, not read off the
	// schematic.
	//
	//   tg[0] T0    the last cycle of an instruction
	//   tg[1] T1X   the cycle after T0, which is the opcode fetch
	//   tg[2..5]    T2..T5
	//
	// Longer instructions do not extend the chain - it goes empty, and the two
	// extension latches carry cycles 7 and 8. klynch71 timing_generator.v.
	//
	// The chain is combinational from the copy taken at the end of the previous
	// cycle, exactly as the die builds it, so everything it reads - `t`, `last`,
	// `sync` - is this cycle's.
	logic [5:0] tg, tg_c2;
	logic       sync_c2;
	logic       ext_t5, ext_t6;

	// `t_zero` restarts the chain. It is not a reset signal: it is high on
	// every opcode fetch, and on T0 of anything longer than two cycles. A
	// two-cycle instruction's T0 comes from `tz_pre_n` instead, which is why
	// t_zero is low there - the netlist says so, the schematic reading does
	// not. klynch71's random_control_logic.v agrees.
	logic tz_pre_n, t_zero;
	assign tz_pre_n = ~pd_two_cycle;

	// `end_x`: this cycle ends the execute phase, so the next one is T0. A
	// two-cycle instruction has no end_x - its T0 comes from `tz_pre_n` - and
	// neither does a branch that is not taken, which has no T0 at all and
	// reaches the next fetch straight off its T2. A taken branch's T3 raises it
	// even though that cycle is already the last. random_control_logic.v.
	// The index add's carry has to come from before the latch on the cycle that
	// runs it: `pgx` is still the previous cycle's there, exactly as it is for
	// the fix-up decision itself.
	// Never on the fetch cycle: IR still holds the instruction that is
	// finishing there, so asking `is_last` about the next cycle would be asking
	// the wrong decode. Every one of the die's own end_x terms is T2 or later.
	logic end_x;
	// A branch contributes only its T3 term. Taken without a page cross it ends
	// on T3 and reaches its target through the short circuit, so that cycle is
	// T3 and the target fetch has an empty chain - there is no T0 anywhere in
	// it. Taken across a page, the same T3 term raises the T0 that does the
	// fix-up. Not taken, it has no T0 either.
	assign end_x = (t != 4'd1)
	             & ((d.mode == M_REL)
	                ? ((t == 4'd3) & tg[3])
	                : (is_last(t + 4'd1,
	                           (t == 4'd3 || t == 4'd4) ? acr_now : pgx,
	                           br_take, br_fix)
	                   & ~pd_two_cycle));

	// The RMW read and the dummy write after it are where the die sets its two
	// chain extension latches, which is how a six-stage chain carries an eight
	// cycle instruction. timing_generator.v, and random_control_logic.v.
	logic [3:0] rmw_rd;
	always_comb begin
		unique case (d.mode)
		M_ZP:        rmw_rd = 4'd3;
		M_ZPX, M_ZPY,
		M_ABS:       rmw_rd = 4'd4;
		M_ABX, M_ABY: rmw_rd = 4'd5;
		M_IZX, M_IZY: rmw_rd = 4'd6;
		default:     rmw_rd = 4'd0;
		endcase
	end
	// Registered, and frozen across a stall: the die's own pair hold their value
	// while ready is low (`shift_inc_dec_mem` keeps `~rdy & ..._c1_c2`), and `t`
	// runs a cycle ahead of the control word there, so reading it live would
	// arm them one cycle early.
	logic ext_t5_r, ext_t6_r;
	assign ext_t5 = hold ? ext_t5_r : ((d.access == A_RMW) && (t == rmw_rd));
	assign ext_t6 = hold ? ext_t6_r : ((d.access == A_RMW) && (t == rmw_rd + 4'd1));

	// t_zero restarts the chain: every opcode fetch, and the T0 that end_x
	// announced a cycle earlier. Held through a recognised RES.
	// The die gates every one of these terms with RDY - `end_x_rdy`,
	// `break_done` and `short_circuit_idx_add` are all `rdy & ...` - and it does
	// so in the phase 2 that closes before the cycle they apply to, so the gate
	// belongs to that cycle, not the announcing one. Here that is `rdy_cy`, read
	// once at this cycle's phase 1. Without it a stall restarts the chain
	// underneath the held cycle, where the die freezes it.
	logic end_x_q;
	assign t_zero = sync | (end_x_q & tg_go) | res_p;

	// The die's own ready, not the pin: `ready_control.v` holds it high through
	// a write, so a stalled write still advances the chain. That is exactly our
	// `hold`.
	logic tg_go;
	assign tg_go = ~hold;

	always_comb begin
		tg[0] = ~(sync | (~t_zero & tz_pre_n)) | (tg_c2[0] & ~tg_go);
		tg[1] = tg_c2[0] & tg_go;
		tg[2] = ~t_zero & ((tg_c2[2] & ~tg_go) | (sync_c2  & tg_go));
		tg[3] = ~t_zero & ((tg_c2[3] & ~tg_go) | (tg_c2[2] & tg_go));
		tg[4] = ~t_zero & ((tg_c2[4] & ~tg_go) | (tg_c2[3] & tg_go));
		tg[5] = ~t_zero & ((tg_c2[5] & ~tg_go) | (tg_c2[4] & tg_go));
	end

	// A jam parks the counter instead of letting it wrap. Nothing resets T on
	// the die, so a 1..8 counter that rolls over would walk back to t == 1 and
	// start fetching again - the part would un-jam, which it never does.
	// The chain decides, not the counter: T0 is the die's "this is the last
	// cycle", and RES can raise it early. That is what parks `t` at 1 through a
	// held reset, and what collapses a BRK's two vector cycles into one when RES
	// lands on them - both without a mechanism of their own.
	assign nt = (last | tg[0]) ? 4'd1
	          : (jammed && t >= 4'd6) ? t : (t + 4'd1);

	// ---- the control word for the cycle about to start ---------------------
	always_comb begin
		nc = CTL_IDLE;

		if (nt == 4'd1) begin
			// Opcode fetch, and the T+ half of whatever just finished. The
			// address usually comes from PC, but a jump, a return or a taken
			// branch puts the new address straight onto ADL/ADH from the
			// ALU, and PC follows from there.
			unique case (n_pc_src)
			PC_EA:  begin nc = addr_dl_add(); end
			PC_ADD: begin nc = addr_add_pch(); end
			PC_FIX: begin nc = addr_fix_hi();  end
			default: nc = addr_pc();
			endcase
			// PC follows whatever went onto the address bus. On the branch
			// fix-up cycle only PCH moves: the low byte was already put into
			// PCL on the dummy fetch, so it just recirculates.
			if (n_pc_src == PC_FIX) begin
				nc.pcl_pcl = 1; nc.adh_pch = 1;
			end else if (n_pc_src == PC_PC) begin
				nc.pcl_pcl = 1; nc.pch_pch = 1;
			end else begin
				nc.adl_pcl = 1;
				if (n_pc_src == PC_ADD) nc.pch_pch = 1; else nc.adh_pch = 1;
			end
			// A RES that reaches T0 while a BRK is still on its vector fetches
			// merges the two vector cycles into one, so the high byte is never
			// read. The die's `<RES low>FD`: this fetch takes ADL from the
			// vector HIGH address, still driven by zero_adl, and ADH from the
			// byte the merged cycle did read. PC follows both, as it always
			// does out of a vector.
			if (vec_merge) begin
				nc = addr_vector(1'b1);
				nc.dl_adh  = 1'b1;
				nc.adl_pcl = 1'b1;
				nc.adh_pch = 1'b1;
			end
			nc.ipc = ~(n_int_g | n_res_g);
			nc = nc | op_a(d);
		end else if (nt == 4'd2) begin
			// Second fetch. Predecode is reading the opcode off the pins
			// right now, and its only job here is to hold PC still for a
			// one-byte instruction so the next fetch does not skip a byte.
			nc = addr_pc();
			nc.pcl_pcl = 1; nc.pch_pch = 1;
			nc.ipc = ~pd_implied & ~int_active;
			nc = nc | op_b(d);
		end else begin
			nc = seq(d, nt, idx_cross_q);
		end

		// A jammed part stops sequencing. Nothing drives the address buses
		// any more, so the precharge shows through and the pins read $FFFF.
		if (jammed) begin
			nc = CTL_IDLE;
			nc.adl_abl = 1'b1;
			nc.adh_abh = 1'b1;
			// The timing generator does not stop the moment the PLA goes
			// quiet: the one-hot T still shifts twice more, and while it does
			// it keeps pulling ADL0 low. The netlist shows the whole visible
			// effect as $FFFF, $FFFE, $FFFE and then $FFFF for ever.
			nc.zero_adl0 = (nt == 4'd4) || (nt == 4'd5);
		end
	end

	// The interrupt sequence ends as the vector's high byte is read, so the
	// fetch that follows is an ordinary instruction and must advance PC. The
	// registered copy is still set at that moment, hence the combinational one.
	logic n_int_g, take_int;

	// ---- interrupt and reset recognition -----------------------------------
	//
	// The die never polls the pins. RES, IRQ and NMI each cross two latches
	// first, and the poll reads the far end of them:
	//
	//   stage 0   a latch open through phase 2, so it ends up holding the pin
	//             as it stood when phase 2 closed
	//   stage 1   a latch open through the next phase 1, which is where the
	//             request reaches the sequencer
	//   stage 2   the poll, at phase 2 of T0 (T2 for a branch)
	//
	// Both latches have closed before the poll, so for anything read in phase 2
	// one register covers stages 0 and 1 - clocked at phi1_en, because that is
	// the moment phase 2 ends. Reading the pin at the poll instead skips the
	// whole chain: an edge placed inside phase 1 is then taken an instruction
	// early, and a level that has risen again by the poll is lost even though
	// stage 1 still holds it. klynch71's interrupt_and_reset_control.v.
	//
	// None of these latches is reset. They have none on the die, and that is
	// what stops an NMI still held low across reset release from looking like a
	// fresh falling edge.
	logic irq_p;                // IRQ, recognised
	logic nmi_p;                // NMI pin, recognised
	logic nmi_l;                // NMIL: this low level has already been spent
	logic nmig, nmig_c2;        // NMI stage 1, and its value one cycle earlier
	logic vec_n_c2, brk_done_q;

	// RESG, half a phase before the register below takes it. The control word
	// being built right now belongs to the next cycle, so the PC hold on the
	// fetch that RES turns into its BRK has to read this one; everything that
	// acts on the current cycle - the write suppression, the vector select and
	// the IR substitution - reads the register.
	logic n_res_g;
	assign n_res_g = res_c2 | (res_g & ~brk_done);

	// RES stage 1. It does not stop the timing generator by itself - it raises
	// `t_zero`, which pins the chain at T0, and it blocks the prefetch path,
	// which keeps SYNC down.
	//
	// Power-up value is "reset recognised": a part comes out of the box with RES
	// held, and the die's settled state then already has the chain pulled. It
	// cannot come from a reset - this is the reset chain.
	/* verilator lint_off PROCASSINIT */
	logic res_c2 = 1'b1, res_p = 1'b1;
	/* verilator lint_on PROCASSINIT */
	// The die has two ways to reach an opcode fetch, and RES only blocks one.
	// `t_res_1 = pre_fetch_rdy | short_circuit_rdy`, and it is `pre_fetch_rdy`
	// that carries `& ~res_p_c2`. A taken branch that does not cross a page
	// reaches its target through the short-circuit instead, so RES landing on
	// its last cycle leaves that fetch alone - SYNC and all - where the same
	// RES on any ordinary last cycle stalls the fetch that follows.
	// klynch71 random_control_logic.v `t_res_1`.
	logic sync_short, res_no_prefetch;
	assign res_no_prefetch = res_p & ~sync_short;

	// The vector window, and the point the sequence ends. The die grounds the
	// NMI request from T5 phase 2 to T0 phase 1, which is our t 6 and 7 once
	// the phase-2 latch above is accounted for; that is what makes a half
	// hijack impossible while still leaving T5 phase 1 open to a full one.
	// `brk_done` is the only thing that clears NMI stage 1, and it lands a
	// cycle before the sequence's own last, so the line is free again by the
	// handler's first fetch.
	// RES reaching T0 while the BRK sequence is still fetching its vector. The
	// chain merges that cycle with T0, which is how the die loses the vector
	// high byte entirely.
	logic vec_merge;
	assign vec_merge = tg[0] && (d.mode == M_BRK) && (t == 4'd6);

	logic vec_cyc, brk_done;
	assign vec_cyc  = (d.mode == M_BRK) && (t == 4'd6 || t == 4'd7);
	assign brk_done = (d.mode == M_BRK) && (t == 4'd6) && !hold;

	// A fresh NMI request: the line is low, that low has not been spent, and no
	// vector fetch is in progress. Stage 1 holds it from there until the
	// sequence finishes; it is not re-derived from the pin in between.
	logic nmi_req;
	assign nmi_req = nmi_p & ~nmi_l & vec_n_c2;
	assign nmig    = nmi_req | (nmig_c2 & ~brk_done_q);

	always_ff @(posedge clk_sys) begin
		if (phi1_en) begin
			irq_p   <= ~irq_n;
			nmi_p   <= ~nmi_n;
			// NMIL arms once the request has reached stage 1 and stays armed
			// for as long as the line is held low, so a level cannot retrigger.
			// Only the line going high again clears it - which is why rearming
			// needs both that and the sequence finishing, in either order.
			nmi_l   <= ~nmi_n & (nmig_c2 | nmi_l);
			nmig_c2 <= nmig;
			// RES stage 0, and stage 2 one phase behind it. RESG holds itself
			// up until the sequence it started finishes, so a pulse far too
			// short to reach stage 2 on its own still runs a whole reset.
			res_c2  <= ~res_n;
			res_g   <= res_c2 | (res_g & ~brk_done_q);
			// The vector window has to be shut again by the phase 1 that ends
			// it, or the handler's own first fetch still blocks a new edge and
			// a second NMI arriving there is lost.
			vec_n_c2 <= ~vec_cyc;
		end
		if (phi2_en) begin
			res_p      <= res_c2;
			// Next cycle's fetch comes off the branch short circuit. `op-T3-branch`
			// is a PLA line, so it needs the chain actually at T3 - a RES that
			// has already pulled it to T0 stops the short circuit firing at all.
			sync_short <= (d.mode == M_REL) && (t == 4'd3) && last && tg[3];
			brk_done_q <= brk_done;

			// The timing generator's own copies. Nothing about RES stops these -
			// the die's latches keep clocking - and that is what lets the chain
			// collapse to T0 under RES rather than freezing where it was. RDY is
			// expressed inside the shift, not by stopping the latch.
			tg_c2   <= tg;
			sync_c2 <= sync;
			// Not while held: `t` runs a cycle ahead of the control word during a
			// stall, so asking `is_last` about the next cycle there asks about
			// the wrong one. The announcement from the last cycle that actually
			// ran is the one that still stands.
			if (!hold) begin
				end_x_q  <= end_x;
				ext_t5_r <= ext_t5;
				ext_t6_r <= ext_t6;
			end
		end
	end

	// NMI is an edge and ignores I; IRQ is a level and does not.
	logic int_req, poll_now;
	assign nmi_pending = nmig;
	assign int_req = nmi_pending || (irq_p && !p[2]);

	// Branches poll at T2 instead of at T0, because a branch that is taken
	// without crossing a page never reaches T0 at all. That is what makes a
	// taken branch delay interrupt recognition by one instruction. A
	// page-crossing branch does reach its last cycle, so it polls again
	// there.
	assign poll_now = (d.mode == M_REL) ? (t == 4'd2 || t == 4'd4) : last;

	// The hijack falls straight out of `addr_vector` reading the request live:
	// an NMI that arrives any time before the vector address is formed steals
	// it from a BRK or an IRQ, and nothing else about the sequence changes - a
	// hijacked BRK still pushes B set and still advances PC by two.

	// The poll result is held until the instruction actually ends.
	//
	// A BRK's own last cycle is the vector high fetch, not a poll point: the
	// next fetch is already the handler's first instruction. Polling there
	// consumed a pending NMI without taking it, because the branch below that
	// ends the sequence runs first - so the NMI vanished and the handler ran
	// to completion. The die polls at the handler's first instruction instead.
	assign take_int = last && !int_active && (d.mode != M_BRK)
	                  && (poll_now ? int_req : int_armed);

	always_comb begin
		n_int_g = int_g;
		if (d.mode == M_BRK && t == 4'd7) n_int_g = 1'b0;
		else if (take_int)                n_int_g = 1'b1;
	end

	// Where the next fetch gets its address. Decided on the last cycle of
	// whatever is finishing.
	always_comb begin
		n_pc_src = PC_PC;
		if (last) begin
			unique case (d.mode)
			M_JAB, M_IND, M_BRK, M_JSR, M_RTI: n_pc_src = PC_EA;
			// t==2 is a branch that was not taken, so PC just carries on.
			M_REL:        n_pc_src = (t == 4'd2) ? PC_PC :
			                         (t == 4'd3) ? PC_ADD : PC_FIX;
			default:      n_pc_src = PC_PC;
			endcase
		end
	end

	// ---- registers ---------------------------------------------------------
	always_ff @(posedge clk_sys) begin
		// Nothing here is gated by RES any more. The die stops none of these
		// latches; what a reset does is raise `t_zero`, which pins the chain at
		// T0, and suppress the prefetch, which keeps SYNC down. `nt` above then
		// holds `t` at 1 on its own.
		if (phi2_en) begin
			// The pins are sampled on phase 2.
			so_last  <= so_n;
			so_edge_q <= so_edge;

			// wr_q is the write term of the netlist's ready latch and is
			// updated every cycle; sync_q only tracks completed cycles.
			wr_q <= c.wr;
			if (!hold) sync_q <= (t == 4'd1);


			if (hold) begin
				t     <= t;
				c_reg <= c_reg;
				// The opcode fetch is where an interrupt turns the opcode into
				// a forced BRK, and RDY can hold that fetch. The die keeps
				// deciding for as long as it is held, so a request arriving
				// inside the stall still catches this fetch. Deciding once,
				// before the stall, takes it a whole instruction later.
				if (t == 4'd1 && !int_active && int_req) begin
					int_g      <= 1'b1;
					int_armed  <= 1'b0;
					// This fetch is now the sequence's first cycle, and a
					// forced BRK does not advance PC. The word was latched
					// before the stall, when it was still an ordinary fetch.
					c_reg.ipc <= 1'b0;
				end
				// ABH was loaded on this held repeat, from an SB that still
				// had H+1 in it. The write that follows must not load it
				// again out of an SB that no longer does.
				if (sh_write_cycle) c_reg.adh_abh <= 1'b0;
			end else begin
				t     <= nt;
				c_reg <= nc;
				// DL still holds the offset here; by the fix-up cycle it does not.
				if (d.mode == M_REL && t == 4'd3) br_back_q <= br_back;

				int_g      <= n_int_g;
				if (poll_now) int_armed <= int_req;
				if (take_int) int_armed <= 1'b0;

				if (t == 4'd1) begin
					ir      <= int_active ? 8'h00 : data_in;
					end

				// The index add's carry. Taken before the latch, because
				// the latched copy is still last cycle's at this point.
				if (t == 4'd3 || t == 4'd4) pgx <= acr_now;

				// pgx is overwritten by the fix-up add's own carry one cycle
				// later, and an H+1 store needs the index carry a cycle after
				// that, on its write. Keep a second copy taken only on the
				// cycle the index add actually ran.
				if ((d.mode == M_ABX || d.mode == M_ABY) && t == 4'd3)
					idx_cross_q <= acr_now;
				if (d.mode == M_IZY && t == 4'd4)
					idx_cross_q <= acr_now;

			end
		end
	end

endmodule
