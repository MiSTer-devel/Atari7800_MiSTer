//============================================================================
// The 6502's datapath control lines, as one bundle.
//
// These are the real lines the random control logic drives, named as
// .agents/evidence/6502/datapath.md section 1.1 names them (Hanson's names,
// with the visual6502 dpc* node in the comment where it helps). Keeping the
// silicon's names means the decode can be checked against the evidence line
// by line instead of by guessing what a made-up name meant.
//
// T-state naming, since the sources collide on it: this core uses the
// visual6502 "timing states" naming. T1 is the last cycle of an instruction
// and drives SYNC; T+ is Hanson's T1X. (The state machine page calls those
// T1F and T1 respectively.)
//============================================================================

package mos6502_pkg;

	// ---- instruction attributes -------------------------------------------
	// The vocabulary is the one .agents/evidence/6502/opcodes.md fixes, so a
	// row of opcodes.csv maps onto these names without translation.

	typedef enum logic [4:0] {
		M_IMP, M_ACC, M_IMM, M_ZP,  M_ZPX, M_ZPY, M_ABS, M_ABX,
		M_ABY, M_IZX, M_IZY, M_REL, M_JAB, M_IND, M_JSR, M_BRK,
		M_RTS, M_RTI, M_PUSH, M_PULL, M_JAM
	} mode_e;

	typedef enum logic [1:0] { A_READ, A_WRITE, A_RMW, A_OTHER } access_e;

	// The read-modify-write half of an instruction. Every stable undocumented
	// opcode is one of these paired with a data operation - SLO is ASL then
	// ORA, RRA is ROR then ADC - so the pair covers them without special
	// cases.
	typedef enum logic [2:0] {
		RMW_NONE, RMW_ASL, RMW_LSR, RMW_ROL, RMW_ROR, RMW_INC, RMW_DEC
	} rmwop_e;

	// What happens to the byte once it has been read.
	typedef enum logic [4:0] {
		D_NONE, D_ORA, D_AND, D_EOR, D_ADC, D_SBC, D_CMP, D_CPX, D_CPY,
		D_LDA,  D_LDX, D_LDY, D_LAX, D_BIT,
		D_ANC,  D_ALR, D_ARR, D_SBX, D_ANE, D_LXA, D_LAS
	} dataop_e;

	// What a write cycle puts on the bus. The last four are the unstable
	// stores, whose value depends on the address high byte.
	typedef enum logic [3:0] {
		ST_NONE, ST_A, ST_X, ST_Y, ST_AX, ST_SHA, ST_SHX, ST_SHY, ST_TAS
	} storesrc_e;

	// One-byte instructions that only move or flip something.
	typedef enum logic [4:0] {
		I_NOP, I_TAX, I_TAY, I_TXA, I_TYA, I_TSX, I_TXS,
		I_INX, I_INY, I_DEX, I_DEY,
		I_CLC, I_SEC, I_CLI, I_SEI, I_CLD, I_SED, I_CLV
	} impop_e;

	typedef struct packed {
		mode_e     mode;
		access_e   access;
		rmwop_e    rmw;
		dataop_e   data;
		storesrc_e store;
		impop_e    imp;
		logic      unstable;   // ANE LXA SHA SHX SHY TAS: value is not fully
		                       // determined by the architectural state
	} decode_t;

	typedef struct packed {
		// ---- bus drivers ---------------------------------------------------
		logic dl_db, dl_adl, dl_adh;      // input data latch onto each bus
		logic dl0_db;                     // only DB0 takes DL: ANC, ALR, ARR
		logic pcl_db, pcl_adl;
		logic pch_db, pch_adh;
		logic s_sb, s_adl;                // stack pointer out latch
		logic x_sb, y_sb;
		logic ac_sb, ac_db;
		logic add_sb06;                   // ALU result bits 6:0 onto SB
		logic add_sb7;                    // bit 7 separately - this is how ROR
		                                  // rotates the carry in: leave it off
		                                  // and SB7 reads back its precharge
		logic add_adl;
		logic p_db;

		// Forcing bits low against the precharge is how the fixed pages are
		// made: $00 zero page, $01 stack, $FF vector.
		logic zero_adl0, zero_adl1, zero_adl2;
		logic zero_adh0, zero_adh17;

		// ---- pass transistors, bidirectional shorts between two buses ------
		logic sb_db, sb_adh;

		// ---- register loads ------------------------------------------------
		logic sb_x, sb_y;
		logic sb_s, s_s;                  // S/S recirculates the out latch
		// The die decrements S through the main ALU. That collides with the
		// operation pipeline here - an ADC finishing while a PHA pushes wants
		// the same bus - so S gets its own incrementer. Not what the silicon
		// does; not observable either, since nothing can see S mid-cycle.
		logic s_inc, s_dec;
		logic sb_ac;                      // SB -> decimal adjust -> A
		logic sb_add, zero_add;           // AI side of the ALU
		logic db_add, ndb_add, adl_add;   // BI side. nDB is how subtract works
		logic adl_pcl, pcl_pcl;           // into PCLS
		logic adh_pch, pch_pch;           // into PCHS
		logic adl_abl, adh_abh;           // into the address pin registers

		// ---- ALU -------------------------------------------------------------
		logic sums, ands, eors, ors, srs;
		logic alucin;                     // 1/ADDC
		logic daa, dsa;                   // decimal add / decimal subtract
		logic add_ff;                     // ADD takes $FF: the die's held-cycle ALU
		logic arr_d;                      // ARR in decimal: sample the nybble tests
		logic arr_daa;                    // ARR in decimal: apply them
		logic ipc;                        // increment PC (#IPC, active here)

		// ---- flag loads ------------------------------------------------------
		logic db0_c, ir5_c, acr_c;        // C from DB0, from IR5, from the ALU
		logic db7_n, dbz_z;               // N from DB7, Z from the DB zero NOR
		logic ir5_i, ir5_d;
		logic db6_v, avr_v, one_v, zero_v;
		logic so_v;                       // S.O. sets V, on phase 2 of its own cycle
		logic db7_c;                      // C from DB bit 7: ANC
		logic arr_flags;                  // ARR's own C and V: bit 6, and
		                                  // bit 6 XOR bit 5
		logic one_i;                      // set I, on the vector fetch

		// Flags taken from the live ALU result at phase 2, rather than off DB
		// at phase 1. ADD is a latch, not a flop: it goes transparent during
		// phase 2, so the result it is capturing is already on SB and DB by
		// the time the flag latches close. That is why a 6502 updates N, Z, C
		// and V one cycle before it writes the register.
		logic alu_n, alu_z, alu_c, alu_v;
		logic db_p;                       // bulk load of Z, I and D from DB

		// P bit 4 as driven onto DB. This is the chip's b_out_n / D1x1: 1 for
		// a software BRK or PHP, 0 when reset or a hardware interrupt is
		// forcing the BRK. There is no B flip flop to read it back from.
		logic b_out;

		// ---- bus cycle -------------------------------------------------------
		logic wr;                         // drive a write cycle (R/W low)
	} ctl_t;

	// Every line is active high and every line is off here, so the sequencer
	// can build a cycle's control word as the OR of independent parts - the
	// address steering, the operation, the interrupt sequencer - the way the
	// random control logic ORs its own terms together. A line nobody asserts
	// is a quiet no-op rather than a latch.
	localparam ctl_t CTL_IDLE = '0;

endpackage
