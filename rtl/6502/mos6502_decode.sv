//============================================================================
// Opcode attribute decode. GENERATED - do not edit by hand.
//
//   source:    .agents/evidence/6502/opcodes.csv
//   generator: .agents/tools/gen6502/gen_decode.py
//
// Regenerate with:  python3 .agents/tools/gen6502/gen_decode.py
//
// This is the table the real chip keeps in its decode PLA. It is generated
// from the evidence file so the two cannot drift, and so a transcription slip
// across 256 rows is not possible.
//============================================================================

module mos6502_decode
	import mos6502_pkg::*;
(
	input  logic [7:0] ir,
	output decode_t    d
);

	// The table builds a plain vector and the struct is that vector reread.
	// Assigning the struct directly is the same hardware, and it is what this
	// generated file used to emit, but Quartus 17's Verilog elaborator crashes
	// trying to extract a ROM from a 256 way case whose target is a packed
	// struct - "Can't extract a ROM based on this value", VRFX
	// verivalue_elab.cpp:3273. A vector target elaborates.
	logic [$bits(decode_t)-1:0] dv;

	always_comb begin
		unique case (ir)
			8'h00: dv = {M_BRK, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // BRK brk
			8'h01: dv = {M_IZX, A_READ, RMW_NONE, D_ORA, ST_NONE, I_NOP, 1'b0};  // ORA izx
			8'h02: dv = {M_JAM, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // JAM jam
			8'h03: dv = {M_IZX, A_RMW, RMW_ASL, D_ORA, ST_NONE, I_NOP, 1'b0};  // SLO izx
			8'h04: dv = {M_ZP, A_READ, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP zp
			8'h05: dv = {M_ZP, A_READ, RMW_NONE, D_ORA, ST_NONE, I_NOP, 1'b0};  // ORA zp
			8'h06: dv = {M_ZP, A_RMW, RMW_ASL, D_NONE, ST_NONE, I_NOP, 1'b0};  // ASL zp
			8'h07: dv = {M_ZP, A_RMW, RMW_ASL, D_ORA, ST_NONE, I_NOP, 1'b0};  // SLO zp
			8'h08: dv = {M_PUSH, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // PHP push
			8'h09: dv = {M_IMM, A_OTHER, RMW_NONE, D_ORA, ST_NONE, I_NOP, 1'b0};  // ORA imm
			8'h0A: dv = {M_ACC, A_OTHER, RMW_ASL, D_NONE, ST_NONE, I_NOP, 1'b0};  // ASL acc
			8'h0B: dv = {M_IMM, A_OTHER, RMW_NONE, D_ANC, ST_NONE, I_NOP, 1'b0};  // ANC imm
			8'h0C: dv = {M_ABS, A_READ, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP abs
			8'h0D: dv = {M_ABS, A_READ, RMW_NONE, D_ORA, ST_NONE, I_NOP, 1'b0};  // ORA abs
			8'h0E: dv = {M_ABS, A_RMW, RMW_ASL, D_NONE, ST_NONE, I_NOP, 1'b0};  // ASL abs
			8'h0F: dv = {M_ABS, A_RMW, RMW_ASL, D_ORA, ST_NONE, I_NOP, 1'b0};  // SLO abs
			8'h10: dv = {M_REL, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // BPL rel
			8'h11: dv = {M_IZY, A_READ, RMW_NONE, D_ORA, ST_NONE, I_NOP, 1'b0};  // ORA izy
			8'h12: dv = {M_JAM, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // JAM jam
			8'h13: dv = {M_IZY, A_RMW, RMW_ASL, D_ORA, ST_NONE, I_NOP, 1'b0};  // SLO izy
			8'h14: dv = {M_ZPX, A_READ, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP zpx
			8'h15: dv = {M_ZPX, A_READ, RMW_NONE, D_ORA, ST_NONE, I_NOP, 1'b0};  // ORA zpx
			8'h16: dv = {M_ZPX, A_RMW, RMW_ASL, D_NONE, ST_NONE, I_NOP, 1'b0};  // ASL zpx
			8'h17: dv = {M_ZPX, A_RMW, RMW_ASL, D_ORA, ST_NONE, I_NOP, 1'b0};  // SLO zpx
			8'h18: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_CLC, 1'b0};  // CLC imp
			8'h19: dv = {M_ABY, A_READ, RMW_NONE, D_ORA, ST_NONE, I_NOP, 1'b0};  // ORA aby
			8'h1A: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP imp
			8'h1B: dv = {M_ABY, A_RMW, RMW_ASL, D_ORA, ST_NONE, I_NOP, 1'b0};  // SLO aby
			8'h1C: dv = {M_ABX, A_READ, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP abx
			8'h1D: dv = {M_ABX, A_READ, RMW_NONE, D_ORA, ST_NONE, I_NOP, 1'b0};  // ORA abx
			8'h1E: dv = {M_ABX, A_RMW, RMW_ASL, D_NONE, ST_NONE, I_NOP, 1'b0};  // ASL abx
			8'h1F: dv = {M_ABX, A_RMW, RMW_ASL, D_ORA, ST_NONE, I_NOP, 1'b0};  // SLO abx
			8'h20: dv = {M_JSR, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // JSR jsr
			8'h21: dv = {M_IZX, A_READ, RMW_NONE, D_AND, ST_NONE, I_NOP, 1'b0};  // AND izx
			8'h22: dv = {M_JAM, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // JAM jam
			8'h23: dv = {M_IZX, A_RMW, RMW_ROL, D_AND, ST_NONE, I_NOP, 1'b0};  // RLA izx
			8'h24: dv = {M_ZP, A_READ, RMW_NONE, D_BIT, ST_NONE, I_NOP, 1'b0};  // BIT zp
			8'h25: dv = {M_ZP, A_READ, RMW_NONE, D_AND, ST_NONE, I_NOP, 1'b0};  // AND zp
			8'h26: dv = {M_ZP, A_RMW, RMW_ROL, D_NONE, ST_NONE, I_NOP, 1'b0};  // ROL zp
			8'h27: dv = {M_ZP, A_RMW, RMW_ROL, D_AND, ST_NONE, I_NOP, 1'b0};  // RLA zp
			8'h28: dv = {M_PULL, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // PLP pull
			8'h29: dv = {M_IMM, A_OTHER, RMW_NONE, D_AND, ST_NONE, I_NOP, 1'b0};  // AND imm
			8'h2A: dv = {M_ACC, A_OTHER, RMW_ROL, D_NONE, ST_NONE, I_NOP, 1'b0};  // ROL acc
			8'h2B: dv = {M_IMM, A_OTHER, RMW_NONE, D_ANC, ST_NONE, I_NOP, 1'b0};  // ANC imm
			8'h2C: dv = {M_ABS, A_READ, RMW_NONE, D_BIT, ST_NONE, I_NOP, 1'b0};  // BIT abs
			8'h2D: dv = {M_ABS, A_READ, RMW_NONE, D_AND, ST_NONE, I_NOP, 1'b0};  // AND abs
			8'h2E: dv = {M_ABS, A_RMW, RMW_ROL, D_NONE, ST_NONE, I_NOP, 1'b0};  // ROL abs
			8'h2F: dv = {M_ABS, A_RMW, RMW_ROL, D_AND, ST_NONE, I_NOP, 1'b0};  // RLA abs
			8'h30: dv = {M_REL, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // BMI rel
			8'h31: dv = {M_IZY, A_READ, RMW_NONE, D_AND, ST_NONE, I_NOP, 1'b0};  // AND izy
			8'h32: dv = {M_JAM, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // JAM jam
			8'h33: dv = {M_IZY, A_RMW, RMW_ROL, D_AND, ST_NONE, I_NOP, 1'b0};  // RLA izy
			8'h34: dv = {M_ZPX, A_READ, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP zpx
			8'h35: dv = {M_ZPX, A_READ, RMW_NONE, D_AND, ST_NONE, I_NOP, 1'b0};  // AND zpx
			8'h36: dv = {M_ZPX, A_RMW, RMW_ROL, D_NONE, ST_NONE, I_NOP, 1'b0};  // ROL zpx
			8'h37: dv = {M_ZPX, A_RMW, RMW_ROL, D_AND, ST_NONE, I_NOP, 1'b0};  // RLA zpx
			8'h38: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_SEC, 1'b0};  // SEC imp
			8'h39: dv = {M_ABY, A_READ, RMW_NONE, D_AND, ST_NONE, I_NOP, 1'b0};  // AND aby
			8'h3A: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP imp
			8'h3B: dv = {M_ABY, A_RMW, RMW_ROL, D_AND, ST_NONE, I_NOP, 1'b0};  // RLA aby
			8'h3C: dv = {M_ABX, A_READ, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP abx
			8'h3D: dv = {M_ABX, A_READ, RMW_NONE, D_AND, ST_NONE, I_NOP, 1'b0};  // AND abx
			8'h3E: dv = {M_ABX, A_RMW, RMW_ROL, D_NONE, ST_NONE, I_NOP, 1'b0};  // ROL abx
			8'h3F: dv = {M_ABX, A_RMW, RMW_ROL, D_AND, ST_NONE, I_NOP, 1'b0};  // RLA abx
			8'h40: dv = {M_RTI, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // RTI rti
			8'h41: dv = {M_IZX, A_READ, RMW_NONE, D_EOR, ST_NONE, I_NOP, 1'b0};  // EOR izx
			8'h42: dv = {M_JAM, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // JAM jam
			8'h43: dv = {M_IZX, A_RMW, RMW_LSR, D_EOR, ST_NONE, I_NOP, 1'b0};  // SRE izx
			8'h44: dv = {M_ZP, A_READ, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP zp
			8'h45: dv = {M_ZP, A_READ, RMW_NONE, D_EOR, ST_NONE, I_NOP, 1'b0};  // EOR zp
			8'h46: dv = {M_ZP, A_RMW, RMW_LSR, D_NONE, ST_NONE, I_NOP, 1'b0};  // LSR zp
			8'h47: dv = {M_ZP, A_RMW, RMW_LSR, D_EOR, ST_NONE, I_NOP, 1'b0};  // SRE zp
			8'h48: dv = {M_PUSH, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // PHA push
			8'h49: dv = {M_IMM, A_OTHER, RMW_NONE, D_EOR, ST_NONE, I_NOP, 1'b0};  // EOR imm
			8'h4A: dv = {M_ACC, A_OTHER, RMW_LSR, D_NONE, ST_NONE, I_NOP, 1'b0};  // LSR acc
			8'h4B: dv = {M_IMM, A_OTHER, RMW_NONE, D_ALR, ST_NONE, I_NOP, 1'b0};  // ALR imm
			8'h4C: dv = {M_JAB, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // JMP jab
			8'h4D: dv = {M_ABS, A_READ, RMW_NONE, D_EOR, ST_NONE, I_NOP, 1'b0};  // EOR abs
			8'h4E: dv = {M_ABS, A_RMW, RMW_LSR, D_NONE, ST_NONE, I_NOP, 1'b0};  // LSR abs
			8'h4F: dv = {M_ABS, A_RMW, RMW_LSR, D_EOR, ST_NONE, I_NOP, 1'b0};  // SRE abs
			8'h50: dv = {M_REL, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // BVC rel
			8'h51: dv = {M_IZY, A_READ, RMW_NONE, D_EOR, ST_NONE, I_NOP, 1'b0};  // EOR izy
			8'h52: dv = {M_JAM, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // JAM jam
			8'h53: dv = {M_IZY, A_RMW, RMW_LSR, D_EOR, ST_NONE, I_NOP, 1'b0};  // SRE izy
			8'h54: dv = {M_ZPX, A_READ, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP zpx
			8'h55: dv = {M_ZPX, A_READ, RMW_NONE, D_EOR, ST_NONE, I_NOP, 1'b0};  // EOR zpx
			8'h56: dv = {M_ZPX, A_RMW, RMW_LSR, D_NONE, ST_NONE, I_NOP, 1'b0};  // LSR zpx
			8'h57: dv = {M_ZPX, A_RMW, RMW_LSR, D_EOR, ST_NONE, I_NOP, 1'b0};  // SRE zpx
			8'h58: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_CLI, 1'b0};  // CLI imp
			8'h59: dv = {M_ABY, A_READ, RMW_NONE, D_EOR, ST_NONE, I_NOP, 1'b0};  // EOR aby
			8'h5A: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP imp
			8'h5B: dv = {M_ABY, A_RMW, RMW_LSR, D_EOR, ST_NONE, I_NOP, 1'b0};  // SRE aby
			8'h5C: dv = {M_ABX, A_READ, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP abx
			8'h5D: dv = {M_ABX, A_READ, RMW_NONE, D_EOR, ST_NONE, I_NOP, 1'b0};  // EOR abx
			8'h5E: dv = {M_ABX, A_RMW, RMW_LSR, D_NONE, ST_NONE, I_NOP, 1'b0};  // LSR abx
			8'h5F: dv = {M_ABX, A_RMW, RMW_LSR, D_EOR, ST_NONE, I_NOP, 1'b0};  // SRE abx
			8'h60: dv = {M_RTS, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // RTS rts
			8'h61: dv = {M_IZX, A_READ, RMW_NONE, D_ADC, ST_NONE, I_NOP, 1'b0};  // ADC izx
			8'h62: dv = {M_JAM, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // JAM jam
			8'h63: dv = {M_IZX, A_RMW, RMW_ROR, D_ADC, ST_NONE, I_NOP, 1'b0};  // RRA izx
			8'h64: dv = {M_ZP, A_READ, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP zp
			8'h65: dv = {M_ZP, A_READ, RMW_NONE, D_ADC, ST_NONE, I_NOP, 1'b0};  // ADC zp
			8'h66: dv = {M_ZP, A_RMW, RMW_ROR, D_NONE, ST_NONE, I_NOP, 1'b0};  // ROR zp
			8'h67: dv = {M_ZP, A_RMW, RMW_ROR, D_ADC, ST_NONE, I_NOP, 1'b0};  // RRA zp
			8'h68: dv = {M_PULL, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // PLA pull
			8'h69: dv = {M_IMM, A_OTHER, RMW_NONE, D_ADC, ST_NONE, I_NOP, 1'b0};  // ADC imm
			8'h6A: dv = {M_ACC, A_OTHER, RMW_ROR, D_NONE, ST_NONE, I_NOP, 1'b0};  // ROR acc
			8'h6B: dv = {M_IMM, A_OTHER, RMW_NONE, D_ARR, ST_NONE, I_NOP, 1'b0};  // ARR imm
			8'h6C: dv = {M_IND, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // JMP ind
			8'h6D: dv = {M_ABS, A_READ, RMW_NONE, D_ADC, ST_NONE, I_NOP, 1'b0};  // ADC abs
			8'h6E: dv = {M_ABS, A_RMW, RMW_ROR, D_NONE, ST_NONE, I_NOP, 1'b0};  // ROR abs
			8'h6F: dv = {M_ABS, A_RMW, RMW_ROR, D_ADC, ST_NONE, I_NOP, 1'b0};  // RRA abs
			8'h70: dv = {M_REL, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // BVS rel
			8'h71: dv = {M_IZY, A_READ, RMW_NONE, D_ADC, ST_NONE, I_NOP, 1'b0};  // ADC izy
			8'h72: dv = {M_JAM, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // JAM jam
			8'h73: dv = {M_IZY, A_RMW, RMW_ROR, D_ADC, ST_NONE, I_NOP, 1'b0};  // RRA izy
			8'h74: dv = {M_ZPX, A_READ, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP zpx
			8'h75: dv = {M_ZPX, A_READ, RMW_NONE, D_ADC, ST_NONE, I_NOP, 1'b0};  // ADC zpx
			8'h76: dv = {M_ZPX, A_RMW, RMW_ROR, D_NONE, ST_NONE, I_NOP, 1'b0};  // ROR zpx
			8'h77: dv = {M_ZPX, A_RMW, RMW_ROR, D_ADC, ST_NONE, I_NOP, 1'b0};  // RRA zpx
			8'h78: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_SEI, 1'b0};  // SEI imp
			8'h79: dv = {M_ABY, A_READ, RMW_NONE, D_ADC, ST_NONE, I_NOP, 1'b0};  // ADC aby
			8'h7A: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP imp
			8'h7B: dv = {M_ABY, A_RMW, RMW_ROR, D_ADC, ST_NONE, I_NOP, 1'b0};  // RRA aby
			8'h7C: dv = {M_ABX, A_READ, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP abx
			8'h7D: dv = {M_ABX, A_READ, RMW_NONE, D_ADC, ST_NONE, I_NOP, 1'b0};  // ADC abx
			8'h7E: dv = {M_ABX, A_RMW, RMW_ROR, D_NONE, ST_NONE, I_NOP, 1'b0};  // ROR abx
			8'h7F: dv = {M_ABX, A_RMW, RMW_ROR, D_ADC, ST_NONE, I_NOP, 1'b0};  // RRA abx
			8'h80: dv = {M_IMM, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP imm
			8'h81: dv = {M_IZX, A_WRITE, RMW_NONE, D_NONE, ST_A, I_NOP, 1'b0};  // STA izx
			8'h82: dv = {M_IMM, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP imm
			8'h83: dv = {M_IZX, A_WRITE, RMW_NONE, D_NONE, ST_AX, I_NOP, 1'b0};  // SAX izx
			8'h84: dv = {M_ZP, A_WRITE, RMW_NONE, D_NONE, ST_Y, I_NOP, 1'b0};  // STY zp
			8'h85: dv = {M_ZP, A_WRITE, RMW_NONE, D_NONE, ST_A, I_NOP, 1'b0};  // STA zp
			8'h86: dv = {M_ZP, A_WRITE, RMW_NONE, D_NONE, ST_X, I_NOP, 1'b0};  // STX zp
			8'h87: dv = {M_ZP, A_WRITE, RMW_NONE, D_NONE, ST_AX, I_NOP, 1'b0};  // SAX zp
			8'h88: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_DEY, 1'b0};  // DEY imp
			8'h89: dv = {M_IMM, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP imm
			8'h8A: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_TXA, 1'b0};  // TXA imp
			8'h8B: dv = {M_IMM, A_OTHER, RMW_NONE, D_ANE, ST_NONE, I_NOP, 1'b1};  // ANE imm
			8'h8C: dv = {M_ABS, A_WRITE, RMW_NONE, D_NONE, ST_Y, I_NOP, 1'b0};  // STY abs
			8'h8D: dv = {M_ABS, A_WRITE, RMW_NONE, D_NONE, ST_A, I_NOP, 1'b0};  // STA abs
			8'h8E: dv = {M_ABS, A_WRITE, RMW_NONE, D_NONE, ST_X, I_NOP, 1'b0};  // STX abs
			8'h8F: dv = {M_ABS, A_WRITE, RMW_NONE, D_NONE, ST_AX, I_NOP, 1'b0};  // SAX abs
			8'h90: dv = {M_REL, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // BCC rel
			8'h91: dv = {M_IZY, A_WRITE, RMW_NONE, D_NONE, ST_A, I_NOP, 1'b0};  // STA izy
			8'h92: dv = {M_JAM, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // JAM jam
			8'h93: dv = {M_IZY, A_WRITE, RMW_NONE, D_NONE, ST_SHA, I_NOP, 1'b1};  // SHA izy
			8'h94: dv = {M_ZPX, A_WRITE, RMW_NONE, D_NONE, ST_Y, I_NOP, 1'b0};  // STY zpx
			8'h95: dv = {M_ZPX, A_WRITE, RMW_NONE, D_NONE, ST_A, I_NOP, 1'b0};  // STA zpx
			8'h96: dv = {M_ZPY, A_WRITE, RMW_NONE, D_NONE, ST_X, I_NOP, 1'b0};  // STX zpy
			8'h97: dv = {M_ZPY, A_WRITE, RMW_NONE, D_NONE, ST_AX, I_NOP, 1'b0};  // SAX zpy
			8'h98: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_TYA, 1'b0};  // TYA imp
			8'h99: dv = {M_ABY, A_WRITE, RMW_NONE, D_NONE, ST_A, I_NOP, 1'b0};  // STA aby
			8'h9A: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_TXS, 1'b0};  // TXS imp
			8'h9B: dv = {M_ABY, A_WRITE, RMW_NONE, D_NONE, ST_TAS, I_NOP, 1'b1};  // TAS aby
			8'h9C: dv = {M_ABX, A_WRITE, RMW_NONE, D_NONE, ST_SHY, I_NOP, 1'b1};  // SHY abx
			8'h9D: dv = {M_ABX, A_WRITE, RMW_NONE, D_NONE, ST_A, I_NOP, 1'b0};  // STA abx
			8'h9E: dv = {M_ABY, A_WRITE, RMW_NONE, D_NONE, ST_SHX, I_NOP, 1'b1};  // SHX aby
			8'h9F: dv = {M_ABY, A_WRITE, RMW_NONE, D_NONE, ST_SHA, I_NOP, 1'b1};  // SHA aby
			8'hA0: dv = {M_IMM, A_OTHER, RMW_NONE, D_LDY, ST_NONE, I_NOP, 1'b0};  // LDY imm
			8'hA1: dv = {M_IZX, A_READ, RMW_NONE, D_LDA, ST_NONE, I_NOP, 1'b0};  // LDA izx
			8'hA2: dv = {M_IMM, A_OTHER, RMW_NONE, D_LDX, ST_NONE, I_NOP, 1'b0};  // LDX imm
			8'hA3: dv = {M_IZX, A_READ, RMW_NONE, D_LAX, ST_NONE, I_NOP, 1'b0};  // LAX izx
			8'hA4: dv = {M_ZP, A_READ, RMW_NONE, D_LDY, ST_NONE, I_NOP, 1'b0};  // LDY zp
			8'hA5: dv = {M_ZP, A_READ, RMW_NONE, D_LDA, ST_NONE, I_NOP, 1'b0};  // LDA zp
			8'hA6: dv = {M_ZP, A_READ, RMW_NONE, D_LDX, ST_NONE, I_NOP, 1'b0};  // LDX zp
			8'hA7: dv = {M_ZP, A_READ, RMW_NONE, D_LAX, ST_NONE, I_NOP, 1'b0};  // LAX zp
			8'hA8: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_TAY, 1'b0};  // TAY imp
			8'hA9: dv = {M_IMM, A_OTHER, RMW_NONE, D_LDA, ST_NONE, I_NOP, 1'b0};  // LDA imm
			8'hAA: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_TAX, 1'b0};  // TAX imp
			8'hAB: dv = {M_IMM, A_OTHER, RMW_NONE, D_LXA, ST_NONE, I_NOP, 1'b1};  // LXA imm
			8'hAC: dv = {M_ABS, A_READ, RMW_NONE, D_LDY, ST_NONE, I_NOP, 1'b0};  // LDY abs
			8'hAD: dv = {M_ABS, A_READ, RMW_NONE, D_LDA, ST_NONE, I_NOP, 1'b0};  // LDA abs
			8'hAE: dv = {M_ABS, A_READ, RMW_NONE, D_LDX, ST_NONE, I_NOP, 1'b0};  // LDX abs
			8'hAF: dv = {M_ABS, A_READ, RMW_NONE, D_LAX, ST_NONE, I_NOP, 1'b0};  // LAX abs
			8'hB0: dv = {M_REL, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // BCS rel
			8'hB1: dv = {M_IZY, A_READ, RMW_NONE, D_LDA, ST_NONE, I_NOP, 1'b0};  // LDA izy
			8'hB2: dv = {M_JAM, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // JAM jam
			8'hB3: dv = {M_IZY, A_READ, RMW_NONE, D_LAX, ST_NONE, I_NOP, 1'b0};  // LAX izy
			8'hB4: dv = {M_ZPX, A_READ, RMW_NONE, D_LDY, ST_NONE, I_NOP, 1'b0};  // LDY zpx
			8'hB5: dv = {M_ZPX, A_READ, RMW_NONE, D_LDA, ST_NONE, I_NOP, 1'b0};  // LDA zpx
			8'hB6: dv = {M_ZPY, A_READ, RMW_NONE, D_LDX, ST_NONE, I_NOP, 1'b0};  // LDX zpy
			8'hB7: dv = {M_ZPY, A_READ, RMW_NONE, D_LAX, ST_NONE, I_NOP, 1'b0};  // LAX zpy
			8'hB8: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_CLV, 1'b0};  // CLV imp
			8'hB9: dv = {M_ABY, A_READ, RMW_NONE, D_LDA, ST_NONE, I_NOP, 1'b0};  // LDA aby
			8'hBA: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_TSX, 1'b0};  // TSX imp
			8'hBB: dv = {M_ABY, A_READ, RMW_NONE, D_LAS, ST_NONE, I_NOP, 1'b0};  // LAS aby
			8'hBC: dv = {M_ABX, A_READ, RMW_NONE, D_LDY, ST_NONE, I_NOP, 1'b0};  // LDY abx
			8'hBD: dv = {M_ABX, A_READ, RMW_NONE, D_LDA, ST_NONE, I_NOP, 1'b0};  // LDA abx
			8'hBE: dv = {M_ABY, A_READ, RMW_NONE, D_LDX, ST_NONE, I_NOP, 1'b0};  // LDX aby
			8'hBF: dv = {M_ABY, A_READ, RMW_NONE, D_LAX, ST_NONE, I_NOP, 1'b0};  // LAX aby
			8'hC0: dv = {M_IMM, A_OTHER, RMW_NONE, D_CPY, ST_NONE, I_NOP, 1'b0};  // CPY imm
			8'hC1: dv = {M_IZX, A_READ, RMW_NONE, D_CMP, ST_NONE, I_NOP, 1'b0};  // CMP izx
			8'hC2: dv = {M_IMM, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP imm
			8'hC3: dv = {M_IZX, A_RMW, RMW_DEC, D_CMP, ST_NONE, I_NOP, 1'b0};  // DCP izx
			8'hC4: dv = {M_ZP, A_READ, RMW_NONE, D_CPY, ST_NONE, I_NOP, 1'b0};  // CPY zp
			8'hC5: dv = {M_ZP, A_READ, RMW_NONE, D_CMP, ST_NONE, I_NOP, 1'b0};  // CMP zp
			8'hC6: dv = {M_ZP, A_RMW, RMW_DEC, D_NONE, ST_NONE, I_NOP, 1'b0};  // DEC zp
			8'hC7: dv = {M_ZP, A_RMW, RMW_DEC, D_CMP, ST_NONE, I_NOP, 1'b0};  // DCP zp
			8'hC8: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_INY, 1'b0};  // INY imp
			8'hC9: dv = {M_IMM, A_OTHER, RMW_NONE, D_CMP, ST_NONE, I_NOP, 1'b0};  // CMP imm
			8'hCA: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_DEX, 1'b0};  // DEX imp
			8'hCB: dv = {M_IMM, A_OTHER, RMW_NONE, D_SBX, ST_NONE, I_NOP, 1'b0};  // SBX imm
			8'hCC: dv = {M_ABS, A_READ, RMW_NONE, D_CPY, ST_NONE, I_NOP, 1'b0};  // CPY abs
			8'hCD: dv = {M_ABS, A_READ, RMW_NONE, D_CMP, ST_NONE, I_NOP, 1'b0};  // CMP abs
			8'hCE: dv = {M_ABS, A_RMW, RMW_DEC, D_NONE, ST_NONE, I_NOP, 1'b0};  // DEC abs
			8'hCF: dv = {M_ABS, A_RMW, RMW_DEC, D_CMP, ST_NONE, I_NOP, 1'b0};  // DCP abs
			8'hD0: dv = {M_REL, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // BNE rel
			8'hD1: dv = {M_IZY, A_READ, RMW_NONE, D_CMP, ST_NONE, I_NOP, 1'b0};  // CMP izy
			8'hD2: dv = {M_JAM, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // JAM jam
			8'hD3: dv = {M_IZY, A_RMW, RMW_DEC, D_CMP, ST_NONE, I_NOP, 1'b0};  // DCP izy
			8'hD4: dv = {M_ZPX, A_READ, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP zpx
			8'hD5: dv = {M_ZPX, A_READ, RMW_NONE, D_CMP, ST_NONE, I_NOP, 1'b0};  // CMP zpx
			8'hD6: dv = {M_ZPX, A_RMW, RMW_DEC, D_NONE, ST_NONE, I_NOP, 1'b0};  // DEC zpx
			8'hD7: dv = {M_ZPX, A_RMW, RMW_DEC, D_CMP, ST_NONE, I_NOP, 1'b0};  // DCP zpx
			8'hD8: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_CLD, 1'b0};  // CLD imp
			8'hD9: dv = {M_ABY, A_READ, RMW_NONE, D_CMP, ST_NONE, I_NOP, 1'b0};  // CMP aby
			8'hDA: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP imp
			8'hDB: dv = {M_ABY, A_RMW, RMW_DEC, D_CMP, ST_NONE, I_NOP, 1'b0};  // DCP aby
			8'hDC: dv = {M_ABX, A_READ, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP abx
			8'hDD: dv = {M_ABX, A_READ, RMW_NONE, D_CMP, ST_NONE, I_NOP, 1'b0};  // CMP abx
			8'hDE: dv = {M_ABX, A_RMW, RMW_DEC, D_NONE, ST_NONE, I_NOP, 1'b0};  // DEC abx
			8'hDF: dv = {M_ABX, A_RMW, RMW_DEC, D_CMP, ST_NONE, I_NOP, 1'b0};  // DCP abx
			8'hE0: dv = {M_IMM, A_OTHER, RMW_NONE, D_CPX, ST_NONE, I_NOP, 1'b0};  // CPX imm
			8'hE1: dv = {M_IZX, A_READ, RMW_NONE, D_SBC, ST_NONE, I_NOP, 1'b0};  // SBC izx
			8'hE2: dv = {M_IMM, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP imm
			8'hE3: dv = {M_IZX, A_RMW, RMW_INC, D_SBC, ST_NONE, I_NOP, 1'b0};  // ISC izx
			8'hE4: dv = {M_ZP, A_READ, RMW_NONE, D_CPX, ST_NONE, I_NOP, 1'b0};  // CPX zp
			8'hE5: dv = {M_ZP, A_READ, RMW_NONE, D_SBC, ST_NONE, I_NOP, 1'b0};  // SBC zp
			8'hE6: dv = {M_ZP, A_RMW, RMW_INC, D_NONE, ST_NONE, I_NOP, 1'b0};  // INC zp
			8'hE7: dv = {M_ZP, A_RMW, RMW_INC, D_SBC, ST_NONE, I_NOP, 1'b0};  // ISC zp
			8'hE8: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_INX, 1'b0};  // INX imp
			8'hE9: dv = {M_IMM, A_OTHER, RMW_NONE, D_SBC, ST_NONE, I_NOP, 1'b0};  // SBC imm
			8'hEA: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP imp
			8'hEB: dv = {M_IMM, A_OTHER, RMW_NONE, D_SBC, ST_NONE, I_NOP, 1'b0};  // SBC imm
			8'hEC: dv = {M_ABS, A_READ, RMW_NONE, D_CPX, ST_NONE, I_NOP, 1'b0};  // CPX abs
			8'hED: dv = {M_ABS, A_READ, RMW_NONE, D_SBC, ST_NONE, I_NOP, 1'b0};  // SBC abs
			8'hEE: dv = {M_ABS, A_RMW, RMW_INC, D_NONE, ST_NONE, I_NOP, 1'b0};  // INC abs
			8'hEF: dv = {M_ABS, A_RMW, RMW_INC, D_SBC, ST_NONE, I_NOP, 1'b0};  // ISC abs
			8'hF0: dv = {M_REL, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // BEQ rel
			8'hF1: dv = {M_IZY, A_READ, RMW_NONE, D_SBC, ST_NONE, I_NOP, 1'b0};  // SBC izy
			8'hF2: dv = {M_JAM, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // JAM jam
			8'hF3: dv = {M_IZY, A_RMW, RMW_INC, D_SBC, ST_NONE, I_NOP, 1'b0};  // ISC izy
			8'hF4: dv = {M_ZPX, A_READ, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP zpx
			8'hF5: dv = {M_ZPX, A_READ, RMW_NONE, D_SBC, ST_NONE, I_NOP, 1'b0};  // SBC zpx
			8'hF6: dv = {M_ZPX, A_RMW, RMW_INC, D_NONE, ST_NONE, I_NOP, 1'b0};  // INC zpx
			8'hF7: dv = {M_ZPX, A_RMW, RMW_INC, D_SBC, ST_NONE, I_NOP, 1'b0};  // ISC zpx
			8'hF8: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_SED, 1'b0};  // SED imp
			8'hF9: dv = {M_ABY, A_READ, RMW_NONE, D_SBC, ST_NONE, I_NOP, 1'b0};  // SBC aby
			8'hFA: dv = {M_IMP, A_OTHER, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP imp
			8'hFB: dv = {M_ABY, A_RMW, RMW_INC, D_SBC, ST_NONE, I_NOP, 1'b0};  // ISC aby
			8'hFC: dv = {M_ABX, A_READ, RMW_NONE, D_NONE, ST_NONE, I_NOP, 1'b0};  // NOP abx
			8'hFD: dv = {M_ABX, A_READ, RMW_NONE, D_SBC, ST_NONE, I_NOP, 1'b0};  // SBC abx
			8'hFE: dv = {M_ABX, A_RMW, RMW_INC, D_NONE, ST_NONE, I_NOP, 1'b0};  // INC abx
			8'hFF: dv = {M_ABX, A_RMW, RMW_INC, D_SBC, ST_NONE, I_NOP, 1'b0};  // ISC abx
		endcase
	end

	assign d = dv;

endmodule
