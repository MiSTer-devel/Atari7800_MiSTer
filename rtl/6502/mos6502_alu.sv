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
// MOS 6502 ALU and the decimal adjust adders.
//
// Structure follows the visual6502 netlist.
//
// Two notes on polarity and placement, because both look wrong at first:
//
//  * The silicon's ALU emits the complement of its result and the ADD hold
//    register inverts it again on phi2. The pair cancels, so this module
//    carries the true value throughout.
//
//  * The decimal adjust is NOT inside the ALU. It sits on the SB -> A path
//    only, so the uncorrected result is what reaches memory, ADL and DB.
//    That is the structural reason the BCD flag anomalies happen: N comes
//    from the uncorrected bit 7, Z from the uncorrected byte, and only the
//    accumulator ever sees the corrected value.
//============================================================================

// The ALU proper. Combinational; the ADD register that captures `res` lives
// in the datapath.
module mos6502_alu #(
	parameter bit BCD_EN = 1    // 0 removes decimal mode entirely (NES 2A03)
) (
	input  logic [7:0] ai,      // A side: SB, or 0 when 0/ADD is asserted
	input  logic [7:0] bi,      // B side: DB, ~DB, or ADL

	// One-hot operation select. No left shift exists: ASL and ROL are done
	// as SUMS with the same value on both sides, and SRS is fed the value on
	// both sides too, so A&B is just the operand.
	input  logic       sums,    // A + B + carry in
	input  logic       ands,    // A & B
	input  logic       eors,    // A ^ B
	input  logic       ors,     // A | B
	input  logic       srs,     // (A & B) >> 1
	input  logic       cin,     // 1/ADDC

	// Only DAA reaches the ALU. DSA is a decimal adjust signal only: decimal
	// SBC leaves the carry chain alone and corrects entirely by subtraction.
	input  logic       daa,     // decimal add: D flag and ADC

	output logic [7:0] res,     // -> ADD hold register
	output logic       acr,     // carry out, with the decimal carry folded in
	output logic       avr,     // overflow, from the carries around bit 7
	output logic       hc       // C34: carry into bit 4, decimal carry included
);

	logic [4:0] lo;             // {carry out of bit 3, low nibble sum}
	logic [4:0] hi;             // {carry out of bit 7, high nibble sum}
	logic [7:0] sum;
	logic       dc34;           // low nibble needed correcting

	// The chain is split at the nibble boundary because the decimal half
	// carry is injected into the chain itself rather than added afterwards.
	// Each half is a plain addition so Quartus uses the carry chain.
	assign lo = {1'b0, ai[3:0]} + {1'b0, bi[3:0]} + {4'b0, cin};

	// A low nibble above 9 forces the carry into bit 4. This is what makes
	// the carry flag come out right in decimal mode.
	assign dc34 = BCD_EN && daa && (lo[4] || (lo[3] && (lo[2] || lo[1])));
	assign hc   = lo[4] | dc34;

	assign hi   = {1'b0, ai[7:4]} + {1'b0, bi[7:4]} + {4'b0, hc};
	assign sum  = {hi[3:0], lo[3:0]};
	// High nibble above 9 makes the carry out. Unlike DC34 this one does not
	// re-enter the chain, it only reaches the C flag and the adjust block.
	logic dc78;
	assign dc78 = BCD_EN && daa && (hi[4] || (sum[7] && (sum[6] || sum[5])));

	// AVR is the carry into bit 7 against the carry out of it, which for a
	// two's complement add is the same as "both operands agreed on sign and
	// the result disagreed". It uses the binary carry out, not the decimal
	// one: V after a decimal ADC comes from this intermediate, before the
	// high nibble is corrected.
	assign avr = ~(ai[7] ^ bi[7]) & (ai[7] ^ sum[7]);

	// Only SUMS and SRS produce a meaningful carry. The control logic does
	// not route ACR to the C flag for the logic operations.
	assign acr = srs ? (ai[0] & bi[0]) : (hi[4] | dc78);

	always_comb begin
		unique case (1'b1)
			sums:    res = sum;
			ands:    res = ai & bi;
			eors:    res = ai ^ bi;
			ors:     res = ai | bi;
			srs:     res = {1'b0, ai[7:1] & bi[7:1]};
			default: res = 8'hFF;   // buses precharge high; nothing driving
		endcase
	end

endmodule


// The decimal adjust adders, on the SB -> A path only.
//
// A conditional inverter bank, not a second adder. Adding 6 to a nibble is
// adding 3 to that nibble's top three bits, and adding or subtracting 3 from
// a three bit field is a fixed flip pattern:
//
//   +3 on {b3,b2,b1}:  b1 always,  b2 when ~b1,      b3 when b1|b2
//   -3 on {b3,b2,b1}:  b1 always,  b2 when  b1,      b3 when ~(b1&b2)
//
// Bits 0 and 4 are never adjusted, which is why only six of the eight bits
// pass through this block in the silicon.
module mos6502_daa #(
	parameter bit BCD_EN = 1
) (
	input  logic [7:0] sb,      // uncorrected value on SB
	input  logic       daa,     // decimal add
	input  logic       dsa,     // decimal subtract
	input  logic       hc,      // C34 from the ALU
	input  logic       acr,     // ALU carry out
	output logic [7:0] out      // corrected value, into A
);

	logic add_lo, sub_lo, add_hi, sub_hi;

	// Add corrects the nibble that carried; subtract corrects the one that
	// borrowed, so the two conditions are complements of each other.
	assign add_lo = BCD_EN && daa && hc;
	assign sub_lo = BCD_EN && dsa && !hc;
	assign add_hi = BCD_EN && daa && acr;
	assign sub_hi = BCD_EN && dsa && !acr;

	always_comb begin
		out = sb;

		out[1] = sb[1] ^ (add_lo | sub_lo);
		out[2] = sb[2] ^ ((add_lo & ~sb[1]) | (sub_lo &  sb[1]));
		out[3] = sb[3] ^ ((add_lo &  (sb[1] | sb[2])) |
		                  (sub_lo & ~(sb[1] & sb[2])));

		out[5] = sb[5] ^ (add_hi | sub_hi);
		out[6] = sb[6] ^ ((add_hi & ~sb[5]) | (sub_hi &  sb[5]));
		out[7] = sb[7] ^ ((add_hi &  (sb[5] | sb[6])) |
		                  (sub_hi & ~(sb[5] & sb[6])));
	end

endmodule
