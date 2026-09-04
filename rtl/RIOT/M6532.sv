// k7800 (c) by Jamie Blanks
//
// Copyright (c) 2021-2026 Jamie Blanks
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


module M6532
(
	input        clk,       // PHI 2
	input        ce,        // Clock enable
	input        res_n,     // reset
	input        ram_init,      // Load the console's power up RAM image
	input        ram_init_7800, // 1 = the 7800 BIOS loader, 0 = zeros
	input  [6:0] addr,      // Address
	input        RW_n,      // 1 = read, 0 = write
	input  [7:0] d_in,
	output       [7:0] d_out,
	input        RS_n,      // RAM select
	output       IRQ_n,
	output       IRQ_n_oe,  // IRQ pulls low, and is otherwise released
	input        CS1,       // Chip select 1, 1 = selected
	input        CS2_n,     // Chip select 2, 0 = selected
	input  [7:0] PA_in,     // Port ins and outs
	output [7:0] PA_out,    // NOTE that port output must be fed back to input
	input  [7:0] PB_in,     // if not altered by a peripheral, in order for
	output [7:0] PB_out,    // the chip to read properly!
	output [7:0] oe,        // Which of D7:D0 the chip is actually driving
	output       PA_read    // A selected read of ORA is happening now
);

reg [7:0] out_a, out_b, data;
reg [7:0] dir_a, dir_b;
reg [7:0] interrupt;
reg [7:0] timer;
reg [9:0] prescaler;

reg [1:0] incr;
logic rollover;
reg [1:0] irq_en;
reg edge_detect;
reg old_pa7;

reg [6:0] init_addr;

// The 128 byte RAM lives in the project block RAM wrapper. Its q lands one
// clock after the address, which is where the old array read landed, so
// d_out takes it live for that one cycle and d_out_reg holds it after.
wire [7:0] ram_q;
reg  [7:0] d_out_reg;
reg        ram_read_r;

// The 7800 BIOS leaves this loader in RIOT RAM before it starts a cart, so
// bypassing the BIOS has to put it there. Byte 8 and the last two bytes are
// deliberate Decathlon workarounds.
wire [7:0] init_image [128] = '{
	8'hA9, 8'h00, 8'hAA, 8'h85, 8'h01, 8'h95, 8'h03, 8'hE8, 8'h00, 8'h2A, 8'hD0, 8'hF9, 8'h85, 8'h02, 8'hA9, 8'h04,
	8'hEA, 8'h30, 8'h23, 8'hA2, 8'h04, 8'hCA, 8'h10, 8'hFD, 8'h9A, 8'h8D, 8'h10, 8'h01, 8'h20, 8'hCB, 8'h04, 8'h20,
	8'hCB, 8'h04, 8'h85, 8'h11, 8'h85, 8'h1B, 8'h85, 8'h1C, 8'h85, 8'h0F, 8'hEA, 8'h85, 8'h02, 8'hA9, 8'h00, 8'hEA,
	8'h30, 8'h04, 8'h24, 8'h03, 8'h30, 8'h09, 8'hA9, 8'h02, 8'h85, 8'h09, 8'h8D, 8'h12, 8'hF1, 8'hD0, 8'h1E, 8'h24,
	8'h02, 8'h30, 8'h0C, 8'hA9, 8'h02, 8'h85, 8'h06, 8'h8D, 8'h18, 8'hF1, 8'h8D, 8'h60, 8'hF4, 8'hD0, 8'h0E, 8'h85,
	8'h2C, 8'hA9, 8'h08, 8'h85, 8'h1B, 8'h20, 8'hCB, 8'h04, 8'hEA, 8'h24, 8'h02, 8'h30, 8'hD9, 8'hA9, 8'hFD, 8'h85,
	8'h08, 8'h6C, 8'hFC, 8'hFF, 8'hEA, 8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'hFF,
	8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'h00, 8'h00
};

// 128 bytes is over the AGENTS.md threshold for an inferred array, so the RAM
// is a wrapper instance. Port A is the CPU side and port B the power up walk;
// they never write together because the console drives ram_init from its
// reset, which is the same signal that holds res_n low.
wire       ram_cpu_wr = res_n && ce && (CS1 & ~CS2_n) && ~RS_n && ~RW_n;
wire [7:0] ram_init_d = ram_init_7800 ? init_image[init_addr] : 8'h00;

cache_ram_dp #(.ADDR_WIDTH(7), .DATA_WIDTH(8)) riot_ram
(
	.clk_i     (clk),
	.addr_a_i  (addr),
	.wren_a_i  (ram_cpu_wr),
	.wdata_a_i (d_in),
	.q_a_o     (ram_q),
	.addr_b_i  (init_addr),
	.wren_b_i  (ram_init),
	.wdata_b_i (ram_init_d),
	.q_b_o     ()
);

// IRQ is open drain on the part: it pulls the line low and otherwise releases
// it to an external pull up. IRQ_n is the resolved level for internal use.
wire irq_active = (interrupt[7] & irq_en[1]) | (interrupt[6] & irq_en[0]);
assign IRQ_n = ~irq_active;
assign IRQ_n_oe = irq_active;

// These wires have a weak pull up, so any undriven wires will be high
assign PA_out = out_a | ~dir_a;
assign PB_out = out_b | ~dir_b;

// The edge detector hangs on the PA7 pin, so it sees the resolved level: the
// same wired AND of drive and feedback that a port A read returns.
wire pa7 = PA_in[7] & PA_out[7];
wire p1 = incr == 2'd0 || rollover;
wire p8 = ~|prescaler[2:0] && incr == 2'd1;
wire p64 = ~|prescaler[5:0] && incr == 2'd2;
wire p1024 = ~|prescaler[9:0] && incr == 2'd3;
wire tick_inc = p1 || p8 || p64 || p1024;

// Both flags are set by the same edge the CPU latches the bus on, so an IFR
// read landing on that cycle has to carry them - the same reason the counter
// is read out one ahead below. Named here so the read and the flag itself
// cannot drift apart.
wire timer_wrap = tick_inc && timer == 0;
wire pa7_edge = (edge_detect && ~old_pa7 && pa7) || (~edge_detect && old_pa7 && ~pa7);

// RES turns the data bus off, and the part drives it only through phase 2 of a
// selected read. Phase 2 here is the one clk_sys cycle ce marks, which is the
// cycle the CPU samples in. All eight lines drive together: even the interrupt
// flag read, which carries flags on D7 and D6 only, pulls D5:D0 to 0 rather
// than leaving them floating (R6532 sheet, Interrupt Flag Register).
wire drive = res_n && (CS1 & ~CS2_n) && RW_n && ce;
assign oe = {8{drive}};

// A peripheral that a read consumes - a trackball step, a paddle dump -
// needs to know when ORA is on the bus. ORA is RS=1, A2=0, A1:A0=00, with
// A4:A3 don't care, so $0280, $0288, $0290 and $0298 are all the same read.
assign PA_read = drive && RS_n && ~addr[2] && ~|addr[1:0];

assign d_out = ram_read_r ? ram_q : d_out_reg;

always_ff @(posedge clk) begin
	// A selected RAM read this cycle puts the wrapper's answer on the bus next
	// cycle; capturing it as well keeps it there once the selection goes away.
	ram_read_r <= res_n && (CS1 & ~CS2_n) && RW_n && ~RS_n;
	if (ram_read_r)
		d_out_reg <= ram_q;

	if ((CS1 & ~CS2_n) && RW_n && RS_n) begin
		if (~addr[2]) begin // Address registers
			case(addr[1:0])
				2'b01: d_out_reg <= dir_a; // DDRA
				2'b11: d_out_reg <= dir_b; // DDRB
				2'b00: d_out_reg <= (PA_in & PA_out); // Input A, always the pin
				// Port B is the exception: a bit set to output reads its own
				// output register back, so an outside low cannot mask a
				// driven high. DDRB=$01, ORB=$01, PB pins $FE reads $FF.
				2'b10: d_out_reg <= (dir_b & out_b) | (~dir_b & PB_in);
			endcase
		end else begin // Timer & Interrupts
			// The counter takes its new value at the end of this cycle and
			// the bus already carries it: write 52 at /8, and the first
			// read after that is 51, not 52.
			if (~addr[0])
				d_out_reg <= tick_inc ? timer - 8'd1 : timer;
			else
				d_out_reg <= {interrupt[7] | timer_wrap, interrupt[6] | pa7_edge, 6'd0};
		end
	end
	if (~res_n) begin
		d_out_reg <= 8'hFF;
		ram_read_r <= 1'b0;
	end
end

always_ff @(posedge clk) begin
	// RES leaves the 128 bytes of RAM alone. This is the console's power up
	// image instead, and the top level decides when a console applies it. It is
	// written a byte a cycle for as long as ram_init is held, so ram_init has to
	// stay up for at least 128 clocks - a console's reset line is held orders of
	// magnitude longer. Loading all 128 at once would cost 1024 flops with a set
	// or a reset on every one.
	if (ram_init) begin
		init_addr <= init_addr + 1'd1;
	end else begin
		init_addr <= 7'd0;
	end

	if (~res_n) begin
		out_a <= 8'h00;
		out_b <= 8'h00;
		dir_a <= 8'h00;
		dir_b <= 8'h00;
		{interrupt, irq_en, edge_detect} <= '0;
		old_pa7 <= pa7;   // RES reports no edge for the level it started on
		incr <= 2'b10; // Increment resets to 64
		timer <= 8'hFF;   // Timer resets to FF
		prescaler <= 0;
		rollover <= 0;
	end else if (ce) begin
		prescaler <= prescaler + 1'd1;

		if (tick_inc)
			timer <= timer - 8'd1;

		if (CS1 & ~CS2_n) begin
			if (~RS_n) begin // RAM selected, written through the wrapper above
			end else if (~addr[2]) begin // Address registers
				if (~RW_n) begin
					case(addr[1:0])
						2'b01: dir_a <= d_in; // DDRA
						2'b11: dir_b <= d_in; // DDRB
						2'b00: out_a <= d_in; // Output A
						2'b10: out_b <= d_in; // Output B
					endcase
				end
			end else begin // Timer & Interrupts
				if (~RW_n) begin
					if (addr[4])begin
						prescaler <= 10'd0;
						rollover <= 0;
						interrupt[7] <= 0;
						incr <= addr[1:0];
						timer <= d_in;
						irq_en[1] <= addr[3];
					end else begin
						irq_en[0] <= addr[1];
						edge_detect <= addr[0];
					end
				end else begin
					if (~addr[0]) begin
						irq_en[1] <= addr[3];
						rollover <= 0;
						interrupt[7] <= 0;
					end else
						interrupt[6] <= 0;
				end
			end
		end

		if (timer_wrap) begin
			interrupt[7] <= 1;
			rollover <= 1;
		end

		// Edge detection
		old_pa7 <= pa7;
		if (pa7_edge)
			interrupt[6] <= 1;
	end
end

endmodule
