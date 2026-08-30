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

module line_ram(
	input  logic               clk_sys, 
	input  logic               RESET,
	output logic [7:0]         PLAYBACK,
	// Databus inputs
	input  logic [2:0]         PALETTE,
	input  logic [7:0]         d_in,
	input  logic               WM,
	input  logic               border,
	// Write enable for databus inputs
	input  logic               latch_byte,
	input  logic               latch_hpos,
	// Memory mapped registers
	input  logic [24:0][7:0]   COLOR_MAP,
	input  logic [1:0]         RM,
	input  logic               KANGAROO_MODE,
	input  logic               BORDER_CONTROL,
	input  logic               COLOR_KILL,
	input  logic               lrc,
	// VGA Control signal
	input  logic [8:0]         LRAM_OUT_COL,
	input  logic               mclk0,
	input  logic               mclk1,
	input  logic               cram_write
);

// Two things argue for keeping the whole line RAM in flops: MARIA writes up to
// four cells at once, and the real part flash-clears the buffer at the start of
// a line. Neither needs fabric for the cell data itself.
//
//   Four at once - consecutive cell addresses always differ in their low two
//   bits, so banking on those hands each bank exactly one of the four writes
//   whatever hpos is. Four ordinary one-write, one-read memories.
//
//   Flash clear - a cleared cell and a cell nobody wrote are the same thing, a
//   background pixel. So the clear is not a clear, it is one bit per cell
//   saying whether this line wrote it, and clearing 160 flops at once is what
//   flops are good at. Only those bits stay in fabric.
//
//     cell address:  [7:2] row, 0..39     [1:0] bank
//     bank 0   bank 1   bank 2   bank 3     each 40 rows x 2 buffers, one M10K
//     └────────┴────────┴────────┴── one write each, rotated by hpos[1:0]
localparam int ROWS = 40;            // 160 cells across four banks

logic        wr_buf;                 // the buffer MARIA is filling
// Rows 0..39 are the line; 40..63 are the scratch the row address lands on when
// the write pointer runs past the end, which is what MARIA's own extra word
// line WL1F is for. Every eight-bit write address therefore has a real home and
// nothing is ever indexed out of range.
logic [63:0] written [0:1][0:3];     // occupancy, [buffer][bank]

logic [2:0] playback_palette;
logic [1:0] playback_color;
logic [4:0] playback_cell;
logic [8:0] playback_ix;
logic [7:0] lram_ix;
logic [7:0] hpos;

// The four cells a byte can touch, before they are handed to banks.
logic [3:0][4:0] cell_data;
logic      [3:0] cell_wr;

logic [3:0][1:0] bank_sel;
logic [3:0][5:0] bank_row;
logic [3:0][4:0] bank_data;
logic      [3:0] bank_wr;
wire  [3:0][4:0] bank_q;
wire  [3:0][4:0] bank_unused;

wire byte_now = ~RESET & mclk0 & latch_byte;
wire swap_now = ~RESET & mclk0 & lrc;
wire wr_buf_next = swap_now ? ~wr_buf : wr_buf;

// The read address is the index the playback counter is about to take, so the
// registered output always lines up with the counter however mclk0 is spaced.
wire [8:0] playback_ix_next = mclk0 ? (border ? 9'd0 : playback_ix + 9'd1)
                                    : playback_ix;
wire [7:0] lram_ix_next = playback_ix_next[8:1];
wire [5:0] rd_row = lram_ix[7:2];
// A read past the end of the line selects no word line at all, so it reads as
// background rather than as whatever the scratch rows happen to hold.
wire       cell_present = rd_row < 6'(ROWS) &&
	written[~wr_buf][lram_ix[1:0]][rd_row];
wire [4:0] cell_read = cell_present ? bank_q[lram_ix[1:0]] : 5'd0;

logic [5:0] pb_map_index[8];
assign pb_map_index = '{5'd0, 5'd3, 5'd6, 5'd9, 5'd12, 5'd15, 5'd18, 5'd21};

always @(posedge clk_sys) begin
	if (mclk0) begin
		if (~border)
			playback_ix <= playback_ix + 1'd1;
		else
			playback_ix <= 0;
	end
	// The color ram will be delayed by one if there's a cram write
	// Uncomment to enable this bug (it's ugly)
	if (mclk0 /*&& ~cram_write*/) begin
		if (playback_color == 2'b0 || border) begin
			PLAYBACK <= (border & ~BORDER_CONTROL) ? 8'd0 : COLOR_MAP[0];
		end else begin
			PLAYBACK <= COLOR_MAP[pb_map_index[playback_palette] + playback_color];
		end
	end

end

always_comb begin
	lram_ix = playback_ix[8:1]; // 2 pixels per lram cell
	playback_cell = cell_read;
	playback_palette = playback_cell[4:2]; // Default to 160A/B
	playback_color = playback_cell[1:0];
	casex (RM)
		2'b0x: begin
			// 160A is read as four double-pixels per byte:
			//      <P2 P1 P0> <D7 D6>
			//      <P2 P1 P0> <D5 D4>
			//      <P2 P1 P0> <D3 D2>
			//      <P2 P1 P0> <D1 D0>
			// 160B is read as two double-pixels per byte:
			//      <P2 D3 D2> <D7 D6>
			//      <P2 D1 D0> <D5 D4>
			// In both cases, the lineram cells are stored in
			// exactly the order specified above. They can be
			// read directly.
			playback_palette = playback_cell[4:2];
			playback_color = playback_cell[1:0];
		end
		2'b10: begin
			// 320B is read as four pixels per byte:
			//      <P2  0  0> <D7 D3>
			//      <P2  0  0> <D6 D2>
			//      <P2  0  0> <D5 D1>
			//      <P2  0  0> <D4 D0>
			// 320B is stored as two cells per byte (wm=1):
			//      [P2 D3 D2 D7 D6]
			//      [P2 D1 D0 D5 D4]
			//
			// 320D is read as eight pixels per byte:
			//      <P2  0  0> <D7 P1>
			//      <P2  0  0> <D6 P0>
			//      <P2  0  0> <D5 P1>
			//      <P2  0  0> <D4 P0>
			//      <P2  0  0> <D3 P1>
			//      <P2  0  0> <D2 P0>
			//      <P2  0  0> <D1 P1>
			//      <P2  0  0> <D0 P0>
			// 320D is stored as four cells per byte (wm=0):
			//      [P2 P1 P0 D7 D6]
			//      [P2 P1 P0 D5 D4]
			//      [P2 P1 P0 D3 D2]
			//      [P2 P1 P0 D1 D0]
			//
			// In both cases, the palette is always <cell[4], 0, 0>
			// For a given pair of pixels, the color selectors
			// are, from left to right, <cell[1], cell[3]> and <cell[0], cell[2]>
			// Example: Either D7,D3:D6,D2 (320B) or D7,P1:D6,P0 (320D)
			playback_palette = {playback_cell[4], 2'b0};
			if (playback_ix[0]) begin
				// Right pixel
				playback_color = {playback_cell[0], playback_cell[2]};
			end else begin
				// Left pixel
				playback_color = {playback_cell[1], playback_cell[3]};
			end
		end
		2'b11: begin
			// 320A is read as eight pixels per byte:
			//      <P2 P1 P0> <D7  0>
			//      <P2 P1 P0> <D6  0>
			//      <P2 P1 P0> <D5  0>
			//      <P2 P1 P0> <D4  0>
			//      <P2 P1 P0> <D3  0>
			//      <P2 P1 P0> <D2  0>
			//      <P2 P1 P0> <D1  0>
			//      <P2 P1 P0> <D0  0>
			// 320A is stored as four cells per byte (wm=0):
			//      [P2 P1 P0 D7 D6]
			//      [P2 P1 P0 D5 D4]
			//      [P2 P1 P0 D3 D2]
			//      [P2 P1 P0 D1 D0]
			//
			// 320C is read as four pixels per byte:
			//      <P2 D3 D2> <D7  0>
			//      <P2 D3 D2> <D6  0>
			//      <P2 D1 D0> <D5  0>
			//      <P2 D1 D0> <D4  0>
			// 320C is stored as two cells per byte (wm=1):
			//      [P2 D3 D2 D7 D6]
			//      [P2 D1 D0 D5 D4]
			//
			// In both cases, the palette is always <cell[4], cell[3], cell[2]>
			// For a given pair of pixels, the color selectors
			// are, from left to right, <cell[1], 0> and <cell[0], 0>
			playback_palette = playback_cell[4:2];
			if (playback_ix[0]) begin
				// Right pixel
				playback_color = {playback_cell[0], 1'b0};
			end else begin
				// Left pixel
				playback_color = {playback_cell[1], 1'b0};
			end
		end
	endcase
end

always_comb begin
	for (int j = 0; j < 4; j++) begin
		cell_data[j] = 5'd0;
		cell_wr[j]   = 1'b0;
	end

	case (WM)
		1'b0: begin
			// "When wm = 0, each byte specifies four pixel cells
			//  of the lineram"
			// This encompasses:
			// 160A:
			//      [P2 P1 P0 D7 D6]
			//      [P2 P1 P0 D5 D4]
			//      [P2 P1 P0 D3 D2]
			//      [P2 P1 P0 D1 D0]
			// 320A:
			//      [P2 P1 P0 D7  0]
			//      [P2 P1 P0 D6  0]
			//      [P2 P1 P0 D5  0]
			//      [P2 P1 P0 D4  0]
			//      [P2 P1 P0 D3  0]
			//      [P2 P1 P0 D2  0]
			//      [P2 P1 P0 D1  0]
			//      [P2 P1 P0 D0  0]
			// 320D:
			//      [P2  0  0 D7 P1]
			//      [P2  0  0 D6 P0]
			//      [P2  0  0 D5 P1]
			//      [P2  0  0 D4 P0]
			//      [P2  0  0 D3 P1]
			//      [P2  0  0 D2 P0]
			//      [P2  0  0 D1 P1]
			//      [P2  0  0 D0 P0]
			// These can all be written into the cells using
			// the same format and read out differently.
			cell_wr[0] = |d_in[7:6] || KANGAROO_MODE;
			cell_data[0] = {PALETTE, d_in[7:6]};
			cell_wr[1] = |d_in[5:4] || KANGAROO_MODE;
			cell_data[1] = {PALETTE, d_in[5:4]};
			cell_wr[2] = |d_in[3:2] || KANGAROO_MODE;
			cell_data[2] = {PALETTE, d_in[3:2]};
			cell_wr[3] = |d_in[1:0] || KANGAROO_MODE;
			cell_data[3] = {PALETTE, d_in[1:0]};
		end
		1'b1: begin
			// "When wm = 1, each byte specifies two cells within the lineram."
			// This encompasses:
			// 160B:
			//      [P2 D3 D2 D7 D6]
			//      [P2 D1 D0 D5 D4]
			// 320B:
			//      [P2  0  0 D7 D3]
			//      [P2  0  0 D6 D2]
			//      [P2  0  0 D5 D1]
			//      [P2  0  0 D4 D0]
			// 320C:
			//      [P2 D3 D2 D7  0]
			//      [P2 D3 D2 D6  0]
			//      [P2 D1 D0 D5  0]
			//      [P2 D1 D0 D4  0]
			// Again, these can be written into the cells in
			// the same format and read out differently. Note:
			// transparency may not be correct in 320B mode here
			// since the color bits are different than 160B and 320C.
			cell_wr[0] = |d_in[7:6] || KANGAROO_MODE;
			cell_data[0] = {PALETTE[2], d_in[3:2], d_in[7:6]};
			cell_wr[1] = |d_in[5:4] || KANGAROO_MODE;
			cell_data[1] = {PALETTE[2], d_in[1:0], d_in[5:4]};
		end
		endcase
end


// hpos advances on a byte, or is loaded by the display list. A byte latched in
// the same cycle wins and advances from the old position.
always_ff @(posedge clk_sys) begin
	if (RESET)
		hpos <= 8'd0;
	else if (mclk0) begin
		if (latch_hpos)
			hpos <= d_in;
		if (latch_byte)
			hpos <= hpos + (WM ? 8'd2 : 8'd4);
	end
end

// Hand each bank the one cell of the four whose address ends in that bank.
always_comb begin
	for (int b = 0; b < 4; b++) begin
		bank_sel[b]  = 2'(b) - hpos[1:0];
		bank_row[b]  = 6'((hpos + 8'(bank_sel[b])) >> 2);
		bank_data[b] = cell_data[bank_sel[b]];
		bank_wr[b]   = byte_now && cell_wr[bank_sel[b]];
	end
end

// A byte latched on the swap cycle belongs to the buffer being cleared, so it
// goes to wr_buf_next and sets its occupancy bit after the clear.
always_ff @(posedge clk_sys) begin
	wr_buf <= wr_buf_next;
	for (int b = 0; b < 4; b++) begin
		if (swap_now)
			written[wr_buf_next][b] <= 64'd0;
		if (bank_wr[b])
			written[wr_buf_next][b][bank_row[b]] <= 1'b1;
	end
end

generate
	genvar b;
	for (b = 0; b < 4; b = b + 1) begin : bank
		cache_ram_dp #(.ADDR_WIDTH(7), .DATA_WIDTH(5)) cells (
			.clk_i     (clk_sys),
			.addr_a_i  ({wr_buf_next, bank_row[b]}),
			.wren_a_i  (bank_wr[b]),
			.wdata_a_i (bank_data[b]),
			.q_a_o     (bank_unused[b]),
			.addr_b_i  ({~wr_buf_next, lram_ix_next[7:2]}),
			.wren_b_i  (1'b0),
			.wdata_b_i (5'd0),
			.q_b_o     (bank_q[b])
		);
	end
endgenerate

endmodule
