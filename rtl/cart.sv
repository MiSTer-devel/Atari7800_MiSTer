// SPDX-License-Identifier: MIT
// Copyright (c) 2019-2026 Jamie Blanks


// Covers the bank switching, ram, and audio hardware from carts
module cart
(
	input  logic        clk_sys,
	input  logic        pclk0,
	input  logic        pclk1,
	input  logic [15:0] address_in,
	input  logic [7:0]  din,
	input  logic        halt_n,
	input  logic [7:0]  rom_din,
	input  logic [15:0] cart_flags,
	input  logic [7:0]  cart_mapper,	// A78 v4 header byte 64; 5 = ACE
	input  logic [31:0] cart_size,
	input  logic [7:0]  cart_save,
	input  logic        cart_cs,
	input  logic        rw, // Write low
	input  logic        reset,
	input  logic        hsc_en,
	input  logic  [7:0] hsc_ram_din,
	input  logic  [7:0] cart_xm,
	input  logic [10:0] ps2_key,
	input  logic        pokey_irq_en,

	// Souper audio command port, one event per $8007 write pair.
	output logic        aud_cmd_valid,
	output logic  [7:0] aud_cmd_data,
	input  logic        minnie_en,
	input  logic        minnie_alt,
	input  logic [7:0]  cartram_data,

	output logic        IRQ_n,
	output logic [7:0]  dout,
	output logic [7:0]  dout_oe,	// which of dout's lines the slot is driving
	output logic        hsc_ram_cs,
	output logic        cart_read,
	output logic [15:0] pokey_audio_r,
	output logic [15:0] pokey_audio_l,
	output logic [15:0] minnie_audio,
	output logic [15:0] ym_audio_r,
	output logic [15:0] ym_audio_l,
	output logic [15:0] covox_r,
	output logic [15:0] covox_l,
	output logic [15:0] sn_audio,
	output logic        external_audio,
	output logic [24:0] rom_address,
	output logic [17:0] cartram_addr,
	output logic        cartram_wr,
	output logic        cartram_rd,
	output logic [7:0]  cartram_wrdata
);

logic [7:0] bank_reg;
logic [7:0] ram_dout;
logic [7:0] ym_dout;
logic [7:0] hsc_rom_dout;
logic [7:0] hsc_ram_dout;
logic [7:0] pokey4k_dout, pokey2_dout;
logic [7:0] minnie_dout;
logic       minnie_doe;
logic       minnie_active;

logic rom_cs, ram_cs, pokey_cs, ym_cs;
logic [2:0] hardware_map[16];
logic [7:0] bank_map[16];
logic [2:0] bank_type; // 00 = Supergame, 01 = Activision, 02 = none 03 = absolute 04 = souper 05 = ACE
logic [31:0] address_offset;
logic [2:0] cart_cs_reg, cart_cs_reg_m;
logic [7:0] bank_mask;
logic [16:0] ram_mask;
logic [7:0] XCTRL1, XCTRL2, XCTRL3, XCTRL4, XCTRL5; // 2-5 currently unused
logic souper_ram_cs;
logic [24:0] souper_addr;
wire souper_en = cart_flags[12];
logic [11:0] souper_bank;
logic [2:0] ram_bank;
logic [8:0] bs_map;
logic [1:0] bankset_count;
logic souper_wr;
logic pokey_irq_n, ym_irq_n;

logic [31:0] cart_size_bs;

wire XCTRL1_cs = (cart_xm[0] && address_in[15:4] == 8'h47) && cart_cs;
assign cart_read = rw && cart_cs && ~cartram_cs;
always @(posedge clk_sys) begin
	if (reset) begin
		XCTRL1 <= 0;
	end else if (pclk0) begin
		if (XCTRL1_cs && ~rw)
		case (address_in[3:0]) // FIXME: ATM there seems not much reason to support anything more than ctrl1
			4'h0: XCTRL1 <= din;
			// 4'h8: XCTRL2 <= din;
			// 4'hC: XCTRL3 <= din;
			// 4'h1: XCTRL4 <= din;
			// 4'h2: XCTRL5 <= din;
		endcase
	end
end


wire is_9b = cart_flags[3];
wire is_bankset = cart_flags[13];
wire is_bankset_mem = cart_flags[14];
wire bankset_banks = is_bankset & bankset_count[1];
assign cart_size_bs = is_bankset ? (cart_size >> 1'd1) : cart_size;

wire [7:0] num_banks = cart_size_bs[21:14];
wire [7:0] highest_bank = |num_banks ? num_banks - 1'd1 : is_9b;
wire [7:0] second_highest_bank = is_9b ? 8'd0 : (|highest_bank ? highest_bank - 1'd1 : 8'd0);
wire [7:0] sg_bank = (bank_reg & bank_mask) + is_9b;
wire is_bankset_52k = (is_bankset && cart_size_bs == 32'hD000);

// ACE, the SN Cart board. $8000-$FFFF is eight 4K pages; four are fixed at the
// EPROM's first 32K and four move. A window moves when a bank number is written
// to that window's base address. $A000/$C000/$E000 take a six bit bank and a two
// bit read transform in D7:D6; $D000 takes a seven bit bank and no transform.
// The power-up values are the linear map, wired into the register bits on the
// board. See references/SNCartDemo/Cartridge_512kb_4kb_bankswitch/Info.txt.
wire is_ace = cart_mapper == 8'd5;
logic [7:0] ace_a, ace_c, ace_d, ace_e;	// $A000 $C000 $D000 $E000
logic [2:0] ace_ram;			// $FFFF: RAM bank, force A8 low, force A9 low
logic [1:0] ace_swap;
logic [7:0] ace_dout;

// A bankset cart serves MARIA a different 48K than it serves SALLY and picks
// between them off /HALT. All it has to go on is what the slot carries: /HALT
// on J1-2 and phi2 on J1-32. Nothing tells it when SALLY actually lets go of
// the bus, so it has to count the handover out for itself.
//
// It cannot switch on the /HALT edge: SALLY keeps the bus for two more cycles
// after /HALT drops, and a cart that switched early would feed the CPU the
// MARIA bank for those two cycles - the "brief execution from the second
// bankset" failure in the test suite's README. Two cycles here is the same
// count SALLY's own flip-flop pair makes.
//
// phi2 is the count's clock because phi2 is the clock the cartridge has.
// Counting on phi1 put the switch a phase late - MARIA was already driving the
// address bus while this still selected SALLY's bank - and no real cart could
// do it anyway, since phi1 never reaches the connector.
always_ff @(posedge clk_sys) if (pclk0) begin
	if (~halt_n) begin
		if (is_bankset && ~bankset_count[1])
			bankset_count <= bankset_count + 1'd1;
	end else begin
		bankset_count <= 2'd0;
	end
end

always_ff @(posedge clk_sys) if (pclk1) begin
	hardware_map <= '{3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0};
	bank_map <= '{8'd0, 8'd0, 8'd0, 8'd0, 8'd0, 8'd0, 8'd0, 8'd0, 8'd0, 8'd0, 8'd0, 8'd0, 8'd0, 8'd0, 8'd0, 8'd0};
	bank_type <= 3'd0;
	address_offset <= 32'd0;
	bank_mask <= 8'b11111111;
	ram_mask <= '1;
	bs_map <= 9'd0;

	// Banking mode selector
	if (is_ace) begin                                          // ACE
		hardware_map <= '{3'd0, 3'd0, 3'd0, 3'd0, 3'd3, 3'd3, 3'd3, 3'd3, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4};
		bank_map <= '{8'd0, 8'd0, 8'd0, 8'd0, 8'd0, 8'd0, 8'd0, 8'd0,
			8'd0, 8'd1, {2'd0, ace_a[5:0]}, 8'd3,
			{2'd0, ace_c[5:0]}, {1'd0, ace_d[6:0]}, {2'd0, ace_e[5:0]}, 8'd7};
		bank_type <= 3'd5;
	end else if (cart_flags[8]) begin                          // Activision
		hardware_map <= '{3'd0, 3'd0, 3'd0, 3'd0, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4};
		bank_map <= '{8'd0, 8'd0, 8'd0, 8'd0, 8'd13, 8'd13, 8'd12, 8'd12, 8'd15, 8'd15, 8'd0, 8'd0, 8'd0, 8'd0, 8'd14, 8'd14};
		bank_map[10] <= {bank_reg[2:0], 1'b0};
		bank_map[11] <= {bank_reg[2:0], 1'b0};
		bank_map[12] <= {bank_reg[2:0], 1'b1};
		bank_map[13] <= {bank_reg[2:0], 1'b1};
		bank_type <= 3'd1;
	end else if (cart_flags[9]) begin                           // Absolute
		hardware_map <= '{3'd0, 3'd0, 3'd0, 3'd0, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4};
		bank_map <= '{8'd0, 8'd0, 8'd0, 8'd0, 8'd0, 8'd0, 8'd0, 8'd0, 8'd2, 8'd2, 8'd2, 8'd2, 8'd3, 8'd3, 8'd3, 8'd3};
		bank_map[4] <= {3'b000, bank_reg[1]};
		bank_map[5] <= {3'b000, bank_reg[1]};
		bank_map[6] <= {3'b000, bank_reg[1]};
		bank_map[7] <= {3'b000, bank_reg[1]};
		bank_type <= 3'd3;
	end else if (cart_flags[12]) begin                           // Souper
		hardware_map <= '{3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4};
		bank_type <= 3'd4;
	end else if (cart_flags[13] || cart_flags[14] || cart_flags[1] || cart_size_bs >= 32'h10000) begin // SuperGame
		hardware_map <= '{3'd0, 3'd0, 3'd0, 3'd0, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4, 3'd4};
		bank_map <= '{8'd0, 8'd0, 8'd0, 8'd0, second_highest_bank, second_highest_bank, second_highest_bank, second_highest_bank, 8'd0, 8'd0, 8'd0, 8'd0, highest_bank, highest_bank, highest_bank, highest_bank};
		bank_map[8] <= sg_bank;
		bank_map[9] <= sg_bank;
		bank_map[10] <= sg_bank;
		bank_map[11] <= sg_bank;
		if (is_bankset && cart_size_bs == 32'h8000) begin // 2x32K contigous banksets (HALT based bankset switching only)
			bank_mask <= 8'b00000001;
			bs_map <= 9'b0_0000_0010;
			bank_map[8] <= 8'd0;
			bank_map[9] <= 8'd0;
			bank_map[10] <= 8'd0;
			bank_map[11] <= 8'd0;
		end else if (is_bankset && cart_size_bs == 32'hC000) begin // 2x48K contigous banksets (HALT based bankset switching only)
			bank_mask <= 8'b00000011;
			bs_map <= 9'b0_0000_0011;
			bank_map[4] <= 8'd0;
			bank_map[5] <= 8'd0;
			bank_map[6] <= 8'd0;
			bank_map[7] <= 8'd0;
			bank_map[8] <= 8'd1;
			bank_map[9] <= 8'd1;
			bank_map[10] <= 8'd1;
			bank_map[11] <= 8'd1;
		end else if (is_bankset_52k) begin // 2x52K contigous banksets (HALT based bankset switching only)
			hardware_map[3] <= 3'd4;
			bank_map[3] <= 8'd0;
			bank_map[4] <= 8'd1;
			bank_map[5] <= 8'd1;
			bank_map[6] <= 8'd1;
			bank_map[7] <= 8'd1;
			bank_map[8] <= 8'd2;
			bank_map[9] <= 8'd2;
			bank_map[10] <= 8'd2;
			bank_map[11] <= 8'd2;
			bank_map[12] <= 8'd3;
			bank_map[13] <= 8'd3;
			bank_map[14] <= 8'd3;
			bank_map[15] <= 8'd3;
		end else if (cart_size_bs[22]) begin
			bank_mask <= 8'b11111111;
			bs_map <= 9'b1_0000_0000;
		end else if (cart_size_bs[21]) begin
			bank_mask <= 8'b01111111;
			bs_map <= 9'b0_1000_0000;
		end else if (cart_size_bs[20]) begin
			bank_mask <= 8'b00111111;
			bs_map <= 9'b0_0100_0000;
		end else if (cart_size_bs[19]) begin
			bank_mask <= 8'b00011111;
			bs_map <= 9'b0_0010_0000;
		end else if (cart_size_bs[18]) begin
			bank_mask <= 8'b00001111;
			bs_map <= 9'b0_0001_0000;
		end else if (cart_size_bs[17]) begin
			bank_mask <= 8'b00000111;
			bs_map <= 9'b0_0000_1000;
		end else if (cart_size_bs[16]) begin
			bank_mask <= 8'b00000011;
			bs_map <= 9'b0_0000_0100;
		end else if (cart_size_bs[15]) begin
			bank_mask <= 8'b00000001;
			bs_map <= 9'b0_0000_0010;
		end else begin
			bank_mask <= 8'b00000000;
			bs_map <= 9'b0_0000_0001;
		end
		bank_type <= 3'd0;
	end else begin                                     // Not banked
		if (cart_size <= 32'h2000) // A7808
			hardware_map <= '{3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd1, 3'd1};
		else if (cart_size <= 32'h4000) // A7816
			hardware_map <= '{3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd1, 3'd1, 3'd1, 3'd1};
		else if (cart_size <= 32'h8000) // A7832
			hardware_map <= '{3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd0, 3'd1, 3'd1, 3'd1, 3'd1, 3'd1, 3'd1, 3'd1, 3'd1};
		else if (cart_size <= 32'hC000) // A7848
			hardware_map <= '{3'd0, 3'd0, 3'd0, 3'd0, 3'd1, 3'd1, 3'd1, 3'd1, 3'd1, 3'd1, 3'd1, 3'd1, 3'd1, 3'd1, 3'd1, 3'd1};
		address_offset <= cart_size <= 32'h10000 ? 32'h10000 - cart_size : 32'd0;
		bank_type <= 3'd2;
	end

	// Alternative hardware at $4k selector
	if (cart_flags[2] || cart_flags[14]) begin // Supergame RAM at $4k
		hardware_map[4] <= 3'd3;
		hardware_map[5] <= 3'd3;
		hardware_map[6] <= 3'd3;
		hardware_map[7] <= 3'd3;
	end else if (cart_flags[5]) begin // Supergame 8kb RAM at $6k
		hardware_map[4] <= 3'd3;
		hardware_map[5] <= 3'd3;
		hardware_map[6] <= 3'd3;
		hardware_map[7] <= 3'd3;
	end else if (cart_flags[7]) begin // Mirror RAM at $4k
		hardware_map[4] <= 3'd3;
		hardware_map[5] <= 3'd3;
		hardware_map[6] <= 3'd3;
		hardware_map[7] <= 3'd3;
		ram_mask[8] <= 0;
		ram_mask[13:12] <= 2'b00;
	end else if (cart_flags[4]) begin // Bank 6 at $4k
		hardware_map[4] <= 3'd4;
		hardware_map[5] <= 3'd4;
		hardware_map[6] <= 3'd4;
		hardware_map[7] <= 3'd4;
		bank_map[4] <= 4'd6;
		bank_map[5] <= 4'd6;
		bank_map[6] <= 4'd6;
		bank_map[7] <= 4'd6;
	end

	if (XCTRL1[5]) begin // FIXME: This is technically banked ram, but I dont want to waste the bram...
		hardware_map[4] <= 3'd3;
		hardware_map[5] <= 3'd3;
	end
	if (XCTRL1[6]) begin
		hardware_map[6] <= 3'd3;
		hardware_map[7] <= 3'd3;
	end

end

// Maybe a mapper will use it someday...
assign IRQ_n = pokey_irq_en ? pokey_irq_n : 1'b1;

wire is_pokey_450 = (((cart_flags[6] || XCTRL1[4]) && address_in[15:4] == 12'h45) && cart_cs);
wire is_pokey_440 = (((cart_flags[10] || XCTRL1[4]) && address_in[15:4] == 8'h44) && cart_cs);
wire is_pokey_4k = ((cart_flags[0] && address_in[15:14] == 2'b01) && cart_cs);
wire is_pokey_800 = ((cart_flags[15] && address_in[15:11] == 5'h1) && cart_cs);
wire pokey4k_wo = cart_flags[0] && (cart_flags[3] || is_bankset);
// One Covox at $0430 on the ACE board, and an SN76489AN at $043F that the four
// register decode below must not claim.
wire is_covox = is_ace ? (address_in == 16'h0430) : (address_in[15:4] == 12'h43);
wire is_sn = is_ace && address_in == 16'h043F;

wire is_ym = (((cart_flags[11] || XCTRL1[7]) && address_in[15:1] == 15'h230) && cart_cs);

// Minnie, GCC 1730. 32 registers at $0460-$047F, the next 32 byte aligned block
// after the two POKEY windows, so it displaces neither. The chip decodes its own
// registers from A4..A0, which is why the window has to be 32 byte aligned.
// Enabled from the OSD; the a78 header's type field has no bits left.
// The OSD can also move the window to $0434-$0453, clear of the YM2151 at
// $0460. The chip still wants offsets 0-31, so the base is subtracted from
// A4..A0 (mod 32) before it reaches the chip.
wire is_minnie = minnie_en && cart_cs && (minnie_alt
	? (address_in >= 16'h0434 && address_in <= 16'h0453)
	: (address_in[15:5] == 11'b0000_0100_011));
wire [4:0] minnie_a = minnie_alt ? address_in[4:0] - 5'd20 : address_in[4:0];

assign external_audio = cart_flags[6] || cart_flags[10] || cart_flags[0] || is_covox || cart_flags[11] || cart_flags[15] || minnie_active || sn_enable;

logic [3:0] address_index;
assign address_index = address_in[15:12];

// Only three of the four ACE windows transform, and the mode is the top two
// bits of whatever was written to that window.
always_comb begin
	ace_swap = 2'd0;
	if (is_ace) case (address_index)
		4'hA: ace_swap = ace_a[7:6];
		4'hC: ace_swap = ace_c[7:6];
		4'hE: ace_swap = ace_e[7:6];
		default: ;
	endcase
end

// Address translation
always_comb begin
	pokey_cs = 0;
	pokey2_cs = 0;
	ram_cs = 0;
	ym_cs = 0;
	rom_address = 25'd0;
	if (is_pokey_450 || is_pokey_800)
		pokey_cs = 1;
	else if (is_pokey_440)
		pokey2_cs = 1;
	else if (is_pokey_4k && (~pokey4k_wo || ~rw))
		pokey_cs = 1;
	else if (is_ym)
		ym_cs = 1;
	else if (is_bankset_mem & ~rw & &address_in[15:14])
		ram_cs = 1;
	else if (cart_cs) case (hardware_map[address_index])
		3'd1: begin           // ROM Data
			rom_address = {1'b0, address_in - address_offset[15:0]};
		end
		3'd2: pokey_cs = 1'b1;// POKEY
		3'd3: ram_cs = 1'b1;  // RAM
		3'd4: begin           // Banked ROM
			case (bank_type)
				3'd0: // SuperGame
					if (is_bankset_52k) begin
						case (bank_map[address_index])
							8'd0: rom_address = (bankset_banks ? 17'h0D000 : 17'h0000) + address_in[11:0];
							8'd1: rom_address = (bankset_banks ? 17'h0E000 : 17'h1000) + address_in[13:0];
							8'd2: rom_address = (bankset_banks ? 17'h12000 : 17'h5000) + address_in[13:0];
							8'd3: rom_address = (bankset_banks ? 17'h16000 : 17'h9000) + address_in[13:0];
							default: ;
						endcase
					end else begin
						rom_address = {bank_map[address_index], address_in[13:0]} + {(bankset_banks ? bs_map : 9'd0), 14'd0};
					end
				3'd1: // Activision
					rom_address = {1'b0, bank_map[address_index], address_in[12:0]};
				3'd2: // No banking
					rom_address = {3'b000, address_in - address_offset[15:0]};
				3'd3: // Absolute
					rom_address = {bank_map[address_index], address_in[13:0]};
				3'd4: // souper
					rom_address = souper_addr;
				3'd5: // ACE. A transformed window also reads each 256 byte
				      // chunk backwards; A8-A11 are left alone.
					rom_address = {5'd0, bank_map[address_index], address_in[11:8],
						address_in[7:0] ^ {8{|ace_swap}}};
				default: ;
			endcase
		end
		default: ;
	endcase
end

//CS Type:
//00 - high impedance
//01 - ROM Data
//02 - POKEY
//03 - RAM
//04 - Banked ROM

logic [7:0] covox_reg[4];

// FIXME: this could possibly overflow, but the output is too relatively quiet without it.
// Possibly if it becomes an issue add a compressor.
always_comb begin
	covox_r = {{1'b0, covox_reg[0]} + covox_reg[2], 7'd0};
	covox_l = {{1'b0, covox_reg[1]} + covox_reg[3], 7'd0};
end

always_ff @(posedge clk_sys) begin
	if (reset) begin
		bank_reg <= 8'd0;
		ram_bank <= 3'd0;
		covox_reg <= '{8'd0, 8'd0, 8'd0, 8'd0};
		ace_a <= 8'd2;
		ace_c <= 8'd4;
		ace_d <= 8'd5;
		ace_e <= 8'd6;
		ace_ram <= 3'd0;
	end else if (~rw & cart_cs & pclk0) begin
		if (is_covox) begin
			// ACE has one Covox channel, so it feeds both sides rather than
			// landing in the right-hand pair alone.
			if (is_ace) begin
				covox_reg[0] <= din;
				covox_reg[1] <= din;
			end else
				covox_reg[address_in[1:0]] <= din;
		end
		if (is_ace) begin
			// The board ANDs A0-A11 low for the hotspot, so only the window's
			// base address latches a bank; $A001 is an ordinary ignored write.
			if (~|address_in[11:0]) case (address_index)
				4'hA: ace_a <= din;
				4'hC: ace_c <= din;
				4'hD: ace_d <= din;
				4'hE: ace_e <= din;
				default: ;
			endcase
			if (&address_in)
				ace_ram <= din[2:0];
		end
		if (bank_type == 3'd0 && address_in[15:14] == 2'b10) begin//supergame bank
			if (cart_flags[5]) begin
				ram_bank <= din[7:5];
				bank_reg <= din[4:0];
			end else begin
				bank_reg <= din;
			end
		end else if (bank_type == 3'd1 && (address_in[15:4]) == 12'hFF8) // activision bank
			bank_reg <= address_in[2:0]; // Note: this will sometimes be set with 4 bits, but should truncate. (Double Dragon)
		else if (bank_type == 3'd3 && address_in[15]) // Absolute
			bank_reg <= din[1:0];
	end
end

wire [14:0] bankset_ram_addr = {bankset_banks | (~rw & &address_in[15:14]), address_in[13:0]};
// ACE RAM is two 16K banks, and the $FFFF register can force A8 or A9 low so a
// short pattern repeats across a wide MARIA fetch.
wire [17:0] ace_ram_addr = {3'd0, ace_ram[0], address_in[13:10],
	address_in[9] & ~ace_ram[2], address_in[8] & ~ace_ram[1], address_in[7:0]};
assign cartram_addr = is_ace ? ace_ram_addr : (is_bankset_mem ? bankset_ram_addr : (souper_en ? souper_addr[17:0] : ({ram_bank, address_in[13:0]} & ram_mask)));
wire   cartram_cs = (ram_cs || (~souper_ram_cs && souper_en));
assign cartram_wr = cartram_cs && ~rw;
assign cartram_rd = cartram_cs &&  rw;
assign cartram_wrdata = din;
assign ram_dout = cartram_data;

// The ACE board permutes the EPROM's data lines on the way out, so one copy of
// a sprite can be read back in the bit order a given MARIA read mode wants.
// Taken from the four octal buffers in DATA_SWAP, one per mode.
always_comb begin
	case (ace_swap)
		2'd1: ace_dout = {rom_din[1], rom_din[0], rom_din[3], rom_din[2],   // 160A
				  rom_din[5], rom_din[4], rom_din[7], rom_din[6]};
		2'd2: ace_dout = {rom_din[5], rom_din[4], rom_din[7], rom_din[6],   // 160B
				  rom_din[1], rom_din[0], rom_din[3], rom_din[2]};
		2'd3: ace_dout = {rom_din[0], rom_din[1], rom_din[2], rom_din[3],   // 320B/320D
				  rom_din[4], rom_din[5], rom_din[6], rom_din[7]};
		default: ace_dout = rom_din;
	endcase
end

//CS Type:
//00 - high impedance
//01 - ROM Data
//02 - POKEY
//03 - RAM
//04 - Banked ROM
// dout_oe marks the lines the slot is actually driving. A window with no
// hardware behind it clears the mask instead of handing a byte back, and the
// bus keeps the charge it already had - top.sv holds that. dout only means
// anything where the mask says so.
//
// The three parts that make sound but never answer a read are the reason to
// keep the mask honest: the Covox DAC at $0430-$043F and the ACE board's
// SN76489AN at $043F have no data pins at all, and the XM control registers at
// $0470-$047F are write only here. None of them appear below, on purpose.
// POKEY and the YM2151 do drive, and Minnie brings its own enable.
always_comb begin
	dout = rom_din;
	dout_oe = '0;

	case(hardware_map[address_index])
		3'd1, 3'd4: begin dout = is_ace ? ace_dout : rom_din; dout_oe = '1; end // ROM Data
		3'd2: begin dout = pokey4k_dout; dout_oe = '1; end // POKEY
		3'd3: begin dout = ram_dout; dout_oe = '1; end     // RAM Data
		default: ;                                         // High impedance
	endcase

	if (is_ym) begin
		dout = ym_dout;
		dout_oe = '1;
	end
	if (hsc_rom_cs) begin
		dout = hsc_rom_dout;
		dout_oe = '1;
	end
	if (hsc_ram_cs) begin
		dout = hsc_ram_dout;
		dout_oe = '1;
	end
	// A write-only $4000 window belongs to the banked ROM the case above
	// selected, so POKEY only takes the read back when it is not.
	if (is_pokey_450 || is_pokey_800 || (is_pokey_4k && ~pokey4k_wo)) begin
		dout = pokey4k_dout;
		dout_oe = '1;
	end
	if (is_pokey_440) begin
		dout = pokey2_dout;
		dout_oe = '1;
	end
	if (is_minnie) begin
		dout = minnie_dout;
		dout_oe = {8{minnie_doe}};
	end
	if (souper_en) begin
		if (~souper_ram_cs)
			dout = ram_dout;
		else
			dout = rom_din;
		dout_oe = '1;
	end

	// Nothing in the slot drives while the CPU has the bus, or while a part on
	// the board rather than the cartridge is the one selected.
	if (~cart_cs || ~rw)
		dout_oe = '0;
end

logic [3:0] ch0, ch1, ch2, ch3, ch0_2, ch1_2, ch2_2, ch3_2;
logic [15:0] pokey_aud, pokey2_aud;
logic [15:0] pokey_mux, pokey2_mux;
logic pokey2_cs;
logic using_two_pokey;

always @(posedge clk_sys) begin
	if (reset)
		using_two_pokey <= 0;
	if (is_pokey_440)
		using_two_pokey <= 1;

	// The AUD node, not a sum of the four channel volumes. POKEY has one
	// output pin: the four DACs share it, their volume bits are not a clean
	// 1:2:4:8, and the total saturates. All three are measured - see
	// pokey_mixer.sv. Summing the channels linearly, which this did before,
	// is audibly wrong once more than one channel is loud.
	pokey_mux <= pokey_aud;
	pokey2_mux <= pokey2_aud;
end

assign pokey_audio_r = (cart_flags[0] || cart_flags[6] || cart_flags[10] || cart_flags[15]) ? pokey_mux : 16'd0;
assign pokey_audio_l = ~using_two_pokey ? pokey_audio_r : pokey2_mux;

// Minnie takes two non-overlapping phase enables. pclk0 is when the processor
// bus is valid; the microcode advances on the clk_sys cycle after it. One
// microcode state per processor clock, so 64 per sample.
logic minnie_ph1;
always_ff @(posedge clk_sys)
	minnie_ph1 <= pclk0;

wire [15:0] minnie_aud;

minnie the_mouse (
	.clk       (clk_sys),
	.ph1_en    (minnie_ph1),
	.ph2_en    (pclk0),
	.reset     (reset),
	.a         (minnie_a),
	.cs        (is_minnie),
	.rw        (rw),
	.d_in      (din),
	.d_out     (minnie_dout),
	.d_oe      (minnie_doe),
	.sample    (),
	.sample_en (),
	.aud       (minnie_aud)
);

// Latched on the first access, the way using_two_pokey is. The OSD option only
// makes the chip reachable; until a program actually writes to it, Minnie
// contributes nothing. Without this, switching the option on would halve every
// other source through external_audio and add Minnie's DC bias to the mix, for
// every game that never uses it.
always_ff @(posedge clk_sys) begin
	if (reset || ~minnie_en)
		minnie_active <= 1'b0;
	else if (is_minnie && ~rw)
		minnie_active <= 1'b1;
end

// Minnie has one output pin, so this is mono. top.sv mixes it into both
// channels.
assign minnie_audio = minnie_active ? minnie_aud : 16'd0;

// SN76489AN on the ACE board at $043F, strobed the way its CPLD does it: the
// byte is caught in a 74273 and /WE+/CE stay low until READY comes back, so
// the chip always sees the bus held through its 32-clock transfer. SN_ENABLE
// latches on the first strobe and closes the switch on the chip's output;
// until then its power-on noise never reaches the console. The chip runs from
// the board's own 3.579545 MHz resonator, which is clk_sys/4 here.
logic [1:0]  sn_clk_div;
logic [1:0]  sn_hold;       // two clk_sys of /CE, long enough for READY to take over
logic [7:0]  sn_d;
logic        sn_enable, sn_ready;
wire         sn_write = is_sn && ~rw && cart_cs && pclk0;
wire         sn_ce_n  = ~(|sn_hold || ~sn_ready);
wire [13:0]  sn_aud;

always_ff @(posedge clk_sys) begin
	sn_clk_div <= sn_clk_div + 2'd1;
	if (reset || ~is_ace) begin
		sn_hold   <= 2'd0;
		sn_enable <= 1'b0;
	end else begin
		sn_hold <= {sn_hold[0], sn_write};
		if (sn_write) sn_enable <= 1'b1;
	end
	if (sn_write) sn_d <= din;
end

sn76489 the_sn (
	.clk      (clk_sys),
	.clk_en   (sn_clk_div == 2'd3),
	.reset    (reset || ~is_ace),
	.ce_n     (sn_ce_n),
	.we_n     (sn_ce_n),
	.d        (sn_d),
	.ready    (sn_ready),
	.ready_oe (),
	.aud      (sn_aud)
);

// Four channels at full level reach the TIA's ceiling, 32760, so TIA plus the
// ACE Covox plus this stays well inside top.sv's 17 bit mix.
assign sn_audio = sn_enable ? {1'b0, sn_aud, 1'b0} : 16'd0;

logic [5:0] keyboard_scan;
logic [1:0] keyboard_response;
logic old_ps2_10;
always @(posedge clk_sys)
	old_ps2_10 <= ps2_key[10];

ps2_to_atari800 ps2_to_pokey (
	.CLK               (clk_sys),
	.RESET_N           (~reset),
	.INPUT             ({12'h000, 3'b000, ps2_key[9], 3'b000, ps2_key[8], 4'h0, ps2_key[7:0]}),
	.KEYBOARD_SCAN     (keyboard_scan),
	.KEYBOARD_RESPONSE (keyboard_response)
);

pokey_adapter the_penguin (
	.CLK                  (clk_sys),
	.PHI1_EN              (pclk1),
	.PHI2_EN              (pclk0),
	.ADDR                 (address_in[3:0]),
	.DATA_IN              (din),
	.WR_EN                (~rw & pokey_cs),
	.RESET_N              (~reset),
	.keyboard_scan_enable (old_ps2_10 != ps2_key[10]),
	.keyboard_scan        (keyboard_scan),
	.keyboard_response    (keyboard_response),

	.POT_IN               (),
	.SIO_IN1              (),
	.SIO_IN2              (),
	.SIO_IN3              (),
	.DATA_OUT             (pokey4k_dout),
	.CHANNEL_0_OUT        (ch0),
	.CHANNEL_1_OUT        (ch1),
	.CHANNEL_2_OUT        (ch2),
	.CHANNEL_3_OUT        (ch3),
	.AUD                  (pokey_aud),

	.IRQ_N_OUT            (pokey_irq_n),
	.SIO_OUT1             (),
	.SIO_OUT2             (),
	.SIO_OUT3             (),
	.SIO_CLOCKIN_IN       (),
	.SIO_CLOCKIN_OUT      (),
	.SIO_CLOCKIN_OE       (),
	.SIO_CLOCKOUT         (),
	.POT_RESET            ()
);

pokey_adapter return_of_pokey (
	.CLK                  (clk_sys),
	.PHI1_EN              (pclk1),
	.PHI2_EN              (pclk0),
	.ADDR                 (address_in[3:0]),
	.DATA_IN              (din),
	.WR_EN                (~rw & pokey2_cs),
	.RESET_N              (~reset),
	.keyboard_scan_enable (),
	.keyboard_scan        (),
	.keyboard_response    (),

	.POT_IN               (),
	.SIO_IN1              (),
	.SIO_IN2              (),
	.SIO_IN3              (),
	.DATA_OUT             (pokey2_dout),
	.CHANNEL_0_OUT        (ch0_2),
	.CHANNEL_1_OUT        (ch1_2),
	.CHANNEL_2_OUT        (ch2_2),
	.CHANNEL_3_OUT        (ch3_2),
	.AUD                  (pokey2_aud),

	.IRQ_N_OUT            (),
	.SIO_OUT1             (),
	.SIO_OUT2             (),
	.SIO_OUT3             (),
	.SIO_CLOCKIN_IN       (),
	.SIO_CLOCKIN_OUT      (),
	.SIO_CLOCKIN_OE       (),
	.SIO_CLOCKOUT         (),
	.POT_RESET            ()
);

wire [15:0] ym_audio_lo, ym_audio_ro;

jt51 ym2151 (
	.rst      (reset),
	.clk      (clk_sys),
	.cen      (pclk1 || pclk0),
	.cen_p1   (pclk0),
	.cs_n     (~ym_cs),
	.wr_n     (rw),
	.a0       (address_in[0]),
	.din      (din),
	.dout     (ym_dout),
	.ct1      (),
	.ct2      (),
	.irq_n    (ym_irq_n),
	.sample   (),
	.left     (),
	.right    (),
	.xleft    (),
	.xright   (),
	.dacleft  (ym_audio_lo),
	.dacright (ym_audio_ro)
);

always @(posedge clk_sys) begin
	if (cart_flags[11] || XCTRL1[7]) begin
		ym_audio_r <= ym_audio_ro;
		ym_audio_l <= ym_audio_lo;
	end else begin
		ym_audio_r <= 0;
		ym_audio_l <= 0;
	end
end

assign hsc_ram_cs = address_in[15:11] == 5'd2 && hsc_en;
wire hsc_rom_cs = address_in[15:12] == 4'd3 && hsc_en;

spram #(
	.addr_width(12),
	.mem_name("HSC"),
	.mem_init_file("mem4.mif"),
	.sim_init_file("rtl/mem4.hex")
) hsc_rom
(
	.address (address_in[11:0]),
	.clock   (clk_sys),
	.data    (8'd0),
	.wren    (1'b0),
	.cs      (1'b1),
	.q       (hsc_rom_dout)
);

assign hsc_ram_dout = hsc_ram_din;

logic souper_rom_cs;
logic [7:0] souper_aud_data;
logic       souper_aud_req;
assign souper_addr = {souper_bank, address_in[6:0]};

souper soup_soup (
	.clk        (clk_sys),
	.pclk1      (pclk0), // FIXME create ce's
	.reset      (reset),
	.halt_n     (halt_n),
	.data       (din),
	.rw         (rw),
	.addr_15    (address_in[15]),
	.addr_14    (address_in[14]),
	.addr_13    (address_in[13]),
	.addr_12    (address_in[12]),
	.addr_11    (address_in[11]),
	.addr_10    (address_in[10]),
	.addr_9     (address_in[9]),
	.addr_8     (address_in[8]),
	.addr_7     (address_in[7]),
	.addr_2     (address_in[2]),
	.addr_1     (address_in[1]),
	.addr_0     (address_in[0]),
	.romSel_n   (souper_rom_cs),
	.ramSel_n   (souper_ram_cs),
	.oe_n       (),
	.wr_n       (souper_wr),
	.mapAddr_7p (souper_bank),
	.audCom       (souper_aud_data),
	.audReq_n     (),
	.audReq_logic (souper_aud_req)
);

// The cartridge pin is open drain, but a consumer inside the FPGA must not
// sample a `Z`. souper.v inverts its request register on every $8007 write, so
// the logical toggle is exported directly and edge-detected here.
//
// A game writes $8007 twice per command so that the request line emits one
// complete pulse whatever level it started at (see
// .agents/evidence/bupchip_8007_command_encoding.md). Delivering one event per
// toggle would therefore duplicate every command, so this counts pairs: the
// byte is published on the second toggle.
logic souper_aud_req_d;
logic souper_aud_phase;

always_ff @(posedge clk_sys) begin
	if (reset) begin
		souper_aud_req_d <= 1'b1;
		souper_aud_phase <= 1'b0;
		aud_cmd_valid <= 1'b0;
		aud_cmd_data <= 8'd0;
	end else begin
		aud_cmd_valid <= 1'b0;
		souper_aud_req_d <= souper_aud_req;
		if (souper_aud_req != souper_aud_req_d) begin
			souper_aud_phase <= ~souper_aud_phase;
			if (souper_aud_phase) begin
				aud_cmd_data  <= souper_aud_data;
				aud_cmd_valid <= 1'b1;
			end
		end
	end
end


endmodule: cart


// cart type word details::
//   bit 0    = pokey at $4000
//   bit 1    = supergame bank switched
//   bit 2    = supergame ram at $4000
//   bit 3    = rom at $4000
//   bit 4    = bank 6 at $4000
//   bit 5    = supergame banked ram
//   bit 6    = pokey at $450
//   bit 7    = mirror ram at $4000
//   bit 8    = activision banking
//   bit 9    = absolute banking
//   bit 10   = pokey at $440
//   bit 11   = ym2151 at $460/$461
//   bit 12   = souper mapper
//   bit 13-15 = special

// controller type byte details:
//   0 = none
//   1 = 7800 joystick
//   2 = lightgun
//   3 = paddle
//   4 = trakball
//   5 = 2600 joystick
//   6 = 2600 driving
//   7 = 2600 keypad
//   8 = ST mouse
//   9 = Amiga mouse

// TV type details:
//   0 = NTSC
//   1 = PAL

// save device details:
//   bit 1    = HSC
//   bit 2    = SaveKey/AtariVox

// expansion module details:
//   bit 1    = XM

// XM registers:
// cntrl1	$470
// 	d0 rof lo on
// 	d1 rof hi on
// 	d2 0=bios,1=top slot
// 	d3 1=hsc on
// 	d4 1=pokey on
// 	d5 1=bank0 on 4000-5fff
// 	d6 1=bank1 on 6000-7fff
// 	d7 1=ym2151 on

// cntrl2	$478  - SALLY RAM bank 8K page multiplexer.
// 	d0-d3 sally ram page 0 a0-a3
// 	d4-d7 sally ram page 1 a0-a3

// cntrl3	$47c  - MARIA RAM bank 8K page multiplexer.
// 	d0-d3 maria ram page 0 a0-a3
// 	d4-d7 maria ram page 1 a0-a3

// cntrl4	$471
// 	d0 1=pia on
// 	d1-d3 flash bank lo a1-a3
// 	d4-d6 flash bank hi a1-a3
// 	d7 1=top slot lock

// cntrl5 	$472
// 	d0 1=48k ram enable
// 	d1 1=ram we# disabled
// 	d2 1=bios enabled (in test mode)
// 	d3 1=POKEY enable/disable locked
// 	d4 1=HSC enable/disable locked - cannot disable after enable
// 	d5 1=PAL HSC enabled, 0=NTSC HSC enabled - cannot disable after enable
