// The OSD list follows this order, so ELF (never selectable) stays last.
typedef enum bit[5:0] {
	BANK00, BANKF8, BANKF6, BANKFE, BANKE0,   BANK3F,   BANKF4,  BANKP2,
	BANKFA, BANKCV, BANK2K, BANKUA, BANKE7,   BANKF0,   BANK32,  BANKAR,
	BANK3E, BANKSB, BANKWD, BANKEF, BANKJANE, BANKDPCP, BANKCTY, BANKCDF,
	BANKBUS, BANKFA2, BANK0840, BANKFC, BANKDF, BANKMC2K, BANKMC4K, BANKMC8K,
	BANKELF, BANKEND
} bss_type ;

module detect2600
(
	input clk,
	input load_start,
	input [24:0] load_addr,
	input load_valid,
	input load_end,
	input [31:0] cart_size,
	input [7:0] data,
	output reg [5:0] force_bs,
	output reg sc,
	output reg [2:0] mapper_revision,
	output reg cdf_ldx,
	output reg cdf_ldy,
	output reg cdf_fetch_offset_enable,
	output reg [7:0] cdf_fetch_offset,
	output reg [31:0] cdfj_entry,
	output reg [31:0] cdfj_stack,
	output reg [15:0] arm_audio_size_addr
);

wire reset = load_start;
wire [24:0] addr = load_addr;
wire enable = load_valid;
wire hasMatch3F;
wire hasMatchBUS;

reg [31:0] elf_window;
reg elf_magic;
reg elf_endian;
reg elf_type;
reg elf_machine;

reg [31:0] revision_window;
reg [31:0] revision_word_1;
reg [31:0] revision_word_2;
reg [2:0] cdf0_count;
reg [2:0] cdf1_count;
reg [2:0] cdfj_count;
reg cdfj_plus;
reg bus_word_seen;
reg [1:0] bus_revision;
reg [31:0] dpc_driver_crc;
reg audio_size_scan;
reg [4:0] audio_size_words;
reg [31:0] fa2_loader_window;
reg fa2_loader;
reg fa2_padding_zero;

wire [31:0] next_revision_window = load_addr[1:0] == 2'd0 ?
	{24'b0, data} : load_addr[1:0] == 2'd1 ?
	{16'b0, data, revision_window[7:0]} : load_addr[1:0] == 2'd2 ?
	{8'b0, data, revision_window[15:0]} :
	{data, revision_window[23:0]};

wire [31:0] next_elf_window = {elf_window[23:0], data};
wire [31:0] next_fa2_loader_window = load_start ? {24'd0, data} :
	{fa2_loader_window[23:0], data};
wire hasMatchELF = cart_size >= 32'd52 && elf_magic && elf_endian &&
	elf_type && elf_machine;
wire cdf_size = cart_size == 32'd32768 || cart_size == 32'd65536 ||
	cart_size == 32'd131072 || cart_size == 32'd262144 ||
	cart_size == 32'd524288;
wire hasMatchDEVC;

always @(posedge clk) begin
	if (load_start) begin
		fa2_loader_window <= '0;
		fa2_loader <= 1'b0;
		fa2_padding_zero <= 1'b1;
	end

	if (load_valid) begin
		fa2_loader_window <= next_fa2_loader_window;
		if (load_addr < 25'd1024 &&
			(next_fa2_loader_window == 32'hA0C11FE0 ||
			 next_fa2_loader_window == 32'h008002E0))
			fa2_loader <= 1'b1;
		if (load_addr >= 25'd29696 && load_addr < 25'd32768 && data != 8'd0)
			fa2_padding_zero <= 1'b0;
	end
end

always @(posedge clk) begin
	if (load_start) begin
		elf_window <= '0;
		elf_magic <= 1'b0;
		elf_endian <= 1'b0;
		elf_type <= 1'b0;
		elf_machine <= 1'b0;
	end

	if (load_valid) begin
		elf_window <= load_start ? {24'd0, data} : next_elf_window;
		if (load_addr <= 25'd7 && next_elf_window == 32'h7F454C46)
			elf_magic <= 1'b1;
		if (load_addr == 25'h05)
			elf_endian <= data == 8'h01;
		if (load_addr == 25'h10)
			elf_type <= data == 8'h01;
		if (load_addr == 25'h12)
			elf_machine <= data == 8'h28;
	end
end

// Stella subtypes are encoded in aligned driver words. DPC+ uses a serial
// first-3-KiB CRC here instead of a parallel MD5 datapath.
always @(posedge clk) begin
	if (load_start) begin
		revision_window <= '0;
		revision_word_1 <= '0;
		revision_word_2 <= '0;
		cdf0_count <= '0;
		cdf1_count <= '0;
		cdfj_count <= '0;
		cdfj_plus <= 1'b0;
		bus_word_seen <= 1'b0;
		bus_revision <= 2'd0;
		dpc_driver_crc <= '0;
		cdf_ldx <= 1'b0;
		cdf_ldy <= 1'b0;
		cdf_fetch_offset_enable <= 1'b0;
		cdf_fetch_offset <= 8'b0;
		cdfj_entry <= 32'b0;
		cdfj_stack <= 32'b0;
		arm_audio_size_addr <= 16'b0;
		audio_size_scan <= 1'b0;
		audio_size_words <= 5'b0;
	end

	if (load_valid) begin
		revision_window <= next_revision_window;
		if (load_addr < 25'd3072)
			dpc_driver_crc <= nextCRC32_D8(data,
				load_start ? 32'b0 : dpc_driver_crc);

		if (load_addr[1:0] == 2'b11 && load_addr < 25'd3072) begin
			revision_word_2 <= revision_word_1;
			revision_word_1 <= next_revision_window;
			if (next_revision_window == 32'hE3C55D3E) begin
				audio_size_scan <= 1'b1;
				audio_size_words <= 5'd20;
			end else if (audio_size_scan) begin
				if (next_revision_window[31:16] == 16'h4000) begin
					arm_audio_size_addr <= next_revision_window[15:0];
					audio_size_scan <= 1'b0;
				end else if (audio_size_words == 5'd1) begin
					audio_size_scan <= 1'b0;
				end else begin
					audio_size_words <= audio_size_words - 5'd1;
				end
			end
			if (load_addr < 25'd2048) begin
				if (next_revision_window == 32'h135200A2)
					cdf_ldx <= 1'b1;
				if (next_revision_window == 32'h135200A0)
					cdf_ldy <= 1'b1;
				if ((next_revision_window & 32'hFFFFFF00) == 32'hE2422000) begin
					cdf_fetch_offset_enable <= 1'b1;
					cdf_fetch_offset <= next_revision_window[7:0];
				end
				if (next_revision_window == 32'h00464443 && cdf0_count < 3)
					cdf0_count <= cdf0_count + 3'd1;
				else if (next_revision_window == 32'h4A464443 && cdfj_count < 3)
					cdfj_count <= cdfj_count + 3'd1;
				else if (next_revision_window[23:0] == 24'h464443 &&
					next_revision_window[31:24] != 8'h00 &&
					next_revision_window[31:24] != 8'h4A && cdf1_count < 3)
					cdf1_count <= cdf1_count + 3'd1;
				if (revision_word_2 == 32'h53554C50 &&
					revision_word_1 == 32'h4A464443 &&
					next_revision_window == 32'h00000001)
					cdfj_plus <= 1'b1;
			end

			if (!bus_word_seen && next_revision_window == 32'h00535542) begin
				bus_word_seen <= 1'b1;
				case (load_addr - 25'd3)
					25'h007F4: bus_revision <= 2'd1;
					25'h00778: bus_revision <= 2'd2;
					25'h00770: bus_revision <= 2'd3;
					default:    bus_revision <= 2'd0;
				endcase
			end
		end

		if (load_addr[1:0] == 2'b11) begin
			if (load_addr - 25'd3 == 25'h017F4)
				cdfj_stack <= next_revision_window;
			if (load_addr - 25'd3 == 25'h017F8)
				cdfj_entry <= next_revision_window & 32'hFFFFFFFE;
		end
	end
end

always @(posedge clk) begin
	if (load_start) begin
		force_bs <= BANK00;
		sc <= 1'b0;
		mapper_revision <= 3'd0;
	end else if (load_end) begin
		sc <= 1'b0;
		mapper_revision <= 3'd0;
		if (hasMatchELF) force_bs<=BANKELF;
		else if ((cart_size=='d24576 || cart_size=='d28672) && !hasMatchDEVC)
			force_bs<=BANKFA2;
		else if (cart_size=='d29696) force_bs<=fa2_loader ? BANKFA2 : BANKDPCP;
		else if (hasMatchCTY && (cart_size=='d32768 || cart_size=='d61440)) force_bs<=BANKCTY;
		else if (hasMatchCDF && cdf_size) begin
			force_bs<=BANKCDF;
			mapper_revision <= cdfj_plus ? 3'd3 :
				(cdfj_count >= 3 ? 3'd2 : (cdf0_count >= 3 ? 3'd0 : 3'd1));
		end
		else if (hasMatchDPCP && cart_size=='d32768) begin
			force_bs<=BANKDPCP;
			mapper_revision <= dpc_driver_crc == 32'hA08CFB13 ? 3'd1 : 3'd0;
		end
		else if (cart_size=='d32768 && has_sc) begin
			force_bs<=BANKF4;
			sc<=1'b1;
		end
		else if (hasMatchFE && cart_size=='d8192) force_bs<=BANKFE;
		else if (hasMatchE0 && cart_size=='d8192) force_bs<=BANKE0;
		else if (hasMatch3E && cart_size>'d4096)  force_bs<=BANK3E;
		else if (hasMatch3F && cart_size>'d4096) force_bs<=BANK3F;
		else if (hasMatchBUS && cart_size=='d32768) begin
			force_bs<=BANKBUS;
			mapper_revision <= {1'b0, bus_revision};
		end
		else if (cart_size=='d32768 && fa2_padding_zero && !has_sc) force_bs<=BANKFA2;
		else if ((cart_size=='d131072 || cart_size=='d262144) && tail_dfbf) begin
			force_bs<=BANKDF;
			sc <= tail_dfbf_sc;
		end
		else if (hasMatchSB && cart_size>='d131072) force_bs<=BANKSB;
		else if (hasMatchEF && cart_size=='d65536 ) begin
			force_bs<=BANKEF;
			sc <= has_sc;
		end
		else if (hasMatchCV) force_bs<=BANKCV;
		else if (hasMatchJANE && cart_size=='d16384) force_bs<=BANKJANE;
		else if (hasMatchE7) force_bs<=BANKE7;
		else if (cart_size == 'h2003) begin // WD dump with 1K banks 2 and 3 swapped
			force_bs<=BANKWD;
			mapper_revision <= 3'd1;
		end
		else if (cart_size == 'h2000 && hasMatchWD ) force_bs<=BANKWD; //  8k
		else if (cart_size == 'h2000 && hasMatchUA ) begin //  8k
			force_bs<=BANKUA;
			mapper_revision <= {2'd0, ua_mickey};
		end
		else if (cart_size == 'h2000 && hasMatch0840) force_bs<=BANK0840;
		else if (cart_size == 'h2000 && hasMatchFC) force_bs<=BANKFC;
		else if (cart_size == 'h1800) force_bs<=BANKAR; //  multiple of 8448 is cassette  AR
		else if (cart_size == 'h2100) force_bs<=BANKAR; //  multiple of 8448 is cassette  AR
		else if (cart_size == 'h4200) force_bs<=BANKAR; //  multiple of 8448 is cassette  AR
		else if (cart_size == 'h6300) force_bs<=BANKAR; //  multiple of 8448 is cassette  AR
		else if (cart_size == 'h8400) force_bs<=BANKAR; //  multiple of 8448 is cassette  AR
		else if (cart_size <= 'h0800) force_bs<=BANK2K; //  2k and less
		else if (cart_size <= 'h1000) begin //  4k and less
			if (has_sc && sc4k_sig) begin
				force_bs<=BANK00;
				sc <= 1'b1;
			end else
				force_bs <= hasMatchFC ? BANKFC : BANK00;
		end
		else if (cart_size <= 'h2000) begin
			force_bs<=BANKF8; //  8k and less
			sc <= has_sc;
		end
		else if (cart_size >= 'h2800 && cart_size <= 'h2900) force_bs<=BANKP2; // 10k+256 and less, should be > 10k < 10k+256?
		else if (cart_size <= 'h3000) force_bs<=BANKFA; // 12k and less
		else if (cart_size == 'h4000 && hasMatchFC) force_bs<=BANKFC;
		else if (cart_size <= 'h4000) begin
			force_bs<=BANKF6; // 16k and less
			sc <= has_sc;
		end
		else if (cart_size == 'h8000 && hasMatchFC) force_bs<=BANKFC;
		else if (cart_size <= 'h8000) begin
			force_bs<=BANKF4; // 32k and less
			sc <= has_sc;
		end
		else if (cart_size < 'h10000) force_bs<=BANK32; // 64k and less
		else if (cart_size == 'h10000) force_bs<=BANKF0; // 64k  - there are a few checks here
		else force_bs<=0;
	end


//wire hasMatchFE = (~hasMatchF8) && (hasMatchFE_0 | hasMatchFE_1 | hasMatchFE_2 | hasMatchFE_3 );
//$display(" hasMatchF8 %x hasMatchFE_0 %x hasMatchFE_1 %x hasMatcHFE_2 %x hasMatchFE_3 %x",hasMatchF8,hasMatchFE_0,hasMatchFE_1,hasMatchFE_2,hasMatchFE_3);
//$display(" hasMatchFE %x ",hasMatchFE);
end

//----------------------------
//  EF detector
//----------------------------
wire hasMatchEF_0 , hasMatchEF_1, hasMatchEF_2,hasMatchEF_3;
wire hasMatchEF = hasMatchEF_0 | hasMatchEF_1| hasMatchEF_2 | hasMatchEF_3;

/*
  uInt8 signature[4][3] = {
    { 0x0C, 0xE0, 0xFF },  // NOP $FFE0
    { 0xAD, 0xE0, 0xFF },  // LDA $FFE0
    { 0x0C, 0xE0, 0x1F },  // NOP $1FE0
    { 0xAD, 0xE0, 0x1F }   // LDA $1FE0
  };
*/
// One byte history and one contiguity test for every pattern below. All the
// matchers watch the same addr, data, enable and reset, so a shift register and
// a 25-bit address compare inside each would be fifty-one copies of this.
reg [63:0] match_stream;
reg [24:0] match_last_addr;
reg        match_have_addr;
wire match_contiguous = match_have_addr && addr == match_last_addr + 25'd1;
// The matchers compare the value the stream is about to take, not the one it
// currently holds.
wire [63:0] match_next = match_contiguous && !reset ?
	{match_stream[55:0], data} : {56'b0, data};

always @(posedge clk) begin
	if (reset) begin
		match_stream <= '0;
		match_last_addr <= '0;
		match_have_addr <= 1'b0;
	end

	if (enable) begin
		match_stream <= match_next;
		match_last_addr <= addr;
		match_have_addr <= 1'b1;
	end
end

match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'h0C, 8'hE0 , 8'hFF }),
	.needmatches(8'd1)
	) match_bytes_EF_0(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchEF_0)
);
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'hAD, 8'hE0 , 8'hFF }),
	.needmatches(8'd1)
	) match_bytes_EF_1(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchEF_1)
);
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'h0C, 8'hE0 , 8'h1F }),
	.needmatches(8'd1)
	) match_bytes_EF_2(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchEF_2)
);
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'hAD, 8'hE0 , 8'h1F }),
	.needmatches(8'd1)
	) match_bytes_EF_3(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchEF_3)
);

//----------------------------
//  DPC+ detector
//----------------------------
match_bytes #(
	.num_bytes(8'd4),
	.pattern({ 8'hA9, 8'hFD, 8'h85, 8'h08 }),
	.needmatches(8'd1)
	) match_bytes_DEVC(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchDEVC)
);

wire hasMatchDPCP;
match_bytes #(
	.num_bytes(8'd4),
	.pattern({ 8'h44, 8'h50 , 8'h43, 8'h2B }),
	.needmatches(8'd2)
	) match_bytes_DPCP(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchDPCP)
);

// BUS ARM drivers contain the ASCII marker twice.
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'h42, 8'h55, 8'h53 }),
	.needmatches(8'd2)
	) match_bytes_BUS(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchBUS)
);
//----------------------------
//  CTY detector
//----------------------------
wire hasMatchCTY;
match_bytes #(
	.num_bytes(8'd5),
	.pattern({ 8'h4C, 8'h45 , 8'h4E, 8'h49, 8'h4E }),
	.needmatches(8'd1)
	) match_bytes_CTY(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchCTY)
);

wire hasMatchCDF =  hasMatchCDF_1  | hasMatchCDF_2;
wire  hasMatchCDF_1,hasMatchCDF_2;

match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'h43, 8'h44 , 8'h46 }),
	.needmatches(8'd3)
	) match_bytes_CDF_1(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchCDF_1)
);
match_bytes #(
	.num_bytes(8'd8),
	.pattern({ 8'h50, 8'h4C , 8'h55, 8'h53, 8'h43, 8'h44, 8'h46, 8'h4A }),
	.needmatches(8'd1)
	) match_bytes_CDF_2(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchCDF_2)
);



/*
wire hasMatchEF = hasMatchEF_1 & hasMatchEF_2 & hasMatchEF_3 & hasMatchEF_4 ;
reg hasMatchEF_1;
reg hasMatchEF_2;
reg hasMatchEF_3;
reg hasMatchEF_4;
always @(posedge clk) begin
$display("hasMatchEF_1 %x %x %x %x %x",hasMatchEF_1,hasMatchEF_2,hasMatchEF_3,hasMatchEF_4,hasMatchEF);
  if (addr=='hFFF8)
  begin
	hasMatchEF_1<=1'b0;
        if (data=='h45) 
		hasMatchEF_1 <= 1'b1;
  end
  if (addr=='hFFF9)
  begin
	hasMatchEF_2<=1'b0;
        if (data=='h46) 
		hasMatchEF_2 <= 1'b1;
  end
  if (addr=='hFFF0)
  begin
$display("data %x",data);
	hasMatchEF_3<=1'b0;
        if (data=='h45) 
		hasMatchEF_3 <= 1'b1;
  end
  if (addr=='hFFFA)
  begin
	hasMatchEF_4<=1'b0;
        if (data=='h46) 
		hasMatchEF_4 <= 1'b1;
  end
   
end
*/



//----------------------------
//  3F detector
//----------------------------

match_bytes #(
	.num_bytes(8'd2),
	.pattern({ 8'h85, 8'h3F }),
	.needmatches(8'd2)
	) match_bytes_3F(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatch3F)
);


//----------------------------
//  3E detector
//----------------------------


/*
bool CartDetector::isProbably3E(const ByteBuffer& image, size_t size)
{
  // 3E cart RAM bankswitching is triggered by storing the bank number
  // in address 3E using 'STA $3E', ROM bankswitching is triggered by
  // storing the bank number in address 3F using 'STA $3F'.
  // We expect the latter will be present at least 2 times, since there
  // are at least two banks

  uInt8 signature1[] = { 0x85, 0x3E };  // STA $3E
  uInt8 signature2[] = { 0x85, 0x3F };  // STA $3F
  return searchForBytes(image, size, signature1, 2)
    && searchForBytes(image, size, signature2, 2, 2);
}
*/
wire hasMatch3E = hasMatch3E_1 & hasMatch3F;
wire hasMatch3E_1;
match_bytes #(
	.num_bytes(8'd2),
	.pattern({ 8'h85, 8'h3E }),
	.needmatches(8'd1)
	) match_bytes_3E_1(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatch3E_1)
);

//------------------------------
// WD detector
//-------------------------------
/*
bool CartDetector::isProbablyWD(const ByteBuffer& image, size_t size)
{
  // WD cart bankswitching switches banks by accessing address 0x30..0x3f
  uInt8 signature[1][3] = {
    { 0xA5, 0x39, 0x4C }  // LDA $39, JMP
  };
  return searchForBytes(image, size, signature[0], 3);
}
*/
wire hasMatchWD;
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'hA5,8'h39, 8'h4C }),
	.needmatches(8'd1)
	) match_bytes_WD_1(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchWD)
);

//------------------------------
// SB detector
//-------------------------------

/*
bool CartDetector::isProbablySB(const ByteBuffer& image, size_t size)
{
  // SB cart bankswitching switches banks by accessing address 0x0800
  uInt8 signature[2][3] = {
    { 0xBD, 0x00, 0x08 },  // LDA $0800,x
    { 0xAD, 0x00, 0x08 }   // LDA $0800
  };
  if(searchForBytes(image, size, signature[0], 3))
    return true;
  else
    return searchForBytes(image, size, signature[1], 3);
}
*/
wire hasMatchSB = hasMatchSB_1 |  hasMatchSB_2;
wire hasMatchSB_1,hasMatchSB_2;
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'hBD,8'h00, 8'h08 }),
	.needmatches(8'd1)
	) match_bytes_SB_1(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchSB_1)
);
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'hAD,8'h00, 8'h08 }),
	.needmatches(8'd1)
	) match_bytes_SB_2(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchSB_2)
);
//------------------------------
// E7 detector
//-------------------------------

/*
bool CartDetector::isProbablyE7(const ByteBuffer& image, size_t size)
{
  // E7 cart bankswitching is triggered by accessing addresses
  // $FE0 to $FE6 using absolute non-indexed addressing
  // To eliminate false positives (and speed up processing), we
  // search for only certain known signatures
  // Thanks to "stella@casperkitty.com" for this advice
  // These signatures are attributed to the MESS project
  uInt8 signature[7][3] = {
	{ 0xAD, 0xE2, 0xFF },  // LDA $FFE2
	{ 0xAD, 0xE5, 0xFF },  // LDA $FFE5
	{ 0xAD, 0xE5, 0x1F },  // LDA $1FE5
	{ 0xAD, 0xE7, 0x1F },  // LDA $1FE7
	{ 0x0C, 0xE7, 0x1F },  // NOP $1FE7
	{ 0x8D, 0xE7, 0xFF },  // STA $FFE7
	{ 0x8D, 0xE7, 0x1F }   // STA $1FE7
  };
  for(uInt32 i = 0; i < 7; ++i)
	if(searchForBytes(image, size, signature[i], 3))
	  return true;

  return false;
}
*/
wire hasMatchE7_0 , hasMatchE7_1 , hasMatchE7_2 , hasMatchE7_3 , hasMatchE7_4 , hasMatchE7_5 , hasMatchE7_6;
wire hasMatchE7 = hasMatchE7_0 | hasMatchE7_1 | hasMatchE7_2 | hasMatchE7_3 | hasMatchE7_4 | hasMatchE7_5 | hasMatchE7_6;
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'hAD, 8'hE2 , 8'hFF }),
	.needmatches(8'd1)
	) match_bytes_E7_0(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchE7_0)
);
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'hAD, 8'hE5 , 8'hFF }),
	.needmatches(8'd1)
	) match_bytes_E7_1(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchE7_1)
);
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'hAD, 8'hE5 , 8'h1F }),
	.needmatches(8'd1)
	) match_bytes_E7_2(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchE7_2)
);
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'hAD, 8'hE7 , 8'h1F }),
	.needmatches(8'd1)
	) match_bytes_E7_3(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchE7_3)
);
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'h0C, 8'hE7 , 8'h1F }),
	.needmatches(8'd1)
	) match_bytes_E7_4(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchE7_4)
);
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'h8D, 8'hE7 , 8'hFF }),
	.needmatches(8'd1)
	) match_bytes_E7_5(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchE7_5)
);
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'hAD, 8'hE7 , 8'h1F }),
	.needmatches(8'd1)
	) match_bytes_E7_6(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchE7_6)
);

//------------------------------
// JANE detector
//-------------------------------
/*
bool CartDetector::isProbablyJANE(ByteSpan image)
{
  static constexpr std::array<uInt8, 4> signature = { 0xad, 0xf1, 0xff, 0x60 };  // LDA $FFF1
  return searchForBytes(image, signature);
}
*/
wire hasMatchJANE;
match_bytes #(
	.num_bytes(8'd4),
	.pattern({ 8'hAD, 8'hF1 , 8'hFF, 8'h60 }),
	.needmatches(8'd1)
	) match_bytes_JANE(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchJANE)
);

//------------------------------
// E0 detector
//-------------------------------
/*
bool CartDetector::isProbablyE0(const ByteBuffer& image, size_t size)
{
  // E0 cart bankswitching is triggered by accessing addresses
  // $FE0 to $FF9 using absolute non-indexed addressing
  // To eliminate false positives (and speed up processing), we
  // search for only certain known signatures
  // Thanks to "stella@casperkitty.com" for this advice
  // These signatures are attributed to the MESS project
  uInt8 signature[8][3] = {
	{ 0x8D, 0xE0, 0x1F },  // STA $1FE0
	{ 0x8D, 0xE0, 0x5F },  // STA $5FE0
	{ 0x8D, 0xE9, 0xFF },  // STA $FFE9
	{ 0x0C, 0xE0, 0x1F },  // NOP $1FE0
	{ 0xAD, 0xE0, 0x1F },  // LDA $1FE0
	{ 0xAD, 0xE9, 0xFF },  // LDA $FFE9
	{ 0xAD, 0xED, 0xFF },  // LDA $FFED
	{ 0xAD, 0xF3, 0xBF }   // LDA $BFF3
  };
  for(uInt32 i = 0; i < 8; ++i)
	if(searchForBytes(image, size, signature[i], 3))
	  return true;

  return false;
}
*/
wire  hasMatchE0_0 , hasMatchE0_1 , hasMatchE0_2 , hasMatchE0_3 , hasMatchE0_4 , hasMatchE0_5 , hasMatchE0_6 , hasMatchE0_7;
wire hasMatchE0 = hasMatchE0_0 | hasMatchE0_1 | hasMatchE0_2 | hasMatchE0_3 | hasMatchE0_4 | hasMatchE0_5 | hasMatchE0_6 | hasMatchE0_7;
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'h8D, 8'hE0 , 8'h1F }),
	.needmatches(8'd1)
	) match_bytes_E0_0(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchE0_0)
);
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'h8D, 8'hE0 , 8'h5F }),
	.needmatches(8'd1)
	) match_bytes_E0_1(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchE0_1)
);
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'h8D, 8'hE9 , 8'hFF }),
	.needmatches(8'd1)
	) match_bytes_E0_2(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchE0_2)
);
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'h0C, 8'hE0 , 8'h1F }),
	.needmatches(8'd1)
	) match_bytes_E0_3(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchE0_3)
);
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'hAD, 8'hE0 , 8'h1F }),
	.needmatches(8'd1)
	) match_bytes_E0_4(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchE0_4)
);
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'hAD, 8'hE9 , 8'hFF }),
	.needmatches(8'd1)
	) match_bytes_E0_5(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchE0_5)
);
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'hAD, 8'hED , 8'hFF }),
	.needmatches(8'd1)
	) match_bytes_E0_6(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchE0_6)
);
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'hAD, 8'hF3 , 8'hBF }),
	.needmatches(8'd1)
	) match_bytes_E0_7(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchE0_7)
);

//------------------------------
// FE detector
//-------------------------------
// we need to check is FE and not F8

wire hasMatchF8_0 , hasMatchF8_1;
wire hasMatchF8 = hasMatchF8_0 | hasMatchF8_1;
wire hasMatchFE_0 , hasMatchFE_1 , hasMatchFE_2 , hasMatchFE_3;
wire hasMatchFE = (~hasMatchF8) && (hasMatchFE_0 | hasMatchFE_1 | hasMatchFE_2 | hasMatchFE_3 );


match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'h8D, 8'hF9 , 8'h1F }),
	.needmatches(8'd2)
	) match_bytes_F8_0(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchF8_0)
);
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'h8D, 8'hF9 , 8'hFF }),
	.needmatches(8'd2)
	) match_bytes_F8_1(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchF8_1)
);


match_bytes #(
	.num_bytes(8'd5),
	.pattern({ 8'h20, 8'h00 , 8'hD0, 8'hC6 , 8'hC5 }),
	.needmatches(8'd1)
	) match_bytes_FE_0(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchFE_0)
);
match_bytes #(
	.num_bytes(8'd5),
	.pattern({ 8'h20, 8'hC3 , 8'hF8, 8'hA5 , 8'h82 }),
	.needmatches(8'd1)
	) match_bytes_FE_1(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchFE_1)
);
match_bytes #(
	.num_bytes(8'd5),
	.pattern({ 8'hD0, 8'hFB , 8'h20, 8'h73 , 8'hFE }),
	.needmatches(8'd1)
	) match_bytes_FE_2(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchFE_2)
);
match_bytes #(
	.num_bytes(8'd5),
	.pattern({ 8'h20, 8'h00 , 8'hF0, 8'h84 , 8'hD6 }),
	.needmatches(8'd1)
	) match_bytes_FE_3(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchFE_3)
);
/*



	// First check for *potential* F8
	uInt8 signature[2][3] = {
	  { 0x8D, 0xF9, 0x1F },  // STA $1FF9
	  { 0x8D, 0xF9, 0xFF }   // STA $FFF9
	};
	bool f8 = searchForBytes(image, size, signature[0], 3, 2) ||
			  searchForBytes(image, size, signature[1], 3, 2);


	else if(isProbablyFE(image, size) && !f8)
	  type = Bankswitch::Type::_FE;


bool CartDetector::isProbablyFE(const ByteBuffer& image, size_t size)
{
  // FE bankswitching is very weird, but always seems to include a
  // 'JSR $xxxx'
  // These signatures are attributed to the MESS project
  uInt8 signature[4][5] = {
	{ 0x20, 0x00, 0xD0, 0xC6, 0xC5 },  // JSR $D000; DEC $C5
	{ 0x20, 0xC3, 0xF8, 0xA5, 0x82 },  // JSR $F8C3; LDA $82
	{ 0xD0, 0xFB, 0x20, 0x73, 0xFE },  // BNE $FB; JSR $FE73
	{ 0x20, 0x00, 0xF0, 0x84, 0xD6 }   // JSR $F000; $84, $D6
  };
  for(uInt32 i = 0; i < 4; ++i)
	if(searchForBytes(image, size, signature[i], 5))
	  return true;

  return false;
}
*/

//------------------------------
// CV detector
//-------------------------------

wire hasMatchCV_0 , hasMatchCV_1;
wire hasMatchCV = hasMatchCV_0 | hasMatchCV_1;


match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'h9D, 8'hFF , 8'hF3 }),
	.needmatches(8'd1)
	) match_bytes_CV_0(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchCV_0)
);
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'h99, 8'h00 , 8'hF4 }),
	.needmatches(8'd1)
	) match_bytes_CV_1(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchCV_1)
);
/*
bool CartDetector::isProbablyCV(const ByteBuffer& image, size_t size)
{
  // CV RAM access occurs at addresses $f3ff and $f400
  // These signatures are attributed to the MESS project
  uInt8 signature[2][3] = {
	{ 0x9D, 0xFF, 0xF3 },  // STA $F3FF.X
	{ 0x99, 0x00, 0xF4 }   // STA $F400.Y
  };
  if(searchForBytes(image, size, signature[0], 3))
	return true;
  else
	return searchForBytes(image, size, signature[1], 3);
}
*/

//------------------------------
// UA detector
//-------------------------------

wire hasMatchUA_0 , hasMatchUA_1 , hasMatchUA_2 , hasMatchUA_3 , hasMatchUA_4 , hasMatchUA_5 , hasMatchUA_6 , hasMatchUA_7 , hasMatchUA_8 , hasMatchUA_9 , hasMatchUA_10 , hasMatchUA_11;
wire hasMatchUA = hasMatchUA_0 | hasMatchUA_1 | hasMatchUA_2 | hasMatchUA_3 | hasMatchUA_4 | hasMatchUA_5 | hasMatchUA_6 | hasMatchUA_7 | hasMatchUA_8 | hasMatchUA_9 | hasMatchUA_10 | hasMatchUA_11;
// Mickey (Digivision) has the two UA hotspots swapped, and LDA $2C0 is the
// only UA access it makes.
wire ua_mickey = hasMatchUA_5 & ~(hasMatchUA_0 | hasMatchUA_1 | hasMatchUA_2 | hasMatchUA_3 | hasMatchUA_4 | hasMatchUA_6 | hasMatchUA_7 | hasMatchUA_8 | hasMatchUA_9 | hasMatchUA_10 | hasMatchUA_11);

//----------------------------
//  0840 detector
//----------------------------
// Econobanking reads $0800 or $0840 (or NOPs through them) at least twice.
wire hasMatch0840_0, hasMatch0840_1, hasMatch0840_2, hasMatch0840_3, hasMatch0840_4;
wire hasMatch0840 = hasMatch0840_0 | hasMatch0840_1 | hasMatch0840_2 | hasMatch0840_3 | hasMatch0840_4;

match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'hAD, 8'h00, 8'h08 }),  // LDA $0800
	.needmatches(8'd2)
	) match_bytes_0840_0(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatch0840_0)
);
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'hAD, 8'h40, 8'h08 }),  // LDA $0840
	.needmatches(8'd2)
	) match_bytes_0840_1(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatch0840_1)
);
match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'h2C, 8'h00, 8'h08 }),  // BIT $0800
	.needmatches(8'd2)
	) match_bytes_0840_2(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatch0840_2)
);
match_bytes #(
	.num_bytes(8'd4),
	.pattern({ 8'h0C, 8'h00, 8'h08, 8'h4C }),  // NOP $0800; JMP
	.needmatches(8'd2)
	) match_bytes_0840_3(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatch0840_3)
);
match_bytes #(
	.num_bytes(8'd4),
	.pattern({ 8'h0C, 8'hFF, 8'h0F, 8'h4C }),  // NOP $0FFF; JMP
	.needmatches(8'd2)
	) match_bytes_0840_4(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatch0840_4)
);

//----------------------------
//  FC detector
//----------------------------
// Amiga Power Play: STA $1FF8 / $FFF8 then $FFF9 and the $FFFC trigger.
wire hasMatchFC_0, hasMatchFC_1, hasMatchFC_2;
wire hasMatchFC = hasMatchFC_0 | hasMatchFC_1 | hasMatchFC_2;

match_bytes #(
	.num_bytes(8'd6),
	.pattern({ 8'h8D, 8'hF8, 8'h1F, 8'h4A, 8'h4A, 8'h8D }),  // STA $1FF8; LSR; LSR; STA
	.needmatches(8'd1)
	) match_bytes_FC_0(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchFC_0)
);
match_bytes #(
	.num_bytes(8'd6),
	.pattern({ 8'h8D, 8'hF8, 8'hFF, 8'h8D, 8'hFC, 8'hFF }),  // STA $FFF8; STA $FFFC
	.needmatches(8'd1)
	) match_bytes_FC_1(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchFC_1)
);
match_bytes #(
	.num_bytes(8'd6),
	.pattern({ 8'h8C, 8'hF9, 8'hFF, 8'hAD, 8'hFC, 8'hFF }),  // STY $FFF9; LDA $FFFC
	.needmatches(8'd1)
	) match_bytes_FC_2(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchFC_2)
);

match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'h8D, 8'h40 , 8'h02 }),
	.needmatches(8'd1)
	) match_bytes_UA_0(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchUA_0)
);

match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'hAD, 8'h40 , 8'h02 }),
	.needmatches(8'd1)
	) match_bytes_UA_1(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchUA_1)
);


match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'hBD, 8'h1F , 8'h02 }),
	.needmatches(8'd1)
	) match_bytes_UA_2(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchUA_2)
);

match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'h2C, 8'hC0 , 8'h02 }),
	.needmatches(8'd1)
	) match_bytes_UA_3(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchUA_3)
);

match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'h8D, 8'hC0 , 8'h02 }),
	.needmatches(8'd1)
	) match_bytes_UA_4(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchUA_4)
);

match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'hAD, 8'hC0 , 8'h02 }),
	.needmatches(8'd1)
	) match_bytes_UA_5(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchUA_5)
);

match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'h2C, 8'hC0 , 8'h0F }),
	.needmatches(8'd1)
	) match_bytes_UA_6(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchUA_6)
);

match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'h2C, 8'hB0 , 8'h0F }),
	.needmatches(8'd1)
	) match_bytes_UA_7(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchUA_7)
);

match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'h2C, 8'hC0 , 8'h0F }),
	.needmatches(8'd1)
	) match_bytes_UA_8(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchUA_8)
);

match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'h8D, 8'hC0 , 8'h0F }),
	.needmatches(8'd1)
	) match_bytes_UA_9(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchUA_9)
);

match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'hAD, 8'hC0 , 8'h0F }),
	.needmatches(8'd1)
	) match_bytes_UA_10(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchUA_10)
);

match_bytes #(
	.num_bytes(8'd3),
	.pattern({ 8'h2C, 8'hC0 , 8'hEF }),
	.needmatches(8'd1)
	) match_bytes_UA_11(
	.enable(enable),
	.clk(clk),
	.reset(reset),
	.stream(match_next),
	.hasMatch(hasMatchUA_11)
);

/*
bool CartDetector::isProbablyUA(ByteSpan image)
{
  // UA cart bankswitching switches to bank 1 by accessing address 0x240
  // using 'STA $240' or 'LDA $240'.
  // Brazilian (Digivision) cart bankswitching switches to bank 1 by accessing
  // address 0x2C0 using 'BIT $2C0', 'STA $2C0', 'LDA $2C0' or 'BIT $FB0'
  static constexpr BSPF::array2D<uInt8, 11, 3> signature = {{
    { 0x8D, 0x40, 0x02 },  // STA $240 (Funky Fish, Pleiades)
    { 0xAD, 0x40, 0x02 },  // LDA $240 (???)
    { 0xBD, 0x1F, 0x02 },  // LDA $21F,X (Gingerbread Man)
    { 0x2C, 0xC0, 0x02 },  // BIT $2C0 (Time Pilot)
    { 0x8D, 0xC0, 0x02 },  // STA $2C0 (Fathom, Vanguard)
    { 0xAD, 0xC0, 0x02 },  // LDA $2C0 (Mickey)
    { 0x2C, 0xB0, 0x0F },  // BIT $FB0 (Digivision Beamrider)
    { 0x2C, 0xC0, 0x0F },  // BIT $FC0  (H.E.R.O., Kung-Fu Master)
    { 0x8D, 0xC0, 0x0F },  // STA $FC0  (Pole Position, Subterranea)
    { 0xAD, 0xC0, 0x0F },  // LDA $FC0  (Front Line, Zaxxon)
    { 0x2C, 0xC0, 0xEF }   // BIT $EFC0 (Motocross)
  }};
  return std::ranges::any_of(signature, [&](const auto& sig) {
    return searchForBytes(image, sig);
  });
}
*/

//------------------------------
// SC detector
//-------------------------------
//


/*
bool CartDetector::isProbablySC(const ByteBuffer& image, size_t size)
{
  // We assume a Superchip cart repeats the first 128 bytes for the second
  // 128 bytes in the RAM area, which is the first 256 bytes of each 4K bank
  const uInt8* ptr = image.get();
  while(size)
  {
	if(std::memcmp(ptr, ptr + 128, 128) != 0)
	  return false;

	ptr  += 4_KB;
	size -= 4_KB;
  }
  return true;
}
*/

// grab and save the CRC for the first 128 bytes
// each 4k check 128 bytes, and fail if CRC doesn't match
reg [31:0] sc_crc0,sc_crc1;
// DF and BF images name their scheme in the last 8 bytes: DFDF, DFSC, BFBF or
// BFSC. A 4K Superchip image puts SC at $FFA.
reg tail_dfbf, tail_dfbf_sc, sc4k_sig;
wire [31:0] tail_word = match_next[31:0];
wire tail_zone = load_addr + 25'd5 >= cart_size[24:0];
always @(posedge clk) begin
	if (load_start) begin
		tail_dfbf <= 1'b0;
		tail_dfbf_sc <= 1'b0;
		sc4k_sig <= 1'b0;
	end else if (enable) begin
		if (tail_zone && (tail_word == "DFDF" || tail_word == "BFBF"))
			tail_dfbf <= 1'b1;
		if (tail_zone && (tail_word == "DFSC" || tail_word == "BFSC")) begin
			tail_dfbf <= 1'b1;
			tail_dfbf_sc <= 1'b1;
		end
		if (load_addr == 25'h0FFB && match_next[15:0] == "SC")
			sc4k_sig <= 1'b1;
	end
end

reg has_sc;
wire [11:0] sc_offset = load_addr[11:0];
wire sc_enable = load_valid;

// 80 - 100

always @(posedge clk) begin
	if (load_start) begin
		sc_crc0 <= 32'b0;
		sc_crc1 <= 32'b0;
		has_sc <= 1'b1;
	end

	if (sc_enable) begin
		if (sc_offset < 12'h080) begin
			sc_crc0 <= nextCRC32_D8(data,
				(load_start || sc_offset == 12'h000) ? 32'b0 : sc_crc0);
		end else if (sc_offset < 12'h100) begin
			sc_crc1 <= nextCRC32_D8(data,
				sc_offset == 12'h080 ? 32'b0 : sc_crc1);
		end else if (sc_offset == 12'h100) begin
			if (has_sc)
				has_sc <= sc_crc0 == sc_crc1;
			sc_crc0 <= 32'b0;
			sc_crc1 <= 32'b0;
		end
	end
end




////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 1999-2008 Easics NV.
// This source file may be used and distributed without restriction
// provided that this copyright statement is not removed from the file
// and that any derivative work contains the original copyright notice
// and the associated disclaimer.
//
// THIS SOURCE FILE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS
// OR IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
// WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Purpose : synthesizable CRC function
//   * polynomial: x^32 + x^26 + x^23 + x^22 + x^16 + x^12 + x^11 + x^10 + x^8 + x^7 + x^5 + x^4 + x^2 + x^1 + 1
//   * data width: 8
//
// Info : tools@easics.be
//        http://www.easics.com
////////////////////////////////////////////////////////////////////////////////
  // polynomial: x^32 + x^26 + x^23 + x^22 + x^16 + x^12 + x^11 + x^10 + x^8 + x^7 + x^5 + x^4 + x^2 + x^1 + 1
  // data width: 8
  // convention: the first serial bit is D[7]
  function [31:0] nextCRC32_D8;

	input [7:0] Data;
	input [31:0] crc;
	reg [7:0] d;
	reg [31:0] c;
	reg [31:0] newcrc;
  begin
	d = Data;
	c = crc;

	newcrc[0] = d[6] ^ d[0] ^ c[24] ^ c[30];
	newcrc[1] = d[7] ^ d[6] ^ d[1] ^ d[0] ^ c[24] ^ c[25] ^ c[30] ^ c[31];
	newcrc[2] = d[7] ^ d[6] ^ d[2] ^ d[1] ^ d[0] ^ c[24] ^ c[25] ^ c[26] ^ c[30] ^ c[31];
	newcrc[3] = d[7] ^ d[3] ^ d[2] ^ d[1] ^ c[25] ^ c[26] ^ c[27] ^ c[31];
	newcrc[4] = d[6] ^ d[4] ^ d[3] ^ d[2] ^ d[0] ^ c[24] ^ c[26] ^ c[27] ^ c[28] ^ c[30];
	newcrc[5] = d[7] ^ d[6] ^ d[5] ^ d[4] ^ d[3] ^ d[1] ^ d[0] ^ c[24] ^ c[25] ^ c[27] ^ c[28] ^ c[29] ^ c[30] ^ c[31];
	newcrc[6] = d[7] ^ d[6] ^ d[5] ^ d[4] ^ d[2] ^ d[1] ^ c[25] ^ c[26] ^ c[28] ^ c[29] ^ c[30] ^ c[31];
	newcrc[7] = d[7] ^ d[5] ^ d[3] ^ d[2] ^ d[0] ^ c[24] ^ c[26] ^ c[27] ^ c[29] ^ c[31];
	newcrc[8] = d[4] ^ d[3] ^ d[1] ^ d[0] ^ c[0] ^ c[24] ^ c[25] ^ c[27] ^ c[28];
	newcrc[9] = d[5] ^ d[4] ^ d[2] ^ d[1] ^ c[1] ^ c[25] ^ c[26] ^ c[28] ^ c[29];
	newcrc[10] = d[5] ^ d[3] ^ d[2] ^ d[0] ^ c[2] ^ c[24] ^ c[26] ^ c[27] ^ c[29];
	newcrc[11] = d[4] ^ d[3] ^ d[1] ^ d[0] ^ c[3] ^ c[24] ^ c[25] ^ c[27] ^ c[28];
	newcrc[12] = d[6] ^ d[5] ^ d[4] ^ d[2] ^ d[1] ^ d[0] ^ c[4] ^ c[24] ^ c[25] ^ c[26] ^ c[28] ^ c[29] ^ c[30];
	newcrc[13] = d[7] ^ d[6] ^ d[5] ^ d[3] ^ d[2] ^ d[1] ^ c[5] ^ c[25] ^ c[26] ^ c[27] ^ c[29] ^ c[30] ^ c[31];
	newcrc[14] = d[7] ^ d[6] ^ d[4] ^ d[3] ^ d[2] ^ c[6] ^ c[26] ^ c[27] ^ c[28] ^ c[30] ^ c[31];
	newcrc[15] = d[7] ^ d[5] ^ d[4] ^ d[3] ^ c[7] ^ c[27] ^ c[28] ^ c[29] ^ c[31];
	newcrc[16] = d[5] ^ d[4] ^ d[0] ^ c[8] ^ c[24] ^ c[28] ^ c[29];
	newcrc[17] = d[6] ^ d[5] ^ d[1] ^ c[9] ^ c[25] ^ c[29] ^ c[30];
	newcrc[18] = d[7] ^ d[6] ^ d[2] ^ c[10] ^ c[26] ^ c[30] ^ c[31];
	newcrc[19] = d[7] ^ d[3] ^ c[11] ^ c[27] ^ c[31];
	newcrc[20] = d[4] ^ c[12] ^ c[28];
	newcrc[21] = d[5] ^ c[13] ^ c[29];
	newcrc[22] = d[0] ^ c[14] ^ c[24];
	newcrc[23] = d[6] ^ d[1] ^ d[0] ^ c[15] ^ c[24] ^ c[25] ^ c[30];
	newcrc[24] = d[7] ^ d[2] ^ d[1] ^ c[16] ^ c[25] ^ c[26] ^ c[31];
	newcrc[25] = d[3] ^ d[2] ^ c[17] ^ c[26] ^ c[27];
	newcrc[26] = d[6] ^ d[4] ^ d[3] ^ d[0] ^ c[18] ^ c[24] ^ c[27] ^ c[28] ^ c[30];
	newcrc[27] = d[7] ^ d[5] ^ d[4] ^ d[1] ^ c[19] ^ c[25] ^ c[28] ^ c[29] ^ c[31];
	newcrc[28] = d[6] ^ d[5] ^ d[2] ^ c[20] ^ c[26] ^ c[29] ^ c[30];
	newcrc[29] = d[7] ^ d[6] ^ d[3] ^ c[21] ^ c[27] ^ c[30] ^ c[31];
	newcrc[30] = d[7] ^ d[4] ^ c[22] ^ c[28] ^ c[31];
	newcrc[31] = d[5] ^ c[23] ^ c[29];
	nextCRC32_D8 = newcrc;
  end
  endfunction

endmodule: detect2600

module match_bytes
(
	input clk,
	input reset,
	input  enable,
	input  [63:0] stream,  // shared byte history, newest byte in [7:0]
	output reg hasMatch
);

parameter [7:0] num_bytes = 8'd1;
parameter [(num_bytes*8)-1:0] pattern = 0;
parameter [7:0] needmatches=8'b1;

reg [7:0] curMatch;

always @(posedge clk)
begin
	if (reset) begin
		curMatch <= 0;
		hasMatch <= 0;
	end

	if (enable) begin
		if (stream[(num_bytes*8)-1:0] == pattern) begin
			if (reset || !hasMatch) begin
				if (reset || curMatch < needmatches)
					curMatch <= (reset ? 8'd0 : curMatch) + 8'd1;
				if ((reset ? 8'd0 : curMatch) >= needmatches - 8'd1)
					hasMatch <= 1'b1;
			end
		end
	end
end

endmodule: match_bytes
