// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jamie Blanks

// Souper-profile ARM memory map, per
// .agents/formats/BUPCHIP_FIRMWARE_CONTRACT.md section 3.
//
//   0x00000000-0x00003FFF  firmware ROM, read-only
//   0x02000000+            ARSC asset window, read-only, external
//   0x40000000-0x40003FFF  private RAM
//   0xE0009000-0xE00090FF  BupChip peripheral
//   everything else        data abort
//
// Anything outside those windows aborts rather than reading zero: a silent zero
// is how a pointer bug turns into a mystery, and the firmware has no business
// touching anything else.
//
// The asset window is deliberately external. It is 212 KiB for Rikki & Vikki,
// which belongs in DDR3, not in M10K.

module bupchip_memory #(
	parameter int ROM_WORDS = 4096,		// 16 KiB
	parameter int RAM_WORDS = 4096,		// 16 KiB
	// Firmware image. Quartus reads the .mif at synthesis, $readmemh reads the
	// .hex in simulation, and both are produced from one binary by
	// .agents/firmware/bupchip/. If these disagree the core behaves differently
	// on hardware than it does in the model.
	parameter     ROM_MIF   = "bupchip.mif",
	parameter     ROM_INIT  = "rtl/bupchip.hex"
) (
	input  logic        clk,
	input  logic        reset,

	// ARM bus.
	// The console's run enable, the same one arm_host takes. Without it the
	// one-cycle answer below is retired while the CPU is paused, and the
	// request it still holds is then accepted again on every pass.
	input  logic        mem_ce,
	input  logic        mem_req,
	input  logic [31:0] mem_addr,
	input  logic        mem_write,
	input  logic [31:0] mem_wdata,
	input  logic  [3:0] mem_wstrb,
	input  logic  [1:0] mem_size,
	output logic        mem_ready,
	output logic        mem_abort,
	output logic [31:0] mem_rdata,

	// Asset window. One word per request, answered with asset_valid.
	output logic        asset_req,
	output logic [31:0] asset_addr,
	input  logic [31:0] asset_rdata,
	input  logic        asset_valid,
	input  logic [31:0] asset_size,

	// Peripheral.
	output logic        reg_sel,
	output logic  [7:0] reg_addr,
	output logic        reg_write,
	output logic [31:0] reg_wdata,
	input  logic [31:0] reg_rdata
);
	// Named rather than written inline: Quartus 17.0.2's SystemVerilog support
	// is partial, and a computed bound inside a part-select is the kind of
	// thing it handles less predictably than a plain localparam.
	localparam int ROM_AW = $clog2(ROM_WORDS);
	localparam int RAM_AW = $clog2(RAM_WORDS);

	localparam logic [31:0] ASSET_BASE = 32'h02000000;
	localparam logic [31:0] MMIO_BASE  = 32'hE0009000;

	typedef enum logic [1:0] { S_IDLE, S_ANSWER, S_ASSET } state_e;
	state_e state;

	// One request is taken at a time; everything below keys off this.
	wire accept = mem_req && state == S_IDLE && !reset;

	// AGENTS.md: memories over 64 bytes use a provided block-RAM wrapper.
	// 16 KiB with byte enables is cache_ram_tdp_dc_be; only the A port is used,
	// because the ARM is the only master on this RAM.
	logic [31:0] ram_q;
	logic [RAM_AW-1:0] ram_addr;
	logic        ram_wren;

	assign ram_addr = mem_addr[RAM_AW+1:2];
	assign ram_wren = accept && dec_ram && !dec_bad && mem_write;

	cache_ram_tdp_dc_be #(
		.ADDR_WIDTH (RAM_AW),
		.DATA_WIDTH (32)
	) private_ram (
		.clk_a_i     (clk),
		.addr_a_i    (ram_addr),
		.wren_a_i    (ram_wren),
		.byteena_a_i (mem_wstrb),
		.wdata_a_i   (mem_wdata),
		.q_a_o       (ram_q),
		.clk_b_i     (clk),
		.addr_b_i    ({RAM_AW{1'b0}}),
		.wren_b_i    (1'b0),
		.byteena_b_i (4'b0),
		.wdata_b_i   (32'b0),
		.q_b_o       ()
	);

	// The firmware ROM uses the same block-RAM wrapper as every other
	// initialised memory in the core, so it picks up its contents the same way.
	logic [31:0] rom_q;
	logic [ROM_AW-1:0] rom_addr;

	spram #(
		.addr_width    (ROM_AW),
		.data_width    (32),
		.mem_init_file (ROM_MIF),
		.sim_init_file (ROM_INIT),
		.mem_name      ("BupChipROM")
	) firmware_rom (
		.clock   (clk),
		.address (rom_addr),
		.data    (32'b0),
		.wren    (1'b0),
		.q       (rom_q),
		.cs      (1'b1)
	);

	// Address decode of the request being offered this cycle.
	wire dec_rom   = mem_addr < (ROM_WORDS * 4);
	wire dec_ram   = mem_addr >= 32'h40000000 &&
	                 mem_addr <  32'h40000000 + (RAM_WORDS * 4);
	wire dec_asset = mem_addr >= ASSET_BASE && mem_addr < ASSET_BASE + asset_size;
	wire dec_mmio  = mem_addr >= MMIO_BASE && mem_addr < MMIO_BASE + 32'h100;

	// ROM is immutable to the ARM and the asset window is read-only, so a write
	// to either is a firmware bug and aborts rather than being dropped.
	wire dec_bad = mem_size == 2'b11 ||
	               (mem_write && (dec_rom || dec_asset)) ||
	               !(dec_rom || dec_ram || dec_asset || dec_mmio);

	// No req_rom: ROM is the default arm of the read mux below, so a flag for
	// it would have no reader. Quartus 10036 caught it after the spram move.
	logic        req_ram, req_asset, req_mmio, req_bad;
	logic [31:0] req_addr;
	logic [31:0] mmio_q;

	// spram registers its read, so present the address in the accept cycle and
	// its q is valid in the answer cycle - the same shape the RAM path uses.
	assign rom_addr = mem_addr[ROM_AW+1:2];


	assign asset_req  = state == S_ASSET;
	assign asset_addr = req_addr - ASSET_BASE;

	// The peripheral is combinational, so present it in the accept cycle and
	// let it settle while the state machine answers.
	assign reg_sel   = accept && dec_mmio;
	assign reg_addr  = mem_addr[7:0];
	assign reg_write = mem_write;
	assign reg_wdata = mem_wdata;

	always_ff @(posedge clk) begin
		if (reset) begin
			state <= S_IDLE;
			req_ram <= 1'b0;
			req_asset <= 1'b0; req_mmio <= 1'b0; req_bad <= 1'b0;
		end else begin
			case (state)
				S_IDLE: if (mem_req) begin
					req_addr  <= mem_addr;
					req_ram   <= dec_ram   && !dec_bad;
					req_asset <= dec_asset && !dec_bad;
					req_mmio  <= dec_mmio  && !dec_bad;
					req_bad   <= dec_bad;

					if (reg_sel)
						mmio_q <= reg_rdata;

					state <= (dec_asset && !dec_bad) ? S_ASSET : S_ANSWER;
				end

				S_ASSET: if (asset_valid) state <= S_ANSWER;

				S_ANSWER: if (mem_ce) state <= S_IDLE;

				default: state <= S_IDLE;
			endcase
		end
	end

	logic [31:0] asset_q;
	always_ff @(posedge clk)
		if (state == S_ASSET && asset_valid) asset_q <= asset_rdata;

	assign mem_ready = state == S_ANSWER;
	assign mem_abort = state == S_ANSWER && req_bad;
	// All-zero until the first request after reset, so top.sv can OR this with
	// the 2600 mappers' answer instead of muxing on the cartridge profile: the
	// BupChip is held in reset whenever they own the CPU. `live` sets on the
	// request edge, so it is already up in the answer cycle that follows, and
	// after an answer the data holds as it always did. The one-hot form keeps
	// every source one level from the output.
	logic live;
	always_ff @(posedge clk)
		if (reset) live <= 1'b0;
		else if (mem_req) live <= 1'b1;
	wire ans_rom = live && !req_ram && !req_asset && !req_mmio;
	assign mem_rdata =
		({32{live && req_ram}}   & ram_q) |
		({32{live && req_asset}} & asset_q) |
		({32{live && req_mmio}}  & mmio_q) |
		({32{ans_rom}}           & rom_q);
endmodule
