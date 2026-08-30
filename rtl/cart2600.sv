// SPDX-License-Identifier: MIT
// Copyright (c) 2019-2026 Jamie Blanks

module cart2600
(
	// Physical Pins
	output logic [7:0]  d_out, // Data bus
	input    [7:0]  d_in,  // Data bus
	input    [12:0] a_in,  // Address bus
	input           rw,

	// Helpers
	input           clk,        // Master Clock
	input           reset,      // System warm reset
	input           ce,         // Original system clock enable (~3.579mhz) used to divide into crystals
	input           phi1,       // CPU Phase 1 Signal (used for FE to catch data at the right moment)
	input           phi2,
	output   [7:0]  oe,         // Output Enable mask
	input    [7:0]  open_bus,   // Input open bus to use when not driving data bus (Obselete, use oe)

	// Autodetect info
	input           sc,         // Superchip Enable
	input    [4:0]  mapper,     // Bankswitching type (ie Mapper)
	input    [2:0]  mapper_revision,
	input           cdf_ldx,
	input           cdf_ldy,
	input           cdf_fetch_offset_enable,
	input    [7:0]  cdf_fetch_offset,
	input   [31:0]  cdfj_entry,
	input   [31:0]  cdfj_stack,
	input   [15:0]  arm_audio_size_addr,
	
	// SDRAM ROM storage interface
	input    [7:0]  rom_do,     // Incoming ROM data from the sdram
	input   [31:0]  rom_size,   // Full rom size for address masking
	output  [18:0]  rom_a,      // Outgoing absolute rom address for image.
	output          rom_read,   // Initiate read from SDRAM
	
	output   [17:0] cartram_addr,
	output          cartram_wr,
	output          cartram_rd,
	output   [7:0]  cartram_wrdata,
	input    [7:0]  cartram_data,
	input           clk_arm,
	input           reset_arm,
	input   [31:0]  cartram_word_data,
	input           load_start,
	input   [24:0]  load_addr,
	input           load_valid,
	input    [7:0]  load_data,
	input           load_end,
	output          load_wait,
	input   [15:0]  mapper_ram_size,
	output          mapper_init_busy,

	// Merged ARM side of the shared cartridge RAM. cart_ram_tdp stays in the
	// core because the 7800 path shares it, but the table writeback and the
	// ARM contend for it here, where both of them live.
	output          arm_ram_en,
	output          arm_ram_write,
	output  [14:0]  arm_ram_addr,
	output  [31:0]  arm_ram_wdata,
	output   [3:0]  arm_ram_wstrb,
	input           arm_ram_accepted,
	input   [31:0]  arm_ram_rdata,

	// DDR3 channel to the core's bridge.
	output  [28:0]  ddr_addr,
	output  [63:0]  ddr_din,
	output   [7:0]  ddr_be,
	output   [7:0]  ddr_len,
	output          ddr_req,
	output          ddr_rnw,
	input           ddr_ack,
	input   [63:0]  ddr_dout,
	input           ddr_rvalid,

	// ARM7TDMI bus, from arm_host in the core.
	output          halt_req,
	input           cpu_halted,
	input           mem_req,
	output          mem_ready,
	output          mem_abort,
	input   [31:0]  mem_addr,
	input           mem_write,
	input   [31:0]  mem_wdata,
	output  [31:0]  mem_rdata,
	input    [1:0]  mem_size,
	input    [3:0]  mem_wstrb,
	input           mem_fetch,
	output          state_req,
	output          state_write,
	output   [5:0]  state_index,
	output  [31:0]  state_wdata,
	input   [31:0]  state_rdata,
	input           state_ready,
	output          state_commit,

	// The 6507 is held while either of these runs; the core gates its clock.
	output          arm_call_busy,
	output          arm_dma_busy,
	output          fa2_nvram_request,
	output          fa2_nvram_write,
	output   [7:0]  fa2_nvram_addr,
	output   [7:0]  fa2_nvram_wdata,
	input    [7:0]  fa2_nvram_rdata,
	input           fa2_nvram_ready,
	output          fa2_nvram_dirty,

	output          bus_stuff_valid,
	output   [7:0]  bus_stuff_data,

	// Tape Signals
	output          tape_audio, // Tape audio output
	input    [1:0]  tape_in,    // ADC tape input
	input           fix_sc_cs   // Fix Supercharger Checksums menu option
);
	`define NUM_MAPPERS BANKEND

	// Muxxing signals
	logic [18:0] rom_addr[`NUM_MAPPERS];
	logic [7:0] direct_do[`NUM_MAPPERS];
	logic [15:0] flags_out[`NUM_MAPPERS]; // Flag bit 0 is direct_do in use, bit 1 is output enable used;
	logic [7:0]  out_en[`NUM_MAPPERS];
	logic        ram_rw[`NUM_MAPPERS];
	logic        ram_sel[`NUM_MAPPERS];
	logic [17:0] ram_a[`NUM_MAPPERS];
	logic [12:0] old_ain;
	logic [7:0]  bg_data;
	logic        ar_read;
	logic [7:0]  cr_do;


	logic [18:0] sel_rom_addr;
	logic [7:0] sel_direct_do;
	logic [15:0] sel_flags_out;
	logic [7:0]  sel_out_en;
	logic        sel_ram_rw;
	logic        sel_ram_sel;
	logic [17:0] sel_ram_a;
	logic [18:0] rom_mask;

	assign rom_mask = rom_size[18:0] - 1'd1;
	assign rom_read = mapper == BANKAR ? ar_read : ~address_change;
	wire is_bad_game = mapper == BANKELF ||
		(mapper == BANKBUS && mapper_revision == 3'd0);

	// Handle unsupportable ARM mappers :(
	spram #(
		.addr_width(11),
		.mem_init_file("ooo.mif"),
		.sim_init_file("rtl/ooo.hex")
	) badgame_ram
	(
		.clock      (clk),
		.address    (a_in[10:0]),
		.data       (8'd0),
		.wren       (1'b0),
		.cs         (1'b1),
		.q          (bg_data)
	);

	// Flags:
	// bit 0 - direct_do in use
	// bit 1 - bitwise & direct_do and rom_do
	assign sel_flags_out = flags_out[mapper];
	assign sel_direct_do = direct_do[mapper];
	assign sel_out_en = out_en[mapper];
	assign sel_ram_rw = ram_rw[mapper];
	assign sel_ram_sel = ram_sel[mapper];
	assign sel_ram_a = ram_a[mapper];
	assign rom_a = rom_addr[mapper] & ((mapper == BANKE7 || mapper == BANK3F) ? rom_mask : {19{1'b1}});
	assign oe = out_en[mapper];

	always_comb begin
		d_out = open_bus;
		if (is_bad_game)
			d_out = bg_data;
		else if (|sel_out_en) begin
			if (sel_flags_out[0])
				d_out = sel_direct_do;
			else if (sel_flags_out[1])
				d_out = (sel_direct_do & rom_do);
			else if (sel_ram_sel) begin
				if (sel_ram_rw)
					d_out = cr_do;
			end else
				d_out = rom_do;
		end
	end

	// Since atari added no clock signal to the cart slot, for most mappers this will be the
	// primary way that they detected when to take action. The address changes typically
	// occur just before or just after phi2 on a real system. On some 7800 systems, A12 is delayed
	// in an atypical way causing this to trigger incorrectly for some games, however this
	// design does not reproduce that issue.
	wire address_change = old_ain != a_in;

	// High from the clock edge that took this access until the address moves
	// on, which is what closes the cartridge RAM write strobe above.
	logic access_taken;
	always @(posedge clk) begin
		if (reset || address_change)
			access_taken <= 1'b0;
		else if (phi2)
			access_taken <= 1'b1;
	end

	always @(posedge clk) begin :reset_2600_cart
		old_ain <= a_in;
	end

	// Bank CTY is compatible with F4 minus the ARM enhanced music
	assign direct_do[BANKCTY]     = direct_do[BANKF4];
	assign flags_out[BANKCTY]     = flags_out[BANKF4];
	assign out_en[BANKCTY]        = out_en[BANKF4];
	assign ram_sel[BANKCTY]       = ram_sel[BANKF4];
	assign ram_rw[BANKCTY]        = ram_rw[BANKF4];
	assign ram_a[BANKCTY]         = ram_a[BANKF4];
	assign rom_addr[BANKCTY]      = rom_addr[BANKF4];

	logic [5:0] cdf_table_index;
	logic [5:0] bus_pointer_index;
	logic [5:0] bus_increment_index;
	logic [5:0] table_pointer_index;
	logic [5:0] table_increment_index;
	logic [31:0] table_pointer;
	logic [31:0] table_increment;
	logic [14:0] table_pointer_base;
	logic [14:0] table_increment_base;
	logic [14:0] table_map_base;
	logic [5:0] table_stream_count;
	logic cdf_pointer_update;
	logic [5:0] cdf_pointer_update_index;
	logic [31:0] cdf_pointer_update_value;
	logic bus_pointer_update;
	logic [5:0] bus_pointer_update_index;
	logic [31:0] bus_pointer_update_value;
	logic bus_map_update;
	logic bus_map_update_selected;
	logic [5:0] bus_map_update_index;
	logic [31:0] bus_map_update_value;
	logic bus_stuff_valid_raw;
	logic [7:0] bus_stuff_data_raw;
	logic table_pointer_write;
	logic [5:0] table_pointer_write_index;
	logic [31:0] table_pointer_write_value;
	logic [1:0] table_family;
	logic [1:0] init_family;
	logic init_ram_en;
	logic [16:0] init_ram_addr;
	logic init_table_pointer_write;
	logic [5:0] init_table_pointer_index;
	logic [31:0] init_table_pointer_wdata;
	logic init_table_increment_write;
	logic [5:0] init_table_increment_index;
	logic [31:0] init_table_increment_wdata;
	logic init_table_map_write;
	logic [5:0] init_table_map_index;
	logic [31:0] init_table_map_wdata;
	logic mapper_wb_idle;
	logic mapper_call_ready;
	logic cdf_ram_en;
	logic cdf_ram_write;
	logic [14:0] cdf_ram_addr;
	logic [7:0] cdf_ram_wdata;
	logic bus_ram_en;
	logic bus_ram_write;
	logic [14:0] bus_ram_addr;
	logic [7:0] bus_ram_wdata;
	logic dpc_call_request;
	logic [31:0] dpc_call_entry;
	logic [31:0] dpc_call_stack;
	logic dpc_call_thumb;
	logic dpc_service_request_raw;
	logic fa2_nvram_request_raw;
	logic fa2_nvram_write_raw;
	logic [7:0] fa2_nvram_addr_raw;
	logic [7:0] fa2_nvram_wdata_raw;
	logic fa2_nvram_dirty_raw;
	logic cdf_call_request;
	logic [31:0] cdf_call_entry;
	logic [31:0] cdf_call_stack;
	logic cdf_call_thumb;
	logic bus_call_request;
	logic [31:0] bus_call_entry;
	logic [31:0] bus_call_stack;
	logic bus_call_thumb;
	logic [6:0] dpc_audio_waveform0;
	logic [6:0] dpc_audio_waveform1;
	logic [6:0] dpc_audio_waveform2;
	logic dpc_audio_note_write;
	logic [1:0] dpc_audio_note_voice;
	logic [7:0] dpc_audio_note_value;
	logic bus_digital_audio;
	logic cdf_digital_audio;
	logic [7:0] arm_audio_amplitude;
	logic fast_jump_valid;
	logic [14:0] fast_jump_query_addr;
	logic audio_ram_en;
	logic [16:0] audio_ram_addr;
	logic audio_ram_grant;
	logic load_end_d = 1'b0;

	// The ARM service the DPC+, BUS and CDF front ends call into. It sits here
	// with them, so only the CPU bus, the DDR3 channel and the shared RAM port
	// cross back out to the core.
	logic        arm_shadow_ready;
	logic        arm_cartram_en, arm_cartram_write, arm_cartram_accepted;
	logic [14:0] arm_cartram_addr;
	logic [31:0] arm_cartram_wdata;
	logic  [3:0] arm_cartram_wstrb;
	logic        mapper_dma_request, mapper_dma_fill;
	logic [24:0] mapper_dma_source;
	logic [16:0] mapper_dma_dest;
	logic [17:0] mapper_dma_count;
	logic  [7:0] mapper_dma_value;
	logic        mapper_dma_ready, mapper_dma_done;
	logic        mapper_wb_en, mapper_wb_write;
	logic [14:0] mapper_wb_addr;
	logic [31:0] mapper_wb_wdata;
	logic  [3:0] mapper_wb_wstrb;
	logic        mapper_wb_accepted;
	logic        dpc_service_request, dpc_service_fill;
	logic [18:0] dpc_service_source;
	logic [14:0] dpc_service_dest;
	logic  [7:0] dpc_service_count, dpc_service_value;
	logic        dpc_service_ready;
	logic        arm_dma_request, arm_dma_fill;
	logic [24:0] arm_dma_source;
	logic [16:0] arm_dma_dest;
	logic [17:0] arm_dma_count;
	logic  [7:0] arm_dma_value;
	logic        arm_dma_ready, arm_dma_done;
	logic        arm_call_request, arm_call_thumb;
	logic [31:0] arm_call_entry, arm_call_stack;
	logic        arm_call_ready, arm_call_done;
	logic [31:0] arm_audio_counter0, arm_audio_counter1, arm_audio_counter2;
	logic [31:0] arm_audio_frequency0, arm_audio_frequency1, arm_audio_frequency2;
	logic [31:0] arm_audio_counter0_return, arm_audio_counter1_return;
	logic [31:0] arm_audio_counter2_return;
	logic [31:0] arm_audio_frequency0_return, arm_audio_frequency1_return;
	logic [31:0] arm_audio_frequency2_return;
	logic        arm_sample_request, arm_sample_ready, arm_sample_busy;
	logic        arm_sample_done;
	logic [24:0] arm_sample_addr;
	logic  [7:0] arm_sample_data;

	// The ARM has one DMA engine. The mapper's own initialization owns it while
	// it runs; afterwards it belongs to the DPC+ fetcher service.
	assign arm_dma_request = mapper_init_busy ? mapper_dma_request :
		dpc_service_request;
	assign arm_dma_fill = mapper_init_busy ? mapper_dma_fill : dpc_service_fill;
	assign arm_dma_source = mapper_init_busy ? mapper_dma_source :
		{6'b0, dpc_service_source};
	assign arm_dma_dest = mapper_init_busy ? mapper_dma_dest :
		{2'b0, dpc_service_dest};
	assign arm_dma_count = mapper_init_busy ? mapper_dma_count :
		{10'b0, dpc_service_count};
	assign arm_dma_value = mapper_init_busy ? mapper_dma_value : dpc_service_value;
	assign mapper_dma_ready = mapper_init_busy && arm_dma_ready;
	assign mapper_dma_done = mapper_init_busy && arm_dma_done;
	assign dpc_service_ready = !mapper_init_busy && arm_dma_ready;

	// Table writeback and the ARM share one port into the cartridge RAM;
	// writeback wins while it is asking.
	assign arm_ram_en = mapper_wb_en || arm_cartram_en;
	assign arm_ram_write = mapper_wb_en ? mapper_wb_write : arm_cartram_write;
	assign arm_ram_addr = mapper_wb_en ? mapper_wb_addr : arm_cartram_addr;
	assign arm_ram_wdata = mapper_wb_en ? mapper_wb_wdata : arm_cartram_wdata;
	assign arm_ram_wstrb = mapper_wb_en ? mapper_wb_wstrb : arm_cartram_wstrb;
	assign arm_cartram_accepted = arm_cartram_en && !mapper_wb_en &&
		arm_ram_accepted;
	assign mapper_wb_accepted = mapper_wb_en && arm_ram_accepted;

`ifndef NO_ARM_MAPPER
	arm_mapper_subsystem arm_mappers (
		.clk_sys         (clk),
		.reset_sys       (reset_arm),
		.mapper_reset    (reset),
		.load_start,
		.load_addr,
		.load_valid,
		.load_end,
		.load_size       (rom_size),
		.load_data,
		.load_wait,
		.clk_arm,
		.reset_arm,
		.shadow_ready    (arm_shadow_ready),
		.mapper_ram_size,
		.dma_request     (arm_dma_request),
		.dma_fill        (arm_dma_fill),
		.dma_source      (arm_dma_source),
		.dma_dest        (arm_dma_dest),
		.dma_count       (arm_dma_count),
		.dma_value       (arm_dma_value),
		.dma_ready       (arm_dma_ready),
		.dma_busy        (arm_dma_busy),
		.dma_done        (arm_dma_done),
		.sample_request  (arm_sample_request),
		.sample_addr     (arm_sample_addr),
		.sample_ready    (arm_sample_ready),
		.sample_busy     (arm_sample_busy),
		.sample_done     (arm_sample_done),
		.sample_data     (arm_sample_data),
		.call_request    (arm_call_request),
		.call_entry      (arm_call_entry),
		.call_stack      (arm_call_stack),
		.call_thumb      (arm_call_thumb),
		.audio_counter0  (arm_audio_counter0),
		.audio_counter1  (arm_audio_counter1),
		.audio_counter2  (arm_audio_counter2),
		.audio_frequency0(arm_audio_frequency0),
		.audio_frequency1(arm_audio_frequency1),
		.audio_frequency2(arm_audio_frequency2),
		.call_ready      (arm_call_ready),
		.call_busy       (arm_call_busy),
		.call_done       (arm_call_done),
		.audio_counter0_return  (arm_audio_counter0_return),
		.audio_counter1_return  (arm_audio_counter1_return),
		.audio_counter2_return  (arm_audio_counter2_return),
		.audio_frequency0_return(arm_audio_frequency0_return),
		.audio_frequency1_return(arm_audio_frequency1_return),
		.audio_frequency2_return(arm_audio_frequency2_return),
		.cpu_halted,
		.ram_en          (arm_cartram_en),
		.ram_write       (arm_cartram_write),
		.ram_addr        (arm_cartram_addr),
		.ram_wdata       (arm_cartram_wdata),
		.ram_wstrb       (arm_cartram_wstrb),
		.ram_accepted    (arm_cartram_accepted),
		.ram_rdata       (arm_ram_rdata),
		.ddr_addr,
		.ddr_din,
		.ddr_be,
		.ddr_len,
		.ddr_req,
		.ddr_rnw,
		.ddr_ack,
		.ddr_dout,
		.ddr_rvalid,
		.halt_req,
		.mem_req,
		.mem_ready,
		.mem_abort,
		.mem_addr,
		.mem_write,
		.mem_wdata,
		.mem_rdata,
		.mem_size,
		.mem_wstrb,
		.mem_fetch,
		.state_req,
		.state_write,
		.state_index,
		.state_wdata,
		.state_rdata,
		.state_ready,
		.state_commit
	);
`else
	// Simulation builds that leave the ARM sources out.
	assign load_wait = 1'b0;
	assign arm_shadow_ready = 1'b0;
	assign arm_dma_ready = 1'b0;
	assign arm_dma_busy = 1'b0;
	assign arm_dma_done = 1'b0;
	assign arm_sample_ready = 1'b0;
	assign arm_sample_busy = 1'b0;
	assign arm_sample_done = 1'b0;
	assign arm_sample_data = 8'b0;
	assign arm_call_ready = 1'b0;
	assign arm_call_busy = 1'b0;
	assign arm_call_done = 1'b0;
	assign arm_audio_counter0_return = 32'b0;
	assign arm_audio_counter1_return = 32'b0;
	assign arm_audio_counter2_return = 32'b0;
	assign arm_audio_frequency0_return = 32'b0;
	assign arm_audio_frequency1_return = 32'b0;
	assign arm_audio_frequency2_return = 32'b0;
	assign arm_cartram_en = 1'b0;
	assign arm_cartram_write = 1'b0;
	assign arm_cartram_addr = 15'b0;
	assign arm_cartram_wdata = 32'b0;
	assign arm_cartram_wstrb = 4'b0;
	assign ddr_addr = 29'b0;
	assign ddr_din = 64'b0;
	assign ddr_be = 8'b0;
	assign ddr_len = 8'd1;
	assign ddr_req = 1'b0;
	assign ddr_rnw = 1'b1;
	assign halt_req = 1'b0;
	assign mem_ready = 1'b0;
	assign mem_abort = 1'b0;
	assign mem_rdata = 32'b0;
	assign state_req = 1'b0;
	assign state_write = 1'b0;
	assign state_index = 6'b0;
	assign state_wdata = 32'b0;
	assign state_commit = 1'b0;
`endif

	always_ff @(posedge clk) begin
		if (load_start)
			load_end_d <= 1'b0;
		else
			load_end_d <= load_end;
	end

	assign table_family = mapper == BANKBUS ? 2'd1 :
		(mapper == BANKCDF ? 2'd2 : 2'd0);
	assign bus_map_update_selected = bus_map_update && mapper == BANKBUS;
	assign bus_stuff_valid = bus_stuff_valid_raw && mapper == BANKBUS;
	assign bus_stuff_data = bus_stuff_data_raw;
	assign dpc_service_request = dpc_service_request_raw && mapper == BANKDPCP;
	assign fa2_nvram_request = fa2_nvram_request_raw && mapper == BANKFA2;
	assign fa2_nvram_write = fa2_nvram_write_raw;
	assign fa2_nvram_addr = fa2_nvram_addr_raw;
	assign fa2_nvram_wdata = fa2_nvram_wdata_raw;
	assign fa2_nvram_dirty = fa2_nvram_dirty_raw && mapper == BANKFA2;
	assign init_family = mapper == BANKDPCP ? 2'd1 :
		(mapper == BANKBUS ? 2'd2 :
		(mapper == BANKCDF ? 2'd3 : 2'd0));
	assign mapper_call_ready = arm_call_ready && mapper_wb_idle &&
		!mapper_init_busy;
	assign table_pointer_index = mapper == BANKCDF ?
		cdf_table_index : bus_pointer_index;
	assign table_increment_index = mapper == BANKCDF ?
		cdf_table_index : bus_increment_index;
	assign table_pointer_write =
		(mapper == BANKCDF && cdf_pointer_update) ||
		(mapper == BANKBUS && bus_pointer_update);
	assign table_pointer_write_index = mapper == BANKCDF ?
		cdf_pointer_update_index : bus_pointer_update_index;
	assign table_pointer_write_value = mapper == BANKCDF ?
		cdf_pointer_update_value : bus_pointer_update_value;

	arm_mapper_tables stream_tables (
		.clk_sys                (clk),
		.family                 (table_family),
		.revision               (mapper_revision),
		.pointer_lookup_index   (table_pointer_index),
		.increment_lookup_index (table_increment_index),
		.pointer                (table_pointer),
		.increment              (table_increment),
		.pointer_base           (table_pointer_base),
		.increment_base         (table_increment_base),
		.map_base               (table_map_base),
		.stream_count           (table_stream_count),
		.sys_pointer_write      (init_table_pointer_write || table_pointer_write),
		.sys_pointer_index      (init_table_pointer_write ?
			init_table_pointer_index : table_pointer_write_index),
		.sys_pointer_wdata      (init_table_pointer_write ?
			init_table_pointer_wdata : table_pointer_write_value),
		.sys_increment_write    (init_table_increment_write),
		.sys_increment_index    (init_table_increment_index),
		.sys_increment_wdata    (init_table_increment_wdata),
		.sys_map_write          (init_table_map_write || bus_map_update_selected),
		.sys_map_index          (init_table_map_write ?
			init_table_map_index : bus_map_update_index),
		.sys_map_wdata          (init_table_map_write ?
			init_table_map_wdata : bus_map_update_value),
		.clk_arm,
		.arm_write              (arm_cartram_en && arm_cartram_write),
		.arm_accepted           (arm_cartram_accepted),
		.arm_addr               (arm_cartram_addr),
		.arm_wdata              (arm_cartram_wdata),
		.arm_wstrb              (arm_cartram_wstrb)
	);

	arm_mapper_ram_init ram_init (
		.clk_sys               (clk),
		.mapper_reset          (reset),
		.load_start,
		.load_end              (load_end_d),
		.family                (init_family),
		.revision              (mapper_revision),
		.mapper_ram_size,
		.busy                  (mapper_init_busy),
		.dma_request           (mapper_dma_request),
		.dma_fill              (mapper_dma_fill),
		.dma_source            (mapper_dma_source),
		.dma_dest              (mapper_dma_dest),
		.dma_count             (mapper_dma_count),
		.dma_value             (mapper_dma_value),
		.dma_ready             (mapper_dma_ready),
		.dma_done              (mapper_dma_done),
		.ram_en                (init_ram_en),
		.ram_addr              (init_ram_addr),
		.ram_word_rdata        (cartram_word_data),
		.table_pointer_write   (init_table_pointer_write),
		.table_pointer_index   (init_table_pointer_index),
		.table_pointer_wdata   (init_table_pointer_wdata),
		.table_increment_write (init_table_increment_write),
		.table_increment_index (init_table_increment_index),
		.table_increment_wdata (init_table_increment_wdata),
		.table_map_write       (init_table_map_write),
		.table_map_index       (init_table_map_index),
		.table_map_wdata       (init_table_map_wdata)
	);

	arm_mapper_writeback table_writeback (
		.clk_sys               (clk),
		.reset_sys             (reset),
		.pointer_write         (table_pointer_write),
		.pointer_addr          (table_pointer_base +
			{9'b0, table_pointer_write_index}),
		.pointer_wdata         (table_pointer_write_value),
		.map_write             (bus_map_update_selected),
		.map_addr              (table_map_base + {9'b0, bus_map_update_index}),
		.map_wdata             (bus_map_update_value),
		.idle                  (mapper_wb_idle),
		.clk_arm,
		.reset_arm             (reset),
		.ram_en                (mapper_wb_en),
		.ram_write             (mapper_wb_write),
		.ram_addr              (mapper_wb_addr),
		.ram_wdata             (mapper_wb_wdata),
		.ram_wstrb             (mapper_wb_wstrb),
		.ram_accepted          (mapper_wb_accepted)
	);

	arm_mapper_audio mapper_audio (
		.clk                    (clk),
		.reset,
		.family                 (init_family),
		.revision               (mapper_revision[1:0]),
		.rom_size,
		.mapper_ram_size,
		.audio_size_addr        (arm_audio_size_addr),
		.bus_digital_audio,
		.cdf_digital_audio,
		.dpc_waveform0          (dpc_audio_waveform0),
		.dpc_waveform1          (dpc_audio_waveform1),
		.dpc_waveform2          (dpc_audio_waveform2),
		.dpc_note_write         (dpc_audio_note_write),
		.dpc_note_voice         (dpc_audio_note_voice),
		.dpc_note_value         (dpc_audio_note_value),
		.call_launch            (arm_call_request),
		.call_done              (arm_call_done),
		.counter0_return        (arm_audio_counter0_return),
		.counter1_return        (arm_audio_counter1_return),
		.counter2_return        (arm_audio_counter2_return),
		.frequency0_return      (arm_audio_frequency0_return),
		.frequency1_return      (arm_audio_frequency1_return),
		.frequency2_return      (arm_audio_frequency2_return),
		.counter0               (arm_audio_counter0),
		.counter1               (arm_audio_counter1),
		.counter2               (arm_audio_counter2),
		.frequency0             (arm_audio_frequency0),
		.frequency1             (arm_audio_frequency1),
		.frequency2             (arm_audio_frequency2),
		.ram_en                 (audio_ram_en),
		.ram_addr               (audio_ram_addr),
		.ram_grant              (audio_ram_grant),
		.ram_byte_data          (cartram_data),
		.ram_word_data          (cartram_word_data),
		.rom_request            (arm_sample_request),
		.rom_addr               (arm_sample_addr),
		.rom_ready              (arm_sample_ready),
		.rom_done               (arm_sample_done),
		.rom_data               (arm_sample_data),
		.amplitude              (arm_audio_amplitude)
	);

	mapper_dpcplus dpcplus (
		.clk,
		.reset                  (reset || mapper != BANKDPCP),
		.access                 (phi2),
		.rw,
		.a_in,
		.d_in,
		.rom_data               (rom_do),
		.stable_fractional      (mapper_revision[0]),
		.d_out                  (direct_do[BANKDPCP]),
		.flags_out              (flags_out[BANKDPCP]),
		.oe                     (out_en[BANKDPCP]),
		.rom_a                  (rom_addr[BANKDPCP]),
		.ram_sel                (ram_sel[BANKDPCP]),
		.ram_rw                 (ram_rw[BANKDPCP]),
		.ram_a                  (ram_a[BANKDPCP]),
		.ram_data               (cartram_data),
		.amplitude              (arm_audio_amplitude),
		.audio_waveform0        (dpc_audio_waveform0),
		.audio_waveform1        (dpc_audio_waveform1),
		.audio_waveform2        (dpc_audio_waveform2),
		.audio_note_write       (dpc_audio_note_write),
		.audio_note_voice       (dpc_audio_note_voice),
		.audio_note_value       (dpc_audio_note_value),
		.call_request           (dpc_call_request),
		.call_entry             (dpc_call_entry),
		.call_stack             (dpc_call_stack),
		.call_thumb             (dpc_call_thumb),
		.call_ready             (mapper_call_ready),
		.service_request        (dpc_service_request_raw),
		.service_fill           (dpc_service_fill),
		.service_source         (dpc_service_source),
		.service_dest           (dpc_service_dest),
		.service_count          (dpc_service_count),
		.service_value          (dpc_service_value),
		.service_ready          (dpc_service_ready)
	);

	mapper_cdf cdf (
		.clk,
		.reset                   (reset || mapper != BANKCDF),
		.access                 (phi2),
		.rw,
		.a_in,
		.d_in,
		.rom_data               (rom_do),
		.revision               (mapper_revision[1:0]),
		.enable_ldx             (cdf_ldx),
		.enable_ldy             (cdf_ldy),
		.fetch_offset_enable    (cdf_fetch_offset_enable),
		.fetch_offset           (cdf_fetch_offset),
		.fast_jump_valid,
		.d_out                  (direct_do[BANKCDF]),
		.flags_out              (flags_out[BANKCDF]),
		.oe                     (out_en[BANKCDF]),
		.rom_a                  (rom_addr[BANKCDF]),
		.table_index            (cdf_table_index),
		.table_pointer,
		.table_increment        (table_increment[15:0]),
		.pointer_update         (cdf_pointer_update),
		.pointer_update_index   (cdf_pointer_update_index),
		.pointer_update_value   (cdf_pointer_update_value),
		.ram_en                 (cdf_ram_en),
		.ram_write              (cdf_ram_write),
		.ram_addr               (cdf_ram_addr),
		.ram_wdata              (cdf_ram_wdata),
		.ram_rdata              (cartram_data),
		.amplitude              (arm_audio_amplitude),
		.digital_audio          (cdf_digital_audio),
		.call_request           (cdf_call_request),
		.call_entry             (cdf_call_entry),
		.call_stack             (cdf_call_stack),
		.call_thumb             (cdf_call_thumb),
		.call_ready             (mapper_call_ready),
		.cdfj_entry,
		.cdfj_stack
	);

	// CDF and BUS3 both ask the same question of the same map, and only one of
	// them is selected at a time.
	assign fast_jump_query_addr = mapper == BANKBUS ?
		rom_addr[BANKBUS][14:0] : rom_addr[BANKCDF][14:0];

	cdf_fastjump_table jump_table (
		.clk_sys    (clk),
		.load_start,
		.load_addr,
		.load_valid,
		.load_data,
		.query_addr (fast_jump_query_addr),
		.query_valid(fast_jump_valid)
	);

	assign ram_sel[BANKCDF] = cdf_ram_en;
	assign ram_rw[BANKCDF] = !cdf_ram_write;
	assign ram_a[BANKCDF] = {3'b0, cdf_ram_addr};

	mapper_bus bus (
		.clk,
		.reset                   (reset || mapper != BANKBUS),
		.access                 (phi2),
		.rw,
		.a_in,
		.d_in,
		.rom_data               (rom_do),
		.revision               (mapper_revision[1:0]),
		.fast_jump_valid,
		.supported              (),
		.d_out                  (direct_do[BANKBUS]),
		.flags_out              (flags_out[BANKBUS]),
		.oe                     (out_en[BANKBUS]),
		.rom_a                  (rom_addr[BANKBUS]),
		.pointer_lookup_index   (bus_pointer_index),
		.increment_lookup_index (bus_increment_index),
		.table_pointer,
		.table_increment,
		.pointer_update         (bus_pointer_update),
		.pointer_update_index   (bus_pointer_update_index),
		.pointer_update_value   (bus_pointer_update_value),
		.map_update             (bus_map_update),
		.map_update_index       (bus_map_update_index),
		.map_update_value       (bus_map_update_value),
		.ram_en                 (bus_ram_en),
		.ram_write              (bus_ram_write),
		.ram_addr               (bus_ram_addr),
		.ram_wdata              (bus_ram_wdata),
		.ram_rdata              (cartram_data),
		.amplitude              (arm_audio_amplitude),
		.digital_audio          (bus_digital_audio),
		.stuff_valid            (bus_stuff_valid_raw),
		.stuff_data             (bus_stuff_data_raw),
		.call_request           (bus_call_request),
		.call_entry             (bus_call_entry),
		.call_stack             (bus_call_stack),
		.call_thumb             (bus_call_thumb),
		.call_ready             (mapper_call_ready)
	);

	assign ram_sel[BANKBUS] = bus_ram_en;
	assign ram_rw[BANKBUS] = !bus_ram_write;
	assign ram_a[BANKBUS] = {3'b0, bus_ram_addr};

	assign arm_call_request = mapper == BANKDPCP ? dpc_call_request :
		(mapper == BANKCDF ? cdf_call_request :
		(mapper == BANKBUS ? bus_call_request : 1'b0));
	assign arm_call_entry = mapper == BANKDPCP ? dpc_call_entry :
		(mapper == BANKCDF ? cdf_call_entry : bus_call_entry);
	assign arm_call_stack = mapper == BANKDPCP ? dpc_call_stack :
		(mapper == BANKCDF ? cdf_call_stack : bus_call_stack);
	assign arm_call_thumb = mapper == BANKDPCP ? dpc_call_thumb :
		(mapper == BANKCDF ? cdf_call_thumb : bus_call_thumb);

	// ELF is not an ARM7 mapper; retain the explicit unsupported screen.
	assign direct_do[BANKELF]     = bg_data;
	assign flags_out[BANKELF]     = 16'd1;
	assign out_en[BANKELF]        = 8'hFF;
	assign ram_sel[BANKELF]       = 0;
	assign ram_rw[BANKELF]        = 1;
	assign ram_a[BANKELF]         = '0;
	assign rom_addr[BANKELF]      = '0;

	assign audio_ram_grant = audio_ram_en && !init_ram_en && !sel_ram_sel;
	assign cartram_addr = init_ram_en ? {1'b0, init_ram_addr} :
		(sel_ram_sel ? sel_ram_a : {1'b0, audio_ram_addr});
	// One access, one write. The strobe is a level that stands for most of the
	// 6507 cycle, so without this a mapper that steps its RAM address at phi2 -
	// DPC+'s DFxWRITE and DFxPUSH move their counter there, CDF's DSWRITE its
	// pointer - writes the same byte a second time at the next address, leaving
	// a stray byte behind every store.
	assign cartram_wr = !init_ram_en && sel_ram_sel && ~sel_ram_rw &&
		~phi1 && ~address_change && ~access_taken;
	assign cartram_rd = init_ram_en || audio_ram_grant ||
		(sel_ram_sel && sel_ram_rw && ~phi1 && ~address_change);
	assign cartram_wrdata = d_in;
	assign cr_do = cartram_data;

	// Other?
	// SV   -- Spectravideo Compumate (seems useless)
	// 0840 -- Econobanking (can't find any games that use it)
	// MC   -- Megacart (doesn't seem like it works on real hardware, also no games)
	// X07  -- X07 Atariage (seems impossible, also cant find any games with it)
	// 4A50 -- 4A50 (never found a game with this)

	mapper_none mapper_none
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANK00]),
		.flags_out  (flags_out[BANK00]),
		.oe         (out_en[BANK00]),
		.ram_sel    (ram_sel[BANK00]),
		.ram_rw     (ram_rw[BANK00]),
		.ram_a      (ram_a[BANK00]),
		.rom_a      (rom_addr[BANK00])
	);

	mapper_F8 mapper_F8
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKF8]),
		.flags_out  (flags_out[BANKF8]),
		.oe         (out_en[BANKF8]),
		.ram_sel    (ram_sel[BANKF8]),
		.ram_rw     (ram_rw[BANKF8]),
		.ram_a      (ram_a[BANKF8]),
		.rom_a      (rom_addr[BANKF8])
	);

	mapper_F6 mapper_F6
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKF6]),
		.flags_out  (flags_out[BANKF6]),
		.oe         (out_en[BANKF6]),
		.ram_sel    (ram_sel[BANKF6]),
		.ram_rw     (ram_rw[BANKF6]),
		.ram_a      (ram_a[BANKF6]),
		.rom_a      (rom_addr[BANKF6])
	);

	mapper_FE mapper_FE
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKFE]),
		.flags_out  (flags_out[BANKFE]),
		.oe         (out_en[BANKFE]),
		.ram_sel    (ram_sel[BANKFE]),
		.ram_rw     (ram_rw[BANKFE]),
		.ram_a      (ram_a[BANKFE]),
		.rom_a      (rom_addr[BANKFE]),
		.ce         (phi1)
	);

	mapper_E0 mapper_E0
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKE0]),
		.flags_out  (flags_out[BANKE0]),
		.oe         (out_en[BANKE0]),
		.ram_sel    (ram_sel[BANKE0]),
		.ram_rw     (ram_rw[BANKE0]),
		.ram_a      (ram_a[BANKE0]),
		.rom_a      (rom_addr[BANKE0])
	);

	mapper_3F mapper_3F
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANK3F]),
		.flags_out  (flags_out[BANK3F]),
		.oe         (out_en[BANK3F]),
		.ram_sel    (ram_sel[BANK3F]),
		.ram_rw     (ram_rw[BANK3F]),
		.ram_a      (ram_a[BANK3F]),
		.rom_a      (rom_addr[BANK3F])
	);

	mapper_F4 mapper_F4
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKF4]),
		.flags_out  (flags_out[BANKF4]),
		.oe         (out_en[BANKF4]),
		.ram_sel    (ram_sel[BANKF4]),
		.ram_rw     (ram_rw[BANKF4]),
		.ram_a      (ram_a[BANKF4]),
		.rom_a      (rom_addr[BANKF4])
	);

	mapper_P2 mapper_P2
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKP2]),
		.flags_out  (flags_out[BANKP2]),
		.oe         (out_en[BANKP2]),
		.ram_sel    (ram_sel[BANKP2]),
		.ram_rw     (ram_rw[BANKP2]),
		.ram_a      (ram_a[BANKP2]),
		.rom_a      (rom_addr[BANKP2]),
		.ce         (ce)
	);

	mapper_FA mapper_FA
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKFA]),
		.flags_out  (flags_out[BANKFA]),
		.oe         (out_en[BANKFA]),
		.ram_sel    (ram_sel[BANKFA]),
		.ram_rw     (ram_rw[BANKFA]),
		.ram_a      (ram_a[BANKFA]),
		.rom_a      (rom_addr[BANKFA])
	);

	mapper_fa2 mapper_FA2
	(
		.clk           (clk),
		.reset         (reset || mapper != BANKFA2),
		.ce            (ce),
		.phi1          (phi1),
		.a_change      (address_change),
		.a_in          (a_in),
		.d_in          (d_in),
		.rom_data      (rom_do),
		.rom_size      (rom_size),
		.d_out         (direct_do[BANKFA2]),
		.flags_out     (flags_out[BANKFA2]),
		.oe            (out_en[BANKFA2]),
		.rom_a         (rom_addr[BANKFA2]),
		.nvram_request (fa2_nvram_request_raw),
		.nvram_write   (fa2_nvram_write_raw),
		.nvram_addr    (fa2_nvram_addr_raw),
		.nvram_wdata   (fa2_nvram_wdata_raw),
		.nvram_rdata   (fa2_nvram_rdata),
		.nvram_ready   (fa2_nvram_ready),
		.nvram_dirty   (fa2_nvram_dirty_raw)
	);
	assign ram_sel[BANKFA2] = 1'b0;
	assign ram_rw[BANKFA2] = 1'b1;
	assign ram_a[BANKFA2] = 18'd0;

	mapper_CV mapper_CV
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKCV]),
		.flags_out  (flags_out[BANKCV]),
		.oe         (out_en[BANKCV]),
		.ram_sel    (ram_sel[BANKCV]),
		.ram_rw     (ram_rw[BANKCV]),
		.ram_a      (ram_a[BANKCV]),
		.rom_a      (rom_addr[BANKCV])
	);

	mapper_2K mapper_2K
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANK2K]),
		.flags_out  (flags_out[BANK2K]),
		.oe         (out_en[BANK2K]),
		.ram_sel    (ram_sel[BANK2K]),
		.ram_rw     (ram_rw[BANK2K]),
		.ram_a      (ram_a[BANK2K]),
		.rom_a      (rom_addr[BANK2K])
	);

	mapper_UA mapper_UA
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKUA]),
		.flags_out  (flags_out[BANKUA]),
		.oe         (out_en[BANKUA]),
		.ram_sel    (ram_sel[BANKUA]),
		.ram_rw     (ram_rw[BANKUA]),
		.ram_a      (ram_a[BANKUA]),
		.rom_a      (rom_addr[BANKUA])
	);

	mapper_E7 mapper_E7
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKE7]),
		.flags_out  (flags_out[BANKE7]),
		.oe         (out_en[BANKE7]),
		.ram_sel    (ram_sel[BANKE7]),
		.ram_rw     (ram_rw[BANKE7]),
		.ram_a      (ram_a[BANKE7]),
		.rom_a      (rom_addr[BANKE7])
	);

	mapper_F0 mapper_F0
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKF0]),
		.flags_out  (flags_out[BANKF0]),
		.oe         (out_en[BANKF0]),
		.ram_sel    (ram_sel[BANKF0]),
		.ram_rw     (ram_rw[BANKF0]),
		.ram_a      (ram_a[BANKF0]),
		.rom_a      (rom_addr[BANKF0])
	);

	mapper_32 mapper_32
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANK32]),
		.flags_out  (flags_out[BANK32]),
		.oe         (out_en[BANK32]),
		.ram_sel    (ram_sel[BANK32]),
		.ram_rw     (ram_rw[BANK32]),
		.ram_a      (ram_a[BANK32]),
		.rom_a      (rom_addr[BANK32]),
		.cold_reset (mapper != BANK32)
	);

	mapper_AR mapper_AR
	(
		.clk        (clk),
		.reset      (reset || mapper != BANKAR),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKAR]),
		.flags_out  (flags_out[BANKAR]),
		.oe         (out_en[BANKAR]),
		.ram_sel    (ram_sel[BANKAR]),
		.ram_rw     (ram_rw[BANKAR]),
		.ram_a      (ram_a[BANKAR]),
		.rom_a      (rom_addr[BANKAR]),
		.ce         (ce),
		.ar_read    (ar_read),
		.rom_do     (rom_do),
		.rom_size   (rom_size[18:0]),
		.audio_data (tape_audio),
		.tape_in    (tape_in),
		.fix_sc_cs  (fix_sc_cs)
	);

	mapper_WD mapper_WD
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKWD]),
		.flags_out  (flags_out[BANKWD]),
		.oe         (out_en[BANKWD]),
		.ram_sel    (ram_sel[BANKWD]),
		.ram_rw     (ram_rw[BANKWD]),
		.ram_a      (ram_a[BANKWD]),
		.rom_a      (rom_addr[BANKWD])
	);

	mapper_3E mapper_3E
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANK3E]),
		.flags_out  (flags_out[BANK3E]),
		.oe         (out_en[BANK3E]),
		.ram_sel    (ram_sel[BANK3E]),
		.ram_rw     (ram_rw[BANK3E]),
		.ram_a      (ram_a[BANK3E]),
		.rom_a      (rom_addr[BANK3E]),
		.rom_size   (rom_size[18:0])
	);

	mapper_SB mapper_SB
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKSB]),
		.flags_out  (flags_out[BANKSB]),
		.oe         (out_en[BANKSB]),
		.ram_sel    (ram_sel[BANKSB]),
		.ram_rw     (ram_rw[BANKSB]),
		.ram_a      (ram_a[BANKSB]),
		.rom_a      (rom_addr[BANKSB])
	);

	mapper_EF mapper_EF
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKEF]),
		.flags_out  (flags_out[BANKEF]),
		.oe         (out_en[BANKEF]),
		.ram_sel    (ram_sel[BANKEF]),
		.ram_rw     (ram_rw[BANKEF]),
		.ram_a      (ram_a[BANKEF]),
		.rom_a      (rom_addr[BANKEF])
	);

	mapper_JANE mapper_JANE
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKJANE]),
		.flags_out  (flags_out[BANKJANE]),
		.oe         (out_en[BANKJANE]),
		.ram_sel    (ram_sel[BANKJANE]),
		.ram_rw     (ram_rw[BANKJANE]),
		.ram_a      (ram_a[BANKJANE]),
		.rom_a      (rom_addr[BANKJANE])
	);

endmodule
