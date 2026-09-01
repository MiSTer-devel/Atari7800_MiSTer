// SPDX-License-Identifier: MIT
// Copyright (c) 2019-2026 Jamie Blanks

module Atari7800 #(
	// Firmware images for the Souper BupChip profile. Quartus reads the .mif
	// when it builds; $readmemh reads the .hex in simulation. Both come from
	// .agents/firmware/bupchip, installed into rtl/ by its `install` target.
	//
	// Never let a comment line begin with the word s-y-n-t-h-e-s-i-s: Quartus
	// treats that as a directive and warns (10335) about whatever word comes
	// next, so ordinary prose wrapping onto such a line becomes one. The
	// earlier wording here did exactly that, twice per build. Enforced by
	// .agents/tests/lint/comment_directives.py - and spelled out above rather
	// than quoted, so this note cannot trip the thing it describes.
	parameter BUPCHIP_ROM_MIF  = "bupchip.mif",
	parameter BUPCHIP_ROM_INIT = "rtl/bupchip.hex"
) (
`ifdef BUPCHIP_FORCE_CMD
	// Simulation only: see the BUPCHIP_FORCE_CMD block below.
	input  logic        bupchip_force_valid,
	input  logic  [7:0] bupchip_force_data,
`endif
	input  logic        clk_sys,
	input  logic        reset,
	input  logic        pause,
	output logic  [7:0] RED,
	output logic  [7:0] GREEN,
	output logic  [7:0] BLUE,
	output logic        HSync,
	output logic        VSync,
	output logic        HBlank,
	output logic        VBlank,
	output logic        VBlank_orig,
	output logic        ce_pix,
	input  logic        PAL,
	input  logic [1:0]  pal_temp,
	input  logic        hsc_en,
	output logic        hsc_ram_cs,
	input  logic  [7:0] hsc_ram_dout,
	output logic  [7:0] dout,
	output logic        cpu_ce,

	output logic [15:0] AUDIO_R,
	output logic [15:0] AUDIO_L,
	input logic         show_border,
	input logic         show_overscan,
	input logic         bypass_bios,
	input logic         tia_mode,
	input logic         cpu_driver,

	input  logic [7:0]  cart_xm,
	output logic        cart_read,
	input  logic [7:0]  cart_out, bios_out,
	output logic [15:0] AB,
	output logic [24:0] cart_addr_out,
	input  logic [15:0] cart_flags,
	input  logic  [7:0] cart_save,
	input  logic [31:0] cart_size,
	output logic  [7:0] cart_din,
	output logic        RW,
	input  logic        loading,

	output       [17:0] cartram_addr,
	output              cartram_wr,
	output              cartram_rd,
	output       [7:0]  cartram_wrdata,
	input        [7:0]  cartram_data,
	input  logic        clk_arm,
	input  logic        arm_reset,          // power-on reset for the ARM mapper
	input  logic        mapper_load_start,
	input  logic [24:0] mapper_load_addr,
	input  logic        mapper_load_valid,
	input  logic  [7:0] mapper_load_data,
	input  logic        mapper_load_end,
	output logic        mapper_load_wait,
	output logic        mapper_init_busy,
	output logic        fa2_nvram_request,
	output logic        fa2_nvram_write,
	output logic  [7:0] fa2_nvram_addr,
	output logic  [7:0] fa2_nvram_wdata,
	input  logic  [7:0] fa2_nvram_rdata,
	input  logic        fa2_nvram_ready,
	output logic        fa2_nvram_dirty,

	// DDR3. Only the ARM mapper's ROM shadow uses it.
	output logic        ddram_clk,
	output logic [28:0] ddram_addr,
	output logic  [7:0] ddram_burstcnt,
	input  logic        ddram_busy,
	input  logic [63:0] ddram_dout,
	input  logic        ddram_dout_ready,
	output logic        ddram_rd,
	output logic [63:0] ddram_din,
	output logic  [7:0] ddram_be,
	output logic        ddram_we,

	// Tia inputs
	input  logic  [3:0] idump,
	output logic  [3:0] i_out,
	input  logic  [1:0] ilatch,
	input  logic        tia_stab,
	output logic        tia_f1,
	output logic        tia_pal,
	output logic        tia_en,

	// Riot inputs
	input  logic  [7:0] PAin,
	input  logic  [7:0] PBin,
	output logic  [7:0] PAout,
	output logic  [7:0] PBout,
	output logic        PAread,

	// 2600 Cart force flags based on detection
	input logic [4:0]  force_bs,
	input logic [2:0]  mapper_revision,
	input logic        cdf_ldx,
	input logic        cdf_ldy,
	input logic        cdf_fetch_offset_enable,
	input logic [7:0]  cdf_fetch_offset,
	input logic [31:0] cdfj_entry,
	input logic [31:0] cdfj_stack,
	input logic [15:0] arm_audio_size_addr,
	input logic        sc,
	input logic [7:0]  clearval,
	input logic [7:0]  random,
	input logic [1:0]  tape_in,
	input logic        fix_sc_cs,
	output logic       tia_hsync,
	output logic signed [23:0] comp,       // Composite video, selected the same way as the RGB path
	output logic       comp_tog,           // Toggles per composite sample, for the CDC
	output logic       comp_hs,
	output logic       comp_vs,
	output logic       comp_hb,
	output logic       comp_vb,
	output logic [9:0] comp_burst_start,   // Burst window differs between the two chips
	output logic [9:0] comp_burst_len,

	input  logic       use_stereo,
	input [10:0]       ps2_key,
	input              pokey_irq,
	input              minnie_en,
	input              decomb,
	input [4:0]        mapper,
	input              pal_load,
	input [9:0]        pal_addr,
	input              pal_wr,
	input [7:0]        pal_data,
	input              blend,
	output [3:0]       i_read
);

	/////////////
	// Signals //
	/////////////

	logic           NMI_n;
	logic           maria_RDY;
	logic           halt_n;
	logic           maria_drive_AB;
	logic [15:0]    maria_AB_out;
	logic           tia_RDY;
	logic           tia_RDY_seen_high;
	logic [3:0]     audv0, audv1;
	logic [7:0]     tia_db_out;
	logic           RDY;
	logic           IRQ_n;
	logic [15:0]    cpu_AB;
	logic           cpu_halt_n;
	logic           cpu_released;
	logic           maria_en;
	logic           lock_ctrl;
	logic           bios_en_b;
	logic [1:0]     ctrl_writes;
	logic [7:0]     read_DB;
	logic [7:0]     write_DB;
	logic           bus_stuff_valid;
	logic [7:0]     bus_stuff_data;
	wire [7:0]      physical_write_DB = tia_en && bus_stuff_valid ?
		(write_DB & bus_stuff_data) : write_DB;
	logic [7:0]     tia_DB_out, riot_DB_out, maria_DB_out, ram0_DB_out, ram1_DB_out, cart_DB_out;
	logic [15:0]    pokey_audio_r, pokey_audio_l, ym_audio_r, ym_audio_l;
	logic [15:0]    minnie_audio;
	logic           mclk0;
	logic           mclk1;
	logic           cs_ram0, cs_ram1, cs_tia, cs_riot, cs_maria;
	logic [7:0]     open_bus;
	wire            cart_read_flag, ext_audio_cart;
	// Souper's player is an external source too, so it must halve the mix.
	wire            ext_audio = ext_audio_cart || souper_profile;
	logic [24:0]    cart_2600_addr_out, cart_7800_addr_out;
	logic [7:0]     cart_2600_DB_out, cart_7800_DB_out;
	logic           cpu_rwn;
	logic [15:0]    covox_r, covox_l;
	logic [15:0]    last_address;
	logic           pclk1, pclk0, pclk1_m, pclk0_m, pclk1_t, pclk0_t;
	logic           pclk1_raw, pclk0_raw;
	logic           phase_source_tia;
	logic           phase_edge_source_tia;
	logic           phase_source_stable;
	logic           phase_valid;
	logic           reset_hold = 1'b1;
	logic [1:0]     reset_phase_wait = 2'd0;
	logic           tia_clk_x2;
	logic           read_2600;
	logic [1:0]     pause_clock;
	logic [17:0]    cartram_addr78, cartram_addr26;
	logic           cartram_wr78, cartram_wr26;
	logic [7:0]     cartram_wrdata78, cartram_wrdata26;
	logic           cartram_rd78, cartram_rd26;
	logic [7:0]     cartram_data_bram;
	logic           arm_call_stall;

	wire tia_clock_after_reset = cpu_driver && bypass_bios && tia_mode;
	wire effective_reset = reset | reset_hold;
	// SALLY must receive real clock enables to recognize reset. MARIA remains
	// the source throughout the shared held reset; the optional TIA source is a
	// phase-safe handoff after every chip has left reset together.
	wire phase_source_request_tia = cpu_driver && !effective_reset &&
		ctrl_writes == 2'd2 && tia_en;
	wire first_phase_is_phi1 = bypass_bios && tia_mode &&
		!tia_clock_after_reset;

	// Reset assertion is immediate. Release lands on the phase before the first
	// one the CPU acts on, so that first visible edge is still source-caused.
	always_ff @(posedge clk_sys) begin
		if (reset) begin
			reset_hold <= 1'b1;
			reset_phase_wait <= 2'd0;
		end
		else if (reset_hold && phase_source_stable && phase_valid &&
			!phase_source_tia) begin
			if (first_phase_is_phi1) begin
				// On the bypassed 2600/MARIA path release lands midway
				// through the high phase, two master half-cycles in.
				if (reset_phase_wait == 2'd1)
					reset_hold <= 1'b0;
				else if (reset_phase_wait != 2'd0)
					reset_phase_wait <= reset_phase_wait - 1'd1;
				else if (pclk0)
					reset_phase_wait <= 2'd3;
			end else if (pclk1) begin
				reset_hold <= 1'b0;
			end
		end
	end

	// In 2600 mode SHB and the CPU phase can collapse onto one clk_sys tick.
	// Require release to survive a phase-1 sample; assertion stays immediate.
	always_ff @(posedge clk_sys) begin
		if (effective_reset)
			tia_RDY_seen_high <= 1'b1;
		else if (!tia_RDY)
			tia_RDY_seen_high <= 1'b0;
		else if (pclk1)
			tia_RDY_seen_high <= 1'b1;
	end
	// On a Harmony/Melody the ARM is the cartridge, so while a mapper call
	// runs the 6507 makes no forward progress; Stella models the same thing
	// as zero elapsed time. rubyQ writes its parameter block, calls the ARM
	// and reads the results back about ten cycles later, so without this the
	// ARM is still reading parameters the 6507 has already overwritten and
	// the 6507 is reading last frame's results. The CPU waits on RDY, which
	// only holds read cycles, and the call is always requested by a write, so
	// the cycle that stalls is the following instruction fetch.
	assign arm_call_stall = tia_en &&
		(arm_call_busy || (!mapper_init_busy && arm_dma_busy));

	// The stall holds the 6507 on one read cycle, and RDY releases inside that
	// cycle, so the CPU consumes it without ever presenting it again. Hiding
	// every phi2 of the stall therefore hides a cycle the CPU really made: CDF
	// misses the `LDA #` opcode fetch that follows the CALLFN write, never arms
	// its fast fetcher, and hands the 6507 the stream index from ROM instead of
	// the stream's byte. Show the mapper the first phi2 of the stall - the same
	// address the CPU ends up completing - and hide the repeats.
	//
	//   phi2   ...  write $1FF3 | fetch | fetch | ... | fetch | operand
	//   stall            0      |   1   |   1   |     |   1   |    0
	//   mapper           take   | take  |  hide |     |  hide |  take
	logic stall_cycle_taken;
	always_ff @(posedge clk_sys) begin
		if (!arm_call_stall)
			stall_cycle_taken <= 1'b0;
		else if (pclk0)
			stall_cycle_taken <= 1'b1;
	end
	wire mapper_phi2 = pclk0 && (!arm_call_stall || !stall_cycle_taken);
	assign RDY = maria_RDY && tia_RDY && (~tia_en || tia_RDY_seen_high) &&
		!arm_call_stall;
	assign cpu_halt_n = (ctrl_writes == 2'd2) ? halt_n : 1'b1;
	assign cart_read = tia_en ? (pause ? ~|pause_clock : read_2600) : ((pause ? pause_clock[0] : (cart_read_flag & mclk1)));
	assign cart_addr_out = tia_en ? cart_2600_addr_out : cart_7800_addr_out;
	assign cart_DB_out = tia_en ? cart_2600_DB_out : cart_7800_DB_out;
	assign cpu_ce = pclk1;
	assign VBlank_orig = maria_en ? maria_vblank : tia_vblank;

	// Track the open bus since FPGA's don't use bidirectional logic internally
	always_ff @(posedge clk_sys) begin
		pause_clock <= pause ? pause_clock + 1'd1 : {1'b0, mclk1};
		open_bus <= (~RW ? physical_write_DB : read_DB);
		last_address <= AB;
	end

	wire cs_cart = ~|{cs_ram0, cs_ram1, cs_tia, cs_riot, cs_maria};

	always_comb begin
		read_DB = open_bus;
		if (cs_ram0)  read_DB = ram0_DB_out;
		if (cs_ram1)  read_DB = ram1_DB_out;
		if (cs_tia)   read_DB = {tia_DB_out[7:6], open_bus[5:0]};
		if (cs_riot)  read_DB = riot_DB_out;
		if (cs_maria) read_DB = maria_DB_out;
		// Last, so the cartridge - the slowest source by far - reaches the bus
		// through the fewest mux levels. cs_cart is the complement of the other
		// selects, so the two can never both be true and the order is free.
		if (cs_cart)  read_DB = (~bios_en_b && AB[15]) ? bios_out : cart_DB_out;

		case ({~cpu_released, maria_drive_AB})
			2'b00 : AB = last_address;
			2'b01 : AB = maria_AB_out;
			2'b10 : AB = cpu_AB;
			2'b11 : AB = cpu_AB & maria_AB_out;
		endcase
		RW = cpu_released ? 1'b1 : cpu_rwn;

	end

	cpu_phase_controller phase_controller
	(
		.clk_sys        (clk_sys),
		.request_tia    (phase_source_request_tia),
		.maria_phi1     (pclk1_m),
		.maria_phi2     (pclk0_m),
		.tia_phi1       (pclk1_t),
		.tia_phi2       (pclk0_t),
		.phi1           (pclk1_raw),
		.phi2           (pclk0_raw),
		.active_tia     (phase_source_tia),
		.edge_source_tia(phase_edge_source_tia),
		.source_stable  (phase_source_stable),
		.phase_valid    (phase_valid)
	);

	assign dout = write_DB;

	// Memory
	logic [10:0] clear_addr;
	always_ff @(posedge clk_sys) clear_addr <= clear_addr + loading;

	spram #(.addr_width(11), .mem_name("RAM0")) ram0
	(
		.clock          (clk_sys),
		.address        (loading ? clear_addr : AB[10:0]),
		.data           (loading ? clearval : write_DB),
		.wren           ((~RW & cs_ram0 & pclk0) || loading),
		.q              (ram0_DB_out),
		.cs             (~pause)
	);

	spram #(.addr_width(11), .mem_name("RAM1")) ram1
	(
		.clock          (clk_sys),
		.address        (loading ? clear_addr : AB[10:0]),
		.data           (loading ? clearval : write_DB),
		.wren           ((~RW & cs_ram1 & pclk0) || loading),
		.q              (ram1_DB_out),
		.cs             (~pause)
	);

	// MARIA
	logic maria_vblank, maria_vblank_ex, maria_vsync, maria_hblank, maria_hsync;

	logic [3:0] maria_luma, maria_chroma;

	maria maria_inst(
		.ce             (~pause || effective_reset),
		.mclk0          (mclk0),
		.mclk1          (mclk1),
		.tia_clk_x2     (tia_clk_x2),
		.AB_in          (AB),
		.AB_out         (maria_AB_out),
		.drive_AB       (maria_drive_AB),
		.hide_border    (~show_border),
		.bypass_bios    (bypass_bios),
		.PAL            (PAL),
		.d_in           (read_DB),
		.write_DB_in    (write_DB),
		.DB_out         (maria_DB_out),
		.reset          (effective_reset),
		.clk_sys        (clk_sys),
		.pclk0          (pclk0_m),
		.pclk1          (pclk1_m),
		.pclk2          (pclk0),
		.RW             (RW),
		.maria_en       (maria_en),
		.YC             ({maria_chroma, maria_luma}),
		.cs_ram0        (cs_ram0),
		.cs_ram1        (cs_ram1),
		.cs_riot        (cs_riot),
		.cs_tia         (cs_tia),
		.cs_maria       (cs_maria),
		.NMI_n          (NMI_n),
		.halt_n         (halt_n),
		.ready          (maria_RDY),
		.vsync          (maria_vsync),
		.vblank         (maria_vblank),
		.vblank_ex      (maria_vblank_ex),
		.hsync          (maria_hsync),
		.hblank         (maria_hblank)
	);

	logic tia_vblank, tia_vsync, tia_hblank, tia_blank_n;
	logic [3:0] tia_chroma;
	logic [2:0] tia_luma;
	logic tia_pix_ce;
	logic cart_ce_2600;

	TIA tia_inst
	(
		.clk            (clk_sys),
		.ce             (tia_clk_x2),     // Clock enable for CLK generation only
		.is_7800        (~(phase_source_tia || phase_edge_source_tia)),
		// The pins echo MARIA's pair in 7800 mode; the controller has to read
		// the divider instead or the phase network is a combinational ring.
		.phi0           (),
		.phi1           (),
		.phi0_gen       (pclk0_t),
		.phi1_gen       (pclk1_t),
		.phi2           (pclk0),
		.RW_n           (RW),
		.rdy            (tia_RDY),
		.addr           ({(AB[5] & tia_en), AB[4:0]}),
		.d_in           (physical_write_DB),
		.d_out          (tia_DB_out),
		.i              (idump),     // On real hardware, these would be ADC pins. i0..3
		.i_out          (i_out),
		.i4             (ilatch[0]),
		.i5             (ilatch[1]),
		.aud0           (audv0),
		.aud1           (audv1),
		.col            (tia_chroma),
		.lum            (tia_luma),
		.BLK_n          (tia_blank_n),
		.sync           (),
		.cs0_n          (~cs_tia),
		.cs2_n          (~cs_tia),
		.rst            (effective_reset),
		.video_ce       (tia_pix_ce),
		.vblank         (tia_vblank),
		.hblank         (),
		.hgap           (tia_hblank),
		.vsync          (tia_vsync),
		.hsync          (tia_hsync),
		.row            (),
		.column         (),
		.phi1_in        (pclk1),
		.open_bus       (open_bus),
		.cart_ce        (cart_ce_2600),
		.decomb         (decomb),
		.is_pal         (tia_pal),
		.stabilize      (tia_stab),
		.is_f1          (tia_f1),
		.paddle_read    (i_read)
	);

	// One output stage for both chips, which is what the board has. MARIA owns it:
	// tia-maria.pdf page 47 shows its COLOR DELAY LINE & LUM sheet taking TC0-3 and
	// TL0-3F beside its own MC0-3 / ML0-3F, gated by MENBLF, driving the single
	// COLOR pin. On a 7800 the 2600's colour never reaches a pin of its own, so the
	// codes are what get selected, not two finished composite signals.
	wire        c_maria = maria_en;
	wire [3:0]  c_col   = c_maria ? maria_chroma : tia_chroma;
	wire [3:0]  c_lum   = c_maria ? maria_luma   : {tia_luma, 1'b0};
	wire        c_hs    = c_maria ? maria_hsync  : tia_hsync;
	wire        c_vs    = c_maria ? maria_vsync  : tia_vsync;
	wire        c_hb    = c_maria ? maria_hblank : tia_hblank;
	wire        c_vb    = c_maria ? maria_vblank : tia_vblank;

	composite_out comp_out
	(
		.clk          (clk_sys),
		// MARIA's burst sits at columns 38-56 and its pixel is 2*fsc; the 2600's is
		// at colour clocks 36-51 and its pixel is a whole cycle.
		.burst_on     (c_maria ? 10'd8  : 10'd16),
		.burst_off    (c_maria ? 10'd44 : 10'd80),
		.chroma_scale (c_maria ? 9'd256 : 9'd236),
		.pix_samples  (c_maria ? 2'd2   : 2'd0),
		.phase_adj    (2'd1),
		.col          (c_col),
		.lum          (c_lum),
		.blank        (c_hb || c_vb),
		.hsync        (c_hs),
		.pix_tick     (c_maria ? mclk0 : tia_pix_ce),
		.hblank       (c_hb),
		.vblank       (c_vb),
		.vsync        (c_vs),
		.temp         (pal_temp),
		.comp         (comp),
		.sample_tog   (comp_tog),
		.hs_out       (comp_hs),
		.vs_out       (comp_vs),
		.hb_out       (comp_hb),
		.vb_out       (comp_vb)
	);

	assign comp_burst_start = c_maria ? 10'd12 : 10'd24;
	assign comp_burst_len   = c_maria ? 10'd36 : 10'd56;

	video_mux mux
	(
		.clk_sys        (clk_sys),
		.maria_luma     (maria_luma),
		.maria_chroma   (maria_chroma),
		.maria_hblank   (maria_hblank),
		.maria_vblank   (show_overscan ? maria_vblank : maria_vblank_ex),
		.maria_hsync    (maria_hsync),
		.maria_vsync    (maria_vsync),
		.maria_pix_ce   (mclk1),
		.tia_luma       (tia_luma),
		.tia_chroma     (tia_chroma),
		.tia_hblank     (tia_hblank),
		.tia_vblank     (tia_vblank),
		.tia_hsync      (tia_hsync),
		.tia_vsync      (tia_vsync),
		.tia_pix_ce     (tia_pix_ce),
		.pause          (pause),
		.is_maria       (maria_en),
		.pal_temp       (pal_temp),
		.pal_load       (pal_load),
		.pal_data       (pal_data),
		.pal_addr       (pal_addr),
		.pal_wr         (pal_wr),
		.is_PAL         (PAL),
		.hblank         (HBlank),
		.vblank         (VBlank),
		.hsync          (HSync),
		.vsync          (VSync),
		.red            (RED),
		.green          (GREEN),
		.blue           (BLUE),
		.pix_ce         (ce_pix),
		.blend          (blend)
	);

	// Audio output is non-linear, and this table represents the proper compressed values of
	// audv0 + audv1.
	// Generated based on the info here:
	// https://atariage.com/forums/topic/271920-tia-sound-abnormalities/
	logic [15:0] audio_lut[32];
	assign audio_lut = '{
		16'h0000, 16'h0842, 16'h0FFF, 16'h1745, 16'h1E1D, 16'h2492, 16'h2AAA, 16'h306E,
		16'h35E4, 16'h3B13, 16'h3FFF, 16'h44AE, 16'h4924, 16'h4D64, 16'h5173, 16'h5554,
		16'h590A, 16'h5C97, 16'h5FFF, 16'h6343, 16'h6665, 16'h6968, 16'h6C4D, 16'h6F17,
		16'h71C6, 16'h745C, 16'h76DA, 16'h7942, 16'h7B95, 16'h7DD3, 16'h7FFF, 16'hFFFF
	};

	logic [15:0] audio_lut_single[16];
	assign audio_lut_single = '{
		16'h0000, 16'h0C63, 16'h17FF, 16'h22E8, 16'h2D2C, 16'h36DB, 16'h3FFF, 16'h48A5,
		16'h50D6, 16'h589C, 16'h5FFF, 16'h6705, 16'h6DB6, 16'h7416, 16'h7A2D, 16'h7FFF
	};

	logic tape_audio;

	// The mixer sums unsigned contributions around a midpoint, so the BupChip's
	// signed PCM is offset into that convention. Halved along with every other
	// external source when ext_audio is set, which is what keeps the sum from
	// clipping when several sources sound at once.
	// Signed two's complement into the mixer's unsigned-around-midpoint
	// convention: inverting the sign bit is exactly the +$4000... offset, and
	// costs one inverter instead of an adder.
	wire [15:0] bupchip_mix_l = souper_profile ?
		{~bupchip_audio_l[15], bupchip_audio_l[14:0]} : 16'd0;
	wire [15:0] bupchip_mix_r = souper_profile ?
		{~bupchip_audio_r[15], bupchip_audio_r[14:0]} : 16'd0;

	wire [4:0] aud_index = audv0 + audv1;
	wire [15:0] tia_r = (use_stereo ? audio_lut_single[audv0] : audio_lut[aud_index]);
	wire [15:0] tia_l = (use_stereo ? audio_lut_single[audv1] : audio_lut[aud_index]);

	// There is an assumption made here that all audio sources will not be used simultaneously at
	// max volume. If this is the case, there will be clipping. Tia audio cannot be greater than
	// 0x7FFF on a single channel. When external audio sources are used, the overall volume will be
	// halved to ensure no clipping. If in the future more than two external audio devices are used
	// at once, eg covox + ym2151 + tia, then more reduction will be needed, but for the time being
	// that seems unlikely.
	wire [16:0] audio_mix_r = tia_r + pokey_audio_r + ym_audio_r + covox_r + minnie_audio + bupchip_mix_r + {tape_audio, 12'd0};
	wire [16:0] audio_mix_l = tia_l + pokey_audio_l + ym_audio_l + covox_l + minnie_audio + bupchip_mix_l + {tape_audio, 12'd0};

	assign AUDIO_R = ext_audio ? audio_mix_r[16:1] : audio_mix_r[15:0];
	assign AUDIO_L = ext_audio ? audio_mix_l[16:1] : audio_mix_l[15:0];

	logic [7:0] ar_ram_addr;
	M6532 riot_inst
	(
		.clk          (clk_sys),
		.ce           (pclk0),     // PHI 2 Clock enable
		.res_n        (~effective_reset),
		// The chip's RES leaves RIOT RAM alone, so the console owns the power
		// up image. A 7800 gets the loader the BIOS would have left behind; a
		// 2600 never had one, so it starts from zero.
		.ram_init     (effective_reset),
		.ram_init_7800(~tia_mode),
		.addr         (AB[6:0]),
		.RW_n         (RW),
		// Bus stuffing pulls the data lines low for any write off the cartridge,
		// zero page included: BUS Draconian builds a JMP vector in RIOT RAM out
		// of stuffed bytes, so the RIOT has to see the same bus the TIA does.
		.d_in         (physical_write_DB),
		.d_out        (riot_DB_out),
		.RS_n         (AB[9]),
		.IRQ_n        (),
		.IRQ_n_oe     (),
		.CS1          (AB[7]),
		.CS2_n        (~cs_riot),
		.PA_in        (PAin),
		.PA_out       (PAout),
		.PB_in        (PBin),
		.PB_out       (PBout),
		.oe           (),
		// The chip decodes its own ORA read; the trackball steps on it.
		.PA_read      (PAread)
	);

	M6502C cpu_inst
	(
		.pclk1        (pclk1_raw),
		.pclk0        (pclk0_raw),
		.phi1_ce      (pclk1),
		.phi2_ce      (pclk0),
		.clk_sys      (clk_sys),
		.reset        (effective_reset),
		.AB           (cpu_AB),
		.DB_IN        (read_DB),
		.DB_OUT       (write_DB),
		.RD           (cpu_rwn),
		.IRQ_n        (IRQ_n),
		.NMI_n        (NMI_n),
		.RDY          (RDY),
		.halt_n       (cpu_halt_n),
		.is_halted    (cpu_released)
	);


	ctrl_reg ctrl
	(
		.clk          (clk_sys),
		.pclk0        (pclk0),
		.d_in         (write_DB[3:0]),
		.cs           (cs_tia),
		.latch_b      (RW | lock_ctrl),
		.rst          (effective_reset),
		.lock_out     (lock_ctrl),
		.bypass_bios  (bypass_bios),
		.maria_en_out (maria_en),
		.bios_en_out  (bios_en_b),
		.tia_en_out   (tia_en),
		.writes       (ctrl_writes),
		.tia_mode     (tia_mode)
	);

	assign cartram_wr = mapper_init_busy ? cartram_wr26 :
		(tia_en ? cartram_wr26 : (cartram_wr78 & mclk1));
	assign cartram_rd = mapper_init_busy ? cartram_rd26 :
		(tia_en ? cartram_rd26 : (cartram_rd78 & mclk1));
	assign cartram_addr = mapper_init_busy ? cartram_addr26 :
		(tia_en ? cartram_addr26 : cartram_addr78);
	assign cartram_wrdata = mapper_init_busy ? cartram_wrdata26 :
		(tia_en ? cartram_wrdata26 : cartram_wrdata78);

	//////////////////////
	// ARM mapper (2600) //
	//////////////////////
	// One ARM7TDMI serves the DPC+, BUS and CDF front ends in cart2600. The
	// 2600 profile - the call controller and the mapper's memory system - lives
	// there with them, sharing the cart RAM through cart_ram_tdp's second port.
	// Only this bus, the DDR3 channel, the load stream and the FA2 NVRAM file
	// cross back out to the framework.

	// CDFJ+ scales its RAM with the ROM; everything else takes the 8K default.
	// Kept here because it reads force_bs, which is the core's own selection
	// rather than the mapper cart2600 ends up running.
	logic [15:0] mapper_ram_size;
	always_comb begin
		mapper_ram_size = 16'd8192;
		if (force_bs == BANKCDF && mapper_revision == 3'd3) begin
			if (cart_size <= 32'd32768)
				mapper_ram_size = 16'd8192;
			else if (cart_size <= 32'd131072)
				mapper_ram_size = 16'd16384;
			else
				mapper_ram_size = 16'd32768;
		end
	end

	logic        arm_ddr_req, arm_ddr_rnw, arm_ddr_ack, arm_ddr_rvalid;
	logic [28:0] arm_ddr_addr;
	logic [63:0] arm_ddr_din, arm_ddr_dout;
	logic  [7:0] arm_ddr_be, arm_ddr_len;

	logic        arm_halt_req, arm_halted;
	logic        arm_mem_req, arm_mem_ready, arm_mem_abort, arm_mem_write;
	logic        arm_mem_fetch;
	logic [31:0] arm_mem_addr, arm_mem_wdata, arm_mem_rdata;
	logic  [1:0] arm_mem_size;
	logic  [3:0] arm_mem_wstrb;
	logic        arm_state_req, arm_state_write, arm_state_ready;
	logic        arm_state_commit;
	logic  [5:0] arm_state_index;
	logic [31:0] arm_state_wdata, arm_state_rdata;

	logic        arm_call_busy, arm_dma_busy;
	logic        arm_ram_en, arm_ram_write;
	logic [14:0] arm_ram_addr;
	logic [31:0] arm_ram_wdata, arm_ram_rdata;
	logic  [3:0] arm_ram_wstrb;
	logic        arm_ram_accepted;

`ifndef NO_ARM_MAPPER
	arm_host arm_host (
		.clk_arm,
		.reset_arm       (arm_reset || (souper_profile && bup_hold)),
		.halt_req        (souper_profile ? 1'b0 : arm_halt_req),
		.halted          (arm_halted),
		.mem_req         (arm_mem_req),
		.mem_ready       (arm_mem_ready),
		.mem_abort       (arm_mem_abort),
		.mem_addr        (arm_mem_addr),
		.mem_write       (arm_mem_write),
		.mem_wdata       (arm_mem_wdata),
		.mem_rdata       (arm_mem_rdata),
		.mem_size        (arm_mem_size),
		.mem_wstrb       (arm_mem_wstrb),
		.mem_seq         (),
		.mem_fetch       (arm_mem_fetch),
		.mem_privileged  (),
		.mem_lock        (),
		.retire          (),
		.state_req       (arm_state_req),
		.state_write     (arm_state_write),
		.state_index     (arm_state_index),
		.state_wdata     (arm_state_wdata),
		.state_rdata     (arm_state_rdata),
		.state_ready     (arm_state_ready),
		.state_commit    (arm_state_commit)
	);
`else
	// Simulation builds that leave the ARM sources out.
	assign arm_halted = 1'b1;
	assign arm_mem_req = 1'b0;
	assign arm_mem_addr = 32'b0;
	assign arm_mem_write = 1'b0;
	assign arm_mem_wdata = 32'b0;
	assign arm_mem_size = 2'b0;
	assign arm_mem_wstrb = 4'b0;
	assign arm_mem_fetch = 1'b0;
	assign arm_state_rdata = 32'b0;
	assign arm_state_ready = 1'b0;
`endif

	// One DDR3 port, one bridge. Both consumers live inside the core, so the
	// arbitration belongs here rather than in the MiSTer wrapper. Channel 2 is
	// idle until the BupChip profile claims it.
	ddram ddr_bridge (
		.clk              (clk_arm),
		.reset            (arm_reset),
		.DDRAM_CLK        (ddram_clk),
		.DDRAM_BUSY       (ddram_busy),
		.DDRAM_BURSTCNT   (ddram_burstcnt),
		.DDRAM_ADDR       (ddram_addr),
		.DDRAM_DOUT       (ddram_dout),
		.DDRAM_DOUT_READY (ddram_dout_ready),
		.DDRAM_RD         (ddram_rd),
		.DDRAM_DIN        (ddram_din),
		.DDRAM_BE         (ddram_be),
		.DDRAM_WE         (ddram_we),
		.ch1_addr         (arm_ddr_addr),
		.ch1_din          (arm_ddr_din),
		.ch1_be           (arm_ddr_be),
		.ch1_len          (arm_ddr_len),
		.ch1_req          (arm_ddr_req),
		.ch1_rnw          (arm_ddr_rnw),
		.ch1_ack          (arm_ddr_ack),
		.ch1_dout         (arm_ddr_dout),
		.ch1_rvalid       (arm_ddr_rvalid),
		.ch2_addr         (bup_ddr_addr),
		.ch2_din          (bup_ddr_din),
		.ch2_be           (bup_ddr_be),
		.ch2_len          (bup_ddr_len),
		.ch2_req          (bup_ddr_req),
		.ch2_rnw          (bup_ddr_rnw),
		.ch2_ack          (bup_ddr_ack),
		.ch2_dout         (bup_ddr_dout),
		.ch2_rvalid       (bup_ddr_rvalid),
		// ch1's consumer does not re-issue yet - that is
		// Atari7800_MiSTer-4ux. The bridge still releases the port, so a stall
		// on the ARM mapper no longer takes the BupChip down with it.
		.ch1_timeout      (),
		.ch2_timeout      (bup_ddr_timeout)
	);

`ifndef EXTERNAL_CARTRAM
	logic [7:0] cartram_data_tdp;
	logic [31:0] cartram_word_data_tdp;
	// The 7800 path shares this RAM, so it stays in the core. cart2600 already
	// arbitrated its writeback against the ARM, and hands over one port.
	cart_ram_tdp cart_ram
	(
		.clk_sys,
		.mapper_en    (mapper_init_busy ? (cartram_wr || cartram_rd) : !pause),
		.mapper_write (cartram_wr),
		.mapper_addr  (cartram_addr[16:0]),
		.mapper_wdata (cartram_wrdata),
		.mapper_rdata (cartram_data_tdp),
		.clk_arm,
		.arm_en       (arm_ram_en),
		.arm_write    (arm_ram_write),
		.arm_addr     (arm_ram_addr),
		.arm_wdata    (arm_ram_wdata),
		.arm_wstrb    (arm_ram_wstrb),
		.arm_rdata    (arm_ram_rdata),
		.arm_accepted (arm_ram_accepted),
		.mapper_word_rdata(cartram_word_data_tdp)
	);
	assign cartram_data_bram = pause ? 8'hFF : cartram_data_tdp;
`else
	assign cartram_data_bram = cartram_data;
	assign arm_ram_rdata = 32'b0;
	assign arm_ram_accepted = 1'b0;
	assign cartram_word_data_tdp = 32'b0;
`endif

	// ---- ARM profile mux ----------------------------------------------------
	// Decision 0040: one CPU, two exclusive clients. A Souper cartridge takes
	// the bus for the BupChip player; everything else leaves it with the 2600
	// call mappers exactly as before. The two never drive it together.
	logic        arm2600_mem_req, arm2600_mem_ready, arm2600_mem_abort;
	logic [31:0] arm2600_mem_rdata;
	logic        bup_mem_ready, bup_mem_abort;
	logic [31:0] bup_mem_rdata;
	logic        bup_hold, bup_load_wait;
	logic        cart2600_load_wait;
	// Either consumer may stall the download; the loader honours the union.
	assign mapper_load_wait = cart2600_load_wait || bup_load_wait;
	logic        bup_cmd_valid;
	logic  [7:0] bup_cmd_data;
`ifdef BUPCHIP_FORCE_CMD
	// Simulation only: a harness-injected command, ORed in beside the
	// cartridge's own writes so both paths stay live.
	wire       bup_cmd_valid_eff = bup_cmd_valid || bupchip_force_valid;
	wire [7:0] bup_cmd_data_eff  = bup_cmd_valid ? bup_cmd_data : bupchip_force_data;
`else
	wire       bup_cmd_valid_eff = bup_cmd_valid;
	wire [7:0] bup_cmd_data_eff  = bup_cmd_data;
`endif
	logic [15:0] bupchip_audio_l, bupchip_audio_r;
	logic [28:0] bup_ddr_addr;
	logic [63:0] bup_ddr_din, bup_ddr_dout;
	logic  [7:0] bup_ddr_be, bup_ddr_len;
	logic        bup_ddr_req, bup_ddr_rnw, bup_ddr_ack, bup_ddr_rvalid;
	logic        bup_ddr_timeout;

	// Bit 12 is the Souper mapper. tia_mode means a 2600 image, which has no
	// A78 header and therefore no Souper flag to trust.
	wire souper_profile = cart_flags[12] && !tia_mode;

	assign arm2600_mem_req = souper_profile ? 1'b0 : arm_mem_req;
	assign arm_mem_ready = souper_profile ? bup_mem_ready : arm2600_mem_ready;
	assign arm_mem_abort = souper_profile ? bup_mem_abort : arm2600_mem_abort;
	assign arm_mem_rdata = souper_profile ? bup_mem_rdata : arm2600_mem_rdata;

	bupchip_subsystem #(.ROM_MIF(BUPCHIP_ROM_MIF), .ROM_INIT(BUPCHIP_ROM_INIT)) bupchip (
		.clk_sys, .clk_arm,
		.reset_arm      (arm_reset),
		.enabled        (souper_profile),
		.load_start     (mapper_load_start),
		.load_addr      (mapper_load_addr),
		.load_valid     (mapper_load_valid),
		.load_data      (mapper_load_data),
		.load_end       (mapper_load_end),
		.asset_start    (),
		.cmd_valid_sys  (bup_cmd_valid_eff),
		.cmd_data_sys   (bup_cmd_data_eff),
		.mem_req        (souper_profile ? arm_mem_req : 1'b0),
		.mem_addr       (arm_mem_addr),
		.mem_write      (arm_mem_write),
		.mem_wdata      (arm_mem_wdata),
		.mem_wstrb      (arm_mem_wstrb),
		.mem_size       (arm_mem_size),
		.mem_ready      (bup_mem_ready),
		.mem_abort      (bup_mem_abort),
		.mem_rdata      (bup_mem_rdata),
		.arm_hold       (bup_hold),
		.load_wait      (bup_load_wait),
		.ddr_addr       (bup_ddr_addr),
		.ddr_din        (bup_ddr_din),
		.ddr_be         (bup_ddr_be),
		.ddr_len        (bup_ddr_len),
		.ddr_req        (bup_ddr_req),
		.ddr_rnw        (bup_ddr_rnw),
		.ddr_ack        (bup_ddr_ack),
		.ddr_dout       (bup_ddr_dout),
		.ddr_rvalid     (bup_ddr_rvalid),
		.ddr_timeout    (bup_ddr_timeout),
		.audio_l        (bupchip_audio_l),
		.audio_r        (bupchip_audio_r)
	);

	cart cart
	(
		.aud_cmd_valid  (bup_cmd_valid),
		.aud_cmd_data   (bup_cmd_data),
		.clk_sys        (clk_sys),
		.pclk0          (pclk0),
		.pclk1          (pclk1),
		.IRQ_n          (IRQ_n),
		.halt_n         (cpu_halt_n),
		.reset          (effective_reset),
		.address_in     (AB[15:0]),
		.din            (write_DB),
		.rom_din        (cart_out),
		.cart_flags     (cart_flags),
		.cart_size      (cart_size),
		.cart_save      (cart_save),
		.cart_cs        (cs_cart),
		.cart_xm        (cart_xm),
		.cart_read      (cart_read_flag),
		.cartram_addr   (cartram_addr78),
		.cartram_wr     (cartram_wr78),
		.cartram_rd     (cartram_rd78),
		.cartram_wrdata (cartram_wrdata78),
		.cartram_data   (cartram_data_bram),
		.hsc_en         (hsc_en),
		.hsc_ram_cs     (hsc_ram_cs),
		.hsc_ram_din    (hsc_ram_dout),
		.rw             (RW),
		.dout           (cart_7800_DB_out),
		.pokey_audio_r  (pokey_audio_r),
		.pokey_audio_l  (pokey_audio_l),
		.minnie_audio   (minnie_audio),
		.ym_audio_r     (ym_audio_r),
		.ym_audio_l     (ym_audio_l),
		.rom_address    (cart_7800_addr_out),
		.open_bus       (open_bus),
		.covox_r        (covox_r),
		.covox_l        (covox_l),
		.external_audio (ext_audio_cart),
		.ps2_key        (ps2_key),
		.pokey_irq_en   (pokey_irq),
		.minnie_en      (minnie_en)
	);

	assign cart_2600_addr_out[24:19] = '0;
	assign cart_din = cpu_rwn ? read_DB : write_DB;

	cart2600 cart2600
	(
		.d_out          (cart_2600_DB_out),
		.d_in           (cart_din),
		.a_in           (AB[12:0]),
		.rw             (RW),
		.reset          (effective_reset),
		.clk            (clk_sys),
		.ce             (cart_ce_2600),
		.phi1           (pclk1),
		// Held with the CPU: the stalled cycle is one held read, seen once.
		.phi2           (mapper_phi2),
		.sc             (sc),
		.mapper         (|mapper ? mapper : force_bs),
		.mapper_revision(mapper_revision),
		.cdf_ldx,
		.cdf_ldy,
		.cdf_fetch_offset_enable,
		.cdf_fetch_offset,
		.cdfj_entry,
		.cdfj_stack,
		.arm_audio_size_addr,
		.rom_do         (cart_out),
		.rom_size       (cart_size),
		.rom_a          (cart_2600_addr_out[18:0]),
		.rom_read       (read_2600),
		.cartram_addr   (cartram_addr26),
		.cartram_wr     (cartram_wr26),
		.cartram_rd     (cartram_rd26),
		.cartram_wrdata (cartram_wrdata26),
		.cartram_data   (cartram_data_bram),
		.clk_arm,
		.reset_arm      (arm_reset),
		.cartram_word_data(cartram_word_data_tdp),
		.load_start     (mapper_load_start),
		.load_addr      (mapper_load_addr),
		.load_valid     (mapper_load_valid),
		.load_data      (mapper_load_data),
		.load_end       (mapper_load_end),
		.load_wait      (cart2600_load_wait),
		.mapper_ram_size,
		.mapper_init_busy,
		.arm_ram_en,
		.arm_ram_write,
		.arm_ram_addr,
		.arm_ram_wdata,
		.arm_ram_wstrb,
		.arm_ram_accepted,
		.arm_ram_rdata,
		.ddr_addr       (arm_ddr_addr),
		.ddr_din        (arm_ddr_din),
		.ddr_be         (arm_ddr_be),
		.ddr_len        (arm_ddr_len),
		.ddr_req        (arm_ddr_req),
		.ddr_rnw        (arm_ddr_rnw),
		.ddr_ack        (arm_ddr_ack),
		.ddr_dout       (arm_ddr_dout),
		.ddr_rvalid     (arm_ddr_rvalid),
		.halt_req       (arm_halt_req),
		.cpu_halted     (arm_halted),
		.mem_req        (arm2600_mem_req),
		.mem_ready      (arm2600_mem_ready),
		.mem_abort      (arm2600_mem_abort),
		.mem_addr       (arm_mem_addr),
		.mem_write      (arm_mem_write),
		.mem_wdata      (arm_mem_wdata),
		.mem_rdata      (arm2600_mem_rdata),
		.mem_size       (arm_mem_size),
		.mem_wstrb      (arm_mem_wstrb),
		.mem_fetch      (arm_mem_fetch),
		.state_req      (arm_state_req),
		.state_write    (arm_state_write),
		.state_index    (arm_state_index),
		.state_wdata    (arm_state_wdata),
		.state_rdata    (arm_state_rdata),
		.state_ready    (arm_state_ready),
		.state_commit   (arm_state_commit),
		.arm_call_busy,
		.arm_dma_busy,
		.fa2_nvram_request,
		.fa2_nvram_write,
		.fa2_nvram_addr,
		.fa2_nvram_wdata,
		.fa2_nvram_rdata,
		.fa2_nvram_ready,
		.fa2_nvram_dirty,
		.bus_stuff_valid,
		.bus_stuff_data,
		.oe             (),
		.open_bus       (open_bus),
		.tape_in        (tape_in),
		.tape_audio     (tape_audio),
		.fix_sc_cs      (fix_sc_cs)
	);

endmodule

// Accept one complete, source-caused two-phase stream. During a handoff the old
// source runs through a matching target edge, then the target supplies the
// opposite phase. This lets TIA remain externally clocked until the boundary.
module cpu_phase_controller
(
	input  logic clk_sys,
	input  logic request_tia,
	input  logic maria_phi1,
	input  logic maria_phi2,
	input  logic tia_phi1,
	input  logic tia_phi2,
	output logic phi1,
	output logic phi2,
	output logic active_tia,
	output logic edge_source_tia,
	output logic source_stable,
	output logic phase_valid
);

	typedef enum logic [1:0] {
		PRIME,
		RUN,
		WAIT_TARGET_SAME,
		TAKE_TARGET_OPPOSITE
	} phase_state_t;

	phase_state_t state = PRIME;
	logic target_tia = 1'b0;
	logic last_phase2 = 1'b1;
	initial begin
		active_tia = 1'b0;
		phase_valid = 1'b0;
	end

	wire active_phi1 = active_tia ? tia_phi1 : maria_phi1;
	wire active_phi2 = active_tia ? tia_phi2 : maria_phi2;
	wire requested_phi1 = request_tia ? tia_phi1 : maria_phi1;
	wire target_phi1 = target_tia ? tia_phi1 : maria_phi1;
	wire target_phi2 = target_tia ? tia_phi2 : maria_phi2;
	wire target_same = last_phase2 ? target_phi2 : target_phi1;
	wire target_matches_edge = (phi1 && target_phi1) ||
		(phi2 && target_phi2);

	assign source_stable = state == RUN;

	always_comb begin
		phi1 = 1'b0;
		phi2 = 1'b0;
		edge_source_tia = active_tia;

		case (state)
			PRIME: begin
				// Establish a known low/phase-1 level before any phase 2.
				edge_source_tia = request_tia;
				phi1 = requested_phi1;
			end
			RUN: begin
				phi1 = last_phase2 && active_phi1;
				phi2 = !last_phase2 && active_phi2;
			end
			WAIT_TARGET_SAME: begin
				phi1 = last_phase2 && active_phi1;
				phi2 = !last_phase2 && active_phi2;
			end
			TAKE_TARGET_OPPOSITE: begin
				edge_source_tia = target_tia;
				if (request_tia == target_tia) begin
					phi1 = last_phase2 && target_phi1;
					phi2 = !last_phase2 && target_phi2;
				end
			end
			default: begin
			end
		endcase
	end

	always_ff @(posedge clk_sys) begin
		if (phi1) begin
			last_phase2 <= 1'b0;
			phase_valid <= 1'b1;
		end else if (phi2) begin
			last_phase2 <= 1'b1;
			phase_valid <= 1'b1;
		end

		case (state)
			PRIME: begin
				if (phi1) begin
					active_tia <= request_tia;
					state <= RUN;
				end
			end
			RUN: begin
				if (request_tia != active_tia) begin
					target_tia <= request_tia;
					state <= WAIT_TARGET_SAME;
				end
			end
			WAIT_TARGET_SAME: begin
				if (request_tia == active_tia)
					state <= RUN;
				else if (request_tia != target_tia)
					target_tia <= request_tia;
				else if (target_same || target_matches_edge)
					state <= TAKE_TARGET_OPPOSITE;
			end
			TAKE_TARGET_OPPOSITE: begin
				if (request_tia == active_tia)
					state <= RUN;
				else if (request_tia != target_tia) begin
					target_tia <= request_tia;
					state <= WAIT_TARGET_SAME;
				end else if (phi1 || phi2) begin
					active_tia <= target_tia;
					state <= RUN;
				end
			end
		endcase
	end

endmodule

// INPUTCTRL register. Uses TIA CS.
module ctrl_reg
(
	input  logic       clk,
	input  logic       latch_b,
	input  logic       rst,
	input  logic [3:0] d_in,
	input  logic       cs,
	input  logic       bypass_bios,
	input  logic       tia_mode,
	input  logic       pclk0,
	output logic       lock_out,
	output logic       maria_en_out,
	output logic       bios_en_out,
	output logic       tia_en_out,
	output logic [1:0] writes
);

	always_ff @(posedge clk) begin
		reg wrote_once;
		if (rst) begin
			lock_out <= 0;
			maria_en_out <= 0;
			bios_en_out <= 0;
			tia_en_out <= 0;
			wrote_once <= 0;
			writes <= 0;
		end else if (bypass_bios && ~wrote_once) begin
			lock_out <= tia_mode ? 1'd1 : 1'd0;
			maria_en_out <= tia_mode ? 1'd0 : 1'd1;
			bios_en_out <= 1;
			wrote_once <= 1;
			tia_en_out <= tia_mode ? 1'd1 : 1'd0;
			writes <= 2'd2;
		end else if (~latch_b && cs && pclk0) begin
			wrote_once <= 1;
			lock_out <= d_in[0];
			maria_en_out <= d_in[1];
			bios_en_out <= d_in[2];
			tia_en_out <= d_in[3];
			if (writes < 2'd2)
				writes <= writes + 1'b1;
		end
	end
endmodule

module M6502C
(
	input         pclk1,     // start of phase 1, from MARIA or TIA
	input         pclk0,     // start of phase 2, from MARIA or TIA
	output        phi1_ce,   // pin 3:  the paired phase 1, for the rest of the system
	output        phi2_ce,   // pin 39: the paired phase 2, for the rest of the system
	input         clk_sys,   // MARIA Clock
	input         reset,     // reset signal
	input  [7:0]  DB_IN,     // data in,
	input         IRQ_n,     // interrupt request
	input         NMI_n,     // non-maskable interrupt request
	input         RDY,       // Ready signal. Pauses CPU when RDY=0
	input         halt_n,    // halt!
	output [15:0] AB,        // address bus
	output [7:0]  DB_OUT,    // data_out,
	output        RD,        // read enable
	output logic  is_halted  // This is used to indicate that sally has released the bus
);

	// The source controller removes MARIA restart and source-handoff hazards.
	// Keep this local pairing gate as the CPU pin invariant: an unexpected
	// duplicate phase must never advance SALLY or any returned system phase.
	logic in_phase2 = 1'b0;
	wire  phi1_en = pclk1 & ~in_phase2;
	wire  phi2_en = pclk0 &  in_phase2;

	always_ff @(posedge clk_sys) begin
		if      (phi1_en) in_phase2 <= 1'b1;
		else if (phi2_en) in_phase2 <= 1'b0;
	end

	// SALLY pin 39 is the system clock - TIA, RIOT, both cartridge slots and
	// MARIA pin 6 all run off it, not off MARIA's own divider. So the rest of
	// the core gets the same paired phases the CPU acted on, and a dropped
	// pulse is dropped for everyone. Taken before the halt gate: the pins keep
	// running while MARIA owns the bus.
	assign phi1_ce = phi1_en;
	assign phi2_ce = phi2_en;

	// SALLY carries the halt handshake itself: the two phase-2 flip-flops that
	// release the bus, and the stall into the core's own RDY. The wrapper is
	// only a name and a pin adapter.
	sally cpu (
		.clk_sys  (clk_sys),
		.phi1_en  (phi1_en),
		.phi2_en  (phi2_en),

		.res_n    (~reset),
		.rdy      (RDY),
		.irq_n    (IRQ_n),
		.nmi_n    (NMI_n),
		.so_n     (1'b1),
		.data_in  (RD ? DB_IN : DB_OUT),
		.data_out (DB_OUT),
		.data_oe  (),
		.addr_out (AB),
		.rw_n     (RD),
		.sync     (),
		.phi1_out (),
		.phi2_out (),

		.halt_n   (halt_n),
		.addr_oe  (),
		.rw_oe    (),
		.is_halted(is_halted),
		.jammed   (),

		.dbg_a (), .dbg_x (), .dbg_y (), .dbg_s (),
		.dbg_p (), .dbg_ir(), .dbg_pc()
	);

endmodule: M6502C
