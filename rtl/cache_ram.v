// Copyright (c) 2026 Jamie Blanks

// Parameterized synchronous block RAMs. Quartus uses Altera primitives;
// other tools use the portable models below. Accesses take one clock.

module cache_ram
#(
	parameter ADDR_WIDTH = 7,
	parameter DATA_WIDTH = 32,
	/* verilator lint_off UNUSEDPARAM */
	parameter MEM_INIT_FILE = " ",
	/* verilator lint_on UNUSEDPARAM */
	parameter SIM_INIT_FILE = " ",
	/* verilator lint_off UNUSEDPARAM */
	parameter DEVICE_FAMILY = "Cyclone V",
	// Caller supplies the whole hint so this module never builds a string.
	// Set ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=<name> to expose the memory
	// to the In-System Memory Content Editor.
	parameter LPM_HINT = "ENABLE_RUNTIME_MOD=NO"
	/* verilator lint_on UNUSEDPARAM */
)
(
	input  wire                  clk_i,
	input  wire [ADDR_WIDTH-1:0] addr_i,
	input  wire                  wren_i,
	input  wire [DATA_WIDTH-1:0] wdata_i,
	output wire [DATA_WIDTH-1:0] q_o
);
	localparam NUM_WORDS = (1 << ADDR_WIDTH);

	`ifdef ALTERA_RESERVED_QIS
	altsyncram #(
		.init_file                     (MEM_INIT_FILE),
		.intended_device_family        (DEVICE_FAMILY),
		.lpm_hint                      (LPM_HINT),
		.lpm_type                      ("altsyncram"),
		.numwords_a                    (NUM_WORDS),
		.operation_mode                ("SINGLE_PORT"),
		.outdata_aclr_a                ("NONE"),
		.outdata_reg_a                 ("UNREGISTERED"),
		.power_up_uninitialized        ("FALSE"),
		.ram_block_type                ("M10K"),
		.read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
		.width_a                       (DATA_WIDTH),
		.width_byteena_a               (1),
		.widthad_a                     (ADDR_WIDTH)
	) u_ram (
		.address_a (addr_i),
		.clock0    (clk_i),
		.data_a    (wdata_i),
		.q_a       (q_o),
		.wren_a    (wren_i)
	);

`else
	reg [DATA_WIDTH-1:0] q_out;

	(* ramstyle = "M10K, no_rw_check" *) reg [DATA_WIDTH-1:0] mem_q [0:NUM_WORDS-1];

	// The Quartus branch sets power_up_uninitialized = "FALSE", so a Cyclone V
	// M10K comes up zeroed at configuration. The portable model zeroes to
	// match the device rather than inventing X state.
	integer init_i;

	initial begin
		if (SIM_INIT_FILE != " ") begin
			$readmemh(SIM_INIT_FILE, mem_q);
		end else begin
			for (init_i = 0; init_i < NUM_WORDS; init_i = init_i + 1) begin
				mem_q[init_i] = {DATA_WIDTH{1'b0}};
			end
		end
	end

	always @(posedge clk_i) begin
		if (wren_i) begin
			mem_q[addr_i] <= wdata_i;
		end

		if (wren_i) begin
			q_out <= wdata_i;
		end else begin
			q_out <= mem_q[addr_i];
		end
	end

	assign q_o = q_out;
`endif

endmodule

// True dual-port RAM with independent related clocks. Callers must prevent
// mixed-port same-address writes; Cyclone V leaves that collision undefined.
module cache_ram_tdp_dc
#(
	parameter ADDR_WIDTH = 7,
	parameter DATA_WIDTH = 8,
	/* verilator lint_off UNUSEDPARAM */
	parameter MEM_INIT_FILE = " ",
	parameter DEVICE_FAMILY = "Cyclone V",
	/* verilator lint_on UNUSEDPARAM */
	parameter SIM_INIT_FILE = " "
)
(
	input  wire                  clk_a_i,
	input  wire [ADDR_WIDTH-1:0] addr_a_i,
	input  wire                  wren_a_i,
	input  wire [DATA_WIDTH-1:0] wdata_a_i,
	output wire [DATA_WIDTH-1:0] q_a_o,
	input  wire                  clk_b_i,
	input  wire [ADDR_WIDTH-1:0] addr_b_i,
	input  wire                  wren_b_i,
	input  wire [DATA_WIDTH-1:0] wdata_b_i,
	output wire [DATA_WIDTH-1:0] q_b_o
);
	localparam NUM_WORDS = (1 << ADDR_WIDTH);

	`ifdef ALTERA_RESERVED_QIS
	altsyncram #(
		.address_reg_b                 ("CLOCK1"),
		.indata_reg_b                  ("CLOCK1"),
		.init_file                     (MEM_INIT_FILE),
		.intended_device_family        (DEVICE_FAMILY),
		.lpm_hint                      ("ENABLE_RUNTIME_MOD=NO"),
		.lpm_type                      ("altsyncram"),
		.numwords_a                    (NUM_WORDS),
		.numwords_b                    (NUM_WORDS),
		.operation_mode                ("BIDIR_DUAL_PORT"),
		.outdata_aclr_a                ("NONE"),
		.outdata_aclr_b                ("NONE"),
		.outdata_reg_a                 ("UNREGISTERED"),
		.outdata_reg_b                 ("UNREGISTERED"),
		.power_up_uninitialized        ("FALSE"),
		.ram_block_type                ("M10K"),
		.read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
		.read_during_write_mode_port_b ("NEW_DATA_NO_NBE_READ"),
		.width_a                       (DATA_WIDTH),
		.width_b                       (DATA_WIDTH),
		.width_byteena_a               (1),
		.width_byteena_b               (1),
		.widthad_a                     (ADDR_WIDTH),
		.widthad_b                     (ADDR_WIDTH),
		.wrcontrol_wraddress_reg_b     ("CLOCK1")
	) u_ram (
		.address_a (addr_a_i),
		.address_b (addr_b_i),
		.clock0    (clk_a_i),
		.clock1    (clk_b_i),
		.data_a    (wdata_a_i),
		.data_b    (wdata_b_i),
		.q_a       (q_a_o),
		.q_b       (q_b_o),
		.wren_a    (wren_a_i),
		.wren_b    (wren_b_i)
	);

	`else
	reg [DATA_WIDTH-1:0] q_a_out;
	reg [DATA_WIDTH-1:0] q_b_out;
	(* ramstyle = "M10K, no_rw_check" *) reg [DATA_WIDTH-1:0] mem_q [0:NUM_WORDS-1];
	integer init_i;

	initial begin
		if (SIM_INIT_FILE != " ") begin
			$readmemh(SIM_INIT_FILE, mem_q);
		end else begin
			for (init_i = 0; init_i < NUM_WORDS; init_i = init_i + 1)
				mem_q[init_i] = {DATA_WIDTH{1'b0}};
		end
	end

	always @(posedge clk_a_i) begin
		if (wren_a_i)
			mem_q[addr_a_i] <= wdata_a_i;
		q_a_out <= wren_a_i ? wdata_a_i : mem_q[addr_a_i];
	end

	always @(posedge clk_b_i) begin
		if (wren_b_i)
			mem_q[addr_b_i] <= wdata_b_i;
		q_b_out <= wren_b_i ? wdata_b_i : mem_q[addr_b_i];
	end

	assign q_a_o = q_a_out;
	assign q_b_o = q_b_out;
	`endif
endmodule

// True dual-port RAM with byte enables on both related-clock ports.
module cache_ram_tdp_dc_be
#(
	parameter ADDR_WIDTH = 6,
	parameter DATA_WIDTH = 32,
	/* verilator lint_off UNUSEDPARAM */
	parameter DEVICE_FAMILY = "Cyclone V"
	/* verilator lint_on UNUSEDPARAM */
)
(
	input  wire                         clk_a_i,
	input  wire        [ADDR_WIDTH-1:0] addr_a_i,
	input  wire                         wren_a_i,
	input  wire [(DATA_WIDTH/8)-1:0]    byteena_a_i,
	input  wire        [DATA_WIDTH-1:0] wdata_a_i,
	output wire        [DATA_WIDTH-1:0] q_a_o,
	input  wire                         clk_b_i,
	input  wire        [ADDR_WIDTH-1:0] addr_b_i,
	input  wire                         wren_b_i,
	input  wire [(DATA_WIDTH/8)-1:0]    byteena_b_i,
	input  wire        [DATA_WIDTH-1:0] wdata_b_i,
	output wire        [DATA_WIDTH-1:0] q_b_o
);
	localparam NUM_WORDS = (1 << ADDR_WIDTH);
	localparam NUM_BYTES = DATA_WIDTH / 8;

	`ifdef ALTERA_RESERVED_QIS
	altsyncram #(
		.address_reg_b                 ("CLOCK1"),
		.indata_reg_b                  ("CLOCK1"),
		.init_file                     ("UNUSED"),
		.intended_device_family        (DEVICE_FAMILY),
		.lpm_hint                      ("ENABLE_RUNTIME_MOD=NO"),
		.lpm_type                      ("altsyncram"),
		.numwords_a                    (NUM_WORDS),
		.numwords_b                    (NUM_WORDS),
		.operation_mode                ("BIDIR_DUAL_PORT"),
		.outdata_aclr_a                ("NONE"),
		.outdata_aclr_b                ("NONE"),
		.outdata_reg_a                 ("UNREGISTERED"),
		.outdata_reg_b                 ("UNREGISTERED"),
		.power_up_uninitialized        ("FALSE"),
		.ram_block_type                ("M10K"),
		.read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
		.read_during_write_mode_port_b ("NEW_DATA_NO_NBE_READ"),
		.width_a                       (DATA_WIDTH),
		.width_b                       (DATA_WIDTH),
		.width_byteena_a               (NUM_BYTES),
		.width_byteena_b               (NUM_BYTES),
		.widthad_a                     (ADDR_WIDTH),
		.widthad_b                     (ADDR_WIDTH),
		.wrcontrol_wraddress_reg_b     ("CLOCK1")
	) u_ram (
		.address_a (addr_a_i),
		.address_b (addr_b_i),
		.byteena_a (byteena_a_i),
		.byteena_b (byteena_b_i),
		.clock0    (clk_a_i),
		.clock1    (clk_b_i),
		.data_a    (wdata_a_i),
		.data_b    (wdata_b_i),
		.q_a       (q_a_o),
		.q_b       (q_b_o),
		.wren_a    (wren_a_i),
		.wren_b    (wren_b_i)
	);

	`else
	reg [DATA_WIDTH-1:0] q_a_out;
	reg [DATA_WIDTH-1:0] q_b_out;
	(* ramstyle = "M10K, no_rw_check" *) reg [DATA_WIDTH-1:0] mem_q [0:NUM_WORDS-1];
	integer init_i;
	integer byte_a;
	integer byte_b;

	initial begin
		for (init_i = 0; init_i < NUM_WORDS; init_i = init_i + 1)
			mem_q[init_i] = {DATA_WIDTH{1'b0}};
	end

	always @(posedge clk_a_i) begin
		if (wren_a_i) begin
			for (byte_a = 0; byte_a < NUM_BYTES; byte_a = byte_a + 1)
				if (byteena_a_i[byte_a])
					mem_q[addr_a_i][byte_a*8 +: 8] <= wdata_a_i[byte_a*8 +: 8];
		end
		q_a_out <= wren_a_i ? wdata_a_i : mem_q[addr_a_i];
	end

	always @(posedge clk_b_i) begin
		if (wren_b_i) begin
			for (byte_b = 0; byte_b < NUM_BYTES; byte_b = byte_b + 1)
				if (byteena_b_i[byte_b])
					mem_q[addr_b_i][byte_b*8 +: 8] <= wdata_b_i[byte_b*8 +: 8];
		end
		q_b_out <= wren_b_i ? wdata_b_i : mem_q[addr_b_i];
	end

	assign q_a_o = q_a_out;
	assign q_b_o = q_b_out;
	`endif
endmodule

module cache_ram_dp
#(
	parameter ADDR_WIDTH = 7,
	parameter DATA_WIDTH = 32,
	/* verilator lint_off UNUSEDPARAM */
	parameter MEM_INIT_FILE = " ",
	parameter DEVICE_FAMILY = "Cyclone V",
	/* verilator lint_on UNUSEDPARAM */
	parameter SIM_INIT_FILE = " "
)
(
	input  wire                  clk_i,
	input  wire [ADDR_WIDTH-1:0] addr_a_i,
	input  wire                  wren_a_i,
	input  wire [DATA_WIDTH-1:0] wdata_a_i,
	output wire [DATA_WIDTH-1:0] q_a_o,
	input  wire [ADDR_WIDTH-1:0] addr_b_i,
	input  wire                  wren_b_i,
	input  wire [DATA_WIDTH-1:0] wdata_b_i,
	output wire [DATA_WIDTH-1:0] q_b_o
);
	localparam NUM_WORDS = (1 << ADDR_WIDTH);

	`ifdef ALTERA_RESERVED_QIS
	altsyncram #(
		.address_reg_b                 ("CLOCK0"),
		.clock_enable_output_a         ("BYPASS"),
		.clock_enable_output_b         ("BYPASS"),
		.indata_reg_b                  ("CLOCK0"),
		.init_file                     (MEM_INIT_FILE),
		.intended_device_family        (DEVICE_FAMILY),
		.lpm_hint                      ("ENABLE_RUNTIME_MOD=NO"),
		.lpm_type                      ("altsyncram"),
		.numwords_a                    (NUM_WORDS),
		.numwords_b                    (NUM_WORDS),
		.operation_mode                ("BIDIR_DUAL_PORT"),
		.outdata_aclr_a                ("NONE"),
		.outdata_aclr_b                ("NONE"),
		.outdata_reg_a                 ("UNREGISTERED"),
		.outdata_reg_b                 ("UNREGISTERED"),
		.power_up_uninitialized        ("FALSE"),
		.ram_block_type                ("M10K"),
		.read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
		.read_during_write_mode_port_b ("NEW_DATA_NO_NBE_READ"),
		.width_a                       (DATA_WIDTH),
		.width_b                       (DATA_WIDTH),
		.width_byteena_a               (1),
		.width_byteena_b               (1),
		.widthad_a                     (ADDR_WIDTH),
		.widthad_b                     (ADDR_WIDTH),
		.wrcontrol_wraddress_reg_b     ("CLOCK0")
	) u_ram (
		.address_a (addr_a_i),
		.address_b (addr_b_i),
		.clock0    (clk_i),
		.data_a    (wdata_a_i),
		.data_b    (wdata_b_i),
		.q_a       (q_a_o),
		.q_b       (q_b_o),
		.wren_a    (wren_a_i),
		.wren_b    (wren_b_i)
	);

`else
	reg [DATA_WIDTH-1:0] q_a_out;
	reg [DATA_WIDTH-1:0] q_b_out;

	(* ramstyle = "M10K, no_rw_check" *) reg [DATA_WIDTH-1:0] mem_q [0:NUM_WORDS-1];

	// The Quartus branch sets power_up_uninitialized = "FALSE", so a Cyclone V
	// M10K comes up zeroed at configuration. The portable model zeroes to
	// match the device rather than inventing X state.
	integer init_i;

	initial begin
		if (SIM_INIT_FILE != " ") begin
			$readmemh(SIM_INIT_FILE, mem_q);
		end else begin
			for (init_i = 0; init_i < NUM_WORDS; init_i = init_i + 1) begin
				mem_q[init_i] = {DATA_WIDTH{1'b0}};
			end
		end
	end

	always @(posedge clk_i) begin
		if (wren_a_i) begin
			mem_q[addr_a_i] <= wdata_a_i;
		end
		if (wren_a_i) begin
			q_a_out <= wdata_a_i;
		end else begin
			q_a_out <= mem_q[addr_a_i];
		end

		if (wren_b_i) begin
			mem_q[addr_b_i] <= wdata_b_i;
		end
		if (wren_b_i) begin
			q_b_out <= wdata_b_i;
		end else begin
			q_b_out <= mem_q[addr_b_i];
		end
	end

	assign q_a_o = q_a_out;
	assign q_b_o = q_b_out;
`endif

endmodule
