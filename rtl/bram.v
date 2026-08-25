// Single port block RAM. Drop-in replacement for the previous VHDL spram,
// built on cache_ram so Quartus gets an Altera primitive and every other
// tool gets a portable model. Reads take one clock; a write is visible on
// q in the same clock (read during write returns the new data).
//
//              cs=1                     cs=0
//   wren  ---> write happens            write suppressed
//   q     ---> memory contents          all ones (deselected)

module spram
#(
	parameter addr_width    = 8,
	parameter data_width    = 8,
	parameter mem_init_file = " ",    // .mif, Quartus only
	parameter sim_init_file = " ",    // .hex for $readmemh, simulation only
	parameter mem_name      = "MEM"   // In-System Memory Content Editor label
)
(
	input  wire                  clock,
	input  wire [addr_width-1:0] address,
	input  wire [data_width-1:0] data,
	/* verilator lint_off UNUSED */
	// Present for pin compatibility only. The VHDL this replaces left the
	// altsyncram clocken0 port unmapped, so enable never had an effect.
	input  wire                  enable,
	/* verilator lint_on UNUSED */
	input  wire                  wren,
	output wire [data_width-1:0] q,
	input  wire                  cs
);
	wire [data_width-1:0] q_int;

	assign q = cs ? q_int : {data_width{1'b1}};

	cache_ram #(
		.ADDR_WIDTH    (addr_width),
		.DATA_WIDTH    (data_width),
		.MEM_INIT_FILE (mem_init_file),
		.SIM_INIT_FILE (sim_init_file),
		.LPM_HINT      ({"ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=", mem_name})
	) u_ram (
		.clk_i   (clock),
		.addr_i  (address),
		.wren_i  (wren & cs),
		.wdata_i (data),
		.q_o     (q_int)
	);

endmodule
