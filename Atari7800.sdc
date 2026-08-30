# Timing constraints for the Atari 7800 core.
#
# sys/sys_top.sdc creates every clock (derive_pll_clocks) and cuts every
# unrelated clock group. Nothing here creates a clock. This file only says
# which paths inside the core do not need the full period the analyzer
# assumes, and why.
#
#
# THE CLOCKS
#
# One PLL (emu|pll) makes all four core clocks from a 572.65625 MHz VCO:
#
#   outclk_0  general[0]  VCO/40  14.31641 MHz  69.847 ns  clk_sys
#   outclk_1  general[1]  VCO/10  57.26563 MHz  17.462 ns  clk_vid
#   outclk_2  general[2]  VCO/80   7.15820 MHz             clk_tia   (unused)
#   outclk_3  general[3]  VCO/8   71.58203 MHz  13.969 ns  clk_arm
#
# clk_tia is wired in Atari7800.sv but nothing is clocked by it; it carries no
# timed path. clk_vid is exactly 4 x clk_sys and clk_arm exactly 5 x clk_sys,
# so all three share edges and sys_top.sdc keeps them in one -exclusive group.
# Every crossing between them is fully timed, at the shorter of the two
# periods in each direction.
#
# Almost everything the core emulates runs on clk_sys behind clock enables,
# not on a clock of its own:
#
#   mclk0 / mclk1   MARIA master, 7.16 MHz - alternate clk_sys ticks
#   pclk1 / pclk0   SALLY phi1 / phi2, 1.79 MHz - one tick in four
#
# Path counts between the three live clocks, from the fit's Setup Transfers
# panel:
#
#   clk_vid -> clk_sys   36,119,381        the SDRAM read result
#   clk_sys -> clk_arm   12,008,633        cartridge selection + the mailbox
#   clk_sys -> clk_vid      873,062        video out, and the SDRAM request
#   clk_arm -> clk_sys          210        mailbox returns only


# THE SDRAM READ RESULT
#
# sdram runs on clk_vid. Its only path into clk_sys is ch0_dout, and every one
# of the 15,594 failing clk_vid -> clk_sys paths in the fitted design launches
# from one register family, sdram|last_data. They land in cart2600's mapper_P2
# flags, DPC+'s service registers, and MARIA's line_ram write side, all through
# the cartridge data bus, which is a deep combinational cone: 17.4 ns of logic
# against a 17.463 ns requirement.
#
# That requirement is the analyzer assuming any clk_sys edge may capture the
# byte. No consumer does. The byte belongs to one bus cycle and is read at the
# enable that ends that cycle:
#
#   2600, clk_sys ticks, phi1 at T:
#     T     SALLY moves the address; address_change high for this tick
#     T+1   rom_read rises - the controller starts at clk_vid 4T+5
#     T+3.75  last_data captured, clk_vid 4T+11
#     T+4   phi2. The mapper takes the byte, clk_vid 4T+16
#                                             -> 5 clk_vid periods
#
#   7800, MARIA master ticks, address out at mclk1 R-2:
#     R-2   AB and cart_read - the controller starts at clk_vid 4R-7
#     R-0.25  last_data captured, clk_vid 4R-1
#     R+1   mclk0. line_ram takes the byte, clk_vid 4R+4
#                                             -> 5 clk_vid periods
#
# Both paths leave 5. Claim 2. The three unclaimed periods cover the modes not
# drawn above - the PAL divider skip, MARIA's slow clock, pause - all of which
# only stretch the interval, and cover the 7800 line being off by a cycle.
#
# Scoping -to the clock rather than to a register list leaves last_data fully
# timed everywhere else it goes. -hold 1 puts the hold check back where the
# default had it; without it, -setup 2 would move the hold check a period late.

if {[get_collection_size [get_clocks -nowarn {*|pll|pll_inst|altera_pll_i|general[0].*|divclk}]] > 0} {

	set core_clk_sys [get_clocks {*|pll|pll_inst|altera_pll_i|general[0].*|divclk}]
	set core_sdram   [get_registers -nowarn {*|sdram:sdram|*}]

	if {[get_collection_size $core_sdram] > 0} {
		set_multicycle_path -setup 2 -from $core_sdram -to $core_clk_sys
		set_multicycle_path -hold  1 -from $core_sdram -to $core_clk_sys
	} else {
		post_message -type warning \
			"Atari7800.sdc: sdram registers not found; the read result is unrelaxed"
	}

	unset core_clk_sys core_sdram
}


# THE SDRAM PINS ARE DELIBERATELY UNCONSTRAINED
#
# SDRAM_DQ and the command pins carry no set_input_delay or set_output_delay,
# here or in any MiSTer core. The board's trace lengths are not ours to state,
# and a guessed number would either hide a real failure or invent one. The
# interface is a stock 57 MHz Sorgelig controller with CAS 2 and a full clk_vid
# period of margin on the return path.


# WHAT IS NOT RELAXED, ON PURPOSE
#
# The clk_sys clock-enable world - MARIA, TIA, RIOT, SALLY and the cartridge -
# holds most of its state for 2 or 4 clk_sys ticks and would take a large
# multi-cycle. It is not written here because it does not need one: with the
# SDRAM crossing above relaxed, no clk_sys -> clk_sys path fails. An exception
# nothing needs is an exception nobody can check.


# The clk_arm island: the ARM7TDMI, its memory system, and the clk_sys seam.
#
# clk_arm is the core's worst clock, -2.515 ns setup, and none of that is an
# SDC problem: the paths are the ARM's own decode and operand mux in series
# with arm_mapper_memory's combinational answer, inside one 13.969 ns period.
# The multiply is genuinely single-cycle - dp_multiply_wait can be 1, and
# MUL_WAIT reads multiply_result on the very next edge when it is. Closing it
# needs RTL. The file below relaxes the one thing that is an SDC problem, the
# quasi-static cartridge selection reaching the ARM's memory map.
source rtl/arm7tdmi/arm7tdmi.sdc
