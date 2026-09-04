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
#   outclk_0  counter[0]  VCO/40  14.31641 MHz  69.847 ns  clk_sys
#   outclk_1  counter[1]  VCO/10  57.26563 MHz  17.462 ns  clk_vid
#   outclk_2  counter[2]  VCO/80   7.15820 MHz             clk_tia   (unused)
#   outclk_3  counter[3]  VCO/8   71.58203 MHz  13.969 ns  clk_arm
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

# Finding emu|pll's outputs. altera_pll names its generated clocks after the
# IP variant it was built as, so the name changes when the IP is retyped:
#
#   pll_type "General"    ...|altera_pll_i|general[N].gpll~PLL_OUTPUT_COUNTER|divclk
#   pll_type "Cyclone V"  ...|altera_pll_i|cyclonev_pll|counter[N].output_counter|divclk
#
# The region retune made emu|pll "Cyclone V"/"Reconfigurable", which renamed
# every output. Both spellings are matched here, and a miss is reported: the
# previous single-pattern guard wrapped the whole file, so the rename turned
# every exception below into a no-op and nothing said a word.
proc a78_pll_divclk {n} {
	foreach pat [list \
		"*|pll|pll_inst|altera_pll_i|cyclonev_pll|counter\[$n\].output_counter|divclk" \
		"*|pll|pll_inst|altera_pll_i|general\[$n\].*|divclk"] {
		set c [get_clocks -nowarn $pat]
		if {[get_collection_size $c] > 0} { return $c }
	}
	return {}
}

set core_clk_sys [a78_pll_divclk 0]

if {[get_collection_size $core_clk_sys] == 0} {
	post_message -type critical_warning \
		"Atari7800.sdc: emu|pll clk_sys not found; the SDRAM read result is unrelaxed"
} else {
	set core_sdram [get_registers -nowarn {*|sdram:sdram|*}]

	if {[get_collection_size $core_sdram] > 0} {
		set_multicycle_path -setup 2 -from $core_sdram -to $core_clk_sys
		set_multicycle_path -hold  1 -from $core_sdram -to $core_clk_sys
	} else {
		post_message -type warning \
			"Atari7800.sdc: sdram registers not found; the read result is unrelaxed"
	}

	unset core_sdram
}
unset core_clk_sys


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
# Since the 2026-08-31 core restructure the multiplier is iterative and never
# on that loop, so the operand loop and the mapper's combinational answer are
# what remain; closing them needs RTL. The file below relaxes the one thing
# that is an SDC problem, the quasi-static cartridge selection reaching the
# ARM's memory map.
source rtl/arm7tdmi/arm7tdmi.sdc
