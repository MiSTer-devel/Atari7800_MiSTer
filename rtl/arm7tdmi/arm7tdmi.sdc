# Timing constraints for the ARM7TDMI and the clk_arm island around it.
#
# Source this from the revision SDC (Atari7800.sdc):
#
#     source rtl/arm7tdmi/arm7tdmi.sdc
#
# It is also safe to source from the standalone probe
# (.agents/build/arm7tdmi_probe/arm7tdmi_probe.sdc): everything below is
# guarded on clk_sys existing, and in the probe it does not.
#
#
# CLOCKS
#
# One PLL (emu|pll) makes every clock the core uses, from a 572.65625 MHz VCO:
#
#   outclk_0  general[0]  VCO/40  14.31641 MHz  69.847 ns  clk_sys
#   outclk_1  general[1]  VCO/10  57.26563 MHz  17.462 ns  clk_vid
#   outclk_3  general[3]  VCO/8   71.58203 MHz  13.969 ns  clk_arm
#
# clk_arm is exactly 5 x clk_sys. sys/sys_top.sdc calls derive_pll_clocks, so
# the clocks already exist; it also puts all four emu PLL outputs in one
# -exclusive group, which means clk_sys <-> clk_arm paths are fully timed
# against each other with a 13.969 ns requirement in both directions. Nothing
# here creates a clock; this file only says which of those paths do not need
# the full 13.969 ns.
#
# There are no clk_vid <-> clk_arm paths at all (measured: 0), so the 3.5 ns
# window the comments in Atari7800.sv warn about is not in play today.
#
#
# WHAT THE CPU CORE ITSELF NEEDS: NOTHING
#
# arm7tdmi_core has no multi-cycle paths worth constraining. Every state that
# looks multi-cycle from the outside re-launches from registers on each clk_arm
# edge, so the logic between registers is always one cycle:
#
#   MUL_WAIT      multiply_result is captured in a single edge, out of the
#                 32x32 -> 64 multiplier plus its sign correction and 64-bit
#                 accumulate. multiply_wait then counts the architectural
#                 cycles the real part spends, but it can be as low as 1, so
#                 the result is read the very next edge. The multiplier is a
#                 hard single-cycle path and only RTL can change that.
#   MEM_ACCESS    entered and finished on consecutive edges when the memory
#                 answers in its request cycle, which it does for a cache hit
#                 or a held-line fetch.
#   BLOCK_ACCESS  one transfer per edge; a one-register LDM enters and leaves
#                 on consecutive edges.
#   HALTED        the state port is a level handshake. state_ready is asserted
#                 in the same edge the core sees state_req, and the controller
#                 moves state_index on the next, so index -> state_rdata is one
#                 cycle.
#
# The only paths inside the core that are structurally longer than one cycle
# are swp_resume_pc and swp_rd, which SWP_READ holds across into SWP_WRITE.
# They are two 32-bit values feeding an adder and a compare, nowhere near the
# critical path, so constraining them would be noise.
#
# The core's real problem is not a missing multi-cycle. It is that
# arm_mapper_memory answers combinationally - mem_ready and mem_rdata are
# driven straight out of the address-phase decode with nothing registered on
# the way - so the mapper's whole memory-map decode sits in series with the
# core's decode, operand mux and barrel shifter inside one clk_arm period.
# That is where the worst clk_arm path lives, and it needs RTL, not SDC.
#
#
# WHAT DOES NEED CONSTRAINING: THE CARTRIDGE SELECTION
#
# Measured on the fitted database, clk_sys -> clk_arm carries 10,941 paths.
# 10,386 of them - 95% - launch from three quasi-static configuration bits:
#
#   hps_io|status[60:56]      OSD "bankswitch scheme" override
#   use_tape                  Supercharger tape mode
#   detect2600|force_bs[*]    scheme detected while the ROM downloaded
#
# Atari7800.sv:367 combines these into `mapper`, cart2600 turns that into the
# ARM's memory map and mapper_ram_size, and arm_mapper_memory reads it in the
# combinational address decode. So an OSD bit lands in the middle of the ARM's
# longest logic cone with a 13.969 ns budget, and two of them miss it
# (status[56] at -0.192, use_tape at -0.097).
#
# None of this is data the ARM is waiting on. It only changes when a ROM is
# loaded or the user picks a scheme by hand, both of which reset the ARM, and
# nothing sequences it - there is no handshake because there is nothing to
# hand off. Cutting it is what it always meant.
#
# Also worth knowing, from the same measurement: clk_arm -> clk_sys carries
# only 210 paths, every one of them a held payload or a toggle entering the
# first flop of a two-flop synchronizer, and the worst passes by 7.772 ns.

if {[get_collection_size [get_clocks -nowarn {*|pll|pll_inst|altera_pll_i|general[0].*|divclk}]] > 0} {

	set arm7_clk_arm [get_clocks {*|pll|pll_inst|altera_pll_i|general[3].*|divclk}]

	# The cartridge selection reaches clk_arm whenever it likes, and nothing on
	# the far side sequences it, so cut it.
	#
	# A false path rather than a multicycle, because there is no interval to
	# derive. A multicycle has to name a number of cycles, and the honest number
	# here would be "however long until the next ROM load" - the clk_sys period
	# these registers happen to hold their value for is real but arbitrary, not a
	# deadline any consumer imposes. Nothing reads these bits in a known cycle,
	# so there is no edge to be relative to and nothing to check the exception
	# against later. Cutting the path says that; a multicycle would dress it up
	# as an interval somebody could later mistake for a derivation.
	#
	# A multicycle would be safe if you wanted one. `-setup N -hold N-1` leaves
	# the hold check on the launch edge, same as an unconstrained path - measured
	# on this design, see decision 0073.
	set arm7_config [add_to_collection \
		[get_registers -nowarn {*hps_io:*|status[*]}] \
		[add_to_collection \
			[get_registers -nowarn {*|use_tape*}] \
			[get_registers -nowarn {*detect2600:*|force_bs[*]}]]]

	if {[get_collection_size $arm7_config] > 0} {
		# Scoping -to the clock and not to a register list leaves these signals
		# fully timed everywhere else they go.
		set_false_path -from $arm7_config -to $arm7_clk_arm
	} else {
		post_message -type warning \
			"arm7tdmi.sdc: cartridge-selection registers not found; clk_arm is unrelaxed"
	}

	# The mailbox crossings in arm_mapper_memory and arm_mapper_controller are
	# deliberately left fully timed. They already pass with 5.285 ns to spare,
	# and cutting them is not free either: each held payload is armed by a
	# toggle through two synchronizer flops, so the far side reads the payload
	# three clk_arm edges after the launch at the earliest. A set_false_path to
	# *_sync1 would let the toggle route arbitrarily fast while the payload
	# stays bounded, which inverts the order the handshake depends on.

	unset arm7_clk_arm arm7_config
}
