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
# edge, so the logic between registers is always one cycle. Rewritten after the
# 2026-08-31 restructure (decisions 0002 and 0004, CHANGES.md items 1-18),
# because the structures the previous text described are gone:
#
#   MUL_WAIT      The multiplier walks eight multiplier bits per edge: one
#                 32x8 product and a 64-bit add from the iteration registers,
#                 with group 0 folded into the entry edge from the execute
#                 registers. The carry-save walk beside it is the same shape.
#                 The retire edge reads only registers (mul_acc, mc_carry),
#                 so the multiplier is never on the forwarding loop - the fit
#                 that briefly had it there measured -3.841 ns, the campaign's
#                 low. The old single-edge 32x32 product no longer exists.
#   MEM_ACCESS    entered and finished on consecutive edges when the memory
#                 answers in its request cycle; loads then spend one MEM_DONE
#                 edge (the manual's trailing I cycle) before resuming.
#   BLOCK_ACCESS  one transfer per edge. The next list position is registered
#                 a cycle ahead (block_next_index, block_rest); the comb
#                 isolate that computes it lands in registers only.
#   HALTED        the state port is a level handshake. state_ready is asserted
#                 in the same edge the core sees state_req, and the controller
#                 moves state_index on the next, so index -> state_rdata is one
#                 cycle. The register file is flat (rf[0:29], laid out as the
#                 state map), so there is no bank capture on halt either.
#
# Registers that are structurally held across more than one edge - the resume
# PCs (data_resume_pc, swp_resume_pc, block_resume_pc, multiply_resume_pc,
# mem_done_pc), the transfer descriptors (data_*, swp_*, block_*), and the
# iteration state - feed adders, compares and muxes nowhere near the critical
# path. Constraining them would be noise, and it would also be wrong for the
# one-register cases (a one-register LDM, an m=1 multiply) that do read them
# on the very next edge.
#
# Where the core's own worst path is, measured standalone at 71.59 MHz on the
# probe (single seeds, 2026-08-31, evidence/arm7tdmi-restructure-fits.md):
# the dependent-instruction operand loop - a forwarded ALU result through the
# forward select, the operand select, the rotator-based shifter and the ALU
# again inside one edge - at -0.81 to -0.89 ns, with the launch nodes that
# each fit named (the block-walk isolate, the read-port index mux, the bus
# qualifiers on the forwarding valid) retired one by one. What remains is the
# loop itself, and it is one cycle by definition: nothing in it is a
# candidate for a multicycle.
#
# Names, for anyone matching registers from a revision SDC or QSF: the
# register file is `rf`, not `regs` and the six bank arrays; the fetch
# counter is `fetch_pc`, not `next_fetch_pc`; the block list is `block_rest`.
# Nothing in this file or the parent's ever named them, so no exception
# silently went stale - but check before assuming.
#
# The core's real integration problem is not a missing multi-cycle. It is
# that arm_mapper_memory answers combinationally - mem_ready and mem_rdata
# are driven straight out of the address-phase decode with nothing registered
# on the way - so the mapper's whole memory-map decode sits in series with the
# core's decode, operand mux and barrel shifter inside one clk_arm period.
# The 2026-08-31 restructure took mem_ready off the forwarding valid inside
# the core (a forward computed on a non-completing cycle is discarded, so the
# qualifier was never needed), which shortens that series path by the bus
# qualifier's share; the mapper's decode itself still needs RTL, not SDC.
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
#   cart_is_7800              set from the ROM header at Atari7800.sv:587
#   a78_cart_extent|*         cartridge size from the download and the header
#
# cart_is_7800 was added after that measurement. It selects the 2600 mapper at
# Atari7800.sv:366 and so lands in the same decode cone; on the 2026-08-30 fit
# it launched all five worst clk_arm paths, -4.027 to -3.978 ns. It is the same
# class as the other three - written once while the ROM downloads, never read
# in a known cycle.
#
# a78_cart_extent arrived the same way and was missed the same way. On the
# 2026-08-31 09:20 fit cart_size_eof launched every one of the hundred worst
# clk_arm paths, -3.744 ns at the top and TNS -2476 ns, which buried whatever
# else clk_arm was failing. It sizes the cartridge from the download, so it is
# the same class again: it stops changing when the ROM finishes loading, and
# the ARM is in reset until after that.
#
# It is matched by instance and not by register name, unlike the four above,
# because physical synthesis retimed registers into its comparators and named
# them itself - LessThan1~3_OTERM7878 and 74 more. Naming cart_size_eof and
# hcart_size cut only -3.744 to -3.491; the retimed copies became the new worst
# paths. A name the fitter invents cannot be written down in advance, so the
# exception has to cover the instance.
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
	set arm7_config [get_registers -nowarn {*hps_io:*|status[*]}]
	foreach arm7_pattern {
		*|use_tape*
		*detect2600:*|force_bs[*]
		*|cart_is_7800*
		*a78_cart_extent:*
	} {
		set arm7_config [add_to_collection $arm7_config \
			[get_registers -nowarn $arm7_pattern]]
	}

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

	unset arm7_clk_arm arm7_config arm7_pattern
}
