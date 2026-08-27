# ARM7TDMI core

`arm7tdmi_core.sv` is the GPL-2.0 ARM/Thumb execution core derived from the
pinned GBA VHDL. `arm7tdmi_pkg.sv` and `arm7tdmi_pin_wrapper.sv` are independent
MIT interfaces. Distribution of the combined core must satisfy GPL-2.0.

The native interface is a single-outstanding request/ready bus. A request and
all payload signals remain stable until `mem_ready`. Read data is an aligned
32-bit word; the core selects and extends byte/halfword lanes. `mem_lock` covers
both SWP transfers.

`reset` is synchronous and active high. `ce=0` freezes architectural and bus
state. `irq_n` and `fiq_n` are active low and are accepted only at an instruction
boundary. `halt_req` likewise waits for an instruction boundary with no pending
transfer before asserting `halted`.

`mem_ready` accepts the current request. `mem_abort` is sampled only with an
accepted request. `mem_size` is byte `00`, halfword `01`, or word `10`; `11` is
reserved. Read data must contain the aligned 32-bit word. Byte and halfword
writes are replicated across `mem_wdata`, with `mem_wstrb` selecting the lanes.
`mem_fetch`, `mem_seq`, and `mem_privileged` identify opcode, sequential, and
privileged transfers. Operation is little-endian only.

With `ce` asserted and zero-wait memory, ordinary ARM and Thumb ALU instructions
retire once per clock after pipeline fill. Register shifts, operand-dependent
multiplies, transfers, block transfers, SWP, branches, and exception refills use
the ARM7TDMI Chapter 6 cycle classes covered by the standalone regression.

State access is available only while `halted`. Indices are defined in
`arm7tdmi_pkg.sv`. A write makes the image dirty; a valid `state_commit` is then
required before `halt_req` can release the core. Commit rejects invalid modes or
misaligned PCs by remaining halted.

The pin wrapper provides positive-edge ARM7TDMI-style bus semantics. It does not
claim the original macrocell's electrical, address-pipelining, or half-cycle
timing.

The pin polarities follow ARM DDI 0210C section 3.4. In particular, despite its
name, `nRW` is high for a write and low for a read. `nOPC` is low for an opcode
fetch, `nTRANS` is low for User mode and high for privileged modes, and `LOCK`
is high across both SWP transfers. Active-low `nWAIT` stretches a request; all
address and control outputs remain stable until it returns high. `ABORT` is
active high.

There is no coprocessor bus or implementation. Coprocessor and other unsupported
instruction encodings enter Undefined Instruction as listed in `CORRECTIONS.md`.
JTAG, EmbeddedICE, debug scan, big-endian operation, and electrical macrocell
compatibility are intentionally absent. The halted-state port is an integration
and savestate interface, not an ARM debug implementation.
