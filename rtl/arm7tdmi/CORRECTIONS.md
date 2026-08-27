# ARM7TDMI correction manifest

Source-equivalence differences are forbidden unless listed here with a primary
manual section and focused test. These entries are applied only at the native
core boundary; they are not changes to the pinned VHDL reference.

| ID | Source behavior | Corrected behavior | Primary section | Focused test |
|---|---|---|---|---|
| ARM7-C001 | The GBA entity has no FIQ pin or FIQ entry path. | Sample active-low `fiq_n` at instruction boundaries, bank R8-R14, save CPSR, set I/F, clear T, and enter `0x1c`. | ARM ARM DDI 0100I A2.6.9 | `tb_exceptions.sv` FIQ case |
| ARM7-C002 | The GBA bus has no instruction/data abort input. | `mem_abort` on a fetch enters Prefetch Abort with LR = fault PC + 4; on data it enters Data Abort with LR = instruction PC + 8. | ARM ARM DDI 0100I A2.6.5-A2.6.6 | `tb_exceptions.sv` abort cases |
| ARM7-C003 | ARM opcode group `111` is routed through the GBA SWI path, so unsupported coprocessor encodings do not enter Undefined mode. | Only SWI encodings enter Supervisor; unsupported coprocessor and undefined encodings save CPSR/LR and enter `0x04` in Undefined mode. | ARM ARM DDI 0100I A2.6.3 | `tb_exceptions.sv` unsupported MRC case |
| ARM7-C004 | The GBA IRQ input includes GBA cycle-delay and DMA coupling. | Sample active-low `irq_n` at a retirement boundary and use architectural IRQ LR, SPSR, mode, mask, and vector state. | ARM ARM DDI 0100I A2.6.8 | `tb_exceptions.sv` IRQ case |
| ARM7-C005 | A non-word-aligned `LDR` writes the aligned bus word unchanged. | Rotate the aligned word right by `8 * address[1:0]`, placing the addressed byte in bits 7:0 in little-endian mode. | ARM ARM DDI 0100I A2.8 and A4 `LDR` alignment | `tb_instruction_classes.sv` unaligned load case and instruction-class differential |
