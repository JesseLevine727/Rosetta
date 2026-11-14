# AI usage log

Kept as the work happens, for the write-up. Overseer: Claude Fable 5.2.
Workers: Claude Opus 5.5. The human owner approves the items listed under
"User approvals" in `PLAN.md`.

| Date | Who | Task | How the result was checked |
|---|---|---|---|
| 13 Nov | Overseer (then Opus 5.5) | Competition breakdown; exact 8x4 area budget from TT and PDK files | Numbers taken from the DEF, LEF and tile-size files, cited in `docs/area-budget.md` |
| 13 Nov | Overseer (Fable 5.2) | Architecture spec, plan, diagram | Approved by the owner |
| 13 Nov | Overseer | Instruction set v0.1, v0.2 | Reviewed independently by three workers (language, model, shared-logic specs), who found 32 + 9 + 25 issues; rulings in `docs/decisions.md` |
| 13 Nov | Worker (Opus 5.5) | Area probe: representative lane and shared logic synthesized to CMOS5L | Overseer checked the headline area and flop counts against the raw Yosys stat files: they match |
| 13 Nov | Worker (Opus 5.5) | Description language draft | Overseer reviewed; every example parses; owner approval pending |
| 13 Nov | Two workers (designer; property writer blind to the RTL) | `rosetta_crc` | Overseer ran prove/bmc/cover on the real RTL: PASS; 14/14 planted bugs caught |
| 13 Nov | Overseer | CRC hill-climb step (-10.7 % area) | All proofs rerun on the changed file: PASS |
| 13 Nov | Two workers, same split | `rosetta_stuff` v1 then v2 | Overseer ran prove/bmc/cover: PASS; 17/17 planted bugs caught. Both workers independently found the same two spec flaws |
| 13 Nov | Worker | Local Tiny Tapeout hardening with SRAM macros | Overseer checked metrics.json and the precheck log |
| 13 Nov | Worker | FPGA harness on PYNQ-Z1 | Demonstrated on the board; clock measured against an independent oscillator |
| 13 Nov | Worker | Toolchain (assembler, model, parser) | Overseer ran the 74 tests |
| 13 Nov | Worker | Shared-logic block specs | Overseer review; reconciliation rulings in `docs/decisions.md` |
