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
| 14 Nov | Worker pairs (designer; property writer blind to the RTL), one pair per block | `rosetta_chan`, `xbar`, `events`, `coding`, `pins`, `field`, `delay`, `seq`, `tick` | Overseer ran prove/bmc/cover on each real RTL file and checked the proven file's checksum against `src/` |
| 14 Nov | Two architect workers, then a cold reviewer | Cycle-exact block specs for the lane and the shared logic | Reviewer found 11 cross-spec errors; designers and property writers found more; overseer ruled on each (rulings 18-54) |
| 14 Nov | Referee worker | Tick gate failure | Reproduced the counterexample, decided from the spec text that the RTL was wrong, fixed one term; overseer reran the proofs |
| 14 Nov | Worker | Lane and chip top-level integration, FPGA first light | Overseer read the board result file; capture decoded independently of the chip |
| 14 Nov | Worker | FPGA loopback: UART receive, listener, rate sweep, SPI | Each checked twice: pin capture decoded independently, and data read back over SPI. Found an assembler encoding bug, fixed by the overseer |
| 14 Nov | Worker | Lane timing by place and route; full-chip hardening | Overseer checked the lane metrics file; full chip did not route, congestion map kept |
