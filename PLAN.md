# Plan

Source of truth for what is being built, in what order, and what must be
true before each step counts as done. The overseer keeps it current.

Deadline: **18 January 2027**. Target submission: **17 January**.

## Rules

1. **The solver decides.** Work merges when its proofs pass, its covers are
   reachable, and it synthesizes inside its area and timing budget. An
   agent's report is not evidence.
2. **Designer and prover are different agents.** Properties are written from
   the spec without sight of the RTL.
3. **Every week ends with a chip that could be submitted.** The Tiny Tapeout
   flow runs on every push from Phase 1 on.
4. **Cut from the top of the ladder, never from verification.**
5. **Every use of AI is logged** in `writeup/ai-log.md` as it happens.

## Scope ladder

| Rung | Contents | Status |
|---|---|---|
| 1 | Lanes, sequencer, host port, compiler. UART, SPI, I2C in every role, proven. Clean TT flow at 60 MHz. | in progress: 11 blocks proven; UART and SPI run on the FPGA; full chip does not route yet; compiler not started |
| 2 | Listen with timestamps, Translate, Intercept. Low-speed USB keyboard. Ethernet transmit. | not started |
| 3 | Stress. Ethernet receive and `ping`. Mutation score. | not started |
| 4 | Imitate. | not started |

Rung 1 is the submission floor. Rungs 2-4 are added only while the schedule
holds.

## Phases

### Phase 0: specification (14-18 Nov), 2-3 workers

| Item | Gate |
|---|---|
| Architecture spec (`docs/architecture.md`) | **done**: approved 13 Nov |
| Instruction set (`docs/isa.md`) with an executable model | every instruction class has a worked example |
| Description language (`docs/language.md`), UART / SPI / I2C written in it | approved by the user |
| Interface spec for every block boundary | each signal has an owner and a timing rule |
| Area probe: one lane synthesized on CMOS5L | **done**: 4 lanes; 337,906 um2 for the chip, 36.6 % density; see architecture section 11 |
| Name chosen, repo and top module renamed | **done**: Rosetta, 13 Nov |

### Phase 1: blocks and first light (19 Nov - 6 Dec), 6-8 workers

Blocks, each with a designer, a property writer and a proof runner:
synchronisers and crossbar, tick sources, line coding, stuffing, check unit,
shift and field execution, events, sequencer and slot wheel, host port,
memory wrappers, top level.

| Milestone | Gate | By |
|---|---|---|
| First light | A UART byte leaves a pin from a compiled description: proven, running on the PYNQ-Z1, and hardened through the TT flow with precheck passing | 29 Nov |
| All blocks | Every block's unbounded proofs and covers pass | 6 Dec |
| Compiler v1 | Emits program images, monitors and the schedule check | 6 Dec |

### Phase 2: rung 1 complete (7-20 Dec), 5-7 workers

| Item | Gate |
|---|---|
| UART, SPI, I2C as controller, target and listener | each program proven against its monitor; each running on the FPGA against a real device |
| Reset proof and non-interference proof | pass on the integrated design |
| Timing | 60 MHz met after place and route at the slow corner |
| **Architecture freeze** | 20 Dec: no new hardware features after this date |

### Phase 3: rungs 2 and 3 (21 Dec - 3 Jan), 4-6 workers

| Item | Gate |
|---|---|
| Listen, Translate, Intercept | demonstrated on the FPGA; listener-never-drives proof |
| Low-speed USB keyboard | enumerates on a real PC from the FPGA |
| Ethernet transmit | packets seen in Wireshark from the FPGA |
| Stress; Ethernet receive and `ping` | only if the items above are done by 28 Dec |
| **RTL freeze** | 3 Jan: bug fixes only after this date |

### Phase 4: sign-off (4-11 Jan), 2-3 workers

| Item | Gate |
|---|---|
| RTL equals synthesized netlist | equivalence proof passes |
| Mutation testing | score reported; every surviving mutant explained |
| Gate-level test and TT precheck | pass in GitHub Actions |
| Imitate | only if everything above is done by 8 Jan |

### Phase 5: write-up and submission (12-17 Jan), 1-2 workers

| Item | Gate |
|---|---|
| Write-up, architecture and instruction-set docs, datasheet page | complete and read end to end by the user |
| Demo video from the FPGA | recorded |
| Final GDS and precheck | green on the submitted commit |
| Submission form | sent by the user on 17 Jan |

## User approvals

1. Architecture spec (Phase 0).
2. Description language (Phase 0).
3. Each step up the scope ladder.
4. Repo rename and final submission.
5. The worker count for each phase before it starts.

## Risks

| Risk | Response |
|---|---|
| 60 MHz not met at the slow corner | Pipeline; the Ethernet claim needs 60 MHz, everything else works slower |
| Routing congestion on three metal layers | Area margin is deliberate; stay at 4 lanes |
| SRAM macro integration (power, LVS) | Do it in the first-light milestone, not later |
| Proofs too slow on the integrated design | Prove blocks unbounded; prove integration with small parameters |
| Description language too weak for USB or Ethernet | Rung 1 does not depend on it; write those protocols at the instruction level if needed |
| Pads slower than estimated (no silicon data yet) | Nothing to do before tape-out; note it in the write-up |

## Status log

- 13 Nov: old design archived, repo reset from the TT template (8x4),
  area budget derived, formal tools installed, idea approved, PYNQ-Z1 found.
- 13 Nov: architecture spec approved; named Rosetta; repo renamed; Phase 0 started.
- 13 Nov (overnight): area probe measured; 4 lanes chosen; three lane shrinks and three-cycle instruction pipeline adopted. Conventions and block specs for CRC and stuffing written; 6 more workers started (CRC and stuffing designer/prover pairs, local hardening, FPGA harness).
- 13-14 Nov (overnight): **two blocks proven and merged**: `rosetta_crc` (4,689 um2 after one hill-climb step) and `rosetta_stuff` (spec v2, 552 um2). **Local hardening works** with two SRAM macros at 16.67 ns: DRC/LVS/antenna 0, precheck pass, slow-corner setup slack +3.39 ns (`/home/elfo/tools/tt-local/`). **FPGA harness works** on the PYNQ-Z1 at a measured 60 MHz (`/home/elfo/tools/rosetta-fpga/`). Toolchain: assembler, cycle-accurate model, language parser, 74 tests pass. ISA v0.2, provisional description language, shared-logic block specs drafted. Rulings toward ISA v0.3 in `docs/decisions.md`.
- 14 Nov 23:50-01:20: work paused by the account's usage limit (eight parallel workers exhausted the five-hour allowance). Resumed with fewer workers at a time.
- 14 Nov, 01:20-03:50 and 06:20-07:30 (overnight, continued): **eleven blocks proven and merged**: crc, stuff, chan, xbar, events, coding, pins, field, delay, seq, tick. Each passed prove, bmc and cover on the real RTL, with properties written by a worker who never saw the RTL. One gate failure (tick: sample strobe fired one cycle after a capture instead of four) was a real RTL bug, refereed and fixed. Rulings 1-54 in `docs/decisions.md`.
- 14 Nov: **first light on the PYNQ-Z1**: the whole chip at 60 MHz (WNS +2.78 ns) loaded over the SPI host link; UART transmit exact to the cycle at 115200 baud; UART lane-to-lane receive 18/18 bytes, tolerant from +5.0 % to -4.6 %; timestamped listener; SPI controller and target between two lanes at 1-3 MHz. Results in `/home/elfo/tools/rosetta-fpga/results/`.
- 14 Nov: **one lane placed and routed at 16.67 ns**: slow-corner setup slack +6.09 ns.
- 14 Nov: **the full chip does NOT route yet** (8x4, two SRAMs): placement and pre-route timing are fine (slow-corner setup +4.28 ns) but detailed routing stalls at about 26,000 violations; two thirds of the congestion is nets crossing the SRAM bodies. Synthesis in the real flow is 483,048 um2, 25 % above the Yosys estimate. This is now the top risk.
- Not yet through their gates (on branch `wip/unproven`): host port (prove and bmc pass; cover run unfinished), memory wrappers, mode-register file, lane top, chip top. Not written: two-copy non-interference proof, reset proof at chip level, netlist equivalence, the compiler (language to program), protocol monitors.
- Usage-limit pauses: 23:50-01:20 and 03:50-06:20.
- 14 Nov: **description language approved by the user** (core syntax, host view, direction of the USB/Ethernet extensions). Host port cover run finished: 44 of 44 covers, PASS (not yet merged).
- 14 Nov: **where the extra area comes from**: logic alone is 388,186 um2; the flow's default synthesis (`DELAY 4`) adds 21k for speed-oriented mapping and 75k for 9,164 fanout buffers (483,048); post-placement repair adds 19k, the clock tree 12k, hold fixing 57k (3,506 buffers on 3,199 endpoints), giving 571,115 um2 at routing. Buffers are about 30 % of it. Test run with `SYNTH_STRATEGY: AREA 0` in `/home/elfo/tools/tt-local/rosetta-full-chip-area0`.
- 14 Nov: **`rosetta_host` proven and merged** (12th block): prove, bmc and cover (44 of 44) pass on the real RTL, md5 efb589ff8fdfec72e625c794711f0091. Merge approved by the user.
