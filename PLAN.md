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
| 1 | Lanes, sequencer, host port, compiler. UART, SPI, I2C in every role, proven. Clean TT flow at 60 MHz. | not started |
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
