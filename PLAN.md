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
- 14 Nov: **the full four-lane chip hardens end to end at 60 MHz in 8x4** (local flow, run `br-p7b`: routing 0, KLayout DRC 0, LVS 0, antenna 0; setup slack slow/typ/fast +2.36/+6.49/+8.65 ns; hold +0.37/+0.20/+0.09 ns; 450,470 um2 of cells). Best recipe so far `br-p5cb`: slow-corner setup +4.38 ns, 67 slew violations. Not yet run on these: TT precheck, gate-level test. The recipe, none of which changes RTL:
  1. `SYNTH_STRATEGY: AREA 0` (the default `DELAY 4` adds 9,164 fanout buffers and does not route even on the good floorplan).
  2. Both SRAMs along the bottom edge at the right end, pins up (`FS` at x = 812.64 and 1262.24, y = 3.78). All 43 chip pins are in the top-left corner; a memory on the left blocks the single horizontal layer (Metal3) between pins and lanes. Global-routing overflow: stacked-left 5,639, this floorplan 383.
  3. `FP_OBSTRUCTIONS` on the 33 um gap between the memories and the 45 um strip right of the data memory. A cell placed in the gap gets a global route straight across the program memory (one net, 40-180 shorts, in five separate runs).
  4. `LAYERS_RC` calibrated to extraction: the tech LEF gives 0.097 fF/um, RCX measures 0.173 fF/um on the routed layout. Used 0.175 (0.233 so the placement-stage average matches).
  5. Place-and-route clock target 5-7 ns via `PNR_SDC_FILE` (sign-off stays 16.67 ns) and `DESIGN_REPAIR_MAX_SLEW_PCT`/`MAX_CAP_PCT` 40. The resizer only tunes the typical corner; slow is about 1.6x slower.
  Rejected: post-global-route repair steps (`RUN_POST_GRT_*`; they close timing but fail on the memory-gap net), `RSZ_CORNERS` (no effect), dropping to three lanes (not needed).
- 14 Nov: **open on hardening**: all passing runs use hold clock uncertainty 0.1 ns instead of the default 0.25 (experiment approved by the user; fast-corner hold slack only +0.07 to +0.09 ns). Runs `br-p5cb-h25` and `br-p6cb-h25` test the default. Runs live in `/home/elfo/tools/tt-local/rosetta-full-chip-*`.
- 14 Nov: **hold margin question closed: the default 0.25 ns is used.** Candidate `br-cand4` passes the whole local flow and the Tiny Tapeout precheck (all 10 checks) with the default clock uncertainty: routing 0, KLayout DRC 0, LVS 0, antenna 0; setup slack slow/typ +2.84/+7.65 ns; hold slack fast/typ +0.08/+0.26 ns; 503,592 um2 of cells. Two more recipe items beyond the five above: (6) `PL_RESIZER_ALLOW_SETUP_VIOS: true`, because with the tightened place-and-route clock the hold fixer otherwise refuses to touch paths that only look setup-critical (self-loop flops then miss hold by 0.05 ns); (7) `SIGNOFF_SDC_FILE` must be set explicitly to the real 16.67 ns constraints, or sign-off silently reuses the tightened ones. Routability limit on this floorplan: about 525,000 um2 of cells routes, 543,000 does not. The exact flow settings of `br-cand4` are saved in `hardening/` (not yet wired into `src/config.json`, which still builds the placeholder). Not yet done on this layout: gate-level test; and the lane top, memory wrappers, mode-register file and chip top inside it are still unproven.
- 14 Nov: **open, to clean up later (user: keep an eye on it): overloaded nets in `br-cand4`** at the slow corner, about 100 slew violations of two kinds: (a) SRAM input pins (address and data-in) see edges of 1.5-2.2 ns against the macro's 0.595 ns limit, so the macro's setup and hold figures are being extrapolated there; (b) about 80 standard-cell pins at 3.2-4.0 ns against a 2.5 ns limit. Neither breaks sign-off timing. (a) matters more; likely fix is a strong buffer next to each memory input, in the flow or as explicit RTL buffers in the memory wrappers.
- 14 Nov: workers started: property writers for the memory wrappers and the mode-register file (from specs only), and the compiler's first milestone (UART, all roles, checked against the hardware-proven hand-written programs and on the RTL).
- 14 Nov: **`rosetta_cfg` proven and merged** (13th block): prove (k-induction, also abc pdr), bmc and cover pass on the real RTL, md5 d1bd0c5e1cb6d98bb96c490ba9aa4b49; 24 of 24 planted bugs killed on the property writer's reference model. Documentation debts it found, none affecting this block: `docs/isa.md` still shows the pre-ruling-10 event-register layout; `isa.md` and `toolchain/rosetta/isa.py` still define THR1/THR2, which ruling 23 deleted (the assembler can encode values the hardware drops); `isa.py` lacks the B-must "never" and trigger "off" codes of ruling 21; "SETM high halves" is still listed as unruled in `decisions.md` (the block follows lane.md 5.1).
- 14 Nov: **compiler milestone 1 (UART) done**: `toolchain/rosetta/compiler.py` compiles the approved language to lane programs for sender, receiver and listener with parity none/even/odd. At 115200 baud the output equals the hand-written FPGA-proven programs word for word apart from documented extra `SETM`s and tags. On the real RTL in Verilator all roles pass, including the parity and framing error paths; bit period exactly 520 cycles; listener timestamps exact to the cycle. 89 toolchain tests pass. No RTL/spec discrepancy found. 14 interpretations of the language are listed in `compiler.INTERPRETATIONS`. Observed limits: parity frames are 10 cycles longer than 11 bits (`CRCINIT` waits for idle); receiver tops out near 940 kBd, listener near 630 kBd. Next: SPI, then I2C.
- 14 Nov: **`rosetta_mem_prog` and `rosetta_mem_data` proven and merged** (blocks 14 and 15, with the thin macro wrapper `rosetta_sram512x32`): prove, bmc and cover pass on the real RTL for both (md5 0d6e7b6a... and 83927f7c...). The unbounded proofs check every macro pin bit-exactly in every cycle through a probe that stands in for the SRAM macro, plus the tracked-word model of mem.md 4.2; 12 of 12 and 28 of 28 planted bugs killed. Spec notes from the property writer: mem.md 3.6 items 7 and 8 are stated too loosely (other writers, QCFG changes) and were proven in a precise form; the behavioural SRAM model masks per byte where the macro masks per bit (they agree for every mask the wrappers drive, which the proof checks bit-exactly); mem.md and top.md give the macro path without the `u_sram` level (the hardening config already uses the real path).
- 14 Nov: **compiler milestone 2 (SPI) done**: controller and target in modes 0-3 from the description of language.md 2.2; listener with one lane per data wire (gap G5). On the RTL all four modes pass at 1 MHz and at the fastest accepted rate (1.5 MHz modes 0/2, 2 MHz modes 1/3), bytes both ways, chip-select rules met on the pins, and the compiled programs interoperate with the FPGA-proven hand-written ones. Fastest rates: controller 6 MHz, target 1.5-2 MHz, listener 2-4 MHz. 100 toolchain tests pass; UART output unchanged. `isa.py` brought in line with rulings 21 and 23. No RTL/spec discrepancy. Interpretations 15-24 added. Two hardware limits noted for a later decision: a target that prepares words ahead loses them when deselected early (pop happens at park time); `cut FIELD` records cannot be produced (no way to read the bit count of an abandoned word). Next: I2C.
- 14 Nov: **compiler milestone 3 (I2C) done; rung 1 of the compiler is complete (UART, SPI, I2C in every role).** Controller (66 words), target (36) and listener (36) from the description of language.md 2.3. On the RTL, one open-drain bus with a controller, two targets and a listener passes multi-byte write, read, write + repeated START + read, an unacknowledged address (controller stops, records the error, halts) and target clock stretching, at 100 kHz, 400 kHz and the fastest accepted link rate 526 kHz; every timing rule of the description is met on the pins; no bus pin is ever driven high; the listener never enables an output. 105 toolchain tests pass; UART and SPI output unchanged. No RTL/spec discrepancy. Interpretations 25-34 added. Limits found (spec-level, for later decisions): the controller changes SDA in the same cycle SCL falls (zero hold margin; observers rely on the 18-cycle confirm window, shown essential by a control run); a target can stretch only while waiting for host data, not while executing or when its out-queue is full; a listener cannot stream 400 kHz I2C reads through the host port (64-word out-queue fills); START/STOP waits are whole low phases (valid but slow); I2C has still never run on the FPGA.
- 14 Nov: usage limit hit at about 21:00; lane-top and chip-top property writers interrupted and resumed at 21:41.
- 14 Nov: **`rosetta_lane` proven and merged** (16th block; the lane top over its nine proven sub-blocks): seven tasks pass on the real RTL (md5 ff379145...): `prove` (abc pdr, 213 assertions unbounded on the full composition, no sub-block abstracted), `prove_hard` (k-induction, busy/tag group, assume-guarantee against `prove`), `prove_r53` (ruling 53 with a pin model), `bmc`, `bmc_gold` (every output equals a spec model, depth 16), `bmc_bus`, `cover`. 49 of 49 planted integration bugs killed on the writer's reference. Overseer's own non-vacuity check on the real RTL: swapping `lt`/`ct` into the field unit and reversing the event write pulses each fail `prove` at the expected assertion. Findings: ruling 53 insufficient (new ruling 55), four lane.md corrections (ruling 56). **Open hazard to examine** (not confirmed): with a delayed clock, a `PIN` on `out3` in the cycle the BAUD timer makes its delayed end-of-bit request may be overridden later (cover `c_haz_pin_dlyclk` reachable; pins.md section 3 says no stale delayed change lands after a PIN). Bounded only: the shared-bus two-copy check (depth 12) and DRIVE result masking (depth 20); both are unbounded by composition with the field and cfg proofs.
- 14 Nov: **chip top `tt_um_jesselevine727_rosetta` proven and merged (17th and last block); every RTL file is now on `main` behind a proof.** 19 jobs pass on the real top (md5 b15617c2...): wiring of every bus, pin and reset with all components as free shells (`top_wire` prove/bmc/cover); structural checks on the complete flattened chip, no flop without async reset and no combinational input-to-output path (`top_struct`); reset values and quiescence, pin mapping, synchroniser depths, host slot timing with real host/xbar/sequencer/channels/memory wrappers and free lanes (`top_chip`, abc pdr); a listener never enables an output (`top_t9`); **lane non-interference, two copies of the chip, unbounded, for each of the four lanes** under the eight assumptions of top.md (`top_ni`, k-induction depth 12); bring-up cover on real lanes and full SRAM models (program word loaded, lane started, pin driven, 15 steps). 38 of 38 planted bugs killed on the writer's reference; overseer's non-vacuity check on the real top: a rotated lane run bit and an inverted `uio_oe` each fail at the expected assertions.
- 14 Nov: **open proof obligations**: (1) T2 reset determinism (two copies reach the same state) is bounded to depth 12 only; induction fails on an unreachable state of the sequencer's D/X pipeline register and needs a sequencer lemma or an observation port. (2) T10 instruction equivalence is by composition only. (3) Bring-up through real SPI frames on the complete chip timed out at 30 min (first two frames served). (4) RTL-versus-netlist equivalence not started. (5) Tick unbounded proof covers PERIOD 0-7 at block level (the lane proof needs no such range). (6) The PIN/delayed-clock hazard above.
- 15 Nov: **compiler enforces ruling 55** (post-assembly check, both clock-wire paths, one test per condition). It caught a real violation: the compiled I2C target's idle `PIN` wrote `out3` under `stretch_hold`; fixed by leaving the clock output's idle level to the `TICK` write (one word changed; everything else word-identical). **I2C with 300 ns hold** (`programs/i2c/i2c_hold.ros`): launch delay 18 cycles (controller) and 14 (target), measured hold 18 cycles at 100 kHz, 400 kHz and 526 kHz, all rules met, maximum rate unchanged; with the hold, a skewed target stays correct even with the confirm window set to 0, so the hold removes the dependence on the window. 111 toolchain tests pass. Interpretations 35 and 36.
- 15 Nov: **the PIN / delayed-clock hazard is real (ruling 57)**: a spec-composition error, no block is buggy. Reachable only with `delay_clock` set, which the compiler never does; hand-written programs could hit it (an I2C-style controller releasing SCL by `PIN` would have it pulled low again D cycles later). Neighbouring cases proven safe unbounded. Compiler rule in force; RTL fix in `rosetta_delay` proposed, awaiting the owner. Ruling 56 wording corrected (`crcw_q` is gated by `run_i`).
- 15 Nov: **the repo now builds the real chip.** `info.yaml` lists the 19 real sources (placeholder `project.v` removed); `src/config.json` is the hardening recipe (`br-cand4`) with `src/pnr_oc.sdc`, `src/signoff.sdc` (both source the flow's base constraints, no hard-coded tool path) and `src/pdn_cfg.tcl`; the two SRAM flow plug-ins are at the repo root; `hardening/` removed. `test/` is a minimal bring-up test (reset quiet; host writes and reads back both memories; host starts lane 0 and it drives `uo[2]`), behavioural SRAM for RTL, PDK SRAM models for gate level. **Full local flow from the repo at the merged RTL: harden exit 0; routing, KLayout DRC, LVS, antenna 0; setup slack slow/typ/fast +2.84/+7.65/+9.32 ns; hold +0.56/+0.26/+0.08 ns; precheck passed (10 of 10); gate-level test 3 of 3 on the final netlist.** Identical timing to `br-cand4`, as expected since no RTL changed. GitHub's own build is the remaining check.
- 15 Nov: **hardware fix for the PIN / delayed-clock hazard merged (ruling 58, owner's decision).** `rosetta_delay` now drops a delayed request made in the cycle a `PIN` writes the same output (one added term; tick.md 9.2 rule 4 amended). Property side updated from the amended spec only: delay proof with 13 of 13 planted bugs killed (6 new, including the old no-drop behaviour); the hazard properties `h_pend3` and `h_keep3` now hold; new lane properties show a capture request is never dropped. Overseer's gate on the real RTL (md5 2c331f7d...): delay prove/bmc/cover, all seven lane tasks, `haz_prove` including out3, `top_struct`, `top_wire`, `top_bringup core`, all PASS; the same proofs against the previous delay unit FAIL (`a_p4_cancel`, `h_pend3`), so they have teeth. 111 toolchain tests pass on the new RTL. Layout to be rebuilt.
