# Amendment: Manchester phase tracking (H1), edge-terminated `RAW DRIVE` (H3), `RECOVER` options (H4)

Status (22 Nov): **H3 and H4 are implemented, proven and merged** (with the timer cancel of section 2.5, ruling 60). **H1 is ON HOLD**: its measured tolerance (section 1.11 and `study/protocols/h1/`) does not meet the 802.3 receiver requirement, so the owner's approval of H1, given on the model's figures, is being re-confirmed. Originally written as a proposal for review. Amends `tick.md`, `field.md`,
`lane.md` 5.1/5.2/6 and isa.md section 4; adds one module,
`rosetta_mdpll` (spec in section 1). Notation as `lane.md` 1 (`pt`,
`a (+) D`, `G = run_i && !abort_i`). Evidence: `study/protocols/REPORT.md`
and `study/protocols/h1/` (RTL prototype, measurements, synthesis).

**Zero rule.** Every change is enabled by a mode-register bit that resets to
0. With all new bits at 0 the lane is cycle-for-cycle today's lane: every
existing property holds unchanged under the antecedent "the new bits are 0"
(section 5 lists which properties need that antecedent written in), and no
compiled program changes, because the compiler and `isa.mode_encode` never
set these bits (they are not fields today, so no compiled immediate has them).

## 0. New mode bits

| Bit | Name | Change | Stored in | Reset | Meaning when 1 |
|---|---|---|---|---|---|
| `TICK[13]` | `mdpll` | H1 | lanes built with `H1 = 1` only (otherwise not stored, reads 0) | 0 | in `RECOVER`, the capture strobe comes from the Manchester phase tracker (section 1) |
| `TICK[14]` | `rone` | H4 | all lanes | 0 | in `RECOVER`, only data edges to level `!TICK[6]` restart the timer (TICK[6] = 1: falling edges) |
| `TICK[15]` | `rsp2` | H4 | all lanes | 0 | in `RECOVER`, capture `PERIOD2` prescaled cycles after the launch tick instead of `PERIOD/2` |
| `CODE[12]` | `rawedge` | H3 | all lanes | 0 | a `RAW DRIVE` also ends at the first data-input edge in or after its effect cycle |

Free today: `lane.md` 5.1 stores `TICK[12:0]` (13 flops) and `CODE[11:0]`
(12 flops); `toolchain/rosetta/isa.py` defines `TICK` fields up to bit 12
(`stretch_hold`) and `CODE` fields up to bit 11 (`delay_prescaled`); bits
13-15 of `TICK` and 12-15 of `CODE` are "not stored, read as 0". `TICK[6]`
(capture on falling edge) is not used by any `RECOVER` rule today (CL/LL are
used only in `tick.md` 4.1 BAUD and 4.2 EDGE), so H4 may give it a meaning in
`RECOVER`.

New flops: 3 mode bits in every lane (`TICK[15:14]`, `CODE[12]`), plus in an
H1 lane `TICK[13]` and the 25 state bits of `rosetta_mdpll` (26 flops after
synthesis of the prototype).

`lane.md` 5.1 table rows become: `TICK` stored bits `[15:0]` (`[13]` only
with `H1 = 1`), 16 flops (15 without H1); `CODE` stored bits `[12:0]`, 13 flops.
5.2 write pulses unchanged.

## 1. H1: Manchester phase tracking (`rosetta_mdpll`, lane parameter `H1`)

### 1.1 Purpose and what it buys

The `PULSE` Manchester decoder classifies each interval against `THR0`; its
reference is the previous edge, so duty-cycle distortion and per-edge jitter
count twice. H1 predicts each mid-bit instant from a tracked phase and
separate rising/falling offsets and accepts an edge as mid-bit when it falls
within a window of the prediction. Measured on the RTL prototype (study,
10BASE-T frames with random phase and +-100 ppm): duty-cycle distortion
tolerated 29.5 ns (present decoder 16.5), alternating per-edge jitter 16.5 ns
(8.2), independent uniform jitter about 12-13.5 ns (9); data-dependent jitter
with long runs of equal bits about 8 ns, the same as or worse than the present
decoder (16.5 for cable-type ISI). It therefore meets the 802.3 +-13.5 ns
receiver figure for duty-cycle and alternating patterns and nearly for random
jitter, and does **not** meet it for data-dependent patterns (section 1.11).
Bit time is fixed at 6 prescaled cycles (10 Mbit/s at 60 MHz, prescale 1;
1.25 Mbit/s at prescale 8; 156.25 kbit/s at 64; 19.53 kbit/s at 512).

### 1.2 Lane parameter

`rosetta_lane #(parameter int H1 = 1)`; `rosetta_top #(parameter logic [N_LANES-1:0] H1_LANES = '1)`
passes `H1_LANES[L]` to lane L. `rosetta_cfg` and `rosetta_tick` take the
same parameter.

- `H1 = 0`: `TICK[13]` is not stored (reads 0), `rosetta_mdpll` is not
  instantiated (generate), and `h1on` (1.3) is the constant 0. The lane is
  today's lane plus the H3/H4 bits of sections 2-3 (if a lane exactly equal
  to today's is wanted, H3 and H4 need parameters of their own).
- `H1 = 1`: as below. With `TICK[13] = 0` the lane behaves exactly as with
  `H1 = 0` at its ports.

### 1.3 Enable

`h1on = H1 && TICK[1:0] == RECOVER && TICK[13]`. It changes only at a `TICK`
write (the sequencer writes registers 0-9 only to an idle lane, `lane.md`
2.1). (Ruling 59 names the enable "`CODE.rx` = Manchester under `RECOVER`";
see 7.9 for why a new bit is used.)

### 1.4 Module `rosetta_mdpll` (instantiated in `rosetta_tick`)

| Port | Width | Meaning |
|---|---|---|
| `clk`, `rst_n` | 1 | clock; async active-low reset |
| `run_i`, `abort_i`, `wr_tick_i` | 1 each | as `tick.md` 1 |
| `cfg_en_i` | 1 | `h1on` |
| `pt_i` | 1 | prescale strobe (`tick.md` 3.1) |
| `dline_i`, `dedge_i` | 1 each | cleaned data level (before invert) and its edge (`coding.md` 3.1) |
| `st_o` | 1 | an accepted mid-bit edge while locked: the capture/sample strobe |
| `lock_o`, `acq_o` | 1 each | observation (proofs, covers): state is LOCK / ACQ |
| `ph_o` [9:0], `o_o` [7:0] | | observation only: the phase and offset registers |

### 1.5 Units, constants, state

Unit `u` = 1/32 prescaled cycle. P = 6 prescaled cycles = 192 u.

| Constant | Value (u) | Meaning |
|---|---|---|
| `PQ` | 192 | bit time |
| `W_ACQ` | 72 | acquisition window, 3P/8 |
| `W_TRK` | 48 | tracking window, P/4 |
| `A_LO` | 120 | shortest acquisition interval, 5P/8 |
| `T_LATE` | 96 | P/2 past the prediction |
| `INC` | `pt_i ? 32 : 0` | one prescaled cycle |

| State | Width | Reset | Meaning |
|---|---|---|---|
| `s` | 2 | UNLOCK (0) | 0 UNLOCK, 1 ACQ, 2 LOCK (3 never reached; treat as UNLOCK) |
| `ph` | 10, signed | 0 | time since the estimated last mid-bit instant (UNLOCK: since the last edge, held once >= `A_LO`) |
| `o` | 8, signed | 0 | duty-cycle offset: rising mid-bit edges are expected at `PQ - o`, falling at `PQ + o` |
| `n` | 4 | 0 | accepted edges since acquisition, saturating at 8 |
| `miss` | 1 | 0 | the last bit time passed without a mid-bit edge |

All arithmetic on `ph` and `o` is two's complement in their widths; the
values reached stay within -177 .. 319 for `ph` (UNLOCK: 0 .. 152) and
-128 .. 127 for `o`, and every intermediate value of 1.6 (`ph_inc`, `base`,
`err`) stays within -464 .. 254, so no operation wraps (a property, 1.9 P5).

### 1.6 Behaviour per cycle

Clear: at the end of a cycle with `!run_i || wr_tick_i || !cfg_en_i`, all
state takes its reset value. Otherwise, with these combinational terms of
cycle t:

```
ph_inc = ph + INC
base   = ph_inc - PQ
err    = dline_i ? base + o : base - o            (o sign-extended to 10 bits)
win    = (s == ACQ) ? W_ACQ : W_TRK
acc    = -win <= err <= win
late   = base >= T_LATE
long   = ph >= A_LO
epos   = err > 0 ;  eneg = err < 0
```

the first matching row gives the next state (unlisted registers keep their
value):

| # | Condition | Next state |
|---|---|---|
| 1 | `s == UNLOCK`, `dedge_i`, `long` | `s = ACQ`, `ph = 0`, `o = 0`, `n = 0`, `miss = 0` |
| 2 | `s == UNLOCK`, `dedge_i`, `!long` | `ph = 0` |
| 3 | `s == UNLOCK`, `!dedge_i`, `long` | (`ph` holds) |
| 4 | `s == UNLOCK`, otherwise | `ph = ph_inc` |
| 5 | `s == ACQ`, `dedge_i`, `acc` | `ph = base >>> 1` (arithmetic shift, rounds toward minus infinity); `n = n + 1`; `miss = 0`; if `n + 1 == 8`: `s = LOCK` |
| 6 | `s == LOCK`, `dedge_i`, `acc` | `ph = epos ? base - 1 : eneg ? base + 1 : base`; `o = o + 1` if `(dline_i ? eneg : epos) && o != 127`, `o = o - 1` if `(dline_i ? epos : eneg) && o != -128`, else `o` holds; `n = min(n + 1, 8)`; `miss = 0` |
| 7 | `s` is ACQ or LOCK, not row 5/6, `late` | `ph = base`; `miss = 1`; `s = UNLOCK` if `s == ACQ` or `miss == 1` |
| 8 | `s` is ACQ or LOCK, otherwise | `ph = ph_inc` |

(In ACQ `o` is 0, so `err = base` and row 5 is `ph = base - err/2`.)

Output (combinational, like `RECOVER`'s strobes):
`st_o = G && cfg_en_i && s == LOCK && dedge_i && acc` (row 6 taken).
`lock_o = s == LOCK`, `acq_o = s == ACQ`.

Meaning. Row 1: in the preamble every edge is a mid-bit edge one bit after
the previous one, so an edge after an interval of at least 5P/8 starts
acquisition with this edge as the phase reference (an edge after idle, which
may be a boundary edge, is rejected by row 7 one bit later). Rows 5-6: an edge
within the window of the prediction is a mid-bit edge; the next prediction is
P after the corrected estimate (gain 1/2 while acquiring, then one unit toward
the error, i.e. at most 1/32 cycle per bit, which tracks +-200 ppm with a
factor of 25 to spare). Other edges (boundary edges, noise) change nothing.
Row 7: no mid-bit edge by half a bit past the prediction: one bit is lost
(no strobe); two in a row, or one during acquisition, unlock. Bits are given
out only in LOCK, after 8 accepted edges: a link pulse (two edges) or the
edge at the end of TP_IDL can never produce a bit.

### 1.7 Integration in `rosetta_tick` (amends `tick.md` 4.3 and 4.4)

New input `cfg_h1_i` (`TICK[13]`); parameter `H1`. `rosetta_mdpll` gets
`cfg_en_i = h1on`, `dline_i`, `dedge_i` (already tick inputs; `dline_i` was
unused), `pt`, `run_i`, `abort_i`, `wr_tick_i`.

When `h1on`: `lt = 0`; `ct = st = cte = h1_st`; `hte = 0`. Everything else
in `tick.md` is unchanged, including the `RECOVER` counter `cnt`, which keeps
running by 4.3 and is not used. When `!h1on`: 4.3 as amended by H4 (section 3).

Bit value: the field takes `rxb`/`rxv` from `rosetta_coding` as today. In
`RECOVER`, `CODE.rx` = 0 (level) or 2 (Manchester) both decode as level
(`coding.md` 3.2), so `rxb = lvl` = the logical level after the mid-bit edge
= the 802.3 bit; `rxv = ok`. `CODE.rx` = 1 (NRZI) or 3 is a programming error
with H1 (compiler rule).

### 1.8 Interactions

| Feature | Behaviour with `h1on` |
|---|---|
| abort | strobe gated (`G`); `rosetta_mdpll` state is not affected (like the `RECOVER` timer and the edge-interval counter, `lane.md` 7.4) |
| halt | state cleared (run_i = 0) |
| `SETM TICK` | clears the state (`wr_tick_i`); enabling H1 always starts in UNLOCK |
| `SETM CODE` | not a clear. A change of data input or invert may give one spurious edge while UNLOCK or ACQ; no bit can result before 8 accepted edges. Programs write `CODE` before `TICK` when enabling H1 |
| prescale | `ph` advances by 32 u per prescale strobe; edges are taken in every cycle |
| differential input | an invalid pair makes no edge (`coding.md` 3.1) and so no strobe; ruling 20/46 hold |
| `SAMPLE`/`EXPECT`, seek, CRC, receive de-stuffing | unchanged: one bit per `st` with `rxv` (`field.md` 5.3); `srx` applies (H1 is not BAUD-timed) |
| `DRIVE`, `EXCHANGE`, `PIN` with `align` | never launch / never take effect (`lt = 0`) until abort or halt. Compiler rule: not allowed while `h1on` |
| `RAW SAMPLE`, events, edge-interval counter | unchanged (they use `dedge` and the raw inputs) |
| launch delay, clock output, stretching (rulings 42, 53, 55) | not involved: no transmit timing and no clock requests under `RECOVER` except the TICK-write idle request (unchanged) |
| ruling 50 (`lt` and `ct` never together) | holds: `lt = 0` |
| field boundaries | strobes are at least 1 cycle apart; a field parked behind a receiving field is current one cycle after its completion (two with source `CRC`/`SR`), so a following field never misses a strobe unless it is `pk_srcdep` and the next strobe comes 1 cycle later (10BASE-T: mid-bit edges are 6 +- 2 cycles apart) |

### 1.9 Required properties

New proof `formal/mdpll/` (block; inputs free except as stated):

1. **P1 clear**: after a cycle with `!run_i || wr_tick_i || !cfg_en_i`, all
   observation outputs and `s` are 0; `st_o = 0` whenever `!cfg_en_i` or `!G`.
2. **P2 rules**: `s`, `ph`, `o`, `n`, `miss` follow table 1.6 exactly (a
   reference model in the checker; observation ports `ph_o`, `o_o`, `lock_o`,
   `acq_o` plus a shadow of `n`, `miss`).
3. **P3 strobe**: `st_o` iff `G && cfg_en_i && lock_o && dedge_i && acc`
   (acc computed from `ph_o`, `o_o`, `dline_i`, `pt_i` by 1.6).
4. **P4 no early bits**: `st_o` only after at least 8 accepted edges since the
   last entry to ACQ; in particular a run of fewer than 10 edges after a clear
   never produces `st_o`.
5. **P5 ranges**: `-177 <= ph <= 319`, and no addition in 1.6 overflows its
   width (10 bits signed).
6. **P6 round trip** (the jitter bound that can be proven). Checker-side
   encoder: prescale 1 (`pt_i = 1`); a free bit per 6-cycle cell; ideal
   Manchester level `w` (first half complement, second half the bit; idle
   high before the first cell); the line `dline_i` is `w` with every rising
   transition delayed by `1 + dr` and every falling transition by `1 + df`
   cycles, `dr`, `df` free constants in {-1, 0, 1} (realised as
   `dline = (dr > df) ? w(t-1-dr) & w(t-1-df) : (dr < df) ? w(t-1-dr) | w(t-1-df) : w(t-1-dr)`),
   `dedge_i = dline_i != dline_i(t-1)`; assumption: the first 12 cells after
   the clear carry alternating bits (a preamble). Claims:
   (a) every `st_o` falls on the displaced mid-bit transition of the current
   cell (cell offset 3, 4 or 5), never on a boundary transition, and
   `dline_i` then equals the cell's bit;
   (b) after the first `st_o`, every later displaced mid-bit transition has
   `st_o` (no bit is lost or inserted);
   (c) cover: `st_o` within 12 cells.
   This is a duty-cycle bound of up to 2 cycles (33 ns) between rising and
   falling edges with a constant phase. Checked by simulation on the
   prototype for all 9 (dr, df), offsets 0-5 and 20-64 preamble bits:
   360/360 frames exact. **Not provable, and false**: arbitrary per-edge
   jitter of +-1 cycle (an adversary can walk the estimate one unit per bit
   and then step), and any displacement whose sign pattern persists over
   many bits (data-dependent jitter): the window (+-1.5 cycles) is then
   exceeded at the step, which is the measured 8 ns limit.

Tick proof (`formal/tick`): group 11 (`a_p11_*`) gains the antecedent
`!h1on` (with H1 = 0 it is unchanged). New group 11h, with `h1on`: `lt_o = 0`,
`hte_o = 0`, `ct_o = st_o = cte_o =` the module's `st_o` (instance-level or by
a black-box abstraction of `rosetta_mdpll` with P3 as its contract).

Lane proof: listener property 12 unchanged (H1 adds no enable); property 14
(no combinational path except from `run_i`) unchanged; new: with `h1on`, the
field takes one bit per `st` (composition of field P2/P3 with tick 11h). Lane
equivalence (eqy or a miter in sby): a lane built with `H1 = 1` whose `TICK`
writes have `cfg_data_i[13] = 0` equals the lane built with `H1 = 0`, port for
port, every cycle.

Covers: acquisition from idle; lock; a strobe per polarity; a boundary edge
rejected in LOCK; a miss (row 7) without unlock; unlock after two misses;
`o` saturating; abort while locked (state kept, strobe suppressed); a link
pulse (two edges) producing no strobe; TICK write while locked.

### 1.10 Worked timing (reference rules; identical to the RTL prototype)

Prescale 1, preamble 1010..., rising edges ideal, falling edges 1 cycle late
(`df = 1`). `ph`, `err` in u (32 u = 1 cycle). Line idle high; the first edge
(t = 21) is the falling edge that starts bit 0's first half.

| t | edge | `s` before | `ph` | `err` | row | `s` after, `ph` next, `o` next | `st_o` |
|---|---|---|---|---|---|---|---|
| 21 | fall | UNLOCK | 128 (held >= 120) | | 1 | ACQ, 0, 0 | |
| 23 | rise | ACQ | 32 | -128 | 8 (outside 72) | ACQ, 64 | |
| 30 | fall | ACQ | 256 | 96 | 7 (outside 72, and base 96 >= 96) | UNLOCK, 96 | |
| 35 | rise | UNLOCK | 128 | | 1 | ACQ, 0, 0 | |
| 42 | fall | ACQ | 192 | 32 | 5 | ACQ, 16 | |
| 47 | rise | ACQ | 144 | -16 | 5 | ACQ, -8 | |
| ... | | | | | 5 | | |
| 83 | rise | ACQ | 138 | -22 | 5 (n reaches 8) | LOCK, -11 | |
| 90 | fall | LOCK | 181 | 21 | 6 | LOCK, 20, o = 1 | **1** (bit value 0) |
| 95 | rise | LOCK | 148 | -11 | 6 | LOCK, -11, o = 2 | **1** (bit value 1) |
| 204 | fall | LOCK | 178 | 3 | 6 | LOCK, 17, o = 16 | 1 (bit value 0) |

`o` converges to 16 u (half the 1-cycle difference between falling and
rising edges) one unit per accepted edge; `err` shrinks to +-1 u. A boundary
edge in LOCK (e.g. t = 195 in the run "11": `err = -93`) changes nothing.

### 1.11 Limits (state them to users)

- Data-dependent jitter whose sign persists over runs of equal bits (cable
  ISI, transmitter pre-emphasis) is tolerated to about 8 ns per crossing;
  this is a property of any decoder that takes its reference from the data
  edges at 16.7 ns sampling (a step of 2A must stay inside the +-25 ns window
  less half a sample).
- Random jitter: about 12-13.5 ns (uniform, per crossing); Gaussian sigma
  about 4 ns.
- Acquisition: about 27 preamble+SFD bits clean (a receiver may lose 37 of
  the 64), more under heavy duty-cycle distortion.
- Size (prototype `study/protocols/h1/rosetta_mdpll_min2.sv`, PFIX = 6,
  PI = 3, F = 5, Yosys + CMOS5L typical): 4,833 um2, 417 cells, 26 flops;
  longest flop-to-flop logic 3.5 ns typical, 5.7 ns slow corner. A
  programmable bit time (`P = PERIOD[3:0]`, 4-15 prescaled cycles) is the same
  rules with `PQ = PERIOD[3:0] * 32` and the constants derived by shifts:
  5,975 um2.

## 2. H3: edge-terminated `RAW DRIVE`

### 2.1 Behaviour (amends `field.md` 6.1 and 7.2)

New `rosetta_field` input `cfg_rawedge_i` (`CODE[12]`). Effect cycle e of a
`RAW DRIVE` is unchanged (7.2; in `BAUD`/`PULSE` it waits for the timer to
stop). The `RAW DRIVE` completes in the first cycle c >= e with
`G && (ot_done_i || (cfg_rawedge_i && dedge_i))`.

`field.md` 6.1 row `RAW DRIVE` becomes: "`raw_go` and (`ot_done_i` or
`cfg_rawedge_i && dedge_i`); or `eff` and (`ot_done_i` or
`cfg_rawedge_i && dedge_i`)".

So with an edge in e itself the drive takes effect (its output request in e
is made as today) and the next operation is current at e + 1. An edge
before e (for example while waiting for the BAUD timer) does not count.
`dedge_i` is `rosetta_coding`'s cleaned edge of the data input (`CODE[5:4]`,
differential check), either direction.

### 2.2 Interactions

| Feature | Behaviour |
|---|---|
| operation timer | **superseded by 2.5 (ruling 60): the timer IS cancelled at an early end.** Original text: not cancelled by the early end (tick.md 7 has no such input). A stale `ot_done` can only matter to an operation that started no timer of its own, and none can: a `PIN` with wait > 0 and a `RAW DRIVE` start a new timer in their effect cycle (a new start suppresses the old timer's `ot_done` in that cycle and replaces it); a `PIN` with wait 0 completes at `pin_we`; `FIELD` and `RAW SAMPLE` do not look at `ot_done`. Proven at lane level (2.3, lane P-H3b) |
| outputs | the early end changes no output; the level of the `RAW DRIVE` stays until the next operation changes it, as at a normal completion |
| launch delay | `RAW DRIVE` requests are never delayed (`coding.md` 4.2, ruling 48): unchanged |
| abort, halt | as today (the `RAW DRIVE` is abandoned / cleared) |
| `SETM CODE` | only to an idle lane (sequencer guarantee), so the bit never changes during a `RAW DRIVE` |
| stuffing, CRC, events, stretching, rulings 42/53/55 | not involved |
| `pk_srcdep` | a next operation with source `CRC` or `SR` is current at c + 2, as after any completion |

### 2.3 Required properties

Field proof: `a_i_rawd_done` is replaced by
`cu_done == G && ((raw_go && (ot_done_i || (cfg_rawedge_i && dedge_i))) || (e_pw && (ot_done_i || (cfg_rawedge_i && dedge_i))))`
for `RAW DRIVE` (with `cfg_rawedge_i = 0` it is today's property); the field
reference model (`field_model.sv`) gets the same term. New: P-H3a "with
`cfg_rawedge_i = 0` a `RAW DRIVE` never completes on `dedge_i` alone".
Lane proof: P-H3b "every `PIN`/`RAW DRIVE` with a timer completes on the
`ot_done` of the timer started in its own effect cycle, or (`RAW DRIVE`,
`CODE[12]`) on an edge" (no stale completion); cfg proof: `CODE[12]` stored.

Covers: early end by an edge in e; by an edge after e; timer end with
`CODE[12] = 1` and no edge; an edge before e in `BAUD` ignored; a `PIN` with
wait > 0 directly after an early-ended `RAW DRIVE`.

### 2.4 Worked timing: a CAN transmitter joining a competing SOF

`BAUD`, prescale 1, `CODE` open-drain data, `rawedge = 1`, data input RXD.
The program parks `RAW DRIVE level=1 (release), N` then the SOF field. Another
node's SOF falling edge reaches the RXD pin in cycle p.

| Cycle | Event |
|---|---|
| p | falling edge on the RXD pin |
| p + 3 | `in_i` shows it; `dedge = 1`: the `RAW DRIVE` completes; the SOF field is promoted at the end of p + 3 |
| p + 4 | SOF field current; timer stopped and `launch_req`: `lt`, `tx_go` (dominant) |
| p + 5 | our TXD pin dominant |

The join costs 5 cycles plus the transceiver loop delay, against 15-25
cycles through an event handler today.

### 2.5 Timer cancel at an early end (ruling 60; replaces the "operation timer" row of 2.2)

Found by the lane-level proof (22 Nov): leaving the timer of an early-ended
`RAW DRIVE` running lets the lane go idle with a live timer, and the
sequencer may then write `PERIOD`..`THR2L`. That breaks the tick block's
proven environment rule "no configuration change other than `TICK` while the
operation timer runs", so for that window the tick's block properties were
not covered. The timer is therefore cancelled.

- `rosetta_field`: new 1-bit output `ot_cancel_o`.
  `ot_cancel_o = 1` exactly in a cycle c in which a `RAW DRIVE` completes
  and `ot_done_i = 0` in c, that is a completion by the edge term of 2.1
  alone: `ot_cancel_o = cu_done(RAW DRIVE in c) && !ot_done_i`. It is 0 in
  every other cycle; with `cfg_rawedge_i = 0` it is constant 0. In the effect
  cycle e with an edge in e, `ot_start_o` and `ot_cancel_o` are both 1 in e
  (unless the timer's own `ot_done` is already 1 in e, the N = 1 case of
  `tick.md` 7, where the drive completes by the timer and `ot_cancel_o = 0`).
- `rosetta_tick`: new 1-bit input `ot_cancel_i`. `tick.md` 7 "The timer is
  cancelled by abort, halt and a new `ot_start`" becomes "... a new
  `ot_start`, and `ot_cancel_i`": if `ot_cancel_i = 1` in cycle c, no timer
  is running from c + 1 on and there is no `ot_done` after c until a later
  `ot_start_i` starts a new timer. This holds also when `ot_start_i = 1` in c
  (the cancel wins: the timer started in c does not run on). `ot_done` in c
  itself does not depend on `ot_cancel_i` (no combinational path from
  `ot_cancel_i` to any output). With no timer running and no start in c,
  `ot_cancel_i` has no effect. Nothing else in the block changes.
- `rosetta_lane`: `u_field.ot_cancel_o` drives `u_tick.ot_cancel_i`.
- Zero rule: with `CODE[12] = 0`, `ot_cancel_o` is constant 0, so the tick
  sees `ot_cancel_i = 0` always and is today's block.

Consequences: an idle lane never has a running operation timer (the ruling-42
statement holds again without exception); the tick's environment rule on
configuration changes holds in the lane as before; P-H3b (2.3) remains and
becomes: after an early end there is no `ot_done` at all until the next
operation starts its own timer.

Required properties. Field: `ot_cancel_o` exactly as defined (new); with
`cfg_rawedge_i = 0` it is 0. Tick: after `ot_cancel_i` in c, `ot_done_o = 0`
in every cycle from c + 1 until the cycle of the next `ot_start_i` (and from
that start on, the new timer follows section 7 exactly); `ot_done_o` in c
equals what it would be with `ot_cancel_i = 0`; with `ot_cancel_i` held 0 the
block is unchanged. Lane: the wiring; an idle lane runs no operation timer;
the tick's configuration-stability rule holds. Covers: cancel in the effect
cycle; cancel later; cancel in the cycle the timer would have ended anyway
(not asserted: the timer wins); a `PIN` with wait > 0 directly after a
cancelled timer, completing on its own timer.

## 3. H4: `RECOVER` options

### 3.1 Behaviour (amends `tick.md` 4.3)

New inputs `cfg_rone_i` (`TICK[14]`), `cfg_rsp2_i` (`TICK[15]`); `TICK[6]`
(`cfg_capfall_i`) selects the edge direction. With `Pr = max(PERIOD, 2)`:

```
redge = dedge_i && (!cfg_rone_i || dline_i == !cfg_capfall_i)   // qualifying edge
H'    = cfg_rsp2_i ? PERIOD2 : Pr >> 1
lt    = redge || (pt && inc >= Pr)                ; next cnt = 0 on lt
ct = st = cte = !redge && pt && inc == H' && !(inc >= Pr)
```

(`dline_i` is the level after the edge, before invert: `TICK[6] = 1` selects
falling edges, the CAN recessive-to-dominant edge with RXD active low.) The
term `!(inc >= Pr)` changes nothing when `rsp2 = 0`, because `H < Pr`; it keeps
ruling 50 (no `lt` and `ct` in one cycle) for `PERIOD2 >= Pr`, where the
capture then never happens. Compiler rule: with `rsp2`, `1 <= PERIOD2 < Pr`.
A non-qualifying edge is neither a launch tick nor a restart. In an abort cycle
the strobes are 0 and `cnt` updates as above (as today). With `h1on`, H1
replaces these strobes (1.7).

### 3.2 Interactions

| Feature | Behaviour |
|---|---|
| fields in `RECOVER` | `DRIVE` launches and aligned `PIN`s follow the new `lt` (qualifying edges and wraps): the CAN ACK drive launches from the recessive-to-dominant edge only |
| `coding.md` (NRZI reference, decode) | uses `st`, which is the new capture: unchanged rules |
| edge-interval counter, `RAW SAMPLE`, events | use every edge: unchanged |
| abort, halt, TICK write | as today (TICK write sets `cnt = 0`, a launch tick at the write) |
| launch delay, stretching, rulings 42/53/55 | not involved |

### 3.3 Required properties

Tick proof: group 11 gets the antecedent `!rone && !rsp2` where it states
`ct == (pt && inc == Hr)` and "a data edge is always an `lt`"; new group 11r:
(a) a qualifying edge is always an `lt` and restarts the count; (b) with
`rone`, a non-qualifying edge is an `lt` only if `pt && inc >= Pr` in that
cycle; (c) `ct` exactly when `!redge && pt && inc == H' && inc < Pr`; (d)
`lt` and `ct` never together (ruling 50) for every `PERIOD2`. Lane proof: no
change beyond the composition. cfg proof: `TICK[15:14]` stored.

Covers: a non-qualifying edge ignored; capture at `PERIOD2`; `PERIOD2 >= Pr`
(no capture); both options together.

### 3.4 Worked timing: one CAN bit at 500 kbit/s

`RECOVER`, `PERIOD = 120`, `PERIOD2 = 90`, `TICK[6] = 1`, `rone = rsp2 = 1`.
A recessive-to-dominant edge reaches `in_i` at t0: `lt` at t0 (count 0), `ct`
at t0 + 90 (75 % sample point), wrap `lt` at t0 + 120. The dominant-to-
recessive edge at t0 + 120 + 7 (slow recessive transceiver edge) does not
restart the count: the next capture stays at t0 + 120 + 90.

## 4. Area and proofs to rerun

| Change | RTL files | Size *(Yosys, typical)* | Proofs to rerun |
|---|---|---|---|
| H1 | new `src/rosetta_mdpll.sv`; `rosetta_tick.sv` (instance, mux, port), `rosetta_cfg.sv` (bit 13), `rosetta_lane.sv`, `rosetta_top.sv` (parameters) | 4.8 k um2 per H1 lane (26 flops) + a few cells of mux | new `formal/mdpll`; `formal/tick` (all tasks); `formal/cfg`; `formal/lane` (all tasks incl. `lane_haz`, `prove_r53`); `formal/top` (bring-up, chip, non-interference); lane equivalence H1 = 1 vs 0 |
| H3 | `rosetta_field.sv` (completion term, port), `rosetta_cfg.sv`, `rosetta_lane.sv` | about 100 um2 per lane *(estimate)* | `formal/field` (model and properties), `formal/cfg`, `formal/lane`, `formal/top` |
| H4 | `rosetta_tick.sv` (`redge`, `H'` mux, ports), `rosetta_cfg.sv`, `rosetta_lane.sv` | about 400 um2 per lane *(estimate: 16-bit mux on H, a few gates)* | `formal/tick`, `formal/cfg`, `formal/lane`, `formal/top` |

The UART, SPI and I2C RTL tests and the FPGA regression rerun unchanged
(compiled images are identical).

## 5. Existing properties: untouched, antecedent, new

| Block | Untouched | Needs the antecedent "new bits 0" | New |
|---|---|---|---|
| `rosetta_cfg` | write pulses, reset, high-half rule | the stored-bits property (now TICK[15:0] / [15:14], CODE[12:0]) | stored new bits |
| `rosetta_tick` | prescaler, BAUD (4.1), EDGE and stretch (4.2), PULSE, sample countdown and ruling 42/52, edge interval, operation timer, clock requests, ruling 50/53/55 groups | group 11 (`a_p11_*`): `!h1on && !rone && !rsp2` | 11h (H1), 11r (H4) |
| `rosetta_mdpll` | | | P1-P6 |
| `rosetta_field` | all except the one below | `a_i_rawd_done` and the model's RAW DRIVE completion: `!cfg_rawedge_i` (or the generalised form of 2.3) | P-H3a |
| `rosetta_coding`, `rosetta_delay`, `rosetta_pins`, `rosetta_events`, `rosetta_crc`, `rosetta_stuff` | all | | |
| `rosetta_lane` | all port properties (1-15), hazard proofs | `lane_model.sv` where it models RECOVER strobes or RAW DRIVE completion | P-H3b; H1 composition; lane equivalence (H1 = 1, bit 13 = 0 vs H1 = 0) |
| top | all (non-interference holds per lane; parameters only) | | |

## 6. Toolchain

- `toolchain/rosetta/isa.py`: `TICK` fields `mdpll` (13), `rone` (14), `rsp2`
  (15); `CODE` field `rawedge` (12). The assembler and disassembler follow
  from the table.
- `toolchain/rosetta/model.py` (executable ISA model): implement the three
  behaviours or refuse a program that sets the bits (today it would ignore
  them silently).
- Compiler: no change for UART/SPI/I2C. Rules to enforce once programs use
  the bits: H1 only in `RECOVER`, no transmit field or aligned `PIN` while
  `h1on`, `CODE.rx` level or Manchester, `CODE` written before `TICK`; H4
  `1 <= PERIOD2 < max(PERIOD, 2)` with `rsp2`.

## 7. Points where the existing specs made this awkward or ambiguous

1. `tick.md` 4.3 has no statement for a capture at or beyond the wrap; with
   `H = Pr >> 1` it cannot happen, so the new `!(inc >= Pr)` term is needed
   only for H4 and is a no-op today.
2. `TICK[6]` is meaningful in BAUD and EDGE only; giving it a `RECOVER`
   meaning (H4 direction) saves a bit but must be stated in isa.md.
3. `tick.md` 1 lists `dline_i` as unused; H1 is its first user.
4. `field.md` 7.2 says a `RAW DRIVE` "holds the bit hardware for its
   duration"; the operation timer has no cancel input (tick.md 7), so H3
   leaves a timer running and relies on the argument of 2.2 (proven at lane
   level, not in the field proof).
5. `lane.md` 5.1 "bits not listed are not stored and read as 0": a
   hand-written image that put 1s in `TICK[15:13]` or `CODE[12]` changes
   meaning after this amendment (none in the repository does).
6. The request "H1 as a lane parameter, off = today's lane" and "H3/H4 in all
   lanes" conflict literally: an H1 = 0 lane carries the 3 H3/H4 bits. They are
   invisible with the bits at 0; separate parameters would remove them.
7. H1 under `RECOVER` makes `lt = 0`, so `DRIVE` and aligned `PIN` never
   proceed: a programming rule, since the hardware cannot guarantee progress
   there (ruling 50 forbids `lt` and `ct` in one cycle, which rules out
   `lt = st`).
8. `coding.md` says Manchester decode needs `PULSE`; H1 decodes Manchester in
   `RECOVER` with the level decoder, which is correct but should be said in
   coding.md 3.2.
9. Ruling 59 describes H1 as "`CODE.rx` = Manchester under `RECOVER`
   timing". That combination is legal and defined today (it decodes as plain
   level, `coding.md` 3.2), so using it as the enable would change today's
   behaviour of a legal setting and break the zero rule; this amendment
   therefore enables H1 with the new bit `TICK[13]`. If the overseer prefers
   no new bit, `h1on = H1 && RECOVER && CODE.rx == 2` works with the same
   module, at the cost of the antecedent "not (`RECOVER` and `CODE.rx` = 2)"
   on the tick group-11 properties and on coding's decode property for that
   case (no compiled program uses the combination).
