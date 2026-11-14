# Block spec: `rosetta_events` (four event detectors)

Status: draft for review (Phase 0, 14 Nov). Part of the lane (`lane.md`);
notation in `lane.md` 1. Uses the `EVC` layout of the overseer ruling on
F8/F9 (`lane.md` 5.1, I17).

Four detectors watch the lane inputs. When one **fires**, its pending bit
`Ek` is set. The sequencer owns arming and handler targets: an armed pending
event makes it vector (`sequencer.md` 5.6), which sends `abort_i` and
`ev_clr_i` back to the lane. `WAIT` and `BR Ek` read the pending bits. This
block never clears a pending bit on its own.

## 1. Ports

| Port | Width | Meaning |
|---|---|---|
| `clk`, `rst_n` | 1 | clock; async active-low reset |
| `run_i` | 1 | lane live |
| `in_i`, `in_q_i` | 4 each | lane inputs this cycle and last cycle (raw: no invert, no differential cleaning) |
| `pt_i` | 1 | prescale strobe (`tick.md` 3.1) |
| `cfg_evc_i` | 4 x 16 | `EVC0`-`EVC3` |
| `evc_wr_i` | 4 | `SETM EVCk` lands at the end of this cycle (`lane.md` 5.2) |
| `cfg_stuffev_i` | 1 | `STUFF[5]`: a stuffing violation raises event 3 |
| `stuff_viol_i` | 1 | `rosetta_stuff.violation_o` (a registered one-cycle pulse) |
| `ev_clr_i` | 4 | clear pending bits at the end of this cycle (sequencer X cycle) |
| `ev_pend_o` | 4 | *reg*. Pending bits `E0`-`E3` |
| `fire_o` | 4 | detector k fires in this cycle (combinational; for proofs and covers only) |

## 2. Configuration of detector k

From `c = cfg_evc_i[k]`:

| Field | Bits | Meaning |
|---|---|---|
| `A` | `c[1:0]` | signal A: `in_i[A]` |
| `L` | `c[2]` | level L |
| `trig` | `c[5:3]` | 0 off; 1 A at level L; 2 A edge to level L; 3 A any edge; 4 A and B both low; 5-7 off |
| `B` | `c[7:6]` | signal B: `in_i[B]` |
| `bm` | `c[9:8]` | B must be: 0 anything, 1 high, 2 low, 3 never (model D21) |
| `K` | `c[15:10]` | confirm window, prescaled cycles, 0-63 |

Signals in cycle t: `a = in_i[A]`, `aq = in_q_i[A]`, `b = in_i[B]`,
`bok = (bm == 0) || (bm == 1 && b) || (bm == 2 && !b)`.

Trigger 4 ignores `L` and `bm`. Triggers 0 and 5-7 never fire, never open a
window (ruling 21: undefined values in mode-register data act as off).

## 3. Behaviour

State per detector: `win` (window open), `cnt` (6 bits), `done` (has fired
in the current episode; level triggers only), `lv` (the level A changed to;
trigger 3 only). Reset and halt values 0. Each detector is independent;
nothing here depends on fields, arming or the sequencer.

### 3.1 Edge triggers (2 and 3)

- Trigger 2: `trg = (a != aq) && a == L && bok`; the condition that must
  then hold is `hc = (a == L) && bok`.
- Trigger 3: `trg = (a != aq) && bok`; at a trigger `lv <= a`; then
  `hc = (a == lv) && bok`.

Per cycle t (with `run_i` and not `evc_wr_i[k]`):

1. If `trg`: if `K == 0`, **fire** in t and `win <= 0`; else `win <= 1`,
   `cnt <= K` (a trigger restarts any open window).
2. Else if `win`: if `!hc`: `win <= 0` (cancelled, no fire). Else if `pt_i`:
   if `cnt == 1`, **fire** in t and `win <= 0`; else `cnt <= cnt - 1`.

So a trigger in cycle `t0` fires in cycle `t0 (+) K` (at prescale 1,
`t0 + K`) iff `hc` holds in every cycle `t0+1 .. t0 (+) K`; at `t0` it holds
by definition of the trigger.

### 3.2 Level triggers (1 and 4)

- Trigger 1: `cond = (a == L) && bok`. Trigger 4: `cond = !a && !b`.

Per cycle t (with `run_i` and not `evc_wr_i[k]`):

1. If `!cond`: `win <= 0`, `done <= 0` (the episode, if any, ended).
2. Else if `!win && !done` (an episode starts in t): if `K == 0`, **fire**
   in t and `done <= 1`; else `win <= 1`, `cnt <= K`.
3. Else if `win && pt_i`: if `cnt == 1`, **fire** in t, `win <= 0`,
   `done <= 1`; else `cnt <= cnt - 1`.
4. Else no change.

So the detector fires once per episode (a maximal run of cycles with
`cond`), in cycle `t0 (+) K` where `t0` is the episode's first cycle, if the
episode lasts that long (model D20). Trigger 1 with K is the "line held at a
level for a duration" event (timeout, UART break, WS2812 latch, TP_IDL);
trigger 4 is "both lines low", for example USB EOP.

### 3.3 Configuration write

`evc_wr_i[k]` in cycle t: the detector still evaluates cycle t with the old
configuration (it may fire in t), and at the end of t `win <= 0`,
`done <= 0`. From t+1 the new configuration applies; a level condition
already true in t+1 starts an episode there ("counts as starting at the
write", model D20). The pending bit is not changed by a write.

### 3.4 Pending bits

`f[k] = fire_o[k]` for k = 0..2, `f[3] = fire_o[3] || (cfg_stuffev_i &&
stuff_viol_i)`. At the end of each cycle:

```
pend[k] <= run_i && ( f[k] || (pend[k] && !ev_clr_i[k]) )
```

A fire and a clear in the same cycle leave the bit set (the new event is not
lost). `ev_pend_o = pend`. While `run_i = 0` the pending bits and all
detector state are held at 0.

The stuffing violation is OR-ed into event 3 (not a replacement of `EVC3`'s
detector); with `EVC3.trig = 0` (the reset value) event 3 reports only
violations. `stuff_viol_i` is high in the cycle after the bad wire bit
(`stuff.md`), so `E3` is visible two cycles after it.

### 3.5 When a fire reaches the sequencer

A fire in cycle t is visible on `ev_pend_o` in t+1. The sequencer acts at
lane L's first D cycle at or after t+1 (t+1 to t+5, later if a delivery is
queued, `sequencer.md` 5.6); the resulting `abort_i` and `ev_clr_i` arrive in
the following X cycle, so 2 to 6 cycles after the fire. From the pin: an
edge on a pin in cycle p is in `in_i` at p+3, so an edge trigger fires at
`(p+3) (+) K` and a level episode that starts on the pin at p fires at
`(p+3) (+) K`. These constants are what the compiler subtracts from event
timestamps (isa 7, C3).

## 4. Required properties

Assumptions: `cfg_evc_i[k]` changes only at the end of a cycle with
`evc_wr_i[k]` (or while `run_i = 0`).

1. **Edge exactness**: for trigger 2 or 3 with K = 0, `fire_o[k]` in t iff
   `trg` in t. For K > 0, `fire_o[k]` in u iff there is a `t0` with `trg`,
   `u = t0 (+) K`, `hc` in all of `t0+1 .. u` and no other `trg` in
   `t0+1 .. u`.
2. **Level exactness**: for trigger 1 or 4, `fire_o[k]` in u iff `cond`
   held in every cycle `t0 .. u` where `t0` is the first cycle of the current
   episode (or the first cycle after a write), `u = t0 (+) K` (or `u = t0`
   for K = 0).
3. **Once per episode**: between two fires of a level-triggered detector
   there is a cycle with `!cond` or a write.
4. **Off**: triggers 0, 5, 6, 7 and `bm = 3` (for triggers 1-3) never fire.
5. **Pending**: `pend` follows 3.4 exactly; set wins over clear; 0 while
   halted and in the cycle after a halted cycle.
6. **Independence**: detector k's `fire_o` does not depend on
   `cfg_evc_i[j]`, j != k.
7. **Stuffing source**: `E3` rises after a cycle with `stuff_viol_i &&
   cfg_stuffev_i` even when `EVC3` is off; never from `stuff_viol_i` without
   `cfg_stuffev_i`.
8. **Covers**: each trigger kind firing with K = 0 and K = 63; a window
   cancelled by A and by B; a write while a level condition is true; fire
   and clear in the same cycle; all four detectors firing in one cycle;
   prescale 8 window.

## 5. Worked timing: I2C START, true and false

Detector 0: `A = in0` (SDA), `trig = 2` (edge to low, `L = 0`), `B = in1`
(SCL), `bm = 1` (high), `K = 4` (the real I2C value is 18; 4 keeps the table
short), prescale 1. Columns are what the lane sees (`in_i`).

| t | SDA | SCL | Detector 0 | `ev_pend_o[0]` |
|---|---|---|---|---|
| 9 | 1 | 1 | idle | 0 |
| 10 | 0 | 1 | `trg` (SDA fell, SCL high): `win`, `cnt = 4` | 0 |
| 11 | 0 | 1 | `hc`, `pt`: `cnt = 3` | 0 |
| 12 | 0 | 1 | `cnt = 2` | 0 |
| 13 | 0 | 1 | `cnt = 1` | 0 |
| 14 | 0 | 1 | `cnt == 1`, `pt`: **fire** (10 + 4) | 0 |
| 15 | 0 | 0 | (SCL falls: the controller's `t_hd_sta` is over) | 1 |
| ... | | | sequencer vectors; `ev_clr_i[0]` in its X cycle clears the bit | |
| 29 | 1 | 1 | data bit 1, SCL high | 0 |
| 30 | 0 | 1 | the controller lowered SCL and SDA in the same pin cycle (zero hold time); SDA's synchroniser resolved first: `trg`, `win`, `cnt = 4` | 0 |
| 31 | 0 | 0 | `!hc` (SCL low): window cancelled, no fire | 0 |

The window rejects the false START of language G4 as long as the two
synchronisers resolve less than K cycles apart; any K of 2 or more covers a
one-cycle skew, and K = 18 also covers UM10204's 300 ns internal hold.
