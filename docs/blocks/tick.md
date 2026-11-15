# Block spec: `rosetta_tick` (tick source) and `rosetta_delay` (launch delay)

Status: draft for review (Phase 0, 14 Nov). Part of the lane (`lane.md`);
notation (`pt`, `a (+) D`, `Pd`, `P2d`, `RTRIP`) is defined in `lane.md` 1.

`rosetta_tick` decides **when** bits happen. It generates launch, capture
and sample strobes for the current field in the four tick modes, drives the
clock output requests (including both kinds of clock stretching), measures
edge intervals for `PULSE` and `RAW SAMPLE`, and times `PIN` waits and
`RAW DRIVE` durations. It does not know bit values (`rosetta_field`,
`rosetta_coding`). `rosetta_delay` (section 9) postpones output changes by
the launch delay.

## 1. Ports of `rosetta_tick`

| Port | Width | Meaning |
|---|---|---|
| `clk`, `rst_n` | 1 | clock; async active-low reset |
| `run_i` | 1 | lane live |
| `abort_i` | 1 | sequencer abort (`lane.md` 7.4) |
| `wr_tick_i` | 1 | a `SETM TICK` lands at the end of this cycle |
| `cfg_mode_i` | 2 | `TICK[1:0]`: 0 `BAUD`, 1 `EDGE`, 2 `RECOVER`, 3 `PULSE` |
| `cfg_ps_i` | 2 | `TICK[3:2]`: prescale 1, 8, 64, 512 |
| `cfg_clkin_i` | 2 | `TICK[5:4]`: clock input |
| `cfg_capfall_i` | 1 | `TICK[6]`: capture on the falling edge |
| `cfg_clkout_i` | 1 | `TICK[7]`: clock output on `out3` |
| `cfg_idle_i` | 1 | `TICK[8]`: clock idle level |
| `cfg_clkod_i` | 1 | `TICK[9]`: clock output open-drain |
| `cfg_rest_i` | 1 | `TICK[10]` |
| `cfg_honour_i` | 1 | `TICK[11]` `stretch_honour` |
| `cfg_hold_i` | 1 | `TICK[12]` `stretch_hold` |
| `cfg_period_i`, `cfg_period2_i` | 16 each | `PERIOD`, `PERIOD2` |
| `cfg_thr0_i` | 8 | `THR0` |
| `cfg_pw0_i`, `cfg_pw1_i` | 8 each | `PULSE[7:0]`, `PULSE[15:8]`: pulse-width high times |
| `cfg_dlyclk_i` | 1 | `CODE[10]`: the launch delay applies to the clock |
| `cfg_dly_i` | 8 | `THR2L[15:8]`: launch delay |
| `in_i`, `in_q_i` | 4 each | lane inputs this cycle and last cycle |
| `dline_i`, `dedge_i` | 1 each | cleaned data level and its edge (`coding.md` 3.1) |
| `pw_bit_i` | 1 | value of the pulse-width bit being sent (`coding.md`) |
| `cu_v_i`, `cu_fld_i`, `cu_tx_i`, `cu_smp_i`, `pk_v_i` | 1 each | operation status (`lane.md` 6.1) |
| `launch_req_i`, `fld_next_i`, `fld_first_i` | 1 each | from `rosetta_field` (`lane.md` 6.1, `field.md` 5.2) |
| `data_wait_i` | 1 | sequencer `qwait` |
| `ot_start_i`, `ot_raw_i` | 1 each | start the operation timer (section 7) |
| `ot_n_i` | 16 | its count |
| `dcap_i` | 1 | `rosetta_delay` applies a delayed capture-level clock change in this cycle |
| `pt_o` | 1 | prescale strobe |
| `lt_o`, `ct_o`, `st_o` | 1 each | launch tick, capture tick, sample strobe |
| `cte_o` | 1 | capture time (section 4) |
| `hte_o` | 1 | pulse-width high time elapsed |
| `spend_o` | 1 | *reg*. A BAUD sample is pending |
| `bt_run_o` | 1 | *reg*. BAUD timer running |
| `bt_stop_o` | 1 | the BAUD timer stops at the end of this cycle |
| `ot_done_o` | 1 | last cycle of the timed operation |
| `d16_o` | 16 | *reg*. Edge interval, prescaled cycles, saturated |
| `ge0_o` | 1 | `min(d16, 255) >= THR0` |
| `creq_o`, `cpay_o[1:0]`, `creq_dly_o`, `creq_cap_o` | 1, 2, 1, 1 | clock output request (section 8) |
| `stretch_o` | 1 | *reg*. The `stretch` flop (4.2). For proofs and covers only |

Outputs marked *reg* are flops. The strobes are combinational from flops,
`in_i`, `in_q_i`, `dline_i`/`dedge_i` (themselves combinational from
`in_i` and a coding flop), the `rosetta_field` status signals (functions of
flops) and `data_wait_i`, `abort_i`, `wr_tick_i`, `run_i`. Two exceptions:
`ot_done_o` also depends on `ot_start_i`, `ot_raw_i`, `ot_n_i` in the same
cycle (section 7), and `dcap_i` affects only the next state (section 5),
never an output in the same cycle.

## 2. Definitions

- `CL = !cfg_capfall_i` (capture level), `LL = !CL` (launch level),
  `IL = cfg_idle_i`.
- `cin = in_i[cfg_clkin_i]`, `cinq = in_q_i[cfg_clkin_i]`.
- Clock mapping of a level c: `map(c) = cfg_clkod_i ? {v3 = 0, e3 = !c} :
  {v3 = c, e3 = 1}`.
- `BAUDT` (the BAUD machinery is in use) = `cfg_mode_i == BAUD ||
  (cfg_mode_i == PULSE && cu_tx_i)`.
- `G` (gate) = `run_i && !abort_i`. In a cycle with `!G` every strobe output
  (`lt`, `ct`, `st`, `cte`, `hte`, `bt_stop`, `ot_done`, `creq` except the
  abort release of 4.2) is 0.

## 3. Prescaler, halt, abort, `TICK` write

### 3.1 Prescaler

State `pre` (9 bits). `pre(t+1) = run_i ? pre(t) + 1 (mod 512) : 0`. Reset 0.
`pt(t) = (cfg_ps_i == 0) || (pre(t) & PM) == PM` with `PM` = 7, 63, 511 for
prescale 8, 64, 512. So `pt` is 1 every cycle at prescale 1, and once every P
cycles otherwise, the first time in the P-th cycle after the lane starts.
`pt_next = pt` evaluated on `pre + 1` is used by the operation timer.

### 3.2 Halt and abort

- `run_i = 0` in cycle t: at the end of t every flop of this block takes its
  reset value (all 0).
- `abort_i` in cycle t: at the end of t `bt_run`, `bt_ph`, `lr`, `hw`,
  `spend`, the sample countdown, `stretch`, `srel` and the operation timer
  are cleared. `cnt` keeps counting in `RECOVER` (the RECOVER timer is not
  affected); in other modes it is cleared. `pre` and `ecnt` are not affected.

### 3.3 `TICK` write

`wr_tick_i` in cycle t (the lane is idle, `lane.md` 2.1): at the end of t
`bt_run`, `bt_ph`, `lr`, `hw`, `spend`, the sample countdown, `stretch`,
`srel` and `cnt` are cleared, and `twq <= 1` (a 1-cycle flop). In cycle t+1
(`twq = 1`), if the new `cfg_clkout_i = 1`: `creq = 1`, `cpay = map(IL)`,
`creq_dly = 0` (model D9). `twq` is cleared at the end of t+1.

## 4. Ticks per mode

Common: `cte`, `ct`, `st`, `lt` are defined below per mode; in a cycle with
`!G` they are 0. "Request clock c" means `creq = 1`, `cpay = map(c)`, made
only when `cfg_clkout_i = 1`.

### 4.1 `BAUD` (and `PULSE` with a transmitting field)

State: `bt_run` (timer running), `bt_ph` (0: launch to capture, 1: capture
to launch), `lr` (end of bit reached, holding), `hw` (honour wait), `cnt`
(16 bits, strobes counted in the current phase). In each cycle t let
`inc = sat16(cnt + pt)`. All rules apply only when `G` and `cfg_mode_i` is
`BAUD` or `PULSE`; the outputs `lt`, `ct`, `cte`, `hte` are passed to the
outputs when `BAUDT` (section 4.4 gives the PULSE mux).

**Stopped** (`bt_run = 0`): if `launch_req_i`: `lt = 1`; request clock `LL`
with `creq_dly = 1`; next `bt_run = 1`, `bt_ph = 0`, `cnt = 0`, `lr = hw = 0`.
Otherwise nothing.

**Phase 0** (`bt_run`, `bt_ph = 0`, `hw = 0`): let `due = inc >= Pd`.

- `cte = due && !spend`. (A capture is postponed while the previous bit's
  sample is pending; this happens only when the bit time is at most RTRIP.)
- If `cte`: request clock `CL` with `creq_dly = 1`, `creq_cap = 1`.
  - Without honour (`cfg_honour_i = 0`): `ct = 1`; next `bt_ph = 1`, `cnt = 0`.
  - With honour: next `hw = 1`.
- Else next `cnt = inc`.
- `hte = pt && inc == H && H != 0`, with `H = pw_bit_i ? cfg_pw1_i :
  cfg_pw0_i` (zero-extended). Coding uses `hte || cte` (`coding.md` 4.3).

**Honour wait** (`bt_run`, `bt_ph = 0`, `hw = 1`): if `cin == CL`: `ct = 1`;
next `bt_ph = 1`, `cnt = 0`, `hw = 0`. Otherwise nothing (the timer pauses
for as long as the clock input reads the launch level, model D10). The
first cycle that can end the wait is the cycle after `cte`.

**Phase 1** (`bt_run`, `bt_ph = 1`): let `eob = lr || inc >= P2d`.

- If `!eob`: next `cnt = inc`.
- If `eob` and `launch_req_i`: `lt = 1`; request clock `LL` (`creq_dly =
  1`); next `bt_ph = 0`, `cnt = 0`, `lr = 0`.
- If `eob`, `!launch_req_i` and `fld_next_i`: **hold**: next `lr = 1`; no
  strobe, no clock change (the clock stays at the capture level).
- If `eob`, `!launch_req_i`, `!fld_next_i`: **stop**: `bt_stop = 1`;
  request clock `cfg_rest_i ? LL : IL` with `creq_dly = 1`; next
  `bt_run = 0`, `lr = 0`.

Timing that follows (prescale 1, no honour, no postponement): launch at L;
clock at `LL` on the pin from L+1; capture tick and clock at `CL` request at
`L + Pd` (pin from `L + Pd + 1`); next launch at `L + Pd + P2d` if the field
has another bit. Bit time `Pd + P2d`, clock low/high `Pd`/`P2d` cycles. With
honour, phase 1 starts at the honour capture, so the high phase on the pin is
`P2d + RTRIP` cycles plus any stretching.

`bt_run_o = bt_run`. `bt_stop_o = bt_stop`.

### 4.2 `EDGE`

- `el = (cin != cinq) && cin == LL` (launch edge), `ec = (cin != cinq) &&
  cin == CL` (capture edge).
- `ct = st = cte = ec`.
- `lt = el || (stretch && fld_first_i)`.

**`stretch_hold`** (only when `cfg_clkout_i && cfg_hold_i`). State
`stretch`, `srel`, and `cnt` used as the release counter.

- Start: if `!stretch && el && !cu_v_i && !pk_v_i && data_wait_i`: request
  clock 0 (`creq_dly = 0`); next `stretch = 1`.
- When `stretch && fld_first_i` (a `FIELD` starts while the clock is held):
  that cycle is a launch tick (above) and next `srel = 1`, `cnt = 0`.
- While `srel`: `inc = sat16(cnt + pt)`; if `inc >= Pd`: request clock `IL`
  (`creq_dly = 0`), next `stretch = srel = 0`; else next `cnt = inc`.
- `abort_i` while `stretch`: request clock `IL` (`creq_dly = 0`) in the abort
  cycle (this request is not gated by `G`); next `stretch = srel = 0`.

So the first data bit is on the wire one cycle after the field starts and the
clock is released `Pd` prescaled cycles after that (model D11). In `EDGE`
nothing else requests the clock (F4).

### 4.3 `RECOVER`

Let `Pr = max(PERIOD, 2)`, `H = Pr >> 1`. State `cnt` counts strobes since
the last launch tick. It runs whenever `run_i`, with or without a field.

- If `dedge_i`: `lt = 1`; next `cnt = 0`.
- Else, with `inc = sat16(cnt + pt)`: if `pt && inc == H`: `ct = st = cte =
  1`. If `pt && inc >= Pr`: `lt = 1`, next `cnt = 0`. Otherwise next
  `cnt = inc`.

So a capture tick falls `H` prescaled cycles after each data edge and each
wrap, and the timer wraps `Pr` prescaled cycles after the last launch tick.
A `TICK` write sets `cnt = 0`, which counts as a launch tick at the write.
In an abort cycle the strobes are 0 but `cnt` updates as above.

### 4.4 `PULSE`

- With a transmitting field current (`cu_tx_i`): the outputs of 4.1.
- Otherwise: `ct = st = dedge_i`, `lt = cte = hte = 0`. Every data edge is a
  capture; `rosetta_coding` decides whether it carries a bit (`rxv`).

The BAUD machinery runs in `PULSE` mode exactly as in 4.1; it can only be
running after a transmitting field, and at that field's end of bit it
stops, or holds if a transmitting field is parked behind a receiving one
(a receiving field never asserts `launch_req`).

## 5. Sample strobe and pending sample

- `!BAUDT` (`EDGE`, `RECOVER`, `PULSE` receive): `st` as in section 4 (same
  cycle as `ct`).
- `BAUDT` (`BAUD`, or `PULSE` with a transmitting field) with honour:
  `st = ct && cu_smp_i`.
- `BAUDT` without honour: state `spend` and a countdown `scnt` (0..RTRIP-1)
  with an armed flag `sarm`.
  - `spend` is set at the end of a cycle with `ct && cu_smp_i`, and cleared
    at the end of a cycle with `st`, or by abort, halt or `TICK` write.
  - The **reference** cycle r is the `ct` cycle if `!(cfg_dlyclk_i &&
    cfg_dly_i != 0 && cfg_clkout_i)`, otherwise the first cycle at or after
    `ct` with `dcap_i` (the delayed capture-level clock change, section 9).
    At the end of r, `sarm <= 1`, `scnt <= 0`.
  - `st = 1` in cycle `r + RTRIP` (if `spend` is still set and `G`); then
    `sarm <= 0`.
  - At most one sample is pending, because phase 0 cannot end while `spend`
    is set (4.1).
- `spend_o = spend`. For `DRIVE` fields (`cu_smp_i = 0`) `spend` stays 0
  and `st` is not used.

Why RTRIP: the clock change requested in cycle r is on the pin from r+1 and,
like every other pin, reaches `in_i` 3 cycles later, at r+4. Sampling the
data input at r+4 therefore reads the data pin as it was when the lane's own
capture edge reached its pin (overseer ruling on F3).

## 6. Edge interval counter and classification

State `ecnt` (16 bits). `ecnt(t+1) = !run_i ? 0 : dedge_i ? (pt ? 1 : 0) :
sat16(ecnt + pt)`. `d16_o = ecnt`.

So in a cycle `t1` with an edge, `d16 = ` the number of strobes in cycles
`t0 .. t1-1`, where `t0` is the previous edge (at prescale 1, `t1 - t0`).
`d8 = min(d16, 255)`; `ge0_o = d8 >= cfg_thr0_i`. It runs in every mode and
is unaffected by abort and `TICK` writes. (`THR1`, `THR2` are deleted,
ruling 23.)

## 7. Operation timer

Started by `ot_start_i` in the effect cycle e of a `PIN` with wait > 0 or a
`RAW DRIVE`. `N = ot_raw_i ? max(ot_n_i, 1) : ot_n_i[11:0] * Pd` (the field
never starts it with N = 0). Then

- `ot_done = 1` in exactly one cycle: `(e (+) N) - 1`, the cycle before the
  N-th strobe after e. At prescale 1 that is `e + N - 1`, so the next
  operation is current at `e + N`.
- **Count ending at the first strobe.** When N = 1 and `pt_next = 1` in e
  (always the case at prescale 1 with N = 1), that cycle is e itself:
  `ot_done = 1` in e, combinationally from `ot_start_i`, where N = 1 iff
  `ot_raw_i ? (ot_n_i <= 1) : (ot_n_i[11:0] == 1 && Pd == 1)`. This is the
  only output of this block that depends on a `rosetta_field` action in the
  same cycle (`lane.md` 6).
- The timer is cancelled by abort, halt and a new `ot_start`.

Implementation note (not normative): an interval counter of 12 bits and a
strobe counter of 16 bits; apart from the start-cycle case above, `ot_done`
uses `pt_next` (3.1) on timer flops, so it is a flop compare, not a deep
path.

## 8. Clock output requests (summary)

All are made only when `cfg_clkout_i = 1`, with `cpay = map(level)`. At most
one per cycle by construction.

| Cause | Level | `creq_dly` | `creq_cap` |
|---|---|---|---|
| BAUD launch (4.1) | `LL` | 1 | 0 |
| BAUD capture time `cte` | `CL` | 1 | 1 |
| BAUD stop | `rest ? LL : IL` | 1 | 0 |
| EDGE stretch start | 0 | 0 | 0 |
| EDGE stretch release, or abort while stretching | `IL` | 0 | 0 |
| cycle after a `TICK` write | `IL` | 0 | 0 |

A `PIN` that writes `out3` in the same cycle wins (`pins.md`).

## 9. `rosetta_delay` (launch delay unit)

Holds at most one postponed output change. It sits between the request
sources (`rosetta_coding`, `rosetta_tick`) and `rosetta_pins`.

### 9.1 Ports

| Port | Width | Meaning |
|---|---|---|
| `clk`, `rst_n`, `run_i`, `abort_i`, `wr_tick_i` | 1 each | as above |
| `cfg_dly_i` | 8 | delay D, `THR2L[15:8]` |
| `cfg_dlyclk_i` | 1 | `CODE[10]`: the clock stream is delayed instead of the data stream |
| `cfg_dlyps_i` | 1 | `CODE[11]`: D counts prescale strobes, not cycles |
| `pt_i` | 1 | prescale strobe |
| `dreq_i`, `dpay_i[4:0]`, `dreq_dly_i` | | data request (`lane.md` 6.5) |
| `creq_i`, `cpay_i[1:0]`, `creq_dly_i`, `creq_cap_i` | | clock request |
| `pin_we_i`, `pin_mask_i[3:0]` | | a `PIN` takes effect this cycle |
| `d_we_o`, `d_pay_o[4:0]` | | data change to apply at the end of this cycle |
| `c_we_o`, `c_pay_o[1:0]` | | clock change to apply at the end of this cycle |
| `dcap_o` | 1 | the clock change applied is a delayed capture-level change |

### 9.2 Behaviour

State: `pv` (pending), `pclk` (pending belongs to the clock stream), `ppay`
(5 bits), `pcap`, `pcnt` (8 bits). All 0 at reset.

A request is **delayed** iff its `_dly` flag is 1, it belongs to the target
stream (`cfg_dlyclk_i ? clock : data`) and `cfg_dly_i != 0`. A request that
is not delayed passes through in the same cycle (`d_we_o = dreq_i`,
`d_pay_o = dpay_i`, likewise for the clock, `dcap_o = 0`).

Per cycle t (not halted):

1. **Maturity.** `mat = pv && (cfg_dlyps_i ? sat8(pcnt + pt_i) : pcnt + 1)
   >= cfg_dly_i`, where `pcnt` counts the cycles (or strobes) in
   `t0+1 .. t-1` after the request cycle t0. So a change requested in t0 is
   applied at the end of `t0 + D` (raw) or of `t0 (+) D` (prescaled), and is
   on the pin one cycle later.
2. **Cancel.** `cancel = abort_i || wr_tick_i || (pv && an undelayed request
   of the same stream in t) || (pin_we_i && pv && (pclk ? pin_mask_i[3] :
   (pin_mask_i[0] || (pin_mask_i[1] && w1))))`, with `w1` the pending
   payload's "also write `out1`" bit: a `PIN` cancels a pending change only
   if it writes an output that change writes (`lane.md` I22).
3. **Output.** If `pv && !cancel && (mat || a new delayed request in t)`:
   the pending change is output in t on its stream (`d_we_o` or `c_we_o`
   with `ppay`), `dcap_o = pclk && pcap`. A pending change is never lost
   except by cancel: a new delayed request flushes it early.
4. **Next state.** A delayed request arriving in t is **dropped** if a
   `PIN` in t writes an output that request would write:
   `drop = pin_we_i && (clock request ? pin_mask_i[3] : (pin_mask_i[0] ||
   (pin_mask_i[1] && w1_new)))`, with `w1_new` = `dpay_i[2]`, the new
   request's "also write `out1`" bit (ruling 58: the `PIN` wins, also
   against a change requested in its own cycle). If a delayed request
   arrives in t and is not dropped (and `!abort_i`, `!wr_tick_i`): `pv = 1`,
   `pclk`, `ppay`, `pcap = creq_cap_i`, `pcnt = 0`. Else if output or
   cancel: `pv = 0`. Else `pcnt` advances (by 1, or by `pt_i` in prescaled
   mode, saturating). Rules 2 and 3 are unchanged by a drop: a dropped
   request still counts as "a new delayed request in t" in rule 3, so a
   pending change the `PIN` does not cancel is still output in t.

An undelayed request and an output of the other stream may occur in the same
cycle; two changes of the same stream in one cycle cannot (rule 2). The
delay must be shorter than the spacing of the changes it delays, or they are
flushed early (the compiler checks; `lane.md` I22). `run_i = 0`: all state
cleared, outputs 0.

## 10. Required properties

`rosetta_tick` (inputs free except: `cfg_` inputs change only in cycles
with `cu_v_i = pk_v_i = 0` and `bt_run_o = 0` (the sequencer writes
registers 0-9 only to an idle lane), or while `run_i = 0`, and the `TICK`
inputs only at the end of a cycle with `wr_tick_i`; `fld_first_i` only when
`cu_fld_i`; `launch_req_i` only when `cu_fld_i`; `ot_start_i` only when
`cu_v_i`):

1. **Prescaler**: `pt` follows 3.1 exactly; with prescale 1 it is always 1.
2. **BAUD exact timing** (prescale 1, honour off, `spend` 0): a launch tick
   at L with the field still having bits gives `cte = ct` exactly at
   `L + Pd` and the next `lt` exactly at `L + Pd + P2d`; no other `lt`,
   `ct` in between. With general prescale: at `L (+) Pd` and
   `(L (+) Pd) (+) P2d`.
3. **Start from stopped**: `bt_run = 0` and `launch_req_i` and `G` imply
   `lt` in the same cycle.
4. **Hold never makes a clock edge or a stop**: in a cycle with `G`,
   `bt_run_o`, `!launch_req_i` and `fld_next_i`: `lt = 0`, `bt_stop = 0`,
   and a `creq` there is the capture request (`creq_cap_o = 1`). With
   prescale 1, honour off and no postponement, after a `ct` at c with
   `!launch_req_i && fld_next_i` in every cycle from `c + P2d` on, there is
   no `creq` and no `lt` until the first cycle with `launch_req_i` (which is
   an `lt`) or `!fld_next_i` (a stop).
5. **Stop**: `bt_stop` implies `G`, `bt_run_o`, `!launch_req_i` and
   `!fld_next_i`; it requests `rest ? LL : IL`, and `bt_run_o` is 0 next
   cycle. With prescale 1 and honour off, after a `ct` at c and no `lt`
   since, `bt_stop` occurs no earlier than `c + P2d`, and in `c + P2d` if
   `!launch_req_i && !fld_next_i` there.
6. **Clock alternation (BAUD)**: between two consecutive clock requests from
   the timer, levels alternate `LL`, `CL`, `LL`, ... except that the stop
   request may repeat `LL` (rest).
7. **Honour**: with honour, `ct` occurs only in a cycle with `cin == CL`
   after a `cte`, and is the first such cycle; `P2d` counts from it.
8. **Sample**: without honour, every `ct` with `cu_smp_i` is followed by
   exactly one `st`, RTRIP cycles after its reference cycle, unless abort,
   halt or `TICK` write intervenes; never two pending.
9. **EDGE**: `ct = st = ec`; `lt = el || (stretch_o && fld_first_i)`.
10. **Stretch**: `stretch_o` rises only after a cycle with `el`,
    `!cu_v_i`, `!pk_v_i`, `data_wait_i`, `cfg_clkout_i && cfg_hold_i`, in
    which `out3` low is requested (never creates a falling edge); the
    release (`IL`) is requested exactly at `s (+) Pd` after a cycle s with
    `stretch_o && fld_first_i`, or in an abort cycle while `stretch_o`;
    apart from these and the `TICK`-write request (16), `EDGE` makes no
    clock request.
11. **RECOVER**: after a launch tick at L with no data edge until then,
    `ct` at `L (+) H` and the next `lt` at `L (+) Pr`; a data edge is always
    an `lt` and restarts the count.
12. **PULSE receive**: `ct = st = dedge_i` when `!cu_tx_i`.
13. **Edge interval**: at a `dedge_i` cycle, `d16` equals the strobe count
    since the previous edge cycle (inclusive), saturated (reference counter).
14. **Operation timer**: exactly one `ot_done` per start, in cycle
    `(e (+) N) - 1`, unless cancelled.
15. **Abort and halt**: in a `!G` cycle all strobes are 0 (except the
    stretch release request); afterwards `bt_run_o = spend_o = stretch_o =
    0`.
16. **`TICK` write**: next cycle, the idle request when `cfg_clkout_i`.
17. **Covers**: each mode's ticks; honour wait of more than 10 cycles; a
    BAUD hold; a postponed capture (bit time 4); a sample at the same cycle
    as the next launch (`Pd = 3`, `P2d = 4`); stretch start and release;
    RECOVER resync by an edge in each half; an operation timer with W = 2 at
    prescale 8; `ot_done` in its start cycle (N = 1).

`rosetta_delay`:

1. With `cfg_dly_i = 0`, outputs equal inputs in the same cycle.
2. A delayed request in t0, with no later request on its stream and no
   cancel, is output exactly at `t0 + D` (raw) or `t0 (+) D` (prescaled).
3. Order: the changes of one stream are output in request order; at most one
   change per stream per cycle.
4. Nothing is output after a cancel until a new request.
5. `dcap_o` only with a clock output whose pending change had `creq_cap_i`.
6. Covers: flush by a new request; cancel by `PIN`; prescaled delay.

## 11. Worked timing

### 11.1 I2C controller bit with clock stretching

`BAUD`, prescale 1, `PERIOD = 6`, `PERIOD2 = 5`, clock output on `out3`
open-drain, capture rising (`CL = 1`, `LL = 0`), `rest = 1`, honour on. SCL
is already held low by `rest`. Field: `DRIVE 1, IMM 0` (SDA open-drain), the
last bit before a pause. The target keeps SCL low on the pin until cycle 12.
"SCL pin" is the wire; `cin` is what the lane sees (pin 3 cycles earlier).

| t | Timer | Request | SCL pin | `cin` | Notes |
|---|---|---|---|---|---|
| 0 | stopped; `launch_req`: `lt` | clock `LL` (drive low); data 0 | 0 | 0 | field starts (s = 0) |
| 1-5 | phase 0, `cnt` 0..4 | | 0 | 0 | SDA low from 1 |
| 6 | `inc = 6 = Pd`: `cte` | clock `CL`: release (`e3 = 0`) | 0 | 0 | honour: `hw = 1` |
| 7-12 | honour wait | | 0 (target holds) | 0 | |
| 13-15 | honour wait | | 1 | 0 | pin released by the target at 13 |
| 16 | `cin == CL`: `ct` | | 1 | 1 | bit captured; field completes; phase 1 |
| 17-20 | phase 1 | | 1 | 1 | |
| 21 | `eob` (16 + 5); no `launch_req`, no `fld_next`: stop | clock `rest`: `LL` | 1 | 1 | `bt_stop`; `busy_o` 1 through 21 |
| 22 | stopped | | 0 | 1 | |

SCL is high on the pin for 9 cycles (13-21), `P2d + RTRIP`, measured from
when it really rose. Without the target's stretch it would rise at 7, be seen
at 10, and fall at 16: also 9 cycles high. If a field had been parked, cycle
21 would be its `lt` instead (SCL low at 22, its data bit with it).

### 11.2 SPI controller read at 3+3 (mode 0)

`BAUD`, prescale 1, `PERIOD = PERIOD2 = 3`, clock push-pull, capture rising.
Field `EXCHANGE 2, msb` with `SR = {.., b1, b0}` (to send) starting at
s = 0, nothing parked. The target drives MISO bits m1, m0, changing them on
the SCK falling edges.

| t | Timer | `spend` | Event | SCK pin | MOSI pin |
|---|---|---|---|---|---|
| 0 | `lt` (L0) | 0 | send `SR[1]` = b1 | 0 | - |
| 3 | `cte`, `ct` (C0); clock 1 | 0 | reference r = 3 | 0 | b1 |
| 4 | phase 1 | 1 | | 1 | b1 |
| 6 | `eob`, `launch_req` (1 of 2 sent): `lt` (L1); clock 0 | 1 | sample pending, so send `SR[0]` = b0 (the bit after the pending shift) | 1 | b1 |
| 7 | | 1 | `st` = 3 + 4: read MISO (pin of cycle 4: m1); shift | 0 | b0 |
| 8 | | 0 | | 0 | b0 |
| 9 | `cte`, `ct` (C1); clock 1; r = 9 | 0 | | 0 | b0 |
| 10 | | 1 | | 1 | b0 |
| 12 | `eob`; no bit left, nothing parked: stop; clock idle 0 | 1 | `bt_stop` | 1 | b0 |
| 13 | stopped | 1 | `st`: read MISO (pin of cycle 10: m0); field completes | 0 | b0 |
| 14 | | 0 | result in the result register | 0 | b0 |

Each bit is 6 cycles (10 MHz). Both samples read MISO at the cycle the SCK
rising edge was on the pin (4 and 10). A target that changes MISO on the
falling edge (pin cycles 1 and 7) has 3 cycles, minus its own output delay
and setup, before the next rising edge reaches the pin. Had another field been parked, cycle 12 would hold
(`fld_next`) and that field would launch at 14 (its `PROMOTE` at the end of
13), giving one SCK high phase of 5 cycles.
