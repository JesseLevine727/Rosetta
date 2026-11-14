# Block spec: `rosetta_lane` (one lane: top level)

Status: draft for review (Phase 0, 14 Nov). Authoritative inputs:
`docs/architecture.md`, `docs/isa.md` v0.2, `docs/blocks/sequencer.md`
section 7 (the lane/sequencer contract, adopted here with the changes in
section 12), `toolchain/MODEL_DECISIONS.md` (D1-D43, F1-F32) and the
overseer's rulings of 14 Nov. Where this spec departs from isa.md or from
the model, section 12 says so.

A lane is the bit-speed hardware for one protocol role. It executes one
**operation** at a time (a `FIELD`, a `RAW`, or a `PIN`, including the
`PIN` the sequencer synthesises for a `WAIT` timeout), keeps one more
operation **parked** behind it, and returns results through a one-entry
result register. It owns the mode registers, the shift register `SR`, the
check unit, the stuffing unit, the `M` flag, the event detectors and the four
output registers. It does not own PC, `R0`-`R3`, `Z`, `N`, run state or event
arming (all in `rosetta_seq`).

Block specs of the lane: this file (top, mode registers, handshake),
`tick.md`, `coding.md`, `field.md`, `events.md`, `pins.md`, and the existing
`crc.md` and `stuff.md`.

## 1. Notation used by every lane spec

- **Cycle** t, "end of cycle t", "visible in t+1": as `sequencer.md` 1. A
  register written at the end of t shows the new value in t+1.
- A **strobe** "in cycle t" is a condition evaluated from flops and inputs
  during cycle t. Its effects are written at the end of t.
- `pt(t)`: the prescale strobe (`tick.md` 3.1). It is 1 in every cycle when
  the prescale is 1.
- `a (+) D`, for D >= 1: the first cycle u > a such that exactly D cycles in
  `a+1 .. u` have `pt = 1`. **At prescale 1, `a (+) D = a + D`.** Every
  prescaled duration in the lane specs is written this way.
- `Pd = max(PERIOD, 1)`, `P2d = max(PERIOD2, 1)`.
- `IN_LAT = 3`: `in_i(t)` is the pin level of cycle t-3 (overseer ruling:
  two synchroniser flops before the crossbar and one register after it).
  The lane's outputs reach the pins combinationally through the crossbar.
  So a change the lane writes at the end of t is on the pin in t+1 and back
  at `in_i` in t+4. **`RTRIP = IN_LAT + 1 = 4`** is this round trip. Both are
  parameters (`IN_LAT` belongs to `xbar.md`; the lane uses only `RTRIP`).
- Operand codes, kinds and bit positions are those of isa.md 3.0-3.3.
  `NONE` = 15, `CRC` = 6, `SR` = 13, `IMM` = 12, `HOSTQ` = 4, `CAP` = 5,
  `MEM` = 7.
- `mask(n)` = the low n bits set. `tagged(v, tag) = (tag != 0) ? {tag,
  v[23:0]} : v`.

## 2. Ports

All outputs are functions of lane flops only, except `out_oe_o`, which is
also gated by `run_i` (one AND gate; `pins.md`). There is no combinational
path from any sequencer input to any output. Inputs marked *shared* are a bus
common to all lanes; the lane looks at them only in a cycle where its own
strobe is set (section 9, property 3).

### 2.1 From the sequencer (`sequencer.md` 7.1; all registered there)

| Port | Width | Meaning |
|---|---|---|
| `clk`, `rst_n` | 1 | clock; async active-low reset |
| `run_i` | 1 | lane live. 0 = halted: section 8 |
| `park_we_i` | 1 | load the parked-operation register at the end of this cycle. Never set when `park_full_o = 1` in the same cycle |
| `park_op_i` | 2 | *shared*. 0 `FIELD`, 1 `RAW`, 2 `PIN` (3 never sent) |
| `park_ctl_i` | 28 | *shared*. Instruction bits `[27:0]`. For a `WAIT` timeout: `{16'h0000, T}` (a `PIN` with no outputs selected, `align = 0`, wait T) |
| `park_data_i` | 32 | *shared*. Resolved source value; 0 for `PIN` and for sources `CRC`, `SR`, `NONE` |
| `res_ack_i` | 1 | the result has been delivered; clear it at the end of this cycle. Only when `res_valid_o = 1` |
| `abort_i` | 1 | abandon the current operation and discard the parked one at the end of this cycle (section 7.4) |
| `ev_clr_i` | 4 | clear these event-pending bits at the end of this cycle |
| `cfg_we_i` | 1 | `SETM`: write a mode register at the end of this cycle |
| `cfg_reg_i` | 5 | *shared*. Register number 0-13 |
| `cfg_hi_i` | 1 | *shared*. Write bits `[31:16]` instead of `[15:0]` |
| `cfg_data_i` | 16 | *shared*. Value |
| `cmd_i` | 2 | 0 none, 1 `CRCINIT`, 2 `CRCCHK`, 3 `STUFFCLR`; acts at the end of this cycle |
| `cmd_data_i` | 32 | *shared*. `{R1, R0}` for `CRCCHK` |
| `data_wait_i` | 1 | the sequencer is waiting for queue or channel data or space (`qwait`); used only by `TICK.stretch_hold` |

Sequencer guarantees, assumed by the lane proofs: all strobes are 0 while
`run_i = 0`; at most one of `park_we_i`, `cfg_we_i`, `cmd_i != 0`,
`res_ack_i`, `abort_i` per cycle, except that `abort_i` may come with
`ev_clr_i`; `cfg_we_i` with `cfg_reg_i` <= 9 and `cmd_i != 0` only when
`busy_o = park_full_o = 0` in that cycle; `cfg_reg_i` <= 13. The lane also
behaves as specified when these are violated, except that results may then
be wrong (no deadlock: every rule below is total).

### 2.2 From the crossbar and the timestamp counter

| Port | Width | Meaning |
|---|---|---|
| `in_i` | 4 | lane inputs `in0`-`in3`, registered in `rosetta_xbar`; `in_i(t)` = pin level in t-3 |
| `ts_i` | 16 | **new**: bits `[15:0]` of the free-running timestamp (`rosetta_chan.ts_o`), value of cycle t |

### 2.3 To the sequencer (`sequencer.md` 7.2)

| Port | Width | Meaning |
|---|---|---|
| `busy_o` | 1 | an operation is current, **or** the BAUD timer is still running to the end of the last bit (section 7.3) |
| `park_full_o` | 1 | the parked-operation register is occupied |
| `res_valid_o` | 1 | a result is waiting for delivery |
| `res_dst_o` | 4 | its destination operand code (never 15, 6, 12, 13, 14) |
| `res_data_o` | 32 | its value, already tagged when the destination is `HOSTQ` or `CAP` (isa 3.1) |
| `res_ts_o` | 16 | **new**: `ts_i` latched in the cycle the result was produced (section 7.5) |
| `dst_busy_o` | 4 | bit n: `Rn` is the destination of the current op, the parked op or the held result (`MEM` marks `R3`) |
| `m_o` | 1 | the `M` flag |
| `ev_pend_o` | 4 | event-pending bits `E0`-`E3` |
| `sr_o` | 32 | the shift register |
| `crc_o` | 32 | the check value, `rosetta_crc.value_o` (final XOR applied) |

### 2.4 To the crossbar

| Port | Width | Meaning |
|---|---|---|
| `out_val_o` | 4 | values of `out0`-`out3` |
| `out_oe_o` | 4 | enables; 0 = released. `out_oe_o = 0` in every cycle with `run_i = 0` |

## 3. Decomposition

| Instance | Module | Spec | Contents |
|---|---|---|---|
| `u_cfg` | `rosetta_lane_cfg` | this file, section 5 | the 14 mode registers, write decode, write side-effect pulses |
| `u_tick` | `rosetta_tick` | `tick.md` | prescaler; BAUD, EDGE, RECOVER, PULSE tick generation; clock output requests; stretching; RTRIP sample timing; edge-interval counter and pulse classification; operation timer |
| `u_dly` | `rosetta_delay` | `tick.md` section 9 | the launch delay: one pending output change, data or clock |
| `u_code` | `rosetta_coding` | `coding.md` | receive line cleaning and decoding; transmit coding and physical mapping |
| `u_field` | `rosetta_field` | `field.md` | parked and current operation registers, promotion, FIELD/RAW/PIN execution, `SR`, bit counters, result register, `M`, CRC and stuffing control |
| `u_crc` | `rosetta_crc` | `crc.md` | check unit (unchanged) |
| `u_stuff` | `rosetta_stuff` | `stuff.md` v2 | stuffing tracker (unchanged) |
| `u_evt` | `rosetta_events` | `events.md` | four event detectors, pending bits |
| `u_pins` | `rosetta_pins` | `pins.md` | the four output registers, request priority, run gating |
| (top) | | this file | `in_q` (previous `in_i`, 4 flops), `crcw_q` (1 flop, section 5.3), wiring |

`in_q(t+1) = in_i(t)` in every cycle (also while halted); reset 0. It is the
only copy of the previous inputs; `u_tick` (clock edges) and `u_evt` (edges)
use it. `u_code` keeps its own previous data level because it may hold it
(`coding.md` 3.1).

## 4. Signal-level diagram

```
             sequencer strobes + shared buses                 in_i[3:0]  in_q[3:0]  ts_i
               |        |          |           |                  |   |      |   |    |
               v        v          v           v                  |   |      |   |    |
          +---------+  +------------------------------------+    |   |      |   |    |
cfg_we -->| u_cfg   |  | u_field                            |<-------------------------+ ts
cfg_*  -->| mode    |  |  park reg -> current op -> result  |----> res_*, busy, park_full,
          | regs    |  |  SR, bit counters, M               |      dst_busy, m, sr
          +---------+  |  promotion, PIN/RAW sequencing     |
            | cfg_*    |   lt ct st ^   | tx_go tx_bit      |
            | wr_*     |   bt_run   |   | tx_rel raw_go     |      +--------+
            v          |   ot_done  |   v                   |----->| u_crc  |-> crc_o
     (to every block)  +------------|---+---------+---------+<-----|        |
                          ^  |      |  |pin_we    | ot_start       +--------+
              fld_*,      |  |      |  |pin_*     |                +--------+
              launch_req  |  v      |  |          v                | u_stuff|
                       +----------------------+   (u_tick)  <----->|        |
     in_i,in_q ------->| u_tick               |                    +--------+
     dline,dedge ----->|  timers, ticks, clock|--- creq ---+          |violation
                       |  stretch, ecnt, sym  |--- pt -----|---+      v
                       +----------------------+            |   |  +--------+
                          ^ dline, dedge, d8               |   +->| u_evt  |-> ev_pend_o
     in_i ------------>+----------------------+            |      +--------+
     out_val_o[1:0] -->| u_code               |--- dreq ---+
                       |  rx clean/decode     |            v
                       |  tx code, mapping    |      +----------+      +---------+
                       +----------------------+      | u_dly    |----->| u_pins  |-> out_val_o
                          ^ rxb, rxv to u_field      | 1 pending|      |         |-> out_oe_o
                                                     +----------+      +---------+
                                                       (PIN writes from u_field go
                                                        straight to u_pins and cancel
                                                        u_dly's pending change)
```

## 5. Mode registers: where they live and how `SETM` reaches them

### 5.1 The register file `rosetta_lane_cfg`

All mode registers are flops in `u_cfg`, reset to 0. On a rising edge with
`run_i && cfg_we_i`: if `cfg_hi_i = 0`, bits `[15:0]` of register
`cfg_reg_i` take `cfg_data_i` (only the stored bits listed below); if
`cfg_hi_i = 1`, bits `[31:16]` of `CRCPOLY` take `cfg_data_i` and a high
write to any other register changes nothing (model D42). Registers keep their
values while halted (`run_i = 0`). Every other block receives the registers
as `cfg_` inputs and assumes them stable while it is active (the sequencer
writes registers 0-9 only when the lane is idle; `EVC0`-`EVC3` may change at
any time and `u_evt` handles that).

Layout as stored. Bits not listed are not stored and read as 0. `EVCk` uses
the **new layout** of the overseer ruling on F8/F9 (section 12, item I17).

| # | Register | Stored bits | Flops | Used by |
|---|---|---|---|---|
| 0 | `TICK` | `[1:0]` mode, `[3:2]` prescale, `[5:4]` clock input, `[6]` capture on falling edge, `[7]` clock output on `out3`, `[8]` clock idle level, `[9]` clock output open-drain, `[10]` `rest`, `[11]` `stretch_honour`, `[12]` `stretch_hold` | 13 | tick, field, pins |
| 1 | `PERIOD` | `[15:0]` | 16 | tick |
| 2 | `PERIOD2` | `[15:0]` | 16 | tick |
| 3 | `THR01` | `[7:0]` `THR0` (`THR1` deleted, ruling 23) | 8 | tick |
| 4 | `THR2L` | `[15:8]` launch delay (`THR2` deleted, ruling 23) | 8 | tick, delay |
| 5 | `CODE` | `[1:0]` rx coding, `[3:2]` tx coding, `[5:4]` data input, `[6]` invert, `[7]` data open-drain, `[8]` differential out, `[9]` differential in, `[10]` delay applies to clock, `[11]` delay prescaled | 12 | coding, tick, delay |
| 6 | `PULSE` | `[7:0]` high time for 0, `[15:8]` high time for 1 | 16 | tick, coding |
| 7 | `STUFF` | `[2:0]` N, `[3]` mode, `[4]` enable, `[5]` violation raises event 3 | 6 | stuff, events |
| 8 | `CRCPOLY` | `[31:0]` | 32 | crc |
| 9 | `CRCCFG` | `[4:0]` width-1, `[5]` init all ones, `[6]` final XOR | 7 | crc |
| 10-13 | `EVC0`-`EVC3` | `[1:0]` signal A, `[2]` level L, `[5:3]` trigger (0 off, 1 A at L, 2 A edge to L, 3 A any edge, 4 A and B both low, 5-7 off), `[7:6]` signal B, `[9:8]` B must be (0 any, 1 high, 2 low, 3 never), `[15:10]` K | 4 x 16 | events |
| | **Total** | | **198** | |

The CRC unit's `cfg_init_i` is `{32{CRCCFG[5]}}`, `cfg_xorout_i =
CRCCFG[6]`, `cfg_width_m1_i = CRCCFG[4:0]`, `cfg_poly_i = CRCPOLY`.

### 5.2 Write pulses

`u_cfg` outputs, all functions of `cfg_we_i`, `cfg_reg_i`, `cfg_hi_i` and
`run_i` in the write cycle t (combinational: the inputs are sequencer flops).
"Write to r" means `run_i && cfg_we_i && cfg_reg_i == r && !cfg_hi_i`; a
high-half write is a write only for register 8 (it changes nothing elsewhere,
so it has no side effect either, I21):

| Pulse | Condition in cycle t | Effect |
|---|---|---|
| `wr_tick` | write to 0 | `u_tick` resets its timing state at the end of t; in t+1 it requests the clock idle level (`tick.md` 3.3). `u_code` resets its Manchester phase; `u_dly` drops a pending change |
| `wr_code` | write to 5 | `u_code` resets its Manchester phase (`coding.md`) |
| `wr_stuff` | write to 7 | `u_stuff.clear_i = 1` in cycle t (satisfies `stuff.md`: cfg changes only in a cycle with `clear_i`) |
| `wr_crc` | write to 8 (either half, `cfg_hi_i` ignored) or 9 | `u_crc.init_i = 1` in cycle t **and** in t+1 (`crcw_q`), section 5.3 |
| `wr_evc[k]` | write to 10+k | `u_evt` restarts detector k at the end of t (`events.md` 3.3) |

### 5.3 Why `wr_crc` initialises twice

`crc.md` allows its `cfg_` inputs to change only at an edge whose cycle has
`init_i = 1`. The write lands at the end of t, so `init_i` must be 1 in t.
That load uses the old configuration, so a second `init_i` in t+1 loads the
new initial value under the new width. From t+2 the CRC state equals the new
initial value in the new width. Programs still issue `CRCINIT` per frame;
this rule only keeps the unit consistent. (isa.md change: section 12, I21.)

### 5.4 Commands

| `cmd_i` | Effect at the end of the cycle |
|---|---|
| 1 `CRCINIT` | `u_crc.init_i = 1` |
| 2 `CRCCHK` | `M <= (u_crc.state_o == cmd_data_i)` (`state_o` is the raw state, zero above bit W-1) |
| 3 `STUFFCLR` | `u_stuff.clear_i = 1` |

`u_crc.init_i = (cmd_i == 1) || wr_crc || crcw_q`. `u_stuff.clear_i =
(cmd_i == 3) || wr_stuff || !run_i` (clearing while halted makes a restarted
lane start with no stuffing history). All are gated by `run_i` except the
last term.

## 6. Internal interfaces

Every signal below is combinational within the cycle named, unless it is
marked *reg* (a flop output). "Driver" is the block that computes it. No
signal in this table depends combinationally on another signal that depends
on it (the loop-free order is: `u_cfg`, `u_code` receive side, `u_field`
status, `u_tick`, `u_field` actions, `u_tick`'s `ot_done`, `u_field`
completion and promotion, `u_code` transmit side, `u_dly`, `u_pins`).
`ot_done` is the one `u_tick` output that may depend on a `u_field` action in
the same cycle: a timed operation whose count ends at the first strobe after
its effect cycle e completes in e itself (`tick.md` 7). `dcap` affects only
`u_tick` state.

### 6.1 `u_field` to `u_tick`

| Signal | Width | Meaning in cycle t |
|---|---|---|
| `cu_v` | 1 | *reg*. An operation is current |
| `cu_fld` | 1 | *reg*. The current operation is a `FIELD` |
| `cu_tx` | 1 | *reg*. ... a `FIELD` of kind `DRIVE` or `EXCHANGE` |
| `cu_smp` | 1 | *reg*. ... a `FIELD` of kind `SAMPLE`, `EXCHANGE` or `EXPECT` |
| `pk_v` | 1 | *reg*. An operation is parked |
| `launch_req` | 1 | the current `FIELD` has a wire bit to launch at a BAUD launch point (`field.md` 5.2). Function of `u_field` flops and `u_stuff.stuff_next_o` only |
| `fld_next` | 1 | a field launch is coming: a BAUD-timed `FIELD` is parked and either no op is current or the current op is a `FIELD` with `launch_req = 0` (exactly `field.md` 5.2; `u_tick` looks at it only at an end of bit, where `launch_req = 0` means no wire bit left, `HOLD`, or a field that is not BAUD-timed). Function of flops and `stuff_next` only |
| `fld_first` | 1 | t is the first cycle in which the current `FIELD` is current (its start cycle s) |
| `ot_start` | 1 | start the operation timer in this cycle (the effect cycle of a `PIN` with wait > 0 or of a `RAW DRIVE`) |
| `ot_raw` | 1 | with `ot_start`: 0 = count `W * Pd` strobes with `W = ot_n[11:0]`; 1 = count `max(ot_n, 1)` strobes |
| `ot_n` | 16 | with `ot_start`: the count |

### 6.2 `u_tick` to `u_field`

| Signal | Width | Meaning in cycle t |
|---|---|---|
| `lt` | 1 | launch tick (`tick.md` 4) |
| `ct` | 1 | capture tick: the current wire bit is captured for bookkeeping |
| `st` | 1 | sample strobe: the data input is read in this cycle (`rxb`, `rxv` valid) |
| `spend` | 1 | *reg*. A BAUD sample is pending (a `ct` happened and its `st` has not) |
| `bt_run` | 1 | *reg*. The BAUD timer is running |
| `bt_stop` | 1 | the BAUD timer stops at the end of this cycle (end of the last bit) |
| `ot_done` | 1 | the operation timer has counted its N strobes: t is the last cycle of the timed operation |
| `d16` | 16 | *reg*. Edge interval in prescaled cycles, saturated at `0xFFFF` (`tick.md` 6) |

### 6.3 `u_code` and `u_tick`

| Signal | Driver | Width | Meaning in cycle t |
|---|---|---|---|
| `dline` | code | 1 | cleaned data input level (before `invert`); holds while a differential pair is invalid (`coding.md` 3.1) |
| `dedge` | code | 1 | `dline` changed in this cycle |
| `cte` | tick | 1 | BAUD capture time (phase 0 ended); in EDGE/RECOVER equal to `ct`; 0 in PULSE receive |
| `hte` | tick | 1 | BAUD phase-0 count reached the pulse-width high time of `pw_bit` (`tick.md` 4.1) |
| `pw_bit` | code | 1 | *reg*. Value of the pulse-width bit being sent |
| `ge0` | tick | 1 | `d8 >= THR0`, with `d8 = min(d16, 255)` |

### 6.4 `u_field` and `u_code`

| Signal | Driver | Width | Meaning in cycle t |
|---|---|---|---|
| `tx_go` | field | 1 | a wire bit of the current field is launched in this cycle |
| `tx_bit` | field | 1 | its value (data bit or stuff bit), before line coding |
| `tx_rel` | field | 1 | a receiving field releases the data output in this cycle |
| `raw_go` | field | 1 | a `RAW DRIVE` takes effect in this cycle |
| `raw_lvl` | field | 1 | its level |
| `dedge` | code | 1 | (also to `u_tick`) data edge; `RAW SAMPLE` completes on it |
| `rxb` | code | 1 | the decoded received bit (valid with `st`) |
| `rxv` | code | 1 | this `st` carries a bit (0: no bit, for example a Manchester boundary edge or an invalid differential pair) |

### 6.5 Output requests

| Signal | Driver | Width | Meaning in cycle t |
|---|---|---|---|
| `dreq` | code | 1 | request a change of the data output(s) at the end of this cycle |
| `dpay` | code | 5 | `{v0, e0, w1, v1, e1}`: value and enable for `out0`; `w1` = also write `out1` with `v1`, `e1` |
| `dreq_dly` | code | 1 | this data request is subject to the launch delay (requests made by `FIELD`s; not `RAW`) |
| `creq` | tick | 1 | request a change of `out3` at the end of this cycle |
| `cpay` | tick | 2 | `{v3, e3}` |
| `creq_dly` | tick | 1 | this clock request is subject to the launch delay (BAUD launch, capture and end-of-bit changes) |
| `creq_cap` | tick | 1 | this clock request is the change to the capture level |
| `d_we`, `d_pay` | dly | 1, 5 | data request to apply at the end of this cycle (immediate or delayed) |
| `c_we`, `c_pay` | dly | 1, 2 | clock request to apply at the end of this cycle |
| `dcap` | dly | 1 | the applied clock request is a (delayed) capture-level change (`tick.md` 5) |
| `pin_we` | field | 1 | a `PIN` takes effect at the end of this cycle |
| `pin_mask`, `pin_val`, `pin_oe` | field | 4 each | `PIN` bits `[27:24]`, `[23:20]`, `[19:16]` |

### 6.6 Others

| Signal | Driver | To | Meaning |
|---|---|---|---|
| `pt` | tick | evt, dly | prescale strobe |
| `crc_en`, `crc_bit` | field | crc | `en_i`, `bit_i` |
| `stf_valid`, `stf_bit` | field | stuff | `bit_valid_i`, `bit_i` |
| `stuff_next`, `stuff_bit` | stuff | field, tick | `stuff_next_o`, `stuff_bit_o` |
| `stuff_viol` | stuff | evt | `violation_o` |
| `out_val` | pins | code | *reg*. `out_val_o` (used as the value kept on release) |

## 7. The operation handshake

This section is the contract between the sequencer and the lane. `field.md`
gives the internal detail; the properties in section 8 are stated on the
lane ports.

### 7.1 Parking

`park_we_i` in cycle t writes the parked register at the end of t:
`pk = {op, ctl, data}` from the shared buses, `pk_v <= 1`. So
`park_full_o = 1` and `dst_busy_o` include it from t+1. Nothing else in the
lane reads the shared buses.

### 7.2 Promotion (the parked op becomes current)

At the end of cycle p the parked op is **promoted** (moved to the current
register, `pk_v <= 0`) iff

```
PROMOTE(p) = run_i && pk_v && !abort_i
             && (!cu_v || cu_done)
             && !(pk_srcdep && cu_v)
```

- `cu_done`: the current op completes in cycle p (`field.md` 6).
- `pk_srcdep`: the parked op is a `FIELD` or `RAW DRIVE` whose source is
  `CRC` or `SR`. Those sources are read at promotion (`field.md` 4.1), so they wait
  until the cycle after the previous op completed, when `SR` and the CRC hold
  their final values.

The promoted op is current from cycle `s = p + 1`, its **start cycle**. From
an idle lane: park in the X cycle t, promotion at the end of t+1, start at
t+2. (`sequencer.md` 8.1 assumed "at once"; the extra cycle is the price of
not bypassing the parked register.)

### 7.3 When an op finishes, and gap-free fields

An op has two moments: **completion** (its result, `M` and `SR` exist, and
the next op may be promoted) and, in BAUD, **the end of the last bit** (the
timer's next launch point, `tick.md` 4.1). Following the overseer ruling on
F1 (model D30):

| Timing | Completion | Bit hardware free for the next op |
|---|---|---|
| EDGE, RECOVER, PULSE receive | the last capture/sample cycle c | c+1 (next op current at c+1, or c+2 if `pk_srcdep`) |
| BAUD and PULSE transmit, `DRIVE` | the last capture tick c | next `FIELD` current at c+1 (c+2 if `pk_srcdep`) and launches at the end of the last bit; a `PIN` or `RAW DRIVE` takes effect when the timer stops |
| BAUD, sampling kinds | the last sample, RTRIP cycles after the last capture tick (at the capture tick with `stretch_honour`; RTRIP after the delayed capture edge with the clock delayed, I22) | as above |
| `PIN`, `RAW DRIVE` | the last cycle of its wait: `(e (+) N) - 1` (which is e itself when the first strobe after e ends the count), or e when N = 0 | next op current exactly at `e (+) N` (`e + 1` when N = 0; one cycle later if it is `pk_srcdep`) |
| `RAW SAMPLE` | the cycle of the data edge it measures | next cycle |

`busy_o = cu_v || bt_run`. So `busy_o` stays 1 after a BAUD field's
completion until the end of its last bit, and a sync instruction (`HALT`,
`SETM TICK`, ...) cannot cut the last bit short (model D25).

**Gap-free.** Consecutive `FIELD`s are gap-free (the second launches at the
first one's end of last bit, or at its own first launch tick in EDGE and
RECOVER) iff the second is current by then. It is current 2 cycles after its
park, but never before the cycle after the first one's completion (2 cycles
after with source `CRC`/`SR`, `pk_srcdep`). So it is gap-free when it is
parked by the end of the first one's completion cycle and, in BAUD, the end
of the last bit comes at least 1 cycle after that completion (2 with source
`CRC`/`SR`). The end of the last bit is `c (+) P2d` for a last capture tick
c; completion is at c for `DRIVE` (and for any kind with `stretch_honour`)
and at `c + RTRIP` for sampling kinds (`c + D + RTRIP` with the clock delayed
by D cycles). At prescale 1 this means `P2d >= 1` (2) for `DRIVE` and
`P2d >= RTRIP + 1` (`RTRIP + 2`) for sampling kinds without honour. At
prescale P, `c (+) P2d` can come as early as `c + (P2d - 1) P + 1`, so the
guarantee is `(P2d - 1) P + 1` at least the same numbers. A field parked
later is still gap-free if it is current by the end of the last bit.
When the next BAUD-timed field is already parked but not yet current at the
end of the previous bit (the previous field's last sample is still pending,
it is in an overrun `HOLD` (7.5), or the promotion is deferred by
`pk_srcdep`), the timer **holds** at the end
of the bit with the clock at the capture level and launches as soon as the
field is current (`tick.md` 4.1). With nothing parked it stops. It never inserts an
idle-level clock pulse between parked fields.

### 7.4 Abort

`abort_i` in cycle t (sent by the sequencer when it vectors to an event or
when an event ends a `WAIT` timeout):

- The current op makes **no progress in cycle t**: no launch, capture,
  sample, completion, result, `M` change, CRC or stuffing update, and no
  output request from it. At the end of t it is discarded (`cu_v <= 0`).
- The parked op is discarded (`pk_v <= 0`); a word its source popped is lost.
- Kept: `SR` (the partial bits), the CRC state, the stuffing run count, the
  bit counter value, `M`, the result register (a result already produced is
  delivered), the output registers. A result held by an overrun (7.5) is
  discarded.
- Cancelled: the BAUD timer (stops; no end-of-bit clock change, so the clock
  keeps its level), a pending BAUD sample, the operation timer, a pending
  launch-delay change, a pending Manchester or pulse-width second half, and
  `stretch_hold` (the clock is released at the end of t).
- The RECOVER timer, the edge-interval counter, the event detectors and the
  pending bits are not affected (`ev_clr_i` acts separately).

### 7.5 Results

A completing `FIELD` or `RAW SAMPLE` with destination other than `NONE`
produces a result `{dst, value, ts}` (`field.md` 6.2). In the
completion cycle c:

- If `res_valid_o = 0` or `res_ack_i = 1` in c: at the end of c the result
  register takes it, `res_valid_o = 1` from c+1, `res_ts_o = ts_i(c)`. The
  value is `tagged(value, tag)` only for destinations `HOSTQ` and `CAP`
  (isa 3.1); otherwise it is the plain value.
- Otherwise (**overrun**): the op enters `HOLD`. It stays current
  (`busy_o = 1`, nothing is promoted), its value stays in `SR`, and it
  completes in the first cycle in which the register can take it (same
  rule). `M` (for `EXPECT`) changes at that completion, and the timestamp is
  `ts_i` of that cycle. A BAUD timer holds or stops as in 7.3; in EDGE, ticks
  that pass during `HOLD` are lost. Nothing is ever overwritten. The compiler's
  schedule check must rule overrun out; the hardware only keeps it safe.
- `res_dst_o`, `res_data_o`, `res_ts_o` are stable while `res_valid_o = 1`
  and change only at the end of a cycle with `res_ack_i` or while
  `res_valid_o = 0`. `res_ack_i` clears `res_valid_o` at the end of its cycle
  unless a new result loads at the same edge.

**Timestamp reconstruction** (change to `sequencer.md` 5.1, section 12 item
I9): for a `CAP` delivery whose first D cycle sees the 32-bit counter `T`,
the timestamp word is `{T[31:16] - (T[15:0] < res_ts ? 1 : 0), res_ts}`. It
is exact when the delivery starts within 65,535 cycles of completion.

### 7.6 Status outputs

- `park_full_o = pk_v`.
- `busy_o = cu_v || bt_run`.
- `dst_busy_o[n] = (cu_v && R(D(cu)) == n) || (pk_v && R(D(pk)) == n) ||
  (res_valid_o && R(res_dst) == n)`, where for an op `D` is its destination
  if it is a `FIELD`, or a `RAW` with direction sample, and none otherwise; and
  `R(d) = 3` for `d = MEM`, `d` for `d` in 0-3, none otherwise (so `MEM`
  marks `R3` from the park on, because its delivery increments `R3`). Only `FIELD`
  and `RAW SAMPLE` ops have destinations; a `PIN` (including a `WAIT`
  timeout, whose bits 13:10 belong to its wait count) never marks one.
- `m_o`, `sr_o`, `crc_o`: registered state. While `busy_o = park_full_o = 0`
  and no `cmd_i`, `cfg_we_i` to 8/9 or `crcw_q`: all three are constant.

## 8. Halt and reset

**Reset** (`rst_n` low): every flop to the value its block spec gives; in
particular all mode registers 0, `pk_v = cu_v = res_valid = 0`, timers
stopped, outputs `oe = 0`, `val = 0`, pending bits 0, `M = 0`, `SR = 0`, CRC
state 0, stuffing history clear.

**Halted** (`run_i = 0` in cycle t): at the end of t, `pk_v`, `cu_v`,
`res_valid`, `bt_run`, `spend`, the operation timer, `stretch`, the delay
unit, Manchester/pulse halves, the event windows and the pending bits are
cleared, the prescaler is reset, and the output enables are cleared. Kept:
mode registers, `SR`, `M`, the CRC state, output values. The stuffing unit is
held in `clear_i`. `out_oe_o = 0` in t itself (gating). So from t+1:
`busy_o = park_full_o = res_valid_o = 0`, `dst_busy_o = 0`, `ev_pend_o = 0`
(`sequencer.md` 7.4). All sequencer strobes are ignored while `run_i = 0`.

## 9. Required properties (lane level, on the lane ports)

Block properties are in each block spec. These are the integration
properties of `rosetta_lane` as a whole. The checker may keep shadow copies
of the mode registers from the `SETM` strobes, and counters of parks and
results. Completion and promotion are not visible at the lane ports; the
properties that need them (6, and the exact halves of 5, 9 and 13) are proved
by composition: `field.md` property 1 (against its observation output
`cu_done_o`), `events.md` 4 and `tick.md` 10. The lane checker states only
the port-level halves below. Liveness (a parked op eventually completes)
is not claimed: fields wait for external clocks and edges.

1. **Halted lane drives nothing.** `out_oe_o = 0` in every cycle with
   `run_i = 0`. If `run_i = 0` in cycle t, then in t+1: `busy_o`,
   `park_full_o`, `res_valid_o` are 0 and `dst_busy_o = ev_pend_o = 0`.
2. **Never started.** From reset until the first cycle with `run_i = 1`,
   `out_oe_o = 0`, `out_val_o = 0`, and all status outputs are 0.
3. **Shared-bus lemma** (`sequencer.md` 7.3). Two copies with equal inputs
   except that `park_op_i`/`park_ctl_i`/`park_data_i` may differ in cycles
   with `park_we_i = 0`, `cfg_reg_i`/`cfg_hi_i`/`cfg_data_i` in cycles with
   `cfg_we_i = 0`, and `cmd_data_i` in cycles with `cmd_i != 2`: every
   output is equal in every cycle (and so is every flop).
4. **Idle stability.** If `busy_o = park_full_o = 0` and `park_we_i = 0` in
   cycle t, then `busy_o = park_full_o = 0` in t+1, and
   `res_valid_o(t+1) = res_valid_o(t) && !res_ack_i(t)`.
5. **Park.** `park_full_o` rises only in the cycle after a `park_we_i`; it
   stays 1 in the cycle after the park (promotion is at the end of t+1 at
   the earliest); in a cycle with `park_full_o = 1`, `busy_o = 0`,
   `run_i = 1` and no `abort_i` it falls at the end of that cycle (an idle
   lane promotes at once); otherwise it falls only at the end of a cycle with
   `busy_o = 1` (a completion), `abort_i` or `run_i = 0`.
6. **Promotion is exactly** `PROMOTE(p)` (composition: `field.md` property
   1 with `cu_done_o`).
7. **No result invented, none duplicated, order kept.** Each `res_valid_o`
   episode (and each result loaded in a `res_ack_i` cycle) matches a distinct
   earlier park of a `FIELD` or `RAW SAMPLE` with destination other than
   `NONE`, with `res_dst_o` equal to that destination, in park order, and none
   matches a park discarded by `abort_i` or halt before it was promoted.
   `res_dst_o`, `res_data_o`, `res_ts_o` are stable from the rise of
   `res_valid_o` to the end of the cycle with `res_ack_i`. (Cover: a result
   for each such park in a bounded run.)
8. **`dst_busy_o`** equals the formula of 7.6 (checker shadows the parked
   and current destinations from `park_ctl_i`).
9. **`M`** changes only at the end of a cycle with `cmd_i = 2` or with
   `busy_o = 1`; if no `EXPECT` (`park_op_i = 0`, `park_ctl_i[27:26] = 3`) has
   been parked since the last `cmd_i = 2` (or reset), it changes only at a
   `cmd_i = 2`. (Exactly at the `EXPECT`'s completion: `field.md` 14.)
10. **`busy_o`** stays 1 from the start of a BAUD field to the end of its
    last bit: with the clock output enabled and no clock delay, it does not
    fall before the clock has made its end-of-bit change (with the clock
    delayed, the change is requested before `busy_o` falls and lands up to D
    cycles later; a halt in between cancels it).
11. **Abort.** After `abort_i` in cycle t: `park_full_o(t+1) = 0`;
    `sr_o(t+1) = sr_o(t)`; `crc_o(t+1) = crc_o(t)`; `res_valid_o(t+1) =
    res_valid_o(t)`; `out_val_o(t+1) = out_val_o(t)` and `out_oe_o(t+1) =
    out_oe_o(t)`, except `out3` when `stretch_hold` was holding it (then
    released).
12. **Listener.** Let E be the parks with (`park_op_i = 2` and
    `park_ctl_i[27:24] & park_ctl_i[19:16] != 0`), or (`park_op_i = 0` and
    `park_ctl_i[27:26]` in {0, 2}), or (`park_op_i = 1` and
    `park_ctl_i[27] = 1`), plus `SETM` writes of `TICK` with bit 7 set. If no
    E event has occurred since reset, `out_oe_o = 0`. (This is `top.md` T9's
    hardware half; `SAMPLE`/`EXPECT` only ever release outputs.)
13. **Events reach the sequencer one cycle after they fire**: `ev_pend_o[k]`
    rises only in the cycle after a detector-k fire (`events.md`) or, for
    k = 3, after `u_stuff.violation_o` with `STUFF[5]` (composition:
    `events.md` property 5). At the lane ports: it rises only after a cycle
    with `run_i = 1`, and falls only after `ev_clr_i[k]` or `run_i = 0`.
14. **Structural:** no combinational path from any input except `run_i` to
    any output; from `run_i` only to `out_oe_o`.
15. **Covers:** gap-free back-to-back fields in BAUD, EDGE and RECOVER; a
    timer hold at end of bit; an overrun `HOLD` and its release; abort of a
    current and a parked op in one cycle; a result loaded in the cycle of
    the previous `res_ack_i`; `pk_srcdep` deferring a promotion; `PIN` after
    a BAUD field taking effect at the end of the last bit; halt during a
    field.

## 10. Worked timing: hand-over between parked fields

Lane 0 (X cycles at t = 3 mod 5). BAUD, prescale 1, `PERIOD = PERIOD2 = 3`,
clock output enabled (capture rising: launch level 0, capture level 1),
`rest = 0`, idle 0. Field A = `DRIVE 2, msb, IMM 0b10`, field B = `DRIVE 1,
IMM 1`, both destination `NONE`. A is parked at t = 8, B at t = 13. Column
"out0/out3" shows the pin levels (the output registers) in that cycle.

| t | Sequencer strobe | `pk_v` | current | Tick | Field action | `busy_o` | out0/out3 |
|---|---|---|---|---|---|---|---|
| 8 | `park_we` A | 0 | - | timer stopped | | 0 | 0/0 |
| 9 | | 1 | - | | `PROMOTE`: A current from 10 | 0 | 0/0 |
| 10 | | 0 | A (s) | `lt` (timer was stopped) | bit 1 launched: `tx_go`, `tx_bit = 1`; clock request 0 | 1 | 0/0 |
| 11 | | 0 | A | phase 0 | | 1 | 1/0 |
| 13 | `park_we` B | 0 | A | `cte`, `ct` (10 + 3); clock request 1 | `ct`: bit captured (rotate `SR`), 1 of 2 | 1 | 1/0 |
| 14 | | 1 | A | phase 1 | | 1 | 1/1 |
| 16 | | 1 | A | end of bit (13 + 3), `launch_req`: `lt`; clock request 0 | bit 0 launched, `tx_bit = 0` | 1 | 1/1 |
| 17 | | 1 | A | | | 1 | 0/0 |
| 19 | | 1 | A | `ct`; clock 1 | 2 of 2: A completes (`cu_done`), `PROMOTE`: B current from 20 | 1 | 0/0 |
| 20 | | 0 | B (s) | phase 1 | | 1 | 0/1 |
| 22 | | 0 | B | end of bit: `launch_req` (B): `lt`; clock 0 | B bit launched, `tx_bit = 1` | 1 | 0/1 |
| 23 | | 0 | B | | | 1 | 1/0 |
| 25 | | 0 | B | `ct`; clock 1 | B completes; nothing parked | 1 | 1/0 |
| 26 | | 0 | - | phase 1 (timer still running) | | 1 | 1/1 |
| 28 | | 0 | - | end of bit, no `launch_req`, no `fld_next`: `bt_stop`; clock request idle (0) | | 1 | 1/1 |
| 29 | | 0 | - | stopped | | 0 | 1/0 |

Bits are 6 cycles each with no gap (out0 changes at 11, 17, 23). `busy_o`
stays 1 until the clock's end-of-bit change at 28. If an `abort_i` came at
t = 18 instead (a lane-0 X cycle), A would be abandoned before its last
capture and B discarded at the end of 18, the timer stopped, out0 and out3
would keep 0 and 0, `SR` would keep A's partial value (`0b01`, rotated
once), and `busy_o` would be 0 from 19. (After 19, `SR` holds B's source:
the load is at promotion.)

## 11. Flop estimate (against about 700)

| Block | Flops | Main items |
|---|---|---|
| `u_cfg` mode registers | 198 | table 5.1 (`THR1`, `THR2` deleted, ruling 23) |
| `u_field` | 199 | parked op 63, current op 30, `SR` 32, bit counters 12, flags 8, result 53 (incl. 16 timestamp), `M` 1 |
| `u_crc`, `u_stuff` | 37 | 32 + 5 (existing RTL) |
| `u_tick` | 80 | prescaler 9, BAUD/RECOVER/stretch counter 16, edge interval 16, operation timer 28, sample delay 3, state 8 |
| `u_dly` | 15 | pending flag, 5-bit payload, capture flag, 8-bit counter |
| `u_code` | 7 | NRZI levels 2, held line and previous level 2, Manchester phase 1, second half 2 |
| `u_evt` | 40 | 4 x (6-bit window, active, done, level) + 4 pending |
| `u_pins` | 8 | values and enables |
| top | 5 | `in_q` 4, `crcw_q` 1 |
| **Total** | **589** | |

Against the brief's 700 this leaves about 111. Like for like with the probe
(698 flops, of which 116 per lane of program state moved to the sequencer,
`top.md` 1) the lane is about 7 over (589 vs 582). Options, cheapest first:
share the
operation timer's 16-bit counter with the BAUD counter outside RECOVER
(-16, costs a rule that a `WAIT` timeout in RECOVER is not exact); trim the
current-op register to the decoded fields (-4); latch-based mode registers
(arch 11 reserve, about -18 % area).

## 12. Decisions and ISA issues

Each item: the affected text, what this spec does, proposed wording. The
first group affects rung 1. Model decisions (D) and findings (F) refer to
`toolchain/MODEL_DECISIONS.md`.

**I1. BAUD sample point (F3, overseer ruling).** isa 2 `BAUD` capture tick:
"`PERIOD` cycles later; the next launch is `PERIOD2` cycles after that".
Implemented: the capture tick (clock change to the capture level) is at
`PERIOD`; the data input is sampled `RTRIP = 4` cycles later, when the
lane's own clock edge has returned through the output register, the pin and
the 3-cycle input path; the next launch may happen before that sample (one
sample pending at most). With `stretch_honour` the sample is at the capture
tick (the clock input already has the same latency). With the clock delayed
(I22) the sample follows the delayed edge. Proposed: "In `BAUD` the data
input is sampled when the lane's own capture edge has come back through the
input path, 4 cycles after the capture tick (at the capture tick with
`stretch_honour`). The next bit may launch before that sample is taken."
Consequence: 3+3 SPI reads work within a field; between two sampling fields
the clock's high phase stretches to 5 cycles (7.3).

**I2. When a field finishes (F1, D30, overseer ruling).** isa 3.3 "`PIN`
waits for the current field to finish." isa 2 "A **field instruction**
occupies the lane's bit hardware for its whole duration." Implemented as
7.3. Proposed: "A field completes at its last capture: its result, `M` and
`SR` are then available. In `BAUD` the bit hardware stays busy until the
end of the last bit, `PERIOD2` after the last capture; a parked field
launches exactly then and a parked `PIN` takes effect then. In `EDGE`,
`RECOVER` and `PULSE` receive it is free at the last capture."

**I3. `early` (F2, overseer ruling).** isa 3.1 "`early`: put the first bit
on the wire when the field starts, not at the first launch tick". Implemented
for `EDGE` and `RECOVER` only (`field.md` 5.1). Proposed addition: "In `BAUD`
a field always starts with a launch, so `early` has no effect." Remove
`early` from the section 5 SPI controller example.

**I4. Clock output in EDGE (F4, overseer ruling).** isa 2 "With the
**clock output** enabled, `out3` goes to the launch level at the launch tick
and to the capture level at the capture tick." Proposed: insert "In `BAUD`,"
at the start, and add "In `EDGE` the clock output is used only by
`stretch_hold` and otherwise rests at its idle level."

**I5. `stretch_hold` (F5, D11).** isa 2 "In `EDGE` mode with
`TICK.stretch_hold` the lane holds the clock line low while it waits."
Proposed: "At a launch edge of the clock input that arrives while no
operation is current or parked and the sequencer is waiting for queue or
channel data, the lane drives the clock output low. When the next field
starts it launches its first bit at once and releases the clock `PERIOD`
prescaled cycles later (the data setup time). An event releases the clock
at once." A target sleeping in `WAIT` never stretches. The condition is
sampled only in the cycle of the launch edge: the compiler's schedule check
must make the sequencer reach its waiting instruction before the edge.

**I6. `stretch_honour` (F6, D10).** isa 4 "`stretch_honour`: in `BAUD`,
pause while the clock input reads low after release". Proposed: "the
capture tick is the first cycle after the clock output is released in which
the clock input reads the capture level; the data input is sampled then and
`PERIOD2` counts from then." The high phase on the pin is therefore
`PERIOD2 + 4` cycles without stretching; the compiler accounts for it.

**I7. `PIN` timing (F20, D31, S5).** isa 3.3 "`align`: take effect at the
next launch tick instead of at once" and "then wait this many `PERIOD`
intervals". Implemented (`field.md` 7, `pins.md`): in `BAUD` and `PULSE` a
`PIN` takes effect when the BAUD timer stops, at once if it is stopped,
`align` or not; in `EDGE` and `RECOVER` at once, or with `align` at the first
launch tick at or after its start. The next operation starts exactly
`wait x PERIOD` prescaled cycles after the `PIN`'s change. Proposed:
replace the `[15]` row by "take effect at the next launch tick (`EDGE`,
`RECOVER`); in `BAUD` a `PIN` always waits for the end of the current bit",
and add "The next operation starts `wait x PERIOD` prescaled cycles after
this one's outputs change. `PIN` writes values and enables as encoded,
without invert, open-drain or differential mapping."

**I8. Prescaler (F24; differs from D34).** isa 2 "The **prescaler**
divides the clock by 1, 8, 64 or 512." Implemented: one free-running 9-bit
counter per lane, held at 0 while halted; every prescaled duration counts
its strobes (notation of section 1). Exact at prescale 1; at prescale P a
duration of D prescaled cycles lasts between `(D-1)P + 1` and `DP` cycles.
The model restarts a prescaler per duration (about 27 more flops). Every
rung-1 and rung-2 protocol runs at prescale 1 except UART below 600 baud.
Proposed: "Prescaled durations count the strobes of one free-running
prescaler started with the lane, so they are exact at prescale 1 and may
start up to one prescaled cycle early otherwise."

**I9. Capture timestamp (overseer ruling; changes `sequencer.md` 5.1 and
decision 6).** isa 3.0 `CAP`: "push timestamp word, then value word".
Implemented: new ports `ts_i[15:0]` and `res_ts_o[15:0]`; the lane latches
the counter's low half in the completion cycle; the sequencer rebuilds the
high half (7.5). Proposed: "The timestamp is the counter value in the cycle
the field completed (its last capture), less the fixed input delay that the
compiler subtracts." In `BAUD` without honour the completion is the last
sample, a fixed RTRIP (plus any clock delay) after the last capture tick;
the compiler subtracts that too, so ruling 12's exactness holds. A result
held back by an overrun `HOLD` carries the time of its late completion
(overrun is a schedule error).

**I10. RECOVER (D5, F24).** isa 2 "half a `PERIOD` after it; any data edge
restarts the bit timer". Proposed: "The bit timer runs continuously. A data
edge is a launch tick and restarts it. The capture tick is `floor(PERIOD /
2)` prescaled cycles after each launch tick, and after `PERIOD` the timer
wraps, which is another launch tick. A receiving field takes every capture
tick while it is current, so one that starts before the middle of a bit
receives that bit. `PERIOD` below 2 acts as 2."

**I11. PULSE receive (D12, F25).** isa 2 `PULSE`: "the symbol is the time
since the previous edge compared with `THR0`-`THR2`"; `CODE[1:0] = 3`
"pulse symbol". Implemented: one bit per high pulse, captured at its falling
edge, `1` iff the pulse lasted at least `THR0`; Manchester receive uses
`THR0` as the short/long boundary. `THR1` and `THR2` then have no use in the
data path. Proposed: say so, and either delete `THR1`/`THR2` (16 flops) or
define 2-bit symbols. Recommend deleting. **Ruling 23: deleted**;
registers 3 and 4 keep only `THR0` and the launch delay (table 5.1).

**I12. Manchester (arch 4.2).** Implemented (`coding.md`): transmit first
half `~bit`, second half `bit` (IEEE 802.3; `invert` gives the other
convention), the second half at the capture time, so `PERIOD` and `PERIOD2`
are the two half-bit times. Receive only in `PULSE` mode: a long interval
(`>= THR0`) or the second of two short ones is a mid-bit edge and gives the
new level as the bit. Proposed: add these two sentences to isa 4 `CODE`.

**I13. Differential input (F26; differs from the model, which gives it no
effect).** isa 4 `CODE[9]` "differential input". Implemented: the
complementary input is `in[data_input ^ 1]`; while the two are equal (USB
SE0 or SE1) the data line holds its last valid level, makes no edge, and no
bit is captured. Without this, a USB receiver captures one bit of SE0 into
the CRC before the EOP event can abort the field (T6). Proposed: "With
differential input, `in[data ^ 1]` is the complementary line. While both
lines are equal the data input holds its last valid level and no bit is
captured." **Ruling 20 accepted.** A BAUD-timed `SAMPLE`/`EXPECT` whose
sample falls on an invalid pair also gives back that launch (`field.md`
5.3), so the timer makes one more clock pulse and the field still gets n
bits. `EXCHANGE` takes every sample as a bit, because its bit has already
been sent and a dropped sample would leave it short with nothing left to
launch; differential input with `EXCHANGE` is therefore not supported (no
protocol needs it). Overseer: confirm this limit.

**I14. Invert, open-drain, differential output (F26).** isa 4 `CODE[6]`,
`[7]`, `[8]`. Proposed: "`invert` applies to receive and transmit.
Open-drain: a 0 drives low and a 1 releases. Differential: `out1` carries the
complement of `out0`'s level, with the same mapping and enable." A
receiving field releases both outputs (I25).

**I15. NRZI (D35).** Proposed: "NRZI as in USB: a 0 is a change of level, a
1 no change. Transmit continues from the last level written to `out0` by a
field, `RAW DRIVE` or `PIN`. Receive compares with the level at the previous
sample tick, whether or not a field was running; initially 1."

**I16. `SR` and the CRC feed (D36, D18, F14).** isa 1 "`SR` ... the field on
the wire; readable after the field". Proposed: "After a field `SR` holds its
value, right-aligned and zero-extended: the driven value after `DRIVE`, the
sampled value after the other kinds. After an abandoned field it holds the
partial state. The check unit takes data bits only (never stuff bits): the
driven bits for `DRIVE` and `EXCHANGE`, the sampled bits for `SAMPLE` and
`EXPECT`, at each bit's capture."

**I17. Event configuration (F8, F9, D20, D21, overseer ruling).** isa 4
`EVC`: "`[1:0]` signal A; `[2]` level L; `[4:3]` trigger (0 A at level L, 1
A edge to level L, 2 A any edge, 3 A and B both low); `[6:5]` signal B;
`[8:7]` B must be (0 anything, 1 high, 2 low); `[15:9]` K". Proposed:
"`[1:0]` A; `[2]` L; `[5:3]` trigger (0 off, 1 A at level L, 2 A edge to
level L, 3 A any edge, 4 A and B both low); `[7:6]` B; `[9:8]` B must be (0
anything, 1 high, 2 low); `[15:10]` K." Semantics in `events.md`: windows,
one fire per episode for triggers 1 and 4, a write restarts the detector.
**Ruling 21:** trigger 5-7 and B-must 3 act as "off" and "never"; the
sequencer does not decode them.

**I18. K range.** isa 4: "held for K more prescaled cycles". With 6 bits,
K <= 63 prescaled cycles: 1.05 us at prescale 1. Enough for I2C (K = 18),
USB EOP (one bit = 40) and 10BASE-T TP_IDL (9); not for USB bus reset
(T5). The lane prescale is shared with the bit timing.

**I19. Stuffing scope (F16, D19, overseer note).** isa 3.1 "`[18]` apply
bit stuffing". Proposed: "Only fields with `[18]` feed the stuffing unit; its
run count carries across them and is cleared by `STUFFCLR`. A stuff bit due
after a field's last data bit belongs to that field. `EXCHANGE` stuffs the
transmitted stream and discards the bit sampled at a stuff position.
Receive destuffing is not available with `BAUD` timing. `STUFF[5]` raises
event 3 in addition to `EVC3`'s detector (which can be turned off)."

**I20. `seek` (F13).** isa 3.1 "`EXPECT` with `seek` keeps shifting past the
field length until the last `n` bits equal the source". Implemented for
`n <= 16` (the pattern is kept in `SR[31:16]`, no extra flops); larger `n`
acts as 16. Proposed: append "(`n` up to 16; source `IMM` cannot seek because
the immediate uses the `seek` bit)". **Limit (review, 14 Nov):** with `BAUD`
timing and no honour, the bit after the matching one is already launched
when the end of a bit comes at or before its sample. That clock pulse then
belongs to the next operation: a `DRIVE` ignores it, an `EXCHANGE` discards
its sample, a `SAMPLE`/`EXPECT` takes it as its first bit. Compiler rule:
seek in `BAUD` needs `stretch_honour` or `(P2d - 1) * prescale + 1 >
RTRIP` (plus any clock delay), unless the next field is meant to take that
bit.

**I21. `SETM` side effects (D8, D9, D42, F21).** isa 3.4 gives none.
Proposed: "Writing `TICK` stops the timers and ends stretching; with the
clock output enabled `out3` goes to its idle level. Writing `CODE` resets the
Manchester receive phase. Writing `STUFF` clears the stuffing history.
Writing `CRCPOLY` or `CRCCFG` initialises the check unit. Writing `EVCk`
restarts detector k. Writing the upper half of a 16-bit register does
nothing."

**I22. Launch delay (D13, G9).** isa 4 `THR2L[15:8]` "launch delay",
`CODE[10]` "launch delay applies to the clock output", `[11]` "launch delay
in prescaled cycles". Proposed: "The delay postpones every data-output
change made by a field, or with `CODE[10]` instead every clock change made by
the `BAUD` timer. One change can be pending: a new one applies the pending
one at once, so the delay must be shorter than the spacing of the changes it
delays. `PIN`, `RAW`, `stretch_hold` and `SETM` changes are never delayed,
and a `PIN` cancels a pending change of an output it writes. With the clock
delayed, `BAUD` samples 4 cycles after the delayed capture edge."

**I23. What an event abandons (F29, D23).** isa 3.6 "When an armed event
fires: the current field is abandoned and `PC = target`. The CRC, the
stuffing run count, the shift register and the number of bits done are kept.
Outputs keep their levels." Proposed addition: the list in 7.4 (parked op
discarded; BAUD timer stopped with the clock at its level; stretching
released; pending delayed and second-half changes cancelled; the cycle of the
abort makes no progress).

**I24. `RAW` (F12, D40).** isa 3.2. Proposed: "With source `IMM` the
duration is `[8:0]` and there is no tag. The duration is the low 16 bits of
the source; 0 acts as 1. `RAW DRIVE` applies invert, open-drain and
differential mapping, is never delayed, holds the bit hardware for its
duration (in `BAUD` it starts at the end of the current bit) and has no
result. `RAW SAMPLE` measures between consecutive edges of the data input,
so back-to-back `RAW SAMPLE`s measure consecutive intervals. Afterwards `SR`
holds the duration (`RAW DRIVE`: its source) or the measured value (`RAW
SAMPLE`)."

**I25. Receiving fields release the data output (D14).** Not in isa.
Proposed: "`SAMPLE` and `EXPECT` release `out0` (and `out1` when
differential) at their first launch tick, or when they start in `RECOVER`
and `PULSE` modes or with `early` in `EDGE`. A controller that reads with `SAMPLE`
leaves its data output released; use `EXCHANGE` to keep driving it."

**I26. Start latency and the result register (sequencer 7).** An op parked
in X cycle t starts at t+2. Results go through one register; an overrun
holds the op (7.5) rather than losing data. Proposed for isa 2: "A field
that is parked when the previous one completes starts without a gap; a
result that cannot be delivered holds the next field back."

**I27. The `PERIOD` unit (F30).** isa 4 `PERIOD`: "the unit for `PIN` and
`WAIT`". In `BAUD` it is half a bit; in `EDGE` it is also the
`stretch_hold` setup time. Proposed: say both.

**I28. I2C controller and a stretching target at STOP or repeated START.**
isa 3.3 has no way to wait for an input. A `PIN` that releases SCL counts
its wait from the release, not from SCL reading high, so a target still
stretching after the last ACK shortens `t_su_sto`/`t_su_sta`. Workaround in
the current ISA: an event "A = SCL at level high" and `WAIT` on it before
the timed `PIN`. Possible fix (not adopted, a few gates): `PIN[14]` "start
counting when the clock input reads 1".

**I29. Section 5 examples.** UART receive: source `R0` on `SAMPLE` should be
`NONE`; `WAIT E0` needs `EVT 0, off, clear` before it (F7, sequencer
decision 7); `EVC0` "edge to low" is now trigger 2. SPI controller: drop
`early`; CS is undriven until the first `PIN`; `cs_lag` needs a `PIN` wait
before CS rises, because CS now rises in the same cycle as the final SCK
change. I2C target: needs `CODE` (open-drain data, `data input = in0`), and
the address needs `HOSTQ` plus `MOVSR` (a tag on a register destination does
nothing); the `EVC` encodings change with I17.

**I30. `busy_o` includes the BAUD timer.** Not an isa change, but the
sequencer's `IDLE` (sync instructions, `HALT`, a `WAIT` timeout start) now
waits for the end of the last bit, not for the last capture. That is what
D25 wants for `HALT`; the cost is up to `PERIOD2` more cycles before a sync
instruction.

### 12.1 Threats to 10 Mbit Ethernet and low-speed USB

- **T1. Ethernet receive margin.** At 3 cycles per half-bit the edge
  intervals measure 2-4 (short) and 5-7 (long) cycles with one cycle of
  quantisation; `THR0 = 5` separates them with no margin for analogue jitter
  above about half a cycle (8 ns). This is arch open question 3; nothing in
  the lane can widen it without sampling on both clock edges.
- **T2. Ethernet receive sequencing.** Mid-bit edges are at least 4 cycles
  apart, and the next field is current 1 cycle after the previous completes
  (2 with source `SR`/`CRC`), so field boundaries keep up. Each 8-bit field
  needs about 3 slots (delivery, `FIELD`, `LOOP`) of its 48 cycles; 32-bit
  fields to `MEM` are better. An overrun `HOLD` (full queue) loses wire bits,
  because PULSE captures cannot be held back.
- **T3. Ethernet transmit.** Manchester at 3+3 with `DRIVE` back to back is
  gap-free; `DRIVE CRC` after the payload needs `P2d >= 2` (it is 3). The
  next field must be parked within the previous one (48 cycles per byte).
  TP_IDL via `PIN` takes effect exactly at the end of the last bit.
- **T4. Link pulses** are host-scheduled (language 4.2); no lane impact.
- **T5. USB bus reset** (SE0 for at least 2.5 us = 150 cycles) cannot be one
  event at prescale 1 (K <= 63). It needs software: the EOP event followed by
  a `WAIT` timeout and a re-check. Workable, costs program words.
- **T6. USB SE0 and the CRC residue** depend on I13. Without it, the first
  capture inside SE0 (20 cycles after its start) feeds a bit to the CRC,
  while an EOP event with K below 20 still reaches the lane as `abort_i`
  2-6 cycles after it fires.
- **T7. USB transmit** EOP via `PIN` starts exactly at the end of the last
  bit (I7), so the last bit is not lengthened (one cycle would be 2.5 % of a
  bit). Turnaround needs `SETM TICK`, which waits for the end of the last
  bit; 7.5 bits is ample.

### 12.2 Cross-spec review corrections (14 Nov)

Found by a cold review of the six lane specs against each other, the
rulings and the fixed `crc.md`/`stuff.md`; all are applied in the specs.

- **R1.** `EXCHANGE` with a trailing stuff bit in `BAUD`: the stuff bit can
  be launched before the last data bit's sample; the field then completed
  with a wire bit still launched, and the next field took its capture.
  `EXCHANGE` now completes only when no launched bit is outstanding
  (`field.md` 6.1), and a capture with nothing launched makes its sample a
  discard (`field.md` 5.3).
- **R2.** A timed op whose count ends at the first strobe after its effect
  cycle completes in that cycle: `ot_done` may be combinational from
  `ot_start` (`tick.md` 7), and `field.md` 6.1 completes a `PIN`/`RAW DRIVE`
  on `ot_done` in its effect cycle (it required `eff`, which is set only at
  the end of e).
- **R3.** `early` released a receiving field's data output in `BAUD`
  (ruling 6: `early` is `EDGE`/`RECOVER` only). Fixed in `field.md` 5.1.
- **R4.** Ruling 20 in `BAUD` receive (I13): the sample is not a bit and its
  launch is given back (`field.md` 5.3).
- **R5.** Ruling 23: `THR1`, `THR2` removed (5.1, 11, I11); `THR0` is used by
  `u_tick` only.
- **R6.** Gap-free conditions restated in raw cycles; at prescale P the old
  `P2d * prescale` form was optimistic by up to P - 1 cycles (7.3).
- **R7.** Tags apply only to `HOSTQ`/`CAP` results (7.5, as `field.md` 6.2
  and isa 3.1); `MEM` marks `R3` for current and parked ops too (7.6).
- **R8.** High-half `SETM` writes have no side-effect pulses except on
  `CRCPOLY` (5.2).
- **R9.** A `PIN` cancels a pending launch-delay change only if it writes an
  output that change writes (`tick.md` 9.2; it cancelled a pending `out0`
  change for a `PIN` that wrote only `out1`).
- **R10.** `PULSE` with a transmitting field uses the BAUD sample rule
  (`tick.md` 5 said `BAUD` only).
- **R11.** Observation outputs for proofs: `rosetta_field.cu_done_o`,
  `hold_o`; `rosetta_tick.stretch_o`. Block assumptions on `cfg_` inputs
  aligned with when the sequencer really writes them (idle lane), and
  properties that named internal flops restated on ports (section 9,
  `tick.md` 10, `field.md` 9, `coding.md` 5).
