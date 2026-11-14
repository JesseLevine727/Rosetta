# Block spec: `rosetta_field` (operation unit and field execution)

Status: draft for review (Phase 0, 14 Nov). Part of the lane (`lane.md`);
notation in `lane.md` 1. The lane-port view of the handshake is `lane.md` 7;
this spec gives the internal rules that produce it.

`rosetta_field` holds the parked and current operations, promotes, executes
`FIELD`, `RAW` and `PIN` operations against the strobes of `rosetta_tick`,
owns `SR`, the bit counters, the result register and `M`, and drives the
check unit and the stuffing unit.

## 1. Ports

| Port | Width | Meaning |
|---|---|---|
| `clk`, `rst_n` | 1 | clock; async active-low reset |
| `run_i`, `abort_i` | 1 each | lane live; sequencer abort |
| `park_we_i`, `park_op_i[1:0]`, `park_ctl_i[27:0]`, `park_data_i[31:0]` | | park strobe and shared buses (`lane.md` 2.1) |
| `res_ack_i` | 1 | result delivered |
| `cmd_i[1:0]`, `cmd_data_i[31:0]` | | only `CRCCHK` (2) is used here |
| `ts_i` | 16 | timestamp, low half |
| `cfg_mode_i` | 2 | `TICK[1:0]` |
| `lt_i`, `ct_i`, `st_i`, `spend_i`, `bt_run_i`, `bt_stop_i`, `ot_done_i` | 1 each | from `rosetta_tick` |
| `cfg_honour_i` | 1 | `TICK[11]` (whether BAUD samples are delayed) |
| `d16_i` | 16 | edge interval |
| `dedge_i`, `rxb_i`, `rxv_i` | 1 each | from `rosetta_coding` |
| `stuff_next_i`, `stuff_bit_i` | 1 each | from `rosetta_stuff` |
| `crc_state_i`, `crc_value_i` | 32 each | from `rosetta_crc` |
| `park_full_o`, `res_valid_o`, `res_dst_o[3:0]`, `res_data_o[31:0]`, `res_ts_o[15:0]`, `dst_busy_o[3:0]`, `m_o`, `sr_o[31:0]` | | lane outputs (`lane.md` 2.3) |
| `cu_v_o`, `cu_fld_o`, `cu_tx_o`, `cu_smp_o`, `pk_v_o`, `launch_req_o`, `fld_next_o`, `fld_first_o`, `ot_start_o`, `ot_raw_o`, `ot_n_o[15:0]` | | to `rosetta_tick` (`lane.md` 6.1) |
| `tx_go_o`, `tx_bit_o`, `tx_rel_o`, `raw_go_o`, `raw_lvl_o` | 1 each | to `rosetta_coding` |
| `pin_we_o`, `pin_mask_o[3:0]`, `pin_val_o[3:0]`, `pin_oe_o[3:0]` | | `PIN` effect, to `rosetta_pins`, `rosetta_delay`, `rosetta_coding` |
| `crc_en_o`, `crc_bit_o` | 1 each | to `rosetta_crc` |
| `stf_valid_o`, `stf_bit_o` | 1 each | to `rosetta_stuff` |
| `cu_done_o`, `hold_o` | 1 each | the current op completes in this cycle (6.1, combinational); the `hold` flop. For proofs and covers only (not used by other blocks) |

`busy_o = cu_v_o || bt_run_i` is formed in the lane top.

## 2. State

| Flops | Width | Reset | Meaning |
|---|---|---|---|
| `pk_v`, `pk_op`, `pk_ctl`, `pk_data` | 1, 2, 28, 32 | 0 | parked operation |
| `cu_v`, `cu_op`, `cu_ctl` | 1, 2, 28 | 0 | current operation (decoded below) |
| `first` | 1 | 0 | this is the current op's start cycle |
| `eff` | 1 | 0 | a `PIN` or `RAW DRIVE` has taken effect (its timer runs) |
| `hold`, `tail` | 1 each | 0 | overrun hold (`lane.md` 7.5); receive-stuffing tail check (5.6) |
| `SR` | 32 | 0 | shift register |
| `bc` | 6 | 0 | data bits captured in the current field |
| `nl` | 6 | 0 | data bits launched (transmit kinds) or launches (BAUD-timed receive) |
| `lnch` | 1 | 0 | the current wire bit has been launched and not yet captured |
| `lstf`, `pstf` | 1 each | 0 | the launched wire bit is a stuff bit; the pending BAUD sample is for a stuff bit |
| `rel` | 1 | 0 | a receiving field has released the data output |
| `mm` | 1 | 0 | `EXPECT` mismatch seen |
| `res_v`, `res_dst`, `res_data`, `res_ts` | 1, 4, 32, 16 | 0 | result register |
| `M` | 1 | 0 | match flag |

About 199 flops (`cu_ctl` may be trimmed to the decoded fields).

### 2.1 Decoded fields of an operation `o` (`pk` or `cu`)

`FIELD` (`op = 0`): `kind = ctl[27:26]` (0 `DRIVE`, 1 `SAMPLE`, 2
`EXCHANGE`, 3 `EXPECT`); `n = ctl[25:21] + 1`; `lsb = ctl[20]`;
`crcf = ctl[19]`; `stf = ctl[18]`; `src = ctl[17:14]`; `dst = ctl[13:10]`;
`early = ctl[9]`; `seek = (src != IMM) && ctl[8]`; `tag = (src != IMM) ?
ctl[7:0] : 0`. `TX = kind in {DRIVE, EXCHANGE}`, `SMP = kind != DRIVE`,
`RX = kind in {SAMPLE, EXPECT}`.

`RAW` (`op = 1`): `drv = ctl[27]`; `level = ctl[26]`; `src = ctl[17:14]`;
`dst = drv ? NONE : ctl[13:10]`; `tag = (src != IMM) ? ctl[7:0] : 0`.

`PIN` (`op = 2`): `mask = ctl[27:24]`, `val = ctl[23:20]`, `oe =
ctl[19:16]`, `align = ctl[15]`, `wait = ctl[11:0]`.

Derived for the current field:

- `bt` (BAUD-timed) = `cfg_mode_i == BAUD || (cfg_mode_i == PULSE && TX)`.
- `stx = stf && TX` (transmit stuffing); `srx = stf && RX && !bt` (receive
  destuffing; not available with BAUD timing, `lane.md` I19).
- `n' = min(n, 16)` (seek window).
- `opos0 = lsb ? 0 : n-1` (the oldest bit not yet shifted).
- `shp = spend_i && !pstf` (a BAUD data sample is pending: the next bit to
  send is one position further on). `opos = lsb ? (shp ? 1 : 0) : (shp ?
  n-2 : n-1)`.

`G = run_i && !abort_i`. In a cycle with `!G` every output strobe of this
block (`tx_go`, `tx_rel`, `raw_go`, `pin_we`, `crc_en`, `stf_valid`,
`ot_start`) is 0, no completion happens and no state changes except as
section 8 says.

## 3. Parking and promotion

- **Park**: at the end of a cycle with `run_i && park_we_i && !abort_i`:
  `pk_v <= 1`, `pk_op <= park_op_i`, `pk_ctl <= park_ctl_i`,
  `pk_data <= park_data_i`. No other rule reads the shared buses.
- **Promote** at the end of cycle p iff `PROMOTE(p)` (`lane.md` 7.2):
  `run_i && pk_v && !abort_i && (!cu_v || cu_done) && !(pk_srcdep &&
  cu_v)`, where `cu_done` is section 6.1 and `pk_srcdep` = `pk` is a
  `FIELD` or a `RAW DRIVE` with `src` in {`CRC`, `SR`}. Then: `cu <= pk`,
  `cu_v <= 1`, `pk_v <= 0`, `first <= 1`, `eff <= hold <= tail <= 0`,
  `lnch <= lstf <= pstf <= rel <= mm <= 0`, `bc <= nl <= 0`, and `SR` is
  loaded (4.1).
- `first` is cleared at the end of every cycle in which it is 1 (unless a
  promotion sets it again).
- If `cu_done` and no promotion at the end of the same cycle: `cu_v <= 0`.

## 4. Start

### 4.1 `SR` load at promotion

`v = (src == CRC) ? crc_value_i : (src == SR) ? SR : pk_data` (values in
cycle p; with `pk_srcdep` the previous op completed at least one cycle
earlier, so these are final).

| Promoted op | `SR` at the end of p |
|---|---|
| `FIELD`, not seek | `v & mask(n)` |
| `FIELD`, seek | `{v[15:0] & mask(n'), 16'h0}` (pattern in the top half, empty window) |
| `RAW DRIVE` | `v` (its duration is `SR[15:0]`) |
| `RAW SAMPLE`, `PIN` | unchanged |

Source `CRC` is `value_o` (final XOR applied), sent in the field's own bit
order; to send "bit W-1 first" the compiler uses `msb` with `n = W` (model
D39).

### 4.2 Start cycle

The op is current from `s = p + 1` (`first = 1` in s). `fld_first_o =
first && cu_op == FIELD`. Ticks in cycle s apply to the new op.

## 5. `FIELD` execution

All rules in this section hold in cycles with `G` and the current op a
`FIELD` with `hold = 0`.

### 5.1 Launch

**Transmit kinds** (`DRIVE`, `EXCHANGE`):

- `EL = first && early && !bt` (the `early` start launch, `EDGE` and
  `RECOVER` only; overseer ruling on F2).
- `left = (nl < n) || (stx && stuff_next_i && nl == n)` (a data bit, or a
  trailing stuff bit, is still to be sent).
- **Launch event** `LE = (lt_i || EL) && !lnch && left`. A launch tick
  while `lnch = 1` is absorbed (it changes nothing); this is how `early`
  works with the launch tick that follows it.
- At `LE`: if `stx && stuff_next_i`: wire bit `x = stuff_bit_i`,
  `lstf <= 1` (`nl` unchanged). Else `x = SR[opos]`, `lstf <= 0`,
  `nl <= nl + 1`. Always `lnch <= 1`, `tx_go = 1`, `tx_bit = x`,
  `stf_valid = stx`, `stf_bit = x`.

**Receive kinds** (`SAMPLE`, `EXPECT`):

- Release: `tx_rel = 1` and `rel <= 1` in the first cycle with `!rel` and
  (`lt_i`, or `first && ((early && !bt) || cfg_mode_i in {RECOVER, PULSE})`).
  (`early` has no effect in `BAUD`, ruling 6.)
- With `bt`: at each `lt_i`, `nl <= nl + 1` (counts the clock pulses the
  BAUD timer makes for this field); a sample that is not a bit gives one
  back (5.3).

### 5.2 Status to `rosetta_tick`

- `launch_req = cu_v && cu_op == FIELD && bt && !hold && !tail && (TX ?
  (!lnch && left) : (seek || nl < n))`.
- `fld_next = pk_v && pk_op == FIELD && pk_bt && (!cu_v || (cu_op ==
  FIELD && !launch_req))`, with `pk_bt` = `bt` evaluated for the parked
  field. So the BAUD timer holds at the end of a bit only when a BAUD-timed
  field is parked and nothing but a finishing field stands before it.
- `cu_fld = cu_v && cu_op == FIELD`; `cu_tx = cu_fld && TX`; `cu_smp =
  cu_fld && SMP`.

`launch_req` and `fld_next` are functions of flops and `stuff_next_i` only.
(Implementation note: keep `left` and `nl < n` as registered flags updated at
each launch, to keep `lt -> tx_go -> request` short.)

### 5.3 Capture and sample

`sv = st_i && (rxv_i || TX)`: a sample is a bit unless `rxv_i = 0`
(invalid differential pair, Manchester boundary, rising pulse edge); an
`EXCHANGE` sample is always a bit, because its bit has already been sent
(`lane.md` I13). In a BAUD-timed `SAMPLE`/`EXPECT`, a cycle with `st_i &&
!rxv_i` gives the launch back: `nl` is decremented (not below 0; together
with an `lt_i` in the same cycle it is unchanged), so `launch_req` asks for
one more clock pulse and the field still receives n bits (ruling 20).

**`DRIVE`**, at `ct_i` with `lnch`: `lnch <= 0`. If `!lstf` (a data bit):
`crc_en = crcf`, `crc_bit = SR[opos0]`, `SR <= ROT(SR)`, `bc <= bc + 1`.
Completion check (6.1) with the new `bc`. A `ct_i` with `lnch = 0` is
ignored.

**`EXCHANGE`**, at `ct_i`: `lnch <= 0`; and when `bt && !cfg_honour_i`
(the sample comes later): `pstf <= lstf || !lnch` (a capture with no bit of
this field launched, which only a `BAUD` seek can leave behind, `lane.md`
I20, gets its sample discarded). The sample is processed:

- `bt && !cfg_honour_i`: at `sv` (the pending sample matures): stuffness `ss =
  pstf`.
- otherwise: at `sv` in a cycle where `ct_i && lnch` (same cycle): `ss =
  lstf`. A sample without a launched bit is ignored.

Then if `ss`: the sampled bit is discarded. Else `crc_en = crcf`, `crc_bit =
SR[opos0]` (the driven bit), `SR <= INS(SR, rxb_i)`, `bc <= bc + 1`.
Completion check after either.

**`SAMPLE`, `EXPECT`**, at `sv`:

- If `srx && stuff_next_i`: a stuff position. `stf_valid = 1`, `stf_bit =
  rxb_i`; nothing else changes (the stuffing unit reports a wrong value as a
  violation). If `tail = 1`: completion.
- Else a data bit: `stf_valid = srx`, `stf_bit = rxb_i`; `crc_en = crcf`,
  `crc_bit = rxb_i`; `bc <= min(bc + 1, seek ? n' : n)`.
  - Not seek: `SR <= INS(SR, rxb_i)`; for `EXPECT` `mm <= mm || (rxb_i !=
    SR[opos0])`. If the new `bc == n`: with `srx`, `tail <= 1`; otherwise
    completion.
  - Seek: `SR[15:0] <= WIN(SR[15:0], rxb_i)`, `SR[31:16]` unchanged. If the
    new `bc >= n'` and `WIN(SR[15:0], rxb_i) == SR[31:16]` (both within
    `mask(n')`): completion.

### 5.4 Shift functions (bit order)

For width n, `msb = !lsb`:

```
INS(S, b):  msb: ({S[30:0], b}) & mask(n)
            lsb: bit i = (i == n-1) ? b : (i < n-1) ? S[i+1] : 0
ROT(S)   :  INS(S, S[opos0])          // DRIVE keeps its value
WIN(W, b):  as INS on 16 bits with n' in place of n
```

So after n data bits `SR[n-1:0]` holds the field value with the first wire
bit at `n-1` (msb) or at 0 (lsb), bits n and above are 0, and a `DRIVE` ends
with `SR` equal to its source masked to n bits (model D36). `EXPECT`
compares each sampled bit with the source bit it replaces (`SR[opos0]`), so
no second register is needed; with source `CRC` that is the CRC as it stood
at the start (isa 3.1).

### 5.5 Stuffing and the check unit

- `stf_valid` is 1 exactly for the wire bits of fields with `stf` (gating per
  field, `stuff.md` v2): at each transmit launch (data or stuff bit) and each
  receive data or stuff position. Fields without `stf` neither feed nor clear
  the run count, so it carries across stuffed fields.
- A stuff bit never shifts `SR`, never counts in `bc`/`nl`, never feeds the
  CRC.
- A stuff bit due after the last data bit belongs to the field: transmit
  launches it (`left`), receive waits for it (`tail`, 5.6). At most one, as
  a correct stuff bit restarts the run.
- `crc_en` is 1 only for data bits of fields with `crcf`: the driven bit for
  `DRIVE` and `EXCHANGE`, the sampled bit for `SAMPLE` and `EXPECT`, at the
  bit's capture (`DRIVE`) or sample (others) (model D18).
- `rosetta_stuff.violation_o` goes to `rosetta_events` (event 3 with
  `STUFF[5]`); this block does not act on it.

### 5.6 Receive-stuffing tail

The stuffing unit's `stuff_next_o` reflects a bit only from the cycle after
it is fed. So after the last data bit of an `srx` field (cycle t, `tail <=
1`): in each cycle with `tail = 1`, if `!stuff_next_i` the field completes in
that cycle; if `stuff_next_i`, it waits for the next `sv`, which is a stuff
position and completes it. A stuffed receive field therefore completes one
cycle after its last data bit, or at its trailing stuff bit.

## 6. Completion

### 6.1 When an op completes (`cu_done`)

In a cycle with `G`:

| Op | Completes in the cycle where |
|---|---|
| `FIELD` `DRIVE` | a `ct_i` processed by 5.3 leaves `bc == n` and `!(stx && stuff_next_i)` |
| `FIELD` `EXCHANGE` | a sample processed by 5.3 (data or discarded) leaves `bc == n`, `!(stx && stuff_next_i)`, and no wire bit launched but not yet captured (`lnch` is 0 at the end of the cycle: a trailing stuff bit launched before the last data sample must be captured and sampled first) |
| `FIELD` `SAMPLE`/`EXPECT`, not seek | the n-th data bit is taken and `!srx`; or `tail = 1` and (`!stuff_next_i`, or `sv` at a stuff position) |
| `FIELD` seek | the window matches (5.3) |
| `FIELD` with `hold = 1` | the result register can take the result (6.2) |
| `PIN` | `pin_we` with `wait == 0` or `ot_done_i`; or `eff` and `ot_done_i` (7.1) |
| `RAW DRIVE` | `raw_go` and `ot_done_i`; or `eff` and `ot_done_i` (7.2) |
| `RAW SAMPLE` | `dedge_i` (7.3) |

`stuff_next_i` in the `DRIVE`/`EXCHANGE` rows is the value in the
completion cycle; it already includes the bit fed at its launch, at least
one cycle earlier. A completing op with a result that cannot be accepted
does not complete; it enters `hold` (6.2).

### 6.2 Result, `SR`, `M`

At completion (first time, not `hold`): the op's value `V` is

- `FIELD`, not seek: the `SR` value after this cycle's update, masked to n
  (`DRIVE`: the source; others: the sampled bits);
- `FIELD`, seek: `{16'h0, WIN(...)}` (the matched window);
- `RAW SAMPLE`: `{16'h0, d16_i}`.

and `SR <= V`, also in a cycle that enters `hold` below (so a held seek or
`RAW SAMPLE` keeps its value in `SR`; for `RAW DRIVE` and `PIN`, `SR` is not
changed here).

Acceptance `acc = !res_v || res_ack_i`. Destination `dst` (decoded, 2.1).

- `dst == NONE`, or the op is a `PIN`/`RAW DRIVE`: complete.
- `dst != NONE` and `acc`: at the end of the cycle `res_v <= 1`,
  `res_dst <= dst`, `res_data <= (dst in {HOSTQ, CAP}) ? tagged(V, tag) :
  V`, `res_ts <= ts_i`; complete.
- `dst != NONE` and `!acc`: `hold <= 1`, no completion; `mm` keeps the
  `EXPECT` outcome. In each later cycle with `hold` and `acc`: the same
  register load with `V = SR`, and complete.

`M` (only for `EXPECT`) is written in the cycle the op actually completes:
`M <= seek ? 1 : !mm'`, where `mm'` includes the bit of that cycle. Other
kinds and abandoned fields leave `M` alone. `CRCCHK` (section 8) is the only
other writer.

`res_v <= 0` at the end of a cycle with `res_ack_i`, unless a new result
loads at the same edge. `res_dst`, `res_data`, `res_ts` change only when a
result loads, which happens only while `res_v = 0` or in an ack cycle.

## 7. `PIN` and `RAW` execution

### 7.1 `PIN`

**Effect cycle** e: the first cycle at or after s, with `G` and `eff = 0`,
in which

- `cfg_mode_i` in {`BAUD`, `PULSE`}: `!bt_run_i || bt_stop_i` (the BAUD
  timer is stopped, or stops in this cycle; `align` makes no difference);
- `cfg_mode_i` in {`EDGE`, `RECOVER`}: `align ? lt_i : 1`.

In e: `pin_we = 1`, `pin_mask = mask`, `pin_val = val`, `pin_oe = oe`. If
`wait == 0` the `PIN` completes in e. Otherwise `ot_start = 1`,
`ot_raw = 0`, `ot_n = {4'h0, wait}`, `eff <= 1`, and the `PIN` completes in
the cycle with `ot_done_i` (which may be e itself). So the next op is
current exactly at `e (+) (wait * Pd)`.

A `WAIT` timeout is a `PIN` with `mask = 0`: `pin_we` with an empty mask
changes no output.

### 7.2 `RAW DRIVE`

Effect cycle e: as a `PIN` with `align = 0`. In e: `raw_go = 1`,
`raw_lvl = level` (`rosetta_coding` maps and drives `out0`, never delayed),
`ot_start = 1`, `ot_raw = 1`, `ot_n = SR[15:0]`, `eff <= 1`. It completes at
`ot_done_i` (possibly in e itself), so the next op is current at
`e (+) max(SR[15:0], 1)`: the level
is held for exactly that many prescaled cycles (at prescale 1). No result.

### 7.3 `RAW SAMPLE`

Completes in the first cycle at or after s with `G && dedge_i`. `V =
{16'h0, d16_i}`: the prescaled time from the previous data edge (which may
be before s) to this one, saturated at `0xFFFF`. Back-to-back `RAW SAMPLE`s
measure consecutive intervals (the next one is current from the cycle after
the edge). Does not touch the outputs.

## 8. Abort, halt, commands

- **Abort** (`run_i && abort_i` in cycle t): no strobes and no completion
  in t (section 2.1, `G`). At the end of t: `cu_v <= 0`, `pk_v <= 0` (also
  if `park_we_i` came in the same cycle), `hold <= tail <= first <= eff <=
  lnch <= rel <= 0`. Kept: `SR`, `bc`, `nl`, `mm`, `M`, the result
  register.
- **Halt** (`run_i = 0` in cycle t): at the end of t, `pk_v`, `cu_v`,
  `res_v`, `hold`, `tail`, `first`, `eff`, `lnch`, `lstf`, `pstf`, `rel`
  are cleared. Kept: `SR`, `M`, `bc`, `nl`, `mm`.
- **`CRCCHK`** (`run_i && cmd_i == 2`): `M <= (crc_state_i == cmd_data_i)`
  at the end of the cycle.

Outputs: `park_full_o = pk_v`; `res_*_o` = the result register; `m_o = M`;
`sr_o = SR`; `dst_busy_o` as `lane.md` 7.6 from `cu`, `pk` and the result
register; `cu_v_o = cu_v`; `pk_v_o = pk_v`.

## 9. Required properties

Assumptions: the sequencer guarantees of `lane.md` 2.1; `cfg_mode_i` and
`cfg_honour_i` change only in cycles with `cu_v_o = pk_v_o = 0` or
`run_i = 0`; tick strobes may be free except: `lt_i`, `ct_i` never in the
same cycle; with `bt`, `lt_i` only with `launch_req_o`; with `bt` and
`!cfg_honour_i`, `spend_i` rises exactly in the cycle after `ct_i &&
cu_smp_o`, falls exactly in the cycle after `st_i` (or abort, halt), `st_i`
only while `spend_i`, and no `ct_i` while `spend_i` (`tick.md` 4.1, 5);
otherwise `spend_i = 0`; `stuff_next_i` may change only in the cycle after
`stf_valid` or `clear` (model the stuffing unit, or instantiate it).
`bt`, `pk` and `cu` decodes are shadowed by the checker from the park buses
and promotions (`pk_v_o` falling without abort or halt).

1. **Park and promote** exactly as section 3, with `cu_done_o` as the
   completion; `cu_done_o` follows 6.1; the load of 4.1 at promotion.
2. **Bit count**: a field completes after exactly n data bits; every data
   bit is exactly one `INS`/`ROT` and (with `crcf`) one `crc_en`; stuff
   bits make none.
3. **Value**: after a non-seek field completes, `SR` equals the reference
   value: for `DRIVE` the source masked to n; for receiving kinds the
   sequence of sampled data bits `b0..b(n-1)` placed at `n-1-k` (msb) or `k`
   (lsb); bits n-31 zero.
4. **Drive order**: the data bits given to `tx_bit` are the source bits in
   order `n-1 .. 0` (msb) or `0 .. n-1` (lsb), including with a pending BAUD
   sample (`shp`) at every launch.
5. **EXPECT**: `M` after completion is 1 iff the sampled value equals the
   source value masked to n (two-copy or reference-register proof).
6. **Seek**: completes at the first data bit after which the last `n'`
   sampled bits, in field order, equal the pattern; never earlier; `SR` then
   holds the pattern.
7. **Stuffing**: with `stf`, the transmit wire bits are the data bits with
   a stuff bit inserted exactly when `stuff_next_i` (and one trailing when
   due); the receive side drops exactly the stuff positions; `stf_valid` is 0
   for fields without `stf`. Round trip with two `rosetta_stuff` instances:
   transmitted data bits equal received data bits (arch obligation 3 at
   field level).
8. **CRC feed**: `crc_en` only for data bits of `crcf` fields, with
   `crc_bit` = the driven bit (`DRIVE`, `EXCHANGE`) or sampled bit
   (`SAMPLE`, `EXPECT`).
9. **Launch absorption**: between a `tx_go` and the next `ct_i`, `lt_i`
   produces no `tx_go`; `early` gives exactly one `tx_go` in s and none at
   the next `lt_i` before a capture.
10. **Release**: a receiving field asserts `tx_rel` at most once, never
    `tx_go`; a `DRIVE` never `tx_rel`.
11. **Result register**: never overwritten while `res_v = 1` without an ack
    in that cycle; stable outputs while valid; one result per completing op
    with a destination; tag applied iff `dst` in {`HOSTQ`, `CAP`} and source
    not `IMM`.
12. **Hold**: `hold_o` rises only at the end of a cycle with `cu_v_o`,
    `res_valid_o` and `!res_ack_i`, for a `FIELD` or `RAW SAMPLE` with a
    destination (never a `PIN`, `RAW DRIVE` or `dst = NONE`); while
    `hold_o`, no promotion, no strobe to coding, CRC or stuffing, and
    `cu_done_o` exactly in the first cycle with `!res_valid_o || res_ack_i`.
13. **`PIN`**: exactly one `pin_we` per `PIN`, at the effect cycle e of 7.1;
    `ot_start` in e iff `wait != 0`; the `PIN` completes in e if `wait == 0`,
    else at the first `ot_done_i` at or after e (e itself included).
14. **`M`** changes only by `CRCCHK` or an `EXPECT` completion.
15. **Abort**: in an abort cycle no output strobe; afterwards `cu_v = pk_v
    = 0` and `SR`, `M`, result register unchanged.
16. **`dst_busy_o`** formula (`lane.md` 7.6); `fld_next_o` equals the
    5.2 formula over `launch_req_o` and the shadowed `pk`/`cu` decodes.
    `launch_req_o` implies `cu_fld_o`, `bt` and `!hold_o`. For a transmit
    kind it is 0 from each `tx_go` until the next `ct_i`, and 0 once the
    field's wire bits (property 7) are all launched; for a receive kind
    without seek it is 1 until the field has had n launches (net of given-back
    launches, 5.3).
17. **Covers**: each kind with each bit order; n = 1 and n = 32; a trailing
    transmit stuff bit; a receive stuff bit inside and after a field; seek
    matching after more than n bits; `EXPECT` mismatch; `hold`; `early` in
    `EDGE`; `PIN` with `align` in `EDGE`; `RAW DRIVE` and two back-to-back
    `RAW SAMPLE`s; `pk_srcdep` deferral; source `CRC` and source `SR`;
    a BAUD `EXCHANGE` whose trailing stuff bit launches before the last data
    sample; a `PIN` with `ot_done_i` in its effect cycle; a BAUD `SAMPLE`
    giving back a launch.

## 10. Worked timing

### 10.1 SPI mode 0 target with `early`

`EDGE`, clock input `in1` (SCK), capture rising, data input `in0` (MOSI),
`out0` = MISO push-pull, prescale 1. Field `EXCHANGE 2, msb, early`, source
bits b1 b0, parked before; promoted at the end of p = 9 (s = 10). The
controller (5 MHz, 6-cycle halves) raises SCK on the pin at 20 and 32 and
lowers it at 26 and 38; it changes MOSI (m1, m0) at its falling edges. The
lane sees each pin 3 cycles late.

| t | `in_i` SCK | Tick | Field | `SR[1:0]` after | MISO pin |
|---|---|---|---|---|---|
| 10 | 0 | | s: `EL`: `x = SR[1]` = b1, `tx_go`; `nl = 1`, `lnch = 1` | b1 b0 | - |
| 11-22 | 0 | | | | b1 |
| 23 | 1 | `ec`: `ct = st` | `lnch`: sample `rxb` = MOSI pin of 20 = m1; `INS`; `bc = 1`; `lnch = 0` | b0 m1 | b1 |
| 29 | 0 | `el`: `lt` | `LE`: `x = SR[1]` = b0; `nl = 2`, `lnch = 1` | b0 m1 | b1 |
| 30 | 0 | | | | b0 |
| 35 | 1 | `ec` | sample m0 (pin of 32); `bc = 2 = n`: **complete**; result; next field promoted at the end of 35 | m1 m0 | b0 |
| 36 | 1 | | next field's s (with `early` it drives its first bit now; without, at 41) | | b0 |
| 41 | 0 | `el` | next field's first `lt` | | |

MISO bit b1 is on the pin 9 cycles before the first SCK rising edge, and b0
changes 4 cycles after the falling edge at 26, leaving 2 cycles of setup
before the rising edge at 32 (language C9 sets the target's top rate from
this).

### 10.2 USB low-speed transmit: NRZI, stuffing and CRC through one `DRIVE`

`BAUD`, prescale 1, `PERIOD = PERIOD2 = 20` (40 cycles per bit), `CODE`:
tx NRZI, differential out (`out0` = D-, so J = 1), `STUFF`: ones, N = 6,
enabled. The previous stuffed field ended with three 1s (run = 3) and
`tl_q = 1`. Field `DRIVE 8, lsb, crc, stuff` with source `0x0F` (data bits
1 1 1 1 0 0 0 0). Launch `L_k = L_0 + 40k` for wire bit k, capture at
`L_k + 20`.

| k | `stuff_next` at launch | Wire bit `x` | `nl` after | NRZI level (D-/D+ on the pins from `L_k + 1`) | At capture `L_k + 20` |
|---|---|---|---|---|---|
| 0 | 0 (run 3) | d0 = 1 | 1 | 1 (J): 1/0 | CRC <- 1, `bc = 1` |
| 1 | 0 (run 4) | d1 = 1 | 2 | 1: 1/0 | CRC <- 1, `bc = 2` |
| 2 | 0 (run 5) | d2 = 1 | 3 | 1: 1/0 | CRC <- 1, `bc = 3` |
| 3 | **1** (run 6) | stuff 0 | 3 | 0 (K): 0/1 | stuff: no shift, no CRC |
| 4 | 0 (run 0) | d3 = 1 | 4 | 0: 0/1 | CRC <- 1, `bc = 4` |
| 5 | 0 (run 1) | d4 = 0 | 5 | 1: 1/0 | CRC <- 0, `bc = 5` |
| 6 | 0 | d5 = 0 | 6 | 0: 0/1 | CRC <- 0, `bc = 6` |
| 7 | 0 | d6 = 0 | 7 | 1: 1/0 | CRC <- 0, `bc = 7` |
| 8 | 0 | d7 = 0 | 8 | 0: 0/1 | CRC <- 0, `bc = 8 = n`, `stuff_next = 0`: **complete** at `L_8 + 20` |

The field takes 9 wire bits. The stuffing unit is fed at each launch; the CRC
takes the 8 data bits at their captures. A following `DRIVE 16, msb, CRC`
(the CRC-16 field, source `CRC`, `pk_srcdep`) is promoted at the end of
`L_8 + 21` and launches at `L_9 = L_8 + 40` with the CRC value final since
`L_8 + 21`. If the last data bits had been six 1s, `left` would have
launched one more (stuff) wire bit at `L_9` and the field would have
completed at `L_9 + 20`.
