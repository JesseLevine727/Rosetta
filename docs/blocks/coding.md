# Block spec: `rosetta_coding` (line coding, receive and transmit)

Status: draft for review (Phase 0, 14 Nov). Part of the lane (`lane.md`);
notation in `lane.md` 1.

Receive side: selects the data input, cleans it (differential pair check),
gives the line level and its edges to `rosetta_tick`, and decodes each sample
strobe into a bit (`rxb`) and whether there is one (`rxv`). Transmit side:
turns each wire bit from `rosetta_field` into one or two line changes (level,
NRZI, Manchester, pulse width), applies invert, open-drain and differential
mapping, and issues data-output requests to `rosetta_delay`.

## 1. Ports

| Port | Width | Meaning |
|---|---|---|
| `clk`, `rst_n` | 1 | clock; async active-low reset |
| `run_i`, `abort_i` | 1 each | lane live; sequencer abort |
| `wr_code_i`, `wr_tick_i` | 1 each | `SETM CODE` / `SETM TICK` lands at the end of this cycle |
| `cfg_mode_i` | 2 | `TICK[1:0]` (only "is it `PULSE`" is used) |
| `cfg_rx_i` | 2 | `CODE[1:0]`: 0 level, 1 NRZI, 2 Manchester, 3 pulse symbol |
| `cfg_tx_i` | 2 | `CODE[3:2]`: 0 level, 1 NRZI, 2 Manchester, 3 pulse width |
| `cfg_din_i` | 2 | `CODE[5:4]`: data input |
| `cfg_inv_i` | 1 | `CODE[6]` invert (both directions) |
| `cfg_od_i` | 1 | `CODE[7]` data output open-drain |
| `cfg_dif_i` | 1 | `CODE[8]` differential output |
| `cfg_dfi_i` | 1 | `CODE[9]` differential input |
| `cfg_pw0_i`, `cfg_pw1_i` | 8 each | `PULSE` high times (only "is it zero" is used) |
| `in_i` | 4 | lane inputs |
| `st_i`, `cte_i`, `hte_i`, `ge0_i` | 1 each | from `rosetta_tick` |
| `tx_go_i`, `tx_bit_i`, `tx_rel_i`, `raw_go_i`, `raw_lvl_i` | 1 each | from `rosetta_field` (`lane.md` 6.4). At most one of `tx_go_i`, `tx_rel_i`, `raw_go_i` per cycle |
| `pin_we_i`, `pin_mask0_i`, `pin_val0_i`, `pin_oe0_i` | 1 each | a `PIN` takes effect; does it write `out0`, with which value and enable |
| `out_val_i` | 2 | current `out_val_o[1:0]` (from `rosetta_pins`) |
| `dline_o`, `dedge_o` | 1 each | cleaned data level (before invert) and its edge |
| `rxb_o`, `rxv_o` | 1 each | decoded bit, and "this sample strobe carries a bit" |
| `pw_bit_o` | 1 | *reg*. Value of the pulse-width bit being sent |
| `dreq_o`, `dpay_o[4:0]`, `dreq_dly_o` | 1, 5, 1 | data-output request `{v0, e0, w1, v1, e1}` (`lane.md` 6.5) |

## 2. State

| Flop | Reset | Meaning |
|---|---|---|
| `dl_q` | 0 | `dline` of the previous cycle |
| `nz_q` | 1 | NRZI receive reference: the logical level at the previous valid sample strobe |
| `mph` | 1 | Manchester receive phase: 1 = the last edge was mid-bit |
| `tl_q` | 0 | transmit: last logical level requested for `out0` (by a field, `RAW DRIVE` or `PIN`) |
| `hp`, `hv` | 0 | a second change of the current wire bit is pending, and its level |
| `pwb` | 0 | value of the last pulse-width bit launched (`pw_bit_o`) |

7 flops. While `run_i = 0` all except `dl_q` take their reset values at
every edge. `dl_q(t+1) = dline(t)` in every cycle, also while halted, so
there is no spurious edge when the lane starts.

## 3. Receive

### 3.1 Line cleaning

- `P = in_i[cfg_din_i]`, `N = in_i[cfg_din_i ^ 1]`.
- `ok = !cfg_dfi_i || (P != N)`. With differential input, a pair at equal
  levels (USB SE0 or SE1) is not data.
- `dline = ok ? P : dl_q` (holds its last valid level while the pair is
  invalid). `dedge = ok && (P != dl_q)`.
- `lvl = dline ^ cfg_inv_i` (the logical level).

`dline` and `dedge` feed the RECOVER resync, PULSE captures, the edge
interval counter and `RAW SAMPLE` (`tick.md`). Events use the raw inputs, not
`dline` (`events.md`).

### 3.2 Decoding (meaningful in a cycle with `st_i`)

Let `PM = (cfg_mode_i == PULSE)`.

| Case | `rxb` | `rxv` | State update at the end of the cycle (only if `st_i`) |
|---|---|---|---|
| `cfg_rx_i == 1` (NRZI, any mode) | `lvl == nz_q` | `ok` | |
| `cfg_rx_i == 0`, or `!PM` and `cfg_rx_i` in {2, 3} | `lvl` | `ok` | |
| `PM`, `cfg_rx_i == 2` (Manchester) | `lvl` | `ge0_i \|\| !mph` | `mph <= ge0_i ? 1 : !mph` |
| `PM`, `cfg_rx_i == 3` (pulse symbol) | `ge0_i` | `lvl == 0` | |

In every case, at the end of a cycle with `st_i && ok`: `nz_q <= lvl`
(the NRZI reference follows every sample, whether or not a field is current,
and in every receive coding).

Meaning:

- **NRZI** (USB convention): 1 = no change since the previous sample, 0 =
  change.
- **Manchester** (`PULSE` mode, where `st` is every data edge, `tick.md`
  4.4). An interval of at least `THR0` prescaled cycles (`ge0`) means the
  edge is mid-bit. A short interval toggles the phase: mid-bit after a
  boundary, boundary after mid-bit. A mid-bit edge carries the bit, which is
  the new logical level (IEEE 802.3: a rising mid-bit edge is a 1). Boundary
  edges carry nothing. Set `THR0` between the short (half-bit) and long
  (one bit) intervals: 5 for 10BASE-T at 3 cycles per half-bit.
- **Pulse symbol**: a falling logical edge ends a high pulse and carries one
  bit, 1 iff the pulse lasted at least `THR0`. Rising edges carry nothing.
  This is model D12 (`lane.md` I11).
- Manchester and pulse symbol need `PULSE` mode; elsewhere they decode as
  level.

## 4. Transmit

### 4.1 Mapping

For a logical level `l`: `p = l ^ cfg_inv_i`, `q = !p`.

```
MAP(l) = { v0 = cfg_od_i ? 0 : p,   e0 = cfg_od_i ? !p : 1,
           w1 = cfg_dif_i,
           v1 = cfg_od_i ? 0 : q,   e1 = cfg_od_i ? !q : 1 }
```

Open-drain: a 0 drives low, a 1 releases. Differential: `out1` carries the
complement of `out0`, mapped the same way (`w1 = 0` leaves `out1` alone).

### 4.2 Requests, in priority order (at most one per cycle)

All are 0 in a cycle with `!run_i`. `tx_go_i`, `tx_rel_i`, `raw_go_i` are
already 0 in an abort cycle (`field.md`), and `cte_i`, `hte_i` too
(`tick.md` 2).

1. **Wire bit** (`tx_go_i`), with `x = tx_bit_i`:

   | `cfg_tx_i` | Level `l` requested now | Also, at the end of the cycle |
   |---|---|---|
   | 0 level | `x` | `hp <= 0` |
   | 1 NRZI | `x ? tl_q : !tl_q` | `hp <= 0` |
   | 2 Manchester | `!x` | `hp <= 1`, `hv <= x` |
   | 3 pulse width | `H(x) != 0` | `pwb <= x`, `hp <= (H(x) != 0)`, `hv <= 0` |

   with `H(x) = x ? cfg_pw1_i : cfg_pw0_i`. Request: `dreq = 1`,
   `dpay = MAP(l)`, `dreq_dly = 1`; `tl_q <= l`.
2. **Release** (`tx_rel_i`): `dreq = 1`, `dpay = {v0 = out_val_i[0], e0 = 0,
   w1 = cfg_dif_i, v1 = out_val_i[1], e1 = 0}` (values kept, enables
   cleared), `dreq_dly = 1`; `hp <= 0`.
3. **`RAW DRIVE`** (`raw_go_i`): `l = raw_lvl_i`; `dreq = 1`,
   `dpay = MAP(l)`, `dreq_dly = 0` (never delayed); `tl_q <= l`; `hp <= 0`.
4. **Second change** `sc = hp && (cfg_tx_i == 3 ? (hte_i || cte_i) :
   cte_i)`, when none of 1-3: `l = hv`; `dreq = 1`, `dpay = MAP(l)`,
   `dreq_dly = 1`; `tl_q <= l`; `hp <= 0`.

So Manchester sends `!x` for the first half (from the launch tick) and `x`
for the second (from the capture time; in BAUD `PERIOD` and `PERIOD2` are
the two half-bit times). Pulse width drives high at the launch and low after
`H(x)` prescaled cycles or at the capture time, whichever is first; a zero
high time sends no pulse. The bit time is `PERIOD + PERIOD2`.

### 4.3 NRZI continuity

`tl_q` is the last logical level requested for `out0` by this block. A `PIN`
that writes `out0` (in a cycle with `pin_we_i && pin_mask0_i`) sets
`tl_q <= (pin_oe0_i ? pin_val0_i : 1) ^ cfg_inv_i` (a released line reads
high). So NRZI transmit continues from the last level written by a field, a
`RAW DRIVE` or a `PIN` (model D35). A `PIN` and a request of this block never
coincide (they are different operations).

### 4.4 Abort, writes

- `abort_i`: `hp <= 0` (a pending second half is cancelled; outputs keep
  their levels).
- `wr_code_i` or `wr_tick_i`: `mph <= 1`, `hp <= 0`.

## 5. Required properties

Assumptions: `cfg_` inputs change only at the end of a cycle with
`wr_code_i` (for `CODE`) or `wr_tick_i` (`TICK`) or while halted;
`cfg_pw0_i`/`cfg_pw1_i` are read only in a cycle with `tx_go_i` and need no
assumption; at most one of `tx_go_i`, `tx_rel_i`, `raw_go_i` per cycle;
`tx_go_i`, `tx_rel_i`, `raw_go_i`, `st_i`, `cte_i`, `hte_i` are 0 in a cycle
with `abort_i` or `!run_i` (`field.md` 2.1, `tick.md` 2). The state `nz_q`,
`mph`, `tl_q`, `hp`, `hv` is not exposed: the checker keeps shadow copies,
which the ports determine completely (reset values of section 2, updates of
3.2, 4.2-4.4).

1. **Mapping**: every request's payload is `MAP(l)` (or the release payload)
   as in 4.1-4.2; with open-drain, `v0 = v1 = 0` always.
2. **Line cleaning**: with `cfg_dfi_i` and `P == N`, `dedge = 0`, `dline`
   holds, and in a cycle with `st_i`, `rxv = 0`. Without `cfg_dfi_i`,
   `dline = P`.
3. **Decoding** exactly as table 3.2, including the `nz_q` and `mph` updates.
4. **Round trip, level and NRZI** (arch obligation 4): a transmitting
   instance whose pin level (`e0 ? v0 : 1`) feeds the data input of a
   receiving instance with the same `CODE`, sampled once per bit between two
   transmit requests, returns the transmitted bits, for every bit sequence,
   both values of invert and open-drain, with or without differential
   in/out (out1 feeding N); for NRZI provided the receiver's `nz_q` starts
   equal to the transmitter's `tl_q`.
5. **Round trip, Manchester**: this block with `rosetta_tick` in `PULSE`
   mode, fed with the line of a transmitting instance whose half-bit changes
   are h cycles apart (h >= 2), with every edge interval within 1 cycle of
   its nominal value (h or 2h), and `h + 1 < THR0 <= 2h - 1`, returns every
   transmitted bit
   after the first long interval (or after a known mid-bit edge), for every
   bit sequence.
6. **Round trip, pulse width**: high times `H0 < THR0 <= H1` give back the
   bits in pulse-symbol mode.
7. **Second change**: after a Manchester or pulse-width `tx_go_i`, exactly
   one second request follows, at the first qualifying `cte_i`/`hte_i`,
   unless another request, abort, or a write comes first; none ever follows
   a level or NRZI bit.
8. **Delay class**: `dreq_dly = 0` exactly for `RAW DRIVE` requests.
9. **NRZI continuity**: the level after an NRZI bit is `tl_q` for a 1 and
   `!tl_q` for a 0, where `tl_q` follows 4.3 including `PIN` writes.
10. **Covers**: each receive and transmit coding; Manchester boundary and
    mid-bit edges; an invalid differential pair during a sample; NRZI after
    a `PIN`; pulse width with a zero high time.

## 6. Worked timing: Manchester receive, 3 cycles per half-bit, 1 cycle jitter

`PULSE` mode, prescale 1, receive Manchester, `THR0 = 5`, no invert. Bits
`1 0 1 1 0 0` (802.3: a 1 is low then high). Nominal edges at 3, 9, 15, 18,
21, 27, 30, 33 (mid-bit at `6k + 3`, boundaries at `6k`); the lane sees them
with jitter at the cycles below. The edge at 3 follows preamble, so `mph = 1`.
`d16` is the interval since the previous edge (`tick.md` 6); `st = dedge`.

| t | Edge (`lvl` after) | `d16` | `ge0` | `mph` before -> after | `rxv` | `rxb` | Bit |
|---|---|---|---|---|---|---|---|
| 3 | rise (1) | >= 5 (preamble) | 1 | 1 -> 1 | 1 | 1 | bit 0 = 1 |
| 10 | fall (0) | 7 | 1 | 1 -> 1 | 1 | 0 | bit 1 = 0 |
| 15 | rise (1) | 5 | 1 | 1 -> 1 | 1 | 1 | bit 2 = 1 |
| 17 | fall (0) | 2 | 0 | 1 -> 0 | 0 | | boundary |
| 21 | rise (1) | 4 | 0 | 0 -> 1 | 1 | 1 | bit 3 = 1 |
| 28 | fall (0) | 7 | 1 | 1 -> 1 | 1 | 0 | bit 4 = 0 |
| 30 | rise (1) | 2 | 0 | 1 -> 0 | 0 | | boundary |
| 33 | fall (0) | 3 | 0 | 0 -> 1 | 1 | 0 | bit 5 = 0 |

Short intervals measure 2-4 and long ones 5-7, so one cycle of jitter on
each edge is the limit at this rate (`lane.md` T1). Each `rxv` strobe is one
`st` that `rosetta_field` takes as a data bit (shift, CRC, count); bits are
at least 4 cycles apart here, so a field boundary (next field current one
cycle after completion) never misses one. Consecutive edges as close as 2
cycles are handled because every decision uses registered `d16` and `mph`.
