# Block spec: `rosetta_pins` (lane output registers)

Status: draft for review (Phase 0, 14 Nov). Part of the lane (`lane.md`);
notation in `lane.md` 1.

The four lane outputs `out0`-`out3`, each a value flop and an enable flop.
Every change of a lane output goes through this block, from three sources:
data requests (field bits, releases, `RAW DRIVE`; via `rosetta_delay`),
clock requests (`rosetta_tick`; via `rosetta_delay`) and `PIN` writes
(`rosetta_field`). This block decides priority and guarantees that a halted
or never-started lane enables nothing.

## 1. Ports

| Port | Width | Meaning |
|---|---|---|
| `clk`, `rst_n` | 1 | clock; async active-low reset |
| `run_i` | 1 | lane live |
| `d_we_i`, `d_pay_i` | 1, 5 | data request `{v0, e0, w1, v1, e1}` to apply at the end of this cycle (`tick.md` 9) |
| `c_we_i`, `c_pay_i` | 1, 2 | clock request `{v3, e3}` |
| `pin_we_i` | 1 | a `PIN` takes effect at the end of this cycle (`field.md` 7.1) |
| `pin_mask_i`, `pin_val_i`, `pin_oe_i` | 4 each | which outputs, new values, new enables |
| `out_val_o` | 4 | *reg*. Output values |
| `out_oe_o` | 4 | output enables: `oe & {4{run_i}}` |

## 2. State and update

State `val[3:0]`, `oe[3:0]`, reset 0. At the end of every cycle, for each
output j, the first matching row applies:

| # | Condition | `val[j]`, `oe[j]` become |
|---|---|---|
| 1 | `!run_i` | `val[j]` kept, `oe[j] = 0` |
| 2 | `pin_we_i && pin_mask_i[j]` | `pin_val_i[j]`, `pin_oe_i[j]` |
| 3 | j = 0 and `d_we_i` | `v0`, `e0` |
| 4 | j = 1 and `d_we_i && w1` | `v1`, `e1` |
| 5 | j = 3 and `c_we_i` | `v3`, `e3` |
| 6 | otherwise | kept |

`out_val_o = val`. `out_oe_o = oe & {4{run_i}}` (the only lane output not
purely registered; `run_i` is a sequencer flop, so this is one AND gate
between flops).

## 3. Ownership

| Output | Written by | Notes |
|---|---|---|
| `out0` | field data requests (`DRIVE`/`EXCHANGE` bits, Manchester and pulse-width halves, release by `SAMPLE`/`EXPECT`), `RAW DRIVE`, `PIN` | the data output |
| `out1` | field data requests with `w1` (only when `CODE[8]` differential: the complement), `RAW DRIVE` (same), `PIN` | a plain `PIN` output when not differential |
| `out2` | `PIN` only | chip select and other static lines |
| `out3` | clock requests (only when `TICK[7]`), `PIN` | the clock output; a plain `PIN` output when `TICK[7] = 0` |

Rules:

- **Last writer wins.** An output keeps whatever was last written, by any
  source. Fields never "own" an output beyond their own requests: between
  fields the outputs hold (isa 2), and a `PIN` may change them.
- **`PIN` wins within a cycle** (row 2 before 3-5). It also cancels a
  delayed change still pending for an output it writes (`tick.md` 9.2), so a
  stale delayed field change cannot land after a `PIN`.
- **`PIN` writes are raw.** Values and enables exactly as encoded; no
  invert, open-drain or differential mapping (`lane.md` I7). Field and
  `RAW` requests are already mapped by `rosetta_coding`; clock requests by
  `rosetta_tick`.
- **Concurrency.** A `PIN` and a field request cannot normally coincide (they
  are different operations; a `PIN` waits for the BAUD timer to stop). The
  cases that can coincide are a `PIN` with a delayed request maturing, and a
  `PIN` with the BAUD timer's end-of-bit clock change in the cycle it stops;
  in both the `PIN` wins for the outputs it names.
- **Glitch rule** (`xbar.md` X3). Field and clock requests on an open-drain
  output change only the enable (`v = 0` always), and a release keeps the
  value, so they never change value and enable of a push-pull `uo` pin in a
  way that can glitch. A `PIN` may change both; that is the program's choice.

## 4. Halted and never-started lanes enable nothing

- After reset `oe = 0`, and `oe[j]` can become 1 only through rows 2-5.
- Row 1: in every cycle with `run_i = 0`, every enable is cleared at the end
  of the cycle, and `out_oe_o = 0` during the cycle by the gate.
- So `out_oe_o = 0` in every cycle in which the lane is halted or has never
  been started, and a lane that starts again starts with every output
  released. The values are kept, which is harmless (released).
- `rosetta_xbar` gates with `run_i` as well (`xbar.md` property 3); either
  gate alone is enough.

**Which instructions can enable an output** (the set E of `top.md` T9, as
lane inputs): `oe[j]` rises only by a `PIN` with `pin_mask[j] &&
pin_oe[j]`; a data request with `e0` or `e1` set, which only `DRIVE` and
`EXCHANGE` launches and `RAW DRIVE` make (releases have `e = 0`); or a clock
request with `e3` set, which needs `TICK[7] = 1`. A listener image without
these never enables an output (`lane.md` 9 property 12).

## 5. Required properties

1. Update table of section 2, for each j, every cycle.
2. `out_oe_o = 0` whenever `run_i = 0`; after a cycle with `run_i = 0`,
   `oe = 0`.
3. From reset until a cycle with one of the enabling inputs (section 4) and
   `run_i`, `out_oe_o = 0` and `out_val_o = 0`.
4. `PIN` priority: in a cycle with `pin_we_i && pin_mask_i[j]` (and
   `run_i`), output j takes the `PIN` values regardless of the other
   requests.
5. Outputs not named by any request keep their values and enables.
6. No combinational path from any input except `run_i` to an output.
7. Covers: each row for each output; `PIN` and clock request in the same
   cycle; data request with and without `w1`; halt with outputs enabled.

## 6. Worked timing: I2C controller STOP after the last ACK, then halt

Continues `tick.md` 11.1 (`BAUD`, `PERIOD = 6`, SCL = `out3` open-drain,
SDA = `out0` open-drain, `rest = 1`). The ACK field completed at 16; four
`PIN`s follow: P1 = no outputs, wait 1 (to keep SDA's change clear of SCL's
falling edge); P2 = `out0` value 0 enable 1 (SDA low), wait 1; P3 = `out3`
value 0 enable 0 (release SCL), wait 1 (`t_su_sto`); P4 = `out0` enable 0
(release SDA), wait 0. "Pin" columns are the wire levels with pull-ups.

| t | Operation | Request applied at the end of t | SCL pin | SDA pin |
|---|---|---|---|---|
| 17 | P1 current (promoted at the end of 16); waits for the timer | | 1 | 0 (target's ACK) |
| 21 | timer stops (`bt_stop`): P1 takes effect (empty mask), timer N = 6 | clock: `rest` = low (`e3 = 1`) | 1 | 0 |
| 22 | | | 0 | 0, the target releases it after seeing SCL low |
| 26 | P1 `ot_done` (21 + 6 - 1): completes; P2 promoted | | 0 | 1 |
| 27 | P2 effect (timer stopped: at once); N = 6 | `out0`: `v = 0`, `e = 1` | 0 | 1 |
| 28 | | | 0 | 0 |
| 32 | P2 done; P3 promoted | | 0 | 0 |
| 33 | P3 effect | `out3`: `e = 0` (release) | 0 | 0 |
| 34 | | | 1 | 0 |
| 38 | P3 done; P4 promoted | | 1 | 0 |
| 39 | P4 effect, wait 0: completes | `out0`: `e = 0` | 1 | 0 |
| 40 | | | 1 | 1 (**STOP**) |
| 50 | host or `HALT`: `run_i = 0` | all enables cleared; `out_oe_o = 0` already in 50 | 1 | 1 |

Each step is exactly `PERIOD` = 6 cycles after the previous change (22, 28,
34, 40), because each `PIN`'s successor is current at `e (+) 6`. The `t_su_sto`
wait counts from the release at 33, not from SCL reading high; a target that
still stretches SCL would shorten it (`lane.md` I28).
