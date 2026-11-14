# Block spec: `rosetta_xbar` (synchronisers and pin crossbar)

Connects the 24 Tiny Tapeout pins to the host port and to the lanes'
4 inputs and 4 outputs each (architecture section 10). Every input path has
exactly two flops. Lane inputs are registered after the crossbar
(architecture section 11). Output paths are combinational from lane flops.

## 1. Pins

| Pin | Index used here | Use |
|---|---|---|
| `ui[0]`, `ui[1]`, `ui[2]` | input 0, 1, 2 | host SCLK, CS_n, MOSI (fixed). Also selectable as lane inputs (listening to the host bus is allowed) |
| `ui[3]`-`ui[7]` | input 3-7 | protocol inputs |
| `uio[0]`-`uio[7]` | input 8-15; output 6-13 | bidirectional protocol pins (open-drain capable) |
| `uo[0]`, `uo[1]` | | host MISO, IRQ (fixed) |
| `uo[2]`-`uo[7]` | output 0-5 | protocol outputs (push-pull only) |

So there are 16 input pins (input index q: `ui[q]` for q < 8, `uio[q-8]` for
q >= 8) and 14 protocol output pins (output index p: `uo[p+2]` for p < 6,
`uio[p-6]` for p >= 6).

## 2. Ports

| Port | Width | Meaning |
|---|---|---|
| `clk`, `rst_n` | 1 | clock; async active-low reset |
| `ui_in_i` | 8 | Tiny Tapeout `ui_in` |
| `uio_in_i` | 8 | Tiny Tapeout `uio_in` |
| `host_sclk_o`, `host_csn_o`, `host_mosi_o` | 1 each | `ui[0..2]` after two flops |
| `host_miso_i`, `host_irq_i` | 1 each | from the host port |
| `lane_in_o` | 16 | registered lane inputs; lane L input j at bit `4L+j` |
| `lane_val_i`, `lane_oe_i` | 16 each | lane output values and enables; lane L output j at bit `4L+j` |
| `run_i` | 4 | lane run bits (sequencer) |
| `uo_out_o` | 8 | Tiny Tapeout `uo_out` |
| `uio_out_o`, `uio_oe_o` | 8 each | Tiny Tapeout `uio_out`, `uio_oe` |
| `hreg_we_i` | 1 | register write strobe (asserted only when `ph = 1`) |
| `hreg_addr_i` | 7 | register address |
| `hreg_wdata_i` | 32 | write data |
| `hreg_rdata_o` | 32 | read data for 0x40-0x45; 0 otherwise. Combinational |
| `obs_insel_o`, `obs_outsel_o`, `obs_loop_o` | 64, 70, 1 | configuration, for proofs |

## 3. State

| Flop | Bits | Reset | Meaning |
|---|---|---|---|
| `s1` | 16 | `0x0002` (CS_n idle high) | first synchroniser stage, one per input pin |
| `hs2` | 3 | `3'b010` | host second stage: `{mosi, csn, sclk}` |
| `lin` | 16 | 0 | lane inputs: second stage, after the crossbar |
| `insel` | 16 x 4 | 0 | lane input source select |
| `outsel` | 14 x 5 | 0 | output pin select `{en, lane[1:0], out[1:0]}` |
| `loop` | 1 | 0 | internal loopback |

## 4. Behaviour

### 4.1 Synchronisers and lane inputs (every clock edge)

- `s1[q] <= pin q` for q = 0..15.
- `hs2 <= s1[2:0]`; `host_sclk_o = hs2[0]`, `host_csn_o = hs2[1]`,
  `host_mosi_o = hs2[2]`.
- `lin[i] <= SRC(insel[i])` for i = 0..15; `lane_in_o = lin`.
- `SRC(s)` with `loop = 0`: `s1[s]`. With `loop = 1`: for s < 8,
  `uo_out_o[s]`; for s >= 8, `uio_oe_o[s-8] ? uio_out_o[s-8] : 1`. The 1
  stands in for a pulled-up released line, so an open-drain self-test (I2C)
  works.

So each lane input path is pin -> `s1` -> crossbar mux -> `lin` -> lane: two
flops, with a static mux between them (section 6, X1). The host path is pin ->
`s1` -> `hs2`.

### 4.2 Output pins (combinational)

For output index p with `outsel[p] = {en, ln, oi}` and `j = 4*ln + oi`:

- `drive(p) = en && run_i[ln] && lane_oe_i[j]`
- `uo_out_o[p+2] = drive(p) && lane_val_i[j]` for p = 0..5. A released
  push-pull pin reads 0. Open-drain wires must be mapped to `uio` pins.
- `uio_oe_o[p-6] = drive(p)`, `uio_out_o[p-6] = drive(p) && lane_val_i[j]`
  for p = 6..13.
- `uo_out_o[0] = host_miso_i`, `uo_out_o[1] = host_irq_i`.

Gating with `run_i` makes "a halted or never-started lane enables no output"
true in this block alone, without relying on the lane.

### 4.3 Configuration registers (end of a cycle with `hreg_we_i`)

Each write replaces the whole register; unused bits are ignored and read 0.

| Addr | Name | Fields |
|---|---|---|
| 0x40 | `XIN0` | `insel` of lane 0 inputs 0-3 at `[3:0]`, `[7:4]`, `[11:8]`, `[15:12]`; lane 1 inputs 0-3 at `[19:16]` ... `[31:28]` |
| 0x41 | `XIN1` | lanes 2 and 3, same layout |
| 0x42 | `XOUT0` | `outsel` for `uo[2]`..`uo[7]` (p = 0..5) at `[5p+4 : 5p]` |
| 0x43 | `XOUT1` | `uio[0]`..`uio[5]` (p = 6..11) at `[5(p-6)+4 : 5(p-6)]` |
| 0x44 | `XOUT2` | `uio[6]`, `uio[7]` (p = 12, 13) at `[4:0]`, `[9:5]` |
| 0x45 | `XCTL` | `[0]` `loop` |

Change the crossbar only while the affected lanes are halted (loader rule).
The hardware does not enforce it.

### 4.4 Reset state

All `en = 0`, so every protocol pin is released: `uo_out_o[7:2] = 0`,
`uio_oe_o = 0`, `uio_out_o = 0`. `loop = 0`. Lane inputs select `ui[0]`
(harmless; all lanes are halted). The host sees CS_n high.

### 4.5 The `uio_oe` rule

`uio_oe_o[k] = 1` exactly when pin `uio[k]` is mapped (`en = 1`) to a lane
that is running and whose selected output is enabled. Nothing else drives a
`uio` pin.

## 5. Select bits and cheaper encodings

| Encoding | Lane input bits | Output bits | Total config flops |
|---|---|---|---|
| Full generality of arch 10: each lane input chooses any of 16 input pins or any of 16 outputs at once (32 sources, 5 bits); each output pin chooses any of 16 lane outputs or none (5 bits) | 16 x 5 = 80 | 14 x 5 = 70 | 150 |
| **Adopted:** 4-bit input selects over the 16 input pins, plus one global loopback bit that swaps every source for its output twin; full output selects | 16 x 4 + 1 = 65 | 70 | **135** (9,180 um2) |
| Option B: as adopted, but pin p can carry only output `p mod 4` of the chosen lane (`{en, lane}`, 3 bits) | 65 | 14 x 3 = 42 | 107 (saves 28 flops, about 1,900 um2) |

The adopted encoding loses only the ability to mix live pins and loopback in
one configuration. Loopback is for silicon self-test (arch 10), where every
input is looped back anyway.

Option B still fits two I2C lanes on `uio` pins (SDA = `out0` on `uio[2]` or
`uio[6]`; SCL = `out3` on `uio[1]` or `uio[5]`). But it forces pin choices on
the board and compiler. Recommend B only if area gets tight.

## 6. Decisions

- **X1. Synchroniser second stage after the mux.** The first flop is per
  pin; the second is the per-lane-input register after the crossbar. Every
  path has two flops, the lane input is registered after the crossbar, and
  this saves 13 flops (vs 32 synchroniser flops plus 16 lane input registers)
  and one cycle of input latency, which helps SPI-target speed (language C9).
  The cost is that the static 16:1 mux (about 1.5 ns) eats into the
  metastability settling time: about 14.9 ns instead of 16.4 ns at the slow
  corner. That is still many orders of magnitude of MTBF. The select must be
  static while a lane uses it (4.3). Reviewer: confirm this is acceptable; the
  fallback is 16 more flops and +1 cycle.
- **X2.** Loopback taps the internal output values, not the pads. It is one
  cycle faster than the external path, and a released `uio` pin reads 1.
- **X3.** A released `uo` pin is 0. A lane that changes an output's value and
  enable in the same cycle can make a glitch on a `uo` pin (AND of two flops).
  The lane spec should avoid changing both together on a clock output, or
  `uo` could be registered (+6 flops, +1 cycle on `uo` only).

## 7. Required properties

1. **Two flops on every input path.** `lane_in_o(t)` equals
   `SRC(insel(t-1))` evaluated on `s1(t-1)`, where `s1(t-1)` is the pins at
   t-2. `host_*_o(t)` are pins `ui[0..2]` at t-2. No combinational path
   exists from `ui_in_i`/`uio_in_i` to any output port.
2. **Output equations** of 4.2 hold in every cycle.
3. **Halted lanes drive nothing.** For every p: if `outsel[p].en = 1` and
   `run_i[outsel[p].lane] = 0`, then the pin is released (`uo_out_o[p+2] = 0`,
   or `uio_oe_o[p-6] = 0` and `uio_out_o[p-6] = 0`).
4. **Unmapped pins drive nothing**: `en = 0` implies released.
5. **`uio_oe` rule** of 4.5, both directions.
6. **Configuration** changes only at edges with `hreg_we_i` and a matching
   address. Reads return what was written (unused bits 0).
7. **Reset**: in the first cycle after reset, all protocol pins are released
   and `host_csn_o = 1`.
8. **Loopback**: with `loop = 1`, `lane_in_o(t+1)[i] = SRC(insel[i])(t)` per
   4.1. With `loop = 0`, lane inputs do not depend on `lane_val_i`/`lane_oe_i`.
9. **Lane independence**: `lane_in_o[4L +: 4]` depends only on `insel` of
   lane L, the pins it selects, and (with loopback) the outputs it selects.
10. Covers: every input select value; every output select value with a
    running lane; loopback through `uo` and through a released `uio` (reads
    1); a lane halting while mapped.

## 8. Worked timing

Lane 1 input 0 selects `uio[3]` (`insel[4] = 11`); `uo[4]` (p = 2) selects
lane 1 output 0 (`outsel[2] = {1, 01, 00}`). Lane 1 is running and its
`out0` enable is 1.

| Cycle | Event | `s1[11]` | `lin[4]` (lane 1 in0) | lane 1 `out0` flop | `uo[4]` |
|---|---|---|---|---|---|
| n | `uio[3]` rises before the edge ending n | 0 | 0 | 0 | 0 |
| n+1 | | 1 | 0 | 0 | 0 |
| n+2 | lane 1 sees `in0 = 1` and sets `out0` at the end of n+2 | 1 | 1 | 0 | 0 |
| n+3 | | 1 | 1 | 1 | 1 (combinational) |

Pin to lane input: the lane sees a pin value two cycles after the edge that
first samples it. Lane output flop to pin: same cycle. Pin to pin through a
lane that reacts at once: 3 cycles plus pad delays. With `loop = 1` and lane
1 in0 selecting source 4 (`uo[4]`), the lane sees its own `out0` change one
cycle after the flop changes.

## 9. Flop count

`s1` 16 + `hs2` 3 + `lin` 16 + `insel` 64 + `outsel` 70 + `loop` 1 =
**170**.
