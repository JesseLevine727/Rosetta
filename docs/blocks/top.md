# Block spec: `tt_um_jesselevine727_rosetta` (top level)

The top level instantiates the shared blocks, the two SRAM wrappers and the
four lanes, synchronises the reset, and maps the Tiny Tapeout ports. It
contains no logic beyond the reset synchroniser and an OR of the register
read buses. This file also holds the **chip-level proof obligations**
(architecture section 13, items 5-8 and 10) with their exact statements.

## 1. Instances

| Instance | Module | Spec | Flops |
|---|---|---|---|
| (inline) | reset synchroniser | section 3 | 2 |
| `u_xbar` | `rosetta_xbar` | `xbar.md` | 170 |
| `u_host` | `rosetta_host` | `host.md` | 69 |
| `u_seq` | `rosetta_seq` | `sequencer.md` | about 580 |
| `u_mem_prog` | `rosetta_mem_prog`, containing `u_macro` | `mem.md` | 0 |
| `u_mem_data` | `rosetta_mem_data`, containing `u_macro` | `mem.md` | 128 |
| `u_chan` | `rosetta_chan` | `channels.md` | 164 |
| `gen_lane[L].u_lane`, L = 0..3 | `rosetta_lane` | `lane.md` (other architect) | |

**Shared logic total: about 1,113 flops.** The probe's 546 did not include
the per-lane sequencer state (PC, registers, flags, run state, event base,
queue pointers: 116 flops x 4 = 464). The probe counted those inside each
lane's 698, and this design moves them here. The like-for-like comparison is
1,113 against 546 + 464 = 1,010:

| Item | Probe | This spec | Difference |
|---|---|---|---|
| Per-lane program state | 368 | 488 | +108 event targets (ISA 3.6 per-event target, vs one vector base), +16 run control (pause, attn, wsta, capw, qwait), -4 flags |
| Queue pointers and sizes | 96 | 128 | 7-bit pointers (queues up to 64 words) and per-lane sizes |
| Pipeline (`ph`, IR, D/X) | 70 | 92 | +32 instruction register required by arch 11, smaller D/X |
| Channels and timestamp | 164 | 164 | |
| Synchronisers and crossbar | 177 | 170 | second sync stage after the mux; global loopback |
| Host port | 135 | 69 | no separate read/write data registers |
| Reset synchroniser | 0 | 2 | |
| **Total** | **1,010** | **1,113** | **+103** |

Options that bring it below the probe: event vector base instead of four
targets (-108, `sequencer.md` decision 11); output select option B (-28,
`xbar.md` 5).

## 2. Ports and Tiny Tapeout mapping

| TT port | Width | Connection |
|---|---|---|
| `ui_in[0]` | 1 | host SCLK -> `u_xbar.ui_in_i[0]` |
| `ui_in[1]` | 1 | host CS_n -> `u_xbar.ui_in_i[1]` |
| `ui_in[2]` | 1 | host MOSI -> `u_xbar.ui_in_i[2]` |
| `ui_in[7:3]` | 5 | protocol inputs -> `u_xbar.ui_in_i[7:3]` |
| `uo_out[0]` | 1 | host MISO, from `u_xbar.uo_out_o[0]` (= `u_host.miso_o`) |
| `uo_out[1]` | 1 | host interrupt, from `u_xbar.uo_out_o[1]` (= `u_host.irq_o`) |
| `uo_out[7:2]` | 6 | protocol outputs, from `u_xbar.uo_out_o[7:2]` |
| `uio_in[7:0]` | 8 | -> `u_xbar.uio_in_i` |
| `uio_out[7:0]` | 8 | from `u_xbar.uio_out_o` |
| `uio_oe[7:0]` | 8 | from `u_xbar.uio_oe_o` (1 = drive) |
| `ena` | 1 | unused (tied into an `_unused` wire) |
| `clk` | 1 | all flops and both macros |
| `rst_n` | 1 | reset synchroniser input only |

`info.yaml` pinout names: `ui[0]` SCLK, `ui[1]` CS_N, `ui[2]` MOSI,
`ui[3..7]` IN3-IN7, `uo[0]` MISO, `uo[1]` IRQ, `uo[2..7]` OUT2-OUT7,
`uio[0..7]` IO0-IO7.

## 3. Reset

- `rst_sync[1:0]`: both flops reset asynchronously to 0 by `rst_n` low; at
  each edge `rst_sync <= {rst_sync[0], 1'b1}`. The internal reset `rst_i_n =
  rst_sync[1]` goes to the `rst_n` port of every block. It asserts
  asynchronously when `rst_n` falls and releases synchronously at the second
  rising edge after `rst_n` rises. This keeps the release from violating
  recovery or removal time, because the TT `rst_n` is not synchronous to
  `clk`.
- **Cycle 0** is the first cycle with `rst_i_n = 1`. All block specs count
  from it.
- Every flop in the design has an asynchronous reset to the value its block
  spec gives (conventions: the library has no flop without reset).
- After reset: `ph = 0`, `ts = 0`, every lane halted (`run = 0`), every
  protocol pin released, `irq_o = 0`, `miso_o = 0`, crossbar unmapped,
  queues empty with 8-word sizes, channels empty.
- SRAM contents and both `A_DOUT` registers are unknown. Nothing depends on
  them before they are written:
  - fetched words are acted on only by eligible lanes, and no lane is
    eligible until the host sets `run`;
  - data memory is read only by running lanes and by host reads;
  - the host reads back only what it wrote (loader rule; property T2 assumes
    it).

## 4. Interconnect

| Signal group | From | To |
|---|---|---|
| `ph` (5) | `u_seq.ph_o` | `u_mem_prog.ph_i`, `u_mem_data.ph_i`, `u_host.ph_i` |
| program fetch | `u_seq.pm_addr_o` | `u_mem_prog.fetch_addr_i` |
| program read data | `u_mem_prog.rdata_o` | `u_seq.pm_rdata_i`, `u_host.pm_rdata_i` |
| host program access | `u_host.h_pm_*` | `u_mem_prog.h_*` |
| lane data access | `u_seq.dm_op/off/hmask/wdata_o` | `u_mem_data.dm_*_i` |
| data read data | `u_mem_data.rdata_o` | `u_seq.dm_rdata_i`, `u_host.dm_rdata_i` |
| host data access | `u_host.h_dm_*` | `u_mem_data.h_*` |
| queue flags | `u_mem_data.qe_o`, `qf_o` / `qne_o` | `u_seq` / `u_host.qne_i` |
| channels | `u_seq.ch_put/get/idx/wdata_o` | `u_chan` |
| channel state, time | `u_chan.ch_valid_o`, `ch_data_o`, `ts_o` | `u_seq` |
| register bus | `u_host.hreg_we/addr/wdata_o` | `u_seq`, `u_mem_data`, `u_chan`, `u_xbar` |
| register read | OR of `hreg_rdata_o` of `u_seq`, `u_mem_data`, `u_chan`, `u_xbar` | `u_host.hreg_rdata_i` |
| run bits | `u_seq.run_o` | `u_mem_data.run_i`, `u_xbar.run_i`, lane `run_i` |
| attention | `u_seq.attn_o` | `u_host.attn_i` |
| host pins | `u_xbar.host_sclk/csn/mosi_o`; `u_host.miso_o`, `irq_o` | `u_host`; `u_xbar.host_miso/irq_i` |
| lane pins | `u_xbar.lane_in_o[4L +: 4]`; lane `out_val_o`, `out_oe_o` | lane `in_i`; `u_xbar.lane_val/oe_i[4L +: 4]` |
| lane control | `u_seq.lane_*_o`, lane outputs | `sequencer.md` section 7 |

Address ownership of the register bus is disjoint (`host.md` section 4), so
the OR is exact.

## 5. Chip-level proof obligations

Numbering follows architecture section 13 where it applies. "Released" for
protocol pin p means `uo_out[p+2] = 0` (p < 6), or `uio_oe[p-6] = 0` and
`uio_out[p-6] = 0` (p >= 6).

Two proof levels are used:

- **Core level**: the top without `u_host` and `u_xbar`'s host path. The
  host request interface (`h_pm_*`, `h_dm_*`, `hreg_*`) is free inputs,
  constrained only by the slot rule: `h_pm_en` only when `ph = 4`; `h_dm_en`
  and `hreg_we` only when `ph = 1`; at most one per cycle.
- **Chip level**: the full top with SPI pins free.

Other lanes may be replaced by free outputs (cutpoints) where stated. This
makes the properties stronger and the proofs smaller.

**T1. Every flop resets** (obligation 5, part 1). Two copies with arbitrary,
independent initial values of every flop and identical inputs, with `rst_n`
low in the first cycle: in cycle 0 every flop is equal in both copies and
equal to its documented reset value. (A flop without reset shows up as a
difference.)

**T2. Reset determinism** (obligation 5, part 2). Two copies A and B at chip
level. Every flop starts at an arbitrary, independent value, and SRAM
contents and `A_DOUT` registers are arbitrary and independent. Inputs are
identical in every cycle, with `rst_n` low in the first cycle. Assume (W) in
each copy: every program word acted on by an EXEC slot, every program word
read by the host, and every data word read by a lane or by the host has been
written since reset (tracked-word model with `written_trk`, `mem.md` 4.2).
Then `uo_out`, `uio_out` and `uio_oe` are equal in A and B in every cycle
from cycle 0. Without (W), the same holds until the first cycle in which any
lane is eligible, except `uo_out[0]` during the data phase of a host read of a
never-written word.

**T3. Quiet until configured.** From cycle 0 until the first served host
write of any kind: `uo_out = 0`, `uio_out = 0`, `uio_oe = 0`, except
`uo_out[0]` during the data phase of a host read. Until the first served
`CTRL[L]` write with bit 0 = 1, every pin whose selector names lane L is
released.

**T4. No lane runs before the host starts it** (obligation 5, part 3).
`run[L]` becomes 1 only at the end of a `ph = 1` cycle in which `hreg_we = 1`,
`hreg_addr = 0x10 + 8L`, `hreg_wdata[0] = 1` (at chip level: the end of a
valid SPI write frame to `CTRL[L]` with bit 0 set). While `run[L] = 0`: lane
L makes no data-memory access in its D cycles, no channel operation, receives
no strobes, and its PC, registers and flags change only by host writes.

**T5. A halted or never-started lane enables no output** (ISA 6 rule 3). In
every cycle, for every protocol pin p whose selector has `en = 1` and names
lane L: if `run[L] = 0` then pin p is released. Provable on `u_xbar` alone
(`xbar.md` property 3). Also required of the lane: `out_oe_o = 0` while
`run_i = 0` (`sequencer.md` 7.4).

**T6. A lane's memory accesses stay in its region** (ISA 6 rule 4). In
every cycle with `ph != 1` and data-macro `A_MEN = 1`: `A_ADDR[8:7] = (ph -
2) mod 5`, the lane whose D cycle it is. In cycles with `ph = 1`, the
data-macro access, if any, is the host's. The program macro has `A_WEN = 1`
only when `ph = 4` with a host write. These hold for **every** instruction
stream (no assumption on program memory), because the region bits come from
`ph`.

**T7. Slot discipline and rate** (ISA 6 rule 1). For every lane L, every
data-memory access, channel operation and lane-L strobe attributable to L
happens in L's D cycle (accesses, channels) or X cycle (strobes), and those
cycles are exactly `ph = L+2` and `ph = L+3` mod 5. This is independent of
all inputs. Every lane has exactly one slot action per 5 cycles.

**T8. Non-interference** (obligation 6). Exact statement, core level, for a
fixed lane L, two copies A and B:

Assumptions, every cycle:

1. `rst_n` identical in A and B and low in the first cycle. Flop initial
   values arbitrary and independent.
2. **Lane L's configuration is equal**: `insel` of lane L's four inputs, and
   every `outsel[p]` whose lane field is L in either copy, are equal in A and
   B after every write. `loop = 0` in both copies. `QCFG[L]` is
   equal.
3. **Lane L's pins are equal**: for each input pin selected by one of lane
   L's inputs, the pin value is equal in A and B. Other pins are free. The
   environment does not couple other lanes' outputs into these pins; that
   would be a wire between lanes.
4. **Host actions on lane L are equal**: in every cycle,
   - the served host writes to `CTRL[L]`, `PC[L]`, `QIN[L]`, `QOUT[L]`,
     `QCFG[L]`, and `IRQSTAT` with bit L set, are equal (same cycle, same
     data);
   - all program-memory writes are equal;
   - data-memory writes to region L are equal.
   Everything else the host does (reads, writes to other lanes' registers and
   regions, channel clears, crossbar fields of other lanes) is free and
   independent.
5. **Shared memory contents are equal where L can see them**: initial program
   memory contents are equal, and initial contents of data region L are
   equal (tracked-word model: tracked address in program memory or region L,
   equal initial `d_trk`).
6. **No channel**: in every D cycle of lane L with action EXEC, the
   instruction in `IR` (of A, and so of B) is not `BR` with cond 15-18, not
   `Q` op 4, 5 or 6, and not `FIELD`/`RAW` with source or destination 8-11.
7. **Lane lemma** (`sequencer.md` 7.3) holds for lane L's module.
8. The other three lanes are free outputs in each copy, independently.

Conclusion, every cycle from 0:

- lane L's `out_val_o` and `out_oe_o` are equal in A and B;
- every protocol pin whose selector names lane L has equal `uo_out` /
  `uio_out` / `uio_oe` in A and B;
- lane L's architectural state is equal: `obs_pc_o[L]`, `obs_r_o[L]`, Z, N,
  `run[L]`, `pause[L]`, `step[L]`, `armed[L]`, `tgt[L]`, `wsta[L]`, `capw[L]`,
  `qwait[L]`, `attn[L]`, and the queue pointers of L;
- all lane-L inputs from the shared logic are equal in every cycle where
  their strobe is set (strobes themselves equal in every cycle).

Proof notes. A relational invariant: lane L's state is equal; the shared
pipeline registers are equal whenever their owner is L (`IR` in L's D cycle,
the D/X register in L's X cycle); and `ph`, `ts` are equal. The proof must
show that L's state never depends on the data SRAM's held `A_DOUT` except in
the X cycle after L's own read, and never on `pm_rdata` except in L's I cycle.
Chip-level corollary: with identical SPI pins in A and B, the same conclusion
holds whatever the other lanes' pins and programs are.

**T9. A listener never enables an output** (obligation 8). Let E be the set
of instruction forms that can enable a lane output. The lane spec defines E;
at least `PIN` with an enable bit set, `FIELD` kinds `DRIVE` and `EXCHANGE`,
`RAW DRIVE`, and `SETM TICK` with the clock-output bit. If no EXEC slot of
lane L ever commits an instruction in E, then lane L's `out_oe_o` is 0 in
every cycle, and every pin whose selector names L is released. The compiler
checks that the listener image contains no E instruction. This property
proves the hardware half.

**T10. Instruction equivalence** (obligation 7). For each lane L, a
reference model holding the per-lane state of `sequencer.md` section 3,
stepped once per slot action of L with the inputs that spec samples in L's D
cycle (and `dm_rdata`, `lane_sr`, channel data as specified), matches L's
observation ports at the end of every X cycle. It also produces the same data
memory, channel and lane-strobe outputs. Field and line behaviour is checked
in the lane proofs against the ISA through the interface of `sequencer.md`
section 7.

**T11. No combinational path from input pins to output pins.** `uo_out`,
`uio_out` and `uio_oe` are functions of flops only. Structural: there is no
path from `ui_in`/`uio_in`/`rst_n` (other than through flop resets) to any
output.

**T12. The host acts only in its slot.** Every effect of a host frame
(memory access, register write) happens in a `ph = 4` cycle (program memory)
or a `ph = 1` cycle (everything else) (`host.md` property 1). Together with
T7, no cycle has both a host and a lane effect on the same resource.

## 6. Worked timing (bring-up)

| Cycle (from reset release) | Event |
|---|---|
| `rst_n` low | all flops at reset values; pins released |
| `rst_n` rises before edge e | `rst_sync = 01` after e, `11` after e+1; cycle 0 starts after e+1 |
| 0 | `ph = 0`, `ts = 0`; lanes halted; `uo_out = 0`, `uio_oe = 0` |
| about 640 per frame at SCLK = f/16 | each 40-bit write frame |
| frames 1..N | `MADDR = 0`, then N `MDATA` writes: program memory, each at a `ph = 4` cycle |
| frame N+1 | `XOUT0 = {..., uo[2] <- lane 0 out0, en}` at a `ph = 1` cycle |
| frame N+2 | `PC[0] = 0` |
| frame N+3, served at `ph = 1` cycle t | `CTRL[0] = 1`: `run[0] = 1` from t+1 |
| t+1 (`ph = 2`) | lane 0's first D cycle: EXEC of the word fetched at t-1 (address `pc[0] = 0`) |
| t+2 (`ph = 3`) | its X cycle: for example a `PIN out0 = 1, enable` is parked to lane 0 |
| t+3 onward | lane 0 applies the `PIN` (lane-spec latency); `uo[2]` follows its `out0` flop in the same cycle |

## 7. Notes for the TT flow

- Source files: `project.v` (this module) and one `src/rosetta_<block>.sv`
  per block, plus the lane files; all listed in `info.yaml` and
  `test/Makefile`.
- Both macros: `MACROS` entries for instances `u_mem_prog.u_macro` and
  `u_mem_data.u_macro`, with power pins `VDD`, `VDDARRAY`, `VSS` (see Ken
  Pettit's walkthrough for placement and obstructions).
- Gate-level simulation needs the SRAM behavioural model, because the
  netlist keeps the macro as a black box.
