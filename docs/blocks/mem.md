# Block spec: `rosetta_mem_prog` and `rosetta_mem_data` (memory wrappers)

Two wrappers, one per SRAM macro. They decide who drives the macro in each
cycle (a pure function of the wheel phase), form the region address, apply the
half-word mask, and, for data memory, hold the host queue pointers. They add
no latency beyond the macro's own registered read.

Wheel terms (`ph`, owners, "D cycle of lane L") are defined in
`sequencer.md` section 1. Lane L's D cycle is `ph = (L+2) mod 5`. The host
owns `ph = 4` for program memory and `ph = 1` for data memory.

## 1. Macro facts

`RM_IHPSG13_1P_512x32_c2_bm_bist`. Source: the PDK datasheet
`libs.ref/sg13cmos5l_sram/doc/RM_IHPSG13_1P_512x32_c2_bm_bist.txt`, its
behavioural model `RM_IHPSG13_1P_core_behavioral_bm_bist.v`, and Ken Pettit's
walkthrough.

| Pin | Dir | Width | Use here |
|---|---|---|---|
| `A_CLK` | in | 1 | `clk` |
| `A_MEN` | in | 1 | memory enable; nothing happens when 0 |
| `A_WEN` | in | 1 | write |
| `A_REN` | in | 1 | read. `A_WEN = A_REN = 1` is write-through; **never used** |
| `A_ADDR` | in | 9 | word address |
| `A_DIN` | in | 32 | write data |
| `A_BM` | in | 32 | per-bit write mask, 1 = write this bit |
| `A_DOUT` | out | 32 | registered read data; **holds its value** in cycles without a read |
| `A_DLY` | in | 1 | tie to 1 (the datasheet requires it; the model stops simulation otherwise) |
| `A_BIST_EN` | in | 1 | tie 0 |
| `A_BIST_CLK`, `A_BIST_MEN`, `A_BIST_WEN`, `A_BIST_REN` | in | 1 each | tie 0 |
| `A_BIST_ADDR` | in | 9 | tie 0 |
| `A_BIST_DIN`, `A_BIST_BM` | in | 32 each | tie 0 |

Behaviour (all inputs sampled at the rising edge of `A_CLK`):

- `A_MEN & A_WEN`: `mem[A_ADDR] <= (mem[A_ADDR] & ~A_BM) | (A_DIN & A_BM)`.
- `A_MEN & A_REN & !A_WEN`: `A_DOUT <= mem[A_ADDR]`, visible after the
  clock-to-out delay, so it is used in the following cycle.
- Otherwise `A_DOUT` and the array hold.
- No reset. The array and `A_DOUT` are unknown at power-up, and nothing may
  rely on a word before it is written.

Timing (datasheet, slow corner 1.08 V 125 C): clock to `A_DOUT` 6.45 ns
(typical 3.85 ns). Setup is at most 0.85 ns on `A_MEN`/`A_WEN` and negative on
address and data. Hold is up to 0.92 ns on address, data and mask, which P&R
must buffer. So a read issued in cycle t gives data that is stable from about
6.5 ns into cycle t+1 until the next read.

Instance names inside the wrappers are fixed as `u_macro`; the LibreLane
`MACROS` entry refers to `u_mem_prog.u_macro` and `u_mem_data.u_macro`.

## 2. `rosetta_mem_prog` (program memory, 512 x 32)

### 2.1 Ports

| Port | Width | Meaning |
|---|---|---|
| `clk` | 1 | clock (no flops in this wrapper, so no reset) |
| `ph_i` | 5 | one-hot wheel phase from the sequencer |
| `fetch_addr_i` | 9 | lane fetch address (`pm_addr_o` of the sequencer) |
| `h_en_i` | 1 | host access this cycle; the host asserts it only when `ph = 4` |
| `h_we_i` | 1 | host write (else read) |
| `h_addr_i` | 9 | host word address |
| `h_wdata_i` | 32 | host write data |
| `rdata_o` | 32 | `A_DOUT` |

### 2.2 Behaviour (combinational drive of the macro in each cycle)

| Condition | `A_MEN` | `A_WEN` | `A_REN` | `A_ADDR` | `A_DIN` | `A_BM` |
|---|---|---|---|---|---|---|
| `ph` in 0..3 | 1 | 0 | 1 | `fetch_addr_i` | 0 | 0 |
| `ph = 4`, `h_en_i`, `h_we_i` | 1 | 1 | 0 | `h_addr_i` | `h_wdata_i` | all ones |
| `ph = 4`, `h_en_i`, `!h_we_i` | 1 | 0 | 1 | `h_addr_i` | 0 | 0 |
| `ph = 4`, `!h_en_i` | 0 | 0 | 0 | 0 | 0 | 0 |

`h_en_i` outside `ph = 4` is ignored. `rdata_o = A_DOUT`. The read for lane k
(address in its F cycle, `ph = k`) arrives in its I cycle (`ph = k+1`), where
the sequencer's IR captures it. A host read issued at `ph = 4` arrives in the
following `ph = 0` cycle, where the host captures it. After a host write or an
idle `ph = 4`, `A_DOUT` keeps lane 3's word during `ph = 0`. The IR captures
it again, but its D owner in `ph = 1` is the host, so it is ignored.

Lanes can never write program memory. The hardware does not stop the host
writing it while lanes run (loader rule, `sequencer.md` decision 23).

### 2.3 Required properties

1. `A_WEN` is 1 only when `ph = 4` and `h_en_i` and `h_we_i`.
2. `A_WEN & A_REN` is never 1; `A_DLY = 1` and all BIST pins are 0 always.
3. In `ph` 0..3 the macro reads `fetch_addr_i`.
4. With the behavioural model (section 4): if the last write to address `a`
   wrote `w`, a fetch of `a` in cycle t gives `rdata_o = w` in cycle t+1.
5. Covers: host write then lane fetch of the same word; host read; a `ph = 4`
   with no host access.

## 3. `rosetta_mem_data` (data memory, 512 x 32, and host queues)

### 3.1 Region layout

The address is `{L, off}`, with `L` = the lane and `off` = 7 bits. For a lane
access `L` is **always the D owner computed from `ph`**, never taken from
instruction bits, so a lane cannot reach another region. Inside each region:

| Offsets | Use |
|---|---|
| `[0, Si)` | host-in queue (host -> lane), `Si = 8 << isz` |
| `[Si, 128 - So)` | free for `MEM` instructions and `MEM` sources and destinations |
| `[128 - So, 128)` | host-out queue (lane -> host), `So = 8 << osz` |

`isz`, `osz` (2 bits each, per lane, host register `QCFG`) give sizes 8, 16,
32 or 64 words. The two queues can never overlap. Queue words are ordinary
memory: `MEM` instructions and the host can read and write them.

### 3.2 Queue pointers

Per lane, four 7-bit pointers: `in_wr` (host-owned), `in_rd` (lane-owned),
`out_wr` (lane-owned), `out_rd` (host-owned). For a queue of size `S`,
`m = 2S - 1`, and with `p'` = `p & m`:

- index `idx(p) = p & (S - 1)`
- `empty(wr, rd) = (wr' == rd')`
- `full(wr, rd) = ((wr' ^ rd') == S)` (same index, opposite wrap bit)
- `next(p) = (p' + 1) & m`
- host-in address offset `= idx(in_rd)`; host-out address offset
  `= (128 - So) | idx(out_wr)`

Pointers are stored as written. Every use masks them, so any value the host
writes gives defined behaviour. All of this is shallow logic: XOR and compare,
no subtraction.

Outputs per lane: `qe_o[L] = empty(in_wr, in_rd)` with `S = Si`;
`qf_o[L] = full(out_wr, out_rd)` with `S = So`;
`qne_o[L] = !empty(out_wr, out_rd)` with `S = So`.

### 3.3 Ports

| Port | Width | Meaning |
|---|---|---|
| `clk`, `rst_n` | 1 | clock; async active-low reset |
| `ph_i` | 5 | one-hot wheel phase |
| `dm_op_i` | 3 | from the sequencer, for the D owner lane: 0 `NONE`, 1 `READ`, 2 `WRITE`, 3 `POP`, 4 `PUSH` |
| `dm_off_i` | 7 | `READ`/`WRITE` offset |
| `dm_hmask_i` | 2 | `WRITE`: `[1]` bits 31:16, `[0]` bits 15:0 |
| `dm_wdata_i` | 32 | `WRITE`/`PUSH` data |
| `h_en_i` | 1 | host access this cycle; asserted only when `ph = 1` |
| `h_we_i` | 1 | host write (full word) |
| `h_addr_i` | 9 | host address `{lane, off}` (any region) |
| `h_wdata_i` | 32 | host write data |
| `run_i` | 4 | lane run bits (from the sequencer) |
| `hreg_we_i` | 1 | register write strobe, only when `ph = 1` |
| `hreg_addr_i` | 7 | register address |
| `hreg_wdata_i` | 32 | write data |
| `hreg_rdata_o` | 32 | read data for `QCFG/QIN/QOUT[L]` and `IRQSTAT[7:4]`; 0 otherwise. Combinational |
| `rdata_o` | 32 | `A_DOUT` |
| `qe_o`, `qf_o`, `qne_o` | 4 each | queue flags (section 3.2); combinational from flops |
| `obs_qcfg_o` | 16 | `{osz, isz}` per lane |
| `obs_in_wr_o`, `obs_in_rd_o`, `obs_out_wr_o`, `obs_out_rd_o` | 28 each | pointers, 7 per lane |

### 3.4 Behaviour

Owner `o` of the D stage: `ph 0 -> lane 3`, `2 -> 0`, `3 -> 1`, `4 -> 2`,
`1 -> host`.

Macro drive in each cycle (first matching row):

| Condition | `A_MEN` | `A_WEN` | `A_REN` | `A_ADDR` | `A_DIN` | `A_BM` |
|---|---|---|---|---|---|---|
| `ph = 1`, `h_en_i`, `h_we_i` | 1 | 1 | 0 | `h_addr_i` | `h_wdata_i` | all ones |
| `ph = 1`, `h_en_i`, `!h_we_i` | 1 | 0 | 1 | `h_addr_i` | 0 | 0 |
| `ph = 1` | 0 | 0 | 0 | 0 | 0 | 0 |
| `READ` | 1 | 0 | 1 | `{o, dm_off_i}` | 0 | 0 |
| `WRITE`, `dm_hmask_i != 0` | 1 | 1 | 0 | `{o, dm_off_i}` | `dm_wdata_i` | `{{16{hmask[1]}}, {16{hmask[0]}}}` |
| `POP`, `!qe_o[o]` | 1 | 0 | 1 | `{o, idx(in_rd[o])}` | 0 | 0 |
| `PUSH`, `!qf_o[o]` | 1 | 1 | 0 | `{o, (128 - So[o]) \| idx(out_wr[o])}` | `dm_wdata_i` | all ones |
| otherwise (`NONE`, `POP` when empty, `PUSH` when full, `WRITE` with mask 0, codes 5-7) | 0 | 0 | 0 | 0 | 0 | 0 |

Lane requests in `ph = 1` and host requests outside `ph = 1` are ignored.

Pointer and configuration updates at the end of the cycle:

- `POP` row taken: `in_rd[o] <= next(in_rd[o])` (with `S = Si[o]`).
- `PUSH` row taken: `out_wr[o] <= next(out_wr[o])` (with `S = So[o]`).
- `ph = 1` and `hreg_we_i`, address per `host.md`:
  - `QCFG[L]`: `isz[L] <= w[1:0]`, `osz[L] <= w[3:2]`.
  - `QIN[L]`: `in_wr[L] <= w[6:0]`; if `run_i[L] = 0`, `in_rd[L] <= w[14:8]`.
  - `QOUT[L]`: `out_rd[L] <= w[6:0]`; if `run_i[L] = 0`, `out_wr[L] <= w[14:8]`.
- Reset: all pointers 0, `isz = osz = 0` (8-word queues).

A lane-owned pointer changes only in that lane's D cycle (never `ph = 1`), and
the host writes only in `ph = 1`, so there is never more than one write per
pointer per cycle.

Read data: `rdata_o = A_DOUT`. For a lane, the word read in its D cycle is
there in its X cycle. For the host, a read at `ph = 1` is there at `ph = 2`.

Register reads (`hreg_rdata_o`):
- `QCFG[L]` = `{28'h0, osz, isz}`.
- `QIN[L]` = `{14'h0, full_in, empty_in, 1'b0, in_rd, 1'b0, in_wr}`, where
  `full_in = full(in_wr, in_rd)` for the host's convenience.
- `QOUT[L]` = `{14'h0, full_out, empty_out, 1'b0, out_wr, 1'b0, out_rd}`.
- `IRQSTAT` bits `[7:4]` = `qne_o`.

### 3.5 How queue traffic maps to accesses

| Operation | Accesses | Slots of the lane | Notes |
|---|---|---|---|
| lane pop (`Q POP`, `FIELD` source `HOSTQ`) | 1 read in the D cycle | 1 | data in the X cycle; waits while empty |
| lane 32-bit push (`Q PUSH*`, delivery to `HOSTQ`) | 1 write in the D cycle | 1 | waits while full |
| `CAP` record | 2 writes (timestamp, then value) | 2 consecutive | each word waits while full. The host may briefly see the timestamp word without its value word |
| host push of one word | 1 `MDATA` write (data memory, `ph = 1`), plus one `QIN` write that may cover many words | 0 | host computes the address from `in_wr` and `QCFG` |
| host pop | 1 `MDATA` read, plus a `QOUT` write that may cover many words | 0 | |
| half-word `MEM` store | 1 write with `A_BM` selecting the half; the sequencer puts the value on both halves | 1 | |
| half-word `MEM` load | 1 full-word read; the sequencer picks the half in X | 1 | |

The host's bandwidth is limited by SPI, not by the slot. One host data
access per wheel turn is 12 M accesses/s; SPI gives at most about 80 k words/s
(`host.md`).

### 3.6 Required properties

1. **Region confinement.** In every cycle with `ph != 1` and `A_MEN = 1`:
   `A_ADDR[8:7] = o`. In particular, a lane's accesses never leave its region
   whatever `dm_*` holds.
2. Host accesses happen only in `ph = 1`; lane requests are ignored in
   `ph = 1`.
3. `A_WEN & A_REN` never; `A_DLY = 1`; BIST pins 0.
4. `POP` with the queue empty, or `PUSH` with it full, makes no access and
   changes no pointer.
5. Pointer ownership: `in_rd[L]`, `out_wr[L]` change only by a pop or push in
   lane L's D cycle, or by a host write while `run_i[L] = 0`. `in_wr[L]`,
   `out_rd[L]` change only by host writes.
6. Queue addresses stay inside their queue: host-in offsets `< Si`, host-out
   offsets `>= 128 - So`.
7. **FIFO behaviour** (with the tracked-word model, section 4.2). If the
   host writes word w at host-in index i and advances `in_wr` past it, and the
   lane pops while `idx(in_rd) = i`, the pop returns w. Likewise, a lane push
   of w at host-out index j is returned by a host read of that address until
   the lane pushes again at j. The lane cannot push again at j until the host
   has advanced `out_rd` past j.
8. **Flag correctness.** With `count = (wr' - rd') & m` (reference only),
   `empty` iff `count = 0` and `full` iff `count = S`. If the host only writes
   pointer values with `count` in [0, S], then `count` stays in [0, S].
9. Half-word store writes only the selected half (tracked word: the other
   half unchanged).
10. Covers: each `isz`/`osz`; wrap of both queues; full and empty; host write
    of a lane-owned pointer while halted (taken) and while running (ignored);
    simultaneous host data access and lane request in the same wheel turn.

## 4. Behavioural model for formal proofs

The wrappers instantiate the macro by name. In proofs, a model module with
the same name and ports replaces it (`read -formal` of the model file instead
of the black box). The real PDK model has `initial` checks and timing; do not
use it.

### 4.1 Full model (BMC and cover)

```
module RM_IHPSG13_1P_512x32_c2_bm_bist (A_CLK, A_MEN, A_WEN, A_REN, A_ADDR,
    A_DIN, A_DLY, A_DOUT, A_BM, A_BIST_CLK, A_BIST_EN, A_BIST_MEN,
    A_BIST_WEN, A_BIST_REN, A_BIST_ADDR, A_BIST_DIN, A_BIST_BM);
  // ports as in section 1
  reg [31:0] mem [0:511];   // no reset, no init: arbitrary initial contents
  reg [31:0] dout;          // no reset: arbitrary initial value
  always @(posedge A_CLK)
    if (A_MEN && A_WEN) begin
      mem[A_ADDR] <= (mem[A_ADDR] & ~A_BM) | (A_DIN & A_BM);
      if (A_REN) dout <= (mem[A_ADDR] & ~A_BM) | (A_DIN & A_BM);
    end else if (A_MEN && A_REN)
      dout <= mem[A_ADDR];
  assign A_DOUT = dout;
endmodule
```

The checker also asserts `A_DLY == 1`, `A_BIST_EN == 0`, `!(A_WEN && A_REN &&
A_MEN)`. With `memory_map` this is 16,416 state bits: fine for BMC and cover,
too large for PDR.

### 4.2 Tracked-word model (unbounded proofs)

Track one arbitrary address. Use two tracked words for ordering properties.

- `(* anyconst *) a_trk [8:0]`; `d_trk [31:0]` with arbitrary initial value
  and no reset.
- Write to `a_trk`: `d_trk <= (d_trk & ~A_BM) | (A_DIN & A_BM)`.
- Read of `a_trk`: `dout <= d_trk`. Read of any other address:
  `dout <= (* anyseq *)` value.
- Optional `written_trk` bit, reset 0 by the harness reset, set by any write
  to `a_trk` with a non-zero mask. Used by the reset-determinism proof
  (`top.md`).

A property proved for every `a_trk` holds for every address. Lane
properties that are about addresses only (region confinement) need no memory
model at all: leave `A_DOUT` free.

## 5. Worked timing

One wheel turn, both macros. Requests: lane 0 pops its host-in queue
(`isz = 1`, `in_rd = 5`); lane 1 pushes (`osz = 2`, `out_wr = 3`); lane 2
does a high-half `MEM` store to offset 0x40; lane 3 is idle; the host reads
data address 0x1A0 and writes program word 0x020. `pc = {0x010, 0x011, 0x012,
0x013}`.

| Cycle | ph | Program macro (addr, op) | Program `A_DOUT` this cycle | Data D owner | Data macro (addr, op, mask) | Data `A_DOUT` this cycle | End-of-cycle updates |
|---|---|---|---|---|---|---|---|
| t | 0 | 0x010 read | previous word | L3 | idle | previous | |
| t+1 | 1 | 0x011 read | word @0x010 -> IR (L0's I) | host | 0x1A0 read | | |
| t+2 | 2 | 0x012 read | word @0x011 -> IR | L0 | {00, 0x05} read (pop) | word @0x1A0 -> host captures | `in_rd[0] <= 6` |
| t+3 | 3 | 0x013 read | word @0x012 -> IR | L1 | {01, 0x60 \| 3 = 0x63} write, all ones | L0's popped word (L0's X) | `out_wr[1] <= 4` |
| t+4 | 4 | 0x020 write (host) | word @0x013 -> IR | L2 | {10, 0x40} write, mask FFFF0000 | L0's word (held) | |
| t+5 | 0 | 0x010 read | still word @0x013 (write does not change `A_DOUT`); IR captures it, ignored by host D | L3 | idle | held | |

Where 0x63 is offset 99 = `128 - 32 + 3`, and lane 1's region base is
128, so `A_ADDR` = 128 + 99 = 0x0E3.

## 6. Flop count

`rosetta_mem_prog`: 0. `rosetta_mem_data`: 4 lanes x (4 x 7 pointer bits + 4
size bits) = **128**.
