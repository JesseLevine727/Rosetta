# Block spec: `rosetta_host` (SPI target host port)

The RP2350 talks to the chip through an SPI target sampled in the chip's
clock domain. One SPI frame is one 32-bit register read or write. Memories
are reached through an address register and a data window. Every effect of a
host frame happens in the host's wheel position: program memory at `ph = 4`,
data memory and all registers at `ph = 1` (`sequencer.md` section 1). So the
host never shares a cycle with a lane.

Flop budget target was about 135; this design uses **69** (section 6).

## 1. Ports

| Port | Width | Meaning |
|---|---|---|
| `clk`, `rst_n` | 1 | clock; async active-low reset |
| `ph_i` | 5 | one-hot wheel phase |
| `sclk_i`, `csn_i`, `mosi_i` | 1 each | `ui[0]`, `ui[1]`, `ui[2]` after two-flop synchronisers (`xbar.md`) |
| `miso_o` | 1 | to `uo[0]`; registered |
| `irq_o` | 1 | to `uo[1]`; registered, active high |
| `h_pm_en_o`, `h_pm_we_o` | 1 each | program memory access this cycle (only when `ph = 4`) |
| `h_pm_addr_o` | 9 | `maddr[8:0]` |
| `h_pm_wdata_o` | 32 | `sr` |
| `pm_rdata_i` | 32 | program SRAM `A_DOUT` |
| `h_dm_en_o`, `h_dm_we_o` | 1 each | data memory access this cycle (only when `ph = 1`) |
| `h_dm_addr_o` | 9 | `maddr[8:0]` |
| `h_dm_wdata_o` | 32 | `sr` |
| `dm_rdata_i` | 32 | data SRAM `A_DOUT` |
| `hreg_we_o` | 1 | register write strobe (only when `ph = 1`) |
| `hreg_addr_o` | 7 | `cmd[6:0]` (always) |
| `hreg_wdata_o` | 32 | `sr` |
| `hreg_rdata_i` | 32 | OR of the `hreg_rdata_o` of seq, mem_data, chan, xbar (each 0 for addresses it does not own) |
| `attn_i` | 4 | per-lane attention flags (sequencer) |
| `qne_i` | 4 | per-lane host-out queue non-empty (mem_data) |
| `obs_sr_o`, `obs_cmd_o`, `obs_bitcnt_o`, `obs_maddr_o`, `obs_hst_o`, `obs_imask_o` | 32, 8, 6, 10, 2, 8 | observation of internal state |

## 2. SPI protocol

**Mode 0** (CPOL = 0, CPHA = 0), most significant bit first. The host
changes MOSI while SCLK is low and samples MISO on SCLK rising. The chip
samples MOSI on (synchronised) SCLK rising and changes MISO on (synchronised)
SCLK falling. MISO is a plain output (`uo[0]`), 0 when not sending.

Frames (CS_n low for the whole frame, high between frames):

| Frame | Bits | Content |
|---|---|---|
| Write | 40 | command byte `{0, A[6:0]}`, then 32 data bits |
| Read | 48 | command byte `{1, A[6:0]}`, then 8 dummy bits (MOSI ignored, MISO 0), then 32 data bits on MISO |

The dummy byte gives the chip time to fetch the data in its slot, so register
and memory reads are handled the same way and need no read-ahead register.

A write takes effect only if at least 40 rising edges arrive. A read's side
effect (the `MDATA` auto-increment) happens once the command byte is
complete, even if the frame is cut short. Bits after the 40th (write) or
48th (read) are ignored until CS_n rises.

### 2.1 Host timing requirements (T = one chip clock period)

| Requirement | Value | Why |
|---|---|---|
| SCLK high time, SCLK low time | >= 8 T each, so SCLK <= f_clk/16 (3.75 MHz at 60 MHz) | MISO changes up to 4 T after a SCLK falling edge (2 synchroniser flops, edge detect, MISO flop). That leaves >= 4 T (67 ns) for pad and Tiny Tapeout mux delay before the host samples |
| MOSI stable around SCLK rising | >= 4 T before and after (mode 0 gives a half period) | MOSI and SCLK synchronisers may resolve one cycle apart |
| CS_n low before first SCLK rising; after last SCLK falling; high between frames | >= 8 T each | edges must be seen in order |

Limit f_clk/16 is conservative; f_clk/12 is probably safe but needs the
real pad delays.

Throughput at 3.75 MHz: about 90 k register writes/s, about 76 k reads/s. A
two-word capture record therefore drains at about 38 k records/s. Language
C4 estimated 50 k/s at 4 MHz; see decision H3.

## 3. Behaviour

### 3.1 State

| Flop | Bits | Reset | Meaning |
|---|---|---|---|
| `sclk_prev` | 1 | 0 | `sclk_i` one cycle earlier |
| `sr` | 32 | 0 | shift register: incoming data, outgoing read data |
| `bitcnt` | 6 | 0 | rising edges seen in this frame, saturating at 48 |
| `cmd` | 8 | 0 | command byte; `cmd[7]` = read |
| `miso` | 1 | 0 | drives `miso_o` |
| `maddr` | 10 | 0 | `[9]` space (0 program, 1 data), `[8:0]` word address |
| `hst` | 2 | 0 | request state: 0 IDLE, 1 PEND, 2 ISSUED |
| `imask` | 8 | 0 | `[3:0]` attention enables, `[7:4]` out-queue enables |
| `irq` | 1 | 0 | drives `irq_o` |

`rise = sclk_i && !sclk_prev`, `fall = !sclk_i && sclk_prev`. On every edge:
`sclk_prev <= sclk_i`; `irq <= |(attn_i & imask[3:0]) | |(qne_i & imask[7:4])`.

### 3.2 Frame receive (end of each cycle)

- `csn_i = 1`: `bitcnt <= 0`, `miso <= 0`. (`sr`, `cmd`, `hst` keep.)
- `csn_i = 0` and `rise`, with `k = bitcnt + 1` (the edge number):
  - `bitcnt <= min(k, 48)`.
  - `k <= 8`: `sr <= {sr[30:0], mosi_i}`.
  - `k = 8`: `cmd <= {sr[6:0], mosi_i}`. If `sr[6] = 1` (read) and
    `hst = IDLE`: `hst <= PEND`.
  - `9 <= k <= 40` and `cmd[7] = 0`: `sr <= {sr[30:0], mosi_i}`. At `k = 40`,
    if `hst = IDLE`: `hst <= PEND`.
  - `17 <= k <= 47` and `cmd[7] = 1`: `sr <= {sr[30:0], 1'b0}`.
  - Otherwise `sr` holds (in particular for `9 <= k <= 16` of a read).
- `csn_i = 0` and `fall`: `miso <= (cmd[7] && 16 <= bitcnt && bitcnt <= 47) ?
  sr[31] : 0`.
- A read-data capture (3.3) in the same cycle as a shift of `sr` takes
  priority. This cannot happen when the timing of 2.1 is met.

So for a read, data bit 31 is on MISO from the falling edge after rising edge
16, bit 0 from the falling edge after rising edge 47, and MISO returns to 0
after rising edge 48.

### 3.3 Request service

Let `A = cmd[6:0]`, `rd = cmd[7]`. Kind: `PROG` if `A = MDATA` and
`maddr[9] = 0`; `DATA` if `A = MDATA` and `maddr[9] = 1`; `REG` otherwise.
All outputs not listed are 0.

| `hst` | Kind | Cycle | Outputs in that cycle | End of cycle |
|---|---|---|---|---|
| PEND | `REG`, write | `ph = 1` | `hreg_we_o = 1` | host-owned registers update (3.4); `hst <= IDLE` |
| PEND | `REG`, read | `ph = 1` | | `sr <= RDATA(A)`; `hst <= IDLE` |
| PEND | `PROG`, write | `ph = 4` | `h_pm_en_o = h_pm_we_o = 1` | `maddr[8:0] <= maddr[8:0] + 1`; `hst <= IDLE` |
| PEND | `PROG`, read | `ph = 4` | `h_pm_en_o = 1` | `maddr[8:0]++`; `hst <= ISSUED` |
| ISSUED | `PROG` | `ph = 0` | | `sr <= pm_rdata_i`; `hst <= IDLE` |
| PEND | `DATA`, write | `ph = 1` | `h_dm_en_o = h_dm_we_o = 1` | `maddr[8:0]++`; `hst <= IDLE` |
| PEND | `DATA`, read | `ph = 1` | `h_dm_en_o = 1` | `maddr[8:0]++`; `hst <= ISSUED` |
| ISSUED | `DATA` | `ph = 2` | | `sr <= dm_rdata_i`; `hst <= IDLE` |

`RDATA(A)` = the host's own registers for `ID`, `MADDR`, `IRQMASK`, else
`hreg_rdata_i`. The auto-increment wraps within `[8:0]` and keeps
`maddr[9]`. A request is served within 5 cycles of `hst` becoming PEND, and a
read's data reaches `sr` within 6.

### 3.4 Host-owned registers (written at the end of a `ph = 1` cycle with `hreg_we_o`)

`MADDR`: `maddr <= sr[9:0]`. `IRQMASK`: `imask <= sr[7:0]`. All other
writes are acted on by the owning block from `hreg_*`.

## 4. Register map

Word registers, 7-bit address. Reserved addresses read 0 and ignore writes.
Read-only registers ignore writes. "Halted" means `run[L] = 0`.

| Addr | Name | Owner | Access | Bits |
|---|---|---|---|---|
| 0x00 | `ID` | host | R | `0x52530401`: `[31:16]` "RS", `[11:8]` `N_LANES`, `[7:0]` version 1 |
| 0x01 | `MADDR` | host | RW | `[9]` space (0 program, 1 data), `[8:0]` word address (data: `{lane, offset}`) |
| 0x02 | `MDATA` | host + mem | RW | the memory word at `MADDR`; every access then increments `MADDR[8:0]` |
| 0x03 | `TIME` | chan | R | timestamp counter (sampled at the end of the `ph = 1` service cycle) |
| 0x04 | `IRQSTAT` | seq, mem | R, W1C | `[3:0]` `attn[L]` (write 1 clears), `[7:4]` host-out queue non-empty, `[11:8]` `run[L]` |
| 0x05 | `IRQMASK` | host | RW | `[3:0]` attention enables, `[7:4]` out-queue enables |
| 0x06 | `CHSTAT` | chan | R, W1C | `[3:0]` channel valid (write 1 clears the valid bit) |
| 0x08-0x0B | `CHDATA0-3` | chan | R | channel data |
| 0x10 + 8L + 0 | `CTRL[L]` | seq | RW | write: `[0]` run, `[1]` pause, `[2]` step. Read: `[0]` run, `[1]` pause, `[2]` step pending, `[3]` attn, `[4]` lane busy, `[5]` lane parked full, `[6]` lane result waiting, `[7]` wsta, `[8]` capw, `[9]` data wait |
| +1 | `PC[L]` | seq | RW | `[8:0]` PC; writes ignored unless halted |
| +2 | `FLAGS[L]` | seq | R | `[0]` Z, `[1]` N, `[2]` M, `[6:3]` E0-E3 pending, `[7]` QE, `[8]` QF, `[12:9]` armed E0-E3 |
| +3 | `R10[L]` | seq | R | `{R1, R0}` |
| +4 | `R32[L]` | seq | R | `{R3, R2}` |
| +5 | `QCFG[L]` | mem | RW | `[1:0]` host-in size code, `[3:2]` host-out size code (8 << code words) |
| +6 | `QIN[L]` | mem | RW | `[6:0]` `in_wr`; `[14:8]` `in_rd` (written only when halted); `[16]` empty, `[17]` full (R) |
| +7 | `QOUT[L]` | mem | RW | `[6:0]` `out_rd`; `[14:8]` `out_wr` (written only when halted); `[16]` empty, `[17]` full (R) |
| 0x30-0x3F | | | | reserved (lanes 4-5 if `N_LANES = 6`) |
| 0x40 | `XIN0` | xbar | RW | lane 0 and 1 input selects (`xbar.md`) |
| 0x41 | `XIN1` | xbar | RW | lane 2 and 3 input selects |
| 0x42 | `XOUT0` | xbar | RW | output selects for `uo[2]`-`uo[7]` |
| 0x43 | `XOUT1` | xbar | RW | output selects for `uio[0]`-`uio[5]` |
| 0x44 | `XOUT2` | xbar | RW | output selects for `uio[6]`, `uio[7]` |
| 0x45 | `XCTL` | xbar | RW | `[0]` internal loopback |

Typical loader sequences:

- Load a program: write `MADDR = 0x000`, then N `MDATA` writes (one frame
  each).
- Start lane L at address a: write `QCFG[L]` and the queue pointers, then
  `PC[L] = a`, then `CTRL[L] = 1`.
- Push words to lane L: write them with `MDATA` at `{1, L, idx(in_wr)}`
  (one `MADDR` write per run of consecutive indices), then write `QIN[L]` with
  the new `in_wr`.
- Drain lane L: read `QOUT[L]`, read `MDATA` from `{1, L, (128 - So) |
  idx(out_rd)}` for each waiting word, then write `QOUT[L]` with the new
  `out_rd`.
- Restart after an error halt: read `PC[L]` and `IRQSTAT`, clear `attn`, set
  `PC[L]`, `CTRL[L] = 1`.

## 5. Interrupt output

`irq_o` = registered OR over lanes of `attn[L] & IRQMASK[L]` and
`qne[L] & IRQMASK[4+L]`. `attn[L]` is set by `MISC IRQ`, `MISC HALT` or an
illegal instruction, and cleared by writing 1 to `IRQSTAT[L]`. The host-out
condition is a level and clears when the host drains the queue. After reset
the masks are 0, so `irq_o = 0`.

## 6. Flop count

`sclk_prev` 1 + `sr` 32 + `bitcnt` 6 + `cmd` 8 + `miso` 1 + `maddr` 10 +
`hst` 2 + `imask` 8 + `irq` 1 = **69**. The probe's host port had 135. The
saving comes from dropping its separate 32-bit write-data and read-data
registers: write data waits in `sr`, which is safe because the next SCLK edge
is at least 16 T away and service takes at most 5 T. Read data goes straight
into `sr` during the dummy byte.

## 7. Required properties

1. **Slot discipline.** `h_pm_en_o` only when `ph = 4`; `h_dm_en_o` and
   `hreg_we_o` only when `ph = 1`; at most one of the three in any cycle, and
   only when `hst = PEND`.
2. **One effect per frame.** A frame produces at most one request. A write
   request is made only after exactly 40 rising edges with `cmd[7] = 0`; a
   read request at rising edge 8 with the read bit set.
3. **Write data.** For a write frame carrying `D` (first data bit is `D[31]`)
   to address A, the request's data (`h_pm_wdata_o`, `h_dm_wdata_o` or
   `hreg_wdata_o`) is `D`, and `hreg_addr_o = A`.
4. **Read data.** For a read frame of address A meeting 2.1, the 32 bits on
   MISO at the 32 rising edges 17-48 are, MSB first, the value served for A:
   `RDATA(A)` at the service cycle, or the memory word.
5. **MISO idle.** `miso_o = 0` whenever CS_n has been high (synchronised)
   for a cycle, and during edges 1-16 of any frame and all of a write frame.
6. **Auto-increment.** Each served `MDATA` request increments `maddr[8:0]` by
   1 mod 512, leaving `maddr[9]`. No other request changes `maddr`, except a
   `MADDR` write.
7. **Service latency.** `hst = PEND` lasts at most 5 cycles; `ISSUED` lasts
   exactly 1 cycle.
8. **IRQ.** `irq_o(t+1) = |(attn_i(t) & imask[3:0]) | |(qne_i(t) &
   imask[7:4])`.
9. **Abort.** CS_n rising before edge 40 of a write frame gives no write.
10. **Reference SPI master.** Under the timing assumptions of 2.1 (encoded in
    the checker as a slow SPI master driving `sclk_i`, `csn_i`, `mosi_i`
    directly, with arbitrary phase relative to `ph_i`), every frame is decoded
    as properties 2-4 state.
11. Covers: a write and a read of each kind (`REG`, `PROG`, `DATA`); two
    consecutive `MDATA` writes; a frame aborted at bit 20; IRQ rising for
    each source.

## 8. Worked timing

Example: SCLK half period exactly 8 T, CS_n falls at the pin in cycle 0,
each SCLK edge at the pin early in the cycle shown. Synchronised signals lag
the pin by 2 cycles; actions happen at the end of the detection cycle.

Write of `PC[0] = 0x020` (frame `0x11`, `0x00000020`):

| Pin event | Pin cycle | Detected (cycle) | Host action at end of cycle |
|---|---|---|---|
| CS_n falls | 0 | 2 (`csn_i = 0`) | `bitcnt <= 0` |
| rising 1 (cmd bit 7 = 0) | 8 | 10 | `sr <= {sr, 0}`, `bitcnt <= 1` |
| rising 8 (cmd bit 0 = 1) | 120 | 122 | `cmd <= 0x11` |
| rising 9 (data bit 31) | 136 | 138 | shift |
| rising 40 (data bit 0) | 632 | 634 | `sr = 0x00000020`; `hst <= PEND` |
| first `ph = 1` cycle after 634 | | 636 if `636 mod 5 = 1` | `hreg_we_o = 1`, `hreg_addr_o = 0x11`, `hreg_wdata_o = 0x20`; sequencer sets `pc[0] <= 0x020` (lane 0 halted); `hst <= IDLE` |
| CS_n rises | 648 | 650 | `bitcnt <= 0`, `miso <= 0` |

Read of `TIME` (frame `0x83`, dummy, 32 data bits):

| Pin event | Detected (cycle) | Host action |
|---|---|---|
| rising 8 | 122 | `cmd <= 0x83`; `hst <= PEND` |
| first `ph = 1` after 122 (cycle 126) | 126 | `sr <= ts` (the counter value in cycle 126); `hst <= IDLE` |
| rising 9-16 (dummy) | 138 ... 250 | no shift |
| falling 16 | 258 | `miso <= sr[31]` (bit 31); pin from cycle 259 |
| rising 17 (host samples bit 31) | 266 | `sr <= sr << 1` |
| falling 17 | 274 | `miso <=` bit 30 |
| falling 47 | 754 | `miso <=` bit 0 |
| rising 48 | 762 | host samples bit 0 |
| falling 48 | 770 | `miso <= 0` |

The MISO bit that changes at the end of cycle 258 is sampled by the host at
pin cycle 264 (rising 17): 5 cycles, minus pad delays, of margin.

## 9. Decisions

- **H1.** One word per frame, no bursts. Simplest, and the flop count halves.
- **H2.** All host effects in the host slot, including register writes, so
  "the host acts only in its slot" holds for every effect (used by
  `top.md` non-interference).
- **H3.** Drain rate is 38 k capture records/s at f_clk/16. If rung 2 needs
  more, add a burst read (CS_n held low, `MDATA` repeated with a 32-bit
  read-ahead register: +32 flops, about 117 k words/s), or raise the SCLK
  limit after pad delays are known.
- **H4.** The host can read but not write R0-R3, mode registers and event
  state. A restart re-runs the program's own initialisation.
