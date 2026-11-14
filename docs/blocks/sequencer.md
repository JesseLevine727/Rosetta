# Block spec: `rosetta_seq` (shared sequencer)

Status: draft for review (Phase 0). Authoritative inputs: `docs/architecture.md`
(approved), `docs/isa.md` v0.2. Where those were ambiguous this spec picks one
behaviour and lists it under "Decisions and ISA issues" at the end.

The sequencer is the field-speed half of the chip. It owns the slot wheel,
the instruction pipeline, the 16-bit ALU, and every lane's program state
(PC, R0-R3, Z, N, run control, event arming). It hands bit operations to the
lanes, delivers their results, and makes all lane accesses to data memory and
channels. It contains no SRAM; the memories are in `rosetta_mem_prog` and
`rosetta_mem_data` (`mem.md`), the channels and timestamp in `rosetta_chan`
(`channels.md`).

Everything here is for `N_LANES = 4`. The generalisation is mechanical (the
wheel has `N_LANES + 1` positions, the host is position `N_LANES`) and is not
specified further.

## 1. Terms and notation

- **Cycle.** One period of `clk`. Cycle 0 is the first cycle in which the
  internal reset is released. "At the end of cycle t" means "at the rising
  edge that ends cycle t"; a register written there shows the new value in
  cycle t+1.
- **Phase** `ph` (0-4). `ph = t mod 5` in cycle t. Port `ph_o` is its one-hot
  form.
- **Owner.** Each of the four pipeline stages serves one owner per cycle.
  Owners are lanes 0-3 and the host H (position 4). The owner of stage S in
  phase `ph` is `(ph - k_S) mod 5` with k = 0 for F, 1 for I, 2 for D, 3 for X:

  | ph | F (program address) | I (instruction register) | D (decide, data memory, channels) | X (execute) |
  |---|---|---|---|---|
  | 0 | L0 | H | L3 | L2 |
  | 1 | L1 | L0 | H | L3 |
  | 2 | L2 | L1 | L0 | H |
  | 3 | L3 | L2 | L1 | L0 |
  | 4 | H | L3 | L2 | L1 |

  So lane L has its **F cycle** at `ph = L`, **I cycle** at `L+1`, **D cycle**
  at `(L+2) mod 5` and **X cycle** at `(L+3) mod 5`: lane 0 D=2 X=3, lane 1
  D=3 X=4, lane 2 D=4 X=0, lane 3 D=0 X=1. The host's program-memory access is
  at `ph = 4`, its data-memory and register access at `ph = 1`.
- **Slot.** One pass of a lane through F, I, D, X. Each lane has exactly one
  slot per wheel turn of five cycles. Every slot ends in exactly one **slot
  action**: NONE, EXEC (one instruction, which may wait), DELIVER (one
  delivery step of a finished bit operation) or VECTOR (event entry).
- **Commit.** An EXEC slot whose instruction does not wait. A waiting
  instruction has no effect except on `qwait[L]` (section 5.3); it is fetched
  again in the next slot.
- Per-lane vectors are packed with lane L in bits `[W*L +: W]`. In tables,
  `x[L]` is lane L's field.
- In the D cycle of lane L: `IR` is the instruction register, `R[n]` is
  `r[L][n]`, `PC` is `pc[L]`, `PC1 = (PC + 1) mod 512`. All lane inputs, `qe_i`,
  `qf_i`, `ch_valid_i`, `ch_data_i` and `ts_i` are their values **during that D
  cycle**.
- Predicates evaluated in lane L's D cycle:
  - `IDLE` = `!lane_busy_i[L] && !lane_park_full_i[L]` (no bit operation running
    or parked).
  - `HAZ` = `(USES(IR) & lane_dst_busy_i[L]) != 0`, with `USES` from table 5.4.
  - `QE` = `qe_i[L]`, `QF` = `qf_i[L]`, `CV(c)` = `ch_valid_i[c]`.
- `tagged(w, tag)` = `tag != 0 ? {tag, w[23:0]} : w`.

## 2. Ports

Groups 2.1-2.5 connect to the shared blocks, 2.6 to the lanes (see section
7 for the contract), 2.7 exposes architectural state for proofs and for the
host read mux.

### 2.1 Clock and wheel

| Port | Width | Meaning |
|---|---|---|
| `clk`, `rst_n` | 1 | clock; async active-low reset (the synchronised internal reset from `top.md`) |
| `ph_o` | 5 | one-hot wheel phase, `ph_o[k] = 1` in cycles with `ph = k`; registered |

### 2.2 Program memory (`mem.md`)

| Port | Width | Meaning |
|---|---|---|
| `pm_addr_o` | 9 | fetch address: `pc[k]` in a cycle with `ph = k`, k = 0..3; 0 when `ph = 4` |
| `pm_rdata_i` | 32 | program SRAM read data (`A_DOUT`). In lane L's I cycle it is the word at the address fetched in L's F cycle |

### 2.3 Data memory (`mem.md`). Driven combinationally in the D cycle.

| Port | Width | Meaning |
|---|---|---|
| `dm_op_o` | 3 | 0 `NONE`, 1 `READ`, 2 `WRITE`, 3 `POP` (host-in queue), 4 `PUSH` (host-out queue); 5-7 never driven. Always `NONE` when `ph = 1` |
| `dm_off_o` | 7 | `READ`/`WRITE`: word offset in the D owner's region. 0 otherwise |
| `dm_hmask_o` | 2 | `WRITE`: `[1]` write bits 31:16, `[0]` write bits 15:0. `PUSH` is always a full word. 0 otherwise |
| `dm_wdata_o` | 32 | `WRITE`/`PUSH` data. 0 otherwise |
| `dm_rdata_i` | 32 | data SRAM `A_DOUT`; in lane L's X cycle it is the word read by L's D-cycle `READ` or `POP` |
| `qe_i` | 4 | lane L's host-in queue is empty |
| `qf_i` | 4 | lane L's host-out queue is full |

### 2.4 Channels and timestamp (`channels.md`). Driven combinationally in the D cycle.

| Port | Width | Meaning |
|---|---|---|
| `ch_put_o` | 1 | put `ch_wdata_o` into channel `ch_idx_o` at the end of this cycle |
| `ch_get_o` | 1 | take channel `ch_idx_o` (clear its valid bit) at the end of this cycle |
| `ch_idx_o` | 2 | channel number |
| `ch_wdata_o` | 32 | put data |
| `ch_valid_i` | 4 | channel valid bits |
| `ch_data_i` | 128 | channel c data at `[32c +: 32]` |
| `ts_i` | 32 | free-running cycle counter |

### 2.5 Host register bus (`host.md`)

| Port | Width | Meaning |
|---|---|---|
| `hreg_we_i` | 1 | register write strobe; asserted only in cycles with `ph = 1` |
| `hreg_addr_i` | 7 | register address (read and write) |
| `hreg_wdata_i` | 32 | write data |
| `hreg_rdata_o` | 32 | read data for the sequencer's registers (host map: 0x04 bits it owns, 0x10-0x2F offsets 0-4); 0 for every other address. Combinational |
| `run_o` | 4 | `run[L]`, registered |
| `attn_o` | 4 | `attn[L]`, registered |

### 2.6 Lane interface

Outputs. Strobes are per lane. The rows marked *shared* are one bus common to
all lanes; their value matters only in a cycle where that lane's strobe is
set (section 7.3).

| Port | Width | Meaning |
|---|---|---|
| `lane_run_o` | 4 | equal to `run_o` |
| `lane_park_we_o` | 4 | load the lane's parked-operation register at the end of this cycle |
| `lane_park_op_o` | 2 | *shared*. 0 `FIELD`, 1 `RAW`, 2 `PIN` (the `WAIT` timeout is sent as a `PIN`, section 5.5) |
| `lane_park_ctl_o` | 28 | *shared*. Instruction bits `[27:0]` (or the synthesised `PIN` for a `WAIT` timeout) |
| `lane_park_data_o` | 32 | *shared*. The resolved source value (table 5.2) |
| `lane_res_ack_o` | 4 | the lane's result has been delivered; the lane clears it at the end of this cycle |
| `lane_abort_o` | 4 | abandon the current operation and discard the parked one at the end of this cycle |
| `lane_ev_clr_o` | 16 | clear event-pending bits (4 per lane) at the end of this cycle |
| `lane_cfg_we_o` | 4 | write a mode register (`SETM`) at the end of this cycle |
| `lane_cfg_reg_o` | 5 | *shared*. Mode register number, `IR[27:23]` |
| `lane_cfg_hi_o` | 1 | *shared*. Write the upper half, `IR[22]` |
| `lane_cfg_data_o` | 16 | *shared*. Value |
| `lane_cmd_o` | 8 | 2 bits per lane: 0 none, 1 `CRCINIT`, 2 `CRCCHK`, 3 `STUFFCLR`; acts at the end of this cycle |
| `lane_cmd_data_o` | 32 | *shared*. `{R1, R0}` with `CRCCHK` |
| `lane_data_wait_o` | 4 | `qwait[L]`: the lane's last instruction or delivery is waiting for queue or channel data (for `TICK.stretch_hold`) |

Inputs. All are registered outputs of the lane (no combinational path from
any sequencer output).

| Port | Width | Meaning |
|---|---|---|
| `lane_busy_i` | 4 | the lane's current-operation register is occupied |
| `lane_park_full_i` | 4 | the lane's parked-operation register is occupied |
| `lane_res_valid_i` | 4 | a finished operation's result is waiting for delivery |
| `lane_res_dst_i` | 16 | its destination operand code (4 bits per lane) |
| `lane_res_data_i` | 128 | its value (32 bits per lane), already tagged |
| `lane_dst_busy_i` | 16 | bit n of lane L: Rn is the destination of the current op, the parked op or the waiting result (destination `MEM` marks R3) |
| `lane_m_i` | 4 | the `M` flag |
| `lane_ev_pend_i` | 16 | event-pending bits `E0`-`E3` (4 per lane) |
| `lane_sr_i` | 128 | the shift register (32 per lane) |
| `lane_crc_i` | 128 | the CRC value, final XOR applied (32 per lane) |

### 2.7 Observation ports

Registered architectural state, read-only. They cost nothing when unused and
exist so properties can be written about ports only.

| Port | Width | Meaning |
|---|---|---|
| `obs_pc_o` | 36 | `pc[L]` |
| `obs_r_o` | 256 | `r[L][n]` at `[64L + 16n +: 16]` |
| `obs_z_o`, `obs_n_o` | 4 each | `Z`, `N` |
| `obs_pause_o`, `obs_step_o` | 4 each | `pause[L]`, `step[L]` |
| `obs_armed_o` | 16 | `armed[L][k]` |
| `obs_tgt_o` | 144 | `tgt[L][k]` at `[36L + 9k +: 9]` |
| `obs_wsta_o`, `obs_capw_o` | 4 each | `wsta[L]`, `capw[L]` |

## 3. Per-lane program state

All of it is in this block, in flops, reset to the value shown. The lane
holds only bit-hardware state (mode registers, current/parked operation,
result, SR, CRC, `M`, event detectors). Argument for this split: everything
the ALU, branch logic and data-memory path read in the D cycle sits next to
them, and the lane interface stays narrow (section 7).

| State | Bits per lane | Reset | Written |
|---|---|---|---|
| `pc` | 9 | 0 | end of L's D cycle; host at end of a `ph = 1` cycle (only while halted) |
| `r[0..3]` | 4 x 16 | 0 | end of L's D cycle or X cycle (table 5.2) |
| `Z`, `N` | 2 | 0 | end of L's X cycle (ALU instructions only) |
| `run` | 1 | 0 | end of L's D cycle (halt); host |
| `pause` | 1 | 0 | host |
| `step` | 1 | 0 | host; cleared at end of each D cycle in which L is eligible |
| `attn` | 1 | 0 | set at end of L's D cycle; host write-1-to-clear |
| `armed[0..3]` | 4 | 0 | end of L's D cycle; cleared on halt |
| `tgt[0..3]` | 4 x 9 | 0 | end of L's D cycle (`EVT`) |
| `wsta` | 1 | 0 | `WAIT` timeout started; end of L's D cycle; cleared on halt |
| `capw` | 1 | 0 | first `CAP` word written; end of L's D cycle; cleared on halt |
| `qwait` | 1 | 0 | end of L's D cycle; cleared on halt |

Total 122 flops per lane. The flags `M` and `E0`-`E3` are lane outputs; `QE`,
`QF` come from `rosetta_mem_data`; channel valid bits from `rosetta_chan`.

## 4. Behaviour: wheel, fetch, slot action

### 4.1 Wheel

- Reset: `ph = 0`. At every clock edge: `ph <= (ph + 1) mod 5`.
- The owner of every stage is a function of `ph` only. Nothing a lane, the
  host or the pins do changes the wheel. This is the basis of the rate
  guarantee and of non-interference.

### 4.2 Fetch (F and I)

- F, cycle with `ph = k`, k = 0..3: `pm_addr_o = pc[k]`. The program memory
  reads every such cycle whether or not lane k is running (`mem.md`). With
  `ph = 4`, `pm_addr_o = 0` and the program memory serves the host.
- I: at the end of **every** cycle, `IR <= pm_rdata_i`. So in lane L's D cycle,
  `IR` is the word at `pc[L]` as it was in L's F cycle. (`pc[L]` can change
  between L's F and D cycles only by a host write, and the host can write it
  only while L is halted, so a running lane always decodes the word at its
  current PC.) `IR` is never decoded combinationally from `pm_rdata_i`.

### 4.3 Slot action (chosen in lane L's D cycle)

`eligible = run[L] && (!pause[L] || step[L])`. The action is the first that
applies:

1. `!eligible`: **NONE**. No outputs, no state change (the host may still
   write).
2. `lane_res_valid_i[L]`: **DELIVER** (section 5.1).
3. `(lane_ev_pend_i[L] & armed[L]) != 0`: **VECTOR** to `k` = the lowest set
   bit (E0 has the highest priority) (section 5.6).
4. Otherwise **EXEC** the instruction in `IR` (section 5.2).

At the end of the D cycle, if `eligible`: `step[L] <= 0`.

The action is chosen once, in the D cycle. A result or event that appears
during the following X cycle is seen in the next slot.

## 5. Behaviour: actions

### 5.1 DELIVER

Let `d = lane_res_dst_i[L]`, `v = lane_res_data_i[L]`. PC never changes.

| `d` | Waits while | During D | End of D | X cycle |
|---|---|---|---|---|
| 0-3 (`Rd`) | never | | `r[L][d] <= v[15:0]` | `lane_res_ack_o[L] = 1` |
| 4 `HOSTQ` | `QF` | `dm_op_o = PUSH`, `dm_wdata_o = v` | | ack |
| 5 `CAP`, `capw = 0` | `QF` | `dm_op_o = PUSH`, `dm_wdata_o = ts_i` | `capw <= 1` | no ack |
| 5 `CAP`, `capw = 1` | `QF` | `dm_op_o = PUSH`, `dm_wdata_o = v` | `capw <= 0` | ack |
| 7 `MEM` | never | `dm_op_o = WRITE`, `dm_off_o = R[3][6:0]`, `dm_hmask_o = 2'b11`, `dm_wdata_o = v` | `r[L][3] <= R[3] + 1` | ack |
| 8-11 (`Cc`, c = d - 8) | `CV(c)` | `ch_put_o = 1`, `ch_idx_o = c`, `ch_wdata_o = v` | | ack |
| any other | never | | | ack (result discarded) |

A `CAP` record therefore takes two consecutive slot actions of the lane; no
other slot action of the lane can come between them, because the result stays
valid (DELIVER has priority) until the second one acks it. The timestamp word
is `ts_i` in the D cycle of the first of them.

### 5.2 EXEC: the instruction table

Fields: `op = IR[31:28]`. Unless the table says otherwise, a commit does
`pc <= PC1` at the end of D. "Halt" means the illegal-instruction halt of
section 5.7. All waits are evaluated in the D cycle with the predicates of
section 1. **Every** instruction also waits while `lane_park_full_i[L] = 1`:
the run-ahead stops at a parked operation (section 5.11). The one exception
is `WAIT` with `wsta = 1`, whose own timeout operation may be the parked one.
The table lists only the conditions beyond that one. An illegal instruction
halts regardless of any wait condition.

Source value `SRC(s)` for `FIELD`/`RAW`, `s = IR[17:14]`, except that for
`RAW` with `IR[27] = 0` (sample) the source is ignored and `s` is taken as 15
(`NONE`: no pop, read, take or hazard check), as in the toolchain model:
`0-3`: `{16'h0, R[s]}`; `4 HOSTQ`: popped word; `7 MEM`: word at `R[3][6:0]`;
`8-11`: `ch_data_i[c]` (c = s - 8); `12 IMM`: `{23'h0, IR[8:0]}`; `6 CRC`,
`13 SR`, `15 NONE`: `32'h0` (the lane resolves `CRC` and `SR` when the
operation starts). Sources 5 and 14 are illegal.

| Instruction | Waits while | During D | End of D | X cycle |
|---|---|---|---|---|
| `FIELD` (0), `RAW` (1) | `HAZ`, or (`s = 4` and `QE`) or (`s = 8..11` and `!CV(c)`) | `s = 4`: `dm_op_o = POP`. `s = 7`: `dm_op_o = READ`, `dm_off_o = R[3][6:0]`. `s = 8..11`: `ch_get_o = 1`, `ch_idx_o = c` | `s = 7`: `r[L][3] <= R[3] + 1` | `lane_park_we_o[L] = 1`, `park_op = IR[29:28]`, `park_ctl = IR[27:0]`, `park_data = SRC(s)` (for `s = 4, 7` this is `dm_rdata_i`) |
| `FIELD`, `RAW` with source 5 or 14, or destination 6, 12, 13 or 14 | | | halt | |
| `PIN` (2) | (only the general condition) | | | park, `park_op = 2`, `park_ctl = IR[27:0]`, `park_data = 0` |
| `SETM` (3), register 0-13 | (`IR[27:23] <= 9` and `!IDLE`), or (`IR[21]` and `HAZ`) | | | `lane_cfg_we_o[L] = 1`, `cfg_reg = IR[27:23]`, `cfg_hi = IR[22]`, `cfg_data = IR[21] ? R[IR[1:0]] : IR[15:0]` |
| `SETM` register 14-31 | | | halt | |
| `LDI` (4) | `HAZ` | | `r[L][IR[27:26]] <= IR[15:0]` | |
| `ALU` (5), ops 0-13 | `HAZ`; ops 12, 13 also `!IDLE` | | | ALU result and flags (section 5.8) |
| `ALU` ops 14, 15 | | | halt | |
| `MEM` (6) | `HAZ` | `dm_off_o = ab ? IR[6:0] : R[ar][6:0]`. Store: `dm_op_o = WRITE`, `dm_hmask_o = hi ? 2'b10 : 2'b01`, `dm_wdata_o = {R[dr], R[dr]}`. Load: `dm_op_o = READ` | if `!ab && pi`: `r[L][ar] <= R[ar] + 1` | load: `r[L][dr] <= hi ? dm_rdata_i[31:16] : dm_rdata_i[15:0]` |
| `BR` (7), cond 0-18 | cond 5, 6 (`M`, `NM`): `!IDLE` | | `pc <= taken ? IR[8:0] : PC1` | |
| `BR` cond 19-31 | | | halt | |
| `LOOP` (8) | `HAZ` | | `pc <= (R[rd] != 1) ? IR[8:0] : PC1` | `r[L][rd] <= R[rd] - 1` (flags unchanged) |
| `WAIT` (9) | section 5.5 | | section 5.5 | section 5.5 |
| `EVT` (10) | never | | `armed[L][k] <= IR[25]`, `tgt[L][k] <= IR[8:0]` | `lane_ev_clr_o[L][k] = IR[24]` |
| `Q POP` (11, op 0) | `QE` or `HAZ` | `dm_op_o = POP` | | `r[L][reg] <= dm_rdata_i[15:0]` |
| `Q PUSH` (op 1) | `QF` or `HAZ` | `dm_op_o = PUSH`, `dm_wdata_o = tagged({16'h0, R[reg]}, tag)` | | |
| `Q PUSHTS` (op 2) | `QF` | `PUSH`, `tagged(ts_i, tag)` | | |
| `Q PUSHCRC` (op 3) | `QF` or `!IDLE` | `PUSH`, `tagged(lane_crc_i[L], tag)` | | |
| `Q PUT` (op 4) | `CV(ch)` or `HAZ` | `ch_put_o = 1`, `ch_idx_o = ch`, `ch_wdata_o = {16'h0, R[reg]}` | | |
| `Q GET` (op 5) | `!CV(ch)` or `HAZ` | `ch_get_o = 1`, `ch_idx_o = ch` | `r[L][reg] <= ch_data_i[ch][15:0]` | |
| `Q PUTSR` (op 6) | `CV(ch)` or `!IDLE` | `ch_put_o = 1`, `ch_idx_o = ch`, `ch_wdata_o = lane_sr_i[L]` | | |
| `Q PUSHI` (op 7) | `QF` | `PUSH`, `tagged({24'h0, IR[15:8]}, tag)` | | |
| `Q` ops 8-15 | | | halt | |
| `MISC NOP` (12, op 0) | never | | | |
| `MISC HALT` (op 1) | `!IDLE` | | `run <= 0`, `attn <= 1`, halt clears (5.7); **PC unchanged** | |
| `MISC IRQ` (op 2) | never | | `attn <= 1` | |
| `MISC CRCINIT` (op 3) | `!IDLE` | | | `lane_cmd_o[L] = 1` |
| `MISC CRCCHK` (op 4) | `!IDLE` or `HAZ` | | | `lane_cmd_o[L] = 2`, `lane_cmd_data_o = {R[1], R[0]}` |
| `MISC STUFFCLR` (op 5) | `!IDLE` | | | `lane_cmd_o[L] = 3` |
| `MISC` ops 6-15 | | | halt | |
| opcodes 13-15 | | | halt | |

Field names: `MEM`: `st = IR[27]`, `dr = IR[26:25]`, `hi = IR[24]`,
`ar = IR[23:22]`, `pi = IR[21]`, `ab = IR[20]`. `LDI`/`LOOP`: `rd = IR[27:26]`.
`EVT`: `k = IR[27:26]`. `Q`: `reg = IR[23:22]`, `ch = IR[21:20]`,
`tag = IR[7:0]`. Bits the ISA marks unused are ignored.

Branch condition `taken` (`BR`, `IR[27:23]`), from values in the D cycle:
0 always; 1 `Z`; 2 `!Z`; 3 `N`; 4 `!N`; 5 `lane_m_i[L]`; 6 `!lane_m_i[L]`;
7 `QE`; 8 `!QE`; 9 `QF`; 10 `!QF`; 11-14 `lane_ev_pend_i[L][cond-11]`;
15-18 `CV(cond-15)`. `Z` and `N` are flops written only in L's own X cycles,
so every branch and loop decision uses registered values.

X-cycle values (`R[...]`, `ch_data_i`) in the table are the values of the D
cycle. `r[L]` is unchanged between L's D and X cycles except by L's own
end-of-D write, which never touches a register the X cycle reads.
`ch_data_i[c]` of a channel taken in the D cycle cannot change before the end
of the following cycle, because the next put lands at the end of a later D
cycle. Implementations may therefore read these in the X cycle instead of
pipelining them.

`MEM` load with post-increment and `dr = ar`: the end-of-D increment is
followed by the end-of-X load, so the register ends up holding the loaded
value.

### 5.3 Waiting

An EXEC or DELIVER that waits changes nothing (no memory, channel, register,
PC, lane strobe) except `qwait`. At the end of every EXEC or DELIVER D cycle:

`qwait <= waited && (a data condition of that row failed)`

where the data conditions are `QE`, `QF`, `!CV(c)` for takes and `CV(c)` for
puts. VECTOR clears `qwait`; NONE leaves it unchanged. `lane_data_wait_o =
qwait`.

The waiting instruction is fetched again in every later slot of the lane until
it commits. This is what the ISA calls "waits" and "parks there": the lane's
PC stays on the instruction, and because no new operation is handed to the
bit hardware, the outputs hold.

### 5.4 Register hazard set `USES(IR)`

| Instruction | Registers in `USES` |
|---|---|
| `FIELD`, `RAW` | `R[s]` if `s` = 0..3; `R3` if `s = 7` |
| `SETM` | `R[IR[1:0]]` if `IR[21]` |
| `LDI`, `LOOP` | `rd` |
| `ALU` | `rd`; plus `rs = IR[21:20]` if `IR[19] = 0` and op is one of 0-8, 11 |
| `MEM` | `dr`; plus `ar` if `ab = 0` |
| `Q POP`, `PUSH`, `PUT`, `GET` | `reg` |
| `MISC CRCCHK` | `R0`, `R1` |
| everything else | none |

The destination of a `FIELD`/`RAW` is not checked against `dst_busy`.
Results are delivered in issue order, so a later write to the same register
lands later.

### 5.5 `WAIT` (mask `M4 = IR[27:24]`, timeout `T = IR[11:0]`)

`hit = (lane_ev_pend_i[L] & M4) != 0`. Evaluated in the D cycle; first
matching row:

| Condition | Result |
|---|---|
| `hit` | commit: `pc <= PC1`, `wsta <= 0`; if `wsta` was 1, `lane_abort_o[L] = 1` in X (cancels the delay) |
| `wsta` and `IDLE` | commit (timeout): `pc <= PC1`, `wsta <= 0` |
| `!wsta` and `T != 0` and `IDLE` | start the timeout: `wsta <= 1`, PC unchanged; in X, park `park_op = 2`, `park_ctl = {16'h0000, T}` (bits 27:12 zero, 11:0 = T) (a `PIN` that changes no output and then waits T `PERIOD` intervals), `park_data = 0` |
| otherwise | wait |

`WAIT` never clears event-pending bits. `WAIT 0, 0` never commits by itself;
it is left only by vectoring or halt.

### 5.6 VECTOR

With `k` the lowest set bit of `lane_ev_pend_i[L] & armed[L]`:

- End of D: `pc <= tgt[L][k]`, `wsta <= 0`, `capw <= 0`, `qwait <= 0`.
- X cycle: `lane_abort_o[L] = 1` and `lane_ev_clr_o[L] = 1 << k`.
- The event stays armed. Other pending events are not cleared and vector in
  later slots if armed.
- The parked operation is discarded and the current one abandoned (the lane
  keeps CRC, stuffing count, SR and bit count, per ISA 3.6). A result that has
  already been produced is **kept** and delivered, because DELIVER has
  priority.
- The first handler instruction is in the lane's next slot: fetched at the F
  cycle 3 cycles after the VECTOR D cycle, decided 5 cycles after it.
- Latency: an event pending from cycle c is acted on at the first D cycle of
  L at or after c (c to c+4), unless deliveries are queued ahead of it.

### 5.7 Halt and single step

Halt causes: `MISC HALT` (waits for `IDLE`), an illegal instruction (opcodes
13-15, `ALU` 14-15, `BR` cond 19-31, `Q` 8-15, `MISC` 6-15, `SETM` register
14-31, `FIELD`/`RAW` source 5 or 14 or destination 6, 12, 13, 14; immediate),
and the host writing `CTRL.run = 0`.

At the end of the D cycle of `HALT` or an illegal instruction: `run <= 0`,
`attn <= 1`, `armed <= 0`, `wsta <= 0`, `capw <= 0`, `qwait <= 0`. PC still
points at the halting instruction. A host write of `CTRL` with `run = 0` makes
the same clears except `attn`. When `run[L]` is 0 the lane holds its bit
hardware idle and enables no output (section 7.4), and the crossbar also
gates its enables (`xbar.md`).

Single step: the host sets `run = 1, pause = 1`. The sequencer then takes no
slot action for the lane, but the lane's bit hardware keeps running and its
outputs stay as programmed. Each host write with `step = 1` makes exactly one
slot action (EXEC, DELIVER or VECTOR) at the lane's next D cycle. An EXEC that
waits uses up the step. `pause = 0` resumes. `pause` has no effect while
halted.

### 5.8 ALU (X cycle)

Operands, captured in the D cycle: `A = R[rd]` (`rd = IR[23:22]`), `B = IR[19]
? IR[15:0] : R[rs]`. Result `y` (16 bits):

| op | Name | `y` | writes `rd` | `N` |
|---|---|---|---|---|
| 0 | `MOV` | `B` | yes | `y[15]` |
| 1 | `ADD` | `A + B` | yes | `y[15]` |
| 2 | `SUB` | `A - B` | yes | borrow: `A < B` (unsigned) |
| 3 | `CMP` | `A - B` | **no** | borrow |
| 4, 5, 6 | `AND`, `OR`, `XOR` | `A & B`, `A \| B`, `A ^ B` | yes | `y[15]` |
| 7 | `SHL` | `A << B[4:0]` (0 when `B[4:0] >= 16`) | yes | `y[15]` |
| 8 | `SHR` | `A >> B[4:0]` (logical; 0 when `B[4:0] >= 16`) | yes | `y[15]` |
| 9 | `INC` | `A + 1` | yes | `y[15]` |
| 10 | `DEC` | `A - 1` | yes | `y[15]` |
| 11 | `ADDC` | `s = A + B` (17 bits); `y = s[15:0] + s[16]` | yes | `y[15]` |
| 12 | `MOVSR` | `lane_sr_i[L][15:0]` | yes | `y[15]` |
| 13 | `MOVSRH` | `lane_sr_i[L][31:16]` | yes | `y[15]` |

At the end of the X cycle: `r[L][rd] <= y` (except `CMP`), `Z <= (y == 0)`,
`N` as above. `ADDC` needs no second fold: when `s[16] = 1`, `s[15:0] <=
0xFFFE`. `LDI`, `LOOP`, `MEM`, `Q` and deliveries do not change `Z`/`N`.

### 5.9 Host-written state (end of a `ph = 1` cycle with `hreg_we_i`)

| Register (`host.md`) | Effect |
|---|---|
| `CTRL[L]` | `run <= w[0]`, `pause <= w[1]`, `step <= w[2]`. If `w[0] = 0`: `armed`, `wsta`, `capw`, `qwait` <= 0 |
| `PC[L]` | `pc <= w[8:0]` if `run[L] = 0`; ignored otherwise |
| `IRQSTAT` (0x04) | `attn[L] <= 0` for each `w[L] = 1` |

No sequencer-initiated write to these registers happens in a `ph = 1` cycle
(no lane has its D cycle there), so host and lane writes never collide.

### 5.10 Instruction rate and costs

**Guarantee.** Every lane has exactly one slot action every 5 cycles, at
fixed phases (D at `ph = L+2`, X at `ph = L+3`), whatever the other lanes, the
host and the pins do. While a lane is eligible and none of its instructions
waits, it commits one instruction every 5 cycles: 12 M instructions/s at 60
MHz. Consecutive commits of one lane have D cycles exactly 5 apart. There is
no faster case.

Costs in slots (one slot = 5 cycles):

| Item | Slots |
|---|---|
| Any instruction that does not wait | 1 |
| Each retry of a waiting instruction | 1 |
| Delivery of a result to `R0`-`R3`, `HOSTQ`, `MEM`, `C0`-`C3` | 1, taken at the lane's first slot after the result appears (it pre-empts the instruction) |
| Delivery to `CAP` | 2 |
| Event entry (VECTOR) | 1, then the handler's first instruction |
| `FIELD`/`RAW` with destination `NONE` | no delivery |

Every instruction and delivery makes at most one data-memory access, in its D
cycle. A host-out push is one access and one slot. A `CAP` record needs two
accesses and so two of the lane's slots. Five wheel positions are enough
because each lane owns one data-memory cycle per turn, and nothing needs two
accesses in one slot except `CAP`, which is spread over two slots.

### 5.11 Run-ahead, in one place

- A bit operation (`FIELD`, `RAW`, `PIN`, or a `WAIT` timeout) is **handed to
  the lane** by writing its parked-operation register in the instruction's X
  cycle (`lane_park_we_o` with op, control bits and resolved source data).
  Source values come from the sequencer's state as it is in the D cycle. That
  includes the pop or channel take, which happen in the D cycle. Only `CRC`
  and `SR` sources are resolved by the lane, when the operation starts.
- The instruction is then complete, and the lane's next instruction runs in
  its next slot. The lane starts the parked operation when the current one
  ends (lane spec).
- While the current operation runs and nothing is parked, the sequencer
  keeps executing the following instructions. It stops (retries every slot,
  PC held) as soon as an operation is parked, so the next bit operation it
  reaches is parked ready to start on the next tick and nothing passes it.
  This is arch 5.2 read literally. Deliveries and event entries still happen
  while it is stopped.
- An instruction also waits when:
  - it uses a register that is the destination of an operation not yet
    delivered (`HAZ`);
  - it is a **sync instruction** and the bit hardware is not `IDLE`. The sync
    instructions are `SETM` of mode registers 0-9, `HALT`, `CRCINIT`,
    `CRCCHK`, `STUFFCLR`, `MOVSR`, `MOVSRH`, `PUSHCRC`, `PUTSR`, `BR M`/`BR
    NM`, and the start of a `WAIT` timeout;
  - its queue or channel data is not there.
- Results come back as the lane's result register (`lane_res_valid_i`, dst,
  data). It is delivered in a DELIVER slot and acknowledged in that slot's X
  cycle. `M`, `SR` and the CRC are lane outputs read only by sync
  instructions, so they are final when read.
- Ordering: deliveries happen in completion order, which is issue order.
  Every other effect happens in program order at commit.

## 6. X-cycle outputs (summary)

In lane L's X cycle the lane strobes (`park_we`, `res_ack`, `abort`, `ev_clr`,
`cfg_we`, `cmd`) are exactly those listed for the action and instruction
committed in the preceding D cycle, and are 0 for NONE or a waiting action.
They come from flops (the D/X pipeline register), not from combinational
decode. Strobes to lane L are 0 in every cycle that is not L's X cycle.

## 7. Lane <-> sequencer interface (assumed; to reconcile with `lane.md`)

This is the contract this spec assumes from the lane side. Signal names are
the lane's (`_i` into the lane).

### 7.1 Signals into the lane (all registered in the sequencer)

| Lane port | From | Timing and meaning |
|---|---|---|
| `run_i` | `lane_run_o[L]` | 1 = lane live. Changes only at the end of L's D cycle or the end of a `ph = 1` cycle |
| `park_we_i` | `lane_park_we_o[L]` | only in L's X cycle; never when `park_full_o = 1` in the same cycle |
| `park_op_i[1:0]` | shared | 0 `FIELD`, 1 `RAW`, 2 `PIN` |
| `park_ctl_i[27:0]` | shared | ISA bits `[27:0]` of that instruction. For `FIELD`/`RAW` with source `IMM` (12), `[8:0]` is the immediate and the lane must take `seek = 0`, `tag = 0` |
| `park_data_i[31:0]` | shared | resolved source value (`SRC(s)`); 0 for `PIN`, `CRC`, `SR`, `NONE` |
| `res_ack_i` | `lane_res_ack_o[L]` | only in L's X cycle, only when `res_valid_o = 1`; the lane clears the result at the end of the cycle |
| `abort_i` | `lane_abort_o[L]` | only in L's X cycle: abandon current op (keep CRC, stuffing count, SR, bits done), discard parked op, keep the result, outputs keep their levels |
| `ev_clr_i[3:0]` | `lane_ev_clr_o[L]` | only in L's X cycle: clear those pending bits at the end of the cycle |
| `cfg_we_i`, `cfg_reg_i[4:0]`, `cfg_hi_i`, `cfg_data_i[15:0]` | `SETM` | only in L's X cycle. Registers 0-9 only when the lane is idle; `EVC0`-`EVC3` (10-13) may arrive while an operation runs. Numbers 14-31 never arrive (they halt) |
| `cmd_i[1:0]`, `cmd_data_i[31:0]` | `MISC` | only in L's X cycle, only when idle. `CRCCHK`: `M <= (CRC state == cmd_data_i)` |
| `data_wait_i` | `lane_data_wait_o[L]` | for `TICK.stretch_hold` (ISA 2: "holds the clock line low while it waits") |
| `in_i[3:0]` | `xbar.md` | registered lane inputs |

Guarantees from the sequencer: in any cycle, at most one of `park_we_i`,
`cfg_we_i`, `cmd_i != 0`, `res_ack_i`, `abort_i` is set, except that `abort_i`
can come with `ev_clr_i`. All strobes are 0 while `run_i = 0`.

### 7.2 Signals from the lane (all registered in the lane)

| Lane port | Required behaviour |
|---|---|
| `busy_o` | current-operation register occupied |
| `park_full_o` | parked-operation register occupied. It becomes 1 only in the cycle after `park_we_i` |
| `res_valid_o`, `res_dst_o[3:0]`, `res_data_o[31:0]` | result of a finished operation whose destination is not `NONE`. Data already tagged (ISA 3.1). Dst and data are stable from the cycle `res_valid_o` rises until the end of the cycle with `res_ack_i` |
| `dst_busy_o[3:0]` | bit n = Rn is the destination of the current op, the parked op or the held result; destination `MEM` marks R3 |
| `m_o` | `M`. Changes only when an `EXPECT` completes or at the end of a `CRCCHK` cycle |
| `ev_pend_o[3:0]` | sticky pending bits; cleared by `ev_clr_i` (recommended: also held at 0 while `run_i = 0`) |
| `sr_o[31:0]` | shift register; stable while idle |
| `crc_o[31:0]` | CRC value (crc.md `value_o`); stable while idle and no `CRCINIT` |
| `out_val_o[3:0]`, `out_oe_o[3:0]` | to the crossbar. `out_oe_o = 0` whenever `run_i = 0` |

**Idle stability** (relied on to decide in D and act in X): if `busy_o = 0`
and `park_full_o = 0` in cycle t and `park_we_i = 0` in cycle t, then both
are 0 in cycle t+1, and `res_valid_o(t+1) = res_valid_o(t) && !res_ack_i(t)`.
In words: nothing starts and no result appears without a park.

**Result overrun.** The sequencer delivers a waiting result at the lane's
next slot. If it is deliverable, that is within 1-5 cycles of `res_valid_o`
rising, plus the X cycle for the ack. It waits longer only for a full queue or
channel, or a paused lane. The lane spec must say what happens when an
operation finishes while the result register is still full. Suggestion: keep
the second result in SR and do not start the parked operation until the
register frees, so nothing is lost and the wire waits.

### 7.3 Shared buses (lane lemma)

`park_op/ctl/data`, `cfg_reg/hi/data` and `cmd_data` are common to all lanes
and carry the X owner's values. **The lane spec must prove**: the lane's next
state and outputs do not depend on these inputs in cycles where its own
corresponding strobe is 0. The non-interference proof (`top.md`) uses this
lemma instead of per-lane gating, which would cost about 320 gates.

### 7.4 Halt

When `run_i` is 0 the lane must, from the next cycle, have `busy_o =
park_full_o = res_valid_o = 0`, `dst_busy_o = 0` and `out_oe_o = 0`. Mode
registers keep their values.

## 8. Worked timing

### 8.1 One lane, cycle by cycle

Lane 0 (F at ph 0, D at ph 2, X at ph 3). Lanes 1-3 halted. `pc[0] = 0x010`
before cycle 0. Program:

```
010: LDI   R0, 0x0041
011: FIELD DRIVE 8, lsb, R0          ; operation A, destination NONE
012: ALU   ADD R0, #1                ; runs while A runs
013: Q     PUSH R0                   ; runs while A runs
014: FIELD SAMPLE 8, lsb, dst R1     ; operation B, parked behind A
015: ALU   CMP R1, #0x55             ; stopped: B parked, then R1 pending
016: BR    NZ, 010
```

| Cycle | ph | F (address) | I | Lane 0 D: action and end-of-D effects | Lane 0 X: outputs and end-of-X effects |
|---|---|---|---|---|---|
| 0 | 0 | L0 @010 | (host) | | |
| 1 | 1 | L1 @000 (halted) | L0: IR <= LDI | (host D) host writes `CTRL0.run = 1`; `run[0] = 1` from cycle 2 | |
| 2 | 2 | L2 | L1 | EXEC `LDI`: `R0 <= 0x0041`, `pc <= 011` | |
| 3 | 3 | L3 | L2 | | no strobes |
| 4 | 4 | host idle | L3 | | |
| 5 | 0 | L0 @011 | | | |
| 6 | 1 | | IR <= FIELD | | |
| 7 | 2 | | | EXEC `FIELD` A: `park_full = 0`, `HAZ = 0`: commit, `pc <= 012` | |
| 8 | 3 | | | | `park_we = 1`, op 0, ctl = `IR[27:0]`, data `0x00000041`. The lane is idle, so A becomes current at once (lane spec) |
| 10 | 0 | L0 @012 | | (A running: `busy = 1`, `park_full = 0`) | |
| 12 | 2 | | | EXEC `ADD`: commit, `pc <= 013` | |
| 13 | 3 | | | | `R0 <= 0x0042`, `Z <= 0`, `N <= 0` |
| 15 | 0 | L0 @013 | | | |
| 17 | 2 | | | EXEC `PUSH`: `QF = 0`: `dm_op = PUSH`, `dm_wdata = 0x00000042`; out pointer +1 at end of cycle; `pc <= 014` | |
| 20 | 0 | L0 @014 | | | |
| 22 | 2 | | | EXEC `FIELD` B: `park_full = 0`: commit, `pc <= 015` | |
| 23 | 3 | | | | `park_we = 1`: B parked behind A; `park_full = 1`, `dst_busy[0] = 0010` from 24 |
| 25 | 0 | L0 @015 | | | |
| 27 | 2 | | | EXEC `CMP R1`: `park_full = 1`: **wait**, PC stays 015 | none |
| 32, 37, ... | 2 | | | same wait each slot (after A ends and B starts, `park_full = 0` but R1 is in `dst_busy`, so `HAZ`: still waiting) | |
| | | | | *B completes; lane has `res_valid = 1`, dst R1, data `0x...55` from cycle 101* | |
| 100 | 0 | L0 @015 | | | |
| 102 | 2 | | | **DELIVER**: `R1 <= 0x0055`; the fetched `CMP` is dropped | |
| 103 | 3 | | | | `res_ack = 1`; `dst_busy` clears from 104 |
| 105 | 0 | L0 @015 | | | |
| 107 | 2 | | | EXEC `CMP`: commit, `pc <= 016` | |
| 108 | 3 | | | | `Z <= 1`, `N <= 0` |
| 110 | 0 | L0 @016 | | | |
| 112 | 2 | | | EXEC `BR NZ`: `Z = 1`, not taken: `pc <= 017` | |

Instructions 010-014 commit at D cycles 2, 7, 12, 17, 22: one per 5 cycles.
Operation A is handed over at the end of cycle 8, and the lane keeps executing
(`ADD`, `PUSH`) while A runs. B is handed over at the end of cycle 23, and then
the sequencer stops until B's result is back.

### 8.2 Delivery before vector (lane 2: D at ph 4, X at ph 0)

E1 armed with `tgt[2][1] = 0x080`. At cycle 41 the lane has a result with
dst `CAP`, and E1 becomes pending at cycle 46.

| Cycle | ph | Lane 2 |
|---|---|---|
| 44 | 4 | D: DELIVER `CAP`, `capw = 0`, `QF = 0`: `dm_op = PUSH`, `dm_wdata = ts_i` (the counter value in cycle 44); `capw <= 1` |
| 45 | 0 | X: none |
| 49 | 4 | D: `res_valid = 1` still: DELIVER, `capw = 1`: `PUSH` result word; `capw <= 0` |
| 50 | 0 | X: `res_ack = 1` |
| 54 | 4 | D: `ev_pend & armed = 0010`: VECTOR k = 1: `pc <= 0x080` |
| 55 | 0 | X: `abort = 1`, `ev_clr = 0010` |
| 57 | 2 | F @080 |
| 59 | 4 | D: first handler instruction |

### 8.3 All lanes at once

Every cycle each stage has exactly one owner (table in section 1). For
example, in a `ph = 3` cycle: lane 3 fetches, the IR receives lane 2's word,
lane 1 makes its decision and data-memory access, and lane 0 executes.

## 9. Required properties

Unless stated, "lane L's D cycle" means a cycle with `ph_o[(L+2) mod 5] = 1`.
The checker can model `IR` as `pm_rdata_i` delayed by one cycle.

1. **Wheel.** `ph_o` is one-hot, `ph_o = 00001` in cycle 0, and
   `ph_o(t+1) = {ph_o(t)[3:0], ph_o(t)[4]}`.
2. **Fetch.** In a cycle with `ph_o[k]`, k <= 3: `pm_addr_o = obs_pc_o[k]`;
   with `ph_o[4]`: `pm_addr_o = 0`.
3. **Host position free.** In `ph = 1` cycles: `dm_op_o = NONE`,
   `ch_put_o = ch_get_o = 0`. In every cycle at most one of `ch_put_o`,
   `ch_get_o` is set.
4. **Never over- or under-run.** `dm_op_o = POP` implies `!qe_i[owner]`;
   `PUSH` implies `!qf_i[owner]`; `ch_put_o` implies `!ch_valid_i[ch_idx_o]`;
   `ch_get_o` implies `ch_valid_i[ch_idx_o]`.
5. **Strobe discipline.** Every lane-L strobe is 0 outside lane L's X cycle
   and whenever `run_o[L] = 0`. `lane_park_we_o[L]` implies
   `!lane_park_full_i[L]`; `lane_res_ack_o[L]` implies `lane_res_valid_i[L]`;
   at most one of park/cfg/cmd/ack/abort per lane per cycle.
6. **Slot locality.** Lane L's state (`obs_*`, `run_o`, `attn_o`,
   `lane_data_wait_o`) changes only at the end of L's D cycle, at the end of
   L's X cycle (`r`, `Z`, `N` only), or at the end of a `ph = 1` cycle with
   `hreg_we_i` to a lane-L register (`CTRL`, `PC`, `IRQSTAT`).
7. **Halted and paused lanes are inert.** If `run_o[L] = 0`, or
   `pause[L] = 1` and `step[L] = 0`, in L's D cycle: in that cycle the
   sequencer makes no data-memory or channel request; at its end no lane-L
   state changes except by the host; and in the next cycle there are no lane-L
   strobes.
8. **No lane runs before the host starts it.** `run_o[L]` rises only at the
   end of a `ph = 1` cycle with `hreg_we_i`, `hreg_addr_i = CTRL[L]`,
   `hreg_wdata_i[0] = 1`.
9. **Priority.** In an eligible D cycle, `lane_res_valid_i[L]` implies the
   DELIVER row of 5.1 and not VECTOR or EXEC. Otherwise armed pending events
   imply VECTOR to the lowest index.
10. **Each instruction** has exactly the effects of its row in 5.2, 5.5, 5.8
    (one property per row: waits imply no effect; commits imply the listed D
    outputs, end-of-D updates and X outputs). Data-memory and channel outputs
    are 0/NONE in D cycles where the row lists none.
11. **Wait means no effect.** In an EXEC or DELIVER D cycle whose wait
    condition holds: no request, no state change except `qwait`, and no
    strobes in the following X cycle.
12. **Hazard and sync.** No EXEC commits with `lane_park_full_i[L] = 1`
    (except `WAIT` with `wsta = 1`) or `HAZ = 1`, and no sync instruction commits unless `IDLE` in its D cycle.
13. **CAP pairing.** A first `CAP` delivery (`capw` 0 to 1) is followed, at
    lane L's next eligible D cycle with `!qf_i[L]`, by a second whose
    `dm_wdata_o` is `lane_res_data_i[L]`. Between them L takes no other slot
    action and `dm_op_o` in L's D cycles is `PUSH` or `NONE`.
14. **Vector.** VECTOR sets `obs_pc_o[L]` to `obs_tgt_o[L][k]` for the lowest
    armed pending k, and gives `lane_abort_o[L] = 1` and `lane_ev_clr_o[L] =
    1 << k` in the X cycle; `armed` is unchanged.
15. **Halt.** `HALT` commits only when `IDLE`; illegal encodings halt at once.
    Both leave `obs_pc_o[L]` unchanged and set `attn_o[L]`, and clear `run`,
    `armed`, `wsta`, `capw`, `qwait`.
16. **Host PC write while running is ignored**; host `CTRL.run = 0` gives the
    clears of 5.9.
17. **Step.** With `run = pause = 1`, a host `step = 1` write is followed by
    exactly one non-NONE slot action, after which `step = 0`.
18. **Rate.** In every window of 5 consecutive cycles each lane has exactly
    one D cycle. If a lane is eligible and its instructions commit, its
    `obs_pc_o` advances at intervals of exactly 5 cycles.
19. **Waits are re-evaluated every slot.** If an instruction waits at D cycle
    t and its conditions are false at t+5 (and `res_valid`/events/eligibility
    allow EXEC), it commits at t+5.
20. **Registered decisions.** `obs_z_o`/`obs_n_o` change only at the end of an
    X cycle that follows a committed `ALU` D cycle.
21. **WAIT.** `WAIT` with `wsta = 0`, `T != 0`, `IDLE` and no hit parks exactly
    once (`park_ctl = {16'h0000, T}`) and sets `wsta`. A later hit commits it
    with an abort; reaching `IDLE` commits it without one. `ev_clr` is never
    set by `WAIT`.
22. **ADDC** equals the reference end-around-carry sum for all operands.
23. **Two-copy non-interference (sequencer level).** Two copies A, B. Lane L's
    inputs (`lane_*_i[L]`) are equal in A and B. Host bus writes to lane-L
    registers are equal; other host writes are free. `qe_i[L]`, `qf_i[L]`,
    `pm_rdata_i` and `dm_rdata_i` are equal in L's I and X cycles
    respectively. `ch_*` inputs are free, and lane L's `IR` is assumed never
    to name a channel (no `BR` 15-18, `Q` 4-6, `FIELD`/`RAW` source or
    destination 8-11). Everything else is free. Then all lane-L strobes,
    `lane_run_o[L]`, `lane_data_wait_o[L]`, the shared buses in L's X cycles,
    lane L's `obs_*`, and `dm_*` outputs in L's D cycles are equal in A and B.
24. **Covers:** every opcode and sub-op commits; each wait reason occurs;
    DELIVER for each destination class; a full `CAP` pair; VECTOR for each k;
    `WAIT` ends by event and by timeout; `MEM` load with `dr = ar`
    post-increment; step; host halt of a running lane; each illegal class;
    two lanes passing a channel in one turn.

## 10. Implementation notes (not normative)

- **Pipeline registers.** `ph` (5, one-hot ring), `IR` (32), and a D/X
  register of about 55 flops: ALU operands A/B (32, reused to carry the
  28-bit park control and op when parking), ALU op and register index (6),
  write source (2), flag enable, the lane strobes and their small fields
  (`ev_clr` 4, `cmd` 2, `park_we`, `park_mem`, `cfg_we`, `ack`, `abort`). Park
  data, `SETM` data and `CRCCHK` data are muxed in the X cycle from registers,
  `ch_data_i`, `IR`-derived bits carried in A/B, and `dm_rdata_i` (allowed by
  the stability note in 5.2).
- **Register file write port.** Per lane, one write per cycle: in L's D cycle
  from the D datapath (`LDI`, `GET`, deliver-to-register, post-increment), in
  L's X cycle from the X datapath (ALU, `LOOP`, `POP`, `MEM` load). Select by
  phase: one 16-bit 2:1 mux per lane, not per register.
- **Timing paths (16.67 ns, slow corner; SRAM clock-to-out 6.45 ns).**
  - D: `IR` -> field decode -> lane/register select -> data-memory address
    or write data -> SRAM pins. Setup about 0.9 ns for `A_MEN`/`A_WEN`.
  - D: `IR` -> register select -> 16-bit increment (post-increment) ->
    register write mux.
  - D: register select -> `!= 1` (`LOOP`) -> PC.
  - X: operands -> 16-bit add/sub -> zero test -> `Z`. Compute `Z` for
    `SUB`/`CMP` as `A == B` in parallel with the subtractor, and use a
    prefix adder.
  - X: `dm_rdata_i` (6.45 ns after the edge) -> half select -> register file
    or `lane_park_data_o` -> lane flop: about 9 ns of slack before mux delay.
- **Flops:** 488 per-lane state (122 x 4) + 5 + 32 + about 55 = **about 580**.

## 11. Decisions and ISA issues

Each item gives the affected text, the decision this spec implements, and
proposed wording. Items 1-6 answer the questions the brief asked to settle.

1. **Pipeline depth: four stages, not three.**
   Affected: arch 11, "Each instruction is spread over three cycles of the
   lane's five-cycle slot spacing: fetch, decode (registered), execute."
   ISA 2: "Each instruction takes three clock cycles (fetch, decode,
   execute)". The SRAM samples its address at a clock edge and returns data
   registered one cycle later, and the instruction word must be registered
   before decode. So fetch needs two cycles (address, then instruction
   register), then decode and execute.
   Proposed: "Each instruction passes through four pipeline cycles inside the
   lane's five-cycle slot: address, instruction register, decode (where data
   memory and channels are accessed), execute. Every stage is shared by the
   lanes in turn, so this costs no speed."

2. **Exact cost of every instruction class.** Every instruction is one slot
   if it does not wait. A delivery is one slot (`CAP` two), taken at the
   lane's next slot and pre-empting the instruction. An event entry is one
   slot. Affected: ISA 2, "Fields are gap-free when the instructions between
   them finish before the current field ends."
   Proposed addition: "Each instruction costs one slot of five cycles, and so
   does each retry while it waits. A field or raw operation with a
   destination other than `NONE` costs one more slot when it completes (two
   for `CAP`), taken before any other work of the lane. An event entry costs
   one slot. The compiler counts all of these in the gap-free check."

3. **What "waits" means, and run-ahead.** A waiting instruction keeps its PC,
   has no effect, and is re-evaluated in every slot. The run-ahead executes
   instructions only while nothing is parked. It stops at a sync instruction
   (list in 5.11), at a register hazard, or at missing queue or channel data.
   Once the next bit operation is parked, nothing executes until that
   operation becomes current. This matches arch 5.2 literally and the
   toolchain model (`model.py`: "run-ahead stops here").
   Affected: ISA 2, "the sequencer runs ahead until it reaches the next
   instruction that needs the bit hardware or depends on this field's result,
   and parks there", and arch 5.2.
   Proposed: "A bit operation (`FIELD`, `RAW`, `PIN`) is handed to the lane's
   parked register and completes. While an operation is parked, the sequencer
   executes nothing more; otherwise it continues. An instruction also waits,
   retrying every slot, when it uses a register that an undelivered operation
   will write; when it is a sync instruction (`SETM` of registers 0-9, `HALT`,
   `CRCINIT`, `CRCCHK`, `STUFFCLR`, `MOVSR`, `MOVSRH`, `PUSHCRC`, `PUTSR`,
   `BR M`, `BR NM`, a `WAIT` timeout) and any operation is running or parked;
   or when its queue or channel data or space is not there."

4. **Queue pointer ownership.** The host owns the host-in write pointer and
   the host-out read pointer. The lane (that is, the sequencer) owns the
   host-in read pointer and the host-out write pointer. The host can write
   lane-owned pointers only while the lane is halted; writes while it runs
   are ignored (`mem.md`).
   Affected: arch 6, "ring buffers with hardware pointers".
   Proposed addition to arch 6: "Each pointer has one writer while the lane
   runs: the producer advances the write pointer, the consumer the read
   pointer. The host can reset the lane's pointers only while the lane is
   halted."

5. **Are five slots enough for `CAP` and pushes?** Yes. A 32-bit push is
   one access in one slot. A `CAP` record is two accesses in two consecutive
   slots of the same lane, made atomic by delivery priority. Borrowing the
   host's cycle when idle was rejected because it would make lane timing
   depend on the host.
   Affected: ISA 3.0 `CAP`: "push timestamp word, then value word".
   Proposed addition: "(two slots; the timestamp is the counter value when
   the first word is written)".

6. **`CAP` timestamp point.** The timestamp is `ts_i` in the D cycle of the
   first delivery slot. That is 1-5 cycles after the lane's result appears,
   depending on wheel phase (more if the queue is full). The listener's
   timestamps have this phase-dependent offset (up to 83 ns) on top of the
   constant pipeline delay. Cheap fix if it matters: a 3-bit per-lane age
   counter subtracted when writing (12 flops plus a 32-bit decrement in D).
   Not adopted. Affected: arch 8, "A free-running 32-bit cycle counter gives
   16.7 ns resolution". Proposed addition: "A capture record's timestamp is
   taken when the record is written, 1-5 cycles after the field ends."

7. **`WAIT` does not clear pending events; the UART receive example is
   broken as written.** In ISA 5, `idle: WAIT E0` follows a byte whose data
   bits made falling edges, so E0 is already pending and `WAIT` falls
   through on a stale edge. Decision: pending bits are cleared only by `EVT
   k, clear` and by vectoring to k. Proposed ISA 3.6 text: "`WAIT` does not
   clear pending bits. To wait for a new event, clear it first with `EVT k,
   off, clear`." Fix the example by inserting `EVT 0, off, clear` before
   `idle: WAIT E0` (and branching to it).

8. **`WAIT`'s timeout uses the bit hardware.** A pending masked event
   completes `WAIT` at once (as in the toolchain model). A timeout is started
   only when the bit hardware is idle. It is handed to the lane as a `PIN`
   that changes nothing and holds T `PERIOD` intervals, and an event that
   arrives first cancels it with `abort`. The sequencer has no timer of its
   own. Affected: ISA 3.6 `WAIT` row. Proposed addition: "The timeout starts
   when the current field has ended."

9. **`PIN` hold runs in the lane.** `PIN` is parked like a field. The
   sequencer runs ahead during its hold. Affected: ISA 3.3, "`PIN` waits for
   the current field to finish." Proposed addition: "...and is then held for
   the given intervals by the lane, while the sequencer continues to the next
   bit operation."

10. **Vectoring details.** Vectoring clears only the vectored event's pending
    bit, keeps it armed, discards the parked operation, and delivers a result
    already produced before entering the handler. Affected: ISA 3.6, "When an
    armed event fires: the current field is abandoned and `PC = target`."
    Proposed addition: "The parked next field is discarded. The event's
    pending bit is cleared and it stays armed. A field that had already
    completed still delivers its result first. Among several armed pending
    events, the lowest number wins."

11. **Event targets cost 144 flops (36 per lane).** Implemented as the ISA
    says. Option: a 9-bit vector base per lane with event k entering at
    `base + k` (one `BR` per entry, +1 slot latency) saves 108 flops (about
    7,300 um2). Recommend deciding before the 20 Dec freeze.

12. **Reserved encodings halt.** Everything reserved halts like a reserved
    opcode: `ALU` 14-15, `BR` cond 19-31, `Q` 8-15, `MISC` 6-15, `SETM`
    register 14-31, `FIELD`/`RAW` source 5 or 14, destination 6, 12, 13 or 14.
    This is the same choice as the toolchain model, and it costs a few gates in
    D. Affected: ISA 3.7, "Opcodes 13-15 halt the lane"; ISA 6 rule 2.
    Proposed: "A reserved opcode, operation code, condition, mode register or
    operand code halts the lane with its outputs released."

13. **Flags.** All `ALU` ops set `Z` and `N`, including `MOV`, `MOVSR`,
    `MOVSRH`. `CMP` does not write `rd`. `LDI`, `LOOP`, `MEM`, `Q` do not
    change flags. Affected: ISA 3.5 ALU row "Sets `Z`; `N` is bit 15 of the
    result, or the borrow for `SUB`/`CMP`". Proposed addition: "`CMP` writes
    only the flags. `LDI`, `LOOP`, `MEM` and `Q` leave the flags unchanged."

14. **ALU operand form.** `rd = rd op src` with `src = imm ? imm16 : rs`.
    `SHL`/`SHR` shift by `src[4:0]` (logical; 16-31 give 0, as in the model).
    `INC`/`DEC` ignore `src`.
    `ADDC` is `rd + src` plus its own carry-out. The ISA gives the op list
    only. Proposed: add a column "operation" with exactly table 5.8.

15. **`MEM` corners.** The address is `Rn[6:0]` (the upper bits are ignored).
    Post-increment adds 1 to the whole 16-bit register. Post-increment with
    `absolute` is ignored. With a load and `dr = ar`, the loaded value wins.
    Proposed addition to ISA 3.5.

16. **`Q` word formats and tags.** `PUSH rs` = `{16'h0, rs}`, `PUSHI` =
    `{24'h0, imm8}`, `PUSHTS` = timestamp, `PUSHCRC` = CRC value (with final
    XOR). For all four, a non-zero `tag` replaces bits 31:24. `PUT` =
    `{16'h0, rs}`; `GET` and `POP` load bits 15:0. The ISA gives the tag rule
    only for `FIELD`. Proposed: add these formats to ISA 3.7.

17. **Source `IMM` and the tag field.** For `FIELD` and `RAW` with source
    `IMM`, `[8:0]` is the immediate, so `seek = 0` and `tag = 0`. `RAW` with
    `IMM` uses `[8:0]` too (ISA 3.2 gives no immediate field). Proposed: say
    so in 3.1 and 3.2.

18. **`HALT` waits for the last field and leaves PC on itself.** It also
    raises `attn` (host interrupt). `MISC IRQ` raises `attn` without halting.
    Proposed ISA 3.7: "`HALT` waits until every field has finished, then
    halts with PC at the `HALT`."

19. **Single step needs a paused state.** Halted means outputs released and
    bit hardware idle (ISA 6 rule 3), so stepping is done from "running,
    paused": the bit hardware runs, and the sequencer takes one slot action
    per step. Affected: arch 9 "per-lane control (run, halt, single-step,
    PC)". Proposed: "...run, pause, single-step (while paused), PC".

20. **`M` lives in the lane.** It is written by `EXPECT` completion and by
    `CRCCHK`, which compares the CRC **state** (as ISA 3.7 says) inside the
    lane. "CRC-good" in arch 5.3 is this `M` after `CRCCHK`. Proposed arch
    5.3: "flags (zero, negative, match, event pending, queue empty/full)".

21. **`BR QE/QF` do not wait for undelivered pushes.** A `FIELD ... HOSTQ`
    pushes when it is delivered, not when it is issued, so `QF` read
    meanwhile reflects only deliveries so far. Proposed note in ISA 3.6.

22. **Channels are linearised in the D cycle, in wheel order.** Puts and
    takes from different lanes never coincide; a later lane sees an earlier
    lane's put 1-4 cycles later. With two producers on one channel, the
    earlier lane in wheel order wins and the other can starve (see
    `channels.md`). Proposed arch 7 addition: "A channel has one producer
    and one consumer by convention."

23. **Host program-memory writes are not blocked while lanes run.** The
    hardware does not check. Arch 6 says "Written by the host while lanes are
    halted"; that stays a loader rule.

24. **Lane-to-lane interference through the host.** Arch 13 item 6 says "a
    lane's pins are identical cycle for cycle whatever the other lanes and
    the host do". The host necessarily affects the lane it controls. Proposed:
    "whatever the other lanes do and whatever the host does to other lanes,
    unless a channel or a shared pin connects them" (exact form in `top.md`).

25. **Divergences from `toolchain/rosetta/model.py`** (written in parallel;
    its `MODEL_DECISIONS.md` was not in the tree when this was written). This
    spec has adopted the model's choices on run-ahead, `WAIT` events, reserved
    codes, shift width and `SETM` of `EVC` registers. These differences remain,
    and the model must change or this spec must, before the compiler's
    schedule check is trusted:
    - **Pipeline timing (model D2, `EXEC_STAGE = 2`).** The model executes 2
      cycles after the slot. Here decisions, memory and channel effects and PC
      are at slot+2 (D), but parks, ALU results and lane commands land at the
      end of slot+3 (X). A field therefore starts one cycle later than in the
      model.
    - **Deliveries cost a slot.** The model delivers a finished result and then
      runs an instruction in the same slot, and writes both `CAP` words at
      once. That needs two data-memory accesses per slot, which one port in one
      cycle per lane cannot do. Here a delivery takes the slot (two for `CAP`).
    - **`CAP` timestamp (model D17: first capture tick of the field).** That
      needs a timestamp latch per lane (32 flops each) or a timestamp bus into
      the lanes. Here it is taken at delivery (decision 6).
    - **`CAP` with one free entry.** The model waits for 2 free entries; here
      each word waits separately.
    - **Lane input latency (model D3: 3 cycles).** Here 2 cycles (`xbar.md`
      X1).
    - **Hazard rules (model D24).** The model tracks the resources of the
      *current* field (SR, M, CRC, stuffing, queue, channel, register) and
      blocks only instructions that touch them. Here the coarser rule applies:
      register hazards from `dst_busy`, and the sync list waits for full
      `IDLE`. So the model lets `CRCINIT` pass a field that does not use the
      CRC, and makes `BR QF` wait for a pending push; this spec does neither.
    - **`WAIT` timeout start.** The model counts from the first execution of
      `WAIT`; here from when the bit hardware is idle.
    - **Queue layout (model D7: host-in queue at offset 64, 32 words).** Here
      the host-in queue is at offset 0 and the host-out queue at the top, with
      sizes in `QCFG` (`mem.md` 3.1).
