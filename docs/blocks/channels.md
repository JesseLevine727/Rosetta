# Block spec: `rosetta_chan` (lane channels and timestamp)

Four 32-bit channel registers, each with a valid bit, shared by all lanes
(architecture section 7). Also the free-running 32-bit timestamp counter
(architecture section 8). All channel operations come from the sequencer in
the D cycle of the lane that executes them (`sequencer.md`). There is one D
owner per cycle, so channel operations from different lanes never coincide
and need no arbitration.

## 1. Ports

| Port | Width | Meaning |
|---|---|---|
| `clk`, `rst_n` | 1 | clock; async active-low reset |
| `ch_put_i` | 1 | put `ch_wdata_i` into channel `ch_idx_i` at the end of this cycle (if empty) |
| `ch_get_i` | 1 | take channel `ch_idx_i` at the end of this cycle (if valid) |
| `ch_idx_i` | 2 | channel number |
| `ch_wdata_i` | 32 | put data |
| `ch_valid_o` | 4 | valid bits; registered |
| `ch_data_o` | 128 | channel c data at `[32c +: 32]`; registered |
| `ts_o` | 32 | timestamp; registered |
| `hreg_we_i` | 1 | register write strobe (asserted only when `ph = 1`) |
| `hreg_addr_i` | 7 | register address |
| `hreg_wdata_i` | 32 | write data |
| `hreg_rdata_o` | 32 | read data for `TIME` (0x03), `CHSTAT` (0x06), `CHDATA0-3` (0x08-0x0B); 0 otherwise. Combinational |

## 2. Behaviour

State: `data[0..3]` (32 bits each), `valid[0..3]`, `ts` (32 bits). Reset:
all 0.

At every clock edge:

- `ts <= ts + 1` (mod 2^32). So `ts_o = t mod 2^32` in cycle t, counted from
  internal reset release. It wraps every 71.6 s at 60 MHz.
- Put: if `ch_put_i && !valid[ch_idx_i]`: `data[ch_idx_i] <= ch_wdata_i`,
  `valid[ch_idx_i] <= 1`. A put to a valid channel does nothing.
- Get: if `ch_get_i && valid[ch_idx_i]`: `valid[ch_idx_i] <= 0`. `data` is
  unchanged (a take leaves the last value readable). A get from an empty
  channel does nothing.
- Both in one cycle: each is judged on the valid bit at the start of the
  cycle; the sequencer never does this.
- Host clear: if `hreg_we_i` and `hreg_addr_i = CHSTAT`: `valid[c] <= 0` for
  each `hreg_wdata_i[c] = 1`. This takes priority over a put in the same
  cycle, which cannot happen because host writes are at `ph = 1`, where no
  lane has a D cycle.

Register reads: `TIME` = `ts_o`; `CHSTAT` = `{28'h0, valid}`; `CHDATA c` =
`data[c]`.

The sequencer reads `ch_valid_o` and `ch_data_o` in the D cycle to decide
waits and branch conditions (`BR C0`-`C3` valid). It drives `ch_put_i` only
when the channel is empty and `ch_get_i` only when it is valid
(`sequencer.md` property 4). So the conditional updates above are a second
guard, not the mechanism.

## 3. Same channel, same wheel turn

The D cycles in a turn come in this fixed order:

| ph | 0 | 1 | 2 | 3 | 4 |
|---|---|---|---|---|---|
| D owner | lane 3 | host (never touches channels) | lane 0 | lane 1 | lane 2 |

Each operation takes effect at the end of its D cycle, and every later D
cycle sees it. The outcome depends only on which D cycle comes first in time:

| Case | Outcome |
|---|---|
| A puts, then B gets, B's D cycle after A's | B takes A's value `(B - A) mod 5` cycles after the put |
| B's get comes before A's put | B waits (sees empty), retries in its next slot 5 cycles later, and takes the value then |
| Two lanes put to the same empty channel | the earlier D cycle succeeds; the other waits until the channel is emptied |
| Two lanes get from the same valid channel | the earlier D cycle takes it; the other waits |
| A puts and A gets the same channel | in different instructions, so different slots; program order |

Latency of a value from lane A to lane B is `(B - A) mod 5` cycles (1-4),
plus up to 5 more if B tried just before. Throughput per channel is at most
one word per 5 cycles.

**Starvation.** With two producers (or two consumers) on one channel and a
busy partner, the producer whose D cycle comes earlier after each take can win
every time, and the other may wait forever. This is deterministic. Channels
are meant to have one producer and one consumer (`sequencer.md` decision 22);
the compiler should enforce that.

**Non-interference.** Two lanes affect each other through this block only
by both naming the same channel. A lane that never names a channel neither
reads nor changes channel state.

## 4. Required properties

1. `ts_o = 0` in cycle 0 after reset, and `ts_o(t+1) = ts_o(t) + 1 mod 2^32`.
2. **Put.** `ch_put_i && !valid[i]` at t (i = `ch_idx_i`) gives `valid[i] = 1`
   and `data[i] = ch_wdata_i(t)` at t+1. `ch_put_i && valid[i]` changes
   nothing.
3. **Get.** `ch_get_i && valid[i]` gives `valid[i] = 0` at t+1, with
   `data[i]` unchanged.
4. **Frame.** `data[c]` changes only by a successful put to c. `valid[c]`
   changes only by a successful put, a successful get, or a host clear of c.
   Channels other than `ch_idx_i` are unchanged by lane operations.
5. **Host clear** behaves as in section 2; `CHSTAT` and `CHDATA` reads return the state.
6. **Linearisation.** A reference model that applies the operations of each
   cycle in cycle order to a 4-entry valid/data array gives the same
   `ch_valid_o`/`ch_data_o` in every cycle.
7. Covers: put then get on each channel; a put to a full channel (ignored); a
   get from an empty channel (ignored); a host clear; `ts` passing
   `0xFFFFFFFF`. The wrap cover needs `ts` treated as a free initial value
   after reset, or an abstraction; justify the depth.

## 5. Worked timing

Lane 0 (D at ph 2) puts `0x1234` into C0; lane 1 (D at ph 3) gets C0 into
R2; lane 2 (D at ph 4) also tries to put into C0.

| Cycle | ph | D owner | Request | `valid[0]` during the cycle | End of cycle |
|---|---|---|---|---|---|
| t | 2 | lane 0 | `Q PUT C0, R1` (R1 = 0x1234): `ch_put_i`, idx 0 | 0 | `data[0] <= 0x1234`, `valid[0] <= 1` |
| t+1 | 3 | lane 1 | `Q GET R2, C0`: `ch_get_i`, idx 0 | 1 | `valid[0] <= 0`; sequencer writes R2 = 0x1234 |
| t+2 | 4 | lane 2 | `Q PUT C0, R0`: channel empty, so put | 0 | `data[0] <= R0`, `valid[0] <= 1` |
| t+5 | 2 | lane 0 | another `PUT C0`: valid, so waits | 1 | none |
| t+6 | 3 | lane 1 | `GET C0`: takes lane 2's value | 1 | `valid[0] <= 0` |
| t+7 | 4 | lane 2 | | 0 | |
| t+10 | 2 | lane 0 | retried `PUT C0`: succeeds | 0 | `valid[0] <= 1` |

Lane 0 lost the race in turn t+5 because lane 2's put at t+2 filled the
channel first: two producers, as section 3 warns.

## 6. Flop count

`data` 128 + `valid` 4 + `ts` 32 = **164**.
