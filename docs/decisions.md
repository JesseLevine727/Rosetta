# Overseer rulings (toward instruction set v0.3)

The instruction set v0.2, the toolchain model (`toolchain/MODEL_DECISIONS.md`,
findings F1-F32, decisions D1-D43) and the shared-logic block specs
(`docs/blocks/sequencer.md` decisions 1-25) were written independently and
disagree in places. These rulings settle the disagreements. `docs/isa.md`
v0.3 will restate them; until then this file wins.

| # | Topic | Ruling |
|---|---|---|
| 1 | Pipeline | Four stages: address, instruction register, decode, execute. A lane still executes at most one instruction per five cycles. |
| 2 | Input latency | Three cycles from pin to lane: two synchroniser flops before the crossbar, one register after it. |
| 3 | Capture in `BAUD` (F3) | The data input is captured the input latency after the lane's own capture edge, so a clock of 3 + 3 cycles reads correctly. |
| 4 | Clock output in `EDGE` (F4) | Drives only for `stretch_hold`. |
| 5 | Stretching (F5, F6) | As model decisions D10 and D11. |
| 6 | `early` (F2) | Applies in `EDGE` and `RECOVER` only. |
| 7 | End of a field (F1) | The result is available at the last capture. The bit hardware is free at the end of the last bit in `BAUD`, at the last capture in `EDGE` and `RECOVER`. |
| 8 | Result delivery | Writing a result to a queue or memory costs one of the lane's slots (two for a capture record). The model must charge for it. |
| 9 | Run-ahead | Nothing executes past a parked operation. The fixed list of instructions that wait for idle bit hardware is the one in `sequencer.md`. A waiting instruction is retried every slot. |
| 10 | Events (F7, F8, F9) | `WAIT` never clears pending bits; `EVT ... clear` does. The trigger field is 3 bits: 0 off (reset value), 1 A at level L, 2 A edge to L, 3 A any edge, 4 A and B both low. K is 6 bits. Level triggers fire once per episode. |
| 11 | Event targets | One 9-bit target per event, as in the ISA. |
| 12 | Timestamps | Latched in the lane at the field's last capture tick (16 bits; the sequencer rebuilds the rest), so listener timestamps are exact to the cycle. |
| 13 | Undefined codes (F10) | All halt the lane. |
| 14 | Synchronisers | Both flops stay ahead of the crossbar multiplexer (the cheaper arrangement would cost settling time). |
| 15 | Reset | A two-flop reset synchroniser at the top level. |
| 16 | Queue pointers | Producer owns the write pointer, consumer the read pointer; the host writes lane-owned pointers only while the lane is halted. |
| 17 | Channels | Host clear and a lane operation on the same channel never coincide; one producer and one consumer per channel is a compiler rule. |

Rulings on the lane specs (`docs/blocks/lane.md` decisions I1-I30):

| # | Topic | Ruling |
|---|---|---|
| 18 | Round trip (I1) | Output flop plus input latency is 4 cycles. In `BAUD` the data input is sampled 4 cycles after the capture tick. Accepted, including the longer clock phase between back-to-back sampling fields at 3 + 3 cycles. |
| 19 | End of field, `PIN` in `BAUD` (I2, I7) | As lane.md: complete at the last capture, busy to the end of the last bit, `PIN` waits for the timer to stop. |
| 20 | Differential input (I13) | While the two lines are equal the data input holds and no bit is captured. |
| 21 | Undefined values in mode-register data (I17) | Treated as off by the lane. Ruling 13 (halt) applies to instruction codes only, because the sequencer does not decode register data. |
| 22 | Prescaler (I8) | One free-running prescaler per lane. The model changes to match. |
| 23 | Pulse thresholds (I11) | Only `THR0` exists (one bit per pulse: short or long). `THR1` and `THR2` are deleted. |
| 24 | Limits (I19, I20) | Receive de-stuffing needs `RECOVER` or `EDGE` timing. `seek` matches at most 16 bits. |
| 25 | I2C controller STOP while a target stretches (I28) | Use an event plus `WAIT`; no new hardware. |
| 26 | Interface additions | `ts_i`/`res_ts_o` for timestamps; `busy_o` covers the end of the last bit; a parked operation starts two cycles after its park; a result overrun holds the completing operation. sequencer.md follows lane.md here. |

Rulings after the cross-review of the lane specs (`lane.md` 12.2, R1-R11):

| # | Topic | Ruling |
|---|---|---|
| 27 | `EXCHANGE` with a differential input | Always takes its sample as a bit (exception to ruling 20; no protocol needs the combination). |
| 28 | `seek` in `BAUD` without `stretch_honour` | Compiler rule: `(PERIOD2 - 1) * prescale + 1` must exceed the 4-cycle round trip. |
| 29 | `BAUD` timestamps | Latched at the last sample, a fixed 4 cycles after the last capture tick; the compiler subtracts the offset. This refines ruling 12. |
| 30 | sequencer.md follows lane.md | `lane_busy_i` covers the end of the last `BAUD` bit; new input `lane_res_ts_i` (16 bits per lane); `CAP` timestamp word is `{ts[31:16] - (ts[15:0] < res_ts), res_ts}`; a parked operation is current two cycles after its park; an overrun holds the completing operation; sync instructions wait for the end of the last `BAUD` bit; top level routes `ts_o` to the lanes. |
| 31 | xbar.md text | Stale where it says two flops and t-2: the proven RTL has three flops on the pin path (t-3), two on the host path, one on loopback. In loopback the round trip is 2 cycles, not 4. |

Rulings on field.md ambiguities (they amend field.md):

| # | Section | Ruling |
|---|---|---|
| 32 | 2.1 / 5.3 | `seek` applies to `SAMPLE` and `EXPECT` only; on `DRIVE` and `EXCHANGE` the bit is ignored. |
| 33 | 5.1 / 5.3 | `EXCHANGE` with `early`: if a capture tick arrives in the field's start cycle, the early launch wins (`lnch = 1`) and that capture takes no bit. |
| 34 | 5.3 / 5.6 | A data-position sample while `tail = 1` takes no bit; the field completes. |
| 35 | 3 / 6.2 / 8 | `hold` is cleared when the held operation completes, as well as at promotion, abort and halt. |
| 36 | 6.1 / 6.2 | A held `RAW SAMPLE` completes on acceptance, like a held `FIELD`. |
| 37 | 8 | Abort keeps `lstf` and `pstf`; promotion clears them. |
| 38 | 2.1 / 6.2 / 8 | Cases the sequencer never produces are still defined: an ack in an abort cycle is ignored; a park in the same cycle as a promotion sets `pk_v`; `CRCCHK` wins over an `EXPECT` completion writing `M`, and acts whenever `run_i` is high. |

Rulings on tick.md ambiguities (they amend tick.md):

| # | Section | Ruling |
|---|---|---|
| 39 | 8 | "At most one clock request per cycle" holds because the idle request made by a `TICK` write wins: a stretch start or a `BAUD` launch in that cycle makes no clock request (the launch keeps its `lt`). |
| 40 | 4.1 | `creq` and `bt_stop` are active whenever the mode is `BAUD` or `PULSE`; only `lt`, `ct`, `cte`, `hte` depend on a `BAUD`-timed field being current. |
| 41 | 4.2 | A second `fld_first` while the stretch release is counting does not restart the count. |
| 42 | 5 | `spend` clears on the `BAUD` sample strobe even if the field has ended; `sarm` clears at the round trip whether or not `st` fired; the countdown is armed on every `BAUD` capture. If a delayed clock change is cancelled, `spend` stays set until abort, halt or a `TICK` write; the lane-level proof must show this cannot happen in a running program. |
| 43 | 7 | A non-raw timer start with a count of 0 does not start the timer. A new start suppresses the old timer's `ot_done` in that cycle. A `TICK` write does not cancel the timer. |
| 44 | 4.3, 4.1 | A `TICK` write produces no `lt`; when nothing applies `cnt` holds; a stop clears `bt_run` and `lr` only. |
| 45 | 9.2 | Delay unit: while halted all outputs are 0; a new delayed request flushes the pending one; the raw counter may saturate at 255. |

Rulings on coding.md ambiguities (they amend coding.md):

| # | Section | Ruling |
|---|---|---|
| 46 | 3.2 / 5.2 | `rxv` is gated by a valid differential pair in every receive coding, including Manchester and pulse-symbol; the `mph` update on `st_i` is not. `rxb_o`/`rxv_o` are not gated by `st_i`. |
| 47 | 3.2, 4.2, 4.3 vs 4.4 | In a same-cycle collision the clearing action wins: a `CODE` or `TICK` write sets `mph` to 1 over a Manchester sample; abort or a write clears `hp` over `tx_go_i`; a `PIN` write of `out0` sets `tl_q` over a request. A release clears `hp` only. |
| 48 | 4.2, 5.8 | `dreq_o` is gated by `run_i` only; same-cycle requests have priority wire bit, release, `RAW DRIVE`, second change. `dreq_dly_o` is 0 exactly for `RAW DRIVE` requests and 1 otherwise. `dpay_o` is unspecified when `dreq_o` is 0. |

Rulings from the field proofs:

| # | Topic | Ruling |
|---|---|---|
| 49 | `seek` in `BAUD` timing | Not allowed; the compiler rejects it (this replaces ruling 28). Reason: a `BAUD` seek launches one bit beyond its match, and that bit's sample can still be pending when the next field starts, which then sends or takes one wrong bit. `seek` is only needed where the far end supplies the timing (`PULSE`, `RECOVER`, `EDGE`), where the problem does not arise. The hardware is unchanged; the field proofs make no end-to-end claim for a field that follows a `BAUD` seek. |
| 50 | Tick properties the field relies on | The tick block must also prove: `lt` and `ct` never in one cycle; in `BAUD` timing `lt` only with `launch_req`; `st = ct` outside `BAUD` timing and `st = ct && cu_smp` with `stretch_honour`; in `BAUD` timing a `ct` needs an `lt` since the last `ct`. |
| 51 | Result destination | The field guarantees only that `res_dst_o` is not `NONE`; the other excluded codes are instruction-level and belong to the sequencer (ruling 13). |

Rulings from the tick proofs:

| # | Topic | Ruling |
|---|---|---|
| 52 | Round-trip countdown (refines ruling 42) | A `BAUD` capture restarts the countdown and discards an older running count, so every sample strobe is exactly the round trip after its own reference cycle. The first tick RTL let the older count run on and fired the sample strobe one cycle after a sampling capture; the proofs caught it and the RTL was fixed. |
| 53 | `lt` and `ct` in one cycle (corrects ruling 50) | False in `EDGE` mode while the lane holds the clock (`stretch_hold`) and the clock's low phase is shorter than the 4-cycle round trip. Compiler rule: `stretch_hold` requires a clock low phase of at least 5 cycles. Open obligation: a lane-level proof that the coincidence cannot occur under that rule, since the field proofs assume it never does. |
| 54 | Tick proof range | The unbounded tick proof covers `PERIOD` and `PERIOD2` from 0 to 7 and timer counts 0 to 7; the bounded run has no range limit to depth 32. Longer periods rest on the counter logic not depending on the period's size, which is not itself proven. |
| 55 | Ruling 53's compiler rule is not sufficient (found by the lane-top proof, 14 Nov) | "`lt` and `ct` never coincide" under `EDGE` + `stretch_hold` holds only with all of: (a) the bus clock's low phase is at least 5 cycles (ruling 53); (b) the lane's clock output is open-drain, its idle level is released, and capture is on the rising edge; (c) no `PIN` writes `out3` while `EDGE` + `stretch_hold` is configured. Without (b) or (c) the lane itself can put a low pulse shorter than the round trip on the shared clock. The compiler must enforce (a)-(c). The remaining conditions of the proof are sequencer guarantees (strobes only in X cycles, `qwait` written only at D, `run` changes only at D or ph = 1). Proven unbounded as `a_e_r53_ltct` in `formal/lane` (`prove_r53`). |
| 56 | Corrections to lane.md found by the lane-top proof | 9.4: `res_valid(t+1) = res_valid(t) && !ack` holds for running cycles only (a halt clears it). 9.11: an abort keeps `crc_o` except in the cycle after a `SETM` 8/9, where `crcw_q` re-initialises the CRC. 5.4: `crcw_q` is not gated by `run_i` (the other init terms are). 2.1 gains the sequencer guarantee "`data_wait` = 0 in the cycle after a `SETM TICK`" (sequencer.md 5.3), which the tick proof relies on. |

Known risks, not rulings: Ethernet receive has about half a cycle of
jitter margin with single-edge sampling (decide on the FPGA whether to
sample on both clock edges); a USB bus reset is timed by software.

Still to rule: F11-F32 details (operand
alignment, CRC as a source with the lsb flag, PIN wait units, SETM halves).
