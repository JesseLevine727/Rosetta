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

Known risks, not rulings: Ethernet receive has about half a cycle of
jitter margin with single-edge sampling (decide on the FPGA whether to
sample on both clock edges); a USB bus reset is timed by software.

Still to rule: F11-F32 details (operand
alignment, CRC as a source with the lsb flag, PIN wait units, SETM halves).
