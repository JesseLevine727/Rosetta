# Model decisions and isa.md findings

`rosetta/model.py` is meant to be exact to the clock cycle, and `docs/isa.md`
v0.2 often does not say on which cycle something happens. This file records
every choice the model makes in those places, quotes the isa.md text the
choice interprets, and lists the problems found in isa.md along the way.
When `docs/blocks/*.md` specs exist for a block, the model will be
reconciled with them. Timing choices are named constants at the top of
`model.py`, or a code comment tagged with the decision number (`D11`).

The rule used throughout: **take the simplest rule that makes the section 5
examples work, and say so**.

## Conventions of the model

- `Lane.step(pins)` is one clock cycle. `pins` are the levels of in0-in3
  during that cycle. The return value is out0-out3 (value and enable) after
  that cycle's clock edge. A bench sees the outputs of cycle t when it
  computes the inputs of cycle t+1, which is one cycle of wire delay.
- Cycle 0 is reset. The timestamp counter is the cycle number, modulo 2^32.
- Within a cycle the order is:
  1. the input pipeline;
  2. the event detectors;
  3. the bit hardware (ticks, launches, captures, field completion, parked
     operation starting);
  4. the sequencer, if this is the lane's execute cycle;
  5. armed events (jump);
  6. scheduled output changes.

## Timing constants

| Constant | Value | isa.md / arch text | Choice |
|---|---|---|---|
| `SLOT_PERIOD` | 5 | arch 5.1 "The wheel has 5 slots: one per lane and one for the host port" | Lane k's slot cycles are t ≡ k (mod 5), counted from reset. The host slot is not modelled (host accesses are instantaneous, D16) |
| `EXEC_STAGE` | 2 | isa 2 "Each instruction takes three clock cycles (fetch, decode, execute) inside the lane's five-cycle slot spacing" | All of an instruction's effects happen in its execute cycle, slot + 2. Fetch and decode have no visible effect (D2) |
| `INPUT_DELAY` | 3 | arch 10 "All inputs pass through two-flop synchronisers"; arch 11 "Lane inputs are registered after the crossbar" | The lane logic in cycle t sees the pins of cycle t-3 |
| `MIN_COUNT` | 1 | isa 4 `PERIOD`, `PERIOD2` are 16 bits, and 0 is not excluded | A count of 0 behaves as 1 (D4) |
| `RECOVER_CAPTURE_NUM/DEN` | 1/2 | isa 2 RECOVER "half a PERIOD after it" | Capture at floor(PERIOD × prescale / 2) cycles after the bit starts (D5) |
| `PULSE_SATURATE` | 255 | arch 4.1 "thresholds (8 bits, prescaled)" | Pulse durations saturate at 255 prescaled cycles (D6) |
| `RAW_SATURATE` | 0xFFFF | isa 3.2 | As specified |
| `DEFAULT_IN_QUEUE`, `DEFAULT_OUT_QUEUE` | words 64-95 and 96-127 | isa 1 "ring buffers inside the lane's memory region"; arch 9 "queue pointers" | Base and size are constructor arguments, standing in for the host-port pointer registers (D7) |

## Decisions

Each entry gives the isa.md text it interprets (in quotes, with its
section), then the rule the model uses.

**D1. Slot phase.** arch 5.1. Lane k executes in cycles where
(t − k − 2) mod 5 = 0. So the same program on lane 3 runs 2 cycles
earlier than on lane 0. `test_lane_index_only_shifts_timing` checks
that this is the only difference.

**D2. Instruction atomicity.** isa 2 "three clock cycles (fetch, decode,
execute)". Each instruction reads and writes state in its execute cycle. An
event that fires in the same cycle is handled after the instruction (step 5).
An event that fires earlier redirects the PC before the execute cycle, so an
instruction "in flight" is never executed. This differs from a pipeline that
would discard a fetched word: there, the latency would be up to one slot
longer.

**D3. Input latency.** See `INPUT_DELAY`. Event detectors, edge detection,
captures and `stretch_honour` all see the delayed inputs.

**D4. Zero counts.** A `PERIOD`, `PERIOD2` or RECOVER bit time of 0 acts as
1. The PIN and WAIT unit also uses the clamped `PERIOD`.

**D5. RECOVER timer.** isa 2 "Launch tick: start of the bit. Capture tick:
half a PERIOD after it; any data edge restarts the bit timer."
- In RECOVER mode the bit timer runs all the time, including between fields.
- A data-input edge sets it to 0, and that cycle is a launch tick. The
  capture tick is at count floor(P×PERIOD/2). At count P×PERIOD the timer
  wraps to 0, which is another launch tick.
- A field that starts before the capture tick of the current bit takes that
  bit; otherwise it waits for the next launch tick. This is what makes the
  section 5 UART receive work: its start-bit `EXPECT` starts a few cycles
  after the falling edge and still samples mid-bit.

**D6. PULSE durations.** The interval since the previous data edge is
counted in prescaled cycles and saturates at 255. The symbol is
(d ≥ THR0) + (d ≥ THR1) + (d ≥ THR2), so unused thresholds must be set to
255.

**D7. Queue layout.** See the constants above. Field results, `CAP` records
and `Q` pushes go to the host-out ring; field sources and `Q POP` read the
host-in ring. `MEM` can address any word of the region, including words
inside the rings (finding F32).

**D8. SETM and the bit hardware.** isa 3.4 says nothing about timing.
`SETM` to registers 0-9 (`TICK` to `CRCCFG`) waits until the bit hardware
is idle: no active or parked operation, and the BAUD timer stopped.
`SETM EVC0`-`EVC3` takes effect at once.

**D9. Writing TICK.** It stops the BAUD timer, clears the RECOVER counter,
and ends any clock stretching. If `clock_out` is 1, `out3` goes to the idle
level at once (open-drain: released when idle is 1).

**D10. `stretch_honour`.** isa 4 "in BAUD, pause while the clock input reads
low after release".
- At the capture time, `out3` goes to the capture level (it is released).
- The capture tick itself (sampling the data input) happens in the first
  cycle at or after that time in which the clock input reads the capture
  level. `PERIOD2` counts from then.
- Even without a stretching target, the high phase is therefore delayed by
  the input latency.

**D11. `stretch_hold`.** isa 2 "In EDGE mode with TICK.stretch_hold the lane
holds the clock line low while it waits."
- Stretching starts at a launch edge of the clock input that arrives while
  no field is active and the sequencer is blocked on data (an empty queue or
  channel, a full queue or channel, or a pending field result). A `WAIT`
  instruction does not count, so a target that is not addressed (sleeping
  in `WAIT none`) never holds the bus.
- `out3` is then driven to 0.
- Stretching ends when the next field starts. That field launches its first
  bit at once, and the clock is released `PERIOD` × prescale cycles later,
  which gives the data setup time before SCL rises. In EDGE mode `PERIOD`
  has no other use.
- An event jump also ends stretching.

**D12. PULSE receive.** isa 2 "capture tick: each input edge; the symbol is
the time since the previous edge compared with THR0-THR2".
- With `CODE.rx = SYMBOL`, one bit is captured per high pulse, at its
  falling edge (the logical level, after `invert`). The bit is 1 if the
  symbol is 1 or more.
- Low intervals produce nothing. The `latch` and reset conditions are
  events.
- With `rx = LEVEL` or `NRZI` in PULSE mode, every edge is a capture of the
  new level.
- A transmitting field in PULSE mode uses BAUD timing.

**D13. Launch delay.** isa 4 `THR2L[15:8]`, `CODE[10]` "launch delay applies
to the clock output", `CODE[11]` "in prescaled cycles".
- The delay postpones every output change caused by a launch tick, which
  includes the low edge of a pulse-width bit.
- With `CODE[10]` it applies **instead** to every clock-output transition
  (launch and capture), and the data is not delayed. This is the only
  reading that lets Stress skew clock against data (G9).
- `PIN` changes are never delayed.

**D14. Releasing the data output.** isa says nothing about what a receiving
field does to `out0`. `SAMPLE` and `EXPECT` release `out0` (and `out1` when
differential) at their first launch tick. In RECOVER, when the field takes
the current bit, and in PULSE mode, they release it when they start.
Without this, the I2C target's ACK would hold SDA low for the whole next
byte.

**D15. Field results.**
- A field's result is written to its destination in the cycle of its last
  capture tick (or of its trailing stuff bit).
- If the destination is full (host-out queue, or a channel already valid),
  the result waits in a write buffer. The sequencer then executes nothing,
  and counts as blocked on data, until the result fits.
- The wire carries on: a parked field still starts on time.
- Hardware will have to do memory writes in the lane's slot (arch 5.1). The
  model ignores that latency, which is not visible on the pins (finding F17).

**D16. Host side.** `host_write`, `host_read` and `Channels` act instantly
between cycles.

**D17. CAP timestamp.** isa 3.0 "push timestamp word, then value word". The
timestamp is the counter value at the field's **first capture tick**.

**D18. CRC input.** The check unit takes data bits only, never stuff bits.
`DRIVE` and `EXCHANGE` feed it the driven bit, `SAMPLE` and `EXPECT` the
sampled bit. The update happens at the bit's capture tick.

**D19. Stuffing scope.**
- Only fields with `[18]` set, while `STUFF.enable` is 1, feed the stuffing
  unit. The run count carries across them and is unchanged by unstuffed
  fields. `STUFFCLR` clears it.
- A stuff bit that falls due after a field's last data bit belongs to that
  field: the field is one wire bit longer. At most one such bit is added.
- The unit follows `docs/blocks/stuff.md`. `test_ones_stuffing_round_trip`
  checks the exact wire bits.

**D20. Level triggers.** isa 4 "An event fires when its trigger has happened
and its level conditions have then held for K more prescaled cycles."
- Trigger 0 ("A at level L") and trigger 3 ("both low") count as
  happening when their condition becomes true.
- They fire once per episode: the condition must go false before they can
  fire again.
- Writing `EVCk` restarts the detector, so a condition that is already true
  counts as starting at the write.
- The window counts K × prescale cycles. If the condition fails at any cycle
  inside the window, the trigger is cancelled.
- For trigger 2 ("any edge"), the level that must hold is the level A
  changed to.

**D21. `EVC.b_must` = 3.** This code is undefined. The B condition is never
satisfied, so the event never fires.

**D22. `early`.** isa 3.1 "put the first bit on the wire when the field
starts, not at the first launch tick".
- EDGE: the first bit is driven when the field starts, and the field then
  waits for a capture edge. A field that is parked starts when the previous
  field completes (its last capture edge), so with `early` its first bit
  changes right after that capture edge. That is the literal reading.
- BAUD: no effect. A BAUD field always starts at a launch tick (finding F2).
- RECOVER: if the field must wait for the next launch tick, the first bit is
  driven at once and that launch tick does not change it again.

**D23. Events.**
- An event that fires sets its pending flag.
- If the event is armed, the following happens: its pending flag is cleared
  (the handler has been taken); the active and parked operations are
  abandoned; any wait is cancelled; stretching is released; and PC is set to
  the target.
- If several armed events fire in one cycle, the lowest number wins and the
  others stay pending.
- Events stay armed after firing. Arming an event that is already pending
  does not jump.
- `WAIT` does **not** clear pending flags; `EVT k, ..., clear` does.
- On abandon, `SR` holds the bits shifted so far, and the number of bits
  done is kept (isa 3.6). The BAUD timer stops, and the clock output keeps
  its level.

**D24. Run-ahead.** isa 2 "the sequencer runs ahead until it reaches the
next instruction that needs the bit hardware or depends on this field's
result, and parks there."
- Instructions that need the bit hardware: `FIELD`, `RAW` and `PIN` park
  behind the active operation. There is one parking place. While it is
  occupied, the sequencer executes nothing.
- An instruction waits (and is retried at every slot) while the incomplete
  field has a result it touches:
  - `SR` (`MOVSR`, `MOVSRH`, source `SR`, `PUTSR`);
  - `M` (`BR M`/`NM`, `CRCCHK`), when the field is an `EXPECT`;
  - the CRC (`CRCINIT`, `CRCCHK`, `PUSHCRC`), when the field updates it;
  - the stuffing count (`STUFFCLR`), when the field stuffs;
  - the destination register, for reads and writes;
  - `R3` and memory, when the destination is `MEM`;
  - the host-out queue (every push, `BR QF`/`QNF`), when the destination is
    `HOSTQ` or `CAP`;
  - the channel, when the destination is a channel.
- Everything else runs ahead.

**D25. HALT.** `HALT` waits until the bit hardware is idle, so a UART stop
bit is sent in full. Halting releases every output, per isa 6 rule 3 "A lane
that is halted ... drives nothing". Reserved opcodes and undefined codes
halt at once (D29).

**D26. Shifts.** `SHL` and `SHR` shift by `src[4:0]`. A shift of 16 or more
gives 0.

**D27. `Q POP` and `Q GET`** load the low 16 bits of the 32-bit word.

**D28. Source read time.** A field's or RAW's source is read when the
instruction executes, when it is issued or parked, so a parked field
already holds its host word. The exception is `CRC`, which is latched when
the field starts (isa 3.1 "the CRC as it stood before the field"). If the
source is empty, the instruction waits and is not parked. An abandoned
parked field loses the word it popped.

**D29. Undefined codes halt.** isa 6 rule 2 covers only reserved opcodes.
The model also halts, releasing all outputs, on any of these: operand 14;
`CAP` as a source; `CRC`, `IMM` or `SR` as a destination; ALU ops 14-15; BR
conditions 19-31; Q ops 8-15; MISC ops 6-15; `SETM` registers 14-31.

**D30. When a field finishes.** isa 3.3 "PIN waits for the current field
to finish".
- A field is **complete** at its last capture tick. Its result, `M` and `SR`
  are available then, and dependent instructions may proceed.
- In BAUD (and in PULSE transmit) the field **holds the bit hardware until
  the end of its last bit**, which is `PERIOD2` after that capture. A parked
  field launches its first bit exactly there, so consecutive fields have no
  gap, and a parked `PIN` takes effect exactly there.
- In EDGE and RECOVER the hardware is free at the last capture tick. A
  parked field waits for its first launch tick; a parked `PIN` takes effect
  at once, or at the next launch tick with `align`.
- See finding F1 for why the choice matters.

**D31. PIN.**
- A `PIN` holds the bit hardware from the moment it takes effect until its
  wait (wait × `PERIOD` × prescale cycles) has passed. The sequencer runs
  ahead meanwhile, and the next field or `PIN` parks and starts exactly when
  the wait ends.
- `align` defers the effect to the next launch tick. If no tick source is
  running (BAUD with the timer stopped), the `PIN` takes effect at once.
- `PIN` writes value and enable exactly as encoded. It does not apply
  open-drain, invert or differential mapping, and it cancels any delayed
  change still pending for the same output.

**D32. WAIT.** `WAIT` is retried at every slot until a masked event is
pending or the timeout (counted from its first execution) has passed. The
wake latency is therefore up to one slot.

**D33. Clock output.**
- The launch level is the opposite of the capture level, and the capture
  level is 1 when `TICK.capture` is RISING.
- In BAUD, each launch tick drives the launch level and each capture tick
  drives the capture level. At the end of a field's last bit, with no field
  following, the clock goes to the launch level if `rest` is set and to the
  idle level otherwise.
- In EDGE mode, `out3` is driven only by `stretch_hold` (finding F4).

**D34. BAUD timer.**
- A field that starts while the timer is stopped launches its first bit in
  that cycle.
- Capture follows at +`PERIOD`×P cycles, and the next launch at
  +`PERIOD2`×P.
- There is no free-running prescaler: every duration is an exact multiple of
  the prescale factor, counted from when it starts.

**D35. NRZI.** Transmit continues from the last level driven on `out0`, by a
field or a `PIN`. Receive compares with the level at the previous NRZI
capture, which is initially 1.

**D36. SR.**
- After a field, `SR` holds the field's value, right-aligned and
  zero-extended to 32 bits: the driven value for `DRIVE`, and the sampled
  value for the receiving kinds.
- A `DRIVE` with a destination writes the driven value there.
- After an abandoned field, `SR` holds the partial bits.

**D37. EXPECT** sets M = (sampled value == source value masked to n bits).
With `seek`, the window of the last n bits is compared after every bit from
the n-th on, in the field's bit order. The sampled value also goes to the
destination.

**D38. Tags.** A non-zero tag replaces bits [31:24] of the value word of:
`FIELD` and `RAW` results going to `HOSTQ` or `CAP` (for `CAP`, the value
word only); and every `Q` push (`PUSH`, `PUSHTS`, `PUSHCRC`, `PUSHI`).

**D39. CRC as a source.** The source is the check value, `value_o`, which
includes the final XOR. It is shifted out in the field's own bit order, so a
compiler must use msb-first with n = W to send "bit W-1 first" (finding
F15).

**D40. RAW.**
- With source `IMM`, bits [8:0] are the immediate, as in `FIELD`, and there
  is no tag.
- `RAW DRIVE` drives `out0` at the given level (applying `invert` and
  open-drain) for src × prescale cycles, holding the bit hardware.
- `RAW SAMPLE` waits for the next data-input edge and writes
  (edge − previous edge) / prescale, saturated at 0xFFFF. Consecutive parked
  `RAW SAMPLE`s measure consecutive intervals.

**D41. MEM.** The address is the register's low 7 bits. Post-increment is
ignored with absolute addressing. A high-half load zero-extends.

**D42. SETM halves.** `SETM` writes one 16-bit half, and the other half
keeps its value. Writes to the upper half of a 16-bit register are ignored.

**D43. Initial inputs.** Until real inputs have passed through the
synchroniser pipeline, the lane sees `initial_inputs` (default: all high).

## Findings

These are places where isa.md v0.2 is ambiguous, contradictory, or cannot
be implemented as written. Each gives the section, what goes wrong (and
whether the model shows it), and a proposed fix. F1-F11 matter for rung 1.

**F1. When a field "finishes" is undefined.** isa 2 (field instruction),
3.3 ("PIN waits for the current field to finish"), section 5 SPI example.
- If a field finishes at its last capture tick, the SPI example's
  `PIN out2=1` raises CS in the same cycle as the last SCK rising edge, so
  a target misses bit 0. The model did this before D30, and the SPI bench
  received 7 bits.
- If a field finishes at the end of its last bit, CS rises with the final
  SCK falling edge, and `align` only matters in EDGE and RECOVER.
- *Fix:* define completion (result available) at the last capture tick, and
  "bit hardware free" at the end of the last bit in BAUD and at the last
  capture tick in EDGE and RECOVER (D30). Say that a parked `PIN` takes
  effect when the hardware is free. For SPI's `cs_lag` rule the compiler
  then emits `PIN align, wait=1` before deselecting.

**F2. `early` does nothing in BAUD, yet the section 5 SPI controller uses
it.** isa 2 BAUD "launch tick: start of the bit", 3.1 `early`.
- A BAUD field starting from idle begins with a launch tick, which already
  puts bit 7 on MOSI before the first rising SCK edge (mode 0 works without
  `early`). A parked field starts at a launch tick too.
- *Fix:* say that `early` applies to EDGE (and RECOVER) only, and drop it
  from the SPI controller example; or define BAUD's first launch tick
  differently and say how.

**F3. BAUD capture samples the input 3 cycles before the clock edge it
generates.** isa 2 (capture tick), arch 10 and 11 (two-flop synchroniser and
input register), section 5 SPI at 10 MHz.
- The lane raises SCK in cycle t and samples MISO in cycle t. Through the
  synchroniser, that is the pin as it was in cycle t-3.
- With `PERIOD` = `PERIOD2` = 3 (the example), the sample point is the
  falling edge, where the target is just changing MISO. Every bit after the
  first therefore reads the previous bit: in the model 0xC3 comes back as
  0xE1 (`test_isa_example_timing_reads_miso_one_bit_late`).
- `PERIOD` 4 / `PERIOD2` 2 (still 10 MHz, using G3's unequal halves) works
  with an ideal target. A real target's clock-to-output time needs more
  margin.
- With the 2-cycle lane-input latency of `docs/blocks/xbar.md` (X1), 3/3
  works, but only with a target that answers in the same cycle (the bench's
  ideal one): the margin is `PERIOD` − latency − 1 = 0 cycles. One cycle of
  target t_CO breaks it again.
- *Fix (choose one):*
  - (a) delay the data capture in BAUD by the input latency, so that it
    samples the pin at the clock edge;
  - (b) add a capture delay field (like the launch delay);
  - (c) make the compiler require `PERIOD` ≥ input latency + 1 + target
    t_CO, and state the maximum controller read rate (about 7.5 MHz with
    equal halves).

  (a) is cheapest: it moves two flops.

**F4. A clock output in EDGE mode deadlocks the I2C target example.** isa 2
("With the clock output enabled, out3 goes to the launch level at the launch
tick and to the capture level at the capture tick"), 4 `TICK[7]`, section 5
I2C target (`EDGE(in1) | ... | clock-out open-drain | stretch_hold`).
- Taken literally, the target would pull SCL low at every falling edge it
  sees. It would then never see a rising edge.
- *Fix:* say that in EDGE mode the clock output is driven only by
  `stretch_hold` and is otherwise at its idle level (D33).

**F5. `stretch_hold` has no start, end or setup time.** isa 2, 4
`TICK[12]`.
- The text does not say what counts as "waiting": a data wait, `WAIT`, or
  instructions that have not finished.
- Stretching may start only after a launch (falling) edge; a target must
  not create one.
- When the data arrives, the first bit must be on SDA **before** SCL is
  released, by t_SU;DAT (250 ns at 100 kHz). Released at once, the setup
  time is 0.
- The field must then launch at once, not wait for a launch edge that
  already passed.
- *Fix:* as D11. Name the setup time: use `PERIOD`, which EDGE mode does not
  otherwise use.

**F6. `stretch_honour` does not account for the synchroniser.** isa 4
`TICK[11]` "pause while the clock input reads low after release".
- After the lane releases SCL, the clock input reads low for at least 3
  cycles even with no stretching.
- A stretching target changes SDA during the stretch, so the controller
  must sample when SCL is really high, not when it released it.
- *Fix:* define the capture tick as the first cycle in which the clock
  input reads the capture level, with `PERIOD2` counted from there (D10).

**F7. `WAIT` and pending flags.** isa 3.6.
- The text does not say whether `WAIT` clears the pending bits it wakes on,
  or whether taking an armed event clears its pending bit.
- The section 5 UART receiver loops on `WAIT E0` with `EVC0` = "in0 edge
  to low". The data bits' falling edges leave E0 pending, so after every
  frame `WAIT E0` returns at once. The program then polls the line once per
  bit until the next real start bit. It works by accident.
- *Fix:* specify that `WAIT` leaves pending flags alone (the model's
  choice), so a program must `EVT k, ..., clear` before waiting (added to
  the README version); or that `WAIT` clears the flags in its mask on wake.
  In either case, say that taking a handler clears that event's pending
  flag (D23).

**F8. An event detector cannot be disabled.** isa 4 `EVC`, `STUFF[5]`.
- The reset value 0 means "A = in0 at level low, K = 0", so every
  unconfigured detector fires whenever in0 is low and sets its pending
  flag. `BR Ek` on an unconfigured event is meaningless.
- Event 3 is also raised by a stuffing violation when `STUFF[5]` is set.
  Its pending bit then also fires whenever in0 is low, unless `EVC3` is
  given a never-true condition.
- *Fix:* add an enable bit (`EVC[16]`, written with `SETM EVCk.hi`), or a
  "never" trigger code. Say that `STUFF[5]` replaces `EVC3`'s detector
  rather than being OR-ed with it.

**F9. Event trigger details.** isa 4.
- The text does not say whether level triggers (0 and 3) fire once or
  continuously.
- For trigger 2, it does not say which level must hold during K.
- K is 7 bits (`[15:9]`, at most 127), while arch 11 says "Event counters
  are 8 bits".
- K is "prescaled cycles", so at prescale 8 the I2C example's K = 18 becomes
  144 cycles (2.4 µs). That is still under t_HD;STA, but it is a trap.
- *Fix:* state D20's rules. Make K and the counter widths agree. Consider
  counting K in clock cycles, because the confirm window is about the
  synchroniser skew, not the bit rate.

**F10. Undefined encodings.** isa 3.0 (operand 14 unlisted; "-" entries for
`CAP` as a source and `CRC`/`IMM`/`SR` as destinations), 3.5 (ALU 14-15),
3.6 (BR 19-31), 3.7 (Q 8-15, MISC 6-15), 3.4/4 (mode registers 14-31), 4
(`b_must` = 3).
- isa 6 rule 2 covers only opcodes 13-15.
- *Fix:* "every undefined code halts the lane with its outputs released",
  proven alongside rule 2 (D29).

**F11. Section 5 examples are incomplete or inconsistent.**
- I2C target: `FIELD SAMPLE 7, R0, tag=2` puts a tag on a register
  destination. Tags only apply to `HOSTQ` and `CAP`, so the host never sees
  the address. The fix needs C2: `SAMPLE 7, HOSTQ, tag=2` then `MOVSR R0`.
- I2C target: no `CODE` setup. SDA must be open-drain (`data_od`), with
  `data_in` naming the SDA input.
- I2C target: everything after the ACK is "...". The README has the
  complete version.
- SPI: CS is undriven (floating) until the first word, and F1/F2/F3 apply.
- UART receive: F7.
- None of the examples set `CODE.data_in`. They rely on the reset value
  in0.

**F12. `RAW` has no immediate field.** isa 3.2 lists `[7:0]` tag and
sources, but `IMM` is a legal source and `RAW DRIVE` "holds out0 for the
source's number of prescaled cycles". *Fix:* the same union as `FIELD`:
`[8:0]` is the immediate when the source is `IMM` (D40).

**F13. The immediate shares bits with `seek` and `tag`.** isa 3.1 `[8:0]`.
- An `EXPECT` against an immediate cannot `seek`, so the Ethernet SFD seek
  needs `LDI` into a register first.
- A field with an `IMM` source cannot carry a tag.
- This is acceptable, but should be stated. Also, language.md 6.3 C1 and C6
  say the immediate is 10 bits; isa.md says 9.

**F14. `SR` and field results are under-specified.** isa 1 "the field on the
wire; readable after the field", 3.1.
- Alignment for n < 32 and for LSB-first.
- The content after `DRIVE` (driven or sampled?) and after `EXCHANGE`.
- Whether `DRIVE` may have a destination.
- Whether `EXCHANGE` updates the CRC with the driven or the sampled bits
  (SD-card CRC needs the driven bits on commands and the sampled bits on
  read data).
- *Fix:* D36, D37, D18. Possibly a per-field "CRC on sampled bits" bit for
  `EXCHANGE`.

**F15. CRC as a source.** isa 3.0 "CRC value, bit W-1 first".
- The text does not say whether the hardware forces msb-first (ignoring
  `[20]`), or what happens when n ≠ W.
- It does not say whether `PUSHCRC` pushes the raw state or the value with
  the final XOR (`CRCCHK` explicitly uses the state).
- *Fix:* say "source `CRC` = `value_o`; the field must be msb-first with n =
  W, which the compiler checks", or make the hardware force it.

**F16. Stuffing boundaries.** isa 3.1 `[18]`, 4 `STUFF`, blocks/stuff.md,
language S2.
- The text does not say where a stuff bit due after a field's last data bit
  goes: at the end of this field, or at the start of the next stuffed field.
  USB needs it before EOP even when no stuffed field follows.
- It does not say whether unstuffed fields feed or reset the run counter, or
  how `EXCHANGE` stuffs (on which stream).
- *Fix:* D19.

**F17. Field results into a full queue, and memory bandwidth.** isa 3.7
"Pushes wait while the host-out queue is full", arch 5.1 "make one
data-memory access only in its own slot".
- A field result cannot "wait" without stopping the wire, and overflow
  behaviour is undefined.
- A `CAP` destination needs two memory writes: two slots.
- A result's write competes with the next instruction's own memory access.
- *Fix:* specify a write buffer (D15), the slot(s) used for write-back, and
  an overflow flag or record that the host can see.

**F18. When the source is read.** isa 3.1.
- If at issue time (D28), the host has less time but parking works for
  `HOSTQ` sources.
- If at start time, an empty queue at the start must hold the wire: for the
  I2C target that is what `stretch_hold` needs; for others it is a gap.
- What an event does to a parked field that already popped a word is also
  undefined.
- *Fix:* specify; D28 is the model's reading.

**F19. "Depends on this field's result" is not defined.** isa 2. The set in
D24 is the model's. It affects gap-free proofs, because the compiler's
schedule check must use the same set. *Fix:* copy D24 into isa.md.

**F20. PIN wait semantics.** isa 3.3 "then wait this many PERIOD intervals".
- The text does not say whether the lane stalls, or whether the `PIN` holds
  the bit hardware so the next field starts exactly when the wait ends
  (D31). Only the second gives exact `t_hd_sta` / `t_su_sto` timing.
- The unit is `PERIOD`, which in BAUD is half a bit (the launch-to-capture
  time), not a bit.
- *Fix:* specify D31. Consider a separate "PIN unit".

**F21. SETM details.** isa 3.4.
- Halves are written separately. Writing the low half of `CRCPOLY` leaves a
  stale upper half from a previous 32-bit polynomial, which is harmless only
  because the CRC masks to W bits.
- What a 16-bit register does with an upper-half write is undefined.
- Whether `SETM` may change `PERIOD` or `CODE` in the middle of a field is
  undefined.
- *Fix:* D8 and D42.

**F22. Q details.** isa 3.7.
- `POP rd` moves a 32-bit word into a 16-bit register, and the text does not
  say which half.
- `PUSHTS` with a non-zero tag destroys timestamp bits [31:24].
- *Fix:* low half (D27); tags only on `PUSH`, `PUSHI` and `PUSHCRC`.

**F23. ALU and MEM details.** isa 3.5.
- The `SHL`/`SHR` amount range (D26) is not given.
- Whether `MOV`, `INC` and `DEC` set flags: the text says "Sets Z" for all,
  and the model does.
- Whether `LOOP` sets flags: the model says no.
- `MEM` with both `absolute` and `postinc` set (D41).

**F24. Prescaler phase and RECOVER rounding.** isa 2.
- The text does not say whether the prescaler is free-running (jitter of up
  to one prescale period on the first bit) or restarted.
- "Half a PERIOD" for odd `PERIOD`.
- The first bit of a field that starts in the middle of a bit.
- *Fix:* D34 and D5.

**F25. PULSE receive is under-specified.** isa 2, 4 `THR01`/`THR2L`, `CODE`
rx = 3.
- The text does not say what enters the shift register: one bit per pulse,
  a 2-bit symbol per edge, or both high and low intervals.
- It does not say how unused thresholds behave, or how Manchester receive
  is built from symbols.
- *Fix:* specify. The model's reading is D12; Manchester is not modelled.

**F26. CODE bits.** isa 4 `CODE`.
- `invert`: receive, transmit or both? (Model: both.)
- `diff_in`: which input is N, and what changes? The language uses only
  "both low" events, so the bit appears to have no effect. (Model: no
  effect.)
- `delay_clock`: instead of the data delay, or as well? (Model: instead,
  D13.)

**F27. Where an instruction's effects fall in the slot.** isa 2, arch 11.
The text does not say at which cycle of fetch, decode and execute the
effects (PC, register, pin) become visible, or what happens to a fetched
instruction when an event fires before it executes. *Fix:* D2.

**F28. A halted lane drives nothing, so `HALT` lets push-pull lines float.**
isa 6 rule 3, 3.7.
- A UART transmitter that halts releases TX.
- The text does not say whether `HALT` waits for the field in progress.
- *Fix:* say so explicitly (D25), and document that a halted transmitter
  needs an external pull-up.

**F29. What an event leaves behind.** isa 3.6 says the CRC, stuffing count,
SR and bit count are kept, but not:
- what happens to the parked field (and its word);
- what happens to a `PIN` wait;
- where the BAUD clock is left (SCK may stay high);
- whether stretching is released.

No instruction can read "the number of bits done". *Fix:* D23. Add a way to
read the bit count if `cut` records need it.

**F30. The `PERIOD` unit means different things in different modes.** isa
3.3, 3.6, 4. `PIN` and `WAIT` count in `PERIOD`, which is half a bit in
BAUD, a whole bit in RECOVER, and in EDGE has no protocol meaning (the model
uses it as the stretch setup time). Timeouts written by the compiler must
know the mode. *Fix:* state it, or add a dedicated timeout unit.

**F31. Inconsistent widths across documents.** The immediate is 9 bits in
isa 3.0 but 10 in language 6.3 C1/C6. The event counter is 8 bits in arch 11
but K is 7 bits in isa 4. The CRC initial value and final XOR are 1-bit
(all-ones or all-zeros) in isa 4, while language 3.12's library has
`xorout 0x1F` and so on, which a compiler must map to the 1-bit form.

**F32. Queue placement is not specified.** isa 1, arch 6 and 9. "Ring buffers
with hardware pointers" exist, but their base and size registers are not in
the host-port map, and `MEM`/`R3` can overwrite queue storage. *Fix:* a
fixed partition (for example the top 64 words), or host-set base and size
registers. Either way, say whether `MEM` may touch them.

## Differences from the block specs written in parallel

`docs/blocks/sequencer.md`, `xbar.md`, `mem.md`, `host.md`, `channels.md` and
`top.md` appeared while this model was being written. They have not been
reconciled yet. A first reading shows these differences, all in named
constants or tagged rules:

| Topic | Model | Block spec | Where |
|---|---|---|---|
| Pipeline | 3 stages; effects at slot + 2 (`EXEC_STAGE`) | 4 stages F, I, D, X; decide in D (ph = L+2), execute and hand over in X (ph = L+3). The spec also reports "three cycles" in isa 2 and arch 11 as wrong | sequencer.md 1 and 11.1 |
| Lane input latency | `INPUT_DELAY` = 3 | 2 (second synchroniser stage after the mux) | xbar.md X1 |
| Run-ahead | literal "parks there": while a parked operation exists the sequencer executes nothing (D24) | the sequencer hands the operation to the lane's parked register and keeps going; only a second bit operation waits | sequencer.md 5.11 |
| Result write-back | instant at the last capture (D15) | a DELIVER slot at the lane's next slot (two for `CAP`), pre-empting the instruction; register hazards against undelivered destinations | sequencer.md 5.1 and 11.2 |
| Ordering with the field | hazards only on resources of the incomplete field (D24) | "sync instructions" (`SETM WAIT HALT CRCINIT CRCCHK STUFFCLR MOVSR MOVSRH PUSHCRC PUTSR BR M/NM`) wait until the bit hardware is idle (nothing running or parked) | sequencer.md 5.11 |
| `SR` source | read at issue (after the field completes) | resolved by the lane when the operation starts, like `CRC` | sequencer.md 5.11 |
| `WAIT` timeout | counted by the sequencer (D32) | sent to the lane as a parked `PIN` that changes nothing, so it occupies the bit hardware | sequencer.md 5.5 |
| Event entry | an armed event jumps when it **fires**; arming a pending event does not jump (D23) | VECTOR when **pending and armed** (lowest k), so arming an already-pending event vectors | sequencer.md 5.6 |
| Undefined codes | also operand 14 and `SETM` 14-31 halt (D29) | halts on opcodes 13-15, ALU 14-15, BR 19-31, Q 8-15, MISC 6-15 | sequencer.md 5.7 |

The same table and decisions apply as before: `WAIT` never clears pending,
an event stays armed, the taken event's pending bit is cleared, the parked
operation is discarded on an event, and `MISC HALT` waits for idle.
