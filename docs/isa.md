# Rosetta instruction set

Status: **v0.2** (13 Nov). This is what the compiler emits and what the
sequencer and lanes execute. Users write protocol descriptions
(`docs/language.md`), not this.

Changes from v0.1: complete encodings; measured-area shrinks (8-bit pulse
thresholds, 1-bit CRC initial value and final XOR); and the fixes for the
gaps found by the language draft (clock stretching by a target, early first
bit, unequal clock halves, event confirm window, residue check, pattern
seek, record tags, hard memory partition). Section 7 lists them.

## 1. What a lane program can see

| State | Size | Notes |
|---|---|---|
| `PC` | 9 bits | 512-word program memory shared by all lanes |
| `R0`-`R3` | 16 bits each | general registers |
| Flags | | `Z` zero, `N` negative/borrow, `M` match, `E0`-`E3` event pending, `QE` host-in queue empty, `QF` host-out queue full |
| Shift register `SR` | 32 bits | the field on the wire; readable after the field |
| CRC register | 32 bits | |
| Mode registers | section 4 | |
| Host-in / host-out queues | 32-bit words | ring buffers inside the lane's memory region |
| Channels `C0`-`C3` | 32 bits + valid | shared between lanes |
| Data memory | 128 x 32 per lane | address = `{lane, 7 bits}`; a lane cannot address another lane's region |

A lane has 4 input signals `in0`-`in3` and 4 output signals `out0`-`out3`,
each output with a value and an enable. `out0` is the data output, `out1`
its complement in differential mode, `out3` the clock output when enabled.

## 2. Timing model

- The **prescaler** divides the clock by 1, 8, 64 or 512. Period counts,
  thresholds, durations and event windows are in prescaled cycles.
- Every bit has a **launch tick** (outputs change) and a **capture tick**
  (inputs are sampled):

| Tick mode | Launch tick | Capture tick |
|---|---|---|
| `BAUD` | start of the bit | `PERIOD` cycles later; the next launch is `PERIOD2` cycles after that |
| `EDGE` | the clock input's non-capture edge | the clock input's capture edge |
| `RECOVER` | start of the bit | half a `PERIOD` after it; any data edge restarts the bit timer |
| `PULSE` | (transmit uses `BAUD` timing) | each input edge; the symbol is the time since the previous edge compared with `THR0`-`THR2` |

- With the **clock output** enabled, `out3` goes to the launch level at the
  launch tick and to the capture level at the capture tick. After a field's
  last bit it either returns to its idle level or stays at the launch level
  (`TICK.rest`), which is what I2C needs.
- A **field instruction** occupies the lane's bit hardware for its whole
  duration. Meanwhile the sequencer runs ahead until it reaches the next
  instruction that needs the bit hardware or depends on this field's
  result, and parks there.
- Each instruction takes three clock cycles (fetch, decode, execute) inside
  the lane's five-cycle slot spacing, so a lane executes at most one
  instruction per five cycles. Fields are gap-free when the instructions
  between them finish before the current field ends. The compiler proves
  this per program.
- An instruction that needs data which is not there (empty queue or
  channel) **waits**; outputs hold. In `EDGE` mode with `TICK.stretch_hold`
  the lane holds the clock line low while it waits.

## 3. Instructions

All instructions are 32 bits; `[31:28]` is the opcode. Unlisted bits are 0.

### 3.0 Operand codes (4 bits)

| Code | Name | As source | As destination |
|---|---|---|---|
| 0-3 | `R0`-`R3` | register, zero-extended | low 16 bits to register |
| 4 | `HOSTQ` | pop host-in queue | push to host-out queue |
| 5 | `CAP` | - | push timestamp word, then value word |
| 6 | `CRC` | CRC value, bit `W-1` first | - |
| 7 | `MEM` | memory at `R3`, then `R3 += 1` | same |
| 8-11 | `C0`-`C3` | take from channel (waits) | put to channel (waits while full) |
| 12 | `IMM` | 9-bit immediate | - |
| 13 | `SR` | the shift register as the last field left it | - |
| 15 | `NONE` | zeros | discard |

### 3.1 `FIELD` (opcode 0)

| Bits | Meaning |
|---|---|
| `[27:26]` | kind: 0 `DRIVE`, 1 `SAMPLE`, 2 `EXCHANGE`, 3 `EXPECT` |
| `[25:21]` | number of bits minus one (1-32) |
| `[20]` | least significant bit first |
| `[19]` | update the CRC with these bits |
| `[18]` | apply bit stuffing |
| `[17:14]` | source |
| `[13:10]` | destination |
| `[9]` | `early`: put the first bit on the wire when the field starts, not at the first launch tick |
| `[8:0]` | source `IMM`: 9-bit immediate. Otherwise `[8]` `seek`, `[7:0]` `tag` |

- `EXPECT` samples, compares with the source and sets `M` on equality. With
  source `CRC` the comparison is against the CRC as it stood before the
  field.
- `EXPECT` with `seek` keeps shifting past the field length until the last
  `n` bits equal the source, then completes with `M` set. An event ends it.
- `tag`: when the destination is `HOSTQ` or `CAP` and `tag` is not 0, bits
  `[31:24]` of the value word are replaced by `tag`, so the host can tell
  records apart. Use tag 0 for fields wider than 24 bits.

### 3.2 `RAW` (opcode 1)

`[27]` direction (0 sample, 1 drive), `[26]` level, `[17:14]` source,
`[13:10]` destination, `[7:0]` tag.

`RAW SAMPLE` writes the prescaled time from the previous input edge to the
next one (saturating at `0xFFFF`). `RAW DRIVE` holds `out0` at `level` for
the source's number of prescaled cycles.

### 3.3 `PIN` (opcode 2)

| Bits | Meaning |
|---|---|
| `[27:24]` | which of `out0`-`out3` to change |
| `[23:20]` | new values |
| `[19:16]` | new enables (0 releases the pin) |
| `[15]` | `align`: take effect at the next launch tick instead of at once |
| `[11:0]` | then wait this many `PERIOD` intervals (0 = none) |

`PIN` waits for the current field to finish. It may set the clock output
(`out3`) directly, which is how START, STOP and repeated START are made.

### 3.4 `SETM` (opcode 3)

`[27:23]` mode register, `[22]` upper half, `[21]` value from register
`[1:0]` instead of immediate `[15:0]`.

### 3.5 Registers and memory

| Op | Encoding | Operation |
|---|---|---|
| 4 `LDI` | `[27:26]` rd, `[15:0]` imm | `rd = imm` |
| 5 `ALU` | `[27:24]` op, `[23:22]` rd, `[21:20]` rs, `[19]` use imm, `[15:0]` imm | 0 `MOV` 1 `ADD` 2 `SUB` 3 `CMP` 4 `AND` 5 `OR` 6 `XOR` 7 `SHL` 8 `SHR` 9 `INC` 10 `DEC` 11 `ADDC` (end-around carry) 12 `MOVSR` (`rd = SR[15:0]`) 13 `MOVSRH` (`rd = SR[31:16]`). Sets `Z`; `N` is bit 15 of the result, or the borrow for `SUB`/`CMP` |
| 6 `MEM` | `[27]` store, `[26:25]` data reg, `[24]` high half, `[23:22]` address reg, `[21]` post-increment, `[20]` absolute, `[6:0]` absolute address | load or store one 16-bit half of a memory word in the lane's own region |

### 3.6 Flow

| Op | Encoding | Operation |
|---|---|---|
| 7 `BR` | `[27:23]` cond, `[8:0]` target | 0 always, 1 `Z`, 2 `NZ`, 3 `N`, 4 `NN`, 5 `M`, 6 `NM`, 7 `QE`, 8 `QNE`, 9 `QF`, 10 `QNF`, 11-14 `E0`-`E3` pending, 15-18 `C0`-`C3` valid |
| 8 `LOOP` | `[27:26]` rd, `[8:0]` target | `rd -= 1`; branch if `rd` is not zero |
| 9 `WAIT` | `[27:24]` event mask, `[11:0]` timeout in `PERIOD` intervals (0 = none) | sleep until a masked event is pending or the timeout ends |
| 10 `EVT` | `[27:26]` k, `[25]` enable, `[24]` clear pending, `[8:0]` target | arm or disarm event `k` |

When an armed event fires: the current field is abandoned and `PC = target`.
The CRC, the stuffing run count, the shift register and the number of bits
done are kept. Outputs keep their levels. Events are "go to": a handler is
a protocol state, so there is no return.

### 3.7 Queues and housekeeping

| Op | Encoding | Operation |
|---|---|---|
| 11 `Q` | `[27:24]` op, `[23:22]` reg, `[21:20]` channel, `[15:8]` imm8, `[7:0]` tag | 0 `POP rd` (waits), 1 `PUSH rs`, 2 `PUSHTS` (timestamp), 3 `PUSHCRC`, 4 `PUT c, rs`, 5 `GET rd, c` (waits), 6 `PUTSR c` (32 bits), 7 `PUSHI imm8` (marker record) |
| 12 `MISC` | `[27:24]` op | 0 `NOP`, 1 `HALT`, 2 `IRQ`, 3 `CRCINIT`, 4 `CRCCHK` (`M` = CRC state equals `{R1, R0}`), 5 `STUFFCLR` |

Pushes wait while the host-out queue is full. Opcodes 13-15 halt the lane.

## 4. Mode registers

| # | Register | Bits |
|---|---|---|
| 0 | `TICK` | `[1:0]` mode (0 `BAUD`, 1 `EDGE`, 2 `RECOVER`, 3 `PULSE`); `[3:2]` prescale (1, 8, 64, 512); `[5:4]` clock input; `[6]` capture on falling edge; `[7]` clock output on `out3`; `[8]` clock idle level; `[9]` clock output open-drain; `[10]` `rest`: after a field stay at the launch level; `[11]` `stretch_honour`: in `BAUD`, pause while the clock input reads low after release; `[12]` `stretch_hold`: in `EDGE`, hold the clock low while waiting |
| 1 | `PERIOD` | 16 bits: launch-to-capture time (`BAUD`), bit time (`RECOVER`), and the unit for `PIN` and `WAIT` |
| 2 | `PERIOD2` | 16 bits: capture-to-next-launch time (`BAUD`) |
| 3 | `THR01` | `[7:0]` `THR0`, `[15:8]` `THR1` |
| 4 | `THR2L` | `[7:0]` `THR2`, `[15:8]` launch delay |
| 5 | `CODE` | `[1:0]` receive coding (0 level, 1 NRZI, 2 Manchester, 3 pulse symbol); `[3:2]` transmit coding (0 level, 1 NRZI, 2 Manchester, 3 pulse width); `[5:4]` data input; `[6]` invert; `[7]` data output open-drain; `[8]` differential output; `[9]` differential input; `[10]` launch delay applies to the clock output; `[11]` launch delay in prescaled cycles |
| 6 | `PULSE` | `[7:0]` high time for a 0, `[15:8]` high time for a 1 (bit time is `PERIOD + PERIOD2`) |
| 7 | `STUFF` | `[2:0]` N; `[3]` mode (0 ones, 1 equal); `[4]` enable; `[5]` a violation raises event 3 |
| 8 | `CRCPOLY` | 32 bits |
| 9 | `CRCCFG` | `[4:0]` width minus one; `[5]` initial value all ones; `[6]` final XOR all ones |
| 10-13 | `EVC0`-`EVC3` | `[1:0]` signal A; `[2]` level L; `[4:3]` trigger (0 A at level L, 1 A edge to level L, 2 A any edge, 3 A and B both low); `[6:5]` signal B; `[8:7]` B must be (0 anything, 1 high, 2 low); `[15:9]` K |

An event fires when its trigger has happened and its level conditions have
then held for K more prescaled cycles. K = 0 fires at once. K above 0 is
both the confirm window that rejects a false I2C START or STOP and the
duration for "line held low for this long".

## 5. Worked examples

Assembly syntax is for this document.

### UART transmit, 8N1, 115200 baud at 60 MHz

```
        SETM  TICK, BAUD
        SETM  PERIOD, 260          ; 60e6 / 115200 / 2
        SETM  PERIOD2, 260
        PIN   out0=1, enable       ; idle high
next:   Q     POP R0               ; waits for the host
        FIELD DRIVE 1, IMM 0       ; start bit
        FIELD DRIVE 8, lsb, R0
        FIELD DRIVE 1, IMM 1       ; stop bit
        BR    always, next
```

### UART receive (a listener uses `CAP` instead of `HOSTQ`)

```
        SETM  TICK, RECOVER
        SETM  PERIOD, 521
        SETM  EVC0, A=in0, edge to low
idle:   WAIT  E0
        FIELD EXPECT 1, IMM 0      ; start bit, sampled mid-bit
        BR    NM, idle
        FIELD SAMPLE 8, lsb, HOSTQ, tag=1
        FIELD EXPECT 1, IMM 1      ; stop bit
        BR    M, idle
        Q     PUSHI 0, tag=0xFE    ; framing error record
        BR    always, idle
```

### SPI controller, mode 0, 8-bit words, 10 MHz

```
        SETM  TICK, BAUD | clock-out | idle low
        SETM  PERIOD, 3
        SETM  PERIOD2, 3
word:   Q     POP R0
        PIN   out2=0, enable       ; chip select low
        FIELD EXCHANGE 8, early, R0, HOSTQ
        PIN   out2=1, enable
        BR    always, word
```

### I2C target, address phase

```
        SETM  TICK, EDGE(in1) | capture rising | clock-out open-drain | stretch_hold
        SETM  EVC0, A=in0 edge to low, B=in1 high, K=18     ; START
        SETM  EVC1, A=in0 edge to high, B=in1 high, K=18    ; STOP
        EVT   1, idle, on
        EVT   0, start, on
idle:   WAIT  none
start:  FIELD SAMPLE 7, R0, tag=2
        FIELD SAMPLE 1, R1
        ALU   CMP R0, 0x50
        BR    NZ, idle
        FIELD DRIVE 1, IMM 0       ; ACK
        ...
```

## 6. Rules the hardware guarantees (and we prove)

1. A lane executes only in its own slot.
2. A reserved opcode halts the lane with all its outputs released.
3. A lane that is halted or not yet started drives nothing.
4. A lane's memory accesses stay inside its own 128-word region.
5. Each instruction's effect on registers, flags, queues and pins is
   exactly the effect defined here.

## 7. Changes made for gaps found in review

| Gap | Fix | Where |
|---|---|---|
| G1 target cannot stretch the clock | `TICK.stretch_hold` | 2, 4 |
| G2 first bit late for leading-edge capture | `FIELD` `early` | 3.1 |
| G3 equal clock halves break I2C fast mode | `PERIOD2` | 4 |
| G4 false START/STOP from separate synchronisers | event window K | 4 |
| G6 no check for a CRC after a variable-length payload | `MISC CRCCHK` | 3.7 |
| G7 cannot find a start pattern at 10 Mbit/s | `EXPECT` `seek` | 3.1 |
| G8 "timeout" event undefined | trigger 0 with K | 4 |
| G9 Stress cannot reach hold or slow-clock setup | launch delay on clock or data, in prescaled cycles | 4 |
| C1 host records carry no type | `tag` | 3.1 |
| C2 a sampled field needed in two places | source `SR`, `ALU MOVSR` | 3.0, 3.5 |
| G5 two inputs in one field (SPI listener) | **not fixed**: an SPI listener uses two lanes | |
| C3 event timestamps are late by a constant | **not fixed**: the compiler subtracts the constant | |
