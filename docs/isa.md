# Rosetta instruction set

Status: **draft v0.1**. This is what the compiler emits and what the
sequencer and lanes execute. Users normally write protocol descriptions
(`docs/language.md`), not this. Field widths may still move once the area
probe and the language draft are in.

## 1. What a lane program can see

| State | Size | Notes |
|---|---|---|
| `PC` | 9 bits | 512-word program memory shared by all lanes |
| `R0`-`R3` | 16 bits each | general registers |
| Flags | `Z` zero, `N` negative/borrow, `M` match, `E0`-`E3` event pending, `QE` host-in queue empty, `QF` host-out queue full |
| Shift register | 32 bits | holds the field on the wire |
| CRC register | 32 bits | |
| Mode registers | see section 4 | tick source, coding, stuffing, CRC, events |
| Host-in / host-out queues | 32-bit words | ring buffers in data memory |
| Channels `C0`-`C3` | 32 bits + valid | shared between lanes |
| Data memory | 512 x 32 | shared; each lane's region is set by the host |

A lane has 4 input signals `in0`-`in3` and 4 output signals `out0`-`out3`,
each output with a value and an enable. The crossbar maps them to pins.

## 2. Timing model

- A **tick** is the moment a bit is launched or captured. The lane's tick
  source (mode register `TICK`) produces a *launch* tick and a *capture*
  tick for every bit. In `BAUD` mode they are the two halves of the bit
  period; in `EDGE` mode they are chosen edges of the external clock; in
  `RECOVER` mode capture is mid-bit; in `PULSE` mode there is one tick per
  input edge.
- A **field instruction** occupies the lane's bit hardware for its whole
  duration. Meanwhile the sequencer runs ahead through following
  instructions until it reaches the next instruction that needs the bit
  hardware or depends on this field's result, and parks there.
- Consecutive fields are gap-free when the instructions between them
  complete, at one instruction per 5 clock cycles, before the current field
  ends. The compiler proves this per program.
- An instruction that needs data which is not there (empty queue, empty
  channel) **waits**; outputs hold their last value.

## 3. Instructions

All instructions are 32 bits. Bits `[31:28]` are the opcode.

### 3.1 `FIELD` (opcode 0): put bits on the wire or take them off

| Bits | Meaning |
|---|---|
| `[27:26]` | kind: `DRIVE`, `SAMPLE`, `EXCHANGE` (both at once), `EXPECT` (sample and compare) |
| `[25:21]` | number of bits minus one (1-32) |
| `[20]` | least significant bit first |
| `[19]` | update the CRC with these bits |
| `[18]` | apply bit stuffing |
| `[17:14]` | source operand (value to drive, or value to compare for `EXPECT`) |
| `[13:10]` | destination operand (where sampled bits go) |
| `[9:0]` | 10-bit immediate, used when the source is `IMM` |

Operands:

| Code | As source | As destination |
|---|---|---|
| `R0`-`R3` | register (16 bits) | register |
| `HOSTQ` | pop from host-in queue | push to host-out queue |
| `CAP` | - | push `{timestamp, value}` to host-out queue |
| `CRC` | the CRC register (after final XOR) | - |
| `MEM` | data memory at `R3`, then `R3 += 1` | same |
| `C0`-`C3` | take from channel | put to channel |
| `IMM` | bits `[9:0]` | - |
| `NONE` | zeros | discard |

`EXPECT` sets flag `M` when the sampled bits equal the source. With source
`CRC` the comparison is against the CRC as it stood before the field, which
is how a received check value is verified.

### 3.2 `RAW` (opcode 1): durations instead of bits

`RAW SAMPLE dst` measures the time from the previous input edge to the next
one, in prescaled clock cycles, and writes it to `dst`. `RAW DRIVE level,
src` holds `out0` at `level` for `src` prescaled cycles. Used to record and
replay waveforms with no description, and for pulse-coded protocols whose
symbols do not fit the threshold classifier.

### 3.3 `PIN` (opcode 2): set outputs directly

| Bits | Meaning |
|---|---|
| `[27:24]` | which of `out0`-`out3` to change |
| `[23:20]` | new values |
| `[19:16]` | new enables (0 releases the pin) |
| `[15:0]` | hold for this many launch ticks before the next bit-hardware instruction (0 = none) |

Used for idle levels, I2C START and STOP, USB end-of-packet, chip select.

### 3.4 `SETM` (opcode 3): write a mode register

`[27:23]` mode register, `[22]` upper or lower 16 bits, `[21]` take the
value from a register (`[1:0]`) instead of the immediate `[15:0]`.

### 3.5 Register and memory instructions

| Opcode | Name | Operation |
|---|---|---|
| 4 | `LDI rd, imm16` | load immediate |
| 5 | `ALU op, rd, rs/imm16` | `MOV ADD SUB CMP AND OR XOR SHL SHR INC DEC ADDC`; sets `Z`, `N`. `ADDC` is add with end-around carry (Internet checksum) |
| 6 | `MEM` | load or store a register half-word at `[rs]`, `[rs]+` or an absolute 9-bit address |

### 3.6 Flow

| Opcode | Name | Operation |
|---|---|---|
| 7 | `BR cond, target` | conditions: always, `Z`, `NZ`, `N`, `NN`, `M`, `NM`, `QE`, `QNE`, `QF`, `E0`-`E3`, channel valid |
| 8 | `LOOP rd, target` | decrement `rd`, branch if not zero |
| 9 | `WAIT mask, timeout` | wait for any event in `mask`, or `timeout` launch ticks (0 = no timeout) |
| 10 | `EVT k, target, on/off` | when event `k` fires while enabled: abandon the current field, release nothing, set `PC = target` |

Events are "go to", not "call": a handler is a protocol state (for example
"a START was seen, begin the address phase"), so there is no return.

### 3.7 Queues and housekeeping

| Opcode | Name | Operation |
|---|---|---|
| 11 | `Q op` | `POP rd` (host-in), `PUSH rs` / `PUSH TS` / `PUSH CRC` (host-out), `PUT c, rs`, `GET rd, c` |
| 12 | `MISC op` | `CRCINIT`, `IRQ` (raise host interrupt), `HALT`, `NOP` |

Opcodes 13-15 are reserved and must halt the lane.

## 4. Mode registers

| Register | Bits | Contents |
|---|---|---|
| `TICK` | 16 | mode (`BAUD`/`EDGE`/`RECOVER`/`PULSE`), prescaler, clock input select, launch edge, capture edge, clock-out enable / idle level / open-drain, honour clock stretching |
| `PERIOD` | 16 | cycles per half bit (`BAUD`) or per bit (`RECOVER`) |
| `THR0`-`THR2` | 16 each | pulse-width thresholds for `PULSE` |
| `CODE` | 16 | receive coding, transmit coding, data input select, open-drain, differential, pulse high-times select |
| `STUFF` | 8 | run length N, mode, enable |
| `CRCPOLY` | 32 | polynomial |
| `CRCINIT` | 32 | initial value |
| `CRCCFG` | 8 | width, bit order, final XOR on/off |
| `LAUNCH` | 8 | extra cycles between a launch tick and the output changing |
| `EVC0`-`EVC3` | 16 each | event condition: edge select on one signal, level of another, or timeout / both-low / stuffing violation |

## 5. Worked examples

Assembly syntax is for this document; the assembler is part of Phase 0.

### UART transmit, 8N1, 115200 baud at 60 MHz

```
        SETM  TICK, BAUD
        SETM  PERIOD, 260          ; 60e6 / 115200 / 2 = 260.4 cycles per half bit
        PIN   out0=1, enable       ; idle high
next:   Q     POP R0               ; waits here until the host sends a byte
        FIELD DRIVE 1, IMM 0       ; start bit
        FIELD DRIVE 8, lsb, R0     ; data
        FIELD DRIVE 1, IMM 1       ; stop bit
        BR    always, next
```

`BR` and `Q POP` run ahead during the stop bit, so back-to-back bytes have
no gap.

### UART receive

```
        SETM  TICK, RECOVER
        SETM  PERIOD, 521
        SETM  EVC0, falling(in0)
idle:   WAIT  E0                   ; start edge; the bit counter re-aligns here
        FIELD EXPECT 1, IMM 0      ; start bit, sampled mid-bit
        BR    NM, idle             ; glitch, not a start bit
        FIELD SAMPLE 8, lsb, HOSTQ
        FIELD EXPECT 1, IMM 1      ; stop bit
        BR    M, idle
        MISC  IRQ                  ; framing error
        BR    always, idle
```

A listener is the same program with destination `CAP`, so every byte
arrives at the host with a 16.7 ns timestamp.

### SPI controller, mode 0, 8-bit words

```
        SETM  TICK, BAUD | clock-out(out1, idle low) | launch=fall | capture=rise
        SETM  PERIOD, 3            ; 10 MHz clock
word:   Q     POP R0
        PIN   out2=0, enable       ; chip select low
        FIELD EXCHANGE 8, R0, HOSTQ
        PIN   out2=1, enable
        BR    always, word
```

### I2C target, address phase (sketch)

```
        SETM  TICK, EDGE(in1) | launch=fall | capture=rise      ; in1 = SCL
        SETM  EVC0, falling(in0) while in1 high                 ; START
        SETM  EVC1, rising(in0) while in1 high                  ; STOP
        EVT   0, start, on
        EVT   1, idle, on
idle:   WAIT  none
start:  FIELD EXPECT 7, IMM 0x50   ; our address
        BR    NM, idle
        FIELD SAMPLE 1, R1         ; read/write bit
        FIELD DRIVE 1, IMM 0       ; ACK (open-drain: drive low)
        ...
```

## 6. Rules the hardware guarantees (and we prove)

1. One instruction per lane per slot; a lane never executes outside its slot.
2. A reserved opcode or a `PC` past the loaded program halts the lane with
   all its outputs released.
3. A lane that is halted or not yet started drives nothing.
4. An instruction's effect on registers, flags, queues and pins is exactly
   the effect defined here (checked against the executable model).
