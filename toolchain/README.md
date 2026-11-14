# Toolchain

Assembler / compiler / configuration flow that turns protocol programs in
`programs/` into images loaded onto the chip, plus the host-side loader for
the Tiny Tapeout demo board (RP2350).

The compiler (description file to program images, monitors and schedule
check) and the loader are still to come. What is here now:

| Module | What it does |
|---|---|
| `rosetta/isa.py` | The instruction set of `docs/isa.md` v0.2 as data: opcode and field tables, mode-register layouts, `encode()`, `decode()`, `disassemble()` |
| `rosetta/asm.py` | Assembler for the syntax below: words, labels, listing |
| `rosetta/model.py` | Cycle-accurate model of one lane plus the shared logic it sees |
| `rosetta/wave.py` | ASCII waveforms of model traces |
| `rosetta/lang.py` | Parser for the core description language (`docs/language.md` 5.1), giving a typed syntax tree |

Everything uses only the CPython standard library. MicroPython has no
`dataclasses`, so `isa.py` will not run on the board as it is. When the
loader is written, the few constants it needs (opcodes, queue layout) will be
generated from `isa.py` into a plain module.

Run the tests from the repository root:

```
python3 -m unittest discover -s toolchain/tests
```

`MODEL_DECISIONS.md` lists every timing choice the model makes where
`docs/isa.md` leaves it open, with the isa.md text each choice interprets,
and the problems found in isa.md while building the model.

## Assembly syntax

One instruction per line. Other line rules:

- `;` starts a comment.
- `name:` defines a label, which is the address of the next instruction.
- Mnemonics, register names and enumerated values are not case-sensitive.
  The one exception: the FIELD flag `crc` is lower case, and the operand
  `CRC` (the check value as a source) is upper case.
- Numbers are written `42`, `0x2A` or `0b101010`.
- `.equ NAME, value` defines a constant.
- `.word value` emits a raw word. The disassembler uses it for words that
  no mnemonic can express exactly.

Operands are separated by commas. A *source* or *destination* is one of
`R0`-`R3`, `HOSTQ`, `CAP`, `CRC`, `MEM`, `C0`-`C3`, `IMM v`, `SR`, `NONE`
(isa 3.0).

| Instruction | Syntax |
|---|---|
| `FIELD` | `FIELD DRIVE n, flags, SRC [, DST]` · `FIELD SAMPLE n, flags, DST` · `FIELD EXCHANGE n, flags, SRC, DST` · `FIELD EXPECT n, flags, SRC [, DST]`. `n` is 1-32. The flags are any of `lsb` (default `msb`), `crc`, `stuff`, `early`, `seek`, `tag=N`. `src=X` and `dst=X` name an operand explicitly. `tag` and `seek` cannot be combined with an `IMM` source, because they share bits [8:0] with the immediate |
| `RAW` | `RAW SAMPLE DST [, tag=N]` · `RAW DRIVE level, SRC` |
| `PIN` | `PIN out0=1, out2=0, out3=z [, align] [, wait=N]`. Each output is `0` or `1` (driven) or `z` (released). Outputs that are not named are left alone. A trailing `enable`, as in isa.md, is accepted and ignored |
| `SETM` | `SETM REG, value` · `SETM REG, field=value, flag, ...` · `SETM REG, Rn` · `SETM REG.hi, ...` for bits [31:16]. Fields that are not named are 0. A bare enumerated value is accepted when it belongs to exactly one field (`SETM TICK, RECOVER`) |
| `LDI` | `LDI Rd, imm16` |
| ALU | `MOV ADD SUB CMP AND OR XOR SHL SHR ADDC Rd, Rs` or `Rd, imm16` · `INC DEC MOVSR MOVSRH Rd`. The prefix `ALU` is optional |
| `MEM` | `LD Rd, [Ra]` · `LD Rd, [Ra+]` (post-increment) · `LD Rd, [addr]` (absolute, 0-127) · `ST` takes the same forms · add `, hi` for bits [31:16] |
| `BR` | `BR cond, target`, where cond is `always Z NZ N NN M NM QE QNE QF QNF E0-E3 C0-C3` (C means channel valid). `BR target` means `always` |
| `LOOP` | `LOOP Rd, target` |
| `WAIT` | `WAIT none` · `WAIT E0\|E2` · `WAIT E1, timeout=N` · `WAIT timeout=N` (N in PERIOD units) |
| `EVT` | `EVT k, target, on\|off [, clear]` |
| `Q` | `Q POP Rd` · `Q PUSH Rs [, tag=N]` · `Q PUSHTS [, tag=N]` · `Q PUSHCRC [, tag=N]` · `Q PUT Cc, Rs` · `Q GET Rd, Cc` · `Q PUTSR Cc` · `Q PUSHI imm8 [, tag=N]`. The prefix `Q` is optional |
| `MISC` | `NOP` `HALT` `IRQ` `CRCINIT` `CRCCHK` `STUFFCLR` |

`SETM` field names, generated from `isa.MODE_REGS`. Enumerated values are
spelled as shown; one-bit fields take 0 or 1, or the bare name for 1.

| Register | Fields |
|---|---|
| 0 `TICK` | `mode`[1:0] BAUD\|EDGE\|RECOVER\|PULSE; `prescale`[3:2] 1\|8\|64\|512; `clock_in`[5:4] in0-in3; `capture`[6] RISING\|FALLING; `clock_out`[7]; `clock_idle`[8]; `clock_od`[9]; `rest`[10]; `stretch_honour`[11]; `stretch_hold`[12] |
| 1 `PERIOD` | `value`[15:0] |
| 2 `PERIOD2` | `value`[15:0] |
| 3 `THR01` | `thr0`[7:0]; `thr1`[15:8] |
| 4 `THR2L` | `thr2`[7:0]; `delay`[15:8] (launch delay) |
| 5 `CODE` | `rx`[1:0] LEVEL\|NRZI\|MANCHESTER\|SYMBOL; `tx`[3:2] LEVEL\|NRZI\|MANCHESTER\|PULSE; `data_in`[5:4] in0-in3; `invert`[6]; `data_od`[7]; `diff_out`[8]; `diff_in`[9]; `delay_clock`[10]; `delay_prescaled`[11] |
| 6 `PULSE` | `high0`[7:0]; `high1`[15:8] |
| 7 `STUFF` | `n`[2:0]; `mode`[3] ONES\|EQUAL; `enable`[4]; `violation_event`[5] |
| 8 `CRCPOLY` | `value`[31:0]: write the low half, then `CRCPOLY.hi` |
| 9 `CRCCFG` | `width`[4:0] 1-32 (stored as width-1); `init_ones`[5]; `xorout_ones`[6] |
| 10-13 `EVC0`-`EVC3` | `a`[1:0] in0-in3; `level`[2] LOW\|HIGH; `trigger`[4:3] LEVEL\|EDGE\|ANY\|BOTHLOW; `b`[6:5] in0-in3; `b_must`[8:7] ANY\|HIGH\|LOW; `k`[15:9] |

The listing has one line per word, in the form `address  word  source line`:

```
000  30000000          SETM  TICK, mode=BAUD
001  30800104          SETM  PERIOD, 260          ; 60e6 / 115200 / 2
```

## The isa.md section 5 examples, complete

These are the four examples of isa.md section 5 in this syntax, completed
(no `...`), plus UART receive as a listener. `tests/test_asm.py`
assembles every block marked `<!-- program: name -->`, and the model
tests run them against independent benches. Where an example had to
differ from isa.md, a `; NOTE` comment says why. The findings are
explained in `MODEL_DECISIONS.md`, section "Findings".

Pins: a lane has `in0`-`in3` and `out0`-`out3`. `out0` is the data output,
the data input is chosen by `CODE.data_in`, and `out3` is the clock output.

### UART transmit, 8N1, 115200 baud at 60 MHz (out0 = TX)

<!-- program: uart_tx -->
```asm
        SETM  TICK, mode=BAUD
        SETM  PERIOD, 260          ; 60e6 / 115200 / 2
        SETM  PERIOD2, 260
        PIN   out0=1               ; idle high
next:   Q     POP R0               ; waits for the host
        FIELD DRIVE 1, IMM 0       ; start bit
        FIELD DRIVE 8, lsb, R0
        FIELD DRIVE 1, IMM 1       ; stop bit
        BR    always, next
```

### UART receive, receiver role (in0 = RX)

<!-- program: uart_rx -->
```asm
        SETM  TICK, mode=RECOVER
        SETM  PERIOD, 521          ; one bit time
        SETM  EVC0, a=in0, trigger=EDGE, level=LOW
idle:   EVT   0, idle, off, clear  ; NOTE: forget edges of the last frame (finding F7)
        WAIT  E0
        FIELD EXPECT 1, IMM 0      ; start bit, sampled mid-bit; a glitch fails here
        BR    NM, idle
        FIELD SAMPLE 8, lsb, HOSTQ, tag=1
        FIELD EXPECT 1, IMM 1      ; stop bit
        BR    M, idle
        Q     PUSHI 0, tag=0xFE    ; framing error record
        BR    always, idle
```

### UART receive as a listener (in0 = RX, never drives)

Each byte becomes a `CAP` record: a timestamp word, then the value word with
tag 1. A framing error becomes a timestamp word, then the marker `0xFE000000`.

<!-- program: uart_listener -->
```asm
        SETM  TICK, mode=RECOVER
        SETM  PERIOD, 521
        SETM  EVC0, a=in0, trigger=EDGE, level=LOW
idle:   EVT   0, idle, off, clear
        WAIT  E0
        FIELD EXPECT 1, IMM 0
        BR    NM, idle             ; a glitch: dropped silently
        FIELD SAMPLE 8, lsb, CAP, tag=1
        FIELD EXPECT 1, IMM 1
        BR    M, idle
        Q     PUSHTS
        Q     PUSHI 0, tag=0xFE    ; framing error, with a timestamp
        BR    always, idle
```

### SPI controller, mode 0, 8-bit words, 10 MHz

out0 = MOSI, in0 = MISO, out2 = CS (active low), out3 = SCK. The host
queues one word per transfer, and the word received on MISO comes back to it.

<!-- program: spi_controller -->
```asm
        SETM  TICK, mode=BAUD, clock_out        ; SCK idle low, capture rising
        SETM  CODE, data_in=in0
        SETM  PERIOD, 4            ; NOTE: isa.md has 3 and 3; see finding F3
        SETM  PERIOD2, 2           ; 4 + 2 cycles = 10 MHz
        PIN   out2=1, out0=0       ; NOTE: deselect before the first word
word:   Q     POP R0
        PIN   out2=0               ; chip select low
        FIELD EXCHANGE 8, early, R0, HOSTQ
        PIN   out2=1               ; at the end of the last bit (finding F1)
        BR    always, word
```

### I2C target at address 0x50, address phase, write and read

in0 = SDA, in1 = SCL. out0 pulls SDA low (open-drain), and out3 pulls SCL
low for clock stretching. The host receives the address with tag 2 and
every written byte with tag 4. It supplies the bytes to be read, and SCL is
stretched until it has.

<!-- program: i2c_target -->
```asm
        SETM  TICK, mode=EDGE, clock_in=in1, capture=RISING, clock_out, clock_idle, clock_od, stretch_hold
        SETM  CODE, data_in=in0, data_od   ; NOTE: open-drain SDA, not set in isa.md
        SETM  PERIOD, 15           ; EDGE: SDA setup before a stretched SCL is released (D11)
        SETM  EVC0, a=in0, trigger=EDGE, level=LOW, b=in1, b_must=HIGH, k=18    ; START
        SETM  EVC1, a=in0, trigger=EDGE, level=HIGH, b=in1, b_must=HIGH, k=18   ; STOP
        EVT   1, idle, on
        EVT   0, start, on
idle:   WAIT  none                 ; until START (or STOP)
start:  FIELD SAMPLE 7, HOSTQ, tag=2   ; NOTE: isa.md writes "R0, tag=2"; tags need HOSTQ/CAP
        MOVSR R0                   ; the address again, for the compare
        FIELD SAMPLE 1, R1         ; read/write bit
        CMP   R0, 0x50
        BR    NZ, idle             ; not us: never drive, wait for STOP or START
        FIELD DRIVE 1, IMM 0       ; ACK
        CMP   R1, 0
        BR    NZ, read
write:  FIELD SAMPLE 8, HOSTQ, tag=4   ; releases SDA at its first launch edge
        FIELD DRIVE 1, IMM 0       ; ACK
        BR    always, write        ; until STOP or repeated START
read:   FIELD DRIVE 8, HOSTQ       ; waits for the host, stretching SCL
        FIELD EXPECT 1, IMM 0      ; the controller's ACK: 0 = more
        BR    M, read
        BR    always, idle         ; NACK: last byte
```

## Model

```python
from rosetta import asm, model, wave

prog = asm.assemble(text)
lane = model.Lane(prog.words, lane=0)       # lane index sets its slot on the wheel
lane.host_write(0x48, 0x69)                 # host-in queue
lane.start(pc=0)
trace = model.run(lane, 12_000, bench)      # bench(cycle, prev_out_values, prev_out_enables) -> in0-in3
lane.host_read_all()                        # host-out queue
print(wave.render({"tx": trace.out(0)}, scale=40))
```

`Lane.step(pins)` advances one clock cycle. It returns `(values, enables)`
for `out0`-`out3`. A bench sees the previous cycle's outputs, so the
wire has one cycle of delay. A wired-AND bus is modelled in the bench
(see `tests/benches.py`). The host side (`host_write`, `host_read`,
`channels`) acts instantly between cycles.

Implemented in full:

- the tick modes `BAUD`, `EDGE` and `RECOVER`, and receive classification in `PULSE`;
- level and NRZI coding, inversion, open-drain, differential output and
  pulse-width transmit;
- the clock output with `PERIOD`/`PERIOD2`, `rest`, idle level, open-drain,
  `stretch_honour` and `stretch_hold`;
- the launch delay on data or clock, bit stuffing and the CRC (both per
  `docs/blocks/`);
- all four FIELD kinds with `early`, `seek` and `tag`;
- `RAW`, `PIN`, `WAIT` and `EVT`, the event detectors with the window K, and
  the ALU, memory, queues, channels, timestamps and run-ahead parking.

Manchester coding (`CODE` rx/tx = 2) raises `NotImplementedError`.

## Waveforms

`wave.render(signals, start, end, scale, elide)` draws one line per signal.
A low level is `_`, a high level is `-` and a released output is `~`. A bus
prints its value at each change. The ruler line shows the cycle number
wherever some signal changes. `elide=N` shortens any stretch of more than N
unchanged columns to N columns and marks it with `/`.

```
      0 2              17        27   32  36    42 45 48    54    60    66 69 72    78 81
cs    ~~~~~~~~~~~~~~~~~----------_____________________________________________________-----
sck   ~~__________________________________--____--____--____--____--____--____--____--___
mosi  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~------____________-----------------------------------
```
