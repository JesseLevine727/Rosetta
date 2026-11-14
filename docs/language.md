# Rosetta description language

> **Status: provisional (13 Nov).** Drafted by a worker, reviewed by the
> overseer, **not yet approved by the owner**. The hardware gaps listed in
> section 6.2 were answered in `docs/isa.md` v0.2 section 7; section 6 here
> still describes the architecture as it was before those fixes.

Status: **proposal for owner review** (draft 0.1, 13 Nov 2026). Not approved,
nothing implemented. It builds on `docs/architecture.md` (approved) and
`docs/isa.md` (draft v0.1). References such as "arch §4.1" and "isa §3.1"
point into those two documents.

Contents: 1 goals · 2 the language by example (UART, SPI, I2C) · 3 the
constructs · 4 USB and Ethernet sketches · 5 grammar · 6 hardware mapping and
gaps · 7 open questions and decisions.

---

## 1. Design goals and non-goals

**Goals**

1. **One description, every role.** A protocol is written once, as wires,
   timing and a sequence of fields. Each field names the party that drives
   it. From that, the compiler produces one program per party, a listener
   program, and a formal monitor. Nobody writes a separate "target version".
2. **Reads like the datasheet.** A hardware engineer should be able to check
   a description against a timing diagram before learning the language. The
   order of the lines is the order of the bits on the wire.
3. **Small.** There are fourteen statement kinds: `protocol`, `param`,
   `parties`, `wire`, `clock`, `event`, `frame`, field lines, `together`,
   `repeat`, `if` and `rule`, plus `crc` and `stuff` for check values and
   bit stuffing. Expressions are plain arithmetic and comparisons. There are no
   variables and no functions.
4. **Nothing the chip cannot do.** Every construct compiles to a named
   feature of the architecture (section 6.1). When the language needs
   something the architecture lacks, section 6 lists it as a gap rather than
   hiding it in the compiler.
5. **Timing rules belong to the protocol.** The same `rule` lines set the
   compiler's delays, become the monitor's assertions, and name what Stress
   is allowed to break.
6. **Mistakes are compile errors.** The compiler rejects any description the
   roles could not agree on, for example a branch that one party cannot see,
   or a party that cannot keep up and has no fallback value.

**Non-goals**

- Application logic, such as USB enumeration or answering ARP and `ping`.
  That runs on the host, against an API generated from the description
  (section 4.3).
- Connecting lanes to each other (Translate, Intercept). That needs a
  separate wiring file, left open in section 7.
- Multi-controller arbitration, echo checking, analog properties (levels,
  slew rates, drive strength).
- More than one bit per wire per tick within one field (dual or quad SPI),
  or sampling on both clock edges (DDR).
- Unknown protocols. Imitate records raw durations and needs no description.
- Being a general programming language or a waveform-drawing language.

---

## 2. The language by example

**How to read the examples.**

- There is one statement per line, `{ }` group statements, and `//` starts a
  comment. I chose braces over indentation because descriptions get pasted
  into issues, mail and chat, where leading whitespace is often mangled, and
  braces read the same to anyone who knows C or Verilog.
- A field line reads `OWNER name[WIDTH] ORDER ...`. For example,
  `target ack[1] = 0` means "the target drives a 1-bit field called `ack`,
  and its value is 0".
- A field with no `=` is **data**: the owner's host supplies the value, and
  every other role delivers it to its own host. A field with `= value` is
  **expected**: the owner drives that value, and everyone else checks it.
- The listener is never declared. Every description has one.

All examples in sections 2 and 3 parse with a Lark transcription of the
grammar in section 5, with no ambiguity.

### 2.1 UART

```
// UART, one direction: 8 data bits, optional parity, 1 stop bit.
// A full-duplex port is two instances of this, one per direction.
protocol uart {
  param baud   = 115200 Hz
  param parity : none | even | odd = none

  parties sender, receiver

  wire  tx by sender idle high
  clock rate baud tolerance 2%

  frame character {
    sender start_bit[1] = 0
    sender data[8] lsb
    if parity == even {
      sender parity_bit[1] = even_parity(data) else continue
    } else if parity == odd {
      sender parity_bit[1] = odd_parity(data) else continue
    }
    sender stop_bit[1] = 1
  }
}
```

`clock rate` means there is no clock wire: bits are timed by an agreed rate.
`parity` is a parameter, so the compiler resolves the `if` at compile time.
The frame has no explicit start condition, so it begins at the first edge
away from the idle level, which is the start bit's falling edge.

**How the roles fall out** (115200 baud, 60 MHz clock):

| Element | sender | receiver | listener |
|---|---|---|---|
| `tx` | output, idle high | input | input |
| timing | `BAUD`, half-bit `PERIOD` 260 (0.16 % fast, inside the 2 %) | `RECOVER`, `PERIOD` 521 | `RECOVER` |
| `start_bit` | drives 0 | frame starts at the falling edge; checks 0 mid-bit (a mismatch is a glitch and is ignored) | same |
| `data` | drives 8 bits from the host, lsb first | samples to the host | records with a timestamp |
| `parity_bit` | drives the check unit's output | compares; on a mismatch it records an error and continues | compares; records only a mismatch |
| `stop_bit` | drives 1 | compares; on a mismatch it records a framing error and ends the frame | compares; records only a mismatch |

**What the host sees** (the host sends `"Hi"`):

```
sender    in : 0x48 0x69
          out: (nothing)
receiver  out: data=0x48  data=0x69
listener  out: t=0.0us data=0x48   t=86.8us data=0x69
errors       : error stop_bit=0     (framing)   error parity_bit=1
```

Two consequences. The receiver owns no fields, so its program is the
listener's program without timestamps: that role costs nothing extra. A
full-duplex UART is two descriptions on two lanes, because one lane has one
tick source and one shift register, and the two directions are timed
independently.

### 2.2 SPI

```
// SPI, modes 0-3 (mode = 2 * cpol + cpha), one target on one chip select.
protocol spi {
  param sck_rate  = 1 MHz
  param cpol      : 0..1  = 0             // level of sck when idle
  param cpha      : 0..1  = 0             // 0: sample on the leading edge
  param word_size : 1..32 = 8             // bits per word

  parties controller, target

  wire sck  by controller idle cpol
  wire cs_n by controller idle high
  wire mosi by controller idle low
  wire miso by target     idle released

  clock sck by controller rate sck_rate capture (cpha == 0 ? leading : trailing)

  event select   = cs_n falls by controller
  event deselect = cs_n rises by controller

  frame transfer after select until deselect {
    repeat {
      together {
        controller mosi_data[word_size] msb on mosi
        target     miso_data[word_size] msb on miso default 0
      }
    }
  }

  rule cs_lead : select to sck changes >= 0.5 bit
  rule cs_lag  : sck changes to deselect >= 0.5 bit
  rule cs_idle : deselect to select >= 1 bit
  rule mosi_su : mosi setup to capture >= 10 ns
  rule miso_su : miso setup to capture >= 10 ns
}
```

- `together` puts two fields on the same ticks, which is a full-duplex
  exchange.
- The open `repeat` (no count) runs until `deselect`. The controller owns
  `deselect`, so the controller's host says how many words to exchange.
- `default 0` is required here. The target cannot slow the clock, so it
  needs something to send when its host has supplied nothing.
- The rule values are typical. A real target's datasheet supplies its own.

**How the roles fall out:**

| Element | controller | target | listener |
|---|---|---|---|
| `sck` | `BAUD` with clock output; idle level from `cpol`, edges from `cpha` | `EDGE` on `sck` | `EDGE` |
| `select`, `deselect` | drives `cs_n`, timed by `cs_lead`, `cs_lag`, `cs_idle` | detects them; `deselect` abandons the current word and releases `miso` | detects and records them |
| the two `together` fields | `EXCHANGE`: drives `mosi` from the host and samples `miso` to the host | `EXCHANGE`: drives `miso` from the host (0 if the host has nothing) and samples `mosi` | samples both: needs two lanes today (gap G5) |
| open `repeat` | word count from the host | loops until `deselect` | same |

**What the host sees** (a two-word transfer, for example a 0x9F command
followed by a dummy word):

```
controller in : 2 0x9F 0x00          (count, then one word per mosi_data)
           out: 0xFF 0xEF            (one per miso_data)
target     in : 0xFF 0xEF            (sent on miso; 0 if the queue is empty)
           out: select mosi_data=0x9F mosi_data=0x00 deselect
listener   out: select mosi_data=0x9F miso_data=0xFF mosi_data=0x00 miso_data=0xEF deselect
                (each with a timestamp)
```

Notes:

- For a target, `miso_data` must be staged before the controller clocks it.
  At 1 MHz a word lasts 8 µs, far less than a host round trip, so a target
  that "answers a command" needs the answer queued in advance. That is
  inherent to SPI, not to Rosetta.
- With `cpha = 0`, the first bit must be on `miso` before the first clock
  edge. The architecture cannot do this on the target side (gap G2).
- The target's top clock rate is set by `miso_su` and the input synchroniser
  latency. It is roughly 5 MHz (cost C9). The compiler reports the exact
  figure for each role.

### 2.3 I2C

```
// I2C, 7-bit addresses, standard mode (100 kHz) or fast mode.
protocol i2c {
  param scl_rate = 100 kHz
  param address  : 0..127 = 0x50          // the target's own address

  parties controller, target

  wire scl open_drain
  wire sda open_drain

  clock scl by controller rate scl_rate capture rising stretch by target

  event start = sda falls while scl high by controller
  event stop  = sda rises while scl high by controller

  frame transfer after start until stop or start {
    controller addr[7] msb
    controller rw[1] (write = 0, read = 1)
    target     ack[1] = 0 when addr == address else end
    if rw == write {
      repeat {
        controller data[8] msb
        target     ack[1] = 0 else end
      }
    } else {
      repeat {
        target     data[8] msb
        controller ack[1] (more = 0, last = 1)
      } while ack == more
    }
  }

  // UM10204 table 10. The first value applies above 100 kHz (fast mode).
  rule t_low    : low scl  >= (scl_rate > 100 kHz ? 1.3 us : 4.7 us)
  rule t_high   : high scl >= (scl_rate > 100 kHz ? 0.6 us : 4.0 us)
  rule t_su_dat : sda setup to scl rises >= (scl_rate > 100 kHz ? 100 ns : 250 ns)
  rule t_hd_dat : sda hold from scl falls >= 0 ns
  rule t_su_sta : scl rises to start >= (scl_rate > 100 kHz ? 0.6 us : 4.7 us)
  rule t_hd_sta : start to scl falls >= (scl_rate > 100 kHz ? 0.6 us : 4.0 us)
  rule t_su_sto : scl rises to stop  >= (scl_rate > 100 kHz ? 0.6 us : 4.0 us)
  rule t_buf    : stop to start      >= (scl_rate > 100 kHz ? 1.3 us : 4.7 us)
}
```

What each piece means:

- **Open-drain lines** have no fixed driver. The `clock` line says who drives
  `scl` and who may stretch it. Field and event owners decide who pulls
  `sda` low.
- **START and STOP** are events. `until stop or start` means the frame ends at
  either one, wherever it is. A repeated START ends one frame and begins the
  next.
- **Addressing** is `when addr == address`. The target pulls `ack` low only
  if it is addressed. Otherwise it releases the line, which reads as 1 (a
  NACK), and drops out until the frame ends. The controller and the listener
  just see `ack = 1` and take `else end`.
- **The direction of the data bytes** follows `rw`. That is an ordinary `if`
  on a field everyone has seen.
- **The end of a read** is decided by the controller's own `ack` bit, which
  is data from its host: `more` or `last`.
- **"SDA is stable while SCL is high"** is not a separate rule. Any `sda`
  change while `scl` is high *is* a START or STOP, and the frame structure
  allows those only at frame boundaries. The monitor checks exactly that.

**How the roles fall out:**

| Element | controller | target | listener |
|---|---|---|---|
| `scl` | `BAUD`, open-drain clock output, waits while stretched | `EDGE` (capture rising, launch falling); stretches when its host is late (gap G1) | `EDGE` |
| `sda` | pulls low for its own 0 bits, releases otherwise | same | input only |
| `start`, `stop` | generated with `PIN` sequences timed by `t_su_sta`, `t_hd_sta`, `t_su_sto`, `t_buf` | detected (`EVC0`, `EVC1`), jumps to a handler | detected and recorded |
| `addr`, `rw` | drives from the host | samples to the host and to a register | records |
| address `ack` | samples; on a NACK it records an error, generates `stop` and halts | drives 0 if `addr == address`, otherwise releases and waits for `stop` or `start` | compares; records a NACK |
| write loop | word count from the host; drives `data`; checks `ack` | samples `data` to the host; drives `ack` = 0 | records |
| read loop | samples `data` to the host; drives `ack` from the host | drives `data` from the host; samples `ack` | records |
| frame end | the host says `stop` or `start` | waits for the event | records |

**What the host sees.** This is a register read: write the pointer 0x10, then
a repeated START, then read 2 bytes. Named values are sent as their codes
(`write` = 0, `more` = 0, `last` = 1). An end choice is its position in the
`until` list (`stop` = 0, `start` = 1).

```
controller in : 0x50 write 1 0x10 start    0x50 read more last stop
                addr rw  cnt data end      addr rw  ack  ack  end
           out: data=0x12 data=0x34
target     in : 0x12 0x34
           out: start addr=0x50 rw=write data=0x10 start addr=0x50 rw=read
                ack=more ack=last stop
listener   out: start addr=0x50 rw=write data=0x10 start addr=0x50 rw=read
                data=0x12 ack=more data=0x34 ack=last stop   (each with a timestamp;
                the target's matched ack=0 bits are implied, not recorded)
```

If nothing answers at 0x50, the controller's output is `error ack=1`. The
controller then drives `stop` and halts, and the loader discards the rest of
that transfer and restarts the lane (section 3.15). A target with a
different address records `start addr=0x50 rw=write stop` and never drives
the bus.

Notes on I2C:

- With equal clock halves, 400 kHz breaks `t_low` (1.25 µs < 1.3 µs). The
  compiler rejects it and reports 384 kHz as the fastest rate that meets
  every rule (gap G3).
- `t_hd_dat >= 0` is what UM10204 allows. Because the hold can be zero, a
  target or listener can see a false START or STOP (gap G4). The formal proof
  of the target program will produce this counterexample, so it is not
  hypothetical.

### 2.4 What the examples show

- Each role is a mechanical reading of the same lines: drive what you own,
  sample what you don't, and the listener samples everything.
- The receiver role often costs nothing extra (UART), and the controller and
  target programs come from the same lines (SPI, I2C).
- Rules are not decoration. They set the compiler's delays (`t_su_sta` sets a
  `PIN` hold count) and they reject impossible parameter sets (400 kHz).
- Rung 1 hits four hardware gaps (G1-G4), all small. They are the main
  findings of this draft (section 6.2).

---

## 3. The constructs

Each construct below gives its syntax, its meaning, and how each role
compiles it. The hardware each construct maps to is in section 6.1.

### 3.1 Lexical rules

- `//` starts a comment that runs to the end of the line.
- A statement ends at the end of its line. Inside `( )`, newlines are ignored,
  so long expressions and value lists can wrap. A block is `{` at the end of
  a line, statements one per line, then `}`. A block holding a single
  statement may sit on one line: `repeat leds { controller grb[24] msb }`.
- Names are letters, digits and `_`, and do not start with a digit.
  Keywords (section 5.3) are reserved and lower case, so `SETUP` is a valid
  name.
- Numbers are written `42`, `0x2A`, `0b10_1010` or `0.6`.
- A quantity is a number with a unit:
  - time: `s`, `ms`, `us`, `ns`
  - frequency: `Hz`, `kHz`, `MHz` (a bit rate is a frequency)
  - `bit`: one bit time of this protocol
  - `%`: a fraction of a nominal value

  Mixing units wrongly (adding a time to a frequency) is a compile error.
- `high` is 1 and `low` is 0 wherever a value is expected.

### 3.2 `protocol` and `param`

```
protocol NAME { statements }
param NAME [: TYPE] = DEFAULT        TYPE is  a | b | c   or   lo..hi
```

- **`protocol`.** A file holds one `protocol`, and may also hold top-level
  `crc` statements (this is how the library is written, section 3.12).
  Statements inside a protocol may come in any order. Field lines appear
  only inside the frame.
- **`param`.** Parameters are **compile-time constants**. The loader passes
  overrides (`baud=9600`), and the compiler emits a separate program for each
  parameter set. Nothing about a parameter is stored on the chip. Section 4
  sketches `runtime` parameters as an extension.

### 3.3 `parties` and roles

```
parties A, B [, C ...]
```

Each party is a role. The listener is always a further role and is never
declared. The whole language rests on this one rule:

| Construct | Owner's program | Another party's program | Listener |
|---|---|---|---|
| wire `by P` | output | input | input |
| field owned by P | drive (value from the host, a constant or the check unit) | sample (deliver to the host, check, or keep for a branch) | sample; record data, and expected values only on a mismatch |
| event owned by P | generate it | detect it and react | detect it and record |
| clock wire owned by P | generate it | follow it | follow it |
| a decision P owns (loop count, which end event) | take it from the host | observe the outcome on the wire | observe and record |

A party never samples its own fields: no echo checking. A listener never
enables an output (arch obligation 8).

### 3.4 `wire`

```
wire NAME [diff (P, N)] [by PARTY] [idle LEVEL] [open_drain] [coding nrzi|manchester]
```

- **`by PARTY`.** Only that party drives the wire. It is an output in that
  party's program and an input everywhere else. Push-pull wires must have a
  `by`.
- **`open_drain`.** A driver pulls low or releases; an external pull-up
  makes a released wire read 1. An open-drain wire needs no `by`. Who may
  pull it low follows the field or event owner, and for a clock, the `clock`
  statement (`stretch by`). It must be mapped to a `uio` pin. Driving a 1
  means releasing the wire, so a wire where nobody drives reads 1.
- **`idle LEVEL`.** The wire's level when no frame is in progress: `high`,
  `low`, an expression giving 0 or 1 (`idle cpol`), or `released` (the owner
  disables its output). An open-drain wire is always idle released and reads
  1.
- **`diff (P, N)`.** One logical wire on two pins. The owner drives N as the
  complement of P. Other roles read the level of P, plus the "both low" state
  for events. For Ethernet, P is fed by an external comparator.
- **`coding`.** How logical bits become wire levels. The default is the plain
  level. `nrzi` uses the USB convention (0 is a transition, 1 is no
  transition). `manchester` uses the IEEE 802.3 convention (1 is low to high
  at mid-bit). Both are used in section 4.

### 3.5 `clock`

A protocol has exactly one `clock` statement, in one of three forms. In all
three, `bit` (the unit) is one bit period.

**Form A: agreed rate, no clock wire.**

```
clock rate R [tolerance T]
```

- The owner of a field launches its bits from an internal timer. Everyone
  else recovers the timing from the data edges.
- `tolerance T` bounds the owner's bit time to 1/R ± T. The monitor asserts
  this for the owner and assumes it for the others. The proof then shows that
  receivers decode correctly at any rate inside the band.
- The compiler rejects a rate whose `PERIOD` rounding error exceeds T.
- **Implicit frame start.** A frame with no `after` begins at the first edge
  away from the idle level. Its first field must be a constant whose first
  bit differs from idle, like the UART start bit. That field is also the
  glitch filter: if it fails, the "frame" is silently dropped and no error is
  recorded.

**Form B: clock wire.**

```
clock W by P rate R capture rising|falling|leading|trailing [stretch by Q]
```

- `W` carries a clock driven by P at rate R. For the other roles, R is the
  fastest clock they must keep up with, and the compiler's schedule check
  uses it.
- **Edges.** `leading` is the first edge away from W's idle level. Data is
  captured on the named edge and launched on the other one.
- **First bit.** With a leading capture edge, the first bit of a frame is
  launched when the frame begins (at its `after` event). This is SPI with
  `cpha = 0`.
- **`stretch by Q`.** Q may hold W at its active level after a launch edge.
  P then waits until it sees W released before it times the next half. W
  must be open-drain.
- In the core language, frames on a clock wire need an explicit `after`.

**Form C: pulse-width symbols.**

```
clock symbols [tolerance T] { V = high T1, low T2  (one line per symbol) }
```

Each bit is one pulse. Example: WS2812 LEDs.

```
// WS2812 LED strip: pulse-width symbols, one-way.
protocol ws2812 {
  param leds = 8

  parties controller, strip

  wire din by controller idle low
  clock symbols tolerance 150 ns {
    0 = high 0.40 us, low 0.85 us
    1 = high 0.80 us, low 0.45 us
  }

  event latch = din low for 50 us by controller

  frame update until latch {
    repeat leds { controller grb[24] msb }
  }
}
```

The transmit hardware chooses only the high time inside a fixed period. So
the compiler requires every symbol to start high and to have the same period
(0.40 + 0.85 = 0.80 + 0.45 = 1.25 µs). Receivers classify each pulse by its
high time, with thresholds halfway between symbols. There are at most four
symbols (three thresholds, arch §4.1). As in form A, a frame with no `after`
begins at the first edge away from idle; every symbol starts high, so that
is the first rising edge.

**Which tick source each role uses:**

| Form | Role driving the current field | Other roles and listener |
|---|---|---|
| A, `clock rate` | `BAUD` | `RECOVER` (`PULSE` for Manchester) |
| B, `clock W by P` | P: `BAUD` with clock output. Other owners: `EDGE` | `EDGE` |
| C, `clock symbols` | `BAUD` with pulse-width coding | `PULSE` |

In form A, when both parties own fields (USB), a role switches between
`BAUD` and `RECOVER` between fields. In form B the tick source is fixed per
role: the I2C target drives its `ack` on `EDGE` ticks.

### 3.6 `event`

```
event NAME = CONDITION by PARTY
CONDITION:   W rises|falls|changes [while V high|low]
             W high|low for D
             W, V both low for D
D:           T   or   Tmin .. Tmax
```

- **The owner generates the event.** For `W edge while V level`, it first
  brings W to the level before the edge, while V is at the opposite level,
  so the change is not itself an event. It then sets V to the named level,
  waits, makes the edge, waits again, and returns V to its level inside the
  frame. The waits come from rules that name the event (`t_su_sta`,
  `t_hd_sta`), or half a bit if there are none.

  For I2C START from inside a frame (SCL low), that comes out as: release
  SDA; release SCL and wait until it reads high; wait `t_su_sta`; pull SDA
  low; wait `t_hd_sta`; pull SCL low. For `for D` conditions, the owner holds
  the level for `Tmax`.
- **Other roles detect it.** When it fires, the role jumps to the event's
  handler and abandons any field in progress. Detection of `for D` fires once
  the condition has lasted `Tmin`.
- **Monitor.** The monitor requires the owner's duration to fall in
  `[Tmin, Tmax]`, and requires each event to occur only where the description
  puts it: at a frame's start or end. The programs, in contrast, are
  tolerant: an observer recovers from an event in the wrong place by jumping
  as usual. The monitor is strict and the programs are forgiving.
- **Limit.** A role can use at most four events, one per event detector.

### 3.7 `frame`

```
frame NAME [after EVENT] [until EVENT {or EVENT}] { body }
```

- **One frame per protocol.** In the core language a protocol has exactly
  one frame. Frames follow each other forever, and the wires are idle between
  them.
- **`after E`.** The frame begins with E. Without `after`, it begins
  implicitly (section 3.5, forms A and C).
- **`until E1 or E2`.** The frame ends when any listed event occurs, wherever
  the body is. If the body finishes first, the owner of those events
  generates one. When it owns more than one, its host chooses which. With no
  `until`, the frame ends when its body ends (UART).
- **Open loops.** An open `repeat` must be the last statement of a frame that
  has `until`. Otherwise observers could not tell where the loop stopped and
  the next statement began.
- An event may both end one frame and begin the next: I2C `start`.

### 3.8 Field lines

```
OWNER NAME[WIDTH] [msb|lsb] [on WIRE] [(N1 = v1, N2 = v2 ...)]
      [= EXPR [when COND] | default EXPR] [else end|continue]
```

**Shape**

- **Width** is an expression of at least 1. The hardware handles 1-32 bits
  per field, so the compiler splits wider fields.
- **Bit order** is required for any field wider than one bit. There is
  deliberately no default, because a silent default would be the most likely
  bug in a description.
- **`on WIRE`** is required when more than one wire could carry the field
  (any wire that is not the clock). The owner must be able to drive that
  wire.

**Value kinds**

- **Data** (no `=`). The owner takes the value from its host, one queue word
  per field. Other roles deliver it to their host. The listener records it.
- **Expected** (`= EXPR`). EXPR may use parameters, earlier fields and check
  functions. The owner drives the value. Others compare it, and a mismatch
  produces an error record and the `else` action. The listener compares too,
  and records the field only on a mismatch, because a matching value is
  already known from the description.
- **Named values**, written `(write = 0, read = 1)`, name the codes. The names
  can be used in expressions, in host records and in the loader API. A
  sampled value not in the list is a mismatch. For the owner, the list
  constrains its host, and the loader checks it. It does not constrain the
  program.
- **`default EXPR`** (data fields only). If the owner's host queue is empty
  when the field must start, the owner drives the default and consumes no
  word.
- **`when COND`** (expected fields only). The owner evaluates COND. If it is
  false, the owner does not drive the field (it releases the wire) and takes
  the `else` action. Others see the released level. The compiler checks that
  this is visible: the wire must be open-drain and the expected value must
  contain a 0 bit. When `when` is false the owner records nothing, because
  it is not an error, only "not for me".
- **`else end`** (the default) abandons the rest of the frame (section 3.15
  says what each role then does). **`else continue`** records the error and
  carries on.

**Names and completeness**

- A field name may repeat. In an expression it means the most recent
  occurrence on the wire. In a check function it means every occurrence.
- Only **complete** fields (last bit captured) may be used in `if`, `while`,
  `when` and expected values. Everyone has seen a complete field, so every
  role takes the same branch.

### 3.9 `together`

```
together { FIELD  FIELD }
```

The two fields occur on the same ticks. They must have the same width and
lie on different wires. A role that owns one and samples the other runs an
`EXCHANGE`. The listener has to sample both, which a lane cannot do today
(gap G5). Example: SPI.

### 3.10 `repeat`

| Form | Meaning | Example |
|---|---|---|
| `repeat N { }` | fixed count N (an expression over parameters) | WS2812 `repeat leds`, Ethernet preamble |
| `repeat { } while COND` | do-while; COND uses fields completed in the body, and every role evaluates it | I2C read loop |
| `repeat { }` (open) | last statement of a frame with `until`. The owner of the `until` events gets an iteration count from its host before the first iteration. Other roles loop until the event | SPI words, I2C write loop |

### 3.11 `if`

```
if COND { } [else if COND { } ...] [else { }]
```

If COND uses only parameters, it is resolved at compile time (UART parity).
If it uses fields, it is a branch at run time that every role takes.

### 3.12 Check values: `crc` and check functions

```
crc NAME = width W poly P init I xorout X [reflect]
```

- **Definition.** The parameters are those of the standard CRC catalogue
  (Williams model, with input and output reflection equal). `reflect` means
  bits enter the register at its least significant end. The check unit
  processes bits in wire order.
- **Use.** A check is used as the expected value of a field of width W:
  `NAME(F1, F2, F3 .. F4)`. It covers every listed field, every occurrence,
  in wire order. A range `A .. B` names every field written from A to B in
  the description. The register is initialised just before the first covered
  field of each frame.
- **Each role.** The owner drives the result. Others compare it.
- **Limit.** One check can be active at a time, because a lane has one check
  unit. The compiler enforces this.

The library is written in the same syntax:

```
// The standard library: ordinary crc statements.
crc even_parity    = width 1  poly 0x1        init 0          xorout 0
crc odd_parity     = width 1  poly 0x1        init 0          xorout 1
crc crc5_usb       = width 5  poly 0x05       init 0x1F       xorout 0x1F       reflect
crc crc16_usb      = width 16 poly 0x8005     init 0xFFFF     xorout 0xFFFF     reflect
crc crc32_ethernet = width 32 poly 0x04C11DB7 init 0xFFFFFFFF xorout 0xFFFFFFFF reflect
```

I checked these with a bit-serial model:

- The three CRCs match the catalogue check values (CRC-5/USB 0x19,
  CRC-16/USB 0xB4C8, CRC-32/ISO-HDLC 0xCBF43926).
- The 1-bit definitions give even and odd parity for all 256 bytes.
- The USB specification describes its CRC as "the unreflected register, sent
  MSb first". That is the same wire bits as `crc5_usb` sent `lsb`; I checked
  this on 1000 random tokens.

### 3.13 Bit stuffing: `stuff`

```
stuff zero after N ones [over A .. B]          USB (N = 6), HDLC (N = 5)
stuff complement after N equal [over A .. B]   CAN (N = 5)
```

- **Coverage.** By default stuffing covers every field of the frame. `over`
  narrows it to a range of fields (CAN stops before the CRC delimiter).
- **Each role.** The owner inserts the extra bits and observers remove them.
  Inserted bits are not fields and are never recorded.
- **Run count.** It carries across consecutive covered fields and starts at
  the first covered bit. For USB that means the last bit of SYNC already
  counts toward six.
- **Violation.** N + 1 equal bits produce an error record, and the frame
  ends.

Example: USB in section 4.1.

### 3.14 Rules, the monitor, and Stress

```
rule NAME : BODY >= T          BODY:  high W | low W                 pulse width
rule NAME : BODY <= T                 W setup to EDGE                W unchanged for T before EDGE
rule NAME : BODY in T1 .. T2          W hold from EDGE               W unchanged for T after EDGE
                                      A to B                         time from each A to the next B
EDGE, A, B:  W rises | W falls | W changes | an event name | capture | launch
```

`capture` and `launch` are the clock's edges, so `mosi setup to capture`
works in every SPI mode.

**Who is responsible for a rule.** The party that drives whatever moves:

- for a pulse width: W's driver;
- for setup and hold: W's driver at that moment, which is the owner of the
  field on it;
- for `A to B`: the owner of B.

**A rule has three uses.**

1. **The compiler** chooses `PERIOD`, the launch delay (`LAUNCH`) and `PIN`
   hold counts so that the role's own rules hold with margin. If they cannot
   all hold, it rejects the program and reports the fastest rate that does
   meet them (I2C at 400 kHz gives 384 kHz). Durations are rounded to 60 MHz
   cycles conservatively: minimums round up, maximums round down.
2. **The monitor.** Each rule becomes an assertion when the role under proof
   is responsible for it, and an assumption otherwise. The monitor also
   contains the rules implied by the description:
   - clock tolerance;
   - frame structure: field order, expected values, check values, stuffing,
     events only at frame boundaries;
   - outputs enabled only during the role's own fields;
   - for the listener, no output enabled ever;
   - host data on the wire equals what the host queued ("for all data",
     arch obligation 9).
3. **Stress.** The loader asks a role to violate one of its own rules by a
   given amount, for example "`mosi_su` by 15 ns" (the bound becomes −5 ns,
   and the setup actually produced is rounded to whole 16.7 ns cycles). The
   compiler moves the matching setting and recompiles the monitor with that
   one rule relaxed by exactly that amount. The proof
   then says the program breaks this rule by this much and nothing else.
   Only rules the role is responsible for can be stressed. Section 6.2 (G9)
   lists what the current launch delay cannot reach.

**How monitors are generated.** The installed Yosys (0.70+385, from the OSS
CAD Suite) has no Verific front end and rejects concurrent SVA sequences: I
checked, and `|->` is a syntax error. So a monitor is a plain Verilog module
made of counters and a frame-tracking state machine, with immediate
`assert` and `assume` statements. Every open-source flow accepts that. Here
is the generated shape for `t_low` (this snippet parses with the installed
Yosys):

```verilog
module i2c_monitor #(parameter DUT_DRIVES_SCL = 1) (input clk, input scl, input sda);
  // rule t_low : low scl >= 4.7 us   -> 282 cycles at 60 MHz
  reg [8:0] scl_low_for = 0;          // cycles scl has been low, saturating
  reg       scl_q = 1;
  always @(posedge clk) begin
    scl_q       <= scl;
    scl_low_for <= scl ? 9'd0 : (&scl_low_for ? scl_low_for : scl_low_for + 9'd1);
    if (scl && !scl_q) begin          // scl rose this cycle
      if (DUT_DRIVES_SCL) assert (scl_low_for >= 9'd282);
      else                assume (scl_low_for >= 9'd282);
    end
  end
endmodule
```

### 3.15 The host view

Every party role has an **in-queue** (host to chip) and an **out-queue**
(chip to host). These are the ring buffers in data memory (arch §6). The
listener has only an out-queue.

**In-queue words**, consumed strictly in description order:

| Item the party owns | Word |
|---|---|
| a data field | its value in the low bits (the code, for named values) |
| an open `repeat` whose end events it owns | the iteration count, before the first iteration |
| a frame end where it owns two or more `until` events | the position of the chosen event in the list |

**Waiting for the host.** A program waits for any word it needs, and the
wires hold their state while it does:

- a UART line stays idle;
- an SPI or I2C controller pauses its clock between bits;
- a target stretches the clock if `stretch by` allows it; otherwise the
  field must have a `default`, and the compiler checks this.

**Out-queue records.**

| Role | Records |
|---|---|
| party | each data field it samples; each event it does not own; each mismatch (`error FIELD=seen`); `cut FIELD` when a foreign event ends the frame partway through a field |
| listener | every data field, every event, every error (an expected field that matches is not recorded), each with a 32-bit timestamp (16.7 ns, wraps every 71.6 s) |

**When a frame is abandoned (`else end`, or a check or stuffing failure).**

- If the role owns the frame's end events, it generates the first event in
  the `until` list (`stop`, for I2C), records the error, raises the host
  interrupt and halts. The loader discards the rest of that frame's words
  and restarts the lane through the host port's run and PC controls.
- Any other role records the error and waits for the frame to end.

**Proposed encoding.** A party record is one word: `{tag[7:0],
value[23:0]}`. A listener record adds the timestamp word that `CAP` already
pushes. The compiler writes a manifest that maps tags to names, kinds and
named values, so the loader can decode records into
`addr=0x50 rw=write ...`. Wider fields arrive as several records. Tags need
a small hardware change (cost C1). Without it, the host must decode by
position, which section 6.3 describes.

### 3.16 What the compiler checks

1. Every name resolves. Units and widths are consistent, and constants fit
   their fields.
2. Branch and loop conditions use only complete fields, and `when` uses only
   fields the owner has seen.
3. An open `repeat` is last in a frame that has `until`. A clock-wire frame
   has `after`.
4. A data field whose owner does not control its timing has a `default`,
   unless the clock is `stretch by` that owner.
5. `when` appears only on open-drain fields whose expected value has a 0
   bit.
6. Per role: at most 4 events, at most one active check value, and at most 4
   inputs and 4 outputs on one lane.
7. The rate rounding is within tolerance, and every role's own rules hold
   with margin. The compiler reports the fastest rate for each role.
8. The gap-free schedule (arch §5.2): the control instructions between two
   fields fit inside the earlier field at the fastest allowed rate.
9. Transmitted symbols share one period and start high.

---

## 4. Stretching it: low-speed USB and 10 Mbit Ethernet

These are sketches. `[ext]` marks constructs that are **not** in the core
language. Both sketches fail to parse with the core grammar and parse with
the extension productions in section 5.2.

### 4.1 Low-speed USB

```
// Low-speed USB (1.5 Mbit/s) packets and transactions. SKETCH: [ext] marks
// constructs that are not in the core language.
protocol usb_ls {
  param address = 0 runtime                           // [ext] host updates it after SET_ADDRESS

  parties host, device

  wire  d diff (dm, dp) idle high coding nrzi        // level = D-, so J = 1 at low speed
  clock rate 1.5 MHz tolerance 1.5%
  stuff zero after 6 ones over sync .. crc16

  event eop = dm, dp both low for 1 bit .. 2 bit by any     // [ext] any: whoever sent the packet

  frame packet(sender) until eop {                     // [ext] party parameter
    sender sync[8] lsb = 0x80 seek                     // [ext] seek
    sender pid[4] lsb (OUT = 0x1, IN = 0x9, SETUP = 0xD, DATA0 = 0x3,
                       DATA1 = 0xB, ACK = 0x2, NAK = 0xA, STALL = 0xE)
    sender pid_check[4] lsb = ~pid
    if pid == OUT or pid == IN or pid == SETUP {
      sender addr[7] lsb
      sender endp[4] lsb
      sender crc5[5] lsb = crc5_usb(addr, endp)
    } else if pid == DATA0 or pid == DATA1 {
      repeat { sender data[8] lsb }
      sender crc16[16] lsb = crc16_usb(data)          // [ext] fixed field after an open repeat
    }
  }

  frame transaction {
    packet(host)                                       // [ext] use: token
    if pid == IN {
      packet(device) when addr == address              // DATA0/1, NAK or STALL
      if pid == DATA0 or pid == DATA1 {
        packet(host)                                   // ACK
      }
    } else {
      packet(host)                                     // DATA0/1 after SETUP or OUT
      packet(device) when addr == address              // ACK, NAK or STALL
    }
  }

  rule turnaround : eop to d changes <= 7.5 bit
}
```

**What fits the core.** The differential wire with NRZI coding, stuffing,
the PID with named values and its complement check (`= ~pid`), CRC-5 and
CRC-16, EOP as a "both low" event with separate generate and detect
durations, and the turnaround rule. The device's 7.5-bit turnaround is
5 µs, which is 300 cycles or 60 instruction slots. That is enough to check
the address, look for a staged reply, switch from `RECOVER` to `BAUD` and
start SYNC.

**What needed extensions:**

1. **Sub-frames with a party argument** (`frame packet(sender)`, used as
   `packet(host)`), plus events owned `by any`. A packet's owner depends on
   the transaction, not on the packet layout.
2. **A fixed field after an open repeat.** A receiver does not know where
   `crc16` starts until EOP. The compiler can only do this by running the
   check unit over everything and testing the residue at EOP. That needs the
   CRC-good flag (gap G6). The last two `data` records are then relabelled
   as `crc16` by the host decoder.
3. **`seek`.** A receiver may miss the first SYNC transitions, so it aligns
   on the pattern instead of counting bits. At 40 cycles per bit this can be
   done in software.
4. **`runtime` parameters.** The device address changes during enumeration.
   It would live in a data-memory word that the host writes, loaded with a
   `MEM` absolute load (isa §3.5).
5. **Defaults chosen per use.** The device's reply to IN should be "the
   staged DATA packet, or NAK if none is staged". In the core that is
   `default` on a data field, but here it has to be attached to one use of
   `packet(device)`. I have not proposed syntax for this.
6. **No reply.** A missing reply (a timeout) and a bus reset (SE0 for at
   least 2.5 µs) are events with no fields. The sketch omits them.

**What stays outside.** Descriptor contents, SET_ADDRESS sequencing, data
toggle tracking, endpoint halt and STALL policy, suspend and resume, and HID
report contents. These are state machines with tables. Section 4.3 says how
I propose to express them.

### 4.2 10 Mbit Ethernet (10BASE-T)

```
// 10BASE-T framing, one direction. SKETCH: [ext] marks non-core constructs.
protocol eth10 {
  parties sender, receiver

  wire  pair diff (p, n) by sender idle released coding manchester
  clock rate 10 MHz tolerance 0.01%

  event tp_idl = pair high for 150 ns .. 300 ns by sender  // TP_IDL

  frame packet until tp_idl {
    repeat 7 { sender preamble[8] lsb = 0x55 }
    sender sfd[8] lsb = 0xD5 seek                       // [ext]
    sender dest[48] net                                 // [ext] network byte order
    sender src[48] net
    sender ethertype[16] net
    repeat { sender payload[8] lsb }
    sender fcs[32] lsb = crc32_ethernet(dest .. payload)   // [ext] trailing field
  }

  rule ifg : tp_idl to pair changes >= 9.6 us
}
```

**What fits the core.** Manchester coding, the preamble and SFD constants,
CRC-32, end of frame as an event (TP_IDL: the line held high for longer than
any data interval, which is at most one bit, 100 ns), and the inter-frame
gap as a rule.

**What needed extensions:**

1. **`net` byte order.** Ethernet sends multi-byte fields most significant
   byte first, but each byte least significant bit first. Neither `msb` nor
   `lsb` describes that. The extension costs only the compiler, which splits
   the field into bytes.
2. **`seek` on the SFD.** Receivers lose an unknown number of preamble bits.
   At 10 Mbit/s a bit is 6 cycles, too fast for a software search, so this
   needs hardware (gap G7).
3. **The trailing FCS** after an open payload: a residue check at TP_IDL, as
   for USB (gap G6).

**What does not fit at all:**

- **Link pulses.** A 10BASE-T station must send a 100 ns pulse every
  16 ± 8 ms while idle, or the PC reports no link. That is periodic
  behaviour with no frame. I propose the host schedules it: a second frame,
  `nlp`, holding one pulse, requested by the host on its own timer. This
  needs more than one frame per protocol, which the core does not allow.
- **Full-size frames.** A 1518-byte frame is 380 of the 512 data-memory
  words that all lanes share. Receive works for small frames such as `ping`,
  not full-size ones.
- **Draining received frames.** 10 Mbit/s is 1.25 MB/s, more than a host
  port clocked at a few MHz can drain (cost C4).

**Rung 2 (transmit) needs no new hardware.** The frame comes from the
in-queue, with CRC-32 from the check unit and TP_IDL from a `PIN` hold. In
the language it needs the core plus `net`, the trailing-field relaxation
(free for a sender, which knows the length), and link pulses, which need
either more than one frame per protocol or a hand-written instruction-level
loop (the fallback in PLAN.md's risk table). Receive (rung 3) needs G6, G7
and the answer to arch open question 3 on sampling margin.

### 4.3 Application behaviour, outside the description

I propose a three-way split.

1. **Reactions with deadlines stay in the description, so they are proven.**
   These are:
   - fixed handshakes (`device pid = ACK` after a SETUP data packet; like
     the NAK default, this is a value chosen per use, extension 5);
   - a `default` reply when the host is not ready (NAK);
   - address filters with `when`.

   The chip makes these decisions within the turnaround time, and the
   monitor proves them.
2. **Everything else is Python on the RP2350**, written against an API the
   compiler generates from the description. Record types are named after
   fields, and send functions take the owned data fields in order. Python's
   only time pressure is "before the host gives up retrying". A USB host
   retries a NAKed IN indefinitely, so Python has time. A sketch:

   ```python
   kbd = rosetta.open("usb_ls.ros", role="device", lane=0)    # program + proof + manifest

   @kbd.on("packet", pid="SETUP")
   def setup(pkt):
       req = kbd.next_data(pkt)                     # the 8-byte SETUP payload, as records
       if req.request == GET_DESCRIPTOR:
           kbd.stage(endp=0, pid="DATA1", data=DEVICE_DESCRIPTOR)   # NAK until staged
       elif req.request == SET_ADDRESS:
           kbd.after_status(lambda: kbd.set_param("address", req.value))
   ```

3. **State shared between the two lives in data memory.** That covers
   `runtime` parameters and one staged-reply mailbox per endpoint (cost C5).

Why not put this in the language? These are state machines of unbounded
size, with tables. Expressing them would turn the language into a
programming language, and the chip has no room for them anyway: 512 program
words are shared by all four lanes. The same split covers Ethernet. ARP and
ICMP echo replies are host Python. The ones'-complement checksum in the
sequencer (arch open question 2, `ADDC` in isa §3.5) is needed only if the
chip should answer `ping` on its own.

### 4.4 Verdict

| Needed for | USB | Ethernet | Core today? | New hardware? |
|---|---|---|---|---|
| differential wire, NRZI / Manchester, stuffing, CRC | yes | yes | yes | no |
| events with durations (EOP, TP_IDL) | yes | yes | yes | define the timeout (G8) |
| sub-frames with a party argument, `by any` | yes | no | no | no |
| fixed field after an open repeat | yes | yes | no | CRC-good residue (G6) |
| `seek` a sync pattern | yes (software) | yes | no | pattern seek at 10 Mbit/s (G7) |
| `net` byte order | no | yes | no | no |
| `runtime` parameters, per-endpoint mailboxes | yes | no | no | no (costs C5) |
| periodic idle behaviour (link pulses) | no | yes | no | no (host timer) |
| application logic | outside | outside | never | no |

The constructs stretch to packet framing with five small extensions. They
do not stretch to transactions without multiple frames, and they should not
stretch to application logic.

---

## 5. Grammar (EBNF)

### 5.1 Core

Notation: `=` defines a rule and `;` ends it. Juxtaposition means sequence.
`|` separates alternatives, `[ ]` is optional, `{ }` is zero or more, and
`( )` groups. Quoted text is a terminal.

```ebnf
file         = { NL } top { NL { NL } top } { NL } ;         (* at most one protocol *)
top          = crc_def | protocol ;

protocol     = "protocol" NAME "{" NL { NL } { item NL { NL } } "}" ;
item         = param | parties | wire | clock | event | crc_def | stuff | frame | rule ;

param        = "param" NAME [ ":" type ] "=" expr ;
type         = NAME "|" NAME { "|" NAME }                     (* enumeration *)
             | INT ".." INT ;                                 (* range *)

parties      = "parties" NAME "," NAME { "," NAME } ;

wire         = "wire" NAME [ "diff" "(" NAME "," NAME ")" ] { wire_attr } ;
wire_attr    = "by" NAME | "idle" expr | "open_drain" | "coding" ( "nrzi" | "manchester" ) ;

clock        = "clock" "rate" expr [ "tolerance" expr ]
             | "clock" NAME "by" NAME "rate" expr "capture" expr [ "stretch" "by" NAME ]
             | "clock" "symbols" [ "tolerance" expr ] "{" NL { NL } symbol NL { NL } { symbol NL { NL } } "}" ;
symbol       = INT "=" "high" expr "," "low" expr ;

event        = "event" NAME "=" condition "by" NAME ;
condition    = NAME edge [ "while" NAME level ]
             | NAME level "for" duration
             | NAME "," NAME "both" "low" "for" duration ;
duration     = expr [ ".." expr ] ;
edge         = "rises" | "falls" | "changes" ;
level        = "high" | "low" ;

crc_def      = "crc" NAME "=" "width" INT "poly" INT "init" INT "xorout" INT [ "reflect" ] ;

stuff        = "stuff" ( "zero" "after" INT "ones" | "complement" "after" INT "equal" ) [ "over" range ] ;
range        = NAME ".." NAME ;

frame        = "frame" NAME [ "after" NAME ] [ "until" NAME { "or" NAME } ] block ;
block        = "{" { NL } [ stmt { NL { NL } stmt } ] { NL } "}" ;
stmt         = field | together | repeat | if ;

field        = NAME NAME "[" expr "]" [ order ] [ "on" NAME ] [ names ] [ value ]
               [ "else" ( "end" | "continue" ) ] ;
order        = "msb" | "lsb" ;
names        = "(" NAME "=" expr { "," NAME "=" expr } ")" ;
value        = "=" expr [ "when" expr ]
             | "default" expr ;

together     = "together" block ;
repeat       = "repeat" [ expr ] block [ "while" expr ] ;
if           = "if" expr block [ "else" ( block | if ) ] ;

rule         = "rule" NAME ":" rule_body ( ( ">=" | "<=" ) expr | "in" expr ".." expr ) ;
rule_body    = level NAME
             | NAME "setup" "to" edge_ref
             | NAME "hold" "from" edge_ref
             | edge_ref "to" edge_ref ;
edge_ref     = NAME edge | NAME | "capture" | "launch" ;

expr         = or_expr [ "?" expr ":" expr ] ;
or_expr      = and_expr { "or" and_expr } ;
and_expr     = not_expr { "and" not_expr } ;
not_expr     = "not" not_expr | cmp_expr ;
cmp_expr     = sum [ ( "==" | "!=" | "<" | "<=" | ">" | ">=" ) sum ] ;
sum          = term { ( "+" | "-" ) term } ;
term         = unary { ( "*" | "/" ) unary } ;
unary        = ( "~" | "-" ) unary | atom ;
atom         = NUMBER [ unit ] | NAME | value_word | NAME "(" [ arg { "," arg } ] ")" | "(" expr ")" ;
arg          = expr | range ;
unit         = "s" | "ms" | "us" | "ns" | "Hz" | "kHz" | "MHz" | "bit" | "%" ;
value_word   = "high" | "low" | "leading" | "trailing" | "rising" | "falling" | "released" ;
```

Lexical rules (section 3.1):

```ebnf
NAME    = ( letter | "_" ) { letter | digit | "_" } ;    (* not a keyword *)
INT     = digit { digit | "_" } | "0x" hex { hex | "_" } | "0b" ( "0" | "1" ) { "0" | "1" | "_" } ;
NUMBER  = INT | digit { digit } "." digit { digit } ;
NL      = end of line ;                                (* ignored inside ( ) *)
comment = "//" { any character but end of line } ;     (* ignored *)
```

### 5.2 Extensions used only by the section 4 sketches (not proposed for rung 1)

```ebnf
param        = "param" NAME [ ":" type ] "=" expr [ "runtime" ] ;
frame        = "frame" NAME [ "(" NAME { "," NAME } ")" ] [ "after" NAME ]
               [ "until" NAME { "or" NAME } ] block ;
stmt         = field | together | repeat | if | use ;
use          = NAME "(" NAME { "," NAME } ")" [ "when" expr ] ;
field        = NAME NAME "[" expr "]" [ order ] [ "on" NAME ] [ names ] [ value ] [ "seek" ]
               [ "else" ( "end" | "continue" ) ] ;
order        = "msb" | "lsb" | "net" ;
(* "any" as an event owner: the owner of the frame the event ends *)
```

### 5.3 Keywords (reserved, case-sensitive)

`after and both by capture changes clock coding complement continue crc
default diff else end equal event falling falls for frame from high hold idle
if in init launch leading low lsb manchester msb not nrzi ones on open_drain
or over param parties poly protocol rate released reflect repeat rises rising
rule setup stretch stuff symbols to together tolerance trailing until when
while width wire xorout zero`, plus the extension words `runtime seek net`.

Checking this draft against its own grammar caught two clashes, `width` and
`end`. I renamed those examples, and section 7 asks whether keywords should
be contextual instead.

---

## 6. Hardware mapping and gaps

### 6.1 What each construct compiles to

| Construct | Hardware (arch / isa draft) |
|---|---|
| `protocol` | One program image per role (isa §3), one monitor module, one manifest of tags and names |
| `param` | Constants folded into immediates, `PERIOD`, `THR0-2`, `LAUNCH` and the CRC registers |
| `parties` | Which roles to emit. The listener never sets an output enable (arch §8, obligation 8) |
| `wire ... by` | Lane output (value and enable) for the owner and lane input for the others, through the crossbar (arch §4, §10) |
| `open_drain` | Open-drain transmit (arch §4.2) on a `uio` pin, plus a lane input on the same pin |
| `idle` | `PIN` at program start and after each frame (isa §3.3) |
| `diff` | Differential transmit (arch §4.2); receive is the plain level of P plus "both low" detection |
| `coding` | `CODE` register: NRZI or Manchester (arch §4.2) |
| `clock rate` | `BAUD` for the owner, `RECOVER` (or `PULSE` for Manchester) for the others (arch §4.1); `PERIOD` |
| `clock W by P` | `BAUD` with clock output on W for P, `EDGE` on W for the others; launch and capture edge select in `TICK` |
| `stretch by` | `TICK` "honour clock stretching" for P (isa §4 only, see S4). Nothing for Q (G1) |
| `clock symbols` | Pulse-width transmit coding with `BAUD`; `PULSE` with `THR0-2` (arch §4.1, §4.2) |
| `event` | Owner: a `PIN` sequence with hold counts. Others: `EVC0-3` plus an `EVT` handler (arch §4.6, isa §3.6) |
| `frame after` / `until` | `EVT` handlers: the start event jumps to the top of the frame; an end event abandons the field in progress |
| data field | `FIELD DRIVE` from `HOSTQ`; `FIELD SAMPLE` to `HOSTQ`, or to `CAP` in the listener (isa §3.1) |
| `= EXPR` | `FIELD DRIVE` from `IMM` or a register; `FIELD EXPECT` plus `BR NM` |
| check function | Per-field CRC flag, `CRCINIT`, `DRIVE` from `CRC` / `EXPECT` against `CRC` (arch §4.4) |
| named values | An `ALU CMP` chain for validation; the names exist only in the manifest |
| `default` | `BR QE` around an alternative `FIELD DRIVE IMM` |
| `when` | `EXPECT` or `CMP` plus `BR`. The false path is a `FIELD SAMPLE` to `NONE` with the output released, then a jump to the frame end |
| `on W` | Data input and output select in `CODE` |
| `together` | `FIELD EXCHANGE` (arch §4.5) |
| `repeat N` | `LDI` plus `LOOP` |
| open `repeat` | Owner: `Q POP` the count, then `LOOP`. Others: an unconditional loop left by `EVT` |
| `repeat ... while` | `BR` on the sampled field (the `M` flag or a register) |
| `if` | Compile-time selection, or `ALU CMP` plus `BR` |
| `crc` | `CRCPOLY`, `CRCINIT`, `CRCCFG` |
| `stuff` | `STUFF` register plus the per-field stuffing flag (arch §4.3) |
| `rule` | Monitor assertion or assumption. Also sets `PERIOD`, `LAUNCH` and `PIN` hold counts, which Stress moves |
| in-queue, out-queue | Host ring buffers in data memory (arch §6); `HOSTQ`, `CAP`, `Q POP`, `Q PUSH` (isa §3.7) |
| error halt | `MISC IRQ`, `MISC HALT`; the host restarts the lane through run and PC control (arch §9) |

### 6.2 What the architecture as written cannot support

These are the important findings. G1-G4 affect the rung-1 claim
"UART, SPI, I2C in every role".

**G1. Clock stretching by a target** (rung 1, I2C target). `EDGE` mode only
follows the clock. Nothing holds `scl` low while the target's next field is
not ready, for example when the host has not yet queued the next read byte.
Without this, `stretch by target` cannot be compiled, and the target's read
data needs a `default` (0xFF is common) with every byte staged in advance.

*Smallest fix:* in `EDGE` mode, when the clock wire is also mapped to an
open-drain output, hold it low from the launch edge until the next field is
parked.

**G2. First bit at frame start for leading-edge capture** (rung 1, SPI
target in modes 0 and 2). With `cpha = 0` the target must put bit 7 on
`miso` when `cs_n` falls, before any clock edge. In `EDGE` mode a field
starts on its first launch tick, which is the first trailing edge: one bit
too late. `PIN` cannot help, because it drives an immediate and not the
field's first data bit. Without the fix, the SPI target is limited to modes
1 and 3.

*Smallest fix:* a field option "launch the first bit when the field starts".

**G3. Unequal clock halves** (rung 1, I2C fast mode). `BAUD` has one
half-period (`PERIOD`), so a generated clock has a 50 % duty cycle. Fast mode
needs at least 1.3 µs low and 0.6 µs high. At 400 kHz the equal halves are
1.25 µs, which breaks `t_low`. The best equal-halves rate is 384 kHz.

*Smallest fix:* separate low and high half-periods for the clock output (a
second 16-bit register).

**G4. False START or STOP from zero hold time** (rung 1, I2C target and
listener). UM10204 allows `sda` to change at the same moment `scl` falls
(`t_hd_dat` ≥ 0). `scl` and `sda` pass through separate two-flop
synchronisers (arch §10) that can resolve a cycle apart. So a data change can
appear as "`sda` edge while `scl` high", which the event detector (arch §4.6)
reports as START or STOP. The formal proof of the target against its monitor
will find this.

*Smallest fix:* an event fires only if its qualifying level still holds N
cycles after the edge. UM10204's note on 300 ns of internal hold suggests
N ≈ 18.

**G5. Two inputs in one field** (rung 2, SPI listener). `EXCHANGE` drives one
wire and samples one wire. A listener must sample `mosi` and `miso` on the
same ticks.

*Smallest fix:* a dual-sample field kind (two inputs into the two halves of
the shift register). Without it, an SPI listener takes two lanes, one per
data wire, and the host merges them by timestamp.

**G6. Residue check (CRC-good)** (rung 2 USB device receive, rung 3
Ethernet receive). A receiver cannot use `EXPECT CRC` when the check value
follows a variable-length payload, because it does not know where the check
field starts. Arch §5.3 lists a "CRC-good" flag but does not define it, and
isa.md v0.1 dropped it.

*Smallest fix:* CRC-good means "the register equals a configured residue",
plus a `BR` condition for it and a residue value (a register, or reuse of
the final-XOR value).

**G7. Pattern seek at the bit rate** (rung 3, Ethernet receive). Receivers
must align on the SFD after losing an unknown number of preamble bits. At
10 Mbit/s a bit is 6 cycles, fewer than the 10 cycles a two-instruction
loop needs (one instruction per 5-cycle slot), so a software bit loop cannot
keep up.

*Smallest fix:* an `EXPECT` variant that keeps shifting until the last N
bits match. USB at low speed has 40 cycles per bit and can seek in
software.

**G8. Level-for-duration events** (WS2812 latch, Ethernet TP_IDL, UART
break). Arch §4.6 says only "a timeout", without saying what resets it. The
language's `W low for D` needs "W at a level, with no edge on W, for D".

*Smallest fix:* specify the timeout as a counter reset by each edge on a
chosen input, qualified by that input's level. Until it is specified, these
events cannot be relied on.

**G9. Stress cannot reach some violations** (rung 3). There is one launch
delay per lane, from 0 to 255 cycles, and it only delays data after its
launch edge. This has two consequences:

- **Hold.** Hold time measured from the capture edge cannot be made shorter
  than half a clock period for data the chip launches, so SPI hold cannot be
  stressed at all. A role that drives both clock and data cannot skew one
  against the other.
- **Setup at slow clocks.** At 100 kHz I2C, setup can only be pushed down to
  about 0.75 µs (300 − 255 cycles). The limit is 250 ns, so Stress cannot
  reach a violation.

*Smallest fix:* a delay on the clock output as well as on data, and a launch
delay counted in prescaled ticks or widened to 16 bits.

**Cost of the rung-1 fixes.** G1-G4, plus the record tag C1 below, are small
counters and multiplexers. I estimate them at a few tens of flops per lane
in total *(estimate)*, compared with about 480 flops per lane in arch §11.

### 6.3 Supported, at a cost

**C1. Record tags.** Out-queue records carry no type, so the host cannot
tell `addr` from `data` or an error from a value. The fallback is that the
host decodes by replaying the description against the values. Events and
errors then need explicit marker pushes (`LDI` plus `Q PUSH`, two
instructions each), and the decoder cannot resynchronise after an overflow.

*Fix:* `FIELD SAMPLE` to `HOSTQ` or `CAP` writes 8 bits of its otherwise
unused 10-bit immediate into the value word's bits [31:24].

**C2. One destination per sample.** A field that steers a branch and must
also reach the host (I2C `rw` and `ack`, USB `pid`) is sampled into a
register and pushed afterwards. That costs one or two instructions, and its
timestamp is late by a few slots (about 0.1-0.3 µs).

*Fix:* sampling to `HOSTQ` or `CAP` also latches the value into a fixed
register.

**C3. Event timestamps.** An event record is pushed by its handler
(`Q PUSH TS`), late by up to one wheel turn plus the handler's instructions.
The constant part can be subtracted.

*Fix:* latch the timestamp in the event detector.

**C4. Host port bandwidth.** A timestamped record is 8 bytes. With a 4 MHz
host SPI clock, the sustained drain is roughly 50,000 records per second
*(estimate)*.

- UART at standard rates and I2C at 100 kHz stream continuously.
- I2C at 400 kHz streams writes (one record per byte). Reads produce two
  records per byte (`data` and the controller's `ack`), about 89,000 per
  second, so they stream only if the compiler merges consecutive fields into
  one record; section 7 asks whether it should.
- SPI at MHz rates and Ethernet capture in bursts, limited by the queue
  space in data memory.

**C5. One host in-queue per lane.** A USB device needs one staged reply per
endpoint. These go in mailboxes in data memory, reached with `MEM` loads. It
costs instructions inside the 60-slot turnaround, and they fit.

**C6. Immediates are 10 bits and registers 16 bits.** Wider constants (the
56-bit preamble, 48-bit addresses) are split into several fields. The
schedule still has no gaps, but each split costs program words.

**C7. Program memory is 512 words, shared by all lanes.** Rough sizes per
role *(estimate)*:

| Role | Words |
|---|---|
| UART | 10-20 |
| SPI | 15-25 |
| I2C | 40-60 |
| USB device | 150-250 |

The USB number should be checked early, because it decides what else can
share the chip with a USB keyboard.

**C8. Lanes.** A full-duplex UART takes 2 lanes, and so does an SPI listener
(while G5 stands). Four lanes hold, for example, one SPI listener and one
full-duplex UART.

**C9. SPI target top speed.** The target launches `miso` after the input
synchroniser, about 4-5 cycles (67-83 ns) plus pad and Tiny Tapeout I/O
delays after the launch edge. The data must then be set up before the
capture edge, half a period later. That gives roughly 5 MHz SCK
*(estimate)*. The compiler derives the exact figure from `miso_su`. Rung-1
SPI target claims should state it.

**C10. The pulse transmit coder selects only the high time.** Symbols that
differ in low time or in period (NEC infrared, 1-Wire) need a `RAW DRIVE`
for each edge. Infrared carrier modulation at 38 kHz is not in the
architecture at all, which matters only for infrared from arch §12.

### 6.4 To pin down in isa.md or the interface spec

- **S1. Clock output and `PIN`.** Say how the clock output behaves between
  fields, and whether `PIN` may override it. I2C needs `scl` to stay low
  between bits and between fields inside a frame, and needs `PIN` control of
  `scl` for START, STOP and repeated START.
- **S2. Stuffing run count.** It must persist across consecutive stuffed
  fields. Its reset point needs defining: for USB, the last bit of SYNC
  counts.
- **S3. What `EVT` keeps.** When `EVT` abandons a field, say what survives:
  the CRC, the stuffing count, and the partial shift contents. The `cut`
  record wants the partial bits.
- **S4. Clock stretching in the architecture.** "Honour clock stretching"
  belongs in arch §4.1. Today it appears only in the isa.md `TICK` register.
- **S5. `PIN` timing.** Does `PIN` wait for the current field to finish? Can
  it be aligned to the next launch edge? Target-side actions after a capture
  edge need this.
- **S6. CRC-good** (G6) and **S7. the timeout event** (G8).

---

## 7. Open questions and decisions

### 7.1 The three decisions I most want from the owner

**D1. Add the rung-1 lane features before the 20 Dec architecture freeze,
or narrow the rung-1 claims?**

The features are G1 (target stretching), G2 (first bit at frame start), G3
(unequal clock halves), G4 (event confirmation window), and C1 (record tags).
Without them, "every role" really means all of the following:

- an I2C target that cannot stretch and must have every read byte staged;
- an SPI target in modes 1 and 3 only;
- I2C no faster than 384 kHz;
- an I2C target and listener that are correct only against controllers that
  give real SDA hold time;
- a host decoder that must replay the description.

*Recommendation:* add all five. Each is a counter or multiplexer inside a
lane, a few tens of flops in total *(estimate)*. G4 will be forced anyway by
the first failing proof. C1 makes the host view robust and costs almost
nothing.

**D2. Approve the host view in section 3.15.**

It has four parts:

- queue words in description order;
- an iteration count for open loops;
- an end-choice word;
- on an error, the frame-owning role halts and the host restarts it.

The alternative is explicit terminator and marker words in the in-queue,
with no counts. That needs tags on in-queue words as well, and makes the
program check every word.

*Recommendation:* approve as proposed. Counts map directly to `LOOP`, and
the error-halt path uses only existing host controls.

**D3. Freeze the core now, and decide the hardware for rungs 2-3 now while
deferring their language.**

- Approve sections 2, 3 and 5.1 as the rung-1 language.
- Approve the five extension directions of section 4 in principle:
  - sub-frames with party arguments and `by any`;
  - fixed fields after open repeats;
  - `seek`;
  - `net` byte order;
  - `runtime` parameters.
- Write the extensions only when rung 2 starts.
- Decide the hardware they depend on before 20 Dec: G6 (CRC-good residue),
  G7 (pattern seek) and G8 (defined timeout). Language can be added later;
  hardware cannot.

*Recommendation:* exactly that. G6 and G8 are cheap and needed for the
rung-2 USB keyboard. G7 matters only for rung-3 Ethernet receive and could
be dropped if rung 3 is unlikely.

### 7.2 Other open questions (with my lean)

1. **Keywords.** Fully reserved (simple, as drafted) or contextual
   (friendlier: `width` and `end` would be usable as names)? *Lean:* a
   contextual lexer in the compiler (Lark's LALR mode provides one), with
   the documentation still telling authors to avoid keywords.
2. **Event durations.** Is `Tmin .. Tmax` (detect at the minimum, generate
   at the maximum) clear enough, or should generate and detect be named
   separately? *Lean:* keep the range.
3. **Listener records.** One record per field (clear), or merge consecutive
   fields (needed for I2C reads at 400 kHz under C4)? *Lean:* one per field
   by default, with a compiler option to merge.
4. **Load-time parameters.** Should parameters be changeable at load time
   without recompiling? *Lean:* no. The compiler is fast, images are small,
   and each image keeps its own proof.
5. **Addressing.** `when` on a field, as drafted, or a party-level statement
   such as `target answers when addr == address`? *Lean:* keep `when`. It is
   local and needs no new concept.
6. **More than one frame per protocol** (USB transactions, Ethernet link
   pulses). *Lean:* not in rung 1. Revisit with the section 4 extensions.
7. **1-Wire read slots.** The controller starts each bit and the target
   decides its length, so one bit has two owners. The core cannot express
   this. *Lean:* leave 1-Wire to instruction-level programs unless a
   per-segment owner in `clock symbols` turns out to be small.
8. **Composing lanes** (Translate, Intercept). This needs a small wiring
   file, for example
   `lane 0 = uart.receiver(baud=9600); lane 1 = spi.controller; lane0.data -> lane1.mosi_data`.
   *Lean:* specify it with Translate in rung 2, outside this document.
9. **Monitor style.** Immediate assertions over generated counters only,
   because the installed Yosys has no Verific. Using concurrent SVA would
   need the commercial Tabby CAD suite. *Lean:* stay with immediate
   assertions.
10. **Naming.** `controller` and `target` (the current SPI and I2C terms),
    and `sender` and `receiver` for one-way links. *Lean:* keep them.
