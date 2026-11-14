"""Independent protocol benches for the model tests.

These generate and decode wire waveforms from first principles (bit times,
edges, bus levels) and never look inside the model, so they check the
model's pins, not its internals."""
from __future__ import annotations

import support  # noqa: F401

CLOCK_HZ = 60_000_000


# ----------------------------------------------------------------- UART ---

def uart_levels(data: list[int], bit_cycles: float, idle_before: int = 1000,
                gap_bits: float = 0.0, idle_after: int = 2000) -> list[int]:
    """Line level per cycle for 8N1 frames at `bit_cycles` cycles per bit."""
    levels = [1] * idle_before
    t = float(len(levels))
    for byte in data:
        for bit in [0] + [(byte >> i) & 1 for i in range(8)] + [1]:
            nxt = t + bit_cycles
            levels += [bit] * (int(round(nxt)) - int(round(t)))
            t = nxt
        t += gap_bits * bit_cycles
        levels += [1] * (int(round(t)) - len(levels))
    return levels + [1] * idle_after


def uart_decode(levels: list[str] | list[int], baud: float) -> tuple[list[int], list[int]]:
    """Decode 8N1 from per-cycle levels ('0'/'1'/'z' or ints; z reads as 1,
    the idle pull-up). Returns (bytes, cycle of each start-bit edge).
    Raises AssertionError on a framing error."""
    lv = [0 if str(v) == "0" else 1 for v in levels]
    bit = CLOCK_HZ / baud
    out, starts, t = [], [], 1
    while t < len(lv):
        if lv[t - 1] == 1 and lv[t] == 0:
            centre = lambda k: int(round(t + (k + 0.5) * bit))   # noqa: E731
            if centre(9) >= len(lv):
                break
            assert lv[centre(0)] == 0, f"start bit at {t} not low at its centre"
            byte = sum(lv[centre(1 + i)] << i for i in range(8))
            assert lv[centre(9)] == 1, f"framing error in byte starting at {t}"
            out.append(byte)
            starts.append(t)
            t = centre(9)
        t += 1
    return out, starts


# ------------------------------------------------------------------ SPI ---

class SpiTarget:
    """SPI mode 0 target on the lane's pins: CS = out2, SCK = out3,
    MOSI = out0; drives MISO on in0. Samples MOSI on SCK rising edges while
    CS is low, shifts MISO on falling edges (bit 7 first, valid from CS fall)."""

    def __init__(self, replies: list[int]):
        self.replies = list(replies)
        self.received: list[int] = []
        self.sck = 0
        self.cs = 1
        self.word = self.bit = self.cur = self.n = 0
        self.rising_edges = 0

    def __call__(self, t: int, val: int, en: int) -> int:
        sck = val >> 3 & 1 if en >> 3 & 1 else 0
        cs = val >> 2 & 1 if en >> 2 & 1 else 1
        mosi = val & 1
        if self.cs == 1 and cs == 0:
            self.word = self.replies.pop(0) if self.replies else 0
            self.bit, self.cur, self.n = 7, 0, 0
        if cs == 0 and sck == 1 and self.sck == 0:
            self.rising_edges += 1
            self.cur = (self.cur << 1) | mosi
            self.n += 1
            if self.n == 8:
                self.received.append(self.cur)
        if cs == 0 and sck == 0 and self.sck == 1:
            self.bit -= 1
        self.sck, self.cs = sck, cs
        miso = self.word >> self.bit & 1 if cs == 0 and self.bit >= 0 else 0
        return 0b1110 | miso


# ------------------------------------------------------------------ I2C ---

class I2cController:
    """Bit-banged I2C controller sharing an open-drain bus with the lane:
    SDA is lane in0/out0, SCL is lane in1/out3. The controller waits while
    SCL is held low by the target (clock stretching)."""

    def __init__(self, half: int = 300):
        self.half = half
        self.script: list[tuple[int, int, int, str]] = []   # (cycles, scl, sda, action)
        self.i = self.c = 0
        self.samples: list[int] = []
        self.sda_bus = self.scl_bus = 1
        self.stretch_cycles = 0

    def idle(self, n: int) -> None:
        self.script.append((n, 1, 1, ""))

    def start(self) -> None:
        h = self.half
        self.script += [(h, 1, 1, ""), (h, 1, 0, ""), (h // 2, 0, 0, "")]

    def stop(self) -> None:
        h = self.half
        self.script += [(h // 2, 0, 0, ""), (h, 1, 0, "stretch"), (h, 1, 1, "")]

    def bit(self, b: int, sample: bool = False) -> None:
        h = self.half
        self.script += [(h // 2, 0, b, ""), (h, 1, b, "stretch+sample" if sample else "stretch"),
                        (h // 2, 0, b, "")]

    def write_byte(self, byte: int) -> None:
        for i in range(7, -1, -1):
            self.bit(byte >> i & 1)
        self.bit(1, sample=True)                 # ACK slot: released, sampled

    def read_byte(self, ack: int) -> None:
        for _ in range(8):
            self.bit(1, sample=True)
        self.bit(ack)

    def done(self) -> bool:
        return self.i >= len(self.script)

    def __call__(self, t: int, val: int, en: int) -> int:
        lane_sda_low = en & 1 and not val & 1
        lane_scl_low = en >> 3 & 1 and not val >> 3 & 1
        n, scl, sda, action = self.script[self.i] if not self.done() else (1, 1, 1, "")
        self.scl_bus = 0 if (not scl or lane_scl_low) else 1
        self.sda_bus = 0 if (not sda or lane_sda_low) else 1
        if not self.done():
            if "stretch" in action and scl and not self.scl_bus:
                self.stretch_cycles += 1         # target holds SCL: wait
            else:
                self.c += 1
                if "sample" in action and self.c == n // 2:
                    self.samples.append(self.sda_bus)
                if self.c >= n:
                    self.i, self.c = self.i + 1, 0
        return 0b1100 | (self.scl_bus << 1) | self.sda_bus

    def bytes_read(self) -> list[int]:
        """Group samples of read_byte calls into bytes (8 samples each)."""
        s = self.samples
        return [sum(s[k + i] << (7 - i) for i in range(8)) for k in range(0, len(s) - 7, 8)]


class I2cTargetBench:
    """I2C target on the lane's open-drain pins (SDA = in0/out0, SCL =
    in1/out3). Detects START, shifts in 8 bits on SCL rising edges, pulls
    SDA low for the ACK bit if the address matches, and holds SCL low for
    `stretch` cycles after every SCL falling edge of the address byte."""

    def __init__(self, address: int, stretch: int = 0):
        self.address, self.stretch = address, stretch
        self.scl = self.sda = 1
        self.bits: list[int] = []
        self.byte: int | None = None
        self.in_frame = False
        self.ack_low = False
        self.hold = 0
        self.falls = 0
        self.high_times: list[int] = []      # bus SCL high durations
        self.low_times: list[int] = []
        self._since = 0

    def __call__(self, t: int, val: int, en: int) -> int:
        lane_sda_low = en & 1 and not val & 1
        lane_scl_low = en >> 3 & 1 and not val >> 3 & 1
        scl = 0 if (lane_scl_low or self.hold > 0) else 1
        sda = 0 if (lane_sda_low or self.ack_low) else 1
        if self.hold > 0:
            self.hold -= 1
        self._since += 1
        if scl != self.scl:
            (self.high_times if self.scl else self.low_times).append(self._since)
            self._since = 0
        if self.scl and scl and self.sda and not sda:          # START
            self.in_frame, self.bits, self.falls = True, [], 0
        if self.in_frame and not self.scl and scl and len(self.bits) < 8:
            self.bits.append(sda)
            if len(self.bits) == 8:
                self.byte = sum(b << (7 - i) for i, b in enumerate(self.bits))
        if self.in_frame and self.scl and not scl:             # falling edge
            self.falls += 1
            if self.ack_low:
                self.ack_low = False
            elif len(self.bits) == 8 and self.byte is not None and self.byte >> 1 == self.address \
                    and self.falls == 9:
                self.ack_low = True
            if self.falls <= 8 and self.stretch:
                self.hold = self.stretch
                scl = 0
        self.scl, self.sda = scl, sda
        return 0b1100 | (scl << 1) | sda
