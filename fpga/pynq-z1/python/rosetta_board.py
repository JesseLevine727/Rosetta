"""PYNQ driver for the Rosetta Tiny Tapeout harness on a PYNQ-Z1.

The Zynq plays the RP2350's role: it clocks the chip at 60 MHz, drives
rst_n / ena / ui_in / uio_in, reads uo_out / uio_out / uio_oe, talks to the
chip's host SPI port (ui0=SCK, ui1=CS_n, ui2=MOSI, uo0=MISO, uo1=IRQ) and
captures every chip pin each chip clock cycle.

    from rosetta_board import load
    b = load()                    # program the FPGA (needs root), reset chip
    b.host_mode('gpio')           # drive ui[0..2] from the processor too
    b.set_inputs(ui=0x35, uio=0x10)
    print(hex(b.get_uo()))        # 0x45 with the template placeholder design
    cap = b.capture(1024, trigger='ui:change', action=lambda: b.set_ui(0x36))
    print(cap.changes()[:5])

Register map and pin mapping: see README.md.
"""
import os
import time

import numpy as np
from pynq import MMIO, Overlay

__all__ = ["load", "RosettaBoard", "Capture", "PINS"]

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_BIT = os.path.join(HERE, "rosetta.bit")

# ----------------------------------------------------------------- address map
GPIO_BASE = {   # AXI GPIO instances (dual channel: ch1 @ +0x0, ch2 @ +0x8)
    "ctrl": 0x41200000, "pins": 0x41210000, "route": 0x41220000, "loop": 0x41230000,
    "trig": 0x41240000, "cap": 0x41250000, "mon": 0x41260000, "freq": 0x41270000,
    # lockstep mode (lockstep.py, README "Lockstep mode")
    "ls": 0x41280000, "run": 0x41290000, "host": 0x412A0000, "net": 0x412B0000,
    "skew": 0x412C0000, "fault": 0x412D0000, "cyc": 0x412E0000, "async": 0x412F0000,
    # waveform player, pulls, CAN bus model, async lines
    "wave": 0x41300000, "chmap": 0x41310000, "pull": 0x41320000, "can": 0x41330000,
    "line": 0x41340000,
}
TRC_BASE = [0x42000000, 0x42010000, 0x42020000]   # change-trace words 0, 1, 2
TRC_DEPTH = 16384
WAVE_BASE, LINE_BASE, WAVE_DEPTH, LINE_DEPTH = 0x42030000, 0x42040000, 16384, 16384
SPI_BASE = 0x41E00000
BRAM_BASE = 0x40000000
CAP_DEPTH = 16384
CH1, CH2 = 0x0, 0x8
F_NOMINAL = 60_000_000

# CTRL (gpio_ctrl ch1)
C_RST_N, C_ENA, C_CLK_RUN, C_STEP, C_HDR_EN, C_CAP_ARM, C_CAP_FORCE = (1 << i for i in range(7))
C_MODE_SHIFT, C_MODE_MASK = 8, 0x300
C_DIV_SHIFT, C_DIV_MASK = 16, 0xFFFF0000
# STATUS (gpio_ctrl ch2)
S_LOCKED, S_ARMED, S_TRIGGERED, S_DONE, S_CHIP_CE = (1 << i for i in range(5))

TRIG_MODES = {"immediate": 0, "level": 1, "edge": 2, "change": 3}

# AXI Quad SPI registers (PG153)
SPI_SRR, SPI_CR, SPI_SR, SPI_DTR, SPI_DRR, SPI_SSR, SPI_TXOCY, SPI_RXOCY = (
    0x40, 0x60, 0x64, 0x68, 0x6C, 0x70, 0x74, 0x78)
CR_SPE, CR_MASTER, CR_CPOL, CR_CPHA = 1 << 1, 1 << 2, 1 << 3, 1 << 4
CR_TXRST, CR_RXRST, CR_MANUAL_SS, CR_INHIBIT, CR_LSB = 1 << 5, 1 << 6, 1 << 7, 1 << 8, 1 << 9
SR_RX_EMPTY, SR_TX_EMPTY = 1 << 0, 1 << 2
SPI_FIFO = 256

# ----------------------------------------------------------------- pin naming
# bit positions inside a capture sample (and loopback source indices for uo/uio)
PINS = {}
for _i in range(8):
    PINS[f"ui{_i}"] = _i
    PINS[f"uo{_i}"] = 8 + _i
    PINS[f"uio{_i}"] = 16 + _i
    PINS[f"oe{_i}"] = 24 + _i
PINS.update(sck=0, cs=1, cs_n=1, mosi=2, miso=8, irq=9)
GROUPS = {"ui": 0, "uo": 8, "uio": 16, "oe": 24}


def _input_index(name):
    """Chip input index for routing: ui0..ui7 -> 0..7, uio0..uio7 -> 8..15."""
    if isinstance(name, int):
        return name
    n = name.lower()
    if n.startswith("uio"):
        return 8 + int(n[3:])
    if n.startswith("ui"):
        return int(n[2:])
    return {"sck": 0, "cs": 1, "cs_n": 1, "mosi": 2}[n]


def _output_index(name):
    """Loopback source index: uo0..uo7 -> 0..7, uio pin 0..7 -> 8..15."""
    if isinstance(name, int):
        return name
    n = name.lower()
    if n.startswith("uio"):
        return 8 + int(n[3:])
    if n.startswith("uo"):
        return int(n[2:])
    return {"miso": 0, "irq": 1}[n]


def parse_trigger(trigger):
    """Return (mode, mask, value).

    None / 'immediate'          trigger at once
    'PIN:KIND'                  PIN in PINS, KIND in high/low/rise/fall/change
    'GROUP:change'              GROUP in ui/uo/uio/oe (any bit of the byte)
    'GROUP=0xNN'                level match of a whole byte
    (mode, mask, value)         raw, mode name or number
    """
    if trigger is None or trigger == "immediate":
        return 0, 0, 0
    if isinstance(trigger, (tuple, list)):
        mode, mask, value = trigger
        return TRIG_MODES.get(mode, mode), mask & 0xFFFFFFFF, value & 0xFFFFFFFF
    t = trigger.replace(" ", "").lower()
    if "=" in t:
        grp, val = t.split("=")
        sh = GROUPS[grp]
        return 1, 0xFF << sh, (int(val, 0) & 0xFF) << sh
    name, kind = t.split(":")
    if name in GROUPS:
        if kind != "change":
            raise ValueError("byte groups only support ':change' or '=value'")
        return 3, 0xFF << GROUPS[name], 0
    bit = 1 << PINS[name]
    return {"high": (1, bit, bit), "low": (1, bit, 0), "rise": (2, bit, bit),
            "fall": (2, bit, 0), "change": (3, bit, 0)}[kind]


# ----------------------------------------------------------------- capture
class Capture:
    """Samples of all chip pins, one per (decimated) chip clock cycle.

    raw[i] bits: [7:0] ui_in, [15:8] uo_out, [23:16] uio pin, [31:24] uio_oe.
    Sample i is the pin state the chip sampled at its i-th captured rising edge.
    """

    def __init__(self, raw, trigger_index, div=0, f_clk=F_NOMINAL):
        self.raw = np.asarray(raw, dtype=np.uint32)
        self.trigger_index = trigger_index
        self.div = div
        self.period_ns = 1e9 * (div + 1) / f_clk

    def __len__(self):
        return len(self.raw)

    ui = property(lambda s: (s.raw & 0xFF).astype(np.uint8))
    uo = property(lambda s: ((s.raw >> 8) & 0xFF).astype(np.uint8))
    uio = property(lambda s: ((s.raw >> 16) & 0xFF).astype(np.uint8))
    oe = property(lambda s: ((s.raw >> 24) & 0xFF).astype(np.uint8))

    def bit(self, pin):
        return ((self.raw >> PINS[pin]) & 1).astype(np.uint8)

    def edges(self, pin, kind="rise"):
        b = self.bit(pin).astype(np.int8)
        d = np.diff(b)
        sel = {"rise": d == 1, "fall": d == -1, "any": d != 0}[kind]
        return np.nonzero(sel)[0] + 1

    def changes(self, limit=None):
        """[(index_rel_trigger, t_ns, ui, uo, uio, oe)] for the first sample and every change."""
        idx = np.concatenate(([0], np.nonzero(np.diff(self.raw.astype(np.int64)))[0] + 1))
        out = []
        for i in idx[:limit]:
            r = int(self.raw[i])
            rel = int(i) - self.trigger_index
            out.append((rel, round(rel * self.period_ns, 1), r & 0xFF, (r >> 8) & 0xFF,
                        (r >> 16) & 0xFF, (r >> 24) & 0xFF))
        return out

    def table(self, limit=40):
        lines = ["  sample    t(ns)   ui  uo  uio  oe"]
        for rel, t, ui, uo, uio, oe in self.changes(limit):
            lines.append(f"  {rel:+6d} {t:9.1f}   {ui:02x}  {uo:02x}  {uio:02x}   {oe:02x}")
        return "\n".join(lines)

    def to_vcd(self, path):
        ps = int(round(self.period_ns * 1000))
        sigs = [("ui_in", 0, 8, "!"), ("uo_out", 8, 8, '"'), ("uio", 16, 8, "#"),
                ("uio_oe", 24, 8, "$"), ("sck", 0, 1, "%"), ("cs_n", 1, 1, "&"),
                ("mosi", 2, 1, "'"), ("miso", 8, 1, "("), ("irq", 9, 1, ")"),
                ("trigger", None, 1, "*")]
        with open(path, "w") as f:
            f.write("$timescale 1ps $end\n$scope module rosetta $end\n")
            for name, _, w, code in sigs:
                f.write(f"$var wire {w} {code} {name} $end\n")
            f.write("$upscope $end\n$enddefinitions $end\n")
            prev = None
            for i, r in enumerate(self.raw.tolist()):
                if r == prev and i != self.trigger_index and i != self.trigger_index + 1:
                    continue
                f.write(f"#{i * ps}\n")
                for name, sh, w, code in sigs:
                    if sh is None:
                        f.write(f"{int(i == self.trigger_index)}{code}\n")
                    elif w == 1:
                        f.write(f"{(r >> sh) & 1}{code}\n")
                    else:
                        f.write(f"b{(r >> sh) & 0xFF:08b} {code}\n")
                prev = r
            f.write(f"#{len(self.raw) * ps}\n")


# ----------------------------------------------------------------- board
class RosettaBoard:
    def __init__(self, bitfile=DEFAULT_BIT, download=True):
        self.bitfile = bitfile
        self.overlay = Overlay(bitfile, download=download)
        ip = self.overlay.ip_dict
        mem = getattr(self.overlay, "mem_dict", {})

        def base(name, default):
            for d in (ip, mem):
                if name in d and "phys_addr" in d[name]:
                    return d[name]["phys_addr"]
            return default

        self.gpio = {k: MMIO(base("gpio_" + k, v), 0x10000) for k, v in GPIO_BASE.items()}
        self.spi = MMIO(base("spi", SPI_BASE), 0x10000)
        self.bram = MMIO(base("cap_bram_ctrl", BRAM_BASE), CAP_DEPTH * 4)
        self.trc = [MMIO(base(f"trc{t}_bram_ctrl", a), TRC_DEPTH * 4) for t, a in enumerate(TRC_BASE)]
        self.wavemem = MMIO(base("wave_bram_ctrl", WAVE_BASE), WAVE_DEPTH * 4)
        self.linemem = MMIO(base("line_bram_ctrl", LINE_BASE), LINE_DEPTH * 4)
        st = self.status_word()
        if (st >> 24) != 0x52:
            raise RuntimeError(f"harness ID mismatch (STATUS={st:#010x}); wrong bitstream?")
        if not st & S_LOCKED:
            raise RuntimeError("60 MHz MMCM not locked")
        self._ctrl = self.gpio["ctrl"].read(CH1)
        self._drive = self.gpio["pins"].read(CH1)
        self._src = self.gpio["route"].read(CH1)
        self._src2 = self.gpio["route"].read(CH2)
        self._loop = [self.gpio["loop"].read(CH1), self.gpio["loop"].read(CH2)]
        self._spi_init()

    # ---------------------------------------------------------- low level
    def _set_ctrl(self, v):
        self._ctrl = v & 0xFFFFFFFF
        self.gpio["ctrl"].write(CH1, self._ctrl)

    def status_word(self):
        return self.gpio["ctrl"].read(CH2)

    def status(self):
        st = self.status_word()
        chip = self.gpio["pins"].read(CH2)
        return dict(locked=bool(st & S_LOCKED), cap_armed=bool(st & S_ARMED),
                    cap_triggered=bool(st & S_TRIGGERED), cap_done=bool(st & S_DONE),
                    clock_enabled=bool(st & S_CHIP_CE), rst_n=bool(self._ctrl & C_RST_N),
                    ena=bool(self._ctrl & C_ENA), headers=bool(self._ctrl & C_HDR_EN),
                    ui=(st >> 8) & 0xFF, uo=chip & 0xFF, uio_out=(chip >> 8) & 0xFF,
                    uio_oe=(chip >> 16) & 0xFF, uio=(chip >> 24) & 0xFF,
                    cycles=self.cycles())

    # ---------------------------------------------------------- reset / clock
    def reset(self, cycles=16):
        """Hold rst_n low for at least `cycles` chip clocks, then release."""
        self._set_ctrl(self._ctrl & ~C_RST_N)
        if self._ctrl & C_CLK_RUN:
            time.sleep(max(cycles / F_NOMINAL, 20e-6))
        else:
            self.step(cycles)
        self._set_ctrl(self._ctrl | C_RST_N)

    def set_rst_n(self, level):
        self._set_ctrl((self._ctrl | C_RST_N) if level else (self._ctrl & ~C_RST_N))

    def set_ena(self, level):
        self._set_ctrl((self._ctrl | C_ENA) if level else (self._ctrl & ~C_ENA))

    def clock_run(self, run=True):
        """Free-run the 60 MHz chip clock (True) or stop it (False) for step()."""
        self._set_ctrl((self._ctrl | C_CLK_RUN) if run else (self._ctrl & ~C_CLK_RUN))

    def step(self, n=1):
        """Give the chip exactly n clock edges (clock must be stopped)."""
        for _ in range(n):
            self._set_ctrl(self._ctrl ^ C_STEP)

    def cycles(self):
        """Free-running count of chip clock edges (32-bit, wraps)."""
        return self.gpio["mon"].read(CH2)

    def measure_clock(self, seconds=1.0):
        """Chip clock frequency three ways:
        sw   : chip-edge counter vs Linux CLOCK_MONOTONIC over `seconds`
        sys  : hardware meter, edges per 125e6 cycles of the 125 MHz board oscillator
        fclk : hardware meter, edges per 100e6 cycles of PS FCLK0
        """
        def stamp():
            t0 = time.monotonic()
            c = self.cycles()
            t1 = time.monotonic()
            return c, (t0 + t1) / 2, t1 - t0
        seq0 = self.status_word() >> 16 & 0xFF
        c0, t0, e0 = stamp()
        time.sleep(seconds)
        c1, t1, e1 = stamp()
        f_sw = ((c1 - c0) & 0xFFFFFFFF) / (t1 - t0)
        # wait for two fresh 1 s windows of both meters
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            seq = self.status_word() >> 16 & 0xFF
            if ((seq & 0xF) - (seq0 & 0xF)) % 16 >= 2 and ((seq >> 4) - (seq0 >> 4)) % 16 >= 2:
                break
            time.sleep(0.05)
        f_sys = self.gpio["freq"].read(CH1)
        f_fclk = self.gpio["freq"].read(CH2)
        return dict(sw_hz=f_sw, sw_window_s=t1 - t0, sw_read_uncertainty_s=e0 + e1,
                    sys_hz=f_sys, fclk_hz=f_fclk)

    # ---------------------------------------------------------- pins
    def set_ui(self, value):
        """Processor drive value for ui_in[7:0] (bits routed elsewhere ignore it)."""
        self._drive = (self._drive & ~0xFF) | (value & 0xFF)
        self.gpio["pins"].write(CH1, self._drive)

    def set_uio(self, value):
        """Processor drive value for uio_in[7:0] (used where uio_oe=0)."""
        self._drive = (self._drive & ~0xFF00) | ((value & 0xFF) << 8)
        self.gpio["pins"].write(CH1, self._drive)

    def set_inputs(self, ui, uio):
        """Set ui and uio drive values in one register write (same chip cycle)."""
        self._drive = (ui & 0xFF) | ((uio & 0xFF) << 8)
        self.gpio["pins"].write(CH1, self._drive)

    def get_ui(self):
        """ui_in exactly as the chip sees it (after routing)."""
        return (self.status_word() >> 8) & 0xFF

    def get_uo(self):
        return self.gpio["pins"].read(CH2) & 0xFF

    def get_uio(self):
        """uio pin values as the chip sees them on uio_in (own driven bits read back)."""
        return (self.gpio["pins"].read(CH2) >> 24) & 0xFF

    def get_uio_out(self):
        return (self.gpio["pins"].read(CH2) >> 8) & 0xFF

    def get_uio_oe(self):
        return (self.gpio["pins"].read(CH2) >> 16) & 0xFF

    def get_pads(self):
        """Actual header pad levels: dict(ui=Arduino IO0-7, uo=JA, uio=JB)."""
        p = self.gpio["mon"].read(CH1)
        return dict(ui=p & 0xFF, uo=(p >> 8) & 0xFF, uio=(p >> 16) & 0xFF)

    def headers(self, enable=True):
        """Drive the board headers (uo on JA, uio on JB where oe=1, ui monitor on IO0-7)."""
        self._set_ctrl((self._ctrl | C_HDR_EN) if enable else (self._ctrl & ~C_HDR_EN))

    # ---------------------------------------------------------- routing
    def _write_routes(self):
        self.gpio["route"].write(CH1, self._src)
        self.gpio["route"].write(CH2, self._src2)
        self.gpio["loop"].write(CH1, self._loop[0])
        self.gpio["loop"].write(CH2, self._loop[1])

    def route(self, inp, source):
        """Select where chip input `inp` ('ui3', 'uio5', ...) comes from:
        'cpu' (processor drive value), 'header' (board pin), 'spi' (ui0..2 only),
        or a chip output name for loopback ('uo4', 'uio2', 'miso', 'irq').
        Loopback goes through one 60 MHz register (+1 cycle vs. a wire)."""
        i = _input_index(inp)
        loop_bit, ext_bit = 1 << (16 + i), 1 << i
        self._src &= ~(loop_bit | ext_bit)
        if i < 3:
            self._src2 &= ~(1 << i)
        if source == "cpu":
            pass
        elif source == "header":
            self._src |= ext_bit
        elif source == "spi":
            if i >= 3:
                raise ValueError("only ui0..ui2 can come from the SPI master")
            self._src2 |= 1 << i
        else:
            k = _output_index(source)
            w, sh = divmod(i, 8)
            self._loop[w] = (self._loop[w] & ~(0xF << (4 * sh))) | (k << (4 * sh))
            self._src |= loop_bit
        self._write_routes()

    def loopback(self, inp, out):
        self.route(inp, out)

    def host_mode(self, mode):
        """'spi': ui0..2 = AXI Quad SPI (SCK, CS_n, MOSI) like the RP2350;
        'gpio': ui0..2 driven by set_ui()/bit-banging."""
        for i in range(3):
            self.route(i, "spi" if mode == "spi" else "cpu")

    def clear_routes(self, host="spi"):
        self._src, self._loop = 0, [0, 0]
        self._src2 = 0x7 if host == "spi" else 0
        self._write_routes()

    # ---------------------------------------------------------- host SPI
    def _spi_init(self):
        self.spi.write(SPI_SRR, 0x0000000A)
        self.spi.write(SPI_SSR, 0xFFFFFFFF)
        self.spi.write(SPI_CR, CR_MASTER | CR_SPE | CR_MANUAL_SS | CR_INHIBIT | CR_TXRST | CR_RXRST)

    def spi_transfer(self, data, cpol=0, cpha=0, lsb_first=False, timeout=1.0):
        """Full-duplex transfer with CS_n held low for the whole buffer.
        SCK = 60 MHz / C_SCK_RATIO (3.75 MHz in the default build)."""
        data = bytes(data)
        base = (CR_MASTER | CR_SPE | CR_MANUAL_SS | (CR_CPOL if cpol else 0)
                | (CR_CPHA if cpha else 0) | (CR_LSB if lsb_first else 0))
        s = self.spi
        s.write(SPI_CR, base | CR_INHIBIT | CR_TXRST | CR_RXRST)
        s.write(SPI_SSR, 0xFFFFFFFE)
        rx = bytearray()
        try:
            for off in range(0, len(data), SPI_FIFO):
                chunk = data[off:off + SPI_FIFO]
                for byte in chunk:
                    s.write(SPI_DTR, byte)
                s.write(SPI_CR, base)                      # release inhibit: go
                got, deadline = 0, time.monotonic() + timeout
                while got < len(chunk):
                    if not s.read(SPI_SR) & SR_RX_EMPTY:
                        rx.append(s.read(SPI_DRR) & 0xFF)
                        got += 1
                    elif time.monotonic() > deadline:
                        raise TimeoutError("SPI transfer timed out")
                s.write(SPI_CR, base | CR_INHIBIT)
        finally:
            s.write(SPI_SSR, 0xFFFFFFFF)
            s.write(SPI_CR, base | CR_INHIBIT)
        return bytes(rx)

    def spi_transfer_bitbang(self, data, cpol=0, cpha=0, step_clock=0):
        """Slow host-SPI fallback: bit-bangs ui0..2 from the processor (needs
        host_mode('gpio')); about 10 kHz SCK. MISO is read from uo[0].
        If step_clock > 0 the chip clock must be stopped (clock_run(False)) and
        the chip is given `step_clock` edges after every pin change, so a host
        transaction can be single-stepped and captured cycle by cycle."""
        if self._src2 & 0x7:
            raise RuntimeError("call host_mode('gpio') first")
        rx = bytearray()

        def put(sck, cs, mosi):
            self.set_ui((self._drive & 0xF8) | (mosi << 2) | (cs << 1) | sck)
            if step_clock:
                self.step(step_clock)
        idle = 1 if cpol else 0
        put(idle, 1, 0)
        put(idle, 0, 0)
        for byte in bytes(data):
            r = 0
            for i in range(7, -1, -1):
                bit = (byte >> i) & 1
                if not cpha:
                    put(idle, 0, bit)
                    put(1 - idle, 0, bit)            # leading edge: both sides sample
                    r = (r << 1) | (self.get_uo() & 1)
                else:
                    put(1 - idle, 0, bit)            # leading edge: shift out
                    put(idle, 0, bit)                # trailing edge: sample
                    r = (r << 1) | (self.get_uo() & 1)
            rx.append(r)
        put(idle, 0, 0)
        put(idle, 1, 0)
        return bytes(rx)

    # ---------------------------------------------------------- capture
    def capture(self, n=4096, trigger=None, pre=0, div=0, action=None, timeout=2.0):
        """Capture n samples (one per chip clock, or per div+1 clocks).

        trigger: see parse_trigger(); `pre` samples before the trigger are kept.
        action : optional callable run once the capture is armed and the
                 pre-trigger part is filled (e.g. lambda: b.spi_transfer(...)).
        Returns a Capture whose trigger sample is at index `pre`.
        """
        if not 1 <= n <= CAP_DEPTH or not 0 <= pre < n:
            raise ValueError(f"need 1 <= n <= {CAP_DEPTH} and 0 <= pre < n")
        mode, mask, value = parse_trigger(trigger)
        self.gpio["trig"].write(CH1, mask)
        self.gpio["trig"].write(CH2, value)
        self.gpio["cap"].write(CH1, ((pre & 0xFFFF) << 16) | (n & 0xFFFF))
        ctrl = (self._ctrl & ~(C_CAP_ARM | C_CAP_FORCE | C_MODE_MASK | C_DIV_MASK)) \
            | (mode << C_MODE_SHIFT) | ((div & 0xFFFF) << C_DIV_SHIFT)
        self._set_ctrl(ctrl)                 # idle, new config
        self._set_ctrl(ctrl | C_CAP_ARM)     # arm
        if pre and self._ctrl & C_CLK_RUN:
            # wait until the pre-trigger part is filled (with a stopped clock it
            # can only be filled by the action itself, e.g. stepped transfers)
            t_end = time.monotonic() + timeout
            while ((self.gpio["cap"].read(CH2) >> 16) & 0x3FFF) < pre:
                if self.status_word() & (S_TRIGGERED | S_DONE) or time.monotonic() > t_end:
                    break
        if action is not None:
            action()
        deadline = time.monotonic() + timeout
        while not self.status_word() & S_DONE:
            if time.monotonic() > deadline:
                self._set_ctrl(ctrl)
                raise TimeoutError("capture did not trigger/finish "
                                   f"(STATUS={self.status_word():#010x})")
        cs = self.gpio["cap"].read(CH2)
        trig_addr = cs & 0x3FFF
        buf = np.array(self.bram.array[:CAP_DEPTH], dtype=np.uint32)
        idx = (trig_addr - pre + np.arange(n)) % CAP_DEPTH
        self._set_ctrl(ctrl)                 # disarm
        return Capture(buf[idx], pre, div)


def load(bitfile=DEFAULT_BIT, download=True, reset=True):
    """Program the FPGA with the harness, set defaults, optionally reset the chip.
    Defaults: 60 MHz clock running, ena=1, ui0..2 from the SPI master, all other
    inputs from the processor (0), headers hi-Z."""
    b = RosettaBoard(bitfile, download)
    b._set_ctrl(C_ENA | C_CLK_RUN)           # rst_n=0, headers off, capture idle
    b.set_inputs(0, 0)
    b.clear_routes(host="spi")
    if reset:
        b.reset()
    return b
