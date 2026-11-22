"""Small targeted programs on the chip RTL, one feature at a time, lane to
lane (src/rosetta_top.sv under Verilator, or the FPGA board with
ROSETTA_BACKEND=board).

These fill the gaps the compiled UART, SPI and I2C scenarios leave, so that
every block and every mode has run on hardware at least once (the coverage
table is rosetta-fpga's regress.py output; each test's `COVERS` entry says
what it exercises). Every test has a definite expected result taken from
the specs (docs/blocks/*.md, docs/isa.md, docs/decisions.md): known CRC
answers, a round trip of the sent data, exact bit times, which events fire.

Bench: lane 0 transmits, lane 1 receives, lanes 2 and 3 help. Pins (unless a
test sets its own): uo2 <- L0.out0, uo3 <- L0.out1, uo4 <- L0.out3,
uo5 <- L1.out0, uo6 <- L1.out3, uo7 <- L2.out0; board wires uo2..uo6 ->
ui3..ui7. Lane inputs: L1 and L2 in0 = ui3 (L0 data), in1 = ui5 (L0 clock),
in2 = ui4 (L0 out1), in3 = ui6; L0 and L3 in0 = ui6 (L1 data), in1 = ui7
(L1 clock), in2 = ui3, in3 = ui5.

Skips when the selected backend is unavailable (rtlsim.available()).
"""
import unittest

import support  # noqa: F401
import rtlsim
from rosetta import asm

NOP = 0xC0000000
ORG = [0, 64, 128, 192]
UO_DATA, UO_COMP, UO_CLK, UO_B, UO_BCLK, UO_C = 2, 3, 4, 5, 6, 7
STD_XIN0 = (6 | 7 << 4 | 3 << 8 | 5 << 12) | (3 | 5 << 4 | 4 << 8 | 6 << 12) << 16
STD_XIN1 = (3 | 5 << 4 | 4 << 8 | 6 << 12) | (6 | 7 << 4 | 3 << 8 | 5 << 12) << 16
TIME, IRQSTAT, IRQMASK, CHSTAT, CHDATA0 = 0x03, 0x04, 0x05, 0x06, 0x08

# test name -> features it exercises on hardware (rosetta-fpga regress.py)
COVERS: dict[str, list[str]] = {}


def covers(*features):
    def deco(fn):
        COVERS[fn.__name__] = list(features)
        return fn
    return deco


def osel(lane: int, out: int) -> int:
    return 0x10 | lane << 2 | out


def image_of(progs: dict[int, str]) -> list[int]:
    words = {lane: asm.assemble(text, ORG[lane]).words for lane, text in progs.items()}
    image = [NOP] * max(ORG[k] + max(64, len(w)) for k, w in words.items())
    for lane, w in words.items():
        nxt = min([ORG[k] for k in progs if ORG[k] > ORG[lane]] + [512])
        assert ORG[lane] + len(w) <= nxt, f"lane {lane} program overlaps the next one"
        image[ORG[lane]:ORG[lane] + len(w)] = w
    return image


def bench(chip, progs, xin=(STD_XIN0, STD_XIN1), outs=None, wires=True):
    chip.load_program(image_of(progs))
    outs = outs or [osel(0, 0), osel(0, 1), osel(0, 3), osel(1, 0), osel(1, 3), osel(2, 0)]
    chip.wr(rtlsim.XOUT0, sum(o << (5 * k) for k, o in enumerate(outs)))
    if wires:
        for f in range(2, 7):
            chip.wire(f, f + 1)
    chip.wr(rtlsim.XIN0, xin[0])
    chip.wr(rtlsim.XIN1, xin[1])


def edges(changes, bit, col=1, start=None):
    """[(cycle, level)] of one pin; col 1 = uo, 2 = ui, 3 = uio_out, 4 = uio_oe, 5 = uio_in."""
    out, prev = [], start
    for row in changes:
        v = row[col] >> bit & 1
        if v != prev:
            out.append((row[0], v))
            prev = v
    return out


def level_at(edge_list, t, default=0):
    v = default
    for c, b in edge_list:
        if c > t:
            break
        v = b
    return v


def sample_at_rising(changes, data_bit, clk_bit):
    """Data pin level at every rising edge of the clock pin (both uo)."""
    clk = edges(changes, clk_bit)
    dat = edges(changes, data_bit)
    return [level_at(dat, c) for c, v in clk[1:] if v == 1]


def to_int(bits, lsb=False):
    if lsb:
        bits = bits[::-1]
    v = 0
    for b in bits:
        v = v << 1 | b
    return v


def crc_ref(data_bits, width, poly, init_ones, xorout):
    """docs/blocks/crc.md update rule, wire order."""
    m = (1 << width) - 1
    s = m if init_ones else 0
    for b in data_bits:
        fb = (s >> (width - 1) & 1) ^ b
        s = ((s << 1) & m) ^ (poly & m if fb else 0)
    return s ^ m if xorout else s


def bytes_bits(data, lsb):
    out = []
    for d in data:
        out += [(d >> i) & 1 for i in (range(8) if lsb else range(7, -1, -1))]
    return out


TX_CLK = """
        SETM  TICK, mode=BAUD, clock_out
        SETM  PERIOD, 8
        SETM  PERIOD2, 8
"""
RX_EDGE = """
        SETM  TICK, mode=EDGE, clock_in=in1
"""


def run_pair(progs, push0=(), run=4000, xin=None, outs=None, qcfg=0xF, pre=None):
    """Start lanes 3..1 (receivers first), then lane 0; push `push0` to lane
    0; run. Returns (changes, queues, ctrl words)."""
    with rtlsim.Chip() as chip:
        bench(chip, progs, **({"xin": xin} if xin else {}), outs=outs)
        if pre:
            pre(chip)
        for lane in sorted(progs, reverse=True):
            if lane != 0:
                chip.start_lane(lane, ORG[lane], qcfg)
        chip.run(200)
        chip.dump()
        if 0 in progs:
            chip.start_lane(0, ORG[0], qcfg)
        if push0:
            chip.push(0, list(push0))
        chip.run(run)
        changes = chip.dump(full=True)
        queues = {k: chip.drain(k, out_size=8 << (qcfg >> 2 & 3)) for k in progs}
        ctrl = {k: chip.rd(rtlsim.lane_reg(k, rtlsim.CTRL)) for k in progs}
    return changes, queues, ctrl


@unittest.skipUnless(rtlsim.available(), rtlsim.unavailable_reason())
class CheckUnit(unittest.TestCase):

    @covers("crc.w16", "crc.init_ones", "crc.msb_first", "crc.expect", "field.crc_flag",
            "field.drive", "field.sample", "field.expect", "tick.BAUD", "tick.clock_out",
            "tick.EDGE", "tick.capture_rising", "seq.loop", "seq.halt", "coding.level")
    def test_crc16_ccitt_round_trip(self):
        """CRC-16/CCITT-FALSE over "123456789" (crc.md: 0x29B1) sent after
        the data, checked on the wire and by the receiver's EXPECT CRC."""
        cfg = """
        SETM  CRCPOLY, 0x1021
        SETM  CRCCFG, width=16, init_ones
        CRCINIT
        LDI   R1, 9
"""
        tx = TX_CLK + cfg + """
        WAIT  timeout=20
loop:   FIELD DRIVE 8, crc, HOSTQ
        LOOP  R1, loop
        FIELD DRIVE 16, CRC
        HALT
"""
        rx = RX_EDGE + cfg + """
loop:   FIELD SAMPLE 8, crc, HOSTQ
        LOOP  R1, loop
        FIELD EXPECT 16, CRC
        BR    M, good
        Q     PUSHI 0xBA, tag=0xEE
        HALT
good:   Q     PUSHI 0x60, tag=0xEE
        HALT
"""
        ch, q, ctrl = run_pair({0: tx, 1: rx}, b"123456789")
        bits = sample_at_rising(ch, UO_DATA, UO_CLK)
        self.assertEqual(len(bits), 9 * 8 + 16)
        self.assertEqual(bytes(to_int(bits[i:i + 8]) for i in range(0, 72, 8)), b"123456789")
        self.assertEqual(to_int(bits[72:]), 0x29B1)
        self.assertEqual(q[1], list(b"123456789") + [0xEE000060])
        self.assertEqual(ctrl[0] & 0x9, 0x8, "lane 0 halted with attention")

    @covers("crc.w32", "crc.w16", "crc.w15", "crc.w8", "crc.w5", "crc.w1", "crc.xorout",
            "crc.init_ones", "crc.init_zero", "crc.lsb_first", "crc.pushcrc", "crc.crcchk",
            "cfg.setm_hi", "cfg.setm_reg", "field.lsb")
    def test_crc_known_answers(self):
        """One lane drives "123456789" through the check unit in six
        configurations (crc.md known answers) and pushes each value; then
        CRCCHK compares the CRC-32 state with {R1, R0}."""
        cases = [  # name, width, poly, init_ones, xorout, lsb, expected (crc.md table)
            ("CRC-32", 32, 0x04C11DB7, 1, 1, True, 0x649C2FD3),
            ("CRC-16/USB", 16, 0x8005, 1, 1, True, 0x132D),
            ("CRC-16/CCITT", 16, 0x1021, 1, 0, False, 0x29B1),
            ("CRC-15/CAN", 15, 0x4599, 0, 0, False, 0x059E),
            ("CRC-8", 8, 0x07, 0, 0, False, 0xF4),
            ("CRC-5/USB", 5, 0x05, 1, 1, True, 0x13),
            ("parity", 1, 0x1, 0, 0, False, 1),
        ]
        prog = TX_CLK
        for i, (name, w, poly, ini, xo, lsb, _) in enumerate(cases):
            flags = "lsb, crc" if lsb else "crc"
            prog += f"""
        SETM  CRCPOLY, {poly & 0xFFFF:#x}
        LDI   R2, {poly >> 16:#x}
        SETM  CRCPOLY.hi, R2
        SETM  CRCCFG, width={w}{', init_ones' if ini else ''}{', xorout_ones' if xo else ''}
        CRCINIT
        LDI   R1, 9
        LDI   R3, 0
l{i}:   LD    R0, [R3+]
        FIELD DRIVE 8, {flags}, R0
        LOOP  R1, l{i}
        Q     PUSHCRC, tag=0
"""
        state = crc_ref(bytes_bits(b"123456789", True), 32, 0x04C11DB7, 1, 0)  # raw CRC-32 state
        prog += f"""
        SETM  CRCPOLY, 0x1DB7
        SETM  CRCPOLY.hi, 0x04C1
        SETM  CRCCFG, width=32, init_ones, xorout_ones
        CRCINIT
        LDI   R1, 9
        LDI   R3, 0
lc:     LD    R0, [R3+]
        FIELD DRIVE 8, lsb, crc, R0
        LOOP  R1, lc
        LDI   R0, {state & 0xFFFF:#x}
        LDI   R1, {state >> 16:#x}
        CRCCHK
        BR    NM, bad
        Q     PUSHI 0x11, tag=0xCC
        LDI   R0, 0
        CRCCHK
        BR    M, bad
        Q     PUSHI 0x22, tag=0xCC
        HALT
bad:    Q     PUSHI 0xBA, tag=0xEE
        HALT
"""
        # the nine bytes in the lane's data memory, via the host (data space)
        def pre(chip):
            chip.wr(rtlsim.MADDR, 0x200 | (0 << 7) | 0)
            for b in b"123456789":
                chip.wr(rtlsim.MDATA, b)
        prog = prog.replace("        SETM  PERIOD2, 8\n", "        SETM  PERIOD2, 8\n", 1)
        _, q, _ = run_pair({0: prog}, run=12000, pre=pre)
        for (name, w, poly, ini, xo, lsb, want), got in zip(cases, q[0]):
            ref = crc_ref(bytes_bits(b"123456789", lsb), w, poly, ini, xo)
            self.assertEqual(ref, want, f"reference model disagrees with crc.md for {name}")
            self.assertEqual(got, want, name)
        self.assertEqual(q[0][len(cases):], [0xCC000011, 0xCC000022])


@unittest.skipUnless(rtlsim.available(), rtlsim.unavailable_reason())
class StuffingAndCoding(unittest.TestCase):

    @covers("stuff.ones", "stuff.n6", "field.stuff_flag")
    def test_stuffing_ones_round_trip(self):
        """USB-style: after six 1s a 0 is inserted (stuff.md); the receiver
        drops it. 0xFF 0xFF then 0x7: wire 1x6 0 1x6 0 1111 0111."""
        st = "        SETM  STUFF, n=6, mode=ONES, enable, violation_event\n"
        tx = TX_CLK + st + """
        WAIT  timeout=20
        FIELD DRIVE 8, stuff, IMM 0xFF
        FIELD DRIVE 8, stuff, IMM 0xFF
        FIELD DRIVE 4, stuff, IMM 0x7
        HALT
"""
        rx = RX_EDGE + st + """
        FIELD SAMPLE 8, stuff, HOSTQ
        FIELD SAMPLE 8, stuff, HOSTQ
        FIELD SAMPLE 4, stuff, HOSTQ
        HALT
"""
        ch, q, _ = run_pair({0: tx, 1: rx})
        self.assertEqual(sample_at_rising(ch, UO_DATA, UO_CLK),
                         [1] * 6 + [0] + [1] * 6 + [0] + [1] * 4 + [0, 1, 1, 1])
        self.assertEqual(q[1], [0xFF, 0xFF, 0x7])

    @covers("stuff.equal", "stuff.n5", "field.stuff_flag")
    def test_stuffing_equal_round_trip(self):
        """CAN-style: after five equal bits the complement is inserted."""
        st = "        SETM  STUFF, n=5, mode=EQUAL, enable\n"
        tx = TX_CLK + st + """
        WAIT  timeout=20
        FIELD DRIVE 8, stuff, IMM 0x00
        FIELD DRIVE 8, stuff, IMM 0xFC
        HALT
"""
        rx = RX_EDGE + st + """
        FIELD SAMPLE 8, stuff, HOSTQ
        FIELD SAMPLE 8, stuff, HOSTQ
        HALT
"""
        ch, q, _ = run_pair({0: tx, 1: rx})
        # 00000 1 000 | 11 111 0 1 00 : 0x00 then 0xFC
        self.assertEqual(sample_at_rising(ch, UO_DATA, UO_CLK),
                         [0] * 5 + [1] + [0] * 3 + [1] * 2 + [1] * 3 + [0] + [1, 0, 0])
        self.assertEqual(q[1], [0x00, 0xFC])

    @covers("stuff.violation_event", "events.vector", "seq.abort")
    def test_stuff_violation_raises_event_3(self):
        """Eight 1s with no stuff bit: the receiver's sixth-1 rule is broken,
        event 3 fires and abandons the field (isa 3.6)."""
        st = "        SETM  STUFF, n=6, mode=ONES, enable, violation_event\n"
        tx = TX_CLK + """
        WAIT  timeout=20
        FIELD DRIVE 8, IMM 0xFF
        HALT
"""
        rx = RX_EDGE + st + """
        EVT   3, bad, on
        FIELD SAMPLE 8, stuff, HOSTQ
        HALT
bad:    Q     PUSHI 3, tag=0xEE
        HALT
"""
        _, q, _ = run_pair({0: tx, 1: rx})
        self.assertEqual(q[1], [0xEE000003])

    @covers("coding.tx.nrzi", "coding.rx.nrzi", "pins.pin")
    def test_nrzi_round_trip(self):
        """NRZI (USB convention): a 0 toggles the line, a 1 keeps it."""
        code = "        SETM  CODE, tx=NRZI, rx=NRZI\n"
        tx = TX_CLK + code + """
        PIN   out0=1
        WAIT  timeout=20
        FIELD DRIVE 8, IMM 0x00
        FIELD DRIVE 8, IMM 0xA5
        HALT
"""
        rx = RX_EDGE + code + """
        FIELD SAMPLE 8, HOSTQ
        FIELD SAMPLE 8, HOSTQ
        HALT
"""
        ch, q, _ = run_pair({0: tx, 1: rx})
        wire = sample_at_rising(ch, UO_DATA, UO_CLK)
        self.assertEqual(wire[:8], [0, 1, 0, 1, 0, 1, 0, 1])
        lvl, ref = 1, []
        for b in [(0xA5 >> (7 - i)) & 1 for i in range(8)]:
            lvl = lvl if b else 1 - lvl
            ref.append(lvl)
        self.assertEqual(wire[8:16], ref)
        self.assertEqual(q[1], [0x00, 0xA5])

    @covers("coding.tx.manchester", "coding.rx.manchester", "tick.PULSE", "tick.thr0")
    def test_manchester_round_trip(self):
        """Manchester (IEEE 802.3: a 1 is low then high), 12 cycles per half
        bit from a BAUD transmitter; a PULSE-mode receiver with THR0 between
        the half-bit and one-bit intervals decodes it (coding.md 3.2). The
        line idles low and the first bit is 1, so the first edge is mid-bit."""
        tx = """
        SETM  TICK, mode=BAUD
        SETM  CODE, tx=MANCHESTER
        SETM  PERIOD, 12
        SETM  PERIOD2, 12
        PIN   out0=0
        WAIT  timeout=20
        FIELD DRIVE 16, HOSTQ
        FIELD DRIVE 16, HOSTQ
        PIN   out0=0
        HALT
"""
        rx = """
        SETM  TICK, mode=PULSE
        SETM  CODE, rx=MANCHESTER
        SETM  THR01, thr0=18
        FIELD SAMPLE 16, HOSTQ
        FIELD SAMPLE 16, HOSTQ
        HALT
"""
        ch, q, _ = run_pair({0: tx, 1: rx}, [0xA53C, 0x0FF0])
        e = edges(ch, UO_DATA, start=0)
        # every bit: a mid-bit edge to its value 12 cycles after the bit start
        t0 = e[0][0] - 12                                   # first bit is 1: rise at mid-bit
        bits = [(0xA53C >> (15 - i)) & 1 for i in range(16)] + [(0x0FF0 >> (15 - i)) & 1 for i in range(16)]
        for k, b in enumerate(bits):
            self.assertEqual(level_at(e, t0 + 24 * k + 6), 1 - b, f"first half of bit {k}")
            self.assertEqual(level_at(e, t0 + 24 * k + 18), b, f"second half of bit {k}")
        self.assertEqual(q[1], [0xA53C, 0x0FF0])

    @covers("coding.tx.pulse_width", "coding.rx.pulse_symbol", "tick.PULSE", "tick.thr0",
            "cfg.pulse")
    def test_pulse_width_and_symbol(self):
        """WS2812-like: 75-cycle bits, high 24 cycles for a 0 and 48 for a 1
        (the pulse ends after its high time or at the capture tick, whichever
        is first: coding.md 4.2, so PERIOD = 50 > 48); the receiver classifies
        high times against THR0 = 36."""
        tx = """
        SETM  TICK, mode=BAUD
        SETM  CODE, tx=PULSE
        SETM  PERIOD, 50
        SETM  PERIOD2, 25
        SETM  PULSE, high0=24, high1=48
        PIN   out0=0
        WAIT  timeout=2
        FIELD DRIVE 24, HOSTQ
        HALT
"""
        rx = """
        SETM  TICK, mode=PULSE
        SETM  CODE, rx=SYMBOL
        SETM  THR01, thr0=36
        FIELD SAMPLE 24, HOSTQ
        HALT
"""
        ch, q, _ = run_pair({0: tx, 1: rx}, [0x123456])
        e = edges(ch, UO_DATA, start=0)
        highs = [b[0] - a[0] for a, b in zip(e, e[1:]) if a[1] == 1]
        self.assertEqual(highs, [48 if 0x123456 >> (23 - i) & 1 else 24 for i in range(24)])
        rises = [c for c, v in e if v == 1]
        self.assertEqual({y - x for x, y in zip(rises, rises[1:])}, {75})
        self.assertEqual(q[1], [0x123456])

    @covers("coding.tx.pulse_width", "cfg.pulse")
    def test_pulse_width_ends_at_capture_tick(self):
        """coding.md 4.2: the pulse goes low after its high time or at the
        capture tick, whichever is first. With PERIOD 40 a 48-cycle high time
        is cut to 40; the 24-cycle one is kept. (The toolchain's model.py
        keeps 48 here: test_model_features expects 48 with PERIOD 40.)"""
        tx = """
        SETM  TICK, mode=BAUD
        SETM  CODE, tx=PULSE
        SETM  PERIOD, 40
        SETM  PERIOD2, 35
        SETM  PULSE, high0=24, high1=48
        PIN   out0=0
        WAIT  timeout=2
        FIELD DRIVE 8, IMM 0xA5
        HALT
"""
        ch, _, _ = run_pair({0: tx})
        e = edges(ch, UO_DATA, start=0)
        highs = [b[0] - a[0] for a, b in zip(e, e[1:]) if a[1] == 1]
        self.assertEqual(highs, [40 if 0xA5 >> (7 - i) & 1 else 24 for i in range(8)])

    @covers("coding.diff_out", "coding.diff_in", "coding.invert", "coding.data_in")
    def test_differential_and_invert(self):
        """Differential output with invert: out1 is always the complement of
        out0, out0 carries the inverted bits; the receiver takes the pair on
        in2/in3 with differential input and invert, and gets the data back."""
        tx = TX_CLK + """
        SETM  CODE, diff_out, invert
        PIN   out0=1, out1=0
        WAIT  timeout=20
        FIELD DRIVE 8, IMM 0x5A
        FIELD DRIVE 8, IMM 0x81
        HALT
"""
        rx = RX_EDGE + """
        SETM  CODE, data_in=in2, diff_in, invert
        FIELD SAMPLE 8, HOSTQ
        FIELD SAMPLE 8, HOSTQ
        HALT
"""
        # L1: in1 = clock (ui5), in2 = out0 (ui3), in3 = out1 (ui4)
        xin1 = 3 | 5 << 4 | 3 << 8 | 4 << 12
        ch, q, _ = run_pair({0: tx, 1: rx}, xin=(STD_XIN0 & 0xFFFF | xin1 << 16, STD_XIN1))
        d0, d1 = edges(ch, UO_DATA), edges(ch, UO_COMP)
        clk = edges(ch, UO_CLK)
        bits = [level_at(d0, c) for c, v in clk[1:] if v == 1]
        self.assertEqual(bits, [1 - ((w >> (7 - i)) & 1) for w in (0x5A, 0x81) for i in range(8)])
        drive_from, drive_to = clk[1][0], clk[-1][0]
        self.assertTrue(all(row[1] >> UO_DATA & 1 != row[1] >> UO_COMP & 1
                            for row in ch if drive_from <= row[0] <= drive_to))
        self.assertEqual(q[1], [0x5A, 0x81])

    @covers("field.seek", "field.early")
    def test_expect_seek_finds_pattern(self):
        """EXPECT with seek shifts until the last 8 bits equal 0xD5, then the
        next field reads the byte after it (isa 3.1, G7)."""
        tx = TX_CLK + """
        WAIT  timeout=20
        FIELD DRIVE 8, IMM 0x55
        FIELD DRIVE 5, IMM 0x15
        FIELD DRIVE 8, IMM 0xD5
        FIELD DRIVE 8, IMM 0x3C
        HALT
"""
        rx = RX_EDGE + """
        LDI   R0, 0xD5
        FIELD EXPECT 8, seek, R0
        FIELD SAMPLE 8, HOSTQ
        HALT
"""
        _, q, _ = run_pair({0: tx, 1: rx})
        self.assertEqual(q[1], [0x3C])


def single_lane_edges(prog, run=3000, bits=(UO_DATA, UO_CLK)):
    """Lane 0 alone; the (cycle, level) edges of the given uo pins, relative
    to the lane start."""
    with rtlsim.Chip() as chip:
        bench(chip, {0: prog}, wires=False)
        chip.run(100)
        chip.dump()
        chip.start_lane(0, 0)
        t0 = chip.cycle()
        chip.run(run)
        ch = chip.dump(full=True)
    return [[(c - t0, v) for c, v in edges(ch, b)] for b in bits]


@unittest.skipUnless(rtlsim.available(), rtlsim.unavailable_reason())
class TimingUnits(unittest.TestCase):
    UART = """
        SETM  TICK, mode=BAUD{pre}
        SETM  PERIOD, 10
        SETM  PERIOD2, 10
        {delay}
        PIN   out0=1
        FIELD DRIVE 8, IMM 0x5A
        PIN   out0=1
        WAIT  timeout=40
        HALT
"""

    @covers("delay.data", "tick.prescale1")
    def test_launch_delay_on_data(self):
        """THR2L delay = 7: every data edge of a field moves 7 cycles later;
        the PINs before and after the field are not delayed. Edges: initial
        level, PIN rise, 7 field edges of 0x5A, PIN rise, HALT release."""
        plain, _ = single_lane_edges(self.UART.format(pre="", delay="SETM  THR2L, delay=0"))
        late, _ = single_lane_edges(self.UART.format(pre="", delay="SETM  THR2L, delay=7"))
        self.assertEqual(len(plain), 11, plain)
        self.assertEqual(plain[1], late[1])                       # PIN: not delayed
        self.assertEqual(plain[-2], late[-2])
        f_plain = [c for c, _ in plain[2:-2]]
        f_late = [c for c, _ in late[2:-2]]
        self.assertEqual([c + 7 for c in f_plain], f_late)
        self.assertEqual([b - a for a, b in zip(f_plain, f_plain[1:])], [20, 20, 20, 40, 20, 20])

    @covers("delay.prescaled", "tick.prescale8")
    def test_launch_delay_prescaled(self):
        """Prescale 8: a bit is (10 + 10) x 8 = 160 cycles; a delay of 3
        prescaled cycles moves each data edge 3 prescaler strobes later."""
        pre = ", prescale=8"
        plain, _ = single_lane_edges(self.UART.format(pre=pre, delay="SETM CODE, delay_prescaled\n"
                                                      "        SETM  THR2L, delay=0"), run=6000)
        late, _ = single_lane_edges(self.UART.format(pre=pre, delay="SETM CODE, delay_prescaled\n"
                                                     "        SETM  THR2L, delay=3"), run=6000)
        f_plain = [c for c, _ in plain[2:-2]]
        f_late = [c for c, _ in late[2:-2]]
        # the first bit starts with the field, the free-running prescaler
        # (ruling 22) aligns the later launches: whole bits from the second on
        gaps = [b - a for a, b in zip(f_plain, f_plain[1:])]
        self.assertEqual(gaps[1:], [160, 160, 320, 160, 160])
        self.assertTrue(152 < gaps[0] <= 160, gaps)
        # D = 3 prescaler strobes (tick.md 9.2): exactly 24 cycles for the
        # launches on the prescaler grid, 17..24 for the first (field start)
        self.assertEqual([c + 24 for c in f_plain[1:]], f_late[1:])
        self.assertTrue(16 < f_late[0] - f_plain[0] <= 24, (f_plain, f_late))

    @covers("delay.clock", "tick.clock_out")
    def test_launch_delay_on_clock(self):
        """CODE.delay_clock: the delay applies to the clock output instead;
        data edges stay, clock edges move 2 cycles."""
        prog = """
        SETM  TICK, mode=BAUD, clock_out
        SETM  PERIOD, 6
        SETM  PERIOD2, 6
        {delay}
        FIELD DRIVE 4, IMM 0xA
        WAIT  timeout=10
        HALT
"""
        d0, c0 = single_lane_edges(prog.format(delay="SETM THR2L, delay=0\n        SETM CODE, delay_clock"))
        d1, c1 = single_lane_edges(prog.format(delay="SETM THR2L, delay=2\n        SETM CODE, delay_clock"))
        self.assertEqual(d0, d1)
        self.assertTrue(len(c0) >= 8)
        self.assertEqual([(c + 2, v) for c, v in c0[1:8]], c1[1:8])

    @covers("tick.prescale64", "tick.prescale512", "tick.period2")
    def test_prescale_64_and_512(self):
        """Bit time = (PERIOD + PERIOD2) x prescale, with unequal halves. The
        first bit starts with the field and the prescaler runs free (ruling
        22), so only the bits after the first are whole."""
        for pre, want in ((64, (3 + 5) * 64), (512, (1 + 2) * 512)):
            p, p2 = (3, 5) if pre == 64 else (1, 2)
            prog = f"""
        SETM  TICK, mode=BAUD, prescale={pre}
        SETM  PERIOD, {p}
        SETM  PERIOD2, {p2}
        PIN   out0=1
        FIELD DRIVE 4, IMM 0x5
        PIN   out0=1
        HALT
"""
            d, _ = single_lane_edges(prog, run=6 * want + 2000)
            f = [c for c, _ in d[2:6]]                       # the field's edges: 0 1 0 1
            self.assertEqual(len(f), 4, (pre, d))
            self.assertTrue(want - pre < f[1] - f[0] <= want, (pre, d))
            self.assertEqual([b - a for a, b in zip(f[1:], f[2:])], [want] * 2, pre)


@unittest.skipUnless(rtlsim.available(), rtlsim.unavailable_reason())
class Events(unittest.TestCase):

    @covers("events.level", "events.k_window", "events.edge", "events.any", "events.bothlow",
            "events.b_high", "events.b_low", "events.clear", "seq.wait_event", "seq.br_event",
            "seq.wait_timeout", "chan.timestamp", "seq.pushts")
    def test_event_triggers(self):
        """Lane 0 makes pulses with PIN waits (PERIOD 1 = one cycle):
        out0 (in0 of lane 1) low for 10 cycles, then low for 40; then out1
        (in2) rises; then both low together. Lane 1's events, each recorded
        with a timestamp once it fires:
          E0 LEVEL low on in0, K = 20: ignores the 10-cycle pulse, fires on
             the 40-cycle one 20 cycles after it starts;
          E1 ANY edge on in2;  E2 BOTHLOW in0/in2;
          E3 EDGE to high on in0 while in2 (B) is low."""
        tx = """
        SETM  PERIOD, 1
        PIN   out0=1, out1=1
        Q     POP R0               ; the host starts the pulses once lane 1 is armed
        PIN   out0=0, wait=10
        PIN   out0=1, wait=200
        PIN   out0=0, wait=40
        PIN   out0=1, wait=200
        PIN   out1=0, wait=200
        PIN   out1=1, wait=200
        PIN   out0=0, out1=0, wait=100
        PIN   out0=1, wait=100
        PIN   out1=1, wait=100
        HALT
"""
        rx = """
        SETM  PERIOD, 1
        SETM  EVC0, a=in0, trigger=LEVEL, level=LOW, k=20
        SETM  EVC1, a=in2, trigger=ANY
        SETM  EVC2, a=in0, trigger=BOTHLOW, b=in2
        SETM  EVC3, a=in0, trigger=EDGE, level=HIGH, b=in2, b_must=LOW
        EVT   0, e0, on, clear
        EVT   1, e1, on, clear
        EVT   2, e2, on, clear
        EVT   3, e3, on, clear
idle:   WAIT  none
e0:     Q     PUSHI 0, tag=0xE0
        Q     PUSHTS
        EVT   0, e0, on, clear
        BR    always, idle
e1:     Q     PUSHI 1, tag=0xE0
        Q     PUSHTS
        EVT   1, e1, on, clear
        BR    always, idle
e2:     Q     PUSHI 2, tag=0xE0
        Q     PUSHTS
        EVT   2, e2, on, clear
        BR    always, idle
e3:     Q     PUSHI 3, tag=0xE0
        Q     PUSHTS
        EVT   3, e3, on, clear
        BR    always, idle
"""
        # L1: in0 = ui3 (L0 out0), in2 = ui4 (L0 out1). Lane 0 first, so both
        # lines are high before lane 1 arms its events.
        with rtlsim.Chip() as chip:
            bench(chip, {0: tx, 1: rx})
            chip.start_lane(0, ORG[0])
            chip.run(100)
            chip.start_lane(1, ORG[1])
            chip.run(100)
            chip.dump()
            chip.push(0, [1])
            chip.run(3000)
            ch = chip.dump(full=True)
            recs = chip.drain(1)
        kinds = [r & 0xFF for r in recs[0::2]]
        self.assertTrue(all(r >> 24 == 0xE0 for r in recs[0::2]), [hex(r) for r in recs])
        # in order: E0 (the 40-cycle pulse; the 10-cycle one is too short for
        # K = 20), E1 twice (out1 falls, rises); out0 and out1 fall together:
        # E1 and E2 fire in one cycle, E1 (lowest) vectors first and E2 in the
        # next slot, before e1's PUSHI (events are go-to, sequencer.md 5.6),
        # so only E2 is recorded; E0 again 20 cycles later (a new low
        # episode); out0 rises while out1 is low: E3; out1 rises: E1; HALT
        # releases both lines: E1 and E2 again (E2 recorded), then E0 (K).
        self.assertEqual(kinds, [0, 1, 1, 2, 0, 3, 1, 2, 0], kinds)
        d0 = edges(ch, UO_DATA)
        falls = [c for c, v in d0 if v == 0]
        ts = recs[1::2]
        t_e0 = [t for k, t in zip(kinds, ts) if k == 0]
        # the TIME counter and the trace count the same clock: E0 fires
        # 20 cycles (K) after the 40-cycle pulse is seen (3 cycles input
        # latency), up to one slot (5 cycles) plus the PUSHI later
        off = t_e0[0] - falls[1]
        self.assertTrue(3 + 20 <= off <= 3 + 20 + 12, off)

    @covers("events.k_window", "seq.wait_timeout")
    def test_wait_timeout_in_period_units(self):
        """WAIT timeout = 5 with PERIOD 10: about 50 cycles between PINs."""
        prog = """
        SETM  PERIOD, 10
        PIN   out0=1
        WAIT  timeout=5
        PIN   out0=0
        WAIT  timeout=9
        PIN   out0=1
        HALT
"""
        d, _ = single_lane_edges(prog, run=400)
        f = [c for c, _ in d[1:4]]
        # timeouts of 5 and 9 PERIODs: the PIN gaps differ by exactly 4 x 10
        # cycles; the rest is a constant few slots of instruction overhead
        self.assertEqual((f[2] - f[1]) - (f[1] - f[0]), 40, d)
        self.assertTrue(50 < f[1] - f[0] <= 50 + 20, d)


@unittest.skipUnless(rtlsim.available(), rtlsim.unavailable_reason())
class SharedLogic(unittest.TestCase):

    @covers("chan.put_get", "chan.putsr", "chan.br_valid", "chan.host_chstat", "chan.host_chdata",
            "chan.host_clear", "chan.get_waits")
    def test_channels_between_lanes(self):
        """Lane 0 PUTs 0x1234 on C1 after a delay; lane 1 polls BR C1, then
        GETs it (and GET waits for data on C0, which lane 0 fills last).
        Lane 1 also PUTSRs the byte it sampled to C3, and lane 0 leaves
        0xBEEF on C2: the host reads both through CHSTAT/CHDATA and clears
        C2's valid bit by writing 1 to CHSTAT bit 2."""
        tx = TX_CLK + """
        LDI   R0, 0x1234
        WAIT  timeout=30
        Q     PUT C1, R0
        FIELD DRIVE 8, IMM 0xA5
        LDI   R1, 0xBEEF
        Q     PUT C2, R1
        WAIT  timeout=60
        LDI   R2, 0x77
        Q     PUT C0, R2
        HALT
"""
        rx = RX_EDGE + """
poll:   BR    C1, take
        BR    always, poll
take:   Q     GET R2, C1
        Q     PUSH R2, tag=9
        FIELD SAMPLE 8, HOSTQ
        Q     PUTSR C3
        Q     GET R3, C0
        Q     PUSH R3, tag=10
        HALT
"""
        with rtlsim.Chip() as chip:
            bench(chip, {0: tx, 1: rx})
            chip.start_lane(1, ORG[1])
            chip.start_lane(0, ORG[0])
            chip.run(4000)
            st = chip.rd(CHSTAT)
            c2, c3 = chip.rd(CHDATA0 + 2), chip.rd(CHDATA0 + 3)
            chip.wr(CHSTAT, 1 << 2)
            st2 = chip.rd(CHSTAT)
            q1 = chip.drain(1)
        self.assertEqual(q1, [0x09001234, 0xA5, 0x0A000077])
        self.assertEqual(st, 0b1100, "C2 and C3 valid, C0 and C1 taken")
        self.assertEqual(c2, 0xBEEF)
        self.assertEqual(c3 & 0xFF, 0xA5)
        self.assertEqual(st2, 0b1000)

    @covers("mem.queue_full_waits", "mem.queue_empty_waits", "mem.qcfg_sizes", "seq.br_queue",
            "host.irq_queue", "host.irqstat", "host.irqmask", "host.time", "host.id", "host.maddr")
    def test_host_queues_full_and_empty(self):
        """8-word queues (QCFG 0): the lane pushes 12 records, waits with the
        host-out queue full (FLAGS.QF, CTRL data wait) until the host drains,
        then pushes the rest. BR QE sees the empty host-in queue; POP waits
        until the host pushes. IRQ (uo1) follows the out-queue condition when
        IRQMASK enables it. ID and TIME through the host port."""
        prog = """
        BR    QNE, bad
        Q     PUSHI 0xE0, tag=0x51     ; the host-in queue is empty
        LDI   R1, 12
loop:   Q     PUSH R1, tag=0x52
        LOOP  R1, loop
        Q     POP R0                   ; waits for the host
        Q     PUSH R0, tag=0x53
        BR    QF, bad
done:   BR    always, done         ; keep running: a QOUT write to a halted lane
                                   ; also writes out_wr (host.md 4)
bad:    Q     PUSHI 0xBA, tag=0xEE
        HALT
"""
        with rtlsim.Chip() as chip:
            ident = chip.rd(0x00)
            chip.wr(rtlsim.MADDR, 0x123)
            maddr = chip.rd(rtlsim.MADDR)
            bench(chip, {0: prog}, wires=False)
            chip.wr(IRQMASK, 0x10)                       # lane 0 out-queue IRQ
            mask = chip.rd(IRQMASK)
            t1, t2 = chip.rd(TIME), chip.rd(TIME)
            chip.dump()
            chip.start_lane(0, ORG[0], qcfg=0)
            chip.run(500)
            flags = chip.rd(rtlsim.lane_reg(0, rtlsim.FLAGS))
            ctrl = chip.rd(rtlsim.lane_reg(0, rtlsim.CTRL))
            irqstat = chip.rd(IRQSTAT)
            first = chip.drain(0, out_size=8)
            chip.run(500)
            second = chip.drain(0, out_size=8)
            ctrl2 = chip.rd(rtlsim.lane_reg(0, rtlsim.CTRL))
            chip.push(0, [0x4242], in_size=8)
            chip.run(300)
            third = chip.drain(0, out_size=8)
            ctrl3 = chip.rd(rtlsim.lane_reg(0, rtlsim.CTRL))
            chip.run(50)
            ch = chip.dump(full=True)
        self.assertEqual(ident, 0x52530401)
        self.assertEqual(maddr, 0x123)
        self.assertEqual(mask, 0x10)
        self.assertTrue(792 - 5 <= t2 - t1 <= 792 + 5, t2 - t1)   # one read frame apart
        recs = [0x510000E0] + [0x52000000 | n for n in range(12, 0, -1)]
        self.assertEqual(first, recs[:8])
        self.assertTrue(flags >> 8 & 1, "QF while full")
        self.assertEqual(ctrl & 0x201, 0x201, "running, waiting on data")
        self.assertEqual(irqstat >> 4 & 0xF, 1, "lane 0 out queue non-empty")
        self.assertEqual(irqstat >> 8 & 0xF, 1, "lane 0 running")
        self.assertEqual(second, recs[8:])
        self.assertEqual(ctrl2 & 0x201, 0x201, "POP waits on the empty host-in queue")
        self.assertEqual(third, [0x53004242])
        self.assertEqual(ctrl3 & 0x9, 0x1, "running")
        irq = edges(ch, 1)
        self.assertEqual([v for _, v in irq[1:]], [1, 0, 1, 0, 1, 0], irq)

    @covers("seq.single_step", "seq.pause", "seq.reserved_halt", "seq.host_halt", "seq.restart",
            "host.irq_attn", "host.irqstat_clear", "pins.release_on_halt", "seq.alu")
    def test_step_pause_halt_restart(self):
        """Paused at PC 0, each CTRL step executes one instruction (R0 counts
        1, 2, 3); resume runs to the reserved opcode, which halts the lane,
        releases its outputs and raises attention (IRQ with IRQMASK bit 0).
        Writing 1 to IRQSTAT clears it; a new PC and CTRL.run restart the
        lane; a host CTRL.run = 0 halts it again."""
        prog = """
        LDI   R0, 1
        ADD   R0, 1
        ADD   R0, 1
        PIN   out0=1
        .word 0xD0000000
again:  PIN   out0=1
spin:   BR    always, spin
"""
        with rtlsim.Chip() as chip:
            bench(chip, {0: prog}, wires=False)
            chip.wr(IRQMASK, 0x01)
            chip.dump()
            chip.wr(rtlsim.lane_reg(0, rtlsim.QCFG), 0xF)
            chip.wr(rtlsim.lane_reg(0, rtlsim.PC), 0)
            chip.wr(rtlsim.lane_reg(0, rtlsim.CTRL), 0x3)          # run + pause
            chip.run(100)
            r0 = [chip.rd(rtlsim.lane_reg(0, rtlsim.R10)) & 0xFFFF]
            pcs = [chip.rd(rtlsim.lane_reg(0, rtlsim.PC))]
            for _ in range(3):
                chip.wr(rtlsim.lane_reg(0, rtlsim.CTRL), 0x7)      # one step
                chip.run(100)
                r0.append(chip.rd(rtlsim.lane_reg(0, rtlsim.R10)) & 0xFFFF)
                pcs.append(chip.rd(rtlsim.lane_reg(0, rtlsim.PC)))
            chip.wr(rtlsim.lane_reg(0, rtlsim.CTRL), 0x1)          # resume
            chip.run(100)
            halted = chip.rd(rtlsim.lane_reg(0, rtlsim.CTRL))
            pc_h = chip.rd(rtlsim.lane_reg(0, rtlsim.PC))
            irqstat = chip.rd(IRQSTAT)
            chip.wr(IRQSTAT, 0x1)
            irqstat2 = chip.rd(IRQSTAT)
            chip.wr(rtlsim.lane_reg(0, rtlsim.PC), 5)
            chip.wr(rtlsim.lane_reg(0, rtlsim.CTRL), 0x1)          # restart at `again`
            chip.run(100)
            running = chip.rd(rtlsim.lane_reg(0, rtlsim.CTRL))
            chip.wr(rtlsim.lane_reg(0, rtlsim.CTRL), 0x0)          # host halt
            chip.run(20)
            stopped = chip.rd(rtlsim.lane_reg(0, rtlsim.CTRL))
            ch = chip.dump(full=True)
        self.assertEqual(r0, [0, 1, 2, 3])
        self.assertEqual(pcs, [0, 1, 2, 3])
        self.assertEqual(halted & 0x9, 0x8, "halted with attention")
        self.assertEqual(pc_h, 4, "PC stays at the halting word")
        self.assertEqual(irqstat & 0xF, 1)
        self.assertEqual(irqstat2 & 0xF, 0)
        self.assertEqual(running & 0x1, 1)
        self.assertEqual(stopped & 0x9, 0, "host halt: no attention")
        out0 = [v for _, v in edges(ch, UO_DATA)[1:]]
        self.assertEqual(out0, [1, 0, 1, 0], "PIN, halt release, restart PIN, host-halt release")
        irq = [v for _, v in edges(ch, 1)[1:]]
        self.assertEqual(irq, [1, 0], "attention IRQ until IRQSTAT is cleared")

    @covers("seq.abort", "events.vector", "events.edge", "field.abandon")
    def test_event_abandons_a_field(self):
        """Lane 1 waits in a 32-bit SAMPLE for a clock that never comes; an
        edge event on its data input abandons the field (isa 3.6) and its
        handler runs: the field is no longer busy and its result never
        arrives."""
        tx = """
        SETM  PERIOD, 1
        PIN   out0=1
        Q     POP R0
        PIN   out0=0
        HALT
"""
        rx = RX_EDGE + """
        SETM  EVC0, a=in0, trigger=EDGE, level=LOW
        EVT   0, ev, on, clear
        FIELD SAMPLE 32, HOSTQ
        MOVSR R0                   ; needs the field's result: no run-ahead past it
        Q     PUSHI 0xBA, tag=0xEE
        HALT
ev:     Q     PUSHI 0x0E, tag=0xE0
        HALT
"""
        with rtlsim.Chip() as chip:
            bench(chip, {0: tx, 1: rx})
            chip.start_lane(0, ORG[0])
            chip.run(50)
            chip.start_lane(1, ORG[1])
            chip.run(100)
            busy = chip.rd(rtlsim.lane_reg(1, rtlsim.CTRL))
            chip.push(0, [1])
            chip.run(500)
            q1 = chip.drain(1)
            done = chip.rd(rtlsim.lane_reg(1, rtlsim.CTRL))
        self.assertEqual(busy & 0x11, 0x11, "running, field busy")
        self.assertEqual(q1, [0xE000000E])
        self.assertEqual(done & 0x19, 0x08)

    @covers("seq.alu", "seq.alu_flags", "seq.mem_ld_st", "seq.loop", "seq.movsr", "mem.data_lane",
            "field.src_mem")
    def test_alu_and_memory(self):
        """ALU results and flags, 16-bit loads and stores in the lane's data
        region, MEM as a field source, MOVSRH, LOOP (isa 3.5, model test)."""
        prog = TX_CLK + """
        LDI   R0, 0xFFFF
        ADD   R0, 1            ; 0, Z
        BR    NZ, bad
        Q     PUSH R0, tag=1
        LDI   R1, 5
        SUB   R1, 7            ; 0xFFFE, borrow
        BR    NN, bad
        Q     PUSH R1, tag=2
        LDI   R2, 0x00F0
        SHL   R2, 4            ; 0x0F00
        OR    R2, 0x000A
        XOR   R2, 0x0F0F       ; 0x0005
        Q     PUSH R2, tag=3
        LDI   R0, 0xFFF0
        ADDC  R0, 0x0020       ; 0x10010 -> 0x0011
        Q     PUSH R0, tag=4
        CMP   R0, 0x11
        BR    NZ, bad
        LDI   R3, 10
        ST    R0, [R3+]
        ST    R2, [R3], hi
        LD    R1, [10]
        Q     PUSH R1, tag=5
        LDI   R3, 11
        FIELD DRIVE 32, MEM    ; source MEM: word 11, then R3 = 12
        MOVSRH R2
        Q     PUSH R2, tag=6
        Q     PUSH R3, tag=7
        LDI   R1, 3
loop:   LOOP  R1, loop
        Q     PUSH R1, tag=8
        AND   R2, 0
        INC   R2
        DEC   R2
        DEC   R2
        SHR   R2, 12
        Q     PUSH R2, tag=9
        HALT
bad:    Q     PUSHI 0xBA, tag=0xEE
        HALT
"""
        _, q, _ = run_pair({0: prog}, run=3000, qcfg=0x5)          # 16-word queues: words 10, 11 free
        self.assertEqual(q[0], [0x01000000, 0x0200FFFE, 0x03000005, 0x04000011,
                                0x05000011, 0x06000005, 0x0700000C, 0x08000000, 0x0900000F])

    @covers("field.raw_drive", "field.raw_sample", "tick.edge_interval")
    def test_raw_drive_and_sample(self):
        """RAW DRIVE holds out0 for 100, 50, 30 cycles; RAW SAMPLE on the
        other lane measures the time between input edges."""
        tx = """
        PIN   out0=1
        Q     POP R0
        RAW   DRIVE 0, IMM 100
        RAW   DRIVE 1, IMM 50
        RAW   DRIVE 0, IMM 30
        PIN   out0=1
        HALT
"""
        rx = """
        RAW   SAMPLE HOSTQ, tag=1
        RAW   SAMPLE HOSTQ, tag=1
        RAW   SAMPLE HOSTQ, tag=1
        HALT
"""
        with rtlsim.Chip() as chip:
            bench(chip, {0: tx, 1: rx})
            chip.start_lane(0, ORG[0])
            chip.run(50)
            chip.start_lane(1, ORG[1])
            chip.run(50)
            chip.dump()
            chip.push(0, [1])
            chip.run(600)
            ch = chip.dump(full=True)
            q1 = chip.drain(1)
        e = edges(ch, UO_DATA)
        self.assertEqual([b[0] - a[0] for a, b in zip(e[1:], e[2:])][:3], [100, 50, 30])
        self.assertEqual(q1[1:], [0x01000000 | 100, 0x01000000 | 50])
        self.assertEqual(q1[0] >> 24, 1)

    @covers("xbar.loopback", "xbar.uio_out", "xbar.uio_oe", "coding.data_od", "xbar.outsel",
            "xbar.insel")
    def test_crossbar_internal_loopback(self):
        """XCTL loopback: lane inputs read the chip's own outputs (no board
        wires); the CRC-16 round trip runs through it. Lane 0's data leaves
        open-drain on uio0 (uio_oe follows the bits, uio_out stays 0) and
        lane 1 reads it back through the loopback's uio twin (xbar.md)."""
        cfg = "        SETM  CRCPOLY, 0x1021\n        SETM  CRCCFG, width=16, init_ones\n"
        tx = TX_CLK + cfg + """
        SETM  CODE, data_od
        PIN   out0=z
        WAIT  timeout=20
        LDI   R1, 4
loop:   FIELD DRIVE 8, crc, HOSTQ
        LOOP  R1, loop
        FIELD DRIVE 16, CRC
        HALT
"""
        rx = RX_EDGE + cfg + """
        LDI   R1, 4
loop:   FIELD SAMPLE 8, crc, HOSTQ
        LOOP  R1, loop
        FIELD EXPECT 16, CRC, HOSTQ
        BR    NM, bad
        Q     PUSHI 0x60, tag=0xEE
bad:    HALT
"""
        # L1 in0 = code 8, in1 = code 4: in loopback, the uio0 twin (L0 out0,
        # open drain: a released pin reads 1) and uo4 (L0 clock)
        xin0 = (STD_XIN0 & 0xFFFF) | (8 | 4 << 4) << 16

        def pre(chip):
            chip.wr(rtlsim.XCTL, 1)
            chip.wr(rtlsim.XOUT1, osel(0, 0))                          # uio0 <- L0 out0
        with rtlsim.Chip() as chip:
            bench(chip, {0: tx, 1: rx}, xin=(xin0, STD_XIN1), wires=False)
            pre(chip)
            xctl = chip.rd(rtlsim.XCTL)
            chip.start_lane(1, ORG[1])
            chip.start_lane(0, ORG[0])
            chip.push(0, list(b"Rose"))
            chip.run(3000)
            ch = chip.dump(full=True)
            q1 = chip.drain(1)
        self.assertEqual(xctl, 1)
        crc = crc_ref(bytes_bits(b"Rose", False), 16, 0x1021, 1, 0)
        self.assertEqual(q1, list(b"Rose") + [crc, 0xEE000060])
        self.assertFalse(any(row[3] & 1 for row in ch), "open drain: uio_out[0] never 1")
        oe = edges(ch, 0, col=4)
        self.assertTrue(len(oe) > 10, "uio_oe[0] carries the bits")
        self.assertEqual(edges(ch, 0, col=5)[-1][1], 1, "released: the pin reads 1")

    @covers("mem.prog_full", "mem.data_full", "host.mdata_autoinc")
    def test_memories_full_range(self):
        """All 512 program words and all 512 data words through MADDR/MDATA
        (auto-increment), read back."""
        prog = [(0x9E3779B1 * (i + 1)) & 0xFFFFFFFF for i in range(512)]
        data = [(0x7F4A7C15 * (i + 3)) & 0xFFFFFFFF for i in range(512)]
        with rtlsim.Chip() as chip:
            chip.load_program(prog)
            chip.wr(rtlsim.MADDR, 0x200)
            for w in data:
                chip.wr(rtlsim.MDATA, w)
            chip.wr(rtlsim.MADDR, 0x200)
            back = [chip.rd(rtlsim.MDATA) for _ in data]
            chip.wr(rtlsim.MADDR, 0)
            back_p = [chip.rd(rtlsim.MDATA) for _ in range(8)]
            maddr = chip.rd(rtlsim.MADDR)
        self.assertEqual(back, data)
        self.assertEqual(back_p, prog[:8])
        self.assertEqual(maddr, 8)


if __name__ == "__main__":
    unittest.main()
