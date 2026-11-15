"""Model features on the pins: check unit, stuffing, coding, pulse width,
launch delay, timeouts, events, ALU, memory, seek, RAW, halt, channels and
an I2C controller. A transmitting lane A (BAUD, clock output on out3)
feeds a receiving lane B (EDGE on in1) where two lanes are needed."""
import unittest

import support  # noqa: F401
from benches import I2cTargetBench, uart_decode
from rosetta import asm, model, wave

TX_HEAD = """
        SETM  TICK, mode=BAUD, clock_out
        SETM  PERIOD, 4
        SETM  PERIOD2, 4
        WAIT  timeout=25           ; let the receiver get ready
"""
RX_HEAD = """
        SETM  TICK, mode=EDGE, clock_in=in1
"""


def lane(text: str, index: int = 0, channels=None) -> model.Lane:
    ln = model.Lane(asm.assemble(text).words, lane=index, channels=channels)
    ln.start(0)
    return ln


def run_pair(a: model.Lane, b: model.Lane, cycles: int):
    """A.out0 -> B.in0 (data), A.out3 -> B.in1 (clock). Returns A's trace."""
    tr = model.Trace()
    va = ea = 0
    for _ in range(cycles):
        data = va & 1 if ea & 1 else 1
        clk = va >> 3 & 1 if ea >> 3 & 1 else 0
        b.step(0b1100 | clk << 1 | data)
        pins = 0b1111
        va, ea = a.step(pins)
        tr.inputs.append(pins)
        tr.out_val.append(va)
        tr.out_en.append(ea)
    return tr


def bits_at_rising_clock(tr: model.Trace) -> list[int]:
    """out0 sampled at every rising edge of out3 (independent decoding)."""
    clk, dat = tr.out(3), tr.out(0)
    return [1 if dat[c] != "0" else 0 for c, o, n in wave.edges(clk) if (o, n) == ("0", "1")]


def to_int(bits: list[int]) -> int:
    v = 0
    for b in bits:
        v = v << 1 | b
    return v


class CheckUnit(unittest.TestCase):
    def test_crc16_ccitt_false_known_answer_on_the_wire_and_expect(self):
        cfg = """
        SETM  CRCPOLY, 0x1021
        SETM  CRCCFG, width=16, init_ones
        CRCINIT
        LDI   R1, 9
"""
        a = lane(TX_HEAD + cfg + """
loop:   FIELD DRIVE 8, crc, HOSTQ
        LOOP  R1, loop
        FIELD DRIVE 16, CRC
        HALT
""")
        b = lane(RX_HEAD + cfg + """
loop:   FIELD SAMPLE 8, early, crc, HOSTQ
        LOOP  R1, loop
        FIELD EXPECT 16, CRC
        BR    M, good
        Q     PUSHI 0xBA, tag=0xEE
        HALT
good:   Q     PUSHI 0x60, tag=0xEE
        HALT
""", 1)
        a.host_write(*b"123456789")
        tr = run_pair(a, b, 2000)
        bits = bits_at_rising_clock(tr)
        self.assertEqual(len(bits), 9 * 8 + 16)
        self.assertEqual(bytes(to_int(bits[i:i + 8]) for i in range(0, 72, 8)), b"123456789")
        self.assertEqual(to_int(bits[72:]), 0x29B1)            # docs/blocks/crc.md table
        self.assertEqual(b.host_read_all(), list(b"123456789") + [0xEE000060])
        self.assertEqual(a.halt_reason, "HALT")


class Stuffing(unittest.TestCase):
    STUFF = "        SETM  STUFF, n=6, mode=ONES, enable, violation_event\n"

    def test_ones_stuffing_round_trip(self):
        a = lane(TX_HEAD + self.STUFF + """
        FIELD DRIVE 8, stuff, IMM 0xFF
        FIELD DRIVE 8, stuff, IMM 0xFF
        FIELD DRIVE 4, stuff, IMM 0x7
        HALT
""")
        b = lane(RX_HEAD + self.STUFF + """
        FIELD SAMPLE 8, early, stuff, HOSTQ
        FIELD SAMPLE 8, stuff, HOSTQ
        FIELD SAMPLE 4, stuff, HOSTQ
        HALT
""", 1)
        tr = run_pair(a, b, 1000)
        wire = bits_at_rising_clock(tr)
        # 1x6, stuff 0, 1x6, stuff 0, 1111, 0111 -> after 4+3 ones no stuff
        self.assertEqual(wire, [1] * 6 + [0] + [1] * 6 + [0] + [1] * 4 + [0, 1, 1, 1])
        self.assertEqual(b.host_read_all(), [0xFF, 0xFF, 0x7])

    def test_violation_raises_event_3(self):
        a = lane(TX_HEAD + """
        FIELD DRIVE 8, IMM 0xFF
        HALT
""")
        b = lane(RX_HEAD + self.STUFF + """
        EVT   3, bad, on
        FIELD SAMPLE 8, early, stuff, HOSTQ
        HALT
bad:    Q     PUSHI 3, tag=0xEE
        HALT
""", 1)
        run_pair(a, b, 1000)
        self.assertEqual(b.host_read_all(), [0xEE000003])


class Coding(unittest.TestCase):
    def test_nrzi_round_trip(self):
        code = "        SETM  CODE, tx=NRZI, rx=NRZI\n"
        a = lane(TX_HEAD + code + "        PIN out0=1\n        FIELD DRIVE 8, IMM 0x00\n"
                 "        FIELD DRIVE 8, IMM 0xA5\n        HALT\n")
        b = lane(RX_HEAD + code + "        FIELD SAMPLE 8, early, HOSTQ\n"
                 "        FIELD SAMPLE 8, HOSTQ\n        HALT\n", 1)
        tr = run_pair(a, b, 1000)
        wire = bits_at_rising_clock(tr)
        self.assertEqual(wire[:8], [0, 1, 0, 1, 0, 1, 0, 1])   # every 0 toggles the line
        self.assertEqual(b.host_read_all(), [0x00, 0xA5])

    def test_pulse_width_transmit_and_pulse_receive(self):
        """WS2812-like: 75-cycle bits, high 24 for a 0 and 48 for a 1;
        the receiver classifies high times against THR0 = 36."""
        a = lane("""
        SETM  TICK, mode=BAUD
        SETM  CODE, tx=PULSE
        SETM  PERIOD, 40
        SETM  PERIOD2, 35
        SETM  PULSE, high0=24, high1=48
        PIN   out0=0
        WAIT  timeout=2
        FIELD DRIVE 24, HOSTQ
        HALT
""")
        b = lane("""
        SETM  TICK, mode=PULSE
        SETM  CODE, rx=SYMBOL
        SETM  THR01, thr0=36
        FIELD SAMPLE 24, HOSTQ
        HALT
""", 1)
        a.host_write(0x123456)
        tr = model.Trace()
        va = ea = 0
        for _ in range(2200):
            b.step(0b1110 | (va & 1 if ea & 1 else 0))
            va, ea = a.step(0b1111)
            tr.out_val.append(va)
            tr.out_en.append(ea)
            tr.inputs.append(0)
        e = wave.edges(tr.out(0))
        highs = [b2 - a2 for (a2, _, n1), (b2, _, _) in zip(e, e[1:]) if n1 == "1"]
        self.assertEqual(highs, [48 if 0x123456 >> (23 - i) & 1 else 24 for i in range(24)])
        rises = [c for c, o, n in e if n == "1"]
        self.assertEqual(set(y - x for x, y in zip(rises, rises[1:])), {75})
        self.assertEqual(b.host_read_all(), [0x123456])


class LaunchDelay(unittest.TestCase):
    UART = """
        SETM  TICK, mode=BAUD
        SETM  PERIOD, 10
        SETM  PERIOD2, 10
        {delay}
        PIN   out0=1
        FIELD DRIVE 8, IMM 0x5A
        HALT
"""

    def edges_with(self, delay: str):
        tr = model.run(lane(self.UART.format(delay=delay)), 400)
        return [c for c, o, n in wave.edges(tr.out(0)) if o != "z"]

    def test_data_delay_shifts_every_data_edge(self):
        plain = self.edges_with("SETM  THR2L, delay=0")
        late = self.edges_with("SETM  THR2L, delay=7")
        self.assertEqual([x + 7 for x in plain[:-1]], late[:-1])   # last edge: HALT releases

    def test_prescaled_delay(self):
        head = "SETM  TICK, mode=BAUD, prescale=8\n        SETM CODE, delay_prescaled\n        "
        plain = self.edges_with(head + "SETM  THR2L, delay=0")
        late = self.edges_with(head + "SETM  THR2L, delay=3")
        self.assertEqual(plain[1] - plain[0], 160)                  # (10 + 10) x 8
        self.assertEqual([x + 24 for x in plain[:-1]], late[:-1])   # 3 prescaled cycles

    def test_delay_on_clock_instead_of_data(self):
        prog = """
        SETM  TICK, mode=BAUD, clock_out
        SETM  PERIOD, 6
        SETM  PERIOD2, 6
        {delay}
        FIELD DRIVE 4, IMM 0xA
        HALT
"""
        t0 = model.run(lane(prog.format(delay="SETM THR2L, delay=0\n        SETM CODE, delay_clock")), 200)
        t1 = model.run(lane(prog.format(delay="SETM THR2L, delay=2\n        SETM CODE, delay_clock")), 200)
        self.assertEqual(t0.out(0)[:60], t1.out(0)[:60])             # data unchanged
        c0 = [c for c, o, n in wave.edges(t0.out(3)) if o != "z"]
        c1 = [c for c, o, n in wave.edges(t1.out(3)) if o != "z"]
        self.assertEqual([c + 2 for c in c0[:6]], c1[:6])


class TimingAndEvents(unittest.TestCase):
    def test_wait_timeout_in_period_units(self):
        tr = model.run(lane("""
        SETM  PERIOD, 10
        PIN   out0=0
        WAIT  timeout=5
        PIN   out0=1
        HALT
"""), 200)
        e = [c for c, o, n in wave.edges(tr.out(0))]
        self.assertTrue(50 <= e[1] - e[0] <= 60, e)

    def test_event_window_k_rejects_short_pulse(self):
        prog = lane("""
        SETM  EVC0, a=in0, trigger=LEVEL, level=LOW, k=20
        PIN   out0=0
        EVT   0, fired, on
idle:   WAIT  none
fired:  PIN   out0=1
        HALT
""")
        pins = [1] * 100 + [0] * 15 + [1] * 100 + [0] * 40 + [1] * 100
        tr = model.run(prog, len(pins), bench=lambda t, v, e: 0b1110 | pins[t])
        rise = next(c for c, o, n in wave.edges(tr.out(0)) if n == "1")
        start = 215                                   # second low pulse on the pin
        # input delay 3, window 20, then the next execute slot (+0..5) and PIN
        self.assertTrue(start + 3 + 20 <= rise <= start + 3 + 20 + 6, rise)

    def test_wait_on_event_and_pending_flag(self):
        tr = model.run(lane("""
        SETM  EVC2, a=in2, trigger=EDGE, level=HIGH
        PIN   out0=0
        WAIT  E2
        PIN   out0=1
        BR    E2, still
        HALT
still:  Q     PUSHI 1, tag=0xEE     ; WAIT does not clear pending (D23)
        HALT
"""), 100, bench=lambda t, v, e: 0b1011 if t < 40 else 0b1111)
        rise = next(c for c, o, n in wave.edges(tr.out(0)) if n == "1")
        self.assertTrue(43 <= rise <= 58, rise)   # edge seen at 43, then WAIT and PIN slots


class Arithmetic(unittest.TestCase):
    def test_alu_flags_and_memory(self):
        ln = lane("""
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
        HALT
bad:    Q     PUSHI 0xBA, tag=0xEE
        HALT
""")
        model.run(ln, 600)
        self.assertEqual(ln.host_read_all(), [0x01000000, 0x0200FFFE, 0x03000005, 0x04000011,
                                              0x05000011, 0x06000005, 0x0700000C, 0x08000000])


class SeekAndRaw(unittest.TestCase):
    def test_expect_seek_finds_the_start_pattern(self):
        a = lane(TX_HEAD + """
        FIELD DRIVE 8, IMM 0x55
        FIELD DRIVE 5, IMM 0x15
        FIELD DRIVE 8, IMM 0xD5
        FIELD DRIVE 8, IMM 0x3C
        HALT
""")
        b = lane(RX_HEAD + """
        LDI   R0, 0xD5
        FIELD EXPECT 8, early, seek, R0
        FIELD SAMPLE 8, HOSTQ
        HALT
""", 1)
        run_pair(a, b, 1000)
        self.assertEqual(b.host_read_all(), [0x3C])

    def test_raw_drive_and_sample(self):
        a = lane("""
        PIN   out0=1
        RAW   DRIVE 0, IMM 100
        RAW   DRIVE 1, IMM 50
        RAW   DRIVE 0, IMM 30
        PIN   out0=1
        HALT
""")
        b = lane("""
        RAW   SAMPLE HOSTQ, tag=1
        RAW   SAMPLE HOSTQ, tag=1
        RAW   SAMPLE HOSTQ, tag=1
        HALT
""", 1)
        va = ea = 0
        for _ in range(400):
            b.step(0b1110 | (va & 1 if ea & 1 else 1))
            va, ea = a.step(0b1111)
        recs = b.host_read_all()
        self.assertEqual(recs[1:], [0x01000000 | 100, 0x01000000 | 50])
        self.assertEqual(recs[0] >> 24, 1)


class HaltAndChannels(unittest.TestCase):
    def test_reserved_opcode_halts_and_releases(self):
        tr = model.run(lane("PIN out0=1, out1=0\n.word 0xD0000000\n"), 30)
        self.assertEqual(tr.out(0)[4], "1")      # PIN at cycle 2, reserved word at 7
        self.assertEqual(tr.out(0)[-1], "z")
        self.assertEqual(tr.out(1)[-1], "z")

    def test_halt_waits_for_the_end_of_the_stop_bit(self):
        ln = lane("""
        SETM  TICK, mode=BAUD
        SETM  PERIOD, 50
        SETM  PERIOD2, 50
        PIN   out0=1
        FIELD DRIVE 1, IMM 0
        FIELD DRIVE 1, IMM 1
        HALT
""")
        tr = model.run(ln, 400)
        e = [c for c, o, n in wave.edges(tr.out(0))]
        self.assertEqual(e[2] - e[1], 100)     # start bit
        self.assertEqual(e[3] - e[2], 100)     # full stop bit before HALT releases
        self.assertEqual(tr.out(0)[-1], "z")

    def test_channel_between_lanes(self):
        ch = model.Channels()
        a = lane("SETM PERIOD, 1\nLDI R0, 0x1234\nWAIT timeout=40\nQ PUT C1, R0\nHALT\n", 0, ch)
        b = lane("Q GET R2, C1\nQ PUSH R2, tag=9\nHALT\n", 1, ch)      # waits for the PUT
        for _ in range(200):
            a.step(0xF)
            b.step(0xF)
        self.assertEqual(b.host_read_all(), [0x09001234])
        self.assertFalse(ch.valid[1])


class I2cControllerRole(unittest.TestCase):
    PROG = """
        SETM  TICK, mode=BAUD, clock_in=in1, clock_out, clock_idle, clock_od, rest, stretch_honour
        SETM  CODE, data_in=in0, data_od
        SETM  PERIOD, 78           ; SCL low 1.3 us
        SETM  PERIOD2, 36          ; SCL high 0.6 us: 400 kHz with unequal halves
        PIN   out0=z, out3=z
        Q     POP R0
        PIN   out0=0, wait=1       ; START, then t_HD;STA
        PIN   out3=0, wait=1
        FIELD DRIVE 8, R0
        FIELD EXPECT 1, IMM 0      ; ACK
        BR    NM, nack
        Q     PUSHI 1, tag=0xA0
        BR    always, stop
nack:   Q     PUSHI 0, tag=0xA0
stop:   PIN   out0=0, wait=1       ; starts at the end of the ACK bit, SCL low (rest)
        PIN   out3=z, wait=1
        PIN   out0=z, wait=1       ; STOP
        HALT
"""

    def transfer(self, address: int, stretch: int):
        ln = lane(self.PROG)
        ln.host_write(0x50 << 1)
        tgt = I2cTargetBench(address, stretch=stretch)
        tr = model.run(ln, 3000, bench=tgt)
        return ln, tgt, tr

    def test_address_ack_unequal_halves(self):
        ln, tgt, _ = self.transfer(0x50, 0)
        self.assertEqual(tgt.byte, 0xA0)
        self.assertEqual(ln.host_read_all(), [0xA0000001])
        self.assertTrue(all(h >= 36 for h in tgt.high_times[:9]), tgt.high_times)
        self.assertTrue(all(lo >= 78 for lo in tgt.low_times[:9]), tgt.low_times)

    def test_nack_from_other_address(self):
        ln, tgt, _ = self.transfer(0x51, 0)
        self.assertEqual(ln.host_read_all(), [0xA0000000])

    def test_honours_clock_stretching(self):
        ln, tgt, _ = self.transfer(0x50, 200)
        self.assertEqual(tgt.byte, 0xA0)
        self.assertEqual(ln.host_read_all(), [0xA0000001])
        self.assertTrue(all(lo >= 200 for lo in tgt.low_times[1:8]), tgt.low_times)
        self.assertTrue(all(h >= 36 for h in tgt.high_times[:9]), tgt.high_times)


if __name__ == "__main__":
    unittest.main()
