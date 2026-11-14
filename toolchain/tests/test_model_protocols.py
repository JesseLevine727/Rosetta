"""Protocol checks on the model's pins, decoded independently (benches.py).

They validate the model and the instruction set, not the chip."""
import unittest

import support  # noqa: F401
from benches import CLOCK_HZ, I2cController, SpiTarget, uart_decode, uart_levels
from rosetta import asm, model, wave

PROGRAMS = support.readme_programs()
BAUD_RATE = 115200


def lane_for(name: str, lane_index: int = 0, **subst: str) -> model.Lane:
    text = PROGRAMS[name]
    for old, new in subst.items():
        text = text.replace(old, new)
    lane = model.Lane(asm.assemble(text).words, lane=lane_index)
    lane.start(0)
    return lane


class UartTransmit(unittest.TestCase):
    DATA = [0x48, 0x69, 0x00, 0xFF, 0x55, 0xA3]

    def run_tx(self, lane_index=0):
        lane = lane_for("uart_tx", lane_index)
        lane.host_write(*self.DATA)
        tr = model.run(lane, len(self.DATA) * 10 * 521 + 3000)
        return lane, tr

    def test_valid_8n1(self):
        _, tr = self.run_tx()
        data, starts = uart_decode(tr.out(0), BAUD_RATE)
        self.assertEqual(data, self.DATA)
        # bytes follow each other with no gap: start edges 10 bit times apart
        for a, b in zip(starts, starts[1:]):
            self.assertAlmostEqual((b - a) / (CLOCK_HZ / BAUD_RATE), 10, delta=10 * 0.02)

    def test_bit_times_within_2_percent(self):
        _, tr = self.run_tx()
        nominal = CLOCK_HZ / BAUD_RATE
        e = [c for c, old, new in wave.edges(tr.out(0)) if old != "z"]
        self.assertGreater(len(e), 20)
        for a, b in zip(e, e[1:]):
            bits = round((b - a) / nominal)
            if bits > 10:
                continue                          # idle after the last byte
            self.assertGreaterEqual(bits, 1)
            per_bit = (b - a) / bits
            self.assertLess(abs(per_bit - nominal) / nominal, 0.02, f"edge {a}->{b}")

    def test_idle_high_before_and_after(self):
        lane, tr = self.run_tx()
        out = tr.out(0)
        first_drive = out.index("1")
        self.assertEqual(set(out[first_drive:first_drive + 5]), {"1"})
        self.assertEqual(out[-1], "1")
        self.assertTrue(lane.running)

    def test_lane_index_only_shifts_timing(self):
        """Non-interference flavour: the same program on lane 3 gives the
        same waveform, shifted by the difference of the slot phases
        (lane 3 executes at cycles 0, 5, ...; lane 0 at 2, 7, ...)."""
        _, t0 = self.run_tx(0)
        _, t3 = self.run_tx(3)
        a, b = t0.out(0), t3.out(0)
        self.assertEqual(wave.edges(a)[0][0] - wave.edges(b)[0][0], 2)
        self.assertTrue(a[2:] == b[:-2])


class UartReceive(unittest.TestCase):
    DATA = [0x00, 0xFF, 0x55, 0xA3, 0x81, 0x7E]

    def receive(self, name, rate_factor, glitch=False, data=None):
        data = data or self.DATA
        levels = uart_levels(data, CLOCK_HZ / BAUD_RATE * rate_factor, idle_before=1500)
        if glitch:                               # 1 us low pulse well before the first byte
            for i in range(400, 460):
                levels[i] = 0
        lane = lane_for(name)
        model.run(lane, len(levels), bench=lambda t, v, e: 0b1110 | levels[t])
        return lane, lane.host_read_all()

    def test_rates_minus_2_nominal_plus_2_percent(self):
        for factor in (1 / 0.98, 1.0, 1 / 1.02):     # sender 2 % slow, exact, 2 % fast
            with self.subTest(rate=f"{1 / factor:.2f}"):
                _, recs = self.receive("uart_rx", factor)
                self.assertEqual(recs, [0x0100_0000 | b for b in self.DATA])

    def test_start_bit_glitch_is_ignored(self):
        lane, recs = self.receive("uart_rx", 1.0, glitch=True)
        self.assertEqual(recs, [0x0100_0000 | b for b in self.DATA])

    def test_framing_error_record(self):
        bit = CLOCK_HZ / BAUD_RATE
        levels = uart_levels([0x5A], bit, idle_before=800)
        stop = int(round(800 + 9 * bit))
        for i in range(stop, stop + int(bit)):
            levels[i] = 0                       # stop bit low
        lane = lane_for("uart_rx")
        model.run(lane, len(levels), bench=lambda t, v, e: 0b1110 | levels[t])
        self.assertEqual(lane.host_read_all()[:2], [0x0100005A, 0xFE00_0000])

    def test_listener_records_timestamps_and_never_drives(self):
        levels = uart_levels([0x48, 0x69], CLOCK_HZ / BAUD_RATE, idle_before=1000)
        lane = lane_for("uart_listener")
        tr = model.run(lane, len(levels), bench=lambda t, v, e: 0b1110 | levels[t])
        recs = lane.host_read_all()
        self.assertEqual(recs[1::2], [0x0100_0048, 0x0100_0069])
        dt_us = (recs[2] - recs[0]) / CLOCK_HZ * 1e6
        self.assertAlmostEqual(dt_us, 86.8, delta=0.5)   # language.md 2.1 host view
        self.assertEqual(set(tr.out_en), {0})

    def test_transmit_to_receive_loopback(self):
        """Lane 0 runs the transmitter, lane 1 the receiver, on one wire."""
        tx, rx = lane_for("uart_tx", 0), lane_for("uart_rx", 1)
        tx.host_write(0x31, 0x32, 0xC0)
        val = en = 0
        for _ in range(35 * 521):
            line = (val & 1) if en & 1 else 1
            val, en = tx.step(0b1111)
            rx.step(0b1110 | line)
        self.assertEqual(rx.host_read_all(), [0x0100_0031, 0x0100_0032, 0x0100_00C0])


class SpiController(unittest.TestCase):
    def transfer(self, words, replies, period="4", period2="2"):
        lane = lane_for("spi_controller", **{"PERIOD, 4 ": f"PERIOD, {period} ",
                                             "PERIOD2, 2 ": f"PERIOD2, {period2} "})
        lane.host_write(*words)
        target = SpiTarget(replies)
        tr = model.run(lane, 60 * len(words) + 200, bench=target)
        return lane, target, tr

    def test_mode0_words_both_directions(self):
        lane, target, tr = self.transfer([0x9F, 0x00, 0xA5], [0xC3, 0x5A, 0x0F])
        self.assertEqual(target.received, [0x9F, 0x00, 0xA5])
        self.assertEqual(lane.host_read_all(), [0xC3, 0x5A, 0x0F])

    def test_mode0_pins(self):
        _, target, tr = self.transfer([0x81], [0x00])
        sck, cs = tr.out(3), tr.out(2)
        self.assertEqual(target.rising_edges, 8)
        rises = [c for c, o, n in wave.edges(sck) if (o, n) == ("0", "1")]
        self.assertEqual([b - a for a, b in zip(rises, rises[1:])], [6] * 7)   # 10 MHz
        fall_cs = next(c for c, o, n in wave.edges(cs) if n == "0")
        rise_cs = next(c for c, o, n in wave.edges(cs) if (o, n) == ("0", "1"))
        self.assertTrue(fall_cs < rises[0] and rises[-1] < rise_cs)
        self.assertTrue(all(sck[c] == "0" for c in range(len(sck)) if cs[c] == "1"))   # idle low
        # MOSI is stable around every rising edge (set at the falling edge before)
        mosi = tr.out(0)
        for r in rises:
            self.assertEqual(mosi[r - 1], mosi[r])

    def test_isa_example_timing_reads_miso_one_bit_late(self):
        """Finding F3: isa.md's 10 MHz example (PERIOD = PERIOD2 = 3) samples
        MISO through the 3-cycle input synchroniser at the cycle it raises
        SCK, i.e. the pin as it was 3 cycles earlier: the previous bit."""
        lane, target, _ = self.transfer([0x9F], [0xC3], period="3", period2="3")
        self.assertEqual(target.received, [0x9F])            # MOSI is fine
        # every sample sees the bit before (the first sees bit 7, already
        # valid since CS fell): 1 1 1 0 0 0 0 1 instead of 1 1 0 0 0 0 1 1
        self.assertEqual(lane.host_read_all(), [0x80 | (0xC3 >> 1)])


class I2cTarget(unittest.TestCase):
    def transaction(self, addr, read=None, writes=(), host=(), late_host=None):
        lane = lane_for("i2c_target")
        lane.host_write(*host)
        c = I2cController(half=300)              # 100 kHz
        c.idle(400)
        c.start()
        c.write_byte(addr << 1 | (1 if read else 0))
        for b in writes:
            c.write_byte(b)
        for k in range(read or 0):
            c.read_byte(ack=0 if k < read - 1 else 1)
        c.stop()
        c.idle(400)
        tr = model.Trace()
        while not c.done() and len(tr) < 100_000:
            model.run(lane, 100, bench=c, trace=tr)
            if late_host and len(tr) == late_host[0]:
                lane.host_write(*late_host[1])
        self.assertTrue(c.done())
        return lane, c, tr

    def test_acknowledges_its_address(self):
        lane, c, _ = self.transaction(0x50)
        self.assertEqual(c.samples, [0])                       # ACK
        self.assertEqual(lane.host_read_all(), [0x0200_0050])  # tag 2: address

    def test_does_not_acknowledge_another_address(self):
        lane, c, tr = self.transaction(0x51)
        self.assertEqual(c.samples, [1])                       # NACK
        self.assertFalse(any(e & 1 for e in tr.out_en))        # never drove SDA
        self.assertFalse(any(e >> 3 & 1 for e in tr.out_en))   # never held SCL

    def test_write_bytes_reach_the_host(self):
        lane, c, _ = self.transaction(0x50, writes=[0x10, 0xA5])
        self.assertEqual(c.samples, [0, 0, 0])
        self.assertEqual(lane.host_read_all(), [0x0200_0050, 0x0400_0010, 0x0400_00A5])

    def test_read_bytes_from_the_host(self):
        lane, c, _ = self.transaction(0x50, read=2, host=(0x12, 0x34))
        self.assertEqual(c.samples[0], 0)
        self.assertEqual(c.samples[1:9], [0, 0, 0, 1, 0, 0, 1, 0])
        self.assertEqual(c.samples[9:17], [0, 0, 1, 1, 0, 1, 0, 0])

    def test_stretches_scl_until_the_host_supplies_data(self):
        lane, c, _ = self.transaction(0x50, read=1, late_host=(12_000, [0xC7]))
        self.assertGreater(c.stretch_cycles, 3000)
        self.assertEqual(c.samples, [0, 1, 1, 0, 0, 0, 1, 1, 1])

    def test_ignores_traffic_after_stop_and_answers_again(self):
        lane, c, _ = self.transaction(0x51)
        self.assertEqual(c.samples, [1])
        c2 = I2cController(half=300)
        c2.start()
        c2.write_byte(0x50 << 1)
        c2.stop()
        for _ in range(500):
            if c2.done():
                break
            model.run(lane, 100, bench=c2)
        self.assertEqual(c2.samples, [0])


if __name__ == "__main__":
    unittest.main()
