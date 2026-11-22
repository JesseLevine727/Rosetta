"""CAN on the chip RTL (Verilator, or the board with ROSETTA_BACKEND=board):
the protocol study's hand-written two-lane nodes (study/protocols/REPORT.md
section 6; programs copied to fixtures/study_can_tx.asm and study_can_rx.asm,
unchanged).

Node A = lane 0 (transmitter) + lane 1 (receiver), node B = lanes 2 and 3.
Each node's two lanes drive open-drain outputs on its TXD net (uio[0..1] for
A, uio[2..3] for B); the harness's CAN bus model ANDs both nets (delayed
dtx) with an external scripted node (waveform channel 7) and gives each
node's RXD (ui[3], ui[4]) the bus delayed drx more cycles: a transceiver loop
delay of dtx + drx. The external node sends 0x700 first; when its
intermission ends both nodes start together; 0x0F0 (B) wins arbitration at
identifier bit 8, A withdraws inside that bit, takes and acknowledges B's
frame, then sends 0x123 and is acknowledged.

Checked against the study's claims (REPORT.md 6.4): the bus carries 0x700,
0x0F0, 0x123 in that order, each with a good CRC and acknowledged; A's
records are "arbitration lost" then "sent and acknowledged", B's "sent and
acknowledged"; both receivers take all three frames with a good CRC and ACK
every frame but their own; A's TXD net is dominant for exactly SOF and two
identifier bits at the contested frame; no lane ever drives against the bus.
Also the listen-only node (a receiver that never drives) and the ACK-only
node (one receiver lane acknowledging the external node).
"""
import os
import unittest

import support
import rtlsim
from protolines import can_decode_bus, can_ext_runs, can_wire, pin_edges
from rosetta import asm

NOP = 0xC0000000
ID_A, DATA_A = 0x123, bytes([0xAB, 0xCD])
ID_B, DATA_B = 0x0F0, bytes([0x55])
ID_X, DATA_X = 0x700, bytes([0x0F, 0xF0, 0x00])


def fixture(name):
    with open(os.path.join(support.FIXTURES, name), encoding="utf-8") as f:
        return f.read()


def programs(kbps: int, kd: int, **extra_subs):
    """The study's substitutions (can_arbitration.py) for bit rate and K."""
    tbit = 60000 // kbps
    per = (tbit * 3) // 4
    subs = {"KD": str(kd), "PERIOD, 90": f"PERIOD, {per}", "PERIOD2, 30": f"PERIOD2, {tbit - per}",
            "PERIOD, 120": f"PERIOD, {tbit}", "wait=8": f"wait={(6 * tbit + per - 1) // per}",
            **extra_subs}

    def one(name, origin, own, free):
        t = fixture(name)
        for k, v in {**subs, "OWN": own, "FREE": free}.items():
            t = t.replace(k, v)
        return asm.assemble(t, origin=origin)
    return tbit, {0: one("study_can_tx.asm", 0, "C0", "C1"), 1: one("study_can_rx.asm", 64, "C0", "C1"),
                  2: one("study_can_tx.asm", 128, "C2", "C3"), 3: one("study_can_rx.asm", 192, "C2", "C3")}


def dominant_runs(rows, bit):
    """(start, length) of every low stretch of uio_in bit `bit` (a TXD net)."""
    out, start = [], None
    for c, v in pin_edges(rows, 5, bit):
        if v == 0:
            start = c
        elif start is not None:
            out.append((start, c - start))
            start = None
    return out


def rx_frames(words):
    """A receiver lane's records -> [(id, control, data bytes, end flags)]."""
    frames, cur = [], None
    for w in words:
        tag, v = w >> 24, w & 0xFFFFFF
        if tag == 0xD1:
            cur = [v, None, [], None]
        elif tag == 0xD2 and cur:
            cur[1] = v
        elif tag == 0xD3 and cur:
            cur[2].append(v)
        elif tag == 0xD4 and cur:
            cur[3] = v
            frames.append((cur[0], cur[1], bytes(cur[2]), cur[3]))
            cur = None
        else:
            frames.append(("other", hex(w)))
    return frames


def run_bus(kbps, loop, kd, lanes=(0, 1, 2, 3), progs=None):
    tbit, p = progs or programs(kbps, kd)
    image = [NOP] * 256
    for prog in p.values():
        image[prog.origin:prog.origin + len(prog.words)] = prog.words
    with rtlsim.Chip() as chip:
        chip.load_program(image)
        chip.wr(rtlsim.XIN0, 3 | (8 << 4) | (3 << 16))       # lane 0 in0 RXD_A, in1 TXD_A; lane 1 in0 RXD_A
        chip.wr(rtlsim.XIN1, 4 | (10 << 4) | (4 << 16))      # lane 2 in0 RXD_B, in1 TXD_B; lane 3 in0 RXD_B
        chip.wr(rtlsim.XOUT1, 0x10 | (0x14 << 5) | (0x18 << 10) | (0x1C << 15))
        chip.canbus(loop // 2, loop - loop // 2)
        for lane, org in ((1, 64), (3, 192), (0, 0), (2, 128)):
            if lane in lanes:
                chip.start_lane(lane, org, qcfg=0xF)
        chip.run(200)
        for lane, ident, data in ((0, ID_A, DATA_A), (2, ID_B, DATA_B)):
            if lane in lanes:
                chip.mem_write(lane, 7, [len(data)] + list(data))
                chip.push(lane, [ident << 1, len(data)])
        chip.run(2000)
        chip.dump()
        bits_x = can_wire(ID_X, DATA_X)
        chip.wave(can_ext_runs(bits_x, tbit, 50))
        chip.run((len(bits_x) + 3 * 140 + 60) * tbit)
        rows = chip.dump(full=True)
        end = chip.cycle()
        recs = {k: chip.drain(k) for k in lanes}
        contention = chip.contention()
    return tbit, rows, recs, contention, end


@unittest.skipUnless(rtlsim.available(), rtlsim.unavailable_reason())
class CanOnRtl(unittest.TestCase):

    def check_arbitration(self, kbps, loop, kd):
        tbit, rows, recs, contention, end = run_bus(kbps, loop, kd)
        # bits sampled at the programs' own sample point, 75 % (PERIOD of can_tx)
        sp = (tbit * 3) // 4
        bus = can_decode_bus(rows, tbit, pin=3, end_t=end, sample=sp)
        self.assertEqual([(f["id"], f["data"]) for f in bus],
                         [(ID_X, DATA_X), (ID_B, DATA_B), (ID_A, DATA_A)])
        for f in bus:
            self.assertTrue(f["crc_ok"], f)
            self.assertEqual((f["crc_delim"], f["ack_slot"], f["ack_delim"], f["eof"]),
                             (1, 0, 1, [1] * 7), f"frame {f['id']:#x} tail: acknowledged")
        # sampled mid-bit (the study's decoder): the ACK the receiver lanes drive is
        # stretched by the loop delay into the ACK delimiter (REPORT.md 6.4: 132 cycles
        # at 500 k); recorded for the report, not a pass criterion
        self.mid_bit_ack_delims = [f["ack_delim"] for f in can_decode_bus(rows, tbit, pin=3, end_t=end)]
        self.assertEqual([f["id"] for f in can_decode_bus(rows, tbit, pin=4, end_t=end)], [ID_X, ID_B, ID_A])
        # transmitters: A lost arbitration (C1), then sent and was acknowledged (C0); B sent (C0)
        self.assertEqual(recs[0], [0xC1000001, 0xC0000000])
        self.assertEqual(recs[2], [0xC0000000])
        # receivers: every frame with a good CRC; ACK driven (4) for the others' frames,
        # own (2) without ACK for its own
        want = {1: [(ID_X, 5), (ID_B, 5), (ID_A, 3)], 3: [(ID_X, 5), (ID_B, 3), (ID_A, 5)]}
        data = {ID_X: DATA_X, ID_B: DATA_B, ID_A: DATA_A}
        for lane in (1, 3):
            got = rx_frames(recs[lane])
            self.assertEqual([(f[0], f[3]) for f in got], want[lane], f"lane {lane}")
            for ident, ctl, d, _ in got:
                self.assertEqual(ctl, len(data[ident]))
                self.assertEqual(d, data[ident])
        # A's TXD net (uio_in bit 0) at the contested frame: dominant for exactly SOF and
        # two identifier bits, then not again until its receiver lane drives B's ACK slot
        a_dom = dominant_runs(rows, 0)
        sof_b = bus[1]["sof_cycle"]
        contested = [d for d in a_dom if sof_b - 2 * tbit <= d[0] <= sof_b + tbit]
        self.assertEqual([d[1] for d in contested], [3 * tbit], a_dom)
        self.assertEqual(contention, 0)
        return bus, a_dom

    def test_arbitration_250k(self):
        self.check_arbitration(250, 15, 60)

    def test_arbitration_500k(self):
        self.check_arbitration(500, 8, 40)

    def test_arbitration_1000k(self):
        self.check_arbitration(1000, 15, 30)

    def test_ack_only_node(self):
        """Lane 1 alone (a receiver lane) acknowledges the external frame."""
        tbit, rows, recs, _, end = run_bus(500, 8, 40, lanes=(1,))
        bus = can_decode_bus(rows, tbit, pin=3, end_t=end)
        self.assertEqual([(f["id"], f["data"], f["crc_ok"], f["ack_slot"]) for f in bus],
                         [(ID_X, DATA_X, True, 0)])
        self.assertEqual([(f[0], f[3]) for f in rx_frames(recs[1])], [(ID_X, 5)])

    def test_listen_only_node(self):
        """The receiver lane without its ACK drive (the study's listen-only
        node: FIELD DRIVE 3 replaced by a 3-bit sample): it takes the frame
        and never drives, so the external frame stays unacknowledged."""
        tbit, p = programs(500, 40, **{"FIELD DRIVE 3, src=IMM 5": "FIELD SAMPLE 3, dst=NONE"})
        tbit, rows, recs, _, end = run_bus(500, 8, 40, lanes=(1,), progs=(tbit, p))
        bus = can_decode_bus(rows, tbit, pin=3, end_t=end)
        self.assertEqual([(f["id"], f["crc_ok"], f["ack_slot"]) for f in bus], [(ID_X, True, 1)])
        self.assertEqual([(f[0], f[2]) for f in rx_frames(recs[1])], [(ID_X, DATA_X)])
        self.assertFalse(any(r[4] & 0x03 for r in rows), "lane 1 never enables its TXD pin")


if __name__ == "__main__":
    unittest.main()
