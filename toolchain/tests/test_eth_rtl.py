"""10 Mbit Ethernet (10BASE-T) on the chip RTL (Verilator, or the board with
ROSETTA_BACKEND=board): the protocol study's hand-written transmit and
receive programs (study/protocols/REPORT.md section 4;
fixtures/study_eth_tx.asm and study_eth_rx.asm, unchanged except the
link-pulse loop count, shortened as the study's eth_tx_check.py does).

Transmit: lane 2 on uo[2] (TD+) and uo[3] (TD-), BAUD 3 + 3 cycles,
Manchester, differential, CRC-32 FCS, TP_IDL, normal link pulses; a board
wire takes TD+ to ui[3], lane 0's receive input. Checked as the study did
(4.6): edge intervals only 3 and 6 cycles, 64 preamble + SFD bits, FCS equal
to zlib's CRC-32, TD+ high 21 cycles after the last transition (last bit 1)
or 18 (last bit 0), TD- the exact complement, link pulses 6 cycles wide and
one loop period apart, and every frame received by lane 0 with a good CRC.

Receive: lane 0 (PULSE timing, Manchester, THR0 = 5, SFD found by EXPECT
seek, payload to data memory, CRC-32 by residue) from the harness's waveform
player on ui[3]: a continuous-time Manchester line with a random phase
against the chip clock and up to +-100 ppm, sampled once per cycle as the
first synchroniser flop sees it (study ethlib.py). Frames of 64-69 bytes
(partial last words) and 120 bytes are received exactly with a good CRC;
frames with duty-cycle distortion 14 ns, alternating jitter 7 ns or uniform
jitter 7 ns per edge (inside the study's limits 16.5 / 8.2 / 8.0 ns) are all
received; beyond the theoretical limit (16.7 ns DCD, 8.3 ns alternating),
at least one of eight phases fails, as the interval classifier predicts.
"""
import os
import random
import unittest
import zlib

import support
import rtlsim
from protolines import frame_wave, make_frame, rev32, runs_from_levels
from rosetta import asm

NOP = 0xC0000000


def fixture(name, origin=0, subs=None):
    with open(os.path.join(support.FIXTURES, name), encoding="utf-8") as f:
        t = f.read()
    for k, v in (subs or {}).items():
        t = t.replace(k, v)
    return asm.assemble(t, origin=origin)


def decode_records(recs):
    out = {}
    for w in recs:
        out.setdefault(w >> 24, []).append(w & 0xFFFFFF)
    return out


def receive(chip, frame, rng, **imp):
    """Play one frame into ui[3]; read lane 0's records and buffer back."""
    levels, _ = frame_wave(frame, rng, **imp)
    chip.wave(runs_from_levels(levels, 1))
    chip.play(400)
    recs = chip.drain(0, out_size=8)
    r = decode_records(recs)
    if not all(k in r for k in (0xE1, 0xE2, 0xE3, 0xEE)) or len(recs) != 4:
        return {"ok": False, "why": [hex(x) for x in recs]}
    nwords = (r[0xE1][0] & 0xFFFF) - 8
    sr = ((r[0xE2][0] & 0xFFFF) << 16) | (r[0xE3][0] & 0xFFFF)
    crc_good = r[0xEE][0] & 1
    if not 0 <= nwords <= 112:
        return {"ok": False, "why": f"word count {nwords}"}
    words = chip.mem_read(0, 8, nwords) if nwords else []
    data = b"".join(rev32(w).to_bytes(4, "little") for w in words)
    k = sr.bit_length() - 1                           # marker: bits in the partial word
    if k > 0:
        part = int(f"{sr & ((1 << k) - 1):0{k}b}"[::-1], 2)
        data += part.to_bytes((k + 7) // 8, "little")[: k // 8]
    return {"ok": crc_good == 1 and data == frame and k % 8 == 0, "crc": crc_good,
            "match": data == frame, "len": len(data), "k": k}


def rx_session():
    """Lane 0 runs the receiver; channel 0 drives ui[3] (idle high)."""
    chip = rtlsim.Chip()
    chip.load_program(fixture("study_eth_rx.asm").words)
    chip.wr(rtlsim.XIN0, 3)
    chip.map(0, 3)
    chip.pull(3, 1)
    chip.tracing(False)
    chip.start_lane(0, 0, qcfg=0)                     # 8-word queues: buffer words 8..119
    chip.run(200)
    return chip


@unittest.skipUnless(rtlsim.available(), rtlsim.unavailable_reason())
class EthOnRtl(unittest.TestCase):

    def test_transmit_loopback(self):
        rx = fixture("study_eth_rx.asm")
        tx = fixture("study_eth_tx.asm", 64, {".equ NLPN, 78": ".equ NLPN, 2"})
        rng = random.Random(4)
        with rtlsim.Chip() as chip:
            chip.load_program(rx.words + [NOP] * (64 - len(rx.words)) + tx.words)
            chip.wr(rtlsim.XIN0, 3)                       # lane 0 in0 <- ui[3]
            chip.wr(rtlsim.XOUT0, 0x18 | (0x19 << 5))     # uo[2] <- lane 2 out0, uo[3] <- out1
            chip.wire(2, 3)
            chip.start_lane(0, 0, qcfg=0)
            chip.start_lane(2, 64, qcfg=0)
            chip.run(60000)                               # two link-pulse periods (NLPN = 2)
            rows = chip.dump(full=True)
            pulses, prev, st = [], 0, None
            for c, uo, *_ in rows:
                p, n = (uo >> 2) & 1, (uo >> 3) & 1
                if p and not n and prev == 0:
                    st = c
                if not p and prev == 1:
                    pulses.append((st, c - st))
                prev = p
            self.assertTrue(len(pulses) >= 3, pulses)
            self.assertEqual({w for _, w in pulses}, {6}, "link pulse 100 ns")
            periods = [b[0] - a[0] for a, b in zip(pulses, pulses[1:])]
            self.assertTrue(all(2 * 12285 <= p <= 2 * 12285 + 200 for p in periods), periods)
            frames = []
            for plen in (46, 50, 102):
                body = bytes(rng.randrange(256) for _ in range(plen + 14))
                words = [int.from_bytes(body[i:i + 4], "little") for i in range(0, len(body), 4)]
                chip.mem_write(2, 16, words)
                chip.dump()
                chip.push(2, [len(words)], in_size=8)
                chip.run(300000)
                rows = chip.dump(full=True)
                frames.append((body, rows, chip.drain(0, out_size=8)))
        for body, rows, recs in frames:
            ed, prev, comp_bad = [], None, 0
            for c, uo, *_ in rows:
                p, n = (uo >> 2) & 1, (uo >> 3) & 1
                if prev is not None and p != prev:
                    ed.append((c, p))
                if p == n == 1:
                    comp_bad += 1
                prev = p
            burst = []
            for c, p in ed:
                if burst and c - burst[-1][0] > 20 and len(burst) > 100:
                    break
                if burst and c - burst[-1][0] > 20:
                    burst = []
                burst.append((c, p))
            self.assertEqual({b[0] - a[0] for a, b in zip(burst, burst[1:])}, {3, 6})
            start = burst[0][0] - 3 if burst[0][1] == 1 else burst[0][0]

            def level_at(t):
                v = 0
                for c, p in burst:
                    if c > t:
                        break
                    v = p
                return v
            bits = [level_at(start + 6 * k + 4) for k in range((burst[-1][0] - start) // 6 + 1)]
            s = "".join(map(str, bits))
            i = s.find("10101011")
            self.assertEqual(i + 8, 64, "preamble + SFD bits")
            data_bits = bits[i + 8:]
            got = bytes(sum(data_bits[8 * k + j] << j for j in range(8)) for k in range(len(data_bits) // 8))
            self.assertEqual(got[:len(body)], body)
            self.assertEqual(got[len(body):len(body) + 4], zlib.crc32(body).to_bytes(4, "little"))
            last = burst[-1]
            after = [e for e in ed if e[0] > last[0]]
            self.assertEqual(after[0][0] - last[0] if last[1] == 1 else None,
                             21 if last[1] == 1 else None, "TP_IDL: TD+ high 350 ns after a final 1")
            if last[1] == 0:                              # final 0: TD+ rises at the bit end, 18 cycles
                self.assertEqual(after[1][0] - after[0][0], 18)
            self.assertEqual(comp_bad, 0, "TD- is the complement of TD+")
            r = decode_records(recs)
            self.assertEqual(r[0xEE], [1], "received with a good CRC")
            self.assertEqual(r[0xE1][0] - 8, (len(body) + 4) // 4)

    def test_receive_frame_lengths(self):
        rng = random.Random(20261121)
        with rx_session() as chip:
            results = []
            for plen in (46, 47, 49, 50, 51, 102):
                fr = make_frame(rng, plen)
                results.append((plen, receive(chip, fr, rng, ppm=rng.uniform(-100, 100))))
        for plen, res in results:
            self.assertTrue(res["ok"], (plen, res))
            self.assertEqual(res["len"], plen + 18)

    def test_receive_inside_margin(self):
        rng = random.Random(7)
        with rx_session() as chip:
            res = []
            for imp in ({"mode": "none", "dcd": 14.0}, {"mode": "alt", "jitter": 7.0},
                        {"mode": "uniform", "jitter": 7.0}):
                for _ in range(3):
                    r = receive(chip, make_frame(rng, 46), rng, ppm=rng.uniform(-100, 100), **imp)
                    res.append((imp, r))
        for imp, r in res:
            self.assertTrue(r["ok"], (imp, r))

    def test_receive_beyond_margin(self):
        """At 0 ppm every edge of a frame has the same phase against the
        clock (50 ns = 3 cycles), so a frame fails or passes as a whole; over
        eight phases spread across one cycle, DCD 20 ns and alternating jitter
        10 ns each fail at least one (they exceed the 16.7 / 8.3 ns limits),
        and a clean frame passes at every phase."""
        rng = random.Random(9)
        out = {}
        with rx_session() as chip:
            for name, imp in (("clean", {"mode": "none"}), ("dcd20", {"mode": "none", "dcd": 20.0}),
                              ("alt10", {"mode": "alt", "jitter": 10.0})):
                out[name] = [receive(chip, make_frame(rng, 46), rng, ppm=0.0,
                                     phase=(k + 0.5) * (1000 / 60) / 8, **imp)["ok"] for k in range(8)]
        self.assertEqual(out["clean"], [True] * 8)
        self.assertIn(False, out["dcd20"], out)
        self.assertIn(False, out["alt10"], out)
        EthOnRtl.beyond = out


if __name__ == "__main__":
    unittest.main()
