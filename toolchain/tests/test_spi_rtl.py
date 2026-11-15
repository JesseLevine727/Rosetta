"""SPI end to end on the chip RTL (src/rosetta_top.sv under Verilator).

Lane 0 runs the compiled controller, lane 1 the target, lanes 2 and 3 the
listener (one lane per data wire, gap G5). Controller pins leave on uo[2]
(mosi), uo[3] (cs_n), uo[4] (sck); the target's miso on uo[5]. Board wires
take uo[2..5] to ui[3..6] and the crossbar hands them to the lanes, as in
the FPGA loopback runs. Each mode 0-3 runs at 1 MHz and at the fastest clock
the compiler accepts for the whole link; the compiler must reject a clock
just above it. Checked on the wires: every bit both ways, the clock period,
gap-free words, and the description's rules (cs_lead, cs_lag, cs_idle,
mosi_su, miso_su). Checked in the queues: every record of every lane against
the manifests, listener timestamps to the cycle. A last test runs the
FPGA-proven hand-written controller and target against the compiled ones.

Skips when Verilator is not installed. ROSETTA_SIM_REPORT=1 prints a summary.
"""
import math
import os
import unittest
from fractions import Fraction

import support
import rtlsim
from rosetta import asm
from rosetta.compiler import CompileError, compile_file

SPI = os.path.join(rtlsim.REPO, "programs", "spi", "spi.ros")
NOP = 0xC0000000
CLK = 60_000_000
TRANSFERS = [([0x9F, 0x00], [0xFF, 0xEF]), ([0xA5, 0x3C, 0x81], [0x5A, 0xC3, 0x18]),
             ([0x01], [0x80])]
BURST = [([0x11], [0x00]), ([0x22], [0x00])]     # back to back, target queue empty: its default
MOSI, CS, SCK, MISO = 2, 3, 4, 5                 # uo bits; ui = uo + 1 through the board wires
ORIGIN = {"controller": 0, "target": 32, "mosi": 72, "miso": 104}
REPORT = bool(os.environ.get("ROSETTA_SIM_REPORT"))


def osel(lane: int, out: int) -> int:
    return 0x10 | (lane << 2) | out


def params(mode: int, rate: str) -> dict:
    return {"cpol": str(mode >> 1), "cpha": str(mode & 1), "sck_rate": rate}


def compile_link(mode: int, rate: str) -> dict:
    p = params(mode, rate)
    return {"controller": compile_file(SPI, "controller", p, origin=ORIGIN["controller"]),
            "target": compile_file(SPI, "target", p, origin=ORIGIN["target"]),
            "mosi": compile_file(SPI, "listener", p, origin=ORIGIN["mosi"], listen_wire="mosi"),
            "miso": compile_file(SPI, "listener", p, origin=ORIGIN["miso"], listen_wire="miso")}


def run_link(images, transfers, word_cycles, count_word=True, burst=()):
    """images: {lane: (origin, words)}. Each transfer: the target's words are
    queued first (it must have them before the controller clocks them), then
    the controller's [count,] words. `burst` transfers are then queued to the
    controller all at once, so they run back to back. Returns pin changes,
    the four queues and the run bits."""
    image = [NOP] * 160
    for org, words in images.values():
        image[org:org + len(words)] = words
    with rtlsim.Chip() as chip:
        chip.load_program(image)
        chip.wr(rtlsim.XOUT0, osel(0, 0) | osel(0, 2) << 5 | osel(0, 3) << 10 | osel(1, 0) << 15)
        chip.wr(rtlsim.XIN0, 6 | 3 << 16 | 5 << 20 | 4 << 24)    # lane 0 in0 = miso; lane 1 mosi, sck, cs_n
        chip.wr(rtlsim.XIN1, (3 | 5 << 4 | 4 << 8) | (6 | 5 << 4 | 4 << 8) << 16)  # lanes 2, 3
        for uo in (MOSI, CS, SCK, MISO):
            chip.wire(uo, uo + 1)
        chip.start_lane(0, images[0][0])             # controller first: cs_n idles high before
        chip.run(200)                                # the others look at it
        for lane in (1, 2, 3):
            if lane in images:
                chip.start_lane(lane, images[lane][0])
        chip.run(200)
        chip.dump()
        for mo, mi in transfers:
            chip.push(1, mi)
            chip.push(0, ([len(mo)] if count_word else []) + mo)
            chip.run(len(mo) * 8 * word_cycles + 2000)
        if burst:
            chip.push(0, [w for mo, _ in burst for w in [len(mo)] + mo])
            chip.run(sum(len(mo) for mo, _ in burst) * 8 * word_cycles + 2000)
        changes = chip.dump()
        queues = {lane: chip.drain(lane) for lane in (0, 1, 2, 3)}
        run = [chip.rd(rtlsim.lane_reg(k, rtlsim.CTRL)) & 1 for k in range(4)]
    return changes, queues, run


def decode(changes, cpol: int, cpha: int):
    """Frames on the pins: per chip-select window the sck edges, the bits at
    each capture edge, and the last mosi/miso change before each capture."""
    cl = (1 - cpol) if cpha == 0 else cpol        # leading edge goes away from idle
    lv, evs = {}, []
    for cyc, uo, _ in changes:
        for pin in (MOSI, CS, SCK, MISO):
            b = (uo >> pin) & 1
            if pin in lv and b != lv[pin]:
                evs.append((cyc, pin, b))
            lv[pin] = b
    frames, cur, last = [], None, {MOSI: None, MISO: None}
    sck_level = cpol
    for cyc, pin, b in evs:
        if pin == CS:
            if b == 0:
                cur = {"fall": cyc, "edges": [], "mosi": [], "miso": [], "setup": [],
                       "idle_ok": sck_level == cpol}
            elif cur is not None:
                cur["rise"] = cyc
                frames.append(cur)
                cur = None
        elif pin == SCK:
            sck_level = b
            if cur is not None:
                cur["edges"].append(cyc)
                if b == cl:
                    cur["mosi"].append(lv_at(evs, MOSI, cyc))
                    cur["miso"].append(lv_at(evs, MISO, cyc))
                    cur["setup"].append(min(cyc - (last[MOSI] or -10**9), cyc - (last[MISO] or -10**9)))
        else:
            last[pin] = cyc
    for f in frames:
        f["words_mosi"] = pack(f["mosi"])
        f["words_miso"] = pack(f["miso"])
    return frames


def lv_at(evs, pin, cyc):
    v = 0
    for c, p, b in evs:
        if c > cyc:
            break
        if p == pin:
            v = b
    return v


def pack(bits):
    return [sum(b << (7 - i) for i, b in enumerate(bits[k:k + 8])) for k in range(0, len(bits), 8)]


@unittest.skipUnless(rtlsim.find_verilator(), "Verilator not installed")
class SpiOnRtl(unittest.TestCase):

    def run_mode(self, mode: int, rate: str):
        cpol, cpha = mode >> 1, mode & 1
        comp = compile_link(mode, rate)
        tm = comp["controller"].manifest["timing"]
        half = tm["clock_cycles"] // 2
        images = {0: (ORIGIN["controller"], comp["controller"].words),
                  1: (ORIGIN["target"], comp["target"].words),
                  2: (ORIGIN["mosi"], comp["mosi"].words), 3: (ORIGIN["miso"], comp["miso"].words)}
        changes, q, run = run_link(images, TRANSFERS, tm["clock_cycles"], burst=BURST)
        self.assertEqual(run, [1, 1, 1, 1])
        xfers = TRANSFERS + BURST
        frames = decode(changes, cpol, cpha)
        # every bit both ways, eight capture edges per word
        self.assertEqual([f["words_mosi"] for f in frames], [mo for mo, _ in xfers])
        self.assertEqual([f["words_miso"] for f in frames], [mi for _, mi in xfers])
        self.assertEqual([len(f["edges"]) for f in frames], [16 * len(mo) for mo, _ in xfers])
        # clock: every edge of a frame exactly PERIOD apart (gap-free words); idle level cpol
        for f in frames:
            self.assertEqual({b - a for a, b in zip(f["edges"], f["edges"][1:])}, {half}, rate)
            self.assertTrue(f["idle_ok"])
        # the description's rules, at the described rate R
        bit = Fraction(CLK) / Fraction(int(float(rate.split()[0]) * 1e6))
        lead = min(f["edges"][0] - f["fall"] for f in frames)
        lag = min(f["rise"] - f["edges"][-1] for f in frames)
        idle = min(b["fall"] - a["rise"] for a, b in zip(frames, frames[1:]))
        setup = min(min(f["setup"]) for f in frames)
        self.assertGreaterEqual(lead, bit / 2, "cs_lead")
        self.assertGreaterEqual(lag, bit / 2, "cs_lag")
        self.assertGreaterEqual(idle, bit, "cs_idle")
        self.assertGreaterEqual(setup, math.ceil(10e-9 * CLK), "mosi_su / miso_su")
        # records, against the manifests' tags
        tag = lambda man, kind, name: next(int(k, 16) for k, v in man["out_queue"]["tags"].items()
                                           if v["kind"] == kind and name in (v.get("field"), v.get("event")))
        tman = comp["target"].manifest
        sel, desel = tag(tman, "event", "select"), tag(tman, "event", "deselect")
        mosi_t, miso_t = tag(tman, "value", "mosi_data"), tag(comp["controller"].manifest, "value", "miso_data")
        self.assertEqual(q[0], [miso_t << 24 | w for _, mi in xfers for w in mi])
        want_t = [r for mo, _ in xfers for r in [sel << 24] + [mosi_t << 24 | w for w in mo] + [desel << 24]]
        self.assertEqual(q[1], want_t)
        self.assertEqual(q[2][1::2], want_t)
        self.assertEqual(q[3][1::2], [r for _, mi in xfers for r in
                                      [sel << 24] + [miso_t << 24 | w for w in mi] + [desel << 24]])
        # listener timestamps: words at the lane's last capture edge (pin + 3), one offset for
        # both lanes; events within the manifest's bounds after the cs_n edge
        last = 14 if cpha == 0 else 15                  # a word's 8th capture edge among its 16 edges
        caps = [e for f in frames for k, e in enumerate(f["edges"]) if k % 16 == last]
        pins = [x for f in frames for x in (f["fall"], f["rise"])]
        offs, lates = set(), []
        for lane in (2, 3):
            ts, recs = q[lane][0::2], q[lane][1::2]
            words = [t for t, r in zip(ts, recs) if r >> 24 not in (sel, desel)]
            offs |= {t - (c + 3) for t, c in zip(words, caps)}
        self.assertEqual(len(offs), 1, "word timestamps not one constant after the capture edge")
        off = offs.pop()
        for lane, man in ((2, comp["mosi"]), (3, comp["miso"])):
            bounds = man.manifest["out_queue"]["timestamps"]["event_late_cycles_after_pin_edge"]
            ts, recs = q[lane][0::2], q[lane][1::2]
            evts = [(t, r >> 24) for t, r in zip(ts, recs) if r >> 24 in (sel, desel)]
            for (t, kind), pin in zip(evts, pins):
                name = "select" if kind == sel else "deselect"
                lates.append((name, t - off - pin))
                lo, hi = bounds[name]
                self.assertTrue(lo <= t - off - pin <= hi, (name, t - off - pin, bounds))
        if REPORT:
            print(f"\n  mode {mode} at {rate}: sck {2 * half} cycles = {CLK / (2 * half) / 1e6:g} MHz; "
                  f"mosi {[f['words_mosi'] for f in frames]} miso {[f['words_miso'] for f in frames]}"
                  f"\n    cs_lead {lead}, cs_lag {lag}, cs_idle {idle} (rules {float(bit / 2):g}, "
                  f"{float(bit / 2):g}, {float(bit):g}); min setup {setup} cycles"
                  f"\n    listener events after the cs_n edge: "
                  f"{sorted(set(lates))}")
        return comp

    def link_max(self, mode: int) -> str:
        """Fastest clock the compiler accepts for every role of the link."""
        comp = compile_link(mode, "1 MHz")
        return min(c.manifest["timing"]["fastest_rate_hz_schedule"] for c in comp.values())

    def check_mode(self, mode: int):
        self.run_mode(mode, "1 MHz")
        top = self.link_max(mode)
        self.assertEqual(top, 1_500_000 if mode & 1 == 0 else 2_000_000)
        self.run_mode(mode, f"{top / 1e6:g} MHz")
        with self.assertRaisesRegex(CompileError, "schedule does not fit"):
            compile_file(SPI, "target", params(mode, f"{(top + 10_000) / 1e6:g} MHz"))

    def test_mode0(self):
        self.check_mode(0)

    def test_mode1(self):
        self.check_mode(1)

    def test_mode2(self):
        self.check_mode(2)

    def test_mode3(self):
        self.check_mode(3)

    def test_against_hand_written(self):
        """Mode 0, 1 MHz, one word per transfer (all the hand-written pair does):
        the FPGA-proven controller with the compiled target and listeners,
        then the compiled controller with the FPGA-proven target."""
        comp = compile_link(0, "1 MHz")
        hand_c = asm.assemble(support_text("spi_controller")).words
        hand_t = asm.assemble(support_text("spi_target"), 32).words
        one = [([0xA5], [0x5A]), ([0x3C], [0xC3]), ([0x00], [0xFF])]
        images = {0: (0, hand_c), 1: (32, comp["target"].words),
                  2: (ORIGIN["mosi"], comp["mosi"].words), 3: (ORIGIN["miso"], comp["miso"].words)}
        changes, q, run = run_link(images, one, 60, count_word=False)
        self.assertEqual(run, [1, 1, 1, 1])
        self.assertEqual([w & 0xFF for w in q[0]], [mi[0] for _, mi in one])     # untagged in the hand program
        self.assertEqual(q[1], [r for mo, _ in one for r in (0x03 << 24, 0x01 << 24 | mo[0], 0x04 << 24)])
        self.assertEqual([f["words_miso"] for f in decode(changes, 0, 0)], [mi for _, mi in one])
        images = {0: (0, comp["controller"].words), 1: (32, hand_t)}
        changes, q, run = run_link(images, one, 60)
        self.assertEqual(run[:2], [1, 1])
        self.assertEqual(q[0], [0x02 << 24 | mi[0] for _, mi in one])
        self.assertEqual(q[1], [mo[0] for mo, _ in one])                          # untagged
        frames = decode(changes, 0, 0)
        self.assertEqual([f["words_mosi"] for f in frames], [mo for mo, _ in one])


def support_text(name: str) -> str:
    with open(os.path.join(support.FIXTURES, f"hand_{name}.asm"), encoding="utf-8") as f:
        return f.read()


if __name__ == "__main__":
    unittest.main()
