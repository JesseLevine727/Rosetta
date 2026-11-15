"""I2C end to end on the chip RTL (src/rosetta_top.sv under Verilator).

Lane 0 runs the compiled controller, lane 1 a target at 0x50, lane 3 a
target at 0x51, lane 2 the listener. The lanes' open-drain outputs leave on
the bidirectional pins (controller scl/sda on uio0/uio1, the targets on
uio2/uio3 and uio4/uio5) and the harness joins them into two wired-AND nets
with pull-ups; every lane reads the bus back through its own pin. The
listener's out0/out3 go to uio6/uio7, off the bus, to watch that it never
enables one. One run per clock rate (100 kHz, 400 kHz, the fastest the
compiler accepts for the link) does, in order: a 3-byte write, a 3-byte
read ended by the controller's `last`, a write + repeated START + read (the
2.3 host-view example), a read during which the target's host is late (the
target stretches scl), and a write to 0x52, which nobody acknowledges.

Checked: the bus decoded independently against the transactions; every
record of every lane; listener timestamps; START/STOP setup and hold, scl
low/high and sda setup/hold against the description's rules; no pin ever
driven high; the listener never enables an output; the stretch.
Skips when Verilator is not installed. ROSETTA_SIM_REPORT=1 prints a summary.
"""
import os
import unittest

import support  # noqa: F401
import rtlsim
from rosetta.compiler import CompileError, compile_file

I2C = os.path.join(rtlsim.REPO, "programs", "i2c", "i2c.ros")
NOP = 0xC0000000
ORIGIN = {"controller": 0, "target": 80, "listener": 128, "target51": 176}
SCL_NET, SDA_NET, BUS, LISTENER_PINS = 0x15, 0x2A, 0x3F, 0xC0
W, R, MORE, LAST, STOP, START = 0, 1, 0, 1, 0, 1
STRETCH = 4000                  # cycles the target's host is late (at least)
REPORT = bool(os.environ.get("ROSETTA_SIM_REPORT"))

# (controller words, target words, frames on the bus: (addr, rw, ack, [(byte, ack)], end))
WRITE = ([0x50, W, 3, 0x10, 0x11, 0x12, STOP], [],
         [(0x50, W, 0, [(0x10, 0), (0x11, 0), (0x12, 0)], "stop")])
READ = ([0x50, R, MORE, MORE, LAST, STOP], [0xA1, 0xB2, 0xC3],
        [(0x50, R, 0, [(0xA1, 0), (0xB2, 0), (0xC3, 1)], "stop")])
WR_RS_RD = ([0x50, W, 1, 0x20, START, 0x50, R, MORE, LAST, STOP], [0x5A, 0xA5],
            [(0x50, W, 0, [(0x20, 0)], "start"), (0x50, R, 0, [(0x5A, 0), (0xA5, 1)], "stop")])
STRETCHED = ([0x50, R, LAST, STOP], [0xF7], [(0x50, R, 0, [(0xF7, 1)], "stop")])
NACK = ([0x52, W, 1, 0x33, STOP], [], [(0x52, W, 1, [], "stop")])
ALL = [WRITE, READ, WR_RS_RD, STRETCHED, NACK]
SKEW = 2                        # the 0x51 target sees scl this many cycles late (G4)


def osel(lane: int, out: int) -> int:
    return 0x10 | (lane << 2) | out


def compile_bus(rate: str) -> dict:
    p = {"scl_rate": rate}
    return {"controller": compile_file(I2C, "controller", p, origin=ORIGIN["controller"]),
            "target": compile_file(I2C, "target", p, origin=ORIGIN["target"]),
            "listener": compile_file(I2C, "listener", p, origin=ORIGIN["listener"]),
            "target51": compile_file(I2C, "target", {**p, "address": "0x51"},
                                     origin=ORIGIN["target51"])}


def run_bus(comp: dict, clock: int):
    image = [NOP] * 224
    for c in comp.values():
        image[c.origin:c.origin + len(c.words)] = c.words
    with rtlsim.Chip() as chip:
        chip.load_program(image)
        x1 = 0
        for k, (lane, out) in enumerate([(0, 3), (0, 0), (1, 3), (1, 0), (3, 3), (3, 0)]):
            x1 |= osel(lane, out) << (5 * k)                 # uio0..uio5
        chip.wr(rtlsim.XOUT1, x1)
        chip.wr(rtlsim.XOUT2, osel(2, 0) | osel(2, 3) << 5)  # listener -> uio6, uio7
        chip.wr(rtlsim.XIN0, 9 | 8 << 4 | 11 << 16 | 10 << 20)   # in0 = sda, in1 = scl (own pins)
        chip.wr(rtlsim.XIN1, (9 | 8 << 4) | (13 | 12 << 4) << 16)
        chip.net(SCL_NET)
        chip.net(SDA_NET)
        chip.skew(4, SKEW)          # target 0x51 reads scl late: sda edges "while scl high"
        chip.start_lane(0, ORIGIN["controller"])
        chip.run(100)
        for lane, name in ((1, "target"), (2, "listener"), (3, "target51")):
            chip.start_lane(lane, ORIGIN[name])
        chip.run(300)
        chip.dump()
        queues: dict = {k: [] for k in range(4)}
        for n, (cw, tw, frames) in enumerate(ALL):
            bits = sum(9 + 9 * len(f[3]) for f in frames)
            if cw is WRITE[0]:                               # back to back with READ: t_buf at
                chip.push(1, READ[1])                        # its minimum
                chip.push(0, WRITE[0] + READ[0])
                bits += sum(9 + 9 * len(f[3]) for f in READ[2])
            elif cw is READ[0]:
                continue
            elif cw is STRETCHED[0]:
                chip.push(0, cw)                             # the target's host is late:
                chip.run(11 * clock + STRETCH)               # it holds scl after its ack
                chip.push(1, tw)
            else:
                chip.push(1, tw)
                chip.push(0, cw)
            chip.run((bits + 12) * clock + 4000)
            for k in range(4):                   # the host drains (a listener fills 64 words fast)
                queues[k] += chip.drain(k)
        rows = chip.dump(full=True)
        ctrl = [chip.rd(rtlsim.lane_reg(k, rtlsim.CTRL)) for k in range(4)]
    return rows, queues, ctrl


def decode(rows):
    """The bus from the net values: frames (addr, rw, ack, bytes, end) and
    the timing of every START, STOP, scl edge and sda change."""
    evs, prev = [], None
    for cyc, _, _, _, _, uin in rows:
        scl, sda = uin & 1, (uin >> 1) & 1
        if prev is not None and (scl, sda) != prev:
            evs.append((cyc, prev, (scl, sda)))
        prev = (scl, sda)
    frames, cur, t = [], None, {"low": [], "high": [], "su_dat": [], "hd_dat": [], "su_sta": [],
                                "hd_sta": [], "su_sto": [], "buf": [], "sda_while_high": 0}
    last_rise = last_fall = last_sda = last_stop = start_at = None
    for cyc, (s0, d0), (s1, d1) in evs:
        if s0 == 1 and s1 == 1 and d0 != d1:                 # sda moves while scl high: an event
            if d1 == 0:
                if cur is not None:
                    cur["end"] = "start"
                    frames.append(cur)
                if last_rise is not None and last_stop is None:
                    t["su_sta"].append(cyc - last_rise)
                if last_stop is not None:
                    t["buf"].append(cyc - last_stop)
                cur, start_at, last_stop = {"bits": [], "rises": []}, cyc, None
            else:
                if cur is not None:
                    cur["end"] = "stop"
                    frames.append(cur)
                    cur = None
                t["su_sto"].append(cyc - last_rise)
                last_stop = cyc
            continue
        if s0 != s1:
            if s1 == 1:                                      # scl rises: a bit
                if last_fall is not None:
                    t["low"].append(cyc - last_fall)
                if last_sda is not None and last_fall is not None and last_sda >= last_fall:
                    t["su_dat"].append(cyc - last_sda)
                last_rise = cyc
                if cur is not None:
                    cur["bits"].append(d1)
                    cur["rises"].append(cyc)
            else:
                if last_rise is not None and last_stop is None:
                    t["high"].append(cyc - last_rise)
                if start_at is not None:
                    t["hd_sta"].append(cyc - start_at)
                    start_at = None
                last_fall = cyc
                if d0 != d1:
                    last_sda = cyc
                    t["hd_dat"].append(0)
        elif d0 != d1:
            if s1 == 1:
                t["sda_while_high"] += 1
            last_sda = cyc
            if last_fall is not None:
                t["hd_dat"].append(cyc - last_fall)
    out = []
    for f in frames:
        b = f["bits"]
        addr = sum(x << (6 - i) for i, x in enumerate(b[:7]))
        nbytes = (len(b) - 9) // 9
        data = [(sum(x << (7 - i) for i, x in enumerate(b[9 + 9 * k:17 + 9 * k])), b[17 + 9 * k])
                for k in range(nbytes)]
        out.append({"frame": (addr, b[7], b[8], data, f["end"]), "rises": f["rises"]})
    return out, t, evs


@unittest.skipUnless(rtlsim.find_verilator(), "Verilator not installed")
class I2cOnRtl(unittest.TestCase):

    def run_rate(self, rate: str):
        comp = compile_bus(rate)
        tc = comp["controller"].manifest["timing"]
        rows, q, ctrl = run_bus(comp, tc["clock_cycles"])
        frames, t, evs = decode(rows)
        want = [f for _, _, fs in ALL for f in fs]
        self.assertEqual([f["frame"] for f in frames], want)
        # records: controller (read data, then the NACK error; it halts), targets, listener
        self.assertEqual(q[0], [0x04 << 24 | b for _, _, fs in ALL for f in fs if f[1] == R
                                for b, _ in f[3]] + [0x43000001])
        self.assertEqual(ctrl[0] & 1, 0, "the controller halts after the NACK (3.15)")
        self.assertEqual([c & 1 for c in ctrl[1:]], [1, 1, 1])

        def target(addr):
            out = []
            for f in want:
                out += [0x05 << 24, 0x01 << 24 | f[0], 0x02 << 24 | f[1]]
                if f[0] == addr and f[2] == 0:
                    out += [(0x04 if f[1] == W else 0x03) << 24 | (b if f[1] == W else a)
                            for b, a in f[3]]
                if f[4] == "stop":
                    out.append(0x06 << 24)
            return out

        self.assertEqual(q[1], target(0x50))
        self.assertEqual(q[3], target(0x51))
        lis = []
        for f in want:
            lis += [0x05 << 24, 0x01 << 24 | f[0], 0x02 << 24 | f[1]]
            if f[2]:
                lis.append(0x43000001)                       # error ack=1, then the frame ends
            for b, a in f[3]:
                lis.append(0x04 << 24 | b)
                if f[1] == R:
                    lis.append(0x03 << 24 | a)               # matched target acks are implied
            if f[4] == "stop":
                lis.append(0x06 << 24)
        ts, recs = q[2][0::2], q[2][1::2]
        self.assertEqual(recs, lis)
        # listener timestamps: values at the lane's last capture edge (pin + 3), one constant
        caps = []
        for f in frames:
            a, rw, ack, data, _ = f["frame"]
            r = f["rises"]
            caps += [r[6], r[7]]
            for k in range(len(data)):
                caps.append(r[16 + 9 * k])
                if rw == R:
                    caps.append(r[17 + 9 * k])
        vals = [x for x, rec in zip(ts, recs) if rec >> 24 in (1, 2, 3, 4)]
        offs = {x - c - 3 for x, c in zip(vals, caps)}
        self.assertEqual(len(offs), 1, offs)
        off = offs.pop()
        bounds = comp["listener"].manifest["out_queue"]["timestamps"]["event_late_cycles_after_pin_edge"]
        pins = [c for c, (s0, d0), (s1, d1) in evs if s0 == s1 == 1 and d0 != d1]
        lates = {"start": set(), "stop": set()}
        for x, pin in zip([x for x, rec in zip(ts, recs) if rec >> 24 in (5, 6)], pins):
            kind = "start" if recs[ts.index(x)] >> 24 == 5 else "stop"
            lates[kind].add(x - off - pin)
        for kind, got in lates.items():
            lo, hi = bounds[kind]
            self.assertTrue(got and all(lo <= g <= hi for g in got), (kind, got, bounds[kind]))
        # the description's rules on the pins
        rules = {k: v["bound_cycles"] for k, v in tc["rules"].items()}
        meas = {"t_low": min(t["low"]), "t_high": min(t["high"]), "t_su_dat": min(t["su_dat"]),
                "t_hd_dat": min(t["hd_dat"]), "t_su_sta": min(t["su_sta"]),
                "t_hd_sta": min(t["hd_sta"]), "t_su_sto": min(t["su_sto"]), "t_buf": min(t["buf"])}
        for name, got in meas.items():
            self.assertGreaterEqual(got, rules[name], name)
        self.assertEqual(t["sda_while_high"], 0, "sda moved while scl high outside START/STOP")
        # open drain: no bus pin ever driven high; the listener never enables an output
        self.assertFalse(any(uout & uoe & BUS for _, _, _, uout, uoe, _ in rows))
        self.assertFalse(any(uoe & LISTENER_PINS for _, _, _, _, uoe, _ in rows))
        # the stretch: the target holds scl (uio2 enabled) while the controller has let go (uio0)
        held = [c for c, _, _, _, uoe, uin in rows if uoe & 0x04 and not uoe & 0x01]
        runs, start, last = [], None, None
        for c, _, _, _, uoe, _ in rows:
            on = bool(uoe & 0x04 and not uoe & 0x01)
            if on and start is None:
                start = c
            if not on and start is not None:
                runs.append(c - start)
                start = None
        self.assertTrue(held and max(runs) >= STRETCH, runs)
        # what the 0x51 target saw: its scl SKEW cycles late; sda changes while that scl was high
        # (other than real START/STOP) would be STARTs or STOPs without the confirm window
        late = {}
        hist = []
        for c, _, _, _, _, uin in rows:
            hist.append((c, uin & 1, (uin >> 1) & 1))
        false_edges = 0
        for (c0, s0, d0), (c1, s1, d1) in zip(hist, hist[1:]):
            if d0 != d1 and s0 == 1 and s1 == 0:             # data moves as scl falls (t_hd_dat 0)
                false_edges += 1
        self.assertGreater(false_edges, 10)
        if REPORT:
            print(f"\n  i2c at {rate}: scl {tc['low_cycles']}+{tc['high_cycles']} cycles "
                  f"({tc['rate_hz']:g} Hz); frames {len(frames)}; controller {[hex(w) for w in q[0]]}"
                  f"\n    measured (cycles) vs rules: "
                  + ", ".join(f"{k} {v}/{rules[k]:g}" for k, v in meas.items())
                  + f"\n    longest stretch {max(runs)} cycles; sda changed as scl fell {false_edges} "
                    f"times (the 0x51 target, {SKEW} cycles late on scl, saw each as an edge while scl "
                    f"high; the confirm window rejected them all)"
                  + f"\n    listener events after the pin edge "
                    f"{ {k: sorted(v) for k, v in lates.items()} }")
        return comp

    def test_100khz(self):
        self.run_rate("100 kHz")

    def test_400khz(self):
        self.run_rate("400 kHz")

    def test_fastest(self):
        comp = compile_bus("400 kHz")
        top = min(c.manifest["timing"]["fastest_rate_hz"] for c in comp.values())
        self.assertEqual(top, 526316)                        # t_low + t_high: 78 + 36 cycles
        self.run_rate("526 kHz")
        with self.assertRaisesRegex(CompileError, "Fastest rate"):
            compile_file(I2C, "controller", {"scl_rate": "527 kHz"})


if __name__ == "__main__":
    unittest.main()
