"""Tolerance of the H1 prototype (rosetta_mdpll.sv) under Verilator, with the
continuous-time Manchester generator of the protocol study
(../sim/ethlib.py), compared with the present lane decoder (the PULSE-mode
interval classifier: the Python model that reproduced the chip RTL limits
exactly in ../sim/results/eth_rx_margin.log).

Impairments (A = amplitude in ns):
  dcd      high pulses longer by A, low pulses shorter by A
  alt      +A / -A on alternate edges (equals DCD of 2A)
  uniform  independent, uniform in [-A, A] per edge
  gauss    independent, Gaussian, sigma A
  isi+ / isi-   each crossing displaced +A (-A) if the pulse before it was
           long, -A (+A) if short: the data-dependent pattern; with long runs
           of equal bits it is a phase step of 2A between runs
  step<B>  +A for B bit times, then -A for B bit times, ... (wander)
Every crossing stays within A of its ideal instant, so A = 13.5 is the
802.3 receiver requirement (narrow pulses 23-77 ns, wide 73-127 ns).
Payloads: 'rand' random bytes; 'runs' blocks of 0x00, 0xFF, 0x55, 0xAA,
0x0F, 0x33 (long runs of equal bits and alternating bits).

Usage: python3 h1_measure.py <variant name> <executable> [frames]
"""
from __future__ import annotations

import json
import math
import os
import random
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))
import ethlib  # noqa: E402
import eth_options  # noqa: E402

TC = ethlib.TC


class Dut:
    def __init__(self, exe):
        self.p = subprocess.Popen([exe], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)

    def run(self, levels):
        self.p.stdin.write("F " + "".join("1" if v else "0" for v in levels) + "\n")
        self.p.stdin.flush()
        b, first, last = self.p.stdout.readline().split()
        return ("" if b == "-" else b), int(first), int(last)

    def close(self):
        self.p.stdin.write("Q\n")
        self.p.stdin.flush()
        self.p.wait()


def payload(rng, kind, n=46):
    if kind == "rand":
        return bytes(rng.randrange(256) for _ in range(n))
    blocks = [0x00, 0xFF, 0x55, 0xAA, 0x0F, 0x33]
    rng.shuffle(blocks)
    out = b""
    for b in blocks:
        out += bytes([b]) * 12
    return out[:max(n, 72)]


def displace(edges, rng, mode, a, bit_ns, t0):
    """Apply a deterministic or random displacement to ideal edges."""
    out = []
    prev_t = None
    for k, (t, lv) in enumerate(edges):
        if mode == "dcd":
            d = -a / 2 if lv == 1 else a / 2
        elif mode == "alt":
            d = a if k % 2 == 0 else -a
        elif mode == "uniform":
            d = rng.uniform(-a, a)
        elif mode == "gauss":
            d = rng.gauss(0.0, a)
        elif mode in ("isi+", "isi-"):
            longp = prev_t is not None and (t - prev_t) > 0.75 * bit_ns
            s = 1 if mode == "isi+" else -1
            d = s * (a if longp else -a)
        elif mode.startswith("step"):
            B = int(mode[4:])
            blk = int((t - t0) / bit_ns) // B
            d = a if blk % 2 == 0 else -a
        else:
            d = 0.0
        prev_t = t
        out.append((t + d, lv))
    out.sort()
    return out


def frame_levels(frame, rng, mode, a, ppm, npre=64, lead_ns=1500.0, tail_ns=1200.0, tp_idl_ns=300.0):
    """Line levels per cycle: idle high, the last `npre` preamble+SFD bits
    (the SFD is the last 8), the frame, TP_IDL, idle. Random phase."""
    bit_ns = ethlib.BIT / (1.0 + ppm * 1e-6)
    pre = ethlib.byte_bits(ethlib.PREAMBLE_SFD)[64 - npre:]
    bits = pre + ethlib.byte_bits(frame)
    phase = rng.uniform(0, TC)
    t0 = lead_ns + phase
    edges, last = ethlib.manchester_edges(bits, t0, bit_ns)
    t_end_bits = t0 + len(bits) * bit_ns
    if last == 0:
        edges.append((t_end_bits, 1))
    edges = displace(edges, rng, mode, a, bit_ns, t0)
    lv = ethlib.sample_levels(edges, t_end_bits + tp_idl_ns + tail_ns)
    return lv, edges


SFD_TAIL = "1010101010101011"


def check_bits(bits, frame):
    want = "".join(str(b) for b in ethlib.byte_bits(frame))
    i = bits.find(SFD_TAIL)
    return i >= 0 and bits[i + 16:i + 16 + len(want)] == want and len(bits) == i + 16 + len(want)


def check_cls(edges, frame):
    st, unit = eth_options.stamps(edges, eth_options.FRONT_ENDS["se60"], random.Random(0))
    bits = eth_options.dec_cls(st, unit, thr_units=5)
    return eth_options.check(bits, frame)


def trial(dut, rng, mode, a, kind, npre=64):
    frame = ethlib.make_frame(rng, 46) if kind == "rand" else (lambda b: b + ethlib.fcs_bytes(b))(
        bytes([2, 0, 0, 0, 0, 1, 2, 0, 0, 0, 0, 2, 8, 0]) + payload(rng, kind))
    lv, edges = frame_levels(frame, rng, mode, a, rng.uniform(-100, 100), npre=npre)
    bits, first, last = dut.run(lv)
    ok_h1 = check_bits(bits, frame)
    ok_cls = check_cls(edges, frame) if npre == 64 else None
    return ok_h1, ok_cls


def limit(dut, rng, mode, kind, n, step=0.5, hi=40.0, which=0):
    a, last_ok = 0.0, None
    while a <= hi:
        ok = True
        for _ in range(n):
            r = trial(dut, rng, mode, a, kind)
            if not r[which]:
                ok = False
                break
        if not ok:
            return last_ok, a
        last_ok = a
        a += step
    return last_ok, None


def main():
    name, exe = sys.argv[1], sys.argv[2]
    n = int(sys.argv[3]) if len(sys.argv) > 3 else 40
    rng = random.Random(21)
    dut = Dut(exe)
    res = {"variant": name, "frames_per_step": n, "limits": {}}
    modes = ["dcd", "alt", "uniform", "gauss", "isi+", "isi-", "step8", "step32", "step128"]
    for mode in modes:
        for kind in ("rand", "runs"):
            if mode in ("uniform", "gauss") and kind == "runs":
                continue
            h1 = limit(dut, rng, mode, kind, n, which=0)
            row = {"h1": h1}
            if name == "base":
                row["classifier"] = limit(dut, rng, mode, kind, n, which=1)
            res["limits"][f"{mode}/{kind}"] = row
            print(f"{name:8s} {mode:8s} {kind:5s} H1 ok<={h1[0]} fail@{h1[1]}"
                  + (f" | classifier ok<={row['classifier'][0]} fail@{row['classifier'][1]}" if "classifier" in row else ""),
                  flush=True)
    # 802.3 point: every pattern at A = 13.5 ns, 200 frames each
    req = {}
    for mode in ["dcd", "alt", "isi+", "isi-", "step8", "step32", "step128", "uniform"]:
        for kind in ("rand", "runs"):
            if mode == "uniform" and kind == "runs":
                continue
            a = 27.0 if mode == "dcd" else 13.5      # DCD of 27 ns = +-13.5 ns per crossing
            fails = sum(1 for _ in range(200) if not trial(dut, rng, mode, a, kind)[0])
            req[f"{mode}/{kind}"] = fails
    res["at_13p5ns_fails_of_200"] = req
    print("802.3 point (A=13.5 ns per crossing; dcd = 27 ns):", req, flush=True)
    # acquisition: fewest preamble+SFD bits (from the end) that still work, clean and at 13.5 ns DCD-type
    acq = {}
    for mode, a in (("none", 0.0), ("dcd", 27.0), ("uniform", 8.0)):
        best = None
        for npre in range(64, 8, -1):
            ok = all(trial(dut, rng, mode, a, "rand", npre=npre)[0] for _ in range(60))
            if ok:
                best = npre
            else:
                break
        acq[f"{mode}@{a}"] = best
    res["min_preamble_bits_incl_sfd"] = acq
    print("acquisition: fewest preamble+SFD bits that decode in 60/60 frames:", acq, flush=True)
    # link pulse and idle: no bits
    lp = []
    for _ in range(50):
        t = rng.uniform(0, TC) + 2000.0
        edges = [(t, 0), (t + 100.0, 1)]                 # idle high, a 100 ns negative-going pulse
        lv = ethlib.sample_levels(edges, t + 3000.0)
        b1, _, _ = dut.run(lv)
        edges = [(t, 1), (t + 100.0, 0)]
        lv = ethlib.sample_levels(edges, t + 3000.0, start_level=0)
        b2, _, _ = dut.run(lv)
        lp.append(len(b1) + len(b2))
    res["link_pulse_bits"] = sum(lp)
    print("bits produced by 100 link pulses (both polarities):", sum(lp), flush=True)
    dut.close()
    with open(os.path.join(HERE, "results", f"h1_{name}.json"), "w") as f:
        json.dump(res, f, indent=1)


if __name__ == "__main__":
    main()
