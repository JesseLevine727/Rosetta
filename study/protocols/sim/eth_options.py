"""Ethernet receive decoder options, modelled in Python on the same
continuous-time Manchester lines as the RTL campaign (eth_rx_margin.py).

Front ends (how edges become timestamps):
  se60   single edge, 60 MHz          (today's hardware)
  de60   both edges of 60 MHz, duty D (sample instants k*Tc and (k+D)*Tc)
  se66   single edge, 66.7 MHz        (the pad ceiling; for completeness)
  tap3   3 equal phases of 60 MHz     (delay taps, ideal)
  ro300  free-running ring oscillator, f unknown in 200..400 MHz
Decoders:
  cls    the lane's interval classifier (coding.md 3.2): interval >= THR
         is mid-bit, two shorts toggle the phase
  dpll   phase-tracking: predicted mid-bit instant, window +-P/4, phase
         update gain 1/8 on accepted mid-bit edges; acquisition by the
         first long interval (the preamble has only mid-bit edges)

For each (front end, decoder, impairment) the script finds the largest
impairment for which every one of N random frames (random phase, frequency
offset uniform in +-100 ppm) decodes bit-exact after the SFD.

Usage: python3 eth_options.py [frames]
"""
from __future__ import annotations

import json
import math
import os
import random
import sys

import ethlib
from ethlib import TC

HERE = os.path.dirname(os.path.abspath(__file__))


# ---------------------------------------------------------------- front ends
def stamps(edges, fe, rng_fe):
    """Continuous edges -> (timestamps in decoder units, levels, unit_ns)."""
    kind = fe["kind"]
    if kind == "grid":                     # uniform sampling grid, resolution r
        r = fe["r"]
        off = rng_fe.uniform(0, r)
        return _grid(edges, [off], r), r
    if kind == "de":                       # two sample instants per period
        d = fe["duty"]
        inst = [0.0, d * TC]
        out = []
        for t, lv in edges:
            k = math.floor(t / TC)
            cands = [k * TC + inst[0], k * TC + inst[1], (k + 1) * TC + inst[0]]
            idx = [2 * k, 2 * k + 1, 2 * k + 2]
            for c, i in zip(cands, idx):
                if c >= t:
                    out.append((i, lv))
                    break
        return _merge(out), TC / 2
    if kind == "ro":                       # ring oscillator with unknown frequency
        f = rng_fe.uniform(fe["fmin"], fe["fmax"])
        r = 1000.0 / f
        off = rng_fe.uniform(0, r)
        return _grid(edges, [off], r), r
    raise ValueError(kind)


def _grid(edges, offs, r):
    out = []
    for t, lv in edges:
        out.append((math.ceil((t - offs[0]) / r - 1e-12), lv))
    return _merge(out)


def _merge(out):
    """Two edges on one sample: the line toggled twice between samples and
    the synchroniser sees neither."""
    res = []
    for k, lv in out:
        if res and res[-1][0] == k:
            res.pop()
            continue
        res.append((k, lv))
    return res


# ------------------------------------------------------------------ decoders
def dec_cls(st, unit_ns, thr_ns=75.0, thr_units=None):
    thr = thr_units if thr_units is not None else thr_ns / unit_ns
    bits, mph, prev = [], 1, None
    for k, lv in st:
        d = k - prev if prev is not None else 1e9
        prev = k
        ge0 = d >= thr
        if ge0 or not mph:
            bits.append(lv)
        mph = 1 if ge0 else (0 if mph else 1)
    return bits


def dec_dpll(st, unit_ns, period_units=None, learn=False, frac_bits=8, polar=True):
    """Phase-tracking decoder (a model of a hardware DPLL; fixed point with
    frac_bits fractional bits). m = predicted next mid-bit instant, P = bit
    period in sample units, o = duty-cycle offset (rising mid-bit edges are
    expected at m - o, falling ones at m + o; polar=False keeps o = 0).
    Acquisition on the preamble (only mid-bit edges there): the first
    interval of at least 3/4 bit starts it; the next 16 mid-bit edges are
    accepted in a wide window (+-3P/8) with phase gains 1/2 (8 edges), 1/8
    (16), 1/32 (16), then a +-P/4 window and gain 1/64 (offset gains 0,
    1/16, 1/32, 1/64). With `learn` (oscillator of unknown frequency) P is first
    measured as the mean of 16 preamble intervals."""
    q = 1 << frac_bits
    P = round((period_units if period_units else 100.0 / unit_ns) * q)
    bits, m, prev, learn_iv, miss, nacc, o = [], None, None, [], 0, 0, 0
    for k, lv in st:
        kq = k * q
        if m is None:
            if prev is not None and (kq - prev) >= (3 * P) // 4:
                if learn:
                    learn_iv.append(kq - prev)
                    if len(learn_iv) < 16:
                        prev = kq
                        continue
                    P = round(sum(learn_iv[-16:]) / 16)
                # this edge is mid-bit; its polarity is lv
                m, nacc, miss, o = kq + P + (o if lv else -o), 0, 0, 0
                m = kq + P
                bits.append(lv)
                prev = kq
                continue
            prev = kq
            continue
        prev = kq
        while kq > m + P // 2:             # a bit went by without a mid-bit edge
            m += P
            miss += 1
            bits.append(None)
        if miss >= 3:                      # lost: re-acquire
            m = None
            continue
        if nacc < 8:
            win, g, go = (3 * P) // 8, 0.5, 0.0
        elif nacc < 24:
            win, g, go = (3 * P) // 8, 0.125, 0.0625
        elif nacc < 40:
            win, g, go = (3 * P) // 8, 1 / 32, 1 / 32
        else:
            win, g, go = P // 4, 1 / 64, 1 / 64
        exp = m - o if lv else m + o       # rising edges at m - o, falling at m + o
        err = kq - exp
        if abs(err) <= win:
            bits.append(lv)
            m = m + P + round(err * g)      # rounded: no bias
            if polar:
                o = o + round((-err if lv else err) * go)
            nacc += 1
            miss = 0
    return bits


SFD_TAIL = [1, 0] * 7 + [1, 1]


def check(bits, frame):
    want = ethlib.byte_bits(frame)
    for i in range(len(bits) - 16):
        if bits[i:i + 16] == SFD_TAIL:
            got = bits[i + 16:i + 16 + len(want)]
            return got == want
    return False


FRONT_ENDS = {
    "se60": {"kind": "grid", "r": TC},
    "de60_d50": {"kind": "de", "duty": 0.50},
    "de60_d45": {"kind": "de", "duty": 0.45},
    "se66": {"kind": "grid", "r": 15.0},
    "tap3": {"kind": "grid", "r": TC / 3},
    "tap4": {"kind": "grid", "r": TC / 4},
    "ro300": {"kind": "ro", "fmin": 200.0, "fmax": 400.0},
}

DECODERS = {
    "cls": lambda st, u, fe: dec_cls(st, u, thr_units=(5 if fe == "se60" else
                                                      9 if fe.startswith("de60") else
                                                      5 if fe == "se66" else
                                                      14 if fe == "tap3" else
                                                      18 if fe == "tap4" else None)),
    "dpll": lambda st, u, fe: dec_dpll(st, u, learn=(fe == "ro300")),
    "dpll1": lambda st, u, fe: dec_dpll(st, u, learn=(fe == "ro300"), polar=False),
}


def trial(rng, fe, dec, mode, val, plen=46):
    frame = ethlib.make_frame(rng, plen)
    ppm = rng.uniform(-100, 100)
    _, edges = ethlib.frame_wave(frame, rng, ppm=ppm, mode=mode,
                                 jitter=val if mode != "none" else 0.0,
                                 dcd=val if mode == "none" else 0.0)
    st, unit = stamps(edges, FRONT_ENDS[fe], rng)
    if fe == "ro300" and dec == "cls":
        # a classifier needs the threshold in oscillator cycles: calibrate
        # (count oscillator cycles per 60 MHz cycle) -> exact unit known here
        bits = dec_cls(st, unit)
    else:
        bits = DECODERS[dec](st, unit, fe)
    return check(bits, frame)


def limit(fe, dec, mode, n, rng, hi=40.0):
    """Largest impairment (0.25 ns steps) with no failure in n frames, by
    scanning upward until the first failing point."""
    v = 0.0
    last_ok = None
    step = 0.5
    while v <= hi:
        ok = all(trial(rng, fe, dec, mode, v) for _ in range(n))
        if not ok:
            return last_ok, v
        last_ok = v
        v += step
    return last_ok, None


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 100
    rng = random.Random(7)
    out = []
    for fe in FRONT_ENDS:
        for dec in ("cls", "dpll1", "dpll"):
            row = {"front_end": fe, "decoder": dec}
            for mode in ("alt", "uniform", "none"):
                ok, bad = limit(fe, dec, mode, n, rng)
                row[mode] = (ok, bad)
            out.append(row)
            print(f"{fe:9s} {dec:5s} alt(worst) ok<={row['alt'][0]} fail@{row['alt'][1]} | "
                  f"uniform ok<={row['uniform'][0]} fail@{row['uniform'][1]} | "
                  f"dcd ok<={row['none'][0]} fail@{row['none'][1]}", flush=True)
    with open(os.path.join(HERE, "results", "eth_options.json"), "w") as f:
        json.dump({"frames_per_point": n, "rows": out}, f, indent=1)


if __name__ == "__main__":
    main()
