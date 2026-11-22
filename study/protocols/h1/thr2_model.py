"""Cheap alternative to H1 (model only, study): the present interval
classifier with a second threshold for pulses of the other level (the
deleted THR1, ruling 23, reused): a pulse ended by a falling edge (a high
pulse) is long iff its interval >= THR0, one ended by a rising edge iff
>= THR1. Static thresholds chosen for a known duty-cycle distortion.
Uses the classifier model that reproduced the chip RTL limits."""
import random
import sys
sys.path.insert(0, "../sim")
import ethlib
import eth_options as eo
import h1_measure as h


def dec2(st, thr_hi, thr_lo):
    bits, mph, prev = [], 1, None
    for k, lv in st:
        d = k - prev if prev is not None else 10 ** 6
        prev = k
        ge0 = d >= (thr_hi if lv == 0 else thr_lo)      # lv = level after the edge
        if ge0 or not mph:
            bits.append(lv)
        mph = 1 if ge0 else (0 if mph else 1)
    return bits


def run(dcd, thr_hi, thr_lo, mode="none", a=0.0, n=60, seed=1):
    rng = random.Random(seed)
    ok = 0
    for _ in range(n):
        fr = ethlib.make_frame(rng, 46)
        lv, edges = h.frame_levels(fr, rng, "dcd", dcd, rng.uniform(-100, 100))
        if a:
            edges = h.displace(edges, rng, mode, a, 100.0, 0.0)
        st, unit = eo.stamps(edges, eo.FRONT_ENDS["se60"], rng)
        ok += eo.check(dec2(st, thr_hi, thr_lo), fr)
    return ok


for dcd in (0, 10, 16.5, 20, 27, 33):
    grid = [(run(dcd, th, tl, "uniform", 4.0, 30, 2), th, tl) for th in range(3, 9) for tl in range(2, 7)]
    top = max(g[0] for g in grid)
    cands = [g for g in grid if g[0] == top]
    _, th, tl = cands[len(cands) // 2]
    lim = None
    for a in (2, 4, 6, 8, 9):
        if run(dcd, th, tl, "uniform", a, 30, 3) == 30:
            lim = a
        else:
            break
    print(f"DCD {dcd:5.1f} ns: thresholds high={th} low={tl}: clean {run(dcd, th, tl, n=30)}/30, "
          f"uniform jitter passing 30/30 up to {lim} ns", flush=True)
