"""Pattern-dependent jitter from a band-limited channel (study): a Manchester
line through a single-pole low-pass (time constant tau), sliced at zero.
Prints the longest 'short' and the shortest 'long' interval between zero
crossings, and the largest deviation of a crossing from its ideal instant,
for random data. Shows which way inter-symbol interference moves the
intervals the lane's classifier looks at. Usage: python3 eth_isi.py"""
import math
import random

import ethlib


def crossings(bits, tau, dt=0.05):
    # piecewise-constant +-1 drive, exact exponential response, crossing by interpolation
    seg = []
    for b in bits:
        seg += [(-1 if b else 1), (1 if b else -1)]        # 1 = low then high
    v, t, out = -1.0, 0.0, []
    for k, s in enumerate(seg):
        t0 = k * 50.0
        # v(t) = s + (v0 - s) exp(-(t - t0)/tau); crossing if sign changes within 50 ns
        if (v > 0) != (s > 0):
            tc = t0 + tau * math.log((s - v) / s)
            if tc - t0 < 50.0:
                out.append((tc, k))
        v = s + (v - s) * math.exp(-50.0 / tau)
    return out


rng = random.Random(1)
bits = [1, 0] * 32 + [rng.randrange(2) for _ in range(4000)]
print("tau_ns  max_short_ns  min_long_ns  max_edge_dev_from_ideal_ns  (classifier needs short<=66.7, long>=83.3)")
for tau in (5, 10, 20, 30, 40, 60):
    cr = crossings(bits, tau)[200:]        # skip the start-up transient
    iv = [(b[0] - a[0], b[1] - a[1]) for a, b in zip(cr, cr[1:])]
    shorts = [d for d, n in iv if n == 1]
    longs = [d for d, n in iv if n == 2]
    devs = [tc - k * 50.0 for tc, k in cr]
    mean = sum(devs) / len(devs)
    print(f"{tau:6}  {max(shorts):12.2f}  {min(longs):11.2f}  {max(abs(d - mean) for d in devs):10.2f}")
