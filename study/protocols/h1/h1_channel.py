"""Band-limited channel (single-pole low-pass, time constant tau, sliced at
zero) through the H1 prototype and through the present classifier (study).
Reports, per tau, frames decoded out of N and the largest edge deviation from
the ideal grid that the channel produced. Usage: h1_channel.py <exe> [N]"""
import math
import random
import sys

import h1_measure as h

sys.path.insert(0, h.os.path.join(h.HERE, "..", "sim"))
import ethlib  # noqa: E402


def rc_edges(bits, tau, t0, bit_ns):
    seg = []
    for b in bits:
        seg += [(-1 if b else 1), (1 if b else -1)]
    half = bit_ns / 2
    v, out = 1.0, []                  # line idle high before the frame
    lvl = 1
    for k, s in enumerate(seg):
        ts = t0 + k * half
        if (v > 0) != (s > 0):
            tc = ts + tau * math.log((s - v) / s)
            if tc - ts < half:
                lvl = 1 if s > 0 else 0
                out.append((tc, lvl))
        v = s + (v - s) * math.exp(-half / tau)
    return out


def main():
    exe = sys.argv[1]
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 40
    dut = h.Dut(exe)
    rng = random.Random(3)
    print("tau_ns  max_dev_ns  H1_ok  classifier_ok  (of", n, ")")
    for tau in (10, 20, 30, 40, 50, 60, 70, 80):
        okh = okc = 0
        dev = 0.0
        for _ in range(n):
            frame = ethlib.make_frame(rng, 46)
            bits = ethlib.byte_bits(ethlib.PREAMBLE_SFD + frame)
            bit_ns = ethlib.BIT / (1 + rng.uniform(-100, 100) * 1e-6)
            t0 = 1500.0 + rng.uniform(0, ethlib.TC)
            edges = rc_edges(bits, tau, t0, bit_ns)
            ideal, _ = ethlib.manchester_edges(bits, t0, bit_ns)
            if len(ideal) == len(edges):          # (a very slow channel can swallow a pulse)
                ds = [t - i for (t, _), (i, _) in zip(edges, ideal)]
                mean = sum(ds[100:]) / len(ds[100:])
                dev = max(dev, max(abs(d - mean) for d in ds[100:]))
            t_end = t0 + len(bits) * bit_ns
            if edges[-1][1] == 0:
                edges.append((t_end, 1))
            lv = ethlib.sample_levels(edges, t_end + 1500.0)
            b, _, _ = dut.run(lv)
            okh += h.check_bits(b, frame)
            okc += h.check_cls(edges, frame)
        print(f"{tau:6}  {dev:9.2f}  {okh:5}  {okc:5}", flush=True)
    dut.close()


if __name__ == "__main__":
    main()
