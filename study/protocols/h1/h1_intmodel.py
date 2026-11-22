"""Integer-cycle encoder model (the one proposed for the H1 round-trip proof):
bit time 6 cycles, half bit 3, every rising edge displaced by a constant
d_r and every falling edge by a constant d_f cycles (|d| <= D), arbitrary
start offset, arbitrary data. Checks on the RTL prototype that, once locked,
the decoder emits exactly the encoded bits (no loss, no insertion).
Usage: h1_intmodel.py <exe> [D] [frames per combination]"""
import itertools
import random
import sys

import h1_measure as h


def levels(bits, dr, df, off):
    ev = []
    lvl = 1
    for k, b in enumerate(bits):
        first, second = 1 - b, b
        tb = off + 6 * k + 20
        if first != lvl:
            ev.append((tb + (dr if first else df), first)); lvl = first
        if second != lvl:
            ev.append((tb + 3 + (dr if second else df), second)); lvl = second
    if lvl == 0:
        ev.append((off + 6 * len(bits) + 20 + dr, 1))
    ev.sort()
    n = ev[-1][0] + 60
    out, e, cur = [], 0, 1
    for t in range(n):
        while e < len(ev) and ev[e][0] <= t:
            cur = ev[e][1]; e += 1
        out.append(cur)
    return out


def main():
    exe = sys.argv[1]
    D = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    n = int(sys.argv[3]) if len(sys.argv) > 3 else 20
    dut = h.Dut(exe)
    rng = random.Random(2)
    fails = 0
    tot = 0
    short = 0
    for dr, df in itertools.product(range(-D, D + 1), repeat=2):
        for _ in range(n):
            data = [rng.randrange(2) for _ in range(rng.choice([64, 200]))]
            if rng.random() < 0.5:                       # long runs and alternation
                data = [1] * 40 + [0] * 40 + [1, 0] * 40 + data
            pre = [1, 0] * 28 + [1, 0, 1, 0, 1, 0, 1, 1]
            npre = rng.randrange(20, 65)
            bits = pre[64 - npre:] + data
            b, _, _ = dut.run(levels(bits, dr, df, rng.randrange(6)))
            allb = "".join(map(str, bits))
            # property: once locked, every strobe is the next encoded bit, so the
            # output is exactly a suffix of the encoded stream (and the frame
            # itself is complete if lock came before the SFD)
            ok = len(b) > 0 and allb.endswith(b)
            locked_in_time = len(b) >= len(data) + 16
            short += not locked_in_time
            tot += 1
            if not ok:
                fails += 1
                if fails <= 3:
                    print("fail", dr, df, npre, len(b))
    print(f"D={D}: {tot - fails}/{tot} frames: output an exact suffix of the encoded bits "
          f"(all d_r, d_f in [-{D},{D}], start offsets 0-5, 20-64 preamble bits); "
          f"{short} locked too late for the SFD")
    dut.close()


if __name__ == "__main__":
    main()
