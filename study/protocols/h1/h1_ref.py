"""Executable restatement of the H1 rules exactly as written in the
amendment (docs/blocks/amendment-eth-can.md section 1.6), for P = 6
prescaled cycles, u = 1/32 prescaled cycle, prescale 1. Checked cycle by
cycle against the RTL prototype (rosetta_mdpll_min2, PFIX=6 PI=3 F=5) and
used to print the worked example. Usage: h1_ref.py <exe> [frames]"""
import random
import sys

import h1_measure as h
import ethlib

P, WACQ, WTRK, ALO, LATE, OMAX, OMIN, NACQ = 192, 72, 48, 120, 96, 127, -128, 8
U, A, L = 0, 1, 2


def sat10(x):
    x &= 0x3FF
    return x - 0x400 if x & 0x200 else x


def run(levels, trace=None):
    st, ph, o, n, miss = U, 0, 0, 0, 0
    prev = 1
    out = []
    for t, lv in enumerate(levels):
        dedge = lv != prev
        prev = lv
        inc = 32
        ph_inc = sat10(ph + inc)
        base = sat10(ph_inc - P)
        err = sat10(base + o) if lv else sat10(base - o)
        win = WACQ if st == A else WTRK
        acc = -win <= err <= win
        st_o = False
        nst, nph, no, nn, nmiss = st, ph_inc, o, n, miss
        if st == U:
            if dedge:
                nph = 0
                if ph >= ALO:
                    nst, no, nn, nmiss = A, 0, 0, 0
            elif ph >= ALO:
                nph = ph
        elif dedge and acc:
            if st == A:
                nph = base >> 1
            else:
                nph = base - 1 if err > 0 else base + 1 if err < 0 else base
            if st == L:
                if (lv and err < 0) or (not lv and err > 0):
                    no = min(o + 1, OMAX)
                elif (lv and err > 0) or (not lv and err < 0):
                    no = max(o - 1, OMIN)
            nn = min(n + 1, NACQ)
            nmiss = 0
            st_o = st == L
            if st == A and n + 1 == NACQ:
                nst = L
        elif base >= LATE:
            nph, nmiss = base, 1
            if st == A or miss:
                nst = U
        if trace is not None and (dedge or nst != st):
            trace.append((t, lv, int(dedge), "UAL"[st], ph, err if st != U else None,
                          int(acc) if st != U and dedge else None, int(st_o), "UAL"[nst], nph, no))
        if st_o:
            out.append((t, lv))
        st, ph, o, n, miss = nst, nph, no, nn, nmiss
    return out


def main():
    exe = sys.argv[1]
    nfr = int(sys.argv[2]) if len(sys.argv) > 2 else 200
    dut = h.Dut(exe)
    rng = random.Random(9)
    same = 0
    for k in range(nfr):
        frame = ethlib.make_frame(rng, 46)
        mode = rng.choice(["none", "dcd", "uniform", "alt", "isi-"])
        lv, _ = h.frame_levels(frame, rng, mode, rng.uniform(0, 20), rng.uniform(-100, 100))
        b, first, last = dut.run([1] * 60 + lv)       # idle lead-in (the harness keeps state between calls)
        ref = run([1] * 60 + lv)
        rb = "".join(str(x) for _, x in ref)
        if rb == b and (not ref or (ref[0][0] == first and ref[-1][0] == last)):
            same += 1
        else:
            print("mismatch", k, mode, len(rb), len(b), ref[:1], first)
    print(f"{same}/{nfr} frames: reference rules == RTL prototype (bits and strobe cycles)")
    dut.close()


if __name__ == "__main__":
    main()
