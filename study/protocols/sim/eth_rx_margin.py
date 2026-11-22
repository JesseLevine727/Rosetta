"""Ethernet 10BASE-T receive margin on the real RTL (study, not a test).

Lane 0 runs eth_rx.asm. The harness drives ui[3] (lane 0 in0) with a
Manchester line computed in continuous time: random phase against the
60 MHz clock per frame, a frequency offset, per-edge jitter and duty-cycle
distortion, then sampled at each clock edge exactly as the first
synchroniser flop sees it. After each frame the host drains the status
records and the frame buffer and checks every byte and the CRC verdict.

Usage: python3 eth_rx_margin.py [frames_per_point] [payload_len]
Writes results to eth_rx_margin.json next to this file.
"""
from __future__ import annotations

import json
import os
import random
import sys
import time

import ethlib
import studysim
from studysim import asm, XIN0

HERE = os.path.dirname(os.path.abspath(__file__))


def setup(chip):
    with open(os.path.join(HERE, "eth_rx.asm")) as f:
        prog = asm.assemble(f.read())
    chip.load_program(prog.words)
    chip.wr(XIN0, 3)                     # lane 0 in0 <- ui[3]
    chip.map(0, 3)                       # wave channel 0 drives ui[3]
    chip.pull(3, 1)
    chip.tracing(False)
    chip.start_lane(0, 0, qcfg=0)        # 8-word queues: buffer = words 8..119
    chip.run(200)
    return prog


def decode_records(recs):
    out = {}
    for w in recs:
        out.setdefault(w >> 24, []).append(w & 0xFFFFFF)
    return out


def rev32(x):
    return int(f"{x:032b}"[::-1], 2)


def receive(chip, frame: bytes, rng, **imp):
    levels, edges = ethlib.frame_wave(frame, rng, **imp)
    chip.wave(studysim.runs_from_levels(levels, 1))
    chip.play(400)
    recs = chip.drain(0, out_size=8)
    r = decode_records(recs)
    if not all(k in r for k in (0xE1, 0xE2, 0xE3, 0xEE)) or len(recs) != 4:
        return {"ok": False, "why": f"records {[hex(x) for x in recs]}", "edges": edges}
    nxt = r[0xE1][0] & 0xFFFF
    nwords = nxt - 8
    sr = ((r[0xE2][0] & 0xFFFF) << 16) | (r[0xE3][0] & 0xFFFF)
    crc_good = r[0xEE][0] & 1
    if not 0 <= nwords <= 112:
        return {"ok": False, "why": f"word count {nwords}", "crc": crc_good, "edges": edges}
    words = chip.mem_read(0, 8, nwords) if nwords else []
    data = b"".join(rev32(w).to_bytes(4, "little") for w in words)
    k = sr.bit_length() - 1              # marker position = number of partial bits
    if k > 0:
        part = int(f"{sr & ((1 << k) - 1):0{k}b}"[::-1], 2)
        data += part.to_bytes((k + 7) // 8, "little")[: k // 8]
    ok = crc_good == 1 and data == frame and k % 8 == 0
    return {"ok": ok, "crc": crc_good, "match": data == frame, "len": len(data), "k": k,
            "edges": edges}


def run_point(chip, rng, n, payload_len, ppm_mode, **imp):
    fails = 0
    first = None
    for i in range(n):
        ppm = rng.choice([-100.0, 100.0]) if ppm_mode == "extreme" else rng.uniform(-100, 100)
        frame = ethlib.make_frame(rng, payload_len)
        res = receive(chip, frame, rng, ppm=ppm, **imp)
        if not res["ok"]:
            fails += 1
            if first is None:
                first = {k: v for k, v in res.items() if k != "edges"}
    return fails, first


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 60
    plen = int(sys.argv[2]) if len(sys.argv) > 2 else 46
    rng = random.Random(20261121)
    results = {"frames_per_point": n, "payload_len": plen, "frame_bytes": plen + 18, "points": []}
    t_start = time.time()
    with studysim.Chip() as chip:
        setup(chip)
        # sanity: clean frames, odd lengths (partial last word)
        for L in (46, 47, 49, 50, 51):
            fr = ethlib.make_frame(rng, L)
            res = receive(chip, fr, rng, ppm=rng.uniform(-100, 100))
            results.setdefault("sanity", []).append({"payload": L, "ok": res["ok"], "k": res.get("k")})
        sweeps = [("uniform", "jitter", [0, 2, 4, 6, 7, 8, 9, 10, 11, 12, 14]),
                  ("alt", "jitter", [0, 4, 6, 7, 7.5, 8, 8.2, 8.5, 9, 10]),
                  ("none", "dcd", [0, 8, 12, 14, 15, 16, 16.5, 17, 18, 20]),
                  ("gauss", "jitter", [1, 2, 2.5, 3, 3.5, 4, 5])]
        for mode, kind, vals in sweeps:
            for v in vals:
                imp = {"mode": mode, "jitter": v if kind == "jitter" else 0.0,
                       "dcd": v if kind == "dcd" else 0.0}
                fails, first = run_point(chip, rng, n, plen, "uniform", **imp)
                pt = {"mode": mode, kind: v, "frames": n, "fails": fails, "first_fail": first}
                results["points"].append(pt)
                print(f"{mode:8s} {kind}={v:5.1f} ns: {fails}/{n} frames failed"
                      f"  ({time.time() - t_start:.0f} s)", flush=True)
    with open(os.path.join(HERE, "results", "eth_rx_margin.json"), "w") as f:
        json.dump(results, f, indent=1)
    print("sanity:", results["sanity"])


if __name__ == "__main__":
    main()
