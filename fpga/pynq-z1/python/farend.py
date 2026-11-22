"""Board-side far-end campaigns (run inside board_server.py: `farend NAME JSON`).

ethsweep: the 10BASE-T receive-margin sweep against a truly asynchronous
source. The study's receive program (lane 0, PULSE timing, Manchester, THR0 =
5) receives frames from async line 0 on ui[3]: every edge computed in
continuous time by the study's line model (protolines.frame_edges: random
payload, a frequency offset uniform in +-100 ppm per frame, then per-edge
jitter or duty-cycle distortion) and placed on a 78.125 ps grid by rh_line.v,
clocked from the board's 125 MHz oscillator, so each frame meets the 60 MHz
chip clock at an uncontrolled phase. A frame passes when every byte and the
CRC verdict come back right (the study's criterion, eth_rx_margin.py).

params: {"program": [words], "points": [[mode, value_ns], ...], "frames": n,
         "payload": 46, "seed": s, "dcd_comp": ns}
dcd_comp: the source's own duty-cycle distortion, subtracted from every
frame (board_tests.line_calibration_chip measures it).
mode: "dcd" (high pulses longer by value), "alt" (+-value per edge,
alternating), "uniform" (uniform in [-value, value] per edge), "none".
Returns {"points": [{mode, value, frames, fails, first_fail}], ...}.
"""
import random
import time

import protolines as pl

XIN0 = 0x40
LANE0_QOUT = 0x10 + 7
MADDR, MDATA = 0x01, 0x02


def _drain(ls, lane=0, out_size=8):
    q = ls.host_read(0x10 + 8 * lane + 7)
    wr_p, rd_p = (q >> 8) & 0x7F, q & 0x7F
    cnt = (wr_p - rd_p) & (2 * out_size - 1)
    base = 0x200 | (lane << 7) | (128 - out_size)
    words = []
    for k in range(cnt):
        ls.host_write(MADDR, base | ((rd_p + k) & (out_size - 1)))
        words.append(ls.host_read(MDATA))
    ls.host_write(0x10 + 8 * lane + 7, wr_p << 8 | wr_p)
    return words


def _mem_read(ls, lane, off, n):
    ls.host_write(MADDR, 0x200 | (lane << 7) | off)
    return [ls.host_read(MDATA) for _ in range(n)]


def _start_lane(ls, lane, pc, qcfg):
    r = lambda k: 0x10 + 8 * lane + k                  # noqa: E731
    for a, d in ((r(0), 0), (r(5), qcfg), (r(6), 0), (r(7), 0), (r(1), pc), (r(0), 1)):
        ls.host_write(a, d)


def eth_receive(ls, frame, rng, ppm, mode, value, comp=0.0):
    """comp: the source's own duty-cycle distortion (generator convention:
    highs longer by comp), measured beforehand and subtracted here."""
    jit = value if mode in ("alt", "uniform") else 0.0
    dcd = (value if mode == "dcd" else 0.0) - comp
    edges, t_end = pl.frame_edges(frame, rng, ppm=ppm, jitter=jit,
                                  mode=mode if mode in ("alt", "uniform") else "none", dcd=dcd)
    changes = [(0.0, {0: (1, 1)})] + [(t, {0: (lv, 1)}) for t, lv in edges]
    ls.line_play(pl.line_events(changes))
    time.sleep(50e-6)                                    # TP_IDL event and the records
    recs = _drain(ls)
    r = {}
    for w in recs:
        r.setdefault(w >> 24, []).append(w & 0xFFFFFF)
    if not all(k in r for k in (0xE1, 0xE2, 0xE3, 0xEE)) or len(recs) != 4:
        return {"ok": False, "why": [hex(x) for x in recs]}
    nwords = (r[0xE1][0] & 0xFFFF) - 8
    sr = ((r[0xE2][0] & 0xFFFF) << 16) | (r[0xE3][0] & 0xFFFF)
    if not 0 <= nwords <= 112:
        return {"ok": False, "why": f"word count {nwords}"}
    words = _mem_read(ls, 0, 8, nwords) if nwords else []
    data = b"".join(pl.rev32(w).to_bytes(4, "little") for w in words)
    k = sr.bit_length() - 1
    if k > 0:
        part = int(f"{sr & ((1 << k) - 1):0{k}b}"[::-1], 2)
        data += part.to_bytes((k + 7) // 8, "little")[: k // 8]
    ok = (r[0xEE][0] & 1) == 1 and data == frame and k % 8 == 0
    return {"ok": ok, "crc": r[0xEE][0] & 1, "match": data == frame, "len": len(data), "k": k}


def ethsweep(ls, p):
    rng = random.Random(p.get("seed", 1))
    ls.begin(reload=True)
    ls.host_write(MADDR, 0)
    for w in p["program"]:
        ls.host_write(MDATA, w)
    ls.host_write(XIN0, 3)                               # lane 0 in0 <- ui[3]
    ls.line_route(3)                                     # async line 0 -> ui[3]
    ls.pull(3, 1)
    ls.tracing(False)
    _start_lane(ls, 0, 0, 0)                             # 8-word queues: buffer words 8..119
    ls.run(200)
    ls.set_freerun(True)
    t0 = time.time()
    out = []
    try:
        # sanity: clean frames of several lengths
        sanity = []
        for plen in (46, 47, 49, 50, 51, 102):
            res = eth_receive(ls, pl.make_frame(rng, plen), rng, rng.uniform(-100, 100), "none", 0.0,
                              p.get("dcd_comp", 0.0))
            sanity.append({"payload": plen, "ok": res["ok"]})
        for mode, value in p["points"]:
            fails, first = 0, None
            for _ in range(p.get("frames", 100)):
                res = eth_receive(ls, pl.make_frame(rng, p.get("payload", 46)), rng,
                                  rng.uniform(-100, 100), mode, float(value), p.get("dcd_comp", 0.0))
                if not res["ok"]:
                    fails += 1
                    first = first or res
            out.append({"mode": mode, "value_ns": value, "frames": p.get("frames", 100),
                        "fails": fails, "first_fail": first})
    finally:
        ls.set_freerun(False)
    return {"points": out, "sanity": sanity, "seconds": round(time.time() - t0, 1),
            "line_contention": ls.g["line"].read(8) >> 4 & 0xFFF}


def run(ls, name, params):
    if name == "ethsweep":
        return ethsweep(ls, params)
    raise ValueError(f"unknown far-end campaign {name!r}")
