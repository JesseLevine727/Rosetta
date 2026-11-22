"""10BASE-T transmit on the real RTL (study): waveform, timing, FCS, TP_IDL
and link pulses, plus a loopback into the receive lane (eth_rx.asm).

Lane 2 transmits (eth_tx.asm) on uo[2] (TD+) and uo[3] (TD-); a board wire
takes uo[2] to ui[3], lane 0's receive input. Usage: python3 eth_tx_check.py
"""
import json
import os
import random
import zlib

import ethlib
import studysim
from studysim import asm, XIN0, XOUT0

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    rx = asm.assemble(open(os.path.join(HERE, "eth_rx.asm")).read())
    tx_text = open(os.path.join(HERE, "eth_tx.asm")).read().replace(".equ NLPN, 78", ".equ NLPN, 2")
    tx = asm.assemble(tx_text, origin=64)
    image = rx.words + [0xC0000000] * (64 - len(rx.words)) + tx.words
    rng = random.Random(4)
    out = {"tx_words": len(tx.words), "rx_words": len(rx.words), "rtl": studysim.rtl_hash(), "frames": []}
    with studysim.Chip() as chip:
        chip.load_program(image)
        chip.wr(XIN0, 3)                                  # lane 0 in0 <- ui[3]
        chip.wr(XOUT0, 0x18 | (0x19 << 5))                # uo[2] <- lane 2 out0, uo[3] <- lane 2 out1
        chip._cmd("L 2 3")                                # board wire uo[2] -> ui[3]
        chip.start_lane(0, 0, qcfg=0)
        chip.start_lane(2, 64, qcfg=0)
        chip.run(60000)                                   # two link-pulse periods (NLPN = 2 here)
        tr = chip.dump(full=True)
        # link pulses on uo[2]/uo[3]
        pulses = []
        prev = 0
        for c, uo, *_ in tr:
            p, n = (uo >> 2) & 1, (uo >> 3) & 1
            if p and not n and prev == 0:
                st = c
            if not p and prev == 1:
                pulses.append((st, c - st))
            prev = p
        out["nlp"] = {"widths_cycles": [w for _, w in pulses][:4],
                      "periods_cycles": [b[0] - a[0] for a, b in zip(pulses, pulses[1:])][:3]}
        for L in (46, 50, 102):
            body = bytes(rng.randrange(256) for _ in range(L + 14))
            words = [int.from_bytes(body[i:i + 4], "little") for i in range(0, len(body), 4)]
            chip.mem_write(2, 16, words)
            chip.dump()
            chip.push(2, [len(words)], in_size=8)
            chip.run(300000)
            tr = chip.dump(full=True)
            # edges of TD+ (uo[2]) and check TD- is its complement while active
            ed, prev, comp_bad = [], None, 0
            for c, uo, *_ in tr:
                p, n = (uo >> 2) & 1, (uo >> 3) & 1
                if prev is not None and p != prev:
                    ed.append((c, p))
                if p == n and p == 1:
                    comp_bad += 1
                prev = p
            # frame = the burst of edges with gaps < 20 cycles
            burst = []
            for c, p in ed:
                if burst and c - burst[-1][0] > 20 and len(burst) > 100:
                    break
                if burst and c - burst[-1][0] > 20:
                    burst = []
                burst.append((c, p))
            t0 = burst[0][0]
            iv = sorted(set(b[0] - a[0] for a, b in zip(burst, burst[1:])))
            # decode: mid-bit edges are on the 6-cycle grid offset 3 from the first boundary
            bits = []
            start = t0 - 3 if burst[0][1] == 1 else t0   # first bit is 1: rise at mid-bit
            lvl = {}
            for c, p in burst:
                lvl[c] = p
            def level_at(t):
                v = 0
                for c, p in burst:
                    if c <= t:
                        v = p
                    else:
                        break
                return v
            nb = (burst[-1][0] - start) // 6 + 1
            for k in range(nb):
                bits.append(level_at(start + 6 * k + 4))     # second half = bit value
            # find SFD
            s = "".join(map(str, bits))
            i = s.find("10101011")
            data_bits = bits[i + 8:]
            nbytes = len(data_bits) // 8
            got = bytes(sum(data_bits[8 * k + j] << j for j in range(8)) for k in range(nbytes))
            fcs_ok = got[:len(body)] == body and got[len(body):len(body) + 4] == zlib.crc32(body).to_bytes(4, "little")
            # TP_IDL: TD+ high after the last mid-bit transition
            last = burst[-1]
            after = [e for e in ed if e[0] > last[0]]
            tp_idl = (after[0][0] - last[0]) if (last[1] == 1 and after) else None
            recs = chip.drain(0, out_size=8)
            out["frames"].append({"bytes": len(body) + 4, "interval_set_cycles": iv,
                                  "preamble_sfd_bits_seen": i + 8, "fcs_ok": fcs_ok,
                                  "tp_idl_high_cycles_after_last_edge": tp_idl,
                                  "diff_violations": comp_bad,
                                  "rx_lane_records": [hex(r) for r in recs]})
    print(json.dumps(out, indent=1))
    json.dump(out, open(os.path.join(HERE, "results", "eth_tx_check.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
