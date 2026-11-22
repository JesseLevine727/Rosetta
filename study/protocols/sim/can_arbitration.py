"""CAN on the real RTL: two Rosetta nodes arbitrating on one bus (study).

Node A = lane 0 (transmitter, can_tx.asm) + lane 1 (receiver, can_rx.asm);
node B = lanes 2 and 3. Each node's two lanes drive open-drain outputs on one
TXD net (uio[0]+uio[1] for A, uio[2]+uio[3] for B: two pins tied on the
board, the transceiver's TXD pull-up makes recessive). The harness bus model
(study_tb.cpp "B dtx drx") is the wired-AND of both TXD nets delayed dtx
cycles, and of an external scripted node (channel 7); each node's RXD (ui[3],
ui[4]) is the bus delayed drx more cycles: a transceiver loop delay of
dtx + drx cycles.

Scenario: the host gives both nodes a frame; an external node sends a frame;
when its intermission ends both nodes start their SOF together; the lower
identifier wins, the loser withdraws within the arbitration bit, its
receiver lane takes and acknowledges the winner's frame, and the loser sends
again after the winner's intermission.

Usage: python3 can_arbitration.py [bitrate_kbps] [loop_delay_cycles] [K]
"""
from __future__ import annotations

import json
import os
import sys

import ethlib
import studysim
from studysim import asm, XIN0, XIN1, XOUT1

HERE = os.path.dirname(os.path.abspath(__file__))


def assemble(name, origin, sub):
    with open(os.path.join(HERE, name)) as f:
        t = f.read()
    for k, v in sub.items():
        t = t.replace(k, v)
    return asm.assemble(t, origin=origin)


def msb(v, n):
    return [(v >> (n - 1 - i)) & 1 for i in range(n)]


def frame_bits(ident, data: bytes):
    """Unstuffed SOF..CRC, then the tail (CRC delimiter, ACK slot as sent
    by the transmitter, ACK delimiter, EOF)."""
    body = [0] + msb(ident, 11) + [0, 0, 0] + msb(len(data), 4)
    for b in data:
        body += msb(b, 8)
    crc = ethlib.crc_unit(body, 15, 0x4599, 0)
    return body + msb(crc, 15)


def stuff(bits):
    out, run, last = [], 0, None
    for b in bits:
        out.append(b)
        run = run + 1 if b == last else 1
        last = b
        if run == 5:
            out.append(1 - b)
            last, run = 1 - b, 1
    return out


def wire(ident, data):
    return stuff(frame_bits(ident, data)) + [1, 1, 1] + [1] * 7


def ext_runs(bits, tbit, lead):
    runs = [(0, 0, lead)]
    for b in bits:
        runs.append((0x80, 0x00, tbit) if b == 0 else (0, 0, tbit))
    return runs


def decode_bus(trace, tbit, pin=3):
    """Decode every frame seen on RXD (ui[pin]): resynchronise on each
    recessive-to-dominant edge, sample at mid-bit, de-stuff to the end of the
    CRC, check the CRC and read the ACK slot."""
    ev = []
    prev = 1
    for c, uo, ui, *_ in trace:
        v = (ui >> pin) & 1
        if v != prev:
            ev.append((c, v))
            prev = v
    if not ev:
        return []

    def level(t):
        lv = 1
        for c, v in ev:
            if c <= t:
                lv = v
            else:
                break
        return lv

    frames = []
    i = 0
    end_t = trace[-1][0]
    while i < len(ev):
        c, v = ev[i]
        if v != 0:
            i += 1
            continue
        # SOF at c. Walk bits with resync on falling edges.
        t_ref, k_ref = c, 0
        raw, k = [], 0
        stuffed_bits, run, last, destuffed = [], 0, None, []
        pos = 0
        sample_times = []
        ok = True
        while True:
            t = t_ref + tbit // 2 + (k - k_ref) * tbit
            if t > end_t:
                ok = False
                break
            # resync: a falling edge between the previous sample and this bit's start
            for cc, vv in ev:
                if vv == 0 and t - tbit < cc <= t - tbit // 2 and cc > t_ref:
                    t_ref, k_ref = cc, k
                    t = t_ref + tbit // 2
            b = level(t)
            sample_times.append(t)
            k += 1
            if len(destuffed) < 1 + 11 + 3 + 4:
                pass
            # de-stuff (up to the end of the CRC sequence)
            in_stuff = True
            if destuffed and len(destuffed) >= 19:
                dlc = int("".join(map(str, destuffed[15:19])), 2)
                n_stuffed = 19 + 8 * min(dlc, 8) + 15
                in_stuff = len(destuffed) < n_stuffed
            else:
                n_stuffed = None
            if in_stuff:
                if run == 5:            # this is a stuff bit
                    run, last = 1, b
                    continue
                destuffed.append(b)
                run = run + 1 if b == last else 1
                last = b
            else:
                destuffed.append(b)
            if n_stuffed is not None and len(destuffed) >= n_stuffed + 3 + 7:
                break
        if not ok or n_stuffed is None:
            break
        body = destuffed[:n_stuffed - 15]
        crc = int("".join(map(str, destuffed[n_stuffed - 15:n_stuffed])), 2)
        ident = int("".join(map(str, body[1:12])), 2)
        dlc = int("".join(map(str, body[15:19])), 2)
        data = bytes(int("".join(map(str, body[19 + 8 * j:27 + 8 * j])), 2) for j in range(min(dlc, 8)))
        tail = destuffed[n_stuffed:n_stuffed + 10]
        frames.append({"sof_cycle": c, "id": hex(ident), "data": data.hex(),
                       "crc_ok": ethlib.crc_unit(body, 15, 0x4599, 0) == crc,
                       "crc_delim": tail[0], "ack_slot": tail[1], "ack_delim": tail[2],
                       "eof": tail[3:10]})
        # skip to after this frame
        t_end = sample_times[-1]
        while i < len(ev) and ev[i][0] <= t_end:
            i += 1
    return frames


def txd_dominant(trace, tbit):
    """Dominant intervals of each node's TXD net (uio_in bit 0 = node A,
    bit 2 = node B) as (start cycle, length in cycles)."""
    res = {}
    for name, bit in (("A", 0), ("B", 2)):
        iv, start, prev = [], None, 1
        for c, uo, ui, uout, uoe, uin in trace:
            v = (uin >> bit) & 1
            if v == 0 and prev == 1:
                start = c
            elif v == 1 and prev == 0 and start is not None:
                iv.append((start, c - start))
            prev = v
        res[name] = iv
    return res


def recs(words):
    return [hex(w) for w in words]


def main():
    kbps = int(sys.argv[1]) if len(sys.argv) > 1 else 500
    loop = int(sys.argv[2]) if len(sys.argv) > 2 else 8
    kd = int(sys.argv[3]) if len(sys.argv) > 3 else 40
    tbit = 60000 // kbps
    per = (tbit * 3) // 4
    subs = {"KD": str(kd), "PERIOD, 90": f"PERIOD, {per}", "PERIOD2, 30": f"PERIOD2, {tbit - per}",
            "PERIOD, 120": f"PERIOD, {tbit}", "wait=8": f"wait={(6 * tbit + per - 1) // per}"}
    ta = assemble("can_tx.asm", 0, {**subs, "OWN": "C0", "FREE": "C1"})
    ra = assemble("can_rx.asm", 64, {**subs, "OWN": "C0", "FREE": "C1"})
    tb = assemble("can_tx.asm", 128, {**subs, "OWN": "C2", "FREE": "C3"})
    rb = assemble("can_rx.asm", 192, {**subs, "OWN": "C2", "FREE": "C3"})
    image = [0xC0000000] * 256
    for p in (ta, ra, tb, rb):
        image[p.origin:p.origin + len(p.words)] = p.words
    id_a, data_a = 0x123, bytes([0xAB, 0xCD])
    id_b, data_b = 0x0F0, bytes([0x55])
    id_x, data_x = 0x700, bytes([0x0F, 0xF0, 0x00])
    out = {"kbps": kbps, "tbit_cycles": tbit, "loop_delay_cycles": loop, "K": kd,
           "programs_words": {"tx": len(ta.words), "rx": len(ra.words)}, "rtl": studysim.rtl_hash()}
    with studysim.Chip() as chip:
        chip.load_program(image)
        chip.wr(XIN0, 3 | (8 << 4) | (3 << 16))             # lane 0 in0 RXD_A, in1 TXD_A net; lane 1 in0 RXD_A
        chip.wr(XIN1, 4 | (10 << 4) | (4 << 16))            # lane 2 in0 RXD_B, in1 TXD_B net; lane 3 in0 RXD_B
        chip.wr(XOUT1, 0x10 | (0x14 << 5) | (0x18 << 10) | (0x1C << 15))
        chip.canbus(loop // 2, loop - loop // 2)
        for lane, org in ((1, 64), (3, 192), (0, 0), (2, 128)):
            chip.start_lane(lane, org, qcfg=0xF)
        chip.run(200)
        for lane, ident, data in ((0, id_a, data_a), (2, id_b, data_b)):
            chip.mem_write(lane, 7, [len(data)] + list(data))
            chip.push(lane, [ident << 1, len(data)])
        chip.run(2000)
        chip.dump()
        bits_x = wire(id_x, data_x)
        chip.wave(ext_runs(bits_x, tbit, 50))
        chip.run((len(bits_x) + 3 * 140 + 60) * tbit)
        tr = chip.dump(full=True)
        out["txd_dominant"] = txd_dominant(tr, tbit)
        out["records"] = {f"lane{k}": recs(chip.drain(k)) for k in range(4)}
        out["bus_frames"] = decode_bus(tr, tbit, pin=3)
        out["bus_frames_at_B"] = [f["id"] for f in decode_bus(tr, tbit, pin=4)]
        # when did each node's transmitter stop driving after losing?
        out["note"] = "ack_slot 0 = acknowledged (dominant)"
    print(json.dumps(out, indent=1))
    with open(os.path.join(HERE, "results", f"can_arbitration_{kbps}k_loop{loop}.json"), "w") as f:
        json.dump(out, f, indent=1)


if __name__ == "__main__":
    main()
