"""Low-speed USB device on the real RTL: transactions and turnaround (study).

Lane 0 runs usb_dev.asm. The harness plays a host on uio[0] (D-) and uio[1]
(D+): NRZI, bit stuffing, CRC5/CRC16, EOP (SE0 2 bits, J 1 bit, release),
with the host's bit rate offset by up to +-1.5 % and a random sub-cycle
phase. D- has the device pull-up (idle J), D+ the host pull-down. The bus the
script decodes is the resolved pin level (host, device or pull), cycle by
cycle. The turnaround is measured on that bus from the SE0-to-J transition
that ends the host's packet to the J-to-K transition that starts the device's
response (USB 2.0 7.1.18.1: at least 2 and, low speed, at most 7.5 bit times
at the A plug; 6.5 at a B receptacle).

Usage: python3 usb_turnaround.py [repeats]
"""
from __future__ import annotations

import json
import math
import os
import random
import sys

import ethlib
import studysim
from studysim import asm, XIN0, XOUT1

HERE = os.path.dirname(os.path.abspath(__file__))
TBIT = 40                     # device bit, cycles (1.5 Mbit/s at 60 MHz)
J, K, SE0 = (1, 0), (0, 1), (0, 0)   # (D-, D+) for low speed
ADDR = 5


def crc_bits(data_bits, width, poly, init):
    v = ethlib.crc_unit(data_bits, width, poly, init) ^ ((1 << width) - 1)
    return [(v >> (width - 1 - i)) & 1 for i in range(width)]


def lsb(v, n):
    return [(v >> i) & 1 for i in range(n)]


def packet_bits(pid, payload_bits=None, crc=None):
    """SYNC + PID + payload + CRC as data bits (before stuffing)."""
    bits = [0, 0, 0, 0, 0, 0, 0, 1] + lsb(pid | ((~pid & 0xF) << 4), 8)
    if payload_bits:
        bits += payload_bits
        if crc == 5:
            bits += crc_bits(payload_bits, 5, 0x05, 0x1F)
        elif crc == 16:
            bits += crc_bits(payload_bits, 16, 0x8005, 0xFFFF)
    elif crc == 16:                       # zero-length data packet
        bits += crc_bits([], 16, 0x8005, 0xFFFF)
    return bits


def token(pid, addr, ep):
    return packet_bits(pid, lsb(addr, 7) + lsb(ep, 4), crc=5)


def data_packet(pid, data: bytes):
    return packet_bits(pid, [b for x in data for b in lsb(x, 8)], crc=16)


def stuff(bits):
    out, run = [], 0
    for b in bits:
        out.append(b)
        run = run + 1 if b else 0
        if run == 6:
            out.append(0)
            run = 0
    return out


def line_states(bits):
    """NRZI from idle J: 0 = change, 1 = no change; then EOP."""
    st, cur = [], J
    for b in stuff(bits):
        if b == 0:
            cur = K if cur == J else J
        st.append(cur)
    return st + [SE0, SE0, J]


def host_wave(states, rng, ppm, lead_cycles):
    """Per-cycle channel runs for the host driving `states` (one per bit),
    starting after `lead_cycles` of not driving; returns runs and the cycle
    offset (from the start of the wave) of the SE0-to-J transition."""
    bit = TBIT / (1.0 + ppm * 1e-6)
    phase = rng.uniform(0, 1)
    t0 = lead_cycles + phase
    n_tot = int(math.ceil(t0 + len(states) * bit)) + 1
    runs = []
    eop_end = None
    prev = None
    for n in range(n_tot):
        t = n                              # sample instant of cycle n
        k = math.floor((t - t0) / bit) if t >= t0 else -1
        if k < 0 or k >= len(states):
            ent = (0, 0)                   # not driving
        else:
            dm, dp = states[k]
            ent = (3, dm | (dp << 1))
            if k == len(states) - 1 and eop_end is None:
                eop_end = n                # first cycle sampled as the final J
        if runs and runs[-1][0] == ent[0] and runs[-1][1] == ent[1]:
            runs[-1][2] += 1
        else:
            runs.append([ent[0], ent[1], 1])
    return [tuple(r) for r in runs], eop_end


def bus_series(trace):
    """(cycle, D-, D+) at every change of the resolved uio[1:0]."""
    out = []
    for c, uo, ui, uout, uoe, uin in trace:
        s = (uin & 1, (uin >> 1) & 1)
        if not out or out[-1][1:] != s:
            out.append((c, *s))
    return out


def level_at(series, c):
    v = None
    for cc, dm, dp in series:
        if cc <= c:
            v = (dm, dp)
        else:
            break
    return v


def decode_device(trace, after):
    """Find the device's packet on the bus after cycle `after` (driven by the
    chip: uio_oe set). Returns (sop cycle, data bits destuffed, eop_end cycle)."""
    sop = None
    for c, uo, ui, uout, uoe, uin in trace:
        if c > after and (uoe & 3) == 3 and (uin & 3) == 0b10:   # K driven by the device
            sop = c
            break
    if sop is None:
        return None
    ser = bus_series([t for t in trace if t[0] >= sop - 1])
    bits, prev, run, k = [], K, 0, 0
    raw = []
    while True:
        st = level_at(ser, sop + TBIT // 2 + k * TBIT)
        if st == SE0 or st is None:
            break
        raw.append(1 if st == prev else 0)
        prev = st
        k += 1
        if k > 200:
            break
    # first raw bit is the SOP K: NRZI 0 from J
    raw[0] = 0
    out, run, skipnext = [], 0, False
    for b in raw:
        if skipnext:
            skipnext = False
            run = 0
            continue
        out.append(b)
        run = run + 1 if b else 0
        if run == 6:
            skipnext = True
    # EOP end: SE0 -> J on the bus after the packet
    eop_end = None
    seen_se0 = False
    for c, dm, dp in ser:
        if c <= sop:
            continue
        if (dm, dp) == SE0:
            seen_se0 = True
        elif seen_se0 and (dm, dp) == J:
            eop_end = c
            break
    se0_len = None
    if eop_end:
        se0_start = max(c for c, dm, dp in ser if c < eop_end and (dm, dp) == SE0 and
                        level_at(ser, c - 1) != SE0)
        se0_len = eop_end - se0_start
    return sop, out, eop_end, se0_len, sop + k * TBIT


def parse(bits):
    if len(bits) < 16 or bits[:8] != [0, 0, 0, 0, 0, 0, 0, 1]:
        return {"bad": "sync", "bits": bits[:16]}
    pidb = sum(b << i for i, b in enumerate(bits[8:16]))
    res = {"pid": pidb}
    body = bits[16:]
    if body:
        data_bits, crcb = body[:-16], body[-16:]
        res["crc_ok"] = crc_bits(data_bits, 16, 0x8005, 0xFFFF) == crcb
        res["data"] = bytes(sum(data_bits[8 * i + j] << j for j in range(8))
                            for i in range(len(data_bits) // 8)).hex()
    return res


class Bus:
    def __init__(self, chip, rng, ppm):
        self.chip, self.rng, self.ppm = chip, rng, ppm

    def host_send(self, states, lead=8):
        c0 = self.chip.cycle()
        runs, eop = host_wave(states, self.rng, self.ppm, lead)
        self.chip.wave(runs)
        self.chip.play(0)
        return c0 + eop                   # absolute cycle of the SE0 -> J

    def watch(self, cycles):
        self.chip.run(cycles)
        return self.chip.dump(full=True)


def listener_packets(chip):
    """Lane 1's records -> list of packets (hex bytes), each closed by an EOP marker."""
    words = chip.drain(1, out_size=64)
    pkts, cur, i = [], None, 0
    while i < len(words):
        w = words[i]
        if w >> 24 == 0xEE:
            if cur is not None:
                pkts.append(bytes(cur).hex())
            cur = None
            i += 1
            continue
        # CAP record: timestamp word then tagged value word
        if i + 1 < len(words):
            v = words[i + 1]
            if v >> 24 == 0xA0:
                cur = []
            elif v >> 24 == 0xB1 and cur is not None:
                cur.append(v & 0xFF)
        i += 2
    return pkts


def run_case(chip, rng, ppm, log):
    bus = Bus(chip, rng, ppm)
    res = {"ppm": ppm}
    chip.dump()
    # 1. IN token, nothing staged: NAK
    e = bus.host_send(line_states(token(0x9, ADDR, 0)))
    tr = bus.watch(1200)
    d = decode_device(tr, e)
    res["listener_in_nak"] = listener_packets(chip)
    res["in_nak"] = {"turnaround_cycles": d[0] - e, "reply": parse(d[1]), "eop_se0": d[3]} if d else None
    # 2. stage 8 bytes as DATA1, IN token: data, then host ACK
    payload = bytes(rng.randrange(256) for _ in range(8))
    chip.mem_write(0, 21, list(payload))
    chip.mem_write(0, 20, [0x8000 | (8 << 8) | 0x4B])
    chip.dump()
    e = bus.host_send(line_states(token(0x9, ADDR, 0)))
    tr = bus.watch(150)
    while True:
        more = bus.watch(100)
        tr += more
        d = decode_device(tr, e)
        if d and d[2]:
            break
        if chip.cycle() - e > 8000:
            break
    if d and d[2]:
        rep = parse(d[1])
        # host ACK 3 bit times after the device EOP end (host limit 2 .. 7.5)
        now = chip.cycle()
        lead = max(8, d[2] + 3 * TBIT - now)
        bus.host_send(line_states(packet_bits(0x2)), lead=lead)
        bus.watch(700)
        res["listener_in_data"] = listener_packets(chip)
        recs = chip.drain(0, out_size=8)
        st = chip.mem_read(0, 20, 1)[0]
        res["in_data"] = {"turnaround_cycles": d[0] - e, "reply_pid": rep.get("pid"),
                          "crc_ok": rep.get("crc_ok"), "data_ok": rep.get("data") == payload.hex(),
                          "eop_se0": d[3], "ack_record": [hex(r) for r in recs],
                          "mailbox_after_ack": st}
    else:
        res["in_data"] = None
    # 3. SETUP + DATA0 (8 bytes): ACK
    setup = bytes(rng.randrange(256) for _ in range(8))
    chip.dump()
    e1 = bus.host_send(line_states(token(0xD, ADDR, 0)))
    e2 = bus.host_send(line_states(data_packet(0x3, setup)), lead=4 * TBIT)
    tr = bus.watch(1200)
    d = decode_device(tr, e2)
    res["listener_setup"] = listener_packets(chip)
    recs = chip.drain(0, out_size=8)
    nbytes = None
    got = None
    for r in recs:
        if r >> 24 == 0xA3:
            nbytes = (r & 0xFFFF) - 32
    if nbytes:
        got = bytes(w & 0xFF for w in chip.mem_read(0, 32, nbytes))
    res["setup_ack"] = {"turnaround_cycles": d[0] - e2, "reply": parse(d[1]),
                        "records": [hex(r) for r in recs],
                        "rx_ok": got is not None and got[:8] == setup} if d else None
    # 4. IN token to another address: silence
    chip.dump()
    e = bus.host_send(line_states(token(0x9, ADDR + 1, 0)))
    tr = bus.watch(700)
    res["other_addr_silent"] = decode_device(tr, e) is None
    res["contention_cycles"] = chip.contention()
    log.append(res)
    return res


def main():
    reps = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    rng = random.Random(15)
    with open(os.path.join(HERE, "usb_dev.asm")) as f:
        prog = asm.assemble(f.read())
    with open(os.path.join(HERE, "usb_listen.asm")) as f:
        lprog = asm.assemble(f.read(), origin=128)
    log = []
    with studysim.Chip() as chip:
        image = prog.words + [0xC0000000] * (128 - len(prog.words)) + lprog.words
        chip.load_program(image)
        chip.wr(XIN0, 8 | (9 << 4) | (8 << 16) | (9 << 20))   # lanes 0 and 1: in0 <- uio[0], in1 <- uio[1]
        chip.wr(XOUT1, 0x10 | (0x11 << 5))            # uio[0] <- lane 0 out0, uio[1] <- out1
        chip.map(0, 8)
        chip.map(1, 9)
        chip.pull(8, 1)                               # D-: device 1.5 k pull-up (J)
        chip.pull(9, 0)                               # D+: host 15 k pull-down
        chip.mem_write(0, 16, [ADDR])
        chip.mem_write(0, 20, [0])
        chip.start_lane(0, 0, qcfg=0)
        chip.start_lane(1, 128, qcfg=0xC)              # listener: 64-word host-out queue
        chip.run(400)
        for i in range(reps):
            ppm = [0, 15000, -15000][i % 3] + rng.uniform(-500, 500)
            r = run_case(chip, rng, ppm, log)
            print(json.dumps(r))
        prog_words = len(prog.words)
        listener_words = len(lprog.words)
    with open(os.path.join(HERE, "results", "usb_turnaround.json"), "w") as f:
        json.dump({"program_words": prog_words, "listener_words": listener_words,
                   "rtl": studysim.rtl_hash(), "cases": log}, f, indent=1)


if __name__ == "__main__":
    main()
