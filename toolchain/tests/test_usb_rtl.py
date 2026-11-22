"""Low-speed USB device and listener on the chip RTL (Verilator, or the board
with ROSETTA_BACKEND=board): the protocol study's hand-written programs
(study/protocols/REPORT.md section 5; fixtures/study_usb_dev.asm and
study_usb_listen.asm, unchanged).

Lane 0 is the device, lane 1 the listener. D- is uio[0] (out0, in0), D+
uio[1] (out1, in1); D- has the device's pull-up (idle J), D+ the host's
pull-down. The host is the harness's waveform player on uio[0..1] (channels
0 and 1): NRZI, bit stuffing, CRC5/CRC16, EOP (SE0 2 bits, J 1 bit, then
release), at a bit rate offset by 0 or +-1.5 % with a random sub-cycle
phase (fixed seeds, so both backends get the same stimulus). The pins read
the resolved bus (host, device or pull) every cycle.

Per run: IN with nothing staged -> NAK; IN with 8 bytes staged as DATA1 ->
DATA1 + data + CRC16, then the host's ACK clears the mailbox; SETUP + DATA0 ->
ACK with the bytes in data memory; IN to another address -> silence.
Checked: reply PIDs, data and CRC16, records, the device's EOP (SE0 75-90
cycles = 1.25-1.50 us), no cycle where host and device both drive, the
listener's packets, and the turnaround from the host's SE0-to-J to the
device's first K: at least 2 and at most 7.5 bit times (USB 2.0 7.1.18.1),
and within the study's measured ranges (IN: 132-138 cycles, SETUP/DATA0 ->
ACK: 82-86; REPORT.md 5.4).
"""
import os
import random
import unittest

import support
import rtlsim
from protolines import (J, K, SE0, USB_TBIT as TBIT, usb_crc_bits, usb_data_packet, usb_host_wave,
                        usb_line_states, usb_packet_bits, usb_token)
from rosetta import asm

ADDR = 5
NOP = 0xC0000000
STUDY_TURNAROUND = {"in": (132, 138), "setup": (82, 86)}   # cycles, REPORT.md 5.4


def fixture(name, origin=0):
    with open(os.path.join(support.FIXTURES, name), encoding="utf-8") as f:
        return asm.assemble(f.read(), origin=origin)


def bus_series(rows):
    out = []
    for c, uo, ui, uout, uoe, uin in rows:
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


def decode_device(rows, after):
    """The device's packet after cycle `after` (K driven by the chip): (start,
    destuffed bits, EOP end, SE0 length)."""
    sop = None
    for c, uo, ui, uout, uoe, uin in rows:
        if c > after and (uoe & 3) == 3 and (uin & 3) == 0b10:
            sop = c
            break
    if sop is None:
        return None
    ser = bus_series([t for t in rows if t[0] >= sop - 1])
    raw, prev, k = [], K, 0
    while k <= 200:
        st = level_at(ser, sop + TBIT // 2 + k * TBIT)
        if st == SE0 or st is None:
            break
        raw.append(1 if st == prev else 0)
        prev = st
        k += 1
    raw[0] = 0
    out, run, skipnext = [], 0, False
    for b in raw:
        if skipnext:
            skipnext, run = False, 0
            continue
        out.append(b)
        run = run + 1 if b else 0
        if run == 6:
            skipnext = True
    eop_end, seen_se0 = None, False
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
    return sop, out, eop_end, se0_len


def parse(bits):
    if len(bits) < 16 or bits[:8] != [0, 0, 0, 0, 0, 0, 0, 1]:
        return {"bad": "sync"}
    res = {"pid": sum(b << i for i, b in enumerate(bits[8:16]))}
    body = bits[16:]
    if body:
        data_bits, crcb = body[:-16], body[-16:]
        res["crc_ok"] = usb_crc_bits(data_bits, 16, 0x8005, 0xFFFF) == crcb
        res["data"] = bytes(sum(data_bits[8 * i + j] << j for j in range(8))
                            for i in range(len(data_bits) // 8))
    return res


def listener_packets(chip):
    """Lane 1's CAP records -> packets (bytes), each closed by its EOP marker."""
    words = chip.drain(1, out_size=64)
    pkts, cur, i = [], None, 0
    while i < len(words):
        w = words[i]
        if w >> 24 == 0xEE:
            if cur is not None:
                pkts.append(bytes(cur))
            cur = None
            i += 1
            continue
        if i + 1 < len(words):
            v = words[i + 1]
            if v >> 24 == 0xA0:
                cur = []
            elif v >> 24 == 0xB1 and cur is not None:
                cur.append(v & 0xFF)
        i += 2
    return pkts


class Host:
    def __init__(self, chip, rng, ppm):
        self.chip, self.rng, self.ppm = chip, rng, ppm

    def send(self, states, lead=8):
        c0 = self.chip.cycle()
        runs, eop = usb_host_wave(states, self.rng, self.ppm, lead)
        self.chip.wave(runs)
        self.chip.play(0)
        return c0 + eop                    # cycle of the SE0 -> J at the end of the packet

    def watch(self, cycles):
        self.chip.run(cycles)
        return self.chip.dump(full=True)


def run_device(ppm, seed):
    rng = random.Random(seed)
    dev, lsn = fixture("study_usb_dev.asm"), fixture("study_usb_listen.asm", 128)
    res = {"ppm": ppm}
    with rtlsim.Chip() as chip:
        chip.load_program(dev.words + [NOP] * (128 - len(dev.words)) + lsn.words)
        chip.wr(rtlsim.XIN0, 8 | (9 << 4) | (8 << 16) | (9 << 20))   # lanes 0, 1: in0 = uio0, in1 = uio1
        chip.wr(rtlsim.XOUT1, 0x10 | (0x11 << 5))                     # uio0 <- lane 0 out0, uio1 <- out1
        chip.map(0, 8)
        chip.map(1, 9)
        chip.pull(8, 1)                                               # D-: device 1.5 k pull-up
        chip.pull(9, 0)                                               # D+: host 15 k pull-down
        chip.mem_write(0, 16, [ADDR])
        chip.mem_write(0, 20, [0])
        chip.start_lane(0, 0, qcfg=0)
        chip.start_lane(1, 128, qcfg=0xC)
        chip.run(400)
        host = Host(chip, rng, ppm)
        chip.dump()
        # 1. IN, nothing staged: NAK
        e = host.send(usb_line_states(usb_token(0x9, ADDR, 0)))
        d = decode_device(host.watch(1200), e)
        res["in_nak"] = {"turn": d[0] - e, "reply": parse(d[1]), "se0": d[3]} if d else None
        res["lsn_in_nak"] = listener_packets(chip)
        # 2. 8 bytes staged as DATA1: data, then the host's ACK
        payload = bytes(rng.randrange(256) for _ in range(8))
        chip.mem_write(0, 21, list(payload))
        chip.mem_write(0, 20, [0x8000 | (8 << 8) | 0x4B])
        chip.dump()
        e = host.send(usb_line_states(usb_token(0x9, ADDR, 0)))
        rows = host.watch(150)
        while True:
            rows += host.watch(100)
            d = decode_device(rows, e)
            if (d and d[2]) or chip.cycle() - e > 8000:
                break
        if d and d[2]:
            rep = parse(d[1])
            lead = max(8, d[2] + 3 * TBIT - chip.cycle())             # host ACK 3 bit times after
            host.send(usb_line_states(usb_packet_bits(0x2)), lead=lead)
            host.watch(700)
            res["lsn_in_data"] = listener_packets(chip)
            res["in_data"] = {"turn": d[0] - e, "pid": rep.get("pid"), "crc_ok": rep.get("crc_ok"),
                              "data": rep.get("data"), "payload": payload, "se0": d[3],
                              "records": chip.drain(0, out_size=8),
                              "mailbox": chip.mem_read(0, 20, 1)[0]}
        # 3. SETUP + DATA0 (8 bytes): ACK
        setup = bytes(rng.randrange(256) for _ in range(8))
        chip.dump()
        host.send(usb_line_states(usb_token(0xD, ADDR, 0)))
        e2 = host.send(usb_line_states(usb_data_packet(0x3, setup)), lead=4 * TBIT)
        d = decode_device(host.watch(1200), e2)
        res["lsn_setup"] = listener_packets(chip)
        recs = chip.drain(0, out_size=8)
        got = None
        n = [(r & 0xFFFF) - 32 for r in recs if r >> 24 == 0xA3]
        if n:
            got = bytes(w & 0xFF for w in chip.mem_read(0, 32, n[0]))
        res["setup"] = {"turn": d[0] - e2, "reply": parse(d[1]), "records": recs, "rx": got,
                        "sent": setup} if d else None
        # 4. IN to another address: silence
        chip.dump()
        e = host.send(usb_line_states(usb_token(0x9, ADDR + 1, 0)))
        res["other_silent"] = decode_device(host.watch(700), e) is None
        res["lsn_other"] = listener_packets(chip)
        res["contention"] = chip.contention()
    return res


def tok(pid, addr, ep):
    """The token bytes as the listener records them: PID, then 11 bits + CRC5."""
    bits = usb_token(pid, addr, ep)[8:]
    return bytes(sum(bits[8 * i + j] << j for j in range(8)) for i in range(len(bits) // 8))


@unittest.skipUnless(rtlsim.available(), rtlsim.unavailable_reason())
class UsbOnRtl(unittest.TestCase):

    def check(self, ppm, seed):
        r = run_device(ppm, seed)
        lo, hi = 2 * TBIT, int(7.5 * TBIT)
        # 1. NAK
        self.assertEqual(r["in_nak"]["reply"], {"pid": 0x5A})
        self.assertTrue(lo <= r["in_nak"]["turn"] <= hi, r["in_nak"])
        self.assertEqual(r["lsn_in_nak"], [tok(0x9, ADDR, 0), bytes([0x5A])])
        # 2. DATA1, CRC16, host ACK recorded, mailbox cleared
        dd = r["in_data"]
        self.assertEqual((dd["pid"], dd["crc_ok"], dd["data"]), (0x4B, True, dd["payload"]))
        self.assertTrue(lo <= dd["turn"] <= hi, dd)
        self.assertEqual(dd["records"], [0xA40000D2])
        self.assertEqual(dd["mailbox"], 0)
        data1 = usb_data_packet(0x3 ^ 0x8, dd["payload"])[8:]          # PID 0x4B, payload, CRC16
        self.assertEqual(r["lsn_in_data"], [tok(0x9, ADDR, 0),
                                            bytes(sum(data1[8 * i + j] << j for j in range(8))
                                                  for i in range(len(data1) // 8)),
                                            bytes([0xD2])])
        # 3. SETUP + DATA0 -> ACK, bytes in memory (with the 2 CRC bytes)
        s = r["setup"]
        self.assertEqual(s["reply"], {"pid": 0xD2})
        self.assertTrue(lo <= s["turn"] <= hi, s)
        self.assertEqual(s["records"], [0xA100002D, 0xA20000C3, 0xA300002A])
        self.assertEqual(s["rx"][:8], s["sent"])
        self.assertEqual(r["lsn_setup"][0], tok(0xD, ADDR, 0))
        self.assertEqual(r["lsn_setup"][2], bytes([0xD2]))
        # 4. silence for another address; the listener still sees the token
        self.assertTrue(r["other_silent"])
        self.assertEqual(r["lsn_other"], [tok(0x9, ADDR + 1, 0)])
        # device EOP: SE0 for 1.25-1.50 us; never a cycle with both driving
        for x in (r["in_nak"], dd):
            self.assertTrue(75 <= x["se0"] <= 90, x)
        self.assertEqual(r["contention"], 0)
        # within the study's measured ranges
        for key, t in (("in", r["in_nak"]["turn"]), ("in", dd["turn"]), ("setup", s["turn"])):
            a, b = STUDY_TURNAROUND[key]
            self.assertTrue(a <= t <= b, (key, t, (a, b)))
        UsbOnRtl.turnarounds[ppm] = (r["in_nak"]["turn"], dd["turn"], s["turn"])
        return r

    turnarounds: dict = {}

    def test_host_clock_nominal(self):
        self.check(0.0, 1)

    def test_host_clock_fast_1p5pct(self):
        self.check(15000.0, 2)

    def test_host_clock_slow_1p5pct(self):
        self.check(-15000.0, 3)


if __name__ == "__main__":
    unittest.main()
