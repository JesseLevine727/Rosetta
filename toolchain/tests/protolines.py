"""Line models for the USB, 10BASE-T and CAN scenarios (tests only, not a
test module). Copied from the protocol study (study/protocols/sim: ethlib.py,
usb_turnaround.py, can_arbitration.py, unchanged in substance) so the shared
tests do not depend on the study directory.

Everything here is computed on the PC and played into the chip through the
harness's waveform player as per-cycle runs, so the Verilator harness and
the FPGA harness (lockstep) receive exactly the same stimulus.
"""
from __future__ import annotations

import math
import random

# ------------------------------------------------------------------ common

def crc_unit(bits, width=32, poly=0x04C11DB7, init=0xFFFFFFFF):
    """The chip's check unit (docs/blocks/crc.md): raw state after `bits`."""
    m = (1 << width) - 1
    s = init & m
    for b in bits:
        fb = ((s >> (width - 1)) & 1) ^ b
        s = ((s << 1) & m) ^ (poly if fb else 0)
    return s


def runs_from_levels(levels, ch_mask=1):
    """Per-cycle list of 0 / 1 / None (not driving) for one channel -> runs."""
    out = []
    prev, n = "init", 0
    for lv in levels:
        if lv == prev:
            n += 1
        else:
            if n:
                out.append(_ent(prev, ch_mask, n))
            prev, n = lv, 1
    if n:
        out.append(_ent(prev, ch_mask, n))
    return out


def _ent(lv, mask, n):
    if lv is None:
        return (0, 0, n)
    return (mask, mask if lv else 0, n)


def pin_edges(rows, col, bit):
    """[(cycle, level)] of one pin from dump(full=True) rows."""
    out, prev = [], None
    for r in rows:
        v = (r[col] >> bit) & 1
        if v != prev:
            out.append((r[0], v))
            prev = v
    return out


# ------------------------------------------------------------------ 10BASE-T

TC = 1000.0 / 60.0          # chip clock period, ns
BIT = 100.0                 # 10 Mbit/s bit time, ns
PREAMBLE_SFD = bytes([0x55] * 7 + [0xD5])


def byte_bits(data: bytes):
    return [(x >> i) & 1 for x in data for i in range(8)]


def fcs_bits(data: bytes):
    """FCS as the chip sends it: DRIVE 32 msb from CRC (value = ~state)."""
    v = crc_unit(byte_bits(data)) ^ 0xFFFFFFFF
    return [(v >> (31 - i)) & 1 for i in range(32)]


def fcs_bytes(data: bytes) -> bytes:
    b = fcs_bits(data)
    return bytes(sum(b[8 * k + i] << i for i in range(8)) for k in range(4))


def make_frame(rng: random.Random, payload_len=46) -> bytes:
    dst = bytes([0x02, 0x00, 0x00, 0x00, 0x00, 0x01])
    src = bytes([0x02, 0x00, 0x00, 0x00, 0x00, 0x02])
    body = dst + src + (0x0800).to_bytes(2, "big") + bytes(rng.randrange(256) for _ in range(payload_len))
    return body + fcs_bytes(body)


def manchester_edges(bits, t0, bit_ns):
    """(time, new level) of every transition for `bits` from a high line
    (a comparator left high by the previous TP_IDL); and the final level."""
    edges = []
    level = 1
    for k, b in enumerate(bits):
        first, second = 1 - b, b
        tb = t0 + k * bit_ns
        if first != level:
            edges.append((tb, first))
            level = first
        if second != level:
            edges.append((tb + bit_ns / 2, second))
            level = second
    return edges, level


def apply_impairments(edges, rng, jitter=0.0, mode="uniform", dcd=0.0):
    """Per-edge shifts: 'uniform' in [-j, j], 'alt' (+j, -j, ...), 'gauss'
    (sigma j); dcd: high pulses longer by dcd (rising edges early by dcd/2,
    falling late by dcd/2)."""
    out = []
    for k, (t, lv) in enumerate(edges):
        if mode == "uniform":
            j = rng.uniform(-jitter, jitter)
        elif mode == "alt":
            j = jitter if k % 2 == 0 else -jitter
        elif mode == "gauss":
            j = rng.gauss(0.0, jitter)
        else:
            j = 0.0
        d = -dcd / 2 if lv == 1 else dcd / 2
        out.append((t + j + d, lv))
    out.sort()
    return out


def sample_levels(edges, t_end, start_level=1, tc=TC, phase=0.0):
    """Line level at each chip sample instant n*tc + phase."""
    n_tot = int(math.ceil((t_end - phase) / tc))
    lv = [start_level] * n_tot
    level = start_level
    ei = 0
    for n in range(n_tot):
        t = n * tc + phase
        while ei < len(edges) and edges[ei][0] <= t:
            level = edges[ei][1]
            ei += 1
        lv[n] = level
    return lv


def frame_edges(frame: bytes, rng, ppm=0.0, jitter=0.0, mode="uniform", dcd=0.0, t0=2000.0):
    """Impaired edges (time ns, new level) of one frame from a high line
    (preamble + SFD + frame + the rise into TP_IDL), first bit at t0; and
    the end time of the last bit."""
    bit_ns = BIT / (1.0 + ppm * 1e-6)
    bits = byte_bits(PREAMBLE_SFD + frame)
    edges, last = manchester_edges(bits, t0, bit_ns)
    t_bits_end = t0 + len(bits) * bit_ns
    if last == 0:
        edges.append((t_bits_end, 1))
    return apply_impairments(edges, rng, jitter, mode, dcd), t_bits_end


def frame_wave(frame: bytes, rng, ppm=0.0, jitter=0.0, mode="uniform", dcd=0.0,
               lead_ns=2000.0, tp_idl_ns=300.0, tail_ns=1500.0, phase=None):
    """Per-cycle levels for one frame: idle high, preamble+SFD, frame, TP_IDL
    (high), idle high; random phase against the chip clock unless given."""
    if phase is None:
        phase = rng.uniform(0.0, TC)
    edges, t_bits_end = frame_edges(frame, rng, ppm, jitter, mode, dcd, lead_ns + phase)
    t_end = t_bits_end + tp_idl_ns + tail_ns
    return sample_levels(edges, t_end), edges


# ------------------------------------------------------------------ async-line events (rh_line.v)

FINE_NS = 2.5 / 32           # 78.125 ps: IDELAYE2 tap at a 200 MHz reference


def line_events(changes, lead_ns=200.0):
    """Event words for the board's async lines (fpga/pynq-z1/hdl/rh_line.v)
    from [(time ns, {line: (level, drive)})], times increasing; consecutive
    events must be >= 4 cycles (20 ns) apart. Returns the words."""
    words, prev = [], None
    for t, upd in changes:
        tf = int(round((t + lead_ns) / FINE_NS))
        coarse, half, tap = tf >> 6, (tf >> 5) & 1, tf & 31
        dt = coarse - prev if prev is not None else 4
        if prev is not None and dt < 4:
            raise ValueError(f"async-line events {dt * 5} ns apart (minimum 20 ns)")
        prev = coarse
        w = (dt & 0xFFFF) | half << 16 | tap << 17
        for line, (lvl, drv) in upd.items():
            w |= 1 << (22 + line) | lvl << (24 + line) | drv << (26 + line)
        words.append(w)
    return words


def rev32(x):
    return int(f"{x:032b}"[::-1], 2)


# ------------------------------------------------------------------ low-speed USB

USB_TBIT = 40                         # device bit, cycles (1.5 Mbit/s at 60 MHz)
J, K, SE0 = (1, 0), (0, 1), (0, 0)    # (D-, D+) for low speed


def usb_crc_bits(data_bits, width, poly, init):
    v = crc_unit(data_bits, width, poly, init) ^ ((1 << width) - 1)
    return [(v >> (width - 1 - i)) & 1 for i in range(width)]


def lsb(v, n):
    return [(v >> i) & 1 for i in range(n)]


def usb_packet_bits(pid, payload_bits=None, crc=None):
    bits = [0, 0, 0, 0, 0, 0, 0, 1] + lsb(pid | ((~pid & 0xF) << 4), 8)
    if payload_bits:
        bits += payload_bits
        if crc == 5:
            bits += usb_crc_bits(payload_bits, 5, 0x05, 0x1F)
        elif crc == 16:
            bits += usb_crc_bits(payload_bits, 16, 0x8005, 0xFFFF)
    elif crc == 16:
        bits += usb_crc_bits([], 16, 0x8005, 0xFFFF)
    return bits


def usb_token(pid, addr, ep):
    return usb_packet_bits(pid, lsb(addr, 7) + lsb(ep, 4), crc=5)


def usb_data_packet(pid, data: bytes):
    return usb_packet_bits(pid, [b for x in data for b in lsb(x, 8)], crc=16)


def usb_stuff(bits):
    out, run = [], 0
    for b in bits:
        out.append(b)
        run = run + 1 if b else 0
        if run == 6:
            out.append(0)
            run = 0
    return out


def usb_line_states(bits):
    """NRZI from idle J (0 = change), then EOP: SE0 2 bits, J 1 bit."""
    st, cur = [], J
    for b in usb_stuff(bits):
        if b == 0:
            cur = K if cur == J else J
        st.append(cur)
    return st + [SE0, SE0, J]


def usb_host_wave(states, rng, ppm, lead_cycles):
    """Channel runs (channels 0 = D-, 1 = D+) for a host sending `states`,
    one per bit at its own rate (ppm offset) and a random sub-cycle phase,
    after `lead_cycles` of not driving. Returns runs and the offset of the
    cycle sampled as the final J (the end of the EOP)."""
    bit = USB_TBIT / (1.0 + ppm * 1e-6)
    phase = rng.uniform(0, 1)
    t0 = lead_cycles + phase
    n_tot = int(math.ceil(t0 + len(states) * bit)) + 1
    runs = []
    eop_end = None
    for n in range(n_tot):
        k = math.floor((n - t0) / bit) if n >= t0 else -1
        if k < 0 or k >= len(states):
            ent = (0, 0)
        else:
            dm, dp = states[k]
            ent = (3, dm | (dp << 1))
            if k == len(states) - 1 and eop_end is None:
                eop_end = n
        if runs and runs[-1][0] == ent[0] and runs[-1][1] == ent[1]:
            runs[-1][2] += 1
        else:
            runs.append([ent[0], ent[1], 1])
    return [tuple(r) for r in runs], eop_end


# ------------------------------------------------------------------ CAN

def msb(v, n):
    return [(v >> (n - 1 - i)) & 1 for i in range(n)]


def can_frame_bits(ident, data: bytes):
    """Unstuffed SOF .. CRC (standard frame, data)."""
    body = [0] + msb(ident, 11) + [0, 0, 0] + msb(len(data), 4)
    for b in data:
        body += msb(b, 8)
    return body + msb(crc_unit(body, 15, 0x4599, 0), 15)


def can_stuff(bits):
    out, run, last = [], 0, None
    for b in bits:
        out.append(b)
        run = run + 1 if b == last else 1
        last = b
        if run == 5:
            out.append(1 - b)
            last, run = 1 - b, 1
    return out


def can_wire(ident, data):
    """What an external node sends: stuffed SOF..CRC, CRC delimiter, ACK slot
    released, ACK delimiter, EOF."""
    return can_stuff(can_frame_bits(ident, data)) + [1, 1, 1] + [1] * 7


def can_ext_runs(bits, tbit, lead):
    """Channel 7 (the external node) runs: drive 0 for dominant, release for
    recessive."""
    runs = [(0, 0, lead)]
    for b in bits:
        runs.append((0x80, 0x00, tbit) if b == 0 else (0, 0, tbit))
    return runs


def can_decode_bus(rows, tbit, pin=3, end_t=None, sample=None):
    """Every frame on an RXD ui pin: resync on recessive-to-dominant edges,
    sample mid-bit, de-stuff to the end of the CRC, check the CRC, read the
    tail (CRC delimiter, ACK slot, ACK delimiter, EOF). `end_t`: the last
    cycle the trace covers (rows only mark changes; the study's decoder used
    the last change, so it could not finish a frame whose EOF had no edge
    after it). `sample`: the sample point in cycles after a bit's start
    (default mid-bit, as the study's decoder)."""
    sp = tbit // 2 if sample is None else sample
    ev, prev = [], 1
    for r in rows:
        v = (r[2] >> pin) & 1
        if v != prev:
            ev.append((r[0], v))
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

    frames, i = [], 0
    end_t = rows[-1][0] if end_t is None else end_t
    while i < len(ev):
        c, v = ev[i]
        if v != 0:
            i += 1
            continue
        t_ref, k_ref, k = c, 0, 0
        run, last, destuffed, sample_times, ok, n_stuffed = 0, None, [], [], True, None
        while True:
            t = t_ref + sp + (k - k_ref) * tbit
            if t > end_t:
                ok = False
                break
            for cc, vv in ev:
                if vv == 0 and t - tbit < cc <= t - sp and cc > t_ref:
                    t_ref, k_ref = cc, k
                    t = t_ref + sp
            b = level(t)
            sample_times.append(t)
            k += 1
            in_stuff = True
            if len(destuffed) >= 19:
                dlc = int("".join(map(str, destuffed[15:19])), 2)
                n_stuffed = 19 + 8 * min(dlc, 8) + 15
                in_stuff = len(destuffed) < n_stuffed
            if in_stuff:
                if run == 5:
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
        dlc = int("".join(map(str, body[15:19])), 2)
        tail = destuffed[n_stuffed:n_stuffed + 10]
        frames.append({"sof_cycle": c, "id": int("".join(map(str, body[1:12])), 2),
                       "data": bytes(int("".join(map(str, body[19 + 8 * j:27 + 8 * j])), 2)
                                     for j in range(min(dlc, 8))),
                       "crc_ok": crc_unit(body, 15, 0x4599, 0) == crc,
                       "crc_delim": tail[0], "ack_slot": tail[1], "ack_delim": tail[2],
                       "eof": tail[3:10]})
        t_end = sample_times[-1]
        while i < len(ev) and ev[i][0] <= t_end:
            i += 1
    return frames
