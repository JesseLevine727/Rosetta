"""10BASE-T helpers for the study: frames, CRC-32 in the chip's form, and a
continuous-time Manchester line model sampled at the chip clock.

Conventions (IEEE 802.3 clause 7/14, as coding.md implements them): bytes go
on the wire least significant bit first; a bit is sent as its complement for
the first half and its value for the second half, so a 1 has a rising
mid-bit edge. The FCS is the complemented CRC, x^31 coefficient first.
"""
from __future__ import annotations

import math
import random

TC = 1000.0 / 60.0          # chip clock period, ns
BIT = 100.0                 # 10 Mbit/s bit time, ns
POLY = 0x04C11DB7


def crc_unit(bits, width=32, poly=POLY, init=0xFFFFFFFF):
    """The chip's check unit (crc.md): returns the raw state after `bits`."""
    m = (1 << width) - 1
    s = init & m
    for b in bits:
        fb = ((s >> (width - 1)) & 1) ^ b
        s = ((s << 1) & m) ^ (poly if fb else 0)
    return s


def byte_bits(data: bytes):
    return [(x >> i) & 1 for x in data for i in range(8)]


def fcs_bits(data: bytes):
    """FCS as the chip sends it: DRIVE 32 msb from CRC (value = ~state)."""
    v = crc_unit(byte_bits(data)) ^ 0xFFFFFFFF
    return [(v >> (31 - i)) & 1 for i in range(32)]


def fcs_bytes(data: bytes) -> bytes:
    b = fcs_bits(data)
    return bytes(sum(b[8 * k + i] << i for i in range(8)) for k in range(4))


def residue():
    """Raw CRC state after a frame and its FCS (constant for every frame)."""
    d = bytes(range(20))
    return crc_unit(byte_bits(d) + fcs_bits(d))


def zlib_check():
    import zlib
    d = b"123456789"
    # the chip's FCS bytes equal the standard CRC-32 little-endian bytes
    return fcs_bytes(d) == zlib.crc32(d).to_bytes(4, "little")


PREAMBLE_SFD = bytes([0x55] * 7 + [0xD5])


def make_frame(rng: random.Random, payload_len=46, dst=None, src=None, etype=0x0800) -> bytes:
    dst = dst or bytes([0x02, 0x00, 0x00, 0x00, 0x00, 0x01])
    src = src or bytes([0x02, 0x00, 0x00, 0x00, 0x00, 0x02])
    body = dst + src + etype.to_bytes(2, "big") + bytes(rng.randrange(256) for _ in range(payload_len))
    return body + fcs_bytes(body)


def manchester_edges(bits, t0, bit_ns):
    """(time, new level) of every transition of the Manchester line for `bits`,
    starting at t0 from the level `start` (the line is high before the frame,
    as a comparator with hysteresis left high by the previous TP_IDL).
    Returns edges and the level at the end of the last bit."""
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
    """Edge time shifts. jitter: peak per-edge jitter (ns); mode 'uniform'
    (independent, uniform in [-j, j]), 'alt' (signs alternate edge by edge:
    every interval is lengthened or shortened by 2j, the worst case for an
    interval classifier), 'gauss' (sigma = jitter). dcd: high pulses longer by
    dcd ns and low pulses shorter (rising edges early by dcd/2, falling late
    by dcd/2)."""
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
    # keep edges ordered (huge jitter could reorder; then they merge)
    out.sort()
    return out


def sample_levels(edges, t_end, start_level=1, tc=TC, phase=0.0):
    """Level of the line at each chip sample instant n*tc + phase, n = 0 ..
    while n*tc + phase < t_end. A single-edge synchroniser sees exactly
    this (the harness applies cycle n's level before rising edge n + 1)."""
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


def frame_wave(frame: bytes, rng, ppm=0.0, jitter=0.0, mode="uniform", dcd=0.0,
               lead_ns=2000.0, tp_idl_ns=300.0, tail_ns=1500.0):
    """Per-cycle levels for one frame: idle high, preamble+SFD, frame, TP_IDL
    (line held high), idle high. Random phase against the chip clock."""
    bit_ns = BIT / (1.0 + ppm * 1e-6)
    bits = byte_bits(PREAMBLE_SFD + frame)
    phase = rng.uniform(0.0, TC)
    t0 = lead_ns + phase
    edges, last = manchester_edges(bits, t0, bit_ns)
    t_bits_end = t0 + len(bits) * bit_ns
    if last == 0:                       # TP_IDL starts with a rise at the boundary
        edges.append((t_bits_end, 1))
    edges = apply_impairments(edges, rng, jitter, mode, dcd)
    t_end = t_bits_end + tp_idl_ns + tail_ns
    return sample_levels(edges, t_end), edges


# ---------------------------------------------------------------------------
# Decoder models (used for the options study; the first one is checked
# against the RTL by eth_rx_margin.py)
# ---------------------------------------------------------------------------

def decode_classifier(sample_times, levels_after, thr):
    """The lane's PULSE/Manchester decoder (coding.md 3.2, tick.md 6) on a
    list of detected edges: sample_times are the (integer, in sample units)
    detection instants, levels_after the level after each edge. thr in the
    same units. Returns decoded bits."""
    bits = []
    mph = 1
    prev = None
    for t, lv in zip(sample_times, levels_after):
        d = (t - prev) if prev is not None else 10 ** 6
        prev = t
        ge0 = d >= thr
        if ge0 or not mph:
            bits.append(lv)
        mph = 1 if ge0 else (0 if mph else 1)
    return bits


def detect(edges, resolution, offset=0.0):
    """Quantise continuous edges to the first sample instant at or after
    them (samples at k*resolution + offset). Returns (k list, level list),
    merging edges that land on the same sample (the line toggled twice
    between samples: both are lost)."""
    ks, lvs = [], []
    for t, lv in edges:
        k = math.ceil((t - offset) / resolution - 1e-12)
        if ks and k == ks[-1]:
            ks.pop()
            lvs.pop()
            if lvs and lvs[-1] == lv:
                continue
            if not lvs and lv == 1:
                continue
        ks.append(k)
        lvs.append(lv)
    return ks, lvs


def decode_dpll(edges_q, bit_units, gain=0.125, window=0.25):
    """Phase-tracking decoder: keeps a running estimate of the mid-bit
    phase (in sample units, fractional) updated with gain `gain` at each
    accepted mid-bit edge, and frequency tracked by the same update. An edge
    is a mid-bit edge when its phase relative to the estimate is within
    +-window*bit of the expected mid-bit position; otherwise a boundary
    edge. edges_q: (sample index, level) pairs. Starts on the first edge
    after a long gap (preamble alternates, so every edge there is mid-bit)."""
    ks, lvs = edges_q
    bits = []
    if not ks:
        return bits
    est = None
    period = bit_units
    for k, lv in zip(ks, lvs):
        if est is None:
            est = float(k)
            bits.append(lv)
            continue
        # nearest expected mid-bit instant
        n = round((k - est) / period)
        if n < 1:
            n = 1 if (k - est) > period * (1 - window) else 0
        if n == 0:
            continue                          # boundary edge (or noise) inside the bit
        err = k - (est + n * period)
        if abs(err) <= window * period:
            bits.extend([None] * (n - 1))     # missed mid-bit edges = lost bits (never in Manchester)
            bits.append(lv)
            est = est + n * period + gain * err
        # else: boundary edge
    return bits
