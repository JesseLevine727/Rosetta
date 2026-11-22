"""Study driver for the Rosetta chip RTL under Verilator.

Same interface as toolchain/tests/rtlsim.py (its 14 Nov version, copied
here so the study does not depend on that file) but built with study_tb.cpp, which adds
waveform injection and a CAN bus model. The build goes to
$ROSETTA_STUDY_BUILD (default: the system temp directory).
"""
from __future__ import annotations

import hashlib
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "toolchain"))

from rosetta import asm  # noqa: E402,F401

# Copied from toolchain/tests/rtlsim.py (14 Nov version) so the study does not
# depend on that file, which other work is changing.
OSS_CAD = "/home/elfo/tools/oss-cad-suite-dl/oss-cad-suite"
SRC = os.path.join(REPO, "src")
MADDR, MDATA = 0x01, 0x02
XIN0, XIN1, XOUT0, XOUT1, XOUT2, XCTL = 0x40, 0x41, 0x42, 0x43, 0x44, 0x45
CTRL, PC, FLAGS, R10, R32, QCFG, QIN, QOUT = range(8)
TOP = "tt_um_jesselevine727_rosetta"


def lane_reg(lane: int, r: int) -> int:
    return 0x10 + 8 * lane + r


def find_verilator():
    import shutil
    for cand in (os.environ.get("ROSETTA_VERILATOR"), os.path.join(OSS_CAD, "bin", "verilator"),
                 shutil.which("verilator")):
        if cand and os.path.isfile(cand) and os.access(cand, os.X_OK):
            return cand
    return None


def _env() -> dict:
    env = dict(os.environ)
    env.pop("VERILATOR_ROOT", None)
    vbin = find_verilator()
    if vbin:
        env["PATH"] = os.path.dirname(vbin) + os.pathsep + env.get("PATH", "")
    return env


def rtl_sources():
    import glob
    files = sorted(glob.glob(os.path.join(SRC, "rosetta_*.sv")))
    return [f for f in files if not f.endswith("rosetta_sram_bb.v")]


def rtl_hash() -> str:
    h = hashlib.sha256()
    for f in rtl_sources():
        with open(f, "rb") as fh:
            h.update(os.path.basename(f).encode() + fh.read())
    return h.hexdigest()[:16]


HARNESS = os.path.join(HERE, "study_tb.cpp")


def build() -> str:
    vbin = find_verilator()
    if vbin is None:
        raise RuntimeError("Verilator not found")
    srcs = rtl_sources()
    h = hashlib.sha256()
    for f in srcs + [HARNESS]:
        with open(f, "rb") as fh:
            h.update(os.path.basename(f).encode() + fh.read())
    base = os.environ.get("ROSETTA_STUDY_BUILD") or tempfile.gettempdir()
    out = os.path.join(base, "rosetta_study_" + h.hexdigest()[:16])
    exe = os.path.join(out, "V" + TOP)
    if os.path.isfile(exe):
        return exe
    os.makedirs(out, exist_ok=True)
    cmd = [vbin, "--cc", "--exe", "--build", "-O2", "-j", "0", "-Wno-fatal", "-Wno-lint",
           "-Wno-style", "-DROSETTA_SRAM_BEHAV", "--top-module", TOP, "--Mdir", out,
           "-o", "V" + TOP, *srcs, HARNESS]
    r = subprocess.run(cmd, env=_env(), capture_output=True, text=True)
    if r.returncode != 0 or not os.path.isfile(exe):
        raise RuntimeError("Verilator build failed:\n" + r.stdout[-4000:] + r.stderr[-4000:])
    return exe


class Chip:
    """One running simulation of the whole chip (study harness)."""

    def __init__(self) -> None:
        self.proc = subprocess.Popen([build()], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     text=True, bufsize=1)

    def _cmd(self, line: str) -> str:
        self.proc.stdin.write(line + "\n")
        self.proc.stdin.flush()
        reply = self.proc.stdout.readline().strip()
        if not reply or reply.startswith("error"):
            raise RuntimeError(f"harness: {line[:60]!r} -> {reply!r}")
        return reply

    def close(self) -> None:
        if self.proc.poll() is None:
            try:
                self.proc.stdin.write("Q\n")
                self.proc.stdin.flush()
                self.proc.wait(timeout=10)
            except Exception:
                self.proc.kill()
                self.proc.wait()
        for pipe in (self.proc.stdin, self.proc.stdout):
            if pipe:
                pipe.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def run(self, n: int) -> None:
        self._cmd(f"R {int(n)}")

    def wr(self, addr: int, data: int) -> None:
        self._cmd(f"W {addr & 0x7F:x} {data & 0xFFFFFFFF:x}")

    def rd(self, addr: int) -> int:
        return int(self._cmd(f"X {addr & 0x7F:x}"), 16)

    def net(self, pins: int) -> None:
        self._cmd(f"N {pins & 0xFF:x}")

    def skew(self, pin: int, cycles: int) -> None:
        self._cmd(f"S {pin} {cycles}")

    def cycle(self) -> int:
        return int(self._cmd("C"))

    def dump(self, full: bool = False):
        self.proc.stdin.write("D\n")
        self.proc.stdin.flush()
        out = []
        while True:
            line = self.proc.stdout.readline().strip()
            if line == "end":
                break
            c, *rest = line.split()
            out.append((int(c), *(int(x, 16) for x in rest)))
        return out if full else [x[:3] for x in out]

    def load_program(self, image, origin: int = 0) -> None:
        self.wr(MADDR, origin & 0x1FF)
        for w in image:
            self.wr(MDATA, w)
        self.wr(MADDR, origin & 0x1FF)
        back = [self.rd(MDATA) for _ in image]
        if back != list(image):
            raise RuntimeError("program memory read-back mismatch")

    def push(self, lane: int, words, in_size: int = 64) -> None:
        q = self.rd(lane_reg(lane, QIN))
        wr_p = q & 0x7F
        for k, w in enumerate(words):
            idx = (wr_p + k) & (in_size - 1)
            self.wr(MADDR, 0x200 | (lane << 7) | idx)
            self.wr(MDATA, w)
        self.wr(lane_reg(lane, QIN), (wr_p + len(words)) & (2 * in_size - 1))

    def drain(self, lane: int, out_size: int = 64):
        q = self.rd(lane_reg(lane, QOUT))
        wr_p, rd_p = (q >> 8) & 0x7F, q & 0x7F
        m = 2 * out_size - 1
        cnt = (wr_p - rd_p) & m
        base = 0x200 | (lane << 7) | (128 - out_size)
        words = []
        for k in range(cnt):
            self.wr(MADDR, base | ((rd_p + k) & (out_size - 1)))
            words.append(self.rd(MDATA))
        self.wr(lane_reg(lane, QOUT), wr_p)
        return words

    def start_lane(self, lane: int, pc: int, qcfg: int = 0xF) -> None:
        self.wr(lane_reg(lane, CTRL), 0)
        self.wr(lane_reg(lane, QCFG), qcfg)
        self.wr(lane_reg(lane, QIN), 0)
        self.wr(lane_reg(lane, QOUT), 0)
        self.wr(lane_reg(lane, PC), pc)
        self.wr(lane_reg(lane, CTRL), 1)

    def map(self, ch: int, pin: int) -> None:
        self._cmd(f"M {ch} {pin}")

    def pull(self, pin: int, v: int) -> None:
        self._cmd(f"U {pin} {v}")

    def wave(self, runs) -> None:
        """runs: iterable of (drive mask, value mask, count)."""
        buf = []
        for d, v, n in runs:
            if n > 0:
                buf.append(f"{d:x} {v:x} {n}")
            if len(buf) >= 2000:
                self._cmd("V " + " ".join(buf))
                buf = []
        if buf:
            self._cmd("V " + " ".join(buf))

    def play(self, extra: int = 0) -> None:
        self._cmd(f"P {int(extra)}")

    def contention(self) -> int:
        return int(self._cmd("K"))

    def tracing(self, on: bool) -> None:
        self._cmd(f"T {1 if on else 0}")

    def canbus(self, dtx: int, drx: int) -> None:
        self._cmd(f"B {dtx} {drx}")

    def mem_read(self, lane: int, off: int, n: int):
        self.wr(MADDR, 0x200 | (lane << 7) | off)
        return [self.rd(MDATA) for _ in range(n)]

    def mem_write(self, lane: int, off: int, words) -> None:
        self.wr(MADDR, 0x200 | (lane << 7) | off)
        for w in words:
            self.wr(MDATA, w)


def runs_from_levels(levels, ch_mask=1):
    """Per-cycle list of 0/1 (or None = Z) for one channel -> RLE runs."""
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
