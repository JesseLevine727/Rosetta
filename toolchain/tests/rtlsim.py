"""Drive the Rosetta chip RTL under Verilator (tests only, not a test module).

    sim = rtlsim.Chip()          # builds once per RTL/harness version, then starts
    sim.wr(addr, data); sim.rd(addr); sim.run(n); sim.wire(uo, ui); sim.close()

The build goes to a cache directory outside the repository:
<ROSETTA_SIM_BUILD or tmp>/rosetta_verilator_<hash of the RTL and harness sources>. `find_verilator()` returns
None when no Verilator is installed, so tests can skip cleanly.

Host register map and loader sequences: docs/blocks/host.md 4; the helpers
below follow the FPGA scripts (rosetta-fpga results/loopback_20261114).
"""
from __future__ import annotations

import glob
import hashlib
import os
import shutil
import subprocess
import tempfile

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SRC = os.path.join(REPO, "src")
HARNESS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rtl", "rosetta_tb.cpp")
TOP = "tt_um_jesselevine727_rosetta"
OSS_CAD = "/home/elfo/tools/oss-cad-suite-dl/oss-cad-suite"

# host.md 4
MADDR, MDATA = 0x01, 0x02
XIN0, XIN1, XOUT0, XOUT1, XOUT2, XCTL = 0x40, 0x41, 0x42, 0x43, 0x44, 0x45
CTRL, PC, FLAGS, R10, R32, QCFG, QIN, QOUT = range(8)


def lane_reg(lane: int, r: int) -> int:
    return 0x10 + 8 * lane + r


def find_verilator() -> str | None:
    for cand in (os.environ.get("ROSETTA_VERILATOR"), os.path.join(OSS_CAD, "bin", "verilator"),
                 shutil.which("verilator")):
        if cand and os.path.isfile(cand) and os.access(cand, os.X_OK):
            return cand
    return None


def _env() -> dict:
    env = dict(os.environ)
    env.pop("VERILATOR_ROOT", None)          # the OSS CAD Suite binary finds its own
    vbin = find_verilator()
    if vbin:
        env["PATH"] = os.path.dirname(vbin) + os.pathsep + env.get("PATH", "")
    return env


def rtl_sources() -> list[str]:
    files = sorted(glob.glob(os.path.join(SRC, "rosetta_*.sv")))
    return [f for f in files if not f.endswith("rosetta_sram_bb.v")]


def build() -> str:
    """Build the simulator if needed; return the executable's path."""
    vbin = find_verilator()
    if vbin is None:
        raise RuntimeError("Verilator not found")
    srcs = rtl_sources()
    h = hashlib.sha256()
    for f in srcs + [HARNESS]:
        with open(f, "rb") as fh:
            h.update(os.path.basename(f).encode() + fh.read())
    base = os.environ.get("ROSETTA_SIM_BUILD") or tempfile.gettempdir()
    out = os.path.join(base, "rosetta_verilator_" + h.hexdigest()[:16])   # rebuilt on any change
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
    """One running simulation of the whole chip."""

    def __init__(self) -> None:
        self.proc = subprocess.Popen([build()], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     text=True, bufsize=1)
        self.changes: list[tuple[int, int, int]] = []     # (cycle, uo, ui)

    def _cmd(self, line: str) -> str:
        assert self.proc.stdin and self.proc.stdout
        self.proc.stdin.write(line + "\n")
        self.proc.stdin.flush()
        reply = self.proc.stdout.readline().strip()
        if not reply or reply.startswith("error"):
            raise RuntimeError(f"harness: {line!r} -> {reply!r}")
        return reply

    def close(self) -> None:
        if self.proc.poll() is None:
            try:
                assert self.proc.stdin
                self.proc.stdin.write("Q\n")
                self.proc.stdin.flush()
                self.proc.wait(timeout=10)
            except Exception:
                self.proc.kill()
                self.proc.wait()
        for pipe in (self.proc.stdin, self.proc.stdout):
            if pipe:
                pipe.close()

    def __enter__(self) -> "Chip":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # -- harness commands
    def run(self, n: int) -> None:
        self._cmd(f"R {int(n)}")

    def wr(self, addr: int, data: int) -> None:
        self._cmd(f"W {addr & 0x7F:x} {data & 0xFFFFFFFF:x}")

    def rd(self, addr: int) -> int:
        return int(self._cmd(f"X {addr & 0x7F:x}"), 16)

    def wire(self, uo: int, ui: int) -> None:
        self._cmd(f"L {uo} {ui}")

    def net(self, pins: int) -> None:
        """Join the uio pins in the mask into one open-drain net (pull-up)."""
        self._cmd(f"N {pins & 0xFF:x}")

    def skew(self, pin: int, cycles: int) -> None:
        """uio pin `pin` reads its net `cycles` late (a slow synchroniser)."""
        self._cmd(f"S {pin} {cycles}")

    def fault(self, offset: int, length: int) -> None:
        self._cmd(f"F {offset} {length}")

    def cycle(self) -> int:
        return int(self._cmd("C"))

    def dump(self, full: bool = False) -> list[tuple]:
        """Pin changes since the last dump: (cycle, uo, ui), or with `full`
        (cycle, uo, ui, uio_out, uio_oe, uio_in)."""
        assert self.proc.stdin and self.proc.stdout
        self.proc.stdin.write("D\n")
        self.proc.stdin.flush()
        out = []
        while True:
            line = self.proc.stdout.readline().strip()
            if line == "end":
                break
            c, *rest = line.split()
            out.append((int(c), *(int(x, 16) for x in rest)))
        self.changes.extend(x[:3] for x in out)
        return out if full else [x[:3] for x in out]

    # -- loader helpers (host.md 4 "typical loader sequences")
    def load_program(self, image: list[int], origin: int = 0) -> None:
        self.wr(MADDR, origin & 0x1FF)
        for w in image:
            self.wr(MDATA, w)
        self.wr(MADDR, origin & 0x1FF)
        back = [self.rd(MDATA) for _ in image]
        if back != list(image):
            raise RuntimeError("program memory read-back mismatch")

    def push(self, lane: int, words: list[int], in_size: int = 64) -> None:
        q = self.rd(lane_reg(lane, QIN))
        wr_p = q & 0x7F
        for k, w in enumerate(words):
            idx = (wr_p + k) & (in_size - 1)
            self.wr(MADDR, 0x200 | (lane << 7) | idx)
            self.wr(MDATA, w)
        self.wr(lane_reg(lane, QIN), (wr_p + len(words)) & (2 * in_size - 1))

    def drain(self, lane: int, out_size: int = 64) -> list[int]:
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
        self.wr(lane_reg(lane, QCFG), qcfg)          # 0xF: 64-word host-in and host-out queues
        self.wr(lane_reg(lane, QIN), 0)
        self.wr(lane_reg(lane, QOUT), 0)
        self.wr(lane_reg(lane, PC), pc)
        self.wr(lane_reg(lane, CTRL), 1)
