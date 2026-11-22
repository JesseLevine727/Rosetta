"""Drive the Rosetta chip RTL, under Verilator or on the FPGA board (tests
only, not a test module).

    sim = rtlsim.Chip()          # a fresh chip: a new simulation, or the board reloaded
    sim.wr(addr, data); sim.rd(addr); sim.run(n); sim.wire(uo, ui); sim.close()

Both backends speak the same line protocol (toolchain/tests/rtl/rosetta_tb.cpp
documents it) and behave identically to the chip edge:

* `verilator` (default): the C++ harness rtl/rosetta_tb.cpp around the RTL.
  The build goes to a cache directory outside the repository:
  <ROSETTA_SIM_BUILD or tmp>/rosetta_verilator_<hash of the RTL and harness sources>.
* `board` (ROSETTA_BACKEND=board): the PYNQ-Z1 FPGA harness in lockstep mode,
  through board_server.py on the board over ssh (/home/elfo/tools/rosetta-fpga,
  README "Lockstep mode"). The harness plays the same host-SPI frames on the
  same chip edges, joins the same nets and records every pin change with its
  chip-cycle number, so every reply, including `dump()`, should match the
  Verilator backend exactly. ROSETTA_BOARD (default xilinx@10.0.0.82) and
  ROSETTA_BOARD_DIR (default /home/xilinx/rosetta/regress) locate it.

`available()` says whether the selected backend can run here, so tests can
skip cleanly (`find_verilator()` returns None without Verilator).
ROSETTA_TRANSCRIPT=<dir> writes every session's commands and replies to
<dir>/<test id>.<n>.txt; the board regression compares the two backends'
transcripts line by line (rosetta-fpga/regress.py).

Host register map and loader sequences: docs/blocks/host.md 4; the helpers
below follow the FPGA scripts (rosetta-fpga results/loopback_20261114).
"""
from __future__ import annotations

import atexit
import glob
import hashlib
import inspect
import os
import re
import shutil
import subprocess
import tempfile
import unittest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SRC = os.path.join(REPO, "src")
HARNESS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rtl", "rosetta_tb.cpp")
TOP = "tt_um_jesselevine727_rosetta"
OSS_CAD = "/home/elfo/tools/oss-cad-suite-dl/oss-cad-suite"

BOARD = os.environ.get("ROSETTA_BOARD", "xilinx@10.0.0.82")
BOARD_DIR = os.environ.get("ROSETTA_BOARD_DIR", "/home/xilinx/rosetta/regress")
BOARD_PYTHON = "/usr/local/share/pynq-venv/bin/python"

# host.md 4
MADDR, MDATA = 0x01, 0x02
XIN0, XIN1, XOUT0, XOUT1, XOUT2, XCTL = 0x40, 0x41, 0x42, 0x43, 0x44, 0x45
CTRL, PC, FLAGS, R10, R32, QCFG, QIN, QOUT = range(8)


def lane_reg(lane: int, r: int) -> int:
    return 0x10 + 8 * lane + r


def backend() -> str:
    b = os.environ.get("ROSETTA_BACKEND", "verilator").strip().lower()
    if b not in ("verilator", "board"):
        raise ValueError(f"ROSETTA_BACKEND={b!r}: use verilator or board")
    return b


def find_verilator() -> str | None:
    for cand in (os.environ.get("ROSETTA_VERILATOR"), os.path.join(OSS_CAD, "bin", "verilator"),
                 shutil.which("verilator")):
        if cand and os.path.isfile(cand) and os.access(cand, os.X_OK):
            return cand
    return None


_board_ok: tuple[bool, str] | None = None


def _board_check() -> tuple[bool, str]:
    """The board answers over ssh and has the server installed (cached)."""
    global _board_ok
    if _board_ok is None:
        try:
            r = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", BOARD,
                                f"test -f {BOARD_DIR}/board_server.py && test -f {BOARD_DIR}/rosetta.bit"],
                               capture_output=True, text=True, timeout=20)
            _board_ok = (r.returncode == 0,
                         "" if r.returncode == 0 else f"board {BOARD} not reachable or server not deployed")
        except (OSError, subprocess.SubprocessError) as e:
            _board_ok = (False, f"board {BOARD} not reachable ({e})")
    return _board_ok


def available() -> bool:
    """The selected backend can run here (Verilator installed / board up)."""
    if backend() == "board":
        return _board_check()[0]
    return find_verilator() is not None


def unavailable_reason() -> str:
    if backend() == "board":
        return _board_check()[1] or "board available"
    return "Verilator not installed"


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


# ---------------------------------------------------------------- transport

class _Link:
    """A line-protocol peer: one reply line per command (D: lines up to 'end').
    Commands that only answer 'ok' are pipelined: their replies are read
    before the next command that returns data, so a slow link (ssh) costs one
    round trip per value instead of one per command. Any 'error' reply raises
    at that point."""

    def __init__(self, argv: list[str], what: str) -> None:
        self.what = what
        self.proc = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     text=True, bufsize=1)
        self.pending: list[str] = []          # commands whose 'ok' is still unread
        self.log = None                       # transcript callback(cmd, reply_lines)
        self.hello = ""

    def _send(self, line: str) -> None:
        assert self.proc.stdin
        self.proc.stdin.write(line + "\n")

    def _readline(self, line: str) -> str:
        assert self.proc.stdout
        reply = self.proc.stdout.readline()
        if not reply:
            raise RuntimeError(f"{self.what}: no reply to {line!r} (exit code {self.proc.poll()})")
        return reply.strip()

    def flush_acks(self) -> None:
        if not self.pending:
            return
        assert self.proc.stdin
        self.proc.stdin.flush()
        while self.pending:
            line = self.pending.pop(0)
            reply = self._readline(line)
            if self.log:
                self.log(line, [reply])
            if reply != "ok":
                self.pending.clear()
                raise RuntimeError(f"{self.what}: {line!r} -> {reply!r}")

    def ack(self, line: str) -> None:
        """A command answered by 'ok'."""
        self._send(line)
        self.pending.append(line)
        if len(self.pending) >= 256:
            self.flush_acks()

    def value(self, line: str) -> str:
        """A command answered by one data line."""
        self.flush_acks()
        assert self.proc.stdin
        self._send(line)
        self.proc.stdin.flush()
        reply = self._readline(line)
        if self.log:
            self.log(line, [reply])
        if not reply or reply.startswith("error"):
            raise RuntimeError(f"{self.what}: {line!r} -> {reply!r}")
        return reply

    def lines(self, line: str) -> list[str]:
        """A command answered by lines up to 'end'."""
        self.flush_acks()
        assert self.proc.stdin
        self._send(line)
        self.proc.stdin.flush()
        out = []
        while True:
            r = self._readline(line)
            if r.startswith("error"):
                raise RuntimeError(f"{self.what}: {line!r} -> {r!r}")
            if r == "end":
                break
            out.append(r)
        if self.log:
            self.log(line, out + ["end"])
        return out

    def close(self) -> None:
        try:
            self.flush_acks()
        except Exception:
            pass
        if self.proc.poll() is None:
            try:
                assert self.proc.stdin
                self.proc.stdin.write("Q\n")
                self.proc.stdin.flush()
                self.proc.wait(timeout=20)
            except Exception:
                self.proc.kill()
                self.proc.wait()
        for pipe in (self.proc.stdin, self.proc.stdout):
            if pipe:
                pipe.close()


_board_link: _Link | None = None


def board_link() -> _Link:
    """The one ssh session to board_server.py, shared by every Chip of this
    process (the server reloads the FPGA for each new Chip)."""
    global _board_link
    if _board_link is None or _board_link.proc.poll() is not None:
        pw = os.environ.get("ROSETTA_BOARD_SUDO_PASS", "xilinx")
        remote = (f"cd {BOARD_DIR} && {{ echo {pw}; cat; }} | sudo -S -p '' env XILINX_XRT=/usr "
                  f"PYTHONDONTWRITEBYTECODE=1 {BOARD_PYTHON} -u board_server.py")
        link = _Link(["ssh", "-o", "BatchMode=yes", "-o", "ServerAliveInterval=15", BOARD, remote],
                     f"board {BOARD}")
        hello = link._readline("(server start)")
        if not hello.startswith("rosetta-board"):
            raise RuntimeError(f"board server did not start: {hello!r}")
        link.hello = hello
        atexit.register(link.close)
        _board_link = link
    return _board_link


# ---------------------------------------------------------------- transcripts

_session_count: dict[str, int] = {}


def _test_id() -> str:
    for fr in inspect.stack():
        obj = fr.frame.f_locals.get("self")
        if isinstance(obj, unittest.TestCase):
            return obj.id()
    return "session"


def _transcript_file() -> str | None:
    d = os.environ.get("ROSETTA_TRANSCRIPT")
    if not d:
        return None
    os.makedirs(d, exist_ok=True)
    tid = re.sub(r"[^A-Za-z0-9_.-]", "_", _test_id())
    n = _session_count.get(tid, 0)
    _session_count[tid] = n + 1
    return os.path.join(d, f"{tid}.{n}.txt")


# ---------------------------------------------------------------- the chip

class Chip:
    """One fresh chip: a new simulation, or the board reloaded and reset."""

    def __init__(self) -> None:
        self.backend = backend()
        if self.backend == "board":
            self.link = board_link()
            self.link.flush_acks()
        else:
            self.link = _Link([build()], "harness")
        self.changes: list[tuple[int, int, int]] = []     # (cycle, uo, ui)
        self._tfile = None
        path = _transcript_file()
        if path:
            self._tfile = open(path, "w")
            self.link.log = self._log
        if self.backend == "board":
            self.link.value("B")                          # new session: reload, reset

    def _log(self, cmd: str, reply: list[str]) -> None:
        if self._tfile and cmd != "B":
            self._tfile.write(f"> {cmd}\n")
            for r in reply:
                self._tfile.write(f"{r}\n")

    def close(self) -> None:
        if self.backend == "board":
            try:
                self.link.flush_acks()
            finally:
                self.link.log = None
        else:
            self.link.close()
        if self._tfile:
            self._tfile.close()
            self._tfile = None

    def __enter__(self) -> "Chip":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # -- harness commands
    def run(self, n: int) -> None:
        self.link.ack(f"R {int(n)}")

    def wr(self, addr: int, data: int) -> None:
        self.link.ack(f"W {addr & 0x7F:x} {data & 0xFFFFFFFF:x}")

    def rd(self, addr: int) -> int:
        return int(self.link.value(f"X {addr & 0x7F:x}"), 16)

    def wire(self, uo: int, ui: int) -> None:
        self.link.ack(f"L {uo} {ui}")

    def net(self, pins: int) -> None:
        """Join the uio pins in the mask into one open-drain net (pull-up)."""
        self.link.ack(f"N {pins & 0xFF:x}")

    def skew(self, pin: int, cycles: int) -> None:
        """uio pin `pin` reads its net `cycles` late (a slow synchroniser)."""
        self.link.ack(f"S {pin} {cycles}")

    def fault(self, offset: int, length: int) -> None:
        self.link.ack(f"F {offset} {length}")

    def cycle(self) -> int:
        return int(self.link.value("C"))

    def dump(self, full: bool = False) -> list[tuple]:
        """Pin changes since the last dump: (cycle, uo, ui), or with `full`
        (cycle, uo, ui, uio_out, uio_oe, uio_in)."""
        out = []
        for line in self.link.lines("D"):
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
        # out_rd = wr_p; [14:8] repeats out_wr, which the register takes only
        # from a halted lane (host.md 4): draining a halted lane keeps its queue
        self.wr(lane_reg(lane, QOUT), wr_p << 8 | wr_p)
        return words

    def start_lane(self, lane: int, pc: int, qcfg: int = 0xF) -> None:
        self.wr(lane_reg(lane, CTRL), 0)
        self.wr(lane_reg(lane, QCFG), qcfg)          # 0xF: 64-word host-in and host-out queues
        self.wr(lane_reg(lane, QIN), 0)
        self.wr(lane_reg(lane, QOUT), 0)
        self.wr(lane_reg(lane, PC), pc)
        self.wr(lane_reg(lane, CTRL), 1)
