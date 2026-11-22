"""Lockstep mode of the Rosetta harness: the board as a cycle-exact twin of
the Verilator harness (toolchain/tests/rtl/rosetta_tb.cpp).

    from rosetta_board import RosettaBoard
    from lockstep import Lockstep
    ls = Lockstep(RosettaBoard())        # program the FPGA once
    ls.begin()                           # fresh chip: reload the PL, reset as the C++ harness does
    ls.host_write(0x01, 0); ls.host_read(0x00)  -> 0x52530401
    ls.run(1000); ls.wire(2, 3); ls.net(0x15); ls.skew(4, 2); ls.fault(4680, 520)
    ls.cycle(); ls.dump()                -> [(cycle, uo, ui, uio_out, uio_oe, uio_in), ...]

The chip clock runs only for run() and for host frames, which the harness's
host engine plays on exactly the chip edges the C++ harness uses. Every pin
change is recorded by the harness's change trace with its chip-cycle number,
so dump() returns what the C++ harness's D command returns. README
"Lockstep mode" has the register map.
"""
import time

import numpy as np

from rosetta_board import (CH1, CH2, C_ENA, C_RST_N, S_LOCKED)

__all__ = ["Lockstep", "LockstepError"]

# LS_CTRL (gpio_ls ch1)
LS_RUN_GO, LS_HOST_GO, LS_TRACE_CLR, LS_TRACE_EN, LS_CYC_CLR = (1 << i for i in range(5))
LS_LOOP_DIR, LS_UIO_ISO, LS_BP_EN, LS_FAULT_ARM = (1 << i for i in range(5, 9))
LS_FAULT_SRC_SHIFT = 12
LS_HOST_ENG = 1 << 16
# LS_STATUS (gpio_ls ch2)
LSS_RUN_BUSY, LSS_HOST_BUSY, LSS_OVERFLOW, LSS_F_ARMED, LSS_F_SEEN, LSS_ASYNC_BUSY, LSS_CE, \
    LSS_BP, LSS_FIRST = (1 << i for i in range(9))
TRC_DEPTH = 16384
HOST_FRAME = {False: 664, True: 792}      # chip cycles of a write / read frame
ASYNC_CLK = 125_000_000                   # the async UART's reference (board oscillator)


class LockstepError(RuntimeError):
    pass


class Lockstep:
    def __init__(self, board, timeout=10.0):
        self.b = board
        self.g = board.gpio
        self.trc = board.trc
        self.timeout = timeout
        self._ls = 0
        self.rp = 0
        self.chunks = []
        self.nets = []
        self.wires = []
        self.cyc = 0

    # ---------------------------------------------------------- low level
    def _set_ls(self, v):
        self._ls = v & 0xFFFFFFFF
        self.g["ls"].write(CH1, self._ls)

    def _toggle(self, bit):
        self._set_ls(self._ls ^ bit)

    def status(self):
        return self.g["ls"].read(CH2)

    def hw_cycle(self):
        lo = self.g["cyc"].read(CH1)
        hi = self.g["cyc"].read(CH2)
        lo2 = self.g["cyc"].read(CH1)
        if lo2 < lo:                              # carried between the reads
            hi = self.g["cyc"].read(CH2)
            lo = lo2
        return (hi & 0xFFFF) << 32 | lo

    def _wait_cycle(self, target, busy_bit, what):
        """Poll until the chip has had exactly `target` edges and `busy_bit`
        is clear, draining the trace meanwhile (it pauses the chip when full)."""
        t_end = time.monotonic() + self.timeout
        while True:
            st = self.status()
            if ((st >> 16) - self.rp) & 0xFFFF >= 4096:
                self.drain()
            if not st & busy_bit:
                c = self.hw_cycle()
                if c == target:
                    return
                if c > target:
                    raise LockstepError(f"{what}: chip cycle {c} passed the target {target}")
            if time.monotonic() > t_end:
                raise LockstepError(f"{what}: timeout at cycle {self.hw_cycle()} "
                                    f"(target {target}, LS_STATUS {st:#010x})")

    # ---------------------------------------------------------- session
    def begin(self, reload=True):
        """A fresh chip, as a new Verilator harness process: reload the PL
        (every flop and RAM back to its configuration value), set up the
        lockstep harness, then 10 chip edges with rst_n low and 10 with rst_n
        high (cycle counter 20 afterwards)."""
        b = self.b
        if reload:
            b.overlay.download()
        t_end = time.monotonic() + 2.0
        while not b.status_word() & S_LOCKED:
            if time.monotonic() > t_end:
                raise LockstepError("MMCM not locked after reload")
        if (b.status_word() >> 24) != 0x52:
            raise LockstepError("harness ID mismatch after reload")
        b._set_ctrl(C_ENA)                              # rst_n 0, clock stopped, headers off
        b.gpio["pins"].write(CH1, 0xFF00)               # ui 0, uio 0xff (unconnected pins read 1)
        b._drive = 0xFF00
        b._src, b._src2, b._loop = 0, 0, [0, 0]
        b._write_routes()
        for name in ("net", "skew", "fault", "async"):
            self.g[name].write(CH1, 0)
        for name in ("skew", "fault", "async"):
            self.g[name].write(CH2, 0)
        self._ls = self.g["ls"].read(CH1) if not reload else 0
        self._set_ls(LS_HOST_ENG | LS_BP_EN | LS_UIO_ISO | LS_LOOP_DIR | LS_TRACE_EN
                     | (self._ls & (LS_RUN_GO | LS_HOST_GO | LS_TRACE_CLR | LS_CYC_CLR | LS_FAULT_ARM)))
        self._toggle(LS_TRACE_CLR)
        self._toggle(LS_CYC_CLR)
        self.rp = self.status() >> 16
        self.g["net"].write(CH2, self.rp)
        self.chunks, self.nets, self.wires = [], [], []
        self._skew = [0, 0]
        if self.hw_cycle() != 0:
            raise LockstepError("cycle counter did not clear")
        self.cyc = 0
        self.run(10)
        b._set_ctrl(C_ENA | C_RST_N)
        self.run(10)

    # ---------------------------------------------------------- the C++ harness's commands
    def run(self, n):                                   # R n
        n = int(n)
        if n > 0:
            self.g["run"].write(CH1, n)
            self._toggle(LS_RUN_GO)
            self.cyc += n
            self._wait_cycle(self.cyc, LSS_RUN_BUSY, f"run {n}")
        self.drain()

    def _frame(self, cmd, data):
        self.g["run"].write(CH2, cmd & 0xFF)
        self.g["host"].write(CH1, data & 0xFFFFFFFF)
        self._toggle(LS_HOST_GO)
        self.cyc += HOST_FRAME[bool(cmd & 0x80)]
        self._wait_cycle(self.cyc, LSS_HOST_BUSY, f"host frame {cmd:#04x}")
        self.drain()

    def host_write(self, addr, data):                   # W a d
        self._frame(addr & 0x7F, data)

    def host_read(self, addr):                          # X a
        self._frame(0x80 | (addr & 0x7F), 0)
        return self.g["host"].read(CH2)

    def wire(self, uo, ui):                             # L f t: uo[f] -> ui[t], no delay
        if not (0 <= uo < 8 and 0 <= ui < 8):
            raise LockstepError("wire: pins 0..7")
        self.b.route(f"ui{ui}", f"uo{uo}")
        self.wires.append((uo, ui))
        if len(self.wires) == 1:                        # the fault watches the first wire's source
            self._set_ls((self._ls & ~(0xF << LS_FAULT_SRC_SHIFT)) | (uo << LS_FAULT_SRC_SHIFT))

    def net(self, mask):                                # N mask
        if len(self.nets) >= 4:
            raise LockstepError("the harness has 4 nets")
        self.nets.append(mask & 0xFF)
        v = 0
        for k, m in enumerate(self.nets):
            v |= m << (8 * k)
        self.g["net"].write(CH1, v)

    def skew(self, pin, cycles):                        # S pin d
        if not (0 <= pin < 8 and 0 <= cycles < 64):
            raise LockstepError("skew: pin 0..7, 0..63 cycles")
        ch = CH1 if pin < 4 else CH2
        sh = 8 * (pin % 4)
        cur = getattr(self, "_skew", [0, 0])
        k = 0 if pin < 4 else 1
        cur[k] = (cur[k] & ~(0x3F << sh)) | (cycles << sh)
        self._skew = cur
        self.g["skew"].write(ch, cur[k])

    def fault(self, offset, length):                    # F off len
        self.g["fault"].write(CH1, offset & 0xFFFFFFFF)
        self.g["fault"].write(CH2, length & 0xFFFFFFFF)
        self._toggle(LS_FAULT_ARM)

    def cycle(self):                                    # C
        c = self.hw_cycle()
        if c != self.cyc:
            raise LockstepError(f"cycle counter {c}, expected {self.cyc}")
        return c

    def drain(self):
        st = self.status()
        if st & LSS_OVERFLOW:
            raise LockstepError("trace overflow")
        wp = st >> 16
        n = (wp - self.rp) & 0xFFFF
        if n == 0:
            return
        i0 = self.rp % TRC_DEPTH
        first = min(n, TRC_DEPTH - i0)
        words = []
        for t in self.trc:
            a = t.array
            w = np.array(a[i0:i0 + first], dtype=np.uint32)
            if first < n:
                w = np.concatenate([w, np.array(a[:n - first], dtype=np.uint32)])
            words.append(w)
        self.chunks.append(words)
        self.rp = wp
        self.g["net"].write(CH2, wp)

    def dump_arrays(self):
        """(cycle, w1, w2) numpy arrays of the changes since the last dump."""
        self.drain()
        if not self.chunks:
            e = np.zeros(0, dtype=np.uint32)
            return np.zeros(0, dtype=np.uint64), e, e
        w0 = np.concatenate([c[0] for c in self.chunks])
        w1 = np.concatenate([c[1] for c in self.chunks])
        w2 = np.concatenate([c[2] for c in self.chunks])
        self.chunks = []
        cyc = (w1 & 0xFFFF).astype(np.uint64) << np.uint64(32) | w0.astype(np.uint64)
        return cyc, w1, w2

    def dump(self):                                     # D
        cyc, w1, w2 = self.dump_arrays()
        ui = (w1 >> 16) & 0xFF
        return [(int(c), int(b2 & 0xFF), int(u), int(b2 >> 8 & 0xFF), int(b2 >> 16 & 0xFF), int(b2 >> 24))
                for c, u, b2 in zip(cyc.tolist(), ui.tolist(), w2.tolist())]

    def dump_lines(self):
        cyc, w1, w2 = self.dump_arrays()
        ui = ((w1 >> 16) & 0xFF).tolist()
        return [f"{c} {w & 0xFF:02x} {u:02x} {w >> 8 & 0xFF:02x} {w >> 16 & 0xFF:02x} {w >> 24:02x}"
                for c, u, w in zip(cyc.tolist(), ui, w2.tolist())]

    # ---------------------------------------------------------- asynchronous UART far end
    def async_uart(self, data, baud, ui, parity="none", stop2=False, gap=0, wait=True):
        """Send `data` (<= 64 bytes) into ui[ui] from the harness's UART
        transmitter clocked by the board's own 125 MHz oscillator, at `baud`
        (phase accumulator: rate = 125 MHz * inc / 2**32). The chip clock
        must free-run meanwhile (board.clock_run(True)). Returns the rate
        actually set."""
        data = bytes(data)
        if not 1 <= len(data) <= 64:
            raise LockstepError("async_uart: 1..64 bytes")
        inc = int(round(baud * 2**32 / ASYNC_CLK))
        par = {"none": 0, "even": 1, "odd": 2}[parity]
        base = (len(data) - 1) << 16 | par << 22 | int(stop2) << 24 | (ui & 7) << 25 | 1 << 28 \
            | (gap & 7) << 29
        cmd = self.g["async"].read(CH2) & ((1 << 14) | (1 << 15))     # keep the toggle bits
        for i, d in enumerate(data):
            cmd ^= 1 << 14
            self.g["async"].write(CH2, base | (cmd & 0xC000) | i << 8 | d)
        self.g["async"].write(CH1, inc)
        cmd ^= 1 << 15
        self.g["async"].write(CH2, base | (cmd & 0xC000))
        rate = ASYNC_CLK * inc / 2**32
        if wait:
            bits = 10 + (par != 0) + int(stop2) + gap
            time.sleep(len(data) * bits / rate + 0.002)
            t_end = time.monotonic() + 2.0
            while self.status() & LSS_ASYNC_BUSY:
                if time.monotonic() > t_end:
                    raise LockstepError("async UART did not finish")
        return rate

    def async_route(self, ui):
        """Connect the async UART (idle high) to ui[ui] without sending."""
        cmd = self.g["async"].read(CH2) & ~(0xF << 25)
        self.g["async"].write(CH2, cmd | (ui & 7) << 25 | 1 << 28)

    def async_release(self):
        """Disconnect the async UART from the chip input (route off)."""
        cmd = self.g["async"].read(CH2)
        self.g["async"].write(CH2, cmd & ~(1 << 28))
