"""Cycle-accurate reference model of one Rosetta lane (docs/isa.md v0.2).

It models one lane plus the parts of the shared logic that lane sees:
program memory, its 128-word data region with the host-in and host-out
ring buffers, the four channels, the timestamp counter and its slot on the
five-slot wheel. `Lane.step(pins)` advances one clock cycle.

Where isa.md leaves the exact cycle open, the choice made here is written
as a named constant or a commented rule, and listed with its isa.md text in
toolchain/MODEL_DECISIONS.md (tags D1, D2, ... below refer to that file).

Not implemented (raise NotImplementedError): Manchester transmit and
receive coding.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Optional

from . import isa
from .isa import FIELD_KINDS, OPERANDS, PRESCALE_FACTORS

# --------------------------------------------------------------------------
# Timing constants (MODEL_DECISIONS.md section "Timing constants")
# --------------------------------------------------------------------------
SLOT_PERIOD = 5          # D1: wheel of 4 lanes + 1 host slot
EXEC_STAGE = 2           # D2: fetch in the slot cycle, decode +1, execute +2
INPUT_DELAY = 3          # D3: 2 synchroniser flops + 1 lane input register
MIN_COUNT = 1            # D4: PERIOD, PERIOD2 and RECOVER bit time of 0 act as 1
RECOVER_CAPTURE_NUM = 1  # D5: RECOVER capture at (PERIOD*prescale) * 1/2
RECOVER_CAPTURE_DEN = 2
PULSE_SATURATE = 0xFF    # D6: pulse durations saturate at 8 bits (THR are 8 bits)
RAW_SATURATE = 0xFFFF    # isa 3.2
DEFAULT_IN_QUEUE = (64, 32)    # D7: (base, size) words in the lane's region
DEFAULT_OUT_QUEUE = (96, 32)

DRIVE, SAMPLE, EXCHANGE, EXPECT = (FIELD_KINDS[k] for k in ("DRIVE", "SAMPLE", "EXCHANGE", "EXPECT"))
BAUD, EDGE, RECOVER, PULSE = (isa.TICK_MODES[k] for k in ("BAUD", "EDGE", "RECOVER", "PULSE"))
OP_HOSTQ, OP_CAP, OP_CRC, OP_MEM, OP_IMM, OP_SR, OP_NONE = (
    OPERANDS[k] for k in ("HOSTQ", "CAP", "CRC", "MEM", "IMM", "SR", "NONE"))
BIT_HW_MODE_REGS = range(0, 10)   # D8: SETM of these waits for the bit hardware


class Halted(Exception):
    """Raised inside the model to halt the lane (reserved or undefined code)."""


# --------------------------------------------------------------------------
# Check unit and stuffing: exactly docs/blocks/crc.md and docs/blocks/stuff.md
# --------------------------------------------------------------------------

class Crc:
    def __init__(self) -> None:
        self.state = 0
        self.poly = 0
        self.width = 1
        self.init_ones = False
        self.xorout = False

    @property
    def mask(self) -> int:
        return (1 << self.width) - 1

    def init(self) -> None:
        self.state = self.mask if self.init_ones else 0

    def update(self, bit: int) -> None:
        fb = ((self.state >> (self.width - 1)) & 1) ^ bit
        self.state = ((self.state << 1) & self.mask) ^ (self.poly & self.mask if fb else 0)

    @property
    def value(self) -> int:
        return self.state ^ (self.mask if self.xorout else 0)


class Stuffer:
    def __init__(self) -> None:
        self.n = 0
        self.equal_mode = False
        self.enabled = False
        self.clear()

    def clear(self) -> None:
        self.run = 0
        self.last: Optional[int] = None

    @property
    def stuff_next(self) -> bool:
        return self.enabled and self.n != 0 and self.run == self.n

    @property
    def stuff_bit(self) -> int:
        return (1 - self.last) if (self.equal_mode and self.last is not None) else 0

    def feed(self, bit: int) -> bool:
        """One wire bit; returns True on a stuffing violation."""
        was_stuff, want = self.stuff_next, self.stuff_bit
        if self.equal_mode:
            self.run = min(self.run + 1, 7) if (self.last is not None and bit == self.last) else 1
            self.last = bit
        else:
            self.run = min(self.run + 1, 7) if bit else 0
        return was_stuff and bit != want


# --------------------------------------------------------------------------
# Event detector (isa 4, EVC0-EVC3)
# --------------------------------------------------------------------------

class EventDetector:
    def __init__(self) -> None:
        self.configure(0)

    def configure(self, value: int) -> None:
        c = isa.mode_decode("EVC0", value)
        self.a, self.level, self.trigger = c["a"], c["level"], c["trigger"]
        self.b, self.b_must, self.k = c["b"], c["b_must"], c["k"]
        self.count: Optional[int] = None
        self.prev_cond = False          # D20: level triggers start fresh after SETM
        self.held_level = 0

    def step(self, inp: int, prev: int, prescale: int) -> bool:
        a = (inp >> self.a) & 1
        a_prev = (prev >> self.a) & 1
        b = (inp >> self.b) & 1
        b_ok = (self.b_must == 0 or (self.b_must == 1 and b == 1)
                or (self.b_must == 2 and b == 0))          # code 3: never (D21)
        t = self.trigger                                     # ruling 10 layout
        if t == 1:                                           # A at level L
            cond = a == self.level and b_ok
            start, hold = cond and not self.prev_cond, cond
            self.prev_cond = cond
        elif t == 2:                                         # A edge to level L
            start = a != a_prev and a == self.level and b_ok
            hold = a == self.level and b_ok
        elif t == 3:                                         # A any edge
            start = a != a_prev and b_ok
            hold = a == self.held_level and b_ok
        elif t == 4:                                         # A and B both low
            cond = a == 0 and b == 0
            start, hold = cond and not self.prev_cond, cond
            self.prev_cond = cond
        else:                                                # 0 and 5-7: off
            start, hold = False, False
        if start:
            self.count, self.held_level = 0, a
        elif self.count is not None:
            self.count = self.count + 1 if hold else None
        if self.count is not None and self.count >= self.k * prescale:
            self.count = None
            return True
        return False


# --------------------------------------------------------------------------
# Shared state seen by a lane
# --------------------------------------------------------------------------

class Channels:
    """C0-C3, 32 bits + valid, shared between lanes (arch 7)."""

    def __init__(self) -> None:
        self.valid = [False] * isa.N_CHANNELS
        self.value = [0] * isa.N_CHANNELS

    def put(self, c: int, v: int) -> bool:
        if self.valid[c]:
            return False
        self.valid[c], self.value[c] = True, v & 0xFFFF_FFFF
        return True

    def get(self, c: int) -> Optional[int]:
        if not self.valid[c]:
            return None
        self.valid[c] = False
        return self.value[c]


class Ring:
    """A host queue: ring buffer of 32-bit words inside the lane's region."""

    def __init__(self, mem: list[int], base: int, size: int) -> None:
        if not (0 <= base and base + size <= isa.DATA_WORDS_PER_LANE and size > 0):
            raise ValueError("queue outside the lane's 128-word region")
        self.mem, self.base, self.size = mem, base, size
        self.head = self.count = 0

    def empty(self) -> bool:
        return self.count == 0

    def full(self) -> bool:
        return self.count == self.size

    def free(self) -> int:
        return self.size - self.count

    def push(self, w: int) -> bool:
        if self.full():
            return False
        self.mem[self.base + (self.head + self.count) % self.size] = w & 0xFFFF_FFFF
        self.count += 1
        return True

    def pop(self) -> Optional[int]:
        if self.empty():
            return None
        w = self.mem[self.base + self.head]
        self.head = (self.head + 1) % self.size
        self.count -= 1
        return w


# --------------------------------------------------------------------------
# Bit-hardware operations: a field, a PIN, a RAW
# --------------------------------------------------------------------------

@dataclass
class FieldOp:
    kind: int
    width: int
    lsb: bool
    crc: bool
    stuff: bool
    src: int
    dst: int
    early: bool
    seek: bool
    tag: int
    value: int = 0                 # driven or expected value (CRC: latched at start)
    phase: str = "launch"          # "launch": waiting for a launch tick; "capture"
    bits: int = 0                  # data bits done
    acc: int = 0                   # received bits (or seek window)
    cur_bit: int = 0               # wire bit currently driven
    cur_stuff: bool = False
    prelaunched: bool = False      # `early` already drove the first bit
    released: bool = False
    trailing_stuff_done: bool = False
    ts: Optional[int] = None       # timestamp of the first capture tick (D17)
    complete: bool = False         # result delivered (last capture tick)
    tail: bool = False             # BAUD: complete, holding the hardware to the bit's end

    @property
    def mask(self) -> int:
        return (1 << self.width) - 1

    @property
    def drives(self) -> bool:
        return self.kind in (DRIVE, EXCHANGE)

    @property
    def receives(self) -> bool:
        return self.kind in (SAMPLE, EXCHANGE, EXPECT)


@dataclass
class PinOp:
    mask: int
    values: int
    enables: int
    align: bool
    wait: int
    applied: bool = False
    until: Optional[int] = None


@dataclass
class RawOp:
    drive: bool
    level: int
    count: int = 0                 # DRIVE: prescaled cycles
    dst: int = OP_NONE
    tag: int = 0
    prev_edge: int = 0
    until: Optional[int] = None


Op = FieldOp | PinOp | RawOp


@dataclass
class Write:
    """A field result waiting for room in its destination (D15)."""
    dst: int
    value: int
    tag: int
    ts: int


# --------------------------------------------------------------------------
# The lane
# --------------------------------------------------------------------------

class Lane:
    def __init__(self, program: list[int] | None = None, lane: int = 0, origin: int = 0,
                 channels: Channels | None = None,
                 in_queue: tuple[int, int] = DEFAULT_IN_QUEUE,
                 out_queue: tuple[int, int] = DEFAULT_OUT_QUEUE,
                 initial_inputs: int = 0b1111) -> None:
        self.prog = [0] * isa.PROGRAM_WORDS
        for i, w in enumerate(program or []):
            self.prog[(origin + i) % isa.PROGRAM_WORDS] = w
        self.lane = lane
        self.mem = [0] * isa.DATA_WORDS_PER_LANE
        self.inq = Ring(self.mem, *in_queue)
        self.outq = Ring(self.mem, *out_queue)
        self.channels = channels or Channels()
        self.regs = [0] * isa.N_REGS
        self.z = self.n = self.m = False
        self.pending = 0
        self.armed = [False] * isa.N_EVENTS
        self.ev_target = [0] * isa.N_EVENTS
        self.detectors = [EventDetector() for _ in range(isa.N_EVENTS)]
        self.modes = [0] * len(isa.MODE_REGS)
        self.crc = Crc()
        self.stuffer = Stuffer()
        self.sr = 0
        self.bits_kept = 0             # number of bits done of an abandoned field
        self.pc = origin
        self.running = False
        self.halt_reason = "not started"
        self.irq = False
        self.overflow = False
        self.cycle = 0
        self.pipe: deque[int] = deque([initial_inputs] * INPUT_DELAY)
        self.inp = self.prev_inp = initial_inputs
        self.out_val = [0, 0, 0, 0]
        self.out_en = [0, 0, 0, 0]
        self.sched: list[tuple[int, int, int, int]] = []   # (cycle, out, value, enable)
        # sequencer
        self.active: Optional[Op] = None
        self.parked: Optional[Op] = None
        self.writes: list[Write] = []
        self.wait_deadline: Optional[int] = None
        self.data_waiting = False
        # tick source
        self.baud_phase = "idle"       # "idle" | "to_capture" | "to_launch"
        self.baud_at = 0
        self.clk_released = False
        self.rec_count = 0
        self.last_edge = 0
        self.nrzi_rx_prev = 1
        self.tx_level = 1
        self.stretching = False
        self.stretch_release_at: Optional[int] = None
        self.last_symbol = 0
        self._decode_modes()

    # ----- host side (the host slot of the wheel; instantaneous here, D16) -----

    def start(self, pc: int | None = None) -> None:
        if pc is not None:
            self.pc = pc
        self.running = True
        self.halt_reason = ""

    def host_write(self, *words: int) -> None:
        for w in words:
            if not self.inq.push(w):
                raise OverflowError("host-in queue full")

    def host_read(self) -> Optional[int]:
        return self.outq.pop()

    def host_read_all(self) -> list[int]:
        out = []
        while (w := self.outq.pop()) is not None:
            out.append(w)
        return out

    # ----- mode registers -----

    def _decode_modes(self) -> None:
        g = isa.mode_decode
        self.tick = g("TICK", self.modes[0])
        self.period = max(MIN_COUNT, self.modes[1] & 0xFFFF)
        self.period2 = max(MIN_COUNT, self.modes[2] & 0xFFFF)
        self.raw_period = self.modes[1] & 0xFFFF
        t01, t2l = g("THR01", self.modes[3]), g("THR2L", self.modes[4])
        self.thr = (t01["thr0"],)                        # ruling 23: THR1, THR2 deleted
        self.code = g("CODE", self.modes[5])
        self.prescale = PRESCALE_FACTORS[self.tick["prescale"]]
        delay = t2l["delay"] * (self.prescale if self.code["delay_prescaled"] else 1)
        self.data_delay = 0 if self.code["delay_clock"] else delay      # D13
        self.clock_delay = delay if self.code["delay_clock"] else 0
        p = g("PULSE", self.modes[6])
        self.pulse_high = (p["high0"], p["high1"])
        s = g("STUFF", self.modes[7])
        self.stuffer.n, self.stuffer.equal_mode, self.stuffer.enabled = s["n"], bool(s["mode"]), bool(s["enable"])
        self.stuff_violation_event = bool(s["violation_event"])
        self.crc.poly = self.modes[8]
        cfg = g("CRCCFG", self.modes[9])
        self.crc.width, self.crc.init_ones, self.crc.xorout = cfg["width"], bool(cfg["init_ones"]), bool(cfg["xorout_ones"])
        self.capture_level = 0 if self.tick["capture"] else 1
        if self.code["rx"] == 2 or self.code["tx"] == 2:
            raise NotImplementedError("Manchester coding is not modelled yet (CODE rx/tx = 2)")

    def _write_mode(self, reg: int, upper: bool, value: int) -> None:
        if reg >= len(isa.MODE_REGS):
            raise Halted(f"SETM to undefined mode register {reg}")
        bits = isa.MODE_REGS[reg].bits
        old = self.modes[reg]
        new = ((old & 0x0000FFFF) | (value << 16)) if upper else ((old & 0xFFFF0000) | value)
        self.modes[reg] = new & ((1 << bits) - 1)
        if reg >= 10:
            self.detectors[reg - 10].configure(self.modes[reg])
            return
        self._decode_modes()
        if reg == 0:                                   # D9: TICK write restarts timers
            self.baud_phase = "idle"
            self.rec_count = 0
            self.stretching, self.stretch_release_at = False, None
            if self.tick["clock_out"]:
                self._clock_level(self.tick["clock_idle"], delay=0)

    # ----- outputs -----

    def _schedule(self, k: int, value: int, enable: int, delay: int) -> None:
        self.sched.append((self.cycle + delay, k, value, enable))

    def _data_level(self, level: int, delay: int | None = None) -> None:
        """Drive out0 (and out1 if differential) with logical line level."""
        d = self.data_delay if delay is None else delay
        lvl = level ^ self.code["invert"]
        if self.code["data_od"]:
            self._schedule(0, 0, 1 - lvl, d)
        else:
            self._schedule(0, lvl, 1, d)
            if self.code["diff_out"]:
                self._schedule(1, 1 - lvl, 1, d)

    def _data_release(self) -> None:
        self._schedule(0, 0, 0, self.data_delay)
        if self.code["diff_out"]:
            self._schedule(1, 0, 0, self.data_delay)

    def _clock_level(self, level: int, delay: int | None = None) -> None:
        d = self.clock_delay if delay is None else delay
        if self.tick["clock_od"]:
            self._schedule(3, 0, 1 - level, d)
        else:
            self._schedule(3, level, 1, d)

    def _apply_scheduled(self) -> None:
        if not self.sched:
            return
        due = [s for s in self.sched if s[0] <= self.cycle]
        if not due:
            return
        self.sched = [s for s in self.sched if s[0] > self.cycle]
        for _, k, v, e in sorted(due, key=lambda s: s[0]):
            self.out_val[k], self.out_en[k] = v, e

    def outputs(self) -> tuple[int, int]:
        if not self.running:
            return 0, 0
        val = sum(v << k for k, v in enumerate(self.out_val))
        en = sum(e << k for k, e in enumerate(self.out_en))
        return val, en

    # ----- the clock cycle -----

    def step(self, pins: int) -> tuple[int, int]:
        """Advance one clock cycle. `pins` are in0-in3 during this cycle;
        returns (values, enables) of out0-out3 after this cycle's edge."""
        self.pipe.append(pins & 0xF)
        self.inp = self.pipe.popleft()
        if self.running:
            fired = 0
            for k, det in enumerate(self.detectors):
                if det.step(self.inp, self.prev_inp, self.prescale):
                    fired |= 1 << k
            try:
                fired |= self._bit_hardware()
                if (self.cycle - self.lane - EXEC_STAGE) % SLOT_PERIOD == 0:
                    self._sequencer()
                if fired:
                    self._take_events(fired)
            except Halted as h:
                self._halt(str(h))
            self._apply_scheduled()
        self.prev_inp = self.inp
        self.cycle += 1
        return self.outputs()

    def _halt(self, reason: str) -> None:
        self.running = False
        self.halt_reason = reason
        self.active = self.parked = None
        self.sched.clear()
        self.out_val, self.out_en = [0, 0, 0, 0], [0, 0, 0, 0]   # isa 6.3: drives nothing

    # ----- tick source and field engine -----

    def _bit(self, k: int) -> int:
        return (self.inp >> k) & 1

    def _bit_hardware(self) -> int:
        """Ticks for this cycle, then advance the active operation.
        Returns event bits raised here (stuffing violation -> event 3)."""
        t = self.cycle
        mode = self.tick["mode"]
        din = self.code["data_in"]
        data, data_prev = self._bit(din), (self.prev_inp >> din) & 1
        data_edge = data != data_prev
        launch = capture = False
        self.raised = 0
        op = self.active
        if mode == EDGE:
            ck = self.tick["clock_in"]
            clk, clk_prev = self._bit(ck), (self.prev_inp >> ck) & 1
            if clk != clk_prev:
                if clk == self.capture_level:
                    capture = True
                else:
                    launch = True
            if (launch and self.tick["stretch_hold"] and self.tick["clock_out"]
                    and op is None and self.data_waiting):
                self.stretching = True                       # D11
                self._clock_level(0, delay=0)
            if self.stretch_release_at is not None and t >= self.stretch_release_at:
                self.stretching, self.stretch_release_at = False, None
                self._clock_level(self.tick["clock_idle"], delay=0)
        elif mode == RECOVER:
            full = max(MIN_COUNT, self.raw_period * self.prescale)
            half = full * RECOVER_CAPTURE_NUM // RECOVER_CAPTURE_DEN
            if data_edge:
                self.rec_count, launch = 0, True
            else:
                self.rec_count += 1
                if self.rec_count == half:
                    capture = True
                if self.rec_count >= full:
                    self.rec_count, launch = 0, True
        pulse_rx = mode == PULSE and (
            (isinstance(op, FieldOp) and not op.drives) or isinstance(op, RawOp))
        if mode == BAUD or (mode == PULSE and not pulse_rx):
            launch, capture = self._baud_ticks(op)
        elif pulse_rx and data_edge:
            level = data ^ self.code["invert"]
            dur = min(PULSE_SATURATE, (t - self.last_edge) // self.prescale)
            if self.code["rx"] == 3:                         # D12: one bit per high pulse
                if level == 0:
                    self.last_symbol = sum(dur >= th for th in self.thr)
                    capture = True
            else:
                capture = True
        if data_edge:
            self.last_edge = t
        # advance the active operation
        if isinstance(op, FieldOp):
            if launch and op.phase == "launch":
                self._launch(op)
            if capture and op.phase == "capture":
                self._capture(op)
        elif isinstance(op, PinOp):
            if not op.applied and launch:
                self._pin_apply(op)
            if op.applied and op.until is not None and t >= op.until:
                self._finish()
        elif isinstance(op, RawOp):
            if op.drive:
                if op.until is not None and t >= op.until:
                    self._finish()
            elif data_edge:
                self._complete_raw_sample(op, t)
        return self.raised

    def _baud_ticks(self, op: Optional[Op]) -> tuple[bool, bool]:
        t = self.cycle
        clock_out = self.tick["clock_out"]
        if self.baud_phase == "to_capture" and t >= self.baud_at:
            if self.tick["stretch_honour"] and clock_out:
                if not self.clk_released:
                    self._clock_level(self.capture_level)
                    self.clk_released = True
                if self._bit(self.tick["clock_in"]) != self.capture_level:
                    return False, False                     # D10: paused by stretching
            elif clock_out:
                self._clock_level(self.capture_level)
            self.baud_phase, self.baud_at = "to_launch", t + self.period2 * self.prescale
            return False, True
        if self.baud_phase == "to_launch" and t >= self.baud_at:
            if isinstance(op, FieldOp) and op.phase == "launch":
                self._baud_begin_bit()
                return True, False
            self.baud_phase = "idle"                         # end of a field's last bit
            if clock_out:
                lvl = 1 - self.capture_level if self.tick["rest"] else self.tick["clock_idle"]
                self._clock_level(lvl)
            if isinstance(op, FieldOp) and op.tail:          # D30: hardware free now;
                self._finish()                               # a parked op starts here
        return False, False

    def _baud_begin_bit(self) -> None:
        self.baud_phase = "to_capture"
        self.baud_at = self.cycle + self.period * self.prescale
        self.clk_released = False
        if self.tick["clock_out"]:
            self._clock_level(1 - self.capture_level)

    def _next_wire_bit(self, op: FieldOp) -> tuple[int, bool]:
        if op.stuff and self.stuffer.stuff_next:
            return self.stuffer.stuff_bit, True
        i = op.bits
        b = (op.value >> (i if op.lsb else op.width - 1 - i)) & 1 if i < op.width else 0
        return b, False

    def _launch(self, op: FieldOp) -> None:
        op.phase = "capture"
        if op.prelaunched:
            op.prelaunched = False
            return
        if op.drives:
            op.cur_bit, op.cur_stuff = self._next_wire_bit(op)
            self._drive_bit(op.cur_bit)
        elif not op.released:
            op.released = True                               # D14
            self._data_release()

    def _drive_bit(self, b: int) -> None:
        tx = self.code["tx"]
        if tx == 0:
            lvl = b
        elif tx == 1:                                        # NRZI: 0 is a transition
            lvl = self.tx_level if b else 1 - self.tx_level
        elif tx == 3:                                        # pulse width
            self._data_level(1)
            high = self.pulse_high[b] * self.prescale
            self._data_level(0, delay=self.data_delay + high)
            self.tx_level = 0
            return
        else:
            raise NotImplementedError("Manchester transmit")
        self.tx_level = lvl
        self._data_level(lvl)

    def _rx_bit(self) -> int:
        raw = self._bit(self.code["data_in"]) ^ self.code["invert"]
        rx = self.code["rx"]
        if rx == 0:
            return raw
        if rx == 1:
            b = 1 if raw == self.nrzi_rx_prev else 0
            self.nrzi_rx_prev = raw
            return b
        if rx == 3:
            return 1 if self.last_symbol >= 1 else 0
        raise NotImplementedError("Manchester receive")

    def _capture(self, op: FieldOp) -> None:
        if op.ts is None:
            op.ts = self.cycle
        sampled = self._rx_bit() if op.receives else 0
        if op.drives:
            wire, is_stuff = op.cur_bit, op.cur_stuff
        else:
            wire, is_stuff = sampled, op.stuff and self.stuffer.stuff_next
        if op.stuff:
            if self.stuffer.feed(wire) and self.stuff_violation_event:
                self.raised |= 1 << 3
        if is_stuff:
            if op.bits >= op.width:
                op.trailing_stuff_done = True
        else:
            data = sampled if op.receives else op.cur_bit
            if op.crc:
                self.crc.update(op.cur_bit if op.drives else sampled)   # D18
            if op.lsb:
                if op.seek and op.bits >= op.width:
                    op.acc = (op.acc >> 1) | (data << (op.width - 1))
                else:
                    op.acc |= data << op.bits
            else:
                op.acc = ((op.acc << 1) | data) & op.mask
            op.bits += 1
        done = op.bits >= op.width
        if done and op.seek and op.kind == EXPECT and op.acc != op.value & op.mask:
            done = False                                     # keep shifting
        if done and op.stuff and self.stuffer.stuff_next and not op.trailing_stuff_done:
            done = False                                     # D19: trailing stuff bit
        if done:
            self._complete_field(op)
        elif self.tick["mode"] == PULSE and not op.drives:
            op.phase = "capture"                             # PULSE receive: edges only
        else:
            op.phase = "launch"

    def _complete_field(self, op: FieldOp) -> None:
        result = (op.value if op.kind == DRIVE else op.acc) & op.mask
        self.sr = result
        if op.kind == EXPECT:
            self.m = result == (op.value & op.mask)
        op.complete = True
        op.phase = "done"
        if op.dst != OP_NONE:
            self.writes.append(Write(op.dst, result, op.tag, op.ts if op.ts is not None else self.cycle))
            self._flush_writes()
        if self.baud_phase == "to_launch":                   # D30
            op.tail = True
        else:
            self._finish()

    def _complete_raw_sample(self, op: RawOp, t: int) -> None:
        v = min(RAW_SATURATE, (t - op.prev_edge) // self.prescale)
        if op.dst != OP_NONE:
            self.writes.append(Write(op.dst, v, op.tag, t))
            self._flush_writes()
        self._finish()

    def _finish(self) -> None:
        """The active operation is done: start the parked one, if any."""
        self.active = None
        nxt, self.parked = self.parked, None
        if nxt is not None:
            self._start(nxt)

    def _start(self, op: Op) -> None:
        t = self.cycle
        self.active = op
        mode = self.tick["mode"]
        if isinstance(op, FieldOp):
            if op.src == OP_CRC:
                op.value = self.crc.value                    # isa 3.1: CRC before the field
            if mode == PULSE and not op.drives:
                op.phase = "capture"
                op.released = True
                self._data_release()
                return
            if mode in (BAUD, PULSE):
                if self.baud_phase == "idle":
                    self._baud_begin_bit()
                    self._launch(op)
                    return
                op.phase = "launch"
            elif mode == EDGE:
                if self.stretching:                          # D11
                    self._launch(op)
                    self.stretch_release_at = t + self.period * self.prescale
                    return
                if op.early:
                    self._launch(op)
                    return
                op.phase = "launch"
                return
            else:                                            # RECOVER (D5)
                full = max(MIN_COUNT, self.raw_period * self.prescale)
                if self.rec_count < full * RECOVER_CAPTURE_NUM // RECOVER_CAPTURE_DEN:
                    self._launch(op)
                    return
                op.phase = "launch"
            if op.early and op.phase == "launch":            # D22
                self._launch(op)
                op.phase, op.prelaunched = "launch", True
        elif isinstance(op, PinOp):
            running_ticks = mode in (EDGE, RECOVER) or self.baud_phase != "idle"
            if not op.align or not running_ticks:
                self._pin_apply(op)
            if op.applied and op.until is not None and t >= op.until:
                self._finish()
        else:
            if op.drive:
                lvl = op.level ^ self.code["invert"]
                if self.code["data_od"]:
                    self._schedule(0, 0, 1 - lvl, 0)
                else:
                    self._schedule(0, lvl, 1, 0)
                op.until = t + op.count * self.prescale
                if op.count == 0:
                    self._finish()
            else:
                op.prev_edge = self.last_edge

    def _pin_apply(self, op: PinOp) -> None:
        op.applied = True
        for k in range(4):
            if op.mask >> k & 1:
                self.sched = [s for s in self.sched if s[1] != k]
                v, e = op.values >> k & 1, op.enables >> k & 1
                self._schedule(k, v, e, 0)
                if k == 0 and e:
                    self.tx_level = v ^ self.code["invert"]
        op.until = self.cycle + op.wait * self.period * self.prescale

    def _flush_writes(self) -> None:
        while self.writes:
            w = self.writes[0]
            if not self._write_dest(w.dst, w.value, w.tag, w.ts):
                return
            self.writes.pop(0)

    def _tagged(self, value: int, tag: int) -> int:
        value &= 0xFFFF_FFFF
        return (value & 0x00FF_FFFF) | (tag << 24) if tag else value

    def _write_dest(self, dst: int, value: int, tag: int = 0, ts: int = 0) -> bool:
        """Write one result; False if the destination is full (caller waits)."""
        if dst <= 3:
            self.regs[dst] = value & 0xFFFF
        elif dst == OP_HOSTQ:
            return self.outq.push(self._tagged(value, tag))
        elif dst == OP_CAP:
            if self.outq.free() < 2:
                return False
            self.outq.push(ts & 0xFFFF_FFFF)
            self.outq.push(self._tagged(value, tag))
        elif dst == OP_MEM:
            self.mem[self.regs[3] & 0x7F] = value & 0xFFFF_FFFF
            self.regs[3] = (self.regs[3] + 1) & 0xFFFF
        elif 8 <= dst <= 11:
            return self.channels.put(dst - 8, value)
        elif dst == OP_NONE:
            pass
        else:
            raise Halted(f"invalid destination operand {dst}")
        return True

    # ----- events -----

    def _take_events(self, fired: int) -> None:
        self.pending |= fired
        for k in range(isa.N_EVENTS):
            if fired >> k & 1 and self.armed[k]:
                self.pending &= ~(1 << k)                    # D23
                self._abandon()
                self.pc = self.ev_target[k]
                return

    def _abandon(self) -> None:
        op = self.active
        if isinstance(op, FieldOp) and not op.complete:
            self.sr = op.acc & op.mask                       # isa 3.6: SR and bits kept
            self.bits_kept = op.bits
        self.active = self.parked = None
        self.wait_deadline = None
        self.data_waiting = False
        if self.baud_phase != "idle":
            self.baud_phase = "idle"
        if self.stretching:                                  # D11
            self.stretching, self.stretch_release_at = False, None
            self._clock_level(self.tick["clock_idle"], delay=0)

    # ----- sequencer -----

    def _hw_idle(self) -> bool:
        return self.active is None and self.parked is None and self.baud_phase == "idle"

    def _result_busy(self) -> Optional[FieldOp]:
        op = self.active
        return op if isinstance(op, FieldOp) and not op.complete else None

    def _hazard(self, reads: set[str] = frozenset(), writes: set[str] = frozenset()) -> bool:
        """True if the instruction touches a result of the incomplete field (D24)."""
        op = self._result_busy()
        if op is None:
            return False
        res = {"SR"}
        if op.kind == EXPECT:
            res.add("M")
        if op.crc:
            res.add("CRC")
        if op.stuff:
            res.add("STUFF")
        if op.dst <= 3:
            res.add(f"R{op.dst}")
        elif op.dst in (OP_HOSTQ, OP_CAP):
            res.add("OUTQ")
        elif op.dst == OP_MEM:
            res |= {"R3", "MEM"}
        elif 8 <= op.dst <= 11:
            res.add(f"C{op.dst - 8}")
        return bool(res & (set(reads) | set(writes)))

    def _sequencer(self) -> None:
        if self.writes:
            self._flush_writes()
            if self.writes:
                self.data_waiting = True
                return
        if self.parked is not None:
            return                                           # run-ahead stops here
        word = self.prog[self.pc]
        if word >> 28 in isa.RESERVED_OPCODES:
            raise Halted(f"reserved opcode {word >> 28} at {self.pc}")
        ins = isa.decode(word)
        self.data_waiting = False
        nxt = self._exec(ins)
        if nxt == "wait":
            return
        self.wait_deadline = None
        self.pc = (self.pc + 1 if nxt is None else nxt) % isa.PROGRAM_WORDS

    def _data_wait(self) -> str:
        self.data_waiting = True
        return "wait"

    def _read_operand(self, code: int, imm: int) -> tuple[bool, int]:
        """(available, value) for a field/RAW source read at issue time."""
        if code <= 3:
            return True, self.regs[code]
        if code == OP_HOSTQ:
            w = self.inq.pop()
            return (w is not None), (w or 0)
        if code == OP_CRC:
            return True, 0                                   # latched at start
        if code == OP_MEM:
            v = self.mem[self.regs[3] & 0x7F]
            self.regs[3] = (self.regs[3] + 1) & 0xFFFF
            return True, v
        if 8 <= code <= 11:
            v = self.channels.get(code - 8)
            return (v is not None), (v or 0)
        if code == OP_IMM:
            return True, imm
        if code == OP_SR:
            return True, self.sr
        if code == OP_NONE:
            return True, 0
        raise Halted(f"invalid source operand {code}")

    def _source_reads(self, code: int) -> set[str]:
        if code <= 3:
            return {f"R{code}"}
        return {OP_SR: {"SR"}, OP_MEM: {"MEM", "R3"}}.get(code, set()) | (
            {f"C{code - 8}"} if 8 <= code <= 11 else set())

    def _issue(self, op: Op) -> None:
        if self.active is None:
            self._start(op)
        else:
            self.parked = op

    def _exec(self, ins: isa.Instr) -> Optional[int] | str:
        f = ins.fields
        op = ins.op
        if op == "FIELD":
            if f["src"] not in isa.SOURCE_OPERANDS or f["dst"] not in isa.DEST_OPERANDS:
                raise Halted(f"FIELD with invalid operand at {self.pc}")
            if self._hazard(reads=self._source_reads(f["src"])):
                return "wait"
            ok, val = self._read_operand(f["src"], f.get("imm"))
            if not ok:
                return self._data_wait()
            self._issue(FieldOp(f["kind"], f["width"], bool(f["lsb"]), bool(f["crc"]),
                                bool(f["stuff"]), f["src"], f["dst"], bool(f["early"]),
                                bool(f.get("seek")), f.get("tag"), value=val))
            return None
        if op == "RAW":
            if f["src"] not in isa.SOURCE_OPERANDS or f["dst"] not in isa.DEST_OPERANDS:
                raise Halted(f"RAW with invalid operand at {self.pc}")
            if f["dir"] == 1:
                if self._hazard(reads=self._source_reads(f["src"])):
                    return "wait"
                ok, val = self._read_operand(f["src"], f.get("imm"))
                if not ok:
                    return self._data_wait()
                self._issue(RawOp(True, f["level"], count=val))
            else:
                self._issue(RawOp(False, 0, dst=f["dst"], tag=f.get("tag")))
            return None
        if op == "PIN":
            self._issue(PinOp(f["mask"], f["values"], f["enables"], bool(f["align"]), f["wait"]))
            return None
        if op == "SETM":
            if f["reg"] in BIT_HW_MODE_REGS and not self._hw_idle():
                return "wait"
            if f["from_reg"] and self._hazard(reads={f"R{f['rs']}"}):
                return "wait"
            value = self.regs[f["rs"]] if f["from_reg"] else f["imm"]
            self._write_mode(f["reg"], bool(f["upper"]), value)
            return None
        if op == "LDI":
            if self._hazard(writes={f"R{f['rd']}"}):
                return "wait"
            self.regs[f["rd"]] = f["imm"]
            return None
        if op == "ALU":
            return self._alu(f)
        if op == "MEM":
            rd, ra = f"R{f['rd']}", f"R{f['ra']}"
            if self._hazard(reads={rd, ra, "MEM"}, writes={rd, ra}):
                return "wait"
            addr = f["addr"] if f["absolute"] else self.regs[f["ra"]] & 0x7F
            shift = 16 if f["hi"] else 0
            if f["store"]:
                w = self.mem[addr] & ~(0xFFFF << shift)
                self.mem[addr] = w | (self.regs[f["rd"]] << shift)
            else:
                self.regs[f["rd"]] = (self.mem[addr] >> shift) & 0xFFFF
            if f["postinc"] and not f["absolute"]:
                self.regs[f["ra"]] = (self.regs[f["ra"]] + 1) & 0xFFFF
            return None
        if op == "BR":
            return self._branch(f["cond"], f["target"])
        if op == "LOOP":
            if self._hazard(writes={f"R{f['rd']}"}):
                return "wait"
            self.regs[f["rd"]] = (self.regs[f["rd"]] - 1) & 0xFFFF
            return f["target"] if self.regs[f["rd"]] != 0 else None
        if op == "WAIT":
            if self.pending & f["mask"]:
                return None
            if f["timeout"]:
                if self.wait_deadline is None:
                    self.wait_deadline = self.cycle + f["timeout"] * self.period * self.prescale
                if self.cycle >= self.wait_deadline:
                    return None
            return "wait"
        if op == "EVT":
            k = f["k"]
            if f["clear"]:
                self.pending &= ~(1 << k)
            self.armed[k] = bool(f["enable"])
            self.ev_target[k] = f["target"]
            return None
        if op == "Q":
            return self._queue(f)
        if op == "MISC":
            name = isa.name_of(isa.MISC_OPS, f["op"])
            if name is None:
                raise Halted(f"undefined MISC op {f['op']}")
            if name == "HALT":
                if not self._hw_idle():                     # D25
                    return "wait"
                raise Halted("HALT")
            if name == "IRQ":
                self.irq = True
            elif name in ("CRCINIT", "CRCCHK"):
                if self._hazard(reads={"CRC", "R0", "R1"}, writes={"CRC", "M"}):
                    return "wait"
                if name == "CRCINIT":
                    self.crc.init()
                else:
                    self.m = self.crc.state == ((self.regs[1] << 16) | self.regs[0])
            elif name == "STUFFCLR":
                if self._hazard(writes={"STUFF"}):
                    return "wait"
                self.stuffer.clear()
            return None
        raise Halted(f"unknown instruction {op}")

    def _alu(self, f: dict[str, int]) -> Optional[int] | str:
        name = isa.name_of(isa.ALU_OPS, f["op"])
        if name is None:
            raise Halted(f"undefined ALU op {f['op']}")
        rd = f["rd"]
        reads = set()
        if name not in ("MOV", "MOVSR", "MOVSRH"):
            reads.add(f"R{rd}")
        if name in ("MOVSR", "MOVSRH"):
            reads.add("SR")
        elif name not in isa.ALU_NO_SOURCE and not f["use_imm"]:
            reads.add(f"R{f['rs']}")
        if self._hazard(reads=reads, writes={f"R{rd}"}):
            return "wait"
        a = self.regs[rd]
        b = f["imm"] if f["use_imm"] else self.regs[f["rs"]]
        borrow = None
        if name == "MOV":
            r = b
        elif name == "ADD":
            r = a + b
        elif name in ("SUB", "CMP"):
            r, borrow = a - b, a < b
        elif name == "AND":
            r = a & b
        elif name == "OR":
            r = a | b
        elif name == "XOR":
            r = a ^ b
        elif name == "SHL":
            r = a << (b & 0x1F)                              # D26
        elif name == "SHR":
            r = a >> (b & 0x1F)
        elif name == "INC":
            r = a + 1
        elif name == "DEC":
            r = a - 1
        elif name == "ADDC":
            s = a + b
            r = (s & 0xFFFF) + (s >> 16)
        elif name == "MOVSR":
            r = self.sr & 0xFFFF
        else:
            r = (self.sr >> 16) & 0xFFFF
        r &= 0xFFFF
        self.z = r == 0
        self.n = borrow if borrow is not None else bool(r >> 15)
        if name != "CMP":
            self.regs[rd] = r
        return None

    def _branch(self, cond: int, target: int) -> Optional[int] | str:
        name = isa.name_of(isa.BR_CONDS, cond)
        if name is None:
            raise Halted(f"undefined branch condition {cond}")
        if name in ("M", "NM") and self._hazard(reads={"M"}):
            return "wait"
        if name in ("QF", "QNF") and self._hazard(reads={"OUTQ"}):
            return "wait"
        if name[0] == "C" and self._hazard(reads={name}):
            return "wait"
        if name in ("QF", "QNF") and self.writes:
            return "wait"
        table = {
            "always": True, "Z": self.z, "NZ": not self.z, "N": self.n, "NN": not self.n,
            "M": self.m, "NM": not self.m, "QE": self.inq.empty(), "QNE": not self.inq.empty(),
            "QF": self.outq.full(), "QNF": not self.outq.full(),
        }
        if name in table:
            taken = table[name]
        elif name[0] == "E":
            taken = bool(self.pending >> int(name[1]) & 1)
        else:
            taken = self.channels.valid[int(name[1])]
        return target if taken else None

    def _queue(self, f: dict[str, int]) -> Optional[int] | str:
        name = isa.name_of(isa.Q_OPS, f["op"])
        if name is None:
            raise Halted(f"undefined Q op {f['op']}")
        reg, ch, tag = f["reg"], f["ch"], f["tag"]
        r = f"R{reg}"
        if name == "POP":
            if self._hazard(writes={r}):
                return "wait"
            w = self.inq.pop()
            if w is None:
                return self._data_wait()
            self.regs[reg] = w & 0xFFFF                      # D27
            return None
        if name in ("PUSH", "PUSHTS", "PUSHCRC", "PUSHI"):
            reads = {"OUTQ"} | ({r} if name == "PUSH" else set()) | ({"CRC"} if name == "PUSHCRC" else set())
            if self._hazard(reads=reads) or self.writes:
                return "wait"
            value = {"PUSH": self.regs[reg], "PUSHTS": self.cycle, "PUSHCRC": self.crc.value,
                     "PUSHI": f["imm8"]}[name]
            if not self.outq.push(self._tagged(value, tag)):
                return self._data_wait()
            return None
        c = f"C{ch}"
        if name in ("PUT", "PUTSR"):
            reads = {c} | ({r} if name == "PUT" else {"SR"})
            if self._hazard(reads=reads):
                return "wait"
            if not self.channels.put(ch, self.regs[reg] if name == "PUT" else self.sr):
                return self._data_wait()
            return None
        # GET
        if self._hazard(reads={c}, writes={r}):
            return "wait"
        v = self.channels.get(ch)
        if v is None:
            return self._data_wait()
        self.regs[reg] = v & 0xFFFF
        return None


# --------------------------------------------------------------------------
# Running a lane against a test bench
# --------------------------------------------------------------------------

@dataclass
class Trace:
    """Per-cycle pins: inputs as supplied, outputs as returned by step()."""
    inputs: list[int] = field(default_factory=list)
    out_val: list[int] = field(default_factory=list)
    out_en: list[int] = field(default_factory=list)

    def out(self, k: int) -> list[str]:
        """out_k per cycle as '0', '1' or 'z' (released)."""
        return [("01"[v >> k & 1] if e >> k & 1 else "z")
                for v, e in zip(self.out_val, self.out_en)]

    def inp(self, k: int) -> list[int]:
        return [v >> k & 1 for v in self.inputs]

    def __len__(self) -> int:
        return len(self.inputs)


Bench = Callable[[int, int, int], int]   # (cycle, prev out values, prev enables) -> pins


def run(lane: Lane, cycles: int, bench: Bench | int = 0b1111,
        trace: Trace | None = None, until: Callable[[Lane], bool] | None = None) -> Trace:
    """Step `lane` for `cycles` cycles. `bench` gives the input pins each
    cycle from the outputs of the previous cycle (one cycle of wire delay),
    or is a constant."""
    tr = trace if trace is not None else Trace()
    val, en = (tr.out_val[-1], tr.out_en[-1]) if tr.out_val else lane.outputs()
    for _ in range(cycles):
        pins = bench if isinstance(bench, int) else bench(lane.cycle, val, en)
        val, en = lane.step(pins)
        tr.inputs.append(pins)
        tr.out_val.append(val)
        tr.out_en.append(en)
        if until is not None and until(lane):
            break
    return tr
