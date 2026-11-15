"""Compiler: protocol description -> one lane program per role.

    from rosetta.compiler import compile_file
    out = compile_file("programs/uart/uart.ros", "receiver", {"parity": "even"})
    out.asm          # assembly text accepted by rosetta.asm
    out.words        # assembled words (at out.origin)
    out.mode_regs    # mode-register values the program's prologue writes
    out.manifest     # host view (language.md 3.15): queues, tags, pins, timing

Stages (each is a function below; the intermediate forms are dataclasses):

1. parse          rosetta.lang                      -> syntax tree
2. elaborate      params, wires, clock, frame        -> ProtocolM
                  (compile-time `if`s resolved, names, units, widths checked)
3. project        one role's view of every field     -> RoleIR
                  (drive / sample / expect, pins, host words, record tags)
4. timing         tick mode, prescale, PERIOD(2)     -> TimingM (in RoleIR)
                  (rate rounding against `tolerance`, receiver sampling window)
5. generate       RoleIR -> instruction lines with schedule metadata
6. schedule       gap-free / frame-start checks on the generated lines
7. assemble       rosetta.asm -> words; manifest

Only what the UART description needs is implemented. Every other construct
raises `Unsupported` naming it. Choices that the language, the ISA or the
rulings leave open are collected in INTERPRETATIONS (each one is applied in
exactly one place, marked `# I<n>`).
"""
from __future__ import annotations

import math
import os
import re
from dataclasses import dataclass, field
from fractions import Fraction
from typing import Optional, Union

from . import asm, isa, lang

CLOCK_HZ = 60_000_000
SLOT = 5            # cycles between two slot actions of one lane (sequencer.md 5.10)
PRESCALES = (1, 8, 64, 512)
STDLIB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stdlib.ros")
MAX_TAG_INDEX = 63

# Record tags (language.md 3.15 leaves the allocation to the compiler): one
# index per distinct field name, in description order, from 1; the kind sits
# in bits [7:6].
TAG_VALUE, TAG_ERROR = 0x00, 0x40

INTERPRETATIONS = {
    1: "3.2 'A file holds one protocol, and may also hold top-level crc "
       "statements (this is how the library is written)': the standard library "
       "(stdlib.ros, the text of 3.12) is always in scope; a crc defined in the "
       "description with the same name replaces the library's.",
    2: "3.2 'The loader passes overrides (baud=9600)': an override without a "
       "unit takes the base unit of the parameter's dimension (Hz, s).",
    3: "3.5 'clock rate R [tolerance T]' with 'The compiler rejects a rate whose "
       "PERIOD rounding error exceeds T': without a tolerance there is nothing "
       "to check against, so the compiler rejects it.",
    4: "2.1 table 'BAUD, half-bit PERIOD 260 (0.16 % fast, inside the 2 %)': "
       "the sender uses PERIOD = PERIOD2 = the half bit rounded (halves up); "
       "only if that misses the tolerance does it round the whole bit and "
       "split it unequally (115200: 260 + 261 would be 0.03 % slow).",
    5: "2.1 table 'RECOVER, PERIOD 521' and 3.5 'receivers decode correctly at "
       "any rate inside the band': the receiver's PERIOD is the bit rounded; "
       "its own rounding must also be within T, and RECOVER must sample every "
       "bit of a frame inside the bit for every far-end bit time within T.",
    6: "3.15 'A program waits for any word it needs, and the wires hold their "
       "state while it does: a UART line stays idle': a clock-rate sender "
       "cannot pause inside a frame, so it pops every host word of the frame "
       "into registers before the frame's first field.",
    7: "3.12 'The register is initialised just before the first covered field "
       "of each frame': CRCINIT is issued at the top of the frame. No field "
       "before the first covered one updates the check unit, so the state is "
       "the same; CRCINIT waits for idle bit hardware, so issuing it between "
       "two fields would break a sender's gap-free bits.",
    8: "3.8 'else end (the default) abandons the rest of the frame' and 3.15 "
       "'Any other role records the error and waits for the frame to end': in "
       "a role that owns no end event, in a frame without `until`, the role "
       "records the error and waits for the next frame start.",
    9: "3.15 'each mismatch (error FIELD=seen)' and 'A listener record adds the "
       "timestamp word that CAP already pushes': an error record is {error "
       "tag, seen value}; a 1-bit constant field pushes the complement of its "
       "expected value (PUSHI), any other field its sampled bits (MOVSR, "
       "PUSH). The listener pushes the timestamp word first with PUSHTS, so "
       "an error's timestamp is that instruction's cycle (the manifest gives "
       "how many cycles after the bit's capture it can be).",
    10: "3.15 'The compiler writes a manifest that maps tags to names, kinds "
        "and named values': index = 1 + position of the field name among the "
        "distinct field names of the frame in description order (all branches, "
        "so tags do not depend on parameters); value record tag = index, error "
        "record tag = 0x40 | index.",
    11: "6.1 'idle: PIN at program start and after each frame': the PIN after "
        "a frame is left out when the frame's last bit is a constant equal to "
        "the idle level (UART's stop bit), so frames stay back to back.",
    12: "lane.md 5.1 'Registers keep their values while halted': every program "
        "writes the timing registers it uses (TICK; PERIOD and PERIOD2 in BAUD, "
        "PERIOD in RECOVER) and CODE, and a driving role THR2L (launch delay 0), "
        "rather than relying on reset values.",
    13: "isa.md 4 'CRCPOLY 32 bits' (asm: 'write the low half, then "
        "CRCPOLY.hi'): the high half is written only for a width or polynomial "
        "beyond 16 bits; the check unit's bits below the width do not depend "
        "on higher polynomial bits (rosetta_crc.sv).",
    14: "3.5 'A frame with no after begins at the first edge away from the "
        "idle level': for an observer, the first such edge after the previous "
        "frame's last field has been checked (the start event's pending bit is "
        "cleared there, so edges inside a frame or a failed stop bit do not "
        "start the next one).",
    15: "3.5 form B 'For the other roles, R is the fastest clock they must keep "
        "up with': the clock owner never runs faster than R: PERIOD = PERIOD2 = "
        "the half period rounded up; the other roles are checked at R exactly.",
    16: "3.14 'The compiler chooses PERIOD, the launch delay (LAUNCH) and PIN hold "
        "counts so that the role's own rules hold' ... 'minimums round up', and "
        "'an assumption otherwise': `bit` is 1/R. The responsible role meets a "
        "rule with PIN waits from its actual PERIOD (frame start to the first "
        "clock change: a wait on the start PIN; last clock change to frame end: "
        "an empty PIN wait; frame end to frame start: a wait on the end PIN); a "
        "setup rule is checked against launch-to-capture. Other roles count on "
        "those bounds as minimum times at R, less EDGE_MARGIN. Only these rule "
        "shapes are implemented.",
    17: "3.8 'If the owner's host queue is empty when the field must start, the "
        "owner drives the default': a target cannot wait for the clock edge, so "
        "it decides when it prepares the word: the first word of a frame while "
        "idle (the start event vectors to a FIELD from the queue or from the "
        "default), later words while the previous one is clocked (a parked word "
        "is popped when parked, up to two ahead). Words prepared but not "
        "clocked when the frame ends are discarded with the abandoned operation "
        "(lane.md 7.4 'a word its source popped is lost'), so the host queues "
        "one frame's words and waits for its end record.",
    18: "3.5 'With a leading capture edge, the first bit of a frame is launched "
        "when the frame begins': `early` on the first word of each frame only; "
        "later words launch at the launch edge between words. The clock owner "
        "gets no `early` (ruling 6: it has no effect in BAUD).",
    19: "3.9 'The listener has to sample both, which a lane cannot do today (gap "
        "G5)': `listen_wire` picks the data wire of a one-lane listener; every "
        "such lane records the frame events too, so each stream decodes on its "
        "own (the host merges them by timestamp and drops the repeats).",
    20: "I10 extended to events (3.15 'each event it does not own'): event names "
        "get record indices after the field names; an event record is {index, "
        "0}. The listener's event timestamp is a PUSHTS in the event's handler; "
        "the manifest gives how late it can be after the pin edge.",
    21: "3.10 'The owner of the until events gets an iteration count from its "
        "host before the first iteration': a count of 0 gives a frame with only "
        "the start and end events.",
    22: "3.15 '`cut FIELD` when a foreign event ends the frame partway through a "
        "field': not produced. No instruction reads how many bits of an "
        "abandoned word were clocked, so the lane cannot tell a cut word from "
        "one that never started.",
    23: "2.2 table 'deselect abandons the current word and releases miso': the "
        "frame-end handler of a role that drives a data wire sets that wire to "
        "its idle level (released for miso).",
    24: "3.7 'Frames follow each other forever': the end event stays armed "
        "between frames too, so a stray edge gives an end record. A clock owner "
        "started after the others leaves cs_n low until it runs (a halted lane "
        "drives nothing), and its idle level is then an edge: the loader starts "
        "the clock owner first.",
    25: "3.14 'for setup and hold: W's driver at that moment ... for A to B: the "
        "owner of B': rules between clock edges and events (A to B), setup to the "
        "capture edge, hold from the launch edge and pulse widths of the clock "
        "are implemented; on an open-drain wire each field owner answers for its "
        "own setup and hold.",
    26: "3.14 'The compiler chooses PERIOD ... so that the role's own rules hold' "
        "with `low scl` / `high scl`: PERIOD = the low phase = max(t_low, half a "
        "bit); PERIOD2 = the rest of the bit less the 4 cycles the honoured capture "
        "waits for scl to read high (tick.md 4.1, model D10), so the high phase on "
        "the pin is PERIOD2 + 4; never faster than R. If t_low + t_high do not fit "
        "1/R the program is rejected with the fastest rate.",
    27: "G4 'an event fires only if its qualifying level still holds N cycles "
        "after the edge. UM10204's note on 300 ns of internal hold suggests N ~ "
        "18': every `edge while level` event gets K = 300 ns rounded up (18 "
        "cycles at 60 MHz); the language has no syntax for it.",
    28: "isa.md 2 'After a field's last bit it either returns to its idle level or "
        "stays at the launch level (TICK.rest), which is what I2C needs': `rest` is "
        "set when a frame event is conditioned on the clock wire, so the clock "
        "never rises inside a frame except for a bit.",
    29: "3.6 'release SDA; release SCL and wait until it reads high; wait "
        "t_su_sta; pull SDA low; wait t_hd_sta': from inside a frame the PIN that "
        "sets SDA also waits the clock's low pulse rule (t_low), so a target lets "
        "go of SDA first; 'wait until it reads high' is an event on the clock "
        "level plus WAIT (ruling 25, lane.md I28); PIN waits are whole PERIODs "
        "(isa 3.3), rounded up. From idle only the edge and its following wait. The "
        "frame's first host word is popped before the start event, so the bus is "
        "not taken before the host is ready.",
    30: "3.15 'an SPI or I2C controller pauses its clock between bits': after a "
        "checked field (an expected ack) the clock owner's branch waits for the "
        "end of the bit (BR M/NM is a sync instruction, sequencer.md 5.11), so the "
        "clock rests at its launch level for a few slots before the next field. "
        "That only lengthens t_low, so the schedule check allows it.",
    31: "3.15 'If the role owns the frame's end events, it generates the first "
        "event in the until list (stop, for I2C), records the error, raises the "
        "host interrupt and halts': in exactly that order.",
    32: "3.8 'If it is false, the owner does not drive the field (it releases the "
        "wire) and takes the else action ... the owner records nothing': the "
        "target branches around its DRIVE and, for else end, waits for the frame "
        "end (no `FIELD SAMPLE` to NONE as 6.1 sketches: not driving is enough).",
    33: "C2 'A field that steers a branch and must also reach the host ... is "
        "sampled into a register and pushed afterwards': a party samples such a "
        "field into a register and pushes its record after the next field is "
        "parked (so the next field is not late); the listener samples it to CAP, "
        "keeping its exact timestamp, and copies it with MOVSR.",
    34: "3.7 'An event may both end one frame and begin the next: I2C start': a "
        "role that detects it has one handler for it, which records it and starts "
        "the frame (a repeated START is recorded once).",
}


# ============================================================== errors

class CompileError(Exception):
    """A description the compiler rejects. `line` is the description line."""

    def __init__(self, message: str, line: int = 0, filename: str = "") -> None:
        where = f"{filename}:{line}: " if line and filename else (f"line {line}: " if line else "")
        super().__init__(where + message)
        self.message, self.line, self.filename = message, line, filename


class Unsupported(CompileError):
    """A valid construct that this compiler milestone does not implement."""

    def __init__(self, construct: str, line: int = 0, filename: str = "", why: str = "") -> None:
        msg = f"`{construct}` is not supported by the compiler yet" + (f" ({why})" if why else "")
        super().__init__(msg, line, filename)
        self.construct = construct


# ============================================================== values

@dataclass(frozen=True)
class Q:
    """A quantity: exact value and dimension (None, time, freq, bit, frac)."""
    v: Fraction
    dim: Optional[str] = None

    def is_int(self) -> bool:
        return self.dim is None and self.v.denominator == 1


@dataclass(frozen=True)
class EnumVal:
    param: str
    member: str


@dataclass(frozen=True)
class Sym:
    word: str                       # leading trailing rising falling released


UNITS = {"s": ("time", Fraction(1)), "ms": ("time", Fraction(1, 10**3)),
         "us": ("time", Fraction(1, 10**6)), "ns": ("time", Fraction(1, 10**9)),
         "Hz": ("freq", Fraction(1)), "kHz": ("freq", Fraction(10**3)),
         "MHz": ("freq", Fraction(10**6)), "bit": ("bit", Fraction(1)),
         "%": ("frac", Fraction(1, 100))}
BASE_UNIT = {"freq": "Hz", "time": "s", "bit": "bit", "frac": "%"}


def _frac(x: Union[int, float]) -> Fraction:
    return Fraction(x) if isinstance(x, int) else Fraction(repr(x))


def fmt_value(v: object) -> object:
    """JSON-friendly form of an elaborated value."""
    if isinstance(v, EnumVal):
        return v.member
    if isinstance(v, Q):
        n = int(v.v) if v.v.denominator == 1 else float(v.v)
        return n if v.dim is None else {"value": n, "unit": BASE_UNIT[v.dim]}
    if isinstance(v, Sym):
        return v.word
    return v


def expr_text(e: object) -> str:
    """Description-syntax text of an expression (for messages and manifests)."""
    if isinstance(e, lang.Number):
        n = e.value if isinstance(e.value, int) else repr(e.value)
        return f"{n}{'' if e.unit in (None, '%') else ' '}{e.unit or ''}"
    if isinstance(e, lang.Name):
        return e.name
    if isinstance(e, lang.ValueWord):
        return e.word
    if isinstance(e, lang.Call):
        return f"{e.func}({', '.join(expr_text(a) for a in e.args)})"
    if isinstance(e, lang.Range):
        return f"{expr_text(e.lo)} .. {expr_text(e.hi)}"
    if isinstance(e, lang.Unary):
        return f"{e.op}{' ' if e.op == 'not' else ''}{expr_text(e.operand)}"
    if isinstance(e, lang.Binary):
        return f"{expr_text(e.left)} {e.op} {expr_text(e.right)}"
    if isinstance(e, lang.Conditional):
        return f"{expr_text(e.cond)} ? {expr_text(e.then)} : {expr_text(e.orelse)}"
    return str(e)


# ============================================================== stage 2: elaboration

@dataclass
class WireM:
    name: str
    by: Optional[str]
    idle: Union[int, str]           # 0, 1 or "released"
    line: int
    open_drain: bool = False        # idle is then 1: released, pulled up (3.4)


@dataclass
class RateClockM:
    rate_hz: Fraction
    tolerance: Fraction
    line: int


@dataclass
class CrcM:
    name: str
    width: int
    poly: int
    init: int
    xorout: int
    reflect: bool
    line: int


@dataclass
class CheckM:
    """A check value used as an expected value: crc(covered fields)."""
    crc: CrcM
    covers: tuple[str, ...]
    text: str


@dataclass
class FieldM:
    owner: str
    name: str
    width: int
    order: Optional[str]            # "msb", "lsb" or None (width 1)
    wire: str
    value: Union[None, int, CheckM]  # None: data field
    on_error: str                   # "end" or "continue"
    index: int                      # record index (I10)
    line: int
    value_text: str = ""
    covered: bool = False           # updates the check unit
    default: Optional[int] = None   # `default EXPR` (data fields)
    names: dict = field(default_factory=dict)   # named values (3.8)
    when: Optional["CondM"] = None  # `when COND` (expected fields)

    @property
    def data(self) -> bool:
        return self.value is None

    def first_wire_bit(self) -> Optional[int]:
        if not isinstance(self.value, int):
            return None
        return (self.value >> (self.width - 1)) & 1 if self.order == "msb" else self.value & 1

    def last_wire_bit(self) -> Optional[int]:
        if not isinstance(self.value, int):
            return None
        return self.value & 1 if self.order == "msb" else (self.value >> (self.width - 1)) & 1


@dataclass
class WireClockM:
    """`clock W by P rate R capture E [stretch by Q]` (language.md 3.5 form B)."""
    wire: str
    by: str
    rate_hz: Fraction
    capture_level: int              # level the capture edge goes to (1: rising)
    capture_text: str
    line: int
    stretch_by: Optional[str] = None
    rate_param: Optional[str] = None    # the parameter R is, if it is one (fastest-rate search)


@dataclass
class EventM:
    """`event NAME = W rises|falls|changes by P` (3.6), plain edges only."""
    name: str
    wire: str
    edge: str
    by: str
    index: int                      # record index (I10)
    line: int
    while_wire: Optional[str] = None
    while_level: Optional[int] = None


@dataclass
class TogetherM:
    fields: tuple[FieldM, FieldM]
    line: int


@dataclass
class RepeatM:
    count: Optional[int]            # None: open repeat
    body: list
    line: int
    while_: Optional["CondM"] = None    # `repeat { } while COND`


@dataclass
class CondM:
    """A run-time condition `FIELD == value` or `FIELD != value` (3.10, 3.11).
    `field` is the occurrence it reads: the most recent one on the path."""
    field: FieldM
    op: str
    value: int
    text: str
    line: int


@dataclass
class IfM:
    cond: CondM
    then: list
    orelse: list
    line: int


@dataclass
class RuleM:
    """A timing rule the compiler implements: `lead` (frame-start event to
    the clock's first change), `lag` (last clock change to the frame-end
    event), `idle` (frame-end event to the next frame start), `setup` (wire
    setup to capture)."""
    name: str
    kind: str
    wire: Optional[str]
    limit: Q                        # a time or a number of bits
    responsible: str
    text: str
    line: int
    a: Optional[tuple] = None       # interval rules: ("event", name) or ("clock", edge)
    b: Optional[tuple] = None


@dataclass
class FrameM:
    name: str
    fields: list[FieldM]            # every field, in description order
    implicit_start: bool
    line: int
    after: Optional[str] = None
    until: tuple[str, ...] = ()
    body: list = field(default_factory=list)    # FieldM, TogetherM, RepeatM


@dataclass
class ProtocolM:
    name: str
    filename: str
    params: dict[str, object]
    parties: tuple[str, ...]
    wires: dict[str, WireM]
    clock: Union[RateClockM, WireClockM]
    crcs: dict[str, CrcM]
    frame: FrameM
    names: dict[str, int]           # field or event name -> record index
    events: dict[str, EventM] = field(default_factory=dict)
    rules: list[RuleM] = field(default_factory=list)


class _Elab:
    """Elaboration of one protocol (stage 2)."""

    def __init__(self, f: lang.File, overrides: dict[str, str], stdlib: Optional[lang.File]):
        if f.protocol is None:
            raise CompileError("the file holds no protocol", 1, f.filename)
        self.file, self.p, self.fn = f, f.protocol, f.filename
        self.params: dict[str, object] = {}
        self.ptypes: dict[str, object] = {}
        self.crcs: dict[str, CrcM] = {}
        for c in (stdlib.crcs if stdlib else ()):           # I1
            self.crcs[c.name] = self._crc(c)
        for c in f.crcs + self.p.items_of(lang.CrcDef):
            self.crcs[c.name] = self._crc(c)
        self.overrides = dict(overrides)

    def err(self, msg: str, line: int = 0) -> CompileError:
        return CompileError(msg, line, self.fn)

    def unsupported(self, construct: str, line: int, why: str = "") -> Unsupported:
        return Unsupported(construct, line, self.fn, why)

    # ---- expressions
    def ev(self, e: object, what: str = "a constant") -> object:
        """Compile-time value of an expression over parameters."""
        if isinstance(e, lang.Number):
            v = _frac(e.value)
            if e.unit is None:
                return Q(v)
            dim, scale = UNITS[e.unit]
            return Q(v * scale, dim)
        if isinstance(e, lang.ValueWord):
            return Q(Fraction(1)) if e.word == "high" else Q(Fraction(0)) if e.word == "low" else Sym(e.word)
        if isinstance(e, lang.Name):
            if e.name in self.params:
                return self.params[e.name]
            for pname, t in self.ptypes.items():
                if isinstance(t, lang.EnumType) and e.name in t.members:
                    return EnumVal(pname, e.name)
            if e.name in self.field_names:
                raise self.unsupported(f"field `{e.name}` in an expression", e.line,
                                       f"{what} must be known at compile time here; run-time "
                                       "decisions on fields are not implemented yet")
            raise self.err(f"unknown name `{e.name}`", e.line)
        if isinstance(e, lang.Call):
            raise self.err(f"`{e.func}(...)` can only be the expected value of a field", e.line)
        if isinstance(e, lang.Unary):
            x = self.ev(e.operand, what)
            if e.op == "not":
                return not self._bool(x, e.line)
            if e.op == "-" and isinstance(x, Q):
                return Q(-x.v, x.dim)
            raise self.unsupported(f"{e.op} (bitwise complement)", e.line)
        if isinstance(e, lang.Conditional):
            return self.ev(e.then if self._bool(self.ev(e.cond), e.line) else e.orelse, what)
        if isinstance(e, lang.Binary):
            if e.op in ("and", "or"):
                a = self._bool(self.ev(e.left, what), e.line)
                b = self._bool(self.ev(e.right, what), e.line)
                return (a and b) if e.op == "and" else (a or b)
            if e.op in ("==", "!="):        # an enum member is read against the other side's type
                a, b = self._enum_side(e.left, e.right, what), self._enum_side(e.right, e.left, what)
            else:
                a, b = self.ev(e.left, what), self.ev(e.right, what)
            if e.op in ("==", "!="):
                if isinstance(a, EnumVal) or isinstance(b, EnumVal):
                    if not (isinstance(a, EnumVal) and isinstance(b, EnumVal) and a.param == b.param):
                        raise self.err(f"`{expr_text(e)}` compares values of different types", e.line)
                    return (a == b) == (e.op == "==")
            if not (isinstance(a, Q) and isinstance(b, Q)):
                raise self.err(f"`{expr_text(e)}`: operands are not numbers", e.line)
            return self._arith(e.op, a, b, e.line)
        raise self.err(f"cannot evaluate `{expr_text(e)}`", getattr(e, "line", 0))

    def _enum_side(self, e: object, other: object, what: str) -> object:
        if isinstance(e, lang.Name) and e.name not in self.params:
            o = self.ev(other, what) if not isinstance(other, lang.Name) or other.name in self.params \
                else None
            if isinstance(o, EnumVal) and e.name in self.ptypes[o.param].members:
                return EnumVal(o.param, e.name)
        return self.ev(e, what)

    def _bool(self, x: object, line: int) -> bool:
        if isinstance(x, bool):
            return x
        raise self.err("a condition must be a comparison", line)

    def _arith(self, op: str, a: Q, b: Q, line: int) -> object:
        if op in ("==", "!=", "<", "<=", ">", ">="):
            if a.dim != b.dim:
                raise self.err(f"comparing a {a.dim or 'number'} with a {b.dim or 'number'}", line)
            return {"==": a.v == b.v, "!=": a.v != b.v, "<": a.v < b.v, "<=": a.v <= b.v,
                    ">": a.v > b.v, ">=": a.v >= b.v}[op]
        if op in ("+", "-"):
            if a.dim != b.dim:
                raise self.err(f"adding a {a.dim or 'number'} to a {b.dim or 'number'}", line)
            return Q(a.v + b.v if op == "+" else a.v - b.v, a.dim)
        if op == "*":
            if a.dim and b.dim:
                if {a.dim, b.dim} == {"time", "freq"}:
                    return Q(a.v * b.v)
                raise self.err(f"multiplying a {a.dim} by a {b.dim}", line)
            return Q(a.v * b.v, a.dim or b.dim)
        if op == "/":
            if b.v == 0:
                raise self.err("division by zero", line)
            if b.dim is None:
                return Q(a.v / b.v, a.dim)
            if a.dim == b.dim:
                return Q(a.v / b.v)
            if a.dim is None and b.dim in ("time", "freq"):
                return Q(a.v / b.v, "freq" if b.dim == "time" else "time")
            raise self.err(f"dividing a {a.dim or 'number'} by a {b.dim}", line)
        raise self.err(f"unknown operator {op}", line)

    def _int(self, e: object, what: str) -> int:
        x = self.ev(e, what)
        if not (isinstance(x, Q) and x.is_int()):
            raise self.err(f"{what} must be a whole number, found `{expr_text(e)}`", getattr(e, "line", 0))
        return int(x.v)

    def _crc(self, c: lang.CrcDef) -> CrcM:
        if not 1 <= c.width <= 32:
            raise self.err(f"crc {c.name}: width {c.width} outside 1..32", c.line)
        ones = (1 << c.width) - 1
        for what, v in (("poly", c.poly), ("init", c.init), ("xorout", c.xorout)):
            if v > ones:
                raise self.err(f"crc {c.name}: {what} 0x{v:X} does not fit width {c.width}", c.line)
        return CrcM(c.name, c.width, c.poly, c.init, c.xorout, c.reflect, c.line)

    # ---- the protocol
    def run(self) -> ProtocolM:
        p = self.p
        self.field_names = {n.name for n in lang.walk(p) if isinstance(n, lang.Field)}
        for item in p.items_of(lang.Param):
            self._param(item)
        for name in self.overrides:
            if name not in self.params:
                raise self.err(f"unknown parameter `{name}` (parameters: "
                               f"{', '.join(self.params) or 'none'})")
        for name, text in self.overrides.items():
            self.params[name] = self._override(name, text)
        parties = p.items_of(lang.Parties)
        if len(parties) != 1:
            raise self.err("a protocol needs exactly one `parties` statement", p.line)
        names = parties[0].names
        if len(set(names)) != len(names) or "listener" in names:
            raise self.err("party names must be distinct and not `listener` (every protocol "
                           "has a listener role already)", parties[0].line)
        self.parties = names
        self.wires: dict[str, WireM] = {}
        clocks: list = []
        events: list[lang.Event] = []
        rules: list[lang.Rule] = []
        for item in p.items:                     # in source order, so the first unsupported
            if isinstance(item, lang.Wire):      # construct is the one reported
                self._wire(item)
            elif isinstance(item, lang.RateClock):
                clocks.append(item)
            elif isinstance(item, lang.WireClock):
                clocks.append(item)
            elif isinstance(item, lang.Event):
                self._event_shape(item)
                events.append(item)
            elif isinstance(item, lang.Stuff):
                raise self.unsupported("stuff", item.line)
            elif isinstance(item, lang.Rule):
                self._rule_shape(item)
                rules.append(item)
            elif isinstance(item, lang.SymbolClock):
                raise self.unsupported("clock symbols", item.line)
        if len(clocks) != 1:
            raise self.err("a protocol needs exactly one `clock` statement", p.line)
        c = clocks[0]
        self.clock = self._clock(c) if isinstance(c, lang.RateClock) else self._wire_clock(c)
        frames = p.items_of(lang.Frame)
        if len(frames) != 1:
            raise self.err("the core language has exactly one frame per protocol", p.line)
        self.name_index: dict[str, int] = {}
        for n in lang.walk(frames[0].body):                     # I10
            if isinstance(n, lang.Field) and n.name not in self.name_index:
                self.name_index[n.name] = len(self.name_index) + 1
        for e in events:                                        # (I20)
            if e.name in self.name_index:
                raise self.err(f"`{e.name}` names both a field and an event", e.line)
            self.name_index[e.name] = len(self.name_index) + 1
        if len(self.name_index) > MAX_TAG_INDEX:
            raise self.err(f"more than {MAX_TAG_INDEX} field and event names: record tags run out",
                           frames[0].line)
        self.events: dict[str, EventM] = {}
        for e in events:
            self._event(e)
        self.frame = self._frame(frames[0])
        rule_ms = [self._rule(r) for r in rules]
        return ProtocolM(p.name, self.fn, self.params, self.parties, self.wires, self.clock,
                         self.crcs, self.frame, self.name_index, self.events, rule_ms)

    # ---- clock wire, events, rules (language.md 3.5 form B, 3.6, 3.14)
    def _wire_clock(self, c: lang.WireClock) -> WireClockM:
        if c.wire not in self.wires:
            raise self.err(f"clock: unknown wire `{c.wire}`", c.line)
        if c.by not in self.parties:
            raise self.err(f"clock: `{c.by}` is not a party", c.line)
        w = self.wires[c.wire]
        if w.by not in (None, c.by):
            raise self.err(f"clock wire `{c.wire}` is driven by {w.by}, not {c.by}", c.line)
        if w.idle == "released":
            raise self.err(f"clock wire `{c.wire}` needs an idle level of 0 or 1", c.line)
        if c.stretch_by is not None:
            if c.stretch_by not in self.parties:
                raise self.err(f"clock: `{c.stretch_by}` is not a party", c.line)
            if not w.open_drain:
                raise self.err(f"`stretch by` needs an open-drain clock wire (3.5)", c.line)
        rate = self.ev(c.rate, "a rate")
        if not (isinstance(rate, Q) and rate.dim == "freq" and rate.v > 0):
            raise self.err(f"clock rate must be a frequency, found `{expr_text(c.rate)}`", c.line)
        cap = self.ev(c.capture, "a capture edge")
        if not (isinstance(cap, Sym) and cap.word in ("leading", "trailing", "rising", "falling")):
            raise self.err("capture must be leading, trailing, rising or falling", c.line)
        idle = int(w.idle)
        level = {"leading": 1 - idle, "trailing": idle, "rising": 1, "falling": 0}[cap.word]
        param = c.rate.name if isinstance(c.rate, lang.Name) and c.rate.name in self.params else None
        return WireClockM(c.wire, c.by, rate.v, level, cap.word, c.line, c.stretch_by, param)

    def _event_shape(self, e: lang.Event) -> None:
        cond = e.condition
        if isinstance(cond, lang.LevelCondition):
            raise self.unsupported("event = W high|low for D", e.line)
        if isinstance(cond, lang.BothLowCondition):
            raise self.unsupported("event = W, V both low for D", e.line)

    def _event(self, e: lang.Event) -> None:
        cond = e.condition
        assert isinstance(cond, lang.EdgeCondition)
        if e.name in self.events:
            raise self.err(f"event `{e.name}` defined twice", e.line)
        if cond.wire not in self.wires:
            raise self.err(f"event `{e.name}`: unknown wire `{cond.wire}`", e.line)
        if e.by not in self.parties:
            raise self.err(f"event `{e.name}`: `{e.by}` is not a party", e.line)
        if self.wires[cond.wire].by not in (None, e.by):
            raise self.err(f"event `{e.name}`: {e.by} cannot make an edge on `{cond.wire}` "
                           f"(it is driven by {self.wires[cond.wire].by})", e.line)
        wl = None
        if cond.while_wire is not None:
            if cond.while_wire not in self.wires or cond.while_wire == cond.wire:
                raise self.err(f"event `{e.name}`: bad `while` wire `{cond.while_wire}`", e.line)
            wl = 1 if cond.while_level == "high" else 0
        self.events[e.name] = EventM(e.name, cond.wire, cond.edge, e.by,
                                     self.name_index[e.name], e.line, cond.while_wire, wl)

    def _rule_shape(self, r: lang.Rule) -> None:
        if r.op != ">=":
            raise self.unsupported(f"rule ... {r.op} (only `>=` bounds)", r.line)
        b = r.body
        if isinstance(b, lang.Interval):
            for ref in (b.start, b.end):
                if isinstance(ref, lang.ClockEdge):
                    raise self.unsupported("rule A to B with A or B `capture` or `launch`", r.line)
        elif isinstance(b, (lang.Setup, lang.Hold)):
            if isinstance(b.edge, lang.EventRef):
                raise self.unsupported("rule W setup to / hold from an event", r.line)

    def _rule_ref(self, ref: object, line: int) -> tuple:
        if isinstance(ref, lang.EventRef):
            if ref.name not in self.events:
                raise self.err(f"unknown event `{ref.name}`", line)
            return ("event", ref.name)
        assert isinstance(ref, lang.WireEdge)
        if not isinstance(self.clock, WireClockM) or ref.wire != self.clock.wire:
            raise self.unsupported("rule edge `W rises|falls|changes` on a wire other than the "
                                   "clock", line)
        return ("clock", ref.edge)

    def _clock_edge(self, ref: object, line: int) -> str:
        """`capture` or `launch` for a clock edge reference."""
        clock = self.clock
        if isinstance(ref, lang.ClockEdge):
            return ref.kind
        if isinstance(ref, lang.WireEdge) and ref.wire == clock.wire and ref.edge != "changes":
            to = 1 if ref.edge == "rises" else 0
            return "capture" if to == clock.capture_level else "launch"
        raise self.unsupported("rule setup/hold against an edge that is not the clock's capture "
                               "or launch edge", line)

    def _rule(self, r: lang.Rule) -> RuleM:
        if not isinstance(self.clock, WireClockM):
            raise self.unsupported("rule (clock-rate form)", r.line)
        lim = self.ev(r.limit, "a rule bound")
        if not (isinstance(lim, Q) and lim.dim in ("time", "bit") and lim.v >= 0):
            raise self.err(f"rule {r.name}: the bound must be a time or bits, found "
                           f"`{expr_text(r.limit)}`", r.line)
        fr, b = self.frame, r.body
        text = f"{r.name} >= {expr_text(r.limit)}"
        clock = self.clock
        if isinstance(b, lang.Interval):
            ka, kb = self._rule_ref(b.start, r.line), self._rule_ref(b.end, r.line)
            start, end = ("event", fr.after), ("event", fr.until[0] if fr.until else None)
            resp = self.events[kb[1]].by if kb[0] == "event" else clock.by     # owner of B (3.14)
            kind = ("lead" if ka == start and kb == ("clock", "changes") else
                    "lag" if ka == ("clock", "changes") and kb == end else
                    "idle" if ka == end and kb == start else "interval")
            return RuleM(r.name, kind, None, lim, resp, text, r.line, ka, kb)
        if isinstance(b, lang.PulseWidth):
            if b.wire != clock.wire:
                raise self.unsupported(f"rule {r.name}: pulse width of a wire other than the clock",
                                       r.line)
            return RuleM(r.name, b.level, b.wire, lim, clock.by, text, r.line)
        edge = self._clock_edge(b.edge, r.line)
        if isinstance(b, lang.Setup) and edge != "capture":
            raise self.unsupported(f"rule {r.name}: setup to the launch edge", r.line)
        if isinstance(b, lang.Hold) and edge != "launch":
            raise self.unsupported(f"rule {r.name}: hold from the capture edge", r.line)
        owners = {f.owner for f in fr.fields if f.wire == b.wire}
        if not owners:
            raise self.err(f"rule {r.name}: no field is sent on `{b.wire}`", r.line)
        kind = "setup" if isinstance(b, lang.Setup) else "hold"
        # on a wire several parties drive (open drain) each owner answers for its own fields
        resp = owners.pop() if len(owners) == 1 else "owner of each field"
        return RuleM(r.name, kind, b.wire, lim, resp, text, r.line)

    def _param(self, item: lang.Param) -> None:
        if item.name in self.params:
            raise self.err(f"parameter `{item.name}` defined twice", item.line)
        self.ptypes[item.name] = item.type
        if isinstance(item.type, lang.EnumType):
            d = item.default
            if not (isinstance(d, lang.Name) and d.name in item.type.members):
                raise self.err(f"default of `{item.name}` must be one of "
                               f"{' | '.join(item.type.members)}", item.line)
            self.params[item.name] = EnumVal(item.name, d.name)
        elif isinstance(item.type, lang.RangeType):
            v = self._int(item.default, f"parameter `{item.name}`")
            if not item.type.lo <= v <= item.type.hi:
                raise self.err(f"`{item.name}` = {v} outside {item.type.lo}..{item.type.hi}", item.line)
            self.params[item.name] = Q(Fraction(v))
        else:
            self.params[item.name] = self.ev(item.default, f"parameter `{item.name}`")

    def _override(self, name: str, text: str) -> object:
        t = self.ptypes[name]
        text = text.strip()
        if isinstance(t, lang.EnumType):
            if text not in t.members:
                raise self.err(f"`{name}` must be one of {' | '.join(t.members)}, not `{text}`")
            return EnumVal(name, text)
        spaced = re.sub(r"^([0-9.x_a-fA-F]+?)(?=[a-zA-Z%])", r"\1 ", text) \
            if not text.startswith("0x") else text
        try:
            e = lang.Parser(lang.tokenize(spaced, f"<{name}>"), f"<{name}>").expr()
        except lang.ParseError as ex:
            raise self.err(f"parameter `{name}`: cannot read `{text}`: {ex.message}") from None
        v = self.ev(e, f"parameter `{name}`")
        old = self.params[name]
        if isinstance(t, lang.RangeType):
            if not (isinstance(v, Q) and v.is_int() and t.lo <= v.v <= t.hi):
                raise self.err(f"`{name}` must be a whole number in {t.lo}..{t.hi}, not `{text}`")
        elif isinstance(old, Q) and isinstance(v, Q) and old.dim and v.dim is None:
            v = Q(v.v, old.dim)                                # I2
        if isinstance(old, Q) and isinstance(v, Q) and old.dim != v.dim:
            raise self.err(f"parameter `{name}` is a {old.dim or 'number'}, `{text}` is a "
                           f"{v.dim or 'number'}")
        return v

    def _wire(self, w: lang.Wire) -> None:
        if w.name in self.wires:
            raise self.err(f"wire `{w.name}` defined twice", w.line)
        if w.diff:
            raise self.unsupported("wire ... diff (P, N)", w.line)
        if w.coding:
            raise self.unsupported(f"wire ... coding {w.coding}", w.line)
        if w.open_drain:
            if w.by is not None and w.by not in self.parties:
                raise self.err(f"wire `{w.name}`: `{w.by}` is not a party", w.line)
            if w.idle is not None:
                v = self.ev(w.idle, "an idle level")
                if not ((isinstance(v, Sym) and v.word == "released") or
                        (isinstance(v, Q) and v.dim is None and v.v == 1)):
                    raise self.err(f"open-drain wire `{w.name}` is idle released (3.4)", w.line)
            self.wires[w.name] = WireM(w.name, w.by, 1, w.line, open_drain=True)
            return
        if w.by is None:
            raise self.err(f"push-pull wire `{w.name}` needs `by PARTY` (3.4)", w.line)
        if w.by not in self.parties:
            raise self.err(f"wire `{w.name}`: `{w.by}` is not a party", w.line)
        idle: Union[int, str] = "released"
        if w.idle is not None:
            v = self.ev(w.idle, "an idle level")
            if isinstance(v, Sym) and v.word == "released":
                idle = "released"
            elif isinstance(v, Q) and v.dim is None and v.v in (0, 1):
                idle = int(v.v)
            else:
                raise self.err(f"wire `{w.name}`: idle must be high, low, 0, 1 or released", w.line)
        self.wires[w.name] = WireM(w.name, w.by, idle, w.line)

    def _clock(self, c: lang.RateClock) -> RateClockM:
        rate = self.ev(c.rate, "a rate")
        if not (isinstance(rate, Q) and rate.dim == "freq" and rate.v > 0):
            raise self.err(f"clock rate must be a frequency, found `{expr_text(c.rate)}`", c.line)
        if c.tolerance is None:
            raise self.unsupported("clock rate R (without tolerance)", c.line,
                                   "the rounding check of 3.16 item 7 needs a tolerance")  # I3
        tol = self.ev(c.tolerance, "a tolerance")
        if not (isinstance(tol, Q) and tol.dim == "frac" and 0 <= tol.v < 1):
            raise self.err(f"tolerance must be a percentage, found `{expr_text(c.tolerance)}`", c.line)
        return RateClockM(rate.v, tol.v, c.line)

    def _frame(self, fr: lang.Frame) -> FrameM:
        form_b = isinstance(self.clock, WireClockM)
        if fr.after:
            if not form_b:
                raise self.unsupported("frame ... after EVENT (clock-rate form)", fr.line)
            if fr.after not in self.events:
                raise self.err(f"frame `{fr.name}`: unknown event `{fr.after}`", fr.line)
        elif form_b:
            raise self.err(f"frame `{fr.name}` is on a clock wire and needs `after EVENT` "
                           "(3.5, 3.16 item 3)", fr.line)
        if fr.until:
            if not form_b:
                raise self.unsupported("frame ... until EVENT (clock-rate form)", fr.line)
            for u in fr.until:
                if u not in self.events:
                    raise self.err(f"frame `{fr.name}`: unknown event `{u}`", fr.line)
        fields: list[FieldM] = []
        body = self._stmts(fr.body, fields, form_b, [])
        if not fields:
            raise self.err(f"frame `{fr.name}` has no fields for these parameters", fr.line)
        checks = {f.value.crc.name for f in fields if isinstance(f.value, CheckM)}
        if len(checks) > 1:
            raise self.err(f"frame `{fr.name}` uses {len(checks)} check values "
                           f"({', '.join(sorted(checks))}); a lane has one check unit (3.12)", fr.line)
        if form_b:
            self._open_repeats_last(body, True, bool(fr.until))         # 3.7, 3.16 item 3
            return FrameM(fr.name, fields, False, fr.line, fr.after, tuple(fr.until), body)
        first = fields[0]
        idle = self.wires[first.wire].idle
        fb = first.first_wire_bit()
        if fb is None or idle == "released" or fb == idle:
            raise self.err(f"frame `{fr.name}` has no `after`, so its first field must be a "
                           f"constant whose first bit differs from the idle level of "
                           f"`{first.wire}` (3.5)", first.line)
        return FrameM(fr.name, fields, True, fr.line, None, (), body)

    def _open_repeats_last(self, body: list, last: bool, has_until: bool) -> None:
        for i, st in enumerate(body):
            tail = last and i == len(body) - 1
            if isinstance(st, RepeatM):
                if st.count is None and st.while_ is None and not (tail and has_until):
                    raise self.err("an open repeat must be the last statement of a frame with "
                                   "`until` (3.7)", st.line)
                self._open_repeats_last(st.body, False, has_until)
            elif isinstance(st, IfM):
                self._open_repeats_last(st.then, tail, has_until)
                self._open_repeats_last(st.orelse, tail, has_until)

    def _runtime(self, e: object) -> bool:
        return any(isinstance(n, lang.Name) and n.name in self.field_names and n.name not in self.params
                   for n in lang.walk(e))

    def _cond(self, e: object, seen: list[FieldM], what: str) -> CondM:
        """A run-time condition: FIELD == value or FIELD != value, where FIELD
        is the most recent complete occurrence on this path (3.8, 3.16 item 2)."""
        line = getattr(e, "line", 0)
        if isinstance(e, lang.Binary) and e.op in ("==", "!="):
            for fe, ve in ((e.left, e.right), (e.right, e.left)):
                if isinstance(fe, lang.Name) and fe.name in self.field_names:
                    hits = [f for f in seen if f.name == fe.name]
                    if not hits:
                        raise self.err(f"{what} uses `{fe.name}`, which is not complete here "
                                       "(3.16 item 2)", line)
                    f = hits[-1]
                    if isinstance(ve, lang.Name) and ve.name in f.names:
                        v = f.names[ve.name]
                    else:
                        v = self._int(ve, f"the value compared with `{f.name}`")
                    if not 0 <= v < (1 << f.width):
                        raise self.err(f"{what}: {v} does not fit `{f.name}`", line)
                    return CondM(f, e.op, v, expr_text(e), line)
        raise self.unsupported(f"{what} other than FIELD == value or FIELD != value", line)

    def _stmts(self, stmts: tuple, fields: list[FieldM], form_b: bool, seen: list[FieldM]) -> list:
        """Statements with compile-time `if`s resolved; every field is also
        appended to `fields` in description order, and to `seen` (the fields
        complete so far on this path)."""
        out: list = []
        for s in stmts:
            if isinstance(s, lang.Field):
                f = self._field(s, fields, seen)
                fields.append(f)
                seen.append(f)
                out.append(f)
            elif isinstance(s, lang.If):
                if self._runtime(s.cond):
                    if not form_b:
                        raise self.unsupported("if on a field (run-time branch) in clock-rate form",
                                               s.line)
                    cond = self._cond(s.cond, seen, "an `if` condition")
                    then = self._stmts(s.body, fields, form_b, list(seen))
                    orelse = (self._stmts((s.orelse,), fields, form_b, list(seen))
                              if isinstance(s.orelse, lang.If) else
                              self._stmts(s.orelse, fields, form_b, list(seen)) if s.orelse else [])
                    out.append(IfM(cond, then, orelse, s.line))
                elif self._bool(self.ev(s.cond, "an `if` condition"), s.line):
                    out += self._stmts(s.body, fields, form_b, seen)
                elif isinstance(s.orelse, lang.If):
                    out += self._stmts((s.orelse,), fields, form_b, seen)
                elif s.orelse is not None:
                    out += self._stmts(s.orelse, fields, form_b, seen)
            elif isinstance(s, lang.Repeat):
                if not form_b:
                    raise self.unsupported("repeat", s.line)
                if s.count is not None:
                    raise self.unsupported("repeat N (counted)", s.line)
                inner = list(seen)
                body = self._stmts(s.body, fields, form_b, inner)
                cond = self._cond(s.while_, inner, "a `while` condition") if s.while_ else None
                out.append(RepeatM(None, body, s.line, cond))
            elif isinstance(s, lang.Together):
                if not form_b:
                    raise self.unsupported("together", s.line)
                inner = self._stmts(s.body, fields, form_b, seen)
                if len(inner) != 2 or not all(isinstance(x, FieldM) for x in inner):
                    raise self.err("`together` holds exactly two fields (3.9)", s.line)
                a, b = inner
                if a.width != b.width or a.wire == b.wire:
                    raise self.err("`together` fields need the same width and different wires "
                                   "(3.9)", s.line)
                out.append(TogetherM((a, b), s.line))
            else:
                raise self.err("unexpected statement in a frame", getattr(s, "line", 0))
        return out

    def _field(self, f: lang.Field, before: list[FieldM], seen: list[FieldM]) -> FieldM:
        if f.owner not in self.parties:
            raise self.err(f"field `{f.name}`: `{f.owner}` is not a party", f.line)
        width = self._int(f.width, f"the width of `{f.name}`")
        if width < 1:
            raise self.err(f"field `{f.name}` has width {width}", f.line)
        if width > 32:
            raise self.unsupported("field wider than 32 bits", f.line, "the compiler must split it")
        if width > 1 and f.order is None:
            raise self.err(f"field `{f.name}` is wider than one bit and needs msb or lsb (3.8)", f.line)
        form_b = isinstance(self.clock, WireClockM)
        if f.names and not form_b:
            raise self.unsupported("named values (N = v, ...) in clock-rate form", f.line)
        names: dict[str, int] = {}
        for nv in f.names:
            if nv.name in names:
                raise self.err(f"`{f.name}`: value name `{nv.name}` given twice", nv.line)
            v = self._int(nv.value, f"the value of `{nv.name}`")
            if not 0 <= v < (1 << width):
                raise self.err(f"`{f.name}`: {nv.name} = {v} does not fit {width} bit(s)", nv.line)
            names[nv.name] = v
        if f.when is not None and not form_b:
            raise self.unsupported("when (clock-rate form)", f.line)
        carriers = [w for w in self.wires if not (form_b and w == self.clock.wire)]
        if f.wire is not None:
            if f.wire not in self.wires:
                raise self.err(f"field `{f.name}`: unknown wire `{f.wire}`", f.line)
            if f.wire not in carriers:
                raise self.err(f"field `{f.name}` cannot be on the clock wire `{f.wire}`", f.line)
            wire = f.wire
        elif len(carriers) == 1:
            wire = carriers[0]
        else:
            raise self.err(f"field `{f.name}` needs `on WIRE`: more than one wire can carry it", f.line)
        if self.wires[wire].by not in (None, f.owner):
            raise self.err(f"field `{f.name}`: `{f.owner}` cannot drive wire `{wire}` "
                           f"(it is driven by {self.wires[wire].by})", f.line)
        value: Union[None, int, CheckM] = None
        text = ""
        if f.value is not None:
            text = expr_text(f.value)
            if isinstance(f.value, lang.Call):
                value = self._check(f, width, before)
            else:
                v = self._int(f.value, f"the expected value of `{f.name}`")
                if not 0 <= v < (1 << width):
                    raise self.err(f"`{f.name}` = {v} does not fit {width} bit(s)", f.line)
                value = v
        default = None
        if f.default is not None:
            if f.value is not None:
                raise self.err(f"`default` is for data fields only (3.8): `{f.name}`", f.line)
            default = self._int(f.default, f"the default of `{f.name}`")
            if not 0 <= default < (1 << width):
                raise self.err(f"default {default} of `{f.name}` does not fit {width} bit(s)", f.line)
        if (form_b and f.value is None and default is None and f.owner != self.clock.by
                and f.owner != self.clock.stretch_by):
            raise self.err(f"data field `{f.name}`: its owner {f.owner} does not control the "
                           "timing, so it needs a `default` (3.16 item 4)", f.line)
        when = None
        if f.when is not None:
            if f.value is None:
                raise self.err(f"`when` is for expected fields only (3.8): `{f.name}`", f.line)
            if not self.wires[wire].open_drain or not isinstance(value, int) \
                    or value == (1 << width) - 1:
                raise self.err(f"`when` on `{f.name}` needs an open-drain wire and an expected "
                               "value with a 0 bit (3.16 item 5)", f.line)
            when = self._cond(f.when, seen, "a `when` condition")
        on_error = f.on_error or "end"
        return FieldM(f.owner, f.name, width, f.order, wire, value, on_error,
                      self.name_index[f.name], f.line, text, default=default, names=names, when=when)

    def _check(self, f: lang.Field, width: int, before: list[FieldM]) -> CheckM:
        call = f.value
        assert isinstance(call, lang.Call)
        if call.func not in self.crcs:
            raise self.err(f"unknown check function `{call.func}`", call.line)
        crc = self.crcs[call.func]
        if crc.width != width:
            raise self.err(f"`{f.name}` has {width} bit(s) but {crc.name} gives {crc.width}", f.line)
        if crc.reflect and crc.width > 1:
            raise self.unsupported("crc ... reflect (wider than 1 bit)", crc.line)
        ones = (1 << crc.width) - 1
        for what, v in (("init", crc.init), ("xorout", crc.xorout)):
            if v not in (0, ones):
                raise self.err(f"crc {crc.name}: {what} must be all zeros or all ones for the "
                               "check unit (isa.md v0.2)", crc.line)
        covers = []
        for a in call.args:
            if isinstance(a, lang.Range):
                raise self.unsupported("field range A .. B in a check function", call.line)
            if not isinstance(a, lang.Name):
                raise self.err(f"{crc.name}(...) takes field names", call.line)
            hits = [b for b in before if b.name == a.name]
            if not hits:
                raise self.err(f"{crc.name}({a.name}): `{a.name}` is not a field written "
                               "before this one", call.line)
            for b in hits:
                b.covered = True
            covers.append(a.name)
        return CheckM(crc, tuple(covers), expr_text(call))


def elaborate(f: lang.File, overrides: Optional[dict[str, str]] = None,
              stdlib: Optional[lang.File] = None) -> ProtocolM:
    return _Elab(f, overrides or {}, stdlib).run()


# ============================================================== stage 3: role projection

@dataclass
class FieldOp:
    """What one role does with one field."""
    f: FieldM
    action: str                     # "drive", "sample" or "expect"
    src: str = "NONE"               # assembler operand
    dst: str = "NONE"
    tag: int = 0                    # value record tag (sample)
    error_tag: int = 0
    mismatch: Optional[str] = None  # expect: "drop" (frame-start glitch), "continue", "end"
    seen: Optional[int] = None      # value seen on a mismatch, when known at compile time

    @property
    def srcdep(self) -> bool:       # lane.md 7.2: promotion waits one cycle
        return self.src in ("CRC", "SR")

    @property
    def deliveries(self) -> int:    # slots its result takes (sequencer.md 5.10)
        return 0 if self.dst == "NONE" else 2 if self.dst == "CAP" else 1


@dataclass
class TimingM:
    tick: str                       # "BAUD" or "RECOVER"
    prescale: int
    period: int
    period2: Optional[int]
    nominal: Fraction               # cycles per bit at the nominal rate
    bit: Fraction                   # this role's own bit time in cycles
    error: Fraction                 # bit / nominal - 1 (> 0: slow)
    tolerance: Fraction
    frame_bits: int
    accept: Optional[tuple[Fraction, Fraction]] = None    # receiver: far-end bit times decoded

    @property
    def half(self) -> int:          # RECOVER: capture this long after an edge or wrap
        return max(self.period, 2) >> 1

    @property
    def far_min(self) -> Fraction:
        return self.nominal * (1 - self.tolerance)

    @property
    def far_max(self) -> Fraction:
        return self.nominal * (1 + self.tolerance)


@dataclass
class RoleIR:
    proto: ProtocolM
    role: str
    listener: bool
    clock_hz: int
    timing: TimingM
    outputs: dict[str, int]         # wire -> lane output index
    inputs: dict[str, int]          # wire -> lane input index
    data_in: Optional[int]
    check: Optional[CrcM]
    start_level: Optional[int]      # observers: the frame starts at an edge to this level
    staging: list[tuple[int, FieldM]]   # owner: (register, field) popped at frame top
    ops: list[FieldOp]


def project(pm: ProtocolM, role: str, clock_hz: int = CLOCK_HZ) -> RoleIR:
    roles = list(pm.parties) + ["listener"]
    if role not in roles:
        raise CompileError(f"`{role}` is not a role of {pm.name} (roles: {', '.join(roles)})")
    listener = role == "listener"
    fields = pm.frame.fields
    owned = [f for f in fields if f.owner == role]
    observed = [f for f in fields if f.owner != role]
    if owned and observed:
        raise Unsupported("a role that drives some fields and samples others in clock-rate form",
                          owned[0].line, pm.filename, "it switches BAUD and RECOVER between fields")
    outputs = {f.wire: 0 for f in owned}                # out0 is the data output (isa.md 1)
    inputs = {f.wire: 0 for f in observed}
    if len(outputs) > 1 or len(inputs) > 1:
        raise Unsupported("fields on more than one wire in one role", fields[0].line, pm.filename)
    for w in pm.wires.values():                         # a driven wire with no fields of its own
        if w.by == role and w.name not in outputs:
            raise Unsupported(f"wire `{w.name}` driven without fields", w.line, pm.filename)
    data_in = 0 if inputs else None

    staging: list[tuple[int, FieldM]] = []
    for f in owned:                                     # I6
        if f.data:
            if f.width > 16:
                raise Unsupported("a host data field wider than 16 bits in clock-rate form",
                                  f.line, pm.filename, "it would be staged in a 16-bit register")
            if len(staging) == isa.N_REGS:
                raise Unsupported("more than 4 host data fields in one clock-rate frame",
                                  f.line, pm.filename)
            staging.append((len(staging), f))
    staged = {id(f): r for r, f in staging}
    checks = [f.value.crc for f in fields if isinstance(f.value, CheckM)]

    ops: list[FieldOp] = []
    for i, f in enumerate(fields):
        if f.owner == role:
            if f.data:
                src = f"R{staged[id(f)]}"
            elif isinstance(f.value, CheckM):
                src = "CRC"
            else:
                if f.value >= 1 << 9:
                    raise Unsupported("an expected constant wider than the 9-bit immediate",
                                      f.line, pm.filename)
                src = f"IMM {f.value}"
            ops.append(FieldOp(f, "drive", src=src))
        elif f.data:
            ops.append(FieldOp(f, "sample", dst="CAP" if listener else "HOSTQ",
                               tag=TAG_VALUE | f.index))
        else:
            if isinstance(f.value, CheckM):
                src, seen = "CRC", None
            else:
                if f.value >= 1 << 9:
                    raise Unsupported("an expected constant wider than the 9-bit immediate",
                                      f.line, pm.filename)
                src, seen = f"IMM {f.value}", ((~f.value) & 1 if f.width == 1 else None)
            if f.width > 16:
                raise Unsupported("an error record for a field wider than 16 bits", f.line, pm.filename)
            mismatch = "drop" if (i == 0 and pm.frame.implicit_start) else f.on_error
            ops.append(FieldOp(f, "expect", src=src, error_tag=TAG_ERROR | f.index,
                               mismatch=mismatch, seen=seen))
    start_level = None
    if observed and pm.frame.implicit_start:
        idle = pm.wires[fields[0].wire].idle
        start_level = 1 - int(idle)
    n_events = 1 if start_level is not None else 0
    if n_events > isa.N_EVENTS:
        raise CompileError(f"role {role} needs {n_events} events; a lane has {isa.N_EVENTS}")
    ir = RoleIR(pm, role, listener, clock_hz, None, outputs, inputs,     # type: ignore[arg-type]
                data_in, checks[0] if checks else None, start_level, staging, ops)
    ir.timing = timing(pm, ir)
    return ir


# ============================================================== stage 4: timing

def _round(x: Fraction) -> int:
    """Nearest whole number, halves up."""
    return math.floor(x + Fraction(1, 2))


def _baud_split(nominal: Fraction, tol: Fraction) -> tuple[int, int, int]:
    """(prescale, PERIOD, PERIOD2) for a BAUD-timed sender."""          # I4
    for p in PRESCALES:
        half = _round(nominal / 2 / p)
        if half > 0xFFFF:
            continue
        if half < 1:
            break
        if abs(Fraction(2 * half * p) / nominal - 1) <= tol:
            return p, half, half
        whole = _round(nominal / p)
        return p, whole // 2, whole - whole // 2
    raise ValueError("rate outside the BAUD timer range")


def timing(pm: ProtocolM, ir: RoleIR) -> TimingM:
    clk, line, fn = Fraction(ir.clock_hz), pm.clock.line, pm.filename
    nominal = clk / pm.clock.rate_hz
    tol = pm.clock.tolerance
    frame_bits = sum(f.width for f in pm.frame.fields)
    owner = any(op.action == "drive" for op in ir.ops)
    rate_txt = _hz(pm.clock.rate_hz)
    if owner:
        try:
            p, pd, p2d = _baud_split(nominal, tol)
        except ValueError:
            raise CompileError(f"rate {rate_txt} is outside what the BAUD timer can make "
                               f"at {ir.clock_hz} Hz", line, fn) from None
        if pd < 1 or p2d < 2:
            raise CompileError(f"rate {rate_txt} is too fast: a bit of {float(nominal):.2f} "
                               "cycles needs PERIOD >= 1 and PERIOD2 >= 2 (lane.md 7.3)", line, fn)
        bit = Fraction((pd + p2d) * p)
        t = TimingM("BAUD", p, pd, p2d, nominal, bit, bit / nominal - 1, tol, frame_bits)
    else:
        p = next((p for p in PRESCALES if _round(nominal / p) <= 0xFFFF), None)
        if p is None:
            raise CompileError(f"rate {rate_txt} is too slow for the RECOVER timer", line, fn)
        pr = _round(nominal / p)
        if pr < 4:
            raise CompileError(f"rate {rate_txt} is too fast for RECOVER at {ir.clock_hz} Hz", line, fn)
        bit = Fraction(pr * p)
        t = TimingM("RECOVER", p, pr, None, nominal, bit, bit / nominal - 1, tol, frame_bits)
    if abs(t.error) > tol:                                                   # I5
        raise CompileError(
            f"rate {rate_txt}: {t.tick} bit of {float(t.bit):g} cycles is {_pct(t.error)} off the "
            f"nominal {float(nominal):.3f} cycles, outside the tolerance {_pct(tol, sign=False)} "
            "(3.16 item 7)", line, fn)
    if not owner:
        t.accept = _accept_band(t)
        lo, hi = t.accept
        if not (lo < t.far_min and t.far_max < hi):
            raise CompileError(
                f"rate {rate_txt}: the receiver (PERIOD {t.period}) decodes {frame_bits}-bit frames "
                f"only for far-end bits of {float(lo):.1f}..{float(hi):.1f} cycles, but the tolerance "
                f"allows {float(t.far_min):.1f}..{float(t.far_max):.1f}", line, fn)
    return t


def _margin(t: TimingM) -> int:
    """Cycles of timing uncertainty: one input-synchroniser cycle, one cycle
    of a same-cycle event, and the free-running prescaler (ruling 22)."""
    return 2 + (t.prescale - 1)


def _accept_band(t: TimingM) -> tuple[Fraction, Fraction]:
    """Far-end bit times for which RECOVER samples bit k (k = 0 at the frame's
    first edge) strictly inside the bit, for every bit of a frame, assuming
    no edge after the first (the worst case: each edge re-aligns the timer)."""
    m, p = _margin(t), t.prescale
    lo, hi = Fraction(0), Fraction(10**9)
    for k in range(t.frame_bits):
        s = Fraction((t.half + k * t.period) * p)        # sample time after the edge (tick.md 4.3)
        lo = max(lo, (s + m) / (k + 1))
        if k:
            hi = min(hi, (s - m) / k)
    return lo, hi


def _hz(x: Fraction) -> str:
    return f"{int(x)} Hz" if x.denominator == 1 else f"{float(x):.6g} Hz"


def _pct(x: Fraction, sign: bool = True) -> str:
    v = float(x) * 100
    if not sign:
        return f"{abs(v):g} %"
    return f"{abs(v):.2f} % {'slow' if v > 0 else 'fast'}" if v else "exact"


# ============================================================== stage 5: code generation

@dataclass
class Line:
    """One generated instruction, with what the schedule check needs."""
    text: str
    comment: str = ""
    label: Optional[str] = None
    kind: str = "plain"             # field | pin | pop | wait_event | plain
    op: Optional[FieldOp] = None
    target: Optional[str] = None
    cond: Optional[str] = None
    sync: bool = False              # waits for idle bit hardware (sequencer.md 5.11)
    clears_event: bool = False
    mark: str = ""                  # anchor for the clock-wire schedule check
    entry: str = ""                 # first line of an event handler: "start" or "end"
    uses: tuple = ()                # registers it reads (waits while a result is due: HAZ)
    aliases: tuple = ()             # further labels at the same address
    rec: object = None              # the field a record instruction reports


def _setm(reg: str, **fields: object) -> str:
    r = isa.MODE_REG_BY_NAME[reg]
    parts = []
    for name, v in fields.items():
        bf = r.field(name)
        if bf.enum is not None:
            parts.append(f"{name}={v}")
        elif bf.width == 1:
            if v:
                parts.append(name)
        else:
            parts.append(f"{name}={v}")
    return f"SETM  {reg}, " + (", ".join(parts) if parts else "0")


class _Gen:
    def __init__(self, ir: RoleIR) -> None:
        self.ir = ir
        self.lines: list[Line] = []
        self.labels: set[str] = set()
        self.pending_label: Optional[str] = None

    def emit(self, text: str, comment: str = "", **kw) -> Line:
        ln = Line(text, comment, **kw)
        if self.pending_label:
            ln.label, self.pending_label = self.pending_label, None
            ln.aliases, self.pending_aliases = tuple(getattr(self, "pending_aliases", ())), []
        self.lines.append(ln)
        return ln

    def label(self, base: str) -> str:
        name, n = base, 2
        while name in self.labels or name.upper() in isa.OPCODES:
            name, n = f"{base}_{n}", n + 1
        self.labels.add(name)
        return name

    def place(self, name: str) -> None:
        if self.pending_label is None:
            self.pending_label = name
        else:                                   # several labels at one address
            self.pending_aliases = list(getattr(self, "pending_aliases", [])) + [name]

    # ---- program
    def run(self) -> list[Line]:
        ir, t = self.ir, self.ir.timing
        pm = ir.proto
        frame = self.label(pm.frame.name)
        # prologue: mode registers                                          # I12
        self.emit(_setm("TICK", mode=t.tick, **({"prescale": t.prescale} if t.prescale > 1 else {})),
                  f"{pm.frame.fields[0].wire} at {_hz(pm.clock.rate_hz)}")
        if t.tick == "BAUD":
            self.emit(f"SETM  PERIOD, {t.period}", f"launch to capture ({_pct(t.error)})")
            self.emit(f"SETM  PERIOD2, {t.period2}", "capture to next launch")
        else:
            self.emit(f"SETM  PERIOD, {t.period}", f"one bit ({_pct(t.error)})")
        win = next(iter(ir.inputs), None)
        self.emit(_setm("CODE", **({"data_in": f"in{ir.data_in}"} if ir.data_in is not None else {})),
                  f"level coding, data input {win} = in{ir.data_in}" if win else "level coding, push-pull")
        if ir.outputs:
            self.emit("SETM  THR2L, 0", "no launch delay")
        if ir.check:
            c = ir.check
            self.emit(f"SETM  CRCPOLY, 0x{c.poly & 0xFFFF:X}", f"check unit: {c.name}")
            if c.width > 16 or c.poly > 0xFFFF:                                  # I13
                self.emit(f"SETM  CRCPOLY.hi, 0x{c.poly >> 16:X}")
            self.emit(_setm("CRCCFG", width=c.width, init_ones=int(c.init != 0),
                            xorout_ones=int(c.xorout != 0)))
        if ir.start_level is not None:
            w = pm.frame.fields[0].wire
            self.emit(_setm("EVC0", a=f"in{ir.inputs[w]}", trigger="EDGE",
                            level="HIGH" if ir.start_level else "LOW"),
                      f"E0: {w} {'rises' if ir.start_level else 'falls'} (frame start)")
        for w, k in ir.outputs.items():
            idle = pm.wires[w].idle
            self.emit(f"PIN   out{k}={'z' if idle == 'released' else idle}", f"{w} idle", kind="pin")
        # frame
        self.place(frame)
        if ir.start_level is not None:                                            # I14
            self.emit(f"EVT   0, {frame}, off, clear", "forget edges seen before this frame",
                      clears_event=True)
        for reg, f in ir.staging:
            self.emit(f"Q     POP R{reg}", f"{f.name}: host word (the line stays idle until it is there)",
                      kind="pop")
        if ir.check:
            self.emit("CRCINIT", "check unit to its initial value", sync=True)            # I7
        if ir.start_level is not None:
            w = pm.frame.fields[0].wire
            self.emit("WAIT  E0", f"frame start: first edge of {w} away from idle", kind="wait_event")
        ops = ir.ops
        for i, op in enumerate(ops):
            self._field(op)
            if op.action == "expect":
                self._mismatch(op, frame, ops[i + 1] if i + 1 < len(ops) else None)
        if ops and ir.outputs:                                                    # I11
            lb = ops[-1].f.last_wire_bit()
            w = ops[-1].f.wire
            idle = pm.wires[w].idle
            if lb is None or idle == "released" or lb != idle:
                self.emit(f"PIN   out{ir.outputs[w]}={'z' if idle == 'released' else idle}",
                          f"{w} back to idle", kind="pin")
        self.emit(f"BR    always, {frame}", "next frame", target=frame, cond="always")
        return self.lines

    def _field(self, op: FieldOp) -> None:
        f = op.f
        flags = []
        if f.width > 1 and f.order == "lsb":
            flags.append("lsb")
        if f.covered:
            flags.append("crc")
        kind = {"drive": "DRIVE", "sample": "SAMPLE", "expect": "EXPECT"}[op.action]
        if op.action == "sample":
            operands = flags + [op.dst] + ([f"tag=0x{op.tag:02X}"] if op.tag else [])
            note = f"{f.name} -> {'capture record' if op.dst == 'CAP' else 'host'}"
        else:
            operands = flags + [op.src]
            if op.action == "drive":
                note = f"{f.name}" + (f" = {f.value_text}" if not f.data else " (host word)")
            else:
                note = f"{f.name} = {f.value_text}"
        self.emit(f"FIELD {kind} {f.width}, " + ", ".join(operands), note, kind="field", op=op)

    def _mismatch(self, op: FieldOp, frame: str, nxt: Optional[FieldOp]) -> None:
        f = op.f
        last = nxt is None
        if op.mismatch == "drop":
            self.emit(f"BR    NM, {frame}", "no match: a glitch, dropped silently (3.5)",
                      target=frame, cond="NM", sync=True)
            return
        ok = frame if nxt is None else self.label(nxt.f.name)
        self.emit(f"BR    M, {ok}", "match", target=ok, cond="M", sync=True)
        if self.ir.listener:
            self.emit("Q     PUSHTS", "error record: timestamp word")
        if op.seen is not None:
            self.emit(f"Q     PUSHI {op.seen}, tag=0x{op.error_tag:02X}",
                      f"error {f.name}={op.seen}")                                 # I9
        else:
            reg = len(self.ir.staging)
            self.emit(f"MOVSR R{reg}", f"the bits seen on {f.name}", sync=True)
            self.emit(f"Q     PUSH R{reg}, tag=0x{op.error_tag:02X}", f"error {f.name}=seen")
        if op.mismatch == "end" and not last:                                     # I8
            self.emit(f"BR    always, {frame}", "else end: wait for the next frame",
                      target=frame, cond="always")
        if not last:
            self.place(ok)


def _label_index(lines: list[Line]) -> dict[str, int]:
    out = {}
    for i, ln in enumerate(lines):
        for name in ((ln.label,) if ln.label else ()) + ln.aliases:
            out[name] = i
    return out


def render(lines: list[Line]) -> str:
    out = []
    for ln in lines:
        out.extend(f"{a}:" for a in ln.aliases)
        lab = f"{ln.label}:" if ln.label else ""
        if len(lab) >= 8:
            out.append(lab)
            lab = ""
        mnem, _, rest = ln.text.partition(" ")
        body = f"{mnem:<6}{rest.strip()}" if rest else mnem
        text = f"{lab:<8}{body}"
        if ln.comment:
            text = f"{text:<36} ; {ln.comment}"
        out.append(text.rstrip())
    return "\n".join(out) + "\n"


# ============================================================== stage 6: schedule check

@dataclass
class Check:
    name: str
    need: int                       # cycles the instructions may take, worst case
    avail: int                      # cycles the wire allows, worst case

    @property
    def ok(self) -> bool:
        return self.need <= self.avail


def _paths(lines: list[Line], frame: str) -> list[list[int]]:
    """Every path through the frame body: from the frame label until control
    returns to it. Conditional branches are taken both ways."""
    index = _label_index(lines)
    start = index[frame]
    out: list[list[int]] = []

    def walk(i: int, path: list[int]) -> None:
        while True:
            ln = lines[i]
            path = path + [i]
            if ln.target is not None:
                tgt = index[ln.target]
                if ln.cond == "always":
                    if tgt == start:
                        out.append(path)
                        return
                    i = tgt
                    continue
                if tgt == start:
                    out.append(path)
                else:
                    walk(tgt, path)
            i += 1

    walk(start, [])
    return out


def schedule(lines: list[Line], ir: RoleIR, t: Optional[TimingM] = None) -> list[Check]:
    """Static worst-case schedule (language.md 3.16 item 8, arch 5.2).

    Cost model (sequencer.md 5, lane.md 7): one instruction or one delivery
    step per slot of 5 cycles; the first slot after a resume may be up to a
    slot away; a parked bit operation is current 2 cycles after its park (3
    with source CRC/SR); a sync instruction waits for idle bit hardware;
    nothing runs ahead of a parked operation. Times on the wire use the
    fastest far end the tolerance allows (observers) or the role's own exact
    bit (BAUD sender), less the margin of _margin()."""
    t = t or ir.timing
    m = _margin(t)
    own = t.tick == "BAUD"
    tb = t.bit if own else min(t.far_min, Fraction(t.period * t.prescale))
    worst: dict[str, Check] = {}

    def add(name: str, need: int, avail: Fraction) -> None:      # keep the worst path per check
        c = Check(name, need, int(math.floor(avail)))
        old = worst.get(name)
        if old is None or c.avail - c.need < old.avail - old.need:
            worst[name] = c

    for path in _paths(lines, _frame_label(lines)):
        fpos = [k for k, i in enumerate(path) if lines[i].kind == "field"]
        for a_k, b_k in zip(fpos, fpos[1:]):
            A, B = lines[path[a_k]].op, lines[path[b_k]].op
            between = [lines[i] for i in path[a_k + 1:b_k]]
            if any(ln.kind in ("pop", "wait_event", "pin") for ln in between) or B.src == "HOSTQ":
                raise CompileError(f"{ir.role}: `{between[0].text}` between `{A.f.name}` and "
                                   f"`{B.f.name}` can wait indefinitely inside a frame")
            prev = [lines[path[k]].op for k in fpos if k < a_k]
            z_del = prev[-1].deliveries if prev else 0
            dep = 1 if B.srcdep else 0
            name = f"{A.f.name} -> {B.f.name}"
            if any(ln.sync for ln in between):
                if own:
                    s = next(ln for ln in between if ln.sync)
                    raise CompileError(f"{ir.role}: `{s.text}` between `{A.f.name}` and "
                                       f"`{B.f.name}` waits for the end of the bit and leaves a "
                                       "gap in BAUD timing")
                need = (A.deliveries + len(between) + 2) * SLOT + 2 + dep
                add(f"{name} (after the comparison)", need, tb - m)
            else:
                need = (z_del + len(between) + 2) * SLOT + 2 + dep
                add(f"{name} (run ahead)", need, A.f.width * tb - (0 if own else m))
        res = [k for k in fpos if lines[path[k]].op.deliveries]
        for a_k, b_k in zip(res, res[1:]):
            A = lines[path[a_k]].op
            bits = sum(lines[path[k]].op.f.width for k in fpos if a_k < k <= b_k)
            add(f"{A.f.name} delivered before {lines[path[b_k]].op.f.name} completes",
                (A.deliveries + 1) * SLOT + 1, bits * tb - m)
        # frame to frame, observers: re-arm the start event before the next
        # frame's first edge, and have the first field current half a bit after it
        if own or not fpos or lines[path[fpos[-1]]].op is not ir.ops[-1]:
            continue
        tail = [lines[i] for i in path[fpos[-1] + 1:]]
        head = [lines[i] for i in path[:fpos[0]]]
        ring = tail + head
        L = lines[path[fpos[-1]]].op
        k = t.frame_bits - 1
        h = t.half * t.prescale
        pr = t.period * t.prescale
        rem = min(t.far_min - h, (k + 1) * t.far_min - h - k * pr)
        clear = next(j for j, ln in enumerate(ring) if ln.clears_event)
        add(f"{L.f.name} -> start event re-armed", (L.deliveries + clear + 2) * SLOT, rem - m)
        add(f"{L.f.name} -> {ir.ops[0].f.name} of the next frame",
            (L.deliveries + len(ring) + 2) * SLOT + 2, rem + h - m)
        wait = next(j for j, ln in enumerate(head) if ln.kind == "wait_event")
        add(f"frame start edge -> {ir.ops[0].f.name} current",
            1 + SLOT + (len(head) - wait) * SLOT + 2, Fraction(h - m))
    return list(worst.values())


def _frame_label(lines: list[Line]) -> str:
    for ln in lines:
        if ln.target and ln.cond == "always":
            return ln.target
    raise AssertionError("no frame loop")


def fastest_rate(lines: list[Line], ir: RoleIR) -> float:
    """Highest rate (Hz) at which the schedule check still passes, with exact
    rounding (3.16 item 7 asks the compiler to report it)."""
    t0 = ir.timing
    clk = Fraction(ir.clock_hz)

    def ok(rate: Fraction) -> bool:
        nom = clk / rate
        if nom < 6:
            return False
        if t0.tick == "BAUD":
            half = nom / 2
            t = TimingM("BAUD", 1, 0, 0, nom, nom, Fraction(0), t0.tolerance, t0.frame_bits)
            if half < 2:
                return False
        else:
            per = max(int(nom), 4)
            t = TimingM("RECOVER", 1, per, None, nom, Fraction(per), Fraction(0), t0.tolerance,
                        t0.frame_bits)
        try:
            return all(c.ok for c in schedule(lines, ir, t))
        except CompileError:
            return False

    lo, hi = Fraction(1), clk / 6
    if ok(hi):
        return float(hi)
    for _ in range(40):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if ok(mid) else (lo, mid)
    return float(lo)


# ============================================================== clock-wire form (form B: SPI)
#
# The clock owner times bits with BAUD and the clock output on out3; every
# other role follows the clock wire in EDGE mode (language.md 3.5 table).
# The frame is `after START until END { repeat { field | together } }`.
# Latencies used below (cycles at 60 MHz, prescale 1):

IN_LAT = 3          # pin to lane input: two synchroniser flops and the crossbar register (rulings 2, 31)
OUT_LAT = 1         # a lane's output request to its pin: the output register (launch delay 0)
RTRIP = 4           # BAUD samples the data input this long after the capture tick (ruling 18)
EDGE_MARGIN = 2     # an asynchronous input edge may be seen a cycle late, plus a same-cycle tick
D_TO_START = 3      # FIELD's D cycle to its start cycle: park in X, promotion, start (lane.md 7.2)


def _handler_instr(k: int) -> int:
    """Worst cycles from an input pin edge to the D cycle of instruction k
    (k = 0 first) of the handler its event vectors to: the lane sees the edge
    IN_LAT later, the pending bit one cycle after that, VECTOR within a slot
    (sequencer.md 5.6), then one slot per instruction."""
    return IN_LAT + 1 + (SLOT - 1) + SLOT * (k + 1)


@dataclass
class WordOp:
    """What one role does on each iteration of the frame's open repeat."""
    tx: Optional[FieldM]            # the field it drives
    rx: Optional[FieldM]            # the field it samples
    src: str = "NONE"
    default_reg: Optional[int] = None
    dst: str = "NONE"
    tag: int = 0

    @property
    def kind(self) -> str:
        return "EXCHANGE" if self.tx and self.rx else "DRIVE" if self.tx else "SAMPLE"

    @property
    def f(self) -> FieldM:
        return self.tx or self.rx   # type: ignore[return-value]

    @property
    def deliveries(self) -> int:
        return 0 if self.dst == "NONE" else 2 if self.dst == "CAP" else 1

    @property
    def srcdep(self) -> bool:
        return False


@dataclass
class TimingB:
    rate: Fraction                  # R (Hz): the owner's clock at most, the others' fastest clock
    nominal: Fraction               # cycles per bit at R
    owner: bool
    capture_level: int              # level the capture edge goes to
    idle_level: int                 # the clock wire's idle level
    prescale: int = 1
    period: Optional[int] = None    # owner: PERIOD = PERIOD2 (I15)
    rules: dict = field(default_factory=dict)       # lead, lag, idle -> cycles at R
    setup: dict = field(default_factory=dict)       # wire -> cycles, rounded up
    waits: dict = field(default_factory=dict)       # owner: PIN waits in PERIODs

    @property
    def half(self) -> Fraction:
        return self.nominal / 2

    @property
    def launch_level(self) -> int:
        return 1 - self.capture_level

    @property
    def leading_capture(self) -> bool:
        return self.capture_level != self.idle_level

    @property
    def own_half(self) -> int:      # owner: cycles per clock phase
        return (self.period or 0) * self.prescale


@dataclass
class RoleB:
    proto: ProtocolM
    role: str
    listener: bool
    clock_hz: int
    timing: TimingB
    outputs: dict[str, int]
    inputs: dict[str, int]
    clock_owner: bool
    event_owner: bool
    start: EventM
    end: EventM
    word: WordOp
    early: bool
    count_reg: Optional[int]
    listen_wire: Optional[str]


def project_b(pm: ProtocolM, role: str, clock_hz: int, listen_wire: Optional[str]) -> RoleB:
    roles = list(pm.parties) + ["listener"]
    if role not in roles:
        raise CompileError(f"`{role}` is not a role of {pm.name} (roles: {', '.join(roles)})")
    fn, fr, clock = pm.filename, pm.frame, pm.clock
    assert isinstance(clock, WireClockM)
    listener = role == "listener"
    body = fr.body
    if not (len(body) == 1 and isinstance(body[0], RepeatM)):
        first = next((x for x in body if not isinstance(x, RepeatM)), body[0])
        raise Unsupported("a clock-wire frame other than one open repeat", first.line, fn)
    rep = body[0]
    if len(rep.body) != 1 or isinstance(rep.body[0], RepeatM):
        raise Unsupported("a repeat holding more than one field or together", rep.line, fn)
    item = rep.body[0]
    fields = item.fields if isinstance(item, TogetherM) else (item,)
    for f in fields:
        if not f.data:
            raise Unsupported("an expected field in a clock-wire frame", f.line, fn)
    owned = [f for f in fields if f.owner == role]
    if len(owned) > 1:
        raise CompileError(f"{role} drives both fields of a `together` (3.9)", item.line, fn)
    tx = owned[0] if owned else None
    others = [f for f in fields if f is not tx]
    if listener:
        if len(others) > 1 and listen_wire is None:
            raise Unsupported("a listener of `together` fields on two wires", item.line, fn,
                              "one lane samples one wire (gap G5): compile one listener per wire, "
                              "listen_wire=" + " or ".join(f.wire for f in others))
        pick = [f for f in others if listen_wire in (None, f.wire)]             # I19
        if not pick:
            raise CompileError(f"listen_wire `{listen_wire}` carries no field of the frame")
        rx = pick[0]
    else:
        if listen_wire is not None:
            raise CompileError("listen_wire applies to the listener role only")
        rx = others[0] if others else None
    clock_owner = role == clock.by
    start, end = pm.events[fr.after], pm.events[fr.until[0]]
    gen = [e for e in (start, end) if e.by == role]
    if len(gen) == 1:
        raise Unsupported("a role that makes one frame event and detects the other", gen[0].line, fn)
    event_owner = bool(gen)
    if event_owner != clock_owner:
        raise Unsupported("frame events made by a role that does not own the clock", start.line, fn)
    if event_owner and "changes" in (start.edge, end.edge):
        raise Unsupported("making an event `W changes`", start.line, fn)

    outputs: dict[str, int] = {}                        # out0 data, out2 events, out3 clock (isa.md 1)
    if clock_owner:
        outputs[clock.wire] = 3
    if tx:
        outputs[tx.wire] = 0
    if event_owner:
        ew = {start.wire, end.wire}
        if len(ew) > 1:
            raise Unsupported("frame events on two different wires", start.line, fn)
        w = ew.pop()
        if w in outputs:
            raise Unsupported("an event wire that also carries fields or the clock", start.line, fn)
        outputs[w] = 2
    for w in pm.wires.values():
        if w.by == role and w.name not in outputs:
            raise Unsupported(f"wire `{w.name}` driven without fields or events", w.line, fn)
    inputs: dict[str, int] = {}
    if rx:
        inputs[rx.wire] = 0                             # data input
    if not clock_owner:
        inputs.setdefault(clock.wire, len(inputs))
    if not event_owner:
        for e in (start, end):
            inputs.setdefault(e.wire, len(inputs))
    if len(inputs) > 4:
        raise CompileError(f"{role} needs {len(inputs)} inputs; a lane has 4 (3.16 item 6)")

    word = WordOp(tx, rx)
    regs = 0
    count_reg = None
    if event_owner:                                     # open repeat: count from the host (3.10)
        count_reg, regs = 0, 1
    if tx:
        word.src = "HOSTQ"                              # the clock owner pauses the clock (3.15)
        if not clock_owner:
            if tx.default is None or tx.default > 0xFFFF:
                raise Unsupported("a default wider than 16 bits", tx.line, fn)
            word.default_reg, regs = regs, regs + 1     # tags need a register source, not IMM
    if rx:
        if rx.width > 24:
            raise Unsupported("a recorded field wider than 24 bits", rx.line, fn,
                              "the record tag takes bits [31:24]")
        word.dst = "CAP" if listener else "HOSTQ"
        word.tag = TAG_VALUE | rx.index
    il = int(pm.wires[clock.wire].idle)
    early = bool(tx) and not clock_owner and clock.capture_level != il       # I18
    ir = RoleB(pm, role, listener, clock_hz, None, outputs, inputs,          # type: ignore[arg-type]
               clock_owner, event_owner, start, end, word, early, count_reg, listen_wire)
    ir.timing = timing_b(pm, ir)
    return ir


def timing_b(pm: ProtocolM, ir: RoleB, rate: Optional[Fraction] = None) -> TimingB:
    clock = pm.clock
    assert isinstance(clock, WireClockM)
    clk = Fraction(ir.clock_hz)
    r = rate if rate is not None else clock.rate_hz
    nominal = clk / r
    t = TimingB(r, nominal, ir.clock_owner, clock.capture_level, int(pm.wires[clock.wire].idle))
    for rule in pm.rules:                                                    # I16
        cyc = rule.limit.v * (clk if rule.limit.dim == "time" else nominal)
        if rule.kind == "setup":
            t.setup[rule.wire] = math.ceil(cyc)
        else:
            t.rules[rule.kind] = cyc
    if not ir.clock_owner:
        return t
    half = nominal / 2
    p = next((p for p in PRESCALES if math.ceil(half / p) <= 0xFFFF), None)
    if p is None:
        raise CompileError(f"clock rate {_hz(r)} is too slow for the BAUD timer", clock.line, pm.filename)
    t.prescale, t.period = p, max(1, math.ceil(half / p))                    # I15
    hc = t.own_half
    need = math.ceil(t.rules.get("lead", 0))          # select -> first clock change
    extra = hc if t.launch_level == t.idle_level else 0
    w_lead = 0 if 1 + extra >= need else math.ceil((need - extra) / hc)
    need = math.ceil(t.rules.get("lag", 0))           # last clock change -> deselect
    base = hc if t.capture_level == t.idle_level else 0
    w_lag = max(0, math.ceil((need - base) / hc))
    w_idle = math.ceil(math.ceil(t.rules.get("idle", 0)) / hc)
    t.waits = {"lead": w_lead, "lag": w_lag, "idle": w_idle}
    if max(t.waits.values()) > 0xFFF:
        raise CompileError("a rule needs a PIN wait longer than 4095 PERIODs", clock.line, pm.filename)
    return t


def _provided(t: TimingB) -> dict:
    """Owner: the time its program leaves for each event rule, in cycles."""
    hc = t.own_half
    w = t.waits
    lead = (w["lead"] * hc if w["lead"] else 1) + (hc if t.launch_level == t.idle_level else 0)
    lag = w["lag"] * hc + (hc if t.capture_level == t.idle_level else 0)
    return {"lead": lead, "lag": lag, "idle": w["idle"] * hc}


class _GenB(_Gen):
    """Code generation for the clock-wire form."""

    def run(self) -> list[Line]:            # type: ignore[override]
        ir, t = self.ir, self.ir.timing
        pm = ir.proto
        clock = pm.clock
        frame = self.label(pm.frame.name)
        tick: dict[str, object] = {"mode": "BAUD" if ir.clock_owner else "EDGE"}
        if t.prescale > 1:
            tick["prescale"] = t.prescale
        if not ir.clock_owner:
            tick["clock_in"] = f"in{ir.inputs[clock.wire]}"
        if t.capture_level == 0:
            tick["capture"] = "FALLING"
        if ir.clock_owner:
            tick["clock_out"] = 1
            tick["clock_idle"] = t.idle_level
        cap = "rising" if t.capture_level else "falling"
        self.emit(_setm("TICK", **tick),
                  f"{clock.wire} {'out' if ir.clock_owner else 'in'}, capture {cap}, idle {t.idle_level}")
        if ir.clock_owner:
            self.emit(f"SETM  PERIOD, {t.period}", f"{clock.wire} at {_hz(Fraction(ir.clock_hz) / (2 * t.own_half))}")
            self.emit(f"SETM  PERIOD2, {t.period}")
        w = ir.word
        self.emit(_setm("CODE", **({"data_in": "in0"} if w.rx else {})),
                  f"level coding" + (f", {w.rx.wire} = in0" if w.rx else ""))
        if w.tx:
            self.emit("SETM  THR2L, 0", "no launch delay")
        if not ir.event_owner:
            for k, e in ((0, ir.start), (1, ir.end)):
                trig = "ANY" if e.edge == "changes" else "EDGE"
                lv = {"rises": ", level=HIGH", "falls": ", level=LOW", "changes": ""}[e.edge]
                self.emit(f"SETM  EVC{k}, a=in{ir.inputs[e.wire]}, trigger={trig}{lv}",
                          f"E{k}: {e.name} ({e.wire} {e.edge})")
        if w.default_reg is not None:
            self.emit(f"LDI   R{w.default_reg}, {w.tx.default}", f"{w.tx.name} default")
        pins = [(k, wn) for wn, k in sorted(ir.outputs.items(), key=lambda x: x[1]) if k != 3]
        if pins:
            self.emit("PIN   " + ", ".join(f"out{k}={self._idle(wn)}" for k, wn in pins),
                      ", ".join(f"{wn} idle" for _, wn in pins), kind="pin")
        if ir.event_owner:
            self._owner(frame)
        else:
            self._observer(frame)
        return self.lines

    def _idle(self, wire: str) -> str:
        idle = self.ir.proto.wires[wire].idle
        return "z" if idle == "released" else str(idle)

    def _field_text(self, src: str, first: bool) -> tuple[str, str]:
        w = self.ir.word
        flags = ["lsb"] if w.f.order == "lsb" and w.f.width > 1 else []
        if first and self.ir.early:
            flags.append("early")
        tag = [f"tag=0x{w.tag:02X}"] if w.tag else []
        if w.kind == "EXCHANGE":
            ops = flags + [src, w.dst] + tag
        elif w.kind == "DRIVE":
            ops = flags + [src]
        else:
            ops = flags + [w.dst] + tag
        note = []
        if w.tx:
            note.append(f"{w.tx.name} from " + ("the host" if src == "HOSTQ" else "the default"))
        if w.rx:
            note.append(f"{w.rx.name} to " + ("a capture record" if w.dst == "CAP" else "the host"))
        return f"FIELD {w.kind} {w.f.width}, " + ", ".join(ops), "; ".join(note)

    def _owner(self, frame: str) -> None:
        ir, t, w = self.ir, self.ir.timing, self.ir.word
        out_e = ir.outputs[ir.start.wire]
        lv = {"falls": 0, "rises": 1}
        cnt = ir.count_reg
        self.place(frame)
        self.emit(f"Q     POP R{cnt}", "iteration count (the line stays idle until it is there)",
                  kind="pop")
        self.emit(f"CMP   R{cnt}, 0")
        wl = f", wait={t.waits['lead']}" if t.waits["lead"] else ""
        self.emit(f"PIN   out{out_e}={lv[ir.start.edge]}{wl}", f"{ir.start.name}", kind="pin")
        end = self.label("end")
        self.emit(f"BR    Z, {end}", "no iterations", target=end, cond="Z")             # I21
        word = self.label("word")
        self.place(word)
        text, note = self._field_text(w.src, False)
        self.emit(text, note, kind="field", op=w)
        self.emit(f"LOOP  R{cnt}, {word}", target=word, cond="loop")
        self.place(end)
        if t.waits["lag"]:
            self.emit(f"PIN   wait={t.waits['lag']}", "lag after the last clock change", kind="pin")
        extra = f", out{ir.outputs[w.tx.wire]}={self._idle(w.tx.wire)}" if w.tx else ""
        wi = f", wait={t.waits['idle']}" if t.waits["idle"] else ""
        self.emit(f"PIN   out{out_e}={lv[ir.end.edge]}{extra}{wi}", f"{ir.end.name}", kind="pin")
        self.emit(f"BR    always, {frame}", "next frame", target=frame, cond="always")

    def _observer(self, frame: str) -> None:                                      # I24
        ir, w = self.ir, self.ir.word
        start, end = self.label(ir.start.name), self.label(ir.end.name)
        if w.default_reg is None:
            self.emit(f"EVT   1, {end}, on, clear", f"{ir.end.name} ends a frame (3.7)")
            self.emit(f"EVT   0, {start}, on, clear", f"{ir.start.name} starts one")
            self.emit("WAIT  none", "", kind="wait_event")
            self.place(end)
            self._record(ir.end, entry="end")
            self.emit("WAIT  none", "", kind="wait_event")
            self.place(start)
            text, note = self._field_text("NONE", True)
            self.emit(text, note, kind="field", op=w, entry="start")
            self._record(ir.start)
            word = self.label("word")
            self.place(word)
            text, note = self._field_text("NONE", False)
            self.emit(text, note, kind="field", op=w)
            self.emit(f"BR    always, {word}", target=word, cond="always")
            return
        # a driving observer: the first word's source is chosen before the frame starts  # I17
        dflt, poll = self.label(ir.start.name + "_d"), self.label("poll")
        host = self.label(ir.start.name + "_h")
        reg = f"R{w.default_reg}"
        self.emit(f"EVT   1, {end}, on, clear", f"{ir.end.name} ends a frame (3.7)")
        self.emit(f"EVT   0, {dflt}, on, clear", f"{ir.start.name} starts one")
        self.emit(f"BR    always, {poll}", target=poll, cond="always")
        self.place(end)                                                           # I23
        self.emit(f"PIN   out{ir.outputs[w.tx.wire]}={self._idle(w.tx.wire)}",
                  f"{ir.end.name}: drop the word in progress, {w.tx.wire} idle", kind="pin",
                  entry="end")
        empty = self.label(ir.end.name + "_e")
        self.emit(f"BR    QE, {empty}", target=empty, cond="QE")
        self.emit(f"EVT   0, {host}, on", f"the next {ir.start.name} sends the host's word",
                  mark="ready")
        self._record(ir.end)
        self.emit("WAIT  none", f"until {ir.start.name} or {ir.end.name}", kind="wait_event")
        self.place(empty)
        self.emit(f"EVT   0, {dflt}, on", f"the next {ir.start.name} sends the default ...",
                  mark="ready")
        self._record(ir.end)
        self.place(poll)
        self.emit(f"BR    QE, {poll}", "... until the host has a word", target=poll, cond="QE")
        self.emit(f"EVT   0, {host}, on", "then that word", mark="ready")
        self.emit("WAIT  none", f"until {ir.start.name} or {ir.end.name}", kind="wait_event")
        started = self.label("started")
        self.place(host)
        text, note = self._field_text("HOSTQ", True)
        self.emit(text, note, kind="field", op=w, entry="start")
        self.emit(f"BR    always, {started}", target=started, cond="always")
        self.place(dflt)
        text, note = self._field_text(reg, True)
        self.emit(text, note, kind="field", op=w, entry="start")
        self.place(started)
        self._record(ir.start)
        word, wd = self.label("word"), self.label("word_d")
        self.place(word)
        self.emit(f"BR    QE, {wd}", "host queue empty: the default", target=wd, cond="QE")
        text, note = self._field_text("HOSTQ", False)
        self.emit(text, note, kind="field", op=w)
        self.emit(f"BR    always, {word}", target=word, cond="always")
        self.place(wd)
        text, note = self._field_text(reg, False)
        self.emit(text, note, kind="field", op=w)
        self.emit(f"BR    always, {word}", target=word, cond="always")

    def _record(self, e: EventM, entry: str = "") -> None:                        # I20
        if self.ir.listener:
            self.emit("Q     PUSHTS", f"{e.name}: timestamp word", mark="ts:" + e.name, entry=entry)
            entry = ""
        self.emit(f"Q     PUSHI 0, tag=0x{e.index:02X}", f"{e.name} record", entry=entry)


def _walk_count(lines: list[Line], start: int, stop, barrier=lambda ln: False) -> int:
    """Most instructions executed from line `start` up to (not including) the
    first line with stop(line), over every branch path; a path does not
    revisit a line, and a path through a line with barrier(line) does not
    count. -1 if no path reaches such a line."""
    index = _label_index(lines)
    best = -1

    def go(i: int, n: int, seen: frozenset) -> None:
        nonlocal best
        while i < len(lines) and i not in seen:
            ln = lines[i]
            if stop(ln):
                best = max(best, n)
                return
            if barrier(ln):
                return
            seen = seen | {i}
            if ln.target is not None:
                tgt = index[ln.target]
                if ln.cond == "always":
                    i, n = tgt, n + 1
                    continue
                go(tgt, n + 1, seen)
            i, n = i + 1, n + 1

    go(start, 0, frozenset())
    return best


def schedule_b(lines: list[Line], ir: RoleB, rate: Optional[Fraction] = None) -> list[Check]:
    """Worst-case schedule for the clock-wire form at the fastest clock R
    (3.16 item 8). Same cost model as schedule(); event handlers start
    _handler_instr(k) after the pin edge. Observers rely on the other
    parties' rules as minimum times at R, less EDGE_MARGIN."""
    exact = rate is not None
    t = timing_b(ir.proto, ir, rate) if exact else ir.timing
    m, w = EDGE_MARGIN, ir.word
    half = t.half
    out: list[Check] = []
    fields = [i for i, ln in enumerate(lines) if ln.kind == "field"]
    bit_own = 2 * (half if exact else t.own_half)
    frame_edge = lambda ln: ln.kind in ("pin", "pop", "wait_event")   # noqa: E731  (between frames)
    worst = None
    for i in fields:
        k = _walk_count(lines, i + 1, lambda ln: ln.kind == "field", frame_edge)
        if k < 0:
            continue
        need = (w.deliveries + k + 2) * SLOT + 2
        avail = w.f.width * (bit_own if ir.clock_owner else t.nominal) - (0 if ir.clock_owner else m)
        c = Check("next word parked while a word is clocked", need, int(math.floor(avail)))
        if worst is None or c.avail - c.need < worst.avail - worst.need:
            worst = c
    if worst:
        out.append(worst)
    if ir.clock_owner:
        if w.rx:
            out.append(Check("sampled words gap-free: half period >= round trip + 1 (lane.md 7.3)",
                             RTRIP + 1, int(math.floor(half))))
        if w.tx and w.tx.wire in t.setup:
            out.append(Check(f"{w.tx.wire} setup before capture", t.setup[w.tx.wire],
                             int(math.floor(half if exact else t.own_half))))
        return out
    lead, idle = t.rules.get("lead", Fraction(0)), t.rules.get("idle", Fraction(0))
    entries = [i for i, ln in enumerate(lines) if ln.entry == "start"]
    k0 = max(_walk_count(lines, i, lambda ln: ln.kind == "field") for i in entries)
    e0 = _handler_instr(k0)
    if w.tx:
        su = t.setup.get(w.tx.wire, 0)
        if ir.early:
            out.append(Check(f"{ir.start.name} -> first {w.tx.wire} bit set up before the first capture",
                             e0 + D_TO_START + OUT_LAT + su, int(math.floor(lead - m))))
        else:
            out.append(Check(f"{ir.start.name} -> first word current before the first launch edge",
                             e0 + D_TO_START, int(math.floor(lead + IN_LAT - m))))
        out.append(Check(f"launch edge -> {w.tx.wire} set up before the capture edge",
                         IN_LAT + OUT_LAT + su, int(math.floor(half - m))))
    if w.rx:
        first_cap = lead if t.leading_capture else lead + half
        out.append(Check(f"{ir.start.name} -> first word current before the first capture edge",
                         e0 + D_TO_START, int(math.floor(first_cap + IN_LAT - m))))
    ready = [i for i, ln in enumerate(lines) if ln.mark == "ready"]
    if ready:
        end_i = next(i for i, ln in enumerate(lines) if ln.entry == "end")
        j = _walk_count(lines, end_i, lambda ln: ln.mark == "ready")
        out.append(Check(f"{ir.end.name} -> ready for the next {ir.start.name}",
                         _handler_instr(j), int(math.floor(idle + IN_LAT - m))))
    return out


def fastest_rate_b(lines: list[Line], ir: RoleB) -> float:
    clk = Fraction(ir.clock_hz)

    def ok(r: Fraction) -> bool:
        try:
            return all(c.ok for c in schedule_b(lines, ir, r))
        except CompileError:
            return False

    lo, hi = Fraction(1), clk / 4
    if ok(hi):
        return float(hi)
    for _ in range(50):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if ok(mid) else (lo, mid)
    return float(lo)


def _manifest_b(ir: RoleB, words: list[int], origin: int, regs: dict[str, int],
                checks: list[Check], lines: list[Line]) -> dict:
    pm, t, w = ir.proto, ir.timing, ir.word
    clock = pm.clock
    tags: dict[str, dict] = {}
    if w.rx:
        tags[f"0x{w.tag:02X}"] = {"kind": "value", "field": w.rx.name, "width": w.rx.width}
    if not ir.event_owner:
        for e in (ir.start, ir.end):
            tags[f"0x{e.index:02X}"] = {"kind": "event", "event": e.name, "value": 0}
    pins = {wn: {"direction": "out", "lane_signal": f"out{k}", "idle": pm.wires[wn].idle}
            for wn, k in ir.outputs.items()}
    pins.update({wn: {"direction": "in", "lane_signal": f"in{k}"} for wn, k in ir.inputs.items()})
    tm: dict[str, object] = {
        "tick": "BAUD" if ir.clock_owner else "EDGE", "clock": clock.wire,
        "capture": clock.capture_text, "capture_level": t.capture_level,
        "rate_hz_described": float(t.rate),
        "fastest_rate_hz_schedule": round(fastest_rate_b(lines, ir)),
        "rules": {r.name: {"kind": r.kind, "bound": r.text, "responsible": r.responsible,
                           "bound_cycles": round(float(t.setup[r.wire] if r.kind == "setup"
                                                       else t.rules[r.kind]), 3)}
                  for r in pm.rules},
    }
    if ir.clock_owner:
        tm.update({"prescale": t.prescale, "PERIOD": t.period, "PERIOD2": t.period,
                   "clock_cycles": 2 * t.own_half,
                   "rate_hz": float(Fraction(ir.clock_hz) / (2 * t.own_half)),
                   "pin_waits_periods": t.waits, "provided_cycles": _provided(t)})
    in_q: list[dict] = []
    if ir.count_reg is not None:
        in_q.append({"item": "iteration count", "word": "number of words in the frame (0: the "
                     "events only)", "when": "before each frame"})
    if w.tx:
        d = {"field": w.tx.name, "width": w.tx.width, "word": f"value in bits [{w.tx.width - 1}:0]",
             "when": "one per iteration"}
        if w.default_reg is not None:
            d["default"] = w.tx.default
            d["note"] = ("words are taken up to two words ahead of the clock; at the end of a "
                         "frame the ones not clocked are discarded, so queue the words of one "
                         "frame, then wait for its end record before queuing the next (I17)")
        in_q.append(d)
    man: dict[str, object] = {
        "format": "rosetta-manifest-0", "protocol": pm.name, "role": ir.role,
        "params": {k: fmt_value(v) for k, v in pm.params.items()},
        "clock_hz": ir.clock_hz,
        "program": {"origin": origin, "words": len(words), "entry": origin},
        "pins": pins,
        "mode_registers": {k: f"0x{v:04X}" for k, v in regs.items()},
        "timing": tm,
        "frame": {"name": pm.frame.name, "after": ir.start.name, "until": ir.end.name},
        "in_queue": in_q, "out_queue": None,
        # I22
        "on_error": "never halts; no `cut` records (the lane cannot tell how much of an "
                    "abandoned word was clocked)",
        "schedule": [{"check": c.name, "need_cycles": c.need, "available_cycles": c.avail}
                     for c in checks],
    }
    if ir.listener:
        late = {}
        for i, ln in enumerate(lines):
            if ln.mark.startswith("ts:"):
                ev = ln.mark[3:]
                k = i - next(j for j in range(i, -1, -1) if lines[j].entry)
                late[ev] = [_handler_instr(k) - (SLOT - 1), _handler_instr(k)]
        others = [f.wire for f in pm.frame.fields if f.wire != (w.rx.wire if w.rx else None)]
        man["listener"] = {"listens_on": w.rx.wire if w.rx else None, "other_lanes_for": others}
        man["out_queue"] = {
            "words_per_record": 2,
            "layout": "timestamp word (32-bit cycle counter), then {tag[31:24], value[23:0]}",
            "timestamps": {"value": f"cycle the lane saw the word's last capture edge ({IN_LAT} "
                                    "cycles after the pin edge)",
                           "event": "cycle of a Q PUSHTS in the handler",
                           "event_late_cycles_after_pin_edge": late},
            "tags": tags}
    elif tags:
        man["out_queue"] = {"words_per_record": 1, "layout": "{tag[31:24], value[23:0]}", "tags": tags}
    return man


def _compile_b(pm: ProtocolM, role: str, clock_hz: int, origin: int,
               listen_wire: Optional[str]) -> "Compiled":
    for r in pm.rules:
        if r.kind not in ("lead", "lag", "idle", "setup"):
            raise Unsupported(f"rule {r.name} in a frame of one open repeat", r.line, pm.filename)
    ir = project_b(pm, role, clock_hz, listen_wire)
    lines = _GenB(ir).run()             # type: ignore[arg-type]
    if ir.listener and any(ln.kind == "pin" or ln.text.startswith(("FIELD DRIVE", "FIELD EXCHANGE"))
                           for ln in lines):
        raise AssertionError("a listener program must never enable an output (arch obligation 8)")
    checks = schedule_b(lines, ir)
    bad = [c for c in checks if not c.ok]
    if bad:
        raise CompileError(
            f"{role}: the schedule does not fit at {_hz(pm.clock.rate_hz)} (3.16 item 8): "
            + "; ".join(f"{c.name} needs {c.need} cycles, has {c.avail}" for c in bad)
            + f". Fastest rate for this role: {fastest_rate_b(lines, ir):.0f} Hz",
            pm.clock.line, pm.filename)
    text = render(lines)
    prog = asm.assemble(text, origin=origin)
    _check_rulings(prog.words, ir.timing.half)
    regs = _mode_regs(prog.words)
    man = _manifest_b(ir, prog.words, origin, regs, checks, lines)
    return Compiled(pm.name, role, {k: fmt_value(v) for k, v in pm.params.items()}, text, origin,
                    prog.words, prog.listing, regs, man, ir, lines, checks)    # type: ignore[arg-type]


def _check_rulings(words: list[int], low_phase: Fraction) -> None:
    """Compiler rules of decisions.md: 49, no `seek` in BAUD timing; 53,
    `stretch_hold` needs a clock low phase of at least 5 cycles."""
    mode = None
    for w in words:
        ins = isa.decode(w)
        if ins.op == "SETM" and ins["reg"] == 0 and not ins["upper"] and not ins["from_reg"]:
            tick = isa.mode_decode("TICK", ins["imm"])
            mode = tick["mode"]
            if tick["stretch_hold"] and low_phase < 5:
                raise CompileError("stretch_hold needs a clock low phase of at least 5 cycles "
                                   "(decisions.md ruling 53)")
        if ins.op == "FIELD" and ins.get("seek") and mode == isa.TICK_MODES["BAUD"]:
            raise CompileError("`seek` in BAUD timing is not allowed (decisions.md ruling 49)")


# ============================================================== clock-wire form, general (I2C)
#
# Frames with fields outside a repeat, run-time `if`, `repeat ... while`,
# `when`, several end events, events with `while`, open-drain wires and
# clock stretching. One data wire carries every field. The clock owner makes
# the frame events as PIN sequences timed by the rules (3.6); everyone else
# detects them with event detectors and a confirm window K (G4).

CONFIRM_NS = 300    # confirm window for edge-while-level events (G4: UM10204's 300 ns hold)


@dataclass
class GOp:
    """One field as one role runs it (the general clock-wire form)."""
    f: FieldM
    kind: str                       # DRIVE, SAMPLE or EXPECT
    src: str = "NONE"
    dst: str = "NONE"
    tag: int = 0

    @property
    def deliveries(self) -> int:
        return 0 if self.dst == "NONE" else 2 if self.dst == "CAP" else 1

    @property
    def srcdep(self) -> bool:
        return False

    @property
    def dst_reg(self) -> Optional[int]:
        return int(self.dst[1]) if self.dst in ("R0", "R1", "R2", "R3") else None


@dataclass
class TimingG:
    rate: Fraction
    nominal: Fraction               # cycles per bit at R
    capture_level: int
    idle_level: int
    rules: dict = field(default_factory=dict)       # low, high, setup, hold -> cycles at R
    intervals: list = field(default_factory=list)   # (a, b, cycles, name)
    prescale: int = 1
    period: Optional[int] = None    # owner: low phase; stretcher: release delay (D11)
    period2: Optional[int] = None   # owner: high phase less the honour round trip
    honour: bool = False
    delay: int = 0                  # owner: launch delay for the hold rule
    k: int = 0                      # confirm window (cycles)

    @property
    def launch_level(self) -> int:
        return 1 - self.capture_level

    def pulse(self, level: int) -> Fraction:
        """Minimum time the clock stays at `level`, from the rules (else half a bit)."""
        return self.rules.get("high" if level else "low", self.nominal / 2)

    @property
    def period_min(self) -> Fraction:
        return self.pulse(0) + self.pulse(1)


@dataclass
class RoleG:
    proto: ProtocolM
    role: str
    listener: bool
    clock_hz: int
    timing: TimingG
    outputs: dict[str, int]
    inputs: dict[str, int]
    clock_owner: bool
    stretcher: bool
    data_wire: str
    events: list                    # frame events: after first, then the other until events
    cond_fields: set                # id() of fields whose value a condition reads


def _general_shape(pm: ProtocolM) -> bool:
    """False for the SPI-shaped frame that project_b handles."""
    fr, clock = pm.frame, pm.clock
    spi = (len(fr.body) == 1 and isinstance(fr.body[0], RepeatM) and fr.body[0].count is None
           and fr.body[0].while_ is None and len(fr.body[0].body) == 1
           and isinstance(fr.body[0].body[0], (FieldM, TogetherM)) and len(fr.until) == 1
           and not any(w.open_drain for w in pm.wires.values()) and clock.stretch_by is None
           and not any(e.while_wire for e in pm.events.values()))
    return not spi


def _walk_body(body: list):
    for st in body:
        if isinstance(st, FieldM):
            yield st
        elif isinstance(st, TogetherM):
            yield from st.fields
        elif isinstance(st, IfM):
            yield from _walk_body(st.then)
            yield from _walk_body(st.orelse)
        elif isinstance(st, RepeatM):
            yield from _walk_body(st.body)


def _conds(body: list):
    for st in body:
        if isinstance(st, FieldM) and st.when:
            yield st.owner, st.when
        elif isinstance(st, IfM):
            yield None, st.cond
            yield from _conds(st.then)
            yield from _conds(st.orelse)
        elif isinstance(st, RepeatM):
            if st.while_:
                yield None, st.while_
            yield from _conds(st.body)


def project_g(pm: ProtocolM, role: str, clock_hz: int, listen_wire: Optional[str]) -> RoleG:
    roles = list(pm.parties) + ["listener"]
    if role not in roles:
        raise CompileError(f"`{role}` is not a role of {pm.name} (roles: {', '.join(roles)})")
    fn, fr, clock = pm.filename, pm.frame, pm.clock
    assert isinstance(clock, WireClockM)
    if listen_wire is not None:
        raise CompileError("listen_wire is only for a listener of `together` fields on two wires")
    listener = role == "listener"
    for st in _walk_stmts(fr.body):
        if isinstance(st, TogetherM):
            raise Unsupported("together in a frame with fields outside a repeat", st.line, fn)
        if isinstance(st, FieldM) and isinstance(st.value, CheckM):
            raise Unsupported("a check value in a clock-wire frame", st.line, fn)
    fields = list(_walk_body(fr.body))
    wires = {f.wire for f in fields}
    if len(wires) != 1:
        raise Unsupported("fields on more than one wire in this frame shape", fr.line, fn)
    data = wires.pop()
    events = [pm.events[fr.after]] + [pm.events[u] for u in fr.until if u != fr.after]
    for e in events:
        if e.by != clock.by:
            raise Unsupported("a frame event made by a role that does not own the clock", e.line, fn)
        if e.wire != data or e.while_wire not in (None, clock.wire) or e.edge == "changes":
            raise Unsupported("a frame event other than `DATA rises|falls [while CLOCK level]`",
                              e.line, fn)
    clock_owner, stretcher = role == clock.by, role == clock.stretch_by
    drives = clock_owner or any(f.owner == role for f in fields)
    samples = listener or any(f.owner != role for f in fields)
    outputs: dict[str, int] = {}
    if drives and not listener:
        outputs[data] = 0
    if clock_owner or stretcher:
        outputs[clock.wire] = 3
    for w in pm.wires.values():
        if w.by == role and w.name not in outputs:
            raise Unsupported(f"wire `{w.name}` driven without fields or events", w.line, fn)
    inputs: dict[str, int] = {}
    if samples:
        inputs[data] = 0
    inputs.setdefault(clock.wire, len(inputs))              # EDGE ticks, honour, the level wait
    cond_fields = {id(c.field) for owner, c in _conds(fr.body) if owner in (None, role)}
    il = int(pm.wires[clock.wire].idle)
    ir = RoleG(pm, role, listener, clock_hz, None, outputs, inputs, clock_owner,    # type: ignore[arg-type]
               stretcher, data, events, cond_fields)
    ir.timing = timing_g(pm, ir)
    return ir


def _walk_stmts(body: list):
    for st in body:
        yield st
        if isinstance(st, IfM):
            yield from _walk_stmts(st.then)
            yield from _walk_stmts(st.orelse)
        elif isinstance(st, RepeatM):
            yield from _walk_stmts(st.body)


def timing_g(pm: ProtocolM, ir: RoleG) -> TimingG:
    clock = pm.clock
    assert isinstance(clock, WireClockM)
    clk = Fraction(ir.clock_hz)
    nominal = clk / clock.rate_hz
    t = TimingG(clock.rate_hz, nominal, clock.capture_level, int(pm.wires[clock.wire].idle))
    for r in pm.rules:                                                       # I25, as I16
        cyc = r.limit.v * (clk if r.limit.dim == "time" else nominal)
        if r.a is not None:
            t.intervals.append((r.a, r.b, cyc, r.name))
        else:
            t.rules[r.kind] = cyc
    t.k = math.ceil(Fraction(CONFIRM_NS, 10**9) * clk)                      # I27
    if t.k > 63:
        raise CompileError("the confirm window does not fit the 6-bit K at this clock")
    line, fn = clock.line, pm.filename
    if ir.clock_owner:                                                       # I26
        t.honour = clock.stretch_by is not None
        rt = IN_LAT + OUT_LAT if t.honour else 0       # the capture waits until the pin reads high
        low = math.ceil(t.pulse(t.launch_level))       # the launch level is the clock's low here
        high = math.ceil(t.pulse(t.capture_level))
        if low + max(high, rt + 1) > nominal:
            fastest = clk / (low + max(high, rt + 1))
            raise CompileError(f"clock rate {_hz(clock.rate_hz)}: the rules need a clock period of at "
                               f"least {low + max(high, rt + 1)} cycles ({low} at the launch level, "
                               f"{max(high, rt + 1)} at the capture level), {float(nominal):.2f} at "
                               f"this rate (3.16 item 7). Fastest rate: {float(fastest):.0f} Hz",
                               line, fn)
        bit = math.ceil(nominal)                       # never faster than R (I15)
        p = next((p for p in PRESCALES if math.ceil(max(low, bit / 2) / p) <= 0xFFFF), None)
        if p is None:
            raise CompileError(f"clock rate {_hz(clock.rate_hz)} is too slow for the BAUD timer", line, fn)
        pd = math.ceil(max(Fraction(low), Fraction(bit, 2)) / p)
        p2 = max(math.ceil(Fraction(max(high - rt, 1), p)), math.ceil(Fraction(bit - pd * p - rt, p)))
        t.prescale, t.period, t.period2 = p, pd, p2
        t.delay = math.ceil(t.rules.get("hold", 0))
        if t.delay > 255:
            raise CompileError("the hold rule needs a launch delay above 255 cycles", line, fn)
    elif ir.stretcher:
        t.period = max(1, math.ceil(t.rules.get("setup", 1)))   # D11: setup before release
    return t


def _wait_units(t: TimingG, cycles: Fraction) -> int:
    """PIN waits count PERIOD intervals (isa 3.3)."""
    return math.ceil(cycles / (t.period * t.prescale)) if cycles > 0 else 0


class _GenG(_Gen):
    """Code generation for the general clock-wire form."""

    def __init__(self, ir: RoleG) -> None:          # type: ignore[override]
        super().__init__(ir)                        # type: ignore[arg-type]
        self.free = [0, 1, 2, 3]
        self.reg: dict[int, int] = {}               # id(field) -> register holding its value
        self.pending: list = []                     # records waiting for the next field
        self.out_of_line: list = []
        self.stage_reg: Optional[int] = None
        self.ev_k: dict[str, int] = {}
        self.entry_next = ""
        self.idle = ""

    def alloc(self, why: str, line: int) -> int:
        if not self.free:
            raise Unsupported(f"a role that needs more than 4 registers ({why})", line,
                              self.ir.proto.filename)
        return self.free.pop(0)

    def release(self, r: int) -> None:
        if r not in self.free:
            self.free.append(r)
            self.free.sort()

    # ---- helpers
    def level_text(self, wire: str, level: int) -> str:
        return "z" if (level == 1 and self.ir.proto.wires[wire].open_drain) else str(level)

    def flush(self, only: Optional[FieldM] = None) -> None:
        keep = []
        for fld, emit in self.pending:
            if only is None or fld is only:
                emit()
            else:
                keep.append((fld, emit))
        self.pending = keep

    def record_event(self, e: EventM) -> list:
        lines = []
        if self.ir.listener:
            lines.append(("Q     PUSHTS", f"{e.name}: timestamp word", "ts:" + e.name))
        lines.append((f"Q     PUSHI 0, tag=0x{e.index:02X}", f"{e.name} record", ""))
        return lines

    def emit_record(self, lines: list, entry: str = "") -> None:
        for i, (text, note, mark) in enumerate(lines):
            self.emit(text, note, mark=mark, entry=entry if i == 0 else "")

    # ---- program
    def run(self) -> list[Line]:                        # type: ignore[override]
        ir, t, pm = self.ir, self.ir.timing, self.ir.proto
        clock = pm.clock
        frame = self.label(pm.frame.name)
        cin = ir.inputs[clock.wire]
        tick: dict[str, object] = {"mode": "BAUD" if ir.clock_owner else "EDGE",
                                   "clock_in": f"in{cin}"}
        if t.prescale > 1:
            tick["prescale"] = t.prescale
        if t.capture_level == 0:
            tick["capture"] = "FALLING"
        od = pm.wires[clock.wire].open_drain
        if ir.clock_owner or ir.stretcher:
            tick.update(clock_out=1, clock_idle=t.idle_level, clock_od=int(od))
        if ir.clock_owner:
            tick["rest"] = int(any(e.while_wire == clock.wire for e in ir.events))   # I28
            tick["stretch_honour"] = int(t.honour)
        if ir.stretcher:
            tick["stretch_hold"] = 1
        self.emit(_setm("TICK", **tick), f"{clock.wire}: " + (
            "clock owner" if ir.clock_owner else "follows" + (", may stretch" if ir.stretcher else "")))
        if ir.clock_owner:
            self.emit(f"SETM  PERIOD, {t.period}", f"{clock.wire} low (launch to capture)")
            self.emit(f"SETM  PERIOD2, {t.period2}", f"{clock.wire} high, after the capture"
                      + (" reads high" if t.honour else ""))
        elif ir.stretcher:
            self.emit(f"SETM  PERIOD, {t.period}", f"{ir.data_wire} setup before a held "
                                                    f"{clock.wire} is released (D11)")
        code: dict[str, object] = {}
        if ir.data_wire in ir.inputs:
            code["data_in"] = f"in{ir.inputs[ir.data_wire]}"
        if ir.data_wire in ir.outputs and pm.wires[ir.data_wire].open_drain:
            code["data_od"] = 1
        self.emit(_setm("CODE", **code), f"{ir.data_wire}" + (" open-drain" if code.get("data_od") else ""))
        if ir.data_wire in ir.outputs:
            self.emit(f"SETM  THR2L, {t.delay << 8}" if t.delay else "SETM  THR2L, 0",
                      f"launch delay {t.delay}")
        if ir.clock_owner:
            lv = ir.events[0].while_level
            self.emit(f"SETM  EVC0, a=in{cin}, trigger=LEVEL, level={'HIGH' if lv else 'LOW'}",
                      f"E0: {clock.wire} reads {lv} (the waits of 3.6, ruling 25)")
        elif not ir.clock_owner:
            for k, e in enumerate(ir.events):
                self.ev_k[e.name] = k
                b = ""
                if e.while_wire:
                    b = (f", b=in{ir.inputs[e.while_wire]}, b_must={'HIGH' if e.while_level else 'LOW'}"
                         f", k={t.k}")
                lv = "HIGH" if e.edge == "rises" else "LOW"
                self.emit(f"SETM  EVC{k}, a=in{ir.inputs[e.wire]}, trigger=EDGE, level={lv}{b}",
                          f"E{k}: {e.name}" + (f", confirmed for {t.k} cycles (G4)" if b else ""))
        outs = sorted(ir.outputs.items(), key=lambda x: x[1])
        if outs:
            self.emit("PIN   " + ", ".join(f"out{k}={self.level_text(w, int(pm.wires[w].idle))}"
                                           if pm.wires[w].idle != "released" else f"out{k}=z"
                                           for w, k in outs),
                      ", ".join(f"{w} idle" for w, _ in outs), kind="pin")
        if ir.clock_owner:
            self.owner(frame)
        else:
            self.observer(frame)
        while self.out_of_line:
            self.out_of_line.pop(0)()
        return self.lines

    # ---- the clock owner
    def owner(self, frame: str) -> None:
        ir, pm = self.ir, self.ir.proto
        fr = pm.frame
        first = next(iter(_walk_body(fr.body)))
        stage = first.owner == ir.role and first.data
        idle, body = self.label("idle"), self.label(fr.name)
        self.idle = idle
        if stage:                                                            # as I6 and I29
            self.stage_reg = self.alloc("staged host word", first.line)
            self.reg[id(first)] = self.stage_reg
        self.place(idle)
        if stage:
            self.emit(f"Q     POP R{self.stage_reg}", f"{first.name}: host word before "
                      f"{ir.events[0].name}", kind="pop")
        self.gen_event(ir.events[0], from_frame=False)
        self.place(body)
        self.body(fr.body)
        self.frame_end_owner(idle, body)

    def gen_event(self, e: EventM, from_frame: bool) -> None:                     # I29
        """Make `W edge while V level` (3.6) with PINs timed by the rules."""
        ir, t, pm = self.ir, self.ir.timing, self.ir.proto
        clock = pm.clock
        w, v = ir.outputs[e.wire], ir.outputs[clock.wire]
        edge = 1 if e.edge == "rises" else 0
        before = 1 - edge
        half = t.nominal / 2
        su = max([c for a, b, c, _ in t.intervals if b == ("event", e.name) and a[0] == "clock"],
                 default=half)
        after = max([c for a, b, c, _ in t.intervals if a == ("event", e.name)], default=half)
        w_after = _wait_units(t, after)
        if from_frame:
            rest = t.launch_level
            if e.while_level is None or e.while_level == rest:
                raise Unsupported("making an event whose `while` level is the clock's rest level",
                                  e.line, pm.filename)
            w_low = _wait_units(t, t.pulse(rest))
            self.emit(f"PIN   out{w}={self.level_text(e.wire, before)}, wait={w_low}",
                      f"{e.wire} to {before}, {clock.wire} stays {rest} (pulse rule)", kind="pin")
            self.emit(f"EVT   0, {self.idle}, off, clear", f"forget {clock.wire} episodes")
            self.emit(f"PIN   out{v}={self.level_text(clock.wire, e.while_level)}",
                      f"release {clock.wire}", kind="pin")
            self.emit("WAIT  E0", f"until {clock.wire} reads {e.while_level} (it may be stretched)",
                      kind="wait_event")
            w_su = _wait_units(t, su)
            if e.name == ir.events[0].name:     # the clock then falls: keep its pulse rule too
                w_high = _wait_units(t, t.pulse(e.while_level))
                if w_su + w_after < w_high:
                    w_after = w_high - w_su
            if w_su:
                self.emit(f"PIN   wait={w_su}", f"{clock.wire} {e.while_level} to {e.name}", kind="pin")
        else:
            idle_w, idle_c = int(pm.wires[e.wire].idle), int(pm.wires[clock.wire].idle)
            if idle_w != before or (e.while_level is not None and idle_c != e.while_level):
                raise Unsupported("making the frame start event from idle wires that are not at "
                                  "its starting levels", e.line, pm.filename)
        self.emit(f"PIN   out{w}={self.level_text(e.wire, edge)}" + (f", wait={w_after}" if w_after else ""),
                  f"{e.name}", kind="pin")

    def frame_end_owner(self, idle: str, body: str) -> None:
        ir, pm = self.ir, self.ir.proto
        fr = pm.frame
        for r in list(self.reg.values()):
            if r != self.stage_reg:
                self.release(r)
        until = [pm.events[u] for u in fr.until]
        if len(until) > 1:                                                   # 3.15 end choice
            re_ = self.alloc("end choice", fr.line)
            self.emit(f"Q     POP R{re_}", "frame end: " + ", ".join(f"{i} {u.name}"
                                                                     for i, u in enumerate(until)),
                      kind="pop")
            labels = [self.label("end_" + u.name) for u in until]
            for i, lab in enumerate(labels[:-1]):
                self.emit(f"CMP   R{re_}, {i}", uses=(re_,))
                self.emit(f"BR    Z, {lab}", target=lab, cond="Z")
            self.emit(f"BR    always, {labels[-1]}", target=labels[-1], cond="always")
            self.release(re_)
        else:
            labels = [None]
        for u, lab in zip(until, labels):
            if lab:
                self.place(lab)
            if u.name == fr.after:                                           # repeated start
                if self.stage_reg is not None:
                    first = next(iter(_walk_body(fr.body)))
                    self.emit(f"Q     POP R{self.stage_reg}", f"{first.name}: host word", kind="pop")
                self.gen_event(u, from_frame=True)
                self.emit(f"BR    always, {body}", target=body, cond="always")
            else:
                self.gen_event(u, from_frame=True)
                self.emit(f"BR    always, {idle}", target=idle, cond="always")

    # ---- observers
    def observer(self, frame: str) -> None:
        ir, pm = self.ir, self.ir.proto
        fr = pm.frame
        start = self.label(fr.after)                                             # I34
        handlers = {e.name: (start if e.name == fr.after else self.label(e.name)) for e in ir.events}
        for e in reversed(ir.events):
            k = self.ev_k[e.name]
            self.emit(f"EVT   {k}, {handlers[e.name]}, on, clear", f"{e.name} -> {handlers[e.name]}")
        self.emit("WAIT  none", "until an event", kind="wait_event")
        for e in ir.events[1:]:
            self.place(handlers[e.name])
            first = True
            if ir.data_wire in ir.outputs:                                   # as I23
                self.emit("PIN   out0=z", f"{e.name}: {ir.data_wire} released", kind="pin", entry="end")
                first = False
            self.emit_record(self.record_event(e), entry="end" if first else "")
            self.emit("WAIT  none", "until the next frame", kind="wait_event")
        self.place(start)
        entry = "start"
        if ir.data_wire in ir.outputs:
            self.emit("PIN   out0=z", f"{fr.after}: {ir.data_wire} released", kind="pin", entry=entry)
            entry = ""
        self.pending.append((None, lambda: self.emit_record(self.record_event(ir.events[0]))))
        self.entry_next = entry
        self.body(fr.body)
        self.flush()
        self.emit("WAIT  none", "frame body done: until the end event", kind="wait_event")

    # ---- statements (both kinds of role)
    def body(self, stmts: list) -> None:
        for st in stmts:
            if isinstance(st, FieldM):
                self.field(st)
            elif isinstance(st, IfM):
                self.if_(st)
            elif isinstance(st, RepeatM):
                self.repeat(st)

    def emit_field(self, op: GOp, flags: list, note: str) -> None:
        f = op.f
        order = ["lsb"] if f.order == "lsb" and f.width > 1 else []
        if op.kind == "SAMPLE":
            ops = order + [op.dst] + ([f"tag=0x{op.tag:02X}"] if op.tag else [])
        else:
            ops = order + [op.src] + ([op.dst] if op.dst != "NONE" else [])
        entry = getattr(self, "entry_next", "")
        self.entry_next = ""
        self.emit(f"FIELD {op.kind} {f.width}, " + ", ".join(ops), note, kind="field", op=op,
                  entry=entry, uses=((int(op.src[1]),) if op.src[:1] == "R" else ()))
        older = self.pending
        self.pending = []
        for _, emit in older:
            emit()

    def field(self, f: FieldM) -> None:
        ir = self.ir
        if f.owner == ir.role:
            self.drive(f)
        else:
            self.observe(f)

    def drive(self, f: FieldM) -> None:
        ir, pm = self.ir, self.ir.proto
        if not f.data:
            notme, pend = None, []
            if f.when:
                notme = self.label("not_" + f.name)
                self.cond_branch(f.when, notme, negate=True)
                pend = list(self.pending)           # records still due on the not-driven path
            self.emit_field(GOp(f, "DRIVE", src=f"IMM {f.value}"), [], f"{f.name} = {f.value_text}")
            if notme:
                end_action = f.on_error

                def block(lab=notme, pend=pend):
                    self.place(lab)
                    saved, self.pending = self.pending, pend
                    self.flush()
                    self.pending = saved
                    if end_action == "end":                                   # I32
                        self.emit("WAIT  none", f"`when` false: {f.name} not driven; until the "
                                  "frame ends", kind="wait_event")
                    else:
                        raise Unsupported("`when ... else continue`", f.line, pm.filename)
                self.out_of_line.append(block)
            return
        if id(f) in self.reg:                                    # staged before the frame event
            src = f"R{self.reg[id(f)]}"
        elif id(f) in ir.cond_fields:
            r = self.alloc(f"the value of {f.name}", f.line)
            self.reg[id(f)] = r
            self.emit(f"Q     POP R{r}", f"{f.name}: host word" + (f" ({', '.join(f'{k}={v}' for k, v in f.names.items())})" if f.names else ""),
                      kind="pop")
            src = f"R{r}"
        elif ir.clock_owner or ir.stretcher:
            src = "HOSTQ"                         # waits: the owner pauses, a stretcher holds the clock
        else:
            raise Unsupported("a default in this frame shape", f.line, pm.filename)
        note = f"{f.name} from the host" + (" (the clock is held until it is there)"
                                           if src == "HOSTQ" and ir.stretcher else "")
        self.emit_field(GOp(f, "DRIVE", src=src), [], note)

    def observe(self, f: FieldM) -> None:                                         # I33
        ir, pm = self.ir, self.ir.proto
        if f.data:
            need = id(f) in ir.cond_fields
            tag = TAG_VALUE | f.index
            if ir.listener:
                op = GOp(f, "SAMPLE", dst="CAP", tag=tag)
                self.emit_field(op, [], f"{f.name} -> capture record")
                if need:
                    r = self.alloc(f"the value of {f.name}", f.line)
                    self.reg[id(f)] = r
                    self.emit(f"MOVSR R{r}", f"{f.name} for a condition", sync=True)
            elif need:
                r = self.alloc(f"the value of {f.name}", f.line)
                self.reg[id(f)] = r
                op = GOp(f, "SAMPLE", dst=f"R{r}")
                self.emit_field(op, [], f"{f.name} -> R{r}")
                self.pending.append((f, lambda r=r, tag=tag, f=f: self.emit(
                    f"Q     PUSH R{r}, tag=0x{tag:02X}", f"{f.name} record", uses=(r,), rec=f)))
            else:
                self.emit_field(GOp(f, "SAMPLE", dst="HOSTQ", tag=tag), [], f"{f.name} -> host")
            return
        op = GOp(f, "EXPECT", src=f"IMM {f.value}")
        self.emit_field(op, [], f"{f.name} = {f.value_text}")
        ok = self.label("ok_" + f.name)
        bad = self.label("bad_" + f.name)
        self.emit(f"BR    NM, {bad}", "no match", target=bad, cond="NM", sync=True)
        seen = (~f.value) & 1 if f.width == 1 else None
        etag = TAG_ERROR | f.index
        pend = list(self.pending)

        def block(lab=bad, pend=pend):
            self.place(lab)
            saved, self.pending = self.pending, pend
            self.flush()
            self.pending = saved
            if ir.clock_owner:                                                # I31
                if f.on_error != "end":
                    raise Unsupported("`else continue` for the clock owner", f.line, pm.filename)
                until = pm.events[pm.frame.until[0]]                         # 3.15: first end event
                self.gen_event(until, from_frame=True)
                self.error_record(f, seen, etag)
                self.emit("IRQ", "host interrupt")
                self.emit("HALT", "the loader restarts the lane (3.15)")
                return
            self.error_record(f, seen, etag)
            if f.on_error == "end":
                self.emit("WAIT  none", "else end: until the frame ends", kind="wait_event")
            else:
                raise Unsupported("`else continue` in this frame shape", f.line, pm.filename)
        self.out_of_line.append(block)

    def error_record(self, f: FieldM, seen: Optional[int], etag: int) -> None:
        if self.ir.listener:
            self.emit("Q     PUSHTS", "error record: timestamp word")
        if seen is None:
            raise Unsupported("an error record for a field wider than 1 bit in this frame shape",
                              f.line, self.ir.proto.filename)
        self.emit(f"Q     PUSHI {seen}, tag=0x{etag:02X}", f"error {f.name}={seen}", rec=f)

    def cond_reg(self, c: CondM) -> int:
        if id(c.field) not in self.reg:
            raise CompileError(f"`{c.text}`: {self.ir.role} does not hold `{c.field.name}`")
        return self.reg[id(c.field)]

    def cond_branch(self, c: CondM, target: str, negate: bool) -> None:
        """Branch to `target` when the condition is false (negate) or true."""
        r = self.cond_reg(c)
        self.flush(only=c.field)
        self.emit(f"CMP   R{r}, {c.value}", f"{c.text}", uses=(r,))
        eq_true = c.op == "=="
        cond = ("NZ" if eq_true else "Z") if negate else ("Z" if eq_true else "NZ")
        self.emit(f"BR    {cond}, {target}", target=target, cond=cond)

    def if_(self, st: IfM) -> None:
        other, done = self.label("else"), self.label("endif")
        self.cond_branch(st.cond, other, negate=True)
        pend = list(self.pending)
        self.body(st.then)
        self.flush()
        self.emit(f"BR    always, {done}", target=done, cond="always")
        self.place(other)
        self.pending = pend
        self.body(st.orelse)
        self.flush()
        self.place(done)

    def repeat(self, st: RepeatM) -> None:
        ir, pm = self.ir, self.ir.proto
        loop, after = self.label("loop"), self.label("after")
        if st.while_ is None:                       # open: the end-event owner gets a count (3.10)
            if ir.clock_owner:
                rc = self.alloc("iteration count", st.line)
                self.emit(f"Q     POP R{rc}", "iteration count", kind="pop")
                self.emit(f"CMP   R{rc}, 0", uses=(rc,))
                self.emit(f"BR    Z, {after}", "no iterations", target=after, cond="Z")
                self.place(loop)
                self.body(st.body)
                self.flush()
                self.emit(f"LOOP  R{rc}, {loop}", target=loop, cond="loop", uses=(rc,))
                self.release(rc)
            else:
                self.place(loop)
                self.body(st.body)
                self.flush()
                self.emit(f"BR    always, {loop}", "until an end event", target=loop, cond="always")
        else:
            self.place(loop)
            self.body(st.body)
            r = self.cond_reg(st.while_)
            self.flush()
            self.emit(f"CMP   R{r}, {st.while_.value}", f"while {st.while_.text}", uses=(r,))
            cond = "Z" if st.while_.op == "==" else "NZ"
            self.emit(f"BR    {cond}, {loop}", target=loop, cond=cond)
        self.place(after)


def schedule_g(lines: list[Line], ir: RoleG) -> list[Check]:
    """Worst-case schedule for the general clock-wire form (3.16 item 8),
    with the cost model of schedule(). The clock owner may pause its clock
    (a gap only lengthens the clock's launch phase, which the description
    allows), so its own checks are the timing ones of timing_g. A role
    that follows the clock must have each field ready in time for the edges
    of the fastest clock the rules allow; a stretcher can hold the clock only
    while it waits for host data (D11), not while it runs instructions."""
    t, out = ir.timing, []
    if ir.clock_owner:                                                       # I30
        return out
    m = EDGE_MARGIN
    cap_phase = t.pulse(t.capture_level)            # capture to the next launch edge
    launch_phase = t.pulse(t.launch_level)
    index = _label_index(lines)
    worst: dict[str, Check] = {}

    def add(name, need, avail):
        c = Check(name, need, int(math.floor(avail)))
        if name not in worst or c.avail - c.need < worst[name].avail - worst[name].need:
            worst[name] = c

    edge = lambda ln: ln.kind in ("wait_event",)                             # noqa: E731
    for i, ln in enumerate(lines):
        if ln.kind != "field":
            continue
        F: GOp = ln.op
        for j, path in _next_fields(lines, index, i):
            G: GOp = lines[j].op
            dep = any(p.sync or (F.dst_reg is not None and F.dst_reg in p.uses) for p in path)
            k = len(path)
            first = launch_phase + cap_phase if G.kind != "DRIVE" else cap_phase
            if dep:
                need = (F.deliveries + k + 2) * SLOT + 2
                add(f"{F.f.name} -> {G.f.name} ({'drive' if G.kind == 'DRIVE' else 'sample'})",
                    need, first - m)
            else:
                need = (F.deliveries + k + 2) * SLOT + 2
                add(f"{F.f.name} -> {G.f.name} (run ahead)", need,
                    F.f.width * (launch_phase + cap_phase) + first - m)
    # after the frame start event: K confirm cycles, then the handler
    fr = ir.proto.frame
    hold = max([c for a, b, c, _ in t.intervals if a == ("event", fr.after)], default=t.nominal / 2)
    for i, ln in enumerate(lines):
        if ln.entry == "start" or (ln.label == fr.after):
            k0 = _walk_count(lines, i, lambda x: x.kind == "field")
            j = next(j for j in range(i, len(lines)) if lines[j].kind == "field")
            G = lines[j].op
            avail = hold + (launch_phase if G.kind != "DRIVE" else 0)
            add(f"{fr.after} -> first field ready", t.k + _handler_instr(k0) + D_TO_START,
                avail + IN_LAT - m)
            break
    return list(worst.values())


def _next_fields(lines: list[Line], index: dict, i: int):
    """(j, lines between) for every field line j reachable from field line i
    without passing another field or a WAIT."""
    out = []

    def go(k: int, path: list, seen: frozenset) -> None:
        while k < len(lines) and k not in seen:
            ln = lines[k]
            if ln.kind == "field":
                out.append((k, path))
                return
            if ln.kind == "wait_event" or ln.text.startswith(("HALT", "IRQ")):
                return
            seen = seen | {k}
            path = path + [ln]
            if ln.target is not None:
                tgt = index[ln.target]
                if ln.cond == "always":
                    k = tgt
                    continue
                go(tgt, path, seen)
            k += 1

    go(i + 1, [], frozenset())
    return out


def _compile_g(f: lang.File, pm: ProtocolM, role: str, clock_hz: int, origin: int,
               params: dict, listen_wire: Optional[str]) -> "Compiled":
    ir = project_g(pm, role, clock_hz, listen_wire)
    gen = _GenG(ir)
    lines = gen.run()
    if ir.listener and any(ln.kind == "pin" or ln.text.startswith("FIELD DRIVE") for ln in lines):
        raise AssertionError("a listener program must never enable an output (arch obligation 8)")
    checks = schedule_g(lines, ir)
    bad = [c for c in checks if not c.ok]
    if bad:
        raise CompileError(
            f"{role}: the schedule does not fit (3.16 item 8): "
            + "; ".join(f"{c.name} needs {c.need} cycles, has {c.avail}" for c in bad),
            pm.clock.line, pm.filename)
    text = render(lines)
    prog = asm.assemble(text, origin=origin)
    _check_rulings(prog.words, ir.timing.pulse(0))
    regs = _mode_regs(prog.words)
    man = _manifest_g(f, params, ir, prog.words, origin, regs, checks, lines)
    return Compiled(pm.name, role, {k: fmt_value(v) for k, v in pm.params.items()}, text, origin,
                    prog.words, prog.listing, regs, man, ir, lines, checks)     # type: ignore[arg-type]


def fastest_rate_g(f: lang.File, params: dict, role: str, clock_hz: int) -> Optional[float]:
    """Highest clock rate at which this role still compiles (the rules
    re-evaluated at each rate: they may depend on it, as I2C's do)."""
    stdlib = lang.parse_file(STDLIB)
    pm0 = elaborate(f, params, stdlib)
    rp = pm0.clock.rate_param
    if rp is None:
        return None
    clk = Fraction(clock_hz)

    def ok(r: Fraction) -> bool:
        try:
            pm = elaborate(f, {**params, rp: f"{r.numerator} Hz / {r.denominator}"}, stdlib)
            ir = project_g(pm, role, clock_hz, None)
            lines = _GenG(ir).run()
            if not all(c.ok for c in schedule_g(lines, ir)):
                return False
            if not ir.clock_owner:                  # a follower keeps up with what the rules allow
                return r <= clk / math.ceil(ir.timing.period_min)
            return True
        except CompileError:
            return False

    lo, hi = Fraction(1000), clk / 8
    if not ok(lo):
        return None
    if ok(hi):
        return float(hi)
    for _ in range(40):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if ok(mid) else (lo, mid)
    return float(lo)


def _manifest_g(f: lang.File, params: dict, ir: RoleG, words: list[int], origin: int,
                regs: dict[str, int], checks: list[Check], lines: list[Line]) -> dict:
    pm, t = ir.proto, ir.timing
    fr, clock = pm.frame, pm.clock
    tags: dict[str, dict] = {}
    for ln in lines:
        op = ln.op
        if isinstance(op, GOp) and op.tag:
            d = {"kind": "value", "field": op.f.name, "width": op.f.width}
            if op.f.names:
                d["names"] = op.f.names
            tags[f"0x{op.tag:02X}"] = d
        if ln.text.startswith("Q     PUSH R") and "tag=" in ln.text:
            tag = int(ln.text.split("tag=")[1], 16)
            fld = ln.rec
            d = {"kind": "value", "field": fld.name, "width": fld.width}
            if fld.names:
                d["names"] = fld.names
            tags[f"0x{tag:02X}"] = d
        if ln.text.startswith("Q     PUSHI") and "tag=" in ln.text:
            tag = int(ln.text.split("tag=")[1], 16)
            if tag & TAG_ERROR:
                fld = ln.rec
                tags[f"0x{tag:02X}"] = {"kind": "error", "field": fld.name, "width": fld.width,
                                        "expected": fld.value_text}
            else:
                ev = next(e for e in pm.events.values() if e.index == tag)
                tags[f"0x{tag:02X}"] = {"kind": "event", "event": ev.name, "value": 0}
    pins = {w: {"direction": "out", "lane_signal": f"out{k}",
                "open_drain": pm.wires[w].open_drain} for w, k in ir.outputs.items()}
    for w, k in ir.inputs.items():
        pins.setdefault(w, {})
        pins[w]["input"] = f"in{k}"
    rules = {}
    for r in pm.rules:
        cyc = r.limit.v * (Fraction(ir.clock_hz) if r.limit.dim == "time" else t.nominal)
        rules[r.name] = {"bound": r.text, "responsible": r.responsible,
                         "bound_cycles": round(float(cyc), 2)}
    tm: dict[str, object] = {
        "tick": "BAUD" if ir.clock_owner else "EDGE", "clock": clock.wire,
        "capture": clock.capture_text, "rate_hz_described": float(t.rate),
        "confirm_window_cycles": t.k if not ir.clock_owner else None, "rules": rules,
        "fastest_rate_hz": (round(x) if (x := fastest_rate_g(f, params, ir.role, ir.clock_hz))
                            else None),
    }
    if ir.clock_owner:
        rt = IN_LAT + OUT_LAT if t.honour else 0
        hc = t.period * t.prescale
        tm.update({"prescale": t.prescale, "PERIOD": t.period, "PERIOD2": t.period2,
                   "low_cycles": hc, "high_cycles": t.period2 * t.prescale + rt,
                   "clock_cycles": hc + t.period2 * t.prescale + rt,
                   "rate_hz": round(ir.clock_hz / (hc + t.period2 * t.prescale + rt), 1),
                   "honours_stretching": t.honour, "launch_delay": t.delay,
                   "note": "the clock pauses (launch level) wherever the program waits: host "
                           "words, and after each checked field, whose branch waits for the end "
                           "of the bit (I30)"})
    elif ir.stretcher:
        tm["PERIOD"] = t.period
    in_q = []
    if ir.clock_owner or any(x.owner == ir.role and x.data for x in _walk_body(fr.body)):
        in_q = _host_words(ir, fr.body, top=True)
    man: dict[str, object] = {
        "format": "rosetta-manifest-0", "protocol": pm.name, "role": ir.role,
        "params": {k: fmt_value(v) for k, v in pm.params.items()},
        "clock_hz": ir.clock_hz,
        "program": {"origin": origin, "words": len(words), "entry": origin},
        "pins": pins, "mode_registers": {k: f"0x{v:04X}" for k, v in regs.items()},
        "timing": tm,
        "frame": {"name": fr.name, "after": fr.after, "until": list(fr.until)},
        "in_queue": in_q, "out_queue": None,
        "on_error": ("generates " + fr.until[0] + ", records the error, raises the interrupt "
                     "and halts (3.15)" if ir.clock_owner else
                     "records the error and waits for the frame to end; never halts"),
        "schedule": [{"check": c.name, "need_cycles": c.need, "available_cycles": c.avail}
                     for c in checks],
    }
    if ir.listener:
        late = {}
        for i, ln in enumerate(lines):
            if ln.mark.startswith("ts:"):
                k = i - next(j for j in range(i, -1, -1) if lines[j].entry)
                late[ln.mark[3:]] = [t.k + _handler_instr(k) - (SLOT - 1), t.k + _handler_instr(k)]
        man["out_queue"] = {
            "words_per_record": 2,
            "layout": "timestamp word (32-bit cycle counter), then {tag[31:24], value[23:0]}",
            "timestamps": {"value": f"cycle the lane saw the field's last capture edge ({IN_LAT} "
                                    "cycles after the pin edge)",
                           "event": "cycle of a Q PUSHTS in the handler, after the confirm window",
                           "event_late_cycles_after_pin_edge": late,
                           "error": "cycle of a Q PUSHTS after the comparison"},
            "tags": tags}
    elif tags:
        man["out_queue"] = {"words_per_record": 1, "layout": "{tag[31:24], value[23:0]}", "tags": tags}
    return man


def _host_words(ir: RoleG, body: list, top: bool = False) -> list:
    """The in-queue words of the clock owner (or a field owner), in order (3.15)."""
    out: list = []
    for st in body:
        if isinstance(st, FieldM):
            if st.owner == ir.role and st.data:
                d = {"field": st.name, "width": st.width}
                if st.names:
                    d["names"] = st.names
                out.append(d)
        elif isinstance(st, IfM):
            then, orelse = _host_words(ir, st.then), _host_words(ir, st.orelse)
            if then or orelse:
                out.append({"if": st.cond.text, "then": then, "else": orelse})
        elif isinstance(st, RepeatM):
            inner = _host_words(ir, st.body)
            if not inner:
                continue
            item = {"repeat": inner}
            if st.while_ is None and ir.clock_owner:
                item["count_first"] = "iteration count, before the first iteration"
            else:
                item["while"] = st.while_.text if st.while_ else "until an end event"
            out.append(item)
    if top and ir.clock_owner and len(ir.proto.frame.until) > 1:
        out.append({"frame_end": {u: i for i, u in enumerate(ir.proto.frame.until)}})
    return out


# ============================================================== stage 7: output

@dataclass
class Compiled:
    protocol: str
    role: str
    params: dict
    asm: str
    origin: int
    words: list[int]
    listing: list[str]
    mode_regs: dict[str, int]
    manifest: dict
    ir: RoleIR
    lines: list[Line]
    checks: list[Check]

    def to_json(self) -> dict:
        return {"asm": self.asm, "origin": self.origin,
                "words": [f"{w:08x}" for w in self.words],
                "mode_registers": {k: f"0x{v:04X}" for k, v in self.mode_regs.items()},
                "manifest": self.manifest}


def _mode_regs(words: list[int]) -> dict[str, int]:
    regs: dict[str, int] = {}
    for w in words:
        ins = isa.decode(w)
        if ins.op != "SETM" or ins["from_reg"]:
            continue
        name = isa.MODE_REGS[ins["reg"]].name
        if ins["upper"]:
            regs[name] = (regs.get(name, 0) & 0xFFFF) | (ins["imm"] << 16)
        else:
            regs[name] = (regs.get(name, 0) & ~0xFFFF) | ins["imm"]
    return regs


def _manifest(ir: RoleIR, words: list[int], origin: int, regs: dict[str, int],
              checks: list[Check], lines: list[Line]) -> dict:
    pm, t = ir.proto, ir.timing
    tags: dict[str, dict] = {}
    for op in ir.ops:
        if op.action == "sample":
            tags[f"0x{op.tag:02X}"] = {"kind": "value", "field": op.f.name, "width": op.f.width}
        elif op.action == "expect" and op.mismatch != "drop":
            tags[f"0x{op.error_tag:02X}"] = {
                "kind": "error", "field": op.f.name, "width": op.f.width,
                "expected": op.f.value_text, "then": op.mismatch}
    pins = {w: {"direction": "out", "lane_signal": f"out{k}", "idle": pm.wires[w].idle}
            for w, k in ir.outputs.items()}
    pins.update({w: {"direction": "in", "lane_signal": f"in{k}"} for w, k in ir.inputs.items()})
    tm: dict[str, object] = {
        "tick": t.tick, "prescale": t.prescale, "PERIOD": t.period,
        "nominal_bit_cycles": round(float(t.nominal), 3), "bit_cycles": float(t.bit),
        "rate_error_percent": round(float(t.error) * 100, 4),
        "tolerance_percent": float(t.tolerance) * 100, "frame_bits": t.frame_bits,
        "fastest_rate_hz_schedule": round(fastest_rate(lines, ir)),
    }
    if t.period2 is not None:
        tm["PERIOD2"] = t.period2
    if t.accept:
        tm["decodes_far_bit_cycles"] = [round(float(t.accept[0]), 2), round(float(t.accept[1]), 2)]
    man: dict[str, object] = {
        "format": "rosetta-manifest-0",
        "protocol": pm.name, "role": ir.role,
        "params": {k: fmt_value(v) for k, v in pm.params.items()},
        "clock_hz": ir.clock_hz,
        "program": {"origin": origin, "words": len(words), "entry": origin},
        "pins": pins,
        "mode_registers": {k: f"0x{v:04X}" for k, v in regs.items()},
        "timing": tm,
        "frame": pm.frame.name,
        "in_queue": [{"field": f.name, "width": f.width, "word": f"value in bits [{f.width - 1}:0]"}
                     for _, f in ir.staging],
        "out_queue": None,
        "on_error": "records the error; never halts",
        "schedule": [{"check": c.name, "need_cycles": c.need, "available_cycles": c.avail}
                     for c in checks],
    }
    if ir.listener:
        late = {}                       # PUSHTS j lines after its field: D cycle 5j-4 .. 5j later
        for i, ln in enumerate(lines):
            if ln.text.startswith("Q     PUSHTS"):
                j = next(j for j in range(1, i + 1) if lines[i - j].kind == "field")
                late[lines[i - j].op.f.name] = [5 * j - 4, 5 * j]
        man["out_queue"] = {
            "words_per_record": 2,
            "layout": "timestamp word (32-bit cycle counter), then {tag[31:24], value[23:0]}",
            "timestamps": {"value": "cycle of the field's last capture tick (rulings 12, 29)",
                           "error": "cycle of a Q PUSHTS after the comparison",
                           "error_late_cycles": late},
            "tags": tags}
    elif tags:
        man["out_queue"] = {"words_per_record": 1, "layout": "{tag[31:24], value[23:0]}", "tags": tags}
    return man


def compile_protocol(f: lang.File, role: str, params: Optional[dict[str, str]] = None,
                     clock_hz: int = CLOCK_HZ, origin: int = 0,
                     listen_wire: Optional[str] = None) -> Compiled:
    stdlib = lang.parse_file(STDLIB)
    pm = elaborate(f, params, stdlib)
    if isinstance(pm.clock, WireClockM):
        if _general_shape(pm):
            return _compile_g(f, pm, role, clock_hz, origin, dict(params or {}), listen_wire)
        return _compile_b(pm, role, clock_hz, origin, listen_wire)
    if listen_wire is not None:
        raise CompileError("listen_wire applies to a listener of a clock-wire protocol")
    ir = project(pm, role, clock_hz)
    lines = _Gen(ir).run()
    if ir.listener and any(ln.kind == "pin" or (ln.op and ln.op.action == "drive") for ln in lines):
        raise AssertionError("a listener program must never enable an output (arch obligation 8)")
    checks = schedule(lines, ir)
    bad = [c for c in checks if not c.ok]
    if bad:
        raise CompileError(
            f"{role}: the schedule does not fit at {_hz(pm.clock.rate_hz)} (3.16 item 8): "
            + "; ".join(f"{c.name} needs {c.need} cycles, has {c.avail}" for c in bad)
            + f". Fastest rate for this role: {fastest_rate(lines, ir):.0f} Hz",
            pm.clock.line, pm.filename)
    text = render(lines)
    prog = asm.assemble(text, origin=origin)
    _check_rulings(prog.words, Fraction(10**9))
    regs = _mode_regs(prog.words)
    man = _manifest(ir, prog.words, origin, regs, checks, lines)
    return Compiled(pm.name, role, {k: fmt_value(v) for k, v in pm.params.items()}, text, origin,
                    prog.words, prog.listing, regs, man, ir, lines, checks)


def compile_text(text: str, role: str, params: Optional[dict[str, str]] = None,
                 filename: str = "<string>", **kw) -> Compiled:
    return compile_protocol(lang.parse(text, filename), role, params, **kw)


def compile_file(path: str, role: str, params: Optional[dict[str, str]] = None, **kw) -> Compiled:
    return compile_protocol(lang.parse_file(path), role, params, **kw)
