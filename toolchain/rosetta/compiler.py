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
        "writes TICK, PERIOD (PERIOD2) and CODE, and a driving role THR2L "
        "(launch delay 0), rather than relying on reset values.",
    13: "isa.md 4 'CRCPOLY 32 bits' (asm: 'write the low half, then "
        "CRCPOLY.hi'): the high half is written only for a width or polynomial "
        "beyond 16 bits; the check unit's bits below the width do not depend "
        "on higher polynomial bits (rosetta_crc.sv).",
    14: "3.5 'A frame with no after begins at the first edge away from the "
        "idle level': for an observer, the first such edge after the previous "
        "frame's last field has been checked (the start event's pending bit is "
        "cleared there, so edges inside a frame or a failed stop bit do not "
        "start the next one).",
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
class FrameM:
    name: str
    fields: list[FieldM]
    implicit_start: bool
    line: int


@dataclass
class ProtocolM:
    name: str
    filename: str
    params: dict[str, object]
    parties: tuple[str, ...]
    wires: dict[str, WireM]
    clock: RateClockM
    crcs: dict[str, CrcM]
    frame: FrameM
    names: dict[str, int]           # field name -> record index


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
        clocks: list[lang.RateClock] = []
        for item in p.items:                     # in source order, so the first unsupported
            if isinstance(item, lang.Wire):      # construct is the one reported
                self._wire(item)
            elif isinstance(item, lang.RateClock):
                clocks.append(item)
            elif isinstance(item, lang.Event):
                raise self.unsupported("event", item.line)
            elif isinstance(item, lang.Stuff):
                raise self.unsupported("stuff", item.line)
            elif isinstance(item, lang.Rule):
                raise self.unsupported("rule", item.line)
            elif isinstance(item, lang.WireClock):
                raise self.unsupported("clock WIRE by PARTY (clock wire)", item.line)
            elif isinstance(item, lang.SymbolClock):
                raise self.unsupported("clock symbols", item.line)
        if len(clocks) != 1:
            raise self.err("a protocol needs exactly one `clock` statement", p.line)
        clock = self._clock(clocks[0])
        frames = p.items_of(lang.Frame)
        if len(frames) != 1:
            raise self.err("the core language has exactly one frame per protocol", p.line)
        frame = self._frame(frames[0])
        return ProtocolM(p.name, self.fn, self.params, self.parties, self.wires, clock,
                         self.crcs, frame, self.name_index)

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
            raise self.unsupported("wire ... open_drain", w.line)
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
        if fr.after:
            raise self.unsupported("frame ... after EVENT", fr.line)
        if fr.until:
            raise self.unsupported("frame ... until EVENT", fr.line)
        self.name_index: dict[str, int] = {}
        for n in lang.walk(fr.body):                           # I10
            if isinstance(n, lang.Field) and n.name not in self.name_index:
                self.name_index[n.name] = len(self.name_index) + 1
        if len(self.name_index) > MAX_TAG_INDEX:
            raise self.err(f"more than {MAX_TAG_INDEX} field names: record tags run out", fr.line)
        fields: list[FieldM] = []
        self._flatten(fr.body, fields)
        if not fields:
            raise self.err(f"frame `{fr.name}` has no fields for these parameters", fr.line)
        first = fields[0]
        idle = self.wires[first.wire].idle
        fb = first.first_wire_bit()
        if fb is None or idle == "released" or fb == idle:
            raise self.err(f"frame `{fr.name}` has no `after`, so its first field must be a "
                           f"constant whose first bit differs from the idle level of "
                           f"`{first.wire}` (3.5)", first.line)
        checks = {f.value.crc.name for f in fields if isinstance(f.value, CheckM)}
        if len(checks) > 1:
            raise self.err(f"frame `{fr.name}` uses {len(checks)} check values "
                           f"({', '.join(sorted(checks))}); a lane has one check unit (3.12)", fr.line)
        return FrameM(fr.name, fields, True, fr.line)

    def _flatten(self, stmts: tuple, out: list[FieldM]) -> None:
        for s in stmts:
            if isinstance(s, lang.Field):
                out.append(self._field(s, out))
            elif isinstance(s, lang.If):
                if self._bool(self.ev(s.cond, "an `if` condition"), s.line):
                    self._flatten(s.body, out)
                elif isinstance(s.orelse, lang.If):
                    self._flatten((s.orelse,), out)
                elif s.orelse is not None:
                    self._flatten(s.orelse, out)
            elif isinstance(s, lang.Repeat):
                raise self.unsupported("repeat", s.line)
            elif isinstance(s, lang.Together):
                raise self.unsupported("together", s.line)
            else:
                raise self.err("unexpected statement in a frame", getattr(s, "line", 0))

    def _field(self, f: lang.Field, before: list[FieldM]) -> FieldM:
        if f.owner not in self.parties:
            raise self.err(f"field `{f.name}`: `{f.owner}` is not a party", f.line)
        width = self._int(f.width, f"the width of `{f.name}`")
        if width < 1:
            raise self.err(f"field `{f.name}` has width {width}", f.line)
        if width > 32:
            raise self.unsupported("field wider than 32 bits", f.line, "the compiler must split it")
        if width > 1 and f.order is None:
            raise self.err(f"field `{f.name}` is wider than one bit and needs msb or lsb (3.8)", f.line)
        if f.names:
            raise self.unsupported("named values (N = v, ...)", f.line)
        if f.when is not None:
            raise self.unsupported("when", f.line)
        if f.default is not None:
            raise self.unsupported("default", f.line)
        if f.wire is not None:
            if f.wire not in self.wires:
                raise self.err(f"field `{f.name}`: unknown wire `{f.wire}`", f.line)
            wire = f.wire
        elif len(self.wires) == 1:
            wire = next(iter(self.wires))
        else:
            raise self.err(f"field `{f.name}` needs `on WIRE`: more than one wire can carry it", f.line)
        if self.wires[wire].by != f.owner:
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
        on_error = f.on_error or "end"
        return FieldM(f.owner, f.name, width, f.order, wire, value, on_error,
                      self.name_index[f.name], f.line, text)

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
        self.lines.append(ln)
        return ln

    def label(self, base: str) -> str:
        name, n = base, 2
        while name in self.labels or name.upper() in isa.OPCODES:
            name, n = f"{base}_{n}", n + 1
        self.labels.add(name)
        return name

    def place(self, name: str) -> None:
        assert self.pending_label is None
        self.pending_label = name

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


def render(lines: list[Line]) -> str:
    out = []
    for ln in lines:
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
    index = {ln.label: i for i, ln in enumerate(lines) if ln.label}
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
                     clock_hz: int = CLOCK_HZ, origin: int = 0) -> Compiled:
    stdlib = lang.parse_file(STDLIB)
    pm = elaborate(f, params, stdlib)
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
    regs = _mode_regs(prog.words)
    man = _manifest(ir, prog.words, origin, regs, checks, lines)
    return Compiled(pm.name, role, {k: fmt_value(v) for k, v in pm.params.items()}, text, origin,
                    prog.words, prog.listing, regs, man, ir, lines, checks)


def compile_text(text: str, role: str, params: Optional[dict[str, str]] = None,
                 filename: str = "<string>", **kw) -> Compiled:
    return compile_protocol(lang.parse(text, filename), role, params, **kw)


def compile_file(path: str, role: str, params: Optional[dict[str, str]] = None, **kw) -> Compiled:
    return compile_protocol(lang.parse_file(path), role, params, **kw)
