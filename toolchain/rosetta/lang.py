"""Parser for the core Rosetta description language (docs/language.md).

Lexical rules are section 3.1, the grammar is section 5.1 and the reserved
words are section 5.3 (including the extension words `runtime seek net`,
whose productions in 5.2 are deliberately not implemented).  The result is a
tree of frozen dataclasses; every node carries its source `line`, which is
left out of equality and repr so trees can be compared structurally.

Interpretations the grammar leaves open (see the module tests):
- A unit (`s ms us ns Hz kHz MHz bit %`) is a unit only directly after a
  number.  The unit words are not reserved, so `bit` is a name elsewhere.
- A number may not touch a letter, digit or `_` (`100kHz`, `0x2bit`):
  word units need a space.  `%` may touch (`2%`).
- Newlines are ignored inside `( )` only, never inside `[ ]` or `{ }`.
- A wire attribute may appear in any order (`{ wire_attr }`) but at most once.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field, fields
from typing import Iterator, List, NamedTuple, Optional, Tuple, Union

KEYWORDS = frozenset("""
    after and both by capture changes clock coding complement continue crc
    default diff else end equal event falling falls for frame from high hold
    idle if in init launch leading low lsb manchester msb not nrzi ones on
    open_drain or over param parties poly protocol rate released reflect
    repeat rises rising rule setup stretch stuff symbols to together tolerance
    trailing until when while width wire xorout zero
    runtime seek net""".split())
UNITS = frozenset(["s", "ms", "us", "ns", "Hz", "kHz", "MHz", "bit", "%"])
VALUE_WORDS = frozenset(["high", "low", "leading", "trailing", "rising", "falling", "released"])
EDGES = ("rises", "falls", "changes")
LEVELS = ("high", "low")
CMP_OPS = ("==", "!=", "<", "<=", ">", ">=")


class ParseError(Exception):
    """A lexical or syntax error, reported as `filename:line:column: message`."""

    def __init__(self, message: str, filename: str, line: int, column: int) -> None:
        super().__init__(f"{filename}:{line}:{column}: {message}")
        self.message, self.filename, self.line, self.column = message, filename, line, column


# ---------------------------------------------------------------- syntax tree

@dataclass(frozen=True)
class Node:
    line: int = field(default=0, compare=False, repr=False, kw_only=True)

@dataclass(frozen=True)
class Number(Node):
    value: Union[int, float]
    unit: Optional[str] = None          # as written: "Hz", "%", "bit", ...

@dataclass(frozen=True)
class Name(Node):
    name: str

@dataclass(frozen=True)
class ValueWord(Node):
    word: str                           # high low leading trailing rising falling released

@dataclass(frozen=True)
class Range(Node):
    lo: Expr                            # NAME .. NAME (call args, `over`) or expr .. expr
    hi: Expr

@dataclass(frozen=True)
class Call(Node):
    func: str
    args: Tuple[Union[Expr, Range], ...]

@dataclass(frozen=True)
class Unary(Node):
    op: str                             # "~", "-" or "not"
    operand: Expr

@dataclass(frozen=True)
class Binary(Node):
    op: str                             # or and == != < <= > >= + - * /
    left: Expr
    right: Expr

@dataclass(frozen=True)
class Conditional(Node):
    cond: Expr
    then: Expr
    orelse: Expr

Expr = Union[Number, Name, ValueWord, Call, Unary, Binary, Conditional]

@dataclass(frozen=True)
class EnumType(Node):
    members: Tuple[str, ...]

@dataclass(frozen=True)
class RangeType(Node):
    lo: int
    hi: int

@dataclass(frozen=True)
class Param(Node):
    name: str
    type: Optional[Union[EnumType, RangeType]]
    default: Expr

@dataclass(frozen=True)
class Parties(Node):
    names: Tuple[str, ...]

@dataclass(frozen=True)
class Wire(Node):
    name: str
    diff: Optional[Tuple[str, str]] = None
    by: Optional[str] = None
    idle: Optional[Expr] = None
    open_drain: bool = False
    coding: Optional[str] = None        # "nrzi" or "manchester"

@dataclass(frozen=True)
class RateClock(Node):
    rate: Expr
    tolerance: Optional[Expr] = None

@dataclass(frozen=True)
class WireClock(Node):
    wire: str
    by: str
    rate: Expr
    capture: Expr
    stretch_by: Optional[str] = None

@dataclass(frozen=True)
class Symbol(Node):
    value: int
    high: Expr
    low: Expr

@dataclass(frozen=True)
class SymbolClock(Node):
    tolerance: Optional[Expr]
    symbols: Tuple[Symbol, ...]

Duration = Union[Expr, Range]

@dataclass(frozen=True)
class EdgeCondition(Node):
    wire: str
    edge: str                           # rises falls changes
    while_wire: Optional[str] = None
    while_level: Optional[str] = None

@dataclass(frozen=True)
class LevelCondition(Node):
    wire: str
    level: str
    duration: Duration

@dataclass(frozen=True)
class BothLowCondition(Node):
    wire_a: str
    wire_b: str
    duration: Duration

@dataclass(frozen=True)
class Event(Node):
    name: str
    condition: Union[EdgeCondition, LevelCondition, BothLowCondition]
    by: str

@dataclass(frozen=True)
class CrcDef(Node):
    name: str
    width: int
    poly: int
    init: int
    xorout: int
    reflect: bool = False

@dataclass(frozen=True)
class Stuff(Node):
    kind: str                           # "zero" (after N ones) or "complement" (after N equal)
    run: int
    over: Optional[Range] = None

@dataclass(frozen=True)
class NamedValue(Node):
    name: str
    value: Expr

@dataclass(frozen=True)
class Field(Node):
    owner: str
    name: str
    width: Expr
    order: Optional[str] = None         # "msb" or "lsb"
    wire: Optional[str] = None          # `on WIRE`
    names: Tuple[NamedValue, ...] = ()
    value: Optional[Expr] = None        # `= EXPR` (expected field)
    when: Optional[Expr] = None
    default: Optional[Expr] = None
    on_error: Optional[str] = None      # "end" or "continue" as written (None: absent)

@dataclass(frozen=True)
class Together(Node):
    body: Tuple[Stmt, ...]

@dataclass(frozen=True)
class Repeat(Node):
    count: Optional[Expr]
    body: Tuple[Stmt, ...]
    while_: Optional[Expr] = None

@dataclass(frozen=True)
class If(Node):
    cond: Expr
    body: Tuple[Stmt, ...]
    orelse: Optional[Union[If, Tuple[Stmt, ...]]] = None   # `else if` gives an If

Stmt = Union[Field, Together, Repeat, If]

@dataclass(frozen=True)
class Frame(Node):
    name: str
    after: Optional[str]
    until: Tuple[str, ...]
    body: Tuple[Stmt, ...]

@dataclass(frozen=True)
class WireEdge(Node):
    wire: str
    edge: str                           # rises falls changes

@dataclass(frozen=True)
class EventRef(Node):
    name: str

@dataclass(frozen=True)
class ClockEdge(Node):
    kind: str                           # "capture" or "launch"

EdgeRef = Union[WireEdge, EventRef, ClockEdge]

@dataclass(frozen=True)
class PulseWidth(Node):
    level: str
    wire: str

@dataclass(frozen=True)
class Setup(Node):
    wire: str
    edge: EdgeRef

@dataclass(frozen=True)
class Hold(Node):
    wire: str
    edge: EdgeRef

@dataclass(frozen=True)
class Interval(Node):
    start: EdgeRef
    end: EdgeRef

@dataclass(frozen=True)
class Rule(Node):
    name: str
    body: Union[PulseWidth, Setup, Hold, Interval]
    op: str                             # ">=", "<=" or "in"
    limit: Union[Expr, Range]           # a Range for "in"

Item = Union[Param, Parties, Wire, RateClock, WireClock, SymbolClock, Event,
             CrcDef, Stuff, Frame, Rule]

@dataclass(frozen=True)
class Protocol(Node):
    name: str
    items: Tuple[Item, ...]

    def items_of(self, *classes: type) -> tuple:
        return tuple(item for item in self.items if isinstance(item, classes))

@dataclass(frozen=True)
class File(Node):
    protocol: Optional[Protocol]
    crcs: Tuple[CrcDef, ...]
    filename: str = "<string>"


def walk(node: object) -> Iterator[Node]:
    """Yield `node` and every node below it, depth first, in source order."""
    if isinstance(node, Node):
        yield node
        for f in fields(node):
            yield from walk(getattr(node, f.name))
    elif isinstance(node, tuple):
        for child in node:
            yield from walk(child)


# ---------------------------------------------------------------- lexer

class Token(NamedTuple):
    kind: str           # "NAME", "NUMBER", "NL", "EOF", a keyword, or punctuation
    text: str
    value: Union[int, float, None]
    line: int
    column: int


TOKEN_RE = re.compile(r"""
    (?P<space>[ \t\r]+)
  | (?P<comment>//[^\n]*)
  | (?P<nl>\n)
  | (?P<number>[0-9]+\.[0-9]+ | 0x[0-9A-Fa-f][0-9A-Fa-f_]* | 0b[01][01_]* | [0-9][0-9_]*)
  | (?P<name>[A-Za-z_][A-Za-z0-9_]*)
  | (?P<punct>\.\. | == | != | <= | >= | [<>+\-*/~?:,=|()\[\]{}%])
""", re.VERBOSE)
WORD_CHAR = re.compile(r"[A-Za-z0-9_]")


def number_value(text: str) -> Union[int, float]:
    digits = text.replace("_", "")
    if digits.startswith("0x"):
        return int(digits[2:], 16)
    if digits.startswith("0b"):
        return int(digits[2:], 2)
    return float(digits) if "." in digits else int(digits)


def tokenize(text: str, filename: str = "<string>") -> List[Token]:
    """Split `text` into tokens; newlines inside `( )` and comments are dropped."""
    tokens: List[Token] = []
    parens: List[Tuple[int, int]] = []          # positions of open `(`
    pos, line, line_start = 0, 1, 0
    while pos < len(text):
        m = TOKEN_RE.match(text, pos)
        col = pos - line_start + 1
        if not m:
            raise ParseError(f"unexpected character {text[pos]!r}", filename, line, col)
        kind, s = m.lastgroup, m.group()
        if kind == "nl":
            if not parens:
                tokens.append(Token("NL", s, None, line, col))
            line, line_start = line + 1, m.end()
        elif kind == "number":
            if WORD_CHAR.match(text, m.end()):
                bad = re.match(r"[A-Za-z0-9_.]*", text[pos:]).group()
                raise ParseError(f"malformed number {bad!r} (put a space between "
                                 "a number and its unit)", filename, line, col)
            tokens.append(Token("NUMBER", s, number_value(s), line, col))
        elif kind == "name":
            tokens.append(Token(s if s in KEYWORDS else "NAME", s, None, line, col))
        elif kind == "punct":
            if s == "(":
                parens.append((line, col))
            elif s == ")":
                if not parens:
                    raise ParseError("unmatched ')'", filename, line, col)
                parens.pop()
            tokens.append(Token(s, s, None, line, col))
        pos = m.end()
    if parens:
        raise ParseError("unclosed '('", filename, *parens[-1])
    tokens.append(Token("EOF", "", None, line, len(text) - line_start + 1))
    return tokens


def describe(tok: Token) -> str:
    if tok.kind == "NAME":
        return f"name {tok.text!r}"
    if tok.kind == "NUMBER":
        return f"number {tok.text}"
    if tok.kind in ("NL", "EOF"):
        return "end of line" if tok.kind == "NL" else "end of file"
    return f"keyword {tok.text!r}" if tok.kind in KEYWORDS else repr(tok.text)


# ---------------------------------------------------------------- parser

class Parser:
    """Recursive descent over the token list, one method per 5.1 production."""

    def __init__(self, tokens: List[Token], filename: str) -> None:
        self.tokens, self.pos, self.filename = tokens, 0, filename

    # -- token helpers
    @property
    def tok(self) -> Token:
        return self.tokens[self.pos]

    def peek(self) -> Token:
        return self.tokens[min(self.pos + 1, len(self.tokens) - 1)]

    def at(self, *kinds: str) -> bool:
        return self.tok.kind in kinds

    def advance(self) -> Token:
        tok = self.tok
        if tok.kind != "EOF":
            self.pos += 1
        return tok

    def accept(self, kind: str) -> Optional[Token]:
        return self.advance() if self.at(kind) else None

    def fail(self, expected: str, tok: Optional[Token] = None) -> ParseError:
        tok = tok or self.tok
        return ParseError(f"expected {expected}, found {describe(tok)}",
                          self.filename, tok.line, tok.column)

    def expect(self, kind: str, expected: Optional[str] = None) -> Token:
        if not self.at(kind):
            names = {"NAME": "a name", "NUMBER": "a number", "NL": "end of line"}
            raise self.fail(expected or names.get(kind, repr(kind)))
        return self.advance()

    def name(self, expected: str = "a name") -> str:
        return self.expect("NAME", expected).text

    def integer(self, expected: str = "an integer") -> int:
        tok = self.expect("NUMBER", expected)
        if not isinstance(tok.value, int):
            raise self.fail(expected, tok)
        return tok.value

    def one_of(self, kinds: Tuple[str, ...]) -> str:
        if not self.at(*kinds):
            raise self.fail(" or ".join(repr(k) for k in kinds))
        return self.advance().kind

    def seq(self, item, sep: str, minimum: int = 1) -> list:
        """`item { sep item }` with at least `minimum` items."""
        items = [item()]
        while len(items) < minimum or self.at(sep):
            self.expect(sep)
            items.append(item())
        return items

    def opt(self, keyword: str, item):
        """`[ keyword item ]`: the item, or None."""
        return item() if self.accept(keyword) else None

    def skip_nl(self) -> None:
        while self.accept("NL"):
            pass

    def line_end(self, expected: str = "end of line") -> None:
        self.expect("NL", expected)
        self.skip_nl()

    # -- file and protocol
    def file(self) -> File:
        self.skip_nl()
        protocol: Optional[Protocol] = None
        crcs: List[CrcDef] = []
        while True:
            if self.at("crc"):
                crcs.append(self.crc_def())
            elif self.at("protocol"):
                if protocol is not None:
                    raise ParseError(f"a file holds at most one protocol (the first is at "
                                     f"line {protocol.line})", self.filename, self.tok.line,
                                     self.tok.column)
                protocol = self.protocol()
            else:
                raise self.fail("'crc' or 'protocol'")
            if self.at("EOF"):
                break
            self.line_end()
            if self.at("EOF"):
                break
        return File(protocol, tuple(crcs), self.filename, line=1)

    def protocol(self) -> Protocol:
        line = self.expect("protocol").line
        name = self.name()
        self.expect("{")
        self.line_end("end of line after '{'")
        items: List[Item] = []
        while not self.at("}"):
            items.append(self.item())
            self.line_end()
        self.advance()
        return Protocol(name, tuple(items), line=line)

    def item(self) -> Item:
        handlers = {"param": self.param, "parties": self.parties, "wire": self.wire,
                    "clock": self.clock, "event": self.event, "crc": self.crc_def,
                    "stuff": self.stuff, "frame": self.frame, "rule": self.rule}
        if self.tok.kind not in handlers:
            raise self.fail("a protocol item (param, parties, wire, clock, event, crc, "
                            "stuff, frame or rule) or '}'")
        return handlers[self.tok.kind]()

    def param(self) -> Param:
        line = self.expect("param").line
        name = self.name()
        ptype: Optional[Union[EnumType, RangeType]] = None
        if self.accept(":"):
            tline = self.tok.line
            if self.at("NAME"):
                ptype = EnumType(tuple(self.seq(self.name, "|", 2)), line=tline)
            elif self.at("NUMBER"):
                lo = self.integer()
                self.expect("..")
                ptype = RangeType(lo, self.integer(), line=tline)
            else:
                raise self.fail("a type (NAME | NAME ... or INT .. INT)")
        self.expect("=")
        return Param(name, ptype, self.expr(), line=line)

    def parties(self) -> Parties:
        line = self.expect("parties").line
        names = self.seq(lambda: self.name("a party name"), ",", 2)
        return Parties(tuple(names), line=line)

    def wire(self) -> Wire:
        line = self.expect("wire").line
        name = self.name()
        diff = None
        if self.accept("diff"):
            self.expect("(")
            p = self.name()
            self.expect(",")
            diff = (p, self.name())
            self.expect(")")
        attrs: dict = {}
        while self.at("by", "idle", "open_drain", "coding"):
            tok = self.advance()
            if tok.kind in attrs:
                raise ParseError(f"duplicate wire attribute {tok.kind!r}", self.filename,
                                 tok.line, tok.column)
            attrs[tok.kind] = (self.name() if tok.kind == "by" else
                               self.expr() if tok.kind == "idle" else
                               True if tok.kind == "open_drain" else
                               self.one_of(("nrzi", "manchester")))
        return Wire(name, diff, attrs.get("by"), attrs.get("idle"),
                    attrs.get("open_drain", False), attrs.get("coding"), line=line)

    def clock(self) -> Union[RateClock, WireClock, SymbolClock]:
        line = self.expect("clock").line
        if self.accept("rate"):
            rate = self.expr()
            return RateClock(rate, self.opt("tolerance", self.expr), line=line)
        if self.accept("symbols"):
            tolerance = self.opt("tolerance", self.expr)
            self.expect("{")
            self.line_end("end of line after '{'")
            symbols = [self.symbol()]
            self.line_end()
            while not self.at("}"):
                symbols.append(self.symbol())
                self.line_end()
            self.advance()
            return SymbolClock(tolerance, tuple(symbols), line=line)
        if not self.at("NAME"):
            raise self.fail("'rate', 'symbols' or a clock wire name")
        wire = self.name()
        self.expect("by")
        by = self.name("a party name")
        self.expect("rate")
        rate = self.expr()
        self.expect("capture")
        capture = self.expr()
        stretch = None
        if self.accept("stretch"):
            self.expect("by")
            stretch = self.name("a party name")
        return WireClock(wire, by, rate, capture, stretch, line=line)

    def symbol(self) -> Symbol:
        line = self.tok.line
        value = self.integer("an integer symbol value")
        self.expect("=")
        self.expect("high")
        high = self.expr()
        self.expect(",")
        self.expect("low")
        return Symbol(value, high, self.expr(), line=line)

    def event(self) -> Event:
        line = self.expect("event").line
        name = self.name()
        self.expect("=")
        cline = self.tok.line
        wire = self.name("a wire name")
        cond: Union[EdgeCondition, LevelCondition, BothLowCondition]
        if self.accept(","):
            other = self.name("a wire name")
            self.expect("both")
            self.expect("low")
            self.expect("for")
            cond = BothLowCondition(wire, other, self.duration(), line=cline)
        elif self.at(*EDGES):
            edge = self.advance().kind
            ww = wl = None
            if self.accept("while"):
                ww, wl = self.name("a wire name"), self.one_of(LEVELS)
            cond = EdgeCondition(wire, edge, ww, wl, line=cline)
        elif self.at(*LEVELS):
            level = self.advance().kind
            self.expect("for")
            cond = LevelCondition(wire, level, self.duration(), line=cline)
        else:
            raise self.fail("'rises', 'falls', 'changes', 'high', 'low' or ','")
        self.expect("by")
        return Event(name, cond, self.name("a party name"), line=line)

    def duration(self) -> Duration:
        lo = self.expr()
        return Range(lo, self.expr(), line=lo.line) if self.accept("..") else lo

    def crc_def(self) -> CrcDef:
        line = self.expect("crc").line
        name = self.name()
        self.expect("=")
        values = []
        for word in ("width", "poly", "init", "xorout"):
            self.expect(word)
            values.append(self.integer())
        return CrcDef(name, *values, reflect=bool(self.accept("reflect")), line=line)

    def stuff(self) -> Stuff:
        line = self.expect("stuff").line
        kind = self.one_of(("zero", "complement"))
        self.expect("after")
        run = self.integer()
        self.expect("ones" if kind == "zero" else "equal")
        return Stuff(kind, run, self.opt("over", self.name_range), line=line)

    def name_range(self) -> Range:
        tok = self.expect("NAME", "a field name")
        self.expect("..")
        hi = self.expect("NAME", "a field name")
        return Range(Name(tok.text, line=tok.line), Name(hi.text, line=hi.line), line=tok.line)

    # -- frames and statements
    def frame(self) -> Frame:
        line = self.expect("frame").line
        name = self.name()
        after = self.opt("after", self.event_name)
        until = self.opt("until", lambda: self.seq(self.event_name, "or")) or []
        return Frame(name, after, tuple(until), self.block(), line=line)

    def event_name(self) -> str:
        return self.name("an event name")

    def block(self) -> Tuple[Stmt, ...]:
        self.expect("{")
        self.skip_nl()
        stmts: List[Stmt] = []
        if not self.at("}"):
            stmts.append(self.stmt())
            while not self.at("}"):
                self.line_end("end of line or '}'")
                if self.at("}"):
                    break
                stmts.append(self.stmt())
        self.advance()
        return tuple(stmts)

    def stmt(self) -> Stmt:
        line = self.tok.line
        if self.at("NAME"):
            return self.field()
        if self.accept("together"):
            return Together(self.block(), line=line)
        if self.accept("repeat"):
            count = None if self.at("{") else self.expr()
            body = self.block()
            return Repeat(count, body, self.opt("while", self.expr), line=line)
        if self.at("if"):
            return self.if_stmt()
        raise self.fail("a field, 'together', 'repeat', 'if' or '}'")

    def if_stmt(self) -> If:
        line = self.expect("if").line
        cond = self.expr()
        body = self.block()
        orelse: Optional[Union[If, Tuple[Stmt, ...]]] = None
        if self.accept("else"):
            orelse = self.if_stmt() if self.at("if") else self.block()
        return If(cond, body, orelse, line=line)

    def field(self) -> Field:
        line = self.tok.line
        owner = self.name()
        name = self.name("a field name")
        self.expect("[")
        width = self.expr()
        self.expect("]")
        order = self.advance().kind if self.at("msb", "lsb") else None
        wire = self.opt("on", lambda: self.name("a wire name"))
        names = self.opt("(", lambda: self.seq(self.named_value, ",")) or []
        if names:
            self.expect(")", "',' or ')'")
        value = when = default = None
        if self.accept("="):
            value = self.expr()
            when = self.opt("when", self.expr)
        elif self.accept("default"):
            default = self.expr()
        on_error = self.opt("else", lambda: self.one_of(("end", "continue")))
        return Field(owner, name, width, order, wire, tuple(names), value, when, default,
                     on_error, line=line)

    def named_value(self) -> NamedValue:
        tok = self.expect("NAME", "a value name")
        self.expect("=")
        return NamedValue(tok.text, self.expr(), line=tok.line)

    # -- rules
    def rule(self) -> Rule:
        line = self.expect("rule").line
        name = self.name()
        self.expect(":")
        body = self.rule_body()
        if self.at(">=", "<="):
            return Rule(name, body, self.advance().kind, self.expr(), line=line)
        if not self.accept("in"):
            raise self.fail("'>=', '<=' or 'in'")
        lo = self.expr()
        self.expect("..")
        return Rule(name, body, "in", Range(lo, self.expr(), line=lo.line), line=line)

    def rule_body(self) -> Union[PulseWidth, Setup, Hold, Interval]:
        line = self.tok.line
        if self.at(*LEVELS):
            level = self.advance().kind
            return PulseWidth(level, self.name("a wire name"), line=line)
        if self.at("NAME") and self.peek().kind in ("setup", "hold"):
            wire = self.name()
            if self.advance().kind == "setup":
                self.expect("to")
                return Setup(wire, self.edge_ref(), line=line)
            self.expect("from")
            return Hold(wire, self.edge_ref(), line=line)
        start = self.edge_ref()
        self.expect("to")
        return Interval(start, self.edge_ref(), line=line)

    def edge_ref(self) -> EdgeRef:
        tok = self.tok
        if self.at("capture", "launch"):
            return ClockEdge(self.advance().kind, line=tok.line)
        name = self.name("an edge (WIRE rises|falls|changes, an event name, "
                         "'capture' or 'launch')")
        if self.at(*EDGES):
            return WireEdge(name, self.advance().kind, line=tok.line)
        return EventRef(name, line=tok.line)

    # -- expressions, loosest binding first
    def expr(self) -> Expr:
        cond = self.or_expr()
        if not self.accept("?"):
            return cond
        then = self.expr()
        self.expect(":")
        return Conditional(cond, then, self.expr(), line=cond.line)

    def binary_chain(self, ops: Tuple[str, ...], operand, repeat: bool = True) -> Expr:
        left = operand()
        while self.at(*ops):
            op = self.advance().kind
            left = Binary(op, left, operand(), line=left.line)
            if not repeat:
                break
        return left

    def or_expr(self) -> Expr:
        return self.binary_chain(("or",), self.and_expr)

    def and_expr(self) -> Expr:
        return self.binary_chain(("and",), self.not_expr)

    def not_expr(self) -> Expr:
        tok = self.accept("not")
        return Unary("not", self.not_expr(), line=tok.line) if tok else self.cmp_expr()

    def cmp_expr(self) -> Expr:
        return self.binary_chain(CMP_OPS, self.sum, repeat=False)   # non-associative

    def sum(self) -> Expr:
        return self.binary_chain(("+", "-"), self.term)

    def term(self) -> Expr:
        return self.binary_chain(("*", "/"), self.unary)

    def unary(self) -> Expr:
        if self.at("~", "-"):
            tok = self.advance()
            return Unary(tok.kind, self.unary(), line=tok.line)
        return self.atom()

    def atom(self) -> Expr:
        tok = self.tok
        if self.accept("NUMBER"):
            unit = None
            if self.at("%") or (self.at("NAME") and self.tok.text in UNITS):
                unit = self.advance().text
            return Number(tok.value, unit, line=tok.line)
        if self.accept("NAME"):
            if not self.accept("("):
                return Name(tok.text, line=tok.line)
            args = [] if self.at(")") else self.seq(self.arg, ",")
            self.expect(")", "',' or ')'")
            return Call(tok.text, tuple(args), line=tok.line)
        if tok.kind in VALUE_WORDS:
            return ValueWord(self.advance().kind, line=tok.line)
        if self.accept("("):
            inner = self.expr()
            self.expect(")")
            return inner
        raise self.fail("an expression")

    def arg(self) -> Union[Expr, Range]:
        if self.at("NAME") and self.peek().kind == "..":
            return self.name_range()
        return self.expr()


def parse(text: str, filename: str = "<string>") -> File:
    """Parse a description; raise ParseError on the first error."""
    return Parser(tokenize(text, filename), filename).file()


def parse_file(path) -> File:
    with open(path, encoding="utf-8") as f:
        return parse(f.read(), str(path))
