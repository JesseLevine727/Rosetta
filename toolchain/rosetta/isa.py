"""Rosetta instruction set v0.2 as data (docs/isa.md sections 3 and 4).

Everything about bit positions lives in the tables below: FORMATS (one per
opcode) and MODE_REGS (one per mode register). encode(), decode(),
mode_encode(), mode_decode() and the disassembler are derived from them.

An instruction is an `Instr(op, fields)`: the mnemonic of its opcode class
("FIELD", "ALU", ...) and a dict of integer field values. Enumerated fields
hold their numeric code; the name tables (OPERANDS, ALU_OPS, ...) give the
names. Round trip: decode(encode(x)) == x for every well-formed x.
"""
from __future__ import annotations

from dataclasses import dataclass, field

WORD_BITS = 32
PC_BITS = 9
PROGRAM_WORDS = 1 << PC_BITS          # 512, shared by all lanes
DATA_WORDS_PER_LANE = 128
N_REGS = 4
REG_BITS = 16
N_CHANNELS = 4
N_EVENTS = 4

# --------------------------------------------------------------------------
# Name tables (isa.md 3.0, 3.1, 3.5-3.7)
# --------------------------------------------------------------------------

OPCODES: dict[str, int] = {
    "FIELD": 0, "RAW": 1, "PIN": 2, "SETM": 3, "LDI": 4, "ALU": 5, "MEM": 6,
    "BR": 7, "LOOP": 8, "WAIT": 9, "EVT": 10, "Q": 11, "MISC": 12,
}
RESERVED_OPCODES = (13, 14, 15)       # isa 3.7: "Opcodes 13-15 halt the lane"

OPERANDS: dict[str, int] = {
    "R0": 0, "R1": 1, "R2": 2, "R3": 3, "HOSTQ": 4, "CAP": 5, "CRC": 6,
    "MEM": 7, "C0": 8, "C1": 9, "C2": 10, "C3": 11, "IMM": 12, "SR": 13,
    "NONE": 15,
}                                     # code 14 is not listed in isa.md
SOURCE_OPERANDS = frozenset(OPERANDS.values()) - {OPERANDS["CAP"]}
DEST_OPERANDS = frozenset(OPERANDS.values()) - {OPERANDS[n] for n in ("CRC", "IMM", "SR")}

FIELD_KINDS: dict[str, int] = {"DRIVE": 0, "SAMPLE": 1, "EXCHANGE": 2, "EXPECT": 3}
RAW_DIRS: dict[str, int] = {"SAMPLE": 0, "DRIVE": 1}

ALU_OPS: dict[str, int] = {
    "MOV": 0, "ADD": 1, "SUB": 2, "CMP": 3, "AND": 4, "OR": 5, "XOR": 6,
    "SHL": 7, "SHR": 8, "INC": 9, "DEC": 10, "ADDC": 11, "MOVSR": 12, "MOVSRH": 13,
}
ALU_NO_SOURCE = frozenset({"INC", "DEC", "MOVSR", "MOVSRH"})

BR_CONDS: dict[str, int] = {
    "always": 0, "Z": 1, "NZ": 2, "N": 3, "NN": 4, "M": 5, "NM": 6,
    "QE": 7, "QNE": 8, "QF": 9, "QNF": 10,
    "E0": 11, "E1": 12, "E2": 13, "E3": 14,
    "C0": 15, "C1": 16, "C2": 17, "C3": 18,
}

Q_OPS: dict[str, int] = {
    "POP": 0, "PUSH": 1, "PUSHTS": 2, "PUSHCRC": 3, "PUT": 4, "GET": 5,
    "PUTSR": 6, "PUSHI": 7,
}
MISC_OPS: dict[str, int] = {
    "NOP": 0, "HALT": 1, "IRQ": 2, "CRCINIT": 3, "CRCCHK": 4, "STUFFCLR": 5,
}

REGS = {"R0": 0, "R1": 1, "R2": 2, "R3": 3}
CHANNELS = {"C0": 0, "C1": 1, "C2": 2, "C3": 3}
INPUTS = {"in0": 0, "in1": 1, "in2": 2, "in3": 3}


def name_of(table: dict[str, int], code: int) -> str | None:
    for k, v in table.items():
        if v == code:
            return k
    return None


# --------------------------------------------------------------------------
# Bit fields and formats
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class BitField:
    """Bits [hi:lo] of a word. The stored value is `value - bias`.

    `when=(other, code)` makes the field present only if field `other`
    equals `code`; `unless=(other, code)` only if it differs. This is how
    the unions of isa.md are written (FIELD [8:0]: immediate or seek+tag)."""
    name: str
    hi: int
    lo: int
    enum: dict[str, int] | None = None
    bias: int = 0
    when: tuple[str, int] | None = None
    unless: tuple[str, int] | None = None

    @property
    def width(self) -> int:
        return self.hi - self.lo + 1

    @property
    def mask(self) -> int:
        return ((1 << self.width) - 1) << self.lo

    @property
    def min(self) -> int:
        return self.bias

    @property
    def max(self) -> int:
        return (1 << self.width) - 1 + self.bias

    def present(self, values: dict[str, int]) -> bool:
        if self.when is not None and values.get(self.when[0]) != self.when[1]:
            return False
        if self.unless is not None and values.get(self.unless[0]) == self.unless[1]:
            return False
        return True

    def extract(self, word: int) -> int:
        return ((word >> self.lo) & ((1 << self.width) - 1)) + self.bias

    def insert(self, value: int) -> int:
        if not self.min <= value <= self.max:
            raise EncodeError(f"field {self.name}={value} outside {self.min}..{self.max}")
        return (value - self.bias) << self.lo


@dataclass(frozen=True)
class Format:
    op: str
    fields: tuple[BitField, ...]

    @property
    def opcode(self) -> int:
        return OPCODES[self.op]

    def present_fields(self, values: dict[str, int]) -> list[BitField]:
        return [f for f in self.fields if f.present(values)]


class EncodeError(ValueError):
    pass


IMM = OPERANDS["IMM"]
B = BitField

FORMATS: dict[str, Format] = {f.op: f for f in [
    Format("FIELD", (                                     # isa 3.1
        B("kind", 27, 26, FIELD_KINDS),
        B("width", 25, 21, bias=1),
        B("lsb", 20, 20),
        B("crc", 19, 19),
        B("stuff", 18, 18),
        B("src", 17, 14, OPERANDS),
        B("dst", 13, 10, OPERANDS),
        B("early", 9, 9),
        B("imm", 8, 0, when=("src", IMM)),
        B("seek", 8, 8, unless=("src", IMM)),
        B("tag", 7, 0, unless=("src", IMM)),
    )),
    Format("RAW", (                                       # isa 3.2
        B("dir", 27, 27, RAW_DIRS),
        B("level", 26, 26),
        B("src", 17, 14, OPERANDS),
        B("dst", 13, 10, OPERANDS),
        B("imm", 8, 0, when=("src", IMM)),                # MODEL_DECISIONS D40
        B("tag", 7, 0, unless=("src", IMM)),
    )),
    Format("PIN", (                                       # isa 3.3
        B("mask", 27, 24),
        B("values", 23, 20),
        B("enables", 19, 16),
        B("align", 15, 15),
        B("wait", 11, 0),
    )),
    Format("SETM", (                                      # isa 3.4
        B("reg", 27, 23),
        B("upper", 22, 22),
        B("from_reg", 21, 21),
        B("imm", 15, 0, when=("from_reg", 0)),
        B("rs", 1, 0, when=("from_reg", 1)),
    )),
    Format("LDI", (B("rd", 27, 26), B("imm", 15, 0))),    # isa 3.5
    Format("ALU", (
        B("op", 27, 24, ALU_OPS),
        B("rd", 23, 22),
        B("rs", 21, 20),
        B("use_imm", 19, 19),
        B("imm", 15, 0),
    )),
    Format("MEM", (
        B("store", 27, 27),
        B("rd", 26, 25),
        B("hi", 24, 24),
        B("ra", 23, 22),
        B("postinc", 21, 21),
        B("absolute", 20, 20),
        B("addr", 6, 0),
    )),
    Format("BR", (B("cond", 27, 23, BR_CONDS), B("target", 8, 0))),     # isa 3.6
    Format("LOOP", (B("rd", 27, 26), B("target", 8, 0))),
    Format("WAIT", (B("mask", 27, 24), B("timeout", 11, 0))),
    Format("EVT", (B("k", 27, 26), B("enable", 25, 25), B("clear", 24, 24), B("target", 8, 0))),
    Format("Q", (                                         # isa 3.7
        B("op", 27, 24, Q_OPS),
        B("reg", 23, 22),
        B("ch", 21, 20),
        B("imm8", 15, 8),
        B("tag", 7, 0),
    )),
    Format("MISC", (B("op", 27, 24, MISC_OPS),)),
]}


@dataclass
class Instr:
    op: str                                   # key of FORMATS, or "RESERVED"
    fields: dict[str, int] = field(default_factory=dict)

    def __getitem__(self, name: str) -> int:
        return self.fields[name]

    def get(self, name: str, default: int = 0) -> int:
        return self.fields.get(name, default)


def make(op: str, /, **values: int) -> Instr:
    """Build an Instr, filling every present field not given with 0
    (the minimum value for biased fields)."""
    fmt = FORMATS[op]
    full: dict[str, int] = {}
    for f in fmt.fields:
        if f.name in values:
            full[f.name] = values[f.name]
        elif f.present({**full, **values}):
            full[f.name] = f.min
    # drop fields whose presence condition fails (e.g. imm when src != IMM)
    full = {f.name: full[f.name] for f in fmt.present_fields(full) if f.name in full}
    extra = set(values) - set(full)
    if extra:
        raise EncodeError(f"{op}: fields {sorted(extra)} not present in this form")
    return Instr(op, full)


def encode(instr: Instr) -> int:
    if instr.op == "RESERVED":
        return (instr["opcode"] << 28) | instr.get("payload")
    fmt = FORMATS[instr.op]
    word = fmt.opcode << 28
    present = fmt.present_fields(instr.fields)
    names = {f.name for f in present}
    if set(instr.fields) != names:
        raise EncodeError(f"{instr.op}: fields {sorted(instr.fields)} != {sorted(names)}")
    for f in present:
        word |= f.insert(instr.fields[f.name])
    return word


def decode(word: int) -> Instr:
    if not 0 <= word < 1 << WORD_BITS:
        raise ValueError(f"not a 32-bit word: {word!r}")
    opcode = word >> 28
    if opcode in RESERVED_OPCODES:
        return Instr("RESERVED", {"opcode": opcode, "payload": word & 0x0FFF_FFFF})
    fmt = FORMATS[name_of(OPCODES, opcode)]   # type: ignore[index]
    values: dict[str, int] = {}
    for f in fmt.fields:                      # presence depends on earlier fields
        if f.present(values):
            values[f.name] = f.extract(word)
    return Instr(fmt.op, values)


def defined_mask(word: int) -> int:
    """Bits of `word` that belong to a field of its format (plus the opcode)."""
    ins = decode(word)
    if ins.op == "RESERVED":
        return 0xFFFF_FFFF
    m = 0xF000_0000
    for f in FORMATS[ins.op].present_fields(ins.fields):
        m |= f.mask
    return m


# --------------------------------------------------------------------------
# Mode registers (isa.md section 4)
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class ModeReg:
    index: int
    name: str
    bits: int
    fields: tuple[BitField, ...]

    @property
    def defined_mask(self) -> int:
        m = 0
        for f in self.fields:
            m |= f.mask
        return m

    def field(self, name: str) -> BitField:
        for f in self.fields:
            if f.name == name:
                return f
        raise KeyError(f"{self.name} has no field {name!r}")


TICK_MODES = {"BAUD": 0, "EDGE": 1, "RECOVER": 2, "PULSE": 3}
PRESCALE_CODES = {"1": 0, "8": 1, "64": 2, "512": 3}
PRESCALE_FACTORS = (1, 8, 64, 512)
RX_CODINGS = {"LEVEL": 0, "NRZI": 1, "MANCHESTER": 2, "SYMBOL": 3}
TX_CODINGS = {"LEVEL": 0, "NRZI": 1, "MANCHESTER": 2, "PULSE": 3}
EDGES = {"RISING": 0, "FALLING": 1}
LEVELS = {"LOW": 0, "HIGH": 1}
STUFF_MODES = {"ONES": 0, "EQUAL": 1}
# Ruling 10 (docs/decisions.md): 3-bit trigger, 0 = off (reset value), K is 6 bits.
# Ruling 21 / lane.md 5.1: trigger codes 5-7 are off as well (they have no
# name: the assembler writes OFF as 0, the disassembler prints such a word as
# a number), and B-must code 3 means "never".
EV_TRIGGERS = {"OFF": 0, "LEVEL": 1, "EDGE": 2, "ANY": 3, "BOTHLOW": 4}
EV_TRIGGER_OFF_CODES = (0, 5, 6, 7)
EV_B_MUST = {"ANY": 0, "HIGH": 1, "LOW": 2, "NEVER": 3}


def _evc(index: int) -> ModeReg:
    return ModeReg(index, f"EVC{index - 10}", 16, (
        B("a", 1, 0, INPUTS),
        B("level", 2, 2, LEVELS),
        B("trigger", 5, 3, EV_TRIGGERS),
        B("b", 7, 6, INPUTS),
        B("b_must", 9, 8, EV_B_MUST),
        B("k", 15, 10),
    ))


MODE_REGS: list[ModeReg] = [
    ModeReg(0, "TICK", 16, (
        B("mode", 1, 0, TICK_MODES),
        B("prescale", 3, 2, PRESCALE_CODES),
        B("clock_in", 5, 4, INPUTS),
        B("capture", 6, 6, EDGES),            # 1 = capture on falling edge
        B("clock_out", 7, 7),
        B("clock_idle", 8, 8),
        B("clock_od", 9, 9),
        B("rest", 10, 10),
        B("stretch_honour", 11, 11),
        B("stretch_hold", 12, 12),
        # Bit 13 is reserved (not stored, reads 0). Ruling 59, amendment-eth-can.md:
        B("rone", 14, 14),                    # RECOVER: only edges to level !capture restart the timer
        B("rsp2", 15, 15),                    # RECOVER: capture PERIOD2 after the launch tick
    )),
    ModeReg(1, "PERIOD", 16, (B("value", 15, 0),)),
    ModeReg(2, "PERIOD2", 16, (B("value", 15, 0),)),
    # Ruling 23: only THR0 exists; THR01[15:8] and THR2L[7:0] are not stored.
    ModeReg(3, "THR01", 16, (B("thr0", 7, 0),)),
    ModeReg(4, "THR2L", 16, (B("delay", 15, 8),)),
    ModeReg(5, "CODE", 16, (
        B("rx", 1, 0, RX_CODINGS),
        B("tx", 3, 2, TX_CODINGS),
        B("data_in", 5, 4, INPUTS),
        B("invert", 6, 6),
        B("data_od", 7, 7),
        B("diff_out", 8, 8),
        B("diff_in", 9, 9),
        B("delay_clock", 10, 10),
        B("delay_prescaled", 11, 11),
        B("rawedge", 12, 12),                 # a RAW DRIVE also ends at the first data edge (ruling 59)
    )),
    ModeReg(6, "PULSE", 16, (B("high0", 7, 0), B("high1", 15, 8))),
    ModeReg(7, "STUFF", 16, (
        B("n", 2, 0),
        B("mode", 3, 3, STUFF_MODES),
        B("enable", 4, 4),
        B("violation_event", 5, 5),
    )),
    ModeReg(8, "CRCPOLY", 32, (B("value", 31, 0),)),
    ModeReg(9, "CRCCFG", 16, (
        B("width", 4, 0, bias=1),
        B("init_ones", 5, 5),
        B("xorout_ones", 6, 6),
    )),
    _evc(10), _evc(11), _evc(12), _evc(13),
]
MODE_REG_BY_NAME: dict[str, ModeReg] = {r.name: r for r in MODE_REGS}
MODE_REG_NAMES: dict[str, int] = {r.name: r.index for r in MODE_REGS}


def mode_encode(reg: str | int, **values: int) -> int:
    """Full register value from field values (missing fields are 0/min)."""
    r = MODE_REGS[reg] if isinstance(reg, int) else MODE_REG_BY_NAME[reg]
    unknown = set(values) - {f.name for f in r.fields}
    if unknown:
        raise EncodeError(f"{r.name} has no fields {sorted(unknown)}")
    word = 0
    for f in r.fields:
        word |= f.insert(values.get(f.name, f.min))
    return word


def mode_decode(reg: str | int, value: int) -> dict[str, int]:
    r = MODE_REGS[reg] if isinstance(reg, int) else MODE_REG_BY_NAME[reg]
    return {f.name: f.extract(value) for f in r.fields}


# --------------------------------------------------------------------------
# Disassembler (prints the syntax accepted by rosetta.asm, see README.md)
# --------------------------------------------------------------------------

def _num(v: int) -> str:
    return str(v) if v < 10 else hex(v)


def _operand(code: int, imm: int | None = None) -> str:
    if code == IMM:
        return f"IMM {_num(imm or 0)}"
    n = name_of(OPERANDS, code)
    return n if n is not None else f"OP{code}"


def _mode_text(reg_index: int, upper: int, imm: int) -> str | None:
    """`field=value, ...` text for a SETM immediate, or None if the value
    has bits that no field of that half describes."""
    if reg_index >= len(MODE_REGS):
        return None
    r = MODE_REGS[reg_index]
    full = imm << 16 if upper else imm
    half_mask = 0xFFFF_0000 if upper else 0x0000_FFFF
    if full & ~r.defined_mask & half_mask:
        return None
    fields = [f for f in r.fields if f.mask & half_mask]
    if len(fields) == 1 and fields[0].enum is None:
        return _num(imm)
    if any(f.mask & ~half_mask for f in fields):     # field straddles halves
        return None
    parts = []
    for f in fields:
        v = f.extract(full)
        if v == f.min and f.enum is None:
            continue
        if f.enum is not None:
            if v == f.min and name_of(f.enum, v) is not None:
                continue
            name = name_of(f.enum, v)
            if name is None:
                return None
            parts.append(f"{f.name}={name}")
        elif f.width == 1:
            parts.append(f.name)
        else:
            parts.append(f"{f.name}={_num(v)}")
    return ", ".join(parts) if parts else "0"


def _target(v: int, labels: dict[int, str] | None) -> str:
    if labels and v in labels:
        return labels[v]
    return str(v)


def format_instr(ins: Instr, labels: dict[int, str] | None = None) -> str | None:
    """Assembler text for `ins`, or None if the syntax cannot express it
    (an undefined sub-code, or a non-zero bit in an unused field)."""
    f = ins.fields
    op = ins.op
    text: str
    used: dict[str, int]                       # fields the text determines
    if op == "FIELD":
        kind = name_of(FIELD_KINDS, f["kind"])
        flags = [n for n in ("lsb", "crc", "stuff", "early", "seek") if f.get(n)]
        src = _operand(f["src"], f.get("imm"))
        dst = _operand(f["dst"])
        if name_of(OPERANDS, f["src"]) is None or name_of(OPERANDS, f["dst"]) is None:
            return None
        if kind == "SAMPLE":
            ops = [dst] + ([f"src={src}"] if f["src"] != OPERANDS["NONE"] else [])
        elif kind == "EXCHANGE":
            ops = [src, dst]
        else:
            ops = [src] + ([dst] if f["dst"] != OPERANDS["NONE"] else [])
        tag = [f"tag={_num(f['tag'])}"] if f.get("tag") else []
        text = f"FIELD {kind} {f['width']}, " + ", ".join(flags + ops + tag)
        return text
    if op == "RAW":
        if name_of(OPERANDS, f["src"]) is None or name_of(OPERANDS, f["dst"]) is None:
            return None
        tag = [f"tag={_num(f['tag'])}"] if f.get("tag") else []
        if f["dir"] == RAW_DIRS["SAMPLE"]:
            extra = []
            if f["level"]:
                extra.append("level=1")
            if f["src"] != OPERANDS["NONE"]:
                extra.append(f"src={_operand(f['src'], f.get('imm'))}")
            return "RAW SAMPLE " + ", ".join([_operand(f["dst"])] + extra + tag)
        extra = [f"dst={_operand(f['dst'])}"] if f["dst"] != OPERANDS["NONE"] else []
        return f"RAW DRIVE {f['level']}, " + ", ".join(
            [_operand(f["src"], f.get("imm"))] + extra + tag)
    if op == "PIN":
        parts = []
        for k in range(4):
            bit = 1 << k
            if f["mask"] & bit:
                if f["enables"] & bit:
                    parts.append(f"out{k}={1 if f['values'] & bit else 0}")
                elif f["values"] & bit:
                    return None
                else:
                    parts.append(f"out{k}=z")
            elif (f["values"] | f["enables"]) & bit:
                return None
        if f["align"]:
            parts.append("align")
        if f["wait"]:
            parts.append(f"wait={f['wait']}")
        return "PIN " + ", ".join(parts) if parts else "PIN"
    if op == "SETM":
        reg = MODE_REGS[f["reg"]].name if f["reg"] < len(MODE_REGS) else str(f["reg"])
        if f["upper"]:
            reg += ".hi"
        if f["from_reg"]:
            return f"SETM {reg}, R{f['rs']}"
        body = _mode_text(f["reg"], f["upper"], f["imm"])
        return f"SETM {reg}, {body if body is not None else _num(f['imm'])}"
    if op == "LDI":
        return f"LDI R{f['rd']}, {_num(f['imm'])}"
    if op == "ALU":
        name = name_of(ALU_OPS, f["op"])
        if name is None:
            return None
        if name in ALU_NO_SOURCE:
            if f["rs"] or f["use_imm"] or f["imm"]:
                return None
            return f"{name} R{f['rd']}"
        if f["use_imm"]:
            if f["rs"]:
                return None
            return f"{name} R{f['rd']}, {_num(f['imm'])}"
        if f["imm"]:
            return None
        return f"{name} R{f['rd']}, R{f['rs']}"
    if op == "MEM":
        mn = "ST" if f["store"] else "LD"
        if f["absolute"]:
            if f["ra"] or f["postinc"]:
                return None
            addr = f"[{f['addr']}]"
        else:
            if f["addr"]:
                return None
            addr = f"[R{f['ra']}{'+' if f['postinc'] else ''}]"
        return f"{mn} R{f['rd']}, {addr}" + (", hi" if f["hi"] else "")
    if op == "BR":
        cond = name_of(BR_CONDS, f["cond"])
        if cond is None:
            return None
        return f"BR {cond}, {_target(f['target'], labels)}"
    if op == "LOOP":
        return f"LOOP R{f['rd']}, {_target(f['target'], labels)}"
    if op == "WAIT":
        evs = "|".join(f"E{k}" for k in range(4) if f["mask"] >> k & 1)
        parts = [evs] if evs else []
        if f["timeout"]:
            parts.append(f"timeout={f['timeout']}")
        return "WAIT " + (", ".join(parts) if parts else "none")
    if op == "EVT":
        return (f"EVT {f['k']}, {_target(f['target'], labels)}, "
                f"{'on' if f['enable'] else 'off'}" + (", clear" if f["clear"] else ""))
    if op == "Q":
        name = name_of(Q_OPS, f["op"])
        if name is None:
            return None
        reg, ch, imm8, tag = f["reg"], f["ch"], f["imm8"], f["tag"]
        tagtxt = f", tag={_num(tag)}" if tag else ""
        if name == "POP" and not (ch or imm8 or tag):
            return f"Q POP R{reg}"
        if name == "PUSH" and not (ch or imm8):
            return f"Q PUSH R{reg}{tagtxt}"
        if name in ("PUSHTS", "PUSHCRC") and not (reg or ch or imm8):
            return f"Q {name}{tagtxt}"
        if name == "PUT" and not (imm8 or tag):
            return f"Q PUT C{ch}, R{reg}"
        if name == "GET" and not (imm8 or tag):
            return f"Q GET R{reg}, C{ch}"
        if name == "PUTSR" and not (reg or imm8 or tag):
            return f"Q PUTSR C{ch}"
        if name == "PUSHI" and not (reg or ch):
            return f"Q PUSHI {_num(imm8)}{tagtxt}"
        return None
    if op == "MISC":
        return name_of(MISC_OPS, f["op"])
    return None


def disassemble(word: int, labels: dict[int, str] | None = None) -> str:
    """One line of assembler text; `.word 0x...` when no mnemonic form
    reproduces the word exactly (reserved opcode, undefined code, stray bits)."""
    ins = decode(word)
    text = None if ins.op == "RESERVED" else format_instr(ins, labels)
    if text is None or (word & ~defined_mask(word)):
        return f".word 0x{word:08X}"
    return text


def disassemble_program(words: list[int], origin: int = 0) -> list[str]:
    return [f"{origin + i:03X}  {w:08X}  {disassemble(w)}" for i, w in enumerate(words)]
