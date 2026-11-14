"""Assembler for Rosetta lane programs. Syntax: toolchain/README.md.

    from rosetta.asm import assemble
    prog = assemble(text)
    prog.words      # list of 32-bit words, prog.words[0] is at `origin`
    prog.listing    # list of "addr  word  source" lines
    prog.labels     # label -> address

Every code and bit position comes from rosetta.isa; this module only knows
the text syntax.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field

from . import isa
from .isa import EncodeError, Instr, make


class AsmError(ValueError):
    def __init__(self, msg: str, line: int = 0, text: str = ""):
        super().__init__(f"line {line}: {msg}" + (f"\n    {text.strip()}" if text else ""))
        self.line = line


@dataclass
class Program:
    words: list[int]
    origin: int = 0
    labels: dict[str, int] = field(default_factory=dict)
    listing: list[str] = field(default_factory=list)
    lines: list[int] = field(default_factory=list)   # source line of each word


_INT = re.compile(r"^-?(0x[0-9a-fA-F_]+|0b[01_]+|[0-9][0-9_]*)$")
_LABEL = re.compile(r"^([A-Za-z_.][A-Za-z0-9_.]*):")
_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def _split_operands(text: str) -> list[str]:
    parts, depth, cur = [], 0, []
    for ch in text:
        if ch == "[":
            depth += 1
        elif ch == "]":
            depth -= 1
        if ch == "," and depth == 0:
            parts.append("".join(cur).strip())
            cur = []
        else:
            cur.append(ch)
    parts.append("".join(cur).strip())
    if parts == [""]:
        return []
    return parts


class _Ctx:
    """Per-line helper: symbol lookup and errors with the line attached."""

    def __init__(self, symbols: dict[str, int], line: int, text: str, final: bool):
        self.symbols, self.line, self.text, self.final = symbols, line, text, final

    def err(self, msg: str) -> AsmError:
        return AsmError(msg, self.line, self.text)

    def value(self, tok: str) -> int:
        tok = tok.strip()
        if _INT.match(tok):
            neg = tok.startswith("-")
            v = int(tok.lstrip("-").replace("_", ""), 0)
            return -v if neg else v
        if tok in self.symbols:
            return self.symbols[tok]
        if _NAME.match(tok) and not self.final:
            return 0                       # forward label, pass 1
        raise self.err(f"unknown value {tok!r}")

    def reg(self, tok: str) -> int:
        t = tok.strip().upper()
        if t not in isa.REGS:
            raise self.err(f"expected a register R0-R3, found {tok!r}")
        return isa.REGS[t]

    def chan(self, tok: str) -> int:
        t = tok.strip().upper()
        if t not in isa.CHANNELS:
            raise self.err(f"expected a channel C0-C3, found {tok!r}")
        return isa.CHANNELS[t]

    def operand(self, tok: str) -> tuple[int, int | None]:
        """(operand code, immediate or None)."""
        t = tok.strip()
        up = t.upper()
        if up.startswith("IMM"):
            rest = t[3:].strip()
            if not rest:
                raise self.err("IMM needs a value: IMM 5")
            return isa.OPERANDS["IMM"], self.value(rest)
        if up in isa.OPERANDS:
            return isa.OPERANDS[up], None
        if up.startswith("OP") and up[2:].isdigit():
            return int(up[2:]), None
        raise self.err(f"expected an operand (R0-R3 HOSTQ CAP CRC MEM C0-C3 IMM v SR NONE), found {tok!r}")


def _is_operand(tok: str) -> bool:
    up = tok.strip().upper()
    return up in isa.OPERANDS or up.startswith("IMM ") or up == "IMM"


def _kv(tok: str) -> tuple[str, str] | None:
    if "=" in tok:
        k, v = tok.split("=", 1)
        return k.strip(), v.strip()
    return None


# --------------------------------------------------------------------------
# Per-instruction parsers. Each returns an isa.Instr.
# --------------------------------------------------------------------------

def _field(c: _Ctx, ops: list[str]) -> Instr:
    if not ops:
        raise c.err("FIELD needs KIND WIDTH")
    head = ops[0].split()
    if len(head) != 2 or head[0].upper() not in isa.FIELD_KINDS:
        raise c.err("FIELD starts with KIND WIDTH, e.g. FIELD DRIVE 8")
    kind = head[0].upper()
    v: dict[str, int] = {"kind": isa.FIELD_KINDS[kind], "width": c.value(head[1])}
    positional: list[tuple[int, int | None]] = []
    src = dst = None
    for tok in ops[1:]:
        kv = _kv(tok)
        if tok in ("lsb", "crc", "stuff", "early", "seek"):   # flags are lower case;
            v[tok] = 1                                        # the operand is CRC
        elif tok == "msb":
            pass
        elif kv and kv[0].lower() == "tag":
            v["tag"] = c.value(kv[1])
        elif kv and kv[0].lower() == "src":
            src = c.operand(kv[1])
        elif kv and kv[0].lower() == "dst":
            dst = c.operand(kv[1])
        elif _is_operand(tok):
            positional.append(c.operand(tok))
        else:
            raise c.err(f"unexpected FIELD operand {tok!r}")
    none = (isa.OPERANDS["NONE"], None)
    if kind == "SAMPLE":
        want = ["dst"]
    elif kind == "EXCHANGE":
        want = ["src", "dst"]
    else:
        want = ["src", "dst"]
    slots = {"src": src, "dst": dst}
    names = [n for n in want if slots[n] is None]
    if len(positional) > len(names):
        raise c.err(f"too many operands for FIELD {kind}")
    for n, p in zip(names, positional):
        slots[n] = p
    if kind == "EXCHANGE" and (slots["src"] is None or slots["dst"] is None):
        raise c.err("FIELD EXCHANGE needs a source and a destination")
    if kind in ("DRIVE", "EXPECT") and slots["src"] is None:
        raise c.err(f"FIELD {kind} needs a source")
    if kind == "SAMPLE" and slots["dst"] is None:
        raise c.err("FIELD SAMPLE needs a destination")
    s, d = slots["src"] or none, slots["dst"] or none
    v["src"], v["dst"] = s[0], d[0]
    if s[1] is not None:
        if "tag" in v or "seek" in v:
            raise c.err("tag and seek share bits [8:0] with the immediate: not allowed with IMM")
        v["imm"] = s[1]
    return make("FIELD", **v)


def _raw(c: _Ctx, ops: list[str]) -> Instr:
    head = ops[0].split(None, 1) if ops else []
    if not head or head[0].upper() not in isa.RAW_DIRS:
        raise c.err("RAW SAMPLE dst  or  RAW DRIVE level, src")
    d = head[0].upper()
    v: dict[str, int] = {"dir": isa.RAW_DIRS[d]}
    rest = ([head[1]] if len(head) > 1 else []) + ops[1:]
    src = dst = None
    positional = []
    for tok in rest:
        kv = _kv(tok)
        if kv and kv[0].lower() == "tag":
            v["tag"] = c.value(kv[1])
        elif kv and kv[0].lower() == "level":
            v["level"] = c.value(kv[1])
        elif kv and kv[0].lower() == "src":
            src = c.operand(kv[1])
        elif kv and kv[0].lower() == "dst":
            dst = c.operand(kv[1])
        else:
            positional.append(tok)
    if d == "SAMPLE":
        if len(positional) != 1:
            raise c.err("RAW SAMPLE takes one destination")
        dst = c.operand(positional[0])
    else:
        if len(positional) != 2:
            raise c.err("RAW DRIVE takes a level and a source")
        v["level"] = c.value(positional[0])
        src = c.operand(positional[1])
    none = (isa.OPERANDS["NONE"], None)
    s, dd = src or none, dst or none
    v["src"], v["dst"] = s[0], dd[0]
    if s[1] is not None:
        if "tag" in v:
            raise c.err("tag shares bits with the immediate")
        v["imm"] = s[1]
    return make("RAW", **v)


def _pin(c: _Ctx, ops: list[str]) -> Instr:
    v = {"mask": 0, "values": 0, "enables": 0, "align": 0, "wait": 0}
    for tok in ops:
        low = tok.lower().replace(" ", "")
        kv = _kv(low)
        if low == "align":
            v["align"] = 1
        elif low == "enable":
            pass                            # isa.md example style; enable is the default
        elif kv and kv[0] == "wait":
            v["wait"] = c.value(kv[1])
        elif kv and re.fullmatch(r"out[0-3]", kv[0]):
            k = int(kv[0][3])
            v["mask"] |= 1 << k
            if kv[1] in ("0", "1"):
                v["enables"] |= 1 << k
                v["values"] |= int(kv[1]) << k
            elif kv[1] != "z":
                raise c.err(f"out{k} takes 0, 1 or z")
        else:
            raise c.err(f"unexpected PIN operand {tok!r}")
    return make("PIN", **v)


def _setm(c: _Ctx, ops: list[str]) -> Instr:
    if len(ops) < 2:
        raise c.err("SETM REG, value")
    name = ops[0].strip()
    upper = 0
    if name.lower().endswith(".hi"):
        name, upper = name[:-3], 1
    if name.upper() in isa.MODE_REG_BY_NAME:
        reg = isa.MODE_REG_BY_NAME[name.upper()]
        index = reg.index
    elif _INT.match(name):
        reg, index = None, c.value(name)
    else:
        raise c.err(f"unknown mode register {name!r}")
    body = ops[1:]
    if len(body) == 1 and body[0].upper() in isa.REGS:
        return make("SETM", reg=index, upper=upper, from_reg=1, rs=c.reg(body[0]))
    if len(body) == 1 and (_INT.match(body[0]) or body[0] in c.symbols):
        value = c.value(body[0])
        if not 0 <= value <= 0xFFFF:
            raise c.err(f"SETM carries 16 bits; write the upper half with {name}.hi")
        return make("SETM", reg=index, upper=upper, from_reg=0, imm=value)
    if reg is None:
        raise c.err("field names need a named register")
    fields: dict[str, int] = {}
    for tok in body:
        kv = _kv(tok)
        if kv:
            fname, sval = kv[0].lower(), kv[1]
            try:
                bf = reg.field(fname)
            except KeyError:
                raise c.err(f"{reg.name} has no field {fname!r}; fields: "
                            + ", ".join(f.name for f in reg.fields)) from None
        else:
            word = tok.strip()
            names = {f.name for f in reg.fields}
            if word.lower() in names:
                bf, sval = reg.field(word.lower()), "1"
            else:                       # a bare enum value: SETM TICK, BAUD
                hits = [f for f in reg.fields if f.enum and word.upper() in
                        {k.upper() for k in f.enum}]
                if len(hits) != 1:
                    raise c.err(f"{word!r} is not a field or value of {reg.name}")
                bf, sval = hits[0], word
        if bf.enum is not None:
            table = {k.upper(): val for k, val in bf.enum.items()}
            if sval.upper() not in table:
                raise c.err(f"{reg.name}.{bf.name} takes one of {', '.join(bf.enum)}")
            fields[bf.name] = table[sval.upper()]
        else:
            fields[bf.name] = c.value(sval)
    try:
        full = isa.mode_encode(reg.name, **fields)
    except EncodeError as e:
        raise c.err(str(e)) from None
    value = (full >> 16) if upper else (full & 0xFFFF)
    if (full >> 16 if not upper else full & 0xFFFF) != 0:
        raise c.err(f"these fields lie in the {'lower' if upper else 'upper'} half of {reg.name}")
    return make("SETM", reg=index, upper=upper, from_reg=0, imm=value)


def _alu(c: _Ctx, name: str, ops: list[str]) -> Instr:
    code = isa.ALU_OPS[name]
    if name in isa.ALU_NO_SOURCE:
        if len(ops) != 1:
            raise c.err(f"{name} takes one register")
        return make("ALU", op=code, rd=c.reg(ops[0]))
    if len(ops) != 2:
        raise c.err(f"{name} takes rd, rs-or-immediate")
    if ops[1].strip().upper() in isa.REGS:
        return make("ALU", op=code, rd=c.reg(ops[0]), rs=c.reg(ops[1]))
    imm = c.value(ops[1])
    if imm < 0:
        imm &= 0xFFFF
    return make("ALU", op=code, rd=c.reg(ops[0]), use_imm=1, imm=imm)


def _mem(c: _Ctx, store: int, ops: list[str]) -> Instr:
    if len(ops) not in (2, 3):
        raise c.err("LD/ST rd, [ra] | [ra+] | [addr] [, hi]")
    v = {"store": store, "rd": c.reg(ops[0])}
    m = re.fullmatch(r"\[\s*([^\]+]+?)\s*(\+?)\s*\]", ops[1].strip())
    if not m:
        raise c.err(f"bad address {ops[1]!r}")
    if m.group(1).upper() in isa.REGS:
        v["ra"] = c.reg(m.group(1))
        v["postinc"] = 1 if m.group(2) else 0
    else:
        if m.group(2):
            raise c.err("post-increment needs an address register")
        v["absolute"], v["addr"] = 1, c.value(m.group(1))
    if len(ops) == 3:
        if ops[2].strip().lower() != "hi":
            raise c.err("the third operand can only be hi")
        v["hi"] = 1
    return make("MEM", **v)


def _q(c: _Ctx, name: str, ops: list[str]) -> Instr:
    v: dict[str, int] = {"op": isa.Q_OPS[name]}
    plain = []
    for tok in ops:
        kv = _kv(tok)
        if kv and kv[0].lower() == "tag":
            v["tag"] = c.value(kv[1])
        else:
            plain.append(tok)
    shape = {"POP": "r", "PUSH": "r", "PUSHTS": "", "PUSHCRC": "", "PUT": "cr",
             "GET": "rc", "PUTSR": "c", "PUSHI": "i"}[name]
    if len(plain) != len(shape):
        raise c.err(f"Q {name} takes {len(shape)} operand(s)")
    for kind, tok in zip(shape, plain):
        if kind == "r":
            v["reg"] = c.reg(tok)
        elif kind == "c":
            v["ch"] = c.chan(tok)
        else:
            v["imm8"] = c.value(tok)
    if "tag" in v and name in ("POP", "PUT", "GET", "PUTSR"):
        raise c.err(f"Q {name} has no tag")
    return make("Q", **v)


def _parse(c: _Ctx, mnemonic: str, ops: list[str]) -> Instr:
    mn = mnemonic.upper()
    if mn in ("ALU", "Q") and ops:                     # "ALU CMP R0, 1", "Q POP R0"
        first = ops[0].split(None, 1)
        mnemonic, mn = first[0], first[0].upper()
        ops = ([first[1]] if len(first) > 1 else []) + ops[1:]
    if mn == "FIELD":
        return _field(c, ops)
    if mn == "RAW":
        return _raw(c, ops)
    if mn == "PIN":
        return _pin(c, ops)
    if mn == "SETM":
        return _setm(c, ops)
    if mn == "LDI":
        if len(ops) != 2:
            raise c.err("LDI rd, imm16")
        imm = c.value(ops[1])
        return make("LDI", rd=c.reg(ops[0]), imm=imm & 0xFFFF if imm < 0 else imm)
    if mn in isa.ALU_OPS:
        return _alu(c, mn, ops)
    if mn in ("LD", "ST"):
        return _mem(c, 1 if mn == "ST" else 0, ops)
    if mn == "BR":
        if len(ops) == 1:
            ops = ["always"] + ops
        if len(ops) != 2:
            raise c.err("BR cond, target")
        conds = {k.upper(): v for k, v in isa.BR_CONDS.items()}
        if ops[0].strip().upper() not in conds:
            raise c.err(f"unknown condition {ops[0]!r}; one of {', '.join(isa.BR_CONDS)}")
        return make("BR", cond=conds[ops[0].strip().upper()], target=c.value(ops[1]))
    if mn == "LOOP":
        if len(ops) != 2:
            raise c.err("LOOP rd, target")
        return make("LOOP", rd=c.reg(ops[0]), target=c.value(ops[1]))
    if mn == "WAIT":
        mask, timeout = 0, 0
        for tok in ops:
            kv = _kv(tok)
            if kv and kv[0].lower() == "timeout":
                timeout = c.value(kv[1])
            elif tok.strip().lower() == "none":
                pass
            else:
                for e in tok.split("|"):
                    e = e.strip().upper()
                    if not re.fullmatch(r"E[0-3]", e):
                        raise c.err(f"WAIT takes none, E0-E3 (joined with |) and timeout=N; found {tok!r}")
                    mask |= 1 << int(e[1])
        return make("WAIT", mask=mask, timeout=timeout)
    if mn == "EVT":
        if len(ops) not in (3, 4):
            raise c.err("EVT k, target, on|off [, clear]")
        onoff = ops[2].strip().lower()
        if onoff not in ("on", "off"):
            raise c.err("EVT third operand is on or off")
        clear = 0
        if len(ops) == 4:
            if ops[3].strip().lower() != "clear":
                raise c.err("EVT fourth operand can only be clear")
            clear = 1
        return make("EVT", k=c.value(ops[0]), target=c.value(ops[1]),
                    enable=1 if onoff == "on" else 0, clear=clear)
    if mn in isa.Q_OPS:
        return _q(c, mn, ops)
    if mn in isa.MISC_OPS:
        if ops:
            raise c.err(f"{mn} takes no operands")
        return make("MISC", op=isa.MISC_OPS[mn])
    raise c.err(f"unknown mnemonic {mnemonic!r}")


def _strip_comment(line: str) -> str:
    return line.split(";", 1)[0].rstrip()


def assemble(text: str, origin: int = 0, symbols: dict[str, int] | None = None) -> Program:
    """Two passes: labels first, then encoding."""
    syms: dict[str, int] = dict(symbols or {})
    raw_lines = text.splitlines()
    # pass 1: labels and .equ
    items: list[tuple[int, str, str, int]] = []   # (line no, mnemonic, operands, addr)
    addr = origin
    for no, line in enumerate(raw_lines, 1):
        body = _strip_comment(line).strip()
        while True:
            m = _LABEL.match(body)
            if not m:
                break
            name = m.group(1)
            if name in syms:
                raise AsmError(f"label {name!r} defined twice", no, line)
            syms[name] = addr
            body = body[m.end():].strip()
        if not body:
            continue
        parts = body.split(None, 1)
        mnemonic, rest = parts[0], parts[1] if len(parts) > 1 else ""
        if mnemonic.lower() == ".equ":
            ops = _split_operands(rest)
            if len(ops) != 2:
                raise AsmError(".equ name, value", no, line)
            syms[ops[0]] = _Ctx(syms, no, line, True).value(ops[1])
            continue
        items.append((no, mnemonic, rest, addr))
        addr += 1
    if addr - origin > isa.PROGRAM_WORDS - origin:
        raise AsmError(f"program does not fit: ends at {addr}", 0)
    # pass 2: encode
    prog = Program(words=[], origin=origin, labels={k: v for k, v in syms.items()
                                                     if k not in (symbols or {})})
    for no, mnemonic, rest, a in items:
        c = _Ctx(syms, no, raw_lines[no - 1], True)
        ops = _split_operands(rest)
        if mnemonic.lower() == ".word":
            if len(ops) != 1:
                raise c.err(".word value")
            word = c.value(ops[0]) & 0xFFFF_FFFF
        else:
            try:
                word = isa.encode(_parse(c, mnemonic, ops))
            except EncodeError as e:
                raise c.err(str(e)) from None
        prog.words.append(word)
        prog.lines.append(no)
        prog.listing.append(f"{a:03X}  {word:08X}  {raw_lines[no - 1].rstrip()}")
    return prog


def listing_text(prog: Program) -> str:
    return "\n".join(prog.listing)
