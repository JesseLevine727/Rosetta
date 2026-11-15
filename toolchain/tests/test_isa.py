"""Encode/decode round trip and the disassembler, all driven by the tables."""
import random
import unittest

import support  # noqa: F401
from rosetta import asm, isa


def random_instr(rng: random.Random, op: str) -> isa.Instr:
    """A random well-formed Instr of format `op` (any in-range field values)."""
    fmt = isa.FORMATS[op]
    values: dict[str, int] = {}
    for f in fmt.fields:
        if f.present(values):
            if f.enum is not None and rng.random() < 0.7:
                values[f.name] = rng.choice(list(f.enum.values()))
            else:
                values[f.name] = rng.randint(f.min, f.max)
    return isa.Instr(op, values)


class RoundTrip(unittest.TestCase):
    def test_decode_encode_every_form(self):
        rng = random.Random(7)
        for op in isa.FORMATS:
            for _ in range(3000):
                x = random_instr(rng, op)
                self.assertEqual(isa.decode(isa.encode(x)), x, msg=f"{x}")

    def test_every_field_kind_and_union_branch(self):
        """Both sides of each union (FIELD/RAW immediate vs tag, SETM imm vs reg)."""
        imm = isa.OPERANDS["IMM"]
        for kind in isa.FIELD_KINDS.values():
            for src in isa.OPERANDS.values():
                x = isa.make("FIELD", kind=kind, width=32, src=src, dst=15)
                self.assertEqual(isa.decode(isa.encode(x)), x)
                self.assertEqual("imm" in x.fields, src == imm)
        for from_reg in (0, 1):
            x = isa.make("SETM", reg=8, upper=1, from_reg=from_reg)
            self.assertEqual(isa.decode(isa.encode(x)), x)

    def test_encode_decode_words_bit_exact(self):
        rng = random.Random(3)
        for _ in range(20000):
            w = rng.getrandbits(32)
            w &= isa.defined_mask(w)
            self.assertEqual(isa.encode(isa.decode(w)), w)

    def test_out_of_range_rejected(self):
        with self.assertRaises(isa.EncodeError):
            isa.encode(isa.make("FIELD", width=33))
        with self.assertRaises(isa.EncodeError):
            isa.make("FIELD", src=isa.OPERANDS["IMM"], tag=1)

    def test_reserved_opcodes(self):
        for op in isa.RESERVED_OPCODES:
            w = (op << 28) | 0x123
            self.assertEqual(isa.decode(w).op, "RESERVED")
            self.assertEqual(isa.disassemble(w), f".word 0x{w:08X}")

    def test_mode_registers(self):
        rng = random.Random(1)
        for r in isa.MODE_REGS:
            for _ in range(500):
                vals = {f.name: rng.randint(f.min, f.max) for f in r.fields}
                self.assertEqual(isa.mode_decode(r.name, isa.mode_encode(r.name, **vals)), vals)

    def test_stored_bits_follow_lane_md(self):
        """lane.md 5.1 'Stored bits' and rulings 10, 21, 23: THR1 and THR2 are
        deleted, trigger codes 5-7 are off, B-must 3 is 'never'."""
        stored = {"TICK": 0x1FFF, "PERIOD": 0xFFFF, "PERIOD2": 0xFFFF, "THR01": 0x00FF,
                  "THR2L": 0xFF00, "CODE": 0x0FFF, "PULSE": 0xFFFF, "STUFF": 0x003F,
                  "CRCPOLY": 0xFFFF_FFFF, "CRCCFG": 0x007F,
                  **{f"EVC{k}": 0xFFFF for k in range(4)}}
        self.assertEqual({r.name: r.defined_mask for r in isa.MODE_REGS}, stored)
        self.assertEqual([f.name for f in isa.MODE_REG_BY_NAME["THR01"].fields], ["thr0"])
        self.assertEqual([f.name for f in isa.MODE_REG_BY_NAME["THR2L"].fields], ["delay"])
        w = asm.assemble("SETM EVC1, a=in2, trigger=LEVEL, level=HIGH, b=in3, b_must=NEVER, k=5").words[0]
        self.assertEqual(w & 0xFFFF, 2 | 1 << 2 | 1 << 3 | 3 << 6 | 3 << 8 | 5 << 10)
        for code in (5, 6, 7):                   # off; no name, printed as a number, exact round trip
            word = asm.assemble(f"SETM EVC0, {code << 3}").words[0]
            self.assertEqual(isa.mode_decode("EVC0", word & 0xFFFF)["trigger"], code)
            self.assertIn(code, isa.EV_TRIGGER_OFF_CODES)
            self.assertEqual(asm.assemble(isa.disassemble(word)).words, [word])
        with self.assertRaises(asm.AsmError):
            asm.assemble("SETM THR2L, thr2=255")

    def test_known_encodings(self):
        """Hand-checked against the tables of isa.md 3.1, 3.4, 3.6."""
        # FIELD DRIVE 8, lsb, R0: kind 0, n-1=7 at [25:21], lsb [20], src R0, dst NONE
        self.assertEqual(asm.assemble("FIELD DRIVE 8, lsb, R0").words, [0x00F03C00])
        # SETM PERIOD, 260: reg 1 at [27:23]
        self.assertEqual(asm.assemble("SETM PERIOD, 260").words, [0x30800104])
        # BR NM, 4: cond 6 at [27:23]
        self.assertEqual(asm.assemble("BR NM, 4").words, [0x73000004])
        # SAMPLE 8 lsb HOSTQ tag=1: src NONE (15) [17:14], dst HOSTQ (4) [13:10]
        self.assertEqual(asm.assemble("FIELD SAMPLE 8, lsb, HOSTQ, tag=1").words,
                         [(1 << 26) | (7 << 21) | (1 << 20) | (15 << 14) | (4 << 10) | 1])


class Disassembler(unittest.TestCase):
    def test_assemble_disassemble_random_words(self):
        """assemble(disassemble(w)) == w for any word (mnemonic or .word)."""
        rng = random.Random(11)
        pretty = 0
        for _ in range(20000):
            w = rng.getrandbits(32)
            if rng.random() < 0.9:
                w &= isa.defined_mask(w)
            text = isa.disassemble(w)
            pretty += not text.startswith(".word")
            self.assertEqual(asm.assemble(text).words, [w], msg=text)
        self.assertGreater(pretty, 5000)

    def test_well_formed_instructions_print_as_mnemonics(self):
        rng = random.Random(5)
        for prog in support.readme_programs().values():
            for w in asm.assemble(prog).words:
                self.assertFalse(isa.disassemble(w).startswith(".word"), isa.disassemble(w))
        del rng


if __name__ == "__main__":
    unittest.main()
