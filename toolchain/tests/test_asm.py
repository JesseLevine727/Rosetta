"""The assembler: README examples, labels, listing, errors."""
import unittest

import support  # noqa: F401
from rosetta import asm, isa

EXPECTED = {"uart_tx", "uart_rx", "uart_listener", "spi_controller", "i2c_target"}


class Examples(unittest.TestCase):
    def test_readme_has_all_examples(self):
        self.assertEqual(set(support.readme_programs()), EXPECTED)

    def test_examples_assemble_and_disassemble(self):
        for name, text in support.readme_programs().items():
            with self.subTest(name):
                prog = asm.assemble(text)
                self.assertGreater(len(prog.words), 5)
                self.assertNotIn("...", text)
                again = asm.assemble("\n".join(isa.disassemble(w) for w in prog.words))
                self.assertEqual(again.words, prog.words)

    def test_uart_tx_listing(self):
        prog = asm.assemble(support.readme_programs()["uart_tx"])
        self.assertEqual(prog.labels, {"next": 4})
        self.assertEqual(prog.listing[0].split()[:2], ["000", "30000000"])
        self.assertTrue(prog.listing[4].startswith("004  B0000000  next:"))
        self.assertEqual(isa.disassemble(prog.words[8]), "BR always, 4")

    def test_listener_never_enables_an_output(self):
        """Static check of arch obligation 8 on the listener image: no PIN,
        no driving FIELD or RAW, no clock output."""
        for w in asm.assemble(support.readme_programs()["uart_listener"]).words:
            ins = isa.decode(w)
            self.assertNotEqual(ins.op, "PIN")
            if ins.op == "FIELD":
                self.assertIn(ins["kind"], (isa.FIELD_KINDS["SAMPLE"], isa.FIELD_KINDS["EXPECT"]))
            if ins.op == "SETM" and ins["reg"] == 0:
                self.assertEqual(isa.mode_decode("TICK", ins["imm"])["clock_out"], 0)


class Syntax(unittest.TestCase):
    def test_isa_md_spellings(self):
        """Forms used verbatim in isa.md section 5."""
        a = asm.assemble
        self.assertEqual(a("SETM TICK, BAUD").words, a("SETM TICK, mode=BAUD").words)
        self.assertEqual(a("PIN out0=1, enable").words, a("PIN out0=1").words)
        self.assertEqual(a("ALU CMP R0, 0x50").words, a("CMP R0, 0x50").words)
        self.assertEqual(a("Q POP R0").words, a("POP R0").words)
        a("EVT 1, 0, on")
        a("WAIT none")
        a("Q PUSHI 0, tag=0xFE")

    def test_forward_labels_and_equ(self):
        prog = asm.assemble(".equ ADDR, 0x50\nstart: BR always, end\n CMP R0, ADDR\nend: HALT")
        self.assertEqual(isa.decode(prog.words[0])["target"], 2)
        self.assertEqual(isa.decode(prog.words[1])["imm"], 0x50)

    def test_setm_fields(self):
        w = asm.assemble("SETM EVC0, a=in0, trigger=EDGE, level=LOW, b=in1, b_must=HIGH, k=18").words[0]
        f = isa.mode_decode("EVC0", isa.decode(w)["imm"])
        self.assertEqual((f["a"], f["trigger"], f["level"], f["b"], f["b_must"], f["k"]), (0, 1, 0, 1, 1, 18))
        hi = isa.decode(asm.assemble("SETM CRCPOLY.hi, 0x04C1").words[0])
        self.assertEqual((hi["reg"], hi["upper"], hi["imm"]), (8, 1, 0x04C1))
        self.assertEqual(isa.decode(asm.assemble("SETM PERIOD, R2").words[0])["rs"], 2)

    def test_crc_flag_versus_crc_operand(self):
        f = isa.decode(asm.assemble("FIELD DRIVE 16, crc, CRC").words[0])
        self.assertEqual((f["crc"], f["src"]), (1, isa.OPERANDS["CRC"]))

    def test_errors_carry_line_numbers(self):
        cases = {
            "NOP\nFIELD DRIVE 33, R0": "line 2",
            "BR always, nowhere": "unknown value",
            "FIELD DRIVE 8, IMM 3, tag=1": "share bits",
            "SETM TICK, mode=FAST": "takes one of",
            "SETM PERIOD, 0x10000": "16 bits",
            "a: NOP\na: NOP": "defined twice",
            "FROB R0": "unknown mnemonic",
            "Q POP R0, tag=3": "no tag",
        }
        for text, msg in cases.items():
            with self.subTest(text):
                with self.assertRaises(asm.AsmError) as cm:
                    asm.assemble(text)
                self.assertIn(msg, str(cm.exception))


if __name__ == "__main__":
    unittest.main()
