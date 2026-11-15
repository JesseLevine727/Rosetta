"""Compiler (rosetta/compiler.py) on the UART description, without simulation.

The RTL runs are in test_uart_rtl.py. Here: every difference from the
hardware-proven hand-written programs is the documented one, the rate checks
of language.md 3.16 item 7 and 8, and unsupported constructs are named.
"""
import contextlib
import io
import json
import os
import unittest

import support
from rosetta import asm, isa
from rosetta.compile import main as cli
from rosetta.compiler import CompileError, Unsupported, compile_file, compile_text

UART = os.path.join(support.TOOLCHAIN, "..", "programs", "uart", "uart.ros")


def hand_words(name: str) -> list[int]:
    with open(os.path.join(support.FIXTURES, f"hand_{name}.asm"), encoding="utf-8") as f:
        return asm.assemble(f.read()).words


def normalise(words: list[int], drop: set[int], retag: dict[int, int]) -> list[tuple]:
    """Decoded instructions without `drop`, branch targets as positions in the
    kept list, tags renamed by `retag`."""
    keep = [i for i in range(len(words)) if i not in drop]
    pos = {i: k for k, i in enumerate(keep)}
    out = []
    for i in keep:
        ins = isa.decode(words[i])
        f = dict(ins.fields)
        if ins.op in ("BR", "EVT", "LOOP"):
            f["target"] = pos[f["target"]]
        if f.get("tag") in retag:
            f["tag"] = retag[f["tag"]]
        out.append((ins.op, f))
    return out


class HandWritten(unittest.TestCase):
    """115200 baud, no parity: the compiled program is the FPGA-proven one
    plus SETM CODE (and SETM THR2L for the sender), each writing the reset
    value 0, with the compiler's record tags (data 0x02, error stop_bit 0x44)
    in place of the hand-picked 0x01 and 0xFE."""

    def check(self, role: str, hand: str, extra: set[str]):
        c = compile_file(UART, role)
        drop = set()
        for i, w in enumerate(c.words):
            ins = isa.decode(w)
            if ins.op == "SETM" and isa.MODE_REGS[ins["reg"]].name in extra:
                self.assertEqual(ins["imm"], 0)
                drop.add(i)
        self.assertEqual(len(drop), len(extra))
        retag = {0x02: 0x01, 0x44: 0xFE}
        self.assertEqual(normalise(c.words, drop, retag), normalise(hand_words(hand), set(), {}))

    def test_sender(self):
        self.check("sender", "uart_tx", {"CODE", "THR2L"})

    def test_receiver(self):
        self.check("receiver", "uart_rx", {"CODE"})

    def test_listener(self):
        self.check("listener", "uart_listener", {"CODE"})


class Rates(unittest.TestCase):

    def test_115200(self):
        tx, rx = compile_file(UART, "sender"), compile_file(UART, "receiver")
        self.assertEqual((tx.mode_regs["PERIOD"], tx.mode_regs["PERIOD2"]), (260, 260))
        self.assertEqual(tx.manifest["timing"]["rate_error_percent"], -0.16)    # 520 cycles: fast
        self.assertEqual(rx.mode_regs["PERIOD"], 521)
        self.assertEqual(rx.manifest["timing"]["rate_error_percent"], 0.032)

    def test_slow_rate_uses_the_prescaler(self):
        c = compile_file(UART, "sender", {"baud": "300"})
        self.assertEqual(isa.mode_decode("TICK", c.mode_regs["TICK"])["prescale"], 1)  # code 1: /8
        self.assertEqual((c.mode_regs["PERIOD"], c.mode_regs["PERIOD2"]), (12500, 12500))

    def test_rounding_outside_tolerance_is_rejected(self):
        with open(UART, encoding="utf-8") as f:
            text = f.read().replace("tolerance 2%", "tolerance 0.01%")
        with self.assertRaisesRegex(CompileError, "outside the tolerance"):
            compile_text(text, "sender")

    def test_schedule_too_slow_for_the_rate(self):
        with self.assertRaisesRegex(CompileError, r"schedule does not fit .* Fastest rate"):
            compile_file(UART, "receiver", {"baud": "2000000"})


class HostView(unittest.TestCase):

    def test_manifest(self):
        tx = compile_file(UART, "sender", {"parity": "even"})
        rx = compile_file(UART, "receiver", {"parity": "even"})
        ls = compile_file(UART, "listener", {"parity": "even"})
        self.assertEqual(tx.manifest["in_queue"], [{"field": "data", "width": 8,
                                                     "word": "value in bits [7:0]"}])
        self.assertIsNone(tx.manifest["out_queue"])
        tags = rx.manifest["out_queue"]["tags"]
        self.assertEqual(tags, {
            "0x02": {"kind": "value", "field": "data", "width": 8},
            "0x43": {"kind": "error", "field": "parity_bit", "width": 1,
                     "expected": "even_parity(data)", "then": "continue"},
            "0x44": {"kind": "error", "field": "stop_bit", "width": 1, "expected": "1",
                     "then": "end"}})
        self.assertEqual(ls.manifest["out_queue"]["tags"], tags)
        self.assertEqual(ls.manifest["out_queue"]["words_per_record"], 2)

    def test_cli_json(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(cli([UART, "--role", "listener", "-p", "parity=odd", "--json"]), 0)
        data = json.loads(out.getvalue())
        c = compile_file(UART, "listener", {"parity": "odd"})
        self.assertEqual(data["words"], [f"{w:08x}" for w in c.words])
        self.assertEqual(asm.assemble(data["asm"]).words, c.words)


class Unsupported_(unittest.TestCase):

    def test_named(self):
        for fixture, construct in (("spi.ros", "clock WIRE by PARTY (clock wire)"),
                                   ("i2c.ros", "wire ... open_drain"),
                                   ("ws2812.ros", "clock symbols")):
            with self.subTest(fixture):
                with self.assertRaises(Unsupported) as cm:
                    compile_file(os.path.join(support.FIXTURES, fixture), "controller")
                self.assertEqual(cm.exception.construct, construct)

    def test_bad_role_and_parameter(self):
        with self.assertRaisesRegex(CompileError, "not a role"):
            compile_file(UART, "target")
        with self.assertRaisesRegex(CompileError, r"one of none \| even \| odd"):
            compile_file(UART, "sender", {"parity": "mark"})


if __name__ == "__main__":
    unittest.main()
