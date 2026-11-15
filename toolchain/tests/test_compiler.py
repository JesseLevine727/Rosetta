"""Compiler (rosetta/compiler.py) on the UART description, without simulation.

The RTL runs are in test_uart_rtl.py and test_spi_rtl.py. Here: every
difference from the hardware-proven hand-written programs is the documented
one, the rate checks of language.md 3.16 items 7 and 8, the SPI limits, and
unsupported constructs are named.
"""
import contextlib
import io
import json
import os
import unittest

import support
from rosetta import asm, isa
from rosetta.compile import main as cli
from fractions import Fraction

from rosetta.compiler import CompileError, Unsupported, _check_rulings, compile_file, compile_text

UART = os.path.join(support.TOOLCHAIN, "..", "programs", "uart", "uart.ros")
SPI = os.path.join(support.TOOLCHAIN, "..", "programs", "spi", "spi.ros")
I2C = os.path.join(support.TOOLCHAIN, "..", "programs", "i2c", "i2c.ros")
HOLD_TICK = "SETM TICK, mode=EDGE, clock_in=in1, clock_out, clock_idle, clock_od, stretch_hold"


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


class Spi(unittest.TestCase):
    """SPI (language.md 2.2). The RTL runs are in test_spi_rtl.py."""

    def test_same_configuration_as_hand_written(self):
        """Mode 0, 1 MHz: every mode register the FPGA-proven programs write
        has the same value in the compiled ones (same pins, clock, events)."""
        for role, hand in (("controller", "spi_controller"), ("target", "spi_target")):
            with self.subTest(role):
                c = compile_file(SPI, role)
                for w in hand_words(hand):
                    ins = isa.decode(w)
                    if ins.op == "SETM":
                        self.assertEqual(c.mode_regs[isa.MODE_REGS[ins["reg"]].name], ins["imm"])

    def test_fastest_clock(self):
        top = {0: 1_500_000, 1: 2_000_000}               # by cpha: the target's first bit
        for cpha, rate in top.items():
            p = {"cpha": str(cpha), "sck_rate": f"{rate} Hz"}
            self.assertEqual(compile_file(SPI, "target", p).manifest["timing"]
                             ["fastest_rate_hz_schedule"], rate)
            with self.assertRaisesRegex(CompileError, "schedule does not fit"):
                compile_file(SPI, "target", {**p, "sck_rate": f"{rate + 10_000} Hz"})
        c = compile_file(SPI, "controller", {"sck_rate": "6 MHz"})
        self.assertEqual((c.mode_regs["PERIOD"], c.mode_regs["PERIOD2"]), (5, 5))
        with self.assertRaisesRegex(CompileError, "round trip"):
            compile_file(SPI, "controller", {"sck_rate": "6.1 MHz"})

    def test_listener_needs_one_lane_per_wire(self):
        with self.assertRaises(Unsupported) as cm:
            compile_file(SPI, "listener")
        self.assertIn("G5", str(cm.exception))
        c = compile_file(SPI, "listener", listen_wire="miso")
        self.assertEqual(c.manifest["listener"], {"listens_on": "miso", "other_lanes_for": ["mosi"]})

    def test_target_data_needs_default(self):
        with self.assertRaisesRegex(CompileError, r"3\.16 item 4"):
            compile_text(spi_text().replace(" default 0", ""), "controller")

    def test_rulings_49_and_53(self):
        seek = asm.assemble("SETM TICK, mode=BAUD\nFIELD SAMPLE 8, HOSTQ, seek").words
        with self.assertRaisesRegex(CompileError, "ruling 49"):
            _check_rulings(seek, Fraction(100))
        hold = asm.assemble(HOLD_TICK).words                # satisfies ruling 55 (b)
        with self.assertRaisesRegex(CompileError, "ruling 53"):     # (a): low phase under 5
            _check_rulings(hold, Fraction(4))
        _check_rulings(hold, Fraction(5))

    def test_ruling_55_b(self):
        for tick in ("SETM TICK, mode=EDGE, clock_out, clock_idle, stretch_hold",         # not od
                     "SETM TICK, mode=EDGE, clock_out, clock_od, stretch_hold",           # idle 0
                     "SETM TICK, mode=EDGE, capture=FALLING, clock_out, clock_od, clock_idle, "
                     "stretch_hold"):                                                     # falling
            with self.subTest(tick):
                with self.assertRaisesRegex(CompileError, r"ruling 55 b"):
                    _check_rulings(asm.assemble(tick).words, Fraction(78))

    def test_ruling_55_c(self):
        with self.assertRaisesRegex(CompileError, r"word 1: PIN 0x29000000 writes out3.*ruling 55 c"):
            _check_rulings(asm.assemble(HOLD_TICK + "\nPIN out0=z, out3=z").words, Fraction(78))
        _check_rulings(asm.assemble(HOLD_TICK + "\nPIN out0=z").words, Fraction(78))
        for rate in ("100 kHz", "400 kHz"):                  # the compiled I2C target passes
            t = compile_file(I2C, "target", {"scl_rate": rate})
            _check_rulings(t.words, Fraction(78))
            self.assertNotIn(True, [isa.decode(w).op == "PIN" and bool(isa.decode(w)["mask"] & 8)
                                    for w in t.words])


def spi_text() -> str:
    with open(SPI, encoding="utf-8") as f:
        return f.read()


class I2c(unittest.TestCase):
    """I2C (language.md 2.3). The RTL runs are in test_i2c_rtl.py."""

    def test_timing_from_the_rules(self):
        c = compile_file(I2C, "controller", {"scl_rate": "400 kHz"})
        t = compile_file(I2C, "target", {"scl_rate": "400 kHz"})
        # t_low 1.3 us = 78 cycles; t_high 0.6 us: 150 - 78 - 4 (capture waits for scl high)
        self.assertEqual((c.mode_regs["PERIOD"], c.mode_regs["PERIOD2"]), (78, 68))
        self.assertEqual(t.mode_regs["PERIOD"], 6)              # t_su_dat 100 ns before release
        self.assertEqual(isa.mode_decode("EVC0", t.mode_regs["EVC0"])["k"], 18)   # 300 ns window
        tick = isa.mode_decode("TICK", t.mode_regs["TICK"])
        self.assertTrue(tick["stretch_hold"] and tick["clock_od"])
        self.assertEqual(c.manifest["timing"]["fastest_rate_hz"], 526316)
        with self.assertRaisesRegex(CompileError, "Fastest rate: 526316 Hz"):
            compile_file(I2C, "controller", {"scl_rate": "527 kHz"})

    def test_host_view(self):
        c = compile_file(I2C, "controller")
        self.assertEqual(c.manifest["in_queue"][-1], {"frame_end": {"stop": 0, "start": 1}})
        tags = compile_file(I2C, "listener").manifest["out_queue"]["tags"]
        self.assertEqual(tags["0x03"], {"kind": "value", "field": "ack", "width": 1,
                                        "names": {"more": 0, "last": 1}})
        self.assertEqual(tags["0x43"]["kind"], "error")


class Unsupported_(unittest.TestCase):

    def test_named(self):
        for fixture, construct in (("ws2812.ros", "clock symbols"),):
            with self.subTest(fixture):
                with self.assertRaises(Unsupported) as cm:
                    compile_file(os.path.join(support.FIXTURES, fixture), "controller")
                self.assertEqual(cm.exception.construct, construct)
        for old, new, construct in (
                ("    repeat {", "    repeat 4 {", "repeat N (counted)"),
                ("event select   = cs_n falls by controller",
                 "event select   = cs_n low for 1 us by controller", "event = W high|low for D"),
                ("rule cs_lead : select to sck changes >= 0.5 bit",
                 "rule cs_lead : select to sck changes <= 0.5 bit", "rule ... <= (only `>=` bounds)")):
            with self.subTest(construct):
                with self.assertRaises(Unsupported) as cm:
                    compile_text(spi_text().replace(old, new), "controller")
                self.assertEqual(cm.exception.construct, construct)
        with self.assertRaisesRegex(CompileError, r"`stretch by` needs an open-drain clock wire"):
            compile_text(spi_text().replace("capture (cpha == 0 ? leading : trailing)",
                                            "capture leading stretch by target"), "controller")

    def test_bad_role_and_parameter(self):
        with self.assertRaisesRegex(CompileError, "not a role"):
            compile_file(UART, "target")
        with self.assertRaisesRegex(CompileError, r"one of none \| even \| odd"):
            compile_file(UART, "sender", {"parity": "mark"})


if __name__ == "__main__":
    unittest.main()
