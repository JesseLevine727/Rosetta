import support  # noqa: F401
from rosetta import lang

import os
import unittest

from rosetta.lang import (Binary, Call, Conditional, EnumType, Field, If, Name, Number,
                          Range, RangeType, Repeat, Together, Unary, ValueWord)

CORE = ["uart", "spi", "i2c", "ws2812", "lib"]
FAILING = {"bad": 5, "usb_ls": 4, "eth10": 12}   # fixture -> line of the first error


def load(name):
    return lang.parse_file(os.path.join(support.FIXTURES, name + ".ros"))


def proto(text):
    """Parse the body lines `text` inside a minimal protocol."""
    return lang.parse("protocol p {\n" + text + "\n}\n").protocol


def expr(text):
    return proto("param x = " + text).items[0].default


def fields(node, name):
    return [n for n in lang.walk(node) if isinstance(n, Field) and n.name == name]


class FixtureTest(unittest.TestCase):
    def test_core_fixtures_parse(self):
        for name in CORE:
            with self.subTest(name):
                self.assertIsInstance(load(name), lang.File)

    def test_failing_fixtures(self):
        for name, line in FAILING.items():
            with self.subTest(name):
                with self.assertRaises(lang.ParseError) as cm:
                    load(name)
                self.assertEqual(cm.exception.line, line)
                self.assertTrue(str(cm.exception).startswith(
                    os.path.join(support.FIXTURES, name + ".ros") + f":{line}:"))

    def test_extension_words_reported(self):
        with self.assertRaises(lang.ParseError) as cm:
            load("usb_ls")
        self.assertIn("expected end of line, found keyword 'runtime'", str(cm.exception))
        with self.assertRaises(lang.ParseError) as cm:
            load("eth10")
        self.assertIn("found keyword 'seek'", str(cm.exception))
        with self.assertRaises(lang.ParseError) as cm:
            load("bad")
        self.assertIn("expected '>=', '<=' or 'in', found '=='", str(cm.exception))

    def test_uart(self):
        p = load("uart").protocol
        self.assertEqual(p.name, "uart")
        baud, parity = p.items_of(lang.Param)
        self.assertEqual(baud.default, Number(115200, "Hz"))
        self.assertEqual(baud.line, 4)
        self.assertEqual(parity.type, EnumType(("none", "even", "odd")))
        self.assertEqual(parity.default, Name("none"))
        (wire,) = p.items_of(lang.Wire)
        self.assertEqual((wire.by, wire.idle), ("sender", ValueWord("high")))
        (clock,) = p.items_of(lang.RateClock)
        self.assertEqual(clock, lang.RateClock(Name("baud"), Number(2, "%")))
        (frame,) = p.items_of(lang.Frame)
        self.assertEqual((frame.after, frame.until), (None, ()))
        branch = frame.body[2]
        self.assertIsInstance(branch, If)
        self.assertIsInstance(branch.orelse, If)              # else if
        self.assertIsNone(branch.orelse.orelse)
        pb = branch.body[0]
        self.assertEqual(pb.value, Call("even_parity", (Name("data"),)))
        self.assertEqual(pb.on_error, "continue")

    def test_spi(self):
        p = load("spi").protocol
        cpol = p.items_of(lang.Param)[1]
        self.assertEqual(cpol.type, RangeType(0, 1))
        sck = p.items_of(lang.Wire)[0]
        self.assertEqual(sck.idle, Name("cpol"))
        self.assertEqual(p.items_of(lang.Wire)[3].idle, ValueWord("released"))
        (clock,) = p.items_of(lang.WireClock)
        self.assertEqual(clock.capture, Conditional(Binary("==", Name("cpha"), Number(0)),
                                                    ValueWord("leading"), ValueWord("trailing")))
        (frame,) = p.items_of(lang.Frame)
        self.assertEqual((frame.after, frame.until), ("select", ("deselect",)))
        (loop,) = frame.body
        self.assertIsInstance(loop, Repeat)
        self.assertIsNone(loop.count)
        (tog,) = loop.body
        self.assertIsInstance(tog, Together)
        self.assertEqual([f.name for f in tog.body], ["mosi_data", "miso_data"])
        self.assertTrue(all(isinstance(f, Field) for f in tog.body))
        self.assertEqual(tog.body[0].wire, "mosi")
        self.assertEqual(tog.body[1].default, Number(0))
        rules = {r.name: r for r in p.items_of(lang.Rule)}
        self.assertEqual(rules["cs_lead"].body, lang.Interval(
            lang.EventRef("select"), lang.WireEdge("sck", "changes")))
        self.assertEqual(rules["cs_lead"].limit, Number(0.5, "bit"))
        self.assertEqual(rules["mosi_su"].body, lang.Setup("mosi", lang.ClockEdge("capture")))

    def test_i2c(self):
        p = load("i2c").protocol
        self.assertEqual(p.items_of(lang.Param)[0].default, Number(100, "kHz"))
        self.assertEqual(p.items_of(lang.Param)[1].default, Number(0x50))
        self.assertTrue(all(w.open_drain for w in p.items_of(lang.Wire)))
        (clock,) = p.items_of(lang.WireClock)
        self.assertEqual((clock.capture, clock.stretch_by), (ValueWord("rising"), "target"))
        start = p.items_of(lang.Event)[0]
        self.assertEqual(start.condition, lang.EdgeCondition("sda", "falls", "scl", "high"))
        (frame,) = p.items_of(lang.Frame)
        self.assertEqual(frame.until, ("stop", "start"))
        self.assertEqual(frame.body[1].names[1], lang.NamedValue("read", Number(1)))
        ack = frame.body[2]
        self.assertEqual((ack.name, ack.value), ("ack", Number(0)))
        self.assertEqual(ack.when, Binary("==", Name("addr"), Name("address")))
        self.assertEqual(ack.on_error, "end")
        branch = frame.body[3]
        self.assertEqual(branch.cond, Binary("==", Name("rw"), Name("write")))
        self.assertIsNone(branch.body[0].while_)
        read_loop = branch.orelse[0]
        self.assertEqual(read_loop.while_, Binary("==", Name("ack"), Name("more")))
        self.assertEqual(len(fields(frame, "ack")), 3)
        rules = {r.name: r for r in p.items_of(lang.Rule)}
        self.assertEqual(len(rules), 8)
        self.assertTrue(all(r.op == ">=" for r in rules.values()))
        self.assertEqual(rules["t_low"].body, lang.PulseWidth("low", "scl"))
        self.assertEqual(rules["t_low"].limit, Conditional(
            Binary(">", Name("scl_rate"), Number(100, "kHz")), Number(1.3, "us"),
            Number(4.7, "us")))
        self.assertEqual(rules["t_hd_dat"].body, lang.Hold("sda", lang.WireEdge("scl", "falls")))
        self.assertEqual(rules["t_buf"].body,
                         lang.Interval(lang.EventRef("stop"), lang.EventRef("start")))

    def test_ws2812(self):
        p = load("ws2812").protocol
        (clock,) = p.items_of(lang.SymbolClock)
        self.assertEqual(clock.tolerance, Number(150, "ns"))
        self.assertEqual(clock.symbols[1], lang.Symbol(1, Number(0.8, "us"), Number(0.45, "us")))
        (latch,) = p.items_of(lang.Event)
        self.assertEqual(latch.condition, lang.LevelCondition("din", "low", Number(50, "us")))
        (loop,) = p.items_of(lang.Frame)[0].body
        self.assertEqual(loop.count, Name("leds"))
        self.assertEqual(loop.body[0].name, "grb")              # one-line block

    def test_lib(self):
        f = load("lib")
        self.assertIsNone(f.protocol)
        self.assertEqual(len(f.crcs), 5)
        self.assertEqual([c.name for c in f.crcs if c.reflect],
                         ["crc5_usb", "crc16_usb", "crc32_ethernet"])
        self.assertEqual(f.crcs[4], lang.CrcDef("crc32_ethernet", 32, 0x04C11DB7,
                                                0xFFFFFFFF, 0xFFFFFFFF, True))


class SyntaxTest(unittest.TestCase):
    def test_precedence(self):
        self.assertEqual(expr("a + b * c == d ? x : y"), Conditional(
            Binary("==", Binary("+", Name("a"), Binary("*", Name("b"), Name("c"))), Name("d")),
            Name("x"), Name("y")))
        self.assertEqual(expr("not a == 1 and b or c"), Binary("or", Binary(
            "and", Unary("not", Binary("==", Name("a"), Number(1))), Name("b")), Name("c")))
        self.assertEqual(expr("- ~a - b / 2"), Binary(
            "-", Unary("-", Unary("~", Name("a"))), Binary("/", Name("b"), Number(2))))
        self.assertEqual(expr("a ? b : c ? d : e"),
                         Conditional(Name("a"), Name("b"), Conditional(Name("c"), Name("d"),
                                                                       Name("e"))))

    def test_numbers_and_units(self):
        self.assertEqual(expr("0x2A"), Number(42))
        self.assertEqual(expr("0b10_1010"), Number(42))
        self.assertEqual(expr("1_000"), Number(1000))
        self.assertEqual(expr("0.6"), Number(0.6))
        self.assertEqual(expr("2 %"), Number(2, "%"))
        self.assertEqual(expr("1.5%"), Number(1.5, "%"))
        self.assertEqual(expr("(3\n MHz)"), Number(3, "MHz"))
        self.assertEqual(expr("bit + s"), Binary("+", Name("bit"), Name("s")))   # not reserved
        with self.assertRaisesRegex(lang.ParseError, "malformed number '100kHz'"):
            expr("100kHz")
        with self.assertRaisesRegex(lang.ParseError, "found name 'Ghz'"):
            expr("5 Ghz")

    def test_calls_ranges_and_named_values(self):
        self.assertEqual(expr("crc16(a .. b, c)"),
                         Call("crc16", (Range(Name("a"), Name("b")), Name("c"))))
        self.assertEqual(expr("f()"), Call("f", ()))
        f = proto("frame f {\n a pid[4] lsb (OUT = 0x1,\n  IN = 0x9) else continue\n}")
        field = f.items[0].body[0]
        self.assertEqual([(n.name, n.value) for n in field.names],
                         [("OUT", Number(1)), ("IN", Number(9))])
        self.assertEqual(field.on_error, "continue")

    def test_other_constructs(self):
        p = proto("wire d diff (dm, dp) idle high coding nrzi\n"
                  "stuff zero after 6 ones over sync .. crc16\n"
                  "stuff complement after 5 equal\n"
                  "event eop = dm, dp both low for 1 bit .. 2 bit by host\n"
                  "rule r : eop to d changes in 1 us .. 2 us\n"
                  "crc c = width 5 poly 0x05 init 0x1F xorout 0x1F\n"
                  "frame f after a until b or c { }")
        wire, s1, s2, ev, rule, crc, frame = p.items
        self.assertEqual(wire, lang.Wire("d", ("dm", "dp"), None, ValueWord("high"), False, "nrzi"))
        self.assertEqual(s1, lang.Stuff("zero", 6, Range(Name("sync"), Name("crc16"))))
        self.assertEqual(s2, lang.Stuff("complement", 5))
        self.assertEqual(ev.condition, lang.BothLowCondition(
            "dm", "dp", Range(Number(1, "bit"), Number(2, "bit"))))
        self.assertEqual((rule.op, rule.limit), ("in", Range(Number(1, "us"), Number(2, "us"))))
        self.assertFalse(crc.reflect)
        self.assertEqual((frame.after, frame.until, frame.body), ("a", ("b", "c"), ()))
        self.assertEqual(frame.line, 8)

    def test_if_else_chain_and_repeat_forms(self):
        p = proto("frame f {\n if a { x y[1] } else if b {\n  x z[1]\n } else {\n  x w[1]\n }\n"
                  " repeat 3 { x v[1] }\n repeat { x u[1] } while u == 1\n}")
        branch, fixed, dowhile = p.items[0].body
        self.assertEqual(branch.orelse.cond, Name("b"))
        self.assertEqual(branch.orelse.orelse[0].name, "w")
        self.assertEqual((fixed.count, fixed.while_), (Number(3), None))
        self.assertEqual(dowhile.while_, Binary("==", Name("u"), Number(1)))


class ErrorTest(unittest.TestCase):
    def check(self, text, line, column, message):
        with self.assertRaises(lang.ParseError) as cm:
            lang.parse(text, "t.ros")
        self.assertEqual(str(cm.exception), f"t.ros:{line}:{column}: {message}")

    def test_messages(self):
        self.check("protocol p {\n  param a =\n}", 2, 12,
                   "expected an expression, found end of line")
        self.check("protocol p {\n  param width = 3\n}", 2, 9,
                   "expected a name, found keyword 'width'")
        self.check("protocol p {\n  wire w by a by b\n}", 2, 15, "duplicate wire attribute 'by'")
        self.check("protocol p {\n  frame f {\n    if a {\n    }\n    else {\n    }\n  }\n}", 5, 5,
                   "expected a field, 'together', 'repeat', 'if' or '}', found keyword 'else'")
        self.check("protocol p {\n  param a = f(1,\n", 2, 14, "unclosed '('")
        self.check("protocol p {\n  param a = 1 $\n}", 2, 15, "unexpected character '$'")
        self.check("protocol p { }", 1, 14, "expected end of line after '{', found '}'")
        self.check("protocol p {\n}\nprotocol q {\n}", 3, 1,
                   "a file holds at most one protocol (the first is at line 1)")
        self.check("crc c = width 1.5 poly 1 init 0 xorout 0", 1, 15,
                   "expected an integer, found number 1.5")
        self.check("protocol p {\n  param a : x = 1\n}", 2, 15, "expected '|', found '='")
        self.check("protocol p {\n  rule r : low w >= a == b == c\n}", 2, 28,
                   "expected end of line, found '=='")
        self.check("", 1, 1, "expected 'crc' or 'protocol', found end of file")


if __name__ == "__main__":
    unittest.main()
