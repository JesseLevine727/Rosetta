"""ASCII waveforms, and an expect-style test that pins the model's SPI
mode 0 timing to the cycle: any change to a timing rule in model.py or
MODEL_DECISIONS.md shows up here as a readable diff."""
import unittest

import support  # noqa: F401
from benches import SpiTarget
from rosetta import asm, model, wave

SPI_EXPECTED = """\
      20          32   37  41    47 50 53    59    65    71 74 77    83 86
cs    ~~----------_____________________________________________________-----
sck   _____________________--____--____--____--____--____--____--____--_____
mosi  ~~_______________------____________-----------------------------------
miso  _____________-----------------________________________------------____"""


class Render(unittest.TestCase):
    def test_bits_bus_and_ruler(self):
        text = wave.render({"a": [0, 0, 1, 1, "z", "z", 1, 0], "bus": ["x", "x", 5, 5, 5, 7, 7, 7]})
        self.assertEqual(text, "     0 2 4 6\n"
                               "a    __--~~-_\n"
                               "bus  |x|5.|7.")

    def test_scale_and_elide(self):
        sig = {"s": [0] * 50 + [1] * 50}
        self.assertEqual(wave.render(sig, scale=10, ruler=False), "s  _____-----")
        self.assertEqual(wave.render(sig, scale=10, elide=2, ruler=False), "s  __/--/")

    def test_edges(self):
        self.assertEqual(wave.edges("0011z0"), [(2, "0", "1"), (4, "1", "z"), (5, "z", "0")])

    def test_spi_mode0_expect(self):
        prog = asm.assemble(support.readme_programs()["spi_controller"])
        lane = model.Lane(prog.words)
        lane.host_write(0x9F)
        lane.start(0)
        tr = model.run(lane, 90, bench=SpiTarget([0xC3]))
        got = wave.render({"cs": tr.out(2), "sck": tr.out(3), "mosi": tr.out(0),
                           "miso": tr.inp(0)}, start=20, end=90)
        self.assertEqual(got, SPI_EXPECTED, "\n" + got)


if __name__ == "__main__":
    unittest.main()
