"""UART end to end on the chip RTL (src/rosetta_top.sv under Verilator).

Lane 0 runs the compiled sender, lane 1 the receiver, lane 2 the listener.
The image goes in through the host SPI port; lane 0's out0 leaves on uo[2],
a board wire takes it to ui[3], and the crossbar gives ui[3] to lanes 1 and
2 as in0 (the FPGA loopback arrangement). For each parity setting the host
sends a few bytes, then one more with a deliberately wrong bit injected on
the wire: the parity bit (parity even/odd, `else continue`) or the stop bit
(parity none, `else end`). Checked: the sender's waveform bit by bit and its
timing, the receiver's and the listener's records against the manifest, the
listener's timestamps to the cycle. A last run puts the hardware-proven
hand-written programs through the same stimulus and compares.

Skips when Verilator is not installed. ROSETTA_SIM_REPORT=1 prints a summary.
"""
import os
import unittest

import support  # noqa: F401  (puts toolchain/ on sys.path)
import rtlsim
from rosetta import asm
from rosetta.compiler import compile_file

UART = os.path.join(rtlsim.REPO, "programs", "uart", "uart.ros")
NOP = 0xC0000000
BYTES = [0x48, 0x69, 0x00, 0xFF, 0x55, 0x0F]     # "Hi", and runs of 9 equal bits
FAULT_BYTE = 0xA5
BIT = 520                                       # sender bit, cycles (PERIOD + PERIOD2)
REPORT = bool(os.environ.get("ROSETTA_SIM_REPORT"))

# Hardware-proven programs (fixtures/hand_uart_*.asm, copied from the FPGA run).
def hand(name: str) -> str:
    with open(os.path.join(support.FIXTURES, f"hand_{name}.asm"), encoding="utf-8") as f:
        return f.read()


def parity_of(b: int) -> int:
    return bin(b).count("1") & 1


def wire_bits(byte: int, parity: str) -> list[int]:
    bits = [0] + [(byte >> i) & 1 for i in range(8)]
    if parity != "none":
        bits.append(parity_of(byte) ^ (parity == "odd"))
    return bits + [1]


def edges(changes, bit_of) -> list[tuple[int, int]]:
    out, prev = [], 1
    for cyc, uo, ui in changes:
        v = bit_of(uo, ui)
        if v != prev:
            out.append((cyc, v))
            prev = v
    return out


def frames(edge_list, nbits):
    """Split a line (idle high) into frames: (start cycle, [edge offsets])."""
    out, i = [], 0
    while i < len(edge_list):
        start, v = edge_list[i]
        assert v == 0, f"frame must start with a falling edge, cycle {start}"
        offs = []
        i += 1
        end = start + nbits * BIT                       # a rise exactly there ends this frame
        while i < len(edge_list) and (edge_list[i][0] < end
                                      or (edge_list[i][0] == end and edge_list[i][1] == 1)):
            offs.append(edge_list[i][0] - start)
            i += 1
        out.append((start, offs))
    return out


def level_at(start, offs, t):
    """Line level t cycles after a frame's falling edge."""
    return sum(1 for o in offs if o <= t) % 2


def capture(start, offs, k):
    """Cycle of RECOVER's capture tick for frame bit k (tick.md 4.3, PERIOD
    521): 260 after the last edge at a bit start, then every 521."""
    j = max([0] + [o // BIT for o in offs if o % BIT == 0 and o // BIT <= k])
    return start + j * BIT + 260 + (k - j) * 521


def run_link(images, sends, fault_bit):
    """images: [(lane, origin, words)]. Sends `sends` bytes, then FAULT_BYTE
    with frame bit `fault_bit` inverted on the wire. Returns the pin changes
    and the drained queues of lanes 1 and 2."""
    image = [NOP] * 96
    for _, org, words in images:
        image[org:org + len(words)] = words
    with rtlsim.Chip() as chip:
        chip.load_program(image)
        chip.wr(rtlsim.XOUT0, 0x10)         # uo[2] <- lane 0 out0 (enable, lane 0, out 0)
        chip.wr(rtlsim.XIN0, 3 << 16)       # lane 1 in0 <- ui[3]
        chip.wr(rtlsim.XIN1, 3)             # lane 2 in0 <- ui[3]
        chip.wire(2, 3)                     # board wire uo[2] -> ui[3]
        org = {lane: o for lane, o, _ in images}
        chip.start_lane(0, org[0])
        chip.run(2000)                      # the line goes idle high first
        chip.start_lane(1, org[1])
        chip.start_lane(2, org[2])
        chip.run(2000)
        chip.dump()
        chip.push(0, sends)
        chip.run(len(sends) * 12 * BIT + 2000)
        chip.fault(fault_bit * BIT, BIT)    # at the next start edge on uo[2]
        chip.push(0, [FAULT_BYTE])
        chip.run(12 * BIT + 2000)
        changes = chip.dump()
        rx, ls = chip.drain(1), chip.drain(2)
        ctrl = [chip.rd(rtlsim.lane_reg(k, rtlsim.CTRL)) & 1 for k in range(3)]
    return changes, rx, ls, ctrl


def compiled_images(parity):
    out = {}
    for lane, role, org in ((0, "sender", 0), (1, "receiver", 32), (2, "listener", 64)):
        out[role] = compile_file(UART, role, {"parity": parity}, origin=org)
    return out, [(0, 0, out["sender"].words), (1, 32, out["receiver"].words),
                 (2, 64, out["listener"].words)]


@unittest.skipUnless(rtlsim.find_verilator(), "Verilator not installed")
class UartOnRtl(unittest.TestCase):

    def check_link(self, parity, changes, rx, ls, ctrl, tags, late_bounds):
        nbits = 11 if parity != "none" else 10
        fault_bit = 9                                   # parity bit, or the stop bit with no parity
        sent = BYTES + [FAULT_BYTE]
        self.assertEqual(ctrl, [1, 1, 1], "a lane halted")

        # the sender's own pin: every frame bit by bit, every edge on the 520-cycle grid
        tx = frames(edges(changes, lambda uo, ui: (uo >> 2) & 1), nbits)
        self.assertEqual(len(tx), len(sent))
        for (start, offs), b in zip(tx, sent):
            self.assertTrue(all(o % BIT == 0 for o in offs), f"edge off the bit grid: {offs}")
            self.assertEqual([level_at(start, offs, k * BIT + BIT // 2) for k in range(nbits)],
                             wire_bits(b, parity), f"byte {b:02x}")
        gaps = [b[0] - a[0] for a, b in zip(tx, tx[1:len(BYTES)])]
        # the wire the receivers see: identical except the injected bit
        line = frames(edges(changes, lambda uo, ui: (ui >> 3) & 1), nbits)
        fstart, foffs = line[-1]
        seen_bit = level_at(fstart, foffs, fault_bit * BIT + BIT // 2)
        self.assertEqual(seen_bit, 1 - wire_bits(FAULT_BYTE, parity)[fault_bit])

        # receiver: one record per byte, plus the error the description asks for
        value, err_par, err_stop = tags
        want = [(value << 24) | b for b in sent]
        err = (err_par << 24) | seen_bit if parity != "none" else (err_stop << 24) | 0
        self.assertEqual(rx, want + [err])
        # listener: {timestamp, record}; data timestamps exact to the cycle
        self.assertEqual(len(ls) % 2, 0)
        ts, recs = ls[0::2], ls[1::2]
        self.assertEqual(recs, want + [err])
        pred = [capture(s, o, 8) for s, o in line]      # last data bit's capture tick
        offset = ts[0] - pred[0]
        self.assertEqual([t - p for t, p in zip(ts, pred)], [offset] * len(pred))
        cap = capture(fstart, foffs, fault_bit)
        late = ts[-1] - offset - cap
        lo, hi = late_bounds["parity_bit" if parity != "none" else "stop_bit"]
        self.assertTrue(lo <= late <= hi, f"error timestamp {late} cycles after the bit's capture, "
                                          f"manifest says {lo}..{hi}")
        if REPORT:
            print(f"\n  parity={parity}: sent {' '.join(f'{b:02x}' for b in sent)} "
                  f"(bit {fault_bit} of the last inverted on the wire)"
                  f"\n    tx bit {BIT} cycles = {60e6 / BIT:.1f} Bd, frame starts {gaps} apart"
                  f"\n    receiver: {' '.join(f'{w:08x}' for w in rx)}"
                  f"\n    listener: {' '.join(f'{t:08x}:{r:08x}' for t, r in zip(ts, recs))}"
                  f"\n    listener ts - capture tick = {offset} for all values; error ts "
                  f"{late} cycles after the bit's capture (manifest: {lo}..{hi})")
        return gaps

    runs: dict = {}

    def run_parity(self, parity):
        if parity in self.runs:
            return self.runs[parity]
        comp, images = compiled_images(parity)
        tags = comp["receiver"].manifest["out_queue"]["tags"]
        self.assertEqual(comp["listener"].manifest["out_queue"]["tags"], tags)
        by_field = {(v["kind"], v["field"]): int(k, 16) for k, v in tags.items()}
        t = (by_field[("value", "data")], by_field.get(("error", "parity_bit"), 0),
             by_field[("error", "stop_bit")])
        changes, rx, ls, ctrl = run_link(images, BYTES, 9)
        late = comp["listener"].manifest["out_queue"]["timestamps"]["error_late_cycles"]
        self.runs[parity] = (self.check_link(parity, changes, rx, ls, ctrl, t, late),
                             (changes, rx, ls))
        return self.runs[parity]

    def test_parity_none_and_framing_error(self):
        gaps, _ = self.run_parity("none")
        self.assertEqual(gaps, [10 * BIT] * (len(BYTES) - 1))       # back to back

    def test_parity_even_and_wrong_parity(self):
        gaps, _ = self.run_parity("even")
        self.assertTrue(all(11 * BIT <= g <= 11 * BIT + 3 * 5 for g in gaps), gaps)

    def test_parity_odd_and_wrong_parity(self):
        gaps, _ = self.run_parity("odd")
        self.assertTrue(all(11 * BIT <= g <= 11 * BIT + 3 * 5 for g in gaps), gaps)

    def test_same_behaviour_as_hand_written(self):
        """Same stimulus through the FPGA-proven programs: the same wire
        waveform, the same records (their tags 0x01/0xFE are the compiler's
        0x02/0x44) and the same listener timestamps relative to the first."""
        _, (c_changes, c_rx, c_ls) = self.run_parity("none")
        images = [(0, 0, asm.assemble(hand("uart_tx")).words),
                  (1, 16, asm.assemble(hand("uart_rx"), 16).words),
                  (2, 48, asm.assemble(hand("uart_listener"), 48).words)]
        h_changes, h_rx, h_ls, ctrl = run_link(images, BYTES, 9)
        self.assertEqual(ctrl, [1, 1, 1])

        def rel_edges(ch):
            e = edges(ch, lambda uo, ui: (uo >> 2) & 1)
            return [(c - e[0][0], v) for c, v in e]
        self.assertEqual(rel_edges(c_changes), rel_edges(h_changes))
        retag = {0x01: 0x02, 0xFE: 0x44}
        fix = [(retag[w >> 24] << 24) | (w & 0xFFFFFF) for w in h_rx]
        self.assertEqual(c_rx, fix)
        self.assertEqual(c_ls[1::2], [(retag[w >> 24] << 24) | (w & 0xFFFFFF) for w in h_ls[1::2]])
        self.assertEqual([t - c_ls[0] for t in c_ls[0::2]], [t - h_ls[0] for t in h_ls[0::2]])


if __name__ == "__main__":
    unittest.main()
