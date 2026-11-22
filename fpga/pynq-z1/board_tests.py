"""Board-only checks of the regression (run on the PC; they drive the board
through board_server.py like the shared scenarios do):

* clock(): the chip clock measured three ways by the harness meters;
* uart_async_sweep(): the compiled UART receiver and listener (lanes 1, 2)
  fed by the harness's asynchronous UART far end, clocked by the board's own
  125 MHz oscillator, swept in rate around the nominal 115200 Bd. The
  lane-to-lane UART scenario runs both ends on one clock and cannot show this.

Expected (not from the board): the compiler's manifest states the far-end
bit times the receiver decodes, timing.decodes_far_bit_cycles ([495.1,
549.67] chip cycles for 115200 Bd, i.e. -5.2 % .. +5.2 %). It follows from
RECOVER (tick.md 4.3) with PERIOD P = 521: a capture H = P // 2 after each
data edge and every P after; the longest run of equal bits is 9 (0x00:
start + 8 data; 0xFF: 8 data + stop), so 8T < H + 8P and 9T <= H + 9P, and
back-to-back frames need the receiver re-armed before the next start edge.
Pass criterion: every rate whose bit time is inside the stated window less
a 0.5 % margin decodes all 16 bytes on both lanes (the window is the
compiler's promise), and the rates at +-7 % fail on at least one byte (the
sweep reaches past both edges). The measured edges are reported against the
stated window; the description's own tolerance (2 %) is well inside.
"""
from __future__ import annotations

import json
import os
import sys
import time

REPO = os.environ.get("ROSETTA_REPO", "/home/elfo/Documents/Rosetta")
sys.path.insert(0, os.path.join(REPO, "toolchain", "tests"))
sys.path.insert(0, os.path.join(REPO, "toolchain"))
os.environ["ROSETTA_BACKEND"] = "board"

import rtlsim  # noqa: E402
from rosetta.compiler import compile_file  # noqa: E402

UART = os.path.join(REPO, "programs", "uart", "uart.ros")
NOP = 0xC0000000
BAUD = 115200
CLK = 60_000_000
# -7.0 .. +7.0 % in 0.5 % steps, then 0.1 % steps across both edges of the
# window the compiler states
SWEEP = sorted(set([x / 2 for x in range(-14, 15)] + [x / 10 for x in range(-58, -47)]
                   + [x / 10 for x in range(48, 62)]))
DATA = bytes([0x00, 0xFF, 0x55, 0xAA, 0x0F, 0xF0, 0x01, 0x80, 0x7F, 0xFE, 0x33, 0xCC,
              0x69, 0x96, 0x00, 0xFF])


def clock() -> dict:
    link = rtlsim.board_link()
    return json.loads(link.value("K"))


def pct_of(t_cycles: float) -> float:
    return CLK / t_cycles / BAUD * 100 - 100


def uart_async_sweep(parity: str = "none") -> dict:
    rx = compile_file(UART, "receiver", {"parity": parity}, origin=32)
    ls = compile_file(UART, "listener", {"parity": parity}, origin=64)
    tags = rx.manifest["out_queue"]["tags"]
    vtag = next(int(k, 16) for k, v in tags.items() if v["kind"] == "value")
    period = rx.manifest["timing"]["PERIOD"]
    t_lo, t_hi = rx.manifest["timing"]["decodes_far_bit_cycles"]
    image = [NOP] * 96
    image[32:32 + len(rx.words)] = rx.words
    image[64:64 + len(ls.words)] = ls.words
    points = []
    with rtlsim.Chip() as chip:
        chip.load_program(image)
        chip.wr(rtlsim.XIN0, 3 << 16)                     # lane 1 in0 <- ui3
        chip.wr(rtlsim.XIN1, 3)                           # lane 2 in0 <- ui3
        chip.link.ack("U 3")                              # ui3 <- async UART line (idle high)
        chip.start_lane(1, 32)
        chip.start_lane(2, 64)
        chip.run(2000)
        chip.dump()
        for pct in SWEEP:
            want = BAUD * (1 + pct / 100)
            chip.link.value("G 1")                        # chip clock free-runs
            got_rate = float(chip.link.value(f"A 3 {want:.3f} {parity} 0 0 {DATA.hex()}"))
            time.sleep(0.002)                             # the last stop bit's capture
            chip.link.value("G 0")                        # back to lockstep
            rows = chip.dump()
            rxq, lsq = chip.drain(1), chip.drain(2)
            e, lv = [], 1
            for c, _, ui in rows:                         # ui3 as the chip saw it
                if ui >> 3 & 1 != lv:
                    lv ^= 1
                    e.append(c)
            bit_cycles = None
            if len(e) >= 2:
                # first start edge to the last edge, in whole bits (bit time from the trace)
                bits = round((e[-1] - e[0]) * want / CLK)
                bit_cycles = (e[-1] - e[0]) / bits if bits else None
            ok_rx = rxq == [vtag << 24 | b for b in DATA]
            ok_ls = lsq[1::2] == [vtag << 24 | b for b in DATA]
            points.append({"offset_pct": pct, "rate_set": round(got_rate, 3),
                           "bit_cycles_measured": round(bit_cycles, 3) if bit_cycles else None,
                           "receiver_ok": ok_rx, "listener_ok": ok_ls,
                           "receiver": [f"{w:08x}" for w in rxq],
                           "listener_values": [f"{w:08x}" for w in lsq[1::2]]})
    lo, hi = pct_of(t_hi), pct_of(t_lo)
    passed = [p["offset_pct"] for p in points if p["receiver_ok"] and p["listener_ok"]]
    must = [p for p in points if lo + 0.5 <= p["offset_pct"] <= hi - 0.5]
    must_fail = [p for p in points if abs(p["offset_pct"]) >= 7.0]
    ok = all(p["receiver_ok"] and p["listener_ok"] for p in must) and \
        not any(p["receiver_ok"] and p["listener_ok"] for p in must_fail) and len(must_fail) == 2
    # decodes outside the stated window (the compiler is conservative there)
    beyond = [p["offset_pct"] for p in points if p["receiver_ok"] and p["listener_ok"]
              and p["bit_cycles_measured"] and not t_lo <= p["bit_cycles_measured"] <= t_hi]
    return {"name": "board.uart_async_sweep", "parity": parity, "period": period,
            "manifest_decodes_far_bit_cycles": [t_lo, t_hi],
            "expected_window_pct": [round(lo, 2), round(hi, 2)],
            "measured_pass_pct": [min(passed), max(passed)] if passed else None,
            "measured_pass_bit_cycles": [min(p["bit_cycles_measured"] for p in points
                                             if p["receiver_ok"] and p["listener_ok"]),
                                         max(p["bit_cycles_measured"] for p in points
                                             if p["receiver_ok"] and p["listener_ok"])] if passed else None,
            "decoded_outside_stated_window_pct": beyond,
            "pass": ok, "points": points}


if __name__ == "__main__":
    print(json.dumps(clock(), indent=1))
    r = uart_async_sweep()
    print(json.dumps({k: v for k, v in r.items() if k != "points"}, indent=1))
    for p in r["points"]:
        print(f"{p['offset_pct']:+5.1f} %  {p['rate_set']:10.1f} Bd  T={p['bit_cycles_measured']}  "
              f"rx={'ok' if p['receiver_ok'] else 'FAIL'} listener={'ok' if p['listener_ok'] else 'FAIL'}")
