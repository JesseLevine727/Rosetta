"""Board-only checks of the regression (run on the PC; they drive the board
through board_server.py like the shared scenarios do):

* clock(): the chip clock measured three ways by the harness meters;
* uart_async_sweep(): the compiled UART receiver and listener (lanes 1, 2)
  fed by the harness's asynchronous UART far end, clocked by the board's own
  125 MHz oscillator, swept in rate around the nominal 115200 Bd. The
  lane-to-lane UART scenario runs both ends on one clock and cannot show this.
* eth_rx_sweep(): the study's 10BASE-T receiver against an asynchronous
  Manchester source (rh_line.v: 78.125 ps edge grid on a 200 MHz clock made
  from the board oscillator) with duty-cycle distortion, alternating and
  uniform per-edge jitter; the study's simulated limits are the reference
  (REPORT.md 4.4). Runs on the board (farend.py ethsweep).
* usb_async(): the study's USB device and listener against an asynchronous
  host (both async lines on uio[0..1], D-/D+) at 0 and +-1.5 %.
* can_async(): the study's two CAN nodes with the external node played by
  async line 0 from its own clock (rate offsets +-0.5 %).

Async-line timing: an edge's time is accumulated 5 ns cycles, + 2.5 ns for
a falling-edge toggle, + IDELAYE2 taps of 78.125 ps: a 78.125 ps grid, plus
the source clock's jitter (MMCM, about 0.1 ns) and an unmeasured rise/fall
asymmetry of the fabric path into the chip input (a small fixed DCD).

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
    return json.loads(link.value("clock"))


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
        chip.link.value("uartroute 3")                    # ui3 <- async UART line (idle high)
        chip.start_lane(1, 32)
        chip.start_lane(2, 64)
        chip.run(2000)
        chip.dump()
        for pct in SWEEP:
            want = BAUD * (1 + pct / 100)
            chip.link.value("freerun 1")                  # chip clock free-runs
            got_rate = float(chip.link.value(f"uart 3 {want:.3f} {parity} 0 0 {DATA.hex()}"))
            time.sleep(0.002)                             # the last stop bit's capture
            chip.link.value("freerun 0")                  # back to lockstep
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


# ---------------------------------------------------------------- async lines

def _fixture(name, origin=0, subs=None):
    from rosetta import asm
    with open(os.path.join(REPO, "toolchain", "tests", "fixtures", name), encoding="utf-8") as f:
        t = f.read()
    for k, v in (subs or {}).items():
        t = t.replace(k, v)
    return asm.assemble(t, origin=origin)


# the study's simulated limits (REPORT.md 4.4: last 0/200 point, first failing point)
ETH_STUDY = {"dcd": (16.5, 17.0), "alt": (8.2, 8.5), "uniform": (8.0, 9.0)}
ETH_POINTS = ([["dcd", v] for v in (-18, -17.5, -17, -16.5, -16, -15, 12, 14, 15, 16, 16.5, 17, 17.5, 18, 20)]
              + [["alt", v] for v in (6, 7, 7.5, 8, 8.2, 8.5, 9, 10)]
              + [["uniform", v] for v in (6, 7, 8, 9, 10, 12)])


def eth_rx_sweep(frames: int = 100, points=None, compensate=True) -> dict:
    """Receive-margin sweep on the board; per impairment, the largest value
    with no failed frame and the smallest with a failure, against the
    study's. The async source's own duty-cycle distortion is measured first
    by the chip (line_calibration_chip) and subtracted from every frame
    (compensate=False runs uncompensated). Pass: clean frames all good, and
    every value up to the study's pass limit minus 1 ns passes, every value
    from the theoretical limit (16.67 ns DCD, 8.33 ns alternating) plus 1 ns
    fails at least once (DCD on both signs)."""
    cal = line_calibration_chip(widths=(401.37, 433.13), bursts=30) if compensate else None
    comp = cal["dcd_ns_mean"] / 2 if cal else 0.0          # generator convention: highs longer by comp
    prog = _fixture("study_eth_rx.asm").words
    link = rtlsim.board_link()
    params = {"program": prog, "points": points or ETH_POINTS, "frames": frames, "seed": 20261122,
              "dcd_comp": comp}
    res = json.loads(link.value("farend ethsweep " + json.dumps(params)))
    summary, ok = {}, all(x["ok"] for x in res["sanity"])
    for mode in ("dcd", "dcd_neg", "alt", "uniform"):
        if mode == "dcd":
            pts = [p for p in res["points"] if p["mode"] == "dcd" and p["value_ns"] >= 0]
        elif mode == "dcd_neg":
            pts = [{**p, "value_ns": -p["value_ns"]} for p in res["points"]
                   if p["mode"] == "dcd" and p["value_ns"] < 0]
        else:
            pts = [p for p in res["points"] if p["mode"] == mode]
        if not pts:
            continue
        passing = [p["value_ns"] for p in pts if p["fails"] == 0]
        failing = [p["value_ns"] for p in pts if p["fails"] > 0]
        last_pass = max([v for v in passing if not failing or v < min(failing)], default=None)
        first_fail = min(failing, default=None)
        st = ETH_STUDY["dcd" if mode == "dcd_neg" else mode]
        summary[mode] = {"board_last_pass_ns": last_pass, "board_first_fail_ns": first_fail,
                         "study_last_pass_ns": st[0], "study_first_fail_ns": st[1],
                         "fails": {str(p["value_ns"]): p["fails"] for p in pts}}
        lim = st[0] - 1.0
        ok &= all(p["fails"] == 0 for p in pts if p["value_ns"] <= lim)
        if mode in ("dcd", "dcd_neg", "alt"):
            theory = 16.67 if mode != "alt" else 8.33
            ok &= all(p["fails"] > 0 for p in pts if p["value_ns"] >= theory + 1.0)
    return {"name": "board.eth_rx_sweep", "pass": ok, "frames_per_point": frames,
            "resolution_ps": 78.125, "source_dcd_compensation_ns": round(comp, 3),
            "source_calibration": cal, "summary": summary, "sanity": res["sanity"],
            "seconds": res["seconds"], "line_contention": res["line_contention"], "points": res["points"]}


def line_calibration(widths=(61.71, 65.37, 69.13), pulses=1500) -> dict:
    """The async line's pulse widths as the chip's input samples them. Pulses
    of a nominal width W (high and low alternately, so W is also the low
    width) are played into ui[3] and the trace gives each pulse's length in
    chip cycles: with the source asynchronous to the chip clock, a pulse of
    width W reads floor(W/T) or that + 1 cycles with the second probability
    frac(W/T), so the mean length x T measures W to about 0.1 ns over 1500
    pulses. high - low is the source's duty-cycle distortion at the chip's
    first flop (routing rise/fall asymmetry, toggle-flop skew); mean - W
    its width error. The widths are chosen so that a period is not a
    rational multiple of the chip clock with a small denominator (60, 65,
    70 ns would visit only 5 phases)."""
    from protolines import line_events
    tc = 1000.0 / 60.0
    out = []
    with rtlsim.Chip() as chip:
        chip.wr(rtlsim.XIN0, 3)
        chip.pull(3, 1)
        chip.link.value("lineroute 3")
        chip.dump()
        chip.link.value("freerun 1")
        for w in widths:
            ch = [(0.0, {0: (1, 1)})] + [(500.0 + k * w, {0: ((k + 1) & 1, 1)}) for k in range(2 * pulses)]
            chip.link.value("lineplay " + " ".join(f"{x:x}" for x in line_events(ch)))
            time.sleep(0.002)
            rows = chip.dump(full=True)
            e, lv = [], None
            for r in rows:
                v = (r[2] >> 3) & 1
                if v != lv:
                    e.append((r[0], v))
                    lv = v
            ln = [(b[0] - a[0], a[1]) for a, b in zip(e, e[1:])]
            ln = ln[2:-2]                                   # whole pulses only
            hi = [x for x, v in ln if v == 1]
            lo = [x for x, v in ln if v == 0]
            mh, ml = sum(hi) / len(hi) * tc, sum(lo) / len(lo) * tc
            out.append({"nominal_ns": w, "high_ns": round(mh, 3), "low_ns": round(ml, 3),
                        "dcd_ns": round(mh - ml, 3), "pulses": [len(hi), len(lo)],
                        "high_cycles": sorted(set(hi)), "low_cycles": sorted(set(lo))})
        chip.link.value("freerun 0")
    return {"name": "board.line_calibration", "points": out,
            "dcd_ns_mean": round(sum(p["dcd_ns"] for p in out) / len(out), 3)}


def line_calibration_chip(widths=(401.37, 417.71, 433.13), bursts=50, per_burst=30) -> dict:
    """The same measurement made by the chip itself: lane 0 runs RAW SAMPLE
    in a loop on in0 = ui[3] (each record = chip cycles from one input edge
    to the next, through the lane's own synchronisers and edge-interval
    counter), so it shows the duty-cycle distortion the receive lanes see.
    Pulses are wider (about 24-26 cycles) so the loop never misses an edge,
    and a burst's 2 x per_burst records fit the 64-word host-out queue."""
    from protolines import line_events
    from rosetta import asm
    tc = 1000.0 / 60.0
    prog = asm.assemble("loop:   RAW   SAMPLE HOSTQ, tag=1\n        BR    always, loop\n").words
    out = []
    with rtlsim.Chip() as chip:
        chip.load_program(prog)
        chip.wr(rtlsim.XIN0, 3)
        chip.pull(3, 1)
        chip.link.value("lineroute 3")
        chip.start_lane(0, 0, qcfg=0xF)
        chip.run(100)
        chip.link.value("freerun 1")
        for w in widths:
            hi, lo = [], []
            for b in range(bursts):
                ch = [(0.0, {0: (1, 1)})] + [(500.0 + k * w, {0: ((k + 1) & 1, 1)})
                                             for k in range(1, 2 * per_burst + 1)]
                chip.link.value("lineplay " + " ".join(f"{x:x}" for x in line_events(ch)))
                time.sleep(0.0005)
                recs = [r & 0xFFFF for r in chip.drain(0)]
                # records: idle (to the first fall, saturated), then low, high, ..., low
                if len(recs) != 2 * per_burst or recs[0] != 0xFFFF:
                    raise RuntimeError(f"calibration burst: {len(recs)} records {recs[:4]}")
                seq = recs[1:]
                lo += seq[0::2]
                hi += seq[1::2]
            mh, ml = sum(hi) / len(hi) * tc, sum(lo) / len(lo) * tc
            out.append({"nominal_ns": w, "high_ns": round(mh, 3), "low_ns": round(ml, 3),
                        "dcd_ns": round(mh - ml, 3), "pulses": [len(hi), len(lo)],
                        "high_cycles": sorted(set(hi)), "low_cycles": sorted(set(lo))})
        chip.link.value("freerun 0")
    return {"name": "board.line_calibration_chip", "points": out,
            "dcd_ns_mean": round(sum(p["dcd_ns"] for p in out) / len(out), 3)}


def _usb_events(states, ppm, release=True):
    """Async-line events for a host sending `states` (D-, D+) one per bit at
    1.5 Mbit/s x (1 + ppm), then releasing the bus one bit after the last."""
    from protolines import USB_TBIT
    bit = USB_TBIT * 1000 / 60 / (1.0 + ppm * 1e-6)     # ns
    ch, prev = [], None
    for k, (dm, dp) in enumerate(states):
        if (dm, dp) != prev:
            ch.append((k * bit, {0: (dm, 1), 1: (dp, 1)}))
            prev = (dm, dp)
    if release:
        ch.append((len(states) * bit, {0: (1, 0), 1: (0, 0)}))
    return ch


def usb_async(ppms=(0.0, 15000.0, -15000.0)) -> dict:
    """The lockstep USB scenario with the host on async lines (its own
    clock): NAK, DATA1 + ACK, SETUP + DATA0 -> ACK, silence for another
    address; turnaround measured on the trace from the host's SE0-to-J."""
    import random
    import test_usb_rtl as tu
    from protolines import (line_events, usb_data_packet, usb_line_states, usb_packet_bits,
                            usb_token, USB_TBIT as TBIT)
    out, ok = [], True
    for ppm in ppms:
        rng = random.Random(int(ppm) + 99)
        dev, lsn = tu.fixture("study_usb_dev.asm"), tu.fixture("study_usb_listen.asm", 128)
        r = {"ppm": ppm}
        with rtlsim.Chip() as chip:
            chip.load_program(dev.words + [NOP] * (128 - len(dev.words)) + lsn.words)
            chip.wr(rtlsim.XIN0, 8 | (9 << 4) | (8 << 16) | (9 << 20))
            chip.wr(rtlsim.XOUT1, 0x10 | (0x11 << 5))
            chip.pull(8, 1)
            chip.pull(9, 0)
            chip.link.value("lineroute 8 9")
            chip.mem_write(0, 16, [tu.ADDR])
            chip.mem_write(0, 20, [0])
            chip.start_lane(0, 0, qcfg=0)
            chip.start_lane(1, 128, qcfg=0xC)
            chip.run(400)
            chip.dump()
            chip.link.value("freerun 1")

            def send(states_list, gap_bits=4):
                ch, t = [], 0.0
                bit = TBIT * 1000 / 60 / (1.0 + ppm * 1e-6)
                for i, st in enumerate(states_list):
                    for tt, upd in _usb_events(st, ppm):
                        ch.append((t + tt, upd))
                    t += (len(st) + gap_bits) * bit
                chip.link.value("lineplay " + " ".join(f"{w:x}" for w in line_events(ch)))
                time.sleep(0.002)
                return chip.dump(full=True)

            def host_eop_end(rows, before):
                """The end of the host's EOP before cycle `before`: the last SE0 -> J
                whose SE0 lasted at least half a bit. An EOP is two bit times of SE0.
                Shorter both-low runs are not EOPs: the asynchronous host's two line
                drivers are released one bit time after the EOP, and on some FPGA
                builds that release shows as one chip cycle of both lines low (found
                22 Nov: it made this measurement read 44-47 cycles instead of 82-87
                while the device answered at the right time). They are counted in
                r["short_se0"] so the harness artifact stays visible."""
                ser = tu.bus_series(rows)
                ends, start = [], None
                for (c, dm, dp), (pc, pdm, pdp) in zip(ser[1:], ser):
                    if (pdm, pdp) != tu.SE0 and (dm, dp) == tu.SE0:
                        start = c
                    if (pdm, pdp) == tu.SE0 and (dm, dp) != tu.SE0:
                        length = c - (start if start is not None else pc)
                        if length < TBIT // 2:
                            r["short_se0"] = r.get("short_se0", 0) + 1
                        elif (dm, dp) == tu.J and c < before:
                            ends.append(c)
                        start = None
                return ends[-1] if ends else None

            def reply(rows):
                d = tu.decode_device(rows, rows[0][0] if rows else 0)
                if not d:
                    return None
                e = host_eop_end(rows, d[0])
                return {"turn": d[0] - e if e else None, "parsed": tu.parse(d[1]), "se0": d[3], "eop_end": d[2]}

            rows = send([usb_line_states(usb_token(0x9, tu.ADDR, 0))])
            r["in_nak"] = reply(rows)
            payload = bytes(rng.randrange(256) for _ in range(8))
            chip.mem_write(0, 21, list(payload))
            chip.mem_write(0, 20, [0x8000 | (8 << 8) | 0x4B])
            rows = send([usb_line_states(usb_token(0x9, tu.ADDR, 0))])
            r["in_data"] = reply(rows)
            send([usb_line_states(usb_packet_bits(0x2))])                  # host ACK
            r["in_data_records"] = chip.drain(0, out_size=8)
            r["mailbox"] = chip.mem_read(0, 20, 1)[0]
            setup = bytes(rng.randrange(256) for _ in range(8))
            rows = send([usb_line_states(usb_token(0xD, tu.ADDR, 0)),
                         usb_line_states(usb_data_packet(0x3, setup))])
            r["setup"] = reply(rows)
            recs = chip.drain(0, out_size=8)
            n = [(x & 0xFFFF) - 32 for x in recs if x >> 24 == 0xA3]
            r["setup_rx_ok"] = bool(n) and bytes(w & 0xFF for w in chip.mem_read(0, 32, n[0]))[:8] == setup
            r["setup_records"] = [hex(x) for x in recs]
            rows = send([usb_line_states(usb_token(0x9, tu.ADDR + 1, 0))])
            r["other_silent"] = tu.decode_device(rows, 0) is None
            r["listener_packets"] = len(tu.listener_packets(chip))
            r["line_contention"] = int(chip.link.value("linestat").split()[2].split("=")[1])
            chip.link.value("freerun 0")
        good = (r["in_nak"] and r["in_nak"]["parsed"] == {"pid": 0x5A}
                and r["in_data"] and r["in_data"]["parsed"].get("pid") == 0x4B
                and r["in_data"]["parsed"].get("crc_ok") and r["in_data"]["parsed"].get("data") == payload
                and r["in_data_records"] == [0xA40000D2] and r["mailbox"] == 0
                and r["setup"] and r["setup"]["parsed"] == {"pid": 0xD2} and r["setup_rx_ok"]
                and r["other_silent"] and r["line_contention"] == 0)
        turns = [x["turn"] for x in (r["in_nak"], r["in_data"], r["setup"]) if x]
        good = bool(good) and len(turns) == 3 and all(t is not None and 2 * TBIT <= t <= 7.5 * TBIT for t in turns)
        r["turnarounds"] = turns
        r["pass"] = good
        for k in ("in_nak", "in_data", "setup"):
            if r.get(k) and r[k].get("parsed", {}).get("data") is not None:
                r[k]["parsed"]["data"] = r[k]["parsed"]["data"].hex()
        r.pop("in_data_records", None)
        ok &= good
        out.append(r)
    return {"name": "board.usb_async", "pass": ok, "runs": out}


def can_async(ppms=(5000.0, -5000.0), kbps=500, loop=8, kd=40) -> dict:
    """The lockstep CAN scenario with the external node on async line 0
    (its own clock, rate offset ppm): the bus must carry 0x700, 0x0F0,
    0x123 in order, all acknowledged, with the same records."""
    import test_can_rtl as tc
    from protolines import can_decode_bus, can_wire, line_events
    out, ok = [], True
    for ppm in ppms:
        tbit, p = tc.programs(kbps, kd)
        image = [NOP] * 256
        for prog in p.values():
            image[prog.origin:prog.origin + len(prog.words)] = prog.words
        with rtlsim.Chip() as chip:
            chip.load_program(image)
            chip.wr(rtlsim.XIN0, 3 | (8 << 4) | (3 << 16))
            chip.wr(rtlsim.XIN1, 4 | (10 << 4) | (4 << 16))
            chip.wr(rtlsim.XOUT1, 0x10 | (0x14 << 5) | (0x18 << 10) | (0x1C << 15))
            chip.canbus(loop // 2, loop - loop // 2)
            chip.link.value("lineroute 16")
            for lane, org in ((1, 64), (3, 192), (0, 0), (2, 128)):
                chip.start_lane(lane, org, qcfg=0xF)
            chip.run(200)
            for lane, ident, data in ((0, tc.ID_A, tc.DATA_A), (2, tc.ID_B, tc.DATA_B)):
                chip.mem_write(lane, 7, [len(data)] + list(data))
                chip.push(lane, [ident << 1, len(data)])
            chip.run(2000)
            chip.dump()
            chip.link.value("freerun 1")
            bits = can_wire(tc.ID_X, tc.DATA_X)
            bit_ns = tbit * 1000 / 60 / (1.0 + ppm * 1e-6)
            ch, prev = [], None
            for k, b in enumerate(bits):
                if b != prev:
                    ch.append((k * bit_ns, {0: (0, 1) if b == 0 else (1, 0)}))
                    prev = b
            chip.link.value("lineplay " + " ".join(f"{w:x}" for w in line_events(ch)))
            time.sleep(0.01)
            rows = chip.dump(full=True)
            end = chip.cycle()
            chip.link.value("freerun 0")
            recs = {k: chip.drain(k) for k in range(4)}
        bus = can_decode_bus(rows, tbit, pin=3, end_t=end, sample=(tbit * 3) // 4)
        r = {"ppm": ppm, "bus": [(hex(f["id"]), f["crc_ok"], f["ack_slot"], f["ack_delim"]) for f in bus],
             "records": {k: [hex(w) for w in v] for k, v in recs.items()}}
        r["pass"] = ([(f["id"], f["data"], f["crc_ok"], f["ack_slot"]) for f in bus]
                     == [(tc.ID_X, tc.DATA_X, True, 0), (tc.ID_B, tc.DATA_B, True, 0), (tc.ID_A, tc.DATA_A, True, 0)]
                     and recs[0] == [0xC1000001, 0xC0000000] and recs[2] == [0xC0000000])
        ok &= r["pass"]
        out.append(r)
    return {"name": "board.can_async", "pass": ok, "runs": out}


if __name__ == "__main__":
    print(json.dumps(clock(), indent=1))
    r = uart_async_sweep()
    print(json.dumps({k: v for k, v in r.items() if k != "points"}, indent=1))
    for p in r["points"]:
        print(f"{p['offset_pct']:+5.1f} %  {p['rate_set']:10.1f} Bd  T={p['bit_cycles_measured']}  "
              f"rx={'ok' if p['receiver_ok'] else 'FAIL'} listener={'ok' if p['listener_ok'] else 'FAIL'}")
