"""Hardware demonstration / self-test of the Rosetta harness with the TT
template placeholder (uo_out = ui_in + uio_in). Needs nothing wired to the board.

    sudo env XILINX_XRT=/usr PYTHONDONTWRITEBYTECODE=1 /usr/local/share/pynq-venv/bin/python demo.py [--no-download]

Writes results/demo_results.json and results/*.vcd next to this file.
"""
import json
import os
import random
import sys
import time

import numpy as np

from rosetta_board import load

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results")
os.makedirs(OUT, exist_ok=True)
R = {}


def section(title):
    print("\n=== " + title + " " + "=" * max(0, 60 - len(title)))


def placeholder_expected(ui, uio):
    return (ui + uio) & 0xFF


def main():
    t0 = time.time()
    b = load(download="--no-download" not in sys.argv)
    R["load_s"] = round(time.time() - t0, 2)
    st = b.status()
    section("load")
    print(f"overlay {b.bitfile} loaded in {R['load_s']} s; status {st}")
    R["status_after_load"] = st

    # ------------------------------------------------------------ adder check
    section("uo_out == (ui_in + uio_in) & 0xFF, all 8 ui bits from the processor")
    b.host_mode("gpio")
    picked = [(0x00, 0x00), (0x01, 0x01), (0x35, 0x10), (0x7F, 0x01), (0xFF, 0x01),
              (0x80, 0x80), (0xA5, 0x5A), (0xC3, 0x3D), (0x12, 0x34), (0xFE, 0xFF)]
    rows = []
    print("   ui   uio | chip sees ui uio | uo_out expect | uio_out uio_oe")
    for ui, uio in picked:
        b.set_inputs(ui, uio)
        seen_ui, seen_uio, uo = b.get_ui(), b.get_uio(), b.get_uo()
        exp = placeholder_expected(ui, uio)
        ok = uo == exp and seen_ui == ui and seen_uio == uio
        rows.append(dict(ui=ui, uio=uio, seen_ui=seen_ui, seen_uio=seen_uio, uo=uo,
                         expected=exp, uio_out=b.get_uio_out(), uio_oe=b.get_uio_oe(), ok=ok))
        print(f"   {ui:02x}   {uio:02x}  |      {seen_ui:02x}  {seen_uio:02x}      |"
              f"   {uo:02x}    {exp:02x}   |   {rows[-1]['uio_out']:02x}     {rows[-1]['uio_oe']:02x}"
              f"   {'OK' if ok else 'MISMATCH'}")
    rnd = random.Random(727)
    n_rand, n_ok = 1000, 0
    for _ in range(n_rand):
        ui, uio = rnd.randrange(256), rnd.randrange(256)
        b.set_inputs(ui, uio)
        n_ok += b.get_uo() == placeholder_expected(ui, uio)
    print(f"   random vectors: {n_ok}/{n_rand} correct")
    R["adder"] = dict(picked=rows, random_total=n_rand, random_ok=n_ok)

    # ------------------------------------------------------------ loopback
    section("internal loopback (registered, +1 cycle)")
    lb = []
    b.set_inputs(0x01, 0x01)
    b.loopback("ui7", "uo1")                 # uo1 -> ui7
    time.sleep(1e-4)
    lb.append(dict(route="uo1->ui7", drive=(0x01, 0x01), ui_seen=b.get_ui(), uo=b.get_uo(),
                   expect_ui=0x81, expect_uo=0x82))
    b.route("ui7", "cpu")
    b.set_inputs(0x02, 0x00)
    b.loopback("uio0", "uo1")                # uo1 -> uio_in[0]
    time.sleep(1e-4)
    lb.append(dict(route="uo1->uio0", drive=(0x02, 0x00), uio_seen=b.get_uio(), uo=b.get_uo(),
                   expect_uio=0x01, expect_uo=0x03))
    b.route("uio0", "cpu")
    lb[0]["ok"] = lb[0]["ui_seen"] == 0x81 and lb[0]["uo"] == 0x82
    lb[1]["ok"] = lb[1]["uio_seen"] == 0x01 and lb[1]["uo"] == 0x03
    for e in lb:
        print("  ", {k: (hex(v) if isinstance(v, int) and not isinstance(v, bool) else v)
                     for k, v in e.items()})
    R["loopback"] = lb

    # loopback oscillator: uo0 -> ui0 with uio=1: ui0 toggles every chip cycle
    b.set_inputs(0x00, 0x01)
    b.loopback("ui0", "uo0")
    cap = b.capture(64, trigger=None)
    b.route("ui0", "cpu")
    ui0 = cap.bit("ui0")
    toggles_every_cycle = bool(np.all(np.diff(ui0.astype(int)) != 0))
    print(f"   oscillator uo0->ui0, uio=1: ui0 = {''.join(map(str, ui0[:24]))}...  "
          f"uo = {' '.join(f'{v:02x}' for v in cap.uo[:8])}  toggles every cycle: {toggles_every_cycle}")
    R["loopback_oscillator"] = dict(ui0=ui0[:32].tolist(), uo=cap.uo[:16].tolist(),
                                    toggles_every_cycle=toggles_every_cycle)
    cap.to_vcd(os.path.join(OUT, "loopback_oscillator.vcd"))

    # ------------------------------------------------------------ capture of processor writes
    section("capture: processor writes to ui (trigger ui:change, 16 pre-trigger samples)")
    seq = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0xFF, 0x00]
    b.set_inputs(0x00, 0x03)

    def writes():
        for v in seq:
            b.set_ui(v)
    cap = b.capture(16384, trigger="ui:change", pre=16, action=writes)
    ch = cap.changes()
    print(cap.table(limit=14))
    ok = all(((ui + 0x03) & 0xFF) == uo for _, _, ui, uo, _, _ in ch[1:])
    seen = [c[2] for c in ch]
    print(f"   ui values seen in order: {[hex(v) for v in seen]}; uo==ui+3 at every change: {ok}")
    gaps = np.diff([c[0] for c in ch[1:]])
    R["capture_writes"] = dict(changes=ch, uo_follows=ok, ui_sequence=seen,
                               cycles_between_writes=gaps.tolist())
    cap.to_vcd(os.path.join(OUT, "capture_writes.vcd"))

    # ------------------------------------------------------------ clock gating / single step
    section("single-step: clock stopped, 8 steps, capture sees exactly one sample per step")
    b.clock_run(False)
    c0 = b.cycles()
    time.sleep(0.01)
    still = b.cycles() == c0
    b.set_inputs(0x10, 0x00)
    # arm (immediate trigger, 8 samples) while stopped, then step with new inputs each time
    b.gpio["trig"].write(0, 0)
    b.gpio["cap"].write(0, 8)
    ctrl = b._ctrl & ~0xFFFF0360
    b._set_ctrl(ctrl)
    b._set_ctrl(ctrl | (1 << 5))
    for k in range(8):
        b.set_ui(0x10 + k)
        b.step(1)
    done = bool(b.status_word() & (1 << 3))
    trig = b.gpio["cap"].read(8) & 0x3FFF
    raw = np.array(b.bram.array[:16384], dtype=np.uint32)[(trig + np.arange(8)) % 16384]
    b._set_ctrl(ctrl)
    steps = (b.cycles() - c0) & 0xFFFFFFFF
    b.clock_run(True)
    ui_steps = [(int(r) & 0xFF) for r in raw]
    uo_steps = [(int(r) >> 8) & 0xFF for r in raw]
    print(f"   counter frozen while stopped: {still}; edges given: {steps}; capture done: {done}")
    print(f"   captured ui per step: {[hex(v) for v in ui_steps]}  uo: {[hex(v) for v in uo_steps]}")
    R["single_step"] = dict(counter_frozen=still, edges=steps, capture_done=done,
                            ui=ui_steps, uo=uo_steps)

    # ------------------------------------------------------------ host SPI
    section("host SPI via AXI Quad SPI (ui0=SCK ui1=CS_n ui2=MOSI, uo0=MISO)")
    b.host_mode("spi")
    b.set_inputs(0x00, 0x00)
    tx = bytes([0xA5, 0x3C, 0x00, 0xFF])
    res = {}

    def xfer():
        res["rx"] = b.spi_transfer(tx)
    cap = b.capture(16384, trigger="cs:fall", pre=32, action=xfer)
    sck_r = cap.edges("sck", "rise")
    periods = np.diff(sck_r)
    cs_lo = np.nonzero(cap.bit("cs") == 0)[0]
    mosi_bits = cap.bit("mosi")[sck_r]        # mode 0: sampled on SCK rising edge
    decoded = bytes(int("".join(map(str, mosi_bits[i:i + 8])), 2) for i in range(0, len(mosi_bits) - 7, 8))
    miso_bits = cap.bit("miso")[sck_r]
    f_sck = 60e6 / np.median(periods) if len(periods) else 0
    print(f"   tx {tx.hex()}  rx {res['rx'].hex()}  (placeholder: MISO=uo0=ui0^uio0 = SCK)")
    print(f"   capture: {len(sck_r)} SCK rising edges, period {np.median(periods):.0f} cycles "
          f"-> SCK {f_sck / 1e6:.3f} MHz; CS low for {len(cs_lo)} cycles; "
          f"MOSI decoded from capture: {decoded.hex()}")
    R["spi"] = dict(tx=tx.hex(), rx=res["rx"].hex(), sck_edges=len(sck_r),
                    sck_period_cycles=float(np.median(periods)) if len(periods) else None,
                    sck_hz=f_sck, cs_low_cycles=int(len(cs_lo)), mosi_from_capture=decoded.hex(),
                    miso_at_sck_rise=miso_bits.tolist()[:16])
    cap.to_vcd(os.path.join(OUT, "spi_transfer.vcd"))
    print(cap.table(limit=12))
    b.host_mode("gpio")
    rx_bb = b.spi_transfer_bitbang(tx)
    b.host_mode("spi")
    print(f"   GPIO bit-bang fallback: tx {tx.hex()} rx {rx_bb.hex()} "
          "(placeholder MISO=SCK, sampled after the leading edge -> 1s)")
    R["spi"]["bitbang_rx"] = rx_bb.hex()

    # ------------------------------------------------------------ header pads (FPGA side)
    section("header pad read-back (nothing wired)")
    b.host_mode("gpio")
    b.set_inputs(0x5A, 0x00)
    p_off = b.get_pads()
    b.headers(True)
    time.sleep(1e-3)
    p_on = b.get_pads()
    b.headers(False)
    print(f"   headers off: {p_off}   headers on: {p_on}  (uo=0x5a, uio_oe=0 -> JB stays input)")
    R["pads"] = dict(off=p_off, on=p_on, uo=b.get_uo())

    # ------------------------------------------------------------ clock frequency
    section("chip clock frequency")
    m = b.measure_clock(2.0)
    from pynq import Clocks
    m["pynq_fclk0_mhz"] = Clocks.fclk0_mhz
    print(f"   software (counter vs CLOCK_MONOTONIC, {m['sw_window_s']:.3f} s): {m['sw_hz'] / 1e6:.6f} MHz")
    print(f"   hw meter vs 125 MHz board oscillator: {m['sys_hz']} edges/s")
    print(f"   hw meter vs PS FCLK0 (PYNQ reports {Clocks.fclk0_mhz} MHz): {m['fclk_hz']} edges/s")
    R["clock"] = m

    b.host_mode("spi")
    with open(os.path.join(OUT, "demo_results.json"), "w") as f:
        json.dump(R, f, indent=1, default=lambda o: o.hex() if isinstance(o, bytes) else str(o))
    # run under sudo: hand the results back to the invoking user
    uid, gid = os.environ.get("SUDO_UID"), os.environ.get("SUDO_GID")
    if uid and gid:
        for root, dirs, files in os.walk(OUT):
            for n in [root] + [os.path.join(root, f) for f in files]:
                os.chown(n, int(uid), int(gid))
    print(f"\nresults written to {OUT}")


if __name__ == "__main__":
    main()
