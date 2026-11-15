"""Command line for the compiler (rosetta/compiler.py).

    cd toolchain
    python3 -m rosetta.compile ../programs/uart/uart.ros --role receiver -p parity=even
    python3 -m rosetta.compile ../programs/uart/uart.ros --role sender -p baud=9600 \\
        --origin 32 -o /tmp/out

Prints the listing (address, word, source) and the host view. With -o DIR it
also writes PROTOCOL_ROLE.asm and PROTOCOL_ROLE.json (words, mode registers,
manifest). --json prints that JSON instead of the listing.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

from .compiler import CLOCK_HZ, CompileError, compile_file
from .lang import ParseError


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python3 -m rosetta.compile",
                                 description="Compile a protocol description for one role.")
    ap.add_argument("file", help="description file (.ros)")
    ap.add_argument("--role", required=True, help="a party of the protocol, or listener")
    ap.add_argument("-p", "--param", action="append", default=[], metavar="NAME=VALUE",
                    help="parameter override, e.g. baud=9600 or parity=even (repeatable)")
    ap.add_argument("--origin", type=lambda s: int(s, 0), default=0,
                    help="program memory address of the first word (default 0)")
    ap.add_argument("--clock-hz", type=int, default=CLOCK_HZ, help="chip clock (default 60 MHz)")
    ap.add_argument("--listen-wire", help="listener of a clock-wire protocol: the data wire this "
                                          "lane samples (gap G5: one lane per data wire)")
    ap.add_argument("-o", "--out-dir", help="write PROTOCOL_ROLE.asm and .json here")
    ap.add_argument("--json", action="store_true", help="print the JSON instead of the listing")
    args = ap.parse_args(argv)

    params = {}
    for p in args.param:
        name, sep, value = p.partition("=")
        if not sep or not name.strip():
            ap.error(f"parameter {p!r} is not NAME=VALUE")
        params[name.strip()] = value.strip()
    try:
        c = compile_file(args.file, args.role, params, clock_hz=args.clock_hz, origin=args.origin,
                         listen_wire=args.listen_wire)
    except (CompileError, ParseError, OSError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    data = c.to_json()
    if args.out_dir:
        os.makedirs(args.out_dir, exist_ok=True)
        stem = os.path.join(args.out_dir, f"{c.protocol}_{c.role}")
        with open(stem + ".asm", "w", encoding="utf-8") as f:
            f.write(c.asm)
        with open(stem + ".json", "w", encoding="utf-8") as f:
            json.dump(data, f, indent=1)
            f.write("\n")
    if args.json:
        print(json.dumps(data, indent=1))
        return 0
    m = c.manifest
    t = m["timing"]
    print(f"; {c.protocol} {c.role}  " + " ".join(f"{k}={v['value'] if isinstance(v, dict) else v}"
                                                   for k, v in c.params.items()))
    print("\n".join(c.listing))
    print("; mode registers: " + " ".join(f"{k}={v}" for k, v in m["mode_registers"].items()))
    if "bit_cycles" in t:                       # clock-rate form
        print(f"; timing: {t['tick']} PERIOD {t['PERIOD']}"
              + (f" PERIOD2 {t['PERIOD2']}" if "PERIOD2" in t else "")
              + f", bit {t['bit_cycles']:g} cycles for a nominal {t['nominal_bit_cycles']} "
                f"({t['rate_error_percent']:+g} %, tolerance {t['tolerance_percent']:g} %)")
    elif "pin_waits_periods" in t or "fastest_rate_hz_schedule" in t:     # clock wire, SPI shape
        own = (f" PERIOD {t['PERIOD']} PERIOD2 {t['PERIOD2']}, clock {t['clock_cycles']} cycles "
               f"= {t['rate_hz']:g} Hz, PIN waits {t['pin_waits_periods']}" if "PERIOD" in t else "")
        print(f"; timing: {t['tick']} on {t['clock']}, capture {t['capture']}{own}; "
              f"schedule fits up to {t['fastest_rate_hz_schedule']} Hz")
    else:                                       # clock wire, general shape
        own = (f" PERIOD {t['PERIOD']} PERIOD2 {t['PERIOD2']}: low {t['low_cycles']}, high "
               f"{t['high_cycles']} cycles = {t['rate_hz']:g} Hz" if "PERIOD2" in t else
               f" PERIOD {t['PERIOD']}" if "PERIOD" in t else "")
        k = f", confirm window {t['confirm_window_cycles']}" if t.get("confirm_window_cycles") else ""
        print(f"; timing: {t['tick']} on {t['clock']}, capture {t['capture']}{own}{k}; "
              f"fastest rate {t['fastest_rate_hz']} Hz")
    for w in m["in_queue"]:
        if "field" in w or "item" in w:
            what = w.get("field") or w.get("item")
            print(f"; in-queue word: {what}" + (f" ({w['width']} bits)" if "width" in w else "")
                  + (f", default {w['default']}" if "default" in w else ""))
        else:
            print(f"; in-queue: {json.dumps(w)}")
    if m["out_queue"]:
        oq = m["out_queue"]
        print(f"; out-queue: {oq['words_per_record']} word(s) per record, {oq['layout']}")
        for tag, d in oq["tags"].items():
            if d["kind"] == "event":
                print(f";   tag {tag}: event {d['event']}")
                continue
            if d.get("names"):
                print(f";   tag {tag}: {d['kind']} {d['field']}[{d['width']}] {d['names']}")
                continue
            extra = f" = {d['expected']}, then {d['then']}" if d["kind"] == "error" else ""
            print(f";   tag {tag}: {d['kind']} {d['field']}[{d['width']}]{extra}")
    for chk in m["schedule"]:
        print(f"; schedule: {chk['check']}: {chk['need_cycles']} of {chk['available_cycles']} cycles")
    return 0


if __name__ == "__main__":
    sys.exit(main())
