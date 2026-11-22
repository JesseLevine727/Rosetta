#!/usr/bin/env python3
"""FPGA regression gate for the Rosetta chip: one command.

    ./regress.py [--repo DIR] [--force-build] [--no-build] [--keep N]

1. Builds the bitstream if the RTL (or the harness) changed since the last
   build: build.sh <repo>/src --define ROSETTA_SRAM_BEHAV. Fails on negative
   slack.
2. Deploys bitstream and board software to the board if they differ.
3. Runs the whole toolchain test suite on Verilator (python3 -m unittest
   discover -s toolchain/tests; the compiled programs are what the tests
   compile), recording a transcript of every chip session.
4. Runs every chip scenario (toolchain/tests/test_*_rtl.py: compiled UART,
   SPI, I2C in all roles, and the targeted block tests) on the board in
   lockstep mode, recording transcripts.
5. Compares the two backends' transcripts session by session: every
   command's reply, including every pin change with its chip cycle, must be
   identical.
6. Board-only checks: the chip clock frequency, and the UART receiver
   against an asynchronous far end swept in rate (board_tests.py).
7. Writes results/regress_<date>_<time>/: summary.json, summary.md,
   coverage.md / coverage.json, the test JSONs, transcripts of failing or
   differing sessions, and VCDs of their pin changes.

Exit status 0 only if everything passed. README "Regression gate".
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
BOARD = os.environ.get("ROSETTA_BOARD", "xilinx@10.0.0.82")
BOARD_DIR = os.environ.get("ROSETTA_BOARD_DIR", "/home/xilinx/rosetta/regress")
DEFINES = ["ROSETTA_SRAM_BEHAV"]
DEPLOY = {"rosetta.bit": os.path.join(OUT, "rosetta.bit"), "rosetta.hwh": os.path.join(OUT, "rosetta.hwh"),
          "rosetta_board.py": os.path.join(HERE, "python", "rosetta_board.py"),
          "lockstep.py": os.path.join(HERE, "python", "lockstep.py"),
          "board_server.py": os.path.join(HERE, "python", "board_server.py")}
CLK_PS = 16667                      # one chip cycle in the VCDs


def log(msg):
    print(f"[regress {dt.datetime.now():%H:%M:%S}] {msg}", flush=True)


def sha256(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


# ------------------------------------------------------------------ build

def chip_sources(rtl):
    r = subprocess.run([os.path.join(HERE, "scripts", "chip_sources.sh"), rtl],
                       capture_output=True, text=True, check=True)
    return [l for l in r.stdout.split() if l]


def build_key(rtl):
    h = hashlib.sha256()
    files = chip_sources(rtl)
    for sub in ("hdl", "hdl/templates", "tcl", "xdc"):
        d = os.path.join(HERE, sub)
        files += sorted(os.path.join(d, f) for f in os.listdir(d)
                        if os.path.isfile(os.path.join(d, f)))
    files.append(os.path.join(HERE, "build.sh"))
    for f in files:
        h.update(os.path.basename(f).encode())
        with open(f, "rb") as fh:
            h.update(fh.read())
    h.update(" ".join(DEFINES).encode())
    return h.hexdigest()


def ensure_build(rtl, force, no_build):
    key = build_key(rtl)
    kf = os.path.join(OUT, "build_key.txt")
    have = open(kf).read().strip() if os.path.exists(kf) else ""
    bit = os.path.join(OUT, "rosetta.bit")
    built = False
    if force or have != key or not os.path.exists(bit):
        if no_build:
            raise SystemExit("bitstream out of date and --no-build given")
        log("RTL or harness changed: building (about 8 min)")
        t0 = time.time()
        cmd = [os.path.join(HERE, "build.sh"), rtl, "--jobs", "8"]
        for d in DEFINES:
            cmd += ["--define", d]
        with open(os.path.join(OUT, "build_console.txt"), "w") as f:
            r = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT)
        if r.returncode != 0:
            raise SystemExit(f"build failed (see {OUT}/build_console.txt)")
        with open(kf, "w") as f:
            f.write(key + "\n")
        built = True
        log(f"built in {time.time() - t0:.0f} s")
    else:
        log("bitstream up to date with the RTL and harness")
    return {"built_now": built, "key": key} | build_report()


def build_report():
    rep = {}
    info = os.path.join(OUT, "build_info.txt")
    if os.path.exists(info):
        for line in open(info):
            if line.startswith(("rtl_git:", "rtl_dir:", "date:", "defines:", "top:")):
                k, v = line.split(":", 1)
                rep[k] = v.strip()
    slack = {}
    p = os.path.join(OUT, "reports", "clock_slack.txt")
    if os.path.exists(p):
        for line in open(p):
            m = re.match(r"(\S+)\s+period=(\S+)\s+WNS=(\S+)\s+WHS=(\S+)", line)
            if m:
                slack[m.group(1)] = {"period_ns": m.group(2), "wns_ns": m.group(3), "whs_ns": m.group(4)}
    rep["clock_slack"] = slack
    p = os.path.join(OUT, "reports", "timing_summary.rpt")
    if os.path.exists(p):
        txt = open(p).read()
        m = re.search(r"WNS\(ns\).*?\n\s*-+.*?\n\s*(-?[\d.]+)\s+(-?[\d.]+)\s+(\d+)\s+(\d+)\s+(-?[\d.]+)", txt, re.S)
        if m:
            rep["design_wns_ns"], rep["design_whs_ns"] = float(m.group(1)), float(m.group(5))
        rep["all_constraints_met"] = "All user specified timing constraints are met" in txt
    p = os.path.join(OUT, "reports", "utilization.rpt")
    util = {}
    if os.path.exists(p):
        for line in open(p):
            m = re.match(r"\|\s*(Slice LUTs|Slice Registers|Block RAM Tile|DSPs|BUFGCTRL|MMCME2_ADV)\s*\|"
                         r"\s*([\d.]+)\s*\|\s*[\d.]+\s*\|\s*[\d.]+\s*\|\s*([\d.]+)\s*\|\s*([\d.<]+)\s*\|", line)
            if m and m.group(1) not in util:
                util[m.group(1)] = {"used": m.group(2), "available": m.group(3), "pct": m.group(4)}
    rep["utilization"] = util
    p = os.path.join(OUT, "reports", "utilization_hier.rpt")
    if os.path.exists(p):
        for line in open(p):
            if re.match(r"\|\s+harness\s", line):
                cols = [c.strip() for c in line.strip().strip("|").split("|")]
                rep["harness_and_chip"] = {"luts": cols[2], "ffs": cols[6], "ramb36": cols[7], "ramb18": cols[8]}
                break
    rep["bit_sha256"] = sha256(os.path.join(OUT, "rosetta.bit")) if os.path.exists(os.path.join(OUT, "rosetta.bit")) else None
    return rep


# ------------------------------------------------------------------ board

def ssh(cmd, timeout=60):
    return subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", BOARD, cmd],
                          capture_output=True, text=True, timeout=timeout)


def deploy():
    r = ssh(f"mkdir -p {BOARD_DIR} && cd {BOARD_DIR} && sha256sum {' '.join(DEPLOY)} 2>/dev/null; true")
    if r.returncode != 0:
        raise SystemExit(f"board {BOARD} not reachable: {r.stderr.strip()}")
    remote = {l.split()[1]: l.split()[0] for l in r.stdout.splitlines() if len(l.split()) == 2}
    todo = [n for n, p in DEPLOY.items() if remote.get(n) != sha256(p)]
    if todo:
        subprocess.run(["scp", "-q", *[DEPLOY[n] for n in todo], f"{BOARD}:{BOARD_DIR}/"], check=True)
        log(f"deployed {', '.join(todo)}")
    else:
        log("board files up to date")
    return todo


# ------------------------------------------------------------------ tests

def run_suite(repo, backend, pattern, tdir, out_json):
    env = dict(os.environ, ROSETTA_BACKEND=backend, ROSETTA_TRANSCRIPT=tdir)
    t0 = time.time()
    with open(out_json + ".log", "w") as f:
        subprocess.run([sys.executable, os.path.join(HERE, "run_tests.py"), repo, pattern, out_json],
                       env=env, stdout=f, stderr=subprocess.STDOUT)
    if not os.path.exists(out_json):
        return {"backend": backend, "tests": [], "counts": {}, "crashed": True,
                "seconds": round(time.time() - t0, 1)}
    return json.load(open(out_json))


def compare_transcripts(vdir, bdir):
    out = []
    names = sorted(set(os.listdir(vdir)) | set(os.listdir(bdir)))
    for n in names:
        a, b = os.path.join(vdir, n), os.path.join(bdir, n)
        if not os.path.exists(a) or not os.path.exists(b):
            out.append({"session": n, "identical": False,
                        "why": "missing on " + ("board" if os.path.exists(a) else "verilator")})
            continue
        la, lb = open(a).read().splitlines(), open(b).read().splitlines()
        if la == lb:
            out.append({"session": n, "identical": True, "lines": len(la),
                        "changes": sum(1 for l in la if l[:1].isdigit() and len(l.split()) == 6)})
            continue
        i = next((k for k, (x, y) in enumerate(zip(la, lb)) if x != y), min(len(la), len(lb)))
        cmd = next((la[k] for k in range(i, -1, -1) if k < len(la) and la[k].startswith("> ")), "")
        out.append({"session": n, "identical": False, "first_diff_line": i + 1, "command": cmd,
                    "verilator": la[i] if i < len(la) else "<end>",
                    "board": lb[i] if i < len(lb) else "<end>"})
    return out


def transcript_dumps(path):
    """All pin-change rows of a transcript's D replies."""
    rows, in_d = [], False
    for line in open(path):
        line = line.strip()
        if line == "> D":
            in_d = True
        elif line.startswith("> "):
            in_d = False
        elif in_d and line != "end":
            p = line.split()
            if len(p) == 6:
                rows.append((int(p[0]), *(int(x, 16) for x in p[1:])))
    return rows


def to_vcd(rows, path, title):
    sigs = [("uo_out", 1, 8), ("ui_in", 2, 8), ("uio_out", 3, 8), ("uio_oe", 4, 8), ("uio_in", 5, 8)]
    codes = {}
    with open(path, "w") as f:
        f.write(f"$comment {title} $end\n$timescale 1ps $end\n$scope module rosetta $end\n")
        k = 0
        for name, _, w in sigs:
            codes[name] = chr(33 + k)
            k += 1
            f.write(f"$var wire {w} {codes[name]} {name} $end\n")
            for b in range(8):
                codes[f"{name}{b}"] = chr(33 + k)
                k += 1
                f.write(f"$var wire 1 {codes[f'{name}{b}']} {name}_{b} $end\n")
        f.write("$upscope $end\n$enddefinitions $end\n")
        prev = None
        for r in rows:
            if prev is not None and r[0] <= prev[0]:          # a new dump restarts nothing: keep order
                pass
            f.write(f"#{r[0] * CLK_PS}\n")
            for name, col, _ in sigs:
                if prev is None or prev[col] != r[col]:
                    f.write(f"b{r[col]:08b} {codes[name]}\n")
                    for b in range(8):
                        f.write(f"{(r[col] >> b) & 1}{codes[f'{name}{b}']}\n")
            prev = r


# ------------------------------------------------------------------ coverage

def decode_session(path, isa):
    """Features a session's loaded programs and host operations configure."""
    prog = {}
    maddr = 0
    pcs, started = {}, set()
    host = set()
    nets = skews = faults = 0
    for line in open(path):
        if not line.startswith("> "):
            continue
        p = line[2:].split()
        c = p[0]
        if c == "W":
            a, d = int(p[1], 16), int(p[2], 16)
            if a == 0x01:
                maddr = d & 0x3FF
            elif a == 0x02:
                if not maddr & 0x200:
                    prog[maddr & 0x1FF] = d
                maddr = (maddr & 0x200) | ((maddr + 1) & 0x1FF)
            elif 0x10 <= a < 0x30:
                lane, r = (a - 0x10) // 8, (a - 0x10) % 8
                if r == 1:
                    pcs[lane] = d & 0x1FF
                if r == 0:
                    if d & 1:
                        started.add(lane)
                    if d & 2:
                        host.add("seq.pause")
                    if d & 4:
                        host.add("seq.single_step")
            elif a == 0x45 and d & 1:
                host.add("xbar.loopback")
            elif a in (0x43, 0x44) and d:
                host.add("xbar.uio_out")
            elif a == 0x05:
                host.add("host.irqmask")
            elif a == 0x06:
                host.add("chan.host_clear")
            elif a == 0x04:
                host.add("host.irqstat_clear")
        elif c == "X":
            a = int(p[1], 16)
            if a == 0x02 and maddr & 0x200 == 0 or a == 0x02:
                maddr = (maddr & 0x200) | ((maddr + 1) & 0x1FF)
            host |= {0x00: {"host.id"}, 0x03: {"host.time"}, 0x04: {"host.irqstat"},
                     0x06: {"chan.host_chstat"}}.get(a, set())
            if 0x08 <= a <= 0x0B:
                host.add("chan.host_chdata")
        elif c == "N":
            nets += 1
        elif c == "S":
            skews += 1
        elif c == "F":
            faults += 1
    f = set(host)
    f |= {f"lane.{l}" for l in started}
    if len(started) == 4:
        f.add("top.four_lanes")
    if prog:
        f.add("mem.prog_load")
    # only the code reachable from the PCs the lanes were started at (a test
    # may fill program memory with data it never runs)
    thr2l = code_dclk = False
    reach, todo = set(), [pcs.get(l, 0) for l in started]
    while todo:
        a = todo.pop()
        if a in reach or a not in prog:
            continue
        reach.add(a)
        try:
            ins = isa.decode(prog[a])
        except Exception:
            continue
        v = ins.fields
        nxt = [a + 1]
        if ins.op == "BR":
            nxt = [v["target"]] if v["cond"] == 0 else [a + 1, v["target"]]
        elif ins.op in ("LOOP",):
            nxt = [a + 1, v["target"]]
        elif ins.op == "EVT":
            nxt = [a + 1, v["target"]]
        elif ins.op == "RESERVED" or (ins.op == "MISC" and v["op"] == 1):
            nxt = []
        todo += [x & 0x1FF for x in nxt]
    for addr in sorted(reach):
        w = prog[addr]
        try:
            ins = isa.decode(w)
        except Exception:
            continue
        v = ins.fields
        op = ins.op
        if op == "SETM":
            reg = v["reg"]
            imm = v.get("imm", 0)
            if v.get("from_reg"):
                f.add("cfg.setm_reg")
                continue
            if v.get("upper"):
                f.add("cfg.setm_hi")
                continue
            f.add("cfg.setm")
            if reg == 0:
                f.add("tick." + ["BAUD", "EDGE", "RECOVER", "PULSE"][imm & 3])
                f.add("tick.prescale" + ["1", "8", "64", "512"][imm >> 2 & 3])
                if imm & 0x80:
                    f.add("tick.clock_out")
                if imm & 0x100:
                    f.add("tick.clock_idle_high")
                if imm & 0x200:
                    f.add("tick.clock_od")
                if imm & 0x400:
                    f.add("tick.rest")
                if imm & 0x800:
                    f.add("tick.stretch_honour")
                if imm & 0x1000:
                    f.add("tick.stretch_hold")
                if imm & 3 == 1:
                    f.add("tick.capture_falling" if imm & 0x40 else "tick.capture_rising")
            elif reg == 3 and imm & 0xFF:
                f.add("tick.thr0")
            elif reg == 4 and imm >> 8:
                thr2l = True
            elif reg == 5:
                f.add({0: "coding.level", 1: "coding.rx.nrzi", 2: "coding.rx.manchester",
                       3: "coding.rx.pulse_symbol"}[imm & 3])
                f.add({0: "coding.level", 1: "coding.tx.nrzi", 2: "coding.tx.manchester",
                       3: "coding.tx.pulse_width"}[imm >> 2 & 3])
                code_dclk |= bool(imm >> 10 & 1)
                for bit, name in ((6, "coding.invert"), (7, "coding.data_od"), (8, "coding.diff_out"),
                                  (9, "coding.diff_in"), (11, "delay.prescaled")):
                    if imm >> bit & 1:
                        f.add(name)
                if imm >> 4 & 3:
                    f.add("coding.data_in")
            elif reg == 6:
                f.add("cfg.pulse")
            elif reg == 7 and imm & 0x10:
                f.add("stuff.ones" if not imm & 8 else "stuff.equal")
                f.add(f"stuff.n{imm & 7}")
                if imm & 0x20:
                    f.add("stuff.violation_event")
            elif reg == 9:
                f.add(f"crc.w{(imm & 31) + 1}")
                f.add("crc.init_ones" if imm & 0x20 else "crc.init_zero")
                if imm & 0x40:
                    f.add("crc.xorout")
            elif 10 <= reg <= 13:
                trig = imm >> 3 & 7
                f |= {1: {"events.level"}, 2: {"events.edge"}, 3: {"events.any"},
                      4: {"events.bothlow"}}.get(trig, set())
                if trig in (1, 2, 3):
                    f |= {1: {"events.b_high"}, 2: {"events.b_low"}}.get(imm >> 8 & 3, set())
                if imm >> 10 and trig:
                    f.add("events.k_window")
        elif op == "FIELD":
            f.add("field." + ["drive", "sample", "exchange", "expect"][v["kind"]])
            for k in ("lsb", "early", "seek"):
                if v.get(k):
                    f.add(f"field.{k}")
            if v.get("crc"):
                f.add("field.crc_flag")
                f.add("crc.lsb_first" if v.get("lsb") else "crc.msb_first")
            if v.get("stuff"):
                f.add("field.stuff_flag")
            if v.get("tag"):
                f.add("field.tag")
            if v.get("src") == 7:
                f.add("field.src_mem")
            if v.get("src") == 6 and v["kind"] == 3:
                f.add("crc.expect")
            if v.get("dst") == 5:
                f.add("seq.cap_records")
        elif op == "RAW":
            f.add("field.raw_drive" if v.get("dir") else "field.raw_sample")
        elif op == "PIN":
            f.add("pins.pin")
            if v.get("wait"):
                f.add("pins.pin_wait")
        elif op in ("LDI", "ALU"):
            f.add("seq.alu")
            if op == "ALU" and v.get("op") in (12, 13):
                f.add("seq.movsr")
        elif op == "MEM":
            f |= {"seq.mem_ld_st", "mem.data_lane"}
        elif op == "LOOP":
            f.add("seq.loop")
        elif op == "BR":
            cond = v["cond"]
            if 7 <= cond <= 10:
                f.add("seq.br_queue")
            elif 11 <= cond <= 14:
                f.add("seq.br_event")
            elif 15 <= cond <= 18:
                f.add("chan.br_valid")
        elif op == "WAIT":
            if v.get("mask"):
                f.add("seq.wait_event")
            if v.get("timeout"):
                f.add("seq.wait_timeout")
        elif op == "EVT":
            if v.get("clear"):
                f.add("events.clear")
        elif op == "Q":
            f |= {2: {"seq.pushts"}, 3: {"crc.pushcrc"}, 4: {"chan.put_get"}, 5: {"chan.put_get"},
                  6: {"chan.putsr"}}.get(v["op"], set())
        elif op == "MISC":
            f |= {1: {"seq.halt"}, 4: {"crc.crcchk"}}.get(v["op"], set())
        elif op == "RESERVED":
            f.add("seq.reserved_halt")
    if thr2l:
        f.add("delay.clock" if code_dclk else "delay.data")
    if nets:
        f.add("xbar.uio_in")
    return f, {"nets": nets, "skews": skews, "faults": faults}


def coverage(repo, bdir, board_results, extra):
    sys.path.insert(0, os.path.join(repo, "toolchain", "tests"))
    sys.path.insert(0, os.path.join(repo, "toolchain"))
    import hw_coverage                       # noqa: E402
    import test_blocks_rtl                   # noqa: E402
    from rosetta import isa                  # noqa: E402
    passed = {t["id"] for t in board_results.get("tests", []) if t["status"] == "pass"}
    declared = {}
    for tid, feats in hw_coverage.SCENARIOS.items():
        declared[tid] = set(feats)
    for t in board_results.get("tests", []):
        name = t["id"].rsplit(".", 1)[-1]
        if t["id"].startswith("test_blocks_rtl.") and name in test_blocks_rtl.COVERS:
            declared[t["id"]] = set(test_blocks_rtl.COVERS[name])
    derived = {}
    for n in sorted(os.listdir(bdir)):
        tid = n.rsplit(".", 2)[0]
        feats, _ = decode_session(os.path.join(bdir, n), isa)
        derived.setdefault(tid, set()).update(feats)
    for name, ok in extra.items():
        declared[name] = set(hw_coverage.BOARD_ONLY[name])
        if ok:
            passed.add(name)
    table = []
    known = hw_coverage.feature_ids()
    unknown = sorted({f for s in list(declared.values()) + list(derived.values()) for f in s} - known)
    for block, fid, what in hw_coverage.FEATURES:
        dec = sorted(t for t, s in declared.items() if fid in s and t in passed)
        der = sorted(t for t, s in derived.items() if fid in s and t in passed and t not in dec)
        table.append({"block": block, "feature": fid, "what": what, "checked_by": dec,
                      "configured_in": der, "on_hardware": bool(dec or der)})
    return {"table": table, "unknown_feature_ids": unknown,
            "not_on_this_board": hw_coverage.NOT_ON_THIS_BOARD}


def short(tid):
    """test_i2c_rtl.I2cHoldOnRtl.test_100khz -> I2cHoldOnRtl.100khz"""
    return re.sub(r"^test_\w+?_rtl\.(\w+)\.test_", r"\1.", tid).replace("board.", "board:")


def coverage_md(cov):
    lines = ["# Hardware coverage (PYNQ-Z1, lockstep board runs)", "",
             "`checked by`: passing board tests whose checks rely on the feature (test_blocks_rtl "
             "COVERS, hw_coverage.SCENARIOS). `configured in`: other passing board tests whose "
             "loaded programs or host operations use it (decoded from the session transcripts).", "",
             "| Block | Feature | What | On hardware | Checked by | Configured in |",
             "|---|---|---|---|---|---|"]
    for r in cov["table"]:
        lines.append(f"| {r['block']} | `{r['feature']}` | {r['what']} | "
                     f"{'yes' if r['on_hardware'] else '**NO**'} | "
                     f"{', '.join(short(t) for t in r['checked_by'][:4])}"
                     f"{' +%d' % (len(r['checked_by']) - 4) if len(r['checked_by']) > 4 else ''} | "
                     f"{', '.join(short(t) for t in r['configured_in'][:3])}"
                     f"{' +%d' % (len(r['configured_in']) - 3) if len(r['configured_in']) > 3 else ''} |")
    missing = [r for r in cov["table"] if not r["on_hardware"]]
    lines += ["", f"Features without a passing board test: {len(missing)}"
              + (": " + ", ".join(f"`{r['feature']}`" for r in missing) if missing else ""), "",
              "## Not exercisable on this board", ""]
    for what, why in cov["not_on_this_board"]:
        lines.append(f"- **{what}.** {why}")
    if cov["unknown_feature_ids"]:
        lines += ["", "Unknown feature ids used by tests: " + ", ".join(cov["unknown_feature_ids"])]
    return "\n".join(lines) + "\n"


# ------------------------------------------------------------------ main

def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--repo", default=os.environ.get("ROSETTA_REPO", "/home/elfo/Documents/Rosetta"))
    ap.add_argument("--force-build", action="store_true")
    ap.add_argument("--no-build", action="store_true")
    ap.add_argument("--skip-board-only", action="store_true", help="skip the async sweep")
    args = ap.parse_args()
    repo = os.path.abspath(args.repo)
    rtl = os.path.join(repo, "src")
    t_start = time.time()
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    res_dir = os.path.join(HERE, "results", f"regress_{stamp}")
    os.makedirs(res_dir)
    work = os.path.join(res_dir, "work")
    vdir, bdir = os.path.join(work, "tr_verilator"), os.path.join(work, "tr_board")
    summary = {"date": dt.datetime.now().isoformat(timespec="seconds"), "repo": repo,
               "repo_commit": subprocess.run(["git", "-C", repo, "rev-parse", "HEAD"], capture_output=True,
                                             text=True).stdout.strip(),
               "repo_dirty": bool(subprocess.run(["git", "-C", repo, "status", "--porcelain", "--", "src"],
                                                 capture_output=True, text=True).stdout.strip()),
               "steps": {}, "failures": []}
    log(f"results -> {res_dir}")

    def step(name, fn):
        t0 = time.time()
        try:
            v = fn()
        except SystemExit as e:
            summary["failures"].append(f"{name}: {e}")
            v = None
        summary["steps"][name] = round(time.time() - t0, 1)
        return v

    b = step("build", lambda: ensure_build(rtl, args.force_build, args.no_build))
    summary["build"] = b
    if b:
        neg = [c for c, s in b["clock_slack"].items() if s["wns_ns"] not in ("n/a",) and
               (float(s["wns_ns"]) < 0 or float(s["whs_ns"]) < 0)]
        if neg or not b.get("all_constraints_met", False):
            summary["failures"].append(f"timing not met: {neg}")
    ok_board = step("deploy", deploy) is not None

    log("Verilator: full toolchain suite")
    v = step("verilator_tests", lambda: run_suite(repo, "verilator", "test*.py", vdir,
                                                  os.path.join(res_dir, "verilator_tests.json")))
    summary["verilator"] = {k: v[k] for k in ("counts", "seconds")} if v and "counts" in v else v
    if v and (v.get("crashed") or v["counts"].get("fail") or v["counts"].get("error")):
        summary["failures"].append("Verilator suite: " + json.dumps(v.get("counts")))
    if ok_board:
        log("board: chip scenarios (lockstep)")
        bd = step("board_tests", lambda: run_suite(repo, "board", "test_*_rtl.py", bdir,
                                                   os.path.join(res_dir, "board_tests.json")))
    else:
        bd = {"tests": [], "counts": {}}
    summary["board"] = {k: bd[k] for k in ("counts", "seconds")} if "counts" in bd else bd
    if bd.get("crashed") or bd["counts"].get("fail") or bd["counts"].get("error") or \
            bd["counts"].get("skip") or not bd.get("tests"):
        summary["failures"].append("board scenarios: " + json.dumps(bd.get("counts")))

    cmp_ = compare_transcripts(vdir, bdir) if os.path.isdir(vdir) and os.path.isdir(bdir) else []
    diffs = [c for c in cmp_ if not c["identical"]]
    summary["transcripts"] = {"sessions": len(cmp_), "identical": len(cmp_) - len(diffs),
                              "pin_changes_compared": sum(c.get("changes", 0) for c in cmp_),
                              "differences": diffs}
    if diffs or not cmp_:
        summary["failures"].append(f"{len(diffs)} of {len(cmp_)} sessions differ between Verilator and board")

    extra = {}
    if ok_board:
        log("board: clock and asynchronous UART sweep")

        def board_only():
            out = os.path.join(res_dir, "board_only.json")
            code = ("import json, sys; sys.path.insert(0, %r); import board_tests as t; "
                    "r = {'clock': t.clock()}; " % HERE
                    + ("" if args.skip_board_only else "r['uart_async_sweep'] = t.uart_async_sweep(); ")
                    + "json.dump(r, open(%r, 'w'), indent=1)" % out)
            with open(out + ".log", "w") as f:
                subprocess.run([sys.executable, "-c", code], stdout=f, stderr=subprocess.STDOUT)
            if not os.path.exists(out):
                raise SystemExit("board-only checks crashed (see board_only.json.log)")
            return json.load(open(out))
        bo = step("board_only", board_only) or {}
        summary["clock"] = bo.get("clock")
        clk = bo.get("clock") or {}
        if not clk or abs(clk.get("fclk_hz", 0) - 60e6) > 10 or abs(clk.get("sys_hz", 0) - 60e6) > 600:
            summary["failures"].append(f"chip clock not 60 MHz: {clk}")
        sw = bo.get("uart_async_sweep")
        if sw:
            summary["uart_async_sweep"] = {k: v2 for k, v2 in sw.items() if k != "points"}
            extra["board.uart_async_sweep"] = sw["pass"]
            if not sw["pass"]:
                summary["failures"].append("asynchronous UART sweep failed")

    try:
        cov = coverage(repo, bdir, bd, extra) if os.path.isdir(bdir) else None
    except Exception as e:                                  # report, do not hide
        cov = None
        summary["failures"].append(f"coverage: {type(e).__name__}: {e}")
    if cov:
        json.dump(cov, open(os.path.join(res_dir, "coverage.json"), "w"), indent=1)
        open(os.path.join(res_dir, "coverage.md"), "w").write(coverage_md(cov))
        summary["coverage"] = {"features": len(cov["table"]),
                               "on_hardware": sum(r["on_hardware"] for r in cov["table"]),
                               "missing": [r["feature"] for r in cov["table"] if not r["on_hardware"]]}

    # keep transcripts and VCDs of failing tests and differing sessions
    bad = {t["id"] for t in (bd.get("tests", []) + (v or {}).get("tests", []))
           if t["status"] in ("fail", "error")}
    keep = {c["session"] for c in diffs}
    for d in (vdir, bdir):
        if os.path.isdir(d):
            for n in os.listdir(d):
                if n.rsplit(".", 2)[0] in bad:
                    keep.add(n)
    if keep:
        fdir = os.path.join(res_dir, "failures")
        os.makedirs(fdir, exist_ok=True)
        for n in sorted(keep):
            for tag, d in (("verilator", vdir), ("board", bdir)):
                p = os.path.join(d, n)
                if os.path.exists(p):
                    shutil.copy(p, os.path.join(fdir, f"{tag}.{n}"))
                    to_vcd(transcript_dumps(p), os.path.join(fdir, f"{tag}.{n[:-4]}.vcd"), f"{tag} {n}")
    if os.path.isdir(bdir):                    # the board's transcripts, for later analysis
        subprocess.run(["tar", "czf", os.path.join(res_dir, "transcripts_board.tar.gz"), "-C", work,
                        "tr_board"], check=False)
    shutil.rmtree(work, ignore_errors=True)

    summary["seconds"] = round(time.time() - t_start, 1)
    summary["pass"] = not summary["failures"]
    json.dump(summary, open(os.path.join(res_dir, "summary.json"), "w"), indent=1)
    md = [f"# Rosetta FPGA regression {summary['date']}: {'PASS' if summary['pass'] else 'FAIL'}", "",
          f"Repo {repo} at {summary['repo_commit'][:12]}{' (src dirty)' if summary['repo_dirty'] else ''}; "
          f"bitstream {((b or {}).get('bit_sha256') or '')[:16]}; total {summary['seconds']} s.", ""]
    if b:
        md.append(f"Timing: design WNS {b.get('design_wns_ns')} ns, WHS {b.get('design_whs_ns')} ns; chip clock "
                  f"{b['clock_slack'].get('clk_wiz_clk_out2')}.")
    if summary.get("clock"):
        c = summary["clock"]
        md.append(f"Chip clock: {c.get('sys_hz')} Hz vs the 125 MHz oscillator, {c.get('fclk_hz')} Hz vs FCLK0, "
                  f"{c.get('sw_hz', 0):.1f} Hz vs Linux time.")
    md += ["", "| Test | Verilator | Board | s (board) |", "|---|---|---|---|"]
    vt = {t["id"]: t for t in (v or {}).get("tests", [])}
    for t in bd.get("tests", []):
        md.append(f"| {t['id']} | {vt.get(t['id'], {}).get('status', '-')} | {t['status']} | {t['seconds']} |")
    md += ["", f"Transcripts: {summary['transcripts']['identical']} of {summary['transcripts']['sessions']} "
           f"sessions identical ({summary['transcripts']['pin_changes_compared']} pin changes compared)."]
    if summary.get("uart_async_sweep"):
        s = summary["uart_async_sweep"]
        md.append(f"Async UART sweep: {'pass' if s['pass'] else 'FAIL'}; decodes {s['measured_pass_pct']} % "
                  f"(bit {s.get('measured_pass_bit_cycles')} cycles), compiler window {s['expected_window_pct']} %.")
    if summary["failures"]:
        md += ["", "## Failures", ""] + [f"- {x}" for x in summary["failures"]]
    open(os.path.join(res_dir, "summary.md"), "w").write("\n".join(md) + "\n")
    log(f"{'PASS' if summary['pass'] else 'FAIL'} in {summary['seconds']} s: {res_dir}")
    for x in summary["failures"]:
        log(f"  {x}")
    sys.exit(0 if summary["pass"] else 1)


if __name__ == "__main__":
    main()
