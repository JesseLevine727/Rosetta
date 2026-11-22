"""Line-protocol server on the PYNQ-Z1: the board as a drop-in for the
Verilator harness (toolchain/tests/rtl/rosetta_tb.cpp), used by
toolchain/tests/rtlsim.py with ROSETTA_BACKEND=board.

Run on the board as root (PL programming and /dev/mem), from the directory
holding rosetta.bit / rosetta.hwh / rosetta_board.py / lockstep.py:

    sudo env XILINX_XRT=/usr /usr/local/share/pynq-venv/bin/python -u board_server.py

It prints one greeting line ("rosetta-board ..."), then reads commands from
stdin and answers each with one line ("ok", a value, or "error ..."); D
answers change lines and then "end".

Upper-case commands are rosetta_tb.cpp's, with the same meaning to the chip
edge (their transcripts are compared with Verilator's):

    R n | W a d | X a | L f t | N mask | S pin d | F off len | C | D | Q
    M ch pin | U pin v | V d v n ... | P n | K | T 0|1 | Y dtx drx [mask rx]...

Lower-case words are for the board only (not in transcripts):

    begin              new session: reload the PL and reset the chip as the
                       C++ harness does (cycle 20 afterwards)          -> "ok"
    info               bitstream sha256, harness ID, session count     -> one line
    clock              free-run the chip clock and measure it three ways
                       (edge counter vs Linux time, vs the 125 MHz
                       oscillator, vs FCLK0), then stop it again       -> JSON line
    uart ui baud parity stop2 gap hexbytes
                       send bytes from the asynchronous UART far end
                       (125 MHz oscillator) into ui[ui]; the chip clock
                       must free-run                                     -> actual rate
    uartroute ui       connect the async UART's line (idle high) to ui[ui] -> "ok"
    freerun 0|1        stop / free-run the chip clock (0 re-syncs the
                       lockstep cycle count)                            -> cycle count
    tracestat          trace write pointer, overflow, LS_STATUS          -> one line
    lineroute p0 p1    route async line 0 / 1 to pin p (0-15), 16 = the CAN
                       bus's external node (line 0), 31 = off            -> "ok"
    linestat           async-line busy, clock ready, contention count     -> one line
    lineplay hexwords  load and play async-line events (rh_line.v); the
                       chip clock must free-run                          -> "ok"
    farend name json   a board-side far-end campaign (farend.py), e.g.
                       ethsweep: the 10BASE-T receive-margin sweep       -> JSON line
"""
import hashlib
import json
import os
import sys
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from rosetta_board import RosettaBoard  # noqa: E402
from lockstep import Lockstep, LockstepError  # noqa: E402


def main():
    bit = os.path.join(HERE, "rosetta.bit")
    with open(bit, "rb") as f:
        sha = hashlib.sha256(f.read()).hexdigest()
    board = RosettaBoard(bit, download=True)
    ls = Lockstep(board)
    sessions = 0
    out = sys.stdout
    out.write(f"rosetta-board bit={sha[:16]} id={board.status_word() >> 24:#x}\n")
    out.flush()
    for raw in sys.stdin:
        line = raw.strip()
        if not line:
            continue
        if line[0].islower():
            cmd, *args = line.split()
        else:
            cmd, args = line[0], line[1:].split()
        try:
            if cmd == "Q":
                break
            elif cmd == "R":
                ls.run(int(args[0]))
                reply = "ok"
            elif cmd == "W":
                ls.host_write(int(args[0], 16), int(args[1], 16))
                reply = "ok"
            elif cmd == "X":
                reply = f"{ls.host_read(int(args[0], 16)):08x}"
            elif cmd == "L":
                ls.wire(int(args[0]), int(args[1]))
                reply = "ok"
            elif cmd == "N":
                ls.net(int(args[0], 16))
                reply = "ok"
            elif cmd == "S":
                ls.skew(int(args[0]), int(args[1]))
                reply = "ok"
            elif cmd == "F":
                ls.fault(int(args[0]), int(args[1]))
                reply = "ok"
            elif cmd == "C":
                reply = str(ls.cycle())
            elif cmd == "D":
                lines = ls.dump_lines()
                lines.append("end")
                out.write("\n".join(lines) + "\n")
                out.flush()
                continue
            elif cmd == "M":
                ls.map(int(args[0]), int(args[1]))
                reply = "ok"
            elif cmd == "U":
                ls.pull(int(args[0]), int(args[1]))
                reply = "ok"
            elif cmd == "V":
                ls.wave([(int(args[i], 16), int(args[i + 1], 16), int(args[i + 2]))
                         for i in range(0, len(args) - 2, 3)])
                reply = "ok"
            elif cmd == "P":
                ls.play(int(args[0]))
                reply = "ok"
            elif cmd == "K":
                reply = str(ls.contention())
            elif cmd == "T":
                ls.tracing(int(args[0]) != 0)
                reply = "ok"
            elif cmd == "Y":
                ls.canbus(int(args[0]), int(args[1]),
                          [(int(args[i], 16), int(args[i + 1])) for i in range(2, len(args) - 1, 2)])
                reply = "ok"
            # ---------------------------------------------------------- board only
            elif cmd == "begin":
                ls.begin(reload=True)
                sessions += 1
                reply = "ok"
            elif cmd == "info":
                reply = f"bit={sha} id={board.status_word() >> 24:#x} sessions={sessions}"
            elif cmd == "clock":
                was = ls.freerun
                ls.set_freerun(True)
                try:
                    reply = json.dumps(board.measure_clock(1.0))
                finally:
                    if not was:
                        ls.set_freerun(False)
            elif cmd == "uart":
                ui, baud, parity, stop2, gap, hexbytes = args
                rate = ls.async_uart(bytes.fromhex(hexbytes), float(baud), int(ui), parity,
                                     bool(int(stop2)), int(gap))
                ls.drain()
                reply = f"{rate:.3f}"
            elif cmd == "uartroute":
                ls.async_route(int(args[0]))
                reply = "ok"
            elif cmd == "freerun":
                reply = str(ls.set_freerun(int(args[0]) != 0))
            elif cmd == "tracestat":
                st = ls.status()
                reply = f"wp={st >> 16} rp={ls.rp} overflow={st >> 2 & 1} status={st:#010x}"
            elif cmd == "lineroute":
                ls.line_route(int(args[0]), int(args[1]) if len(args) > 1 else 31)
                reply = "ok"
            elif cmd == "linestat":
                st = ls.g["line"].read(8)
                reply = f"busy={st >> 1 & 1} ready={st >> 2 & 1} contention={st >> 4 & 0xFFF}"
            elif cmd == "lineplay":
                ls.line_play([int(w, 16) for w in args])
                reply = "ok"
            elif cmd == "farend":
                import farend
                reply = json.dumps(farend.run(ls, args[0], json.loads(" ".join(args[1:]) or "{}")))
            else:
                reply = f"error unknown command {cmd!r}"
        except (LockstepError, ValueError, IndexError, KeyError) as e:
            reply = f"error {type(e).__name__}: {e}"
        except Exception as e:                       # keep serving; report the fault
            traceback.print_exc(file=sys.stderr)
            reply = f"error {type(e).__name__}: {e}"
        out.write(reply + "\n")
        out.flush()


if __name__ == "__main__":
    main()
