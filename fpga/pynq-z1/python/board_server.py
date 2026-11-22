"""Line-protocol server on the PYNQ-Z1: the board as a drop-in for the
Verilator harness (toolchain/tests/rtl/rosetta_tb.cpp), used by
toolchain/tests/rtlsim.py with ROSETTA_BACKEND=board.

Run on the board as root (PL programming and /dev/mem), from the directory
holding rosetta.bit / rosetta.hwh / rosetta_board.py / lockstep.py:

    sudo env XILINX_XRT=/usr /usr/local/share/pynq-venv/bin/python -u board_server.py

It prints one greeting line ("rosetta-board ..."), then reads commands from
stdin and answers each with one line ("ok", a value, or "error ..."); D
answers change lines and then "end". The commands of rosetta_tb.cpp, with
the same meaning:

    R n | W a d | X a | L f t | N mask | S pin d | F off len | C | D | Q

plus, for the board only:

    B                  new session: reload the PL and reset the chip as the
                       C++ harness does (cycle 20 afterwards)          -> "ok"
    I                  bitstream sha256, harness ID, session count     -> one line
    K                  free-run the chip clock and measure it three ways
                       (edge counter vs Linux time, vs the 125 MHz
                       oscillator, vs FCLK0), then stop it again       -> JSON line
    A ui baud parity stop2 gap hexbytes
                       send bytes from the asynchronous UART far end
                       (125 MHz oscillator) into ui[ui] while the chip
                       clock free-runs; the trace keeps recording        -> actual rate
    U ui               connect the async UART's line (idle high) to ui[ui]   -> "ok"
    G 0|1              stop / free-run the chip clock (G 0 re-syncs the
                       lockstep cycle count)                            -> cycle count
    P                  trace status: write pointer, overflow, LS_STATUS -> one line
"""
import hashlib
import json
import os
import sys
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from rosetta_board import RosettaBoard, C_CLK_RUN  # noqa: E402
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
        cmd, args = line[0], line[1:].split()
        try:
            if cmd == "Q":
                break
            elif cmd == "B":
                ls.begin(reload=True)
                sessions += 1
                reply = "ok"
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
            elif cmd == "I":
                reply = f"bit={sha} id={board.status_word() >> 24:#x} sessions={sessions}"
            elif cmd == "K":                      # free-run the chip clock for the meters
                board._set_ctrl(board._ctrl | C_CLK_RUN)
                try:
                    reply = json.dumps(board.measure_clock(1.0))
                finally:
                    board._set_ctrl(board._ctrl & ~C_CLK_RUN)
                    time.sleep(0.001)
                    ls.cyc = ls.hw_cycle()
                    ls.drain()
            elif cmd == "A":
                ui, baud, parity, stop2, gap, hexbytes = args
                rate = ls.async_uart(bytes.fromhex(hexbytes), float(baud), int(ui), parity,
                                     bool(int(stop2)), int(gap))
                ls.drain()
                reply = f"{rate:.3f}"
            elif cmd == "U":
                ls.async_route(int(args[0]))
                reply = "ok"
            elif cmd == "G":
                if int(args[0]):
                    board._set_ctrl(board._ctrl | C_CLK_RUN)
                else:
                    board._set_ctrl(board._ctrl & ~C_CLK_RUN)
                    time.sleep(0.001)
                    ls.cyc = ls.hw_cycle()
                ls.drain()
                reply = str(ls.hw_cycle())
            elif cmd == "P":
                st = ls.status()
                reply = f"wp={st >> 16} rp={ls.rp} overflow={st >> 2 & 1} status={st:#010x}"
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
