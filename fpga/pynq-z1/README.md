# Rosetta FPGA harness for the PYNQ-Z1

This puts the Rosetta Tiny Tapeout RTL (`tt_um_jesselevine727_rosetta`) on the
PYNQ-Z1's XC7Z020 so it can be run on real hardware before tape-out. The Zynq
ARM core takes the place of the demo board's RP2350:

* a 60 MHz chip clock from an MMCM, gated by a BUFGCE so it can free-run or
  be single-stepped;
* `rst_n` and `ena`, all 8 `ui_in` and all 8 `uio_in` driven from AXI GPIO;
  `uo_out`, `uio_out` and `uio_oe` read back;
* the chip's host SPI port (`ui[0]`=SCK, `ui[1]`=CS_n, `ui[2]`=MOSI,
  `uo[0]`=MISO, `uo[1]`=IRQ) driven by an **AXI Quad SPI** master at
  3.75 MHz, with a GPIO bit-bang fallback;
* every chip input can take its value from the processor, from a board header
  pin, or from a **loopback** of any chip output or uio pin. This means the
  harness can be tested with nothing wired to the board;
* all 24 chip pins brought out to PMOD JA/JB and the Arduino header, with
  `uio` going through tri-state IOBUFs controlled by `uio_oe`;
* a **logic analyser built into the FPGA**: it records `{uio_oe, uio, uo, ui}`
  on every chip clock edge into a 16384-sample BRAM, with a mask/value
  trigger (level, edge, or change) and pre-trigger samples;
* two hardware frequency meters for the chip clock, one against the board's
  independent 125 MHz oscillator and one against PS FCLK0;
* a **lockstep mode** that makes the board a cycle-exact twin of the
  Verilator harness (`toolchain/tests/rtl/rosetta_tb.cpp`): the chip clock
  runs only for counted runs and for host frames played by a host-SPI engine
  on exactly the chip edges the C++ harness uses; a change trace records
  every pin change with its chip-cycle number; uo->ui wires with no delay;
  uio pins joined into open-drain nets (wired-AND, pull-up); per-pin input
  skew; a one-shot inversion fault; and an **asynchronous UART far end**
  clocked by the 125 MHz oscillator. See "Lockstep mode";
* `regress.py`, the **one-command regression gate**: build if the RTL
  changed, program the board, run every chip scenario on Verilator and on
  the board, compare the two edge for edge, write a dated results directory,
  exit non-zero on any failure. See "Regression gate".

The chip's RTL is pulled into the block design as a **module reference**
(through the harness module). To swap the template placeholder for the real
design, run `build.sh` with a different source directory. Nothing in the
Rosetta repo is modified.

## Regression gate (`regress.py`)

```sh
cd /home/elfo/Documents/Rosetta/fpga/pynq-z1
./regress.py                     # exit 0 = pass; results in results/regress_<date>_<time>/
./regress.py --force-build       # rebuild even if nothing changed;  --no-build: never build
```

What one run does (about 8 minutes when no build is needed, plus about 9
minutes for a build):

1. **Build if needed.** A key over the RTL sources (`info.yaml` list), the
   harness HDL, Tcl, XDC, `build.sh` and the defines is compared with
   `out/build_key.txt`; on any change `build.sh <repo>/src --define
   ROSETTA_SRAM_BEHAV` runs. Negative slack on any clock fails the gate.
2. **Deploy** `rosetta.bit`, `rosetta.hwh`, `rosetta_board.py`, `lockstep.py`
   and `board_server.py` to `/home/xilinx/rosetta/regress/` if their hashes
   differ.
3. **Verilator**: the whole toolchain suite (`python3 -m unittest discover -s
   toolchain/tests`), with a transcript of every chip session.
4. **Board**: every chip scenario (`toolchain/tests/test_*_rtl.py`: the
   compiled UART, SPI and I2C scenarios in every role, and the targeted
   block tests `test_blocks_rtl.py`) with `ROSETTA_BACKEND=board`: the same
   test code, the same checks, the board in lockstep mode. The board server
   reloads the FPGA for every session, so each starts from configuration
   state as a fresh Verilator process starts from zero.
5. **Edge-for-edge comparison**: each session's transcript (every command
   and every reply, including every pin change with its chip cycle) must be
   identical on both backends. A difference names the session, the command
   and the first differing line.
6. **Board-only** (`board_tests.py`, each with a JSON result in
   `board_only.json`): the chip clock by the three meters; the compiled UART
   receiver and listener against the asynchronous UART far end, swept from
   -7 % to +7 % around 115200 Bd (expected window from the compiler's
   manifest); the study's 10BASE-T receiver against an asynchronous
   Manchester source with duty-cycle distortion and alternating / uniform
   per-edge jitter on a 78.125 ps grid (`eth_rx_sweep`, run on the board by
   `farend.py`; the study's simulated limits are the reference: this is the
   sweep the H1 decoder will have to beat); the study's USB device against an
   asynchronous host at 0 and +-1.5 % (`usb_async`); the study's CAN nodes
   with the external node on its own clock (`can_async`).
7. **Results** in `results/regress_<date>_<time>/`: `summary.json` and
   `summary.md` (build, timing, utilisation, clock, per-test results on both
   backends, transcript comparison, sweep), `coverage.md` / `coverage.json`
   (every block and mode against the board tests that exercise it),
   `verilator_tests.json` / `board_tests.json` (per test, with tracebacks),
   `board_only.json`, `transcripts_board.tar.gz` (every board session's
   commands and replies), and in `failures/` the transcripts and VCDs (pin
   changes, 1 cycle = 16.667 ns) of every failing test and differing session.

`ROSETTA_BOARD`, `ROSETTA_BOARD_DIR`, `ROSETTA_REPO` override the board, its
directory and the repository. The gate needs the board: an unreachable board
is a failure, not a skip.

### Running the shared scenarios by hand

```sh
cd /home/elfo/Documents/Rosetta
python3 -m unittest discover -s toolchain/tests                                   # Verilator (default)
ROSETTA_BACKEND=board python3 -m unittest discover -s toolchain/tests -p 'test_*_rtl.py'   # the board
ROSETTA_TRANSCRIPT=/tmp/tr python3 -m unittest ...                                 # keep transcripts
```

Each scenario is written once, against `rtlsim.Chip`; `ROSETTA_BACKEND`
selects the C++ harness (a local process) or `board_server.py` (over ssh,
one connection per test process, the FPGA reloaded per `Chip()`). Both
speak the same line protocol, so the tests cannot tell them apart, and
without a board (or without Verilator) the tests skip.

### Adding a protocol's scenarios

1. Write `toolchain/tests/test_<proto>_rtl.py` against `rtlsim.Chip` (load
   the compiled images, wire pins with `wire()` / `net()`, push, `run()`,
   `dump()`, `drain()`), decorated
   `@unittest.skipUnless(rtlsim.available(), rtlsim.unavailable_reason())`.
   Nothing else is needed for it to run on both backends and be compared.
2. List what each test exercises in `toolchain/tests/hw_coverage.py`
   (`SCENARIOS`), adding feature ids to `FEATURES` if the protocol uses a
   mode nothing else does.
3. What both backends offer (rosetta_tb.cpp header): up to 8 uo->ui wires,
   4 open-drain nets over the uio pins, per-uio skew 0-63 cycles, one
   inversion fault window on the wires, a waveform player (8 channels on any
   input pins, run-length queue streamed without limit, pulls, contention
   count) and a CAN bus model (up to 4 nodes' TXD nets, transceiver delays,
   an external node on channel 7). The trace holds 16384 entries between
   drains and is lossless (the chip pauses). Stimulus is computed on the PC
   (`toolchain/tests/protolines.py` has USB, 10BASE-T and CAN line models)
   and played as runs, so both backends get it edge for edge.
4. Board only: the asynchronous UART far end on any ui pin, and two
   asynchronous lines with a 78.125 ps edge grid (`rh_line.v`) on any pin
   or as the CAN bus's external node, fed with event lists from the same
   line models (`protolines.line_events`). A far end that must also react
   in real time (a USB host answering within 7.5 bits, a CAN node
   arbitrating) needs a small HDL model or a real device on the headers.

### Adding the H1 / H3 / H4 scenarios (ruling 59)

The baseline scenarios for the three new protocols are in
`toolchain/tests/test_usb_rtl.py`, `test_eth_rtl.py` and `test_can_rtl.py`
(the study's programs, unchanged, in `fixtures/study_*.asm`), and the
board-only far ends in `board_tests.py` (`eth_rx_sweep`, `usb_async`,
`can_async`). When an RTL change lands:

* **H1 (Manchester phase-tracking decoder under RECOVER).** Add the new
  receive program as a fixture and run `test_eth_rtl`'s receive tests on it
  (the frame generator, impairments and checks are program-independent);
  then call `board_tests.eth_rx_sweep(points=...)` with its words via
  `farend ethsweep` (the program is a parameter) and extend the points to the
  H1 targets (DCD to about 35 ns, alternating jitter to about 18 ns,
  uniform to about 14 ns). The classifier's board result in
  `results-latest/` is the baseline to beat. Limitation: the async lines
  need events at least 20 ns apart, so DCD above 30 ns (low pulses under
  20 ns) needs a deeper event prefetch in `rh_line.v` (two entries: 15 ns),
  or two IDELAYs per line used alternately for anything shorter.
* **H3 (edge-terminated RAW DRIVE).** A CAN transmitter variant that joins
  a competing SOF: in lockstep, sweep the external node's SOF (waveform
  channel 7) across a bit time against an idle bus and check the join
  within about 5 cycles plus the loop delay at 1 Mbit/s; on the board, the
  same with `can_async` (the external node on its own clock).
* **H4 (RECOVER options).** CAN receivers resynchronising on recessive-to-
  dominant edges only, sampling at 75-87.5 %. The bus model applies one
  delay to both edge directions; a real transceiver is slower on the
  dominant-to-recessive edge (about 50 ns, REPORT.md 6.5), so add a
  per-direction delay to the `Y` command in both harnesses
  (rosetta_tb.cpp and the CAN block in rosetta_harness.v, plus
  tb_lockstep.v), then check the sample point on the trace.

Each new program needs only fixtures and tests; each harness feature must be
added to both harnesses with the same meaning, checked in Icarus with
`sim/run_lockstep.sh` against the Verilator harness before a build.

## Files

| Path | What |
|---|---|
| `build.sh` | `./build.sh <rtl-src-dir>`: Vivado build that produces `out/rosetta.bit` and `out/rosetta.hwh` |
| `tcl/build.tcl` | Project-mode build: block design, module reference, synthesis, implementation, reports |
| `tcl/ps7_pynq_z1.tcl` | PYNQ-Z1 PS7 board settings (crystal, PLLs, DDR3, MIO) taken from the PYNQ v3.1 base overlay; no board files needed |
| `hdl/rosetta_harness.v` | Harness top (the module reference): clock gating, routing, IOBUFs, capture, meters |
| `hdl/rh_capture.v`, `hdl/rh_freqmeter.v` | Logic capture and frequency meter |
| `hdl/rh_hostspi.v` | Lockstep host-SPI engine: the C++ harness's frames, edge for edge |
| `hdl/rh_trace.v` | Change trace: one entry per pin change, 48-bit chip-cycle stamp, 16384 entries |
| `hdl/rh_async_uart.v` | Asynchronous UART far end on the 125 MHz oscillator (rate by phase accumulator) |
| `hdl/rh_wave.v` | Lockstep waveform player (the C++ harness's V/P queue) |
| `hdl/rh_line.v` | Two asynchronous lines with a 78.125 ps edge grid (MMCM, toggle flops, IDELAYE2) |
| `python/farend.py` | Board-side far-end campaigns (the 10BASE-T receive-margin sweep) |
| `python/lockstep.py` | Lockstep driver: the C++ harness's commands on the board |
| `python/board_server.py` | Line-protocol server on the board: `rtlsim.Chip` talks to it over ssh |
| `regress.py` | The regression gate (one command) |
| `run_tests.py` | Runs the toolchain tests on one backend, per-test JSON (used by `regress.py`) |
| `board_tests.py` | Board-only checks: clock meters, asynchronous UART rate sweep |
| `sim/run_lockstep.sh`, `sim/tb_lockstep.v` | Icarus check of the lockstep harness against the Verilator harness: plays a command file through the harness registers and writes replies in `rosetta_tb.cpp`'s format, to `diff` against the Verilator harness's own replies |
| `hdl/templates/rosetta_chip.v.in` | Shim with a fixed name that instantiates the TT top; `build.sh` fills in the module name |
| `xdc/pynq_z1_rosetta.xdc` | Pins (from Digilent's `PYNQ-Z1_C.xdc`), pulls, clocks, CDC constraints |
| `python/rosetta_board.py` | PYNQ driver (the API below) |
| `python/demo.py` | Hardware self-test and demo used for the results below |
| `sim/run_sim.sh`, `sim/tb_harness.v` | Icarus simulation of harness and chip. The self-checks assume the template placeholder (adder); with the real design it still compiles and runs, but the adder checks will fail |
| `scripts/chip_sources.sh` | Chip source list (from info.yaml, or glob); used by `build.sh` and `run_sim.sh` |
| `ref/pynq_base.tcl` | PYNQ v3.1 base-overlay block-design script copied from the board; `tcl/ps7_pynq_z1.tcl` was extracted from it |
| `out/` | Build outputs: `rosetta.bit`, `rosetta.hwh`, `reports/`, `build.log`, `build_info.txt` |
| `results/` | Results copied back from the board (JSON, VCD waveforms, console log); `results/regress_<date>_<time>/` from each `regress.py` run |
| `archive/` | The harness as it was before the lockstep work (21 Nov), for reference |

## Rebuild

```sh
cd /home/elfo/Documents/Rosetta/fpga/pynq-z1
./build.sh /home/elfo/Documents/Rosetta/src --define ROSETTA_SRAM_BEHAV   # about 8 min on this machine
```

The Rosetta chip needs `--define ROSETTA_SRAM_BEHAV`: the SRAM macro does not
exist on the FPGA, and the define selects the behavioural memory in
`src/rosetta_sram512x32.sv` (block RAM). `regress.py` passes it and builds
only when the RTL or the harness changed. `sim/run_sim.sh` with
`sim/tb_harness.v` checks the template placeholder (adder) only; the real
chip's harness is checked by `sim/run_lockstep.sh` (section "Lockstep
mode").

* The source list comes from `source_files:` in `<rtl-src-dir>/../info.yaml`
  (the Tiny Tapeout convention). Use `--glob` to take every `*.v`/`*.sv` in
  the directory instead. `*.vh`/`*.svh` files in the directory are copied
  along with the sources so that `` `include `` works.
* The top module comes from `top_module:` in the same `info.yaml`. Override
  it with `--top NAME`.
* **`default_nettype` rewrite.** Vivado builds from copies of the sources in
  `out/gen/rtl/`, and in those copies `` `default_nettype none `` becomes
  `` `default_nettype wire ``. The reason: under `none`, IEEE 1800 makes an
  `input logic` port (a data type with no net type) illegal. Vivado enforces
  this (`Synth 8-6735: net type must be explicitly specified`), while Yosys
  and Verilator accept it. The Rosetta blocks use exactly that style (for
  example `src/rosetta_crc.sv`), so without the rewrite the real design will
  not synthesise in Vivado. The originals are never touched, and
  `--keep-nettype` turns the rewrite off. A fix on the RTL side is to declare
  inputs as `input wire [N:0] x` (or `input wire logic`), which all three
  tools accept.
* Other options: `--out DIR` (default `./out`), `--jobs N` (default 8),
  `--sck-ratio N` (AXI Quad SPI SCK = 60 MHz / N; default 16 = 3.75 MHz),
  `--define NAME[=VALUE]` (repeatable; a Verilog define for every source,
  recorded in `out/build_info.txt` with a hash of the harness files).
* Swapping in the real design is one command:
  `./build.sh /home/elfo/Documents/Rosetta/src` after `src/` has been updated
  (or point it at any other checkout).
* After a build, check `out/reports/clock_slack.txt` (WNS/WHS per clock),
  `out/reports/timing_summary.rpt` and `out/reports/mmcm_settings.txt`.
  `out/build_info.txt` records the RTL directory, its git commit, and the
  sha256 of every source file.

Requirements: Vivado 2025.1 at `/home/elfo/Documents/2025.1/Vivado/bin/vivado`
(override with `VIVADO=`). No PYNQ-Z1 board files are needed; the build
targets `xc7z020clg400-1` directly.

## Deploy and run on the board

```sh
B=xilinx@10.0.0.82
ssh $B mkdir -p /home/xilinx/rosetta
scp out/rosetta.bit out/rosetta.hwh python/rosetta_board.py python/demo.py $B:/home/xilinx/rosetta/
ssh $B 'cd /home/xilinx/rosetta && echo xilinx | sudo -S env XILINX_XRT=/usr PYTHONDONTWRITEBYTECODE=1 \
        /usr/local/share/pynq-venv/bin/python demo.py'
scp -r $B:/home/xilinx/rosetta/results ./results           # JSON + VCD waveforms
```

* Programming the PL needs root, which is why `sudo` is used. Run from
  `/home/xilinx/rosetta`, not from `/home/xilinx`: the latter contains a stale
  `pynq/` source directory that shadows the installed package.
* `.bit` and `.hwh` must sit next to each other with the same base name, as
  PYNQ requires. `demo.py` hands its `results/` back to the invoking user.
  PYNQ itself writes a root-owned `loaded.xclbin` into the working directory.
* To work interactively:
  `sudo env XILINX_XRT=/usr PYTHONDONTWRITEBYTECODE=1 /usr/local/share/pynq-venv/bin/python`, then
  `from rosetta_board import load; b = load()`. `load(download=False)`
  skips programming but still applies the defaults and resets the chip. To
  attach without changing any state, use
  `from rosetta_board import RosettaBoard; b = RosettaBoard(download=False)`.

## Python API (`rosetta_board.py`)

```python
from rosetta_board import load
b = load()                       # program PL, defaults, then reset(); returns RosettaBoard
b.reset(cycles=16)               # rst_n low for >= cycles chip clocks, then high
b.set_rst_n(0|1); b.set_ena(0|1)
b.host_mode('spi'|'gpio')        # ui0..2 from AXI Quad SPI (default) or from set_ui()
b.set_ui(v); b.set_uio(v)        # processor drive values
b.set_inputs(ui, uio)            # both in one register write (same chip cycle)
b.get_ui()                       # ui_in as the chip sees it (after routing)
b.get_uo()                       # uo_out
b.get_uio()                      # uio pins as the chip sees them on uio_in
b.get_uio_out(); b.get_uio_oe()
b.spi_transfer(bytes, cpol=0, cpha=0, lsb_first=False) -> bytes   # CS_n low for the whole buffer
b.spi_transfer_bitbang(bytes, cpol=0, cpha=0, step_clock=0) -> bytes  # ~1 ms/byte; optional single-step
b.capture(n=4096, trigger=None, pre=0, div=0, action=None, timeout=2.0) -> Capture
b.route('ui3', 'cpu'|'header'|'spi'|'uo5'|'uio2'|'miso'|'irq')   # source of any chip input
b.loopback('uio0', 'uo1')        # same as route()
b.clear_routes(host='spi')
b.headers(True|False)            # drive the board headers (off after load)
b.get_pads()                     # actual pad levels {ui: IO0-7, uo: JA, uio: JB}
b.clock_run(True|False); b.step(n); b.cycles()
b.measure_clock(seconds=1.0)     # {'sw_hz', 'sys_hz', 'fclk_hz', ...}
b.status()                       # dict of everything
```

Trigger strings for `capture()`:

* `None` triggers immediately.
* `'PIN:KIND'`, where PIN is `ui0..7`, `uo0..7`, `uio0..7`, `oe0..7` or one
  of the aliases `sck cs cs_n mosi miso irq`, and KIND is `high low rise fall
  change`.
* `'ui:change'` (also `uo`, `uio`, `oe`) triggers on any bit of that byte.
* `'uo=0x45'` triggers on a whole-byte level match.
* `(mode, mask, value)` sets the trigger registers directly.

`action` is called once the capture is armed and the pre-trigger part is
filled. For example:
`b.capture(16384, 'cs:fall', pre=32, action=lambda: b.spi_transfer(b'\xa5'))`.

`Capture` holds one sample per captured chip clock edge. It provides:

* `.raw` (uint32 samples) and the byte arrays `.ui .uo .uio .oe`;
* `.bit('uo3')` and `.edges('sck', 'rise')` for single pins;
* `.changes()` and `.table()`, which list the samples where something
  changed, relative to the trigger;
* `.to_vcd(path)` to export for GTKWave;
* `.trigger_index`, which equals `pre`;
* `.period_ns`, which is 16.667 x (div+1). It is only meaningful while the
  clock free-runs: with a stopped clock, each sample is one `step()`.

## Register map

The 8 AXI GPIOs are dual-channel and 32 bits wide: channel 1 is at +0x0 and
channel 2 at +0x8 from each base. W means the processor writes the channel
(a GPIO output), R means it reads it (a GPIO input). Everything runs in the
60 MHz domain, so writes take effect on a clean chip clock edge.

| Base | Ch | Name | Bits |
|---|---|---|---|
| 0x4120_0000 | 1 W | CTRL | [0] rst_n (reset value 0), [1] ena (1), [2] clk_run (1), [3] step toggle (each toggle = one chip edge while clk_run=0), [4] hdr_en (0), [5] cap_arm (rising edge arms, 0 = idle), [6] cap_force (force trigger), [9:8] trig_mode (0 immediate, 1 level, 2 edge into match, 3 change), [31:16] cap_div (store 1 of div+1 cycles) |
| 0x4120_0000 | 2 R | STATUS | [0] MMCM locked, [1] cap armed, [2] cap triggered, [3] cap done, [4] chip clock enabled, [15:8] ui_in as seen by the chip, [19:16] FREQ_SYS update count, [23:20] FREQ_FCLK update count, [31:24] 0x52 harness ID |
| 0x4121_0000 | 1 W | DRIVE | [7:0] ui drive value, [15:8] uio drive value |
| 0x4121_0000 | 2 R | CHIP | [7:0] uo_out, [15:8] uio_out, [23:16] uio_oe, [31:24] uio pin value (= uio_in seen by the chip) |
| 0x4122_0000 | 1 W | SRC | [15:0] from-header mask, [31:16] loopback mask. Input index i: 0-7 = ui[i], 8-15 = uio[i-8] |
| 0x4122_0000 | 2 W | SRC2 | [2:0] ui0..2 from the SPI master (reset value 0b111) |
| 0x4123_0000 | 1 W | LOOP_LO | nibble i (i = 0..7): loopback source for ui[i] |
| 0x4123_0000 | 2 W | LOOP_HI | nibble j (j = 0..7): loopback source for uio_in[j]. Source k: 0-7 = uo[k], 8-15 = uio pin k-8 |
| 0x4124_0000 | 1 W | TRIG_MASK | 32-bit sample mask |
| 0x4124_0000 | 2 W | TRIG_VALUE | 32-bit sample value |
| 0x4125_0000 | 1 W | CAP_CFG | [15:0] length n (1..16384; 0 = 16384), [31:16] pre-trigger samples (< n) |
| 0x4125_0000 | 2 R | CAP_STATUS | [13:0] trigger address, [29:16] write pointer, [30] triggered, [31] done |
| 0x4126_0000 | 1 R | PADS | [7:0] Arduino IO0-7 pad levels, [15:8] JA pad levels, [23:16] JB pad levels |
| 0x4126_0000 | 2 R | CYCLES | 32-bit count of delivered chip clock edges (a counter clocked by the gated chip clock itself) |
| 0x4127_0000 | 1 R | FREQ_SYS | chip clock edges per 125,000,000 cycles of the 125 MHz board oscillator, updated every second |
| 0x4127_0000 | 2 R | FREQ_FCLK | chip clock edges per 100,000,000 FCLK0 cycles, updated every second |
| 0x41E0_0000 | | AXI Quad SPI | PG153 registers (SPICR 0x60, SPISR 0x64, DTR 0x68, DRR 0x6C, SSR 0x70); standard mode, master, 1 SS, FIFO 256, SCK = 60 MHz / 16 |
| 0x4000_0000 | | Capture BRAM | 16384 x 32-bit samples, read-only from the PS side (AXI BRAM controller) |

Capture sample format: [7:0] `ui_in`, [15:8] `uo_out`, [23:16] uio pin
(`uio_oe ? uio_out : uio_in`), [31:24] `uio_oe`. Sample *i* is the pin
state the chip registers at its *i*-th captured rising edge. Arming fills
`pre` samples first, then waits for the trigger. The trigger sample is at
`trig_addr`, and the window is `[trig_addr-pre, trig_addr+n-pre-1]`
(modulo 16384).

How each chip input is chosen. Priority is loopback, then header, then SPI
(ui0..2 only), then processor:

```
ui_in[i]  = SRC.loop[i] ? reg(src_k) : SRC.ext[i] ? pad[i] : (i<3 && SRC2.spi[i]) ? spi[i] : DRIVE[i]
uio_in[j] = uio_oe[j] ? uio_out[j]   : (same selection for input 8+j)     # a driven pin reads itself
```

The loopback source goes through one 60 MHz register (`reg()` above). This
breaks combinational loops through the chip; the template placeholder is
purely combinational. As a result a loopback is **one cycle later** than a
real wire between two pins.

## Lockstep mode

The board as a cycle-exact twin of the Verilator harness. In lockstep the
chip clock is stopped except for

* **runs** of exactly `RUN_N` chip edges (the C++ harness's `R n`), and
* **host frames** played by `rh_hostspi` on ui0..2: the C++ harness's
  `spi_frame()` (mode 0, 8-cycle phases: 664 edges for a write, 792 for a
  read), one input value per chip edge, MISO sampled after the same edge.

Everything the harness does is counted in chip edges, so the chip sees the
same input on the same edge as under Verilator, whatever the processor's
timing. `rh_trace` stores an entry whenever any of {uo, ui, uio_out,
uio_oe, uio_in} changes, stamped with the number of chip edges before it:
exactly the C++ harness's `D` output. When the trace is nearly full the
harness pauses the chip clock until the processor drains it, so no change is
ever lost. Wires from uo to ui have no delay (every chip output is a flop
output, so there is no loop), uio pins in a net read the wired-AND of the
pins that drive low (pull-up), a pin may read its value up to 63 chip edges
late, and a one-shot fault inverts every wire into ui for a window counted
from the next falling edge of the first wire's source. Pins not in a net
read their DRIVE value (`uio_iso`), as unconnected pins read 1 in the C++
harness.

The waveform player (`rh_wave.v`) and the CAN bus model are the C++
harness's `M U V P K T Y` commands, also edge for edge: the player shows one
queued cycle per chip edge (prefetching runs from a 16384-entry block-RAM
queue, and pausing a lockstep run for a cycle when a run is not fetched yet),
and the server streams longer waves without ever letting the queue run dry
during a clocked command. The CAN delay lines are 256-deep shift registers
clocked by chip edges from configuration, as the C++ harness's ring buffers
record every cycle from reset.

`python/board_server.py` serves the C++ harness's upper-case commands and
board-only lower-case words (its docstring lists them): `begin` (reload the
PL, reset as the C++ harness does: 10 edges in reset, 10 out, cycle 20),
`clock`, `uart`/`uartroute` (async UART), `freerun 0|1`, `tracestat`,
`lineroute`/`lineplay`/`linestat` (async lines), and `farend NAME JSON` for
campaigns that run on the board (`farend.py`: `ethsweep`).

### Asynchronous lines (`rh_line.v`, board only)

An MMCM on the board's 125 MHz oscillator makes a 200 MHz clock independent
of the chip's. Each event (computed by the processor in continuous time)
sets one or both lines' level and drive at `dt` 5 ns cycles after the
previous event, plus 2.5 ns if `half` (a negedge toggle flop; each line is
the XOR of a posedge and a negedge toggle flop, glitch-free), plus `tap` x
78.125 ps through the line's IDELAYE2 (DATAIN path, VAR_LOAD, loaded two
cycles before the edge while the line is still; IDELAYCTRL reference 200
MHz): a 78.125 ps grid. Events must be at least 20 ns apart. Accuracy
limits: the MMCM output jitter (about 0.1 ns) and an unmeasured rise/fall
asymmetry of the fabric path from the IDELAY to the chip input. A line
routed to a pin behaves as a waveform channel with priority over the
channels (drive / value, pulls, uio contention counted in LINE_STATUS);
pin code 16 makes line 0 the CAN bus's external node.
`sim/run_lockstep.sh cmds out` checks the HDL the same way in Icarus: it
plays a command file (for example `sim/lockstep_smoke.cmds`, or the
commands of any transcript) through the harness registers with the real
chip and writes the replies, to `diff` against the Verilator harness run on
the same file (`Vtt_um_jesselevine727_rosetta < cmds`). Before the first
build the UART (parity none, with the fault), SPI mode 3 and I2C fastest
scenarios (nets and skew) were checked this way: byte-identical replies.

### Lockstep registers (thirteen more dual-channel AXI GPIOs, 60 MHz domain)

| Base | Ch | Name | Bits |
|---|---|---|---|
| 0x4128_0000 | 1 W | LS_CTRL | toggles (each change acts once): [0] run go, [1] host frame go, [2] trace clear (next stored entry is unconditional; clears overflow), [4] cycle counter clear, [8] fault arm; levels: [3] trace enable, [5] `loop_dir` (uo loopbacks without the register), [6] `uio_iso`, [7] backpressure (pause lockstep runs while the trace is nearly full), [15:12] fault source uo index, [16] `host_eng` (ui0..2 from the engine) |
| 0x4128_0000 | 2 R | LS_STATUS | [0] run busy, [1] host frame busy, [2] trace overflow, [3] fault armed, [4] fault seen, [5] async UART busy, [6] chip_ce, [7] trace backpressure, [8] trace first-entry pending, [31:16] trace write pointer (entries, mod 65536) |
| 0x4129_0000 | 1 W | RUN_N | chip edges for the next run |
| 0x4129_0000 | 2 W | HOST_CMD | [6:0] register address, [7] read |
| 0x412A_0000 | 1 W | HOST_DATA | write data |
| 0x412A_0000 | 2 R | HOST_RDATA | last read frame's 32 data bits |
| 0x412B_0000 | 1 W | NETS | four 8-bit uio pin masks, one open-drain net each |
| 0x412B_0000 | 2 W | TRACE_RP | [15:0] processor's read pointer (entries) |
| 0x412C_0000 | 1/2 W | SKEW_LO / SKEW_HI | 6-bit skew per uio pin, pins 0-3 / 4-7 in bytes 0-3 |
| 0x412D_0000 | 1/2 W | FAULT_OFF / FAULT_LEN | inversion window start and length after the falling edge, chip edges |
| 0x412E_0000 | 1/2 R | CYC_LO / CYC_HI | lockstep cycle counter (48 bits) |
| 0x412F_0000 | 1 W | ASYNC_INC | async UART phase increment: rate = 125 MHz x inc / 2^32 |
| 0x412F_0000 | 2 W | ASYNC_CMD | [7:0] byte, [13:8] buffer address, [14] write toggle, [15] go toggle, [21:16] bytes - 1, [23:22] parity (0 none, 1 even, 2 odd), [24] two stop bits, [27:25] target ui index, [28] route on, [31:29] idle bits between frames |
| 0x4130_0000 | 1 W | WAVE_WP | [15:0] waveform runs written (mod 65536) |
| 0x4130_0000 | 2 R | WAVE_STATUS | [0] player empty, [31:16] runs fetched (mod 65536) |
| 0x4131_0000 | 1/2 W | CHMAP_LO / CHMAP_HI | waveform channel k in byte k (0-3) / k-4 (4-7): [4] mapped, [3:0] pin (0-7 ui, 8-15 uio) |
| 0x4132_0000 | 1 W | PULL | [15:0] pull on per pin, [31:16] pull value |
| 0x4132_0000 | 2 R | CONTENTION | pin-cycles where a channel and the chip both drove a uio pin |
| 0x4133_0000 | 1 W | CAN_CFG | [7:0] dtx, [15:8] drx, [19:16] node enables, [31:20] RXD ui pin of nodes 0-3 (3 bits each) |
| 0x4133_0000 | 2 W | CAN_MASKS | TXD uio pin mask of nodes 0-3 (bytes 0-3) |
| 0x4134_0000 | 1 W | LINE_CTRL | [0] go toggle, [5:1] line 0 pin (0-15, 16 = CAN external node, 31 = off; reset 31), [10:6] line 1 pin, [31:16] events to play |
| 0x4134_0000 | 2 R | LINE_STATUS | [0] player empty, [1] line busy, [2] line clock ready (MMCM locked, IDELAYCTRL ready), [15:4] line contention, [31:16] runs fetched |
| 0x4200_0000, 0x4201_0000, 0x4202_0000 | | TRACE w0, w1, w2 | 16384 entries each: w0 = cycle[31:0]; w1 = {flags[7:0] (bit 0 fault active, bit 1 async UART line, bits 2-3 async lines 0-1), ui, cycle[47:32]}; w2 = {uio_in, uio_oe, uio_out, uo} |
| 0x4203_0000 | | WAVE queue | 16384 runs: [31:24] drive mask, [23:16] value mask, [15:0] cycles |
| 0x4204_0000 | | LINE events | 16384 events: [15:0] dt (5 ns cycles), [16] half, [21:17] tap, [23:22] lines set, [25:24] levels, [27:26] drives |

The CTRL GPIO now resets to 0x2 (ena = 1, rst_n = 0, clock stopped), so the
chip gets no edge between configuration and `load()` or the server.

## Header pin mapping (PYNQ-Z1)

PMOD pins 1-4 are the top row and pins 7-10 the bottom row; pins 5/11 are
GND and 6/12 are 3.3 V. The PMOD signals have 200 ohm series resistors on the
PYNQ-Z1. All of these pins are 3.3 V only: do not drive them with 5 V
Arduino-shield logic. All header pins are hi-Z after `load()` until
`b.headers(True)`.

| Chip pin | Role (architecture.md section 10) | Board header pin | FPGA pin | Direction (hdr_en=1) |
|---|---|---|---|---|
| ui[0] | host SPI SCK | Arduino IO0 | T14 | monitor output (input if routed `'header'`) |
| ui[1] | host SPI CS_n | Arduino IO1 | U12 | monitor output (same) |
| ui[2] | host SPI MOSI | Arduino IO2 | U13 | monitor output (same). This is the PUDC_B pin, which has an external pull-up, so it reads 1 when undriven |
| ui[3] | protocol input | Arduino IO3 | V13 | input when routed `'header'`, else monitor |
| ui[4] | protocol input | Arduino IO4 | V15 | same |
| ui[5] | protocol input | Arduino IO5 | T15 | same |
| ui[6] | protocol input | Arduino IO6 | R16 | same |
| ui[7] | protocol input | Arduino IO7 | U17 | same |
| uo[0] | host SPI MISO | PMOD JA pin 1 | Y18 | output |
| uo[1] | host IRQ | PMOD JA pin 2 | Y19 | output |
| uo[2] | protocol output | PMOD JA pin 3 | Y16 | output |
| uo[3] | protocol output | PMOD JA pin 4 | Y17 | output |
| uo[4] | protocol output | PMOD JA pin 7 | U18 | output |
| uo[5] | protocol output | PMOD JA pin 8 | U19 | output |
| uo[6] | protocol output | PMOD JA pin 9 | W18 | output |
| uo[7] | protocol output | PMOD JA pin 10 | W19 | output |
| uio[0] | bidirectional | PMOD JB pin 1 | W14 | IOBUF: drives when uio_oe[0]=1, else input |
| uio[1] | bidirectional | PMOD JB pin 2 | Y14 | same, uio_oe[1] |
| uio[2] | bidirectional | PMOD JB pin 3 | T11 | same, uio_oe[2] |
| uio[3] | bidirectional | PMOD JB pin 4 | T10 | same, uio_oe[3] |
| uio[4] | bidirectional | PMOD JB pin 7 | V16 | same, uio_oe[4] |
| uio[5] | bidirectional | PMOD JB pin 8 | W16 | same, uio_oe[5] |
| uio[6] | bidirectional | PMOD JB pin 9 | V12 | same, uio_oe[6] |
| uio[7] | bidirectional | PMOD JB pin 10 | W13 | same, uio_oe[7] |

The 19 protocol pins are ui[3:7], uo[2:7] and uio[0:7]. The 5 host-port pins
are also brought out, as outputs, so a logic analyser can watch the host
SPI. Pulls: IO0-7 and JA have weak pull-downs; JB has weak pull-ups, so an
open-drain bus idles high (add real pull-ups for I2C). To feed a protocol
input from outside, run `b.route('ui3', 'header')` **before** connecting the
driver. Otherwise the pin is a monitor output, and the two outputs fight
each other. To feed a uio input, route it `'header'`;
the IOBUF only drives while the chip sets `uio_oe`. LEDs: LD0 is a chip-clock
heartbeat (bit 25 of CYCLES), LD1 is rst_n, LD2 is capture done, LD3 is
hdr_en.

## Design choices

* **Host SPI uses AXI Quad SPI, not bit-banged GPIO.** On this board one
  `set_ui()` call from Python takes about 25-30 us (measured from the
  capture: about 1500-1800 chip cycles between consecutive writes), so bit-banging from Python manages only
  about 10 kHz of SCK, with jitter. The architecture calls for a host clock of
  "a few MHz". AXI Quad SPI gives exact 3.75 MHz SCK (60 MHz / 16, which
  leaves the chip's 2-flop synchroniser 8 samples per SCK half period),
  hardware CS framing, a 256-byte FIFO, and every CPOL/CPHA mode.
  `spi_transfer_bitbang()` stays available for slow or odd timing, and with
  `step_clock` it can single-step the chip through a host transaction. The
  SCK divider is set at build time (`--sck-ratio`).
* **Clocking.** FCLK0 (100 MHz) feeds an MMCM. One MMCM output, through a
  BUFG, clocks all the AXI logic and the harness. A second, phase-aligned
  output goes through a BUFGCE to the chip. Everything is therefore
  synchronous: processor writes reach the chip on a clean edge, and the
  capture is exact to the cycle with no clock-domain crossing. The only
  crossings are the two frequency-meter references, through 2-flop
  synchronisers.
* **The PS7 is configured from the PYNQ base overlay's own settings**, so the
  FCLK0 divisors that PYNQ programs from the `.hwh` are the ones the board's
  FSBL expects.
* **Placeholder caveat.** `ena` and `rst_n` are wired and controllable, but
  the template design ignores them. The real design's flops will run on the
  gated chip clock (`clk_out2` domain), which `build.tcl` times together with
  the harness.

## Hardware results: USB, 10BASE-T and CAN added (21 Nov 2026, evening)

Same RTL (repository 8fc3800, `src/` unchanged), harness with the waveform
player, CAN bus model and async lines; bitstream `63dfda4ddf78...`. Results:
`results/regress_20261121_221510/` (summary and coverage also in
`results-latest/`).

**Timing** (all constraints met): chip clock WNS +1.884 / WHS +0.060 ns,
harness clock +1.694 / +0.017 ns, async-line clock (200 MHz) +0.659 /
+0.122 ns. **Utilisation**: 13312 LUTs (25.0 %), 14288 FFs (13.4 %), 98 BRAM
tiles (70 %), 2 MMCMs, 6 BUFGs. **Clock**: 60,000,229 Hz against the
oscillator, 60,000,000 against FCLK0.

**Gate**: PASS in 480 s. 148 toolchain tests on Verilator; 53 chip scenarios
on the board (the 41 before, plus USB x 3, 10BASE-T x 4, CAN x 5), all
passing; all 62 sessions identical on both backends (1,956,496 pin changes).
The main Verilator harness also reproduces the study's own harness byte for
byte on the USB, Ethernet and CAN scenarios (same commands, `Y` given as the
study's `B`).

**USB turnaround** (host SE0-to-J to the device's first K; limit 80-300
cycles = 2-7.5 bit times): lockstep 136-137 cycles (IN) and 83-85 (SETUP /
DATA0 -> ACK) at host clock 0 and +-1.5 %; asynchronous host on the board
135-138 and 85-86. Study: 132-138 and 82-86.

**10BASE-T receive margin against an asynchronous source** (100 frames per
point, payload 46, +-100 ppm per frame, 78.125 ps edge grid): the source
itself has a duty-cycle distortion at the chip input of +1.1 ns (high
pulses 1.1 ns longer, low pulses 1.1 ns shorter; measured each run by the
chip's own RAW SAMPLE on 400 ns pulses, and confirmed by the harness trace
at 62-69 ns pulses), subtracted from every frame. Last value with no
failure / first value with failures:

| Impairment | Board | Study (simulation) |
|---|---|---|
| DCD, high pulses longer | 16.0 / 16.5 ns (2 of 100) | 16.5 / 17.0 ns |
| DCD, low pulses longer | 16.5 / 17.0 ns | (symmetric) |
| Alternating jitter per edge | 8.2 / 8.5 ns | 8.2 / 8.5 ns |
| Uniform jitter per edge | 8.0 / 9.0 ns | 8.0 / 9.0 ns |

Uncompensated, the same sweep fails earlier, by the source's distortion
(DCD 15.0 / 16.0 ns, alternating 7.5 / 8.0 ns, and -17 / -18 ns on the
negative side: centred on +1.1 ns).

**CAN with the external node on its own clock** (+-0.5 %): 0x700, 0x0F0,
0x123 on the bus, each acknowledged, the same records as in lockstep.

## Hardware results: the regression gate on the real chip (21 Nov 2026)

Bitstream built from `/home/elfo/Documents/Rosetta/src` at commit 4590881
(the RTL is identical at 14dcb9c and 7b46524), `--define ROSETTA_SRAM_BEHAV`,
sha256 `ec85fcc700dc11e4...`. Full results: `results/regress_20261121_210745/` (an identical earlier pass: `results/regress_20261121_210357/`).

**Timing** (routed, all constraints met): chip clock `clk_wiz_clk_out2`
WNS +2.287 ns, WHS +0.102 ns; the worst path is a chip output through the
harness's zero-delay loopback into a chip input synchroniser (run -> uo ->
ui -> `u_xbar/s1`), the worst path inside the chip is `u_tick/cnt` ->
`u_field/sr` CE in lane 3, 13.37 ns, 15 levels, +2.899 ns. Harness clock
`clk_out1` +1.619 / +0.027 ns; `sysclk` +2.433 ns; `clk_fpga_0` +5.377 ns.

**Utilisation**: 11830 LUTs (22.2 %), 12274 FFs (11.5 %), 66 BRAM tiles
(47 %: 16 capture, 48 change trace, 2 chip memories), 1 MMCM, 5 BUFG. The
chip alone: 7670 LUTs, 3490 FFs, 2 RAMB18 (`out/reports/utilization_hier6.rpt`).

**Clock measured on the board**: 60,000,215-60,000,219 edges/s against the
125 MHz oscillator over the runs (+3.6 ppm, crystal tolerance), 60,000,000
against FCLK0 (phase locked), 59,999,919-60,000,033 against Linux time.

**Scenarios**: all 41 chip scenarios pass on both backends (16 compiled
UART/SPI/I2C scenarios, 25 targeted block tests) and the whole toolchain
suite (136 tests) passes on Verilator. Every one of the 50 chip sessions is
identical on the board and under Verilator, command for command and pin
change for pin change (about 1.5 million changes compared).

**Asynchronous UART**: the compiled receiver and listener, fed by the
125 MHz-clocked far end, decode every byte from -5.2 % to +5.6 % (far-end
bit 549.4 to 493.2 chip cycles); the compiler's manifest promises 549.67 to
495.1. The slow edge is where the compiler says; at the fast edge the chip
decodes about 1.9 cycles (0.4 %) beyond the promise, so the compiler's
re-arm allowance is conservative.

## Hardware results (PYNQ-Z1 at 10.0.0.82, 13 Nov 2026, template placeholder)

The placeholder design is `uo_out = ui_in + uio_in`, `uio_out = 0`,
`uio_oe = 0`. Everything was run with nothing wired to the board. The full
console log is in `results/results_console.txt`, the machine-readable results
in `results/demo_results.json`, and waveforms in `results/*.vcd`. The
bitstream used was `out/rosetta.bit` (sha256 `c070df709dcb...2298`, the same
on both sides).

**Adder through the harness** (all 8 ui bits from the processor). Each row is
`set_inputs(ui, uio)`, then a read of what the chip actually saw and of
`uo_out`:

| ui | uio | chip saw ui/uio | uo_out | (ui+uio)&0xFF | uio_out / uio_oe |
|---|---|---|---|---|---|
| 00 | 00 | 00/00 | 00 | 00 | 00/00 |
| 01 | 01 | 01/01 | 02 | 02 | 00/00 |
| 35 | 10 | 35/10 | 45 | 45 | 00/00 |
| 7f | 01 | 7f/01 | 80 | 80 | 00/00 |
| ff | 01 | ff/01 | 00 | 00 | 00/00 |
| 80 | 80 | 80/80 | 00 | 00 | 00/00 |
| a5 | 5a | a5/5a | ff | ff | 00/00 |
| c3 | 3d | c3/3d | 00 | 00 | 00/00 |
| 12 | 34 | 12/34 | 46 | 46 | 00/00 |
| fe | ff | fe/ff | fd | fd | 00/00 |

On top of these, 1000 random (ui, uio) vectors were all correct (1000/1000).

**Loopback.** Routing `uo1 -> ui7` with ui=01 and uio=01 gave a chip-seen ui
of 0x81 and uo 0x82, as expected. Routing `uo1 -> uio_in0` with ui=02 and
uio=00 gave a chip-seen uio of 0x01 and uo 0x03. With `uo0 -> ui0` and
uio=01 the loop oscillates: the capture shows ui0 = `0101...` and
uo = `01 02 01 02 ...`, toggling on **every** chip cycle. That is exactly the
one-register loopback latency.

**Capture of processor writes** (`trigger='ui:change'`, `pre=16`, 16384
samples). All 10 writes appear, in order, and uo = ui + 3 at every change.
The write spacing is the Python MMIO speed:

```
  sample    t(ns)   ui  uo  uio  oe
     -16    -266.7   00  03  03   00
      +0       0.0   01  04  03   00
   +1801   30016.7   02  05  03   00    (1801 chip cycles later)
   ...  04 08 10 20 40 80 ff 00 follow at 1521-1631 cycle intervals
```

**Clock gating and single-step.** With the clock stopped, the CYCLES counter
stayed frozen. After 8 `step()` calls the counter advanced by exactly 8 edges.
A capture armed while stopped holds exactly one sample per step, with ui =
10 11 12 13 14 15 16 17 and uo the same, because uio = 0.

**Host SPI** (AXI Quad SPI, mode 0). The test was `spi_transfer(a5 3c 00 ff)`,
captured with the trigger `cs:fall`. The capture shows 32 SCK rising edges
with a period of 16 chip cycles, i.e. **SCK = 3.750 MHz**. MOSI sampled at
the SCK rising edges in the capture decodes to `a5 3c 00 ff`. CS_n stays low
for the whole transfer. The bytes read back on MISO are `00 00 00 00`: the
placeholder makes MISO = uo0 = SCK with zero delay, and the master samples it
on the same edge on which it raises SCK. A real target that changes MISO on
the other edge will be read correctly. The GPIO bit-bang fallback read back
`ff ff ff ff` (it samples after the edge). A single-stepped bit-bang
transfer (`step_clock=2`) captured exactly 2 samples per pin change, and
MOSI decoded to `10100101` = 0xA5.

**Header pads** (nothing connected). With headers off, the pads read
IO0-7 = 0x04 (IO2 is PUDC_B, pulled up on the board), JA = 0x00 (pull-down)
and JB = 0xFF (pull-up). With `headers(True)` and ui = 0x5A they read
IO0-7 = 0x5A (monitor), JA = 0x5A (uo), and JB = 0xFF (still inputs, because
uio_oe = 0). So the FPGA's output drivers are confirmed working. The
connectors themselves were not tested, because nothing was plugged in.

**Chip clock = 60 MHz, measured three ways**, all on the CYCLES counter,
which is clocked by the gated chip clock itself:

| Method | Result |
|---|---|
| Edge counter vs Linux `CLOCK_MONOTONIC` (ARM timer), 2.002 s window | 59.999930 MHz in the final run (59.99988-59.99997 over 3 runs; read uncertainty is about 1 ppm) |
| Hardware meter vs the board's independent 125 MHz oscillator | 60,000,218 edges/s in the final run (60,000,218-60,000,222 over the runs: +3.6 ppm, within crystal tolerance) |
| Hardware meter vs PS FCLK0 (PYNQ: 100.0 MHz) | 60,000,000 edges/s exactly (the clock is phase-locked: confirms the MMCM ratio is exactly 0.6) |

MMCM settings, chosen by the clocking wizard (`out/reports/mmcm_settings.txt`):
CLKIN = FCLK0 100 MHz, DIVCLK_DIVIDE = 5, CLKFBOUT_MULT_F = 51 (VCO =
1020 MHz), CLKOUT0_DIVIDE_F = 17 and CLKOUT1_DIVIDE = 17, giving 60.000 MHz.
The wizard estimates 254 ps peak-to-peak output jitter.

**Timing** (routed, `out/reports/timing_summary.rpt`): all constraints are
met, with no critical warnings. The only DRC warning is RTSTAT-10, unused
nets inside IP.

| Clock | Period | WNS | WHS |
|---|---|---|---|
| `clk_out1` 60 MHz: harness, AXI, capture | 16.667 ns | **+8.120 ns** | +0.027 ns |
| `clk_wiz_clk_out2` 60 MHz: chip clock after the BUFGCE | 16.667 ns | **+13.953 ns** (intra) | +0.062 ns |
| chip clock -> harness clock / harness clock -> chip clock | 16.667 ns | +11.794 / +12.368 ns | +0.138 / +0.901 ns |
| `clk_fpga_0` 100 MHz (meter reference) | 10 ns | +5.937 ns | +0.249 ns |
| `sysclk` 125 MHz (meter reference) | 8 ns | +4.129 ns | +0.263 ns |

With the placeholder, the chip-clock domain holds only the harness edge
counter (32 endpoints), because the template has no flops. Once the real
design is in, `out/reports/clock_slack.txt` gives its WNS on the
`clk_wiz_clk_out2` line.

**Swap test with real Rosetta SystemVerilog.** A scratch tree with a test
top that instantiates the repo's `rosetta_crc.sv` on a few ui/uo pins, plus
its own `info.yaml`, was built with the same single `build.sh` command. It
synthesised and closed timing: the chip clock `clk_wiz_clk_out2` had
WNS +9.28 ns (now with real flops) and the harness clock +7.665 ns. It needed
the `default_nettype` rewrite described under Rebuild. Without it, Vivado
stops with Synth 8-6735 on the `input logic` ports.

Utilisation: 2494 LUTs (4.7 %), 4894 FFs (4.6 %), 17 BRAM36 tiles
(16 of them for the capture buffer), 1 MMCM and 4 BUFGs out of 32.

## Known limitations

* **Loopback is one cycle later than a wire** (the register is needed to
  avoid combinational loops through a combinational chip). Captures record
  what the chip actually saw, so the extra cycle is visible in them. With
  `LS_CTRL.loop_dir` (lockstep mode) a uo source is a plain wire, as in the
  Verilator harness; uio sources keep the register.
* **Lockstep stops the chip clock between commands.** Silicon's clock never
  stops; the chip is fully synchronous, so a paused clock is invisible to it,
  but nothing in lockstep exercises a real asynchronous input except the
  async UART far end (one ui pin, free-running clock).
* **The async UART far end** is one transmitter (8 data bits, optional
  parity, 1-2 stop bits, up to 64 bytes per burst, any one ui pin). Other
  asynchronous far ends need their own small HDL model.
* **External header inputs are not synchronised in the FPGA.** They go
  straight to the chip, just as on silicon. The chip's own synchronisers
  handle them; the capture flop may see a metastable sample at the instant
  an external edge arrives.
* **`uio_in` follows the pin.** For a uio bit with `uio_oe = 1`, the chip
  sees its own `uio_out` on `uio_in`, as on silicon. A design with a
  combinational `uio_in -> uio_out` path would therefore form a loop.
  Rosetta registers its inputs, so this does not occur.
* **SCK rate is fixed at build time** (`--sck-ratio`). For slower or odd
  timing, use `spi_transfer_bitbang()`.
* **The capture's ns axis assumes a free-running clock.** With a stepped
  clock, each sample is one step.
