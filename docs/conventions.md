# RTL and formal conventions

Every worker follows these. They exist so blocks written in parallel fit
together and so every block can be proven with the open-source tools.

## RTL

- SystemVerilog, restricted to what open-source Yosys accepts with
  `read_verilog -sv`: `logic`, `always_ff`, `always_comb`, parameters,
  packed arrays, `typedef enum`/`struct packed`, `generate`. No interfaces,
  classes, unpacked structs, or `unique`/`priority`.
- One module per file, `src/rosetta_<block>.sv`, module `rosetta_<block>`.
  Every file starts with `` `default_nettype none ``.
- One clock `clk`, rising edge only. One reset `rst_n`, asynchronous,
  active low. **Every flop is reset** (the cell library has no flop without
  reset, and silicon flops power up at unknown values).
- Declare inputs as `input wire`, outputs as `output logic`. `input logic`
  under `` `default_nettype none `` is rejected by Vivado (Synth 8-6735),
  although Yosys and Verilator accept it.
- Port suffixes: `_i` input, `_o` output. Configuration inputs are prefixed
  `cfg_` and are assumed stable while the block is active; they change only in a
  cycle where the block is being initialised or cleared, or in reset.
- No latches, no combinational loops, no tri-states inside the chip, no
  `initial` blocks, no delays.
- Registered outputs wherever the block spec does not say otherwise. Target
  clock is 16.67 ns on a slow 130 nm process: keep logic between flops
  shallow (roughly 12 gate levels or fewer).
- Tool note: this machine sets `VERILATOR_ROOT` for an older install; run the OSS CAD Suite Verilator as `env -u VERILATOR_ROOT verilator ...`.
- No vendor primitives. Memories are only the two SRAM macros, behind
  `rosetta_mem_*` wrappers.

## Formal

Tools: `/home/elfo/tools/oss-cad-suite-dl/oss-cad-suite/bin` (sby, yosys,
eqy, mcy, bitwuzla, boolector, yices, z3).

- The property writer works **from the block spec only** and does not read
  the RTL. Properties are about ports.
- Files, per block:
  - `formal/<block>/<block>_props.sv`: a checker module taking the block's
    ports as inputs, with `assert`, `assume`, `cover`.
  - `formal/<block>/<block>_fv.sv`: top level for the proof; instantiates
    the block and the checker, leaves inputs free.
  - `formal/<block>/<block>.sby`: tasks `prove` (unbounded: `abc pdr`
    and/or k-induction), `bmc`, `cover`.
- Reset in proofs: flops start at arbitrary values; the proof assumes
  `rst_n` is low for the first cycle and checks behaviour after release.
  Do not rely on `initial` values.
- Every `assume` needs a `cover` showing the assumed-legal behaviour is
  reachable, and every feature in the spec needs at least one `cover`.
- A bounded result counts only with a written justification of the depth.
- A proof run is reported with the exact sby output lines (PASS/FAIL per
  task) and the engine used.
