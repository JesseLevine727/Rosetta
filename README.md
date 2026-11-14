![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg)

# Rosetta

An open-source, reprogrammable protocol emulator ASIC for the [Jane Street
protocol emulator ASIC
competition](https://blog.janestreet.com/protocol-emulator-asic-competition/).
Target: IHP 130 nm SG13CMOS5L, Tiny Tapeout 8x4 tiles, submission due
18 January 2027.

One protocol description gives a controller, a target and a listener, each with a proof that it follows the protocol. See [`docs/architecture.md`](docs/architecture.md) and [`PLAN.md`](PLAN.md).

## Repository layout

| Path | Contents |
|---|---|
| `src/` | Synthesizable RTL and the LibreLane config (`config.json`) |
| `formal/` | Formal proofs for every block and integration step, plus equivalence checks |
| `test/` | cocotb gate-level smoke test run by the Tiny Tapeout flow |
| `toolchain/` | Assembler / compiler and the host-side program loader |
| `programs/` | Protocol programs (UART, SPI, I2C, ...) |
| `docs/` | Datasheet page (`info.md`), architecture, ISA, area budget |
| `writeup/` | Submission write-up, covering design rationale, verification and AI usage |
| `info.yaml` | Tiny Tapeout project metadata (8x4 tiles, top module, pinout) |

GDS, precheck and gate-level simulation results come from the Tiny Tapeout
GitHub Actions (`.github/workflows/gds.yaml`). Local hardening writes to
`runs/` (not committed).

## Constraints

See [`docs/area-budget.md`](docs/area-budget.md): 1.225 mm2 die, about 45K
synthesized cells with no macros, Metal1-Metal4 routing, 24 digital I/O, up
to 60 MHz clock from the demo board.

## License

Apache-2.0. See [`LICENSE`](LICENSE).
