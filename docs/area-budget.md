# Area budget: 8x4 tiles on IHP SG13CMOS5L

How many logic cells fit, worked out from the Tiny Tapeout files and the PDK
rather than estimated. Re-check this against real synthesis as soon as there
is RTL.

## Sources

| Quantity | Value | Source |
|---|---|---|
| Die | 1724.16 x 710.64 um = **1.2253 mm2** | `tt-support-tools/tech/ihp-sg13cmos5l/tile_sizes.yaml` |
| Placement rows | **186 rows x 3580 sites** | `tt-support-tools/tech/ihp-sg13cmos5l/def/tt_block_8x4_pgvdd.def` |
| Site | 0.48 x 3.78 um (1.8144 um2) | `sg13cmos5l_stdcell.lef` (`CoreSite`) |
| Placeable core | 665,880 sites = **1,208,173 um2** | rows x sites x site area |
| Routing layers | Metal1-Metal4 only (TopMetal1 is the chip power grid) | `tt-support-tools/tech.py` |
| Default placement density | 60 % (users report up to 80 %) | template `src/config.json` |

The blog's "about 1.0 mm2" is 32 x 202 x 155 um, which leaves out the
inter-tile spacing that becomes usable when tiles are merged. The real
placeable area is about 20 % larger.

## Cell sizes (exact, from the LEF)

| Cell | Area (um2) | Sites |
|---|---|---|
| inv_1 | 5.44 | 3 |
| nand2_1 / nor2_1 | 7.26 | 4 |
| a21oi_1 / o21ai_1 / and2_1 | 9.07 | 5 |
| xor2_1 / xnor2_1 | 14.52 | 8 |
| mux2_1 | 18.14 | 10 |
| dlhq_1 (latch) | 30.85 | 17 |
| mux4_1 | 38.10 | 21 |
| **dfrbpq_1 (smallest flop, has reset)** | **48.99** | 27 |

There is no flop without reset and no full-adder cell. Averages measured on
a real IHP place-and-route of an RV32 SoC (15,569 cells): combinational
cells 9.16 um2, all cells 13.88 um2, 12 % of cells were flops.

## Budget

P&R adds about 10 % on top of synthesis for clock tree, hold fixing and
resizing, so this table divides that out. The numbers are what synthesis may
report.

| Density | Usable um2 | NAND2 equivalents | Cells, 12 % flops (flops) | Cells, 25 % flops (flops) | Cells per tile |
|---|---|---|---|---|---|
| 40 % | 439,336 | 60,535 | 31,512 (3,781) | 22,979 (5,745) | 985 |
| 50 % | 549,169 | 75,668 | 39,390 (4,727) | 28,723 (7,181) | 1,231 |
| **60 %** | **659,003** | **90,802** | **47,268 (5,672)** | **34,468 (8,617)** | **1,477** |
| 70 % | 768,837 | 105,935 | 55,146 (6,617) | 40,213 (10,053) | 1,723 |

The organizers' "about 1K cells per tile" matches about 40 % density. The
TT default is 60 %. Because only Metal2-Metal4 are available for signal
routing, **plan for about 45K synthesized cells and about 5.5K flops with no
macros**. Treat 60-70 % as achievable only after a real routing run proves it.

## Storage

| Storage | um2 per bit |
|---|---|
| Flop + write-enable mux | 67.1 |
| Latch + mux | 49.0 |
| SRAM 512x32 macro | 4.87 |
| SRAM 2048x32 macro | 3.98 |

Every SRAM macro removes its full footprint plus a halo from the placeable
area. Footprints (exact LEF sizes):

| Macro | um | mm2 | % of core |
|---|---|---|---|
| 1P 64x16 | 236.8 x 64.36 | 0.0152 | 1.3 |
| 1P 256x16 | 236.8 x 118.78 | 0.0281 | 2.3 |
| 1P 256x32 | 416.64 x 118.78 | 0.0495 | 4.1 |
| 1P 512x32 | 416.64 x 191.34 | 0.0797 | 6.6 |
| 1P 1024x32 | 416.64 x 336.46 | 0.1402 | 11.6 |
| 1P 2048x32 | 416.64 x 626.7 | 0.2611 | 21.6 |
| 1P 8192x32 (32 KB) | 1520.16 x 618.3 | 0.9399 | 77.8 |
| 2P 256x32 | 702.83 x 136.97 | 0.0963 | 8.0 |
| 2P 512x32 | 685.49 x 219.77 | 0.1507 | 12.5 |

The macros are single-port (1P) or dual-port (2P), have registered read
data (one cycle of latency) and cannot be preloaded, so a program loader is
mandatory.

## Timing reference

A previous RV32 multi-cycle core on IHP closed at the typical corner but had
a **~44 ns critical path at the slow corner** (1.08 V, 125 C). The board
clock is up to 60 MHz (16.7 ns). Keep logic depth shallow and pipeline early.
