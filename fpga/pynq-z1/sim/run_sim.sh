#!/usr/bin/env bash
# Simulate the harness + chip RTL with Icarus Verilog.
#   sim/run_sim.sh [rtl-src-dir] [chip-top]      (GLOB=1 to take every *.v/*.sv)
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"; ROOT="$(dirname "$HERE")"
RTL="${1:-/home/elfo/Documents/Rosetta/src}"
TOP="${2:-tt_um_jesselevine727_rosetta}"
OSS=/home/elfo/tools/oss-cad-suite-dl/oss-cad-suite/bin
W="$ROOT/sim/work"; mkdir -p "$W"
sed "s/@CHIP_TOP@/$TOP/" "$ROOT/hdl/templates/rosetta_chip.v.in" > "$W/rosetta_chip.v"
mapfile -t CHIP < <("$ROOT/scripts/chip_sources.sh" "$RTL" ${GLOB:+--glob} | grep -vE '\.s?vh$')
"$OSS/iverilog" -g2012 -Wall -Wno-timescale -o "$W/tb.vvp" -s tb_harness -I "$RTL" \
    "$HERE/models.v" "$HERE/tb_harness.v" "$ROOT"/hdl/*.v "$W/rosetta_chip.v" "${CHIP[@]}"
"$OSS/vvp" -n "$W/tb.vvp"
