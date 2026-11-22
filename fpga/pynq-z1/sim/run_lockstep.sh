#!/usr/bin/env bash
# Lockstep equivalence check of the harness against the Verilator harness.
#   sim/run_lockstep.sh <cmds-file> <out-file> [rtl-src-dir]
# Builds the harness + chip (ROSETTA_SRAM_BEHAV) with Icarus, plays the
# command file through the harness registers like python/board_server.py,
# and writes the replies in rosetta_tb.cpp's format to <out-file>.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"; ROOT="$(dirname "$HERE")"
CMDS="$(readlink -f "$1")"; OUTF="$(readlink -f "$2")"
RTL="${3:-/home/elfo/Documents/Rosetta/src}"
TOP="${TOP:-tt_um_jesselevine727_rosetta}"
OSS=/home/elfo/tools/oss-cad-suite-dl/oss-cad-suite/bin
W="$ROOT/sim/work_ls"; mkdir -p "$W"
sed "s/@CHIP_TOP@/$TOP/" "$ROOT/hdl/templates/rosetta_chip.v.in" > "$W/rosetta_chip.v"
mapfile -t CHIP < <("$ROOT/scripts/chip_sources.sh" "$RTL" 2>/dev/null | grep -vE '\.s?vh$')
if [[ ! -f "$W/tb.vvp" || -n "$(find "$ROOT/hdl" "$HERE/tb_lockstep.v" "${CHIP[@]}" -newer "$W/tb.vvp" 2>/dev/null)" ]]; then
    "$OSS/iverilog" -g2012 -Wall -Wno-timescale -Wno-implicit-dimensions -DROSETTA_SRAM_BEHAV -o "$W/tb.vvp" \
        -s tb_lockstep -I "$RTL" "$HERE/models.v" "$HERE/tb_lockstep.v" "$ROOT"/hdl/*.v "$W/rosetta_chip.v" "${CHIP[@]}"
fi
"$OSS/vvp" -n "$W/tb.vvp" +cmds="$CMDS" +out="$OUTF"
