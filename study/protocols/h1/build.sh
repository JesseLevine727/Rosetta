#!/bin/sh
# usage: build.sh <outdir> <module: rosetta_mdpll | rosetta_mdpll_bb> [-Gparam=value ...]
set -e
out=$1; top=$2; shift 2
d=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$out"
sed "s/Vrosetta_mdpll/V$top/g" "$d/h1_tb.cpp" > "$out/tb.cpp"
env -u VERILATOR_ROOT /home/elfo/tools/oss-cad-suite-dl/oss-cad-suite/bin/verilator --cc --exe --build -O2 -Wno-fatal -Wno-lint -Wno-style \
  --top-module "$top" --Mdir "$out" "$@" "$d/$top.sv" "$out/tb.cpp" > "$out/build.log" 2>&1
echo "$out/V$top"
