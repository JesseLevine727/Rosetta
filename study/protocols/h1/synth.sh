#!/bin/sh
# usage: synth.sh <name> [chparam args, e.g. "-set F 6 -set DCD 0"]
# Yosys synthesis of the H1 prototype to the CMOS5L typical library: cells,
# flops, area, and ABC's critical-path estimate (combinational, flop to flop,
# no wires, no clock-to-q / setup).
L=$(ls -d /home/elfo/tools/tt-local/pdk/ciel/ihp-sg13/versions/*/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lib | head -1)
LIB=$L/sg13cmos5l_stdcell_typ_1p20V_25C.lib
SLOW=$L/sg13cmos5l_stdcell_slow_1p08V_125C.lib
name=$1; shift; TOP=${TOP:-rosetta_mdpll}
out=$(dirname "$0")/results/synth_$name.log
/home/elfo/tools/oss-cad-suite-dl/oss-cad-suite/bin/yosys -q -l "$out" -p "
read_verilog -sv $(dirname "$0")/$TOP.sv
chparam $* $TOP
synth -flatten -top $TOP
dfflibmap -liberty $LIB
abc -liberty $LIB -script +strash;dretime;map,-D,9000;topo;stime,-p
tee -o /dev/stdout stat -liberty $LIB
" > /dev/null 2>&1
/home/elfo/tools/oss-cad-suite-dl/oss-cad-suite/bin/yosys -q -l "$out.slow" -p "
read_verilog -sv $(dirname "$0")/$TOP.sv
chparam $* $TOP
synth -flatten -top $TOP
dfflibmap -liberty $SLOW
abc -liberty $SLOW -script +strash;dretime;map,-D,16000;topo;stime,-p
" > /dev/null 2>&1
echo "== $name ($*)"
grep -E "Chip area|cells$|dfrbp|sg13cmos5l_df" "$out" | sed 's/^ *//' | head -8
grep -E "WireLoad|Gates|Delay" "$out" | tail -2
echo "slow corner:"; grep -E "Delay" "$out.slow" | tail -1
