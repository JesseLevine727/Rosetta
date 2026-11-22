#!/usr/bin/env bash
# Zero-rule check from the property side for formal/tick (amendment H4).
#
#  dir1  PREVIOUS property files (git HEAD formal/tick/tick_props.sv,
#        tick_fv.sv, tick.sby) against the NEW reference model
#        (ref/rosetta_tick.sv) with cfg_rone_i = cfg_rsp2_i = 0
#        (ref/zr/wrap_new_as_old.sv). All tasks must PASS.
#  dir2  NEW property files with TICK_ASSUME_H4_OFF (and FV_NO_WORKED_REC,
#        the H4 worked timing) against the PRE-amendment reference model
#        (git HEAD formal/tick/ref/rosetta_tick.sv, ref/zr/wrap_old_as_new.sv).
#        prove and bmc must PASS (the H4 covers are unreachable there).
#
#   usage: ref/zr/run_zero_rule.sh <dir1|dir2> [tasks...]
set -u
here="$(cd "$(dirname "$0")/../.." && pwd)"          # formal/tick
source /home/elfo/tools/oss-cad-suite-dl/oss-cad-suite/environment
JOBS=${JOBS:-3}
TMO=${TMO:-1800}
dir=${1:-dir1}; shift || true
tasks=("$@"); [ ${#tasks[@]} = 0 ] && tasks=(prove bmc cover)
b="$here/ref/build/zr"
mkdir -p "$b"
cd "$here"
git show HEAD:formal/tick/tick_props.sv > "$b/old_tick_props.sv"
git show HEAD:formal/tick/tick_fv.sv    > "$b/old_tick_fv.sv"
git show HEAD:formal/tick/tick.sby      > "$b/old_tick.sby"
git show HEAD:formal/tick/ref/rosetta_tick.sv \
    | sed 's/^module rosetta_tick #(/module rosetta_tick_pre #(/' > "$b/old_ref.sv"
sed 's/^module rosetta_tick #(/module rosetta_tick_h34 #(/' ref/rosetta_tick.sv > "$b/new_ref.sv"
cat "$b/new_ref.sv" ref/zr/wrap_new_as_old.sv > "$b/dut_dir1.sv"
cat "$b/old_ref.sv" ref/zr/wrap_old_as_new.sv > "$b/dut_dir2.sv"
if [ "$dir" = dir1 ]; then
    sed -e "s#^rosetta_tick.sv .*#rosetta_tick.sv $b/dut_dir1.sv#" \
        -e "s#^tick_props.sv\$#tick_props.sv $b/old_tick_props.sv#" \
        -e "s#^tick_fv.sv\$#tick_fv.sv $b/old_tick_fv.sv#" \
        "$b/old_tick.sby" > "$b/zr_dir1.sby"
else
    sed -e "s#^rosetta_tick.sv .*#rosetta_tick.sv $b/dut_dir2.sv#" \
        -e "s#^tick_props.sv\$#tick_props.sv $here/tick_props.sv#" \
        -e "s#^tick_fv.sv\$#tick_fv.sv $here/tick_fv.sv#" \
        -e "s#^read -formal rosetta_tick.sv#read -define TICK_ASSUME_H4_OFF FV_NO_WORKED_REC\nread -formal rosetta_tick.sv#" \
        tick.sby > "$b/zr_dir2.sby"
fi
run_one() {
    t=$1
    cp "$b/zr_$dir.sby" "$b/zr_${dir}__$t.sby"
    start=$(date +%s.%N)
    timeout "$TMO" sby -f -d "$b/zr_${dir}_$t" "$b/zr_${dir}__$t.sby" "$t" > "$b/zr_${dir}_$t.log" 2>&1
    end=$(date +%s.%N)
    status=$(grep -E "DONE \((PASS|FAIL|UNKNOWN|ERROR)" "$b/zr_${dir}_$t.log" | tail -1 | sed 's/.*DONE (\([A-Z]*\).*/\1/')
    [ -z "$status" ] && status=TIMEOUT
    printf "zr %-5s %-6s %-8s %7.1fs  %s\n" "$dir" "$t" "$status" "$(echo "$end - $start" | bc)" \
        "$(grep -h "Assert failed in\|Unreached cover" "$b/zr_${dir}_$t.log" | sed 's/.*: //' | sort -u | tr '\n' ' ')"
}
export -f run_one
export b dir TMO
printf "%s\n" "${tasks[@]}" | xargs -P "$JOBS" -n 1 bash -c 'run_one "$0"'
