#!/usr/bin/env bash
# Non-vacuity check for formal/mem against the PROPERTY WRITER'S PRIVATE
# REFERENCE MODELS (ref/rosetta_mem_prog.sv, ref/rosetta_mem_data.sv, not the
# design).
#
# For each variant (NONE = clean reference, MUT_* = one planted bug) this
# writes ref/build/<block>_<variant>.sby from ../mem_<block>.sby with the
# design line pointed at the reference model (absolute paths) and
# `read -define <MUT>` added, then runs the given tasks. Variants run in
# parallel (JOBS, default 6).
#
#   usage: [DEFS="X Y"] [TAG=name] ref/run_mutants.sh prog|data [tasks...] [-- variants...]
#   tasks: prove | bmc | cover          (default: prove bmc)
#   DEFS:  extra `read -define` for every variant; TAG is appended to the
#          variant name in build/ to keep runs apart.
set -u
here="$(cd "$(dirname "$0")/.." && pwd)"          # formal/mem
source /home/elfo/tools/oss-cad-suite-dl/oss-cad-suite/environment
JOBS=${JOBS:-6}
TMO=${TMO:-1800}
DEFS=${DEFS:-}
TAG=${TAG:-}

blk=${1:-}
case "$blk" in prog|data) shift ;; *) echo "usage: $0 prog|data [tasks...] [-- variants...]"; exit 2 ;; esac
src="rosetta_mem_$blk.sv"
job="mem_$blk"

tasks=(); variants=(); seen=0
for a in "$@"; do
    if [ "$a" = "--" ]; then seen=1; continue; fi
    if [ $seen = 0 ]; then tasks+=("$a"); else variants+=("$a"); fi
done
[ ${#tasks[@]} = 0 ] && tasks=(prove bmc)
[ ${#variants[@]} = 0 ] && variants=(NONE $(grep -o '`ifdef MUT_[A-Z0-9_]*\|`ifndef MUT_[A-Z0-9_]*\|`elsif MUT_[A-Z0-9_]*' "$here/ref/$src" | sed 's/.* //' | sort -u))

build="$here/ref/build"
mkdir -p "$build"

run_one() {
    v=$1$TAG; t=$2
    sby_file="$build/${job}_${v}__$t.sby"   # one copy per task: own status db
    log="$build/${job}_${v}_$t.log"
    wd="$build/${job}_${v}_$t"
    cp "$build/${job}_$v.sby" "$sby_file"
    start=$(date +%s.%N)
    timeout "$TMO" sby -f -d "$wd" "$sby_file" "$t" > "$log" 2>&1
    end=$(date +%s.%N)
    status=$(grep -E "DONE \((PASS|FAIL|UNKNOWN|ERROR)" "$log" | tail -1 | sed 's/.*DONE (\([A-Z]*\).*/\1/')
    [ -z "$status" ] && status=TIMEOUT
    why=$( { grep -h "Assert failed in" "$log" | sed -e 's/.*: //' -e 's/ \[failed before\]//' -e 's/^.*\.\(u_[a-z_]*\)\./\1./';
             grep -h "Unreached cover statement" "$log" | sed -e 's/.*statement at //' -e 's/^/unreached:/'; } \
           | sed -e 's/ *([^)]*)//' | sort -u | tr '\n' ' ')
    printf "%-5s %-20s %-6s %-8s %7.1fs  %s\n" "$blk" "$v" "$t" "$status" "$(echo "$end - $start" | bc)" "$why"
}
export -f run_one
export build TMO TAG job blk

for v0 in "${variants[@]}"; do
    v=$v0$TAG
    sed -e "s#^$src .*#$src $here/ref/$src#" \
        -e "s#^rosetta_sram512x32.sv .*#rosetta_sram512x32.sv $here/../../src/rosetta_sram512x32.sv#" \
        -e "s#^mem_probe.sv\$#mem_probe.sv $here/mem_probe.sv#" \
        -e "s#^${job}_props.sv\$#${job}_props.sv $here/${job}_props.sv#" \
        -e "s#^${job}_fv.sv\$#${job}_fv.sv $here/${job}_fv.sv#" \
        "$here/$job.sby" > "$build/${job}_$v.sby.tmp"
    defs="$DEFS"
    [ "$v0" != NONE ] && defs="$defs $v0"
    for d in $defs; do
        sed -i -e "s#^read -formal rosetta_sram512x32.sv#read -define $d\nread -formal rosetta_sram512x32.sv#" "$build/${job}_$v.sby.tmp"
    done
    mv "$build/${job}_$v.sby.tmp" "$build/${job}_$v.sby"     # atomic: safe with a concurrent run
done

for v in "${variants[@]}"; do for t in "${tasks[@]}"; do echo "$v $t"; done; done \
    | xargs -P "$JOBS" -n 2 bash -c 'run_one "$0" "$1"'
