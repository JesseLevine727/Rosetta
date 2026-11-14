#!/usr/bin/env bash
# Non-vacuity check for formal/tick against the PROPERTY WRITER'S PRIVATE
# REFERENCE MODELS (ref/rosetta_tick.sv, ref/rosetta_delay.sv; not the design).
#
# For each variant (NONE = clean reference, MUT_* = one planted bug) this
# writes ref/build/<job>_<variant>.sby from ../<job>.sby with the design line
# pointed at the reference model (absolute paths) and `read -define <MUT>`
# added, then runs the given tasks. Runs in parallel (JOBS, default 8).
#
#   usage: [JOBS=n] [TMO=s] ref/run_mutants.sh <tick|delay> [tasks...] [-- variants...]
#   tasks: prove | bmc | cover          (default: all three)
#   e.g.   ref/run_mutants.sh tick prove bmc -- NONE MUT_RTRIP3
#   DEFS:  extra `read -define` for every variant, e.g. TICK_NO_MODEL_CHECK or
#          DELAY_NO_MODEL_CHECK (port-level statements only); TAG is appended
#          to the run names in build/ to keep such runs apart.
set -u
here="$(cd "$(dirname "$0")/.." && pwd)"          # formal/tick
source /home/elfo/tools/oss-cad-suite-dl/oss-cad-suite/environment
JOBS=${JOBS:-8}
TMO=${TMO:-3600}
DEFS=${DEFS:-}
TAG=${TAG:-}

job=${1:-}; shift || true
case "$job" in tick|delay) ;; *) echo "usage: $0 <tick|delay> [tasks] [-- variants]"; exit 2;; esac
design=rosetta_$job.sv

tasks=(); variants=(); seen=0
for a in "$@"; do
    if [ "$a" = "--" ]; then seen=1; continue; fi
    if [ $seen = 0 ]; then tasks+=("$a"); else variants+=("$a"); fi
done
[ ${#tasks[@]} = 0 ] && tasks=(prove bmc cover)
[ ${#variants[@]} = 0 ] && variants=(NONE $(grep -o '`ifdef MUT_[A-Z0-9_]*\|`elsif MUT_[A-Z0-9_]*' "$here/ref/$design" | sed 's/.* //' | sort -u))

build="$here/ref/build"
mkdir -p "$build"

run_one() {
    v=$1$TAG; t=$2
    sby_file="$build/${job}_${v}__$t.sby"
    log="$build/${job}_${v}_$t.log"
    wd="$build/${job}_${v}_$t"
    cp "$build/${job}_$v.sby" "$sby_file"
    start=$(date +%s.%N)
    timeout "$TMO" sby -f -d "$wd" "$sby_file" "$t" > "$log" 2>&1
    end=$(date +%s.%N)
    status=$(grep -E "DONE \((PASS|FAIL|UNKNOWN|ERROR)" "$log" | tail -1 | sed 's/.*DONE (\([A-Z]*\).*/\1/')
    [ -z "$status" ] && status=TIMEOUT
    why=$( { grep -h "Assert failed in" "$log" | sed -e 's/.*: //' -e 's/ \[failed before\]//' -e 's/^.*\.\([a-z_0-9]*\)$/\1/';
             grep -h "Unreached cover statement" "$log" | sed -e 's/.*statement at //' -e 's/^/unreached:/'; } \
           | sed -e 's/ *([^)]*)//' | sort -u | tr '\n' ' ')
    printf "%-6s %-20s %-6s %-8s %7.1fs  %s\n" "$job" "$v" "$t" "$status" "$(echo "$end - $start" | bc)" "$why"
}
export -f run_one
export build TMO job TAG

for v0 in "${variants[@]}"; do
    v=$v0$TAG
    sed -e "s#^$design .*#$design $here/ref/$design#" \
        -e "s#^${job}_props.sv\$#${job}_props.sv $here/${job}_props.sv#" \
        -e "s#^${job}_fv.sv\$#${job}_fv.sv $here/${job}_fv.sv#" \
        "$here/$job.sby" > "$build/${job}_$v.sby"
    defs="$DEFS"
    [ "$v0" != NONE ] && defs="$defs $v0"
    for d in $defs; do
        sed -i -e "s#^read -formal $design#read -define $d\nread -formal $design#" "$build/${job}_$v.sby"
    done
done

for v in "${variants[@]}"; do for t in "${tasks[@]}"; do echo "$v $t"; done; done \
    | xargs -P "$JOBS" -n 2 bash -c 'run_one "$0" "$1"'
