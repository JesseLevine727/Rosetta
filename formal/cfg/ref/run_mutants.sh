#!/usr/bin/env bash
# Non-vacuity check for formal/cfg against the PROPERTY WRITER'S PRIVATE
# REFERENCE MODEL (ref/rosetta_cfg.sv, not the design).
#
# For each variant (NONE = clean reference, MUT_* = one planted bug) this
# writes ref/build/<variant>.sby from ../cfg.sby with the design line pointed
# at the reference model (absolute paths) and `read -define <MUT>` added,
# then runs the given tasks. Variants run in parallel (JOBS, default 8).
#
#   usage: [DEFS="X Y"] [TAG=name] ref/run_mutants.sh [tasks...] [-- variants...]
#   tasks: prove | bmc | cover          (default: prove bmc)
#   DEFS:  extra `read -define` for every variant, e.g. CFG_PULSE_RST_FREE
#          or FV_NO_BUS; TAG is appended to the variant name in build/ to
#          keep runs apart.
set -u
here="$(cd "$(dirname "$0")/.." && pwd)"          # formal/cfg
source /home/elfo/tools/oss-cad-suite-dl/oss-cad-suite/environment
JOBS=${JOBS:-8}
TMO=${TMO:-1800}
DEFS=${DEFS:-}
TAG=${TAG:-}

tasks=(); variants=(); seen=0
for a in "$@"; do
    if [ "$a" = "--" ]; then seen=1; continue; fi
    if [ $seen = 0 ]; then tasks+=("$a"); else variants+=("$a"); fi
done
[ ${#tasks[@]} = 0 ] && tasks=(prove bmc)
[ ${#variants[@]} = 0 ] && variants=(NONE $(grep -o '`ifdef MUT_[A-Z0-9_]*\|`ifndef MUT_[A-Z0-9_]*\|`elsif MUT_[A-Z0-9_]*' "$here/ref/rosetta_cfg.sv" | sed 's/.* //' | sort -u))

build="$here/ref/build"
mkdir -p "$build"

run_one() {
    v=$1$TAG; t=$2
    sby_file="$build/${v}__$t.sby"       # one copy per task: own status db
    log="$build/${v}_$t.log"
    wd="$build/${v}_$t"
    cp "$build/$v.sby" "$sby_file"
    start=$(date +%s.%N)
    timeout "$TMO" sby -f -d "$wd" "$sby_file" "$t" > "$log" 2>&1
    end=$(date +%s.%N)
    status=$(grep -E "DONE \((PASS|FAIL|UNKNOWN|ERROR)" "$log" | tail -1 | sed 's/.*DONE (\([A-Z]*\).*/\1/')
    [ -z "$status" ] && status=TIMEOUT
    why=$( { grep -h "Assert failed in" "$log" | sed -e 's/.*: //' -e 's/ \[failed before\]//' -e 's/^.*\.\(u_[a-z_]*props\)\./\1./';
             grep -h "Unreached cover statement" "$log" | sed -e 's/.*statement at //' -e 's/^/unreached:/'; } \
           | sed -e 's/ *([^)]*)//' | sort -u | tr '\n' ' ')
    printf "%-20s %-6s %-8s %7.1fs  %s\n" "$v" "$t" "$status" "$(echo "$end - $start" | bc)" "$why"
}
export -f run_one
export build TMO TAG

for v0 in "${variants[@]}"; do
    v=$v0$TAG
    sed -e "s#^rosetta_cfg.sv .*#rosetta_cfg.sv $here/ref/rosetta_cfg.sv#" \
        -e "s#^cfg_props.sv\$#cfg_props.sv $here/cfg_props.sv#" \
        -e "s#^cfg_fv.sv\$#cfg_fv.sv $here/cfg_fv.sv#" \
        "$here/cfg.sby" > "$build/$v.sby.tmp"
    defs="$DEFS"
    [ "$v0" != NONE ] && defs="$defs $v0"
    for d in $defs; do
        sed -i -e "s#^read -formal rosetta_cfg.sv#read -define $d\nread -formal rosetta_cfg.sv#" "$build/$v.sby.tmp"
    done
    mv "$build/$v.sby.tmp" "$build/$v.sby"     # atomic: safe with a concurrent run
done

for v in "${variants[@]}"; do for t in "${tasks[@]}"; do echo "$v $t"; done; done \
    | xargs -P "$JOBS" -n 2 bash -c 'run_one "$0" "$1"'
