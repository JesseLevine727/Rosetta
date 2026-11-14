#!/usr/bin/env bash
# Non-vacuity check for formal/coding against the PROPERTY WRITER'S PRIVATE
# REFERENCE MODEL (ref/rosetta_coding.sv, not the design).
#
# For each variant (NONE = clean reference, MUT_* = one planted bug) this
# copies ../coding.sby into ref/build/ (gitignored), points the design line
# at the reference model, adds `read -define <MUT>`, and runs the tasks.
#
#   usage: ref/run_mutants.sh [tasks...] [-- variants...]
#   tasks: prove | bmc | cover
#   e.g.   ref/run_mutants.sh prove bmc cover
#          ref/run_mutants.sh bmc -- MUT_NRZI_POL MUT_PW_OFF1
#   env:   JOBS=n runs n variants in parallel (default 1)
#          NOMODEL=1 adds `read -define <BLOCK>_NO_MODEL_CHECK`: the exact
#          model comparison is removed, so only the port-level statements
#          are left (shows that they catch the bugs on their own)
#          DEFINES="A B" adds `read -define A` ... before the proof top
#          (e.g. DEFINES=FV_NO_UNIT: round trips only); TAG=x names the runs
#
# Failures are reported by assertion label (or `// @name` tag).
set -u
here="$(cd "$(dirname "$0")/.." && pwd)"          # formal/coding
blk=coding
source /home/elfo/tools/oss-cad-suite-dl/oss-cad-suite/environment

tasks=(); variants=(); seen_sep=0
for a in "$@"; do
    if [ "$a" = "--" ]; then seen_sep=1; continue; fi
    if [ $seen_sep = 0 ]; then tasks+=("$a"); else variants+=("$a"); fi
done
[ ${#tasks[@]} = 0 ] && tasks=(prove bmc cover)
[ ${#variants[@]} = 0 ] && variants=(NONE $(grep -o '`ifdef MUT_[A-Z0-9_]*\|`elsif MUT_[A-Z0-9_]*\|`ifndef MUT_[A-Z0-9_]*' "$here/ref/rosetta_$blk.sv" | sed 's/.* //' | sort -u))

build="$here/ref/build"
mkdir -p "$build"
cd "$here"

propname() {
    while read -r x; do
        if [[ "$x" =~ ([a-z_]+)\.sv:([0-9]+) ]]; then
            f=${BASH_REMATCH[1]}; l=${BASH_REMATCH[2]}
            t=$(sed -n "${l},$((l+3))p" "$here/$f.sv" 2>/dev/null | grep -o '// @[a-z0-9_]*' | head -1 | sed 's#// @##')
            if [ -n "$t" ]; then echo "$t"; continue; fi
        fi
        echo "$x" | awk '{print $1}' | awk '{n = split($0, a, "."); print a[n]}'
    done
}

run_one() {
    v=$1; t=$2
    sby_file="$build/$v${NOMODEL:+_nm}${TAG:+_$TAG}_$t.sby"   # one file per task (parallel runs)
    sed -e "s#^rosetta_$blk.sv .*#rosetta_$blk.sv ref/rosetta_$blk.sv#" "$here/$blk.sby" > "$sby_file"
    if [ "${NOMODEL:-0}" = 1 ]; then
        def=$(echo "$blk" | tr a-z A-Z)_NO_MODEL_CHECK
        sed -i -e "s#^read -formal ${blk}_props.sv#read -define $def\nread -formal ${blk}_props.sv#" "$sby_file"
    fi
    for d in ${DEFINES:-}; do
        sed -i -e "s#^read -formal ${blk}_fv.sv#read -define $d\nread -formal ${blk}_fv.sv#" "$sby_file"
    done
    if [ "$v" != NONE ]; then
        sed -i -e "s#^read -formal rosetta_$blk.sv#read -define $v\nread -formal rosetta_$blk.sv#" "$sby_file"
    fi
    wd="$build/${v}${NOMODEL:+_nm}${TAG:+_$TAG}_$t"
    start=$(date +%s.%N)
    sby -f -d "$wd" "$sby_file" "$t" > "$build/${v}${NOMODEL:+_nm}${TAG:+_$TAG}_$t.log" 2>&1
    end=$(date +%s.%N)
    status=$(grep -E "DONE \((PASS|FAIL|UNKNOWN|ERROR)" "$build/${v}${NOMODEL:+_nm}${TAG:+_$TAG}_$t.log" | tail -1 | sed 's/.*DONE (\([A-Z]*\).*/\1/')
    [ -z "$status" ] && status=ERROR
    why=$( { grep -h "Assert failed in" "$build/${v}${NOMODEL:+_nm}${TAG:+_$TAG}_$t.log" | sed -e 's/.*Assert failed in [^:]*: //' -e 's/ \[failed before\]//';
             grep -h "Unreached cover statement" "$build/${v}${NOMODEL:+_nm}${TAG:+_$TAG}_$t.log" | sed -e 's/.*statement at //' -e 's/^/unreached:/'; } \
         | propname | sort | uniq -c | awk '{printf "%s%s ", $2, ($1 > 1 ? "x" $1 : "")}')
    printf "%-20s %-6s %-7s %7.1fs  %s\n" "$v" "$t" "$status" "$(echo "$end - $start" | bc)" "$why"
}
export -f run_one propname
export build here blk NOMODEL DEFINES TAG

JOBS=${JOBS:-1}
for v in "${variants[@]}"; do for t in "${tasks[@]}"; do echo "$v $t"; done; done |
    xargs -P "$JOBS" -L 1 bash -c 'run_one "$0" "$1"'
