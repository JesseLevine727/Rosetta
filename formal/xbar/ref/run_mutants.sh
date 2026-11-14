#!/usr/bin/env bash
# Non-vacuity check for formal/xbar against the PROPERTY WRITER'S PRIVATE
# REFERENCE MODEL (ref/rosetta_xbar.sv, not the design).
#
# For each variant (NONE = clean reference, MUT_* = one planted bug) this
# copies ../xbar.sby into ref/build/ (gitignored), points the design line at
# the reference model, adds `read -define <MUT>`, and runs the given tasks.
#
#   usage: ref/run_mutants.sh [tasks...] [-- variants...]
#   tasks: prove | bmc | cover | bmckg  (bmckg = the bmc task at depth 8 with
#          `smtbmc --keep-going`, listing every assertion that fails within
#          the depth instead of only the first)
#   e.g.   ref/run_mutants.sh prove bmc cover
#          ref/run_mutants.sh bmckg -- MUT_ONE_SYNC MUT_NO_LIN
#
# Properties inside generate loops cannot carry labels in Yosys (labels are
# not scoped by generate block), so they are tagged `// @name` on their first
# source line; failures are reported by that name.
set -u
here="$(cd "$(dirname "$0")/.." && pwd)"          # formal/xbar
source /home/elfo/tools/oss-cad-suite-dl/oss-cad-suite/environment

tasks=()
variants=()
seen_sep=0
for a in "$@"; do
    if [ "$a" = "--" ]; then seen_sep=1; continue; fi
    if [ $seen_sep = 0 ]; then tasks+=("$a"); else variants+=("$a"); fi
done
[ ${#tasks[@]} = 0 ] && tasks=(prove bmc cover)
[ ${#variants[@]} = 0 ] && variants=(NONE $(grep -o '^`ifdef MUT_[A-Z0-9_]*\|^`elsif MUT_[A-Z0-9_]*' "$here/ref/rosetta_xbar.sv" | sed 's/.* //' | sort -u))

build="$here/ref/build"
mkdir -p "$build"
cd "$here"

# property name from "...xbar_props.sv:LINE..." (tag on that line) or a label
propname() {
    while read -r x; do
        if [[ "$x" =~ xbar_props\.sv:([0-9]+) ]]; then
            l=${BASH_REMATCH[1]}
            t=$(sed -n "${l}p" "$here/xbar_props.sv" | grep -o '// @[a-z0-9_]*' | sed 's#// @##')
            if [ -n "$t" ]; then echo "$t"; continue; fi
        fi
        echo "$x" | awk '{print $1}' | awk '{n = split($0, a, "."); print a[n]}'
    done
}

for v in "${variants[@]}"; do
    sby_file="$build/$v.sby"
    sed -e 's#^rosetta_xbar.sv .*#rosetta_xbar.sv ref/rosetta_xbar.sv#' "$here/xbar.sby" > "$sby_file"
    if [ "$v" != NONE ]; then
        sed -i -e "s#^read -formal rosetta_xbar.sv#read -define $v\nread -formal rosetta_xbar.sv#" "$sby_file"
    fi
    sed -e 's#^bmc: smtbmc #bmc: smtbmc --keep-going #' -e 's#^bmc: depth .*#bmc: depth 8#' \
        "$sby_file" > "$build/${v}_kg.sby"
    for t in "${tasks[@]}"; do
        start=$(date +%s.%N)
        if [ "$t" = bmckg ]; then
            wd="$build/${v}_kg_bmc"
            sby -f -d "$wd" "$build/${v}_kg.sby" bmc > "$build/${v}_$t.log" 2>&1
        else
            wd="$build/${v}_$t"
            sby -f -d "$wd" "$sby_file" "$t" > "$build/${v}_$t.log" 2>&1
        fi
        end=$(date +%s.%N)
        status=$(grep -E "DONE \((PASS|FAIL|UNKNOWN|ERROR)" "$build/${v}_$t.log" | tail -1 | sed 's/.*DONE (\([A-Z]*\).*/\1/')
        [ -z "$status" ] && status=ERROR
        why=$( { grep -h "Assert failed in" "$build/${v}_$t.log" | sed -e 's/.*Assert failed in [^:]*: //' -e 's/ \[failed before\]//';
                 grep -h "Unreached cover statement" "$build/${v}_$t.log" | sed -e 's/.*statement at //' -e 's/^/unreached:/'; } \
             | propname | sort | uniq -c | awk '{printf "%s%s ", $2, ($1 > 1 ? "x" $1 : "")}')
        printf "%-20s %-6s %-7s %6.1fs  %s\n" "$v" "$t" "$status" "$(echo "$end - $start" | bc)" "$why"
    done
done
