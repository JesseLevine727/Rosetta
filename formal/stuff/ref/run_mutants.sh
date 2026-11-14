#!/usr/bin/env bash
# Non-vacuity check for formal/stuff against the PROPERTY WRITER'S PRIVATE
# REFERENCE MODEL (ref/rosetta_stuff.sv, not the design).
#
# For each variant (NONE = clean reference, MUT_* = one planted bug) this
# copies ../stuff.sby into ref/build/, points the design line at the
# reference model, adds `read -define <MUT>`, and runs the given tasks.
#
#   usage: ref/run_mutants.sh [tasks...] [-- variants...]
#   tasks: prove | bmc | cover | bmckg  (bmckg = the bmc task at depth 16 with
#          `smtbmc --keep-going`, listing every assertion that fails
#          within the depth instead of only the first)
#   e.g.   ref/run_mutants.sh prove bmc cover
#          ref/run_mutants.sh bmc -- MUT_OFF_BY_ONE MUT_CLEAR_IGNORED
set -u
here="$(cd "$(dirname "$0")/.." && pwd)"          # formal/stuff
source /home/elfo/tools/oss-cad-suite-dl/oss-cad-suite/environment

tasks=()
variants=()
seen_sep=0
for a in "$@"; do
    if [ "$a" = "--" ]; then seen_sep=1; continue; fi
    if [ $seen_sep = 0 ]; then tasks+=("$a"); else variants+=("$a"); fi
done
[ ${#tasks[@]} = 0 ] && tasks=(prove bmc cover)
[ ${#variants[@]} = 0 ] && variants=(NONE $(grep -o '^`ifdef MUT_[A-Z0-9_]*\|^`elsif MUT_[A-Z0-9_]*' "$here/ref/rosetta_stuff.sv" | sed 's/.* //' | sort -u))

build="$here/ref/build"
mkdir -p "$build"
cd "$here"
for v in "${variants[@]}"; do
    sby_file="$build/$v.sby"
    sed -e 's#^rosetta_stuff.sv .*#rosetta_stuff.sv ref/rosetta_stuff.sv#' "$here/stuff.sby" > "$sby_file"
    if [ "$v" != NONE ]; then
        sed -i -e "s#^read -formal rosetta_stuff.sv#read -define $v\nread -formal rosetta_stuff.sv#" "$sby_file"
    fi
    sed -e 's#^bmc: smtbmc #bmc: smtbmc --keep-going #' -e 's#^bmc: depth .*#bmc: depth 16#' \
        "$sby_file" > "$build/${v}_kg.sby"
    for t in "${tasks[@]}"; do
        start=$(date +%s.%N)
        if [ "$t" = bmckg ]; then
            sby -f "$build/${v}_kg.sby" bmc > "$build/${v}_$t.log" 2>&1
            wd="$build/${v}_kg_bmc"
        else
            sby -f "$sby_file" "$t" > "$build/${v}_$t.log" 2>&1
            wd="$build/${v}_$t"
        fi
        end=$(date +%s.%N)
        status=$(grep -E "DONE \((PASS|FAIL|UNKNOWN|ERROR)" "$build/${v}_$t.log" | tail -1 | sed 's/.*DONE (\([A-Z]*\).*/\1/')
        [ -z "$status" ] && status=ERROR
        # assertions that failed / covers not reached (from the full summary)
        why=$( { grep -h "Assert failed in" "$build/${v}_$t.log" | sed -e 's/.*: //' -e 's/ \[failed before\]//';
                 cat "$wd/FAIL" "$wd/UNKNOWN" "$wd/ERROR" 2>/dev/null \
                 | awk 'function leaf(x,  a, n) { n = split(x, a, "."); return a[n] }
                        /failed assertion/ {print leaf($3); next}
                        /unreached cover statements:/ {u=1; next}
                        u && /^  / {print "unreached:" leaf($1); next} {u=0}'; } \
             | sort -u | tr '\n' ' ')
        printf "%-24s %-6s %-6s %6.1fs  %s\n" "$v" "$t" "$status" "$(echo "$end - $start" | bc)" "$why"
    done
done
