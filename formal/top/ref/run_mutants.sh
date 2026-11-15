#!/usr/bin/env bash
# Non-vacuity check for formal/top against the PROPERTY WRITER'S PRIVATE
# REFERENCE MODEL (ref/rosetta_top.sv, not the design).
#
# For each variant (NONE = clean reference, MUT_* = one planted bug) and each
# job (top_wire, top_chip, top_struct, top_ni, top_bringup, ...) this writes
# ref/build/<variant>__<job>.sby from ../<job>.sby with the design line
# pointed at the reference model (absolute paths) and `read -define <MUT>`
# added, then runs the given tasks. Runs go in parallel (JOBS, default 6),
# each capped at TMO seconds (default 1800 = the 30-minute cap).
#
#   usage: [JOBS=n] [TMO=s] [DEFS="X Y"] [TAG=t] ref/run_mutants.sh \
#              job:task [job:task ...] [-- variants...]
#   e.g.   ref/run_mutants.sh top_wire:prove top_chip:prove -- NONE MUT_UIO_OE_POL
#   With no variants: NONE and every MUT_* in ref/rosetta_top.sv.
#
# Output, one line per run:  variant job:task STATUS seconds  failing-props
set -u
here="$(cd "$(dirname "$0")/.." && pwd)"          # formal/top
source /home/elfo/tools/oss-cad-suite-dl/oss-cad-suite/environment
JOBS=${JOBS:-6}
TMO=${TMO:-1800}
DEFS=${DEFS:-}
TAG=${TAG:-}

runs=(); variants=(); seen=0
for a in "$@"; do
    if [ "$a" = "--" ]; then seen=1; continue; fi
    if [ $seen = 0 ]; then runs+=("$a"); else variants+=("$a"); fi
done
[ ${#runs[@]} = 0 ] && runs=(top_wire:prove top_struct:prove top_chip:prove)
[ ${#variants[@]} = 0 ] && variants=(NONE $(grep -o '`ifdef MUT_[A-Z0-9_]*\|`ifndef MUT_[A-Z0-9_]*\|`elsif MUT_[A-Z0-9_]*' "$here/ref/rosetta_top.sv" | sed 's/.* //' | grep -v '^MUT_$' | sort -u))

build="$here/ref/build"
mkdir -p "$build"

mk_sby() {   # variant job -> build/<variant><TAG>__<job>.sby
    v0=$1; job=$2; v=$v0$TAG
    out="$build/${v}__${job}.sby"
    # every non-comment [files] entry that is a bare name gets an absolute
    # source; the design line is pointed at the reference model
    awk -v here="$here" '
        /^\[/ { sec=$0 }
        sec=="[files]" && /^rosetta_top\.sv / { print "rosetta_top.sv " here "/ref/rosetta_top.sv"; next }
        sec=="[files]" && /^[A-Za-z0-9_.]+$/ { print $0 " " here "/" $0; next }
        sec=="[files]" && /^[A-Za-z0-9_.]+ \.\.\// { split($0, f, " "); print f[1] " " here "/" f[2]; next }
        { print }' "$here/$job.sby" > "$out.tmp"
    defs="$DEFS"
    [ "$v0" != NONE ] && defs="$defs $v0"
    for d in $defs; do
        sed -i -e "s#^read -formal rosetta_top.sv#read -define $d\nread -formal rosetta_top.sv#" "$out.tmp"
    done
    mv "$out.tmp" "$out"
}

run_one() {
    v=$1$TAG; job=${2%%:*}; t=${2##*:}
    sby_file="$build/${v}__${job}.sby"
    tcopy="$build/${v}__${job}__$t.sby"       # one copy per task: own status db
    log="$build/${v}__${job}_$t.log"
    wd="$build/${v}__${job}_$t"
    cp "$sby_file" "$tcopy"
    start=$(date +%s.%N)
    timeout "$TMO" sby -f -d "$wd" "$tcopy" "$t" > "$log" 2>&1
    end=$(date +%s.%N)
    status=$(grep -E "DONE \((PASS|FAIL|UNKNOWN|ERROR)" "$log" | tail -1 | sed 's/.*DONE (\([A-Z]*\).*/\1/')
    [ -z "$status" ] && status=TIMEOUT
    why=""
    [ "$status" != PASS ] && why=$( { grep -h "Assert failed in" "$log" | grep -v "induction" | sed -e 's/.*: //' -e 's/ \[failed before\]//' -e 's/^.*\.\(u_[a-z_]*\)\./\1./';
             grep -h "Unreached cover statement" "$log" | sed -e 's/.*statement at //' -e 's/^/unreached:/';
             grep -h "^ERROR\|ERROR:" "$log" | head -2 | cut -c1-120; } \
           | sed -e 's/ *([^)]*)//' | sort -u | tr '\n' ' ')
    printf "%-24s %-20s %-8s %7.1fs  %s\n" "$v" "$job:$t" "$status" "$(echo "$end - $start" | bc)" "$why"
}
export -f run_one
export build TMO TAG

for v in "${variants[@]}"; do
    for r in "${runs[@]}"; do mk_sby "$v" "${r%%:*}"; done
done

for v in "${variants[@]}"; do for r in "${runs[@]}"; do echo "$v $r"; done; done \
    | xargs -P "$JOBS" -n 2 bash -c 'run_one "$0" "$1"'
