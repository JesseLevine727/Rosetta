#!/usr/bin/env bash
# Non-vacuity check for formal/events against the PROPERTY WRITER'S PRIVATE
# REFERENCE MODEL (ref/rosetta_events.sv, not the design).
#
# For each variant (NONE = clean reference, MUT_* = one planted bug) this
# writes ref/build/<variant>.sby from ../events.sby with the design line
# pointed at the reference model (absolute paths) and `read -define <MUT>`
# added, then runs the given tasks. Variants run in parallel (JOBS, default 8).
#
#   usage: [DEFS="X Y"] [TAG=name] ref/run_mutants.sh [tasks...] [-- variants...]
#   tasks: prove | bmc | cover          (default: all three)
#   DEFS:  extra `read -define` for every variant, e.g. EVENTS_NO_MODEL_CHECK
#          (port-level statements only); TAG is appended to the variant name
#          in build/ to keep runs apart.
#
# Properties inside generate loops cannot carry labels in Yosys, so they are
# tagged `// @name` on their source line; failures are reported by that name.
set -u
here="$(cd "$(dirname "$0")/.." && pwd)"          # formal/events
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
[ ${#tasks[@]} = 0 ] && tasks=(prove bmc cover)
[ ${#variants[@]} = 0 ] && variants=(NONE $(grep -o '`ifdef MUT_[A-Z0-9_]*\|`ifndef MUT_[A-Z0-9_]*\|`elsif MUT_[A-Z0-9_]*' "$here/ref/rosetta_events.sv" | sed 's/.* //' | sort -u))

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
    # failing property: label, or the `// @tag` on the reported source line
    why=$( { grep -h "Assert failed in\|failed assertion" "$log" | grep -o '[a-z_]*\.sv:[0-9]*\|: [a-z][a-z0-9_]*$' ;
             grep -h "Unreached cover statement" "$log" | sed -e 's/.*statement at //' -e 's/^/unreached:/'; } |
           while read -r x; do
               if [[ "$x" =~ ^([a-z_]+\.sv):([0-9]+) ]]; then
                   f=${BASH_REMATCH[1]}; l=${BASH_REMATCH[2]}
                   tag=$(sed -n "${l}p" "$here/$f" 2>/dev/null | grep -o '// @[a-z0-9_]*' | sed 's#// @##')
                   [ -z "$tag" ] && tag=$(sed -n "${l}p" "$here/$f" 2>/dev/null | grep -o '^ *[a-z][a-z0-9_]*:' | tr -d ' :')
                   if [ -n "$tag" ]; then echo "$tag"; else echo "$f:$l"; fi
               else
                   echo "$x" | sed 's/^: //'
               fi
           done | sort -u | tr '\n' ' ')
    printf "%-22s %-6s %-8s %7.1fs  %s\n" "$v" "$t" "$status" "$(echo "$end - $start" | bc)" "$why"
}
export -f run_one
export build TMO TAG here

for v0 in "${variants[@]}"; do
    v=$v0$TAG
    sed -e "s#^rosetta_events.sv .*#rosetta_events.sv $here/ref/rosetta_events.sv#" \
        -e "s#^events_props.sv\$#events_props.sv $here/events_props.sv#" \
        -e "s#^events_fv.sv\$#events_fv.sv $here/events_fv.sv#" \
        "$here/events.sby" > "$build/$v.sby"
    defs="$DEFS"
    [ "$v0" != NONE ] && defs="$defs $v0"
    for d in $defs; do
        sed -i -e "s#^read_verilog -sv -formal rosetta_events.sv#read -define $d\nread_verilog -sv -formal rosetta_events.sv#" "$build/$v.sby"
    done
done

for v in "${variants[@]}"; do for t in "${tasks[@]}"; do echo "$v $t"; done; done \
    | xargs -P "$JOBS" -n 2 bash -c 'run_one "$0" "$1"'
