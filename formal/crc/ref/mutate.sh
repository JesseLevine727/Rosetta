#!/usr/bin/env bash
# Non-vacuity runs for formal/crc: the property writer's private reference
# model (ref/rosetta_crc.sv), clean and with each planted bug, against the
# unchanged properties and crc.sby. Nothing is written into the repo.
#
#   ref/mutate.sh                 clean + every mutant, all three tasks
#   ref/mutate.sh MUT_PRIO ...    just these (CLEAN = no bug)
#   TASKS="bmc" ref/mutate.sh     choose tasks (default: prove bmc cover)
#   OUT=/some/dir ref/mutate.sh   keep run directories there
#   DEFS="CRC_FV_CFG_STABLE"      extra defines for the properties
#   KEEPGOING=1                   bmc keeps going after a failure, so every
#                                 assertion that can fail is listed
set -u
here=$(cd "$(dirname "$0")/.." && pwd)
out=${OUT:-$(mktemp -d)}
tasks=${TASKS:-"prove bmc cover"}
all="CLEAN MUT_FB_TAP MUT_NO_MASK MUT_PRIO MUT_XOR_STATE MUT_WIDTH_OB1 MUT_HOLD
     MUT_SYNC_RST MUT_RST_VAL MUT_OUT_UNMASK MUT_COMB_EN MUT_XOR_UNMASK
     MUT_POLY_SHIFT MUT_W1 MUT_W32_MASK"
muts=${*:-$all}
src=/home/elfo/tools/oss-cad-suite-dl/oss-cad-suite/environment
[ -f "$src" ] && . "$src"

for m in $muts; do
    d=$out/$m
    rm -rf "$d"; mkdir -p "$d"
    {
        [ "$m" != CLEAN ] && echo "\`define $m"
        cat "$here/ref/rosetta_crc.sv"
    } > "$d/rosetta_crc.sv"
    {
        for x in ${DEFS:-}; do echo "\`define $x"; done
        cat "$here/crc_props.sv"
    } > "$d/crc_props.sv"
    sed -e "s|^rosetta_crc.sv .*|rosetta_crc.sv $d/rosetta_crc.sv|" \
        -e "s|^crc_props.sv\$|crc_props.sv $d/crc_props.sv|" \
        -e "s|^crc_fv.sv\$|crc_fv.sv $here/crc_fv.sv|" \
        -e "s|^crc_ka_states.svh\$|crc_ka_states.svh $here/crc_ka_states.svh|" \
        "$here/crc.sby" > "$d/crc.sby"
    [ -n "${KEEPGOING:-}" ] && sed -i 's|^bmc: smtbmc |bmc: smtbmc --keep-going |' "$d/crc.sby"
    (cd "$d" && sby -f crc.sby $tasks > sby.log 2>&1)
    for t in $tasks; do
        log=$d/crc_$t/logfile.txt
        st=$(grep -h "DONE" "$log" 2>/dev/null | tail -1 | sed 's/.*DONE/DONE/')
        tm=$(grep -h "Elapsed clock time" "$log" 2>/dev/null | tail -1 | sed 's/.*: //')
        # failed assertions: label (or file:line) and step where reported
        fails=$(grep -h "summary: *failed assertion" "$log" 2>/dev/null |
                sed -E 's/.*failed assertion crc_fv\.([^ ]*)( at [^ ]*)?( step ([0-9]+))?.*/\1@\4/' |
                sed 's/@$//' | sort -u | tr '\n' ' ')
        unreached=$(grep -h "Unreached cover statement at" "$log" 2>/dev/null |
                sed -E 's/.*statement at crc_fv\.([^:]*): *(.*)/\1.\2/' | sort -u | tr '\n' ' ')
        printf '%-15s %-6s %-20s %-10s %s%s\n' "$m" "$t" "${st:-NO RESULT}" "${tm:-}" \
            "$fails" "${unreached:+UNREACHED: $unreached}"
    done
done
echo "run directories: $out"
