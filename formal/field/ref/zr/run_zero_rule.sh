#!/usr/bin/env bash
# Zero-rule check from the property side for formal/field (amendment H3).
#
#  dir1  PREVIOUS property files (git HEAD formal/field/field_props.sv,
#        field_model.sv, field_spec.svh, field_fv.sv, field.sby) against the
#        NEW reference model (ref/rosetta_field.sv) with cfg_rawedge_i = 0
#        (ref/zr/wrap_new_as_old.sv). All tasks must PASS.
#  dir2  NEW property files with FIELD_ASSUME_H3_OFF against the PRE-
#        amendment reference model (git HEAD formal/field/ref/rosetta_field.sv,
#        ref/zr/wrap_old_as_new.sv). prove, prove_e2e and bmc must PASS (the
#        H3 covers are unreachable there).
#
#   usage: ref/zr/run_zero_rule.sh <dir1|dir2> [tasks...]
set -u
here="$(cd "$(dirname "$0")/../.." && pwd)"          # formal/field
source /home/elfo/tools/oss-cad-suite-dl/oss-cad-suite/environment
JOBS=${JOBS:-4}
TMO=${TMO:-1800}
dir=${1:-dir1}; shift || true
tasks=("$@"); [ ${#tasks[@]} = 0 ] && tasks=(prove prove_e2e bmc cover)
b="$here/ref/build/zr"
mkdir -p "$b"
cd "$here"
for f in field_props.sv field_model.sv field_spec.svh field_fv.sv field.sby; do
    git show HEAD:formal/field/$f > "$b/old_$f"
done
git show HEAD:formal/field/ref/rosetta_field.sv \
    | sed 's/^module rosetta_field (/module rosetta_field_pre (/' > "$b/old_ref.sv"
sed 's/^module rosetta_field (/module rosetta_field_h3 (/' ref/rosetta_field.sv > "$b/new_ref.sv"
cat "$b/new_ref.sv" ref/zr/wrap_new_as_old.sv > "$b/dut_dir1.sv"
cat "$b/old_ref.sv" ref/zr/wrap_old_as_new.sv > "$b/dut_dir2.sv"
if [ "$dir" = dir1 ]; then
    sed -e "s#^rosetta_field.sv .*#rosetta_field.sv $b/dut_dir1.sv#" \
        -e "s#^field_spec.svh\$#field_spec.svh $b/old_field_spec.svh#" \
        -e "s#^field_model.sv\$#field_model.sv $b/old_field_model.sv#" \
        -e "s#^field_props.sv\$#field_props.sv $b/old_field_props.sv#" \
        -e "s#^field_fv.sv\$#field_fv.sv $b/old_field_fv.sv#" \
        "$b/old_field.sby" > "$b/zr_dir1.sby"
else
    sed -e "s#^rosetta_field.sv .*#rosetta_field.sv $b/dut_dir2.sv#" \
        -e "s#^field_spec.svh\$#field_spec.svh $here/field_spec.svh#" \
        -e "s#^field_model.sv\$#field_model.sv $here/field_model.sv#" \
        -e "s#^field_props.sv\$#field_props.sv $here/field_props.sv#" \
        -e "s#^field_fv.sv\$#field_fv.sv $here/field_fv.sv#" \
        -e "s#^read_verilog -sv -formal rosetta_field.sv#read -define FIELD_ASSUME_H3_OFF\nread_verilog -sv -formal rosetta_field.sv#" \
        field.sby > "$b/zr_dir2.sby"
fi
run_one() {
    t=$1
    cp "$b/zr_$dir.sby" "$b/zr_${dir}__$t.sby"
    start=$(date +%s.%N)
    timeout "$TMO" sby -f -d "$b/zr_${dir}_$t" "$b/zr_${dir}__$t.sby" "$t" > "$b/zr_${dir}_$t.log" 2>&1
    end=$(date +%s.%N)
    status=$(grep -E "DONE \((PASS|FAIL|UNKNOWN|ERROR)" "$b/zr_${dir}_$t.log" | tail -1 | sed 's/.*DONE (\([A-Z]*\).*/\1/')
    [ -z "$status" ] && status=TIMEOUT
    printf "zr %-5s %-9s %-8s %7.1fs  %s\n" "$dir" "$t" "$status" "$(echo "$end - $start" | bc)" \
        "$(grep -h "Assert failed in\|Unreached cover" "$b/zr_${dir}_$t.log" | sed 's/.*: //' | sort -u | tr '\n' ' ')"
}
export -f run_one
export b dir TMO
printf "%s\n" "${tasks[@]}" | xargs -P "$JOBS" -n 1 bash -c 'run_one "$0"'
