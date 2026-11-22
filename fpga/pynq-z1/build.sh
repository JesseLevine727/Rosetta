#!/usr/bin/env bash
# Build the Rosetta PYNQ-Z1 harness bitstream around a Tiny Tapeout RTL tree.
#
#   ./build.sh <rtl-src-dir> [--top MODULE] [--out DIR] [--jobs N] [--sck-ratio N] [--glob]
#              [--keep-nettype] [--define NAME[=VALUE]]...
#
#   <rtl-src-dir>  directory holding the chip RTL (e.g. ~/Documents/Rosetta/src)
#   --top          TT top module (default: top_module from <dir>/../info.yaml,
#                  else tt_um_jesselevine727_rosetta)
#   --out          output directory (default: ./out)
#   --jobs         Vivado parallel jobs (default: 8)
#   --sck-ratio    AXI Quad SPI SCK divider from 60 MHz (default 16 -> 3.75 MHz)
#   --glob         take every *.v/*.sv in the dir instead of info.yaml source_files
#   --keep-nettype do not rewrite `default_nettype none -> wire in the build copies
#   --define       Verilog define for every source, repeatable (the Rosetta chip
#                  needs ROSETTA_SRAM_BEHAV: behavioural SRAM, block RAM on the FPGA)
#
# Output: <out>/rosetta.bit, <out>/rosetta.hwh, <out>/reports/*, <out>/build.log
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
VIVADO="${VIVADO:-/home/elfo/Documents/2025.1/Vivado/bin/vivado}"

usage() { sed -n '2,22p' "$0"; exit 1; }
[[ $# -ge 1 ]] || usage
RTL="$(cd "$1" && pwd)"; shift
TOP=""; OUT="$HERE/out"; JOBS=8; SCK=16; USE_GLOB=0; KEEP_NETTYPE=0; DEFINES=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --top) TOP="$2"; shift 2;;
        --out) OUT="$2"; shift 2;;
        --jobs) JOBS="$2"; shift 2;;
        --sck-ratio) SCK="$2"; shift 2;;
        --glob) USE_GLOB=1; shift;;
        --keep-nettype) KEEP_NETTYPE=1; shift;;
        --define) [[ "$2" =~ ^[A-Za-z_][A-Za-z0-9_]*(=[^[:space:]]*)?$ ]] || { echo "bad --define $2" >&2; exit 1; }
                  DEFINES+=("$2"); shift 2;;
        *) usage;;
    esac
done
mkdir -p "$OUT/gen"; OUT="$(cd "$OUT" && pwd)"

INFO="$(dirname "$RTL")/info.yaml"
GLOBARG=(); [[ $USE_GLOB -eq 1 ]] && GLOBARG=(--glob)
mapfile -t SRC < <("$HERE/scripts/chip_sources.sh" "$RTL" "${GLOBARG[@]}")
[[ ${#SRC[@]} -gt 0 ]] || { echo "no RTL sources found in $RTL" >&2; exit 1; }

# Vivado builds from copies in <out>/gen/rtl. By default `default_nettype none
# is rewritten to `default_nettype wire there: under `none`, IEEE 1800 makes
# an `input logic` port (data type, no net type) an error, which Vivado
# enforces (Synth 8-6735) while Yosys/Verilator accept it. The originals are
# never modified; --keep-nettype builds the files unchanged.
GEN_RTL="$OUT/gen/rtl"; rm -rf "$GEN_RTL"; mkdir -p "$GEN_RTL"
FILES=()
for f in "${SRC[@]}"; do
    dst="$GEN_RTL/$(basename "$f")"
    [[ -e "$dst" ]] && { echo "duplicate source file name $(basename "$f")" >&2; exit 1; }
    if [[ $KEEP_NETTYPE -eq 1 ]]; then
        cp "$f" "$dst"
    else
        sed -E 's/^([[:space:]]*)`default_nettype[[:space:]]+none\b/\1`default_nettype wire \/* rosetta-fpga build: was none *\//' "$f" > "$dst"
        cmp -s "$f" "$dst" || echo "build.sh: $(basename "$f"): default_nettype none -> wire (build copy only)"
    fi
    FILES+=("$dst")
done

if [[ -z "$TOP" && -f "$INFO" ]]; then
    TOP="$(awk -F: '/^[[:space:]]*top_module:/{v=$2; sub(/#.*/,"",v); gsub(/[ "\047\t]/,"",v); print v; exit}' "$INFO")"
fi
TOP="${TOP:-tt_um_jesselevine727_rosetta}"
echo "build.sh: top module $TOP"
printf '%s\n' "${FILES[@]}" | tee "$OUT/gen/chip_files.txt"
sed "s/@CHIP_TOP@/$TOP/" "$HERE/hdl/templates/rosetta_chip.v.in" > "$OUT/gen/rosetta_chip.v"

# provenance
{
    echo "rtl_dir: $RTL"; echo "top: $TOP"; echo "sck_ratio: $SCK"; echo "date: $(date -Is)"
    if git -C "$RTL" rev-parse HEAD >/dev/null 2>&1; then
        echo "rtl_git: $(git -C "$RTL" rev-parse HEAD)$(git -C "$RTL" diff --quiet -- . || echo ' (dirty)')"
    fi
    echo "keep_nettype: $KEEP_NETTYPE"
    echo "defines: ${DEFINES[*]:-}"
    echo "harness: $(cat "$HERE"/hdl/*.v "$HERE"/hdl/templates/* "$HERE"/tcl/*.tcl "$HERE"/xdc/*.xdc | sha256sum | cut -d' ' -f1)"
    sha256sum "${SRC[@]}"
} > "$OUT/build_info.txt"

cd "$OUT"
"$VIVADO" -mode batch -nojournal -log "$OUT/build.log" -source "$HERE/tcl/build.tcl" \
    -tclargs "$OUT/gen/chip_files.txt" "$OUT" "$JOBS" "$SCK" "$OUT/gen/rosetta_chip.v" "${DEFINES[*]:-}"

echo
echo "== per-clock slack (ns) =="; cat "$OUT/reports/clock_slack.txt"
echo "== MMCM =="; cat "$OUT/reports/mmcm_settings.txt"
ls -l "$OUT/rosetta.bit" "$OUT/rosetta.hwh"
