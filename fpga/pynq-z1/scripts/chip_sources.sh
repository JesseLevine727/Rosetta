#!/usr/bin/env bash
# Print the chip RTL source list for <rtl-src-dir>, one absolute path per line.
#   scripts/chip_sources.sh <rtl-src-dir> [--glob]
# Default: the `source_files:` list of <rtl-src-dir>/../info.yaml (Tiny Tapeout
# convention); falls back to (or with --glob uses) every *.v / *.sv in the dir.
# Header files (*.vh, *.svh) in the dir are always appended.
set -euo pipefail
RTL="$(cd "$1" && pwd)"; USE_GLOB=0; [[ "${2:-}" == "--glob" ]] && USE_GLOB=1
INFO="$(dirname "$RTL")/info.yaml"
yaml_sources() {
    awk '/^[[:space:]]*source_files:/{f=1;next}
         f && /^[[:space:]]*-/{s=$0; sub(/^[[:space:]]*-[[:space:]]*/,"",s); sub(/[[:space:]]*#.*/,"",s);
                              gsub(/["\047]/,"",s); if (s!="") print s; next}
         f && /^[[:space:]]*[A-Za-z_]+:/{f=0}' "$1"
}
if [[ $USE_GLOB -eq 0 && -f "$INFO" && -n "$(yaml_sources "$INFO")" ]]; then
    echo "chip_sources: from $INFO" >&2
    yaml_sources "$INFO" | while read -r s; do
        [[ -f "$RTL/$s" ]] || { echo "chip_sources: missing $RTL/$s" >&2; exit 1; }
        echo "$RTL/$s"
    done
else
    echo "chip_sources: globbed from $RTL" >&2
    find "$RTL" -maxdepth 1 \( -name '*.v' -o -name '*.sv' \) | sort
fi
find "$RTL" -maxdepth 1 \( -name '*.vh' -o -name '*.svh' \) | sort
