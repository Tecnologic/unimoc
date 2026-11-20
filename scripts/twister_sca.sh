#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Runs all UNIMOC twister scenarios, then clang-tidy (through CodeChecker,
# the analyzer of Zephyr's SCA) on the repository sources of these builds.
# A twister failure or a clang-tidy finding in UNIMOC code fails the run.
# Run from anywhere inside the west workspace; extra arguments are passed to
# twister, for example
#   scripts/twister_sca.sh -p native_sim/native/64
# Defaults: all scenarios of the repository, platforms native_sim/native/64,
# unimoc_jacobi and nucleo_h743zi; a -T or -p argument replaces the respective default.
#
# clang-tidy does not run inside each build (ZEPHYR_SCA_VARIANT) any more:
# the shared sources are compiled by up to 37 builds, and analysing each
# copy took about three quarters of the run. scripts/sca_compile_db.py
# merges the compile databases of the twister output to one command per
# source file and platform, and CodeChecker analyses that once, in parallel.
# The report is in <twister output>/sca/codechecker/.
set -euo pipefail
repo="$(cd "$(dirname "$0")/.." && pwd)"
# Without the analyzers CodeChecker finds nothing and the run passes silently.
for tool in clang-tidy CodeChecker; do
  command -v "$tool" >/dev/null || { echo "error: $tool not found in PATH" >&2; exit 1; }
done
topdir="$(cd "$repo" && west topdir)"

platforms=(-p native_sim/native/64 -p unimoc_jacobi -p nucleo_h743zi)
roots=(-T "$repo")
for arg in "$@"; do
  [[ "$arg" == "-p" || "$arg" == --platform* ]] && platforms=()
  [[ "$arg" == "-T" || "$arg" == --testsuite-root* ]] && roots=()
done

outdir=twister-out
args=("$@")
for ((i = 0; i < ${#args[@]}; i++)); do
  case "${args[i]}" in
    -O | --outdir) outdir="${args[i + 1]}" ;;
    --outdir=*) outdir="${args[i]#--outdir=}" ;;
  esac
done

cd "$topdir"
twister_status=0
west twister "${roots[@]}" "${platforms[@]}" --inline-logs "$@" || twister_status=$?

# clang-tidy, once per source file and platform, on the builds twister left.
report="$outdir/sca/codechecker"
rm -rf "$report"
mkdir -p "$report"
python3 "$repo/scripts/sca_compile_db.py" "$repo" "$outdir" "$report/compile_commands.json"
analyze_status=0
CodeChecker analyze "$report/compile_commands.json" \
  --analyzers clang-tidy \
  --analyzer-config clang-tidy:take-config-from-directory=true \
  --skip "$repo/scripts/codechecker_skip.txt" \
  --keep-gcc-include-fixed --keep-gcc-intrin \
  --no-missing-checker-error \
  --jobs "$(nproc)" \
  --output "$report/codechecker.plist" || analyze_status=$?
parse_status=0
CodeChecker parse "$report/codechecker.plist" --trim-path-prefix "$topdir" || parse_status=$?

if ((twister_status != 0)); then
  echo "error: twister failed (exit $twister_status)" >&2
fi
if ((analyze_status != 0 && analyze_status != 2)); then
  # 2 only says that reports were found; parse below names them.
  echo "error: CodeChecker analyze failed (exit $analyze_status)" >&2
fi
if ((parse_status != 0)); then
  echo "error: clang-tidy findings (report in $report)" >&2
fi
((twister_status == 0 && (analyze_status == 0 || analyze_status == 2) && parse_status == 0))
