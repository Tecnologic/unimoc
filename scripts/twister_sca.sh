#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Runs all UNIMOC twister scenarios with clang-tidy as Zephyr static code
# analysis (ZEPHYR_SCA_VARIANT=codechecker). A clang-tidy finding in UNIMOC
# code fails the build of that scenario. Run from anywhere inside the west
# workspace; extra arguments are passed to twister, for example
#   scripts/twister_sca.sh -p native_sim/native/64
# Defaults: all scenarios of the repository, platforms native_sim/native/64
# and bcc; a -T or -p argument replaces the respective default.
set -euo pipefail
repo="$(cd "$(dirname "$0")/.." && pwd)"
topdir="$(cd "$repo" && west topdir)"

platforms=(-p native_sim/native/64 -p bcc)
roots=(-T "$repo")
for arg in "$@"; do
  [[ "$arg" == "-p" || "$arg" == --platform* ]] && platforms=()
  [[ "$arg" == "-T" || "$arg" == --testsuite-root* ]] && roots=()
done

cd "$topdir"
exec west twister "${roots[@]}" "${platforms[@]}" --inline-logs \
  -x=ZEPHYR_SCA_VARIANT=codechecker \
  "-x=CODECHECKER_ANALYZE_OPTS=--analyzer-config;clang-tidy:take-config-from-directory=true;--analyzers;clang-tidy;--skip;$repo/scripts/codechecker_skip.txt;--no-missing-checker-error" \
  -x=CODECHECKER_PARSE_EXIT_STATUS=y \
  "$@"
