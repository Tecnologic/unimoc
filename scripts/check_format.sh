#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
#
# clang-format check of all UNIMOC C/C++ sources (vendored code excluded).
#   scripts/check_format.sh         report files that are not formatted (exit 1)
#   scripts/check_format.sh --fix   format them in place
set -euo pipefail
cd "$(dirname "$0")/.."

mapfile -t files < <(git ls-files '*.c' '*.cpp' '*.h' '*.hpp' |
  grep -v -e '^lib/cymon/' -e '^hardware/batterycasecontroller/')

if [[ "${1:-}" == "--fix" ]]; then
  clang-format -i "${files[@]}"
else
  clang-format --dry-run --Werror "${files[@]}"
fi
