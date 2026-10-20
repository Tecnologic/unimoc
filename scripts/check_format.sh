#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
#
# clang-format check of all UNIMOC C/C++ sources (vendored code excluded).
#   scripts/check_format.sh         report files that are not formatted (exit 1)
#   scripts/check_format.sh --fix   format them in place
#
# The formatting depends on the clang-format version, so the version is
# pinned (the one Zephyr's own CI uses): pip install clang-format==22.1.1
# Set CLANG_FORMAT to use a binary that is not first on PATH.
set -euo pipefail
cd "$(dirname "$0")/.."

required_version="22.1.1"
clang_format="${CLANG_FORMAT:-clang-format}"
if ! "$clang_format" --version 2>/dev/null | grep -q "version ${required_version}\b"; then
  echo "error: clang-format ${required_version} required, found: $("$clang_format" --version 2>/dev/null || echo none)" >&2
  echo "       pip install clang-format==${required_version}" >&2
  exit 2
fi

mapfile -t files < <(git ls-files '*.c' '*.cpp' '*.h' '*.hpp' |
  grep -v -e '^lib/cymon/' -e '^hardware/batterycasecontroller/')

if [[ "${1:-}" == "--fix" ]]; then
  "$clang_format" -i "${files[@]}"
else
  "$clang_format" --dry-run --Werror "${files[@]}"
fi
