#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Collect the files of one firmware build for upload (CI and release).
#   package_firmware.sh <target> <build-dir> <name-suffix> <out-dir>
# <target> is bcc or sim; the files are named unimoc-<target>-<name-suffix>.*
# A new board or simulator adds a case here and a row to the matrix of
# .github/workflows/build-firmware.yml (docs/release_plan.md).
set -euo pipefail

target="$1"
build="$2"
suffix="$3"
out="$4"
mkdir -p "$out"

case "$target" in
bcc)
  for ext in elf hex bin; do
    cp "$build/zephyr/zephyr.$ext" "$out/unimoc-bcc-$suffix.$ext"
  done
  cp "$build/zephyr/zephyr.map" "$out/unimoc-bcc-$suffix.map"
  ;;
sim)
  # tar keeps the executable bit, which the artifact zip drops.
  tar -C "$build/zephyr" -czf "$out/unimoc-native_sim-$suffix.tar.gz" \
    --transform "s/zephyr.exe/unimoc-native_sim/" zephyr.exe
  ;;
*)
  echo "unknown target: $target" >&2
  exit 1
  ;;
esac
