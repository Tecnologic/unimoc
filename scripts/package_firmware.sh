#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Collect the files of one firmware build for upload (CI and release).
#   package_firmware.sh <target> <build-dir> <name-suffix> <out-dir>
# <target> is jacobi, nucleo_h743zi or sim; the files are named unimoc-<target>-<name-suffix>.*
# jacobi and nucleo_h743zi are sysbuild builds (MCUboot and the drive): .elf/.hex/.bin/.map of the drive,
# -bootloader.hex of MCUboot and -signed.hex (the image MCUboot runs; both for a first flash),
# -signed.bin (the same as binary) and -update.bin (the same with the UNIMOC header: the file that BEGIN_SOFTWARE_UPDATE
# downloads, `docs/zephyr_workflow.md`, "Bootloader and firmware update").
# The sim also writes the register description file unimoc-registers-<name-suffix>.json
# (docs/gui_plan.md, section 4): every register with type, default, unit,
# limits, effect and help, from the simulated drive on a fresh flash file.
# A new board or simulator adds a case here and a row to the matrix of
# .github/workflows/build-firmware.yml (docs/release_plan.md).
set -euo pipefail

target="$1"
build="$2"
suffix="$3"
out="$4"
mkdir -p "$out"

case "$target" in
jacobi | nucleo_h743zi)
  app="$build/drive/zephyr"
  for ext in elf hex bin map; do
    cp "$app/zephyr.$ext" "$out/unimoc-$target-$suffix.$ext"
  done
  cp "$app/zephyr.signed.bin" "$out/unimoc-$target-$suffix-signed.bin"
  cp "$build/mcuboot/zephyr/zephyr.hex" "$out/unimoc-$target-$suffix-bootloader.hex"
  cp "$app/zephyr.signed.hex" "$out/unimoc-$target-$suffix-signed.hex"
  vcs="$(git -C "$(dirname "$0")/.." rev-parse --short=16 HEAD)"
  python3 "$(dirname "$0")/make_firmware_image.py" --board "$(grep -m1 '^CONFIG_BOARD=' "$app/.config" | cut -d'"' -f2)" \
    --vcs "$vcs" "$app/zephyr.signed.bin" "$out/unimoc-$target-$suffix-update.bin"
  ;;
sim)
  # tar keeps the executable bit, which the artifact zip drops.
  tar -C "$build/zephyr" -czf "$out/unimoc-native_sim-$suffix.tar.gz" \
    --transform "s/zephyr.exe/unimoc-native_sim/" zephyr.exe
  flash="$(mktemp -d)"
  trap 'rm -rf "$flash"' EXIT
  timeout 60 "$build/zephyr/zephyr.exe" -flash="$flash/flash.bin" \
    --describe-registers="$out/unimoc-registers-$suffix.json" >/dev/null
  python3 -m json.tool "$out/unimoc-registers-$suffix.json" >/dev/null
  ;;
*)
  echo "unknown target: $target" >&2
  exit 1
  ;;
esac
