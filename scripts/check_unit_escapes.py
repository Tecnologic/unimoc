#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Ratchet on the escapes from the unit types (docs: TODO.md, "Unit types").
#
# Counts per C++ file the calls of `.Value()` and the constructions of a unit
# type from a raw number (`Current{x}`, `unit::Voltage(x)`; the zero `Current{}`
# does not count). The counts may only go down: a file above its line in
# scripts/unit_escapes_baseline.txt fails the check. New files start at 0, so
# new code is written in units or, at a real boundary (DSDL serialization,
# cymon float mirror, register encoding, raw ADC counts), raises the baseline
# in the same change, visible in the review.
#
#   scripts/check_unit_escapes.py            check (exit 1 on growth)
#   scripts/check_unit_escapes.py --update   rewrite the baseline to the current counts
from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BASELINE = ROOT / "scripts" / "unit_escapes_baseline.txt"
UNITS = ROOT / "lib" / "units" / "units.hpp"


def unit_type_names() -> list[str]:
    text = UNITS.read_text(encoding="utf-8")
    return re.findall(r"^using (\w+) = Unit<", text, flags=re.MULTILINE)


def sources() -> list[str]:
    files = subprocess.run(
        ["git", "ls-files", "*.cpp", "*.hpp", "*.h"], cwd=ROOT, check=True, capture_output=True, text=True
    ).stdout.split()
    # The units themselves and the vendored trace library are not counted.
    return [f for f in files if not f.startswith("lib/cymon/") and f != "lib/units/units.hpp"]


def count(path: str, construction: re.Pattern[str]) -> int:
    text = (ROOT / path).read_text(encoding="utf-8")
    return len(re.findall(r"\.Value\(\)", text)) + len(construction.findall(text))


def main() -> int:
    names = "|".join(unit_type_names())
    construction = re.compile(rf"\b(?:unit::)?(?:{names})[{{(](?![}})])")
    counts = {path: n for path in sources() if (n := count(path, construction)) > 0}

    if "--update" in sys.argv[1:]:
        lines = [f"{n} {path}" for path, n in sorted(counts.items())]
        header = "# Unit-type escapes per file (scripts/check_unit_escapes.py); may only go down.\n"
        BASELINE.write_text(header + "\n".join(lines) + "\n", encoding="utf-8")
        print(f"baseline: {sum(counts.values())} escapes in {len(counts)} files")
        return 0

    baseline: dict[str, int] = {}
    for line in BASELINE.read_text(encoding="utf-8").splitlines():
        if line and not line.startswith("#"):
            n, path = line.split(maxsplit=1)
            baseline[path] = int(n)

    grown = [(path, n, baseline.get(path, 0)) for path, n in sorted(counts.items()) if n > baseline.get(path, 0)]
    for path, n, allowed in grown:
        print(f"error: {path}: {n} unit escapes (.Value() or a raw unit construction), baseline {allowed}")
    shrunk = sum(1 for path, allowed in baseline.items() if counts.get(path, 0) < allowed)
    if grown:
        print("Keep the calculation in the unit types; at a real boundary raise the baseline "
              "with scripts/check_unit_escapes.py --update.")
        return 1
    if shrunk:
        print(f"note: {shrunk} files below their baseline; lock it in with scripts/check_unit_escapes.py --update")
    print(f"unit escapes: {sum(counts.values())} (baseline {sum(baseline.values())})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
