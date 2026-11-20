#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Merges the compile databases of a twister run for one clang-tidy pass.

Every twister build compiles the shared sources again (lib/cyphal, src/,
hardware/zephyr_common, ...): the same file, with the same flags up to the
build directory, in up to 37 builds. clang-tidy as Zephyr SCA analysed each
of them in each build. This script keeps one compile command per source file
and platform (the platform directory of twister-out), so `twister_sca.sh`
analyses each repository file once per platform. Kconfig variants of one
platform hardly differ for the analysis: the code selects with IS_ENABLED(),
which compiles both branches.

Only files inside the repository and outside build output are kept; the
CodeChecker skip file (scripts/codechecker_skip.txt) still decides which of
them are analysed.

Usage: sca_compile_db.py <repository> <twister output directory> <output file>
"""

import json
import sys
from pathlib import Path


def main() -> int:
    if len(sys.argv) != 4:
        print(__doc__, file=sys.stderr)
        return 1
    repo = Path(sys.argv[1]).resolve()
    outdir = Path(sys.argv[2]).resolve()
    output = Path(sys.argv[3])

    databases = sorted(outdir.rglob("compile_commands.json"))
    if not databases:
        print(f"error: no compile_commands.json below {outdir}", file=sys.stderr)
        return 1

    excluded = [outdir, repo / "build", repo / "twister-out"]
    excluded += list((repo / "apps").glob("*/build"))
    kept: dict[tuple[str, str], dict] = {}
    total = 0
    for database in databases:
        platform = database.relative_to(outdir).parts[0]
        for entry in json.loads(database.read_text()):
            source = Path(entry["directory"], entry["file"]).resolve()
            if not source.is_relative_to(repo) or any(source.is_relative_to(e) for e in excluded):
                continue
            total += 1
            kept.setdefault((platform, str(source)), entry)

    output.write_text(json.dumps(list(kept.values()), indent=1))
    print(f"{len(databases)} builds, {total} compiled repository files, "
          f"{len(kept)} to analyse once per platform")
    return 0


if __name__ == "__main__":
    sys.exit(main())
