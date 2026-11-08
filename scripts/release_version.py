#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Release version and change log of UNIMOC (docs/release_plan.md).

The Zephyr version file ``apps/drive/VERSION`` is the single source of the version;
every other application (``apps/*/VERSION``, Zephyr reads one per application)
carries a copy that ``apply`` writes and ``current`` checks.
git-cliff (``cliff.toml``) derives the next version from the Conventional
Commits since the last ``v*`` tag.

  release_version.py current            print the version of apps/drive/VERSION
  release_version.py next               print the next version (no change)
  release_version.py apply              write apps/*/VERSION and CHANGELOG.md
  release_version.py check              fail unless apps/drive/VERSION is the next version
  release_version.py pending            print true if a release is waiting, else false
  release_version.py notes              print the release notes of the next version

``--version X.Y.Z[-extra]`` forces the version (``release-as`` PR label).
Without a ``v*`` tag the first release is the version already in apps/drive/VERSION.
"""

import argparse
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
VERSION_FILE = ROOT / "apps" / "drive" / "VERSION"
# The version files of all applications; they hold the same version.
VERSION_FILES = sorted((ROOT / "apps").glob("*/VERSION"))
CHANGELOG = ROOT / "CHANGELOG.md"
SEMVER = re.compile(r"^(\d+)\.(\d+)\.(\d+)(?:-([0-9A-Za-z.]+))?$")


def read_version(path: Path = VERSION_FILE) -> str:
    """Version of apps/drive/VERSION (or @p path) as ``X.Y.Z`` or ``X.Y.Z-extra``."""
    fields = {}
    for line in path.read_text().splitlines():
        key, _, value = line.partition("=")
        fields[key.strip()] = value.strip()
    version = f"{fields['VERSION_MAJOR']}.{fields['VERSION_MINOR']}.{fields['PATCHLEVEL']}"
    if fields.get("EXTRAVERSION"):
        version += "-" + fields["EXTRAVERSION"]
    return version


def check_versions_agree() -> None:
    """Fails unless every apps/*/VERSION holds the version of apps/drive/VERSION."""
    version = read_version()
    for path in VERSION_FILES:
        if read_version(path) != version:
            sys.exit(f"{path.relative_to(ROOT)} is {read_version(path)}, apps/drive/VERSION is {version}")


def write_version(version: str) -> None:
    """Rewrite every apps/*/VERSION; VERSION_TWEAK stays as it is."""
    match = SEMVER.match(version)
    if not match:
        sys.exit(f"not a version: {version}")
    for path in VERSION_FILES:
        old = {}
        for line in path.read_text().splitlines():
            key, _, value = line.partition("=")
            old[key.strip()] = value.strip()
        path.write_text(
            f"VERSION_MAJOR = {match[1]}\n"
            f"VERSION_MINOR = {match[2]}\n"
            f"PATCHLEVEL = {match[3]}\n"
            f"VERSION_TWEAK = {old.get('VERSION_TWEAK', '0')}\n"
            f"EXTRAVERSION ={' ' + match[4] if match[4] else ''}\n"
        )


def cliff(*args: str) -> str:
    result = subprocess.run(
        ["git-cliff", "--config", str(ROOT / "cliff.toml"), *args],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout


def has_release_tag() -> bool:
    tags = subprocess.run(
        ["git", "tag", "--list", "v[0-9]*"], cwd=ROOT, check=True, capture_output=True, text=True
    ).stdout
    return bool(tags.strip())


def next_version(forced: str | None) -> str:
    if forced:
        return forced
    if not has_release_tag():
        return read_version()
    bumped = cliff("--bumped-version").strip().removeprefix("v")
    if not SEMVER.match(bumped):
        sys.exit(f"git-cliff returned no version: {bumped!r}")
    return bumped


def releasable() -> bool:
    """Whether a fix, feature or similar change is waiting for a release."""
    return bool(cliff("--unreleased", "--strip", "all").strip())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("command", choices=["current", "next", "apply", "check", "pending", "notes"])
    parser.add_argument("--version", help="force the version instead of deriving it")
    args = parser.parse_args()
    if args.version and not SEMVER.match(args.version):
        sys.exit(f"not a version: {args.version}")

    check_versions_agree()
    if args.command == "current":
        print(read_version())
        return
    if args.command == "pending":
        print("true" if args.version or not has_release_tag() or releasable() else "false")
        return

    version = next_version(args.version)
    if args.command == "next":
        print(version)
    elif args.command == "check":
        if read_version() != version:
            sys.exit(f"apps/drive/VERSION is {read_version()}, the commits ask for {version}")
    elif args.command == "notes":
        print(cliff("--unreleased", "--tag", f"v{version}", "--strip", "header").strip())
    else:
        if has_release_tag() and not args.version and not releasable():
            print("nothing to release")
            return
        write_version(version)
        # git-cliff trims the blank line between the header and the first release.
        CHANGELOG.write_text(cliff("--tag", f"v{version}").replace("by hand.\n##", "by hand.\n\n##", 1))
        print(version)


if __name__ == "__main__":
    main()
