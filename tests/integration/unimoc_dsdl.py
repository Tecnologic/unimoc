# SPDX-License-Identifier: GPL-3.0-or-later
#
# The DSDL of the pycyphal integration tests (tests/integration/*_sim):
# compiles the public regulated data types, the UNIMOC and the cymon
# namespaces into <tmp>/unimoc_pycyphal on the first run (about a minute) and
# puts them first on sys.path. Shared by the drive and the VDC scenarios.

from __future__ import annotations

import fcntl
import sys
import tempfile
from pathlib import Path

import pycyphal.dsdl

REPO = Path(__file__).resolve().parents[2]
_PUBLIC_DSDL = REPO.parent / "modules" / "lib" / "public_regulated_data_types"
_CYMON_DSDL = REPO / "lib" / "cymon" / "dsdl" / "cymon"
_NAMESPACES = [_PUBLIC_DSDL / "uavcan", _PUBLIC_DSDL / "reg", REPO / "dsdl" / "unimoc", _CYMON_DSDL]
_GENERATED = Path(tempfile.gettempdir()) / "unimoc_pycyphal"


def compile_dsdl() -> None:
    """Compiles the DSDL namespaces once, again when a vendor type changes.

    Explicitly, not by the pycyphal import hook: the workspace root is on
    sys.path, so `import unimoc` would find the repository instead of the
    generated package. The output directory goes first on sys.path.
    """
    stamp = _GENERATED / ".stamp"
    sources = [*(REPO / "dsdl").rglob("*.dsdl"), *_CYMON_DSDL.rglob("*.dsdl")]
    newest = max(path.stat().st_mtime for path in sources)
    _GENERATED.mkdir(parents=True, exist_ok=True)
    # Twister runs the pytest scenarios in parallel: one compiles, the others wait for it.
    with open(_GENERATED.parent / "unimoc_pycyphal.lock", "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if not stamp.exists() or stamp.stat().st_mtime < newest:
            for namespace in _NAMESPACES:
                pycyphal.dsdl.compile(namespace, [str(path) for path in _NAMESPACES], _GENERATED)
            stamp.touch()
    sys.path.insert(0, str(_GENERATED))
    sys.modules.pop("unimoc", None)
    sys.modules.pop("cymon", None)
