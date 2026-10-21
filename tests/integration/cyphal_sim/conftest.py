# SPDX-License-Identifier: GPL-3.0-or-later
#
# pycyphal integration tests of the native_sim drive (scenario
# unimoc.app.cyphal_integration in app/testcase.yaml). Twister builds the
# application and starts it (fixture `dut`); a pycyphal node connects to the
# first slcan pseudo terminal of the simulation, so no SocketCAN is needed.
# Skipped when pycyphal, python-can or pyserial are not installed
# (scripts/requirements-cyphal-tools.txt). The DSDL is compiled into
# <tmp>/unimoc_pycyphal on the first run (about a minute).

from __future__ import annotations

import asyncio
import logging
import os
import tempfile
import re
import sys
from pathlib import Path

import pytest

pytest.importorskip("pycyphal")
pytest.importorskip("can")
pytest.importorskip("serial")

import pycyphal.dsdl  # noqa: E402

_REPO = Path(__file__).resolve().parents[3]
_PUBLIC_DSDL = _REPO.parent / "modules" / "lib" / "public_regulated_data_types"
_NAMESPACES = [_PUBLIC_DSDL / "uavcan", _PUBLIC_DSDL / "reg", _REPO / "dsdl" / "unimoc"]
_GENERATED = Path(tempfile.gettempdir()) / "unimoc_pycyphal"


def _compile_dsdl() -> None:
    """Compiles the DSDL namespaces once, again when a vendor type changes.

    Explicitly, not by the pycyphal import hook: the workspace root is on
    sys.path, so `import unimoc` would find the repository instead of the
    generated package. The output directory goes first on sys.path.
    """
    stamp = _GENERATED / ".stamp"
    newest = max(path.stat().st_mtime for path in (_REPO / "dsdl").rglob("*.dsdl"))
    if not stamp.exists() or stamp.stat().st_mtime < newest:
        for namespace in _NAMESPACES:
            pycyphal.dsdl.compile(namespace, [str(path) for path in _NAMESPACES], _GENERATED)
        stamp.touch()
    sys.path.insert(0, str(_GENERATED))
    sys.modules.pop("unimoc", None)


_compile_dsdl()

# Twister runs pytest at DEBUG; pycyphal logs every frame there.
for _name in ("pycyphal", "can", "nunavut", "pydsdl"):
    logging.getLogger(_name).setLevel(logging.WARNING)

sys.path.insert(0, str(Path(__file__).parent))

from sim_client import SimClient  # noqa: E402

PTY_PATTERN = re.compile(r"uart_1 connected to pseudotty: (\S+)")


def wait_for_pty(dut) -> str:
    """Reads the boot output of the simulation until it names the first slcan terminal."""
    lines = dut.readlines_until(regex=PTY_PATTERN.pattern, timeout=30)
    for line in reversed(lines):
        match = PTY_PATTERN.search(line)
        if match:
            return match.group(1)
    raise AssertionError("the simulation did not report its slcan pseudo terminal")


@pytest.fixture(scope="session")
def loop():
    event_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(event_loop)
    yield event_loop
    event_loop.close()


@pytest.fixture(scope="session")
def sim(dut, loop) -> SimClient:
    """A pycyphal client of the running simulation, starting from factory settings."""
    client = SimClient(loop, wait_for_pty(dut))
    client.wait_for_node()
    assert client.command(65532) == 0  # FACTORY_RESET: no settings of an earlier run
    yield client
    client.close()
