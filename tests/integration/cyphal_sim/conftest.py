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
_CYMON_DSDL = _REPO / "lib" / "cymon" / "dsdl" / "cymon"
_NAMESPACES = [_PUBLIC_DSDL / "uavcan", _PUBLIC_DSDL / "reg", _REPO / "dsdl" / "unimoc", _CYMON_DSDL]
_GENERATED = Path(tempfile.gettempdir()) / "unimoc_pycyphal"


def _compile_dsdl() -> None:
    """Compiles the DSDL namespaces once, again when a vendor type changes.

    Explicitly, not by the pycyphal import hook: the workspace root is on
    sys.path, so `import unimoc` would find the repository instead of the
    generated package. The output directory goes first on sys.path.
    """
    stamp = _GENERATED / ".stamp"
    sources = [*(_REPO / "dsdl").rglob("*.dsdl"), *_CYMON_DSDL.rglob("*.dsdl")]
    newest = max(path.stat().st_mtime for path in sources)
    if not stamp.exists() or stamp.stat().st_mtime < newest:
        for namespace in _NAMESPACES:
            pycyphal.dsdl.compile(namespace, [str(path) for path in _NAMESPACES], _GENERATED)
        stamp.touch()
    sys.path.insert(0, str(_GENERATED))
    sys.modules.pop("unimoc", None)
    sys.modules.pop("cymon", None)


_compile_dsdl()

# Twister runs pytest at DEBUG; pycyphal logs every frame there.
for _name in ("pycyphal", "can", "nunavut", "pydsdl"):
    logging.getLogger(_name).setLevel(logging.WARNING)

sys.path.insert(0, str(Path(__file__).parent))

from sim_client import STANDBY, SimClient, natural8, real  # noqa: E402

PTY_PATTERN = re.compile(r"uart_1 connected to pseudotty: (\S+)")


def wait_for_pty(dut) -> str:
    """Reads the boot output of the simulation until it names the first slcan terminal."""
    lines = dut.readlines_until(regex=PTY_PATTERN.pattern, timeout=30)
    for line in reversed(lines):
        match = PTY_PATTERN.search(line)
        if match:
            return match.group(1)
    raise AssertionError("the simulation did not report its slcan pseudo terminal")


# The motor the tests were written for, as tests/common/sim_test_motor.hpp: the
# factory profile describes the default motor (a hoverboard hub motor), the
# tests a small one without a permanent magnet (they add one where they need it).
TEST_MOTOR = {
    "unimoc.motor.pole_pairs": natural8(1),
    "unimoc.motor.stator.R": real(0.1),
    "unimoc.motor.stator.L": real(1e-3),
    "unimoc.motor.pmsm.flux_pm": real(0.0),
    "unimoc.motor.pmsm.L_d": real(1e-3),
    "unimoc.motor.pmsm.L_q": real(1e-3),
    "unimoc.motor.mechanics.J": real(1e-4),
    "unimoc.motor.limits.i_max": real(50.0),  # the capability of the sim power stage
    "unimoc.motor.limits.omega_max": real(2000.0),
    "unimoc.motor.limits.omega_min": real(-2000.0),
    "unimoc.control.current.kp_d": real(1.0),
    "unimoc.control.current.ki_d": real(100.0),
    "unimoc.control.current.kp_q": real(1.0),
    "unimoc.control.current.ki_q": real(100.0),
    "unimoc.control.speed.kp": real(0.01),
    "unimoc.control.speed.ki": real(10.0),
}


def configure_test_motor(client: SimClient) -> None:
    """Writes the test motor; the controller and the plant take it while disengaged."""
    for name, value in TEST_MOTOR.items():
        assert client.write(name, value) is not None, name
    client.hold(0.2, STANDBY)


@pytest.fixture(scope="session")
def loop():
    event_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(event_loop)
    yield event_loop
    event_loop.close()


@pytest.fixture(scope="session")
def sim(dut, loop) -> SimClient:
    """A pycyphal client of the running simulation, starting from factory settings with the test motor."""
    client = SimClient(loop, wait_for_pty(dut))
    client.wait_for_node()
    assert client.command(65532) == 0  # FACTORY_RESET: no settings of an earlier run
    configure_test_motor(client)
    yield client
    client.close()
