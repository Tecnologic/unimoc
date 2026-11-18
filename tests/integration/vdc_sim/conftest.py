# SPDX-License-Identifier: GPL-3.0-or-later
#
# pycyphal integration tests of the native_sim VDC (scenario
# unimoc.app.vdc.cyphal_integration in apps/vdc/testcase.yaml). Twister builds
# the VDC and starts it (fixture `dut`); a pycyphal node on the first slcan
# pseudo terminal plays the four wheel drives. Skipped when pycyphal,
# python-can or pyserial are not installed (scripts/requirements-cyphal-tools.txt).

from __future__ import annotations

import asyncio
import logging
import re
import sys
from pathlib import Path

import pytest

pytest.importorskip("pycyphal")
pytest.importorskip("can")
pytest.importorskip("serial")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from unimoc_dsdl import compile_dsdl  # noqa: E402

compile_dsdl()

# Twister runs pytest at DEBUG; pycyphal logs every frame there.
for _name in ("pycyphal", "can", "nunavut", "pydsdl"):
    logging.getLogger(_name).setLevel(logging.WARNING)

sys.path.insert(0, str(Path(__file__).parent))

from vdc_client import VdcClient  # noqa: E402

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
def vdc(dut, loop) -> VdcClient:
    """A pycyphal client of the running VDC that plays every wheel drive."""
    client = VdcClient(loop, wait_for_pty(dut))
    client.wait_for_node()
    yield client
    client.close()
