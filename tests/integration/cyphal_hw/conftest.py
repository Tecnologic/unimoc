# SPDX-License-Identifier: GPL-3.0-or-later
#
# pycyphal tests of a real drive board on a CAN FD bus (README.md in this
# directory). Manual only: there is no twister scenario (no testcase.yaml), CI
# never runs them, and without --hardware every test is skipped. The DSDL is
# compiled into <tmp>/unimoc_pycyphal on the first run (about a minute).

from __future__ import annotations

import asyncio
import logging
import sys
from pathlib import Path

import pytest

pytest.importorskip("pycyphal")
pytest.importorskip("can")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from unimoc_dsdl import compile_dsdl  # noqa: E402

compile_dsdl()

for _name in ("pycyphal", "can", "nunavut", "pydsdl"):
    logging.getLogger(_name).setLevel(logging.WARNING)

sys.path.insert(0, str(Path(__file__).parent))

from hw_client import HwClient  # noqa: E402


def pytest_addoption(parser) -> None:
    group = parser.getgroup("unimoc hardware")
    group.addoption("--hardware", action="store_true", help="run against a real board (otherwise everything is skipped)")
    group.addoption("--can-iface", default="can0",
                    help="SocketCAN interface of a CAN FD adapter, 64-byte frames (default can0)")
    group.addoption("--allow-engage", action="store_true",
                    help="also run the tests that engage the drive, i.e. enable the power stage outputs")
    group.addoption("--allow-factory-reset", action="store_true",
                    help="also run FACTORY_RESET: it erases the settings and the stored node-ID")
    group.addoption("--allow-flash-bank", action="store_true",
                    help="the board has a firmware bank: do not expect BEGIN_SOFTWARE_UPDATE to be refused")


def pytest_configure(config) -> None:
    config.addinivalue_line("markers", "engage: enables the power stage outputs (needs --allow-engage)")
    config.addinivalue_line("markers", "factory_reset: erases the settings (needs --allow-factory-reset)")
    config.addinivalue_line("markers", "no_flash_bank: for a board without a firmware bank (skipped with --allow-flash-bank)")


def pytest_collection_modifyitems(config, items) -> None:
    for item in items:
        if not config.getoption("--hardware"):
            item.add_marker(pytest.mark.skip(reason="manual hardware tests: pass --hardware (README.md)"))
        elif "engage" in item.keywords and not config.getoption("--allow-engage"):
            item.add_marker(pytest.mark.skip(reason="enables the power stage outputs: pass --allow-engage"))
        elif "factory_reset" in item.keywords and not config.getoption("--allow-factory-reset"):
            item.add_marker(pytest.mark.skip(reason="erases settings and node-ID: pass --allow-factory-reset"))
        elif "no_flash_bank" in item.keywords and config.getoption("--allow-flash-bank"):
            item.add_marker(pytest.mark.skip(reason="the board has a firmware bank"))


@pytest.fixture(scope="session")
def loop():
    event_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(event_loop)
    yield event_loop
    event_loop.close()


@pytest.fixture(scope="session")
def drive(request, loop) -> HwClient:
    """A pycyphal client of the board; gives an anonymous drive its node-ID by plug-and-play."""
    if not request.config.getoption("--hardware"):
        pytest.skip("manual hardware tests: pass --hardware")
    client = HwClient(loop, request.config.getoption("--can-iface"))
    client.wait_for_drive()
    yield client
    client.close()
