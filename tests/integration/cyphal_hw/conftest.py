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
    group.addoption("--no-bootloader", action="store_true",
                    help="the board runs the application without MCUboot (no sysbuild): expect BEGIN_SOFTWARE_UPDATE to be refused")
    group.addoption("--update-image", type=Path, default=None,
                    help="UNIMOC update image (scripts/make_firmware_image.py) of this board with a HIGHER version than the "
                         "running firmware: runs the update tests; the board then runs this image")
    group.addoption("--unconfirmed-image", type=Path, default=None,
                    help="UNIMOC update image newer than --update-image, built with CONFIG_UNIMOC_UPDATE_CONFIRM_DELAY_MS "
                         "so long that it does not confirm itself: checks the swap back (after --update-image)")
    group.addoption("--current-image", type=Path, default=None,
                    help="UNIMOC update image of the running firmware (same version): checks that it is refused as not newer")


def pytest_configure(config) -> None:
    config.addinivalue_line("markers", "engage: enables the power stage outputs (needs --allow-engage)")
    config.addinivalue_line("markers", "factory_reset: erases the settings (needs --allow-factory-reset)")
    config.addinivalue_line("markers", "no_bootloader: for a build without MCUboot (needs --no-bootloader)")
    config.addinivalue_line("markers", "swap_back: swaps an image in that never confirms (needs --unconfirmed-image)")
    config.addinivalue_line("markers", "update: swaps a new image in (needs --update-image, runs last, changes the firmware)")


def pytest_collection_modifyitems(config, items) -> None:
    for item in items:
        if not config.getoption("--hardware"):
            item.add_marker(pytest.mark.skip(reason="manual hardware tests: pass --hardware (README.md)"))
        elif "engage" in item.keywords and not config.getoption("--allow-engage"):
            item.add_marker(pytest.mark.skip(reason="enables the power stage outputs: pass --allow-engage"))
        elif "factory_reset" in item.keywords and not config.getoption("--allow-factory-reset"):
            item.add_marker(pytest.mark.skip(reason="erases settings and node-ID: pass --allow-factory-reset"))
        elif "no_bootloader" in item.keywords and not config.getoption("--no-bootloader"):
            item.add_marker(pytest.mark.skip(reason="the board runs MCUboot: pass --no-bootloader for a build without it"))
        elif "swap_back" in item.keywords and config.getoption("--unconfirmed-image") is None:
            item.add_marker(pytest.mark.skip(reason="needs --unconfirmed-image (README.md)"))
        elif "update" in item.keywords and config.getoption("--update-image") is None:
            item.add_marker(pytest.mark.skip(reason="swaps the firmware: pass --update-image <image>"))


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
