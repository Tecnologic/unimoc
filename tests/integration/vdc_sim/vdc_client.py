# SPDX-License-Identifier: GPL-3.0-or-later
#
# Synchronous wrapper around a pycyphal node for the VDC integration tests:
# one CAN interface (slcan, classic CAN), local node-ID 127, the VDC at
# node-ID 1. The client plays the wheel drives: the VDC takes it as the drive
# of every wheel (unimoc.vdc.drive_node_ids), so it receives their readiness
# on 1024 + 127 and publishes their drive status. The VDC runs in wall time
# (the simulation follows the wall clock), so the waits here are wall time.

from __future__ import annotations

import asyncio
import time
from typing import Any, Callable

import pycyphal
import pycyphal.application
from pycyphal.transport.can import CANTransport
from pycyphal.transport.can.media.pythoncan import PythonCANMedia

import uavcan.node
import uavcan.primitive.array
import uavcan.register
import reg.udral.service.common
import unimoc.cart

# The faster slcan reader of the drive tests (it patches python-can on import).
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "cyphal_sim"))
from sim_client import real, scalar  # noqa: E402

VDC = 1
LOCAL = 127
PORT_WHEEL_COMMAND = 110
PORT_DRIVE_STATUS = 214
PORT_READINESS = 1024 + LOCAL

ENGAGED = reg.udral.service.common.Readiness_0_1.ENGAGED
STANDBY = reg.udral.service.common.Readiness_0_1.STANDBY
POWER_STAGE_ENABLED = unimoc.cart.DriveStatus_0_1.POWER_STAGE_ENABLED
POWER_STAGE_DISABLED = unimoc.cart.DriveStatus_0_1.POWER_STAGE_DISABLED


def bit(value: bool) -> uavcan.register.Value_1_0:
    return uavcan.register.Value_1_0(bit=uavcan.primitive.array.Bit_1_0([value]))


def natural8_array(values: list[int]) -> uavcan.register.Value_1_0:
    return uavcan.register.Value_1_0(natural8=uavcan.primitive.array.Natural8_1_0(values))


def values(value: uavcan.register.Value_1_0) -> list[Any]:
    """All elements of a register value (scalar() of sim_client takes the first)."""
    for field in ("natural8", "natural16", "real32", "bit"):
        member = getattr(value, field, None)
        if member is not None:
            return [element for element in member.value]
    raise AssertionError(f"unexpected register value {value}")


class VdcClient:
    def __init__(self, loop: asyncio.AbstractEventLoop, pty: str) -> None:
        self.loop = loop
        self.pty = pty
        self.wheels_reporting = [False] * 4  # the drive status the client publishes, per wheel
        self.engaged = False  # the drives report ENGAGED with the power stage on
        self._status_task: asyncio.Task | None = None
        self._run(self._start())

    def _run(self, coroutine):
        return self.loop.run_until_complete(coroutine)

    async def _start(self) -> None:
        media = PythonCANMedia(f"slcan:{self.pty}", 1_000_000, 8)
        transport = CANTransport(media, local_node_id=LOCAL)
        registry = pycyphal.application.make_registry(environment_variables={})
        self.node = pycyphal.application.make_node(
            uavcan.node.GetInfo_1_0.Response(name="org.unimoc.integration_test"),
            registry=registry,
            transport=transport,
        )
        self.node.start()
        self.heartbeat = self.node.make_subscriber(uavcan.node.Heartbeat_1_0)
        self.wheel_command = self.node.make_subscriber(unimoc.cart.WheelCommand_0_1, PORT_WHEEL_COMMAND)
        self.readiness = self.node.make_subscriber(reg.udral.service.common.Readiness_0_1, PORT_READINESS)
        self.drive_status = self.node.make_publisher(unimoc.cart.DriveStatus_0_1, PORT_DRIVE_STATUS)
        self._status_task = asyncio.ensure_future(self._publish_status())

    async def _publish_status(self) -> None:
        """The drive status of the wheels that report, at 50 Hz as the drives."""
        while True:
            for wheel, reporting in enumerate(self.wheels_reporting):
                if reporting:
                    await self.drive_status.publish(unimoc.cart.DriveStatus_0_1(
                        wheel_speed=1.0,
                        estimated_torque=0.5,
                        drive_torque_limit=50.0,
                        brake_torque_limit=50.0,
                        dc_link_voltage=36.0,
                        wheel=wheel,
                        power_stage=POWER_STAGE_ENABLED if self.engaged else POWER_STAGE_DISABLED,
                        engaged=self.engaged,
                    ))
            await asyncio.sleep(0.02)

    def close(self) -> None:
        if self.node is None:
            return
        if self._status_task is not None:
            self._status_task.cancel()
        self.node.close()
        self.node = None
        self._run(asyncio.sleep(0.5))  # let the media threads stop

    def wait(self, seconds: float) -> None:
        self._run(asyncio.sleep(seconds))

    def wait_for_node(self, timeout: float = 30.0) -> uavcan.node.Heartbeat_1_0:
        """Waits for the first heartbeat of the VDC (the slcan port opens with a delay)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            async def go():
                return await self.heartbeat.receive_for(2.0)

            result = self._run(go())
            if result is not None and result[1].source_node_id == VDC:
                return result[0]
        raise AssertionError("no heartbeat of the VDC")

    def next_heartbeat(self, timeout: float = 5.0) -> uavcan.node.Heartbeat_1_0:
        return self.expect(self.heartbeat, lambda _message: True, timeout,
                           source=VDC)

    # --- Services --------------------------------------------------------------

    def call(self, service, request, timeout: float = 10.0):
        async def go():
            client = self.node.make_client(service, VDC)
            client.response_timeout = timeout
            try:
                result = await client(request)
            finally:
                client.close()
            assert result is not None, f"no response to {service.__name__}"
            return result

        return self._run(go())

    def get_info(self) -> uavcan.node.GetInfo_1_0.Response:
        return self.call(uavcan.node.GetInfo_1_0, uavcan.node.GetInfo_1_0.Request())

    def access(self, name: str, value: uavcan.register.Value_1_0 | None = None) -> uavcan.register.Access_1_0.Response:
        request = uavcan.register.Access_1_0.Request(
            name=uavcan.register.Name_1_0(name), value=value or uavcan.register.Value_1_0()
        )
        return self.call(uavcan.register.Access_1_0, request)

    def read(self, name: str) -> Any:
        return scalar(self.access(name).value)

    def read_all(self, name: str) -> list[Any]:
        return values(self.access(name).value)

    def write(self, name: str, value: uavcan.register.Value_1_0) -> Any:
        return scalar(self.access(name, value).value)

    def register_names(self) -> list[str]:
        names = []
        for index in range(256):
            name = self.call(uavcan.register.List_1_0, uavcan.register.List_1_0.Request(index)).name
            text = name.name.tobytes().decode()
            if not text:
                break
            names.append(text)
        return names

    # --- Messages ----------------------------------------------------------------

    def expect(self, subscriber, predicate: Callable[[Any], bool], timeout: float = 5.0, source: int | None = None):
        """The first message received from now on that satisfies @p predicate (wall time)."""
        async def go():
            while (await subscriber.receive_for(0.0)) is not None:
                pass
            last = None
            deadline = self.loop.time() + timeout
            while self.loop.time() < deadline:
                result = await subscriber.receive_for(0.05)
                if result is None or (source is not None and result[1].source_node_id != source):
                    continue
                last = result[0]
                if predicate(last):
                    return last
            raise AssertionError(f"no message as expected within {timeout} s; the last: {last}")

        return self._run(go())


__all__ = ["VdcClient", "ENGAGED", "STANDBY", "bit", "natural8_array", "real", "LOCAL", "PORT_READINESS"]
