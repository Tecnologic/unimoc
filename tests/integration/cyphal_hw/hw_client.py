# SPDX-License-Identifier: GPL-3.0-or-later
#
# Synchronous wrapper around a pycyphal node for the hardware tests: one
# SocketCAN interface with CAN FD frames (64 bytes), local node-ID 127, a
# centralized plug-and-play allocator, and the drive at whatever node-ID it
# was given (found by its heartbeat). Wall-clock time throughout: this is a
# real board, not the simulation of tests/integration/cyphal_sim.

from __future__ import annotations

import asyncio
import shutil
import tempfile
import time
from pathlib import Path
from typing import Any

import pycyphal
import pycyphal.application
import pycyphal.application.file
import pycyphal.application.plug_and_play
from pycyphal.transport.can import CANTransport
from pycyphal.transport.can.media.socketcan import SocketCANMedia

# DSDL namespaces, compiled by tests/integration/unimoc_dsdl.py.
import uavcan.diagnostic
import uavcan.node
import uavcan.node.port
import uavcan.primitive.array
import uavcan.register
import uavcan.si.unit.torque
import reg.udral.service.common
import cymon
import unimoc.cart
import unimoc.drive

LOCAL = 127
FD_MTU = 64

PORT_TORQUE_SETPOINT = 100
PORT_TELEMETRY = 212
PORT_DRIVE_STATUS = 214
PORT_COMMISSIONING = 215
PORT_DIAGNOSTIC = 8184
PORT_CYMON_VARIABLES = 120
PORT_CYMON_BUFFER_INFO = 121
PORT_CYMON_SETUP = 122
PORT_CYMON_ARM = 123
PORT_CYMON_READ = 124

# Vendor status of the heartbeat (docs/gui_plan.md, section 3); bits 0..2 are the power stage.
POWER_STAGE_MASK = 0x07
POWER_STAGE_DISABLED = 0
POWER_STAGE_ENABLED = 1
POWER_STAGE_FAULT = 2
POWER_STAGE_BLOCKED = 3
VENDOR_ENGAGED = 0x08
VENDOR_TRIP_LATCHED = 0x10
VENDOR_EMERGENCY_STOP = 0x20

CMD_BEGIN_SOFTWARE_UPDATE = 65533
CMD_EMERGENCY_STOP = 65531
CMD_FACTORY_RESET = 65532
CMD_STORE_PERSISTENT_STATES = 65530
CMD_RESTART = 65535
CMD_RESET_TIMING = 768

STATUS_SUCCESS = 0
STATUS_FAILURE = 1
STATUS_BAD_COMMAND = 3
STATUS_BAD_STATE = 5

SLEEP = reg.udral.service.common.Readiness_0_1.SLEEP
STANDBY = reg.udral.service.common.Readiness_0_1.STANDBY
ENGAGED = reg.udral.service.common.Readiness_0_1.ENGAGED


def real(value: float) -> uavcan.register.Value_1_0:
    return uavcan.register.Value_1_0(real32=uavcan.primitive.array.Real32_1_0([value]))


def natural8(value: int) -> uavcan.register.Value_1_0:
    return uavcan.register.Value_1_0(natural8=uavcan.primitive.array.Natural8_1_0([value]))


def scalar(value: uavcan.register.Value_1_0) -> Any:
    """First element of a numeric register value, or the text of a string."""
    if value.string is not None:
        return value.string.value.tobytes().decode()
    for name in ("real32", "natural8", "natural16", "natural32", "integer64", "bit"):
        field = getattr(value, name)
        if field is not None:
            return field.value[0]
    return None


class HwClient:
    def __init__(self, loop: asyncio.AbstractEventLoop, iface: str) -> None:
        self.loop = loop
        self.iface = iface
        self.drive: int | None = None
        self.records: list[str] = []
        self._stream: asyncio.Task | None = None
        self._db = Path(tempfile.mkdtemp(prefix="unimoc_hw_")) / "allocation.db"
        self._run(self._start())

    def _run(self, coroutine):
        return self.loop.run_until_complete(coroutine)

    async def _start(self) -> None:
        iface = self.iface.removeprefix("socketcan:")
        transport = CANTransport(SocketCANMedia(iface, FD_MTU), local_node_id=LOCAL)
        registry = pycyphal.application.make_registry(environment_variables={})
        self.node = pycyphal.application.make_node(
            uavcan.node.GetInfo_1_0.Response(name="org.unimoc.hardware_test"), registry=registry, transport=transport)
        self.node.start()
        # Gives an anonymous drive its node-ID (uavcan.pnp.NodeIDAllocationData).
        self.allocator = pycyphal.application.plug_and_play.CentralizedAllocator(self.node, self._db)
        # The file server the drive downloads a firmware update from (BEGIN_SOFTWARE_UPDATE).
        self.file_root = Path(tempfile.mkdtemp(prefix="unimoc_hw_files_"))
        self.file_server = pycyphal.application.file.FileServer(self.node, [self.file_root])
        self.heartbeat = self.node.make_subscriber(uavcan.node.Heartbeat_1_0)
        self.port_list = self.node.make_subscriber(uavcan.node.port.List_1_0)
        self.diagnostic = self.node.make_subscriber(uavcan.diagnostic.Record_1_1, PORT_DIAGNOSTIC)
        self.diagnostic.receive_in_background(self._on_record)

    async def _on_record(self, record: uavcan.diagnostic.Record_1_1, _meta) -> None:
        self.records.append(record.text.tobytes().decode())

    def close(self) -> None:
        if self.node is None:
            return
        self._stop_stream()
        self.node.close()
        self.node = None
        self._run(asyncio.sleep(0.5))  # let the media threads stop

    # --- Finding the drive ---------------------------------------------------------

    def wait_for_drive(self, timeout: float = 60.0) -> int:
        """Waits for a heartbeat of a node other than this one; the allocator answers an anonymous drive meanwhile."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            result = self._run(self.heartbeat.receive_for(2.0))
            if result is not None and result[1].source_node_id != LOCAL:
                self.drive = result[1].source_node_id
                return self.drive
        raise AssertionError(f"no heartbeat of a drive on {self.iface}: adapter up (see README), drive powered, FD frames?")

    def wait_for_restart(self, timeout: float = 60.0, max_uptime: int = 30) -> int:
        """After RESTART or FACTORY_RESET: waits for the new start (the node-ID may be new).

        The old image can still send heartbeats for a moment, with an uptime that may be small as well,
        so a heartbeat only counts after the node was seen gone (a gap) or its uptime went backwards.
        """
        self.sleep(1.0)
        deadline = time.monotonic() + timeout
        previous: int | None = None
        restarted = False
        while time.monotonic() < deadline:
            result = self._run(self.heartbeat.receive_for(2.0))
            if result is None:
                restarted = True  # silence: the board is down (or the bootloader swaps)
                continue
            if result[1].source_node_id == LOCAL:
                continue
            uptime = result[0].uptime
            if (previous is not None and uptime < previous) or uptime <= 2:
                restarted = True  # the uptime went backwards, or this is the first heartbeat of a start
            previous = uptime
            if restarted and uptime < max_uptime:
                self.drive = result[1].source_node_id
                return self.drive
        raise AssertionError("the drive did not come back after the restart")

    def serve_file(self, content: bytes | Path, name: str = "unimoc_update.bin") -> str:
        """Makes @p content the file @p name of this node's file server; returns the path to give BEGIN_SOFTWARE_UPDATE."""
        target = self.file_root / name
        if isinstance(content, Path):
            shutil.copyfile(content, target)
        else:
            target.write_bytes(content)
        return name

    # --- Services --------------------------------------------------------------------

    def call(self, service, request, timeout: float = 5.0, port: int | None = None):
        async def go():
            client = (self.node.make_client(service, self.drive) if port is None
                      else self.node.make_client(service, self.drive, port))
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

    def get_info_after_restart(self, timeout: float = 30.0) -> uavcan.node.GetInfo_1_0.Response:
        """GetInfo, repeated for a while: after a long silence (the bootloader swaps the images) the adapter, whose
        frames nobody acknowledged meanwhile, can need a moment on the bus again (README.md, bench setup)."""
        deadline = time.monotonic() + timeout
        while True:
            try:
                return self.get_info()
            except AssertionError:
                if time.monotonic() > deadline:
                    raise
                self.sleep(1.0)

    def access(self, name: str, value: uavcan.register.Value_1_0 | None = None):
        request = uavcan.register.Access_1_0.Request(
            name=uavcan.register.Name_1_0(name), value=value or uavcan.register.Value_1_0())
        return self.call(uavcan.register.Access_1_0, request)

    def read(self, name: str) -> Any:
        return scalar(self.access(name).value)

    def write(self, name: str, value: uavcan.register.Value_1_0) -> Any:
        return scalar(self.access(name, value).value)

    def register_names(self) -> list[str]:
        names = []
        for index in range(512):
            name = self.call(uavcan.register.List_1_0, uavcan.register.List_1_0.Request(index)).name
            text = name.name.tobytes().decode()
            if not text:
                break
            names.append(text)
        return names

    def command_output(self, code: int, parameter: str = "") -> tuple[int, str]:
        """The status and the output text of an ExecuteCommand."""
        request = uavcan.node.ExecuteCommand_1_3.Request(command=code, parameter=parameter)
        response = self.call(uavcan.node.ExecuteCommand_1_3, request)
        return response.status, response.output.tobytes().decode()

    def command(self, code: int, parameter: str = "") -> int:
        return self.command_output(code, parameter)[0]

    # --- Subjects --------------------------------------------------------------------

    def latest_heartbeat(self, timeout: float = 5.0) -> uavcan.node.Heartbeat_1_0:
        """The next heartbeat of the drive; the queue is emptied first, so it is never an old one."""
        async def go():
            while (await self.heartbeat.receive_for(0.0)) is not None:
                pass
            deadline = self.loop.time() + timeout
            while self.loop.time() < deadline:
                result = await self.heartbeat.receive_for(1.0)
                if result is not None and result[1].source_node_id == self.drive:
                    return result[0]
            raise AssertionError("no heartbeat of the drive")

        return self._run(go())

    def wait_for_status(self, predicate, timeout: float = 6.0) -> uavcan.node.Heartbeat_1_0:
        """The first heartbeat that satisfies @p predicate (of the heartbeat); the status follows the drive at 1 Hz."""
        last = None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            last = self.latest_heartbeat()
            if predicate(last):
                return last
        raise AssertionError(f"the heartbeat did not reach the expected state; last: {last}")

    def vendor_status(self) -> int:
        return self.latest_heartbeat().vendor_specific_status_code

    def subscribe(self, message_type, port: int):
        return self.node.make_subscriber(message_type, port)

    def next_message(self, subscriber, timeout: float = 5.0):
        """The next message after the call (the queue is emptied first)."""
        async def go():
            while (await subscriber.receive_for(0.0)) is not None:
                pass
            result = await subscriber.receive_for(timeout)
            assert result is not None, f"no message within {timeout} s"
            return result[0]

        return self._run(go())

    def count_messages(self, message_type, port: int, seconds: float) -> int:
        """Number of messages of a subject received within @p seconds."""
        count = 0

        async def on_message(_message, _meta) -> None:
            nonlocal count
            count += 1

        async def go():
            subscriber = self.node.make_subscriber(message_type, port)
            subscriber.receive_in_background(on_message)
            await asyncio.sleep(seconds)
            subscriber.close()

        self._run(go())
        return count

    def record_mark(self) -> int:
        return len(self.records)

    def wait_for_record(self, text: str, mark: int, timeout: float = 5.0) -> str:
        """Waits for a diagnostic record containing @p text after @p mark."""
        async def go():
            deadline = self.loop.time() + timeout
            while self.loop.time() < deadline:
                for record in self.records[mark:]:
                    if text in record:
                        return record
                await asyncio.sleep(0.05)
            raise AssertionError(f"no record with '{text}' within {timeout} s; received: {self.records[mark:]}")

        return self._run(go())

    def hold_readiness(self, readiness: int, torque: float | None = None, period: float = 0.1) -> None:
        """Publishes the readiness (and a torque setpoint [N m]) in the background until stop_readiness() or close()."""
        self._stop_stream()

        async def start():
            publisher = self.node.make_publisher(reg.udral.service.common.Readiness_0_1, 1024 + self.drive)
            torque_publisher = (None if torque is None
                                else self.node.make_publisher(uavcan.si.unit.torque.Scalar_1_0, PORT_TORQUE_SETPOINT))

            async def repeat():
                while True:
                    await publisher.publish(reg.udral.service.common.Readiness_0_1(readiness))
                    if torque_publisher is not None:
                        await torque_publisher.publish(uavcan.si.unit.torque.Scalar_1_0(torque))
                    await asyncio.sleep(period)

            self._stream = asyncio.ensure_future(repeat())

        self._run(start())

    def stop_readiness(self) -> None:
        self._stop_stream()

    def _stop_stream(self) -> None:
        if self._stream is not None:
            self._stream.cancel()
            self._stream = None

    def sleep(self, seconds: float) -> None:
        self._run(asyncio.sleep(seconds))

    # --- Traces (cymon) ----------------------------------------------------------------

    def trace_variables(self) -> list[tuple[str, str]]:
        variables = []
        for index in range(256):
            response = self.call(cymon.GetVariableList_0_1, cymon.GetVariableList_0_1.Request(index),
                                 port=PORT_CYMON_VARIABLES)
            name = response.name.tobytes().decode()
            if not name:
                break
            variables.append((name, response.unit.tobytes().decode()))
        return variables

    def trace_buffer_info(self) -> cymon.GetBufferInfo_0_1.Response:
        return self.call(cymon.GetBufferInfo_0_1, cymon.GetBufferInfo_0_1.Request(), port=PORT_CYMON_BUFFER_INFO)

    def trace_setup(self, channels: list[int], sample_period_us: float, num_samples: int,
                    pretrigger_samples: int) -> cymon.SetupCapture_0_1.Response:
        request = cymon.SetupCapture_0_1.Request(channel_ids=channels, sample_period_us=sample_period_us,
                                                 num_samples=num_samples, pretrigger_samples=pretrigger_samples)
        return self.call(cymon.SetupCapture_0_1, request, port=PORT_CYMON_SETUP)

    def trace_arm(self, mode: int, source: int = 0, level: float = 0.0) -> int:
        request = cymon.ArmTrigger_0_1.Request(arm=True, source_variable_id=source,
                                               mode=cymon.TriggerMode_0_1(mode), level=level)
        return self.call(cymon.ArmTrigger_0_1, request, port=PORT_CYMON_ARM).result.value

    def trace_read(self, frame_offset: int = 0, max_frames: int = 0) -> cymon.ReadSamples_0_1.Response:
        request = cymon.ReadSamples_0_1.Request(frame_offset=frame_offset, max_frames=max_frames)
        return self.call(cymon.ReadSamples_0_1, request, port=PORT_CYMON_READ)
