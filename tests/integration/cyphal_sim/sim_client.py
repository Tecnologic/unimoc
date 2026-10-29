# SPDX-License-Identifier: GPL-3.0-or-later
#
# Synchronous wrapper around a pycyphal node for the integration tests: one
# CAN interface (slcan, classic CAN), local node-ID 127, the drive at node-ID 10.

from __future__ import annotations

import asyncio
import time
from typing import Any, Callable

import pycyphal
import pycyphal.application
from pycyphal.transport.can import CANTransport
from pycyphal.transport.can.media.pythoncan import PythonCANMedia

# DSDL namespaces, compiled by the pycyphal import hook (CYPHAL_PATH in conftest.py).
import uavcan.diagnostic
import uavcan.node
import uavcan.node.port
import uavcan.primitive.array
import uavcan.register
import uavcan.si.unit.angle
import uavcan.si.unit.angular_acceleration
import uavcan.si.unit.angular_velocity
import uavcan.si.unit.torque
import reg.udral.physics.dynamics.rotation
import reg.udral.physics.kinematics.rotation
import reg.udral.service.common
import cymon
import unimoc.cart
import unimoc.drive
import unimoc.sim

DRIVE = 10
LOCAL = 127

PORT_READINESS = 1024 + DRIVE  # the factory readiness follows the node-ID
READINESS_FOLLOWS_NODE_ID = 65534
PORT_CURRENT_SETPOINT = 107
PORT_TORQUE_SETPOINT = 100
PORT_SPEED_SETPOINT = 101
PORT_POSITION_SETPOINT = 102
PORT_SERVO_SETPOINT = 108
PORT_TELEMETRY = 212
PORT_PLANT = 213
PORT_WHEEL_COMMAND = 110
PORT_PEDAL_STATE = 111
PORT_BATTERY_STATUS = 112
PORT_DRIVE_STATUS = 214
PORT_COMMISSIONING = 215

# Trace services (cymon), defaults of the uavcan.srv.cymon_*.id registers.
PORT_CYMON_VARIABLES = 120
PORT_CYMON_BUFFER_INFO = 121
PORT_CYMON_SETUP = 122
PORT_CYMON_ARM = 123
PORT_CYMON_READ = 124

# Vendor status of the heartbeat (docs/gui_plan.md, section 3); bits 0..2 are the power stage.
VENDOR_ENGAGED = 0x08
VENDOR_TRIP_LATCHED = 0x10
VENDOR_EMERGENCY_STOP = 0x20
VENDOR_SETTINGS_PENDING = 0x40
VENDOR_LOCAL_CONTROL = 0x80

ENGAGED = reg.udral.service.common.Readiness_0_1.ENGAGED
SLEEP = reg.udral.service.common.Readiness_0_1.SLEEP
STANDBY = reg.udral.service.common.Readiness_0_1.STANDBY


def real(value: float) -> uavcan.register.Value_1_0:
    return uavcan.register.Value_1_0(real32=uavcan.primitive.array.Real32_1_0([value]))


def natural8(value: int) -> uavcan.register.Value_1_0:
    return uavcan.register.Value_1_0(natural8=uavcan.primitive.array.Natural8_1_0([value]))


def natural16(value: int) -> uavcan.register.Value_1_0:
    return uavcan.register.Value_1_0(natural16=uavcan.primitive.array.Natural16_1_0([value]))


def scalar(value: uavcan.register.Value_1_0) -> Any:
    """First element of a numeric register value, or the text of a string."""
    if value.string is not None:
        return value.string.value.tobytes().decode()
    for name in ("real32", "natural8", "natural16", "natural32", "integer64", "bit"):
        field = getattr(value, name)
        if field is not None:
            return field.value[0]
    return None


class SimClient:
    def __init__(self, loop: asyncio.AbstractEventLoop, pty: str) -> None:
        self.loop = loop
        self.pty = pty
        self.records: list[str] = []
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
        self.telemetry = self.node.make_subscriber(unimoc.drive.Telemetry_0_1, PORT_TELEMETRY)
        self.plant = self.node.make_subscriber(unimoc.sim.Plant_0_1, PORT_PLANT)
        self.drive_status = self.node.make_subscriber(unimoc.cart.DriveStatus_0_1, PORT_DRIVE_STATUS)
        self.commissioning = self.node.make_subscriber(unimoc.drive.CommissioningStatus_0_1, PORT_COMMISSIONING)
        self.port_list = self.node.make_subscriber(uavcan.node.port.List_1_0)
        self.diagnostic = self.node.make_subscriber(uavcan.diagnostic.Record_1_1)
        self.diagnostic.receive_in_background(self._on_record)
        self.readiness = self.node.make_publisher(reg.udral.service.common.Readiness_0_1, PORT_READINESS)
        self.current = self.node.make_publisher(unimoc.drive.CurrentSetpoint_0_1, PORT_CURRENT_SETPOINT)
        self.torque = self.node.make_publisher(uavcan.si.unit.torque.Scalar_1_0, PORT_TORQUE_SETPOINT)
        self.speed = self.node.make_publisher(uavcan.si.unit.angular_velocity.Scalar_1_0, PORT_SPEED_SETPOINT)
        self.position = self.node.make_publisher(uavcan.si.unit.angle.Scalar_1_0, PORT_POSITION_SETPOINT)
        self.servo = self.node.make_publisher(reg.udral.physics.dynamics.rotation.Planar_0_1, PORT_SERVO_SETPOINT)
        self.wheel_command_publisher = self.node.make_publisher(unimoc.cart.WheelCommand_0_1, PORT_WHEEL_COMMAND)
        self.pedal_state_publisher = self.node.make_publisher(unimoc.cart.PedalState_0_1, PORT_PEDAL_STATE)

    async def _on_record(self, record: uavcan.diagnostic.Record_1_1, _meta) -> None:
        self.records.append(record.text.tobytes().decode())

    def close(self) -> None:
        if self.node is None:
            return
        self.node.close()
        self.node = None
        self._run(asyncio.sleep(0.5))  # let the media threads stop

    # --- Services --------------------------------------------------------------

    def call(self, service, request, timeout: float = 3.0, port: int | None = None):
        async def go():
            # A port-ID for the types without a fixed one (cymon).
            client = self.node.make_client(service, DRIVE) if port is None else self.node.make_client(service, DRIVE, port)
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

    def command(self, code: int, parameter: str = "") -> int:
        return self.command_output(code, parameter)[0]

    def command_output(self, code: int, parameter: str = "") -> tuple[int, str]:
        """The status and the output text of an ExecuteCommand."""
        request = uavcan.node.ExecuteCommand_1_3.Request(command=code, parameter=parameter)
        response = self.call(uavcan.node.ExecuteCommand_1_3, request)
        return response.status, response.output.tobytes().decode()

    # --- Traces (cymon) -----------------------------------------------------------

    def trace_variables(self) -> list[tuple[str, str]]:
        """Name and unit of every trace variable; the index is the variable ID."""
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

    def trace_wait_complete(self, timeout: float = 5.0) -> cymon.ReadSamples_0_1.Response:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = self.trace_read()
            if status.state.value == cymon.CaptureState_0_1.COMPLETE:
                return status
            self.wait(0.05)
        raise AssertionError("the capture did not complete")

    def trace_frames(self) -> list[list[float]]:
        """Reads a complete capture page by page; one list of channel values per frame."""
        status = self.trace_read()
        channels = status.num_channels
        per_page = max(1, 64 // channels)
        samples: list[float] = []
        offset = 0
        while True:
            page = self.trace_read(offset, per_page)
            samples.extend(float(value) for value in page.samples)
            offset += len(page.samples) // channels
            if page.end_of_data or not len(page.samples):
                break
        return [samples[index:index + channels] for index in range(0, len(samples), channels)]

    # --- Subjects ----------------------------------------------------------------

    def receive(self, subscriber, timeout: float = 3.0):
        async def go():
            result = await subscriber.receive_for(timeout)
            assert result is not None, "no message"
            return result

        return self._run(go())

    def heartbeat_now(self) -> uavcan.node.Heartbeat_1_0:
        """The newest heartbeat of the drive received so far (they come once per second)."""
        return self.latest(self.heartbeat, settle=0.0)

    def wait_for_node(self, timeout: float = 30.0) -> uavcan.node.Heartbeat_1_0:
        """Waits for the first heartbeat of the drive (the slcan port opens with a delay)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            async def go():
                return await self.heartbeat.receive_for(2.0)

            result = self._run(go())
            if result is not None and result[1].source_node_id == DRIVE:
                return result[0]
        raise AssertionError("no heartbeat of the drive")

    def latest(self, subscriber, settle: float = 0.3):
        """The newest message after @p settle seconds (drops what was queued before)."""
        async def go():
            await asyncio.sleep(settle)
            last = None
            while (result := await subscriber.receive_for(0.0)) is not None:
                last = result
            if last is None:
                last = await subscriber.receive_for(2.0)
            assert last is not None, "no message"
            return last[0]

        return self._run(go())

    def hold(self, seconds: float, readiness: int, setpoint: Callable[[], None] | None = None,
             period: float = 0.05) -> None:
        """Publishes readiness (and a setpoint) every @p period for @p seconds."""
        async def go():
            deadline = self.loop.time() + seconds
            while self.loop.time() < deadline:
                await self.readiness.publish(reg.udral.service.common.Readiness_0_1(readiness))
                if setpoint is not None:
                    await setpoint()
                await asyncio.sleep(period)

        self._run(go())

    def publish_for(self, seconds: float, setpoint: Callable[[], None], period: float = 0.02) -> None:
        """Publishes @p setpoint every @p period for @p seconds, without the readiness."""
        async def go():
            deadline = self.loop.time() + seconds
            while self.loop.time() < deadline:
                await setpoint()
                await asyncio.sleep(period)

        self._run(go())

    def wheel_command(self, front_left: float):
        """Wheel command with @p front_left [N m] for the drive's wheel, the others zero."""
        async def publish():
            await self.wheel_command_publisher.publish(unimoc.cart.WheelCommand_0_1(wheel_torque=[front_left, 0.0, 0.0, 0.0]))

        return publish

    def pedals(self, drive: float, brake: float):
        """Plausible pedal state, both channels equal."""
        async def publish():
            await self.pedal_state_publisher.publish(unimoc.cart.PedalState_0_1(
                drive_pedal=[drive, drive], brake_pedal=[brake, brake],
                drive_pedal_plausible=True, brake_pedal_plausible=True))

        return publish

    def current_setpoint(self, i_d: float, i_q: float):
        async def publish():
            await self.current.publish(unimoc.drive.CurrentSetpoint_0_1(i_d=i_d, i_q=i_q))

        return publish

    def torque_setpoint(self, newton_meter: float):
        async def publish():
            await self.torque.publish(uavcan.si.unit.torque.Scalar_1_0(newton_meter))

        return publish

    def speed_setpoint(self, radian_per_second: float):
        async def publish():
            await self.speed.publish(uavcan.si.unit.angular_velocity.Scalar_1_0(radian_per_second))

        return publish

    def position_setpoint(self, radian: float):
        async def publish():
            await self.position.publish(uavcan.si.unit.angle.Scalar_1_0(radian))

        return publish

    def servo_setpoint(self, radian: float, radian_per_second: float, newton_meter: float):
        """UDRAL servo setpoint; NaN marks a field as not commanded."""
        rotation = reg.udral.physics.dynamics.rotation
        kinematics = reg.udral.physics.kinematics.rotation.Planar_0_1(
            angular_position=uavcan.si.unit.angle.Scalar_1_0(radian),
            angular_velocity=uavcan.si.unit.angular_velocity.Scalar_1_0(radian_per_second),
            angular_acceleration=uavcan.si.unit.angular_acceleration.Scalar_1_0(float("nan")),
        )
        message = rotation.Planar_0_1(kinematics=kinematics, torque=uavcan.si.unit.torque.Scalar_1_0(newton_meter))

        async def publish():
            await self.servo.publish(message)

        return publish

    def wait(self, seconds: float) -> None:
        self._run(asyncio.sleep(seconds))
