# SPDX-License-Identifier: GPL-3.0-or-later
#
# Synchronous wrapper around a pycyphal node for the integration tests: one
# CAN interface (slcan, classic CAN), local node-ID 127, the drive at node-ID 10.
#
# Durations (wait, hold, publish_for, the settle time of latest) are virtual
# time of the drive, counted by the control cycles of its telemetry: the
# simulation runs only about 1.4 times faster than real time, so on a loaded
# machine (twister builds the other scenarios in parallel) it falls behind the
# wall clock, and a timeout of the drive would not have elapsed yet.
#
# The readiness and setpoint of hold() stay on the bus after it returns, also
# while the client waits for a service response or a message: a slow response
# on a loaded machine would otherwise outlast the 0.5 s command timeout and
# disengage the drive between two holds. wait() and publish_for() end them,
# where a test wants the drive to see the readiness stop.

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
import uavcan.primitive.scalar
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
PORT_HOMING_STATE = 204

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

CONTROL_CYCLE = 62.5e-6  # [s], unimoc.sim.cycle_period_ns at the default 32 kHz PWM

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
        self.virtual_time = 0.0  # [s] control cycles seen in the telemetry; continues over a controller restart
        self._last_cycles: int | None = None
        self._stream: asyncio.Task | None = None  # readiness and setpoint of the last hold()
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
        self.homing = self.node.make_subscriber(uavcan.primitive.scalar.Natural8_1_0, PORT_HOMING_STATE)
        self.port_list = self.node.make_subscriber(uavcan.node.port.List_1_0)
        self.diagnostic = self.node.make_subscriber(uavcan.diagnostic.Record_1_1)
        self.diagnostic.receive_in_background(self._on_record)
        # A second subscriber of the telemetry for the clock; latest(self.telemetry) keeps its own queue.
        self._clock = self.node.make_subscriber(unimoc.drive.Telemetry_0_1, PORT_TELEMETRY)
        self._clock.receive_in_background(self._on_clock)
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

    async def _on_clock(self, telemetry: unimoc.drive.Telemetry_0_1, _meta) -> None:
        cycles = int(telemetry.control_cycles)
        if self._last_cycles is not None:
            # A restart of the controller (settings, reset of the simulation) counts from 0 again.
            self.virtual_time += (cycles - self._last_cycles if cycles >= self._last_cycles else cycles) * CONTROL_CYCLE
        self._last_cycles = cycles

    async def _sleep(self, seconds: float, wall: bool = False) -> None:
        """Sleeps @p seconds of virtual time, or of wall time with @p wall.

        Virtual time advances in steps of the telemetry period (50 ms), so the
        sleep ends up to one step later. When virtual time stands still (a
        paused simulation, a restart, no telemetry) for ten times @p seconds
        of wall time, the sleep ends; a slow simulation that still advances
        (a loaded machine) gets all the wall time it needs.
        """
        if wall:
            await asyncio.sleep(seconds)
            return
        start = self.virtual_time
        stall_limit = 10.0 * seconds + 1.0
        last_progress, last_time = self.loop.time(), self.virtual_time
        while self.virtual_time - start < seconds:
            if self.virtual_time != last_time:
                last_progress, last_time = self.loop.time(), self.virtual_time
            elif self.loop.time() - last_progress > stall_limit:
                break
            await asyncio.sleep(0.01)

    def close(self) -> None:
        if self.node is None:
            return
        self._stop_stream()
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
        """Polls the capture until it is complete, for at most @p timeout seconds of virtual time."""
        for _ in range(round(timeout / 0.05)):
            status = self.trace_read()
            if status.state.value == cymon.CaptureState_0_1.COMPLETE:
                return status
            self.wait(0.05)
        raise AssertionError(f"the capture did not complete (state {status.state.value})")

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
            if settle > 0.0:
                await self._sleep(settle)
            last = None
            while (result := await subscriber.receive_for(0.0)) is not None:
                last = result
            if last is None:
                last = await subscriber.receive_for(2.0)
            assert last is not None, "no message"
            return last[0]

        return self._run(go())

    def hold(self, seconds: float, readiness: int, setpoint: Callable[[], None] | None = None,
             period: float = 0.05, wall: bool = False) -> None:
        """Publishes readiness (and a setpoint) every @p period of wall time for @p seconds.

        The publications go on in the background after the return (see the
        header) until the next hold(), wait(), publish_for() or close().
        """
        async def publish():
            await self.readiness.publish(reg.udral.service.common.Readiness_0_1(readiness))
            if setpoint is not None:
                await setpoint()

        self._stop_stream()

        async def start():
            self._stream = asyncio.ensure_future(self._repeat_forever(publish, period))
            await self._sleep(seconds, wall)

        self._run(start())

    def publish_for(self, seconds: float, setpoint: Callable[[], None], period: float = 0.02) -> None:
        """Publishes @p setpoint every @p period of wall time for @p seconds, without the readiness."""
        self._stop_stream()
        self._run(self._repeat(setpoint, seconds, period, False))

    async def _repeat(self, publish: Callable[[], Any], seconds: float, period: float, wall: bool) -> None:
        """Calls @p publish every @p period of wall time until @p seconds have passed (see _sleep)."""
        done = asyncio.ensure_future(self._sleep(seconds, wall))
        try:
            while not done.done():
                await publish()
                await asyncio.wait([done], timeout=period)
        finally:
            done.cancel()

    @staticmethod
    async def _repeat_forever(publish: Callable[[], Any], period: float) -> None:
        """Calls @p publish every @p period of wall time while the event loop runs, until cancelled."""
        while True:
            await publish()
            await asyncio.sleep(period)

    def _stop_stream(self) -> None:
        """Ends the background publications of the last hold()."""
        if self._stream is None:
            return
        self._stream.cancel()
        try:
            self._run(self._stream)
        except asyncio.CancelledError:
            pass
        self._stream = None

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

    def wait(self, seconds: float, wall: bool = False) -> None:
        """Waits @p seconds of virtual time, or of wall time with @p wall (see _sleep), without publishing.

        Ends the readiness and setpoint of the last hold(): the drive sees them stop.
        """
        self._stop_stream()
        self._run(self._sleep(seconds, wall))
