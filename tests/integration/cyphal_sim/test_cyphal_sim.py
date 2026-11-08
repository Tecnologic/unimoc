# SPDX-License-Identifier: GPL-3.0-or-later
#
# The native_sim drive as a Cyphal node, tested from pycyphal over the slcan
# pseudo terminal: node services, registers, arming, setpoints, telemetry,
# the simulation registers, diagnostics, traces (cymon), persistence across
# a restart and the register description file (--describe-registers).
# The tests share one simulation process and run in file order.

from __future__ import annotations

import json
import math
import subprocess
from pathlib import Path

import pytest

from conftest import wait_for_pty
from sim_client import (
    CART_TIMEOUT,
    COMMAND_TIMEOUT,
    CONTROL_CYCLE,
    DRIVE,
    ENGAGED,
    PORT_CYMON_READ,
    PORT_CYMON_VARIABLES,
    PORT_READINESS,
    READINESS_FOLLOWS_NODE_ID,
    SLEEP,
    STANDBY,
    VENDOR_EMERGENCY_STOP,
    VENDOR_ENGAGED,
    VENDOR_SETTINGS_PENDING,
    VENDOR_TRIP_LATCHED,
    SimClient,
    natural8,
    natural16,
    natural32,
    real,
)
import cymon
import unimoc.drive

VALUE_TYPES = (
    "empty", "string", "unstructured", "bit",
    "integer64", "integer32", "integer16", "integer8",
    "natural64", "natural32", "natural16", "natural8",
    "real64", "real32", "real16",
)


def value_type(value) -> tuple[str, int]:
    """The selected field of a uavcan.register.Value and its number of elements."""
    for name in VALUE_TYPES:
        field = getattr(value, name, None)
        if field is not None:
            return name, (0 if name == "empty" else len(field.value))
    raise AssertionError(f"unknown register value {value}")


def describe_registers(dut, directory: Path) -> dict:
    """Runs the binary of the test with --describe-registers on a fresh flash file."""
    executable = Path(dut.device_config.build_dir) / "zephyr" / "zephyr.exe"
    output = directory / "registers.json"
    subprocess.run(
        [str(executable), f"-flash={directory / 'flash.bin'}", f"--describe-registers={output}"],
        check=True, timeout=60, capture_output=True,
    )
    return json.loads(output.read_text())



def test_heartbeat_and_get_info(sim: SimClient):
    heartbeat = sim.wait_for_node()
    assert heartbeat.mode.value == 0  # OPERATIONAL
    info = sim.get_info()
    assert info.protocol_version.major == 1
    assert info.name.tobytes().decode() == "unimoc"
    assert info.software_vcs_revision_id != 0  # git hash of the build
    # Disabled, nothing latched, no local control (the pytest build has no e-bike).
    assert heartbeat.vendor_specific_status_code == 0
    assert heartbeat.health.value == 0  # NOMINAL


def test_port_list_names_the_drive_ports(sim: SimClient):
    port_list = sim.receive(sim.port_list, timeout=12.0)[0]
    publishers = {item.value for item in port_list.publishers.sparse_list}
    subscribers = {item.value for item in port_list.subscribers.sparse_list}
    assert {7509, 212, 213, 214} <= publishers
    assert {PORT_READINESS, 107, 100, 110, 111, 112} <= subscribers
    assert port_list.servers.mask[384] and port_list.servers.mask[385]
    assert port_list.servers.mask[430] and port_list.servers.mask[435]


def test_registers_list_read_write_and_validation(sim: SimClient):
    names = sim.register_names()
    assert len(names) == len(set(names))
    for name in ("uavcan.node.id", "unimoc.motor.pole_pairs", "uavcan.sub.readiness.id",
                 "uavcan.pub.telemetry.type", "unimoc.sim.shaft_mode", "uavcan.node.unique_id"):
        assert name in names
    assert len(names) >= 77 + 5 + 8  # settings, drive info, simulation

    assert sim.read("uavcan.node.id") == DRIVE
    assert sim.read("uavcan.sub.readiness.type") == "reg.udral.service.common.Readiness.0.1"
    assert sim.read("uavcan.sub.readiness.id") == READINESS_FOLLOWS_NODE_ID
    response = sim.access("unimoc.motor.pole_pairs")
    assert response.mutable and response.persistent
    assert response.value.natural8 is not None

    assert sim.write("unimoc.motor.limits.i_max", real(20.0)) == pytest.approx(20.0)
    assert sim.write("unimoc.motor.limits.i_max", real(500.0)) == pytest.approx(20.0)  # above the capability
    assert sim.write("unimoc.motor.pole_pairs", natural8(7)) == 7
    assert sim.write("unimoc.does_not_exist", real(1.0)) is None  # unknown register: empty value
    assert sim.write("uavcan.pub.telemetry.type", real(1.0)) == "unimoc.drive.Telemetry.0.1"
    # Two ports on one subject-ID are rejected; a free one is taken.
    assert sim.write("uavcan.sub.torque_setpoint.id", natural16(PORT_READINESS)) == 100  # readiness
    assert sim.write("uavcan.sub.torque_setpoint.id", natural16(READINESS_FOLLOWS_NODE_ID)) == 100
    assert sim.write("uavcan.pub.cymon_trigger.id", natural16(212)) == 65535  # telemetry
    assert sim.write("uavcan.pub.cymon_trigger.id", natural16(300)) == 300
    assert sim.write("uavcan.pub.cymon_trigger.id", natural16(65535)) == 65535


def test_every_port_has_a_type_register(sim: SimClient):
    """Each port-ID register `uavcan.<pub|sub|cln|srv>.<name>.id` has its read-only `.type` (docs/gui_plan.md)."""
    names = set(sim.register_names())
    ports = [name for name in names
             if name.split(".")[:2][-1] in ("pub", "sub", "cln", "srv") and name.endswith(".id")]
    assert len(ports) >= 20
    for port in ports:
        type_name = port[: -len(".id")] + ".type"
        assert type_name in names, f"{port} has no {type_name}"
        response = sim.access(type_name)
        assert not response.mutable, type_name
        assert response.value.string is not None and response.value.string.value.tobytes(), type_name


def test_registers_json_describes_every_register_on_the_bus(dut, sim: SimClient, tmp_path: Path):
    described = describe_registers(dut, tmp_path)
    assert described["schema"] == 1
    info = sim.get_info()
    firmware = described["firmware"]
    major, minor, _ = firmware["version"].split(".", 2)
    assert (int(major), int(minor)) == (info.software_version.major, info.software_version.minor)
    assert int(firmware["vcs_revision"], 16) == info.software_vcs_revision_id

    entries = described["registers"]
    by_name = {entry["name"]: entry for entry in entries}
    assert len(by_name) == len(entries), "a register is described twice"
    # The same registers in the order of uavcan.register.List.
    assert [entry["name"] for entry in entries] == sim.register_names()

    for name, entry in by_name.items():
        response = sim.access(name)
        assert value_type(response.value) == (entry["type"], entry["size"]), name
        assert (response.mutable, response.persistent) == (entry["mutable"], entry["persistent"]), name
        assert entry["help"], f"{name} has no help"
        if entry["mutable"]:
            assert entry["applies"] in ("immediate", "disengaged", "restart"), name
        if "min" in entry and "max" in entry:
            assert entry["min"] <= entry["max"], name

    # The persistent settings, among them the e-bike registers past the 128th.
    settings = [entry for entry in entries if entry["persistent"] and entry["mutable"]]
    assert len(settings) == 132
    assert "unimoc.ebike.display_timeout" in by_name


def test_unknown_commands_are_rejected(sim: SimClient):
    assert sim.command(65534) == 3  # POWER_OFF: BAD_COMMAND
    assert sim.command(42) == 3


def test_engage_current_setpoint_and_telemetry(sim: SimClient):
    assert sim.write("unimoc.sim.shaft_mode", natural8(1)) == 1  # imposed speed, 0 rad/s: locked rotor
    mark = sim.record_mark()
    sim.hold(0.5, ENGAGED, sim.current_setpoint(0.0, 8.0))
    telemetry = sim.expect(sim.telemetry, lambda t: t.engaged and abs(t.i_q - 8.0) < 0.5)
    assert telemetry.power_stage == 1  # ENABLED
    assert telemetry.i_q_ref == pytest.approx(8.0)
    assert telemetry.dc_link_voltage == pytest.approx(48.0, abs=0.5)
    sim.expect_record("engaged", mark)
    # ENABLED and engaged
    sim.expect(sim.heartbeat, lambda h: h.vendor_specific_status_code & 0x07 == 1
               and h.vendor_specific_status_code & VENDOR_ENGAGED)

    # The current limit of the previous test (20 A) applies.
    sim.hold(0.2, ENGAGED, sim.current_setpoint(0.0, 40.0))
    sim.expect(sim.telemetry, lambda t: t.i_q_ref == pytest.approx(20.0))


def test_readiness_timeout_disengages_with_a_record(sim: SimClient):
    sim.hold(0.2, ENGAGED, sim.current_setpoint(0.0, 2.0))
    sim.expect(sim.telemetry, lambda t: t.engaged)
    mark = sim.record_mark()
    sim.wait(0.0)  # the readiness stops: the command timeout disengages
    telemetry = sim.expect(sim.telemetry, lambda t: not t.engaged)
    assert telemetry.power_stage == 0  # DISABLED
    sim.expect_record("disengaged: readiness timeout", mark)


def test_store_is_refused_while_engaged(sim: SimClient):
    sim.hold(0.2, ENGAGED, sim.current_setpoint(0.0, 1.0))
    sim.expect(sim.telemetry, lambda t: t.engaged and t.power_stage == 1)
    assert sim.command(65530) == 5  # STORE: BAD_STATE while the power stage is enabled
    sim.hold(0.2, SLEEP)
    sim.expect(sim.telemetry, lambda t: not t.engaged)


def test_torque_setpoint_uses_the_torque_constant(sim: SimClient):
    assert sim.write("unimoc.motor.pmsm.flux_pm", real(5e-3)) == pytest.approx(5e-3)
    torque_constant = 1.5 * 7 * 5e-3  # pole pairs written above
    sim.hold(0.2, STANDBY)
    sim.hold(0.2, ENGAGED, sim.torque_setpoint(2.0 * torque_constant))
    sim.expect(sim.telemetry, lambda t: t.engaged and t.i_q_ref == pytest.approx(2.0, rel=1e-3))
    sim.hold(0.2, SLEEP)


def test_wheel_command_and_drive_status(sim: SimClient):
    torque_constant = 1.5 * 7 * 5e-3  # the motor of the test before
    # The wheel command and the pedal state are stale after 0.1 s by default; on
    # a loaded machine the publications of this client can pause that long.
    assert sim.write("unimoc.cart.timeout", real(CART_TIMEOUT)) == pytest.approx(CART_TIMEOUT)
    sim.hold(0.2, STANDBY)  # applied while disengaged
    sim.hold(0.2, ENGAGED, sim.wheel_command(2.0 * torque_constant))
    sim.expect(sim.telemetry, lambda t: t.engaged and t.i_q_ref == pytest.approx(2.0, rel=1e-3))
    # The estimated torque follows the current.
    status = sim.expect(sim.drive_status, lambda s: s.engaged
                        and abs(s.estimated_torque - 2.0 * torque_constant) < 0.5 * torque_constant)
    assert not status.limp_home
    assert status.wheel == 0
    assert status.dc_link_voltage == pytest.approx(48.0, abs=0.5)
    assert math.isnan(status.motor_temperature)
    sim.hold(0.2, SLEEP)


def test_limp_home_on_the_pedals_without_the_vdc(sim: SimClient):
    torque_constant = 1.5 * 7 * 5e-3
    assert sim.write("unimoc.cart.limp_home_torque", real(2.0 * torque_constant)) == pytest.approx(2.0 * torque_constant)
    sim.hold(0.2, STANDBY)  # applied while disengaged
    sim.hold(0.2, ENGAGED, sim.wheel_command(0.0))
    sim.expect(sim.telemetry, lambda t: t.engaged)
    mark = sim.record_mark()
    # The VDC stops (readiness and wheel command), the pedal unit stays beyond the readiness timeout.
    sim.hold(COMMAND_TIMEOUT + 0.5, None, sim.pedals(1.0, 0.0), period=0.02)
    sim.expect(sim.telemetry, lambda t: t.engaged and t.i_q_ref == pytest.approx(2.0, rel=1e-3))
    sim.expect(sim.drive_status, lambda s: s.limp_home)
    assert "disengaged: readiness timeout" not in sim.records[mark:]
    # Without the pedals the torque ramps to zero and the readiness timeout disengages.
    sim.wait(0.0)
    sim.expect(sim.telemetry, lambda t: not t.engaged, timeout=10.0)
    sim.expect_record("disengaged: readiness timeout", mark)
    assert sim.write("unimoc.cart.timeout", real(0.1)) == pytest.approx(0.1)  # the default again
    sim.hold(0.2, SLEEP)


def test_external_enable_blocks_engage(sim: SimClient):
    assert sim.write("unimoc.sim.external_enable", real(0.0)) == 0
    sim.hold(0.2, ENGAGED, sim.current_setpoint(0.0, 1.0))
    telemetry = sim.expect(sim.telemetry, lambda t: t.power_stage == 3)  # BLOCKED
    assert not telemetry.engaged
    sim.hold(0.3, ENGAGED, sim.current_setpoint(0.0, 1.0))
    assert not sim.next(sim.telemetry).engaged  # still refused
    assert sim.write("unimoc.sim.external_enable", real(1.0)) == 1
    sim.expect(sim.telemetry, lambda t: t.power_stage == 2)  # FAULT: the break input latched
    assert sim.command(65530) == 5  # no flash save with a latched fault
    sim.hold(0.2, STANDBY)  # acknowledges it without engaging
    telemetry = sim.expect(sim.telemetry, lambda t: t.power_stage == 0)
    assert not telemetry.engaged


def test_imposed_speed_drives_the_plant(sim: SimClient):
    assert sim.write("unimoc.sim.imposed_speed", real(50.0)) == pytest.approx(50.0)
    plant = sim.expect(sim.plant, lambda p: abs(p.mechanical_speed - 50.0) < 0.05)
    first = plant.mechanical_angle
    sim.wait(0.5)
    advance = sim.next(sim.plant).mechanical_angle - first
    assert advance > 10.0  # about 25 rad in 0.5 s
    assert sim.write("unimoc.sim.imposed_speed", real(0.0)) == 0
    assert math.isfinite(advance)


def settled_plant(sim: SimClient, timeout: float = 5.0):
    """The plant once the shaft stands: two plant messages in a row at the same angle."""
    previous = None

    def still(plant) -> bool:
        nonlocal previous
        settled = (previous is not None and abs(plant.mechanical_angle - previous) < 5e-3
                   and abs(plant.mechanical_speed) < 0.1)
        previous = plant.mechanical_angle
        return settled

    return sim.expect(sim.plant, still, timeout)


def test_speed_setpoint_runs_the_speed_loop(sim: SimClient):
    # Pole pairs (7) and flux (5 mWb) from the tests above; Hall fusion and a speed PI.
    assert sim.write("unimoc.observer.hall.enable", natural8(1)) == 1
    assert sim.write("unimoc.observer.pmsm_flux.R_speed", real(1.0)) == pytest.approx(1.0)
    assert sim.write("unimoc.control.speed.kp", real(0.01)) == pytest.approx(0.01)
    assert sim.write("unimoc.control.speed.ki", real(40.0)) == pytest.approx(40.0)
    assert sim.write("unimoc.sim.shaft_mode", natural8(0)) == 0  # free shaft
    sim.hold(0.2, STANDBY)  # the settings apply while disengaged

    sim.hold(0.5, ENGAGED, sim.speed_setpoint(30.0))
    sim.expect(sim.telemetry, lambda t: t.engaged)
    sim.expect(sim.plant, lambda p: abs(p.mechanical_speed - 30.0) < 2.0)

    # UDRAL servo: the velocity field selects the speed loop.
    sim.hold(0.5, ENGAGED, sim.servo_setpoint(math.nan, -20.0, math.nan))
    sim.expect(sim.plant, lambda p: abs(p.mechanical_speed + 20.0) < 2.0)

    # Position loop: a 3 rad move between two held positions.
    sim.hold(1.5, ENGAGED, sim.position_setpoint(0.0))
    start = settled_plant(sim).mechanical_angle
    sim.hold(1.5, ENGAGED, sim.position_setpoint(3.0))
    moved = settled_plant(sim).mechanical_angle - start
    assert moved == pytest.approx(3.0, abs=0.1)

    sim.hold(0.3, SLEEP)
    assert sim.write("unimoc.sim.shaft_mode", natural8(1)) == 1  # locked again (imposed 0 rad/s)


START_HOMING = 0x0400
HOMING_SEARCHING = 1
HOMING_DONE = 3


def test_homing_runs_against_the_stop(sim: SimClient):
    # The motor and speed loop settings of the test above; the homing current bounds the push.
    status, output = sim.command_output(START_HOMING)
    assert status == 5 and "position control" in output  # not engaged
    assert sim.write("unimoc.control.pos.homing_current", real(3.0)) == pytest.approx(3.0)
    assert sim.write("unimoc.sim.shaft_mode", natural8(0)) == 0  # free shaft
    sim.hold(0.2, STANDBY)  # the settings apply while disengaged
    sim.hold(0.5, ENGAGED, sim.speed_setpoint(0.0))
    sim.expect(sim.telemetry, lambda t: t.engaged)
    status, output = sim.command_output(START_HOMING)
    assert status == 5 and "position control" in output  # speed mode

    sim.hold(0.5, ENGAGED, sim.position_setpoint(0.0))
    sim.expect(sim.telemetry, lambda t: t.engaged)
    assert sim.command(START_HOMING) == 0
    sim.expect(sim.homing, lambda h: h.value == HOMING_SEARCHING)
    sim.expect(sim.plant, lambda p: abs(p.mechanical_speed - 5.0) < 1.0)
    status, output = sim.command_output(START_HOMING)
    assert status == 5 and "runs already" in output

    # The stop: a locked shaft. The run pushes with the homing current and takes the stop as zero.
    assert sim.write("unimoc.sim.shaft_mode", natural8(1)) == 1
    # A plant message sent before the lock would still show the shaft turning.
    stop = sim.expect(sim.plant, lambda p: abs(p.mechanical_speed) < 1e-6).mechanical_angle
    sim.expect(sim.homing, lambda h: h.value == HOMING_DONE)
    sim.hold(0.5, ENGAGED, sim.position_setpoint(0.0))
    assert abs(sim.next(sim.telemetry).i_q) <= 3.0 + 0.3

    # The drive holds the home until the setpoint changes, then moves in the new frame.
    assert sim.write("unimoc.sim.shaft_mode", natural8(0)) == 0
    sim.hold(0.5, ENGAGED, sim.position_setpoint(0.0))
    assert settled_plant(sim).mechanical_angle - stop == pytest.approx(0.0, abs=0.1)
    sim.hold(1.5, ENGAGED, sim.position_setpoint(-2.0))
    assert settled_plant(sim).mechanical_angle - stop == pytest.approx(-2.0, abs=0.1)

    sim.hold(0.3, SLEEP)
    sim.expect(sim.telemetry, lambda t: not t.engaged)
    assert sim.next(sim.homing).value == HOMING_DONE  # the home holds across a disengage
    assert sim.write("unimoc.sim.shaft_mode", natural8(1)) == 1


def test_timing_registers_without_the_probes(sim: SimClient):
    # The application builds without CONFIG_UNIMOC_HOT_PATH_TIMING: the registers exist, nothing is counted.
    assert sim.read("unimoc.timing.clock_hz") == 1_000_000_000
    response = sim.access("unimoc.timing.control_cycle")
    assert list(response.value.natural32.value) == [0, 0, 0, 0]
    assert not response.mutable
    assert len(sim.access("unimoc.timing.overruns").value.natural32.value) == 2
    assert sim.command(0x0300) == 0  # RESET_TIMING
    assert sim.access("unimoc.timing.reset").value.empty is not None  # the register is gone


def test_dc_link_overvoltage_trips_the_drive(sim: SimClient):
    sim.hold(0.2, STANDBY)
    sim.hold(0.2, ENGAGED, sim.current_setpoint(0.0, 1.0))
    sim.expect(sim.telemetry, lambda t: t.engaged)
    mark = sim.record_mark()
    assert sim.write("unimoc.sim.dc_link_voltage", real(59.0)) == pytest.approx(59.0)  # trip at 58 V
    telemetry = sim.expect(sim.telemetry, lambda t: not t.engaged)
    assert telemetry.power_stage == 2  # FAULT
    sim.expect_record("disengaged: DC-link overvoltage trip", mark)
    sim.hold(1.2, ENGAGED)  # a rearm is needed: ENGAGED alone does not clear the trip
    heartbeat = sim.next(sim.heartbeat)
    assert heartbeat.vendor_specific_status_code & VENDOR_TRIP_LATCHED
    assert not heartbeat.vendor_specific_status_code & VENDOR_ENGAGED
    assert heartbeat.health.value == 1  # ADVISORY
    assert not sim.next(sim.telemetry).engaged

    assert sim.write("unimoc.sim.dc_link_voltage", real(48.0)) == pytest.approx(48.0)
    sim.hold(0.2, STANDBY)  # acknowledges the trip
    sim.expect(sim.telemetry, lambda t: t.power_stage == 0)  # DISABLED
    sim.expect(sim.heartbeat, lambda h: h.vendor_specific_status_code == 0 and h.health.value == 0)


def test_settings_that_need_a_restart_are_reported_while_engaged(sim: SimClient):
    sim.hold(0.2, STANDBY)
    sim.hold(0.2, ENGAGED, sim.current_setpoint(0.0, 1.0))
    sim.expect(sim.telemetry, lambda t: t.engaged)
    assert not sim.read("unimoc.settings.pending")

    # A gain is applied at once; a limit waits for the disengage and says so.
    assert sim.write("unimoc.control.current.kp_q", real(0.6)) == pytest.approx(0.6)
    sim.hold(0.2, ENGAGED, sim.current_setpoint(0.0, 1.0))
    assert not sim.read("unimoc.settings.pending")
    try:
        mark = sim.record_mark()
        assert sim.write("unimoc.motor.limits.i_max", real(19.0)) == pytest.approx(19.0)
        sim.expect_register("unimoc.settings.pending", bool)
        sim.expect_record("settings wait for the disengage: unimoc.motor.limits.i_max", mark)
        sim.expect(sim.heartbeat, lambda h: h.vendor_specific_status_code & VENDOR_SETTINGS_PENDING)
        assert sim.next(sim.telemetry).engaged  # the pending limit does not disengage

        mark = sim.record_mark()
        sim.hold(0.2, SLEEP)
        sim.expect_register("unimoc.settings.pending", lambda pending: not pending)
        sim.expect_record("waiting settings applied", mark)
        sim.expect(sim.heartbeat, lambda h: not h.vendor_specific_status_code & VENDOR_SETTINGS_PENDING)
    finally:
        # Also after a failure: the store test below expects the limit of before.
        sim.hold(0.2, SLEEP)
        assert sim.write("unimoc.motor.limits.i_max", real(20.0)) == pytest.approx(20.0)  # as before


def test_trace_variables_ports_and_registers(sim: SimClient):
    variables = sim.trace_variables()
    names = [name for name, _unit in variables]
    assert names[:12] == ["i_d", "i_q", "i_d_ref", "i_q_ref", "u_d", "u_q", "electrical_angle",
                          "electrical_speed", "dc_link_voltage", "i_a", "i_b", "i_c"]
    assert variables[1] == ("i_q", "A")
    assert ("sim.electrical_speed", "rad/s") in variables  # plant truth of the simulation

    info = sim.trace_buffer_info()
    assert info.buffer_bytes == 8192
    assert info.max_channels == 8
    assert info.max_read_samples == 64
    assert info.tick_period_us == pytest.approx(62.5)  # the control cycle at 32 kHz PWM

    assert sim.read("uavcan.srv.cymon_read.id") == PORT_CYMON_READ
    assert sim.read("uavcan.srv.cymon_read.type") == "cymon.ReadSamples.0.1"
    assert sim.read("uavcan.pub.cymon_trigger.id") == 65535  # disabled by default
    response = sim.access("uavcan.srv.cymon_setup.id")
    assert response.mutable and response.persistent
    port_list = sim.receive(sim.port_list, timeout=12.0)[0]
    assert all(port_list.servers.mask[port] for port in range(PORT_CYMON_VARIABLES, PORT_CYMON_READ + 1))


def test_trace_capture_of_i_q_with_an_immediate_trigger(sim: SimClient):
    i_q = [name for name, _unit in sim.trace_variables()].index("i_q")
    setup = sim.trace_setup([i_q], sample_period_us=250.0, num_samples=200, pretrigger_samples=20)
    assert setup.result.value == 0
    assert setup.actual_sample_period_us == pytest.approx(250.0)  # 4 control cycles
    assert sim.trace_arm(cymon.TriggerMode_0_1.IMMEDIATE) == 0
    status = sim.trace_wait_complete()
    assert status.num_channels == 1
    assert status.num_samples == 200
    assert status.pretrigger_samples == 20
    assert status.sample_period_us == pytest.approx(250.0)
    frames = sim.trace_frames()
    assert len(frames) == 200
    assert all(abs(frame[0]) < 0.5 for frame in frames)  # disengaged: no current


def test_trace_triggers_on_a_current_step(sim: SimClient):
    variables = [name for name, _unit in sim.trace_variables()]
    i_q_ref, i_q = variables.index("i_q_ref"), variables.index("i_q")
    setup = sim.trace_setup([i_q_ref, i_q], sample_period_us=125.0, num_samples=400, pretrigger_samples=40)
    assert setup.result.value == 0
    assert sim.trace_arm(cymon.TriggerMode_0_1.RISING, source=i_q_ref, level=2.0) == 0
    assert sim.trace_read().state.value == cymon.CaptureState_0_1.ARMED
    # The trigger counts once the pre-trigger part is recorded (40 samples, 80 control cycles):
    # a step before that is no edge. The register read follows the arm in the drive's order.
    armed_at = sim.read("unimoc.sim.cycle_count")
    sim.expect_register("unimoc.sim.cycle_count", lambda cycles: cycles >= armed_at + 160)
    # Locked rotor (imposed speed 0 from the earlier tests), 4 A step.
    sim.hold(0.2, ENGAGED, sim.current_setpoint(0.0, 4.0))
    sim.expect(sim.telemetry, lambda t: t.engaged and t.i_q_ref == pytest.approx(4.0))
    sim.trace_wait_complete()  # the readiness stays on: a capture after the trigger
    sim.hold(0.2, SLEEP)
    frames = sim.trace_frames()
    assert len(frames) == 400
    assert all(frame[0] < 2.0 for frame in frames[:40])  # before the trigger sample
    assert frames[40][0] == pytest.approx(4.0)
    settled = [frame[1] for frame in frames[-100:]]
    assert sum(settled) / len(settled) == pytest.approx(4.0, abs=0.5)


def test_startup_aid_runs_steps_and_aborts(sim: SimClient):
    status_type = unimoc.drive.CommissioningStatus_0_1
    # Phase 1 expects no motor: the plant's shaft stands, so it induces no current (overcurrent check).
    assert sim.write("unimoc.sim.shaft_mode", natural8(1)) == 1
    assert sim.write("unimoc.sim.imposed_speed", real(0.0)) == 0
    sim.hold(0.2, SLEEP)
    assert sim.next(sim.commissioning).state == status_type.STATE_IDLE
    assert sim.command(status_type.COMMAND_CONTINUE) == 5  # BAD_STATE: nothing runs

    assert sim.command(status_type.COMMAND_START_STARTUP_AID) == 0
    try:
        status = sim.expect(sim.commissioning, lambda s: s.procedure == status_type.PROCEDURE_STARTUP_AID)
        assert status.step == 1  # PWM_DISABLE
        sim.expect(sim.telemetry, lambda t: t.power_stage == 1)  # the FSM releases the outputs

        # A continue is taken once the step has measured; while it measures it is refused, with the reason.
        # ADC_OFFSET_CAL takes one sample per spin of the Cyphal thread (1024, about a second), so the
        # refusal is checked right after the continue, before anything else.
        assert sim.command(status_type.COMMAND_CONTINUE) == 0
        assert sim.command_output(status_type.COMMAND_CONTINUE) == (5, "the step is still measuring")
        sim.expect(sim.commissioning, lambda s: s.step == 2)  # ADC_OFFSET_CAL
        assert sim.command_output(status_type.COMMAND_SET_READING, "vdc=x") == (4, "not a positive number")
        assert sim.command(status_type.COMMAND_SET_READING, "vdc=48.0") == 0
        results = sim.access("unimoc.startup.results").value.natural8.value
        assert len(results) == 14 and results[1] == 1  # PWM_DISABLE passed
    finally:
        # Also after a failure: a running aid keeps the outputs and refuses the simulation reset below.
        assert sim.command(status_type.COMMAND_ABORT) == 0
    status = sim.expect(sim.commissioning, lambda s: s.state == status_type.STATE_ABORTED)
    assert status.step == 13  # FAULT
    assert status.fault == status_type.FAULT_ABORTED
    sim.expect(sim.telemetry, lambda t: t.power_stage == 0)  # DISABLED


def test_paused_simulation_advances_by_whole_cycles(sim: SimClient):
    sim.hold(0.2, SLEEP)
    assert sim.read("unimoc.sim.cycle_period_ns") == 62500  # 32 kHz PWM, four half periods
    assert sim.write("unimoc.sim.paused", natural8(1))
    start = sim.read("unimoc.sim.cycle_count")
    sim.wait(0.2, wall=True)
    assert sim.read("unimoc.sim.cycle_count") == start  # virtual time stands still

    sim.write("unimoc.sim.advance", natural16(1600))  # 0.1 s of virtual time, run before the response
    assert sim.read("unimoc.sim.cycle_count") == start + 1600
    sim.write("unimoc.sim.advance", natural16(20000))  # more than 1 s at once: refused
    assert sim.read("unimoc.sim.cycle_count") == start + 1600

    # Reset: virtual time 0, plant at standstill, still paused; then exactly 16 cycles.
    assert sim.write("unimoc.sim.imposed_speed", real(10.0)) == pytest.approx(10.0)
    sim.write("unimoc.sim.advance", natural16(160))
    sim.write("unimoc.sim.reset", natural8(1))
    assert sim.read("unimoc.sim.cycle_count") == 0
    assert sim.read("unimoc.sim.paused")
    sim.write("unimoc.sim.advance", natural16(16))
    assert sim.read("unimoc.sim.cycle_count") == 16
    assert sim.write("unimoc.sim.imposed_speed", real(0.0)) == 0

    # A settings restart of the controller keeps the pause (the count restarts with it).
    assert sim.write("unimoc.control.speed.kp", real(0.01)) == pytest.approx(0.01)
    sim.hold(0.2, STANDBY, wall=True)
    restarted = sim.read("unimoc.sim.cycle_count")
    sim.wait(0.2, wall=True)
    assert sim.read("unimoc.sim.cycle_count") == restarted

    assert not sim.write("unimoc.sim.paused", natural8(0))
    sim.expect_register("unimoc.sim.cycle_count", lambda count: count > restarted + 1000)  # real time again
    sim.write("unimoc.sim.advance", natural16(10))  # only while paused
    assert not sim.read("unimoc.sim.paused")


def test_paused_drive_timeouts_follow_virtual_time(sim: SimClient):
    assert sim.write("unimoc.sim.shaft_mode", natural8(1)) == 1  # locked rotor
    sim.hold(0.2, ENGAGED, sim.current_setpoint(0.0, 1.0))
    sim.expect(sim.telemetry, lambda t: t.engaged)
    try:
        assert sim.write("unimoc.sim.paused", natural8(1))
        sim.wait(COMMAND_TIMEOUT + 0.5, wall=True)  # no readiness for longer than the command timeout, but paused
        assert sim.next(sim.telemetry).engaged  # the telemetry goes on in real time

        mark = sim.record_mark()
        start = sim.read("unimoc.sim.cycle_count")
        assert sim.write("unimoc.sim.run_until", natural32(start + 4800)) is not None  # 0.3 s of virtual time
        assert sim.read("unimoc.sim.cycle_count") == start + 4800
        assert sim.next(sim.telemetry).engaged  # still inside the timeout
        assert sim.write("unimoc.sim.run_until", natural32(start)) is not None  # backwards: refused
        assert sim.read("unimoc.sim.cycle_count") == start + 4800

        # Beyond the command timeout since the last readiness, at most 1 s per advance.
        remaining = round((COMMAND_TIMEOUT + 0.1 - 0.3) / CONTROL_CYCLE)
        while remaining > 0:
            step = min(remaining, 16000)
            sim.write("unimoc.sim.advance", natural16(step))
            remaining -= step
        sim.expect(sim.telemetry, lambda t: not t.engaged)
        sim.expect_record("disengaged: readiness timeout", mark)
    finally:
        sim.write("unimoc.sim.paused", natural8(0))


def test_dc_link_capacitor_registers(sim: SimClient):
    sim.hold(0.2, SLEEP)
    assert sim.read("unimoc.sim.dc_link_capacitance") == 0.0  # the ideal source by default
    assert sim.write("unimoc.sim.dc_link_capacitance", real(2e-3)) == pytest.approx(2e-3)
    try:
        assert sim.write("unimoc.sim.battery_resistance", real(0.0)) == pytest.approx(0.05)  # refused
        assert sim.write("unimoc.sim.battery_resistance", real(0.1)) == pytest.approx(0.1)
        assert sim.write("unimoc.sim.dc_link_capacitance", real(-1.0)) == pytest.approx(2e-3)  # refused
        assert sim.read("unimoc.sim.contactor_closed")
        sim.hold(0.3, SLEEP)
        assert sim.next(sim.plant).dc_link_voltage == pytest.approx(48.0, abs=0.1)  # charged at the start
        assert sim.next(sim.telemetry).dc_link_voltage == pytest.approx(48.0, abs=0.5)
    finally:
        assert sim.write("unimoc.sim.dc_link_capacitance", real(0.0)) == 0.0


def test_identification_measures_the_simulated_motor(sim: SimClient):
    status_type = unimoc.drive.CommissioningStatus_0_1
    assert sim.write("unimoc.sim.shaft_mode", natural8(1)) == 1  # standstill on a held shaft
    assert sim.write("unimoc.sim.imposed_speed", real(0.0)) == 0
    sim.hold(0.2, SLEEP)
    assert sim.command_output(status_type.COMMAND_START_IDENTIFICATION, "M3") == (4, "expected M1, M2 or M1,M2")
    assert sim.command(status_type.COMMAND_START_IDENTIFICATION, "M1,M2") == 0
    try:
        # The first status of the run; the status of the startup aid before may still be on its way.
        status = sim.expect(sim.commissioning, lambda s: s.procedure == status_type.PROCEDURE_IDENTIFICATION)
        assert status.state == status_type.STATE_MEASURING
        assert sim.command(status_type.COMMAND_START_STARTUP_AID) == 5  # BAD_STATE: one procedure at a time
        sim.expect(sim.telemetry, lambda t: t.power_stage == 1)  # the FSM releases the outputs
        # About 4 s of drive time.
        status = sim.expect(sim.commissioning, lambda s: s.state != status_type.STATE_MEASURING, timeout=20.0)
        assert status.state == status_type.STATE_DONE, status
    finally:
        assert sim.command(status_type.COMMAND_ABORT) == 0
    assert status.failed == 0
    sim.expect(sim.telemetry, lambda t: t.power_stage == 0)  # DISABLED
    # The plant takes the motor of the settings (the test motor: 0.1 Ohm, 1 mH, an ideal bridge).
    assert sim.read("unimoc.identify.result.R") == pytest.approx(sim.read("unimoc.motor.stator.R"), rel=0.03)
    assert sim.read("unimoc.identify.result.L_d") == pytest.approx(sim.read("unimoc.motor.pmsm.L_d"), rel=0.03)
    assert sim.read("unimoc.identify.result.L_q") == pytest.approx(sim.read("unimoc.motor.pmsm.L_q"), rel=0.03)
    assert sim.read("unimoc.identify.result.dead_time") == pytest.approx(0.0, abs=2e-8)


def test_emergency_stop_latches_until_the_restart(sim: SimClient):
    assert sim.command(65531) == 0  # EMERGENCY_STOP
    sim.expect(sim.heartbeat, lambda h: h.vendor_specific_status_code & VENDOR_EMERGENCY_STOP
               and h.health.value == 2)  # CAUTION
    sim.hold(1.2, ENGAGED, sim.current_setpoint(0.0, 1.0))  # engaging is refused
    assert not sim.next(sim.telemetry).engaged
    heartbeat = sim.next(sim.heartbeat)
    assert heartbeat.vendor_specific_status_code & VENDOR_EMERGENCY_STOP
    assert not heartbeat.vendor_specific_status_code & VENDOR_ENGAGED
    sim.hold(0.2, SLEEP)  # the restart of the next test clears the latch


def test_store_and_restart_keep_the_settings(sim: SimClient, dut):
    assert sim.command(65530) == 0  # STORE
    assert sim.command(65535) == 0  # RESTART: the simulation re-executes itself
    sim.close()
    restarted = SimClient(sim.loop, wait_for_pty(dut))
    try:
        heartbeat = restarted.wait_for_node()
        assert not heartbeat.vendor_specific_status_code & VENDOR_EMERGENCY_STOP  # cleared by the restart
        assert restarted.read("unimoc.motor.pole_pairs") == 7
        assert restarted.read("unimoc.motor.limits.i_max") == pytest.approx(20.0)
        assert restarted.read("unimoc.sim.shaft_mode") == 0  # simulation registers are volatile
        assert restarted.command(65532) == 0  # FACTORY_RESET for the next run
    finally:
        restarted.close()
