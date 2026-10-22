# SPDX-License-Identifier: GPL-3.0-or-later
#
# The native_sim drive as a Cyphal node, tested from pycyphal over the slcan
# pseudo terminal: node services, registers, arming, setpoints, telemetry,
# the simulation registers, diagnostics, traces (cymon) and persistence across
# a restart.
# The tests share one simulation process and run in file order.

from __future__ import annotations

import math

import pytest

from conftest import wait_for_pty
from sim_client import (
    DRIVE,
    ENGAGED,
    PORT_CYMON_READ,
    PORT_CYMON_VARIABLES,
    SLEEP,
    STANDBY,
    SimClient,
    natural8,
    real,
)
import cymon


def test_heartbeat_and_get_info(sim: SimClient):
    heartbeat = sim.wait_for_node()
    assert heartbeat.mode.value == 0  # OPERATIONAL
    info = sim.get_info()
    assert info.protocol_version.major == 1
    assert info.name.tobytes().decode() == "unimoc"
    assert info.software_vcs_revision_id != 0  # git hash of the build


def test_port_list_names_the_drive_ports(sim: SimClient):
    port_list = sim.receive(sim.port_list, timeout=12.0)[0]
    publishers = {item.value for item in port_list.publishers.sparse_list}
    subscribers = {item.value for item in port_list.subscribers.sparse_list}
    assert {7509, 212, 213} <= publishers
    assert {106, 107, 100} <= subscribers
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
    response = sim.access("unimoc.motor.pole_pairs")
    assert response.mutable and response.persistent
    assert response.value.natural8 is not None

    assert sim.write("unimoc.motor.limits.i_max", real(20.0)) == pytest.approx(20.0)
    assert sim.write("unimoc.motor.limits.i_max", real(500.0)) == pytest.approx(20.0)  # above the capability
    assert sim.write("unimoc.motor.pole_pairs", natural8(7)) == 7
    assert sim.write("unimoc.does_not_exist", real(1.0)) is None  # unknown register: empty value
    assert sim.write("uavcan.pub.telemetry.type", real(1.0)) == "unimoc.drive.Telemetry.0.1"


def test_unknown_commands_are_rejected(sim: SimClient):
    assert sim.command(65534) == 3  # POWER_OFF: BAD_COMMAND
    assert sim.command(42) == 3


def test_engage_current_setpoint_and_telemetry(sim: SimClient):
    assert sim.write("unimoc.sim.shaft_mode", natural8(1)) == 1  # imposed speed, 0 rad/s: locked rotor
    sim.hold(1.5, ENGAGED, sim.current_setpoint(0.0, 8.0))
    telemetry = sim.latest(sim.telemetry, settle=0.0)
    assert telemetry.engaged
    assert telemetry.power_stage == 1  # ENABLED
    assert telemetry.i_q_ref == pytest.approx(8.0)
    assert telemetry.i_q == pytest.approx(8.0, abs=0.5)
    assert telemetry.dc_link_voltage == pytest.approx(48.0, abs=0.5)
    assert any(record == "engaged" for record in sim.records)

    # The current limit of the previous test (20 A) applies.
    sim.hold(0.5, ENGAGED, sim.current_setpoint(0.0, 40.0))
    assert sim.latest(sim.telemetry, settle=0.0).i_q_ref == pytest.approx(20.0)


def test_readiness_timeout_disengages_with_a_record(sim: SimClient):
    sim.hold(0.5, ENGAGED, sim.current_setpoint(0.0, 2.0))
    sim.wait(1.2)  # no readiness for longer than the 0.5 s command timeout
    telemetry = sim.latest(sim.telemetry)
    assert not telemetry.engaged
    assert telemetry.power_stage == 0  # DISABLED
    assert "disengaged: readiness timeout" in sim.records


def test_store_is_refused_while_engaged(sim: SimClient):
    sim.hold(0.5, ENGAGED, sim.current_setpoint(0.0, 1.0))
    assert sim.command(65530) == 5  # STORE: BAD_STATE while the power stage is enabled
    sim.hold(0.2, SLEEP)
    assert not sim.latest(sim.telemetry).engaged


def test_torque_setpoint_uses_the_torque_constant(sim: SimClient):
    assert sim.write("unimoc.motor.pmsm.flux_pm", real(5e-3)) == pytest.approx(5e-3)
    torque_constant = 1.5 * 7 * 5e-3  # pole pairs written above
    sim.hold(0.2, STANDBY)
    sim.hold(1.0, ENGAGED, sim.torque_setpoint(2.0 * torque_constant))
    assert sim.latest(sim.telemetry, settle=0.0).i_q_ref == pytest.approx(2.0, rel=1e-3)
    sim.hold(0.2, SLEEP)


def test_external_enable_blocks_engage(sim: SimClient):
    assert sim.write("unimoc.sim.external_enable", real(0.0)) == 0
    sim.hold(0.5, ENGAGED, sim.current_setpoint(0.0, 1.0))
    telemetry = sim.latest(sim.telemetry, settle=0.0)
    assert not telemetry.engaged
    assert telemetry.power_stage == 3  # BLOCKED
    assert sim.write("unimoc.sim.external_enable", real(1.0)) == 1
    assert sim.latest(sim.telemetry).power_stage == 2  # FAULT: the break input latched
    assert sim.command(65530) == 5  # no flash save with a latched fault
    sim.hold(0.2, STANDBY)  # acknowledges it without engaging
    telemetry = sim.latest(sim.telemetry)
    assert telemetry.power_stage == 0 and not telemetry.engaged


def test_imposed_speed_drives_the_plant(sim: SimClient):
    assert sim.write("unimoc.sim.imposed_speed", real(50.0)) == pytest.approx(50.0)
    plant = sim.latest(sim.plant)
    assert plant.mechanical_speed == pytest.approx(50.0, rel=1e-3)
    first = plant.mechanical_angle
    sim.wait(0.5)
    advance = sim.latest(sim.plant, settle=0.0).mechanical_angle - first
    assert advance > 10.0  # about 25 rad in 0.5 s
    assert sim.write("unimoc.sim.imposed_speed", real(0.0)) == 0
    assert math.isfinite(advance)


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
    # Locked rotor (imposed speed 0 from the earlier tests), 4 A step.
    sim.hold(0.5, ENGAGED, sim.current_setpoint(0.0, 4.0))
    sim.hold(0.2, SLEEP)
    sim.trace_wait_complete()
    frames = sim.trace_frames()
    assert len(frames) == 400
    assert all(frame[0] < 2.0 for frame in frames[:40])  # before the trigger sample
    assert frames[40][0] == pytest.approx(4.0)
    settled = [frame[1] for frame in frames[-100:]]
    assert sum(settled) / len(settled) == pytest.approx(4.0, abs=0.5)


def test_store_and_restart_keep_the_settings(sim: SimClient, dut):
    assert sim.command(65530) == 0  # STORE
    assert sim.command(65535) == 0  # RESTART: the simulation re-executes itself
    sim.close()
    restarted = SimClient(sim.loop, wait_for_pty(dut))
    try:
        restarted.wait_for_node()
        assert restarted.read("unimoc.motor.pole_pairs") == 7
        assert restarted.read("unimoc.motor.limits.i_max") == pytest.approx(20.0)
        assert restarted.read("unimoc.sim.shaft_mode") == 0  # simulation registers are volatile
        assert restarted.command(65532) == 0  # FACTORY_RESET for the next run
    finally:
        restarted.close()
