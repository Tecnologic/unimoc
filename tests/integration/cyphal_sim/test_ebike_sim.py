# SPDX-License-Identifier: GPL-3.0-or-later
#
# The e-bike build of the native_sim drive (CONFIG_UNIMOC_EBIKE, scenario
# unimoc.app.cyphal_integration.ebike in app/testcase.yaml), driven by the
# simulated rider and display over the unimoc.sim.pedal_* and
# unimoc.sim.display_* registers. The drive engages on demand: the readiness
# ENGAGED of the bus is ignored, STANDBY or SLEEP release it. The tests share
# one simulation process and run in file order; the scenario without the
# e-bike deselects them (marker `ebike`).

from __future__ import annotations

import pytest

from sim_client import (
    ENGAGED,
    SLEEP,
    VENDOR_ENGAGED,
    VENDOR_LOCAL_CONTROL,
    SimClient,
    natural8,
    real,
)

pytestmark = pytest.mark.ebike

RIDER_REGISTERS = (
    "unimoc.sim.pedal_cadence",
    "unimoc.sim.pedal_torque",
    "unimoc.sim.display_on",
    "unimoc.sim.display_level",
    "unimoc.sim.display_max_speed",
    "unimoc.sim.display_light",
    "unimoc.sim.display_frames",
)
CADENCE = 6.0  # [rad/s], about 57 rpm
PEDAL_TORQUE = 20.0  # [N m]


def rider(sim: SimClient, cadence: float, torque: float) -> None:
    assert sim.write("unimoc.sim.pedal_cadence", real(cadence)) == pytest.approx(cadence)
    assert sim.write("unimoc.sim.pedal_torque", real(torque)) == pytest.approx(torque)


def display(sim: SimClient, level: int, on: bool = True) -> None:
    assert sim.write("unimoc.sim.display_level", natural8(level)) == level
    assert sim.write("unimoc.sim.display_on", real(1.0 if on else 0.0)) == on


def test_rider_registers_are_volatile_with_limits(sim: SimClient):
    names = sim.register_names()
    for name in RIDER_REGISTERS:
        assert name in names
        response = sim.access(name)
        assert not response.persistent, name
        assert response.mutable == (name != "unimoc.sim.display_frames"), name
    assert sim.read("unimoc.sim.display_on") == 0
    assert sim.read("unimoc.sim.display_max_speed") == 25
    assert sim.write("unimoc.sim.pedal_cadence", real(-1.0)) == pytest.approx(0.0)  # no direction
    assert sim.write("unimoc.sim.display_level", natural8(8)) == 0  # levels 0 to 7
    assert sim.write("unimoc.sim.display_max_speed", natural8(5)) == 25  # 10 to 63 km/h


def test_the_drive_reports_local_control(sim: SimClient):
    heartbeat = sim.next(sim.heartbeat)
    assert heartbeat.vendor_specific_status_code & VENDOR_LOCAL_CONTROL


def test_the_readiness_of_the_bus_does_not_engage_the_drive(sim: SimClient):
    # The test motor gets a magnet (torque constant 1.5 * 1 * 0.05 = 75 mN m/A) on a locked rotor.
    assert sim.write("unimoc.motor.pmsm.flux_pm", real(0.05)) == pytest.approx(0.05)
    assert sim.write("unimoc.sim.shaft_mode", natural8(1)) == 1  # imposed speed, 0 rad/s
    sim.hold(1.0, ENGAGED, sim.current_setpoint(0.0, 2.0))
    assert not sim.next(sim.telemetry).engaged


def test_the_display_alone_does_not_engage_but_is_answered(sim: SimClient):
    sim.wait(1.2)  # no STANDBY from the fixture any more: the e-bike re-arms with the pedals still
    display(sim, 3)
    frames = sim.expect_register("unimoc.sim.display_frames", lambda count: count >= 3)
    assert frames >= 3
    assert not sim.next(sim.telemetry).engaged


def test_pedaling_engages_the_drive_with_assist(sim: SimClient):
    rider(sim, CADENCE, PEDAL_TORQUE)
    telemetry = sim.expect(sim.telemetry, lambda t: t.engaged and t.i_q_ref > 1.0, timeout=5.0)
    assert telemetry.power_stage == 1  # ENABLED
    sim.expect(sim.heartbeat, lambda h: h.vendor_specific_status_code & VENDOR_ENGAGED)


def test_stopping_the_pedals_ends_the_assist_and_releases_the_drive(sim: SimClient):
    rider(sim, 0.0, 0.0)
    sim.expect(sim.telemetry, lambda t: abs(t.i_q_ref) < 1e-3, timeout=3.0)
    # The engage hold (2 s) at zero torque, then the drive is released.
    telemetry = sim.expect(sim.telemetry, lambda t: not t.engaged, timeout=6.0)
    assert telemetry.power_stage == 0  # DISABLED


def test_level_zero_gives_no_assist(sim: SimClient):
    display(sim, 0)
    rider(sim, CADENCE, PEDAL_TORQUE)
    sim.wait(1.5)
    assert not sim.next(sim.telemetry).engaged
    rider(sim, 0.0, 0.0)


def test_a_silent_display_gives_no_assist(sim: SimClient):
    assert sim.write("unimoc.sim.display_on", real(0.0)) == 0
    assert sim.read("unimoc.sim.display_frames") == 0  # the silenced display forgets its frames
    assert sim.write("unimoc.sim.display_level", natural8(3)) == 3  # kept for the next display_on
    sim.wait(2.5)  # the display timeout (2 s): the last request no longer counts
    rider(sim, CADENCE, PEDAL_TORQUE)
    sim.wait(1.5)
    assert not sim.next(sim.telemetry).engaged
    rider(sim, 0.0, 0.0)


def test_sleep_on_the_bus_takes_the_drive_off_the_e_bike(sim: SimClient):
    assert sim.write("unimoc.sim.display_on", real(1.0)) == 1
    rider(sim, CADENCE, PEDAL_TORQUE)
    sim.expect(sim.telemetry, lambda t: t.engaged, timeout=5.0)
    sim.hold(0.5, SLEEP)
    sim.expect(sim.telemetry, lambda t: not t.engaged)
    # The rider keeps pedaling: the drive stays released until the pedals stood still.
    sim.wait(1.5)
    assert not sim.next(sim.telemetry).engaged
    rider(sim, 0.0, 0.0)
    sim.wait(1.5)  # the re-arm time (1 s)
    rider(sim, CADENCE, PEDAL_TORQUE)
    sim.expect(sim.telemetry, lambda t: t.engaged, timeout=5.0)
    rider(sim, 0.0, 0.0)
    sim.expect(sim.telemetry, lambda t: not t.engaged, timeout=8.0)


def test_the_torque_offset_is_a_persistent_setting(sim: SimClient):
    response = sim.access("unimoc.ebike.torque_offset")
    assert response.mutable and response.persistent
    assert sim.write("unimoc.ebike.torque_offset", real(0.5)) == pytest.approx(0.5)
    assert sim.write("unimoc.ebike.torque_offset", real(20.0)) == pytest.approx(0.5)  # at most 10 N m
    assert sim.write("unimoc.ebike.torque_offset", real(0.0)) == pytest.approx(0.0)
