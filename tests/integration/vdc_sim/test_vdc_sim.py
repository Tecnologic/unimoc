# SPDX-License-Identifier: GPL-3.0-or-later
#
# Black-box tests of the simulated VDC over Cyphal (scenario
# unimoc.app.vdc.cyphal_integration): the node, its registers, the readiness
# of the drives, the wheel command from the torque request and the drive
# status collected. The client plays the four wheel drives.

from __future__ import annotations

import math

import pytest

from vdc_client import ENGAGED, LOCAL, STANDBY, VdcClient, bit, natural8_array, real

# The tolerance of float16 on the bus.
TORQUE_TOLERANCE = 0.05


@pytest.fixture(autouse=True)
def drives(vdc: VdcClient):
    """Every test starts with the client as the drive of every wheel, all reporting, disengaged."""
    assert vdc.read_all("unimoc.vdc.drive_node_ids") is not None
    vdc.write("unimoc.vdc.drive_node_ids", natural8_array([LOCAL] * 4))
    # Wide enough for the gaps between two calls of the test on a loaded machine.
    vdc.write("unimoc.vdc.status_timeout", real(1.0))
    vdc.write("unimoc.vdc.engage", bit(False))
    vdc.write("unimoc.vdc.torque_request", real(0.0))
    vdc.write("unimoc.vdc.front_share", real(0.5))
    vdc.wheels_reporting = [True] * 4
    vdc.engaged = False
    yield
    vdc.write("unimoc.vdc.engage", bit(False))


def test_node_info_and_heartbeat(vdc: VdcClient):
    info = vdc.get_info()
    assert info.name.tobytes().decode() == "org.unimoc.vdc"
    heartbeat = vdc.next_heartbeat()
    assert heartbeat.mode.value == 0  # OPERATIONAL


def test_every_port_has_its_type_register(vdc: VdcClient):
    names = vdc.register_names()
    ports = [name[: -len(".id")] for name in names if name.startswith("uavcan.") and name.endswith(".id")
             and not name.startswith("uavcan.node.")]
    assert "uavcan.pub.wheel_command" in ports
    assert "uavcan.sub.drive_status" in ports
    for port in ports:
        assert f"{port}.type" in names, port
    assert vdc.read("uavcan.pub.wheel_command.type") == "unimoc.cart.WheelCommand.0.1"
    assert vdc.read("uavcan.pub.readiness_rear_left.id") == 1024 + LOCAL


def test_standby_and_zero_torque_while_not_engaged(vdc: VdcClient):
    vdc.write("unimoc.vdc.torque_request", real(40.0))
    readiness = vdc.expect(vdc.readiness, lambda _message: True)
    assert readiness.value == STANDBY
    command = vdc.expect(vdc.wheel_command, lambda _message: True)
    assert all(abs(torque) < TORQUE_TOLERANCE for torque in command.wheel_torque)


def test_engaged_drives_share_the_torque_request(vdc: VdcClient):
    vdc.write("unimoc.vdc.engage", bit(True))
    vdc.expect(vdc.readiness, lambda message: message.value == ENGAGED)
    vdc.engaged = True  # the drives follow the readiness
    vdc.write("unimoc.vdc.torque_request", real(40.0))
    command = vdc.expect(
        vdc.wheel_command,
        lambda message: all(abs(torque - 10.0) < TORQUE_TOLERANCE for torque in message.wheel_torque))
    assert len(command.wheel_torque) == 4
    assert vdc.read_all("unimoc.vdc.drive_available") == [1, 1, 1, 1]
    speeds = vdc.read_all("unimoc.vdc.wheel_speed")
    assert all(math.isclose(speed, 1.0, abs_tol=1e-2) for speed in speeds)

    # More on the front axle.
    vdc.write("unimoc.vdc.front_share", real(0.75))
    vdc.expect(vdc.wheel_command, lambda message: abs(message.wheel_torque[0] - 15.0) < TORQUE_TOLERANCE
               and abs(message.wheel_torque[3] - 5.0) < TORQUE_TOLERANCE)


def test_a_lost_drive_takes_its_axle_out(vdc: VdcClient):
    vdc.write("unimoc.vdc.engage", bit(True))
    vdc.engaged = True
    vdc.write("unimoc.vdc.torque_request", real(40.0))
    vdc.expect(vdc.wheel_command, lambda message: abs(message.wheel_torque[3] - 10.0) < TORQUE_TOLERANCE)
    vdc.write("unimoc.vdc.status_timeout", real(0.2))
    vdc.wheels_reporting = [True, True, True, False]  # the rear right drive stops reporting
    command = vdc.expect(vdc.wheel_command, lambda message: abs(message.wheel_torque[3]) < TORQUE_TOLERANCE)
    assert abs(command.wheel_torque[2]) < TORQUE_TOLERANCE  # its axle partner too
    assert abs(command.wheel_torque[0] - 20.0) < TORQUE_TOLERANCE
    assert abs(command.wheel_torque[1] - 20.0) < TORQUE_TOLERANCE
    heartbeat = vdc.next_heartbeat()
    assert heartbeat.health.value == 1  # ADVISORY: a configured drive is missing
