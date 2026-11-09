# SPDX-License-Identifier: GPL-3.0-or-later
#
# A real drive board (NUCLEO-H743ZI with the drive application, later Jacobi)
# as a Cyphal node on a CAN FD bus, tested from pycyphal: plug-and-play,
# node services, registers, subjects and their rates, traces (cymon), the
# emergency stop, persistence across a restart, the firmware update refusal,
# the engage and the factory reset. Manual only (README.md): the tests share
# one board and run in file order, the last ones change the board's state.

from __future__ import annotations

import time

import pytest

from hw_client import (
    CMD_BEGIN_SOFTWARE_UPDATE,
    CMD_EMERGENCY_STOP,
    CMD_FACTORY_RESET,
    CMD_RESET_TIMING,
    CMD_RESTART,
    CMD_STORE_PERSISTENT_STATES,
    ENGAGED,
    LOCAL,
    PORT_COMMISSIONING,
    PORT_DRIVE_STATUS,
    PORT_TELEMETRY,
    POWER_STAGE_DISABLED,
    POWER_STAGE_ENABLED,
    STANDBY,
    STATUS_BAD_COMMAND,
    STATUS_FAILURE,
    STATUS_SUCCESS,
    VENDOR_EMERGENCY_STOP,
    VENDOR_ENGAGED,
    VENDOR_TRIP_LATCHED,
    HwClient,
    natural8,
    real,
)
import cymon
import unimoc.cart
import unimoc.drive

# Registers every drive build has (a sample of the ones README.md lists).
EXPECTED_REGISTERS = {
    "uavcan.node.id", "uavcan.node.description", "uavcan.node.unique_id",
    "uavcan.pub.drive_status.id", "uavcan.pub.telemetry.id", "uavcan.sub.readiness.id",
    "unimoc.motor.pole_pairs", "unimoc.motor.limits.i_max", "unimoc.control.current.kp_d",
    "unimoc.control.pwm_frequency", "unimoc.telemetry.rate", "unimoc.settings.pending",
    "unimoc.can.bus_off_count", "unimoc.update.state", "unimoc.timing.clock_hz",
}

HEALTH_NOMINAL = 0
HEALTH_CAUTION = 2


def acknowledge(drive: HwClient) -> None:
    """STANDBY acknowledges a latched trip or fault without releasing the outputs."""
    drive.hold_readiness(STANDBY)
    drive.sleep(1.5)
    drive.stop_readiness()
    drive.sleep(1.2)  # the drive has to see the readiness stop before the next test engages
    drive.wait_for_status(lambda beat: not beat.vendor_specific_status_code & VENDOR_TRIP_LATCHED)


@pytest.fixture()
def clean_drive(drive: HwClient) -> HwClient:
    """The board is ready: power stage disabled and no trip. Otherwise the wiring of the bench is wrong (README.md)."""
    acknowledge(drive)
    status = drive.vendor_status()
    assert status & 0x07 == POWER_STAGE_DISABLED and not status & VENDOR_TRIP_LATCHED, (
        f"vendor status {status:#04x}: the break input (PE15 to 3V3) must be released and the three current inputs "
        "inside the overcurrent window, see README.md (bench setup)")
    return drive


# --- Plug-and-play and node services ----------------------------------------------------

def test_plug_and_play_gave_the_drive_a_node_id(drive: HwClient) -> None:
    assert drive.drive is not None and drive.drive != LOCAL
    assert drive.read("uavcan.node.id") == drive.drive


def test_get_info(drive: HwClient) -> None:
    info = drive.get_info()
    assert info.name.tobytes().decode() == "unimoc"
    assert (info.protocol_version.major, info.protocol_version.minor) == (1, 0)
    assert any(info.unique_id), "the unique-ID is the hwinfo device ID"
    assert info.software_vcs_revision_id != 0


def test_heartbeat_is_once_per_second(drive: HwClient) -> None:
    first = drive.latest_heartbeat()
    second = drive.latest_heartbeat()
    assert 0 <= second.uptime - first.uptime <= 2
    assert first.health.value in (0, 1, 2)


def test_port_list_names_the_application_subjects(drive: HwClient) -> None:
    message = drive.next_message(drive.port_list, timeout=15.0)
    publishers = {int(item.value) for item in message.publishers.sparse_list}
    subscribers = {int(item.value) for item in message.subscribers.sparse_list}
    assert {7509, 212, 214, 215} <= publishers
    assert 1024 + drive.drive in subscribers  # the readiness follows the node-ID


# --- Registers ---------------------------------------------------------------------------

def test_register_list_has_the_settings(drive: HwClient) -> None:
    names = set(drive.register_names())
    assert EXPECTED_REGISTERS <= names, sorted(EXPECTED_REGISTERS - names)
    assert len(names) > 100


def test_register_write_reads_back_and_is_restored(clean_drive: HwClient) -> None:
    original = clean_drive.read("unimoc.control.current.kp_d")
    try:
        assert clean_drive.write("unimoc.control.current.kp_d", real(0.123)) == pytest.approx(0.123, rel=1e-5)
        assert clean_drive.read("unimoc.control.current.kp_d") == pytest.approx(0.123, rel=1e-5)
    finally:
        clean_drive.write("unimoc.control.current.kp_d", real(original))
    assert clean_drive.read("unimoc.control.current.kp_d") == pytest.approx(original)
    assert clean_drive.read("unimoc.settings.pending") in (False, 0)


def test_bus_has_no_bus_off(drive: HwClient) -> None:
    assert drive.read("unimoc.can.bus_off_count") == 0


# --- Subjects ----------------------------------------------------------------------------

def test_publish_rates(drive: HwClient) -> None:
    seconds = 4.0
    telemetry_rate = float(drive.read("unimoc.telemetry.rate"))
    status = drive.count_messages(unimoc.cart.DriveStatus_0_1, PORT_DRIVE_STATUS, seconds) / seconds
    telemetry = drive.count_messages(unimoc.drive.Telemetry_0_1, PORT_TELEMETRY, seconds) / seconds
    commissioning = drive.count_messages(unimoc.drive.CommissioningStatus_0_1, PORT_COMMISSIONING, seconds)
    assert status == pytest.approx(float(drive.read("unimoc.cart.status_rate")), rel=0.25)
    assert telemetry == pytest.approx(telemetry_rate, rel=0.25)
    assert commissioning >= 1  # 0.5 Hz while idle


# --- Commands ----------------------------------------------------------------------------

def test_unknown_command_is_refused(drive: HwClient) -> None:
    assert drive.command(999) == STATUS_BAD_COMMAND


def test_reset_timing_succeeds(drive: HwClient) -> None:
    assert drive.command(CMD_RESET_TIMING) == STATUS_SUCCESS


# --- Traces (cymon) ----------------------------------------------------------------------

def test_trace_captures_samples(drive: HwClient) -> None:
    variables = [name for name, _unit in drive.trace_variables()]
    assert {"i_d", "i_q", "dc_link_voltage", "i_a", "i_b", "i_c"} <= set(variables)
    info = drive.trace_buffer_info()
    assert info.max_channels >= 4 and info.tick_period_us > 0
    channels = [variables.index(name) for name in ("i_a", "i_b", "i_c", "dc_link_voltage")]
    assert drive.trace_setup(channels, info.tick_period_us, 64, 8).result.value == 0
    assert drive.trace_arm(mode=3) == 0  # IMMEDIATE
    deadline = time.monotonic() + 5.0
    status = drive.trace_read()
    while status.state.value != cymon.CaptureState_0_1.COMPLETE and time.monotonic() < deadline:
        time.sleep(0.05)
        status = drive.trace_read()
    assert status.state.value == cymon.CaptureState_0_1.COMPLETE
    page = drive.trace_read(0, 8)
    assert status.num_channels == 4 and status.num_samples == 64 and len(page.samples) == 8 * 4


# --- Emergency stop and restart ----------------------------------------------------------

def test_emergency_stop_latches_until_a_restart(drive: HwClient) -> None:
    assert drive.command(CMD_EMERGENCY_STOP) == STATUS_SUCCESS
    heartbeat = drive.wait_for_status(lambda beat: beat.vendor_specific_status_code & VENDOR_EMERGENCY_STOP)
    assert heartbeat.health.value == HEALTH_CAUTION
    drive.hold_readiness(ENGAGED)
    drive.sleep(2.5)  # long enough for the status to show an engage, if one happened
    refused = not drive.vendor_status() & VENDOR_ENGAGED
    drive.stop_readiness()
    assert refused, "an engage after the emergency stop must be refused"
    assert drive.command(CMD_RESTART) == STATUS_SUCCESS
    drive.wait_for_restart()
    assert not drive.vendor_status() & VENDOR_EMERGENCY_STOP


# --- Persistence ------------------------------------------------------------------------

def test_settings_and_node_id_survive_a_restart(clean_drive: HwClient) -> None:
    drive = clean_drive
    node_id = drive.drive
    original = drive.read("unimoc.motor.pole_pairs")
    changed = 9 if original != 9 else 7
    try:
        assert drive.write("unimoc.motor.pole_pairs", natural8(changed)) == changed
        assert drive.command(CMD_STORE_PERSISTENT_STATES) == STATUS_SUCCESS
        assert drive.command(CMD_RESTART) == STATUS_SUCCESS
        assert drive.wait_for_restart() == node_id, "the stored node-ID comes back without a new allocation"
        assert drive.read("uavcan.node.id") == node_id
        assert drive.read("unimoc.motor.pole_pairs") == changed
    finally:
        drive.write("unimoc.motor.pole_pairs", natural8(original))
        drive.command(CMD_STORE_PERSISTENT_STATES)
    assert drive.read("unimoc.motor.pole_pairs") == original


def test_unsaved_setting_is_gone_after_a_restart(clean_drive: HwClient) -> None:
    drive = clean_drive
    original = drive.read("unimoc.motor.pole_pairs")
    drive.write("unimoc.motor.pole_pairs", natural8(9 if original != 9 else 7))
    assert drive.command(CMD_RESTART) == STATUS_SUCCESS
    drive.wait_for_restart()
    assert drive.read("unimoc.motor.pole_pairs") == original


# --- Firmware update ---------------------------------------------------------------------

@pytest.mark.no_flash_bank
def test_firmware_update_is_refused_without_a_bank(drive: HwClient) -> None:
    """The H7 boards have no flash writer and bootloader yet (README.md, Cyphal status): a refusal, nothing erased."""
    assert drive.command(CMD_BEGIN_SOFTWARE_UPDATE, "image.bin") == STATUS_FAILURE
    assert drive.read("unimoc.update.state") == 0
    assert drive.read("unimoc.update.received") == 0


# --- Engage ------------------------------------------------------------------------------

@pytest.mark.engage
def test_engage_enables_the_power_stage_and_a_lost_master_disengages(clean_drive: HwClient) -> None:
    drive = clean_drive
    mark = drive.record_mark()
    drive.hold_readiness(ENGAGED, torque=0.0)
    try:
        drive.wait_for_record("engaged", mark, timeout=5.0)
        beat = drive.wait_for_status(lambda b: b.vendor_specific_status_code & VENDOR_ENGAGED)
        status = beat.vendor_specific_status_code
        assert status & 0x07 == POWER_STAGE_ENABLED
        assert not status & VENDOR_TRIP_LATCHED
    finally:
        drive.stop_readiness()
    drive.wait_for_record("disengaged: readiness timeout", mark, timeout=5.0)
    beat = drive.wait_for_status(lambda b: not b.vendor_specific_status_code & VENDOR_ENGAGED)
    assert beat.vendor_specific_status_code & 0x07 == POWER_STAGE_DISABLED


# --- Factory reset (last: the node-ID is gone) -------------------------------------------

@pytest.mark.factory_reset
def test_factory_reset_restores_defaults_and_clears_the_node_id(clean_drive: HwClient) -> None:
    drive = clean_drive
    original = drive.read("unimoc.motor.pole_pairs")
    drive.write("unimoc.motor.pole_pairs", natural8(9 if original != 9 else 7))
    drive.write("unimoc.control.current.kp_d", real(0.2))
    assert drive.command(CMD_STORE_PERSISTENT_STATES) == STATUS_SUCCESS
    assert drive.command(CMD_FACTORY_RESET) == STATUS_SUCCESS
    assert drive.read("unimoc.motor.pole_pairs") == 15  # the factory motor
    assert drive.read("unimoc.control.current.kp_d") == pytest.approx(0.7, rel=1e-5)
    assert drive.read("uavcan.node.id") == 0
    assert drive.command(CMD_RESTART) == STATUS_SUCCESS
    new_id = drive.wait_for_restart()  # plug-and-play allocates again
    assert new_id != LOCAL
    acknowledge(drive)
    # The new allocation is saved: it survives another restart without the allocator.
    assert drive.command(CMD_RESTART) == STATUS_SUCCESS
    assert drive.wait_for_restart() == new_id
