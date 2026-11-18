# SPDX-License-Identifier: GPL-3.0-or-later
#
# Bring-up of the Nucleo adapter for the legacy 4850 power board (README.md,
# "Nucleo adapter"; docs/zephyr_workflow.md). Read-only: the tests read the
# raw ADC counts, the external enable and the Hall code from the bench
# registers of the STM32H7 platform (unimoc.h7.*) and check them against the
# stage of the bring-up given by --adapter-stage. Nothing is engaged.
#
#   logic  the Nucleo on the adapter, no power board: Vdc reads zero, the
#          NTC inputs read open (pull-up), the Hall inputs read 7 without
#          sensors. The phase current inputs float and are not checked.
#   power  the power board attached, the gate drivers on J601, the DC link
#          from a current-limited bench supply (--dc-link-volts, read from a
#          meter): the INA240 zero offsets and their noise, Vdc, the bridge
#          NTC, the limits of the bench build (boards/variants/nucleo_bench_12v.overlay).
#
# Run with -s to see the table of the readings.

from __future__ import annotations

import math
import statistics

import pytest

from hw_client import POWER_STAGE_BLOCKED, POWER_STAGE_MASK, VENDOR_TRIP_LATCHED, HwClient

pytestmark = pytest.mark.bench

# Analog front end of apps/drive/boards/nucleo_h743zi.overlay (foc0) and of the adapter.
ADC_FULL_SCALE = 4095
ADC_REFERENCE_V = 3.3
CURRENT_ZERO_COUNTS = 2048
SHUNT_OHM = 666667e-9  # three 2 mOhm shunts in parallel per phase
AMPLIFIER_GAIN = 20  # INA240A1
DIVIDER_RATIO = (47000 + 2200) / 2200
NTC_PULLUP_OHM, NTC_R0_OHM, NTC_BETA, NTC_T0_K = 10000.0, 10000.0, 3950.0, 298.15

# Limits of the checks.
SAMPLES = 40
CURRENT_OFFSET_TOLERANCE_COUNTS = 25  # about 1.5 A; the startup aid calibrates the rest
CURRENT_NOISE_LIMIT_COUNTS = 4.0  # standard deviation of single reads
OPEN_INPUT_COUNTS = 4000  # an NTC input with nothing on it sits at the pull-up
VDC_TOLERANCE = 0.03  # relative, plus one count of the divider
BRIDGE_TEMPERATURE_RANGE_C = (5.0, 60.0)
BENCH_TRIP_VOLTAGE_MAX = 16.0  # dc-link-millivolts of the bench overlay
BENCH_MOTOR_CURRENT_MAX = 10.0  # max-motor-current-amps of the bench overlay

CURRENTS = ("ia", "ib", "ic")
CHANNELS = (*CURRENTS, "va", "vb", "vc", "vdc", "bridge_temperature", "motor_temperature")


def counts_to_pin_volts(counts: float) -> float:
    return counts * ADC_REFERENCE_V / (ADC_FULL_SCALE + 1)


def counts_to_amps(counts: float) -> float:
    return counts_to_pin_volts(counts - CURRENT_ZERO_COUNTS) / (SHUNT_OHM * AMPLIFIER_GAIN)


def counts_to_divider_volts(counts: float) -> float:
    return counts_to_pin_volts(counts) * DIVIDER_RATIO


def counts_to_celsius(counts: float) -> float:
    volts = counts_to_pin_volts(counts)
    resistance = NTC_PULLUP_OHM * volts / (ADC_REFERENCE_V - volts)
    return 1.0 / (1.0 / NTC_T0_K + math.log(resistance / NTC_R0_OHM) / NTC_BETA) - 273.15


@pytest.fixture(scope="module")
def stage(request) -> str:
    value = request.config.getoption("--adapter-stage")
    if value is None:
        pytest.skip("Nucleo adapter bring-up: pass --adapter-stage logic|power (README.md)")
    return value


@pytest.fixture(scope="module")
def readings(drive: HwClient, stage: str) -> dict[str, list[int]]:
    """SAMPLES single reads of every channel, printed as a table."""
    names = set(drive.register_names())
    missing = [f"unimoc.h7.adc.{channel}" for channel in CHANNELS if f"unimoc.h7.adc.{channel}" not in names]
    assert not missing, f"the firmware has no bench registers (STM32H7 build from this branch?): {missing}"
    samples = {channel: [int(drive.read(f"unimoc.h7.adc.{channel}")) for _ in range(SAMPLES)] for channel in CHANNELS}
    print(f"\nNucleo adapter, stage {stage}: {SAMPLES} reads per channel")
    print(f"{'channel':<20}{'mean':>8}{'stdev':>8}{'min':>6}{'max':>6}  value")
    for channel, values in samples.items():
        mean = statistics.fmean(values)
        if channel in CURRENTS:
            value = f"{counts_to_amps(mean):+.2f} A ({counts_to_pin_volts(mean):.3f} V)"
        elif channel.endswith("temperature"):
            value = "open" if mean >= OPEN_INPUT_COUNTS else f"{counts_to_celsius(mean):.1f} degC"
        else:
            value = f"{counts_to_divider_volts(mean):.2f} V"
        print(f"{channel:<20}{mean:8.1f}{statistics.pstdev(values):8.2f}{min(values):6d}{max(values):6d}  {value}")
    print(f"external enable {drive.read('unimoc.h7.external_enable')}, Hall code {drive.read('unimoc.h7.hall_code')}, "
          f"vendor status {drive.vendor_status():#04x}")
    return samples


def test_external_enable_matches_the_power_stage(drive: HwClient, stage: str) -> None:
    released = bool(drive.read("unimoc.h7.external_enable"))
    blocked = drive.vendor_status() & POWER_STAGE_MASK == POWER_STAGE_BLOCKED
    assert released != blocked, (
        f"J401 reads {'closed' if released else 'open'}, but the power stage is "
        f"{'blocked' if blocked else 'not blocked'}")


def test_hall_code(drive: HwClient, stage: str, request) -> None:
    code = drive.read("unimoc.h7.hall_code")
    if request.config.getoption("--halls"):
        assert 1 <= code <= 6, f"Hall code {code}: with sensors connected only 1 to 6 are valid"
    else:
        assert code == 7, f"Hall code {code}: without sensors the 2k2 pull-ups give 7"


def test_dc_link(readings: dict[str, list[int]], stage: str, request) -> None:
    volts = counts_to_divider_volts(statistics.fmean(readings["vdc"]))
    if stage == "logic":
        assert volts < 0.5, f"Vdc reads {volts:.2f} V without a power board"
        return
    expected = request.config.getoption("--dc-link-volts")
    if expected is None:
        pytest.skip("pass --dc-link-volts with the meter reading of the DC link")
    tolerance = expected * VDC_TOLERANCE + counts_to_divider_volts(1)
    assert volts == pytest.approx(expected, abs=tolerance), f"Vdc reads {volts:.2f} V, the meter {expected:.2f} V"


def test_ntc_inputs(readings: dict[str, list[int]], stage: str) -> None:
    bridge = statistics.fmean(readings["bridge_temperature"])
    if stage == "logic":
        assert bridge >= OPEN_INPUT_COUNTS, f"bridge NTC input reads {bridge:.0f} counts without a power board"
        return
    assert bridge < OPEN_INPUT_COUNTS, "bridge NTC (power board TH101) reads open"
    celsius = counts_to_celsius(bridge)
    low, high = BRIDGE_TEMPERATURE_RANGE_C
    assert low <= celsius <= high, f"bridge temperature {celsius:.1f} degC on the bench"


def test_current_offsets(readings: dict[str, list[int]], stage: str) -> None:
    if stage == "logic":
        pytest.skip("no INA240 without the power board: the current inputs float")
    for channel in CURRENTS:
        mean = statistics.fmean(readings[channel])
        assert abs(mean - CURRENT_ZERO_COUNTS) <= CURRENT_OFFSET_TOLERANCE_COUNTS, (
            f"{channel} sits at {mean:.1f} counts ({counts_to_amps(mean):+.2f} A), the INA240 zero is "
            f"{CURRENT_ZERO_COUNTS}: check +3.3VA, REF1/REF2 and the J102 pinout")


def test_current_noise(readings: dict[str, list[int]], stage: str) -> None:
    if stage == "logic":
        pytest.skip("no INA240 without the power board: the current inputs float")
    for channel in CURRENTS:
        noise = statistics.pstdev(readings[channel])
        assert noise <= CURRENT_NOISE_LIMIT_COUNTS, (
            f"{channel} noise {noise:.2f} counts ({counts_to_amps(CURRENT_ZERO_COUNTS + noise):.2f} A rms): "
            "check GNDA (FB602) and the filters")


def test_no_trip_with_the_power_board(drive: HwClient, readings: dict[str, list[int]], stage: str) -> None:
    if stage == "logic":
        pytest.skip("the floating current inputs latch a trip without the power board")
    assert not drive.vendor_status() & VENDOR_TRIP_LATCHED, "a trip is latched with the outputs off"


def test_bench_build_limits(drive: HwClient, stage: str) -> None:
    if stage == "logic":
        pytest.skip("the limits matter once the power board is attached")
    trip = drive.read("unimoc.battery.limits.trip_voltage")
    current = drive.read("unimoc.motor.limits.i_max")
    assert trip <= BENCH_TRIP_VOLTAGE_MAX and current <= BENCH_MOTOR_CURRENT_MAX, (
        f"DC-link trip {trip:.1f} V, motor current {current:.1f} A: flash the bench build "
        "(boards/variants/nucleo_bench_12v.overlay) for the first power-on")
