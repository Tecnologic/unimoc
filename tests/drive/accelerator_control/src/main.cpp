/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file main.cpp
 *    @brief ztest coverage of the accelerator control (throttle, brake lever, ramps, faults, engage hold)
 *           and of the board input supervision (power board detect, gate supply).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <zephyr/ztest.h>
#include <cstdint>
#include <optional>
#include "accelerator_control.hpp"
#include "unimoc_test.hpp"

namespace {

using namespace unimoc::unit;
using unimoc::drive::AcceleratorControl;
using unimoc::drive::AcceleratorMode;
using unimoc::drive::AcceleratorOutput;
using unimoc::drive::AcceleratorParameters;
using unimoc::drive::AcceleratorVehicle;
using unimoc::drive::BoardSupervision;
using unimoc::drive::BoardVerdict;
using unimoc::hardware::BoardInputs;
using unimoc::hardware::GateSupplyLimits;

constexpr uint64_t kMs = 1000U;
/// The factory calibration: 0.8 V released, 2.6 V fully pressed.
constexpr Voltage kReleased = 0.8_V;

/// Sensor voltage of @p travel (0 to 1) of the factory calibration.
Voltage At(const float travel) { return kReleased + (1.8_V * travel); }

/// Torque at @p travel beyond the 5 % deadband, of @p full.
Torque Expected(const float travel, const Torque full) { return full * ((travel - 0.05F) / 0.95F); }

/// The control at 1 ms steps, with the vehicle of the test; it follows its own engage like the drive.
struct Rig {
  explicit Rig(const AcceleratorMode mode = AcceleratorMode::kThrottleAndBrake) {
    AcceleratorParameters parameters{};
    parameters.mode = mode;
    control.Configure(parameters);
    vehicle.torque_min = -100.0_Nm;
    vehicle.torque_max = 100.0_Nm;
  }

  /// Runs @p milliseconds with the inputs; returns the output of the last step.
  AcceleratorOutput Run(const uint32_t milliseconds,
                        const std::optional<Voltage> throttle,
                        const std::optional<Voltage> brake = kReleased) {
    AcceleratorOutput output{};
    for (uint32_t i = 0U; i < milliseconds; ++i) {
      now_us += kMs;
      output = control.Update(now_us, throttle, brake, vehicle);
      vehicle.engaged = output.engage && !vehicle.faulted;
    }
    return output;
  }

  AcceleratorControl control;
  AcceleratorVehicle vehicle;
  uint64_t now_us{0U};
};

}  // namespace

ZTEST_SUITE(accelerator_control, nullptr, nullptr, nullptr, nullptr, nullptr);

ZTEST(accelerator_control, test_off_never_asks_for_the_drive) {
  Rig rig{AcceleratorMode::kOff};
  zexpect_false(rig.control.Active());
  const AcceleratorOutput kOutput = rig.Run(100U, At(1.0F));
  zexpect_false(kOutput.engage);
  UNIMOC_EXPECT_FLOAT_EQ(kOutput.torque, 0.0_Nm);
}

ZTEST(accelerator_control, test_a_pressed_accelerator_after_the_configure_waits_for_the_release) {
  Rig rig;
  zexpect_true(rig.control.Active());
  zexpect_false(rig.Run(50U, At(0.5F)).engage, "pressed at the start: no request");
  zexpect_false(rig.Run(10U, kReleased).engage);
  zexpect_true(rig.Run(1U, At(0.5F)).engage, "pressed again after the release");
}

ZTEST(accelerator_control, test_the_torque_follows_the_travel_with_the_rise_time) {
  Rig rig;
  (void)rig.Run(10U, kReleased);
  // Full torque (10 N m) in 0.3 s: 50 ms rise 1.67 N m.
  const AcceleratorOutput kRising = rig.Run(50U, At(1.0F));
  zexpect_true(kRising.engage);
  UNIMOC_EXPECT_NEAR(kRising.torque, 10.0_Nm * (49.0F / 300.0F), 0.05_Nm);
  const AcceleratorOutput kHalf = rig.Run(500U, At(0.5F));
  UNIMOC_EXPECT_NEAR(kHalf.torque, Expected(0.5F, 10.0_Nm), 1e-3_Nm);
  zexpect_false(kHalf.sensor_fault);
}

ZTEST(accelerator_control, test_the_deadband_gives_no_torque) {
  Rig rig;
  (void)rig.Run(10U, kReleased);
  const AcceleratorOutput kOutput = rig.Run(100U, At(0.04F));
  zexpect_false(kOutput.engage);
  UNIMOC_EXPECT_FLOAT_EQ(kOutput.torque, 0.0_Nm);
}

ZTEST(accelerator_control, test_the_release_holds_the_drive_then_releases_it) {
  Rig rig;
  (void)rig.Run(10U, kReleased);
  (void)rig.Run(500U, At(1.0F));
  // Down to zero in at most 0.1 s, the drive stays engaged for the hold (2 s).
  const AcceleratorOutput kReleasedOutput = rig.Run(150U, kReleased);
  zexpect_true(kReleasedOutput.engage);
  UNIMOC_EXPECT_FLOAT_EQ(kReleasedOutput.torque, 0.0_Nm);
  zexpect_true(rig.Run(1800U, kReleased).engage);
  zexpect_false(rig.Run(100U, kReleased).engage, "after the hold");
}

ZTEST(accelerator_control, test_the_brake_lever_brakes_against_the_motion_and_wins) {
  Rig rig;
  (void)rig.Run(10U, kReleased);
  rig.vehicle.motor_speed = 50.0_rad_per_s;
  (void)rig.Run(500U, At(1.0F));
  // Both pressed: the brake wins, 5 N m at full travel against the forward motion.
  const AcceleratorOutput kForward = rig.Run(500U, At(1.0F), At(1.0F));
  zexpect_true(kForward.engage);
  UNIMOC_EXPECT_NEAR(kForward.torque, -Expected(1.0F, 5.0_Nm), 1e-3_Nm);
  rig.vehicle.motor_speed = -50.0_rad_per_s;
  const AcceleratorOutput kBackward = rig.Run(500U, kReleased, At(0.5F));
  UNIMOC_EXPECT_NEAR(kBackward.torque, Expected(0.5F, 5.0_Nm), 1e-3_Nm);
}

ZTEST(accelerator_control, test_the_brake_fades_out_towards_standstill) {
  Rig rig;
  (void)rig.Run(10U, kReleased);
  rig.vehicle.motor_speed = 5.0_rad_per_s;  // half the fade speed (10 rad/s)
  const AcceleratorOutput kSlow = rig.Run(500U, kReleased, At(1.0F));
  UNIMOC_EXPECT_NEAR(kSlow.torque, -0.5F * 5.0_Nm, 1e-3_Nm);
  rig.vehicle.motor_speed = AngularVelocity{};
  const AcceleratorOutput kStanding = rig.Run(500U, kReleased, At(1.0F));
  UNIMOC_EXPECT_FLOAT_EQ(kStanding.torque, 0.0_Nm);
}

ZTEST(accelerator_control, test_a_standing_motor_is_not_engaged_for_the_brake) {
  Rig rig;
  (void)rig.Run(10U, kReleased);
  zexpect_false(rig.Run(100U, kReleased, At(1.0F)).engage);
}

ZTEST(accelerator_control, test_without_the_brake_mode_the_lever_is_not_read) {
  Rig rig{AcceleratorMode::kThrottle};
  (void)rig.Run(10U, kReleased, std::nullopt);
  const AcceleratorOutput kOutput = rig.Run(500U, At(1.0F), std::nullopt);
  zexpect_false(kOutput.sensor_fault);
  UNIMOC_EXPECT_NEAR(kOutput.torque, 10.0_Nm, 1e-3_Nm);
}

ZTEST(accelerator_control, test_a_sensor_out_of_range_drops_the_torque_until_both_are_released) {
  Rig rig;
  (void)rig.Run(10U, kReleased);
  (void)rig.Run(500U, At(1.0F));
  // A broken wire: 0.2 V is below 0.8 V - 0.3 V.
  const AcceleratorOutput kBroken = rig.Run(150U, 0.2_V);
  zexpect_true(kBroken.sensor_fault);
  UNIMOC_EXPECT_FLOAT_EQ(kBroken.torque, 0.0_Nm);
  zexpect_false(kBroken.engage, "released once the torque is down");
  // The contact comes back with the accelerator pressed: still nothing.
  const AcceleratorOutput kBack = rig.Run(100U, At(1.0F));
  zexpect_true(kBack.sensor_fault);
  zexpect_false(kBack.engage);
  zexpect_false(rig.Run(1U, kReleased).sensor_fault);
  zexpect_true(rig.Run(1U, At(1.0F)).engage);
  zexpect_true(rig.Run(100U, std::nullopt).sensor_fault, "no input is a fault as well");
}

ZTEST(accelerator_control, test_a_fault_waits_for_the_release) {
  Rig rig;
  (void)rig.Run(10U, kReleased);
  (void)rig.Run(100U, At(1.0F));
  rig.vehicle.faulted = true;  // the drive tripped; it stays faulted until the control releases it
  zexpect_false(rig.Run(10U, At(1.0F)).engage);
  rig.vehicle.faulted = false;  // the drive re-armed on the missing request
  zexpect_false(rig.Run(10U, At(1.0F)).engage, "still pressed: no new request");
  (void)rig.Run(1U, kReleased);
  zexpect_true(rig.Run(1U, At(1.0F)).engage);
}

ZTEST(accelerator_control, test_inhibit_waits_for_the_release) {
  Rig rig;
  (void)rig.Run(10U, kReleased);
  (void)rig.Run(100U, At(1.0F));
  rig.control.Inhibit();
  (void)rig.Run(200U, At(1.0F));
  zexpect_false(rig.Run(1U, At(1.0F)).engage);
  (void)rig.Run(1U, kReleased);
  zexpect_true(rig.Run(1U, At(1.0F)).engage);
}

ZTEST(accelerator_control, test_the_torque_stays_in_the_window_of_the_drive) {
  Rig rig;
  rig.vehicle.torque_max = 3.0_Nm;
  (void)rig.Run(10U, kReleased);
  UNIMOC_EXPECT_NEAR(rig.Run(500U, At(1.0F)).torque, 3.0_Nm, 1e-3_Nm);
}

ZTEST(accelerator_control, test_the_parameters_come_from_the_settings) {
  unimoc::settings::NvmSettings settings{};
  settings.accel_mode = 1U;
  settings.accel_throttle_min = 1.0_V;
  settings.accel_torque_max = 7.0_Nm;
  const AcceleratorParameters kParameters = AcceleratorParameters::From(settings);
  zexpect_equal(static_cast<int>(kParameters.mode), static_cast<int>(AcceleratorMode::kThrottle));
  UNIMOC_EXPECT_FLOAT_EQ(kParameters.throttle_min, 1.0_V);
  UNIMOC_EXPECT_FLOAT_EQ(kParameters.torque_max, 7.0_Nm);
  settings.accel_mode = 9U;
  zexpect_equal(static_cast<int>(AcceleratorParameters::From(settings).mode),
                static_cast<int>(AcceleratorMode::kOff));
}

// ----- BoardSupervision ------------------------------------------------------------------------------

ZTEST_SUITE(board_supervision, nullptr, nullptr, nullptr, nullptr, nullptr);

namespace {

constexpr GateSupplyLimits kLimits{10.0_V, 10.5_V};

BoardInputs Jacobi(const bool power_board, const Voltage gate_supply) {
  return BoardInputs{power_board, gate_supply, kReleased, kReleased};
}

}  // namespace

ZTEST(board_supervision, test_a_board_without_the_inputs_is_ready) {
  BoardSupervision supervision;
  const BoardVerdict kVerdict = supervision.Update(0U, BoardInputs{}, kLimits, true);
  zexpect_true(kVerdict.ready);
  zexpect_false(kVerdict.trip);
  zexpect_true(kVerdict.reason.empty());
}

ZTEST(board_supervision, test_jacobi_with_power_board_and_gate_supply_is_ready) {
  BoardSupervision supervision;
  zexpect_true(supervision.Update(0U, Jacobi(true, 12.0_V), kLimits, false).ready);
}

ZTEST(board_supervision, test_without_the_power_board_it_is_not_ready) {
  BoardSupervision supervision;
  const BoardVerdict kVerdict = supervision.Update(0U, Jacobi(false, 12.0_V), kLimits, false);
  zexpect_false(kVerdict.ready);
  zexpect_false(kVerdict.trip, "not engaged");
  zexpect_true(kVerdict.reason == "no power board");
}

ZTEST(board_supervision, test_a_low_gate_supply_refuses_the_engage_only) {
  BoardSupervision supervision;
  const BoardVerdict kVerdict = supervision.Update(0U, Jacobi(true, 10.2_V), kLimits, true);
  zexpect_false(kVerdict.ready);
  zexpect_false(kVerdict.trip, "above the trip limit an engaged drive keeps running");
  zexpect_true(kVerdict.reason == "gate supply low");
}

ZTEST(board_supervision, test_an_undervoltage_trips_once_it_lasted) {
  BoardSupervision supervision;
  zexpect_false(supervision.Update(0U, Jacobi(true, 12.0_V), kLimits, true).trip);
  zexpect_false(supervision.Update(1U * kMs, Jacobi(true, 9.0_V), kLimits, true).trip);
  zexpect_false(supervision.Update(5U * kMs, Jacobi(true, 9.0_V), kLimits, true).trip, "4 ms");
  // One good conversion restarts the delay.
  zexpect_false(supervision.Update(6U * kMs, Jacobi(true, 12.0_V), kLimits, true).trip);
  zexpect_false(supervision.Update(7U * kMs, Jacobi(true, 9.0_V), kLimits, true).trip);
  const BoardVerdict kVerdict = supervision.Update(12U * kMs, Jacobi(true, 9.0_V), kLimits, true);
  zexpect_true(kVerdict.trip);
  zexpect_true(kVerdict.reason == "gate supply undervoltage");
}

ZTEST(board_supervision, test_the_lost_power_board_trips_an_engaged_drive) {
  BoardSupervision supervision;
  (void)supervision.Update(0U, Jacobi(false, 12.0_V), kLimits, true);
  zexpect_true(supervision.Update(BoardSupervision::kTripDelayUs, Jacobi(false, 12.0_V), kLimits, true).trip);
  zexpect_false(supervision
                    .Update(BoardSupervision::kTripDelayUs + kMs, Jacobi(false, 12.0_V), kLimits, false)
                    .trip,
                "a disengaged drive does not trip");
}
