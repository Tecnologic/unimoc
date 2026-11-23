/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file pedal_map_test.cpp
 *    @brief ztest coverage of the pedal map: pedal channels, brake first, speed limit, soft start, drive
 * modes.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "pedal_map.hpp"
#include <zephyr/ztest.h>
#include <limits>
#include "unimoc_test.hpp"

namespace unimoc::vehicle::test {

using namespace unit;

namespace {

constexpr Torque kTolerance = 1.0e-3_Nm;
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
/// Long enough for every ramp of the test parameters to settle in one cycle.
constexpr Time kLongStep = 10.0_s;
constexpr AngularVelocity kCruise = 5.0_rad_per_s;

PedalMapParameters TestParameters() {
  PedalMapParameters parameters;
  parameters.modes[ModeIndex(DriveMode::kChild)] = {10.0_rad_per_s, 10.0_Nm, 5.0_Nm / 1.0_s};
  parameters.modes[ModeIndex(DriveMode::kAdult)] = {30.0_rad_per_s, 40.0_Nm, 100.0_Nm / 1.0_s};
  parameters.brake_torque = 50.0_Nm;
  parameters.dead_band = 0.0_ratio;
  return parameters;
}

PedalChannels Pedal(const Ratio first, const Ratio second, const bool plausible = true) {
  return PedalChannels{{first, second}, plausible};
}

Pedals Drive(const Ratio travel) { return Pedals{Pedal(travel, travel), Pedal(0.0_ratio, 0.0_ratio)}; }

Pedals Brake(const Ratio travel) { return Pedals{Pedal(0.0_ratio, 0.0_ratio), Pedal(travel, travel)}; }

PedalMap MakeMap(const DriveMode mode = DriveMode::kChild) {
  PedalMap map;
  map.Configure(TestParameters());
  map.RequestMode(mode);
  (void)map.Update(Drive(0.0_ratio), 0.0_rad_per_s, kLongStep);  // standstill: the mode applies
  return map;
}

}  // namespace

ZTEST(pedal_map, test_drive_pedal_takes_the_smaller_channel) {
  UNIMOC_EXPECT_FLOAT_EQ(DriveTravel(Pedal(0.4_ratio, 0.6_ratio)), 0.4_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(DriveTravel(Pedal(1.5_ratio, 1.2_ratio)), 1.0_ratio);
}

ZTEST(pedal_map, test_implausible_or_failed_drive_pedal_is_released) {
  UNIMOC_EXPECT_FLOAT_EQ(DriveTravel(Pedal(0.8_ratio, 0.8_ratio, false)), 0.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(DriveTravel(Pedal(Ratio{kNaN}, 0.8_ratio)), 0.0_ratio);
}

ZTEST(pedal_map, test_brake_pedal_takes_the_larger_valid_channel_even_when_implausible) {
  UNIMOC_EXPECT_FLOAT_EQ(BrakeTravel(Pedal(0.3_ratio, 0.7_ratio)), 0.7_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(BrakeTravel(Pedal(0.3_ratio, 0.7_ratio, false)), 0.7_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(BrakeTravel(Pedal(Ratio{kNaN}, 0.5_ratio, false)), 0.5_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(BrakeTravel(Pedal(Ratio{kNaN}, Ratio{kNaN}, false)), 0.0_ratio);
}

ZTEST(pedal_map, test_dead_band_is_rescaled) {
  UNIMOC_EXPECT_FLOAT_EQ(PastDeadBand(0.05_ratio, 0.1_ratio), 0.0_ratio);
  UNIMOC_EXPECT_NEAR(PastDeadBand(0.55_ratio, 0.1_ratio), 0.5_ratio, 1.0e-6_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(PastDeadBand(1.0_ratio, 0.1_ratio), 1.0_ratio);
}

ZTEST(pedal_map, test_drive_pedal_scales_the_mode_torque) {
  PedalMap child = MakeMap(DriveMode::kChild);
  UNIMOC_EXPECT_NEAR(child.Update(Drive(0.5_ratio), 0.0_rad_per_s, kLongStep), 5.0_Nm, kTolerance);
  PedalMap adult = MakeMap(DriveMode::kAdult);
  UNIMOC_EXPECT_NEAR(adult.Update(Drive(0.5_ratio), 0.0_rad_per_s, kLongStep), 20.0_Nm, kTolerance);
}

ZTEST(pedal_map, test_brake_has_priority_and_opposes_the_motion) {
  PedalMap map = MakeMap();
  const Pedals kBoth{Pedal(1.0_ratio, 1.0_ratio), Pedal(0.5_ratio, 0.5_ratio)};
  UNIMOC_EXPECT_NEAR(map.Update(kBoth, kCruise, kLongStep), -25.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(map.Update(Brake(0.5_ratio), -kCruise, kLongStep), 25.0_Nm, kTolerance);
}

ZTEST(pedal_map, test_brake_fades_out_at_standstill_and_never_drives_backward) {
  PedalMap map = MakeMap();
  UNIMOC_EXPECT_NEAR(map.Update(Brake(1.0_ratio), 0.5_rad_per_s, kLongStep), -25.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(map.Update(Brake(1.0_ratio), 0.0_rad_per_s, kLongStep), 0.0_Nm, kTolerance);
}

ZTEST(pedal_map, test_speed_limit_fades_the_drive_torque_and_brakes_above_it) {
  PedalMap map = MakeMap();  // child: 10 rad/s, band 1 rad/s, 10 N m
  UNIMOC_EXPECT_NEAR(map.Update(Drive(1.0_ratio), 8.0_rad_per_s, kLongStep), 10.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(map.Update(Drive(1.0_ratio), 9.5_rad_per_s, kLongStep), 5.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(map.Update(Drive(1.0_ratio), 10.0_rad_per_s, kLongStep), 0.0_Nm, kTolerance);
  // Downhill above the limit with the pedals released: regenerative braking.
  UNIMOC_EXPECT_NEAR(map.Update(Drive(0.0_ratio), 10.5_rad_per_s, kLongStep), -5.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(map.Update(Drive(0.0_ratio), 20.0_rad_per_s, kLongStep), -10.0_Nm, kTolerance);
  // The brake pedal brakes harder than the limit.
  UNIMOC_EXPECT_NEAR(map.Update(Brake(1.0_ratio), 20.0_rad_per_s, kLongStep), -50.0_Nm, kTolerance);
}

ZTEST(pedal_map, test_soft_start_ramps_only_a_rising_drive_torque) {
  PedalMap map = MakeMap();  // child: 5 N m/s
  UNIMOC_EXPECT_NEAR(map.Update(Drive(1.0_ratio), 0.0_rad_per_s, 0.5_s), 2.5_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(map.Update(Drive(1.0_ratio), 0.0_rad_per_s, 0.5_s), 5.0_Nm, kTolerance);
  // Releasing and braking apply at once.
  UNIMOC_EXPECT_NEAR(map.Update(Drive(0.0_ratio), kCruise, 0.01_s), 0.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(map.Update(Brake(1.0_ratio), kCruise, 0.01_s), -50.0_Nm, kTolerance);
  // From braking the ramp starts at zero, not at the brake torque.
  UNIMOC_EXPECT_NEAR(map.Update(Drive(1.0_ratio), kCruise, 0.2_s), 1.0_Nm, kTolerance);
}

ZTEST(pedal_map, test_drive_mode_changes_only_at_standstill) {
  PedalMap map = MakeMap(DriveMode::kChild);
  map.RequestMode(DriveMode::kAdult);
  zexpect_equal(map.RequestedMode(), DriveMode::kAdult);
  UNIMOC_EXPECT_NEAR(map.Update(Drive(1.0_ratio), kCruise, kLongStep), 10.0_Nm, kTolerance);
  zexpect_equal(map.ActiveMode(), DriveMode::kChild);
  UNIMOC_EXPECT_NEAR(map.Update(Drive(1.0_ratio), 0.0_rad_per_s, kLongStep), 40.0_Nm, kTolerance);
  zexpect_equal(map.ActiveMode(), DriveMode::kAdult);
}

ZTEST(pedal_map, test_lost_pedals_brake_gently) {
  PedalMap map = MakeMap();
  UNIMOC_EXPECT_NEAR(map.UpdateLost(kCruise, kLongStep), -10.0_Nm, kTolerance);  // 0.2 of 50 N m
  UNIMOC_EXPECT_NEAR(map.UpdateLost(0.0_rad_per_s, kLongStep), 0.0_Nm, kTolerance);
}

ZTEST(pedal_map, test_mode_at_maps_register_numbers) {
  zexpect_equal(ModeAt(0U), DriveMode::kChild);
  zexpect_equal(ModeAt(1U), DriveMode::kAdult);
  zexpect_equal(ModeAt(7U), DriveMode::kChild);
}

ZTEST_SUITE(pedal_map, nullptr, nullptr, nullptr, nullptr, nullptr);

}  // namespace unimoc::vehicle::test
