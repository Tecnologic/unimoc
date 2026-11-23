/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file pedal_map_test.cpp
 *    @brief ztest coverage of the pedal map: pedal channels, brake first, gears, release, speed limit and
 * ramp.
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
constexpr Time kStep = 0.1_s;
constexpr Velocity kCruise = 5.0_m_per_s;

/// Reverse 2 m/s; first gear 4 m/s with a 1 m/s^2 ramp; second 10 m/s and third 20 m/s without a ramp.
PedalMapParameters TestParameters() {
  PedalMapParameters parameters;
  parameters.gears[GearIndex(kReverse)] = {2.0_m_per_s, 0.0_m_per_s2};
  parameters.gears[GearIndex(1)] = {4.0_m_per_s, 1.0_m_per_s2};
  parameters.gears[GearIndex(2)] = {10.0_m_per_s, 0.0_m_per_s2};
  parameters.gears[GearIndex(3)] = {20.0_m_per_s, 0.0_m_per_s2};
  parameters.drive_torque = 40.0_Nm;
  parameters.brake_torque = 50.0_Nm;
  parameters.dead_band = 0.0_ratio;
  parameters.standstill_speed = 1.0_m_per_s;
  parameters.ramp_lead = 0.5_m_per_s;
  return parameters;
}

PedalChannels Pedal(const Ratio first, const Ratio second, const bool plausible = true) {
  return PedalChannels{{first, second}, plausible};
}

Pedals Drive(const Ratio travel) { return Pedals{Pedal(travel, travel), Pedal(0.0_ratio, 0.0_ratio)}; }

Pedals Brake(const Ratio travel) { return Pedals{Pedal(0.0_ratio, 0.0_ratio), Pedal(travel, travel)}; }

/// A map in @p gear with the forward gears released up to @p release, taken over at standstill.
PedalMap MakeMap(const Gear gear, const Gear release = kTopGear) {
  PedalMap map;
  map.Configure(TestParameters());
  map.SetRelease(release);
  map.RequestGear(gear);
  (void)map.Update(Drive(0.0_ratio), 0.0_m_per_s, kStep);
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

ZTEST(pedal_map, test_drive_torque_is_the_same_in_every_gear) {
  PedalMap second = MakeMap(2);
  UNIMOC_EXPECT_NEAR(second.Update(Drive(0.5_ratio), 0.0_m_per_s, kStep), 20.0_Nm, kTolerance);
  PedalMap third = MakeMap(3);
  UNIMOC_EXPECT_NEAR(third.Update(Drive(0.5_ratio), 0.0_m_per_s, kStep), 20.0_Nm, kTolerance);
}

ZTEST(pedal_map, test_neutral_has_no_drive_torque) {
  PedalMap map = MakeMap(kNeutral);
  UNIMOC_EXPECT_NEAR(map.Update(Drive(1.0_ratio), 0.0_m_per_s, kStep), 0.0_Nm, kTolerance);
}

ZTEST(pedal_map, test_reverse_drives_backward_up_to_its_limit) {
  PedalMap map = MakeMap(kReverse);  // 2 m/s, band 0.2 m/s
  UNIMOC_EXPECT_NEAR(map.Update(Drive(0.5_ratio), 0.0_m_per_s, kStep), -20.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(map.Update(Drive(1.0_ratio), -1.9_m_per_s, kStep), -20.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(map.Update(Drive(1.0_ratio), -2.0_m_per_s, kStep), 0.0_Nm, kTolerance);
}

ZTEST(pedal_map, test_brake_has_priority_and_opposes_the_motion) {
  PedalMap map = MakeMap(2);
  const Pedals kBoth{Pedal(1.0_ratio, 1.0_ratio), Pedal(0.5_ratio, 0.5_ratio)};
  UNIMOC_EXPECT_NEAR(map.Update(kBoth, kCruise, kStep), -25.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(map.Update(Brake(0.5_ratio), -1.5_m_per_s, kStep), 25.0_Nm, kTolerance);
}

ZTEST(pedal_map, test_brake_fades_out_at_standstill_and_never_drives_the_other_way) {
  PedalMap map = MakeMap(2);
  UNIMOC_EXPECT_NEAR(map.Update(Brake(1.0_ratio), 0.5_m_per_s, kStep), -25.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(map.Update(Brake(1.0_ratio), 0.0_m_per_s, kStep), 0.0_Nm, kTolerance);
}

ZTEST(pedal_map, test_speed_limit_fades_the_drive_torque_and_brakes_above_it) {
  PedalMap map = MakeMap(2);  // 10 m/s without a ramp, band 1 m/s
  UNIMOC_EXPECT_NEAR(map.Update(Drive(1.0_ratio), 8.0_m_per_s, kStep), 40.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(map.Update(Drive(1.0_ratio), 9.5_m_per_s, kStep), 20.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(map.Update(Drive(1.0_ratio), 10.0_m_per_s, kStep), 0.0_Nm, kTolerance);
  // Downhill above the limit with the pedals released: regenerative braking.
  UNIMOC_EXPECT_NEAR(map.Update(Drive(0.0_ratio), 10.5_m_per_s, kStep), -20.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(map.Update(Drive(0.0_ratio), 20.0_m_per_s, kStep), -40.0_Nm, kTolerance);
  // The brake pedal brakes harder than the limit.
  UNIMOC_EXPECT_NEAR(map.Update(Brake(1.0_ratio), 20.0_m_per_s, kStep), -50.0_Nm, kTolerance);
}

ZTEST(pedal_map, test_speed_ramp_starts_gently_but_keeps_the_full_torque_uphill) {
  PedalMap map = MakeMap(1);  // 4 m/s, 1 m/s^2, lead 0.5 m/s, band 0.4 m/s
  // Flat start: the limit rises by 0.1 m/s per step, a quarter of the band.
  UNIMOC_EXPECT_NEAR(map.Update(Drive(1.0_ratio), 0.0_m_per_s, kStep), 10.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(map.RampedLimit(), 0.1_m_per_s, 1.0e-5_m_per_s);
  // Uphill the cart does not move: the limit runs up to the lead ahead and the full torque follows.
  for (int i = 0; i < 10; ++i) {
    (void)map.Update(Drive(1.0_ratio), 0.0_m_per_s, kStep);
  }
  UNIMOC_EXPECT_NEAR(map.RampedLimit(), 0.6_m_per_s, 1.0e-5_m_per_s);
  UNIMOC_EXPECT_NEAR(map.Request(), 40.0_Nm, kTolerance);
}

ZTEST(pedal_map, test_speed_ramp_restarts_from_the_speed_of_the_cart_and_stops_at_the_gear_limit) {
  PedalMap map = MakeMap(1);
  (void)map.Update(Drive(0.0_ratio), 2.0_m_per_s, kStep);
  UNIMOC_EXPECT_NEAR(map.RampedLimit(), 2.0_m_per_s, 1.0e-5_m_per_s);
  UNIMOC_EXPECT_NEAR(map.Update(Drive(1.0_ratio), 2.0_m_per_s, kStep), 10.0_Nm, kTolerance);
  for (int i = 0; i < 40; ++i) {
    (void)map.Update(Drive(1.0_ratio), 3.9_m_per_s, kStep);
  }
  UNIMOC_EXPECT_NEAR(map.RampedLimit(), 4.0_m_per_s, 1.0e-5_m_per_s);
  UNIMOC_EXPECT_NEAR(map.Request(), 10.0_Nm, kTolerance);
}

ZTEST(pedal_map, test_gears_above_the_release_are_lowered) {
  PedalMap map = MakeMap(1, 1);
  map.RequestGear(3);
  (void)map.Update(Drive(0.0_ratio), 0.0_m_per_s, kStep);
  zexpect_equal(map.RequestedGear(), 3);
  zexpect_equal(map.ActiveGear(), 1);
  map.SetRelease(3);
  (void)map.Update(Drive(0.0_ratio), 0.0_m_per_s, kStep);
  zexpect_equal(map.ActiveGear(), 3);
  // Withdrawing the release while fast lowers the gear at once; the lower limit brakes.
  map.SetRelease(1);
  zexpect_equal(map.ActiveGear(), 1);
  UNIMOC_EXPECT_NEAR(map.Update(Drive(1.0_ratio), 10.0_m_per_s, kStep), -40.0_Nm, kTolerance);
}

ZTEST(pedal_map, test_other_direction_waits_in_neutral_for_standstill) {
  PedalMap map = MakeMap(2);
  map.RequestGear(kReverse);
  UNIMOC_EXPECT_NEAR(map.Update(Drive(1.0_ratio), kCruise, kStep), 0.0_Nm, kTolerance);
  zexpect_equal(map.ActiveGear(), kNeutral);
  // Still rolling: still neutral, also in the next cycles.
  UNIMOC_EXPECT_NEAR(map.Update(Drive(1.0_ratio), kCruise, kStep), 0.0_Nm, kTolerance);
  zexpect_equal(map.ActiveGear(), kNeutral);
  UNIMOC_EXPECT_NEAR(map.Update(Drive(1.0_ratio), 0.0_m_per_s, kStep), -40.0_Nm, kTolerance);
  zexpect_equal(map.ActiveGear(), kReverse);
}

ZTEST(pedal_map, test_neutral_holds_the_released_limit_by_braking) {
  PedalMap map = MakeMap(kNeutral, 1);  // first gear 4 m/s, band 0.4 m/s
  UNIMOC_EXPECT_NEAR(map.Update(Drive(0.0_ratio), 3.0_m_per_s, kStep), 0.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(map.Update(Drive(0.0_ratio), 5.0_m_per_s, kStep), -40.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(map.Update(Drive(0.0_ratio), -3.0_m_per_s, kStep), 40.0_Nm, kTolerance);
}

ZTEST(pedal_map, test_lost_pedals_brake_gently) {
  PedalMap map = MakeMap(2);
  UNIMOC_EXPECT_NEAR(map.UpdateLost(kCruise, kStep), -10.0_Nm, kTolerance);  // 0.2 of 50 N m
  UNIMOC_EXPECT_NEAR(map.UpdateLost(0.0_m_per_s, kStep), 0.0_Nm, kTolerance);
}

ZTEST(pedal_map, test_gear_numbers) {
  zexpect_true(IsGear(-1) && IsGear(0) && IsGear(3));
  zexpect_false(IsGear(-2) || IsGear(4));
  zexpect_equal(GearIndex(kReverse), 0U);
  zexpect_equal(GearIndex(2), 2U);
  zexpect_equal(Direction(kReverse), -1);
  zexpect_equal(Direction(kNeutral), 0);
  zexpect_equal(Direction(3), 1);
}

ZTEST_SUITE(pedal_map, nullptr, nullptr, nullptr, nullptr, nullptr);

}  // namespace unimoc::vehicle::test
