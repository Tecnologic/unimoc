/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file dc_link_limit_test.cpp
 *    @brief Tests for the battery current limits and the regenerative derating.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "dc_link_limit.hpp"
#include <zephyr/ztest.h>
#include "unimoc_test.hpp"

namespace unimoc::control::test {

using namespace unit;

namespace {

constexpr DcLinkLimit kLimit{.drive_current_max = 10.0_A,
                             .charge_current_max = 4.0_A,
                             .regen_derate_start = 54.0_V,
                             .trip = 58.0_V};
constexpr MagneticFlux kTorqueConstant{0.05F};  // N m/A

system::Rotor<Current> Q(const float i_q) { return {Current{}, Current{i_q}}; }

}  // namespace

ZTEST(dc_link_limit, test_charge_limit_falls_linearly_to_zero_at_the_trip_level) {
  UNIMOC_EXPECT_FLOAT_EQ(kLimit.AllowedChargeCurrent(48.0_V).Value(), 4.0F);
  UNIMOC_EXPECT_FLOAT_EQ(kLimit.AllowedChargeCurrent(54.0_V).Value(), 4.0F);
  UNIMOC_EXPECT_NEAR(kLimit.AllowedChargeCurrent(56.0_V).Value(), 2.0F, 1e-5F);
  UNIMOC_EXPECT_FLOAT_EQ(kLimit.AllowedChargeCurrent(58.0_V).Value(), 0.0F);
  UNIMOC_EXPECT_FLOAT_EQ(kLimit.AllowedChargeCurrent(70.0_V).Value(), 0.0F);
}

ZTEST(dc_link_limit, test_inside_the_limits_the_current_is_unchanged) {
  // 20 A * 0.05 N m/A * 100 rad/s = 100 W, about 2 A from 48 V.
  const auto kOut = kLimit.Limit({Current{-3.0F}, Current{20.0F}}, kTorqueConstant, 100.0_rad_per_s, 48.0_V);
  UNIMOC_EXPECT_FLOAT_EQ(kOut.d.Value(), -3.0F);
  UNIMOC_EXPECT_FLOAT_EQ(kOut.q.Value(), 20.0F);
  // At standstill the q current makes no power: no limit.
  UNIMOC_EXPECT_FLOAT_EQ(kLimit.Limit(Q(80.0F), kTorqueConstant, 0.0_rad_per_s, 48.0_V).q.Value(), 80.0F);
}

ZTEST(dc_link_limit, test_motoring_is_limited_to_the_drive_current) {
  // 480 W allowed at 48 V: i_q * 0.05 * 200 = 480 W -> 48 A, in both directions of rotation.
  UNIMOC_EXPECT_NEAR(kLimit.Limit(Q(80.0F), kTorqueConstant, 200.0_rad_per_s, 48.0_V).q.Value(),
                     48.0F,
                     1e-3F);
  UNIMOC_EXPECT_NEAR(kLimit.Limit(Q(-80.0F), kTorqueConstant, -200.0_rad_per_s, 48.0_V).q.Value(),
                     -48.0F,
                     1e-3F);
}

ZTEST(dc_link_limit, test_braking_is_limited_to_the_derated_charge_current) {
  // 192 W charge at 48 V: |i_q| * 0.05 * 200 = 192 W -> 19.2 A.
  UNIMOC_EXPECT_NEAR(kLimit.Limit(Q(-80.0F), kTorqueConstant, 200.0_rad_per_s, 48.0_V).q.Value(),
                     -19.2F,
                     1e-3F);
  UNIMOC_EXPECT_NEAR(kLimit.Limit(Q(80.0F), kTorqueConstant, -200.0_rad_per_s, 48.0_V).q.Value(),
                     19.2F,
                     1e-3F);
  // Half derated at 56 V: 2 A * 56 V = 112 W -> 11.2 A.
  UNIMOC_EXPECT_NEAR(kLimit.Limit(Q(-80.0F), kTorqueConstant, 200.0_rad_per_s, 56.0_V).q.Value(),
                     -11.2F,
                     1e-3F);
  // From the trip level on no braking current at all; motoring is still allowed.
  UNIMOC_EXPECT_FLOAT_EQ(kLimit.Limit(Q(-80.0F), kTorqueConstant, 200.0_rad_per_s, 58.0_V).q.Value(), 0.0F);
  UNIMOC_EXPECT_FLOAT_EQ(kLimit.Limit(Q(5.0F), kTorqueConstant, 200.0_rad_per_s, 58.0_V).q.Value(), 5.0F);
}

ZTEST(dc_link_limit, test_allowed_torque_is_the_same_power_window) {
  // 200 rad/s at 48 V: 480 W motoring, 192 W braking.
  const auto kForward = kLimit.AllowedTorque(200.0_rad_per_s, 48.0_V);
  UNIMOC_EXPECT_NEAR(kForward.upper.Value(), 2.4F, 1e-5F);
  UNIMOC_EXPECT_NEAR(kForward.lower.Value(), -0.96F, 1e-5F);
  // Reverse: motoring is negative torque, braking positive.
  const auto kReverse = kLimit.AllowedTorque(-200.0_rad_per_s, 48.0_V);
  UNIMOC_EXPECT_NEAR(kReverse.lower.Value(), -2.4F, 1e-5F);
  UNIMOC_EXPECT_NEAR(kReverse.upper.Value(), 0.96F, 1e-5F);
  // Consistent with Limit(): the braking bound as a q current is the limited current.
  UNIMOC_EXPECT_NEAR((kForward.lower / kTorqueConstant).Value(),
                     kLimit.Limit(Q(-80.0F), kTorqueConstant, 200.0_rad_per_s, 48.0_V).q.Value(),
                     1e-3F);
  // Standstill: unbounded.
  const auto kStill = kLimit.AllowedTorque(0.0_rad_per_s, 48.0_V);
  zexpect_true(kStill.lower.Value() < -1e30F && kStill.upper.Value() > 1e30F);
}

ZTEST(dc_link_limit, test_without_a_dc_link_reading_nothing_is_scaled) {
  UNIMOC_EXPECT_FLOAT_EQ(kLimit.Limit(Q(-80.0F), kTorqueConstant, 200.0_rad_per_s, 0.0_V).q.Value(), -80.0F);
}

}  // namespace unimoc::control::test

ZTEST_SUITE(dc_link_limit, nullptr, nullptr, nullptr, nullptr, nullptr);
