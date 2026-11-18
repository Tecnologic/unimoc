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
                             .trip = 58.0_V,
                             .stator_resistance = Resistance{}};

/// The same limits with the copper losses of a 0.1 Ohm winding.
constexpr DcLinkLimit kCopperLimit{.drive_current_max = 10.0_A,
                                   .charge_current_max = 4.0_A,
                                   .regen_derate_start = 54.0_V,
                                   .trip = 58.0_V,
                                   .stator_resistance = 0.1_Ohm};
constexpr MagneticFlux kTorqueConstant{0.05F};  // N m/A

system::Rotor<Current> Q(const float i_q) { return {Current{}, Current{i_q}}; }

}  // namespace

ZTEST(dc_link_limit, test_charge_limit_falls_linearly_to_zero_at_the_trip_level) {
  UNIMOC_EXPECT_FLOAT_EQ(kLimit.AllowedChargeCurrent(48.0_V), 4.0_A);
  UNIMOC_EXPECT_FLOAT_EQ(kLimit.AllowedChargeCurrent(54.0_V), 4.0_A);
  UNIMOC_EXPECT_NEAR(kLimit.AllowedChargeCurrent(56.0_V), 2.0_A, 1e-5_A);
  UNIMOC_EXPECT_FLOAT_EQ(kLimit.AllowedChargeCurrent(58.0_V), 0.0_A);
  UNIMOC_EXPECT_FLOAT_EQ(kLimit.AllowedChargeCurrent(70.0_V), 0.0_A);
}

ZTEST(dc_link_limit, test_inside_the_limits_the_current_is_unchanged) {
  // 20 A * 0.05 N m/A * 100 rad/s = 100 W, about 2 A from 48 V.
  const auto kOut = kLimit.Limit({-3.0_A, 20.0_A}, kTorqueConstant, 100.0_rad_per_s, 48.0_V);
  UNIMOC_EXPECT_FLOAT_EQ(kOut.d, -3.0_A);
  UNIMOC_EXPECT_FLOAT_EQ(kOut.q, 20.0_A);
  // At standstill the q current makes no power: no limit.
  UNIMOC_EXPECT_FLOAT_EQ(kLimit.Limit(Q(80.0F), kTorqueConstant, 0.0_rad_per_s, 48.0_V).q, 80.0_A);
}

ZTEST(dc_link_limit, test_motoring_is_limited_to_the_drive_current) {
  // 480 W allowed at 48 V: i_q * 0.05 * 200 = 480 W -> 48 A, in both directions of rotation.
  UNIMOC_EXPECT_NEAR(kLimit.Limit(Q(80.0F), kTorqueConstant, 200.0_rad_per_s, 48.0_V).q, 48.0_A, 1e-3_A);
  UNIMOC_EXPECT_NEAR(kLimit.Limit(Q(-80.0F), kTorqueConstant, -200.0_rad_per_s, 48.0_V).q, -48.0_A, 1e-3_A);
}

ZTEST(dc_link_limit, test_braking_is_limited_to_the_derated_charge_current) {
  // 192 W charge at 48 V: |i_q| * 0.05 * 200 = 192 W -> 19.2 A.
  UNIMOC_EXPECT_NEAR(kLimit.Limit(Q(-80.0F), kTorqueConstant, 200.0_rad_per_s, 48.0_V).q, -19.2_A, 1e-3_A);
  UNIMOC_EXPECT_NEAR(kLimit.Limit(Q(80.0F), kTorqueConstant, -200.0_rad_per_s, 48.0_V).q, 19.2_A, 1e-3_A);
  // Half derated at 56 V: 2 A * 56 V = 112 W -> 11.2 A.
  UNIMOC_EXPECT_NEAR(kLimit.Limit(Q(-80.0F), kTorqueConstant, 200.0_rad_per_s, 56.0_V).q, -11.2_A, 1e-3_A);
  // From the trip level on no braking current at all; motoring is still allowed.
  UNIMOC_EXPECT_FLOAT_EQ(kLimit.Limit(Q(-80.0F), kTorqueConstant, 200.0_rad_per_s, 58.0_V).q, 0.0_A);
  UNIMOC_EXPECT_FLOAT_EQ(kLimit.Limit(Q(5.0F), kTorqueConstant, 200.0_rad_per_s, 58.0_V).q, 5.0_A);
}

ZTEST(dc_link_limit, test_allowed_torque_is_the_same_power_window) {
  // 200 rad/s at 48 V: 480 W motoring, 192 W braking.
  const auto kForward = kLimit.AllowedTorque(200.0_rad_per_s, 48.0_V, kTorqueConstant);
  UNIMOC_EXPECT_NEAR(kForward.upper, 2.4_Nm, 1e-5_Nm);
  UNIMOC_EXPECT_NEAR(kForward.lower, -0.96_Nm, 1e-5_Nm);
  // Reverse: motoring is negative torque, braking positive.
  const auto kReverse = kLimit.AllowedTorque(-200.0_rad_per_s, 48.0_V, kTorqueConstant);
  UNIMOC_EXPECT_NEAR(kReverse.lower, -2.4_Nm, 1e-5_Nm);
  UNIMOC_EXPECT_NEAR(kReverse.upper, 0.96_Nm, 1e-5_Nm);
  // Consistent with Limit(): the braking bound as a q current is the limited current.
  UNIMOC_EXPECT_NEAR(kForward.lower / kTorqueConstant,
                     kLimit.Limit(Q(-80.0F), kTorqueConstant, 200.0_rad_per_s, 48.0_V).q,
                     1e-3_A);
  // Standstill: unbounded.
  const auto kStill = kLimit.AllowedTorque(0.0_rad_per_s, 48.0_V, kTorqueConstant);
  zexpect_true(kStill.lower < -1e30_Nm && kStill.upper > 1e30_Nm);
}

ZTEST(dc_link_limit, test_copper_losses_limit_the_current_at_standstill) {
  // 480 W from 48 V into 3/2 * 0.1 Ohm: |i| <= sqrt(480 / 0.15) = 56.57 A.
  UNIMOC_EXPECT_NEAR(kCopperLimit.Limit(Q(80.0F), kTorqueConstant, 0.0_rad_per_s, 48.0_V).q,
                     56.5685_A,
                     1e-3_A);
  UNIMOC_EXPECT_NEAR(kCopperLimit.Limit(Q(-80.0F), kTorqueConstant, 0.0_rad_per_s, 48.0_V).q,
                     -56.5685_A,
                     1e-3_A);
  // A d current takes its share: sqrt(3200 - 30^2) = 47.96 A.
  const auto kWithD = kCopperLimit.Limit({30.0_A, 80.0_A}, kTorqueConstant, 0.0_rad_per_s, 48.0_V);
  UNIMOC_EXPECT_FLOAT_EQ(kWithD.d, 30.0_A);
  UNIMOC_EXPECT_NEAR(kWithD.q, 47.958_A, 1e-3_A);
  // The limited point draws exactly the drive current.
  UNIMOC_EXPECT_NEAR(kCopperLimit.BatteryCurrent(kWithD, kTorqueConstant, 0.0_rad_per_s, 48.0_V),
                     10.0_A,
                     1e-3_A);
  // A d current that alone exceeds the limit leaves no q current.
  UNIMOC_EXPECT_FLOAT_EQ(kCopperLimit.Limit({60.0_A, 10.0_A}, kTorqueConstant, 0.0_rad_per_s, 48.0_V).q,
                         0.0_A);
  // The torque window at standstill is now bounded as well.
  const auto kStill = kCopperLimit.AllowedTorque(0.0_rad_per_s, 48.0_V, kTorqueConstant);
  UNIMOC_EXPECT_NEAR(kStill.upper, kTorqueConstant * 56.5685_A, 1e-4_Nm);
  UNIMOC_EXPECT_NEAR(kStill.lower, -(kTorqueConstant * 56.5685_A), 1e-4_Nm);
}

ZTEST(dc_link_limit, test_motoring_limit_includes_the_copper_losses) {
  // 200 rad/s: 10 V i_q + 0.15 Ohm i_q^2 = 480 W -> i_q = 32.33 A (48 A without losses).
  const auto kOut = kCopperLimit.Limit(Q(80.0F), kTorqueConstant, 200.0_rad_per_s, 48.0_V);
  UNIMOC_EXPECT_NEAR(kOut.q, 32.3257_A, 1e-3_A);
  UNIMOC_EXPECT_NEAR(kCopperLimit.BatteryCurrent(kOut, kTorqueConstant, 200.0_rad_per_s, 48.0_V),
                     10.0_A,
                     1e-4_A);
  // The same in reverse.
  UNIMOC_EXPECT_NEAR(kCopperLimit.Limit(Q(-80.0F), kTorqueConstant, -200.0_rad_per_s, 48.0_V).q,
                     -32.3257_A,
                     1e-3_A);
  // Below the limit nothing changes.
  UNIMOC_EXPECT_FLOAT_EQ(kCopperLimit.Limit(Q(20.0F), kTorqueConstant, 200.0_rad_per_s, 48.0_V).q, 20.0_A);
}

ZTEST(dc_link_limit, test_charge_limit_ignores_the_copper_losses) {
  // 400 rad/s: the charge bound is the mechanical power alone, 192 W / 20 V = 9.6 A.
  // The real charge current is lower by the losses, also with R_s estimated too high.
  const auto kOut = kCopperLimit.Limit(Q(-80.0F), kTorqueConstant, 400.0_rad_per_s, 48.0_V);
  UNIMOC_EXPECT_NEAR(kOut.q, -9.6_A, 1e-3_A);
  zexpect_true(kCopperLimit.BatteryCurrent(kOut, kTorqueConstant, 400.0_rad_per_s, 48.0_V) > -4.0_A,
               "less than the charge limit reaches the battery");
  UNIMOC_EXPECT_NEAR(kCopperLimit.Limit(Q(80.0F), kTorqueConstant, -400.0_rad_per_s, 48.0_V).q,
                     9.6_A,
                     1e-3_A);
  // At low speed the drive limit bounds braking too: at 20 rad/s the losses of
  // 60 A (540 W less 60 W braking power) take the whole 480 W.
  const auto kSlow = kCopperLimit.Limit(Q(-80.0F), kTorqueConstant, 20.0_rad_per_s, 48.0_V);
  UNIMOC_EXPECT_NEAR(kSlow.q, -60.0_A, 1e-3_A);
  UNIMOC_EXPECT_NEAR(kCopperLimit.BatteryCurrent(kSlow, kTorqueConstant, 20.0_rad_per_s, 48.0_V),
                     10.0_A,
                     1e-3_A);
}

ZTEST(dc_link_limit, test_without_a_dc_link_reading_nothing_is_scaled) {
  UNIMOC_EXPECT_FLOAT_EQ(kLimit.Limit(Q(-80.0F), kTorqueConstant, 200.0_rad_per_s, 0.0_V).q, -80.0_A);
}

}  // namespace unimoc::control::test

ZTEST_SUITE(dc_link_limit, nullptr, nullptr, nullptr, nullptr, nullptr);
