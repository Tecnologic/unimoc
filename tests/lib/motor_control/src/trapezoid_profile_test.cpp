/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file trapezoid_profile_test.cpp
 *    @brief ztest coverage of the trapezoidal speed profile of a positioning move.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "trapezoid_profile.hpp"
#include <zephyr/ztest.h>
#include "unimoc_test.hpp"

namespace unimoc::control::test {

using namespace unit;

namespace {

constexpr Time kDt{0.5e-3F};
constexpr AngularVelocity kSpeedLimit = 40.0_rad_per_s;
constexpr AngularAcceleration kAccelLimit = 400.0_rad_per_s2;

/// Slack of the float steps against the limits.
constexpr AngularAcceleration kAccelSlack = 1.0_rad_per_s2;

TrapezoidProfile MakeProfile() {
  TrapezoidProfile profile;
  profile.speed_limit = kSpeedLimit;
  profile.accel_limit = kAccelLimit;
  return profile;
}

/// Extremes of a run of the profile toward one target.
struct Run {
  unsigned steps{0U};
  AngularVelocity fastest;
  AngularAcceleration largest_accel;
  Angle furthest;  ///< Largest position, for the overshoot of a positive move.
  bool arrived{false};
};

Run RunTo(TrapezoidProfile& profile, const Angle target, const unsigned max_steps) {
  Run run;
  run.furthest = profile.position;
  for (; run.steps < max_steps && !profile.AtTarget(target); ++run.steps) {
    profile.Step(target, kDt);
    run.fastest = Max(run.fastest, Abs(profile.speed));
    run.largest_accel = Max(run.largest_accel, Abs(profile.acceleration));
    run.furthest = Max(run.furthest, profile.position);
  }
  run.arrived = profile.AtTarget(target);
  return run;
}

}  // namespace

ZTEST(trapezoid_profile, test_long_move_cruises_at_the_speed_limit) {
  TrapezoidProfile profile = MakeProfile();
  // 10 rad: 0.1 s ramps (2 rad) and 0.2 s cruise (8 rad).
  const Run kRun = RunTo(profile, 10.0_rad, 4000U);
  zassert_true(kRun.arrived);
  UNIMOC_EXPECT_FLOAT_EQ(profile.position, 10.0_rad);
  UNIMOC_EXPECT_NEAR(kRun.fastest, kSpeedLimit, 1e-3_rad_per_s);
  zexpect_true(kRun.largest_accel <= kAccelLimit + kAccelSlack);
  // No overshoot of the target.
  zexpect_true(kRun.furthest <= 10.0_rad + 1e-5_rad);
  // d / v + v / a = 0.25 s + 0.1 s.
  UNIMOC_EXPECT_NEAR(static_cast<float>(kRun.steps) * kDt, 0.35_s, 2e-3_s);
}

ZTEST(trapezoid_profile, test_short_move_is_a_triangle) {
  TrapezoidProfile profile = MakeProfile();
  // 1 rad: the peak sqrt(a d) = 20 rad/s stays below the limit.
  const Run kRun = RunTo(profile, 1.0_rad, 4000U);
  zassert_true(kRun.arrived);
  UNIMOC_EXPECT_NEAR(kRun.fastest, 20.0_rad_per_s, 0.3_rad_per_s);
  zexpect_true(kRun.largest_accel <= kAccelLimit + kAccelSlack);
  zexpect_true(kRun.furthest <= 1.0_rad + 1e-5_rad);
  // 2 sqrt(d / a) = 0.1 s.
  UNIMOC_EXPECT_NEAR(static_cast<float>(kRun.steps) * kDt, 0.1_s, 2e-3_s);
}

ZTEST(trapezoid_profile, test_negative_move) {
  TrapezoidProfile profile = MakeProfile();
  profile.Reset(2.0_rad);
  const Run kRun = RunTo(profile, -3.0_rad, 4000U);
  zassert_true(kRun.arrived);
  UNIMOC_EXPECT_FLOAT_EQ(profile.position, -3.0_rad);
  UNIMOC_EXPECT_NEAR(kRun.fastest, kSpeedLimit, 1e-3_rad_per_s);
  zexpect_true(kRun.largest_accel <= kAccelLimit + kAccelSlack);
}

ZTEST(trapezoid_profile, test_target_reversal_keeps_the_acceleration_limit) {
  TrapezoidProfile profile = MakeProfile();
  (void)RunTo(profile, 10.0_rad, 300U);  // cruising forward
  zassert_true(profile.speed > 30.0_rad_per_s);
  // The new target lies behind: the reference brakes, turns and comes back.
  const Run kRun = RunTo(profile, 0.0_rad, 4000U);
  zassert_true(kRun.arrived);
  UNIMOC_EXPECT_FLOAT_EQ(profile.position, 0.0_rad);
  zexpect_true(kRun.largest_accel <= kAccelLimit + kAccelSlack);
}

ZTEST(trapezoid_profile, test_start_from_a_moving_shaft) {
  TrapezoidProfile profile = MakeProfile();
  // Moving away from the target at 30 rad/s.
  profile.Reset(0.0_rad, -30.0_rad_per_s);
  const Run kRun = RunTo(profile, 0.5_rad, 4000U);
  zassert_true(kRun.arrived);
  UNIMOC_EXPECT_FLOAT_EQ(profile.position, 0.5_rad);
  zexpect_true(kRun.largest_accel <= kAccelLimit + kAccelSlack);
}

ZTEST(trapezoid_profile, test_lowered_speed_limit_brakes_with_the_acceleration_limit) {
  TrapezoidProfile profile = MakeProfile();
  (void)RunTo(profile, 20.0_rad, 300U);
  profile.speed_limit = 10.0_rad_per_s;
  profile.Step(20.0_rad, kDt);
  UNIMOC_EXPECT_NEAR(profile.acceleration, -kAccelLimit, kAccelSlack);
  const Run kRun = RunTo(profile, 20.0_rad, 10000U);
  zassert_true(kRun.arrived);
  zexpect_true(kRun.largest_accel <= kAccelLimit + kAccelSlack);
}

ZTEST(trapezoid_profile, test_acceleration_follows_the_speed) {
  TrapezoidProfile profile = MakeProfile();
  profile.Step(5.0_rad, kDt);
  UNIMOC_EXPECT_NEAR(profile.acceleration, kAccelLimit, kAccelSlack);
  UNIMOC_EXPECT_NEAR(profile.speed, kAccelLimit * kDt, 1e-4_rad_per_s);
  UNIMOC_EXPECT_NEAR(profile.position, kAccelLimit * kDt * kDt, 1e-7_rad);
}

ZTEST(trapezoid_profile, test_without_limits_the_reference_jumps) {
  TrapezoidProfile profile;
  profile.Step(3.0_rad, kDt);
  zexpect_true(profile.AtTarget(3.0_rad));
  profile = MakeProfile();
  profile.accel_limit = AngularAcceleration{};
  profile.Step(-1.0_rad, kDt);
  zexpect_true(profile.AtTarget(-1.0_rad));
}

ZTEST(trapezoid_profile, test_at_target_it_rests) {
  TrapezoidProfile profile = MakeProfile();
  profile.Reset(1.5_rad);
  for (int i = 0; i < 10; ++i) {
    profile.Step(1.5_rad, kDt);
  }
  zexpect_true(profile.AtTarget(1.5_rad));
  UNIMOC_EXPECT_FLOAT_EQ(profile.acceleration, AngularAcceleration{});
}

ZTEST_SUITE(trapezoid_profile, nullptr, nullptr, nullptr, nullptr, nullptr);

}  // namespace unimoc::control::test
