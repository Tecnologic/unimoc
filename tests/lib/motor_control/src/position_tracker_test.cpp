/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file position_tracker_test.cpp
 *    @brief ztest coverage of the multi-turn position tracker.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "position_tracker.hpp"
#include <zephyr/ztest.h>
#include <cmath>
#include <numbers>
#include "unimoc_test.hpp"

namespace unimoc::observer::test {

using namespace unit;

// Shared helpers of the position_tracker suite.
namespace {

using Tracker = PositionTracker;
using Angle = unimoc::unit::Angle;
using RotorAngle = unimoc::system::RotorAngle;

constexpr float kPi = std::numbers::pi_v<float>;
constexpr float kTwoPi = 2.0F * kPi;
constexpr int kPolePairs = 4;

}  // namespace

// --- Default state
ZTEST(position_tracker, test_default_state_is_zero) {
  Tracker tracker;
  UNIMOC_EXPECT_FLOAT_EQ(tracker.PositionRad().Value(), 0.0F);
  UNIMOC_EXPECT_FLOAT_EQ(tracker.PositionRev().Value(), 0.0F);
  zexpect_false(tracker.IsHomed());
}

// --- The adapter uses RotorAngle's absolute electrical position
ZTEST(position_tracker, test_converts_absolute_electrical_position_to_mechanical) {
  Tracker tracker;
  const RotorAngle kAngle = RotorAngle::FromAngle(0.5_rad);
  tracker.Update(kAngle, kPolePairs);
  UNIMOC_EXPECT_NEAR(tracker.PositionRad().Value(), 0.5F / static_cast<float>(kPolePairs), 1e-6F);
  zexpect_equal(kAngle, RotorAngle::FromAngle(0.5_rad));
}

ZTEST(position_tracker, test_uses_source_revolution_count_across_positive_wrap) {
  Tracker tracker;
  RotorAngle angle = RotorAngle::FromAngle(Angle{kPi - 0.01F});
  angle.Advance(0.02_rad);
  tracker.Update(angle, kPolePairs);
  zexpect_equal(angle.Revolutions(), 1);
  UNIMOC_EXPECT_NEAR(tracker.PositionRad().Value(), (kPi + 0.01F) / static_cast<float>(kPolePairs), 1e-5F);
}

ZTEST(position_tracker, test_uses_source_revolution_count_across_negative_wrap) {
  Tracker tracker;
  RotorAngle angle = RotorAngle::FromAngle(Angle{-kPi + 0.01F});
  angle.Advance(-0.02_rad);
  tracker.Update(angle, kPolePairs);
  zexpect_equal(angle.Revolutions(), -1);
  UNIMOC_EXPECT_NEAR(tracker.PositionRad().Value(), (-kPi - 0.01F) / static_cast<float>(kPolePairs), 1e-5F);
}

ZTEST(position_tracker, test_converts_multiple_source_revolutions) {
  Tracker tracker;
  const RotorAngle kAngle = RotorAngle::FromRaw(0, 4 * kPolePairs);
  tracker.Update(kAngle, kPolePairs);
  UNIMOC_EXPECT_NEAR(tracker.PositionRad().Value(), 4.0F * kTwoPi, 1e-5F);
  UNIMOC_EXPECT_NEAR(tracker.PositionRev().Value(), 4.0F, 1e-6F);
}

// --- Position formula: (turns * 2π + theta) / pole_pairs − home_offset
ZTEST(position_tracker, test_position_formula) {
  Tracker tracker;
  tracker.Update(RotorAngle::FromAngle(Angle{kPi / 2.0F}), kPolePairs);
  UNIMOC_EXPECT_NEAR(tracker.PositionRad().Value(), kPi / 8.0F, 1e-5F);
}

// --- SetHome() zeroes position
ZTEST(position_tracker, test_set_home_clears_position) {
  Tracker tracker;
  tracker.Update(RotorAngle::FromAngle(Angle{kPi / 2.0F}), kPolePairs);
  tracker.SetHome();
  UNIMOC_EXPECT_FLOAT_EQ(tracker.PositionRad().Value(), 0.0F);
  UNIMOC_EXPECT_FLOAT_EQ(tracker.PositionRev().Value(), 0.0F);
  zexpect_true(tracker.IsHomed());
}

// --- After homing, position is relative to home
ZTEST(position_tracker, test_position_relative_to_home) {
  Tracker tracker;
  tracker.Update(RotorAngle{}, kPolePairs);
  tracker.SetHome();

  tracker.Update(RotorAngle::FromAngle(Angle{kPi / 2.0F}), kPolePairs);
  UNIMOC_EXPECT_NEAR(tracker.PositionRad().Value(), kPi / 8.0F, 1e-5F);
}

ZTEST(position_tracker, test_home_offset_uses_mechanical_position_and_leaves_source_untouched) {
  Tracker tracker;
  const RotorAngle kHome = RotorAngle::FromRaw(0, 12);
  tracker.Update(kHome, 3);
  tracker.SetHome();
  const RotorAngle kMoved = RotorAngle::FromRaw(0, 15);
  tracker.Update(kMoved, 3);
  UNIMOC_EXPECT_NEAR(tracker.PositionRad().Value(), kTwoPi, 1e-5F);
  zexpect_equal(kMoved.Revolutions(), 15);
}

// --- reset() clears all state
ZTEST(position_tracker, test_reset_clears_all) {
  Tracker tracker;
  tracker.Update(RotorAngle::FromAngle(Angle{kPi / 2.0F}), kPolePairs);
  tracker.SetHome();
  tracker.Update(RotorAngle::FromAngle(1.0_rad), kPolePairs);
  tracker.Reset();
  UNIMOC_EXPECT_FLOAT_EQ(tracker.PositionRad().Value(), 0.0F);
  zexpect_false(tracker.IsHomed());
}

ZTEST(position_tracker, test_zero_pole_pairs_leaves_state_unchanged) {
  Tracker tracker;
  tracker.Update(RotorAngle::FromAngle(1.0_rad), kPolePairs);
  const auto kPosition = tracker.PositionRad();
  tracker.Update(RotorAngle::FromAngle(2.0_rad), 0);
  zexpect_equal(tracker.PositionRad(), kPosition);
}

}  // namespace unimoc::observer::test

ZTEST_SUITE(position_tracker, nullptr, nullptr, nullptr, nullptr, nullptr);
