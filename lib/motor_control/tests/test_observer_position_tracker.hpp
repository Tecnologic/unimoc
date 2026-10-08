#pragma once

#ifndef UNIMOC_TEST_POSITION_TRACKER_H_
#define UNIMOC_TEST_POSITION_TRACKER_H_

#include <gtest/gtest.h>
#include <cmath>
#include <numbers>
#include "position_tracker.hpp"

namespace unimoc::observer::test {

using namespace unit;

class PositionTrackerTest : public ::testing::Test {
 protected:
  using Tracker = PositionTracker;
  using Angle = unimoc::unit::Angle;
  using RotorAngle = unimoc::system::RotorAngle;

  static constexpr float kPi = std::numbers::pi_v<float>;
  static constexpr float kTwoPi = 2.0F * kPi;
  static constexpr int kPolePairs = 4;
};

// --- Default state
TEST_F(PositionTrackerTest, DefaultStateIsZero) {
  Tracker tracker;
  EXPECT_FLOAT_EQ(tracker.PositionRad().Value(), 0.0F);
  EXPECT_FLOAT_EQ(tracker.PositionRev().Value(), 0.0F);
  EXPECT_FALSE(tracker.IsHomed());
}

// --- The adapter uses RotorAngle's absolute electrical position
TEST_F(PositionTrackerTest, ConvertsAbsoluteElectricalPositionToMechanical) {
  Tracker tracker;
  const RotorAngle kAngle = RotorAngle::FromAngle(0.5_rad);
  tracker.Update(kAngle, kPolePairs);
  EXPECT_NEAR(tracker.PositionRad().Value(), 0.5F / static_cast<float>(kPolePairs), 1e-6F);
  EXPECT_EQ(kAngle, RotorAngle::FromAngle(0.5_rad));
}

TEST_F(PositionTrackerTest, UsesSourceRevolutionCountAcrossPositiveWrap) {
  Tracker tracker;
  RotorAngle angle = RotorAngle::FromAngle(Angle{kPi - 0.01F});
  angle.Advance(0.02_rad);
  tracker.Update(angle, kPolePairs);
  EXPECT_EQ(angle.Revolutions(), 1);
  EXPECT_NEAR(tracker.PositionRad().Value(), (kPi + 0.01F) / static_cast<float>(kPolePairs), 1e-5F);
}

TEST_F(PositionTrackerTest, UsesSourceRevolutionCountAcrossNegativeWrap) {
  Tracker tracker;
  RotorAngle angle = RotorAngle::FromAngle(Angle{-kPi + 0.01F});
  angle.Advance(-0.02_rad);
  tracker.Update(angle, kPolePairs);
  EXPECT_EQ(angle.Revolutions(), -1);
  EXPECT_NEAR(tracker.PositionRad().Value(), (-kPi - 0.01F) / static_cast<float>(kPolePairs), 1e-5F);
}

TEST_F(PositionTrackerTest, ConvertsMultipleSourceRevolutions) {
  Tracker tracker;
  const RotorAngle kAngle = RotorAngle::FromRaw(0, 4 * kPolePairs);
  tracker.Update(kAngle, kPolePairs);
  EXPECT_NEAR(tracker.PositionRad().Value(), 4.0F * kTwoPi, 1e-5F);
  EXPECT_NEAR(tracker.PositionRev().Value(), 4.0F, 1e-6F);
}

// --- Position formula: (turns * 2π + theta) / pole_pairs − home_offset
TEST_F(PositionTrackerTest, PositionFormula) {
  Tracker tracker;
  tracker.Update(RotorAngle::FromAngle(Angle{kPi / 2.0F}), kPolePairs);
  EXPECT_NEAR(tracker.PositionRad().Value(), kPi / 8.0F, 1e-5F);
}

// --- SetHome() zeroes position
TEST_F(PositionTrackerTest, SetHomeClearsPosition) {
  Tracker tracker;
  tracker.Update(RotorAngle::FromAngle(Angle{kPi / 2.0F}), kPolePairs);
  tracker.SetHome();
  EXPECT_FLOAT_EQ(tracker.PositionRad().Value(), 0.0F);
  EXPECT_FLOAT_EQ(tracker.PositionRev().Value(), 0.0F);
  EXPECT_TRUE(tracker.IsHomed());
}

// --- After homing, position is relative to home
TEST_F(PositionTrackerTest, PositionRelativeToHome) {
  Tracker tracker;
  tracker.Update(RotorAngle{}, kPolePairs);
  tracker.SetHome();

  tracker.Update(RotorAngle::FromAngle(Angle{kPi / 2.0F}), kPolePairs);
  EXPECT_NEAR(tracker.PositionRad().Value(), kPi / 8.0F, 1e-5F);
}

TEST_F(PositionTrackerTest, HomeOffsetUsesMechanicalPositionAndLeavesSourceUntouched) {
  Tracker tracker;
  const RotorAngle kHome = RotorAngle::FromRaw(0, 12);
  tracker.Update(kHome, 3);
  tracker.SetHome();
  const RotorAngle kMoved = RotorAngle::FromRaw(0, 15);
  tracker.Update(kMoved, 3);
  EXPECT_NEAR(tracker.PositionRad().Value(), kTwoPi, 1e-5F);
  EXPECT_EQ(kMoved.Revolutions(), 15);
}

// --- reset() clears all state
TEST_F(PositionTrackerTest, ResetClearsAll) {
  Tracker tracker;
  tracker.Update(RotorAngle::FromAngle(Angle{kPi / 2.0F}), kPolePairs);
  tracker.SetHome();
  tracker.Update(RotorAngle::FromAngle(1.0_rad), kPolePairs);
  tracker.Reset();
  EXPECT_FLOAT_EQ(tracker.PositionRad().Value(), 0.0F);
  EXPECT_FALSE(tracker.IsHomed());
}

TEST_F(PositionTrackerTest, ZeroPolePairsLeavesStateUnchanged) {
  Tracker tracker;
  tracker.Update(RotorAngle::FromAngle(1.0_rad), kPolePairs);
  const auto kPosition = tracker.PositionRad();
  tracker.Update(RotorAngle::FromAngle(2.0_rad), 0);
  EXPECT_EQ(tracker.PositionRad(), kPosition);
}

}  // namespace unimoc::observer::test

#endif /* UNIMOC_TEST_POSITION_TRACKER_H_ */
