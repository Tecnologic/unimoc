/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file rotor_angle_test.cpp
 *	@brief Unit tests for the fixed-point electrical rotor angle.
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */

#include "rotor_angle.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <numbers>
#include "units.hpp"

using unimoc::system::RotorAngle;
using unimoc::unit::Angle;
using namespace unimoc::unit;

namespace {
constexpr float kPi = std::numbers::pi_v<float>;
constexpr float kTwoPi = 2.0F * kPi;
constexpr std::int64_t kCountsPerRevolution = 0x1'0000'0000LL;

/// Sine and cosine tolerance of the 512 entry linearly interpolated table.
constexpr float kTrigTolerance = 2.0e-5F;
}  // namespace

// Compile-time usability of the whole interface.
static_assert(RotorAngle{}.Raw() == 0);
static_assert(RotorAngle{}.Revolutions() == 0);
static_assert(RotorAngle{}.Cos().Value() == 1.0F);
static_assert(RotorAngle::FromRaw(0x4000'0000).Raw() == 0x4000'0000);
static_assert(RotorAngle::FromAngle(Angle{kTwoPi + 0.1F}).Revolutions() == 1);
static_assert(RotorAngle{} == RotorAngle::FromRaw(0));

class RotorAngleTest : public ::testing::Test {
 protected:
  void SetUp() override {}

  void TearDown() override {}
};

TEST_F(RotorAngleTest, DefaultConstructorIsZero) {
  const RotorAngle kAngle;
  EXPECT_EQ(kAngle.Raw(), 0);
  EXPECT_EQ(kAngle.Revolutions(), 0);
  EXPECT_EQ(kAngle.AbsoluteRaw(), 0);
  EXPECT_FLOAT_EQ(kAngle.AngleInRevolution().Value(), 0.0F);
  EXPECT_NEAR(kAngle.Sin().Value(), 0.0F, kTrigTolerance);
  EXPECT_NEAR(kAngle.Cos().Value(), 1.0F, kTrigTolerance);
}

TEST_F(RotorAngleTest, RawScaleMapsFullRangeToOneRevolution) {
  EXPECT_NEAR(RotorAngle::FromRaw(INT32_MIN).AngleInRevolution().Value(), -kPi, 1.0e-6F);
  EXPECT_NEAR(RotorAngle::FromRaw(-0x4000'0000).AngleInRevolution().Value(), -kPi / 2.0F, 1.0e-6F);
  EXPECT_NEAR(RotorAngle::FromRaw(0x4000'0000).AngleInRevolution().Value(), kPi / 2.0F, 1.0e-6F);
  EXPECT_NEAR(RotorAngle::FromRaw(INT32_MAX).AngleInRevolution().Value(), kPi, 1.0e-6F);
}

TEST_F(RotorAngleTest, FromAngleRoundTripsWithinResolution) {
  for (int i = -180; i < 180; ++i) {
    const float kExpected = (static_cast<float>(i) * kPi) / 180.0F;
    const RotorAngle kAngle = RotorAngle::FromAngle(Angle{kExpected});
    EXPECT_EQ(kAngle.Revolutions(), 0);
    EXPECT_NEAR(kAngle.AngleInRevolution().Value(), kExpected, 1.0e-6F);
  }
}

TEST_F(RotorAngleTest, SineAndCosineMatchStandardLibrary) {
  constexpr int kSamples = 4096;
  for (int i = 0; i < kSamples; ++i) {
    const float kExpected = -kPi + ((kTwoPi * static_cast<float>(i)) / static_cast<float>(kSamples));
    const RotorAngle kAngle = RotorAngle::FromAngle(Angle{kExpected});
    EXPECT_NEAR(kAngle.Sin().Value(), std::sin(kExpected), kTrigTolerance) << "i=" << i;
    EXPECT_NEAR(kAngle.Cos().Value(), std::cos(kExpected), kTrigTolerance) << "i=" << i;
  }
}

TEST_F(RotorAngleTest, SineAndCosineFollowRawWrap) {
  // The seam at +-pi must not introduce a sign flip.
  const RotorAngle kBelow = RotorAngle::FromRaw(INT32_MAX);
  const RotorAngle kAbove = RotorAngle::FromRaw(INT32_MIN);
  EXPECT_NEAR(kBelow.Sin().Value(), kAbove.Sin().Value(), 1.0e-5F);
  EXPECT_NEAR(kBelow.Cos().Value(), kAbove.Cos().Value(), 1.0e-5F);
  EXPECT_NEAR(kAbove.Cos().Value(), -1.0F, kTrigTolerance);
}

TEST_F(RotorAngleTest, AdvanceCountsPositiveWrap) {
  RotorAngle angle = RotorAngle::FromAngle(Angle{kPi - 0.1F});
  ASSERT_EQ(angle.Revolutions(), 0);

  angle.Advance(0.2_rad);
  EXPECT_EQ(angle.Revolutions(), 1);
  EXPECT_NEAR(angle.AngleInRevolution().Value(), -kPi + 0.1F, 1.0e-5F);
  EXPECT_NEAR(angle.AbsoluteAngle().Value(), kPi + 0.1F, 1.0e-5F);
}

TEST_F(RotorAngleTest, AdvanceCountsNegativeWrap) {
  RotorAngle angle = RotorAngle::FromAngle(Angle{-kPi + 0.1F});
  ASSERT_EQ(angle.Revolutions(), 0);

  angle.Advance(-0.2_rad);
  EXPECT_EQ(angle.Revolutions(), -1);
  EXPECT_NEAR(angle.AngleInRevolution().Value(), kPi - 0.1F, 1.0e-5F);
  EXPECT_NEAR(angle.AbsoluteAngle().Value(), -kPi - 0.1F, 1.0e-5F);
}

TEST_F(RotorAngleTest, AdvanceAcceptsManyRevolutionsAtOnce) {
  RotorAngle angle;
  angle.Advance(100.5_rad * kTwoPi);
  EXPECT_EQ(angle.Revolutions(), 101);
  EXPECT_NEAR(angle.AngleInRevolution().Value(), -kPi, 1.0e-3F);

  angle.Advance(-100.5_rad * kTwoPi);
  EXPECT_EQ(angle.Revolutions(), 0);
}

TEST_F(RotorAngleTest, RepeatedAdvanceDoesNotCompoundError) {
  constexpr int kStepsPerRevolution = 1000;
  constexpr int kRevolutions = 25;
  constexpr int kSteps = kStepsPerRevolution * kRevolutions;
  const Angle kStep{kTwoPi / static_cast<float>(kStepsPerRevolution)};

  RotorAngle angle;
  for (int i = 0; i < kSteps; ++i) {
    angle.Advance(kStep);
  }

  // Accumulation is integer, so the error is bounded by half a count per call and
  // never compounds; a float phase accumulator would drift far beyond that.
  EXPECT_EQ(angle.Revolutions(), kRevolutions);
  EXPECT_LE(std::abs(static_cast<double>(angle.Raw())), 0.5 * kSteps);
}

TEST_F(RotorAngleTest, AbsoluteRawIsMonotonicAcrossWrap) {
  RotorAngle angle = RotorAngle::FromAngle(Angle{kPi - 0.05F});
  std::int64_t previous = angle.AbsoluteRaw();
  for (int i = 0; i < 100; ++i) {
    angle.Advance(0.01_rad);
    EXPECT_GT(angle.AbsoluteRaw(), previous);
    previous = angle.AbsoluteRaw();
  }
  EXPECT_EQ(angle.Revolutions(), 1);
}

TEST_F(RotorAngleTest, AbsoluteRawMatchesAbsoluteAngle) {
  const RotorAngle kAngle = RotorAngle::FromAngle(7.0_rad);
  const double kExpected = static_cast<double>(kAngle.AbsoluteRaw()) *
                           (2.0 * std::numbers::pi_v<double> / static_cast<double>(kCountsPerRevolution));
  EXPECT_NEAR(kAngle.AbsoluteAngle().Value(), static_cast<float>(kExpected), 1.0e-5F);
}

TEST_F(RotorAngleTest, DifferenceTakesShortestPath) {
  const RotorAngle kAhead = RotorAngle::FromAngle(Angle{kPi - 0.1F});
  const RotorAngle kBehind = RotorAngle::FromAngle(Angle{-kPi + 0.1F});

  EXPECT_NEAR((kBehind - kAhead).Value(), 0.2F, 1.0e-5F);
  EXPECT_NEAR((kAhead - kBehind).Value(), -0.2F, 1.0e-5F);
}

TEST_F(RotorAngleTest, DifferenceIgnoresRevolutionCounter) {
  const RotorAngle kFirst = RotorAngle::FromRaw(0x2000'0000, 3);
  const RotorAngle kSecond = RotorAngle::FromRaw(0x2000'0000, -7);
  EXPECT_FLOAT_EQ((kFirst - kSecond).Value(), 0.0F);
}

TEST_F(RotorAngleTest, OffsetOperators) {
  const RotorAngle kBase = RotorAngle::FromAngle(1.0_rad);

  RotorAngle mutated = kBase;
  mutated += 0.5_rad;
  EXPECT_NEAR(mutated.AngleInRevolution().Value(), 1.5F, 1.0e-5F);
  mutated -= 0.5_rad;
  EXPECT_EQ(mutated, kBase);

  EXPECT_NEAR((kBase + 0.25_rad).AngleInRevolution().Value(), 1.25F, 1.0e-5F);
  EXPECT_NEAR((kBase - 0.25_rad).AngleInRevolution().Value(), 0.75F, 1.0e-5F);
  EXPECT_EQ(kBase, RotorAngle::FromAngle(1.0_rad));
}

TEST_F(RotorAngleTest, EqualityComparesPhaseAndRevolutions) {
  EXPECT_EQ(RotorAngle::FromRaw(1234, 5), RotorAngle::FromRaw(1234, 5));
  EXPECT_NE(RotorAngle::FromRaw(1234, 5), RotorAngle::FromRaw(1234, 6));
  EXPECT_NE(RotorAngle::FromRaw(1234, 5), RotorAngle::FromRaw(1235, 5));
}
