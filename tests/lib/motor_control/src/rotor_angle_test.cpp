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
 *	SPDX-License-Identifier: GPL-3.0-or-later
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */

#include "rotor_angle.hpp"
#include <zephyr/ztest.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include "unimoc_test.hpp"
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

ZTEST(rotor_angle, test_default_constructor_is_zero) {
  const RotorAngle kAngle;
  zexpect_equal(kAngle.Raw(), 0);
  zexpect_equal(kAngle.Revolutions(), 0);
  zexpect_equal(kAngle.AbsoluteRaw(), 0);
  UNIMOC_EXPECT_FLOAT_EQ(kAngle.AngleInRevolution().Value(), 0.0F);
  UNIMOC_EXPECT_NEAR(kAngle.Sin().Value(), 0.0F, kTrigTolerance);
  UNIMOC_EXPECT_NEAR(kAngle.Cos().Value(), 1.0F, kTrigTolerance);
}

ZTEST(rotor_angle, test_raw_scale_maps_full_range_to_one_revolution) {
  UNIMOC_EXPECT_NEAR(RotorAngle::FromRaw(INT32_MIN).AngleInRevolution().Value(), -kPi, 1.0e-6F);
  UNIMOC_EXPECT_NEAR(RotorAngle::FromRaw(-0x4000'0000).AngleInRevolution().Value(), -kPi / 2.0F, 1.0e-6F);
  UNIMOC_EXPECT_NEAR(RotorAngle::FromRaw(0x4000'0000).AngleInRevolution().Value(), kPi / 2.0F, 1.0e-6F);
  UNIMOC_EXPECT_NEAR(RotorAngle::FromRaw(INT32_MAX).AngleInRevolution().Value(), kPi, 1.0e-6F);
}

ZTEST(rotor_angle, test_from_angle_round_trips_within_resolution) {
  for (int i = -180; i < 180; ++i) {
    const float kExpected = (static_cast<float>(i) * kPi) / 180.0F;
    const RotorAngle kAngle = RotorAngle::FromAngle(Angle{kExpected});
    zexpect_equal(kAngle.Revolutions(), 0);
    UNIMOC_EXPECT_NEAR(kAngle.AngleInRevolution().Value(), kExpected, 1.0e-6F);
  }
}

ZTEST(rotor_angle, test_sine_and_cosine_match_standard_library) {
  constexpr int kSamples = 4096;
  for (int i = 0; i < kSamples; ++i) {
    const float kExpected = -kPi + ((kTwoPi * static_cast<float>(i)) / static_cast<float>(kSamples));
    const RotorAngle kAngle = RotorAngle::FromAngle(Angle{kExpected});
    UNIMOC_EXPECT_NEAR(kAngle.Sin().Value(),
                       std::sin(kExpected),
                       kTrigTolerance,
                       "i=%d",
                       static_cast<int>(i));
    UNIMOC_EXPECT_NEAR(kAngle.Cos().Value(),
                       std::cos(kExpected),
                       kTrigTolerance,
                       "i=%d",
                       static_cast<int>(i));
  }
}

ZTEST(rotor_angle, test_sine_and_cosine_follow_raw_wrap) {
  // The seam at +-pi must not introduce a sign flip.
  const RotorAngle kBelow = RotorAngle::FromRaw(INT32_MAX);
  const RotorAngle kAbove = RotorAngle::FromRaw(INT32_MIN);
  UNIMOC_EXPECT_NEAR(kBelow.Sin().Value(), kAbove.Sin().Value(), 1.0e-5F);
  UNIMOC_EXPECT_NEAR(kBelow.Cos().Value(), kAbove.Cos().Value(), 1.0e-5F);
  UNIMOC_EXPECT_NEAR(kAbove.Cos().Value(), -1.0F, kTrigTolerance);
}

ZTEST(rotor_angle, test_advance_counts_positive_wrap) {
  RotorAngle angle = RotorAngle::FromAngle(Angle{kPi - 0.1F});
  zassert_equal(angle.Revolutions(), 0);

  angle.Advance(0.2_rad);
  zexpect_equal(angle.Revolutions(), 1);
  UNIMOC_EXPECT_NEAR(angle.AngleInRevolution().Value(), -kPi + 0.1F, 1.0e-5F);
  UNIMOC_EXPECT_NEAR(angle.AbsoluteAngle().Value(), kPi + 0.1F, 1.0e-5F);
}

ZTEST(rotor_angle, test_advance_counts_negative_wrap) {
  RotorAngle angle = RotorAngle::FromAngle(Angle{-kPi + 0.1F});
  zassert_equal(angle.Revolutions(), 0);

  angle.Advance(-0.2_rad);
  zexpect_equal(angle.Revolutions(), -1);
  UNIMOC_EXPECT_NEAR(angle.AngleInRevolution().Value(), kPi - 0.1F, 1.0e-5F);
  UNIMOC_EXPECT_NEAR(angle.AbsoluteAngle().Value(), -kPi - 0.1F, 1.0e-5F);
}

ZTEST(rotor_angle, test_advance_accepts_many_revolutions_at_once) {
  RotorAngle angle;
  angle.Advance(100.5_rad * kTwoPi);
  zexpect_equal(angle.Revolutions(), 101);
  UNIMOC_EXPECT_NEAR(angle.AngleInRevolution().Value(), -kPi, 1.0e-3F);

  angle.Advance(-100.5_rad * kTwoPi);
  zexpect_equal(angle.Revolutions(), 0);
}

ZTEST(rotor_angle, test_repeated_advance_does_not_compound_error) {
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
  zexpect_equal(angle.Revolutions(), kRevolutions);
  zexpect_true((std::abs(static_cast<double>(angle.Raw()))) <= (0.5 * kSteps));
}

ZTEST(rotor_angle, test_absolute_raw_is_monotonic_across_wrap) {
  RotorAngle angle = RotorAngle::FromAngle(Angle{kPi - 0.05F});
  std::int64_t previous = angle.AbsoluteRaw();
  for (int i = 0; i < 100; ++i) {
    angle.Advance(0.01_rad);
    zexpect_true((angle.AbsoluteRaw()) > (previous));
    previous = angle.AbsoluteRaw();
  }
  zexpect_equal(angle.Revolutions(), 1);
}

ZTEST(rotor_angle, test_absolute_raw_matches_absolute_angle) {
  const RotorAngle kAngle = RotorAngle::FromAngle(7.0_rad);
  const double kExpected = static_cast<double>(kAngle.AbsoluteRaw()) *
                           (2.0 * std::numbers::pi_v<double> / static_cast<double>(kCountsPerRevolution));
  UNIMOC_EXPECT_NEAR(kAngle.AbsoluteAngle().Value(), static_cast<float>(kExpected), 1.0e-5F);
}

ZTEST(rotor_angle, test_difference_takes_shortest_path) {
  const RotorAngle kAhead = RotorAngle::FromAngle(Angle{kPi - 0.1F});
  const RotorAngle kBehind = RotorAngle::FromAngle(Angle{-kPi + 0.1F});

  UNIMOC_EXPECT_NEAR((kBehind - kAhead).Value(), 0.2F, 1.0e-5F);
  UNIMOC_EXPECT_NEAR((kAhead - kBehind).Value(), -0.2F, 1.0e-5F);
}

ZTEST(rotor_angle, test_difference_ignores_revolution_counter) {
  const RotorAngle kFirst = RotorAngle::FromRaw(0x2000'0000, 3);
  const RotorAngle kSecond = RotorAngle::FromRaw(0x2000'0000, -7);
  UNIMOC_EXPECT_FLOAT_EQ((kFirst - kSecond).Value(), 0.0F);
}

ZTEST(rotor_angle, test_offset_operators) {
  const RotorAngle kBase = RotorAngle::FromAngle(1.0_rad);

  RotorAngle mutated = kBase;
  mutated += 0.5_rad;
  UNIMOC_EXPECT_NEAR(mutated.AngleInRevolution().Value(), 1.5F, 1.0e-5F);
  mutated -= 0.5_rad;
  zexpect_equal(mutated, kBase);

  UNIMOC_EXPECT_NEAR((kBase + 0.25_rad).AngleInRevolution().Value(), 1.25F, 1.0e-5F);
  UNIMOC_EXPECT_NEAR((kBase - 0.25_rad).AngleInRevolution().Value(), 0.75F, 1.0e-5F);
  zexpect_equal(kBase, RotorAngle::FromAngle(1.0_rad));
}

ZTEST(rotor_angle, test_equality_compares_phase_and_revolutions) {
  zexpect_equal(RotorAngle::FromRaw(1234, 5), RotorAngle::FromRaw(1234, 5));
  zexpect_not_equal(RotorAngle::FromRaw(1234, 5), RotorAngle::FromRaw(1234, 6));
  zexpect_not_equal(RotorAngle::FromRaw(1234, 5), RotorAngle::FromRaw(1235, 5));
}

ZTEST(rotor_angle, test_from_absolute_raw_inverts_absolute_raw) {
  constexpr std::int64_t kRevolution = 0x1'0000'0000LL;
  constexpr std::int64_t kHalfRevolution = 0x8000'0000LL;
  const std::array<std::int64_t, 9> kPositions = {0,
                                                  1,
                                                  -1,
                                                  kHalfRevolution - 1,
                                                  kHalfRevolution,
                                                  -kHalfRevolution,
                                                  -kHalfRevolution - 1,
                                                  (7 * kRevolution) + 12345,
                                                  (-7 * kRevolution) - 12345};
  for (const std::int64_t kPosition : kPositions) {
    const RotorAngle kAngle = RotorAngle::FromAbsoluteRaw(kPosition);
    zexpect_equal(kAngle.AbsoluteRaw(), kPosition);
  }

  // +pi lands on the seam: phase -pi of the next revolution.
  zexpect_equal(RotorAngle::FromAbsoluteRaw(kHalfRevolution), RotorAngle::FromRaw(INT32_MIN, 1));
  zexpect_equal(RotorAngle::FromAbsoluteRaw(-kHalfRevolution), RotorAngle::FromRaw(INT32_MIN, 0));
}

ZTEST(rotor_angle, test_from_absolute_raw_scales_mechanical_to_electrical_exactly) {
  constexpr std::int64_t kPolePairs = 7;
  const RotorAngle kMechanical = RotorAngle::FromAngle(Angle{(3.0F * kTwoPi) + 0.3F});
  const RotorAngle kElectrical = RotorAngle::FromAbsoluteRaw(kMechanical.AbsoluteRaw() * kPolePairs);
  zexpect_equal(kElectrical.AbsoluteRaw(), kMechanical.AbsoluteRaw() * kPolePairs);
  UNIMOC_EXPECT_NEAR(kElectrical.AbsoluteAngle().Value(),
                     kMechanical.AbsoluteAngle().Value() * 7.0F,
                     1.0e-4F);
}

ZTEST_SUITE(rotor_angle, nullptr, nullptr, nullptr, nullptr, nullptr);
