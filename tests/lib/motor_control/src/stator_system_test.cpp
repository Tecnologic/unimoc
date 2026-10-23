/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file stator_system_test.cpp
 *    @brief ztest coverage of the stator (alpha/beta) reference frame.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "stator_system.hpp"
#include <zephyr/ztest.h>
#include <concepts>
#include "rotor_angle.hpp"
#include "rotor_system.hpp"
#include "three_phase_system.hpp"
#include "unimoc_test.hpp"

using Stator = unimoc::system::Stator<unimoc::unit::Ratio>;
using Rotor = unimoc::system::Rotor<unimoc::unit::Ratio>;
using RotorAngle = unimoc::system::RotorAngle;
using namespace unimoc::unit;

ZTEST(stator_system, test_to_rotor_at_zero_angle) {
  const Stator kStator{1.0F, 2.0F};
  const RotorAngle kAngle;

  const auto kRotor = kStator.ToRotor(kAngle);

  UNIMOC_EXPECT_NEAR(kRotor.d.Value(), 1.0F, 1.0e-5F);
  UNIMOC_EXPECT_NEAR(kRotor.q.Value(), 2.0F, 1.0e-5F);
}

ZTEST(stator_system, test_to_rotor_at_quarter_turn) {
  const Stator kStator{1.0F, 2.0F};
  const RotorAngle kAngle = RotorAngle::FromRaw(0x4000'0000);

  const auto kRotor = kStator.ToRotor(kAngle);

  UNIMOC_EXPECT_NEAR(kRotor.d.Value(), 2.0F, 1.0e-5F);
  UNIMOC_EXPECT_NEAR(kRotor.q.Value(), -1.0F, 1.0e-5F);
}

ZTEST(stator_system, test_park_and_inverse_park_round_trip) {
  const Stator kOriginal{1.25F, -0.75F};
  const RotorAngle kAngle = RotorAngle::FromAngle(0.6435011_rad);

  const Rotor kRotor = kOriginal.ToRotor(kAngle);
  const Stator kRestored = kRotor.ToStator(kAngle);

  UNIMOC_EXPECT_NEAR(kRestored.alpha.Value(), kOriginal.alpha.Value(), 1.0e-5F);
  UNIMOC_EXPECT_NEAR(kRestored.beta.Value(), kOriginal.beta.Value(), 1.0e-5F);
}

ZTEST(stator_system, test_park_transform_is_constexpr) {
  constexpr Stator kStator{1.0F, 2.0F};
  constexpr Rotor kRotor = kStator.ToRotor(RotorAngle{});

  static_assert(kRotor.d.Value() == 1.0F);
  static_assert(kRotor.q.Value() == 2.0F);
  UNIMOC_EXPECT_FLOAT_EQ(kRotor.d.Value(), 1.0F);
}

ZTEST(stator_system, test_length_squared_avoids_square_root) {
  constexpr unimoc::system::Stator kStator{3.0_A, 4.0_A};

  static_assert(std::same_as<decltype(kStator.LengthSquared()), decltype(Current{} * Current{})>);
  static_assert(kStator.LengthSquared() == 5.0_A * 5.0_A);
  UNIMOC_EXPECT_FLOAT_EQ(kStator.LengthSquared(), 5.0_A * 5.0_A);
}

ZTEST(stator_system, test_clarke_and_park_preserve_the_unit_type) {
  const unimoc::system::ThreePhase<unimoc::unit::Current> kPhase{1.0F, -0.5F, -0.5F};

  const unimoc::system::Stator<unimoc::unit::Current> kStator = kPhase.ToStator();
  const unimoc::system::Rotor<unimoc::unit::Current> kRotor = kStator.ToRotor(RotorAngle{});

  UNIMOC_EXPECT_NEAR(kRotor.d.Value(), 1.0F, 1.0e-5F);
}

ZTEST(stator_system, test_inverse_clarke_preserves_the_unit_type) {
  const unimoc::system::Stator<unimoc::unit::Current> kStator{1.0F, 0.0F};

  const auto kPhase = kStator.ToThreePhase();

  UNIMOC_EXPECT_NEAR(kPhase.a.Value(), 1.0F, 1.0e-5F);
  UNIMOC_EXPECT_NEAR(kPhase.b.Value(), -0.5F, 1.0e-5F);
  UNIMOC_EXPECT_NEAR(kPhase.c.Value(), -0.5F, 1.0e-5F);
}

ZTEST_SUITE(stator_system, nullptr, nullptr, nullptr, nullptr, nullptr);
