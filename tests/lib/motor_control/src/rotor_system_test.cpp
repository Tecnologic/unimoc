/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file rotor_system_test.cpp
 *    @brief ztest coverage of the rotor (d/q) reference frame.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "rotor_system.hpp"
#include <zephyr/ztest.h>
#include <concepts>
#include "rotor_angle.hpp"
#include "unimoc_test.hpp"

using Rotor = unimoc::system::Rotor<unimoc::unit::Ratio>;
using Stator = unimoc::system::Stator<unimoc::unit::Ratio>;
using RotorAngle = unimoc::system::RotorAngle;

ZTEST(rotor_system, test_to_stator_at_zero_angle) {
  const Rotor kRotor{1.0F, 2.0F};
  const RotorAngle kAngle;

  const auto kStator = kRotor.ToStator(kAngle);

  UNIMOC_EXPECT_NEAR(kStator.alpha.Value(), 1.0F, 1.0e-5F);
  UNIMOC_EXPECT_NEAR(kStator.beta.Value(), 2.0F, 1.0e-5F);
}

ZTEST(rotor_system, test_to_stator_at_quarter_turn) {
  const Rotor kRotor{1.0F, 2.0F};
  const RotorAngle kAngle = RotorAngle::FromRaw(0x4000'0000);

  const auto kStator = kRotor.ToStator(kAngle);

  UNIMOC_EXPECT_NEAR(kStator.alpha.Value(), -2.0F, 1.0e-5F);
  UNIMOC_EXPECT_NEAR(kStator.beta.Value(), 1.0F, 1.0e-5F);
}

ZTEST(rotor_system, test_length_squared_avoids_square_root) {
  constexpr Rotor kRotor{3.0F, 4.0F};

  static_assert(std::same_as<decltype(kRotor.LengthSquared()), float>);
  static_assert(kRotor.LengthSquared() == 25.0F);
  UNIMOC_EXPECT_FLOAT_EQ(kRotor.LengthSquared(), 25.0F);
}

ZTEST_SUITE(rotor_system, nullptr, nullptr, nullptr, nullptr, nullptr);
