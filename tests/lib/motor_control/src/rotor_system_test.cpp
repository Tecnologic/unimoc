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
using namespace unimoc::unit;

ZTEST(rotor_system, test_to_stator_at_zero_angle) {
  const Rotor kRotor{1.0F, 2.0F};
  const RotorAngle kAngle;

  const auto kStator = kRotor.ToStator(kAngle);

  UNIMOC_EXPECT_NEAR(kStator.alpha, 1.0_ratio, 1.0e-5_ratio);
  UNIMOC_EXPECT_NEAR(kStator.beta, 2.0_ratio, 1.0e-5_ratio);
}

ZTEST(rotor_system, test_to_stator_at_quarter_turn) {
  const Rotor kRotor{1.0F, 2.0F};
  const RotorAngle kAngle = RotorAngle::FromRaw(0x4000'0000);

  const auto kStator = kRotor.ToStator(kAngle);

  UNIMOC_EXPECT_NEAR(kStator.alpha, -2.0_ratio, 1.0e-5_ratio);
  UNIMOC_EXPECT_NEAR(kStator.beta, 1.0_ratio, 1.0e-5_ratio);
}

ZTEST(rotor_system, test_length_squared_avoids_square_root) {
  constexpr unimoc::system::Rotor kRotor{3.0_A, 4.0_A};

  static_assert(std::same_as<decltype(kRotor.LengthSquared()), decltype(Current{} * Current{})>);
  static_assert(kRotor.LengthSquared() == 5.0_A * 5.0_A);
  UNIMOC_EXPECT_FLOAT_EQ(kRotor.LengthSquared(), 5.0_A * 5.0_A);
}

ZTEST_SUITE(rotor_system, nullptr, nullptr, nullptr, nullptr, nullptr);
