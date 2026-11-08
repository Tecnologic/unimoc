/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file atan2_test.cpp
 *    @brief ztest coverage of the libm-free Atan2.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "atan2.hpp"
#include <zephyr/ztest.h>
#include <cmath>
#include <numbers>
#include "sin_cos.hpp"
#include "unimoc_test.hpp"

using namespace unimoc::unit;
using unimoc::observer::Atan2;

ZTEST_SUITE(atan2, nullptr, nullptr, nullptr, nullptr, nullptr);

ZTEST(atan2, test_the_angle_of_every_direction_is_within_1e_5_rad) {
  // Every 0.1 degree around the circle, with a flux vector of 20 mWb.
  for (int step = -1799; step <= 1800; ++step) {
    const Angle kAngle = (static_cast<float>(step) * 0.1F) * (std::numbers::pi_v<float> / 180.0F) * 1.0_rad;
    const unimoc::system::SinCos<Ratio> kSinCos(kAngle);
    const MagneticFlux kMagnitude = 0.02_Wb;
    const Angle kResult = Atan2(kMagnitude * kSinCos.sin, kMagnitude * kSinCos.cos);
    UNIMOC_EXPECT_NEAR(kResult, kAngle, 2e-5_rad);
  }
}

ZTEST(atan2, test_the_axes_and_the_zero_vector) {
  UNIMOC_EXPECT_NEAR(Atan2(0.0_A, 1.0_A), 0.0_rad, 1e-6_rad);
  UNIMOC_EXPECT_NEAR(Atan2(1.0_A, 0.0_A), 0.5F * std::numbers::pi_v<float> * 1.0_rad, 1e-5_rad);
  UNIMOC_EXPECT_NEAR(Atan2(-1.0_A, 0.0_A), -0.5F * std::numbers::pi_v<float> * 1.0_rad, 1e-5_rad);
  UNIMOC_EXPECT_NEAR(Atan2(0.0_A, -1.0_A), std::numbers::pi_v<float> * 1.0_rad, 1e-5_rad);
  UNIMOC_EXPECT_FLOAT_EQ(Atan2(0.0_A, 0.0_A), 0.0_rad);
}

ZTEST(atan2, test_it_matches_std_atan2_at_random_points) {
  // A fixed pseudo-random sequence (LCG), vectors of different lengths.
  uint32_t state = 12345U;
  for (int i = 0; i < 1000; ++i) {
    state = state * 1664525U + 1013904223U;
    const float kXValue = static_cast<float>(static_cast<int32_t>(state >> 8U) % 2001 - 1000) * 0.01F;
    state = state * 1664525U + 1013904223U;
    const float kYValue = static_cast<float>(static_cast<int32_t>(state >> 8U) % 2001 - 1000) * 0.01F;
    const Angle kExpected = std::atan2(kYValue, kXValue) * 1.0_rad;
    UNIMOC_EXPECT_NEAR(Atan2(kYValue * 1.0_V, kXValue * 1.0_V), kExpected, 2e-5_rad);
  }
}
