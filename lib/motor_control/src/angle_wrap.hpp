/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file angle_wrap.hpp
 *    @brief Wrapping of angles and angle differences to one revolution.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cmath>
#include <numbers>
#include "units.hpp"

/**
 * @namespace unimoc::observer observer algorithms namespace
 */
namespace unimoc::observer {

/**
 * @brief Wraps an angle to (-pi, pi].
 *
 * Meant for angles and angle differences that are at most a few revolutions
 * away from the range (estimates, innovations); every step removes one
 * revolution. A non-finite angle is returned unchanged.
 *
 * @param angle Angle to wrap.
 * @return The equivalent angle in (-pi, pi].
 */
[[nodiscard]] constexpr unit::Angle WrapAngle(const unit::Angle angle) noexcept {
  constexpr float kPi = std::numbers::pi_v<float>;
  constexpr float kTwoPi = 2.0F * kPi;
  float value = angle.Value();
  if (!std::isfinite(value)) {
    return angle;
  }
  while (value > kPi) {
    value -= kTwoPi;
  }
  while (value <= -kPi) {
    value += kTwoPi;
  }
  return unit::Angle{value};
}

}  // namespace unimoc::observer
