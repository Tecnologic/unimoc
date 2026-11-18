/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file atan2.hpp
 *    @brief The angle of a vector without libm (zero-latency and observer contexts).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <numbers>
#include "units.hpp"

namespace unimoc::observer {

namespace atan2_detail {

/// atan(z) for 0 <= z <= 1: Hastings' polynomial, error below 1e-5 rad.
[[nodiscard]] constexpr unit::Ratio AtanUnit(const unit::Ratio ratio) noexcept {
  using unit::operator""_ratio;
  const unit::Ratio kZ2 = ratio * ratio;
  return ratio * (0.9998660_ratio +
                  (kZ2 * (-0.3302995_ratio +
                          (kZ2 * (0.1801410_ratio + (kZ2 * (-0.0851330_ratio + (kZ2 * 0.0208351_ratio))))))));
}

}  // namespace atan2_detail

/**
 * @brief The angle of the vector (@p x_part, @p y_part), as std::atan2(y_part, x_part), without a library
 * call.
 *
 * Octant reduction and a polynomial for atan on [0, 1]: the error is below
 * 1e-5 rad over the whole circle. Both components share one unit (a flux, a
 * current, a ratio). The zero vector gives 0.
 *
 * @return The angle in [-pi, pi].
 */
template <typename Rep, typename Period, typename Dim>
[[nodiscard]] constexpr unit::Angle Atan2(const unit::Unit<Rep, Period, Dim>& y_part,
                                          const unit::Unit<Rep, Period, Dim>& x_part) noexcept {
  using unit::operator""_rad;
  constexpr unit::Angle kHalfPi = 0.5F * std::numbers::pi_v<float> * 1.0_rad;
  constexpr unit::Angle kPi = std::numbers::pi_v<float> * 1.0_rad;
  const unit::Unit<Rep, Period, Dim> kZero{};
  const unit::Unit<Rep, Period, Dim> kAbsX = unit::Abs(x_part);
  const unit::Unit<Rep, Period, Dim> kAbsY = unit::Abs(y_part);
  if (!(kAbsX > kZero) && !(kAbsY > kZero)) {
    return unit::Angle{};
  }
  // First octant: the smaller over the larger component.
  const bool kSteep = kAbsY > kAbsX;
  const unit::Ratio kRatio = kSteep ? kAbsX / kAbsY : kAbsY / kAbsX;
  unit::Angle angle = atan2_detail::AtanUnit(kRatio) * 1.0_rad;
  if (kSteep) {
    angle = kHalfPi - angle;
  }
  if (x_part < kZero) {
    angle = kPi - angle;
  }
  return (y_part < kZero) ? -angle : angle;
}

}  // namespace unimoc::observer
