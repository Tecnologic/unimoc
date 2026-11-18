/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file resistance_regression.hpp
 *    @brief Stator resistance and dead-time voltage from (current, voltage) points.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include <optional>
#include "units.hpp"

namespace unimoc::identification {

/**
 * @brief Least squares of u = R i + U_dt sign(i) over measured operating points.
 *
 * Measurement M1 of the motor identification (docs/motor_identification_plan.md,
 * section 4.1): at standstill and a fixed angle the current controller holds
 * a d current at several levels of both signs; the mean voltage demand of each
 * level is the resistive drop plus the dead-time voltage, which changes sign
 * with the current. The slope is the stator resistance, the step at zero the
 * dead-time voltage (the old firmware's single point u / i carried it in R).
 *
 * The normal equations of the two regressors i and sign(i):
 *
 *   | sum i^2   sum |i| | | R    |   | sum u i       |
 *   | sum |i|   n       | | U_dt | = | sum u sign(i) |
 *
 * Points with zero current carry no sign and are ignored.
 */
class ResistanceRegression {
 public:
  /// Result of the fit.
  struct Fit {
    unit::Resistance resistance;      ///< Slope: the stator resistance.
    unit::Voltage dead_time_voltage;  ///< Intercept at a positive current: the dead-time voltage.
  };

  /** @brief Forgets all points. */
  constexpr void Reset() noexcept { *this = ResistanceRegression{}; }

  /**
   * @brief Adds one operating point.
   * @param current Mean current of the point (not zero).
   * @param voltage Mean voltage demand of the point.
   */
  constexpr void Add(unit::Current current, unit::Voltage voltage) noexcept {
    if (current == unit::Current{}) {
      return;
    }
    using unit::operator""_ratio;
    const unit::Ratio kSign = current > unit::Current{} ? 1.0_ratio : -1.0_ratio;
    sum_ii_ = sum_ii_ + (current * current);
    sum_abs_ = sum_abs_ + (current * kSign);
    sum_ui_ = sum_ui_ + (voltage * current);
    sum_us_ = sum_us_ + (voltage * kSign);
    if (current > unit::Current{}) {
      ++positive_;
    } else {
      ++negative_;
    }
  }

  /**
   * @brief Number of points added.
   * @return Points with a nonzero current.
   */
  [[nodiscard]] constexpr uint32_t Count() const noexcept { return positive_ + negative_; }

  /**
   * @brief Solves the fit.
   * @return The resistance and the dead-time voltage; empty without points of
   * both signs or with all points at one current magnitude (singular).
   */
  [[nodiscard]] constexpr std::optional<Fit> Solve() const noexcept {
    if (positive_ == 0U || negative_ == 0U) {
      return std::nullopt;
    }
    const auto kCount = static_cast<float>(Count());
    const auto kDeterminant = (sum_ii_ * kCount) - (sum_abs_ * sum_abs_);
    // Relative to sum i^2 * n: the magnitudes must differ (otherwise R and U_dt are not separable).
    if (!(kDeterminant > sum_ii_ * kCount * 1.0e-6F)) {
      return std::nullopt;
    }
    return Fit{((sum_ui_ * kCount) - (sum_abs_ * sum_us_)) / kDeterminant,
               ((sum_ii_ * sum_us_) - (sum_abs_ * sum_ui_)) / kDeterminant};
  }

 private:
  decltype(unit::Current{} * unit::Current{}) sum_ii_;
  unit::Current sum_abs_;
  unit::Power sum_ui_;
  unit::Voltage sum_us_;
  uint32_t positive_{0U};
  uint32_t negative_{0U};
};

}  // namespace unimoc::identification
