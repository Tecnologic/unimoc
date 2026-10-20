/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file position_tracker.hpp
 *    @brief Mechanical position and homing adapter for an electrical RotorAngle.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include <numbers>
#include "rotor_angle.hpp"
#include "units.hpp"

/**
 * @namespace observer observer algorithms namespace
 */
namespace unimoc::observer {

/**
 * @brief Mechanical position and homing adapter for an electrical RotorAngle.
 *
 * RotorAngle owns electrical phase and revolution counting. This adapter
 * converts its absolute electrical position to mechanical position using the
 * pole-pair count and applies an independent mechanical home offset. Its
 * homed position is for the outer mechanical position loop only; electrical
 * transforms and current control must continue to use the unmodified
 * RotorAngle source.
 *
 */
struct PositionTracker {
  /// Returns the mechanical position relative to home [rad].
  [[nodiscard]] constexpr unit::Angle PositionRad() const noexcept { return position_rad_; }

  /// Returns the mechanical position relative to home [revolutions].
  [[nodiscard]] constexpr unit::Ratio PositionRev() const noexcept { return position_rev_; }

  /// Returns true after SetHome() and until Reset().
  [[nodiscard]] constexpr bool IsHomed() const noexcept { return is_homed_; }

  /**
   * @brief Convert the source's absolute electrical angle to mechanical position.
   *
   * Call once per control cycle before the position controller update. A zero
   * pole-pair count is ignored and leaves the last calculated position intact.
   *
   * @param kElectricalAngle Source angle; its phase and revolution count are
   *                         never modified by this adapter.
   * @param kPolePairs       Positive number of electrical revolutions per
   *                         mechanical revolution.
   */
  constexpr void Update(const system::RotorAngle& kElectricalAngle, const std::uint8_t kPolePairs) noexcept {
    if (kPolePairs == 0) {
      return;
    }

    raw_position_rad_ = kElectricalAngle.AbsoluteAngle() / static_cast<float>(kPolePairs);
    UpdatePosition();
  }

  /** @brief Latch the latest converted mechanical position as home. */
  constexpr void SetHome() noexcept {
    home_offset_rad_ = raw_position_rad_;
    is_homed_ = true;
    UpdatePosition();
  }

  /** @brief Clear the home reference and cached mechanical position. */
  constexpr void Reset() noexcept {
    raw_position_rad_ = unit::Angle{};
    home_offset_rad_ = unit::Angle{};
    position_rad_ = unit::Angle{};
    position_rev_ = unit::Ratio{};
    is_homed_ = false;
  }

 private:
  unit::Angle position_rad_;
  unit::Ratio position_rev_;
  bool is_homed_{false};
  unit::Angle raw_position_rad_;
  unit::Angle home_offset_rad_;

  constexpr void UpdatePosition() noexcept {
    constexpr unit::Angle kTwoPi{2.0F * std::numbers::pi_v<float>};
    position_rad_ = raw_position_rad_ - home_offset_rad_;
    position_rev_ = position_rad_ / kTwoPi;
  }
};

}  // namespace unimoc::observer
