/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file trapezoid_profile.hpp
 *    @brief Online trapezoidal speed profile of a positioning move.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include "units.hpp"

/**
 * @namespace unimoc::control control algorithms namespace
 */
namespace unimoc::control {

/**
 * @brief Online trapezoidal speed profile toward a target position.
 *
 * The drive plans its own trajectory when a position setpoint arrives without
 * one (README.md, section Positioning): the reference accelerates with
 * `accel_limit` up to `speed_limit`, cruises, and decelerates with
 * `accel_limit` so that it stops at the target. A short move stays a
 * triangle. The profile is planned anew every step from its own state
 * (position, speed), so a target that changes during a move, a lowered limit,
 * or a start from a moving shaft continue smoothly: the speed never changes
 * by more than `accel_limit · dt` per step.
 *
 * Discrete deceleration: in steps of Δv = a·dt, a stop from v = n·Δv needs
 * the distance Δv·dt·n(n+1)/2. The largest speed that still stops at the
 * remaining distance d is therefore v = Δv·(√(1/4 + 2d/(Δv·dt)) − 1/2), so
 * the reference reaches the target without overshoot. The last step snaps to
 * the target once the speed can stop within it.
 *
 * Without a usable limit (accel_limit or speed_limit not positive and finite)
 * the reference jumps to the target at once.
 */
struct TrapezoidProfile {
  /// Largest speed of the profile [rad/s].
  unit::AngularVelocity speed_limit;

  /// Acceleration and deceleration of the profile [rad/s²].
  unit::AngularAcceleration accel_limit;

  /// Position reference [rad].
  unit::Angle position;

  /// Speed reference: the speed feedforward of the position loop [rad/s].
  unit::AngularVelocity speed;

  /// Acceleration of the last step: the torque feedforward J·a [rad/s²].
  unit::AngularAcceleration acceleration;

  /**
   * @brief Starts the profile at a position and speed, normally the measured ones.
   * @param start Position to plan from [rad].
   * @param start_speed Speed to plan from [rad/s].
   */
  constexpr void Reset(const unit::Angle start,
                       const unit::AngularVelocity start_speed = unit::AngularVelocity{}) noexcept {
    position = start;
    speed = unit::IsFinite(start_speed) ? start_speed : unit::AngularVelocity{};
    acceleration = unit::AngularAcceleration{};
  }

  /// True when the reference rests at @p target.
  [[nodiscard]] constexpr bool AtTarget(const unit::Angle target) const noexcept {
    return position == target && speed == unit::AngularVelocity{};
  }

  /**
   * @brief Advances the reference by one step toward @p target.
   * @param target Target position [rad]; a non-finite target holds the reference.
   * @param dt Step period.
   */
  void Step(const unit::Angle target, const unit::Time dt) noexcept {
    if (!unit::IsFinite(target) || !(dt > unit::Time{})) {
      acceleration = unit::AngularAcceleration{};
      return;
    }
    const bool kLimited = speed_limit > unit::AngularVelocity{} && unit::IsFinite(accel_limit) &&
                          accel_limit > unit::AngularAcceleration{};
    if (!kLimited) {
      position = target;
      speed = unit::AngularVelocity{};
      acceleration = unit::AngularAcceleration{};
      return;
    }
    const unit::AngularVelocity kDeltaSpeed = accel_limit * dt;
    const unit::Angle kDistance = target - position;
    const unit::Angle kRemaining = unit::Abs(kDistance);

    // The largest speed that still stops at the target (discrete deceleration).
    using unit::operator""_ratio;
    const unit::Ratio kStopSteps =
        unit::Sqrt(0.25_ratio + (2.0F * kRemaining / (kDeltaSpeed * dt))) - 0.5_ratio;
    const unit::AngularVelocity kCruise = unit::Min(speed_limit, kDeltaSpeed * kStopSteps);
    const unit::AngularVelocity kWanted = (kDistance >= unit::Angle{}) ? kCruise : -kCruise;
    const unit::AngularVelocity kNext = kWanted.Clamp(speed - kDeltaSpeed, speed + kDeltaSpeed);

    // Arrival: this step reaches the target, and the speed can stop within it.
    const unit::Angle kTravel = kNext * dt;
    const bool kReaches = (kDistance >= unit::Angle{}) ? kTravel >= kDistance : kTravel <= kDistance;
    if (kReaches && unit::Abs(speed) <= kDeltaSpeed) {
      acceleration = (unit::AngularVelocity{} - speed) / dt;
      position = target;
      speed = unit::AngularVelocity{};
      return;
    }
    acceleration = (kNext - speed) / dt;
    speed = kNext;
    position += kTravel;
  }
};

}  // namespace unimoc::control
