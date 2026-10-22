/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file dc_link_limit.hpp
 *    @brief Battery current limits and the DC-link overvoltage derating of regenerative braking.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <limits>
#include "rotor_system.hpp"
#include "units.hpp"

/**
 * @namespace unimoc::control control algorithms namespace
 */
namespace unimoc::control {

/**
 * @brief Keeps the battery current of a PMSM drive within its limits.
 *
 * The battery current is estimated from the mechanical power of the q
 * current, I_bat = k_t i_q omega / V_dc with the torque constant
 * k_t = 3/2 p psi_pm. The estimate ignores the losses: when motoring the real
 * battery current is somewhat higher than the estimate, when braking lower,
 * so the charge limit errs on the safe side.
 *
 * Regenerative braking is derated with the DC-link voltage: the charge limit
 * falls linearly from its full value at `regen_derate_start` to zero at
 * `trip`, where the control cycle trips the power stage (section 7.1 of
 * docs/system_plan.md). Only the q current is scaled; the d current makes no
 * power in this estimate.
 */
struct DcLinkLimit {
  /// Largest battery discharge (motoring) current.
  unit::Current drive_current_max;
  /// Largest battery charge (regenerative) current.
  unit::Current charge_current_max;
  /// DC-link voltage above which the charge limit falls.
  unit::Voltage regen_derate_start;
  /// DC-link voltage at which no charge current is allowed.
  unit::Voltage trip;

  /**
   * @brief Charge current allowed at a DC-link voltage.
   * @param dc_link DC-link voltage.
   * @return The full charge limit up to `regen_derate_start`, zero from `trip`.
   */
  [[nodiscard]] constexpr unit::Current AllowedChargeCurrent(const unit::Voltage dc_link) const noexcept {
    if (!(dc_link > regen_derate_start)) {
      return charge_current_max;
    }
    if (!(dc_link < trip)) {
      return unit::Current{};
    }
    return charge_current_max * ((trip - dc_link) / (trip - regen_derate_start));
  }

  /// Range of the shaft torque.
  struct TorqueRange {
    unit::Torque lower;  ///< Most negative torque.
    unit::Torque upper;  ///< Most positive torque.
  };

  /**
   * @brief Shaft torque that keeps the battery current within the limits at a speed.
   *
   * The same power window as Limit(), as torque: for the output bounds of a
   * speed controller, so its integral does not wind up against the limit.
   *
   * @param speed Mechanical speed of the shaft.
   * @param dc_link DC-link voltage.
   * @return The torque range; unbounded at standstill or without a DC-link reading.
   */
  [[nodiscard]] constexpr TorqueRange AllowedTorque(const unit::AngularVelocity speed,
                                                    const unit::Voltage dc_link) const noexcept {
    constexpr float kInfinity = std::numeric_limits<float>::infinity();
    if (!(dc_link > unit::Voltage{}) || speed == unit::AngularVelocity{} || !unit::IsFinite(speed)) {
      return {unit::Torque{-kInfinity}, unit::Torque{kInfinity}};
    }
    const unit::Torque kMotoring = (dc_link * drive_current_max) / speed;
    const unit::Torque kBraking = -(dc_link * AllowedChargeCurrent(dc_link)) / speed;
    return (speed > unit::AngularVelocity{}) ? TorqueRange{kBraking, kMotoring}
                                             : TorqueRange{kMotoring, kBraking};
  }

  /**
   * @brief Scales the q current so the estimated battery current stays within the limits.
   * @param current Rotor-frame current reference.
   * @param torque_constant k_t = 3/2 p psi_pm (N m/A, the dimension of Wb).
   * @param speed Mechanical speed of the shaft.
   * @param dc_link DC-link voltage.
   * @return The reference with the q current scaled down where needed.
   */
  [[nodiscard]] constexpr system::Rotor<unit::Current> Limit(const system::Rotor<unit::Current>& current,
                                                             const unit::MagneticFlux torque_constant,
                                                             const unit::AngularVelocity speed,
                                                             const unit::Voltage dc_link) const noexcept {
    if (!(dc_link > unit::Voltage{})) {
      return current;  // no DC-link reading: the power stage makes no power either
    }
    const unit::Power kPower = (torque_constant * current.q) * speed;
    const unit::Power kMotoringMax = dc_link * drive_current_max;
    const unit::Power kBrakingMax = -(dc_link * AllowedChargeCurrent(dc_link));
    if (kPower > kMotoringMax) {
      return {current.d, current.q * (kMotoringMax / kPower)};
    }
    if (kPower < kBrakingMax) {
      return {current.d, current.q * (kBrakingMax / kPower)};
    }
    return current;
  }
};

}  // namespace unimoc::control
