/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file speed_controller.hpp
 *    @brief PI speed controller with a torque output and a ramped reference.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cmath>
#include "nvm_settings.hpp"
#include "units.hpp"

/**
 * @namespace unimoc::control control algorithms namespace
 */
namespace unimoc::control {

/**
 * @brief PI speed controller of the mechanical shaft speed with a torque output.
 *
 * The reference follows the setpoint with at most `accel_limit`; the PI works
 * on the error between the ramped reference and the measured speed:
 *
 *   T = kp · (e + ki · ∫e dt),   e = ω_ref − ω
 *
 * The output is limited to ±`torque_limit`. The integral stops while the
 * output is saturated in the direction of the error (conditional
 * integration), so a long saturation does not wind it up. The caller turns
 * the torque into a current reference (torque constant, MTPA).
 */
struct SpeedController {
  /// Proportional gain [N m per rad/s of mechanical speed].
  unit::TorquePerAngularVelocity kp;

  /// Integral gain 1/T_i [1/s]; 0 makes the controller proportional only.
  unit::InverseTime ki;

  /// Largest rate of change of the reference [rad/s²]; 0 or less: no ramp.
  unit::AngularAcceleration accel_limit;

  /// Output limit, symmetric [N m].
  unit::Torque torque_limit;

  /// Ramped speed reference [rad/s].
  unit::AngularVelocity reference;

  /// Integral part of the output [N m].
  unit::Torque integral;

  /// Output of the last Update() [N m].
  unit::Torque output;

  /**
   * @brief Loads the gains and the ramp from the settings and resets the state.
   *
   * The torque limit depends on the motor type and the current limit; the
   * caller sets it.
   *
   * @param settings Validated settings.
   */
  constexpr void Init(const settings::NvmSettings& settings) noexcept {
    kp = settings.speed_kp;
    ki = settings.speed_ki;
    accel_limit = settings.speed_accel_limit;
    Reset();
  }

  /**
   * @brief Clears the integral and the output.
   * @param start Reference to ramp from, normally the measured speed.
   */
  constexpr void Reset(const unit::AngularVelocity start = unit::AngularVelocity{}) noexcept {
    reference = start;
    integral = unit::Torque{};
    output = unit::Torque{};
  }

  /**
   * @brief Runs one step of the controller.
   * @param setpoint Speed setpoint [rad/s].
   * @param measured Measured or estimated speed [rad/s].
   * @param dt Time since the last step.
   * @return The torque reference, limited to ±torque_limit.
   */
  constexpr unit::Torque Update(const unit::AngularVelocity setpoint,
                                const unit::AngularVelocity measured,
                                const unit::Time dt) noexcept {
    if (!std::isfinite(setpoint.Value())) {
      return Track(setpoint, measured, dt);
    }
    if (accel_limit > unit::AngularAcceleration{}) {
      const unit::AngularVelocity kStep = accel_limit * dt;
      return Track(setpoint.Clamp(reference - kStep, reference + kStep), measured, dt);
    }
    return Track(setpoint, measured, dt);
  }

  /**
   * @brief Runs one step without the ramp: the reference is taken as it is.
   *
   * For a reference that is already limited, such as the speed demand of a
   * position loop.
   *
   * @param speed_reference Speed reference [rad/s].
   * @param measured Measured or estimated speed [rad/s].
   * @param dt Time since the last step.
   * @return The torque reference, limited to ±torque_limit.
   */
  constexpr unit::Torque Track(const unit::AngularVelocity speed_reference,
                               const unit::AngularVelocity measured,
                               const unit::Time dt) noexcept {
    if (!std::isfinite(speed_reference.Value()) || !std::isfinite(measured.Value())) {
      Reset(reference);  // keep the last finite reference, drop the output
      return output;
    }
    reference = speed_reference;

    const unit::Torque kLimit = torque_limit > unit::Torque{} ? torque_limit : unit::Torque{};
    const unit::AngularVelocity kError = reference - measured;
    const unit::Torque kProportional = kp * kError;
    const unit::Torque kIntegral = integral + kProportional * unit::Ratio{ki.Value() * dt.Value()};
    const unit::Torque kUnlimited = kProportional + kIntegral;
    const bool kWindsUp = (kUnlimited > kLimit && kError > unit::AngularVelocity{}) ||
                          (kUnlimited < -kLimit && kError < unit::AngularVelocity{});
    if (!kWindsUp) {
      integral = kIntegral.Clamp(-kLimit, kLimit);
    }
    output = (kProportional + integral).Clamp(-kLimit, kLimit);
    return output;
  }
};

}  // namespace unimoc::control
