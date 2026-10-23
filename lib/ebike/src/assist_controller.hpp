/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file assist_controller.hpp
 *    @brief Pedal assist law: motor torque from pedal torque and power, assist level and limits.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include "units.hpp"

/**
 * @namespace unimoc::ebike e-bike application logic namespace
 */
namespace unimoc::ebike {

using namespace unit;

/// Number of assist levels the display can select (level 0 is "off").
inline constexpr std::size_t kAssistLevels = 8U;

/**
 * @brief Parameters of the assist law.
 *
 * The defaults are a starting point for a first ride; none is tuned.
 */
struct AssistConfig {
  /// Base assist ratio (motor power / pedal power) per assist level; level 0 is off.
  std::array<Ratio, kAssistLevels>
      level_gain{0.0_ratio, 0.3_ratio, 0.6_ratio, 1.0_ratio, 1.5_ratio, 2.0_ratio, 2.0_ratio, 2.0_ratio};
  /// The ratio grows with exp(pedal power / power_scale): the assist rises with the effort.
  Power power_scale{300.0_W};
  /// Upper limit of the total assist ratio (base ratio times the exponential).
  Ratio max_ratio{4.0_ratio};
  /// Motor torque per pedal torque at standstill of the motor (low-speed torque assist). Above the speed at
  /// which the motor outruns the crank by this ratio, the law holds the motor power at ratio times the pedal
  /// power instead.
  Ratio torque_ratio{0.5_ratio};
  /// Pedal torque below this gives no assist (sensor noise, a foot resting on the pedal).
  Torque torque_deadband{2.0_Nm};
  /// No assist below this cadence.
  AngularVelocity min_cadence{2.0_rad_per_s};
  /// The assist fades out over this motor speed range below the speed limit.
  AngularVelocity speed_taper{20.0_rad_per_s};
  /// Time to ramp the output from zero to the torque limit.
  Time rise_time{0.5_s};
  /// Time to ramp the output from the torque limit to zero (the assist ends quickly when the pedaling stops).
  Time fall_time{0.15_s};
};

/// Inputs of the assist law for one step.
struct AssistInput {
  /// The crank turns (PedalObserver::Pedaling()).
  bool pedaling{false};
  /// Crank speed.
  AngularVelocity cadence;
  /// Mean pedal torque, offset removed.
  Torque pedal_torque;
  /// Motor shaft speed, either direction.
  AngularVelocity motor_speed;
  /// Motor shaft speed at the speed limit of the display; zero or less gives no assist.
  AngularVelocity speed_limit;
  /// Largest motor torque (current limit of the display and of the drive).
  Torque torque_limit;
  /// Assist level selected on the display.
  uint8_t level{0U};
};

/**
 * @brief Assist law of the e-bike.
 *
 * With pedal torque \f$\tau_p\f$ (after the deadband) and cadence \f$\omega_c\f$ the assist ratio is
 * \f$g = \min(g_{level} \cdot e^{P_p / P_{scale}}, g_{max})\f$ with \f$P_p = \tau_p \omega_c\f$. The motor
 * torque is \f$\tau_m = g \cdot \tau_p \cdot \rho\f$, with \f$\rho = \min(\rho_{max}, \omega_c /
 * |\omega_m|)\f$:
 *
 * - At low speed \f$\rho\f$ is the constant \f$\rho_{max}\f$: the motor torque follows the pedal torque.
 * - At higher speed \f$\rho = \omega_c / |\omega_m|\f$: the motor power \f$\tau_m \omega_m\f$ equals \f$g\f$
 * times the pedal power.
 *
 * The change at the crossover speed is continuous. The result fades out towards the speed limit, is limited
 * to the torque limit and ramps (slowly up, quickly down), so it never steps. Everything is zero without
 * pedaling, below the minimum cadence, at level 0 and without a valid speed or torque limit.
 *
 * Call Step() at a fixed rate from one thread (the control thread, not the hot path).
 */
class AssistController {
 public:
  /**
   * @brief Creates the controller.
   * @param config Parameters of the law.
   */
  explicit AssistController(const AssistConfig& config = AssistConfig{}) noexcept : config_(config) {}

  /**
   * @brief Advances the law by one step.
   * @param dt Time since the last step.
   * @param input Pedal, motor and limit values.
   * @return Motor torque setpoint, zero or more in the driving direction.
   */
  Torque Step(const Time dt, const AssistInput& input) noexcept {
    const Torque kTarget = Target(input);
    const Torque kLimit = std::max(input.torque_limit, Torque{});
    const Torque kUp = kLimit * (dt / config_.rise_time);
    // A lowered limit still ramps down in the fall time.
    const Torque kDown = std::max(kLimit, torque_) * (dt / config_.fall_time);
    if (kTarget > torque_) {
      torque_ = std::min(torque_ + kUp, kTarget);
    } else {
      torque_ = std::max(torque_ - kDown, kTarget);
    }
    return torque_;
  }

  /// Motor torque setpoint of the last step.
  [[nodiscard]] Torque MotorTorque() const noexcept { return torque_; }
  /// The assist is delivering torque.
  [[nodiscard]] bool Active() const noexcept { return torque_ > Torque{}; }
  /// Assist ratio (motor power / pedal power) of the last step's target, before the ramp.
  [[nodiscard]] Ratio AssistRatio() const noexcept { return ratio_; }

 private:
  static constexpr float kMaxExponent = 20.0F;

  // Target torque of the law before the ramp; zero when the assist is off.
  Torque Target(const AssistInput& input) noexcept {
    ratio_ = Ratio{};
    const std::size_t kLevel = std::min<std::size_t>(input.level, kAssistLevels - 1U);
    const Torque kDriving = input.pedal_torque - config_.torque_deadband;
    if (!input.pedaling || kLevel == 0U || input.cadence < config_.min_cadence || kDriving <= Torque{} ||
        input.speed_limit <= AngularVelocity{} || input.torque_limit <= Torque{}) {
      return Torque{};
    }
    const Power kPedalPower = kDriving * input.cadence;
    // The exponent is capped so a huge power cannot overflow to infinity (and give NaN with a zero gain).
    const Ratio kBoost{std::exp(std::min((kPedalPower / config_.power_scale).Value(), kMaxExponent))};
    ratio_ = std::min(kBoost * config_.level_gain[kLevel], config_.max_ratio);

    const AngularVelocity kMotor =
        input.motor_speed < AngularVelocity{} ? -input.motor_speed : input.motor_speed;
    Ratio speed_ratio = config_.torque_ratio;
    if (kMotor * config_.torque_ratio > input.cadence) {
      speed_ratio = input.cadence / kMotor;
    }
    const Ratio kFade = std::clamp((input.speed_limit - kMotor) / config_.speed_taper, Ratio{}, 1.0_ratio);
    return std::min(kDriving * ratio_ * speed_ratio * kFade, input.torque_limit);
  }

  AssistConfig config_;
  Torque torque_;
  Ratio ratio_;
};

}  // namespace unimoc::ebike
