/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file mechanical_model.hpp
 *	@brief Rigid single-mass shaft model with load torque and friction.
 *
 *	SPDX-License-Identifier: GPL-3.0-or-later
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */
#pragma once

#include "motor_model.hpp"
#include "rotor_angle.hpp"
#include "units.hpp"

namespace unimoc::sim {

/**
 * @brief How the shaft speed is determined.
 */
enum class ShaftMode : std::uint8_t {
  /// The speed follows the torque balance of the inertia.
  kFree,
  /// The speed is imposed, as on a dynamometer; zero locks the rotor.
  kImposedSpeed,
};

/**
 * @brief Mechanical parameters of motor and load.
 */
struct MechanicalParameters {
  /// Rotor and load inertia J.
  unit::Inertia inertia{unit::Inertia{1.0e-4F}};
  /// Viscous friction coefficient B.
  unit::TorquePerAngularVelocity viscous_friction;
  /// Coulomb (dry) friction torque magnitude.
  unit::Torque coulomb_friction;
};

/**
 * @brief Rigid shaft: J domega/dt = T_em - T_load - B omega - T_c sign(omega).
 *
 * The speed is integrated with an explicit Euler step and the angle with the
 * mean speed of the step. The mechanical angle is a RotorAngle, so the position
 * accumulates exactly and electrical angles are derived from it by integer
 * pole-pair scaling (ElectricalAngle()).
 *
 * Coulomb friction holds the shaft at standstill until the driving torque
 * exceeds it, and never reverses the direction of motion within a step.
 */
class MechanicalModel {
 public:
  /** @brief Constructs a model with default parameters. */
  constexpr MechanicalModel() = default;

  /**
   * @brief Constructs a model with the given parameters.
   * @param parameters Mechanical parameters; check them with Validate().
   */
  explicit constexpr MechanicalModel(const MechanicalParameters& parameters) : parameters_(parameters) {}

  /**
   * @brief Checks the parameters.
   * @return Status::kOk or the first invalid parameter.
   */
  [[nodiscard]] constexpr Status Validate() const noexcept {
    if (parameters_.inertia <= unit::Inertia{}) {
      return Status::kInvalidInertia;
    }
    if (parameters_.viscous_friction < unit::TorquePerAngularVelocity{} ||
        parameters_.coulomb_friction < unit::Torque{}) {
      return Status::kInvalidFriction;
    }
    return Status::kOk;
  }

  /**
   * @brief Returns the parameters.
   * @return The mechanical parameters.
   */
  [[nodiscard]] constexpr const MechanicalParameters& Parameters() const noexcept { return parameters_; }

  /** @brief Sets angle and speed to zero. The shaft mode and the imposed speed are kept. */
  constexpr void Reset() noexcept {
    angle_ = {};
    speed_ = {};
  }

  /**
   * @brief Selects how the speed is determined.
   * @param mode Free shaft or imposed speed.
   * @param imposed_speed Mechanical speed used in ShaftMode::kImposedSpeed.
   */
  constexpr void SetShaftMode(ShaftMode mode, unit::AngularVelocity imposed_speed = {}) noexcept {
    mode_ = mode;
    imposed_speed_ = imposed_speed;
    if (mode_ == ShaftMode::kImposedSpeed) {
      speed_ = imposed_speed_;
    }
  }

  /**
   * @brief Returns the shaft mode.
   * @return The active shaft mode.
   */
  [[nodiscard]] constexpr ShaftMode Mode() const noexcept { return mode_; }

  /**
   * @brief Sets the mechanical state, for example to start from an operating point.
   * @param angle Mechanical angle.
   * @param speed Mechanical speed; ignored in ShaftMode::kImposedSpeed.
   */
  constexpr void SetState(const system::RotorAngle& angle, unit::AngularVelocity speed) noexcept {
    angle_ = angle;
    if (mode_ == ShaftMode::kFree) {
      speed_ = speed;
    }
  }

  /**
   * @brief Integrates speed and angle over one step.
   * @param electromagnetic_torque Torque of the motor on the shaft.
   * @param load_torque External load torque; positive opposes positive rotation.
   * @param time_step Step size; must be positive.
   */
  constexpr void Step(unit::Torque electromagnetic_torque,
                      unit::Torque load_torque,
                      unit::Time time_step) noexcept {
    const unit::AngularVelocity kStartSpeed = speed_;
    if (mode_ == ShaftMode::kImposedSpeed) {
      speed_ = imposed_speed_;
    } else {
      speed_ = FreeSpeed(electromagnetic_torque - load_torque, time_step);
    }
    angle_.Advance((kStartSpeed + speed_) * time_step * 0.5F);
  }

  /**
   * @brief Returns the mechanical angle.
   * @return The shaft angle including whole revolutions.
   */
  [[nodiscard]] constexpr const system::RotorAngle& Angle() const noexcept { return angle_; }

  /**
   * @brief Returns the mechanical speed.
   * @return The shaft speed.
   */
  [[nodiscard]] constexpr unit::AngularVelocity Speed() const noexcept { return speed_; }

  /**
   * @brief Returns the electrical angle for a pole-pair count.
   * @param pole_pairs Pole-pair count of the motor.
   * @return The mechanical angle times the pole-pair count, exact in Q1.31 counts.
   */
  [[nodiscard]] constexpr system::RotorAngle ElectricalAngle(std::uint8_t pole_pairs) const noexcept {
    return system::RotorAngle::FromAbsoluteRaw(angle_.AbsoluteRaw() * static_cast<std::int64_t>(pole_pairs));
  }

 private:
  /**
   * @brief Computes the speed at the end of a step of the free shaft.
   * @param drive_torque Electromagnetic minus load torque.
   * @param time_step Step size.
   * @return The speed after viscous and Coulomb friction.
   */
  [[nodiscard]] constexpr unit::AngularVelocity FreeSpeed(unit::Torque drive_torque,
                                                          unit::Time time_step) const noexcept {
    const unit::Torque kCoulomb = parameters_.coulomb_friction;
    const unit::Torque kViscous = parameters_.viscous_friction * speed_;

    if (speed_ == unit::AngularVelocity{}) {
      // Static friction holds the shaft until the drive torque exceeds it.
      if (drive_torque > kCoulomb) {
        return ((drive_torque - kCoulomb) / parameters_.inertia) * time_step;
      }
      if (drive_torque < -kCoulomb) {
        return ((drive_torque + kCoulomb) / parameters_.inertia) * time_step;
      }
      return {};
    }

    const unit::Torque kFriction =
        (speed_ > unit::AngularVelocity{}) ? kViscous + kCoulomb : kViscous - kCoulomb;
    const unit::AngularVelocity kNext =
        speed_ + (((drive_torque - kFriction) / parameters_.inertia) * time_step);

    // Friction stops the shaft but never reverses it within one step.
    const bool kReversed = (speed_ > unit::AngularVelocity{}) ? (kNext < unit::AngularVelocity{})
                                                              : (kNext > unit::AngularVelocity{});
    const bool kFrictionStops = (drive_torque <= kCoulomb) && (drive_torque >= -kCoulomb);
    if (kReversed && kFrictionStops) {
      return {};
    }
    return kNext;
  }

  /// Mechanical parameters.
  MechanicalParameters parameters_;
  /// Shaft mode.
  ShaftMode mode_{ShaftMode::kFree};
  /// Speed imposed in ShaftMode::kImposedSpeed.
  unit::AngularVelocity imposed_speed_;
  /// Mechanical angle.
  system::RotorAngle angle_;
  /// Mechanical speed.
  unit::AngularVelocity speed_;
};

}  // namespace unimoc::sim
