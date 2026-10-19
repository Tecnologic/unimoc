/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file pmsm_model.hpp
 *	@brief Electrical d/q model of a permanent-magnet synchronous motor.
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include "motor_model.hpp"
#include "rotor_system.hpp"
#include "units.hpp"

namespace unimoc::sim {

/**
 * @brief Electrical parameters of a PMSM.
 */
struct PmsmParameters {
  /// Pole-pair count.
  std::uint8_t pole_pairs{1U};
  /// Stator phase resistance R_s.
  unit::Resistance stator_resistance{unit::Resistance{0.1F}};
  /// d-axis inductance L_d.
  unit::Inductance l_d{unit::Inductance{1.0e-3F}};
  /// q-axis inductance L_q.
  unit::Inductance l_q{unit::Inductance{1.0e-3F}};
  /// Permanent-magnet flux linkage psi_PM (peak value per phase).
  unit::MagneticFlux flux_pm;
};

/**
 * @brief PMSM in the rotating d/q frame aligned with the magnet axis.
 *
 * The model uses the amplitude-invariant Clarke transform of the library:
 *
 *   L_d di_d/dt = v_d - R_s i_d + omega_e L_q i_q
 *   L_q di_q/dt = v_q - R_s i_q - omega_e (L_d i_d + psi_PM)
 *   T_em        = 3/2 p (psi_PM i_q + (L_d - L_q) i_d i_q)
 *
 * Each step is a backward-Euler step in the rotor frame with the speed held
 * constant. It is stable for any step size and speed, and a constant operating
 * point (DC in the rotor frame) is reproduced exactly. Transients are slightly
 * damped when the electrical angle per step grows.
 *
 * An open stator (bridge off) forces the currents to zero. Freewheeling through
 * the body diodes when the back-EMF exceeds the DC-link voltage is not modelled.
 * Magnetic saturation, cross saturation, iron losses and temperature are not
 * modelled either.
 */
class PmsmModel {
 public:
  /** @brief Constructs a model with default parameters. */
  constexpr PmsmModel() = default;

  /**
   * @brief Constructs a model with the given parameters.
   * @param parameters Electrical parameters; check them with Validate().
   */
  explicit constexpr PmsmModel(const PmsmParameters& parameters) : parameters_(parameters) {}

  /**
   * @brief Returns the simulated motor type.
   * @return MotorType::kPmsm.
   */
  [[nodiscard]] static constexpr system::MotorType Type() noexcept { return system::MotorType::kPmsm; }

  /**
   * @brief Checks the parameters.
   * @return Status::kOk or the first invalid parameter.
   */
  [[nodiscard]] constexpr Status Validate() const noexcept {
    if (parameters_.pole_pairs == 0U) {
      return Status::kInvalidPolePairs;
    }
    if (parameters_.stator_resistance <= unit::Resistance{}) {
      return Status::kInvalidResistance;
    }
    if (parameters_.l_d <= unit::Inductance{} || parameters_.l_q <= unit::Inductance{}) {
      return Status::kInvalidInductance;
    }
    if (parameters_.flux_pm < unit::MagneticFlux{}) {
      return Status::kInvalidFlux;
    }
    return Status::kOk;
  }

  /**
   * @brief Returns the parameters.
   * @return The electrical parameters.
   */
  [[nodiscard]] constexpr const PmsmParameters& Parameters() const noexcept { return parameters_; }

  /** @brief Sets the stator currents to zero. */
  constexpr void Reset() noexcept { current_ = {}; }

  /**
   * @brief Sets the stator current state, for example to start from an operating point.
   * @param current Stator current in the d/q frame.
   */
  constexpr void SetCurrent(const system::Rotor<unit::Current>& current) noexcept { current_ = current; }

  /**
   * @brief Integrates the stator currents over one step.
   * @param input Stator voltage, electrical angle and speed of the step.
   * @param time_step Step size; must be positive.
   */
  constexpr void Step(const ElectricalInput& input, unit::Time time_step) noexcept {
    if (!input.stator_connected) {
      current_ = {};
      return;
    }
    const system::Rotor<unit::Voltage> kVoltage = input.stator_voltage.ToRotor(input.electrical_angle);
    // Electrical angle per step; radians are dimensionless.
    const float kRotation = (input.electrical_speed * time_step).Value();
    const unit::Inductance kResistiveDt = parameters_.stator_resistance * time_step;

    // Backward Euler: solve
    //   | L_d + R dt   -w dt L_q   | |i_d'|   | L_d i_d + v_d dt            |
    //   | w dt L_d      L_q + R dt | |i_q'| = | L_q i_q + (v_q - w psi) dt  |
    const unit::Inductance kDiagonalD = parameters_.l_d + kResistiveDt;
    const unit::Inductance kCouplingQ = parameters_.l_q * kRotation;
    const unit::Inductance kCouplingD = parameters_.l_d * kRotation;
    const unit::Inductance kDiagonalQ = parameters_.l_q + kResistiveDt;
    const unit::MagneticFlux kFluxBalanceD = (parameters_.l_d * current_.d) + (kVoltage.d * time_step);
    const unit::MagneticFlux kFluxBalanceQ =
        (parameters_.l_q * current_.q) +
        ((kVoltage.q - (input.electrical_speed * parameters_.flux_pm)) * time_step);

    current_.d = (kFluxBalanceD + (kFluxBalanceQ * (kCouplingQ / kDiagonalQ))) /
                 (kDiagonalD + (kCouplingQ * (kCouplingD / kDiagonalQ)));
    current_.q = (kFluxBalanceQ - (kCouplingD * current_.d)) / kDiagonalQ;
  }

  /**
   * @brief Returns the stator current and the electromagnetic torque.
   * @return The electrical outputs at the end of the last step.
   */
  [[nodiscard]] constexpr ElectricalOutput Output() const noexcept {
    return ElectricalOutput{current_, ElectromagneticTorque(), unit::Current{}};
  }

  /**
   * @brief Returns the electromagnetic torque of the current state.
   * @return T_em = 3/2 p (psi_PM i_q + (L_d - L_q) i_d i_q).
   */
  [[nodiscard]] constexpr unit::Torque ElectromagneticTorque() const noexcept {
    const unit::MagneticFlux kFlux = parameters_.flux_pm + ((parameters_.l_d - parameters_.l_q) * current_.d);
    return kFlux * current_.q * (kThreeHalves * static_cast<float>(parameters_.pole_pairs));
  }

 private:
  /// Torque factor of the amplitude-invariant Clarke transform.
  static constexpr float kThreeHalves = 1.5F;

  /// Electrical parameters.
  PmsmParameters parameters_;
  /// Stator current in the d/q frame.
  system::Rotor<unit::Current> current_;
};

static_assert(MotorModel<PmsmModel>);

}  // namespace unimoc::sim
