/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file asm_model.hpp
 *	@brief Electrical d/q model of an asynchronous (induction) motor with a squirrel-cage rotor.
 *
 *	SPDX-License-Identifier: GPL-3.0-or-later
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

using unit::operator""_mH;
using unit::operator""_Ohm;
using unit::operator""_rad;
using unit::operator""_ratio;

/// L^2, the determinant of a coupled pair of windings.
using InductanceSquared = decltype(unit::Inductance{} * unit::Inductance{});

/**
 * @brief Electrical parameters of an induction motor (T equivalent circuit, per phase).
 */
struct AsmParameters {
  /// Pole-pair count.
  std::uint8_t pole_pairs{1U};
  /// Stator resistance R_s.
  unit::Resistance stator_resistance{0.5_Ohm};
  /// Rotor resistance R_r, referred to the stator.
  unit::Resistance rotor_resistance{0.3_Ohm};
  /// Stator self inductance L_s = L_m + L_sigma_s.
  unit::Inductance l_s{50.0_mH};
  /// Rotor self inductance L_r = L_m + L_sigma_r, referred to the stator.
  unit::Inductance l_r{50.0_mH};
  /// Magnetizing (mutual) inductance L_m; L_m^2 < L_s L_r.
  unit::Inductance l_m{47.0_mH};
};

/**
 * @brief Induction motor in the d/q frame that turns with the rotor.
 *
 * The frame of the simulation is the electrical rotor angle, as for the
 * synchronous motors: the rotor windings stand still in it, the stator turns
 * at -omega_e. With the flux linkages as states and the amplitude-invariant
 * Clarke transform:
 *
 *   dpsi_s/dt = v_s - R_s i_s - omega_e J psi_s
 *   dpsi_r/dt =     - R_r i_r
 *   psi_s = L_s i_s + L_m i_r,   psi_r = L_r i_r + L_m i_s
 *   T_em  = 3/2 p (psi_sd i_sq - psi_sq i_sd)
 *
 * (J turns a vector by +90 degrees). The slip is the difference between the
 * speed of the stator field and omega_e; at synchronous speed the rotor
 * current and the torque vanish.
 *
 * Each step is a backward-Euler step with the speed held constant: stable for
 * any step size and speed. An open stator forces the stator current to zero;
 * the rotor flux then decays with the rotor time constant L_r / R_r.
 * Saturation, iron losses, skin effect and temperature are not modelled.
 */
class AsmModel {
 public:
  constexpr AsmModel() = default;

  /// @param parameters Electrical parameters; check them with Validate().
  explicit constexpr AsmModel(const AsmParameters& parameters) : parameters_(parameters) {}

  [[nodiscard]] static constexpr system::MotorType Type() noexcept { return system::MotorType::kAsm; }

  /**
   * @brief Checks the parameters.
   * @return Status::kOk or the first invalid parameter (the leakage must be positive: L_m^2 < L_s L_r).
   */
  [[nodiscard]] constexpr Status Validate() const noexcept {
    if (parameters_.pole_pairs == 0U) {
      return Status::kInvalidPolePairs;
    }
    if (parameters_.stator_resistance <= unit::Resistance{} ||
        parameters_.rotor_resistance <= unit::Resistance{}) {
      return Status::kInvalidResistance;
    }
    if (parameters_.l_s <= unit::Inductance{} || parameters_.l_r <= unit::Inductance{} ||
        parameters_.l_m <= unit::Inductance{} || !(Determinant() > InductanceSquared{})) {
      return Status::kInvalidInductance;
    }
    return Status::kOk;
  }

  [[nodiscard]] constexpr const AsmParameters& Parameters() const noexcept { return parameters_; }

  /// Sets all flux linkages (and so the currents) to zero.
  constexpr void Reset() noexcept {
    stator_flux_ = {};
    rotor_flux_ = {};
  }

  /**
   * @brief Integrates the flux linkages over one step.
   * @param input Stator voltage, electrical rotor angle and speed of the step.
   * @param time_step Step size; must be positive.
   */
  constexpr void Step(const ElectricalInput& input, const unit::Time time_step) noexcept {
    const unit::Ratio kRotorDecay = parameters_.rotor_resistance * time_step / parameters_.l_r;
    if (!input.stator_connected) {
      // No stator current: psi_r = L_r i_r decays with R_r / L_r, psi_s = L_m / L_r psi_r.
      rotor_flux_.d = rotor_flux_.d / (1.0_ratio + kRotorDecay);
      rotor_flux_.q = rotor_flux_.q / (1.0_ratio + kRotorDecay);
      stator_flux_.d = rotor_flux_.d * (parameters_.l_m / parameters_.l_r);
      stator_flux_.q = rotor_flux_.q * (parameters_.l_m / parameters_.l_r);
      return;
    }
    const system::Rotor<unit::Voltage> kVoltage = input.stator_voltage.ToRotor(input.electrical_angle);
    const auto kDeterminant = Determinant();
    // Dimensionless step coefficients: dt R L / (L_s L_r - L_m^2).
    const unit::Ratio kStatorSelf =
        parameters_.stator_resistance * time_step * parameters_.l_r / kDeterminant;
    const unit::Ratio kStatorMutual =
        parameters_.stator_resistance * time_step * parameters_.l_m / kDeterminant;
    const unit::Ratio kRotorSelf = parameters_.rotor_resistance * time_step * parameters_.l_s / kDeterminant;
    const unit::Ratio kRotorMutual =
        parameters_.rotor_resistance * time_step * parameters_.l_m / kDeterminant;
    const unit::Ratio kRotation = input.electrical_speed * time_step / 1.0_rad;

    // Rotor: psi_r' (1 + kRotorSelf) = psi_r + kRotorMutual psi_s'.
    const unit::Ratio kRotorDiagonal = 1.0_ratio + kRotorSelf;
    // Stator, with psi_r' substituted: A psi_s' + w J psi_s' = rhs.
    const unit::Ratio kDiagonal = 1.0_ratio + kStatorSelf - (kStatorMutual * (kRotorMutual / kRotorDiagonal));
    const system::Rotor<unit::MagneticFlux> kRight{stator_flux_.d + (kVoltage.d * time_step) +
                                                       (rotor_flux_.d * (kStatorMutual / kRotorDiagonal)),
                                                   stator_flux_.q + (kVoltage.q * time_step) +
                                                       (rotor_flux_.q * (kStatorMutual / kRotorDiagonal))};
    // [A -w; w A] [d; q] = [rd; rq]
    const unit::Ratio kSystem = (kDiagonal * kDiagonal) + (kRotation * kRotation);
    stator_flux_.d = ((kRight.d * kDiagonal) + (kRight.q * kRotation)) / kSystem;
    stator_flux_.q = ((kRight.q * kDiagonal) - (kRight.d * kRotation)) / kSystem;
    rotor_flux_.d = (rotor_flux_.d + (stator_flux_.d * kRotorMutual)) / kRotorDiagonal;
    rotor_flux_.q = (rotor_flux_.q + (stator_flux_.q * kRotorMutual)) / kRotorDiagonal;
  }

  /// Stator current and electromagnetic torque at the end of the last step.
  [[nodiscard]] constexpr ElectricalOutput Output() const noexcept {
    return ElectricalOutput{StatorCurrent(), ElectromagneticTorque(), unit::Current{}};
  }

  /// Stator current from the flux linkages: i_s = (L_r psi_s - L_m psi_r) / (L_s L_r - L_m^2).
  [[nodiscard]] constexpr system::Rotor<unit::Current> StatorCurrent() const noexcept {
    const auto kDeterminant = Determinant();
    return {((stator_flux_.d * parameters_.l_r) - (rotor_flux_.d * parameters_.l_m)) / kDeterminant,
            ((stator_flux_.q * parameters_.l_r) - (rotor_flux_.q * parameters_.l_m)) / kDeterminant};
  }

  /// Rotor flux linkage in the rotor frame (simulation truth).
  [[nodiscard]] constexpr const system::Rotor<unit::MagneticFlux>& RotorFlux() const noexcept {
    return rotor_flux_;
  }

  /// T_em = 3/2 p (psi_sd i_sq - psi_sq i_sd).
  [[nodiscard]] constexpr unit::Torque ElectromagneticTorque() const noexcept {
    const system::Rotor<unit::Current> kCurrent = StatorCurrent();
    return ((stator_flux_.d * kCurrent.q) - (stator_flux_.q * kCurrent.d)) *
           (kThreeHalves * static_cast<float>(parameters_.pole_pairs));
  }

 private:
  static constexpr float kThreeHalves = 1.5F;

  [[nodiscard]] constexpr InductanceSquared Determinant() const noexcept {
    return (parameters_.l_s * parameters_.l_r) - (parameters_.l_m * parameters_.l_m);
  }

  AsmParameters parameters_;
  system::Rotor<unit::MagneticFlux> stator_flux_;
  system::Rotor<unit::MagneticFlux> rotor_flux_;
};

static_assert(MotorModel<AsmModel>);

}  // namespace unimoc::sim
