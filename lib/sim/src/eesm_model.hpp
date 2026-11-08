/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file eesm_model.hpp
 *	@brief Electrical d/q model of an electrically excited synchronous motor with its field winding.
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
 * @brief Electrical parameters of an EESM: stator d/q with saliency and the field winding.
 */
struct EesmParameters {
  /// Pole-pair count.
  std::uint8_t pole_pairs{1U};
  /// Stator phase resistance R_s.
  unit::Resistance stator_resistance{0.1_Ohm};
  /// d-axis inductance L_d.
  unit::Inductance l_d{1.0_mH};
  /// q-axis inductance L_q.
  unit::Inductance l_q{1.0_mH};
  /// Mutual inductance L_m between the field winding and the stator d axis: psi_f = L_m i_f at i_d = 0.
  unit::Inductance l_m{0.5_mH};
  /// Field winding resistance R_f.
  unit::Resistance field_resistance{2.0_Ohm};
  /// Field winding self inductance L_f; 3/2 L_m^2 < L_d L_f.
  unit::Inductance field_inductance{50.0_mH};
};

/**
 * @brief EESM in the rotor d/q frame (d along the field winding).
 *
 * With the flux linkages as states and the amplitude-invariant Clarke transform:
 *
 *   dpsi_d/dt  = v_d - R_s i_d + omega_e psi_q
 *   dpsi_q/dt  = v_q - R_s i_q - omega_e psi_d
 *   dpsi_fw/dt = v_f - R_f i_f
 *   psi_d  = L_d i_d + L_m i_f,   psi_q = L_q i_q,   psi_fw = L_f i_f + 3/2 L_m i_d
 *   T_em   = 3/2 p (psi_d i_q - psi_q i_d)
 *
 * The 3/2 of the field linkage keeps the mutual coupling reciprocal in the
 * amplitude-invariant frame (the power of the d axis is 3/2 v_d i_d). The
 * field voltage comes from the excitation module (ElectricalInput::field_voltage);
 * the field current is an output.
 *
 * Each step is a backward-Euler step with the speed held constant. An open
 * stator forces the stator current to zero; the field winding keeps following
 * its voltage. Saturation and damper windings are not modelled.
 */
class EesmModel {
 public:
  constexpr EesmModel() = default;

  /// @param parameters Electrical parameters; check them with Validate().
  explicit constexpr EesmModel(const EesmParameters& parameters) : parameters_(parameters) {}

  [[nodiscard]] static constexpr system::MotorType Type() noexcept { return system::MotorType::kEesm; }

  /// @return Status::kOk or the first invalid parameter.
  [[nodiscard]] constexpr Status Validate() const noexcept {
    if (parameters_.pole_pairs == 0U) {
      return Status::kInvalidPolePairs;
    }
    if (parameters_.stator_resistance <= unit::Resistance{} ||
        parameters_.field_resistance <= unit::Resistance{}) {
      return Status::kInvalidResistance;
    }
    if (parameters_.l_d <= unit::Inductance{} || parameters_.l_q <= unit::Inductance{} ||
        parameters_.l_m <= unit::Inductance{} || parameters_.field_inductance <= unit::Inductance{} ||
        !(Determinant() > InductanceSquared{})) {
      return Status::kInvalidInductance;
    }
    return Status::kOk;
  }

  [[nodiscard]] constexpr const EesmParameters& Parameters() const noexcept { return parameters_; }

  /// Sets all flux linkages (and so the currents) to zero.
  constexpr void Reset() noexcept {
    flux_ = {};
    field_flux_ = unit::MagneticFlux{};
  }

  /**
   * @brief Integrates the flux linkages over one step.
   * @param input Stator and field voltage, electrical rotor angle and speed of the step.
   * @param time_step Step size; must be positive.
   */
  constexpr void Step(const ElectricalInput& input, const unit::Time time_step) noexcept {
    if (!input.stator_connected) {
      // i_d = i_q = 0: L_f di_f/dt = v_f - R_f i_f.
      const unit::Current kField =
          ((parameters_.field_inductance * FieldCurrent()) + (input.field_voltage * time_step)) /
          (parameters_.field_inductance + (parameters_.field_resistance * time_step));
      SetOpenStator(kField);
      return;
    }
    const system::Rotor<unit::Voltage> kVoltage = input.stator_voltage.ToRotor(input.electrical_angle);
    const auto kDeterminant = Determinant();
    const unit::Inductance kFieldMutual = parameters_.l_m * kThreeHalves;
    const unit::Ratio kStatorSelf =
        parameters_.stator_resistance * time_step * parameters_.field_inductance / kDeterminant;
    const unit::Ratio kStatorMutual =
        parameters_.stator_resistance * time_step * parameters_.l_m / kDeterminant;
    const unit::Ratio kFieldSelf = parameters_.field_resistance * time_step * parameters_.l_d / kDeterminant;
    const unit::Ratio kFieldCoupling = parameters_.field_resistance * time_step * kFieldMutual / kDeterminant;
    const unit::Ratio kRotation = input.electrical_speed * time_step / 1.0_rad;

    // Field: psi_fw' (1 + kFieldSelf) = psi_fw + v_f dt + kFieldCoupling psi_d'.
    const unit::Ratio kFieldDiagonal = 1.0_ratio + kFieldSelf;
    const unit::MagneticFlux kFieldRight = field_flux_ + (input.field_voltage * time_step);
    // d: A_d psi_d' - w psi_q' = r_d, with psi_fw' substituted; q: A_q psi_q' + w psi_d' = r_q.
    const unit::Ratio kDiagonalD =
        1.0_ratio + kStatorSelf - (kStatorMutual * (kFieldCoupling / kFieldDiagonal));
    const unit::Ratio kDiagonalQ = 1.0_ratio + (parameters_.stator_resistance * time_step / parameters_.l_q);
    const unit::MagneticFlux kRightD =
        flux_.d + (kVoltage.d * time_step) + (kFieldRight * (kStatorMutual / kFieldDiagonal));
    const unit::MagneticFlux kRightQ = flux_.q + (kVoltage.q * time_step);
    const unit::Ratio kSystem = (kDiagonalD * kDiagonalQ) + (kRotation * kRotation);
    flux_.d = ((kRightD * kDiagonalQ) + (kRightQ * kRotation)) / kSystem;
    flux_.q = ((kRightQ * kDiagonalD) - (kRightD * kRotation)) / kSystem;
    field_flux_ = (kFieldRight + (flux_.d * kFieldCoupling)) / kFieldDiagonal;
  }

  /// Stator current, torque and field current at the end of the last step.
  [[nodiscard]] constexpr ElectricalOutput Output() const noexcept {
    return ElectricalOutput{StatorCurrent(), ElectromagneticTorque(), FieldCurrent()};
  }

  /// Stator current: i_d = (L_f psi_d - L_m psi_fw) / D, i_q = psi_q / L_q.
  [[nodiscard]] constexpr system::Rotor<unit::Current> StatorCurrent() const noexcept {
    return {((flux_.d * parameters_.field_inductance) - (field_flux_ * parameters_.l_m)) / Determinant(),
            flux_.q / parameters_.l_q};
  }

  /// Field current: i_f = (L_d psi_fw - 3/2 L_m psi_d) / D.
  [[nodiscard]] constexpr unit::Current FieldCurrent() const noexcept {
    return ((field_flux_ * parameters_.l_d) - (flux_.d * (parameters_.l_m * kThreeHalves))) / Determinant();
  }

  /// Sets the field current of a stator without current (a test starting from an excited rotor).
  constexpr void SetFieldCurrent(const unit::Current field_current) noexcept { SetOpenStator(field_current); }

  /// T_em = 3/2 p (psi_d i_q - psi_q i_d).
  [[nodiscard]] constexpr unit::Torque ElectromagneticTorque() const noexcept {
    const system::Rotor<unit::Current> kCurrent = StatorCurrent();
    return ((flux_.d * kCurrent.q) - (flux_.q * kCurrent.d)) *
           (kThreeHalves * static_cast<float>(parameters_.pole_pairs));
  }

 private:
  static constexpr float kThreeHalves = 1.5F;

  /// D = L_d L_f - 3/2 L_m^2 of the d-axis / field system.
  [[nodiscard]] constexpr InductanceSquared Determinant() const noexcept {
    return (parameters_.l_d * parameters_.field_inductance) -
           ((parameters_.l_m * parameters_.l_m) * kThreeHalves);
  }

  constexpr void SetOpenStator(const unit::Current field_current) noexcept {
    flux_.d = parameters_.l_m * field_current;
    flux_.q = unit::MagneticFlux{};
    field_flux_ = parameters_.field_inductance * field_current;
  }

  EesmParameters parameters_;
  system::Rotor<unit::MagneticFlux> flux_;  ///< psi_d, psi_q
  unit::MagneticFlux field_flux_;           ///< psi_fw
};

static_assert(MotorModel<EesmModel>);

}  // namespace unimoc::sim
