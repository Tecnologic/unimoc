/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file motor_thermal_model.hpp
 *    @brief Two-mass thermal model of the motor winding, driven by the copper losses.
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

using unit::operator""_degC;
using unit::operator""_per_K;
using unit::operator""_ratio;
using unit::operator""_s;

/// Temperature coefficient of the resistance of copper near 20 °C.
inline constexpr unit::TemperatureCoefficient kCopperTemperatureCoefficient = 3.93e-3_per_K;

/**
 * @brief Estimates the winding temperature of a motor from its copper losses.
 *
 * Two thermal masses (docs/motor_thermal_plan.md, decision 1): the winding
 * (heat capacity C_w) and the stator with the housing (C_s), coupled by the
 * thermal resistance R_ws; the stator gives the heat to the ambient through
 * R_sa:
 *
 *     C_w dT_w/dt = P_cu - (T_w - T_s) / R_ws
 *     C_s dT_s/dt = (T_w - T_s) / R_ws - (T_s - T_amb) / R_sa
 *     P_cu        = 3/2 R_s(T_w) |i_dq|^2
 *     R_s(T)      = R_s,ref (1 + alpha (T - T_ref))
 *
 * The copper losses follow the winding temperature, so a hot winding heats
 * faster. Step() integrates with a backward Euler step: unconditionally
 * stable and exact in the steady state, so a long or irregular step of the
 * calling thread does no harm. Iron losses are not modelled.
 *
 * Step the model at about kStepPeriod with the mean |i_dq|^2 of that time,
 * not at the rate of the control thread: the temperatures are floats, and
 * at 2 kHz the change of a step is close to their resolution (about 8 uK at
 * 100 °C), so the slow stator mass would lose a good part of its heating to
 * rounding.
 *
 * The temperatures are in degrees Celsius. Correct() pulls the estimate
 * towards a measured winding temperature (from the stator resistance or a
 * sensor) and moves both masses by the same amount.
 */
class MotorThermalModel {
 public:
  /// Step period the model is meant for (see the class description).
  static constexpr unit::Time kStepPeriod = 0.1_s;

  /// Square of a current [A^2], the |i_dq|^2 of the copper losses.
  using CurrentSquared = decltype(unit::Current{} * unit::Current{});

  /// Motor parameters of the model.
  struct Parameters {
    /// Stator phase resistance R_s,ref at `reference_temperature`.
    unit::Resistance stator_resistance;
    /// Winding temperature T_ref at which `stator_resistance` was measured.
    unit::Temperature reference_temperature{25.0_degC};
    /// Temperature coefficient alpha of the winding resistance.
    unit::TemperatureCoefficient temperature_coefficient{kCopperTemperatureCoefficient};
    /// Heat capacity C_w of the winding.
    unit::HeatCapacity winding_capacity;
    /// Heat capacity C_s of the stator and the housing.
    unit::HeatCapacity stator_capacity;
    /// Thermal resistance R_ws from the winding to the stator.
    unit::ThermalResistance winding_to_stator;
    /// Thermal resistance R_sa from the stator to the ambient.
    unit::ThermalResistance stator_to_ambient;
    /// Winding temperature at which the current derating starts.
    unit::Temperature derate_start{100.0_degC};
    /// Winding temperature at which no current is allowed.
    unit::Temperature limit{130.0_degC};

    /// @return True when the capacities and the thermal resistances are positive and finite.
    [[nodiscard]] constexpr bool Valid() const noexcept {
      return Positive(stator_resistance) && Positive(winding_capacity) && Positive(stator_capacity) &&
             Positive(winding_to_stator) && Positive(stator_to_ambient) &&
             unit::IsFinite(reference_temperature) && unit::IsFinite(temperature_coefficient);
    }

   private:
    template <typename U>
    [[nodiscard]] static constexpr bool Positive(const U& value) noexcept {
      return unit::IsFinite(value) && value > U{};
    }
  };

  constexpr MotorThermalModel() noexcept = default;

  /**
   * @brief Sets the parameters and starts both masses at one temperature.
   * @param parameters Motor parameters.
   * @param start Temperature of the winding and the stator.
   */
  constexpr explicit MotorThermalModel(const Parameters& parameters,
                                       const unit::Temperature start = 25.0_degC) noexcept
      : parameters_(parameters), winding_(start), stator_(start) {}

  /**
   * @brief Replaces the parameters and keeps the temperatures (new settings of a warm motor).
   * @param parameters Motor parameters.
   */
  constexpr void SetParameters(const Parameters& parameters) noexcept { parameters_ = parameters; }

  /// @return The parameters of the model.
  [[nodiscard]] constexpr const Parameters& GetParameters() const noexcept { return parameters_; }

  /**
   * @brief Starts both masses at one temperature, such as the ambient after power-up.
   * @param start Temperature of the winding and the stator.
   */
  constexpr void Reset(const unit::Temperature start) noexcept {
    winding_ = start;
    stator_ = start;
  }

  /// @return The estimated winding temperature T_w.
  [[nodiscard]] constexpr unit::Temperature Winding() const noexcept { return winding_; }

  /// @return The estimated stator and housing temperature T_s.
  [[nodiscard]] constexpr unit::Temperature Stator() const noexcept { return stator_; }

  /**
   * @brief Stator resistance at a winding temperature.
   * @param temperature Winding temperature.
   * @return R_s,ref (1 + alpha (T - T_ref)), not below zero.
   */
  [[nodiscard]] constexpr unit::Resistance StatorResistanceAt(
      const unit::Temperature temperature) const noexcept {
    const unit::Ratio kScale =
        1.0_ratio + (parameters_.temperature_coefficient * (temperature - parameters_.reference_temperature));
    return parameters_.stator_resistance * unit::Max(kScale, 0.0_ratio);
  }

  /// @return The stator resistance at the estimated winding temperature.
  [[nodiscard]] constexpr unit::Resistance StatorResistance() const noexcept {
    return StatorResistanceAt(winding_);
  }

  /**
   * @brief Winding temperature that a measured stator resistance stands for.
   * @param resistance Measured stator phase resistance.
   * @return T_ref + (R / R_s,ref - 1) / alpha.
   */
  [[nodiscard]] constexpr unit::Temperature TemperatureFromResistance(
      const unit::Resistance resistance) const noexcept {
    return parameters_.reference_temperature +
           (((resistance / parameters_.stator_resistance) - 1.0_ratio) / parameters_.temperature_coefficient);
  }

  /**
   * @brief Copper losses of all three phases at the estimated winding temperature.
   * @param current_squared |i_dq|^2 of the amplitude-invariant rotor-frame current.
   * @return 3/2 R_s(T_w) |i_dq|^2.
   */
  [[nodiscard]] constexpr unit::Power CopperLoss(const CurrentSquared current_squared) const noexcept {
    return (StatorResistance() * current_squared) * 1.5F;
  }

  /**
   * @brief Advances the temperatures by one time step.
   *
   * The copper losses are taken at the winding temperature at the start of
   * the step. Invalid parameters or a step that is not positive leave the
   * state unchanged.
   *
   * @param current_squared Mean |i_dq|^2 over the step.
   * @param ambient Ambient (coolant) temperature.
   * @param dt Length of the step.
   */
  constexpr void Step(const CurrentSquared current_squared,
                      const unit::Temperature ambient,
                      const unit::Time dt) noexcept {
    if (!parameters_.Valid() || !(dt > unit::Time{}) || !unit::IsFinite(current_squared) ||
        !unit::IsFinite(ambient)) {
      return;
    }
    // Backward Euler: the linear 2x2 system of the conductances [W/K].
    const auto kWindingStorage = parameters_.winding_capacity / dt;
    const auto kStatorStorage = parameters_.stator_capacity / dt;
    const auto kCoupling = 1.0F / parameters_.winding_to_stator;
    const auto kCooling = 1.0F / parameters_.stator_to_ambient;
    const unit::Power kWindingSource = (kWindingStorage * winding_) + CopperLoss(current_squared);
    const unit::Power kStatorSource = (kStatorStorage * stator_) + (kCooling * ambient);
    const auto kWindingDiagonal = kWindingStorage + kCoupling;
    const auto kStatorDiagonal = kStatorStorage + kCoupling + kCooling;
    // (C_w/dt + 1/R_ws)(C_s/dt + 1/R_ws + 1/R_sa) - 1/R_ws^2, without the
    // cancellation of the 1/R_ws^2 terms (a long step leaves almost only them).
    const auto kDeterminant = (kWindingStorage * kStatorDiagonal) + (kCoupling * (kStatorStorage + kCooling));
    winding_ = ((kWindingSource * kStatorDiagonal) + (kCoupling * kStatorSource)) / kDeterminant;
    stator_ = ((kStatorSource * kWindingDiagonal) + (kCoupling * kWindingSource)) / kDeterminant;
  }

  /**
   * @brief Pulls the estimate towards a measured winding temperature.
   *
   * Both masses move by the same amount, so the modelled gradient between
   * winding and stator stays.
   *
   * @param measured Measured winding temperature.
   * @param weight Share of the difference taken over, clamped to [0, 1].
   */
  constexpr void Correct(const unit::Temperature measured, const unit::Ratio weight) noexcept {
    if (!unit::IsFinite(measured) || !unit::IsFinite(weight)) {
      return;
    }
    const unit::Temperature kShift = (measured - winding_) * weight.Clamp(0.0_ratio, 1.0_ratio);
    winding_ += kShift;
    stator_ += kShift;
  }

  /**
   * @brief Share of the motor current limit allowed at the estimated winding temperature.
   * @return 1 up to `derate_start`, falling linearly to 0 at `limit`.
   */
  [[nodiscard]] constexpr unit::Ratio CurrentLimitScale() const noexcept {
    if (!(winding_ > parameters_.derate_start)) {
      return 1.0_ratio;
    }
    if (!(winding_ < parameters_.limit)) {
      return 0.0_ratio;
    }
    return (parameters_.limit - winding_) / (parameters_.limit - parameters_.derate_start);
  }

 private:
  Parameters parameters_{};
  unit::Temperature winding_{25.0_degC};
  unit::Temperature stator_{25.0_degC};
};

}  // namespace unimoc::control
