/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file stator_resistance_estimator.hpp
 *    @brief Online estimate of the stator resistance at low speed, for the winding temperature.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include <optional>
#include "rotor_system.hpp"
#include "units.hpp"

/**
 * @namespace unimoc::control control algorithms namespace
 */
namespace unimoc::control {

/**
 * @brief Estimates the stator resistance of a synchronous machine in operation.
 *
 * The power balance of the rotor frame of a PMSM or EESM (the factor 3/2 of
 * the amplitude-invariant transform on both sides left out):
 *
 *     u_d i_d + u_q i_q = R_s |i|^2 + omega psi i_q + omega (L_d - L_q) i_d i_q + dW/dt
 *     W = 1/2 (L_d i_d^2 + L_q i_q^2)
 *
 * The estimator sums the left side minus the known terms and |i|^2 over a
 * window of consecutive control cycles and fits R_s through the origin
 * (docs/motor_thermal_plan.md, decision 3). The magnetic energy W of the
 * window's first and last sample removes the inductive term. Only cycles at
 * low electrical speed and with enough current count: there the resistive
 * voltage dominates the speed terms and the residual inverter error. One
 * cycle outside these bounds discards the window.
 *
 * Not for an induction machine (the rotor resistance and the slip enter the
 * balance). Runs in the observer after each control cycle; Add() is a few
 * multiplications.
 */
class StatorResistanceEstimator {
 public:
  /// Bounds and motor constants of the estimate.
  struct Parameters {
    /// Largest |omega_el| of a cycle that counts.
    unit::AngularVelocity speed_max;
    /// Smallest current magnitude of a cycle that counts.
    unit::Current current_min;
    /// d-axis inductance L_d.
    unit::Inductance inductance_d;
    /// q-axis inductance L_q.
    unit::Inductance inductance_q;
    /// Period of the control cycle (the spacing of the samples).
    unit::Time cycle;
    /// Consecutive valid cycles of a window.
    uint32_t window_cycles{1024U};
  };

  /// One control cycle.
  struct Sample {
    system::Rotor<unit::Voltage> voltage;  ///< Mean applied rotor-frame voltage.
    system::Rotor<unit::Current> current;  ///< Mean measured rotor-frame current.
    unit::AngularVelocity speed;           ///< Electrical angular velocity omega.
    unit::MagneticFlux flux;               ///< Rotor flux psi (PM or field) along d.
  };

  constexpr StatorResistanceEstimator() noexcept = default;

  /// @param parameters Bounds and motor constants.
  constexpr explicit StatorResistanceEstimator(const Parameters& parameters) noexcept
      : parameters_(parameters) {}

  /// @return The bounds and motor constants.
  [[nodiscard]] constexpr const Parameters& GetParameters() const noexcept { return parameters_; }

  /// Discards the window in progress.
  constexpr void Restart() noexcept {
    power_ = unit::Power{};
    current_squared_ = CurrentSquared{};
    first_energy_ = unit::Energy{};
    last_energy_ = unit::Energy{};
    count_ = 0U;
  }

  /**
   * @brief Adds one control cycle.
   * @param sample Voltage, current, speed and flux of the cycle.
   * @return The resistance of a window that this cycle completes; nothing otherwise.
   */
  [[nodiscard]] constexpr std::optional<unit::Resistance> Add(const Sample& sample) noexcept {
    const CurrentSquared kCurrentSquared = sample.current.LengthSquared();
    const bool kValid = unit::IsFinite(sample.speed) && unit::Abs(sample.speed) <= parameters_.speed_max &&
                        kCurrentSquared >= parameters_.current_min * parameters_.current_min &&
                        unit::IsFinite(kCurrentSquared) && unit::IsFinite(sample.voltage.d) &&
                        unit::IsFinite(sample.voltage.q);
    if (!kValid) {
      Restart();
      return std::nullopt;
    }
    const unit::Energy kEnergy = MagneticEnergy(sample.current);
    if (count_ == 0U) {
      first_energy_ = kEnergy;
    }
    last_energy_ = kEnergy;
    const unit::Power kInput = (sample.voltage.d * sample.current.d) + (sample.voltage.q * sample.current.q);
    const unit::Voltage kBackEmf = sample.speed * sample.flux;
    const unit::Voltage kSaliency =
        sample.speed * ((parameters_.inductance_d - parameters_.inductance_q) * sample.current.d);
    power_ += kInput - (kBackEmf * sample.current.q) - (kSaliency * sample.current.q);
    current_squared_ += kCurrentSquared;
    ++count_;
    if (count_ < parameters_.window_cycles) {
      return std::nullopt;
    }
    // The sum of dW/dt over the window is the change of W over one cycle period.
    const unit::Power kInductive = (last_energy_ - first_energy_) / parameters_.cycle;
    const unit::Resistance kResistance = (power_ - kInductive) / current_squared_;
    Restart();
    if (!unit::IsFinite(kResistance) || !(kResistance > unit::Resistance{})) {
      return std::nullopt;
    }
    return kResistance;
  }

 private:
  using CurrentSquared = decltype(unit::Current{} * unit::Current{});

  [[nodiscard]] constexpr unit::Energy MagneticEnergy(
      const system::Rotor<unit::Current>& current) const noexcept {
    return ((parameters_.inductance_d * (current.d * current.d)) +
            (parameters_.inductance_q * (current.q * current.q))) *
           0.5F;
  }

  Parameters parameters_{};
  unit::Power power_{};
  CurrentSquared current_squared_{};
  unit::Energy first_energy_{};
  unit::Energy last_energy_{};
  uint32_t count_{0U};
};

}  // namespace unimoc::control
