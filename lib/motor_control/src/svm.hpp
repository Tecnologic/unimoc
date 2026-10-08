/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file svm.hpp
 *    @brief Unit-typed space-vector PWM modulator.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <algorithm>
#include "nvm_settings.hpp"
#include "stator_system.hpp"
#include "three_phase_system.hpp"

/**
 * @namespace control control algorithms namespace
 */
namespace unimoc::control {

using namespace unit;

/**
 * @brief Space Vector Modulation (SVPWM) with centered PWM.
 *
 * Converts a stationary-frame voltage reference (α/β, normalised to the DC-link
 * voltage) into three PWM duty cycles.
 *
 * Conventions
 * -----------
 * - v_alpha and v_beta are normalised by the DC bus voltage V_dc so that
 *   a value of 1.0 represents V_dc.  The full linear SVPWM range corresponds to
 *   a vector magnitude of 1/√3 ≈ 0.577.
 * - The zero-sequence offset is chosen so that the sum of the three duty cycles
 *   is 1.5 before clamping (each centred around 0.5), giving centred / symmetric PWM.
 * - After centring the output duty cycles are clamped to [duty_min, duty_max]
 *   (default 5 % … 95 %) to leave headroom for current measurement and dead-time
 *   compensation without preloading the timer counter.  Clamping breaks the
 *   sum of 1.5 when the reference exceeds the linear range.
 */
struct Svm {
  /// Minimum duty cycle (keeps time for ADC sampling and dead-time headroom).
  unit::Ratio duty_min{0.05_ratio};
  /// Maximum duty cycle (symmetric headroom on the upper side).
  unit::Ratio duty_max{0.95_ratio};

  /**
   * @brief Load SVM duty limits from NVM settings.
   *
   * @param settings Validated NVM settings.
   */
  constexpr void Init(const settings::NvmSettings& settings) noexcept {
    duty_min = settings.svm_duty_min;
    duty_max = settings.svm_duty_max;
  }

  /**
   * @brief Compute three-phase duty cycles from phase voltage references.
   *
   * @param phase_ratios  Phase voltage references normalised by V_dc.
   * @return   Three-phase duty cycles [0, 1] clamped to [duty_min, duty_max].
   */
  [[nodiscard]] constexpr system::ThreePhase<unit::Ratio> Calculate(
      const system::ThreePhase<unit::Ratio>& phase_ratios) const noexcept {
    unit::Ratio phase_ratio_a = phase_ratios.a;
    unit::Ratio phase_ratio_b = phase_ratios.b;
    unit::Ratio phase_ratio_c = phase_ratios.c;

    // --- Zero-sequence injection for centred SVM ---
    // The zero-sequence component centres the modulated waveforms so that the
    // mid-point of (max + min) is always at 0.  Adding it to each phase shifts
    // all duties to be symmetric around 0.5.
    unit::Ratio ratio_max = std::max({phase_ratio_a, phase_ratio_b, phase_ratio_c});
    unit::Ratio ratio_min = std::min({phase_ratio_a, phase_ratio_b, phase_ratio_c});
    unit::Ratio ratio_mid = (ratio_max + ratio_min) * -0.5F;

    // Convert phase voltages [-0.5, 0.5] → duty cycles [0, 1]
    unit::Ratio duty_a = 0.5_ratio + phase_ratio_a + ratio_mid;
    unit::Ratio duty_b = 0.5_ratio + phase_ratio_b + ratio_mid;
    unit::Ratio duty_c = 0.5_ratio + phase_ratio_c + ratio_mid;

    // --- Clamp to [duty_min, duty_max] ---
    duty_a = duty_a.Clamp(duty_min.Value(), duty_max.Value());
    duty_b = duty_b.Clamp(duty_min.Value(), duty_max.Value());
    duty_c = duty_c.Clamp(duty_min.Value(), duty_max.Value());

    return system::ThreePhase<unit::Ratio>{duty_a, duty_b, duty_c};
  }

  /**
   * @brief Compute duty cycles from an alpha/beta voltage, the DC-link voltage and a dead-time correction.
   *
   * Normalises with a single reciprocal, adds the correction, applies the inverse
   * Clarke transform and modulates.
   *
   * @param voltage         Stationary-frame voltage vector.
   * @param dc_link_voltage DC-link voltage; values below 1 V are treated as 1 V to avoid division by zero.
   * @param dead_time_ratio Dead-time compensation in the alpha/beta frame, already normalised by V_dc.
   * @return   Three-phase duty cycles [0, 1] clamped to [duty_min, duty_max].
   */
  [[nodiscard]] constexpr system::ThreePhase<unit::Ratio> CalculateWithDeadTimeCompensation(
      const system::Stator<unit::Voltage>& voltage,
      unit::Voltage dc_link_voltage,
      const system::Stator<unit::Ratio>& dead_time_ratio) const noexcept {
    const float kInverseDcVoltage = 1.0F / std::max(dc_link_voltage.Value(), 1.0F);
    const system::Stator<unit::Ratio> kVoltageRatio{(voltage.alpha.Value() * kInverseDcVoltage) +
                                                        dead_time_ratio.alpha.Value(),
                                                    (voltage.beta.Value() * kInverseDcVoltage) +
                                                        dead_time_ratio.beta.Value()};

    return Calculate(kVoltageRatio.ToThreePhase());
  }
};

}  // namespace unimoc::control
