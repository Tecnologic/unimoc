/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file dead_time_compensation.hpp
 *    @brief Unit-typed inverter dead-time compensation.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <concepts>
#include "nvm_settings.hpp"
#include "stator_system.hpp"
#include "three_phase_system.hpp"

/**
 * @namespace unimoc::control control algorithms namespace
 */
namespace unimoc::control {

/**
 * @brief Dead-time compensation for a three-phase voltage-source inverter.
 *
 * Gate-driver dead time prevents both switches of a half-bridge from
 * conducting simultaneously, but introduces an unwanted voltage error that
 * depends on the sign of the phase current.  For each phase the error is:
 *
 *   ΔV_phase = sign(i_phase) · V_dc · t_dead · f_pwm
 *
 * This class computes the compensation voltage in the stationary α/β frame
 * so that it can be added to the voltage reference before it is passed to
 * the SVM modulator.
 *
 * A configurable threshold @p i_threshold is used around zero current to
 * apply a soft sign function, smoothing out chattering at phase-current
 * zero crossings.
 *
 * @tparam T  Floating-point type (float by default).
 */
template <std::floating_point T = float>
struct DeadTimeCompensation {
  /// Dead time of the gate driver [s].
  unit::Time dead_time;

  /// PWM switching frequency [Hz].
  unit::Frequency f_pwm;

  /**
   * @brief Threshold current [A] for the soft sign function.
   *
   * Currents with magnitude below this value produce a linearly-interpolated
   * sign rather than a hard ±1, which avoids large voltage spikes and chattering
   * near zero-crossings.
   */
  unit::Current i_threshold;

  /**
   * @brief Load dead-time compensation parameters from NVM settings.
   *
   * @param settings Validated NVM settings.
   */
  constexpr void Init(const settings::NvmSettings& settings) noexcept {
    dead_time = settings.dtc_dead_time;
    f_pwm = settings.dtc_f_pwm;
    i_threshold = settings.dtc_i_threshold;
  }

  /**
   * @brief Compute the α/β compensation voltage to add to the modulator input.
   *
   * The stationary-frame currents (i_alpha, i_beta) are used to reconstruct
   * the three phase currents via the inverse Clarke transform, the dead-time
   * voltage error is estimated for each phase, and the result is transformed
   * back to α/β via the (amplitude-invariant) Clarke transform.
   *
   * @param i_ab  Measured stator current in the stationary α/β frame [A].
   * @return      Compensation voltage vector (normalised by V_dc) to add to
   *              the α/β voltage reference.
   */
  [[nodiscard]] constexpr system::Stator<unit::Ratio> Calculate(
      const system::Stator<unit::Current>& i_ab) const noexcept {
    // --- Reconstruct three-phase currents from α/β ---
    const auto kPhaseCurrents = i_ab.ToThreePhase();

    // --- Soft sign function: clamp(i / threshold, -1, +1) ---
    // This provides linear interpolation through zero, preventing chattering.
    using unit::operator""_ratio;
    constexpr unit::Ratio kMinusOne = -1.0_ratio;
    constexpr unit::Ratio kOne = 1.0_ratio;
    const unit::Ratio kSignA = (kPhaseCurrents.a / i_threshold).Clamp(kMinusOne, kOne);
    const unit::Ratio kSignB = (kPhaseCurrents.b / i_threshold).Clamp(kMinusOne, kOne);
    const unit::Ratio kSignC = (kPhaseCurrents.c / i_threshold).Clamp(kMinusOne, kOne);

    // Normalised per-phase voltage error: sign · t_dead · f_pwm
    const unit::Ratio kDtNorm = dead_time * f_pwm;
    const system::ThreePhase<unit::Ratio> kPhaseErrors{kSignA * kDtNorm, kSignB * kDtNorm, kSignC * kDtNorm};

    // --- Clarke transform (amplitude-invariant) back to α/β ---
    return kPhaseErrors.ToStator();
  }
};

}  // namespace unimoc::control
