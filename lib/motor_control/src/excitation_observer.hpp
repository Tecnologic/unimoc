/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file excitation_observer.hpp
 *    @brief Rotor excitation current observer for EESM.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <concepts>
#include "units.hpp"

/**
 * @namespace unimoc global namespace
 */
namespace unimoc {
/**
 * @namespace observer observer algorithms namespace
 */
namespace observer {

using namespace unit;

/**
 * @brief Rotor excitation current observer for EESM.
 *
 * Overview
 * --------
 * In an Electrically Excited Synchronous Machine (EESM) the effective
 * PM-equivalent rotor flux linkage is:
 *
 *   ψ_f = L_m · I_f
 *
 * This observer filters the measured rotor excitation current I_f_meas with a
 * first-order low-pass filter to reject PWM-switching ripple and ADC noise.
 * It then computes the estimated effective flux linkage ψ_f_hat.
 *
 * Both outputs (i_f_hat and psi_f_hat) are used by the EESM control path:
 *  - i_f_hat  → feedback for ExcitationController in CurrentMode.
 *  - psi_f_hat → feedback for ExcitationController in FluxMode, and as the
 *                dynamic ψ_PM substitute fed into Mtpa::flux_pm and
 *                MechanicalObserver when the rotor flux is variable.
 *
 * First-order low-pass filter
 * ---------------------------
 * The discrete-time approximation (Euler forward) is:
 *
 *   i_f_hat[k+1] = i_f_hat[k] + (dt / tau) · (i_f_meas − i_f_hat[k])
 *
 * where tau [s] is the filter time constant.  A value of tau ≈ 1–5 ms
 * typically gives good noise rejection while tracking the commanded current.
 *
 * @tparam T  Floating-point type (float by default).
 */
template <std::floating_point T = float>
struct ExcitationObserver {
  // -------------------------------------------------------------------------
  // Motor parameter
  // -------------------------------------------------------------------------

  /// Mutual (magnetising) inductance L_m [H].
  /// Scales the filtered current to the effective rotor flux linkage.
  unit::Inductance L_m{47.0_mH};

  // -------------------------------------------------------------------------
  // Filter parameter
  // -------------------------------------------------------------------------

  /// Low-pass filter time constant tau [s].
  /// Adjust to balance noise rejection against response speed.
  unit::Time tau{2.0_ms};

  // -------------------------------------------------------------------------
  // Outputs (updated by update())
  // -------------------------------------------------------------------------

  /// Estimated (filtered) rotor excitation current î_f [A].
  unit::Current i_f_hat{};

  /// Estimated effective rotor flux linkage ψ̂_f = L_m · î_f [Wb].
  unit::MagneticFlux psi_f_hat{};

  /**
   * @brief Update the excitation observer.
   *
   * Call once per control cycle.
   *
   * @param i_f_meas  Measured rotor excitation current [A].
   *                  If a direct measurement is unavailable, pass the
   *                  ExcitationController::i_f_ref as an open-loop estimate.
   * @param dt        Control period [s].
   */
  constexpr void Update(const unit::Current i_f_meas, const unit::Time dt) noexcept {
    // First-order low-pass: i_f_hat += (dt / tau) * (i_f_meas - i_f_hat)
    if (tau.Value() > 1.0e-12F) {
      const T kAlpha = dt.Value() / tau.Value();
      i_f_hat = unit::Current{i_f_hat.Value() + kAlpha * (i_f_meas.Value() - i_f_hat.Value())};
    } else {
      // Zero time constant — pass through immediately
      i_f_hat = i_f_meas;
    }

    psi_f_hat = L_m * i_f_hat;
  }

  /**
   * @brief Reset observer state.
   *
   * Call on mode transitions or fault recovery.
   *
   * @param i_f_init  Optional initial current estimate [A] (default 0).
   */
  constexpr void Reset(const unit::Current i_f_init = unit::Current{}) noexcept {
    i_f_hat = i_f_init;
    psi_f_hat = L_m * i_f_hat;
  }
};

}  // namespace observer
}  // namespace unimoc
