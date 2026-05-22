/*
       __  ___   ________  _______  ______
      / / / / | / /  _/  |/  / __ \/ ____/
     / / / /  |/ // // /|_/ / / / / /
    / /_/ / /|  // // /  / / /_/ / /___
    \____/_/ |_/___/_/  /_/\____/\____/

    Universal Motor Control  2026 Alexander <tecnologic86@gmail.com> Evers

    This file is part of UNIMOC.

    UNIMOC is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#ifndef UNIMOC_OBSERVER_EXCITATION_OBSERVER_H_
#define UNIMOC_OBSERVER_EXCITATION_OBSERVER_H_

#include <algorithm>
#include <cmath>
#include <concepts>

/**
 * @namespace unimoc global namespace
 */
namespace unimoc
{
/**
 * @namespace observer observer algorithms namespace
 */
namespace observer
{

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
struct ExcitationObserver
{
    // -------------------------------------------------------------------------
    // Motor parameter
    // -------------------------------------------------------------------------

    /// Mutual (magnetising) inductance L_m [H].
    /// Scales the filtered current to the effective rotor flux linkage.
    T L_m{static_cast<T>(47e-3)};

    // -------------------------------------------------------------------------
    // Filter parameter
    // -------------------------------------------------------------------------

    /// Low-pass filter time constant tau [s].
    /// Adjust to balance noise rejection against response speed.
    T tau{static_cast<T>(2e-3)};

    // -------------------------------------------------------------------------
    // Outputs (updated by update())
    // -------------------------------------------------------------------------

    /// Estimated (filtered) rotor excitation current î_f [A].
    T i_f_hat{static_cast<T>(0)};

    /// Estimated effective rotor flux linkage ψ̂_f = L_m · î_f [Wb].
    T psi_f_hat{static_cast<T>(0)};

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
    constexpr void
    update(const T i_f_meas, const T dt) noexcept
    {
        // First-order low-pass: i_f_hat += (dt / tau) * (i_f_meas - i_f_hat)
        if (tau > static_cast<T>(1e-12))
        {
            i_f_hat += (dt / tau) * (i_f_meas - i_f_hat);
        }
        else
        {
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
    constexpr void
    reset(const T i_f_init = static_cast<T>(0)) noexcept
    {
        i_f_hat   = i_f_init;
        psi_f_hat = L_m * i_f_hat;
    }
};

}  // namespace observer
}  // namespace unimoc

#endif /* UNIMOC_OBSERVER_EXCITATION_OBSERVER_H_ */
