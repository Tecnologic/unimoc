/*
       __  ___   ________  _______  ______
      / / / / | / /  _/  |/  / __ \/ ____/
     / / / /  |/ // // /|_/ / / / / /
    / /_/ / /|  // // /  / / /_/ / /___
    \____/_/ |_/___/_/  /_/\____/\____/

    Universal Motor Control  2025 Alexander <tecnologic86@gmail.com> Evers

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

#ifndef UNIMOC_OBSERVER_MECHANICAL_OBSERVER_H_
#define UNIMOC_OBSERVER_MECHANICAL_OBSERVER_H_

#include <algorithm>
#include <cmath>
#include <concepts>
#include <numbers>
#include "StatorReference.hpp"

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
 * @brief Full-order back-EMF observer with phase-locked loop (PLL) for PMSM.
 *
 * Overview
 * --------
 * The observer estimates the extended back-EMF vector (ê_α, ê_β) in the
 * stationary α/β frame.  A PLL then locks onto that vector to extract the
 * electrical rotor angle θ̂ and electrical angular velocity ω̂.
 *
 * Observer equations (Euler forward integration, stationary frame)
 * ---------------------------------------------------------------
 *   dî_α/dt = (1/L) · (V_α − R·î_α − ê_α) + g_i · (i_α − î_α)
 *   dî_β/dt = (1/L) · (V_β − R·î_β − ê_β) + g_i · (i_β − î_β)
 *   dê_α/dt = g_e · (i_α − î_α)
 *   dê_β/dt = g_e · (i_β − î_β)
 *
 * PLL (angle tracking)
 * --------------------
 * The angle error is derived without calling atan2 by projecting ê onto the
 * estimated d-axis direction:
 *
 *   ε = ê_α · cos(θ̂) + ê_β · sin(θ̂)   ≈ |ê| · sin(θ_true − θ̂)
 *
 * A PI controller on ε drives θ̂ → θ_true:
 *   ω̂ += K_i_pll · ε · dt
 *   θ̂ += (ω̂ + K_p_pll · ε) · dt
 *
 * An additional correction term can be injected from an HFI observer (or any
 * other auxiliary position sensor, e.g. hall sensors) via inject_angle_error().
 *
 * Flying-start support
 * --------------------
 * The observer needs no special initialisation.  When the motor is already
 * spinning the back-EMF is immediately observable, and the PLL converges
 * regardless of the initial angle estimate.  Convergence speed is governed
 * by K_p_pll, K_i_pll, g_i and g_e.
 *
 * @tparam T  Floating-point type (float by default).
 */
template <std::floating_point T = float>
struct MechanicalObserver
{
    // -------------------------------------------------------------------------
    // Motor parameters
    // -------------------------------------------------------------------------

    /// Stator phase resistance R [Ω].
    T R{static_cast<T>(0.1)};

    /// Stator inductance L [H] (use L_q for IPMSM, or average (L_d+L_q)/2).
    T L{static_cast<T>(1e-3)};

    // -------------------------------------------------------------------------
    // Observer gains
    // -------------------------------------------------------------------------

    /// Current observer correction gain g_i  [1/s].
    T g_i{static_cast<T>(2000.0)};

    /// Back-EMF observer correction gain g_e  [V/(A·s)].
    T g_e{static_cast<T>(50000.0)};

    // -------------------------------------------------------------------------
    // PLL gains
    // -------------------------------------------------------------------------

    /// PLL proportional gain K_p [rad/s per unit error].
    T pll_kp{static_cast<T>(500.0)};

    /// PLL integral gain K_i [rad/s² per unit error].
    T pll_ki{static_cast<T>(5000.0)};

    // -------------------------------------------------------------------------
    // Observer state
    // -------------------------------------------------------------------------

    /// Estimated α-axis stator current [A].
    T i_alpha_hat{static_cast<T>(0)};
    /// Estimated β-axis stator current [A].
    T i_beta_hat{static_cast<T>(0)};

    /// Estimated α-axis back-EMF [V].
    T e_alpha_hat{static_cast<T>(0)};
    /// Estimated β-axis back-EMF [V].
    T e_beta_hat{static_cast<T>(0)};

    // -------------------------------------------------------------------------
    // PLL state
    // -------------------------------------------------------------------------

    /// Estimated electrical rotor angle θ̂ [rad].
    T theta{static_cast<T>(0)};

    /// Estimated electrical angular velocity ω̂ [rad/s].
    T omega{static_cast<T>(0)};

    // -------------------------------------------------------------------------
    // Output (updated by update())
    // -------------------------------------------------------------------------

    /// sin(θ̂) — ready for use in Park / inverse-Park transforms.
    T sin_theta{static_cast<T>(0)};
    /// cos(θ̂) — ready for use in Park / inverse-Park transforms.
    T cos_theta{static_cast<T>(1)};

    /**
     * @brief Run one observer step.
     *
     * Call this once per PWM/control interrupt.
     *
     * @param v_ab  Applied stator voltage vector α/β [V].
     * @param i_ab  Measured stator current vector α/β [A].
     * @param dt    Control period [s].
     */
    constexpr void
    update(const system::StatorReference<T>& v_ab,
           const system::StatorReference<T>& i_ab,
           const T                           dt) noexcept
    {
        // --- Current prediction error ---
        const T err_alpha = i_ab.alpha - i_alpha_hat;
        const T err_beta  = i_ab.beta  - i_beta_hat;

        // --- Back-EMF observer integration ---
        const T inv_L = static_cast<T>(1) / L;

        i_alpha_hat += dt * (inv_L * (v_ab.alpha - R * i_alpha_hat - e_alpha_hat)
                             + g_i * err_alpha);
        i_beta_hat  += dt * (inv_L * (v_ab.beta  - R * i_beta_hat  - e_beta_hat)
                             + g_i * err_beta);

        e_alpha_hat += dt * g_e * err_alpha;
        e_beta_hat  += dt * g_e * err_beta;

        // --- PLL: angle error from back-EMF projection onto estimated d-axis ---
        //
        // ê_α = |E| · (−sin θ),  ê_β = |E| · cos θ
        //
        // The d-axis direction at estimated angle θ̂ is [cos θ̂, sin θ̂].
        // Projecting ê onto the d-axis gives:
        //   ε = ê_α · cos θ̂ + ê_β · sin θ̂  =  |E| · sin(θ − θ̂)
        //
        // For small errors ε ≈ |E| · (θ − θ̂).
        const T angle_error = e_alpha_hat * cos_theta + e_beta_hat * sin_theta;

        // PI controller drives angle_error → 0
        omega += pll_ki * angle_error * dt;
        theta += (omega + pll_kp * angle_error) * dt;

        // Wrap θ̂ to (−π, π]
        wrap_angle(theta);

        // Pre-compute sin/cos for use by downstream transforms
        sin_theta = std::sin(theta);
        cos_theta = std::cos(theta);
    }

    /**
     * @brief Inject an external angle error correction into the PLL.
     *
     * Used by the HFI observer (and optionally hall-sensor observers) to feed
     * additional position information into the same PLL without running a
     * separate angle estimator.
     *
     * @param angle_error  Signed angle error [rad] (positive when estimated
     *                     angle is lagging the true angle).
     * @param dt           Control period [s].
     */
    constexpr void
    inject_angle_error(const T angle_error, const T dt) noexcept
    {
        omega += pll_ki * angle_error * dt;
        theta += pll_kp * angle_error * dt;
        wrap_angle(theta);
        sin_theta = std::sin(theta);
        cos_theta = std::cos(theta);
    }

    /**
     * @brief Reset observer and PLL state.
     *
     * Call when re-enabling the drive after a fault or when a reliable initial
     * angle is available (e.g. from an encoder).
     *
     * @param theta_init  Initial electrical angle [rad].
     * @param omega_init  Initial electrical angular velocity [rad/s].
     */
    constexpr void
    reset(const T theta_init = static_cast<T>(0),
          const T omega_init = static_cast<T>(0)) noexcept
    {
        i_alpha_hat = static_cast<T>(0);
        i_beta_hat  = static_cast<T>(0);
        e_alpha_hat = static_cast<T>(0);
        e_beta_hat  = static_cast<T>(0);

        theta     = theta_init;
        omega     = omega_init;
        sin_theta = std::sin(theta);
        cos_theta = std::cos(theta);
    }

private:
    /// Wrap angle to (−π, π].
    static constexpr void
    wrap_angle(T& angle) noexcept
    {
        constexpr T pi     = std::numbers::pi_v<T>;
        constexpr T two_pi = static_cast<T>(2) * pi;

        while (angle > pi)
            angle -= two_pi;
        while (angle <= -pi)
            angle += two_pi;
    }
};

}  // namespace observer
}  // namespace unimoc

#endif /* UNIMOC_OBSERVER_MECHANICAL_OBSERVER_H_ */
