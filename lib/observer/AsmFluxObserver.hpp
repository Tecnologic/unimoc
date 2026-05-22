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

#ifndef UNIMOC_OBSERVER_ASM_FLUX_OBSERVER_H_
#define UNIMOC_OBSERVER_ASM_FLUX_OBSERVER_H_

#include <algorithm>
#include <cmath>
#include <concepts>
#include <numbers>
#include "StatorReference.hpp"
#include "MechanicalObserver.hpp"

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
 * @brief Full-order Luenberger rotor-flux observer for induction motors (ASM).
 *
 * Overview
 * --------
 * The observer estimates the stator current vector (î_sα, î_sβ) and the rotor
 * flux linkage vector (ψ̂_rα, ψ̂_rβ) in the stationary α/β frame.  It uses the
 * measured stator voltage and current together with the estimated electrical
 * rotor speed (from the MechanicalObserver PLL) as inputs.
 *
 * Induction motor model (stationary α/β frame)
 * ---------------------------------------------
 * Let σ = 1 − L_m² / (L_s · L_r) be the total leakage factor.
 *
 * Stator current dynamics:
 *   dî_sα/dt = −(R_s/(σ·L_s) + (1−σ)/(σ·T_r)) · î_sα
 *              + (L_m/(σ·L_s·L_r)) · (ψ̂_rα/T_r + ω̂_r · ψ̂_rβ)
 *              + v_sα/(σ·L_s)
 *              + g_i · (i_sα − î_sα)
 *
 * Rotor flux dynamics:
 *   dψ̂_rα/dt = (L_m/T_r) · î_sα − (1/T_r) · ψ̂_rα − ω̂_r · ψ̂_rβ
 *              + g_flux · (i_sα − î_sα)
 *
 * (β-axis equations are identical with α↔β and appropriate sign changes.)
 *
 * Correction gain strategy
 * ------------------------
 * Both the current and flux equations are corrected by the current prediction
 * error (i_s − î_s), similar to a reduced-order Luenberger observer.  This
 * eliminates the need to measure rotor quantities.
 *
 * Feeding the MechanicalObserver
 * -------------------------------
 * The rotor flux angle θ_flux = atan2(ψ̂_rβ, ψ̂_rα) is the field reference
 * frame angle for FOC.  It is injected into the MechanicalObserver's shared
 * PLL via inject_angle_error() so that both observers converge on the same
 * angle estimate, enabling a smooth PMSM↔ASM transition if needed.
 *
 * @tparam T  Floating-point type (float by default).
 */
template <std::floating_point T = float>
struct AsmFluxObserver
{
    // -------------------------------------------------------------------------
    // Motor parameters
    // -------------------------------------------------------------------------

    /// Stator resistance R_s [Ω].
    T R_s{static_cast<T>(0.5)};

    /// Rotor resistance R_r [Ω].
    T R_r{static_cast<T>(0.3)};

    /// Stator self-inductance L_s [H].
    T L_s{static_cast<T>(50e-3)};

    /// Rotor self-inductance L_r [H] (≈ L_s for squirrel-cage motors).
    T L_r{static_cast<T>(50e-3)};

    /// Mutual (magnetising) inductance L_m [H].
    T L_m{static_cast<T>(47e-3)};

    // -------------------------------------------------------------------------
    // Observer gains
    // -------------------------------------------------------------------------

    /// Stator-current correction gain g_i [1/s].
    ///
    /// Governs how aggressively the current prediction error corrects the
    /// estimated stator current.  A value in the range [2·R_s/σL_s … 10·R_s/σL_s]
    /// is a reasonable starting point.
    T g_i{static_cast<T>(500.0)};

    /// Rotor-flux correction gain g_flux [Wb/(A·s)].
    ///
    /// Determines how fast the flux estimate responds to the current error.
    /// Set larger than g_i to prevent flux lag during transients.
    T g_flux{static_cast<T>(5000.0)};

    // -------------------------------------------------------------------------
    // Observer state
    // -------------------------------------------------------------------------

    /// Estimated α-axis stator current [A].
    T i_alpha_hat{static_cast<T>(0)};
    /// Estimated β-axis stator current [A].
    T i_beta_hat{static_cast<T>(0)};

    /// Estimated α-axis rotor flux linkage [Wb].
    T psi_r_alpha{static_cast<T>(0)};
    /// Estimated β-axis rotor flux linkage [Wb].
    T psi_r_beta{static_cast<T>(0)};

    // -------------------------------------------------------------------------
    // Outputs (updated by update())
    // -------------------------------------------------------------------------

    /// Estimated rotor flux magnitude |ψ̂_r| [Wb].
    T flux_magnitude{static_cast<T>(0)};

    /// Estimated rotor flux angle θ_flux = atan2(ψ̂_rβ, ψ̂_rα) [rad].
    T flux_angle{static_cast<T>(0)};

    /// sin(θ_flux) — ready for field-oriented transforms.
    T sin_flux{static_cast<T>(0)};
    /// cos(θ_flux) — ready for field-oriented transforms.
    T cos_flux{static_cast<T>(1)};

    /**
     * @brief Run one observer step and update the MechanicalObserver PLL.
     *
     * Call this once per PWM/control interrupt.
     *
     * @param v_ab     Applied stator voltage vector α/β [V].
     * @param i_ab     Measured stator current vector α/β [A].
     * @param dt       Control period [s].
     * @param mech_obs MechanicalObserver whose PLL receives the flux-angle correction.
     */
    constexpr void
    update(const system::StatorReference<T>& v_ab,
           const system::StatorReference<T>& i_ab,
           const T                           dt,
           MechanicalObserver<T>&            mech_obs) noexcept
    {
        // --- Derived parameters (computed every cycle for generality; in
        //     practice the compiler will CSE these if parameters are const.) ---
        const T sigma   = calc_sigma();
        const T T_r     = L_r / R_r;           // rotor time constant [s]
        const T inv_T_r = R_r / L_r;           // 1/T_r
        const T inv_sLs = static_cast<T>(1) / (sigma * L_s);

        // Stator-current time-constant coefficient
        const T alpha_i = R_s * inv_sLs + (static_cast<T>(1) - sigma) * inv_T_r / sigma;

        // Flux-to-current coupling coefficient L_m/(σ·L_s·L_r)
        const T k_flux  = L_m / (sigma * L_s * L_r);

        // Electrical rotor speed from MechanicalObserver PLL
        const T omega_r = mech_obs.omega;

        // --- Current prediction error ---
        const T err_alpha = i_ab.alpha - i_alpha_hat;
        const T err_beta  = i_ab.beta  - i_beta_hat;

        // --- Stator-current observer ---
        const T di_alpha = -alpha_i * i_alpha_hat
                           + k_flux * (inv_T_r * psi_r_alpha + omega_r * psi_r_beta)
                           + inv_sLs * v_ab.alpha
                           + g_i * err_alpha;

        const T di_beta  = -alpha_i * i_beta_hat
                           + k_flux * (inv_T_r * psi_r_beta - omega_r * psi_r_alpha)
                           + inv_sLs * v_ab.beta
                           + g_i * err_beta;

        // --- Rotor-flux observer ---
        const T dpsi_r_alpha = (L_m * inv_T_r) * i_alpha_hat
                                - inv_T_r * psi_r_alpha
                                - omega_r * psi_r_beta
                                + g_flux * err_alpha;

        const T dpsi_r_beta  = (L_m * inv_T_r) * i_beta_hat
                                - inv_T_r * psi_r_beta
                                + omega_r * psi_r_alpha
                                + g_flux * err_beta;

        // --- Euler integration ---
        i_alpha_hat += dt * di_alpha;
        i_beta_hat  += dt * di_beta;
        psi_r_alpha += dt * dpsi_r_alpha;
        psi_r_beta  += dt * dpsi_r_beta;

        // --- Compute flux magnitude and angle ---
        flux_magnitude = std::sqrt(psi_r_alpha * psi_r_alpha + psi_r_beta * psi_r_beta);
        flux_angle     = std::atan2(psi_r_beta, psi_r_alpha);
        sin_flux       = std::sin(flux_angle);
        cos_flux       = std::cos(flux_angle);

        // --- Feed flux angle into MechanicalObserver PLL ---
        //
        // Compute the angle error between the flux angle and the PLL estimate.
        // Using the cross-product sine approximation (no atan2 needed):
        //   sin(θ_flux − θ̂) ≈ sin(θ_flux)·cos(θ̂) − cos(θ_flux)·sin(θ̂)
        //
        // This is the same technique used in the PMSM back-EMF PLL.
        const T angle_error = sin_flux * mech_obs.cos_theta
                              - cos_flux * mech_obs.sin_theta;

        mech_obs.inject_angle_error(angle_error, dt);
    }

    /**
     * @brief Reset the observer state.
     *
     * Call on mode transitions or fault recovery.
     */
    constexpr void
    reset() noexcept
    {
        i_alpha_hat    = static_cast<T>(0);
        i_beta_hat     = static_cast<T>(0);
        psi_r_alpha    = static_cast<T>(0);
        psi_r_beta     = static_cast<T>(0);
        flux_magnitude = static_cast<T>(0);
        flux_angle     = static_cast<T>(0);
        sin_flux       = static_cast<T>(0);
        cos_flux       = static_cast<T>(1);
    }

private:
    /// Compute total leakage factor σ = 1 − L_m² / (L_s · L_r).
    [[nodiscard]] constexpr T
    calc_sigma() const noexcept
    {
        return static_cast<T>(1) - (L_m * L_m) / (L_s * L_r);
    }
};

}  // namespace observer
}  // namespace unimoc

#endif /* UNIMOC_OBSERVER_ASM_FLUX_OBSERVER_H_ */
