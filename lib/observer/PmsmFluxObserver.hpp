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

#ifndef UNIMOC_OBSERVER_PMSM_FLUX_OBSERVER_H_
#define UNIMOC_OBSERVER_PMSM_FLUX_OBSERVER_H_

#include <cmath>
#include <concepts>
#include "RotorReference.hpp"
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
 * @brief Voltage-model PMSM flux observer with closed-loop feedback correction.
 *
 * Overview
 * --------
 * The observer estimates the permanent-magnet flux linkage vector by
 * integrating the back-EMF in the stationary (α/β) frame and then
 * transforming the result to the estimated rotor (d/q) frame.
 *
 * Rotor-frame back-EMF
 * --------------------
 *   e_d = V_d − R_s · i_d + fb_d
 *   e_q = V_q − R_s · i_q + fb_q
 *
 * where fb_d, fb_q are closed-loop feedback terms that prevent integrator
 * drift (see below).
 *
 * Stator-frame flux integration
 * -----------------------------
 * The back-EMF is transformed to the stator frame by an inverse Park
 * transform using the current angle estimate from the MechanicalObserver:
 *
 *   e_α = e_d · cos θ̂ − e_q · sin θ̂
 *   e_β = e_d · sin θ̂ + e_q · cos θ̂
 *
 * The stator-frame flux is then obtained by integration:
 *   ψ_α += e_α · dt
 *   ψ_β += e_β · dt
 *
 * Estimated PM flux in rotor frame
 * ---------------------------------
 * The integrated flux is transformed back to the rotor frame (Park transform):
 *   ψ_d =  ψ_α · cos θ̂ + ψ_β · sin θ̂
 *   ψ_q = −ψ_α · sin θ̂ + ψ_β · cos θ̂
 *
 * The inductance-induced flux is subtracted to isolate the PM flux:
 *   ψ_d_PM = ψ_d − L_d · i_d
 *   ψ_q_PM = ψ_q − L_q · i_q
 *
 * Closed-loop feedback (anti-drift)
 * ----------------------------------
 * The difference between the flux set-point (desired PM flux) and the
 * estimated PM flux drives a proportional feedback into the back-EMF:
 *
 *   fb_d = C_d · (ψ_set_d − ψ_d_PM)
 *   fb_q = C_q · (ψ_set_q − ψ_q_PM)
 *
 * This prevents integrator wind-up in steady state and pulls the estimate
 * towards the known PM flux magnitude.
 *
 * Feeding the MechanicalObserver Kalman filter
 * ---------------------------------------------
 * The estimated angle error derived from the flux vector is injected into
 * the MechanicalObserver via inject_angle_error(), so both estimators share
 * the same rotor-angle reference.
 *
 * @tparam T  Floating-point type (float by default).
 */
template <std::floating_point T = float>
struct PmsmFluxObserver
{
    // =========================================================================
    // Motor parameters  (set from NvmSettings before first use)
    // =========================================================================

    /// Stator phase resistance R_s [Ω].
    T rs{static_cast<T>(0.1)};

    /// d-axis inductance L_d [H].
    T L_d{static_cast<T>(1e-3)};

    /// q-axis inductance L_q [H].
    T L_q{static_cast<T>(1e-3)};

    // =========================================================================
    // Feedback (anti-drift) gains
    // =========================================================================

    /// d-axis feedback gain C_d [1/s].  Larger values reduce drift faster but
    /// also damp transient response.
    T C_d{static_cast<T>(50.0)};

    /// q-axis feedback gain C_q [1/s].
    T C_q{static_cast<T>(1.0)};

    // =========================================================================
    // Observer state
    // =========================================================================

    /// Stator-frame integrated flux α-component ψ_α [Wb].
    T psi_alpha{static_cast<T>(0)};

    /// Stator-frame integrated flux β-component ψ_β [Wb].
    T psi_beta{static_cast<T>(0)};

    /// Rotor-frame d-axis back-EMF feedback term fb_d [V].
    T fb_d{static_cast<T>(0)};

    /// Rotor-frame q-axis back-EMF feedback term fb_q [V].
    T fb_q{static_cast<T>(0)};

    // =========================================================================
    // Outputs (updated by calculate())
    // =========================================================================

    /// Estimated PM flux in the rotor d-axis [Wb] (after inductance subtraction).
    T psi_pm_d{static_cast<T>(0)};

    /// Estimated PM flux in the rotor q-axis [Wb] (after inductance subtraction).
    T psi_pm_q{static_cast<T>(0)};

    // =========================================================================
    // Public API
    // =========================================================================

    /**
     * @brief Run one observer step and feed the angle correction to the
     *        MechanicalObserver Kalman filter.
     *
     * Call this once per PWM/control interrupt.  The MechanicalObserver must
     * have already been updated via predict() before this call so that
     * sin_theta / cos_theta reflect the latest angle estimate.
     *
     * @param u_dq     Applied rotor-frame voltage vector [V].
     * @param i_dq     Measured rotor-frame current vector [A].
     * @param set_flux Desired PM flux in the rotor frame [Wb]
     *                 (typically { ψ_PM, 0 } for SPMSM).
     * @param dt       Control period [s].
     * @param mech_obs MechanicalObserver whose Kalman filter receives the
     *                 flux-derived angle correction.
     */
    constexpr void
    calculate(const system::RotorReference<T>& u_dq,
              const system::RotorReference<T>& i_dq,
              const system::RotorReference<T>& set_flux,
              const T                          dt,
              MechanicalObserver<T>&           mech_obs) noexcept
    {
        // Snapshot sin/cos from the mechanical observer
        const T sn = mech_obs.sin_theta;
        const T cs = mech_obs.cos_theta;

        // -----------------------------------------------------------------
        // Rotor-frame back-EMF (voltage model + feedback)
        // -----------------------------------------------------------------
        const T bemf_d = u_dq.d - rs * i_dq.d + fb_d;
        const T bemf_q = u_dq.q - rs * i_dq.q + fb_q;

        // -----------------------------------------------------------------
        // Inverse Park: rotor → stator frame
        //   e_α = bemf_d · cos θ̂ − bemf_q · sin θ̂
        //   e_β = bemf_d · sin θ̂ + bemf_q · cos θ̂
        // -----------------------------------------------------------------
        const T bemf_alpha = bemf_d * cs - bemf_q * sn;
        const T bemf_beta  = bemf_d * sn + bemf_q * cs;

        // -----------------------------------------------------------------
        // Stator-frame flux integration
        // -----------------------------------------------------------------
        psi_alpha += bemf_alpha * dt;
        psi_beta  += bemf_beta  * dt;

        // -----------------------------------------------------------------
        // Park: stator → rotor frame
        //   ψ_d =  ψ_α · cos θ̂ + ψ_β · sin θ̂
        //   ψ_q = −ψ_α · sin θ̂ + ψ_β · cos θ̂
        // -----------------------------------------------------------------
        const T psi_d =  psi_alpha * cs + psi_beta * sn;
        const T psi_q = -psi_alpha * sn + psi_beta * cs;

        // -----------------------------------------------------------------
        // Subtract inductance-induced flux to get PM flux estimate
        // -----------------------------------------------------------------
        psi_pm_d = psi_d - L_d * i_dq.d;
        psi_pm_q = psi_q - L_q * i_dq.q;

        // -----------------------------------------------------------------
        // Closed-loop feedback update
        // -----------------------------------------------------------------
        fb_d = C_d * (set_flux.d - psi_pm_d);
        fb_q = C_q * (set_flux.q - psi_pm_q);

        // -----------------------------------------------------------------
        // Inject angle error into the MechanicalObserver Kalman filter.
        //
        // The PM flux should align with the d-axis when the angle estimate
        // is correct (ψ_q_PM ≈ 0).  The q-axis component is a signed angle
        // error signal: positive means the d-axis estimate lags the flux.
        //
        // We negate it so the convention matches: positive ε → advance θ̂.
        // -----------------------------------------------------------------
        const T angle_error = -psi_pm_q;
        mech_obs.inject_angle_error(angle_error, dt);
    }

    /**
     * @brief Reset the observer state.
     *
     * Call on fault recovery or control-mode transitions.
     */
    constexpr void
    reset() noexcept
    {
        psi_alpha = static_cast<T>(0);
        psi_beta  = static_cast<T>(0);
        fb_d      = static_cast<T>(0);
        fb_q      = static_cast<T>(0);
        psi_pm_d  = static_cast<T>(0);
        psi_pm_q  = static_cast<T>(0);
    }
};

}  // namespace observer
}  // namespace unimoc

#endif /* UNIMOC_OBSERVER_PMSM_FLUX_OBSERVER_H_ */
