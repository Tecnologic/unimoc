/*
       __  ___   ________  _______  ______
      / / / / | / /  _/  |/  / __ \/ ____/
     / / / /  |/ // // /|_/ / / / / /
    / /_/ / /|  // // /  / / /_/ / /___
    \____/_/ [_/___/_/  /_/\____/\____/

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

#ifndef UNIMOC_CONTROL_ASM_FLUX_CONTROLLER_H_
#define UNIMOC_CONTROL_ASM_FLUX_CONTROLLER_H_

#include <algorithm>
#include <cmath>
#include <concepts>

/**
 * @namespace unimoc global namespace
 */
namespace unimoc
{
/**
 * @namespace control control algorithms namespace
 */
namespace control
{

/**
 * @brief Rotor-flux magnitude controller for induction motors (ASM).
 *
 * Overview
 * --------
 * In rotor-flux-oriented (FOC) control of an induction motor the d-axis current
 * (aligned with the rotor flux vector) determines the rotor flux magnitude,
 * while the q-axis current determines the torque.
 *
 * This controller runs a PI loop on the flux error:
 *
 *   e_flux = ψ_r* − |ψ̂_r|
 *   i_d*   = Kp · e_flux + integrator
 *
 * The integrator is clamped to [i_d_min, i_d_max] to prevent windup.
 *
 * Slip frequency feedforward
 * --------------------------
 * In steady state the slip angular frequency is:
 *
 *   ω_slip = (R_r / L_r) · (i_q / (ψ_r / L_m))
 *           = (1 / T_r)  · (L_m · i_q / ψ_r)
 *
 * where T_r = L_r / R_r is the rotor time constant.  Adding this to the
 * estimated rotor speed ω̂_r gives the stator excitation frequency:
 *
 *   ω_s = ω̂_r + ω_slip
 *
 * The caller should use ω_s to advance the field angle if an open-loop
 * feedforward scheme is preferred alongside the closed-loop flux observer.
 *
 * @tparam T  Floating-point type (float by default).
 */
template <std::floating_point T = float>
struct AsmFluxController
{
    // -------------------------------------------------------------------------
    // Motor parameters
    // -------------------------------------------------------------------------

    /// Rotor resistance R_r [Ω].
    T R_r{static_cast<T>(0.3)};

    /// Rotor self-inductance L_r [H].
    T L_r{static_cast<T>(50e-3)};

    /// Mutual (magnetising) inductance L_m [H].
    T L_m{static_cast<T>(47e-3)};

    // -------------------------------------------------------------------------
    // Controller gains
    // -------------------------------------------------------------------------

    /// Proportional gain K_p [A/Wb].
    T kp{static_cast<T>(10.0)};

    /// Integral gain K_i [A/(Wb·s)].
    T ki{static_cast<T>(50.0)};

    // -------------------------------------------------------------------------
    // Output limits
    // -------------------------------------------------------------------------

    /// Minimum d-axis current [A] (must be ≥ 0; negative magnetising current
    /// is not useful in a squirrel-cage motor).
    T i_d_min{static_cast<T>(0.0)};

    /// Maximum d-axis current [A].
    T i_d_max{static_cast<T>(10.0)};

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------

    /// PI integrator state [A].
    T integrator{static_cast<T>(0)};

    // -------------------------------------------------------------------------
    // Output
    // -------------------------------------------------------------------------

    /// d-axis current reference i_d* [A] (updated by update()).
    T i_d_ref{static_cast<T>(0)};

    /// Computed slip angular frequency ω_slip [rad/s] (updated by update()).
    T omega_slip{static_cast<T>(0)};

    /**
     * @brief Update the flux controller.
     *
     * Call once per control cycle.
     *
     * @param psi_r_ref     Rotor flux magnitude reference |ψ_r*| [Wb].
     * @param psi_r_meas    Estimated rotor flux magnitude |ψ̂_r| [Wb]
     *                      (from AsmFluxObserver::flux_magnitude).
     * @param i_q           Current q-axis current component [A] (for slip
     *                      frequency calculation).
     * @param dt            Control period [s].
     * @return              d-axis current reference i_d* [A].
     */
    constexpr T
    update(const T psi_r_ref,
           const T psi_r_meas,
           const T i_q,
           const T dt) noexcept
    {
        // --- PI flux controller ---
        const T error = psi_r_ref - psi_r_meas;

        integrator += ki * error * dt;
        integrator  = std::clamp(integrator, i_d_min, i_d_max);

        i_d_ref = std::clamp(kp * error + integrator, i_d_min, i_d_max);

        // --- Slip frequency feedforward ---
        // ω_slip = (1/T_r) · (L_m · i_q / ψ_r*)
        // Use the flux reference in the denominator to avoid division by
        // a near-zero measured flux during start-up.
        if (psi_r_ref > static_cast<T>(1e-6))
        {
            const T T_r  = L_r / R_r;
            omega_slip = (L_m / (T_r * psi_r_ref)) * i_q;
        }
        else
        {
            omega_slip = static_cast<T>(0);
        }

        return i_d_ref;
    }

    /// Reset controller state (call on enable or fault recovery).
    constexpr void
    reset() noexcept
    {
        integrator = static_cast<T>(0);
        i_d_ref    = static_cast<T>(0);
        omega_slip = static_cast<T>(0);
    }
};

}  // namespace control
}  // namespace unimoc

#endif /* UNIMOC_CONTROL_ASM_FLUX_CONTROLLER_H_ */
