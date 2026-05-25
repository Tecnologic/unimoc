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

#ifndef UNIMOC_CONTROL_CURRENT_CONTROLLER_H_
#define UNIMOC_CONTROL_CURRENT_CONTROLLER_H_

#include <cmath>
#include <concepts>
#include "RotorReference.hpp"

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
 * @brief d/q-axis PI current controller with cross-coupling feedforward and
 *        circular voltage-vector limiting.
 *
 * Overview
 * --------
 * Runs one PI control step per PWM interrupt to regulate the rotor-frame
 * d- and q-axis stator currents independently.  The controller is designed
 * to be called from the highest-priority ISR, so all state is maintained in
 * plain member variables — no dynamic allocation, no OS calls.
 *
 * Cross-coupling feedforward
 * --------------------------
 * The voltage model of a PMSM in the rotor frame contains coupling terms
 * that, if ignored, appear as disturbances to the current loops:
 *
 *   v_d_ff = −ω · L_q · i_q
 *   v_q_ff = +ω · (L_d · i_d + ψ_PM)
 *
 * These feedforward terms are added to the PI output before limiting so that
 * the PI integrators only need to compensate residual errors, not the
 * (dominant) back-EMF and coupling voltages.
 *
 * Voltage vector limiting (circular)
 * ------------------------------------
 * The combined d/q voltage vector is clamped to a circle of radius v_max
 * (normalised by V_dc, typically 0.9 to preserve SVM headroom).  When the
 * vector must be scaled back, both components are reduced proportionally so
 * that the angle (i.e., the torque-to-flux ratio) is preserved.
 *
 * Anti-windup
 * -----------
 * Integration advances only when the output is not saturated.  Saturation
 * is detected per-axis independently: the integrator for axis X freezes when
 * the output for that axis has been clipped by the circular limiter.  A
 * simple check compares the unlimited and limited outputs; if they differ,
 * the integrator is not advanced.
 *
 * Usage
 * -----
 * @code
 *   // Initialise from NvmSettings:
 *   unimoc::control::CurrentController<float> cc;
 *   cc.kp_d = settings.current_kp_d;  cc.ki_d = settings.current_ki_d;
 *   cc.kp_q = settings.current_kp_q;  cc.ki_q = settings.current_ki_q;
 *   cc.L_d  = settings.L_d;           cc.L_q  = settings.L_q;
 *   cc.psi  = settings.flux_pm;
 *   cc.v_max = settings.current_v_max;
 *
 *   // In the ISR:
 *   auto u_dq = cc.update(i_ref, i_meas, omega, dt_fast);
 * @endcode
 *
 * @tparam T  Floating-point type (float by default).
 */
template <std::floating_point T = float>
struct CurrentController
{
    // =========================================================================
    // Motor parameters  (set from NvmSettings before first use)
    // =========================================================================

    /// d-axis inductance L_d [H].
    T L_d{static_cast<T>(1e-3)};

    /// q-axis inductance L_q [H].
    T L_q{static_cast<T>(1e-3)};

    /// Permanent-magnet flux linkage ψ_PM [Wb].
    T psi{static_cast<T>(0)};

    // =========================================================================
    // Controller gains
    // =========================================================================

    /// d-axis proportional gain [V/A].
    T kp_d{static_cast<T>(1.0)};

    /// d-axis integral gain [V/(A·s)].
    T ki_d{static_cast<T>(100.0)};

    /// q-axis proportional gain [V/A].
    T kp_q{static_cast<T>(1.0)};

    /// q-axis integral gain [V/(A·s)].
    T ki_q{static_cast<T>(100.0)};

    // =========================================================================
    // Output limit
    // =========================================================================

    /// Maximum voltage vector magnitude as a fraction of V_dc (range (0, 1]).
    /// Typically set to 0.9 to preserve SVM headroom and avoid over-modulation.
    /// The actual voltage limit applied inside update() is v_max * v_dc [V].
    T v_max{static_cast<T>(0.9)};

    // =========================================================================
    // Integrator state
    // =========================================================================

    /// d-axis integrator accumulator [V].
    T integrator_d{static_cast<T>(0)};

    /// q-axis integrator accumulator [V].
    T integrator_q{static_cast<T>(0)};

    // =========================================================================
    // Public API
    // =========================================================================

    /**
     * @brief Run one PI current control step.
     *
     * Call once per PWM interrupt (or once per sub-step if running 4-step HFI).
     *
     * @param i_ref   Rotor-frame d/q current reference [A].
     * @param i_meas  Rotor-frame d/q measured current [A].
     * @param omega   Estimated electrical angular velocity ω̂ [rad/s].
     * @param dt      Control period for this step [s].
     * @param v_dc    DC-link voltage [V], used to scale the voltage limit
     *                (limit = v_max * v_dc).
     * @return        Rotor-frame d/q voltage demand [V], clamped to the
     *                voltage circle of radius v_max * v_dc.
     */
    constexpr system::RotorReference<T>
    update(const system::RotorReference<T>& i_ref,
           const system::RotorReference<T>& i_meas,
           const T                          omega,
           const T                          dt,
           const T                          v_dc) noexcept
    {
        // --- Current errors ---
        const T e_d = i_ref.d - i_meas.d;
        const T e_q = i_ref.q - i_meas.q;

        // --- Cross-coupling feedforward ---
        //   v_ff_d = −ω · L_q · i_q
        //   v_ff_q = +ω · (L_d · i_d + ψ_PM)
        const T v_ff_d = -omega * L_q * i_meas.q;
        const T v_ff_q =  omega * (L_d * i_meas.d + psi);

        // --- PI output (unlimited) ---
        T u_d_raw = kp_d * e_d + integrator_d + v_ff_d;
        T u_q_raw = kp_q * e_q + integrator_q + v_ff_q;

        // --- Circular voltage-vector limiting ---
        // The limit is expressed in volts: v_limit = v_max [fraction] * v_dc [V].
        const T v_limit = v_max * v_dc;
        const T mag_sq = u_d_raw * u_d_raw + u_q_raw * u_q_raw;
        T u_d = u_d_raw;
        T u_q = u_q_raw;

        bool saturated = false;
        if (mag_sq > v_limit * v_limit)
        {
            const T scale = v_limit / std::sqrt(mag_sq);
            u_d = u_d_raw * scale;
            u_q = u_q_raw * scale;
            saturated = true;
        }

        // --- Integrator update with anti-windup ---
        // Only advance the integrator when the output is unsaturated so that
        // the accumulated voltage does not exceed the achievable range.
        if (!saturated)
        {
            integrator_d += ki_d * e_d * dt;
            integrator_q += ki_q * e_q * dt;
        }

        return system::RotorReference<T>{u_d, u_q};
    }

    /**
     * @brief Reset integrator state.
     *
     * Call when re-enabling the controller after a fault or mode transition.
     */
    constexpr void
    reset() noexcept
    {
        integrator_d = static_cast<T>(0);
        integrator_q = static_cast<T>(0);
    }
};

}  // namespace control
}  // namespace unimoc

#endif /* UNIMOC_CONTROL_CURRENT_CONTROLLER_H_ */
