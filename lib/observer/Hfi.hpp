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

#ifndef UNIMOC_OBSERVER_HFI_H_
#define UNIMOC_OBSERVER_HFI_H_

#include <cmath>
#include <concepts>
#include "StatorReference.hpp"
#include "RotorReference.hpp"
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
 * @brief 4-step High-Frequency Injection (HFI) observer for IPMSM.
 *
 * Overview
 * --------
 * At low speed the back-EMF magnitude is too small for the MechanicalObserver
 * to converge reliably.  HFI exploits the inductance saliency of an interior
 * PMSM (L_d ≠ L_q) to extract rotor position at standstill and low speed.
 *
 * Injection scheme (VESC-style 4-step)
 * -------------------------------------
 * The injection alternates over four consecutive PWM periods.  Each step
 * superimposes a fixed-amplitude voltage pulse @p v_inject onto the regular
 * modulator output in the estimated rotating (d/q) frame:
 *
 *   Step 0:  +V_inj in estimated d-axis  →  sample Δî_dq
 *   Step 1:  −V_inj in estimated d-axis  →  sample Δî_dq
 *   Step 2:  +V_inj in estimated q-axis  →  sample Δî_dq
 *   Step 3:  −V_inj in estimated q-axis  →  sample Δî_dq
 *
 * The caller must apply the injection voltage (returned by get_injection_voltage())
 * to the modulator before each step, sample the current afterwards, and then
 * call update() with that current sample.
 *
 * Angle error extraction
 * ----------------------
 * Due to saliency (ΔL = L_d − L_q ≠ 0) the current response to d-axis pulses
 * leaks into the q-axis when the estimated angle has an error ε:
 *
 *   q-axis leak (steps 0 & 1):  Δî_q ∝ (1/L_d − 1/L_q) · sin(2ε)
 *   d-axis leak (steps 2 & 3):  Δî_d ∝ (1/L_d − 1/L_q) · sin(2ε)  (opposite sign)
 *
 * The combined error signal is:
 *
 *   err = Δî_q(0) − Δî_q(1)
 *       − (Δî_d(2) − Δî_d(3))
 *
 * which is proportional to sin(2ε).  This signal is fed to
 * MechanicalObserver::inject_angle_error() after scaling, so that both the
 * HFI and the back-EMF observer share the same PLL.
 *
 * Note: HFI has an inherent 180° pole ambiguity because sin(2ε) = sin(2(ε+π)).
 *       An initial polarity test (brief d-axis current pulse at startup) is
 *       required to resolve it.
 *
 * @tparam T  Floating-point type (float by default).
 */
template <std::floating_point T = float>
struct Hfi
{
    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------

    /// HFI injection voltage [V] (referred to V_dc; typically 5–20 % of Vdc).
    T v_inject{static_cast<T>(0.0)};

    /**
     * @brief Gain that maps the raw angle error signal to the PLL injection [1/V].
     *
     * Tune so that the PLL bandwidth is in the desired range.  A higher value
     * gives faster convergence but more sensitivity to noise.
     */
    T error_gain{static_cast<T>(1.0)};

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------

    /// Current injection step (0–3).
    int step{0};

    // Stored d/q current samples from each injection step
    T i_d_step0{static_cast<T>(0)};
    T i_q_step0{static_cast<T>(0)};
    T i_d_step1{static_cast<T>(0)};
    T i_q_step1{static_cast<T>(0)};
    T i_d_step2{static_cast<T>(0)};
    T i_q_step2{static_cast<T>(0)};
    T i_d_step3{static_cast<T>(0)};
    T i_q_step3{static_cast<T>(0)};

    /**
     * @brief Return the injection voltage to add to the modulator input this step.
     *
     * The returned vector is in the stationary α/β frame.  It must be added to
     * the regular voltage reference before passing to the SVM modulator.
     *
     * Call this *before* the PWM period starts (before the current is sampled).
     *
     * @param sin_th  sin(θ̂) from MechanicalObserver.
     * @param cos_th  cos(θ̂) from MechanicalObserver.
     * @return        Injection voltage in the α/β frame [V].
     */
    [[nodiscard]] constexpr system::StatorReference<T>
    get_injection_voltage(const T sin_th, const T cos_th) const noexcept
    {
        // d-axis unit vector in α/β: [cos θ̂, sin θ̂]
        // q-axis unit vector in α/β: [−sin θ̂, cos θ̂]

        T v_d{static_cast<T>(0)};
        T v_q{static_cast<T>(0)};

        switch (step)
        {
            case 0: v_d = +v_inject; break;  // +Vd
            case 1: v_d = -v_inject; break;  // −Vd
            case 2: v_q = +v_inject; break;  // +Vq
            case 3: v_q = -v_inject; break;  // −Vq
            default: break;
        }

        // Inverse Park: dq → α/β
        // v_α = v_d·cos θ̂ − v_q·sin θ̂
        // v_β = v_d·sin θ̂ + v_q·cos θ̂
        return system::StatorReference<T>{
            v_d * cos_th - v_q * sin_th,
            v_d * sin_th + v_q * cos_th,
        };
    }

    /**
     * @brief Record the current sample for the active step and advance state.
     *
     * Call this *after* the current has been sampled at the end of the PWM
     * period during which the injection was applied.
     *
     * When a full 4-step cycle is complete (after step 3) the angle error is
     * extracted and injected into @p mech_obs via inject_angle_error().
     *
     * @param i_ab    Measured stator current in the α/β frame [A].
     * @param sin_th  sin(θ̂) from MechanicalObserver.
     * @param cos_th  cos(θ̂) from MechanicalObserver.
     * @param dt      Control period [s].
     * @param mech_obs  Reference to the mechanical observer that owns the PLL.
     */
    constexpr void
    update(const system::StatorReference<T>& i_ab,
           const T                           sin_th,
           const T                           cos_th,
           const T                           dt,
           MechanicalObserver<T>&            mech_obs) noexcept
    {
        // Park transform: α/β → estimated d/q
        // i_d =  i_α·cos θ̂ + i_β·sin θ̂
        // i_q = −i_α·sin θ̂ + i_β·cos θ̂
        const T i_d =  i_ab.alpha * cos_th + i_ab.beta * sin_th;
        const T i_q = -i_ab.alpha * sin_th + i_ab.beta * cos_th;

        switch (step)
        {
            case 0: i_d_step0 = i_d; i_q_step0 = i_q; break;
            case 1: i_d_step1 = i_d; i_q_step1 = i_q; break;
            case 2: i_d_step2 = i_d; i_q_step2 = i_q; break;
            case 3:
            {
                i_d_step3 = i_d;
                i_q_step3 = i_q;

                // --- Angle error extraction ---
                //
                // Δî_q from d-axis injection: proportional to sin(2ε)
                const T delta_iq_d = i_q_step0 - i_q_step1;

                // Δî_d from q-axis injection: proportional to −sin(2ε)
                const T delta_id_q = i_d_step2 - i_d_step3;

                // Combined error: adds both estimates, doubling SNR
                const T raw_error = delta_iq_d - delta_id_q;

                // Feed scaled error into the shared PLL of the mechanical observer
                mech_obs.inject_angle_error(raw_error * error_gain, dt);
                break;
            }
            default: break;
        }

        // Advance to the next step (wraps 3→0)
        step = (step + 1) & 3;
    }

    /// Reset HFI state (call on mode transitions or fault recovery).
    constexpr void
    reset() noexcept
    {
        step      = 0;
        i_d_step0 = i_q_step0 = static_cast<T>(0);
        i_d_step1 = i_q_step1 = static_cast<T>(0);
        i_d_step2 = i_q_step2 = static_cast<T>(0);
        i_d_step3 = i_q_step3 = static_cast<T>(0);
    }
};

}  // namespace observer
}  // namespace unimoc

#endif /* UNIMOC_OBSERVER_HFI_H_ */
