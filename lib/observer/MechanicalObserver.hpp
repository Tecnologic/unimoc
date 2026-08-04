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

#ifndef UNIMOC_OBSERVER_MECHANICAL_OBSERVER_H_
#define UNIMOC_OBSERVER_MECHANICAL_OBSERVER_H_

#include <array>
#include <cmath>
#include <concepts>
#include <numbers>
#include "RotorReference.hpp"

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
 * @brief Kalman-filter-based mechanical observer for rotor angle and speed.
 *
 * Overview
 * --------
 * The observer tracks the three mechanical states:
 *   x = [ω̂,  θ̂,  m̂_l]ᵀ
 * where ω̂ is the estimated electrical angular velocity [rad/s], θ̂ is the
 * estimated electrical rotor angle [rad], and m̂_l is the estimated load
 * torque [N·m].
 *
 * Prediction step  (call predict() once per control cycle)
 * ---------------------------------------------------------
 * The electric torque is computed from the rotor-frame d/q currents:
 *
 *   m_el = (3/2) · [ψ_PM · i_q + (L_d − L_q) · i_d · i_q]
 *
 * The states are then propagated using Euler forward integration:
 *
 *   ω̂  += (dt/J) · (m_el − m̂_l)
 *   θ̂  += ω̂ · dt
 *   m̂_l unchanged (load is modelled as a random walk)
 *
 * After prediction ω̂ is clamped to [omega_min, omega_max] to enforce
 * hardware speed limits, and θ̂ is wrapped to (−π, π].
 *
 * Measurement update  (call inject_angle_error() once per control cycle)
 * -----------------------------------------------------------------------
 * An external angle measurement (from a flux observer, HFI, or hall sensors)
 * provides the innovation:
 *
 *   ε = θ_measured − θ̂
 *
 * A Kalman filter with process noise Q (scalar, applied uniformly to all
 * three state variances) and measurement noise R (scalar) computes the
 * optimal gain k and corrects the state:
 *
 *   ω̂  += k[0] · ε
 *   θ̂  += k[1] · ε
 *   m̂_l += k[2] · ε
 *
 * The covariance propagation uses the linearised state-transition matrix
 * evaluated at the current dt and J.
 *
 * Outputs
 * -------
 * After each call to predict() or inject_angle_error(), the fields
 * sin_theta and cos_theta are updated so downstream Park/inverse-Park
 * transforms can use them directly.
 *
 * @tparam T  Floating-point type (float by default).
 */
template <std::floating_point T = float>
struct MechanicalObserver
{
    // =========================================================================
    // Motor / mechanics parameters  (set from NvmSettings before first use)
    // =========================================================================

    /// Permanent-magnet flux linkage ψ_PM [Wb].
    T psi{static_cast<T>(0)};

    /// d-axis inductance L_d [H].
    T L_d{static_cast<T>(1e-3)};

    /// q-axis inductance L_q [H].
    T L_q{static_cast<T>(1e-3)};

    /// Rotor + load inertia J [kg·m²].
    T J{static_cast<T>(1e-4)};

    // =========================================================================
    // Speed limits  (set from motor / hardware constraints)
    // =========================================================================

    /// Maximum electrical angular velocity in the forward direction [rad/s].
    T omega_max{static_cast<T>(2000.0)};

    /// Maximum electrical angular velocity in the reverse direction [rad/s]
    /// (must be ≤ 0).
    T omega_min{static_cast<T>(-2000.0)};

    // =========================================================================
    // Kalman filter noise parameters
    // =========================================================================

    /// Process noise variance Q (applied to all three state covariances).
    T Q{static_cast<T>(1e-5)};

    /// Measurement noise variance R.
    T R{static_cast<T>(1e-4)};

    // =========================================================================
    // Observer state
    // =========================================================================

    /// Estimated electrical angular velocity ω̂ [rad/s].
    T omega{static_cast<T>(0)};

    /// Estimated electrical rotor angle θ̂ [rad], wrapped to (−π, π].
    T theta{static_cast<T>(0)};

    /// Estimated load torque m̂_l [N·m].
    T m_l{static_cast<T>(0)};

    /// Last computed electrical torque m_el [N·m].
    T m_el{static_cast<T>(0)};

    // =========================================================================
    // Outputs (updated by predict() and inject_angle_error())
    // =========================================================================

    /// sin(θ̂) — ready for use in Park / inverse-Park transforms.
    T sin_theta{static_cast<T>(0)};

    /// cos(θ̂) — ready for use in Park / inverse-Park transforms.
    T cos_theta{static_cast<T>(1)};

    // =========================================================================
    // Kalman filter internal state  (covariance matrices)
    // =========================================================================

    /// State error covariance matrix P (3×3, row-major).
    T p[3][3]{};

    /// Predicted covariance matrix P_k (3×3, row-major, working storage).
    T pk[3][3]{};

    /// Kalman gain vector k (3 elements, one per state).
    T k[3]{};

    /// Innovation covariance scalar S.
    T s{};

    // =========================================================================
    // Public API
    // =========================================================================

    /**
     * @brief Prediction step — propagate the mechanical model one control cycle.
     *
     * Computes the electrical torque from the rotor-frame d/q current vector,
     * integrates the mechanical equations of motion, and clamps ω̂ to
     * [omega_min, omega_max] to enforce hardware limits.
     *
     * Call once per PWM/control interrupt **before** inject_angle_error().
     *
     * @param i_dq  Measured rotor-frame d/q stator current [A].
     * @param dt    Control period [s].
     */
    constexpr void
    predict(const system::RotorReference<T>& i_dq, const T dt) noexcept
    {
        const T tsj = dt / J;

        // Electric torque: T_e = (3/2) · [ψ_PM · i_q + (L_d − L_q) · i_d · i_q]
        m_el = static_cast<T>(1.5) *
               (psi * i_dq.q + (L_d - L_q) * i_dq.d * i_dq.q);

        // Integrate angular velocity
        omega += tsj * (m_el - m_l);

        // Clamp to hardware speed limits
        if (omega > omega_max)
            omega = omega_max;
        else if (omega < omega_min)
            omega = omega_min;

        // Recover from NaN/Inf
        if (!std::isfinite(omega))
            omega = static_cast<T>(0);

        // Integrate angle
        theta += omega * dt;
        wrap_angle(theta);

        sin_theta = std::sin(theta);
        cos_theta = std::cos(theta);
    }

    /**
     * @brief Measurement update — Kalman correction from an external angle error.
     *
     * Propagates the state-error covariance using the linearised mechanics model
     * and then applies the Kalman-optimal correction to [ω̂, θ̂, m̂_l].
     *
     * This is also the entry point used by the ASM flux observer and HFI
     * observer to inject auxiliary angle information into the same estimator.
     *
     * Call once per PWM/control interrupt **after** predict().
     *
     * @param angle_error  Innovation ε = θ_measured − θ̂ [rad].
     * @param dt           Control period [s].
     */
    constexpr void
    inject_angle_error(const T angle_error, const T dt) noexcept
    {
        const T tsj = dt / J;

        // -----------------------------------------------------------------
        // Covariance prediction  P_k = F·P·Fᵀ + Q·I
        //
        // State-transition matrix (linearised about current state):
        //   F = | 1    0   -tsj |
        //       | dt   1    0   |
        //       | 0    0    1   |
        // -----------------------------------------------------------------

        pk[0][2] = p[0][2] - p[2][2] * tsj;

        pk[0][0] = p[0][0] + Q - p[2][0] * tsj - pk[0][2] * tsj;
        pk[1][0] = p[1][0] + p[0][0] * dt - tsj * (p[1][2] + p[0][2] * dt);
        pk[2][0] = p[2][0] - tsj * p[2][2];

        pk[0][1] = p[0][1] + dt * (p[0][0] - tsj * p[2][0]) - tsj * p[2][1];
        pk[1][1] = p[1][1] + Q + p[0][1] * dt + dt * (p[1][0] + p[0][0] * dt);
        pk[2][1] = p[2][1] + p[2][0] * dt;

        // p[0][2] already stored in pk[0][2] above
        pk[1][2] = p[1][2] + p[0][2] * dt;
        pk[2][2] = p[2][2] + Q;

        // -----------------------------------------------------------------
        // Innovation covariance and Kalman gain
        //   S   = H · P_k · Hᵀ + R  =  pk[1][1] + R  (H = [0, 1, 0])
        //   k   = P_k · Hᵀ / S
        // -----------------------------------------------------------------
        s    = static_cast<T>(1) / (pk[1][1] + R);
        k[0] = pk[0][1] * s;
        k[1] = pk[1][1] * s;
        k[2] = pk[2][1] * s;

        // -----------------------------------------------------------------
        // Covariance update  P = (I − k·H) · P_k
        // -----------------------------------------------------------------
        const T k1m1 = k[1] - static_cast<T>(1);

        p[0][0] = pk[0][0] - k[0] * pk[1][0];
        p[1][0] = -pk[1][0] * k1m1;
        p[2][0] = pk[2][0] - k[2] * pk[1][0];

        p[0][1] = pk[0][1] - k[0] * pk[1][1];
        p[1][1] = -pk[1][1] * k1m1;
        p[2][1] = pk[2][1] - k[2] * pk[1][1];

        p[0][2] = pk[0][2] - k[0] * pk[1][2];
        p[1][2] = -pk[1][2] * k1m1;
        p[2][2] = pk[2][2] - k[2] * pk[1][2];

        // -----------------------------------------------------------------
        // State correction
        // -----------------------------------------------------------------
        omega += k[0] * angle_error;
        theta += k[1] * angle_error;
        m_l   += k[2] * angle_error;

        // Re-clamp after correction
        if (omega > omega_max)
            omega = omega_max;
        else if (omega < omega_min)
            omega = omega_min;

        if (!std::isfinite(omega))
            omega = static_cast<T>(0);

        wrap_angle(theta);

        sin_theta = std::sin(theta);
        cos_theta = std::cos(theta);
    }

    /**
     * @brief Reset all observer state to zero.
     *
     * Call on fault recovery, mode transitions, or when a reliable initial
     * angle and speed are available.
     *
     * @param theta_init  Initial electrical angle [rad].
     * @param omega_init  Initial electrical angular velocity [rad/s].
     */
    constexpr void
    reset(const T theta_init = static_cast<T>(0),
          const T omega_init = static_cast<T>(0)) noexcept
    {
        omega = omega_init;
        theta = theta_init;
        m_l   = static_cast<T>(0);
        m_el  = static_cast<T>(0);

        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                p[i][j] = pk[i][j] = static_cast<T>(0);

        for (int i = 0; i < 3; ++i)
            k[i] = static_cast<T>(0);

        s = static_cast<T>(0);

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
