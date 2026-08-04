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

#ifndef UNIMOC_CONTROL_DEAD_TIME_COMPENSATION_H_
#define UNIMOC_CONTROL_DEAD_TIME_COMPENSATION_H_

#include <algorithm>
#include <cmath>
#include <concepts>
#include "StatorReference.hpp"

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
struct DeadTimeCompensation
{
    /// Dead time of the gate driver [s].
    T dead_time{static_cast<T>(0.0)};

    /// PWM switching frequency [Hz].
    T f_pwm{static_cast<T>(10000.0)};

    /**
     * @brief Threshold current [A] for the soft sign function.
     *
     * Currents with magnitude below this value produce a linearly-interpolated
     * sign rather than a hard ±1, which avoids large voltage spikes and chattering
     * near zero-crossings.
     */
    T i_threshold{static_cast<T>(0.1)};

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
    [[nodiscard]] constexpr system::StatorReference<T>
    calculate(const system::StatorReference<T>& i_ab) const noexcept
    {
        // --- Reconstruct three-phase currents from α/β ---
        constexpr T k = static_cast<T>(0.8660254037844386);  // √3 / 2

        T ia = i_ab.alpha;
        T ib = static_cast<T>(-0.5) * i_ab.alpha + k * i_ab.beta;
        T ic = static_cast<T>(-0.5) * i_ab.alpha - k * i_ab.beta;

        // --- Soft sign function: clamp(i / threshold, -1, +1) ---
        // This provides linear interpolation through zero, preventing chattering.
        T sign_a = std::clamp(ia / i_threshold, static_cast<T>(-1), static_cast<T>(1));
        T sign_b = std::clamp(ib / i_threshold, static_cast<T>(-1), static_cast<T>(1));
        T sign_c = std::clamp(ic / i_threshold, static_cast<T>(-1), static_cast<T>(1));

        // Normalised per-phase voltage error: sign · t_dead · f_pwm
        T dt_norm = dead_time * f_pwm;
        T dva     = sign_a * dt_norm;
        T dvb     = sign_b * dt_norm;
        T dvc     = sign_c * dt_norm;

        // --- Clarke transform (amplitude-invariant) back to α/β ---
        constexpr T two_thirds = static_cast<T>(2.0 / 3.0);

        T d_alpha = two_thirds * (dva - static_cast<T>(0.5) * dvb - static_cast<T>(0.5) * dvc);
        T d_beta  = two_thirds * (k * dvb - k * dvc);

        return system::StatorReference<T>{d_alpha, d_beta};
    }
};

}  // namespace control
}  // namespace unimoc

#endif /* UNIMOC_CONTROL_DEAD_TIME_COMPENSATION_H_ */
