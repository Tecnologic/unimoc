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

#ifndef UNIMOC_CONTROL_SVM_H_
#define UNIMOC_CONTROL_SVM_H_

#include <algorithm>
#include <array>
#include <concepts>
#include "ThreePhase.hpp"
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
 * @brief Space Vector Modulation (SVPWM) with centered PWM.
 *
 * Converts a stationary-frame voltage reference (α/β, normalised to the DC-link
 * voltage) into three PWM duty cycles.
 *
 * Conventions
 * -----------
 * - v_alpha and v_beta are normalised by the DC bus voltage V_dc so that
 *   a value of 1.0 represents V_dc.  The full linear SVPWM range corresponds to
 *   a vector magnitude of 1/√3 ≈ 0.577.
 * - The zero-sequence offset is chosen so that the sum of the three duty cycles
 *   is always 1.5 (each centred around 0.5), giving centred / symmetric PWM.
 * - After centring the output duty cycles are clamped to [duty_min, duty_max]
 *   (default 5 % … 95 %) to leave headroom for current measurement and dead-time
 *   compensation without preloading the timer counter.
 *
 * @tparam T  Floating-point type (float by default).
 */
template <std::floating_point T = float>
struct Svm
{
    /// Minimum duty cycle (keeps time for ADC sampling and dead-time headroom).
    T duty_min{static_cast<T>(0.05)};
    /// Maximum duty cycle (symmetric headroom on the upper side).
    T duty_max{static_cast<T>(0.95)};

    /**
     * @brief Compute three-phase duty cycles from a stationary-frame voltage vector.
     *
     * @param v  Voltage vector with alpha/beta components normalised by V_dc.
     * @return   Three-phase duty cycles [0, 1] clamped to [duty_min, duty_max].
     */
    [[nodiscard]] constexpr system::ThreePhase<T>
    calculate(const system::StatorReference<T>& v) const noexcept
    {
        // --- Inverse Clarke (amplitude-invariant) ---
        // Transforms the α/β reference into three phase-voltage references.
        constexpr T k = static_cast<T>(0.8660254037844386);  // √3 / 2

        T va = v.alpha;
        T vb = static_cast<T>(-0.5) * v.alpha + k * v.beta;
        T vc = static_cast<T>(-0.5) * v.alpha - k * v.beta;

        // --- Zero-sequence injection for centred SVM ---
        // The zero-sequence component centres the modulated waveforms so that the
        // mid-point of (max + min) is always at 0.  Adding it to each phase shifts
        // all duties to be symmetric around 0.5.
        T vmax = std::max({va, vb, vc});
        T vmin = std::min({va, vb, vc});
        T v0   = static_cast<T>(-0.5) * (vmax + vmin);

        // Convert phase voltages [-0.5, 0.5] → duty cycles [0, 1]
        T da = static_cast<T>(0.5) + va + v0;
        T db = static_cast<T>(0.5) + vb + v0;
        T dc = static_cast<T>(0.5) + vc + v0;

        // --- Clamp to [duty_min, duty_max] ---
        da = std::clamp(da, duty_min, duty_max);
        db = std::clamp(db, duty_min, duty_max);
        dc = std::clamp(dc, duty_min, duty_max);

        return system::ThreePhase<T>{da, db, dc};
    }
};

}  // namespace control
}  // namespace unimoc

#endif /* UNIMOC_CONTROL_SVM_H_ */
