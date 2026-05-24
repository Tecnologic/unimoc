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

#ifndef UNIMOC_CONTROL_FIELD_WEAKENING_H_
#define UNIMOC_CONTROL_FIELD_WEAKENING_H_

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
 * @brief Field-weakening controller with i_d current control.
 *
 * Above base speed the required voltage vector magnitude exceeds the available
 * DC-link headroom.  Field weakening reduces the rotor flux by commanding a
 * negative d-axis current, thereby lowering the back-EMF and extending the
 * speed range.
 *
 * Implementation
 * --------------
 * A pure integrating (I-only) controller acts on the voltage headroom error:
 *
 *   e_v = V_max − |V_s|
 *
 * When the applied voltage vector stays below V_max the error is positive and
 * the integrator output (i_d_fw) is driven toward zero.  When the limit is
 * exceeded the error is negative and i_d_fw is driven negative, weakening the
 * field.
 *
 * The output is clamped to [i_d_min, 0] so that field strengthening (positive
 * i_d) is never commanded, and excessive de-magnetisation is prevented.
 *
 * @tparam T  Floating-point type (float by default).
 */
template <std::floating_point T = float>
struct FieldWeakening
{
    /// Maximum allowed voltage vector magnitude (normalised by V_dc, range (0, 1]).
    T v_max{static_cast<T>(0.9)};

    /// Integrator gain K_i [A / (V·s)].
    T ki{static_cast<T>(10.0)};

    /// Most negative i_d allowed [A] (prevents de-magnetisation).
    T i_d_min{static_cast<T>(-10.0)};

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------

    /// Integrator state: field-weakening d-axis current [A].
    T i_d_fw{static_cast<T>(0.0)};

    /**
     * @brief Update the field-weakening integrator and return the i_d correction.
     *
     * Call this once per control cycle.  The returned value should be added to
     * the MTPA i_d reference to obtain the total d-axis current set-point.
     *
     * @param v_s  Applied voltage vector in the stationary α/β frame
     *             (normalised by V_dc).
     * @param dt   Control cycle period [s].
     * @return     Field-weakening i_d correction [A]  (≤ 0).
     */
    constexpr T
    update(const system::StatorReference<T>& v_s, const T dt) noexcept
    {
        const T v_mag = std::sqrt(v_s.alpha * v_s.alpha + v_s.beta * v_s.beta);
        const T error = v_max - v_mag;

        // Integrate voltage headroom error into i_d correction
        i_d_fw += ki * error * dt;

        // Clamp: i_d_fw must remain in [i_d_min, 0]
        i_d_fw = std::clamp(i_d_fw, i_d_min, static_cast<T>(0));

        return i_d_fw;
    }

    /// Reset integrator state (call on enable / mode transitions).
    constexpr void
    reset() noexcept
    {
        i_d_fw = static_cast<T>(0);
    }
};

}  // namespace control
}  // namespace unimoc

#endif /* UNIMOC_CONTROL_FIELD_WEAKENING_H_ */
