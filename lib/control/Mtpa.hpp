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

#ifndef UNIMOC_CONTROL_MTPA_H_
#define UNIMOC_CONTROL_MTPA_H_

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
 * @brief Maximum Torque Per Ampere (MTPA) algorithm for interior PMSM.
 *
 * For a surface PMSM (L_d = L_q) the optimal d-axis current is zero.
 * For an interior PMSM (L_d ≠ L_q, saliency ΔL = L_d − L_q) exploiting the
 * reluctance torque requires a non-zero i_d.  The MTPA condition minimises the
 * total stator current magnitude for a given torque requirement.
 *
 * Closed-form MTPA solution:
 *
 *   i_d* = ψ_PM / (2·ΔL) − √( (ψ_PM / (2·ΔL))² + (i_s / 2)² )
 *
 * where ΔL = L_d − L_q (negative for IPMSM with L_d < L_q), ψ_PM is the
 * permanent-magnet flux linkage, and i_s = |i_dq| is the total current amplitude.
 *
 * For surface PMSM or when ΔL ≈ 0 the formula degenerates; in that case
 * i_d* = 0 is returned.
 *
 * @tparam T  Floating-point type (float by default).
 */
template <std::floating_point T = float>
struct Mtpa
{
    /// Permanent-magnet flux linkage ψ_PM [Wb].
    T flux_pm{static_cast<T>(0.0)};

    /// d-axis inductance L_d [H].
    T L_d{static_cast<T>(1e-3)};

    /// q-axis inductance L_q [H].
    T L_q{static_cast<T>(1e-3)};

    /**
     * @brief Compute the MTPA d-axis current reference.
     *
     * @param i_s  Desired total stator current magnitude |i_dq| [A].
     *             Must be non-negative.
     * @return     Optimal d-axis current i_d* [A].
     *             For SPMSM (L_d == L_q) or trivial flux this returns 0.
     */
    [[nodiscard]] constexpr T
    calculate(const T i_s) const noexcept
    {
        const T delta_L = L_d - L_q;

        // Avoid division by zero (surface PMSM or no saliency)
        if (std::abs(delta_L) < static_cast<T>(1e-9))
        {
            return static_cast<T>(0);
        }

        // Half-saliency denominator term
        const T xi    = flux_pm / (static_cast<T>(2) * delta_L);
        const T i_s_2 = i_s * static_cast<T>(0.5);

        // MTPA closed-form solution
        const T i_d = xi - std::copysign(std::sqrt(xi * xi + i_s_2 * i_s_2),
                                          static_cast<T>(1));

        // Limit to current magnitude (i_d must not exceed i_s in absolute value)
        return std::clamp(i_d, -i_s, static_cast<T>(0));
    }
};

}  // namespace control
}  // namespace unimoc

#endif /* UNIMOC_CONTROL_MTPA_H_ */
