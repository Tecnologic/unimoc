/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file mtpa.hpp
 *    @brief Maximum-torque-per-ampere current split for interior PMSM.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cmath>
#include <concepts>
#include "units.hpp"

/**
 * @namespace unimoc global namespace
 */
namespace unimoc {
/**
 * @namespace control control algorithms namespace
 */
namespace control {

using namespace unit;

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
struct Mtpa {
  /// Permanent-magnet flux linkage ψ_PM [Wb].
  unit::MagneticFlux flux_pm{};

  /// d-axis inductance L_d [H].
  unit::Inductance L_d{1.0_mH};

  /// q-axis inductance L_q [H].
  unit::Inductance L_q{1.0_mH};

  /**
   * @brief Compute the MTPA d-axis current reference.
   *
   * @param i_s  Desired total stator current magnitude |i_dq| [A].
   *             Must be non-negative.
   * @return     Optimal d-axis current i_d* [A].
   *             For SPMSM (L_d == L_q) or trivial flux this returns 0.
   */
  [[nodiscard]] constexpr unit::Current calculate(const unit::Current i_s) const noexcept {
    const T delta_L = L_d.Value() - L_q.Value();

    // Avoid division by zero (surface PMSM or no saliency)
    if (std::abs(delta_L) < static_cast<T>(1e-9)) {
      return unit::Current{};
    }

    // Half-saliency denominator term
    const T xi = flux_pm.Value() / (static_cast<T>(2) * delta_L);
    const T i_s_2 = i_s.Value() * static_cast<T>(0.5);

    // MTPA closed-form solution
    const T i_d = xi - std::copysign(std::sqrt(xi * xi + i_s_2 * i_s_2), static_cast<T>(1));

    // Limit to current magnitude (i_d must not exceed i_s in absolute value)
    return unit::Current{i_d}.Clamp(-i_s.Value(), static_cast<T>(0));
  }
};

}  // namespace control
}  // namespace unimoc
