/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file field_weakening.hpp
 *    @brief Field-weakening controller that lowers the d-axis current above base speed.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cmath>
#include <concepts>
#include "stator_system.hpp"
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
struct FieldWeakening {
  /// Maximum allowed voltage vector magnitude [V].
  unit::Voltage v_max{0.9_V};

  /// Integrator gain K_i [A / (V·s)].
  unit::CurrentPerVoltageTime ki{10.0_A_per_V_s};

  /// Most negative i_d allowed [A] (prevents de-magnetisation).
  unit::Current i_d_min{-10.0_A};

  // -------------------------------------------------------------------------
  // State
  // -------------------------------------------------------------------------

  /// Integrator state: field-weakening d-axis current [A].
  unit::Current i_d_fw;

  /**
   * @brief Update the field-weakening integrator and return the i_d correction.
   *
   * Call this once per control cycle.  The returned value should be added to
   * the MTPA i_d reference to obtain the total d-axis current set-point.
   *
  * @param v_s  Applied voltage vector in the stationary α/β frame [V].
   * @param dt   Control cycle period [s].
  /// @return     Field-weakening i_d correction [A] (≤ 0).
   */
  constexpr unit::Current Update(const system::Stator<unit::Voltage>& v_s, const unit::Time dt) noexcept {
    const T kVAlpha = static_cast<T>(v_s.alpha.Value());
    const T kVBeta = static_cast<T>(v_s.beta.Value());
    const unit::Voltage kVMag{std::sqrt(kVAlpha * kVAlpha + kVBeta * kVBeta)};
    const T kError = v_max.Value() - kVMag.Value();

    // Integrate voltage headroom error into i_d correction
    i_d_fw = unit::Current{i_d_fw.Value() + ki.Value() * kError * dt.Value()};

    // Clamp: i_d_fw must remain in [i_d_min, 0]
    i_d_fw = unit::Current{i_d_fw.Value()}.Clamp(i_d_min.Value(), 0.0F);

    return i_d_fw;
  }

  /// Reset integrator state (call on enable / mode transitions).
  constexpr void Reset() noexcept { i_d_fw = unit::Current{}; }
};

}  // namespace control
}  // namespace unimoc
