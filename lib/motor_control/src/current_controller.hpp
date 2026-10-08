/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file current_controller.hpp
 *    @brief d/q-axis current controller.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cmath>
#include <concepts>
#include "rotor_system.hpp"

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
 * Anti-windup (back-calculation)
 * --------------------------------
 * The output is always hard-limited to the voltage circle, including when
 * the feedforward terms alone drive saturation.  The integrator uses a
 * back-calculation scheme rather than a simple freeze:
 *
 *   integrator += (ki * e + kb * (u_limited - u_raw)) * dt
 *
 * When unsaturated `u_limited == u_raw`, so the correction term is zero and
 * the integrator advances normally.  When the output is clipped, the
 * difference `(u_limited - u_raw)` is negative, which winds the integrator
 * back proportionally.  Because the correction is applied to the *total*
 * output difference (PI + feedforward), the integrator is also correctly
 * de-saturated when the feedforward terms alone push the output over the
 * limit, avoiding the steady-state current error that a pure-freeze strategy
 * produces in that case.  The back-calculation gain `kb` controls how
 * aggressively the integrator is wound back; a value of `ki / kp` gives a
 * tracking time-constant equal to the integral time Ti = kp / ki.
 *
 * Usage
 * -----
 * @code
 *   // Initialise from NvmSettings:
 *   unimoc::control::CurrentController<float> cc;
 *   cc.kp_d = settings.current_kp_d;  cc.ki_d = settings.current_ki_d;
 *   cc.kp_q = settings.current_kp_q;  cc.ki_q = settings.current_ki_q;
 *   cc.kb_d = settings.current_kb_d;  cc.kb_q = settings.current_kb_q;
 *   cc.L_d  = settings.l_d;            cc.L_q = settings.l_q;
 *   cc.psi  = settings.flux_pm;
 *   cc.v_max = settings.current_v_max;
 *
 *   // In the ISR:
 *   auto u_dq = cc.update(i_ref, i_meas, omega, dt_fast, v_dc);
 * @endcode
 *
 * @tparam T  Floating-point type (float by default).
 */
template <std::floating_point T = float>
struct CurrentController {
  // =========================================================================
  // Motor parameters  (set from NvmSettings before first use)
  // =========================================================================

  /// d-axis inductance L_d [H].
  unit::Inductance L_d{1.0_mH};

  /// q-axis inductance L_q [H].
  unit::Inductance L_q{1.0_mH};

  /// Permanent-magnet flux linkage ψ_PM [Wb].
  unit::MagneticFlux psi{};

  // =========================================================================
  // Controller gains
  // =========================================================================

  /// d-axis proportional gain [V/A].
  unit::VoltagePerCurrent kp_d{1.0_V_per_A};

  /// d-axis integral gain [V/(A·s)].
  unit::VoltagePerCurrentTime ki_d{100.0_V_per_A_s};

  /// q-axis proportional gain [V/A].
  unit::VoltagePerCurrent kp_q{1.0_V_per_A};

  /// q-axis integral gain [V/(A·s)].
  unit::VoltagePerCurrentTime ki_q{100.0_V_per_A_s};

  // =========================================================================
  // Back-calculation anti-windup gains
  // =========================================================================

  /// d-axis back-calculation gain [1/s].
  /// Controls how aggressively the integrator is wound back when the output
  /// is saturated.  A value of ki_d / kp_d makes the tracking time-constant
  /// equal to the integral time Ti = kp_d / ki_d.  Must be > 0.
  unit::InverseTime kb_d{100.0_per_s};

  /// q-axis back-calculation gain [1/s].
  unit::InverseTime kb_q{100.0_per_s};

  // =========================================================================
  // Output limit
  // =========================================================================

  /// Maximum voltage vector magnitude as a fraction of V_dc (range (0, 1]).
  /// Typically set to 0.9 to preserve SVM headroom and avoid over-modulation.
  /// The actual voltage limit applied inside update() is v_max * v_dc [V].
  unit::Ratio v_max{0.9_ratio};

  // =========================================================================
  // Integrator state
  // =========================================================================

  /// d-axis integrator accumulator [V].
  unit::Voltage integrator_d{};

  /// q-axis integrator accumulator [V].
  unit::Voltage integrator_q{};

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
  constexpr system::Rotor<unit::Voltage> update(const system::Rotor<unit::Current>& i_ref,
                                                const system::Rotor<unit::Current>& i_meas,
                                                const unit::AngularVelocity omega,
                                                const unit::Time dt,
                                                const unit::Voltage v_dc) noexcept {
    // --- Current errors ---
    const T e_d = i_ref.d.Value() - i_meas.d.Value();
    const T e_q = i_ref.q.Value() - i_meas.q.Value();

    // --- Cross-coupling feedforward ---
    //   v_ff_d = −ω · L_q · i_q
    //   v_ff_q = +ω · (L_d · i_d + ψ_PM)
    const T v_ff_d = -omega.Value() * L_q.Value() * i_meas.q.Value();
    const T v_ff_q = omega.Value() * (L_d.Value() * i_meas.d.Value() + psi.Value());

    // --- Total output before limiting (PI + feedforward) ---
    const T u_d_raw = kp_d.Value() * e_d + integrator_d.Value() + v_ff_d;
    const T u_q_raw = kp_q.Value() * e_q + integrator_q.Value() + v_ff_q;

    // --- Circular voltage-vector limiting ---
    // The limit is expressed in volts: v_limit = v_max [fraction] * v_dc [V].
    // The total output (PI + feedforward) is always hard-clamped to the
    // voltage circle, even when the feedforward terms alone push it over.
    const T v_limit = v_max.Value() * v_dc.Value();
    const T mag_sq = u_d_raw * u_d_raw + u_q_raw * u_q_raw;
    T u_d = u_d_raw;
    T u_q = u_q_raw;

    if (mag_sq > v_limit * v_limit) {
      const T scale = v_limit / std::sqrt(mag_sq);
      u_d = u_d_raw * scale;
      u_q = u_q_raw * scale;
    }

    // --- Integrator update with back-calculation anti-windup ---
    // The tracking error (u - u_raw) is zero when unsaturated and becomes
    // negative when the output is clipped.  The kb term feeds this error
    // back into the integrator, winding it back in proportion to how much
    // limiting occurred.  This prevents windup both when the PI output
    // and when the feedforward terms alone drive the total output into
    // saturation — unlike a simple freeze which only detects total
    // saturation and cannot distinguish the two cases.
    integrator_d = unit::Voltage{integrator_d.Value() +
                                 (ki_d.Value() * e_d + kb_d.Value() * (u_d - u_d_raw)) * dt.Value()};
    integrator_q = unit::Voltage{integrator_q.Value() +
                                 (ki_q.Value() * e_q + kb_q.Value() * (u_q - u_q_raw)) * dt.Value()};

    return system::Rotor<unit::Voltage>{u_d, u_q};
  }

  /**
   * @brief Reset integrator state.
   *
   * Call when re-enabling the controller after a fault or mode transition.
   */
  constexpr void reset() noexcept {
    integrator_d = unit::Voltage{};
    integrator_q = unit::Voltage{};
  }
};

}  // namespace control
}  // namespace unimoc
