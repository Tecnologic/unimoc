/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file mechanical_observer.hpp
 *    @brief Unit-typed mechanical angle, speed, and torque observer.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cmath>
#include <concepts>
#include <numbers>
#include "nvm_settings.hpp"
#include "rotor_system.hpp"

/**
 * @namespace unimoc global namespace
 */
namespace unimoc {
/**
 * @namespace observer observer algorithms namespace
 */
namespace observer {

using namespace unit;

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
struct MechanicalObserver {
  // =========================================================================
  // Motor / mechanics parameters (loaded from NvmSettings by init())
  // =========================================================================

  /// Permanent-magnet flux linkage ψ_PM [Wb].
  unit::MagneticFlux psi{};

  /// d-axis inductance L_d [H].
  unit::Inductance L_d{};

  /// q-axis inductance L_q [H].
  unit::Inductance L_q{};

  /// Rotor + load inertia J [kg·m²].
  unit::Inertia J{};

  // =========================================================================
  // Speed limits  (set from motor / hardware constraints)
  // =========================================================================

  /// Maximum electrical angular velocity in the forward direction [rad/s].
  unit::AngularVelocity omega_max{};

  /// Maximum electrical angular velocity in the reverse direction [rad/s]
  /// (must be ≤ 0).
  unit::AngularVelocity omega_min{};

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
  unit::AngularVelocity omega{};

  /// Estimated electrical rotor angle θ̂ [rad], wrapped to (−π, π].
  unit::Angle theta{};

  /// Estimated load torque m̂_l [N·m].
  unit::Torque m_l{};

  /// Last computed electrical torque m_el [N·m].
  unit::Torque m_el{};

  // =========================================================================
  // Outputs (updated by predict() and inject_angle_error())
  // =========================================================================

  /// sin(θ̂) — ready for use in Park / inverse-Park transforms.
  unit::Ratio sin_theta{};

  /// cos(θ̂) — ready for use in Park / inverse-Park transforms.
  unit::Ratio cos_theta{1.0_ratio};

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
   * @brief Load mechanical observer parameters from non-volatile settings.
   *
   * This also clears the observer state so a new settings snapshot cannot
   * be combined with state produced by older motor parameters.
   *
   * @param settings Validated NVM settings.
   */
  constexpr void init(const settings::NvmSettings& settings) noexcept {
    psi = settings.flux_pm;
    L_d = settings.l_d;
    L_q = settings.l_q;
    J = settings.motor_j;
    omega_max = settings.motor_omega_max;
    omega_min = settings.motor_omega_min;
    Q = settings.mech_obs_q;
    R = settings.mech_obs_r;
    reset();
  }

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
   * @param dt    Control period.
   */
  constexpr void predict(const system::Rotor<unit::Current>& i_dq, const unit::Time dt) noexcept {
    const T tsj = dt.Value() / J.Value();

    // Electric torque: T_e = (3/2) · [ψ_PM · i_q + (L_d − L_q) · i_d · i_q]
    m_el =
        unit::Torque{static_cast<T>(1.5) * (psi.Value() * i_dq.q.Value() +
                                            (L_d.Value() - L_q.Value()) * i_dq.d.Value() * i_dq.q.Value())};

    // Integrate angular velocity
    omega += unit::AngularVelocity{tsj * (m_el.Value() - m_l.Value())};

    // Clamp to hardware speed limits
    if (omega > omega_max)
      omega = omega_max;
    else if (omega < omega_min)
      omega = omega_min;

    // Recover from NaN/Inf
    if (!std::isfinite(omega.Value())) omega = unit::AngularVelocity{};

    // Integrate angle
    theta += unit::Angle{omega.Value() * dt.Value()};
    wrap_angle(theta);

    sin_theta = unit::Ratio{std::sin(theta.Value())};
    cos_theta = unit::Ratio{std::cos(theta.Value())};
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
   * @param angle_error  Innovation ε = θ_measured − θ̂.
   * @param dt           Control period.
   */
  constexpr void inject_angle_error(const unit::Angle angle_error, const unit::Time dt) noexcept {
    const T tsj = dt.Value() / J.Value();
    const T angle_error_value = angle_error.Value();

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
    pk[1][0] = p[1][0] + p[0][0] * dt.Value() - tsj * (p[1][2] + p[0][2] * dt.Value());
    pk[2][0] = p[2][0] - tsj * p[2][2];

    pk[0][1] = p[0][1] + dt.Value() * (p[0][0] - tsj * p[2][0]) - tsj * p[2][1];
    pk[1][1] = p[1][1] + Q + p[0][1] * dt.Value() + dt.Value() * (p[1][0] + p[0][0] * dt.Value());
    pk[2][1] = p[2][1] + p[2][0] * dt.Value();

    // p[0][2] already stored in pk[0][2] above
    pk[1][2] = p[1][2] + p[0][2] * dt.Value();
    pk[2][2] = p[2][2] + Q;

    // -----------------------------------------------------------------
    // Innovation covariance and Kalman gain
    //   S   = H · P_k · Hᵀ + R  =  pk[1][1] + R  (H = [0, 1, 0])
    //   k   = P_k · Hᵀ / S
    // -----------------------------------------------------------------
    s = static_cast<T>(1) / (pk[1][1] + R);
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
    omega += unit::AngularVelocity{k[0] * angle_error_value};
    theta += unit::Angle{k[1] * angle_error_value};
    m_l += unit::Torque{k[2] * angle_error_value};

    // Re-clamp after correction
    if (omega > omega_max)
      omega = omega_max;
    else if (omega < omega_min)
      omega = omega_min;

    if (!std::isfinite(omega.Value())) omega = unit::AngularVelocity{};

    wrap_angle(theta);

    sin_theta = unit::Ratio{std::sin(theta.Value())};
    cos_theta = unit::Ratio{std::cos(theta.Value())};
  }

  /**
   * @brief Reset all observer state to zero.
   *
   * Call on fault recovery, mode transitions, or when a reliable initial
   * angle and speed are available.
   *
   * @param theta_init  Initial electrical angle.
   * @param omega_init  Initial electrical angular velocity.
   */
  constexpr void reset(const unit::Angle theta_init = unit::Angle{},
                       const unit::AngularVelocity omega_init = unit::AngularVelocity{}) noexcept {
    omega = omega_init;
    theta = theta_init;
    m_l = unit::Torque{};
    m_el = unit::Torque{};

    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) p[i][j] = pk[i][j] = static_cast<T>(0);

    for (int i = 0; i < 3; ++i) k[i] = static_cast<T>(0);

    s = static_cast<T>(0);

    sin_theta = unit::Ratio{std::sin(theta.Value())};
    cos_theta = unit::Ratio{std::cos(theta.Value())};
  }

 private:
  /// Wrap angle to (−π, π].
  static constexpr void wrap_angle(unit::Angle& angle) noexcept {
    constexpr T pi = std::numbers::pi_v<T>;
    constexpr T two_pi = static_cast<T>(2) * pi;
    T angle_value = angle.Value();

    while (angle_value > pi) angle_value -= two_pi;
    while (angle_value <= -pi) angle_value += two_pi;

    angle = unit::Angle{angle_value};
  }
};

}  // namespace observer
}  // namespace unimoc
