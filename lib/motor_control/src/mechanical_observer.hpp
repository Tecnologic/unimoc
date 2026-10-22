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
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include "angle_wrap.hpp"
#include "nvm_settings.hpp"
#include "rotor_system.hpp"

/**
 * @namespace unimoc::observer observer algorithms namespace
 */
namespace unimoc::observer {

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
 * Prediction step  (call Predict() once per control cycle)
 * ---------------------------------------------------------
 * The electric (shaft) torque is computed from the rotor-frame d/q currents:
 *
 *   m_el = (3/2) · p · [ψ_PM · i_q + (L_d − L_q) · i_d · i_q]
 *
 * The states are then propagated using Euler forward integration; the
 * electrical speed changes p times faster than the shaft speed:
 *
 *   ω̂  += (dt/J) · (p · m_el − m̂_l)
 *   θ̂  += ω̂ · dt
 *   m̂_l unchanged (load is modelled as a random walk)
 *
 * The load torque m̂_l is referred to the electrical speed (p times the
 * shaft load torque), so its process noise Q acts on the electrical speed
 * like that of the other states.
 *
 * After prediction ω̂ is clamped to twice [omega_min, omega_max], a guard
 * against a numeric runaway (the outer loop enforces the speed limits on
 * the setpoint; an estimate pinned at the limit would hide an overspeed),
 * and θ̂ is wrapped to (−π, π].
 *
 * Measurement updates  (docs/hotpath_plan.md, D2)
 * ----------------------------------------------
 * Once per control cycle PredictCovariance() propagates the state-error
 * covariance with the linearised state-transition matrix (process noise Q,
 * applied uniformly to the three state variances). Then any number of
 * independent measurements correct the state, each with its own noise:
 *
 *   UpdateAngle(ε, R):  ε = θ_measured − θ̂, wrapped to (−π, π]
 *                       (flux observer, Hall edge and sector, HFI)
 *   UpdateSpeed(ε, R):  ε = ω_measured − ω̂  (Hall speed bound)
 *
 * Each update computes the Kalman gain k for its measurement and corrects
 *
 *   ω̂  += k[0] · ε
 *   θ̂  += k[1] · ε
 *   m̂_l += k[2] · ε
 *
 * Sequential scalar updates are the exact Kalman update for independent
 * measurements. InjectAngleError(ε, dt) is PredictCovariance(dt) followed
 * by UpdateAngle(ε, R) with the default R, for observers that own a cycle.
 *
 * Outputs
 * -------
 * After each call to Predict() or an update, the fields
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
  unit::MagneticFlux psi;

  /// d-axis inductance L_d [H].
  unit::Inductance L_d;

  /// q-axis inductance L_q [H].
  unit::Inductance L_q;

  /// Rotor + load inertia J [kg·m²].
  unit::Inertia J;

  /// Pole-pair count p: electrical per mechanical angle.
  uint8_t pole_pairs{1U};

  // =========================================================================
  // Speed limits  (set from motor / hardware constraints)
  // =========================================================================

  /// Maximum electrical angular velocity in the forward direction [rad/s].
  unit::AngularVelocity omega_max;

  /// Maximum electrical angular velocity in the reverse direction [rad/s]
  /// (must be ≤ 0).
  unit::AngularVelocity omega_min;

  // =========================================================================
  // Kalman filter noise parameters
  // =========================================================================

  /// Process noise variance Q (applied to all three state covariances).
  T Q{static_cast<T>(1e-5)};

  /// Default measurement noise variance R of an angle measurement [rad²].
  T R{static_cast<T>(1e-4)};

  // =========================================================================
  // Observer state
  // =========================================================================

  /// Estimated electrical angular velocity ω̂ [rad/s].
  unit::AngularVelocity omega;

  /// Estimated electrical rotor angle θ̂ [rad], wrapped to (−π, π].
  unit::Angle theta;

  /// Estimated load torque m̂_l, referred to the electrical speed (p times the shaft load torque) [N·m].
  unit::Torque m_l;

  /// Last computed electrical (shaft) torque m_el [N·m].
  unit::Torque m_el;

  // =========================================================================
  // Outputs (updated by Predict() and the measurement updates)
  // =========================================================================

  /// sin(θ̂) — ready for use in Park / inverse-Park transforms.
  unit::Ratio sin_theta;

  /// cos(θ̂) — ready for use in Park / inverse-Park transforms.
  unit::Ratio cos_theta{1.0_ratio};

  // =========================================================================
  // Kalman filter internal state  (covariance matrices)
  // =========================================================================

  /// State error covariance matrix P (3×3, row-major).
  std::array<std::array<T, 3>, 3> p{};

  /// Predicted covariance matrix P_k (3×3, row-major, working storage).
  std::array<std::array<T, 3>, 3> pk{};

  /// Kalman gain vector k (3 elements, one per state).
  std::array<T, 3> k{};

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
  constexpr void Init(const settings::NvmSettings& settings) noexcept {
    psi = settings.flux_pm;
    L_d = settings.l_d;
    L_q = settings.l_q;
    J = settings.motor_j;
    pole_pairs = settings.pole_pairs;
    omega_max = settings.motor_omega_max;
    omega_min = settings.motor_omega_min;
    Q = settings.mech_obs_q;
    R = settings.mech_obs_r;
    Reset();
  }

  /**
   * @brief Prediction step — propagate the mechanical model one control cycle.
   *
   * Computes the electrical torque from the rotor-frame d/q current vector,
   * integrates the mechanical equations of motion, and clamps ω̂ to twice
   * [omega_min, omega_max] (numeric guard).
   *
   * Call once per control cycle after the measurement updates; it moves the
   * estimate to the reference time of the next cycle.
   *
   * @param i_dq  Measured rotor-frame d/q stator current [A].
   * @param dt    Control period.
   */
  constexpr void Predict(const system::Rotor<unit::Current>& i_dq, const unit::Time dt) noexcept {
    const T kTsj = dt.Value() / J.Value();
    const T kPolePairs = static_cast<T>(pole_pairs);

    // Shaft torque: T_e = (3/2) · p · [ψ_PM · i_q + (L_d − L_q) · i_d · i_q]
    m_el = unit::Torque{
        static_cast<T>(1.5) * kPolePairs *
        (psi.Value() * i_dq.q.Value() + (L_d.Value() - L_q.Value()) * i_dq.d.Value() * i_dq.q.Value())};

    // Integrate the electrical angular velocity
    omega += unit::AngularVelocity{kTsj * (kPolePairs * m_el.Value() - m_l.Value())};
    ClampSpeed();

    // Integrate angle
    theta += unit::Angle{omega.Value() * dt.Value()};
    theta = observer::WrapAngle(theta);

    sin_theta = unit::Ratio{std::sin(theta.Value())};
    cos_theta = unit::Ratio{std::cos(theta.Value())};
  }

  /**
   * @brief Covariance prediction for one control cycle: P = F·P·Fᵀ + Q·I.
   *
   * State-transition matrix, linearised about the current state:
   *
   *   F = | 1    0   -tsj |      tsj = dt / J
   *       | dt   1    0   |
   *       | 0    0    1   |
   *
   * Call once per control cycle, before the measurement updates of that cycle.
   *
   * @param dt Control period.
   */
  constexpr void PredictCovariance(const unit::Time dt) noexcept {
    const T kTsj = dt.Value() / J.Value();
    const T kDt = dt.Value();

    pk[0][2] = p[0][2] - p[2][2] * kTsj;

    pk[0][0] = p[0][0] + Q - p[2][0] * kTsj - pk[0][2] * kTsj;
    pk[1][0] = p[1][0] + p[0][0] * kDt - kTsj * (p[1][2] + p[0][2] * kDt);
    pk[2][0] = p[2][0] - kTsj * p[2][2];

    pk[0][1] = p[0][1] + kDt * (p[0][0] - kTsj * p[2][0]) - kTsj * p[2][1];
    pk[1][1] = p[1][1] + Q + p[0][1] * kDt + kDt * (p[1][0] + p[0][0] * kDt);
    pk[2][1] = p[2][1] + p[2][0] * kDt;

    // p[0][2] already stored in pk[0][2] above
    pk[1][2] = p[1][2] + p[0][2] * kDt;
    pk[2][2] = p[2][2] + Q;

    p = pk;
  }

  /**
   * @brief Kalman update with an angle measurement (H = [0, 1, 0]).
   *
   * @param innovation  ε = θ_measured − θ̂; wrapped to (−π, π] here.
   * @param noise       Measurement noise variance of this measurement [rad²]; positive.
   */
  constexpr void UpdateAngle(const unit::Angle innovation, const T noise) noexcept {
    Correct(1U, WrapAngle(innovation).Value(), noise);
  }

  /**
   * @brief Kalman update with a speed measurement (H = [1, 0, 0]).
   *
   * @param innovation  ε = ω_measured − ω̂ [rad/s, electrical].
   * @param noise       Measurement noise variance of this measurement [(rad/s)²]; positive.
   */
  constexpr void UpdateSpeed(const unit::AngularVelocity innovation, const T noise) noexcept {
    Correct(0U, innovation.Value(), noise);
  }

  /**
   * @brief Holds the estimate still: the rotor is known to stand.
   *
   * Sets ω̂ = 0 and the load torque to the torque balance (m̂_l = p · m_el),
   * so the prediction does not accelerate the estimate either. Angle and
   * covariance are kept. For a standstill detected from outside, such as
   * Hall sensors without an edge for a long time.
   */
  constexpr void HoldStill() noexcept {
    omega = unit::AngularVelocity{};
    m_l = m_el * static_cast<T>(pole_pairs);
  }

  /**
   * @brief Covariance prediction and angle update with the default noise R.
   *
   * This is the entry point of the ASM flux observer and the HFI observer,
   * which inject their angle information once per period.
   *
   * @param angle_error  Innovation ε = θ_measured − θ̂.
   * @param dt           Control period.
   */
  constexpr void InjectAngleError(const unit::Angle angle_error, const unit::Time dt) noexcept {
    PredictCovariance(dt);
    UpdateAngle(angle_error, R);
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
  constexpr void Reset(const unit::Angle theta_init = unit::Angle{},
                       const unit::AngularVelocity omega_init = unit::AngularVelocity{}) noexcept {
    omega = omega_init;
    theta = theta_init;
    m_l = unit::Torque{};
    m_el = unit::Torque{};

    p = {};
    pk = {};
    k = {};
    s = static_cast<T>(0);

    sin_theta = unit::Ratio{std::sin(theta.Value())};
    cos_theta = unit::Ratio{std::cos(theta.Value())};
  }

 private:
  /// Range of the speed estimate relative to the speed limits.
  static constexpr T kSpeedGuard = static_cast<T>(2);

  /// Clamps the speed estimate to twice the speed limits and recovers from NaN/Inf.
  constexpr void ClampSpeed() noexcept {
    const unit::AngularVelocity kUpper = omega_max * kSpeedGuard;
    const unit::AngularVelocity kLower = omega_min * kSpeedGuard;
    if (omega > kUpper) {
      omega = kUpper;
    } else if (omega < kLower) {
      omega = kLower;
    }
    if (!std::isfinite(omega.Value())) {
      omega = unit::AngularVelocity{};
    }
  }

  /**
   * @brief Scalar Kalman update of the measured state @p state (0 = ω, 1 = θ).
   *
   *   S = P[s][s] + noise,   k = P[:, s] / S,   P = P − k · P[s, :],   x += k · ε
   */
  constexpr void Correct(const std::size_t state, const T innovation, const T noise) noexcept {
    const T kVariance = p[state][state] + noise;
    if (!(kVariance > static_cast<T>(0)) || !std::isfinite(innovation)) {
      return;  // no usable measurement
    }
    s = static_cast<T>(1) / kVariance;
    for (std::size_t row = 0U; row < 3U; ++row) {
      k[row] = p[row][state] * s;
    }
    const std::array<T, 3> kMeasuredRow = p[state];
    for (std::size_t row = 0U; row < 3U; ++row) {
      for (std::size_t column = 0U; column < 3U; ++column) {
        p[row][column] -= k[row] * kMeasuredRow[column];
      }
    }

    omega += unit::AngularVelocity{k[0] * innovation};
    theta += unit::Angle{k[1] * innovation};
    m_l += unit::Torque{k[2] * innovation};
    ClampSpeed();

    theta = observer::WrapAngle(theta);

    sin_theta = unit::Ratio{std::sin(theta.Value())};
    cos_theta = unit::Ratio{std::cos(theta.Value())};
  }
};

}  // namespace unimoc::observer
