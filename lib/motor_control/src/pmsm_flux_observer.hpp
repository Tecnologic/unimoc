/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file pmsm_flux_observer.hpp
 *    @brief Voltage-model PMSM flux observer with closed-loop angle feedback.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cmath>
#include <concepts>
#include <optional>
#include "mechanical_observer.hpp"
#include "nvm_settings.hpp"
#include "rotor_system.hpp"
#include "stator_system.hpp"

/**
 * @namespace unimoc::observer observer algorithms namespace
 */
namespace unimoc::observer {

/**
 * @brief Voltage-model PMSM flux observer with closed-loop feedback correction.
 *
 * Overview
 * --------
 * The observer estimates the permanent-magnet flux linkage vector by
 * integrating the back-EMF in the stationary (α/β) frame and then
 * transforming the result to the estimated rotor (d/q) frame.
 *
 * Rotor-frame back-EMF
 * --------------------
 *   e_d = V_d − R_s · i_d + fb_d
 *   e_q = V_q − R_s · i_q + fb_q
 *
 * where fb_d, fb_q are closed-loop feedback terms that prevent integrator
 * drift (see below).
 *
 * Stator-frame flux integration
 * -----------------------------
 * The back-EMF is transformed to the stator frame by an inverse Park
 * transform using the current angle estimate from the MechanicalObserver:
 *
 *   e_α = e_d · cos θ̂ − e_q · sin θ̂
 *   e_β = e_d · sin θ̂ + e_q · cos θ̂
 *
 * The stator-frame flux is then obtained by integration:
 *   ψ_α += e_α · dt
 *   ψ_β += e_β · dt
 *
 * Estimated PM flux in rotor frame
 * ---------------------------------
 * The integrated flux is transformed back to the rotor frame (Park transform):
 *   ψ_d =  ψ_α · cos θ̂ + ψ_β · sin θ̂
 *   ψ_q = −ψ_α · sin θ̂ + ψ_β · cos θ̂
 *
 * The inductance-induced flux is subtracted to isolate the PM flux:
 *   ψ_d_PM = ψ_d − L_d · i_d
 *   ψ_q_PM = ψ_q − L_q · i_q
 *
 * Closed-loop feedback (anti-drift)
 * ----------------------------------
 * The difference between the flux set-point (desired PM flux) and the
 * estimated PM flux drives a proportional feedback into the back-EMF:
 *
 *   fb_d = C_d · (ψ_set_d − ψ_d_PM)
 *   fb_q = C_q · (ψ_set_q − ψ_q_PM)
 *
 * This prevents integrator wind-up in steady state and pulls the estimate
 * towards the known PM flux magnitude.
 *
 * Angle measurement for the MechanicalObserver Kalman filter
 * ----------------------------------------------------------
 * The PM flux lies on the d axis when the angle estimate is right. Its q
 * component in the estimated frame is ψ_q_PM ≈ |ψ_set| · sin(θ − θ̂), so
 *
 *   ε = ψ_q_PM / ψ_set_d   [rad]
 *
 * is the innovation of an angle measurement. Calculate() returns it; the
 * caller feeds it to MechanicalObserver::UpdateAngle() with a noise that
 * grows at low speed, where the back-EMF carries no angle information
 * (docs/hotpath_plan.md, sections 5 and 6, D2).
 *
 * @tparam T  Floating-point type (float by default).
 */
template <std::floating_point T = float>
struct PmsmFluxObserver {
  // =========================================================================
  // Motor parameters (loaded from NvmSettings by init())
  // =========================================================================

  /// Stator phase resistance R_s [Ω].
  unit::Resistance rs;

  /// d-axis inductance L_d [H].
  unit::Inductance L_d;

  /// q-axis inductance L_q [H].
  unit::Inductance L_q;

  // =========================================================================
  // Feedback (anti-drift) gains
  // =========================================================================

  /// d-axis feedback gain C_d [1/s].  Larger values reduce drift faster but
  /// also damp transient response.
  unit::InverseTime C_d;

  /// q-axis feedback gain C_q [1/s].
  unit::InverseTime C_q;

  // =========================================================================
  // Observer state
  // =========================================================================

  /// Stator-frame integrated flux α-component ψ_α [Wb].
  unit::MagneticFlux psi_alpha;

  /// Stator-frame integrated flux β-component ψ_β [Wb].
  unit::MagneticFlux psi_beta;

  /// Rotor-frame d-axis back-EMF feedback term fb_d [V].
  unit::Voltage fb_d;

  /// Rotor-frame q-axis back-EMF feedback term fb_q [V].
  unit::Voltage fb_q;

  // =========================================================================
  // Outputs (updated by calculate())
  // =========================================================================

  /// Estimated PM flux in the rotor d-axis [Wb] (after inductance subtraction).
  unit::MagneticFlux psi_pm_d;

  /// Estimated PM flux in the rotor q-axis [Wb] (after inductance subtraction).
  unit::MagneticFlux psi_pm_q;

  // =========================================================================
  // Public API
  // =========================================================================

  /**
   * @brief Load motor and observer parameters from non-volatile settings.
   *
   * This also clears the integrated observer state so a new settings
   * snapshot cannot be combined with state produced by older parameters.
   *
   * @param settings Validated NVM settings.
   */
  constexpr void Init(const settings::NvmSettings& settings) noexcept {
    rs = settings.stator_r;
    L_d = settings.l_d;
    L_q = settings.l_q;
    C_d = settings.pmsm_flux_obs_c_d;
    C_q = settings.pmsm_flux_obs_c_q;
    Reset();
  }

  /**
   * @brief Run one observer step and return the angle error of the estimate.
   *
   * Call this once per control cycle with the estimate the cycle used
   * (sin_theta / cos_theta of @p mech_obs).
   *
   * @param u_dq     Applied rotor-frame voltage vector [V].
   * @param i_dq     Measured rotor-frame current vector [A].
   * @param set_flux Desired PM flux in the rotor frame [Wb]
   *                 (typically { ψ_PM, 0 } for SPMSM).
   * @param dt       Control period [s].
   * @param mech_obs MechanicalObserver whose angle estimate frames the step.
   * @return The angle error ε = θ − θ̂ [rad], or nothing when the set flux
   *         is not positive (no permanent magnet configured).
   */
  [[nodiscard]] constexpr std::optional<unit::Angle> Calculate(const system::Rotor<unit::Voltage>& u_dq,
                                                               const system::Rotor<unit::Current>& i_dq,
                                                               const system::Rotor<unit::MagneticFlux>& set_flux,
                                                               const unit::Time dt,
                                                               const MechanicalObserver<T>& mech_obs) noexcept {
    // Snapshot sin/cos from the mechanical observer
    const T kSn = mech_obs.sin_theta.Value();
    const T kCs = mech_obs.cos_theta.Value();

    // -----------------------------------------------------------------
    // Rotor-frame back-EMF (voltage model + feedback)
    // -----------------------------------------------------------------
    const unit::Voltage kBemfD = u_dq.d - rs * i_dq.d + fb_d;
    const unit::Voltage kBemfQ = u_dq.q - rs * i_dq.q + fb_q;

    // -----------------------------------------------------------------
    // Inverse Park: rotor → stator frame
    //   e_α = bemf_d · cos θ̂ − bemf_q · sin θ̂
    //   e_β = bemf_d · sin θ̂ + bemf_q · cos θ̂
    // -----------------------------------------------------------------
    const unit::Voltage kBemfAlpha{kBemfD.Value() * kCs - kBemfQ.Value() * kSn};
    const unit::Voltage kBemfBeta{kBemfD.Value() * kSn + kBemfQ.Value() * kCs};

    // -----------------------------------------------------------------
    // Stator-frame flux integration
    // -----------------------------------------------------------------
    psi_alpha += kBemfAlpha * dt;
    psi_beta += kBemfBeta * dt;

    // -----------------------------------------------------------------
    // Park: stator → rotor frame
    //   ψ_d =  ψ_α · cos θ̂ + ψ_β · sin θ̂
    //   ψ_q = −ψ_α · sin θ̂ + ψ_β · cos θ̂
    // -----------------------------------------------------------------
    const unit::MagneticFlux kPsiD{psi_alpha.Value() * kCs + psi_beta.Value() * kSn};
    const unit::MagneticFlux kPsiQ{-psi_alpha.Value() * kSn + psi_beta.Value() * kCs};

    // -----------------------------------------------------------------
    // Subtract inductance-induced flux to get PM flux estimate
    // -----------------------------------------------------------------
    psi_pm_d = kPsiD - L_d * i_dq.d;
    psi_pm_q = kPsiQ - L_q * i_dq.q;

    // -----------------------------------------------------------------
    // Closed-loop feedback update
    // -----------------------------------------------------------------
    fb_d = C_d * (set_flux.d - psi_pm_d);
    fb_q = C_q * (set_flux.q - psi_pm_q);

    // -----------------------------------------------------------------
    // Angle error. The PM flux lies on the d axis when the estimate is
    // right (ψ_q_PM ≈ 0). A positive ψ_q_PM means the flux leads the
    // estimate: the estimate lags and must advance (positive ε).
    // -----------------------------------------------------------------
    if (!(set_flux.d > unit::MagneticFlux{})) {
      return std::nullopt;
    }
    return unit::Angle{(psi_pm_q / set_flux.d).Value()};
  }

  /**
   * @brief Reset the observer state.
   *
   * Call on fault recovery or control-mode transitions.
   */
  constexpr void Reset() noexcept {
    psi_alpha = unit::MagneticFlux{};
    psi_beta = unit::MagneticFlux{};
    fb_d = unit::Voltage{};
    fb_q = unit::Voltage{};
    psi_pm_d = unit::MagneticFlux{};
    psi_pm_q = unit::MagneticFlux{};
  }
};

}  // namespace unimoc::observer
