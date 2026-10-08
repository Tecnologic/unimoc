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

#ifndef UNIMOC_OBSERVER_ASM_FLUX_OBSERVER_H_
#define UNIMOC_OBSERVER_ASM_FLUX_OBSERVER_H_

#include <algorithm>
#include <cmath>
#include <concepts>
#include <numbers>
#include "mechanical_observer.hpp"
#include "stator_system.hpp"
#include "units.hpp"

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
 * @brief Full-order Luenberger rotor-flux observer for induction motors (ASM).
 *
 * Overview
 * --------
 * The observer estimates the stator current vector (î_sα, î_sβ) and the rotor
 * flux linkage vector (ψ̂_rα, ψ̂_rβ) in the stationary α/β frame.  It uses the
 * measured stator voltage and current together with the estimated electrical
 * rotor speed (from the MechanicalObserver PLL) as inputs.
 *
 * Induction motor model (stationary α/β frame)
 * ---------------------------------------------
 * Let σ = 1 − L_m² / (L_s · L_r) be the total leakage factor.
 *
 * Stator current dynamics:
 *   dî_sα/dt = −(R_s/(σ·L_s) + (1−σ)/(σ·T_r)) · î_sα
 *              + (L_m/(σ·L_s·L_r)) · (ψ̂_rα/T_r + ω̂_r · ψ̂_rβ)
 *              + v_sα/(σ·L_s)
 *              + g_i · (i_sα − î_sα)
 *
 * Rotor flux dynamics:
 *   dψ̂_rα/dt = (L_m/T_r) · î_sα − (1/T_r) · ψ̂_rα − ω̂_r · ψ̂_rβ
 *              + g_flux · (i_sα − î_sα)
 *
 * (β-axis equations are identical with α↔β and appropriate sign changes.)
 *
 * Correction gain strategy
 * ------------------------
 * Both the current and flux equations are corrected by the current prediction
 * error (i_s − î_s), similar to a reduced-order Luenberger observer.  This
 * eliminates the need to measure rotor quantities.
 *
 * Feeding the MechanicalObserver
 * -------------------------------
 * The rotor flux angle θ_flux = atan2(ψ̂_rβ, ψ̂_rα) is the field reference
 * frame angle for FOC.  It is injected into the MechanicalObserver's shared
 * PLL via inject_angle_error() so that both observers converge on the same
 * angle estimate, enabling a smooth PMSM↔ASM transition if needed.
 *
 * @tparam T  Floating-point type (float by default).
 */
template <std::floating_point T = float>
struct AsmFluxObserver {
  // -------------------------------------------------------------------------
  // Motor parameters
  // -------------------------------------------------------------------------

  /// Stator resistance R_s [Ω].
  unit::Resistance R_s{0.5_Ohm};

  /// Rotor resistance R_r [Ω].
  unit::Resistance R_r{0.3_Ohm};

  /// Stator self-inductance L_s [H].
  unit::Inductance L_s{50.0_mH};

  /// Rotor self-inductance L_r [H] (≈ L_s for squirrel-cage motors).
  unit::Inductance L_r{50.0_mH};

  /// Mutual (magnetising) inductance L_m [H].
  unit::Inductance L_m{47.0_mH};

  // -------------------------------------------------------------------------
  // Observer gains
  // -------------------------------------------------------------------------

  /// Stator-current correction gain g_i [1/s].
  ///
  /// Governs how aggressively the current prediction error corrects the
  /// estimated stator current.  A value in the range [2·R_s/σL_s … 10·R_s/σL_s]
  /// is a reasonable starting point.
  unit::InverseTime g_i{500.0_per_s};

  /// Rotor-flux correction gain g_flux [Wb/(A·s)].
  ///
  /// Determines how fast the flux estimate responds to the current error.
  /// Set larger than g_i to prevent flux lag during transients.
  unit::MagneticFluxPerCurrentTime g_flux{5000.0_Wb_per_A_s};

  // -------------------------------------------------------------------------
  // Observer state
  // -------------------------------------------------------------------------

  /// Estimated α-axis stator current [A].
  unit::Current i_alpha_hat{};
  /// Estimated β-axis stator current [A].
  unit::Current i_beta_hat{};

  /// Estimated α-axis rotor flux linkage [Wb].
  unit::MagneticFlux psi_r_alpha{};
  /// Estimated β-axis rotor flux linkage [Wb].
  unit::MagneticFlux psi_r_beta{};

  // -------------------------------------------------------------------------
  // Outputs (updated by update())
  // -------------------------------------------------------------------------

  /// Estimated rotor flux magnitude |ψ̂_r| [Wb].
  unit::MagneticFlux flux_magnitude{};

  /// Estimated rotor flux angle θ_flux = atan2(ψ̂_rβ, ψ̂_rα) [rad].
  unit::Angle flux_angle{};

  /// sin(θ_flux) — ready for field-oriented transforms.
  unit::Ratio sin_flux{};
  /// cos(θ_flux) — ready for field-oriented transforms.
  unit::Ratio cos_flux{1.0_ratio};

  /**
   * @brief Run one observer step and update the MechanicalObserver PLL.
   *
   * Call this once per PWM/control interrupt.
   *
   * @param v_ab     Applied stator voltage vector α/β [V].
   * @param i_ab     Measured stator current vector α/β [A].
   * @param dt       Control period [s].
   * @param mech_obs MechanicalObserver whose PLL receives the flux-angle correction.
   */
  constexpr void update(const system::Stator<unit::Voltage>& v_ab,
                        const system::Stator<unit::Current>& i_ab,
                        const unit::Time dt,
                        MechanicalObserver<T>& mech_obs) noexcept {
    // --- Derived parameters (computed every cycle for generality; in
    //     practice the compiler will CSE these if parameters are const.) ---
    const T sigma = calc_sigma();
    const T inv_T_r = R_r.Value() / L_r.Value();  // 1/T_r
    const T inv_sLs = static_cast<T>(1) / (sigma * L_s.Value());

    // Stator-current time-constant coefficient
    const T alpha_i = R_s.Value() * inv_sLs + (static_cast<T>(1) - sigma) * inv_T_r / sigma;

    // Flux-to-current coupling coefficient L_m/(σ·L_s·L_r)
    const T k_flux = L_m.Value() / (sigma * L_s.Value() * L_r.Value());

    // Electrical rotor speed from MechanicalObserver PLL
    const T omega_r = mech_obs.omega.Value();

    // --- Current prediction error ---
    const T err_alpha = i_ab.alpha.Value() - i_alpha_hat.Value();
    const T err_beta = i_ab.beta.Value() - i_beta_hat.Value();

    // --- Stator-current observer ---
    const T di_alpha = -alpha_i * i_alpha_hat.Value() + k_flux * (inv_T_r * psi_r_alpha.Value() + omega_r * psi_r_beta.Value()) +
                       inv_sLs * v_ab.alpha.Value() + g_i.Value() * err_alpha;

    const T di_beta = -alpha_i * i_beta_hat.Value() + k_flux * (inv_T_r * psi_r_beta.Value() - omega_r * psi_r_alpha.Value()) +
                      inv_sLs * v_ab.beta.Value() + g_i.Value() * err_beta;

    // --- Rotor-flux observer ---
    const T dpsi_r_alpha =
        (L_m.Value() * inv_T_r) * i_alpha_hat.Value() - inv_T_r * psi_r_alpha.Value() - omega_r * psi_r_beta.Value() + g_flux.Value() * err_alpha;

    const T dpsi_r_beta =
        (L_m.Value() * inv_T_r) * i_beta_hat.Value() - inv_T_r * psi_r_beta.Value() + omega_r * psi_r_alpha.Value() + g_flux.Value() * err_beta;

    // --- Euler integration ---
    i_alpha_hat = unit::Current{i_alpha_hat.Value() + dt.Value() * di_alpha};
    i_beta_hat = unit::Current{i_beta_hat.Value() + dt.Value() * di_beta};
    psi_r_alpha = unit::MagneticFlux{psi_r_alpha.Value() + dt.Value() * dpsi_r_alpha};
    psi_r_beta = unit::MagneticFlux{psi_r_beta.Value() + dt.Value() * dpsi_r_beta};

    // --- Compute flux magnitude and angle ---
    flux_magnitude = unit::MagneticFlux{std::sqrt(psi_r_alpha.Value() * psi_r_alpha.Value() + psi_r_beta.Value() * psi_r_beta.Value())};
    flux_angle = unit::Angle{std::atan2(psi_r_beta.Value(), psi_r_alpha.Value())};
    sin_flux = unit::Ratio{std::sin(flux_angle.Value())};
    cos_flux = unit::Ratio{std::cos(flux_angle.Value())};

    // --- Feed flux angle into MechanicalObserver PLL ---
    //
    // Compute the angle error between the flux angle and the PLL estimate.
    // Using the cross-product sine approximation (no atan2 needed):
    //   sin(θ_flux − θ̂) ≈ sin(θ_flux)·cos(θ̂) − cos(θ_flux)·sin(θ̂)
    //
    // This is the same technique used in the PMSM back-EMF PLL.
    const T angle_error = sin_flux.Value() * mech_obs.cos_theta.Value() - cos_flux.Value() * mech_obs.sin_theta.Value();

    mech_obs.inject_angle_error(unit::Angle{angle_error}, dt);
  }

  /**
   * @brief Reset the observer state.
   *
   * Call on mode transitions or fault recovery.
   */
  constexpr void reset() noexcept {
    i_alpha_hat = unit::Current{};
    i_beta_hat = unit::Current{};
    psi_r_alpha = unit::MagneticFlux{};
    psi_r_beta = unit::MagneticFlux{};
    flux_magnitude = unit::MagneticFlux{};
    flux_angle = unit::Angle{};
    sin_flux = unit::Ratio{};
    cos_flux = 1.0_ratio;
  }

 private:
  /// Compute total leakage factor σ = 1 − L_m² / (L_s · L_r).
  [[nodiscard]] constexpr T calc_sigma() const noexcept { return static_cast<T>(1) - (L_m.Value() * L_m.Value()) / (L_s.Value() * L_r.Value()); }
};

}  // namespace observer
}  // namespace unimoc

#endif /* UNIMOC_OBSERVER_ASM_FLUX_OBSERVER_H_ */
