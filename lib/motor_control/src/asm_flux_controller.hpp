/*
       __  ___   ________  _______  ______
      / / / / | / /  _/  |/  / __ \/ ____/
     / / / /  |/ // // /|_/ / / / / /
    / /_/ / /|  // // /  / / /_/ / /___
    \____/_/ [_/___/_/  /_/\____/\____/

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

#ifndef UNIMOC_CONTROL_ASM_FLUX_CONTROLLER_H_
#define UNIMOC_CONTROL_ASM_FLUX_CONTROLLER_H_

#include <cmath>
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
 * @brief Rotor-flux magnitude controller for induction motors (ASM).
 *
 * Overview
 * --------
 * In rotor-flux-oriented (FOC) control of an induction motor the d-axis current
 * (aligned with the rotor flux vector) determines the rotor flux magnitude,
 * while the q-axis current determines the torque.
 *
 * This controller runs a PI loop on the flux error:
 *
 *   e_flux = ψ_r* − |ψ̂_r|
 *   i_d*   = Kp · e_flux + integrator
 *
 * The integrator is clamped to [i_d_min, i_d_max] to prevent windup.
 *
 * Slip frequency feedforward
 * --------------------------
 * In steady state the slip angular frequency is:
 *
 *   ω_slip = (R_r / L_r) · (i_q / (ψ_r / L_m))
 *           = (1 / T_r)  · (L_m · i_q / ψ_r)
 *
 * where T_r = L_r / R_r is the rotor time constant.  Adding this to the
 * estimated rotor speed ω̂_r gives the stator excitation frequency:
 *
 *   ω_s = ω̂_r + ω_slip
 *
 * The caller should use ω_s to advance the field angle if an open-loop
 * feedforward scheme is preferred alongside the closed-loop flux observer.
 *
 */
struct AsmFluxController {
  // -------------------------------------------------------------------------
  // Motor parameters
  // -------------------------------------------------------------------------

  /// Rotor resistance R_r [Ω].
  unit::Resistance R_r{0.3_Ohm};

  /// Rotor self-inductance L_r [H].
  unit::Inductance L_r{50.0_mH};

  /// Mutual (magnetising) inductance L_m [H].
  unit::Inductance L_m{47.0_mH};

  // -------------------------------------------------------------------------
  // Controller gains
  // -------------------------------------------------------------------------

  /// Proportional gain K_p [A/Wb].
  unit::CurrentPerMagneticFlux kp{10.0_A_per_Wb};

  /// Integral gain K_i [A/(Wb·s)].
  unit::CurrentPerMagneticFluxTime ki{50.0_A_per_Wb_s};

  // -------------------------------------------------------------------------
  // Output limits
  // -------------------------------------------------------------------------

  /// Minimum d-axis current [A] (must be ≥ 0; negative magnetising current
  /// is not useful in a squirrel-cage motor).
  unit::Current i_d_min{};

  /// Maximum d-axis current [A].
  unit::Current i_d_max{10.0_A};

  // -------------------------------------------------------------------------
  // State
  // -------------------------------------------------------------------------

  /// PI integrator state [A].
  unit::Current integrator{};

  // -------------------------------------------------------------------------
  // Output
  // -------------------------------------------------------------------------

  /// d-axis current reference i_d* [A] (updated by update()).
  unit::Current i_d_ref{};

  /// Computed slip angular frequency ω_slip [rad/s] (updated by update()).
  unit::AngularVelocity omega_slip{};

  /**
   * @brief Update the flux controller.
   *
   * Call once per control cycle.
   *
   * @param psi_r_ref     Rotor flux magnitude reference |ψ_r*| [Wb].
   * @param psi_r_meas    Estimated rotor flux magnitude |ψ̂_r| [Wb]
   *                      (from AsmFluxObserver::flux_magnitude).
   * @param i_q           Current q-axis current component [A] (for slip
   *                      frequency calculation).
   * @param dt            Control period [s].
   * @return              d-axis current reference i_d* [A].
   */
  constexpr unit::Current update(const unit::MagneticFlux psi_r_ref,
                                 const unit::MagneticFlux psi_r_meas,
                                 const unit::Current i_q,
                                 const unit::Time dt) noexcept {
    // --- PI flux controller ---
    const unit::MagneticFlux error = psi_r_ref - psi_r_meas;

    integrator = (integrator + ki * error * dt).Clamp(i_d_min, i_d_max);

    i_d_ref = (kp * error + integrator).Clamp(i_d_min, i_d_max);

    // --- Slip frequency feedforward ---
    // ω_slip = (1/T_r) · (L_m · i_q / ψ_r*)
    // Use the flux reference in the denominator to avoid division by
    // a near-zero measured flux during start-up.
    if (psi_r_ref > unit::MagneticFlux{1.0_uWb}) {
      const unit::Time rotor_time_constant = L_r / R_r;
      const unit::Ratio slip_ratio = L_m * i_q / psi_r_ref;
      omega_slip = (slip_ratio * unit::Angle{1.0F}) / rotor_time_constant;
    } else {
      omega_slip = unit::AngularVelocity{};
    }

    return i_d_ref;
  }

  /// Reset controller state (call on enable or fault recovery).
  constexpr void reset() noexcept {
    integrator = unit::Current{};
    i_d_ref = unit::Current{};
    omega_slip = unit::AngularVelocity{};
  }
};

}  // namespace control
}  // namespace unimoc

#endif /* UNIMOC_CONTROL_ASM_FLUX_CONTROLLER_H_ */
