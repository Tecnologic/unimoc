/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file excitation_controller.hpp
 *    @brief Excitation current and flux controller for EESM.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <concepts>
#include "units.hpp"

/**
 * @namespace unimoc::control control algorithms namespace
 */
namespace unimoc::control {

using namespace unit;

/**
 * @brief Control mode for the ExcitationController.
 *
 * Selects whether the PI loop regulates the rotor excitation current directly
 * (CurrentMode) or the effective rotor flux linkage (FluxMode).
 */
enum class ExcitationMode : unsigned char {
  /// Regulate rotor excitation current I_f [A] directly.
  /// The setpoint is interpreted as a current reference in amperes.
  kCurrentMode,

  /// Regulate effective rotor flux linkage ψ_f = L_m · I_f [Wb].
  /// The setpoint is interpreted as a flux reference in weber.
  /// The controller internally converts the flux setpoint to a current
  /// setpoint via  I_f* = ψ_f* / L_m  before running the PI loop.
  kFluxMode,
};

/**
 * @brief PI controller for the rotor excitation winding of an EESM.
 *
 * Overview
 * --------
 * An Electrically Excited Synchronous Machine (EESM) has a wound rotor fed
 * by a separate DC current source (H-bridge, chopper, or brushless exciter).
 * The effective PM-equivalent rotor flux is:
 *
 *   ψ_f = L_m · I_f
 *
 * This controller regulates either I_f (CurrentMode) or ψ_f (FluxMode) via a
 * standard PI loop, producing a current reference i_f_ref for the hardware
 * excitation driver.
 *
 * Cyphal interface
 * ----------------
 * The setpoint member is written by the Cyphal subscription callback each
 * time a new rotor-field setpoint message arrives on the network.  The
 * hardware excitation driver (PWM output, DAC, or on-board H-bridge) reads
 * i_f_ref once per control cycle.  For future on-board hardware an on-chip
 * PWM/DAC output pin can be directly driven from i_f_ref without any
 * additional software layer.
 *
 * @tparam T  Floating-point type (float by default).
 */
template <std::floating_point T = float>
struct ExcitationController {
  // -------------------------------------------------------------------------
  // Configuration
  // -------------------------------------------------------------------------

  /// Active control mode (CurrentMode or FluxMode).
  ExcitationMode mode{ExcitationMode::kCurrentMode};

  /// Mutual (magnetising) inductance L_m [H].
  /// Used only in FluxMode to convert a flux setpoint to a current setpoint.
  unit::Inductance L_m{47.0_mH};

  // -------------------------------------------------------------------------
  // PI gains
  // -------------------------------------------------------------------------

  /// Proportional gain K_p [1] for the current-reference loop.
  unit::Ratio kp{5.0_ratio};

  /// Integral gain K_i [1/s].
  unit::InverseTime ki{50.0_per_s};

  // -------------------------------------------------------------------------
  // Output limits
  // -------------------------------------------------------------------------

  /// Minimum rotor excitation current i_f_ref [A] (≥ 0 for unidirectional
  /// exciters; may be negative for reversible H-bridge drives).
  unit::Current i_f_min;

  /// Maximum rotor excitation current i_f_ref [A].
  unit::Current i_f_max{10.0_A};

  // -------------------------------------------------------------------------
  // Setpoint (written by Cyphal callback or application code)
  // -------------------------------------------------------------------------

  /// Current-mode rotor excitation current setpoint [A].
  ///
  /// Interpretation depends on @p mode:
  ///  - CurrentMode: desired rotor excitation current I_f* [A].
  ///  - FluxMode:    desired effective flux linkage ψ_f* [Wb].
  ///
  /// Write this member from the Cyphal subscription callback in CurrentMode.
  unit::Current current_setpoint;

  /// Flux-mode rotor field setpoint [Wb].
  /// Write this member from the Cyphal subscription callback in FluxMode.
  unit::MagneticFlux flux_setpoint;

  // -------------------------------------------------------------------------
  // State
  // -------------------------------------------------------------------------

  /// PI integrator state [A].
  unit::Current integrator;

  // -------------------------------------------------------------------------
  // Output (read by hardware excitation driver each cycle)
  // -------------------------------------------------------------------------

  /// Rotor excitation current reference i_f* [A].
  ///
  /// Feed this value into the hardware excitation driver (H-bridge duty
  /// cycle, DAC voltage, or brushless exciter current command) every cycle.
  unit::Current i_f_ref;

  /**
   * @brief Update the excitation controller.
   *
   * Call once per control cycle.
   *
   * @param i_f_meas  Measured (or estimated) rotor excitation current [A].
   *                  Use ExcitationObserver::i_f_hat if a direct measurement
   *                  is unavailable.
   * @param dt        Control period [s].
   * @return          Updated rotor excitation current reference i_f* [A].
   */
  constexpr unit::Current Update(const unit::Current i_f_meas, const unit::Time dt) noexcept {
    // Convert setpoint to a current reference depending on the active mode.
    unit::Current i_f_setpoint;
    if (mode == ExcitationMode::kFluxMode && L_m > 1.0e-9_H) {
      i_f_setpoint = flux_setpoint / L_m;
    } else {
      i_f_setpoint = current_setpoint;
    }

    // PI loop
    const unit::Current kError = i_f_setpoint - i_f_meas;

    integrator = (integrator + (ki * kError * dt)).Clamp(i_f_min, i_f_max);

    i_f_ref = ((kp * kError) + integrator).Clamp(i_f_min, i_f_max);

    return i_f_ref;
  }

  /// Reset controller state (call on enable or fault recovery).
  constexpr void Reset() noexcept {
    integrator = unit::Current{};
    i_f_ref = unit::Current{};
  }
};

}  // namespace unimoc::control
