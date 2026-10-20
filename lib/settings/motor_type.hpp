/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file motor_type.hpp
 *    @brief Supported motor types that select the control path.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

/**
 * @namespace system coordinate and motor type definitions
 */
namespace unimoc::system {

/**
 * @brief Supported motor types.
 *
 * Used as a compile-time or run-time tag to select the appropriate control
 * path (observer, flux controller, MTPA, HFI, …).
 */
enum class MotorType : unsigned char {
  /// Permanent-Magnet Synchronous Motor (surface or interior magnets).
  /// Requires: back-EMF observer (MechanicalObserver), MTPA, HFI (optional),
  ///           field-weakening, dead-time compensation, SVPWM.
  kPmsm,

  /// Asynchronous (Induction) Motor.
  /// Requires: rotor-flux observer (AsmFluxObserver), flux controller
  ///           (AsmFluxController), mechanical observer fed via
  ///           InjectAngleError(), SVPWM, dead-time compensation.
  kAsm,

  /// Electrically Excited Synchronous Machine (wound-rotor synchronous motor).
  ///
  /// The rotor excitation winding is fed by an external DC current source
  /// (H-bridge, chopper, or brushless exciter).  The stator-side control path
  /// is identical to PMSM (back-EMF observer, MTPA, field-weakening, SVPWM),
  /// but the effective PM flux linkage ψ_PM is no longer constant — it is
  /// determined by the rotor excitation current I_f via:
  ///
  ///   ψ_f = L_m · I_f
  ///
  /// Requires: ExcitationController (regulates I_f or ψ_f via PI loop),
  ///           ExcitationObserver (filters measured I_f and estimates ψ_f),
  ///           MechanicalObserver, MTPA (using ψ_f_hat instead of fixed ψ_PM),
  ///           field-weakening, dead-time compensation, SVPWM.
  ///
  /// Cyphal interface: setpoint (current or flux) is written by the Cyphal
  /// subscription callback into ExcitationController::setpoint; the hardware
  /// driver (PWM/DAC) reads ExcitationController::i_f_ref each cycle.
  kEesm,
};

}  // namespace unimoc::system
