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

#ifndef UNIMOC_SYSTEM_MOTOR_TYPE_H_
#define UNIMOC_SYSTEM_MOTOR_TYPE_H_

/**
 * @namespace unimoc global namespace
 */
namespace unimoc
{
/**
 * @namespace system coordinate and motor type definitions
 */
namespace system
{

/**
 * @brief Supported motor types.
 *
 * Used as a compile-time or run-time tag to select the appropriate control
 * path (observer, flux controller, MTPA, HFI, …).
 */
enum class MotorType : unsigned char
{
    /// Permanent-Magnet Synchronous Motor (surface or interior magnets).
    /// Requires: back-EMF observer (MechanicalObserver), MTPA, HFI (optional),
    ///           field-weakening, dead-time compensation, SVPWM.
    PMSM,

    /// Asynchronous (Induction) Motor.
    /// Requires: rotor-flux observer (AsmFluxObserver), flux controller
    ///           (AsmFluxController), mechanical observer fed via
    ///           inject_angle_error(), SVPWM, dead-time compensation.
    ASM,

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
    EESM,
};

}  // namespace system
}  // namespace unimoc

#endif /* UNIMOC_SYSTEM_MOTOR_TYPE_H_ */
