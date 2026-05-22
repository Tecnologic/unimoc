/*
       __  ___   ________  _______  ______
      / / / / | / /  _/  |/  / __ \/ ____/
     / / / /  |/ // // /|_/ / / / / /
    / /_/ / /|  // // /  / / /_/ / /___
    \____/_/ |_/___/_/  /_/\____/\____/

    Universal Motor Control  2025 Alexander <tecnologic86@gmail.com> Evers

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
};

}  // namespace system
}  // namespace unimoc

#endif /* UNIMOC_SYSTEM_MOTOR_TYPE_H_ */
