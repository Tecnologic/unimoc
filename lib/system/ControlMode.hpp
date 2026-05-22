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

#ifndef UNIMOC_SYSTEM_CONTROL_MODE_H_
#define UNIMOC_SYSTEM_CONTROL_MODE_H_

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
 * @brief Active outer control loop selection.
 *
 * The top-level control loop selects one of these modes at runtime.  The
 * inner current/torque loops are always active; only the outer setpoint
 * source changes.
 *
 * TORQUE
 *   The torque (q-axis current) setpoint is commanded directly.
 *   No speed or position outer loop is active.
 *   Cyphal: receives uavcan.si.unit.torque or a normalised torque request.
 *
 * SPEED
 *   A speed PI loop drives the torque setpoint.
 *   The speed setpoint is received via Cyphal.
 *
 * POSITION
 *   A position P loop feeds a speed PI loop which drives the torque setpoint.
 *   Requires PositionTracker to be homed (is_homed == true) before commands
 *   are tracked; the application should refuse position commands until homed.
 *   The position setpoint (PositionController::pos_ref_rad) is received via
 *   Cyphal; homing is triggered via a Cyphal service call.
 */
enum class ControlMode : unsigned char
{
    /// Direct torque (q-axis current) command.
    TORQUE,

    /// Speed PI loop — output is torque setpoint.
    SPEED,

    /// Cascaded position P + speed PI loops — output is torque setpoint.
    /// PositionController and PositionTracker must be active.
    POSITION,
};

}  // namespace system
}  // namespace unimoc

#endif /* UNIMOC_SYSTEM_CONTROL_MODE_H_ */
