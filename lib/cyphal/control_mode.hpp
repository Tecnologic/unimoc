/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file control_mode.hpp
 *    @brief Cyphal control-mode definitions and UDRAL setpoint mapping.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cmath>
#include <concepts>
#include <optional>

/**
 * @namespace unimoc global namespace
 */
namespace unimoc {
/**
 * @namespace cyphal Cyphal protocol and application definitions
 */
namespace cyphal {

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
enum class ControlMode : unsigned char {
  /// Direct torque (q-axis current) command.
  TORQUE,

  /// Speed PI loop — output is torque setpoint.
  SPEED,

  /// Cascaded position P + speed PI loops — output is torque setpoint.
  /// PositionController and PositionTracker must be active.
  POSITION,
};

/**
 * @brief Select control mode from UDRAL servo rotational dynamics setpoint fields.
 *
 * Implements the mode selection described in:
 * `reg/udral/service/actuator/servo/_.0.1.dsdl`
 * using:
 * `reg/udral/physics/dynamics/rotation/Planar.0.1.dsdl`.
 *
 * Rule:
 * 1) First finite kinematics field selects the controlled quantity:
 *    angular_position -> POSITION, angular_velocity -> SPEED.
 * 2) angular_acceleration is not currently supported by UNIMOC and is ignored.
 * 3) If no kinematics command is provided, finite torque selects TORQUE mode.
 * 4) If nothing actionable is finite, return std::nullopt (ignore setpoint).
 *
 * @tparam T Floating-point type used for the four setpoint fields.
 * @param angular_position Angular position setpoint, or a non-finite value
 *                         when absent.
 * @param angular_velocity Angular velocity setpoint, or a non-finite value
 *                         when absent.
 * @param angular_acceleration Unsupported acceleration setpoint. A finite
 *                             value is ignored and produces no mode unless a
 *                             higher-priority field is finite.
 * @param torque Torque setpoint, or a non-finite value when absent.
 * @return The selected control mode, or `std::nullopt` for an empty or
 *         unsupported setpoint.
 */
template <std::floating_point T>
[[nodiscard]] inline std::optional<ControlMode> select_control_mode_from_udral_servo_rotation(
    const T angular_position,
    const T angular_velocity,
    const T angular_acceleration,
    const T torque) noexcept {
  if (std::isfinite(angular_position)) {
    return ControlMode::POSITION;
  }
  if (std::isfinite(angular_velocity)) {
    return ControlMode::SPEED;
  }
  if (std::isfinite(angular_acceleration)) {
    return std::nullopt;
  }
  if (std::isfinite(torque)) {
    return ControlMode::TORQUE;
  }
  return std::nullopt;
}

}  // namespace cyphal
}  // namespace unimoc
