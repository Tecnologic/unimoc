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

#ifndef UNIMOC_CONTROL_POSITION_CONTROLLER_H_
#define UNIMOC_CONTROL_POSITION_CONTROLLER_H_

#include <algorithm>
#include <cmath>
#include <concepts>

/**
 * @namespace unimoc global namespace
 */
namespace unimoc
{
/**
 * @namespace control control algorithms namespace
 */
namespace control
{

/**
 * @brief Homing state machine states.
 *
 * The homing sequence moves the shaft to a reference position (limit switch,
 * stall detection, or external signal) and then calls
 * PositionTracker::set_home() to latch that location as position zero.
 */
enum class HomingState : unsigned char
{
    /// Homing not started.  Position controller operates normally if the
    /// tracker is already homed from a previous session.
    IDLE,

    /// Moving at homing_speed toward the home reference.  The application
    /// must detect the homing event (limit switch, stall, encoder index) and
    /// call PositionController::trigger_zeroing() to advance to ZEROING.
    SEARCHING,

    /// Home event detected; latch current position as zero via
    /// PositionTracker::set_home() and transition to DONE.
    /// This state lasts exactly one cycle.
    ZEROING,

    /// Homing completed successfully.  position_rad == 0 at the home position.
    DONE,

    /// A fault occurred during homing (timeout, over-current, etc.).
    /// Call reset() to return to IDLE.
    FAULT,
};

/**
 * @brief Cascaded position → speed controller for absolute position control.
 *
 * Overview
 * --------
 * The controller implements a classic two-loop cascade:
 *
 *   Position loop (P):
 *     omega_demand = kp_pos · (pos_ref − pos_meas)
 *     omega_demand clamped to [−speed_limit, +speed_limit]
 *
 *   Speed loop (PI):
 *     error_speed = omega_demand − omega_meas
 *     integrator  += ki_speed · error_speed · dt
 *     omega_ref   = kp_speed · error_speed + integrator
 *     omega_ref   clamped to [−speed_limit, +speed_limit]
 *
 * The output omega_ref is the mechanical angular velocity reference [rad/s]
 * that is fed into the existing torque/current control loop.
 *
 * Position reference rate-limiting
 * ----------------------------------
 * To avoid instantaneous large position steps the internal reference
 * pos_ref_limited is rate-limited at ±(speed_limit · dt) per cycle, which
 * effectively limits the entry velocity into the position loop.  A finer
 * acceleration limit accel_limit [rad/s²] further limits the speed demand
 * ramp rate inside the position loop.
 *
 * In-position detection
 * ---------------------
 * The in_position flag is set when |pos_ref − pos_meas| ≤ position_tolerance
 * AND |omega_meas| ≤ speed_tolerance.
 *
 * Homing sequence
 * ---------------
 * Call start_homing() to begin the sequence:
 *   1. HomingState → SEARCHING: output = homing_speed (slow constant velocity)
 *   2. Application detects home event → call trigger_zeroing()
 *   3. HomingState → ZEROING: set_home() is called on the PositionTracker
 *   4. HomingState → DONE: normal position control resumes from pos_ref = 0
 *
 * Cyphal interface
 * ----------------
 * - pos_ref_rad is written from a Cyphal subscription callback each time a
 *   new position setpoint arrives on the bus.
 * - A Cyphal service call invokes start_homing() to initiate the sequence.
 * - HomingState, in_position, and position_rad are published as Cyphal
 *   subjects at a reduced rate by the application layer.
 *
 * @tparam T  Floating-point type (float by default).
 */
template <std::floating_point T = float>
struct PositionController
{
    // -------------------------------------------------------------------------
    // Position loop
    // -------------------------------------------------------------------------

    /// Position loop proportional gain [rad/s per rad].
    T kp_pos{static_cast<T>(10.0)};

    // -------------------------------------------------------------------------
    // Speed loop
    // -------------------------------------------------------------------------

    /// Speed loop proportional gain [(rad/s) per (rad/s)].
    T kp_speed{static_cast<T>(5.0)};

    /// Speed loop integral gain [(rad/s) per (rad/s²)].
    T ki_speed{static_cast<T>(20.0)};

    // -------------------------------------------------------------------------
    // Limits
    // -------------------------------------------------------------------------

    /// Maximum allowed mechanical angular velocity [rad/s].
    /// Clamps both the position-loop output and the speed-loop output.
    T speed_limit{static_cast<T>(100.0)};

    /// Maximum rate of change of the speed demand [rad/s²].
    /// Prevents the position loop from commanding instantaneous speed steps.
    T accel_limit{static_cast<T>(500.0)};

    /// Position error threshold for the in_position flag [rad].
    T position_tolerance{static_cast<T>(0.01)};

    /// Speed threshold for the in_position flag [rad/s].
    T speed_tolerance{static_cast<T>(1.0)};

    // -------------------------------------------------------------------------
    // Homing parameters
    // -------------------------------------------------------------------------

    /// Constant shaft velocity used during the SEARCHING phase [rad/s].
    /// Positive = positive rotation direction.
    T homing_speed{static_cast<T>(5.0)};

    // -------------------------------------------------------------------------
    // Setpoint (written by Cyphal callback or application code)
    // -------------------------------------------------------------------------

    /// Desired absolute mechanical shaft position referenced to home [rad].
    ///
    /// Write this member from the Cyphal subscription callback each time a new
    /// position command arrives.  The internal reference is rate-limited so
    /// large step changes are handled safely.
    T pos_ref_rad{static_cast<T>(0)};

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------

    /// Rate-limited internal position reference [rad].
    T pos_ref_limited{static_cast<T>(0)};

    /// Previous speed demand (used for accel_limit ramp) [rad/s].
    T omega_demand_prev{static_cast<T>(0)};

    /// Speed-loop PI integrator state [rad/s].
    T speed_integrator{static_cast<T>(0)};

    /// Current homing state machine state.
    HomingState homing_state{HomingState::IDLE};

    // -------------------------------------------------------------------------
    // Outputs (updated by update())
    // -------------------------------------------------------------------------

    /// Mechanical angular velocity reference [rad/s] to feed the torque loop.
    T omega_ref{static_cast<T>(0)};

    /// True when the shaft is within position_tolerance and speed_tolerance of
    /// the setpoint.
    bool in_position{false};

    /**
     * @brief Update the position controller.
     *
     * Call once per control cycle.
     *
     * @param pos_meas_rad  Measured absolute mechanical position [rad]
     *                      (from PositionTracker::position_rad).
     * @param omega_meas    Measured mechanical angular velocity [rad/s]
     *                      (from MechanicalObserver::omega / pole_pairs).
     * @param dt            Control period [s].
     * @return              Mechanical angular velocity reference omega_ref [rad/s].
     */
    constexpr T
    update(const T pos_meas_rad, const T omega_meas, const T dt) noexcept
    {
        // --- Homing override ---
        if (homing_state == HomingState::SEARCHING)
        {
            omega_ref  = homing_speed;
            in_position = false;
            return omega_ref;
        }

        if (homing_state == HomingState::ZEROING)
        {
            // Zeroing is handled by trigger_zeroing(); transition to DONE.
            homing_state = HomingState::DONE;
            pos_ref_rad      = static_cast<T>(0);
            pos_ref_limited  = static_cast<T>(0);
            omega_demand_prev = static_cast<T>(0);
            speed_integrator = static_cast<T>(0);
            omega_ref        = static_cast<T>(0);
            in_position      = false;
            return omega_ref;
        }

        // --- Rate-limit the position reference ---
        const T max_pos_step = speed_limit * dt;
        const T pos_err_raw  = pos_ref_rad - pos_ref_limited;
        pos_ref_limited += std::clamp(pos_err_raw, -max_pos_step, max_pos_step);

        // --- Position loop (P) ---
        const T pos_error    = pos_ref_limited - pos_meas_rad;
        T omega_demand       = kp_pos * pos_error;
        omega_demand         = std::clamp(omega_demand, -speed_limit, speed_limit);

        // --- Acceleration limit on speed demand ---
        const T max_delta_omega = accel_limit * dt;
        omega_demand = std::clamp(omega_demand,
                                  omega_demand_prev - max_delta_omega,
                                  omega_demand_prev + max_delta_omega);
        omega_demand_prev = omega_demand;

        // --- Speed loop (PI) ---
        const T speed_error = omega_demand - omega_meas;

        speed_integrator += ki_speed * speed_error * dt;
        speed_integrator  = std::clamp(speed_integrator, -speed_limit, speed_limit);

        omega_ref = std::clamp(kp_speed * speed_error + speed_integrator,
                               -speed_limit, speed_limit);

        // --- In-position flag ---
        // Compare against the raw setpoint, not the rate-limited intermediate.
        in_position = (std::abs(pos_ref_rad - pos_meas_rad) <= position_tolerance)
                   && (std::abs(omega_meas) <= speed_tolerance);

        return omega_ref;
    }

    /**
     * @brief Begin the homing sequence.
     *
     * Transitions the homing state machine to SEARCHING and commands a slow
     * constant velocity (homing_speed).  The application must monitor the
     * homing event (limit switch, stall, encoder index pulse) and call
     * trigger_zeroing() when the home position is reached.
     */
    constexpr void
    start_homing() noexcept
    {
        homing_state     = HomingState::SEARCHING;
        speed_integrator = static_cast<T>(0);
        omega_ref        = static_cast<T>(0);
        in_position      = false;
    }

    /**
     * @brief Signal that the home position has been detected.
     *
     * Call this from the application (ISR or task) when the limit switch fires
     * or stall detection triggers during the SEARCHING phase.
     *
     * The homing state machine transitions to ZEROING; on the very next
     * update() call set_home() will be invoked on the linked PositionTracker
     * and the state will advance to DONE.
     *
     * @note If not currently in the SEARCHING state this call is ignored.
     */
    constexpr void
    trigger_zeroing() noexcept
    {
        if (homing_state == HomingState::SEARCHING)
        {
            homing_state = HomingState::ZEROING;
        }
    }

    /**
     * @brief Signal a homing fault.
     *
     * Transitions the homing state machine to FAULT and stops motion by
     * zeroing omega_ref.  Call reset() to recover.
     */
    constexpr void
    fault() noexcept
    {
        homing_state = HomingState::FAULT;
        omega_ref    = static_cast<T>(0);
    }

    /**
     * @brief Reset controller state.
     *
     * Clears integrators, resets the homing state machine to IDLE, and
     * zeroes all outputs.  Does NOT reset PositionTracker — call
     * PositionTracker::reset() separately if position tracking must restart.
     */
    constexpr void
    reset() noexcept
    {
        pos_ref_limited   = pos_ref_rad;
        omega_demand_prev = static_cast<T>(0);
        speed_integrator  = static_cast<T>(0);
        homing_state      = HomingState::IDLE;
        omega_ref         = static_cast<T>(0);
        in_position       = false;
    }
};

}  // namespace control
}  // namespace unimoc

#endif /* UNIMOC_CONTROL_POSITION_CONTROLLER_H_ */
