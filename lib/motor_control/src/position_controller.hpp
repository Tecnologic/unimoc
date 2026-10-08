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

#include <cmath>
#include <concepts>
#include "units.hpp"

/**
 * @namespace unimoc global namespace
 */
namespace unimoc::control {

using namespace unit;

/**
 * @brief Homing state machine states.
 *
 * The homing sequence moves the shaft to a reference position (limit switch,
 * stall detection, or external signal) and then calls
 * PositionTracker::SetHome() to latch that location as mechanical position zero.
 */
enum class HomingState : unsigned char {
  /// Homing not started.  Position controller operates normally if the
  /// tracker is already homed from a previous session.
  kIdle,

  /// Moving at homing_speed toward the home reference.  The application
  /// must detect the homing event (limit switch, stall, encoder index) and
  /// call PositionController::TriggerZeroing() to advance to ZEROING.
  kSearching,

  /// Home event detected; latch current position as zero via
  /// PositionTracker::SetHome() and transition to DONE.
  /// This state lasts exactly one cycle.
  kZeroing,

  /// Homing completed successfully.  position_rad == 0 at the home position.
  kDone,

  /// A fault occurred during homing (timeout, over-current, etc.).
  /// Call Reset() to return to IDLE.
  kFault,
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
 * Call StartHoming() to begin the sequence:
 *   1. HomingState → SEARCHING: output = homing_speed (slow constant velocity)
 *   2. Transition to ZEROING occurs either:
 *      - explicitly via TriggerZeroing(), or
 *      - automatically when |homing_current_feedback| reaches
 *        homing_block_current_threshold during SEARCHING.
 *   3. HomingState → ZEROING: SetHome() is called through the configured
 *      home callback (typically PositionTracker::SetHome()).
 *   4. HomingState → DONE: normal position control resumes from pos_ref = 0
 *
 * Cyphal interface
 * ----------------
 * - pos_ref_rad is written from a Cyphal subscription callback each time a
 *   new position setpoint arrives on the bus.
 * - A Cyphal service call invokes StartHoming() to initiate the sequence.
 * - HomingState, in_position, and home-relative position are published as Cyphal
 *   subjects at a reduced rate by the application layer.
 *
 * @tparam T  Floating-point type (float by default).
 */
template <std::floating_point T = float>
struct PositionController {
  using HomeCallback = void (*)(void*);

  // -------------------------------------------------------------------------
  // Position loop
  // -------------------------------------------------------------------------

  /// Position loop proportional gain [rad/s per rad].
  unit::AngularVelocityPerAngle kp_pos{10.0_rad_per_s_per_rad};

  // -------------------------------------------------------------------------
  // Speed loop
  // -------------------------------------------------------------------------

  /// Speed loop proportional gain [(rad/s) per (rad/s)].
  unit::Ratio kp_speed{5.0_ratio};

  /// Speed loop integral gain [(rad/s) per (rad/s²)].
  unit::InverseTime ki_speed{20.0_per_s};

  // -------------------------------------------------------------------------
  // Limits
  // -------------------------------------------------------------------------

  /// Maximum allowed mechanical angular velocity [rad/s].
  /// Clamps both the position-loop output and the speed-loop output.
  unit::AngularVelocity speed_limit{100.0_rad_per_s};

  /// Maximum rate of change of the speed demand [rad/s²].
  /// Prevents the position loop from commanding instantaneous speed steps.
  unit::AngularAcceleration accel_limit{500.0_rad_per_s2};

  /// Position error threshold for the in_position flag [rad].
  unit::Angle position_tolerance{0.01_rad};

  /// Speed threshold for the in_position flag [rad/s].
  unit::AngularVelocity speed_tolerance{1.0_rad_per_s};

  /// Position-step threshold [rad] above which trapezoidal planning is used.
  unit::Angle trapezoid_jump_threshold{0.5_rad};

  // -------------------------------------------------------------------------
  // Homing parameters
  // -------------------------------------------------------------------------

  /// Constant shaft velocity used during the SEARCHING phase [rad/s].
  /// Positive = positive rotation direction.
  unit::AngularVelocity homing_speed{5.0_rad_per_s};

  /// Current threshold [A] used for blocked-drive homing detection.
  /// If > 0 and |homing_current_feedback| >= threshold while SEARCHING,
  /// the state transitions to ZEROING automatically.
  unit::Current homing_block_current_threshold;

  // -------------------------------------------------------------------------
  // Setpoint (written by Cyphal callback or application code)
  // -------------------------------------------------------------------------

  /// Desired absolute mechanical shaft position referenced to home [rad].
  ///
  /// Write this member from the Cyphal subscription callback each time a new
  /// position command arrives.  The internal reference is rate-limited so
  /// large step changes are handled safely.
  unit::Angle pos_ref_rad;

  // -------------------------------------------------------------------------
  // State
  // -------------------------------------------------------------------------

  /// Rate-limited internal position reference [rad].
  unit::Angle pos_ref_limited;

  /// Previous speed demand (used for accel_limit ramp) [rad/s].
  unit::AngularVelocity omega_demand_prev;

  /// Speed-loop PI integrator state [rad/s].
  unit::AngularVelocity speed_integrator;

  /// Current homing state machine state.
  HomingState homing_state{HomingState::kIdle};

  /// Optional callback used to perform the home latch in ZEROING.
  HomeCallback home_callback{nullptr};

  /// Opaque callback context (typically PositionTracker*).
  void* home_callback_context{nullptr};

  // -------------------------------------------------------------------------
  // Outputs (updated by update())
  // -------------------------------------------------------------------------

  /// Mechanical angular velocity reference [rad/s] to feed the torque loop.
  unit::AngularVelocity omega_ref;

  /// True when the shaft is within position_tolerance and speed_tolerance of
  /// the setpoint.
  bool in_position{false};

  /**
   * @brief Update the position controller.
   *
   * Call once per control cycle.
   *
   * @param pos_meas_rad  Measured home-relative mechanical position [rad]
   *                      (from PositionTracker::PositionRad()). This is not
   *                      the electrical angle used by current control.
   * @param omega_meas    Measured mechanical angular velocity [rad/s]
   *                      (from MechanicalObserver::omega / pole_pairs).
   * @param delta_time    Control period [s].
   * @param homing_current_feedback
   *                      Absolute-current feedback used for blocked-drive
   *                      homing detection (typically q-axis current) [A].
   * @return              Mechanical angular velocity reference omega_ref [rad/s].
   */
  constexpr unit::AngularVelocity Update(unit::Angle pos_meas_rad,
                                         unit::AngularVelocity omega_meas,
                                         unit::Time delta_time,
                                         unit::Current homing_current_feedback = unit::Current{}) noexcept {
    const T kPosMeas = pos_meas_rad.Value();
    const T kOmegaMeasValue = omega_meas.Value();
    // --- Homing override ---
    if (homing_state == HomingState::kSearching) {
      if (homing_block_current_threshold.Value() > 0.0F &&
          std::abs(homing_current_feedback.Value()) >= homing_block_current_threshold.Value()) {
        homing_state = HomingState::kZeroing;
      } else {
        omega_ref = homing_speed;
        in_position = false;
        return omega_ref;
      }
    }

    if (homing_state == HomingState::kZeroing) {
      if (home_callback != nullptr && home_callback_context != nullptr) {
        home_callback(home_callback_context);
      }
      homing_state = HomingState::kDone;
      pos_ref_rad = unit::Angle{};
      pos_ref_limited = unit::Angle{};
      omega_demand_prev = unit::AngularVelocity{};
      speed_integrator = unit::AngularVelocity{};
      omega_ref = unit::AngularVelocity{};
      in_position = false;
      return omega_ref;
    }

    // --- Position reference planning ---
    const T kMaxPosStep = speed_limit.Value() * delta_time.Value();
    const T kSetpointJumpMag = std::abs(pos_ref_rad.Value() - kPosMeas);
    if (kSetpointJumpMag > trapezoid_jump_threshold.Value()) {
      const T kPosErrRaw = pos_ref_rad.Value() - pos_ref_limited.Value();
      pos_ref_limited += unit::Angle{kPosErrRaw}.Clamp(-unit::Angle{kMaxPosStep}, unit::Angle{kMaxPosStep});
    } else {
      pos_ref_limited = pos_ref_rad;
    }

    // --- Position loop (P) ---
    const T kPosError = pos_ref_limited.Value() - kPosMeas;
    T omega_demand = kp_pos.Value() * kPosError;
    omega_demand = unit::AngularVelocity{omega_demand}.Clamp(-speed_limit, speed_limit).Value();

    // --- Acceleration limit on speed demand ---
    const T kMaxDeltaOmega = accel_limit.Value() * delta_time.Value();
    omega_demand = unit::AngularVelocity{omega_demand}
                       .Clamp(omega_demand_prev - unit::AngularVelocity{kMaxDeltaOmega},
                              omega_demand_prev + unit::AngularVelocity{kMaxDeltaOmega})
                       .Value();
    omega_demand_prev = unit::AngularVelocity{omega_demand};

    // --- Speed loop (PI) ---
    const T kSpeedError = omega_demand - kOmegaMeasValue;

    speed_integrator =
        unit::AngularVelocity{speed_integrator.Value() + ki_speed.Value() * kSpeedError * delta_time.Value()}
            .Clamp(-speed_limit, speed_limit);

    omega_ref =
        unit::AngularVelocity{kp_speed.Value() * kSpeedError + speed_integrator.Value()}.Clamp(-speed_limit,
                                                                                               speed_limit);

    // --- In-position flag ---
    // Compare against the raw setpoint, not the rate-limited intermediate.
    in_position = (std::abs(pos_ref_rad.Value() - kPosMeas) <= position_tolerance.Value()) &&
                  (std::abs(kOmegaMeasValue) <= speed_tolerance.Value());

    return omega_ref;
  }

  /**
   * @brief Begin the homing sequence.
   *
   * Transitions the homing state machine to SEARCHING and commands a slow
   * constant velocity (homing_speed).  The application must monitor the
   * homing event (limit switch, stall, encoder index pulse) and call
   * TriggerZeroing() when the home position is reached.
   */
  constexpr void StartHoming() noexcept {
    homing_state = HomingState::kSearching;
    speed_integrator = unit::AngularVelocity{};
    omega_ref = unit::AngularVelocity{};
    in_position = false;
  }

  /**
   * @brief Signal that the home position has been detected.
   *
   * Call this from the application (ISR or task) when the limit switch fires
   * or stall detection triggers during the SEARCHING phase.
   *
   * The homing state machine transitions to ZEROING; on the very next
   * Update() call the configured home callback is invoked and the state
   * advances to DONE.
   *
   * @note If not currently in the SEARCHING state this call is ignored.
   */
  constexpr void TriggerZeroing() noexcept {
    if (homing_state == HomingState::kSearching) {
      homing_state = HomingState::kZeroing;
    }
  }

  /**
   * @brief Configure the callback used to latch home during ZEROING.
   *
   * @param callback    Function called once in ZEROING.
   * @param context     Opaque pointer passed to callback.
   */
  constexpr void SetHomeCallback(HomeCallback callback, void* context) noexcept {
    home_callback = callback;
    home_callback_context = context;
  }

  /**
   * @brief Signal a homing fault.
   *
   * Transitions the homing state machine to FAULT and stops motion by
   * zeroing omega_ref.  Call Reset() to recover.
   */
  constexpr void Fault() noexcept {
    homing_state = HomingState::kFault;
    omega_ref = unit::AngularVelocity{};
  }

  /**
   * @brief Reset controller state.
   *
   * Clears integrators, resets the homing state machine to kIdle, and
   * zeroes all outputs. Does NOT reset PositionTracker; call
   * PositionTracker::Reset() separately if position tracking must restart.
   */
  constexpr void Reset() noexcept {
    pos_ref_limited = pos_ref_rad;
    omega_demand_prev = unit::AngularVelocity{};
    speed_integrator = unit::AngularVelocity{};
    homing_state = HomingState::kIdle;
    omega_ref = unit::AngularVelocity{};
    in_position = false;
  }
};

}  // namespace unimoc::control
