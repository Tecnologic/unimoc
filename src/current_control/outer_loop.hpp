/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file outer_loop.hpp
 *    @brief Outer loop of the control thread: setpoint limiting, speed and position control.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <atomic>
#include <cstdint>
#include "battery_limits.hpp"
#include "current_control_isr.hpp"
#include "cyphal/control_mode.hpp"
#include "dc_link_limit.hpp"
#include "mailbox.hpp"
#include "nvm_settings.hpp"
#include "position_controller.hpp"
#include "position_tracker.hpp"
#include "rotor_angle.hpp"
#include "rotor_system.hpp"
#include "speed_controller.hpp"
#include "units.hpp"

/**
 * @namespace current_control current-control subsystem namespace
 */
namespace unimoc::current_control {

/**
 * @brief Setpoint of the outer loop, written by the drive (Cyphal thread).
 *
 * Speeds and positions are mechanical (shaft) values. The position is
 * relative to the home of the position tracker.
 */
struct OuterSetpoint {
  /// The drive is engaged; otherwise the outer loop commands zero current.
  bool active{false};
  /// Selects which of the fields below applies.
  cyphal::ControlMode mode{cyphal::ControlMode::kTorque};
  /// kTorque: d/q current reference [A].
  system::Rotor<unit::Current> current{unit::Current{}, unit::Current{}};
  /// kSpeed: mechanical speed setpoint [rad/s].
  unit::AngularVelocity speed;
  /// kPosition: mechanical position setpoint [rad].
  unit::Angle position;
};

/// State of the outer loop after its last step, for telemetry and tests.
struct OuterStatus {
  /// Steps run since Init().
  uint32_t steps{0U};
  /// The setpoint was active (engaged) in the last step.
  bool active{false};
  /// Mode of the last step.
  cyphal::ControlMode mode{cyphal::ControlMode::kTorque};
  /// Estimated mechanical speed [rad/s].
  unit::AngularVelocity speed;
  /// Estimated mechanical position relative to home [rad].
  unit::Angle position;
  /// Speed reference of the speed controller (ramped) [rad/s].
  unit::AngularVelocity speed_reference;
  /// Torque reference of the speed controller [N m].
  unit::Torque torque_reference;
  /// Current reference handed to the control cycle [A].
  system::Rotor<unit::Current> current_reference{unit::Current{}, unit::Current{}};
  /// Position mode: the shaft is within the position and speed tolerance.
  bool in_position{false};
  /// Most negative shaft torque the drive can apply now: the motor current
  /// limit and the battery power window at the present speed [N m].
  unit::Torque torque_min;
  /// Most positive shaft torque the drive can apply now [N m].
  unit::Torque torque_max;
  /// The charge limit is reduced: by the BMS below the settings, or by the
  /// DC-link overvoltage derating.
  bool regen_derated{false};
  /// Homing run of the position mode (StartHoming()); kDone while the home
  /// of the last run holds, kIdle again after Init().
  control::HomingState homing{control::HomingState::kIdle};
};

/**
 * @brief Outer loop of the drive (milestone M7 of docs/hotpath_plan.md).
 *
 * Execution model
 * ---------------
 *  - The observer interrupt calls OnObserverCycle() after every control
 *    cycle. It hands the angle and speed estimate to the thread (sequence
 *    lock) and returns true every kCyclesPerStep cycles: then the platform
 *    wakes the control thread (2 kHz at a 16 kHz control cycle).
 *  - The control thread calls Step(). It limits the setpoint, runs the speed
 *    controller (speed mode) or the position loop and the speed controller
 *    (position mode), turns the torque into a q current with the torque
 *    constant (PMSM, i_d = 0), limits it to the motor current and to the
 *    battery current limits (control::DcLinkLimit: regenerative braking
 *    derated with the DC-link voltage; the limits of the settings or those
 *    of the BMS, SetBatteryLimits()) and hands the current reference to the
 *    control cycle (CurrentControlIsr::SetCurrentRef(), a mailbox).
 *  - The drive (Cyphal thread) writes setpoints and parameters through
 *    mailboxes; it never touches the controllers.
 *
 * Without an active setpoint (disengaged) the loop commands zero current and
 * keeps its controllers reset, so engaging starts from the measured speed.
 *
 * The position is counted from the electrical angle estimate; the step
 * unwraps it, which needs |electrical speed| < pi / (kCyclesPerStep * dt)
 * (6283 rad/s at a 16 kHz control cycle).
 *
 * Homing: the home is the position at Init() until a homing run sets it.
 * StartHoming() runs the shaft at `unimoc.control.pos.homing_speed` with at
 * most `unimoc.control.pos.homing_current` until it stalls against the stop
 * (kHomingBlockTime at the torque bound, below half the homing speed); that
 * position becomes zero and the loop holds it until the position setpoint
 * changes. A run needs the position mode engaged; a disengage or another
 * mode ends it in kFault.
 */
class OuterLoop {
 public:
  /// Control cycles per outer-loop step.
  static constexpr uint32_t kCyclesPerStep = 8U;

  /// Time the shaft must stall at the homing torque before it counts as home.
  static constexpr unit::Time kHomingBlockTime{0.1F};

  /// Return the single global instance.
  static OuterLoop& Instance() noexcept {
    static OuterLoop obj;
    return obj;
  }

  /**
   * @brief Loads the parameters and resets the loop.
   *
   * Thread context, while the drive is disengaged (StartRuntime()). The
   * parameters reach the control thread through a mailbox; the loop restarts
   * with them at its next step.
   *
   * @param settings Validated settings.
   * @param isr Control cycle that receives the current reference.
   */
  void Init(const settings::NvmSettings& settings, CurrentControlIsr& isr) noexcept;

  /// Hands a new setpoint to the control thread (one writer: the drive).
  void SetSetpoint(const OuterSetpoint& setpoint) noexcept { setpoint_.Write(setpoint); }

  /**
   * @brief Replaces the battery current limits of the settings (BMS).
   *
   * The control thread takes them at its next step and keeps them across
   * Init(); invalid limits return to the settings. One writer: the drive.
   */
  void SetBatteryLimits(const BatteryLimits& limits) noexcept { battery_.Write(limits); }

  /**
   * @brief Starts a homing run (one writer: the drive).
   *
   * The control thread takes it at its next step: in the engaged position
   * mode it starts searching (kSearching, also again after kDone or kFault),
   * otherwise it ends in kFault. Status().homing reports the run.
   */
  void StartHoming() noexcept {
    homing_requests_.store(homing_requests_.load(std::memory_order_relaxed) + 1U, std::memory_order_release);
  }

  /**
   * @brief Observer side, after every control cycle.
   * @return True when the control thread is due for a step.
   */
  [[nodiscard]] bool OnObserverCycle() noexcept;

  /// One step of the outer loop (control thread).
  void Step() noexcept;

  /// State after the last step; any thread below the control thread.
  [[nodiscard]] OuterStatus Status() const noexcept { return status_.Read(); }

  /// Gains of the speed and position loops; they may change while the loop runs.
  struct Gains {
    unit::TorquePerAngularVelocity speed_kp;
    unit::InverseTime speed_ki;
    unit::AngularAcceleration speed_accel_limit;
    unit::AngularVelocityPerAngle position_kp;
    unit::AngularVelocity position_speed_limit;
    unit::AngularAcceleration position_accel_limit;
    unit::Angle position_tolerance;
    unit::AngularVelocity position_speed_tolerance;
  };

  /// The gains of the settings.
  [[nodiscard]] static Gains GainsFrom(const settings::NvmSettings& settings) noexcept;

  /**
   * @brief Changes the gains while the loop runs (live tuning).
   *
   * The control thread takes them at its next step without restarting the
   * controllers or the position count (unlike Init()). One writer: the drive.
   */
  void SetGains(const Gains& gains) noexcept;

 private:
  /// Parameters of the loop, derived from the settings.
  struct Parameters {
    uint32_t generation{0U};  ///< Changes with every Init().
    uint8_t pole_pairs{1U};
    unit::Time dt;                       ///< Period of one step.
    unit::Current current_limit;         ///< Magnitude limit of the current reference.
    unit::MagneticFlux torque_constant;  ///< PMSM 3/2 p psi_pm [N m/A = Wb]; 0: no torque available.
    control::DcLinkLimit dc_link;        ///< Battery current limits, regenerative derating.
    unit::AngularVelocity speed_min;     ///< Mechanical, from the electrical limits.
    unit::AngularVelocity speed_max;
    unit::AngularVelocity homing_speed;  ///< Mechanical speed of the homing run.
    unit::Current homing_current;        ///< Current bound of the homing run.
    Gains gains;
  };

  /// Angle and speed estimate handed from the observer.
  struct ObserverState {
    uint32_t cycles{0U};
    unit::Angle electrical_angle;
    unit::AngularVelocity electrical_speed;
  };

  OuterLoop() = default;

  void LoadParameters(const Parameters& parameters) noexcept;
  void ApplyGains(const Gains& gains) noexcept;
  void Restart(unit::AngularVelocity speed) noexcept;
  [[nodiscard]] unit::Torque PositionTorque(unit::Angle setpoint,
                                            unit::AngularVelocity measured,
                                            const control::DcLinkLimit::TorqueRange& range) noexcept;
  void UpdateHoming(const OuterSetpoint& setpoint, unit::AngularVelocity speed) noexcept;
  void TakeHomingRequest(const OuterSetpoint& setpoint, unit::AngularVelocity speed) noexcept;
  [[nodiscard]] unit::Torque SearchHome(unit::AngularVelocity speed,
                                        const control::DcLinkLimit::TorqueRange& range,
                                        unit::Angle setpoint) noexcept;
  [[nodiscard]] system::Rotor<unit::Current> TorqueToCurrent(unit::Torque torque) const noexcept;
  [[nodiscard]] system::Rotor<unit::Current> LimitCurrent(
      system::Rotor<unit::Current> current) const noexcept;

  CurrentControlIsr* isr_{nullptr};
  Mailbox<OuterSetpoint> setpoint_{};
  Mailbox<BatteryLimits> battery_{};
  Mailbox<Parameters> parameters_{};
  Mailbox<Gains> gains_{};
  std::atomic<uint32_t> gains_generation_{0U};  ///< Counts SetGains(); the control thread compares.
  std::atomic<uint32_t> homing_requests_{0U};   ///< Counts StartHoming(); the control thread compares.
  SeqLock<ObserverState> observer_{};
  SeqLock<OuterStatus> status_{};
  uint32_t observer_cycles_{0U};
  uint32_t generation_{0U};

  // Control thread only.
  Parameters active_{};
  uint32_t applied_gains_generation_{0U};
  uint32_t applied_homing_requests_{0U};
  control::HomingState homing_{control::HomingState::kIdle};
  uint32_t blocked_steps_{0U};  ///< Steps the homing run has stalled at its torque bound.
  bool hold_home_{false};       ///< After a homing run: hold zero until the setpoint changes.
  unit::Angle held_setpoint_;   ///< The position setpoint when the run ended.
  control::SpeedController speed_{};
  control::PositionController<float> position_{};
  observer::PositionTracker tracker_{};
  system::RotorAngle electrical_position_;
  unit::Angle last_angle_;
  bool tracking_{false};
  cyphal::ControlMode mode_{cyphal::ControlMode::kTorque};
  bool was_active_{false};
  uint32_t steps_{0U};
};

}  // namespace unimoc::current_control
