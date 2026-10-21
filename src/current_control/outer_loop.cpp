/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file outer_loop.cpp
 *    @brief Outer loop of the control thread: setpoint limiting, speed and position control.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "outer_loop.hpp"
#include <cmath>
#include "angle_wrap.hpp"

namespace unimoc::current_control {

using namespace unit;

// =============================================================================
// init — thread context, while disengaged
// =============================================================================

void OuterLoop::Init(const settings::NvmSettings& settings, CurrentControlIsr& isr) noexcept {
  isr_ = &isr;
  const auto kPolePairs = static_cast<float>(settings.pole_pairs);

  Parameters parameters{};
  parameters.generation = ++generation_;
  parameters.pole_pairs = settings.pole_pairs;
  parameters.dt = Time{static_cast<float>(kCyclesPerStep) * DtSlowFromPwmFrequency(settings.pwm_frequency)};
  parameters.current_limit = settings.motor_i_max;
  parameters.torque_constant = 1.5F * kPolePairs * settings.flux_pm.Value();
  parameters.speed_min = settings.motor_omega_min / kPolePairs;
  parameters.speed_max = settings.motor_omega_max / kPolePairs;
  parameters.speed_kp = settings.speed_kp;
  parameters.speed_ki = settings.speed_ki;
  parameters.speed_accel_limit = settings.speed_accel_limit;
  parameters.position_kp = settings.pos_kp_pos;
  parameters.position_speed_limit = settings.pos_speed_limit;
  parameters.position_accel_limit = settings.pos_accel_limit;
  parameters.position_tolerance = settings.pos_position_tolerance;
  parameters.position_speed_tolerance = settings.pos_speed_tolerance;
  parameters_.Write(parameters);
}

// =============================================================================
// on_observer_cycle — observer interrupt, after every control cycle
// =============================================================================

bool OuterLoop::OnObserverCycle() noexcept {
  if (isr_ == nullptr) {
    return false;
  }
  const auto& mech_obs = isr_->mech_obs;
  observer_.Write(ObserverState{isr_->state.control_cycles, mech_obs.theta, mech_obs.omega});
  if (++observer_cycles_ < kCyclesPerStep) {
    return false;
  }
  observer_cycles_ = 0U;
  return true;
}

// =============================================================================
// step — control thread
// =============================================================================

void OuterLoop::Step() noexcept {
  if (isr_ == nullptr) {
    return;
  }
  const Parameters kParameters = parameters_.Read();
  if (kParameters.generation != active_.generation) {
    LoadParameters(kParameters);
  }
  const ObserverState kObserver = observer_.Read();
  const OuterSetpoint kSetpoint = setpoint_.Read();

  // -------------------------------------------------------------------------
  // 1. Mechanical position and speed from the electrical estimate. The
  //    position starts at home (zero) after every restart of the controller.
  // -------------------------------------------------------------------------
  if (tracking_) {
    electrical_position_.Advance(observer::WrapAngle(kObserver.electrical_angle - last_angle_));
    tracker_.Update(electrical_position_, active_.pole_pairs);
  } else {
    electrical_position_ = system::RotorAngle::FromAngle(kObserver.electrical_angle);
    tracker_.Reset();
    tracker_.Update(electrical_position_, active_.pole_pairs);
    tracker_.SetHome();
    tracking_ = true;
  }
  last_angle_ = kObserver.electrical_angle;
  const AngularVelocity kSpeed = kObserver.electrical_speed / static_cast<float>(active_.pole_pairs);

  OuterStatus status{};
  status.steps = ++steps_;
  status.speed = kSpeed;
  status.position = tracker_.PositionRad();

  // -------------------------------------------------------------------------
  // 2. Current reference of the active mode. A new mode or a new engagement
  //    starts the controllers from the measured speed.
  // -------------------------------------------------------------------------
  system::Rotor<Current> current{Current{}, Current{}};
  if (!kSetpoint.active) {
    Restart(kSpeed);
    mode_ = cyphal::ControlMode::kTorque;
    was_active_ = false;
  } else {
    if (!was_active_ || kSetpoint.mode != mode_) {
      Restart(kSpeed);
      position_.pos_ref_rad = kSetpoint.position;
      position_.Reset();
    }
    was_active_ = true;
    mode_ = kSetpoint.mode;
    switch (mode_) {
      case cyphal::ControlMode::kTorque:
        current = kSetpoint.current;
        break;
      case cyphal::ControlMode::kSpeed: {
        const AngularVelocity kTarget = kSetpoint.speed.Clamp(active_.speed_min, active_.speed_max);
        current = TorqueToCurrent(speed_.Update(kTarget, kSpeed, active_.dt));
        break;
      }
      case cyphal::ControlMode::kPosition: {
        position_.pos_ref_rad = kSetpoint.position;
        const AngularVelocity kDemand = position_.SpeedDemand(tracker_.PositionRad(), kSpeed, active_.dt)
                                            .Clamp(active_.speed_min, active_.speed_max);
        current = TorqueToCurrent(speed_.Track(kDemand, kSpeed, active_.dt));
        status.in_position = position_.in_position;
        break;
      }
    }
  }
  current = LimitCurrent(current);
  isr_->SetCurrentRef(current);

  status.active = kSetpoint.active;
  status.mode = mode_;
  status.speed_reference = speed_.reference;
  status.torque_reference = speed_.output;
  status.current_reference = current;
  status_.Write(status);
}

// =============================================================================
// helpers
// =============================================================================

void OuterLoop::LoadParameters(const Parameters& parameters) noexcept {
  active_ = parameters;
  speed_.kp = parameters.speed_kp;
  speed_.ki = parameters.speed_ki;
  speed_.accel_limit = parameters.speed_accel_limit;
  speed_.torque_limit = Torque{parameters.current_limit.Value() * parameters.torque_constant};
  position_.kp_pos = parameters.position_kp;
  position_.speed_limit = parameters.position_speed_limit;
  position_.accel_limit = parameters.position_accel_limit;
  position_.position_tolerance = parameters.position_tolerance;
  position_.speed_tolerance = parameters.position_speed_tolerance;
  // The controller restarted with these settings: the observer angle starts
  // again at zero, so the position is counted anew from here.
  tracking_ = false;
  was_active_ = false;
}

void OuterLoop::Restart(const AngularVelocity speed) noexcept {
  speed_.Reset(speed);
  position_.pos_ref_rad = tracker_.PositionRad();
  position_.Reset();
}

system::Rotor<Current> OuterLoop::TorqueToCurrent(const Torque torque) const noexcept {
  if (!(active_.torque_constant > 0.0F)) {
    return {Current{}, Current{}};  // no flux: the motor makes no torque from i_q
  }
  return {Current{}, Current{torque.Value() / active_.torque_constant}};
}

system::Rotor<Current> OuterLoop::LimitCurrent(const system::Rotor<Current> current) const noexcept {
  const float kD = current.d.Value();
  const float kQ = current.q.Value();
  if (!std::isfinite(kD) || !std::isfinite(kQ)) {
    return {Current{}, Current{}};
  }
  const float kMagnitude = std::sqrt((kD * kD) + (kQ * kQ));
  const float kLimit = active_.current_limit.Value();
  if (kMagnitude <= kLimit || kMagnitude <= 0.0F) {
    return current;
  }
  const float kScale = kLimit / kMagnitude;
  return {Current{kD * kScale}, Current{kQ * kScale}};
}

}  // namespace unimoc::current_control
