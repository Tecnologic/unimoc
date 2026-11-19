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
#include "angle_wrap.hpp"
#include "slow_update.hpp"

namespace unimoc::current_control {

using namespace unit;

namespace {

/// Band beyond the ambient and the thermal limit in which a temperature from the stator resistance is
/// plausible.
constexpr Temperature kRsPlausibleMargin = 40.0_degC;

/// Largest difference between a temperature from the stator resistance and the model that is used.
constexpr Temperature kRsMaxInnovation = 80.0_degC;

}  // namespace

// =============================================================================
// init — thread context, while disengaged
// =============================================================================

void OuterLoop::Init(const settings::NvmSettings& settings, CurrentControlIsr& isr) noexcept {
  isr_ = &isr;
  const auto kPolePairs = static_cast<float>(settings.pole_pairs);

  Parameters parameters{};
  parameters.generation = ++generation_;
  parameters.pole_pairs = settings.pole_pairs;
  parameters.dt = isr.state.dt_slow * static_cast<float>(kCyclesPerStep);
  parameters.current_limit = settings.motor_i_max;
  parameters.torque_constant = MotorTorqueConstant(settings, MagneticFlux{});
  parameters.motor_type = settings.motor_type;
  parameters.asm_flux.R_r = settings.asm_r_r;
  parameters.asm_flux.L_r = settings.asm_l_r;
  parameters.asm_flux.L_m = settings.asm_l_m;
  parameters.asm_flux.kp = settings.asm_flux_kp;
  parameters.asm_flux.ki = settings.asm_flux_ki;
  parameters.asm_flux.i_d_min = settings.asm_flux_i_d_min;
  parameters.asm_flux.i_d_max = settings.asm_flux_i_d_max;
  parameters.asm_flux_reference = settings.asm_flux_psi_r;
  field_flux_.Write(MagneticFlux{});
  parameters.dc_link = control::DcLinkLimit{.drive_current_max = settings.battery_drive_current_max,
                                            .charge_current_max = settings.battery_charge_current_max,
                                            .regen_derate_start = settings.dc_link_regen_derate_start,
                                            .trip = settings.dc_link_trip,
                                            .stator_resistance = Resistance{}};  // set by each step
  parameters.speed_min = settings.motor_omega_min / kPolePairs;
  parameters.speed_max = settings.motor_omega_max / kPolePairs;
  parameters.inertia = settings.motor_j;
  parameters.homing_speed = settings.pos_homing_speed;
  parameters.homing_current = settings.pos_homing_current;
  parameters.gains = GainsFrom(settings);
  parameters.thermal =
      control::MotorThermalModel::Parameters{.stator_resistance =
                                                 (settings.motor_type == system::MotorType::kAsm)
                                                     ? settings.asm_r_s
                                                     : settings.stator_r,
                                             .reference_temperature = settings.stator_r_temperature,
                                             .temperature_coefficient =
                                                 control::kCopperTemperatureCoefficient,
                                             .winding_capacity = settings.thermal_c_winding,
                                             .stator_capacity = settings.thermal_c_stator,
                                             .winding_to_stator = settings.thermal_r_winding,
                                             .stator_to_ambient = settings.thermal_r_stator,
                                             .derate_start = settings.thermal_derate_start,
                                             .limit = settings.thermal_limit};
  parameters.thermal_ambient = settings.thermal_ambient;
  parameters.rs_correction_time = settings.thermal_rs_correction_time;
  parameters.sensor_correction_time = settings.thermal_sensor_correction_time;
  parameters_.Write(parameters);
  SetGains(parameters.gains);  // live gains handed over before are older than these
}

MagneticFlux OuterLoop::MotorTorqueConstant(const settings::NvmSettings& settings,
                                            const MagneticFlux field_flux) noexcept {
  const float kTorqueFactor = 1.5F * static_cast<float>(settings.pole_pairs);
  switch (settings.motor_type) {
    case system::MotorType::kAsm:
      return (settings.asm_l_r > Inductance{})
                 ? settings.asm_flux_psi_r * ((settings.asm_l_m / settings.asm_l_r) * kTorqueFactor)
                 : MagneticFlux{};
    case system::MotorType::kEesm:
      return field_flux * kTorqueFactor;
    case system::MotorType::kPmsm:
    default:
      return settings.flux_pm * kTorqueFactor;
  }
}

OuterLoop::Gains OuterLoop::GainsFrom(const settings::NvmSettings& settings) noexcept {
  return Gains{.speed_kp = settings.speed_kp,
               .speed_ki = settings.speed_ki,
               .speed_accel_limit = settings.speed_accel_limit,
               .position_kp = settings.pos_kp_pos,
               .position_speed_limit = settings.pos_speed_limit,
               .position_accel_limit = settings.pos_accel_limit,
               .position_torque_limit = settings.pos_torque_limit,
               .position_tolerance = settings.pos_position_tolerance,
               .position_speed_tolerance = settings.pos_speed_tolerance};
}

void OuterLoop::SetGains(const Gains& gains) noexcept {
  gains_.Write(gains);
  gains_generation_.store(gains_generation_.load(std::memory_order_relaxed) + 1U, std::memory_order_release);
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
  const uint32_t kGainsGeneration = gains_generation_.load(std::memory_order_acquire);
  if (kGainsGeneration != applied_gains_generation_) {
    applied_gains_generation_ = kGainsGeneration;
    ApplyGains(gains_.Read());
  }
  const ObserverState kObserver = observer_.Read();
  const OuterSetpoint kSetpoint = setpoint_.Read();
  control::DcLinkLimit dc_link = active_.dc_link;
  const BatteryLimits kBattery = battery_.Read();
  if (kBattery.valid) {
    dc_link.drive_current_max = kBattery.drive_current;
    dc_link.charge_current_max = kBattery.charge_current;
  }
  const Voltage kDcLink = isr_->state.dc_link;
  UpdateThermal();
  // The copper losses of the battery estimate at the winding temperature.
  dc_link.stator_resistance = thermal_.StatorResistance();
  if (active_.motor_type == system::MotorType::kEesm) {
    // The torque constant follows the field current the excitation module measures.
    active_.torque_constant = field_flux_.Read() * (1.5F * static_cast<float>(active_.pole_pairs));
  }
  speed_.torque_limit = active_.torque_constant * current_limit_;

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
  // ASM: the observer tracks the rotor flux frame; the rotor turns slower by the slip.
  const AngularVelocity kElectricalSpeed =
      (active_.motor_type == system::MotorType::kAsm)
          ? kObserver.electrical_speed - SlowUpdate::Instance().SlipSpeed()
          : kObserver.electrical_speed;
  const AngularVelocity kSpeed = kElectricalSpeed / static_cast<float>(active_.pole_pairs);

  OuterStatus status{};
  status.steps = ++steps_;
  status.speed = kSpeed;
  status.position = tracker_.PositionRad();
  // The battery power window as shaft torque, within the motor torque.
  const control::DcLinkLimit::TorqueRange kTorqueRange =
      dc_link.AllowedTorque(kSpeed, kDcLink, active_.torque_constant);
  const Torque kMotorTorque = active_.torque_constant * current_limit_;
  status.torque_min = unit::Max(kTorqueRange.lower, -kMotorTorque);
  status.torque_max = unit::Min(kTorqueRange.upper, kMotorTorque);
  status.regen_derated = dc_link.AllowedChargeCurrent(kDcLink) < active_.dc_link.charge_current_max;
  status.winding_temperature = thermal_.Winding();
  status.stator_temperature = thermal_.Stator();
  status.stator_resistance = thermal_.StatorResistance();
  status.thermally_derated = current_limit_ < active_.current_limit;
  status.thermal_current = thermal_current_;
  status.rs_valid = rs_valid_;
  status.rs_estimate = rs_estimate_;
  status.rs_temperature = rs_temperature_;
  status.rs_rejected = rs_rejected_;

  UpdateHoming(kSetpoint, kSpeed);

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
    // The speed controller stays inside the battery power window, so its
    // integral does not wind up against the current limit below.
    speed_.torque_min = kTorqueRange.lower;
    speed_.torque_max = kTorqueRange.upper;
    switch (mode_) {
      case cyphal::ControlMode::kTorque:
        current = kSetpoint.current;
        break;
      case cyphal::ControlMode::kSpeed: {
        const AngularVelocity kTarget = kSetpoint.speed.Clamp(active_.speed_min, active_.speed_max);
        current = TorqueToCurrent(speed_.Update(kTarget, kSpeed, active_.dt));
        break;
      }
      case cyphal::ControlMode::kPosition:
        current = TorqueToCurrent(PositionTorque(kSetpoint, kSpeed, kTorqueRange));
        status.in_position = position_.in_position;
        status.trajectory =
            kSetpoint.trajectory && !hold_home_ && homing_ != control::HomingState::kSearching;
        break;
    }
  }
  if (active_.motor_type == system::MotorType::kAsm) {
    // The d current magnetizes an induction motor: the flux controller sets it while engaged.
    if (kSetpoint.active) {
      current.d = active_.asm_flux.Update(active_.asm_flux_reference,
                                          SlowUpdate::Instance().RotorFlux(),
                                          current.q,
                                          active_.dt);
    } else {
      active_.asm_flux.integrator = Current{};
    }
  }
  current = dc_link.Limit(LimitCurrent(current), active_.torque_constant, kSpeed, kDcLink);
  isr_->SetCurrentRef(current);

  status.active = kSetpoint.active;
  status.mode = mode_;
  status.speed_reference = speed_.reference;
  status.torque_reference = speed_.output;
  status.current_reference = current;
  status.homing = homing_;
  status_.Write(status);
}

// =============================================================================
// helpers
// =============================================================================

void OuterLoop::LoadParameters(const Parameters& parameters) noexcept {
  active_ = parameters;
  ApplyGains(parameters.gains);
  // New settings of a warm motor: the temperatures stay, only the first start begins at the ambient.
  thermal_.SetParameters(parameters.thermal);
  if (thermal_restart_.exchange(false, std::memory_order_acq_rel) || !thermal_started_) {
    thermal_.Reset(parameters.thermal_ambient);
    rs_valid_ = false;
    rs_rejected_ = false;
    thermal_started_ = true;
  }
  current_limit_ = parameters.current_limit * thermal_.CurrentLimitScale();
  speed_.torque_limit = parameters.torque_constant * current_limit_;
  SlowUpdate::Instance().SetStatorResistance(thermal_.StatorResistance());  // a warm motor at once
  // The controller restarted with these settings: the observer angle starts
  // again at zero, so the position is counted anew from here.
  tracking_ = false;
  was_active_ = false;
  homing_ = control::HomingState::kIdle;  // the home is the position at the restart again
  hold_home_ = false;
}

void OuterLoop::ApplyGains(const Gains& gains) noexcept {
  active_.gains = gains;
  speed_.kp = gains.speed_kp;
  speed_.ki = gains.speed_ki;
  speed_.accel_limit = gains.speed_accel_limit;
  position_.kp_pos = gains.position_kp;
  position_.speed_limit = gains.position_speed_limit;
  position_.accel_limit = gains.position_accel_limit;
  position_.position_tolerance = gains.position_tolerance;
  position_.speed_tolerance = gains.position_speed_tolerance;
}

void OuterLoop::Restart(const AngularVelocity speed) noexcept {
  speed_.Reset(speed);
  position_.pos_ref_rad = tracker_.PositionRad();
  position_.Reset();
  profile_.Reset(tracker_.PositionRad(), speed);  // a planned move starts from the shaft
}

Torque OuterLoop::PositionTorque(const OuterSetpoint& setpoint,
                                 const AngularVelocity measured,
                                 const control::DcLinkLimit::TorqueRange& range) noexcept {
  if (homing_ == control::HomingState::kSearching) {
    return SearchHome(measured, range, setpoint.position);
  }
  if (hold_home_ && setpoint.position != held_setpoint_) {
    hold_home_ = false;  // a new target in the frame of the new home
  }
  if (hold_home_ || !setpoint.trajectory) {
    return PlannedMove(hold_home_ ? Angle{} : setpoint.position, measured, range);
  }
  // The master plans: its position, speed and torque are the reference of
  // this instant. A planned move that follows starts from this reference.
  const Angle kReference = setpoint.position;
  const AngularVelocity kFeedforward = IsFinite(setpoint.speed) ? setpoint.speed : AngularVelocity{};
  profile_.Reset(kReference, kFeedforward);
  const AngularVelocity kDemand =
      position_.TrajectoryDemand(kReference, kReference, kFeedforward, tracker_.PositionRad(), measured)
          .Clamp(active_.speed_min, active_.speed_max);
  return speed_.Track(kDemand, measured, active_.dt, setpoint.torque);
}

Torque OuterLoop::PlannedMove(const Angle target,
                              const AngularVelocity measured,
                              const control::DcLinkLimit::TorqueRange& range) noexcept {
  const Gains& gains = active_.gains;
  // The torque bound of the move: the positioning limit within the motor and
  // battery limits. Its acceleration leaves the torque to the inertia.
  const Torque kMotorTorque = speed_.torque_limit;
  const Torque kBound = (gains.position_torque_limit > Torque{})
                            ? unit::Min(gains.position_torque_limit, kMotorTorque)
                            : kMotorTorque;
  speed_.torque_min = unit::Max(range.lower, -kBound);
  speed_.torque_max = unit::Min(range.upper, kBound);
  AngularAcceleration accel = gains.position_accel_limit;
  if (active_.inertia > Inertia{} && kBound > Torque{}) {
    const AngularAcceleration kTorqueAccel = kBound / active_.inertia;
    accel = (accel > AngularAcceleration{}) ? unit::Min(accel, kTorqueAccel) : kTorqueAccel;
  }
  // The speed limit of the move within the speed range of the motor.
  const AngularVelocity kSpeedLimit =
      unit::Min(gains.position_speed_limit, unit::Min(active_.speed_max, -active_.speed_min));
  profile_.speed_limit = kSpeedLimit;
  profile_.accel_limit = accel;
  profile_.Step(target, active_.dt);

  const AngularVelocity kDemand =
      position_.TrajectoryDemand(target, profile_.position, profile_.speed, tracker_.PositionRad(), measured)
          .Clamp(-kSpeedLimit, kSpeedLimit);
  return speed_.Track(kDemand, measured, active_.dt, active_.inertia * profile_.acceleration);
}

void OuterLoop::UpdateHoming(const OuterSetpoint& setpoint, const AngularVelocity speed) noexcept {
  // A homing run lives in the engaged position mode only.
  const uint32_t kRequests = homing_requests_.load(std::memory_order_acquire);
  if (kRequests != applied_homing_requests_) {
    applied_homing_requests_ = kRequests;
    TakeHomingRequest(setpoint, speed);
  }
  const bool kPositionMode = setpoint.active && setpoint.mode == cyphal::ControlMode::kPosition;
  if (homing_ == control::HomingState::kSearching && !kPositionMode) {
    homing_ = control::HomingState::kFault;
  }
  if (!setpoint.active) {
    hold_home_ = false;
  }
}

void OuterLoop::TakeHomingRequest(const OuterSetpoint& setpoint, const AngularVelocity speed) noexcept {
  if (!setpoint.active || setpoint.mode != cyphal::ControlMode::kPosition) {
    homing_ = control::HomingState::kFault;  // the drive checked it; the setpoint changed since
    return;
  }
  homing_ = control::HomingState::kSearching;
  blocked_steps_ = 0U;
  hold_home_ = false;
  speed_.Reset(speed);
}

Torque OuterLoop::SearchHome(const AngularVelocity speed,
                             const control::DcLinkLimit::TorqueRange& range,
                             const Angle setpoint) noexcept {
  const Torque kHomingTorque = active_.torque_constant * active_.homing_current;
  speed_.torque_min = unit::Max(range.lower, -kHomingTorque);
  speed_.torque_max = unit::Min(range.upper, kHomingTorque);
  const Torque kTorque = speed_.Update(active_.homing_speed, speed, active_.dt);

  // Stalled against the stop: at the torque bound in the direction of the run, and slow.
  const bool kForward = active_.homing_speed > AngularVelocity{};
  const Torque kBound = kForward ? unit::Min(speed_.torque_limit, speed_.torque_max)
                                 : -unit::Max(-speed_.torque_limit, speed_.torque_min);
  const Torque kPush = kForward ? kTorque : -kTorque;
  const bool kStalled = kBound > Torque{} && kPush >= 0.99F * kBound &&
                        unit::Abs(speed) < 0.5F * unit::Abs(active_.homing_speed);
  blocked_steps_ = kStalled ? blocked_steps_ + 1U : 0U;
  if (static_cast<float>(blocked_steps_) * active_.dt < kHomingBlockTime) {
    return kTorque;
  }
  tracker_.SetHome();
  homing_ = control::HomingState::kDone;
  hold_home_ = true;
  held_setpoint_ = setpoint;
  Restart(speed);  // the position loop starts at zero, the new home
  return speed_.output;
}

system::Rotor<Current> OuterLoop::TorqueToCurrent(const Torque torque) const noexcept {
  if (!(active_.torque_constant > MagneticFlux{})) {
    return {Current{}, Current{}};  // no flux: the motor makes no torque from i_q
  }
  return {Current{}, torque / active_.torque_constant};
}

void OuterLoop::UpdateThermal() noexcept {
  const ThermalWindow kWindow = SlowUpdate::Instance().Thermal();
  if (kWindow.sequence != thermal_sequence_) {
    thermal_sequence_ = kWindow.sequence;
    thermal_.Step(kWindow.current_squared, active_.thermal_ambient, kWindow.duration);
    thermal_current_ = unit::Sqrt(kWindow.current_squared);
    // The winding temperature the measured resistance stands for corrects the
    // model slowly: each estimate by its share of the correction time constant.
    if (kWindow.resistance_estimates > 0U) {
      rs_valid_ = true;
      rs_estimate_ = kWindow.resistance;
      rs_temperature_ = thermal_.TemperatureFromResistance(kWindow.resistance);
      // A temperature far outside the motor's range is an inverter error, not
      // the winding (no or a wrong dead-time compensation): it is not used.
      // So is one far from the model: a warm start differs by tens of kelvin,
      // an uncompensated dead time by a hundred and more.
      rs_rejected_ = !(rs_temperature_ > active_.thermal_ambient - kRsPlausibleMargin) ||
                     !(rs_temperature_ < active_.thermal.limit + kRsPlausibleMargin) ||
                     !(unit::Abs(rs_temperature_ - thermal_.Winding()) < kRsMaxInnovation);
      if (!rs_rejected_ && active_.rs_correction_time > Time{}) {
        thermal_.Correct(rs_temperature_, kWindow.resistance_time / active_.rs_correction_time);
      }
    }
    // A motor sensor sees the stator mass: it corrects with its own time constant.
    const MotorSensor kSensor = motor_sensor_.Read();
    if (kSensor.valid && active_.sensor_correction_time > Time{}) {
      thermal_.CorrectStator(kSensor.temperature, kWindow.duration / active_.sensor_correction_time);
    }
    // The observers follow the warm winding.
    SlowUpdate::Instance().SetStatorResistance(thermal_.StatorResistance());
  }
  current_limit_ = active_.current_limit * thermal_.CurrentLimitScale();
}

system::Rotor<Current> OuterLoop::LimitCurrent(const system::Rotor<Current> current) const noexcept {
  if (!unit::IsFinite(current.d) || !unit::IsFinite(current.q)) {
    return {Current{}, Current{}};
  }
  const Current kMagnitude = current.Length();
  const Current kLimit = current_limit_;
  if (kMagnitude <= kLimit || kMagnitude <= Current{}) {
    return current;
  }
  const Ratio kScale = kLimit / kMagnitude;
  return {current.d * kScale, current.q * kScale};
}

}  // namespace unimoc::current_control
