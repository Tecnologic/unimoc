/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file accelerator_control.cpp
 *    @brief Local torque setpoint from the accelerator and the brake lever, board input supervision.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "accelerator_control.hpp"
#include <algorithm>

namespace unimoc::drive {

using unit::operator""_us;

AcceleratorParameters AcceleratorParameters::From(const settings::NvmSettings& settings) noexcept {
  AcceleratorParameters parameters{};
  parameters.mode = (settings.accel_mode <= static_cast<uint8_t>(AcceleratorMode::kThrottleAndBrake))
                        ? static_cast<AcceleratorMode>(settings.accel_mode)
                        : AcceleratorMode::kOff;
  parameters.throttle_min = settings.accel_throttle_min;
  parameters.throttle_max = settings.accel_throttle_max;
  parameters.brake_min = settings.accel_brake_min;
  parameters.brake_max = settings.accel_brake_max;
  parameters.torque_max = settings.accel_torque_max;
  parameters.brake_torque_max = settings.accel_brake_torque_max;
  parameters.engage_hold = settings.accel_engage_hold;
  return parameters;
}

void AcceleratorControl::Configure(const AcceleratorParameters& parameters) noexcept {
  parameters_ = parameters;
  torque_ = unit::Torque{};
  started_ = false;
  // A changed calibration has to see both inputs released before the first request.
  inhibited_ = true;
}

std::optional<unit::Ratio> AcceleratorControl::Travel(const std::optional<unit::Voltage> voltage,
                                                      const unit::Voltage released,
                                                      const unit::Voltage pressed) const noexcept {
  if (!voltage.has_value() || *voltage < released - parameters_.fault_margin ||
      *voltage > pressed + parameters_.fault_margin) {
    return std::nullopt;
  }
  const unit::Ratio kTravel = std::clamp((*voltage - released) / (pressed - released), 0.0_ratio, 1.0_ratio);
  if (kTravel <= parameters_.deadband) {
    return 0.0_ratio;
  }
  return (kTravel - parameters_.deadband) / (1.0_ratio - parameters_.deadband);
}

unit::Torque AcceleratorControl::Ramp(const unit::Torque target, const unit::Time step) const noexcept {
  const unit::Torque kFull = std::max(parameters_.torque_max, parameters_.brake_torque_max);
  const bool kReversing = (target < 0.0_Nm && torque_ > 0.0_Nm) || (target > 0.0_Nm && torque_ < 0.0_Nm);
  const bool kFalling = kReversing || unit::Abs(target) < unit::Abs(torque_);
  const unit::Time kRamp = kFalling ? parameters_.fall_time : parameters_.rise_time;
  if (kRamp <= unit::Time{} || kFull <= unit::Torque{}) {
    return target;
  }
  const unit::Torque kStep = kFull * (step / kRamp);
  return std::clamp(target, torque_ - kStep, torque_ + kStep);
}

AcceleratorOutput AcceleratorControl::Update(const uint64_t now_us,
                                             const std::optional<unit::Voltage> throttle,
                                             const std::optional<unit::Voltage> brake,
                                             const AcceleratorVehicle& vehicle) noexcept {
  if (!Active()) {
    torque_ = unit::Torque{};
    return AcceleratorOutput{};
  }
  unit::Time step;
  if (started_) {
    step = 1.0_us * static_cast<float>(now_us - last_us_);
  }
  started_ = true;
  last_us_ = now_us;

  const std::optional<unit::Ratio> kThrottle =
      Travel(throttle, parameters_.throttle_min, parameters_.throttle_max);
  const std::optional<unit::Ratio> kBrake = (parameters_.mode == AcceleratorMode::kThrottleAndBrake)
                                                ? Travel(brake, parameters_.brake_min, parameters_.brake_max)
                                                : std::optional<unit::Ratio>{0.0_ratio};
  const bool kInRange = kThrottle.has_value() && kBrake.has_value();
  const bool kReleased = kInRange && *kThrottle <= 0.0_ratio && *kBrake <= 0.0_ratio;
  if (!kInRange) {
    sensor_fault_ = true;
  }
  // A fault, a refused engage or the bus re-arm only once both inputs are released.
  if (vehicle.faulted && !vehicle.engaged) {
    inhibited_ = true;
  }
  if (kReleased) {
    sensor_fault_ = false;
    inhibited_ = false;
  }

  AcceleratorOutput output{};
  output.sensor_fault = sensor_fault_;
  if (sensor_fault_ || inhibited_) {
    torque_ = vehicle.engaged ? Ramp(unit::Torque{}, step) : unit::Torque{};
    output.engage = vehicle.engaged && torque_ != unit::Torque{};  // ramp down, then release
    output.torque = torque_;
    return output;
  }

  // The brake lever wins over the accelerator; it brakes against the motion and fades out at standstill.
  unit::Torque target;
  const unit::AngularVelocity kSpeed = vehicle.motor_speed;
  const unit::Ratio kFade = std::min(unit::Abs(kSpeed) / parameters_.brake_fade_speed, 1.0_ratio);
  bool demand = false;
  if (*kBrake > 0.0_ratio) {
    const unit::Torque kBraking = parameters_.brake_torque_max * (*kBrake * kFade);
    target = (kSpeed > unit::AngularVelocity{}) ? -kBraking : kBraking;
    demand = kFade > parameters_.deadband;  // no engage just to hold a standing motor
  } else if (*kThrottle > 0.0_ratio) {
    target = parameters_.torque_max * *kThrottle;
    demand = true;
  }
  target = std::clamp(target, std::min(vehicle.torque_min, 0.0_Nm), std::max(vehicle.torque_max, 0.0_Nm));

  if (demand) {
    last_demand_us_ = now_us;
  }
  const unit::Time kSinceDemand = 1.0_us * static_cast<float>(now_us - last_demand_us_);
  const bool kHolding = kSinceDemand < parameters_.engage_hold;
  torque_ = vehicle.engaged ? Ramp(target, step) : unit::Torque{};
  output.engage = demand || (vehicle.engaged && (kHolding || torque_ != unit::Torque{}));
  output.torque = torque_;
  return output;
}

BoardVerdict BoardSupervision::Update(const uint64_t now_us,
                                      const hardware::BoardInputs& inputs,
                                      const hardware::GateSupplyLimits& limits,
                                      const bool engaged) noexcept {
  BoardVerdict verdict{};
  bool failing = false;
  if (inputs.power_board.has_value() && !*inputs.power_board) {
    verdict = BoardVerdict{false, false, "no power board"};
    failing = true;
  } else if (inputs.gate_supply.has_value()) {
    if (*inputs.gate_supply < limits.trip) {
      verdict = BoardVerdict{false, false, "gate supply undervoltage"};
      failing = true;
    } else if (*inputs.gate_supply < limits.engage) {
      verdict = BoardVerdict{false, false, "gate supply low"};
    }
  }
  if (failing && !failing_) {
    failing_since_us_ = now_us;
  }
  failing_ = failing;
  verdict.trip = engaged && failing && now_us - failing_since_us_ >= kTripDelayUs;
  return verdict;
}

}  // namespace unimoc::drive
