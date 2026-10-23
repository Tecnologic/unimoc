/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file ebike_control.cpp
 *    @brief E-bike control: pedal sensor, assist law and display link, engagement on demand.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "ebike_control.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <span>

namespace unimoc::ebike {

namespace {

constexpr std::size_t kMaxPulsesPerUpdate = 8U;
constexpr std::size_t kMaxDisplayBytes = 16U;
constexpr uint64_t kMaxStepUs = 20'000U;  // a long gap is a pause of the thread, not a long time step
// Battery bars: per-cell voltage of the bar thresholds (index 1: empty .. 5: four bars) and their hysteresis.
constexpr std::array<float, 6> kCellVolts{0.0F, 3.15F, 3.35F, 3.55F, 3.75F, 3.95F};
constexpr float kBarHysteresisCellVolts = 0.05F;
// The bars follow the voltage only while the load is small (the voltage sags under load).
constexpr unit::Current kBarsMaxCurrent = 2.0_A;
constexpr std::array<kunteng::BatteryLevel, 6> kBarCodes{kunteng::BatteryLevel::kBelowEmpty,
                                                         kunteng::BatteryLevel::kEmpty,
                                                         kunteng::BatteryLevel::kOneBar,
                                                         kunteng::BatteryLevel::kTwoBars,
                                                         kunteng::BatteryLevel::kThreeBars,
                                                         kunteng::BatteryLevel::kFourBars};
constexpr uint8_t kErrorFault = 0x01U;
constexpr unit::Time kMaxWheelPeriod = 65'535.0_ms;

}  // namespace

void EbikeControl::Configure(const EbikeParameters& parameters) noexcept {
  parameters_ = parameters;
  pedal_ = PedalObserver(parameters.pedal);
  assist_ = AssistController(parameters.assist);
  // One km/h of vehicle speed turns the wheel at (1 / 3.6 m/s) / (circumference / 2 pi) and the motor gear
  // times that.
  const float kRadPerSecondPerKmh = (2.0F * std::numbers::pi_v<float> * parameters.gear_ratio) /
                                    (std::max(parameters.wheel_circumference_m, 0.1F) * 3.6F);
  motor_speed_per_kmh_ = unit::AngularVelocity{kRadPerSecondPerKmh};
  wanting_ = false;
  latched_ = false;
  output_ = EbikeOutput{};
}

EbikeOutput EbikeControl::Update(const uint64_t now_us, const EbikeVehicle& vehicle) noexcept {
  if (started_ && now_us - last_update_us_ < kStepUs) {
    return output_;
  }
  const uint64_t kStepElapsedUs = started_ ? std::min(now_us - last_update_us_, kMaxStepUs) : kStepUs;
  if (!started_) {
    started_ = true;
    pedal_still_since_us_ = now_us;
  }
  last_update_us_ = now_us;
  const unit::Time kDt = unit::Time{static_cast<float>(kStepElapsedUs) * 1e-6F};

  // Pedal sensor.
  std::array<unit::Time, kMaxPulsesPerUpdate> ages{};
  const std::size_t kPulses = io_->read_cadence_pulses(std::span<unit::Time>{ages});
  pedal_.Step(kDt,
              io_->read_crank_torque(),
              std::span<const unit::Time>{ages.data(), std::min(kPulses, ages.size())});

  ServiceDisplay(now_us, vehicle);

  // Directions: the pedals and the display speed limit are in the cart frame, the rotor frame flips with
  // `reversed`.
  const unit::AngularVelocity kForwardSpeed =
      parameters_.reversed ? -vehicle.motor_speed : vehicle.motor_speed;
  AssistInput input;
  input.pedaling = pedal_.Pedaling();
  input.cadence = pedal_.Cadence();
  input.pedal_torque = pedal_.MeanTorque();
  input.motor_speed = kForwardSpeed;
  input.speed_limit = motor_speed_per_kmh_ * static_cast<float>(request_.max_speed);
  input.torque_limit = parameters_.reversed ? -vehicle.torque_min : vehicle.torque_max;
  input.level = display_alive_ ? request_.assist_level : 0U;

  // Engagement on demand.
  const bool kWant = display_alive_ && request_.assist_level > 0U && pedal_.Pedaling();
  if (pedal_.Pedaling()) {
    pedal_still_since_us_ = now_us;
  }
  if (vehicle.faulted && (wanting_ || was_engaged_ || kWant)) {
    latched_ = true;
  }
  if (latched_ &&
      now_us - pedal_still_since_us_ >= static_cast<uint64_t>(parameters_.rearm_pedal_stop.Value() * 1e6F)) {
    latched_ = false;
  }
  if (kWant && !latched_) {
    wanting_ = true;
    last_want_us_ = now_us;
  } else if (wanting_ && (latched_ || now_us - last_want_us_ >=
                                          static_cast<uint64_t>(parameters_.engage_hold.Value() * 1e6F))) {
    wanting_ = false;
  }
  was_engaged_ = vehicle.engaged;

  // The assist torque is zero until the drive is engaged, and the ramp restarts from zero when it is not
  // wanted.
  if (!wanting_ || latched_) {
    input.level = 0U;
  }
  const unit::Torque kTorque = assist_.Step(kDt, input);

  output_.engage = wanting_ && !latched_;
  output_.torque = parameters_.reversed ? -kTorque : kTorque;
  output_.light = display_alive_ && request_.light;
  return output_;
}

void EbikeControl::ServiceDisplay(const uint64_t now_us, const EbikeVehicle& vehicle) noexcept {
  std::array<uint8_t, kMaxDisplayBytes> bytes{};
  const std::size_t kCount = std::min(io_->read_display(std::span<uint8_t>{bytes}), bytes.size());
  for (std::size_t i = 0U; i < kCount; ++i) {
    kunteng::DisplayRequest request;
    if (!decoder_.Push(bytes[i], request)) {
      continue;
    }
    request_ = request;
    last_request_us_ = now_us;
    display_alive_ = true;

    kunteng::ControllerStatus status;
    status.battery_level = BatteryBars(vehicle);
    status.nominal_voltage = parameters_.nominal_voltage;
    status.wheel_period = WheelPeriod(vehicle.motor_speed);
    status.error_code = (vehicle.faulted || latched_) ? kErrorFault : 0U;
    status.pas = pedal_.Pedaling();
    status.battery_current = vehicle.battery_current;
    status.motor_temperature = vehicle.motor_temperature;
    const auto kFrame = kunteng::EncodeControllerFrame(status);
    io_->write_display(std::span<const uint8_t>{kFrame});
  }
  if (display_alive_ &&
      now_us - last_request_us_ >= static_cast<uint64_t>(parameters_.display_timeout.Value() * 1e6F)) {
    display_alive_ = false;
  }
}

kunteng::BatteryLevel EbikeControl::BatteryBars(const EbikeVehicle& vehicle) noexcept {
  if (vehicle.battery_current < kBarsMaxCurrent && vehicle.battery_current > -kBarsMaxCurrent) {
    const float kCells = static_cast<float>(std::max<uint8_t>(parameters_.battery_cells, 1U));
    const float kCellVoltage = (vehicle.dc_link / 1.0_V).Value() / kCells;
    std::size_t up = 0U;
    std::size_t down = 0U;
    for (std::size_t level = 1U; level < kCellVolts.size(); ++level) {
      if (kCellVoltage >= kCellVolts[level] + kBarHysteresisCellVolts) {
        up = level;
      }
      if (kCellVoltage >= kCellVolts[level] - kBarHysteresisCellVolts) {
        down = level;
      }
    }
    if (bars_ < up) {
      bars_ = up;
    } else if (bars_ > down) {
      bars_ = down;
    }
  }
  return kBarCodes[bars_];
}

unit::Time EbikeControl::WheelPeriod(const unit::AngularVelocity motor_speed) const noexcept {
  const unit::AngularVelocity kSpeed = motor_speed < unit::AngularVelocity{} ? -motor_speed : motor_speed;
  if (kSpeed <= unit::AngularVelocity{}) {
    return kMaxWheelPeriod;
  }
  // One wheel revolution is 2 pi * gear ratio radians of the motor shaft.
  const unit::Angle kWheelTurn = unit::Angle{2.0F * std::numbers::pi_v<float> * parameters_.gear_ratio};
  return std::min(kWheelTurn / kSpeed, kMaxWheelPeriod);
}

}  // namespace unimoc::ebike
