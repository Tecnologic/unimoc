/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file cart_control.cpp
 *    @brief Torque source of a cart wheel drive: wheel command, limp-home from
 *           the pedals, battery limits of the BMS.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "cart_control.hpp"

namespace unimoc::drive {

namespace {

using namespace unit;

/// Pedal travel below which a pedal counts as released (sensor noise at rest).
constexpr Ratio kPedalDeadband = 0.05_ratio;

/// @p value limited to 0..1; NaN gives 0.
[[nodiscard]] Ratio ClampTravel(const Ratio value) noexcept {
  return unit::IsFinite(value) ? value.Clamp(0.0_ratio, 1.0_ratio) : 0.0_ratio;
}

/// A duration in microseconds as a time.
[[nodiscard]] Time Microseconds(const uint64_t duration_us) noexcept {
  return 1.0e-6_s * static_cast<float>(duration_us);
}

}  // namespace

CartParameters CartParameters::From(const settings::NvmSettings& settings) noexcept {
  return CartParameters{.wheel = settings.cart_wheel,
                        .reversed = settings.cart_reversed != 0U,
                        .timeout = settings.cart_timeout,
                        .torque_ramp = settings.cart_torque_ramp,
                        .limp_home_torque = settings.cart_limp_home_torque,
                        .standstill_speed = settings.cart_standstill_speed,
                        .battery_share = settings.cart_battery_share};
}

void CartControl::Reset() noexcept {
  mode_ = CartMode::kOff;
  torque_ = Torque{};
}

void CartControl::OnWheelCommand(const Torque wheel_torque, const uint64_t now_us) noexcept {
  has_command_ = true;
  command_us_ = now_us;
  command_torque_ = unit::IsFinite(wheel_torque) ? wheel_torque : Torque{};
}

void CartControl::OnPedalState(const CartPedals& pedals, const uint64_t now_us) noexcept {
  has_pedals_ = true;
  pedals_us_ = now_us;
  pedals_ = pedals;
}

void CartControl::OnBatteryStatus(const CartBattery& battery, const uint64_t now_us) noexcept {
  has_battery_ = true;
  battery_us_ = now_us;
  battery_ = battery;
}

void CartControl::OnBusLost() noexcept {
  has_command_ = false;
  has_pedals_ = false;
  has_battery_ = false;
}

bool CartControl::Fresh(const bool received, const uint64_t at_us, const uint64_t now_us) const noexcept {
  return received && Microseconds(now_us - at_us) <= parameters_.timeout;
}

CartOutput CartControl::Update(const uint64_t now_us,
                               const AngularVelocity shaft_speed,
                               const bool engaged) noexcept {
  const Time kDt = updated_ ? Microseconds(now_us - last_update_us_) : Time{};
  updated_ = true;
  last_update_us_ = now_us;
  if (!engaged) {
    Reset();
    return CartOutput{};
  }

  const bool kCommand = Fresh(has_command_, command_us_, now_us);
  const bool kPedals = Fresh(has_pedals_, pedals_us_, now_us);
  const AngularVelocity kSpeed = ToCart(shaft_speed);
  const bool kStandstill = unit::Abs(kSpeed) <= parameters_.standstill_speed;
  switch (mode_) {
    case CartMode::kOff:
      if (kCommand) {
        mode_ = CartMode::kVdc;
      }
      break;
    case CartMode::kVdc:
      if (!kCommand) {
        mode_ = kPedals ? CartMode::kLimpHome : CartMode::kStopping;
      }
      break;
    case CartMode::kLimpHome:
    case CartMode::kStopping:
      if (kCommand && kStandstill) {
        mode_ = CartMode::kVdc;
      } else {
        mode_ = kPedals ? CartMode::kLimpHome : CartMode::kStopping;
      }
      break;
  }

  switch (mode_) {
    case CartMode::kOff:
      torque_ = Torque{};
      break;
    case CartMode::kVdc:
      torque_ = command_torque_;
      break;
    case CartMode::kLimpHome:
    case CartMode::kStopping: {
      const Torque kTarget = (mode_ == CartMode::kLimpHome) ? PedalTorque(kSpeed) : Torque{};
      const Torque kStep = parameters_.torque_ramp * kDt;
      torque_ = kTarget.Clamp(torque_ - kStep, torque_ + kStep);
      break;
    }
  }

  return CartOutput{.mode = mode_,
                    .torque = ToCart(torque_),
                    .holds_engagement = mode_ == CartMode::kLimpHome ||
                                        (mode_ == CartMode::kStopping && torque_ != Torque{})};
}

Torque CartControl::PedalTorque(const AngularVelocity speed) const noexcept {
  Ratio brake = 0.0_ratio;
  for (const Ratio kChannel : pedals_.brake) {
    brake = unit::Max(brake, ClampTravel(kChannel));  // the larger valid channel
  }
  if (brake > kPedalDeadband) {
    // Against the motion; fades out towards standstill, so it never drives backward.
    const Ratio kFade = (parameters_.standstill_speed > AngularVelocity{})
                            ? unit::Min(unit::Abs(speed) / parameters_.standstill_speed, 1.0_ratio)
                            : 1.0_ratio;
    const Torque kBrake = parameters_.limp_home_torque * brake * kFade;
    if (speed > AngularVelocity{}) {
      return -kBrake;
    }
    return (speed < AngularVelocity{}) ? kBrake : Torque{};
  }
  if (!pedals_.drive_plausible) {
    return Torque{};  // an implausible drive pedal counts as released
  }
  const Ratio kDrive = unit::Min(ClampTravel(pedals_.drive[0]), ClampTravel(pedals_.drive[1]));
  return (kDrive > kPedalDeadband) ? parameters_.limp_home_torque * kDrive : Torque{};
}

current_control::BatteryLimits CartControl::Battery(const uint64_t now_us) const noexcept {
  if (!Fresh(has_battery_, battery_us_, now_us)) {
    return current_control::BatteryLimits{};
  }
  if (!unit::IsFinite(battery_.discharge_limit) || !unit::IsFinite(battery_.charge_limit)) {
    return current_control::BatteryLimits{};  // no limits from the BMS: the settings apply
  }
  const bool kConnected =
      battery_.contactor == Contactor::kClosed || battery_.contactor == Contactor::kOpening;
  if (!kConnected) {
    return current_control::BatteryLimits{.valid = true};  // no battery current at all
  }
  return current_control::BatteryLimits{.valid = true,
                                        .drive_current = unit::Max(battery_.discharge_limit, Current{}) *
                                                         parameters_.battery_share,
                                        .charge_current = unit::Max(battery_.charge_limit, Current{}) *
                                                          parameters_.battery_share};
}

}  // namespace unimoc::drive
