/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file cart_control.hpp
 *    @brief Torque source of a cart wheel drive: wheel command, limp-home from
 *           the pedals, battery limits of the BMS.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cstdint>
#include "battery_limits.hpp"
#include "nvm_settings.hpp"
#include "units.hpp"

namespace unimoc::drive {

/// Parameters of the cart control, from the settings (`unimoc.cart.*`).
struct CartParameters {
  /// Wheel of this drive (unimoc.cart.WheelCommand.WHEEL_*).
  uint8_t wheel{0U};
  /// Positive rotor torque drives the cart backward.
  bool reversed{false};
  /// Timeout of the wheel command, the pedal state and the battery status.
  unit::Time timeout;
  /// Torque ramp when the source of the torque changes.
  settings::TorqueRate torque_ramp;
  /// Wheel torque of a fully pressed pedal in limp-home mode.
  unit::Torque limp_home_torque;
  /// Wheel speed below which the cart stands still.
  unit::AngularVelocity standstill_speed;
  /// Share of the BMS current limits this drive may use.
  unit::Ratio battery_share;

  /// The parameters of the settings.
  [[nodiscard]] static CartParameters From(const settings::NvmSettings& settings) noexcept;
};

/// Pedals of unimoc.cart.PedalState; a channel is the pedal travel 0 to 1, NaN when failed.
struct CartPedals {
  std::array<unit::Ratio, 2> drive{};
  std::array<unit::Ratio, 2> brake{};
  bool drive_plausible{false};
  bool brake_plausible{false};
};

/// Contactor of the battery (unimoc.cart.BatteryStatus.CONTACTOR_*).
enum class Contactor : uint8_t {
  kOpen = 0,
  kPrecharge = 1,
  kClosed = 2,
  kOpening = 3,
};

/// Current limits of the BMS (unimoc.cart.BatteryStatus).
struct CartBattery {
  unit::Current discharge_limit;
  unit::Current charge_limit;
  Contactor contactor{Contactor::kOpen};
};

/// Source of the torque of a cart drive.
enum class CartMode : uint8_t {
  /// No wheel command since the engage: the setpoint subjects of the drive apply.
  kOff,
  /// The torque follows the wheel command of the VDC.
  kVdc,
  /// The VDC timed out: the torque comes from the pedals.
  kLimpHome,
  /// VDC and pedals lost, or the bus went off: the torque ramps to zero.
  kStopping,
};

/// Result of one CartControl::Update().
struct CartOutput {
  CartMode mode{CartMode::kOff};
  /// Shaft torque in the rotor direction, ramped; applies unless the mode is kOff.
  unit::Torque torque;
  /// Limp-home, or still ramping down: the readiness timeout waits.
  bool holds_engagement{false};
};

/**
 * @brief Decides the torque of a cart wheel drive (docs/system_plan.md, section 7).
 *
 * - The first wheel command after the engage makes the drive a cart drive
 *   (kVdc): the torque of its wheel applies on reception, limited by the
 *   drive afterwards. Before that (kOff) the drive follows its other
 *   setpoint subjects.
 * - Without a wheel command for the timeout the drive goes to limp-home
 *   (kLimpHome) while the pedal state is fresh: the brake pedal has
 *   priority, the brake torque opposes the motion and fades out below the
 *   standstill speed (it never drives backward), the drive pedal drives
 *   forward only, both up to the limp-home torque. Without the pedals too
 *   (or after a bus-off) the torque ramps to zero (kStopping).
 * - A fresh wheel command takes over again only at standstill.
 * - Every change of the source ramps the torque at the torque ramp; the
 *   wheel command itself is applied unramped (the VDC shapes it).
 * - The battery limits of the BMS replace those of the settings while the
 *   battery status is fresh, scaled by the battery share; with the
 *   contactor open or precharging the drive takes no battery current.
 *
 * Directions: the wheel command, the pedals and the standstill speed are in
 * the cart frame (positive drives the cart forward); the output is in the
 * rotor frame. Pure logic in the Cyphal thread: no bus, no hardware.
 */
class CartControl {
 public:
  /// Takes new parameters; the state is kept.
  void Configure(const CartParameters& parameters) noexcept { parameters_ = parameters; }

  /// Back to kOff with zero torque (disengaged).
  void Reset() noexcept;

  /// A wheel command: @p wheel_torque is the entry of this drive's wheel (cart frame).
  void OnWheelCommand(unit::Torque wheel_torque, uint64_t now_us) noexcept;

  /// A pedal state.
  void OnPedalState(const CartPedals& pedals, uint64_t now_us) noexcept;

  /// A battery status.
  void OnBatteryStatus(const CartBattery& battery, uint64_t now_us) noexcept;

  /// The bus went off: every input counts as lost at once.
  void OnBusLost() noexcept;

  /**
   * @brief Advances the mode and the ramp.
   * @param now_us Monotonic time [us].
   * @param shaft_speed Mechanical speed of the rotor (rotor frame).
   * @param engaged The power stage is released for the setpoint; otherwise Reset().
   */
  [[nodiscard]] CartOutput Update(uint64_t now_us, unit::AngularVelocity shaft_speed, bool engaged) noexcept;

  /// Battery limits for the outer loop; invalid without a fresh battery status.
  [[nodiscard]] current_control::BatteryLimits Battery(uint64_t now_us) const noexcept;

  /// Mode of the last Update().
  [[nodiscard]] CartMode Mode() const noexcept { return mode_; }

  /// Rotor frame to cart frame (and back: the mapping is its own inverse).
  template <typename U>
  [[nodiscard]] U ToCart(const U value) const noexcept {
    return parameters_.reversed ? -value : value;
  }

 private:
  [[nodiscard]] bool Fresh(bool received, uint64_t at_us, uint64_t now_us) const noexcept;
  /// Limp-home torque of the pedals at the cart speed @p speed (cart frame).
  [[nodiscard]] unit::Torque PedalTorque(unit::AngularVelocity speed) const noexcept;

  CartParameters parameters_{};
  CartMode mode_{CartMode::kOff};
  unit::Torque torque_;  ///< Cart frame, ramped.
  uint64_t last_update_us_{0U};
  bool updated_{false};

  bool has_command_{false};
  uint64_t command_us_{0U};
  unit::Torque command_torque_;

  bool has_pedals_{false};
  uint64_t pedals_us_{0U};
  CartPedals pedals_{};

  bool has_battery_{false};
  uint64_t battery_us_{0U};
  CartBattery battery_{};
};

}  // namespace unimoc::drive
