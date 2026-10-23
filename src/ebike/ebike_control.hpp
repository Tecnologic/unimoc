/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file ebike_control.hpp
 *    @brief E-bike control: pedal sensor, assist law and display link, engagement on demand.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cstdint>
#include "assist_controller.hpp"
#include "ebike_interface.hpp"
#include "kunteng_protocol.hpp"
#include "pedal_observer.hpp"
#include "units.hpp"

namespace unimoc::ebike {

/// Parameters of the e-bike control. The defaults are a starting point, none is tuned (docs/ebike_plan.md).
struct EbikeParameters {
  PedalObserverConfig pedal{};
  AssistConfig assist{};
  /// The drive stays engaged (at zero torque) this long after the assist ended.
  unit::Time engage_hold{2.0_s};
  /// After a fault the pedals must stand still this long before the drive engages again.
  unit::Time rearm_pedal_stop{1.0_s};
  /// Without a display request for this long the display counts as lost: no assist.
  unit::Time display_timeout{2.0_s};
  /// Rolling circumference of the wheel [m]; the display of the wheel size has to match it.
  float wheel_circumference_m{2.1F};
  /// Motor shaft revolutions per wheel revolution (1: hub motor).
  float gear_ratio{1.0F};
  /// Series cells of the battery (battery bars of the display).
  uint8_t battery_cells{10U};
  /// Nominal battery voltage the display is set to (24, 36, 48).
  uint8_t nominal_voltage{36U};
  /// Positive rotor torque and speed drive backward.
  bool reversed{false};
};

/// What the drive tells the e-bike control about the vehicle.
struct EbikeVehicle {
  /// Mechanical speed of the rotor.
  unit::AngularVelocity motor_speed;
  /// Largest shaft torque the drive applies forward now (motor and battery limits).
  unit::Torque torque_limit;
  /// DC link voltage (battery bars).
  unit::Voltage dc_link;
  /// Battery current, positive when discharging (display power).
  unit::Current battery_current;
  unit::Temperature motor_temperature;
  /// The power stage is released for the setpoint.
  bool engaged{false};
  /// A fault or trip is latched, or an engage was refused (the drive waits for a re-arm).
  bool faulted{false};
};

/// Result of one EbikeControl::Update().
struct EbikeOutput {
  /// The drive is to be engaged (the e-bike's readiness). False disengages it and re-arms it.
  bool engage{false};
  /// Shaft torque in the rotor direction; applies while engaged.
  unit::Torque torque;
  /// The display asks for the light.
  bool light{false};
};

/**
 * @brief The e-bike on the battery case controller (docs/ebike_plan.md).
 *
 * Per update (at most every millisecond):
 *
 * - Reads the crank torque and the cadence pulses and runs the PedalObserver.
 * - Reads the display line, decodes the requests (assist level, speed limit, light) and answers each valid
 *   request with the status frame (battery bars, wheel period, current, temperature).
 * - Runs the AssistController on the observer outputs, the display's level and speed limit and the drive's
 *   torque limit.
 *
 * Engagement on demand: the e-bike asks for the engaged drive while the crank turns at an assist level above
 * 0, and for the engage hold afterwards, then releases it. A fault or an engage the drive refused latches
 * until the pedals have stood still for the re-arm time, so a persistent fault cannot re-engage the drive in
 * a loop. Without a display request for the display timeout the level counts as 0.
 *
 * Pure logic in the Cyphal thread; all hardware access goes through hardware::EbikeIo.
 */
class EbikeControl {
 public:
  /// Update period [us]; calls closer together return the last result.
  static constexpr uint64_t kStepUs = 1'000U;

  /**
   * @param io Hardware access; must outlive the control.
   */
  explicit EbikeControl(const hardware::EbikeIo& io) noexcept : io_(&io) { Configure(EbikeParameters{}); }

  /// Takes new parameters and restarts the pedal observer and the assist ramp.
  void Configure(const EbikeParameters& parameters) noexcept;

  /**
   * @brief Advances the control.
   * @param now_us Monotonic time [us].
   * @param vehicle State of the vehicle.
   * @return What the drive is to do.
   */
  EbikeOutput Update(uint64_t now_us, const EbikeVehicle& vehicle) noexcept;

  [[nodiscard]] const PedalObserver& Pedal() const noexcept { return pedal_; }
  [[nodiscard]] const AssistController& Assist() const noexcept { return assist_; }
  /// A valid display request arrived within the display timeout.
  [[nodiscard]] bool DisplayAlive() const noexcept { return display_alive_; }
  /// The latest display request.
  [[nodiscard]] const kunteng::DisplayRequest& Request() const noexcept { return request_; }

 private:
  void ServiceDisplay(uint64_t now_us, const EbikeVehicle& vehicle) noexcept;
  [[nodiscard]] kunteng::BatteryLevel BatteryBars(const EbikeVehicle& vehicle) noexcept;
  [[nodiscard]] unit::Time WheelPeriod(unit::AngularVelocity motor_speed) const noexcept;

  const hardware::EbikeIo* io_;
  EbikeParameters parameters_;
  /// Motor shaft speed of one km/h of vehicle speed.
  unit::AngularVelocity motor_speed_per_kmh_;
  PedalObserver pedal_;
  AssistController assist_;
  kunteng::DisplayDecoder decoder_;
  kunteng::DisplayRequest request_;

  bool started_{false};
  uint64_t last_update_us_{0U};
  uint64_t last_request_us_{0U};
  bool display_alive_{false};
  EbikeOutput output_;
  // Engagement.
  bool wanting_{false};
  uint64_t last_want_us_{0U};
  bool latched_{false};
  uint64_t pedal_still_since_us_{0U};
  bool was_engaged_{false};
  // Battery bars with hysteresis (0: below empty .. 5: four bars).
  std::size_t bars_{5U};
};

}  // namespace unimoc::ebike
