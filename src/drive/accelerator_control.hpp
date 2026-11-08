/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file accelerator_control.hpp
 *    @brief Local torque setpoint from the accelerator and the brake lever of the board, and the
 *           supervision of the board inputs that gate the engage.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include <optional>
#include <string_view>
#include "board_inputs_interface.hpp"
#include "nvm_settings.hpp"
#include "units.hpp"

namespace unimoc::drive {

using unit::operator""_V;
using unit::operator""_Nm;
using unit::operator""_s;
using unit::operator""_ratio;
using unit::operator""_rad_per_s;

/// What `unimoc.accel.mode` selects.
enum class AcceleratorMode : uint8_t {
  kOff = 0,
  kThrottle = 1,          ///< Accelerator only; the brake input is not read.
  kThrottleAndBrake = 2,  ///< Accelerator and brake lever.
};

/// Parameters of AcceleratorControl; the first group comes from the settings (`unimoc.accel.*`).
struct AcceleratorParameters {
  AcceleratorMode mode{AcceleratorMode::kOff};
  unit::Voltage throttle_min{0.8_V};
  unit::Voltage throttle_max{2.6_V};
  unit::Voltage brake_min{0.8_V};
  unit::Voltage brake_max{2.6_V};
  unit::Torque torque_max{10.0_Nm};
  unit::Torque brake_torque_max{5.0_Nm};
  unit::Time engage_hold{2.0_s};

  /// Share of the travel below which an input counts as released.
  unit::Ratio deadband{0.05_ratio};
  /// A sensor voltage this far outside its calibrated range is a wire break or a short.
  unit::Voltage fault_margin{0.3_V};
  /// The braking torque fades out linearly below this mechanical speed, so the brake never reverses.
  unit::AngularVelocity brake_fade_speed{10.0_rad_per_s};
  /// Time to ramp the torque from zero to its full value, and back.
  unit::Time rise_time{0.3_s};
  unit::Time fall_time{0.1_s};

  [[nodiscard]] static AcceleratorParameters From(const settings::NvmSettings& settings) noexcept;
};

/// The drive state AcceleratorControl::Update() needs.
struct AcceleratorVehicle {
  /// Mechanical speed of the rotor.
  unit::AngularVelocity motor_speed;
  /// Shaft torque window the drive can apply now (motor and battery limits), rotor frame.
  unit::Torque torque_min;
  unit::Torque torque_max;
  /// The power stage is released for the setpoint.
  bool engaged{false};
  /// A fault or trip is latched, or an engage was refused (the drive waits for a re-arm).
  bool faulted{false};
};

/// Result of one AcceleratorControl::Update().
struct AcceleratorOutput {
  /// The drive is to be engaged. False disengages it and re-arms it.
  bool engage{false};
  /// Shaft torque in the rotor direction; applies while engaged.
  unit::Torque torque;
  /// A sensor reads outside its range (or is missing); latched until both inputs read released again.
  bool sensor_fault{false};
};

/**
 * @brief Torque setpoint of a drive without the e-bike from its accelerator and brake lever.
 *
 * The accelerator gives a drive torque up to `torque_max`, proportional to its
 * travel beyond the deadband. A pulled brake lever (mode 2) zeroes the
 * accelerator and brakes regeneratively up to `brake_torque_max`, against the
 * direction of motion and fading out below `brake_fade_speed`. The torque
 * ramps with `rise_time` and `fall_time` and stays within the drive's torque window.
 *
 * The control asks for the drive while an input is pressed and for
 * `engage_hold` after the release. After a fault, a refused engage or
 * Inhibit(), both inputs have to read released before it asks again; so does
 * a sensor fault (a voltage outside the calibrated range by `fault_margin`).
 */
class AcceleratorControl {
 public:
  void Configure(const AcceleratorParameters& parameters) noexcept;

  /// The mode is not off.
  [[nodiscard]] bool Active() const noexcept { return parameters_.mode != AcceleratorMode::kOff; }

  /// Runs once per drive spin with the board inputs.
  [[nodiscard]] AcceleratorOutput Update(uint64_t now_us,
                                         std::optional<unit::Voltage> throttle,
                                         std::optional<unit::Voltage> brake,
                                         const AcceleratorVehicle& vehicle) noexcept;

  /// The bus took the drive (STANDBY, SLEEP): no request until both inputs read released.
  void Inhibit() noexcept { inhibited_ = true; }

 private:
  /// Travel of one input above the deadband, 0 to 1; nullopt: the sensor is out of range.
  [[nodiscard]] std::optional<unit::Ratio> Travel(std::optional<unit::Voltage> voltage,
                                                  unit::Voltage released,
                                                  unit::Voltage pressed) const noexcept;
  [[nodiscard]] unit::Torque Ramp(unit::Torque target, unit::Time step) const noexcept;

  AcceleratorParameters parameters_{};
  unit::Torque torque_;
  uint64_t last_us_{0U};
  uint64_t last_demand_us_{0U};
  bool started_{false};
  bool sensor_fault_{false};
  bool inhibited_{false};
};

/// Verdict of BoardSupervision::Update().
struct BoardVerdict {
  /// The board inputs allow the engage.
  bool ready{true};
  /// An engaged power stage has to go off with a latched fault.
  bool trip{false};
  /// Why the board is not ready (or trips); empty while ready.
  std::string_view reason;
};

/**
 * @brief Supervision of the power board detect and the gate driver supply.
 *
 * Not ready without the power board or below the engage limit of the gate
 * supply. While engaged, the loss of the power board or a gate supply below
 * the trip limit trips once it lasted `kTripDelayUs` (a single noisy
 * conversion does not). A board without an input does not gate on it.
 */
class BoardSupervision {
 public:
  /// How long a trip condition has to last [us].
  static constexpr uint64_t kTripDelayUs = 5'000U;

  [[nodiscard]] BoardVerdict Update(uint64_t now_us,
                                    const hardware::BoardInputs& inputs,
                                    const hardware::GateSupplyLimits& limits,
                                    bool engaged) noexcept;

 private:
  bool failing_{false};
  uint64_t failing_since_us_{0U};
};

}  // namespace unimoc::drive
