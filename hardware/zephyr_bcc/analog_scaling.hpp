/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file analog_scaling.hpp
 *    @brief ADC count to physical unit conversion of the battery case controller.
 *
 *    Board constants of the analog front end. Header only and free of
 *    Zephyr and HAL includes, so it can be unit tested on the host.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include "units.hpp"

namespace unimoc::hardware::analog {

inline constexpr float kAdcResolution = 4096.0F;  ///< 12 bit.
inline constexpr float kVoltageReference = 3.3F;  ///< VDDA [V].

/// Raw count of a current channel at zero current (amplifier biased to mid-scale).
inline constexpr int32_t kCurrentZeroCounts = 2048;

inline constexpr float kShuntResistance = 0.0005F;  ///< [Ohm]
inline constexpr float kCurrentAmplifierGain = 50.0F;
inline constexpr float kVoltageDividerRatio = (33.0F + 1.0F) / 1.0F;  ///< 33k / 1k
inline constexpr float kTorqueScale = 70.0F;                          ///< E-Rider T9N [Nm/V]
inline constexpr float kTorqueRestVoltage = 1.5F;                     ///< E-Rider T9N output at zero load [V]

[[nodiscard]] constexpr float CountsToVolts(const float counts) noexcept {
  return counts * kVoltageReference / kAdcResolution;
}

// The conversions below are the boundary from raw counts to the unit types:
// the scale factors are plain floats here, the results carry their unit.

/// Signed phase current from a raw conversion.
[[nodiscard]] constexpr unit::Current CountsToCurrent(const uint32_t raw) noexcept {
  const auto kCentered = static_cast<float>(static_cast<int32_t>(raw) - kCurrentZeroCounts);
  return unit::Current{CountsToVolts(kCentered) / (kShuntResistance * kCurrentAmplifierGain)};
}

/// Current of one count above zero: the resolution of a current channel.
inline constexpr unit::Current kCurrentPerCount =
    CountsToCurrent(static_cast<uint32_t>(kCurrentZeroCounts + 1));

/// Voltage in front of the 33k/1k divider.
[[nodiscard]] constexpr unit::Voltage CountsToVoltage(const uint32_t raw) noexcept {
  return unit::Voltage{CountsToVolts(static_cast<float>(raw)) * kVoltageDividerRatio};
}

/// Crank torque from the sensor voltage above its nominal rest voltage. The offset of the individual
/// sensor remains; the pedal observer (lib/ebike) trims it.
[[nodiscard]] constexpr unit::Torque CountsToTorque(const uint32_t raw) noexcept {
  return unit::Torque{(CountsToVolts(static_cast<float>(raw)) - kTorqueRestVoltage) * kTorqueScale};
}

static_assert(CountsToCurrent(static_cast<uint32_t>(kCurrentZeroCounts)) == unit::Current{});
static_assert(kCurrentPerCount > unit::Current{});

// E-Rider T9N: 1.5 V at rest, 70 Nm/V, so 2.0 V is 35 Nm. The output range ends at 3.5 V, above VDDA: the
// converter saturates at (3.3 V - 1.5 V) * 70 Nm/V = 126 Nm.
namespace detail {
using unit::operator""_Nm;
constexpr uint32_t kRestCounts =
    static_cast<uint32_t>(kTorqueRestVoltage / kVoltageReference * kAdcResolution);
constexpr uint32_t kTwoVoltCounts = static_cast<uint32_t>(2.0F / kVoltageReference * kAdcResolution);
static_assert(CountsToTorque(kRestCounts) < 0.1_Nm && CountsToTorque(kRestCounts) > -0.1_Nm);
static_assert(CountsToTorque(kTwoVoltCounts) > 34.0_Nm && CountsToTorque(kTwoVoltCounts) < 36.0_Nm);
}  // namespace detail

namespace detail {

inline constexpr std::size_t kNtcTableSize = 128;
inline constexpr float kNtcBeta = 3950.0F;
inline constexpr float kNtcT0 = 298.15F;  ///< 25 degC [K]
inline constexpr float kNtcR0 = 10000.0F;
inline constexpr float kNtcPullup = 10000.0F;
inline constexpr float kNtcAdcStep = (kAdcResolution - 1.0F) / static_cast<float>(kNtcTableSize - 1);

}  // namespace detail

/// Temperature of a 10k NTC (beta 3950) with 10k pull-up, by table interpolation.
[[nodiscard]] inline unit::Temperature CountsToTemperature(const uint32_t raw) noexcept {
  using namespace detail;
  static const std::array<float, kNtcTableSize> kTable = [] {
    std::array<float, kNtcTableSize> table{};
    for (std::size_t i = 1; i < kNtcTableSize; ++i) {
      const float kVolts = CountsToVolts(static_cast<float>(i) * kNtcAdcStep);
      const float kResistance = kNtcPullup * (kVoltageReference / kVolts - 1.0F);
      const float kKelvin = 1.0F / (1.0F / kNtcT0 + (1.0F / kNtcBeta) * std::log(kResistance / kNtcR0));
      table[i] = kKelvin - 273.15F;
    }
    table[0] = table[1];  // 0 counts is an open circuit, hold the first valid value
    return table;
  }();

  const float kPosition = static_cast<float>(raw) / kNtcAdcStep;
  const auto kLower = static_cast<std::size_t>(kPosition);
  const std::size_t kUpper = (kLower + 1 >= kNtcTableSize) ? kNtcTableSize - 1 : kLower + 1;
  const std::size_t kLowerClamped = (kLower >= kNtcTableSize) ? kNtcTableSize - 1 : kLower;
  const float kFraction = kPosition - static_cast<float>(kLowerClamped);
  return unit::Temperature{kTable[kLowerClamped] + kFraction * (kTable[kUpper] - kTable[kLowerClamped])};
}

}  // namespace unimoc::hardware::analog
