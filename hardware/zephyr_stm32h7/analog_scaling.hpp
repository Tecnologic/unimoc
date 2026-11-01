/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file analog_scaling.hpp
 *    @brief ADC count to physical unit conversion of the 4850 power board.
 *
 *    Constants of the analog front end of the 4850 power board, taken from
 *    the legacy firmware (unimoc_old_software, branch electric_quad,
 *    hardware/4850/analog.cpp): three 2 mOhm shunts in parallel per phase
 *    with an INA240A1 (20 V/V) biased to mid-scale, a 24k / 1k2 DC-link
 *    divider, 10k NTCs (beta 3950) to ground with a 10k pull-up. On a
 *    NUCLEO-H743ZI without the power board the values only scale whatever
 *    is wired to the pins. Header only and free of Zephyr and HAL includes.
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
inline constexpr float kVoltageReference = 3.3F;  ///< VREF+ = VDDA [V].

/// Raw count of a current channel at zero current (amplifier biased to mid-scale).
inline constexpr int32_t kCurrentZeroCounts = 2048;

inline constexpr float kShuntResistance = 0.002F / 3.0F;  ///< three 2 mOhm shunts in parallel [Ohm]
inline constexpr float kCurrentAmplifierGain = 20.0F;     ///< INA240A1
inline constexpr float kVoltageDividerRatio = (24.0F + 1.2F) / 1.2F;  ///< 24k / 1k2

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

/// Voltage in front of the 24k / 1k2 divider.
[[nodiscard]] constexpr unit::Voltage CountsToVoltage(const uint32_t raw) noexcept {
  return unit::Voltage{CountsToVolts(static_cast<float>(raw)) * kVoltageDividerRatio};
}

/// DC-link voltage of one count: the resolution of the DC-link channel.
inline constexpr unit::Voltage kVoltagePerCount = CountsToVoltage(1U);

static_assert(CountsToCurrent(static_cast<uint32_t>(kCurrentZeroCounts)) == unit::Current{});
static_assert(kCurrentPerCount > unit::Current{});

namespace detail {

inline constexpr std::size_t kNtcTableSize = 128;
inline constexpr float kNtcBeta = 3950.0F;
inline constexpr float kNtcT0 = 298.15F;  ///< 25 degC [K]
inline constexpr float kNtcR0 = 10000.0F;
inline constexpr float kNtcPullup = 10000.0F;
inline constexpr float kNtcAdcStep = (kAdcResolution - 1.0F) / static_cast<float>(kNtcTableSize - 1);

}  // namespace detail

/// Temperature of a 10k NTC (beta 3950) to ground with a 10k pull-up, by table interpolation.
[[nodiscard]] inline unit::Temperature CountsToTemperature(const uint32_t raw) noexcept {
  using namespace detail;
  static const std::array<float, kNtcTableSize> kTable = [] {
    std::array<float, kNtcTableSize> table{};
    for (std::size_t i = 0; i + 1 < kNtcTableSize; ++i) {
      const float kVolts = CountsToVolts(static_cast<float>(i) * kNtcAdcStep);
      const float kResistance = kNtcPullup * kVolts / (kVoltageReference - kVolts);
      const float kKelvin = 1.0F / (1.0F / kNtcT0 + (1.0F / kNtcBeta) * std::log(kResistance / kNtcR0));
      table[i] = kKelvin - 273.15F;
    }
    // Full scale is an open circuit (no NTC): hold the last valid value.
    table[kNtcTableSize - 1] = table[kNtcTableSize - 2];
    table[0] = table[1];  // 0 counts is a short: hold the first finite value
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
