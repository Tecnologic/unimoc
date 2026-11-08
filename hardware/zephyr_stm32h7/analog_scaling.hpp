/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file analog_scaling.hpp
 *    @brief ADC count to physical unit conversion of the STM32H7 power stages.
 *
 *    The shunt, the current amplifier gain and the voltage dividers come
 *    from the foc0 node of the board devicetree (Jacobi: one 0.5 mOhm shunt
 *    per phase; the legacy 4850 power board: three 2 mOhm in parallel; both
 *    with an INA240A1 biased to mid-scale and 47k / 2k2 dividers). The NTCs
 *    are 10k (beta 3950) to ground with a 10k pull-up on both boards.
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
#include "foc_stm32h7_dt.hpp"
#include "units.hpp"

namespace unimoc::hardware::analog {

inline constexpr float kAdcResolution = 4096.0F;  ///< 12 bit.
inline constexpr float kVoltageReference = 3.3F;  ///< VREF+ = VDDA [V].

/// Raw count of a current channel at zero current (amplifier biased to mid-scale).
inline constexpr int32_t kCurrentZeroCounts = 2048;

/// Shunt of one phase [Ohm] (foc0 shunt-nano-ohm).
inline constexpr float kShuntResistance = static_cast<float>(foc_stm32h7::kShuntNanoOhm) * 1.0e-9F;
/// Current amplifier gain (foc0 current-amplifier-gain).
inline constexpr float kCurrentAmplifierGain = static_cast<float>(foc_stm32h7::kCurrentAmplifierGain);
/// Divider ratio of Vdc and the phase voltages (foc0 voltage-divider-ohms).
inline constexpr float kVoltageDividerRatio =
    static_cast<float>(foc_stm32h7::kDividerTopOhm + foc_stm32h7::kDividerBottomOhm) /
    static_cast<float>(foc_stm32h7::kDividerBottomOhm);

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

/// Voltage in front of the divider.
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
