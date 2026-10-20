/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file hardware_interface_config.hpp
 *    @brief Static configuration of the simulation (native_sim).
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>

namespace unimoc::hardware {

inline constexpr uint8_t kVersionMajor = 1;  ///< Major version of the hardware interface.
inline constexpr uint8_t kVersionMinor = 0;  ///< Minor version of the hardware interface.
inline constexpr uint8_t kVersionPatch = 0;  ///< Patch version of the hardware interface.

inline constexpr uint32_t kVersion = (static_cast<uint32_t>(kVersionMajor) << 16U) |
                                     (static_cast<uint32_t>(kVersionMinor) << 8U) |
                                     kVersionPatch;  ///< Combined hardware interface version.

inline constexpr uint8_t MOTORS = 1;  ///< Number of motors supported by the hardware interface.

// The public interface always exposes ia, ib, and ic. A two-sensor backend
// reconstructs ic as -(ia + ib); a three-sensor backend reads all phases.
inline constexpr uint8_t kCurrentSensorCount = 3U;
inline constexpr bool kCurrentSenseIsLowSide = false;
inline constexpr bool kCurrentSenseHasPhaseMeasurements = true;
inline constexpr bool kCurrentSamplesArePwmWindowed = false;

// Optional temperature channels provided by this board.
inline constexpr bool kCpuTemperatureAvailable = false;
inline constexpr bool kBridgeTemperatureAvailable = false;
inline constexpr bool kMotorTemperatureAvailable = false;

// The simulation emulates the same cadence as the BCC: one current sample per
// PWM half period, one control cycle (control and observer handler) every
// kHalfPeriodsPerCycle samples.
inline constexpr uint8_t kHalfPeriodsPerCycle = 4U;

}  // namespace unimoc::hardware
