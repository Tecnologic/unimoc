/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file hardware_interface_config.hpp
 *    @brief Static configuration of the STM32H7 platform (Jacobi, NUCLEO-H743ZI).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
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

inline constexpr uint8_t kMotorCount = 1;  ///< Number of motors supported by the hardware interface.

// The public interface always exposes ia, ib, and ic. A two-sensor backend
// reconstructs ic as -(ia + ib); a three-sensor backend reads all phases.
// Inline or low-side sensing: settings::HardwareCapabilities::current_sense
// of the settings profile.
inline constexpr uint8_t kCurrentSensorCount = 3U;
inline constexpr bool kCurrentSenseHasPhaseMeasurements = true;

// Optional temperature channels provided by this board.
inline constexpr bool kCpuTemperatureAvailable = false;
inline constexpr bool kBridgeTemperatureAvailable = true;
inline constexpr bool kMotorTemperatureAvailable = true;

// Interrupt layout (docs/hotpath_plan.md):
//
//   sample:   end of the injected sequence of the phase A ADC, zero latency, once per PWM half
//             period. Copies the raw currents, checks the overcurrent limit and
//             runs the current control, which writes the duties of the next
//             half period directly into the CCR preload.
//   observer: software-pended normal-priority interrupt after each control
//             cycle (every fourth sample). It is preempted by the sample interrupt.

// Timestamps of the samples and the Hall edges: the DWT cycle
// counter at the 400 MHz core clock (silicon rev. Y limit; the board overlay
// sets the clock tree, hardware_interface.cpp checks it), wrapping every 10.7 s.
inline constexpr uint32_t kTimestampFrequency = 400'000'000U;

}  // namespace unimoc::hardware
