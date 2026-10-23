/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file sim_ebike.hpp
 *    @brief Simulated rider and display of the native_sim platform.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cstdint>
#include "kunteng_protocol.hpp"
#include "units.hpp"

namespace unimoc::hardware::sim {

/// Turns the simulated crank: @p cadence (crank speed) with a mean pedal torque of @p torque; zero cadence
/// stands still.
void SetPedal(unit::AngularVelocity cadence, unit::Torque torque) noexcept;

/// The simulated display polls every 100 ms with this assist level and speed limit [km/h].
void SetDisplay(uint8_t level, uint8_t max_speed_kmh, bool light = false) noexcept;

/// The simulated display falls silent and forgets the status frames it received.
void SilenceDisplay() noexcept;

/// The last status frame the controller sent to the display; all zero before the first one.
[[nodiscard]] std::array<uint8_t, ebike::kunteng::kControllerFrameSize> LastDisplayStatus() noexcept;

/// Number of status frames the controller sent.
[[nodiscard]] uint32_t DisplayStatusCount() noexcept;

/**
 * @brief Takes over the clock of the simulated rider and display.
 *
 * By default they follow the kernel time. A test that runs virtual time itself hands the time in with
 * SetTime() instead.
 */
void UseManualTime(bool manual) noexcept;

/// The time of the rider and display in manual mode [us].
void SetTime(uint64_t now_us) noexcept;

}  // namespace unimoc::hardware::sim
