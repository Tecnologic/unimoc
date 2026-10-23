/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file ebike_interface.hpp
 *    @brief Platform boundary of the e-bike: pedal sensor and display line (thread context only).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include "units.hpp"

namespace unimoc::hardware {

/**
 * @brief Hardware access of the e-bike layer (docs/ebike_plan.md), called from one thread.
 *
 * The platform keeps the interrupt side (edge capture of the cadence pulse and of the display line, UART
 * transmit); these functions only hand out what was collected. None of them blocks.
 */
struct EbikeIo {
  /// Crank torque, instantaneous reading above the nominal rest voltage of the sensor (offset not trimmed).
  unit::Torque (*read_crank_torque)() noexcept;
  /**
   * Cadence pulses since the last call, oldest first, as the time that has passed since each one.
   * Returns the number of ages written to @p ages; pulses beyond the span are dropped.
   */
  std::size_t (*read_cadence_pulses)(std::span<unit::Time> ages) noexcept;
  /// Bytes received from the display since the last call; returns the number written to @p bytes.
  std::size_t (*read_display)(std::span<uint8_t> bytes) noexcept;
  /// Queues bytes for the display; a frame that does not fit the transmit queue is dropped.
  void (*write_display)(std::span<const uint8_t> bytes) noexcept;
};

/// The e-bike hardware of the platform.
[[nodiscard]] const EbikeIo& GetEbikeIo() noexcept;

/// Starts the interrupt side of the e-bike hardware (pins, capture, transmit); thread context, once.
void StartEbikeIo() noexcept;

}  // namespace unimoc::hardware
