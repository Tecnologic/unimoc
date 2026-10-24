/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file timing_registers.hpp
 *    @brief The execution times of the interrupts as uavcan.register (M0).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstddef>
#include <string_view>
#include "cyphal/node.hpp"

namespace unimoc::drive {

/**
 * @brief Reports the M0 timing statistics (hardware/hot_path_timing.hpp).
 *
 * - `unimoc.timing.sample_isr`, `.control_cycle`, `.observer`
 *   (natural32[4], read only): min, mean and max in cycles of the timestamp
 *   clock and the number of records since the last reset. All zero while
 *   CONFIG_UNIMOC_HOT_PATH_TIMING is off.
 * - `unimoc.timing.overruns` (natural32[2], read only): duty plans applied
 *   late and control cycles that started while the observer still ran.
 * - `unimoc.timing.clock_hz` (natural32[1], read only): the timestamp clock.
 * - `unimoc.timing.reset` (natural8[1]): write 1 to start new statistics.
 *
 * None of these registers is persistent. The provider runs in the Cyphal thread.
 */
class TimingRegisters final : public cyphal::RegisterProvider {
 public:
  [[nodiscard]] std::size_t Count() const noexcept override;
  [[nodiscard]] std::string_view Name(std::size_t index) const noexcept override;
  [[nodiscard]] cyphal::RegisterFlags Read(std::size_t index,
                                           cyphal::RegisterValue& value) const noexcept override;
  [[nodiscard]] cyphal::RegisterWriteResult Write(std::size_t index,
                                                  const cyphal::RegisterValue& value) noexcept override;
};

}  // namespace unimoc::drive
