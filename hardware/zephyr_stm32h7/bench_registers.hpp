/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file bench_registers.hpp
 *    @brief Read-only bench registers of the STM32H7 platform (`unimoc.h7.*`).
 *
 *    The raw ADC counts of every foc0 channel, the level of the external
 *    enable and the Hall code, for the bring-up of a board or the Nucleo
 *    adapter (tests/integration/cyphal_hw, test_bench_inputs.py). Thread
 *    context only.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include "cyphal/register_value.hpp"

namespace unimoc::hardware {

/// The bench registers; added to the node by CyphalPlatformStart().
[[nodiscard]] cyphal::RegisterProvider& BenchRegisters() noexcept;

/**
 * @brief The present Hall code (bit 0 = A), read from the pins.
 *
 * Implemented in hardware_interface.cpp. 0 before the first Initialize()
 * configured the Hall inputs.
 */
[[nodiscard]] uint8_t PresentHallCode() noexcept;

}  // namespace unimoc::hardware
