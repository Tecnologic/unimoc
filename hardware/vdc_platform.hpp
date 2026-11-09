/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file vdc_platform.hpp
 *    @brief Platform boundary of the vehicle dynamics controller (VDC) application.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include <span>
#include "cyphal/can_media.hpp"

namespace unimoc::hardware {

/// Starts the Cyphal thread of the VDC (hardware/zephyr_common/vdc_runtime.cpp).
void StartVdc() noexcept;

/// @name Implemented by each platform (hardware/zephyr_*/vdc_platform.cpp)
/// @{

/// Prepares the platform before the CAN interfaces start (transceiver pins).
void VdcPlatformInit() noexcept;

/// CAN interfaces of the platform besides the chosen `zephyr,canbus`; null entries are skipped.
[[nodiscard]] std::span<cyphal::CanMedia* const> VdcPlatformInterfaces() noexcept;

/**
 * @brief The node-ID of the VDC.
 * @param configured The node-ID of the build (CONFIG_UNIMOC_VDC_NODE_ID).
 * @return @p configured, or the one of the platform (native_sim: `--node-id`).
 */
[[nodiscard]] uint8_t VdcPlatformNodeId(uint8_t configured) noexcept;

/// @}

}  // namespace unimoc::hardware
