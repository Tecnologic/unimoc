/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file cyphal_platform.hpp
 *    @brief Platform boundary of the Cyphal node.
 *
 *    The Cyphal thread, the CAN interface on the chosen `zephyr,canbus` and
 *    the diagnostic log backend are shared by all Zephyr platforms
 *    (hardware/zephyr_common). A platform adds only what is specific to it
 *    (hardware/zephyr_bcc, hardware/zephyr_sim): extra CAN interfaces,
 *    registers and publications. Thread context only.
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
#include "cyphal/node.hpp"
#include "nvm_settings.hpp"
#include "settings_store.hpp"

namespace unimoc::hardware {

/**
 * @brief Starts the Cyphal thread of the drive (motor 0).
 * @param store Settings store of the firmware; must outlive the thread.
 */
void StartCyphal(settings::SettingsStore& store) noexcept;

/// @name Implemented by each platform
/// @{

/// Node-ID given on the command line (native_sim `--node-id`), applied to the RAM settings at start.
[[nodiscard]] uint8_t CyphalPlatformNodeIdOverride() noexcept;

/// Prepares the platform before the CAN interfaces start (transceiver pins).
void CyphalPlatformInit() noexcept;

/// CAN interfaces of the platform besides the chosen `zephyr,canbus`; null entries are skipped.
[[nodiscard]] std::span<cyphal::CanMedia* const> CyphalPlatformInterfaces() noexcept;

/**
 * @brief Adds platform registers and publishers to the node before it starts.
 * @param node The drive's Cyphal node.
 * @param settings Live settings at start (telemetry rate, motor parameters).
 */
void CyphalPlatformStart(cyphal::CyphalNode& node, const settings::NvmSettings& settings) noexcept;

/// Runs the platform part of the node (publications) after each spin.
void CyphalPlatformSpin(cyphal::CyphalNode& node, uint64_t now_us) noexcept;

/// Called after the drive applied new settings to the controller.
void CyphalPlatformSettingsApplied(const settings::NvmSettings& settings) noexcept;

/// @}

}  // namespace unimoc::hardware
