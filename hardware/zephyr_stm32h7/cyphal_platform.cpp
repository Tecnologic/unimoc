/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file cyphal_platform.cpp
 *    @brief Cyphal platform part of the STM32H7 platform.
 *
 *    One CAN interface (the chosen zephyr,canbus, FDCAN1) and no platform
 *    registers. An optional standby pin of the transceiver
 *    (can-standby-gpios of zephyr,user) is driven inactive before the
 *    controller starts. Not run on hardware yet.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "cyphal_platform.hpp"
#include <zephyr/sys/util.h>
#include "can_transceiver.hpp"
#include "drive_node.hpp"
#include "firmware_bank.hpp"

namespace unimoc::hardware {

void CyphalPlatformSettingsOverrides(settings::NvmSettings& /*settings*/) noexcept {}

void CyphalPlatformInit() noexcept { InitCanTransceiver(); }

std::span<cyphal::CanMedia* const> CyphalPlatformInterfaces() noexcept { return {}; }

void CyphalPlatformStart(cyphal::CyphalNode& /*node*/, const settings::NvmSettings& /*settings*/) noexcept {}

void CyphalPlatformTraceSignals(cymon::Device& /*device*/) noexcept {}

void CyphalPlatformSpin(cyphal::CyphalNode& node, const uint64_t now_us) noexcept {
  static_cast<void>(node);
  static_cast<void>(now_us);
  // The image that MCUboot swapped in runs on trial until it has been on the bus for a while.
  // (firmware_bank.cpp is part of the build only with CONFIG_UNIMOC_FIRMWARE_UPDATE_FLASH.)
  if constexpr (IS_ENABLED(CONFIG_UNIMOC_FIRMWARE_UPDATE_FLASH)) {
    if (ConfirmRunningImage(now_us)) {
      (void)node.Log(cyphal::Severity::kNotice, "update: the new image is confirmed");
    }
  }
}

uint64_t CyphalPlatformDriveTimeUs(const uint64_t now_us) noexcept { return now_us; }

void CyphalPlatformSettingsApplied(const settings::NvmSettings& /*settings*/) noexcept {}

// The excitation module is on the bus. The update bank is slot 1 of MCUboot; a build without the bootloader
// has none and refuses updates.
void CyphalPlatformDriveHooks(drive::DriveHooks& hooks) noexcept {
  if constexpr (IS_ENABLED(CONFIG_UNIMOC_FIRMWARE_UPDATE_FLASH)) {
    hooks.firmware_bank = &UpdateBank();
  }
}

void CyphalPlatformReady(cyphal::CyphalNode& /*node*/, const cyphal::NodeInfo& /*info*/) noexcept {}

}  // namespace unimoc::hardware
