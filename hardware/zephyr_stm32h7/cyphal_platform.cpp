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
#include "can_transceiver.hpp"

namespace unimoc::hardware {

void CyphalPlatformSettingsOverrides(settings::NvmSettings& /*settings*/) noexcept {}

void CyphalPlatformInit() noexcept { InitCanTransceiver(); }

std::span<cyphal::CanMedia* const> CyphalPlatformInterfaces() noexcept { return {}; }

void CyphalPlatformStart(cyphal::CyphalNode& /*node*/, const settings::NvmSettings& /*settings*/) noexcept {}

void CyphalPlatformTraceSignals(cymon::Device& /*device*/) noexcept {}

void CyphalPlatformSpin(cyphal::CyphalNode& /*node*/, uint64_t /*now_us*/) noexcept {}

uint64_t CyphalPlatformDriveTimeUs(const uint64_t now_us) noexcept { return now_us; }

void CyphalPlatformSettingsApplied(const settings::NvmSettings& /*settings*/) noexcept {}

// The excitation module is on the bus; flash bank 2 waits for the bootloader.
void CyphalPlatformDriveHooks(drive::DriveHooks& /*hooks*/) noexcept {}

void CyphalPlatformReady(cyphal::CyphalNode& /*node*/, const cyphal::NodeInfo& /*info*/) noexcept {}

}  // namespace unimoc::hardware
