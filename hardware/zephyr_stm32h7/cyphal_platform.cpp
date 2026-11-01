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
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(unimoc_cyphal, CONFIG_UNIMOC_LOG_LEVEL);

namespace unimoc::hardware {

namespace {

const struct gpio_dt_spec kCanStandby = GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), can_standby_gpios, {});

}  // namespace

void CyphalPlatformSettingsOverrides(settings::NvmSettings& /*settings*/) noexcept {}

void CyphalPlatformInit() noexcept {
  if (kCanStandby.port == nullptr) {
    return;  // transceiver without a standby pin
  }
  if (!gpio_is_ready_dt(&kCanStandby) || gpio_pin_configure_dt(&kCanStandby, GPIO_OUTPUT_INACTIVE) != 0) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays):
    // the arrays are inside the Zephyr LOG macro.
    LOG_ERR("CAN transceiver standby pin not configured");
    // NOLINTEND(cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays)
  }
}

std::span<cyphal::CanMedia* const> CyphalPlatformInterfaces() noexcept { return {}; }

void CyphalPlatformStart(cyphal::CyphalNode& /*node*/, const settings::NvmSettings& /*settings*/) noexcept {}

void CyphalPlatformTraceSignals(cymon::Device& /*device*/) noexcept {}

void CyphalPlatformSpin(cyphal::CyphalNode& /*node*/, uint64_t /*now_us*/) noexcept {}

uint64_t CyphalPlatformDriveTimeUs(const uint64_t now_us) noexcept { return now_us; }

void CyphalPlatformSettingsApplied(const settings::NvmSettings& /*settings*/) noexcept {}

void CyphalPlatformReady(const cyphal::CyphalNode& /*node*/, const cyphal::NodeInfo& /*info*/) noexcept {}

}  // namespace unimoc::hardware
