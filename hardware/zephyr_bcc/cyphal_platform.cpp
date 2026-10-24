/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file cyphal_platform.cpp
 *    @brief Cyphal platform part of the battery case controller.
 *
 *    The BCC has one CAN interface (FDCAN3, the chosen zephyr,canbus) and
 *    no platform registers. The transceiver is an SN65HVD230: its Rs pin
 *    (can-slope-gpios of zephyr,user) is driven low for high-speed mode
 *    before the controller starts. Not run on the board yet.
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

const struct gpio_dt_spec kCanSlope = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), can_slope_gpios);

}  // namespace

uint8_t CyphalPlatformNodeIdOverride() noexcept { return cyphal::kNodeIdUnset; }

void CyphalPlatformInit() noexcept {
  // Rs low: high-speed mode (high would be standby, the transceiver only listens).
  if (!gpio_is_ready_dt(&kCanSlope) || gpio_pin_configure_dt(&kCanSlope, GPIO_OUTPUT_INACTIVE) != 0) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays):
    // the arrays are inside the Zephyr LOG macro.
    LOG_ERR("CAN transceiver Rs pin not configured");
    // NOLINTEND(cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays)
  }
}

std::span<cyphal::CanMedia* const> CyphalPlatformInterfaces() noexcept { return {}; }

void CyphalPlatformStart(cyphal::CyphalNode& /*node*/, const settings::NvmSettings& /*settings*/) noexcept {}

void CyphalPlatformTraceSignals(cymon::Device& /*device*/) noexcept {}

void CyphalPlatformSpin(cyphal::CyphalNode& /*node*/, uint64_t /*now_us*/) noexcept {}

void CyphalPlatformSettingsApplied(const settings::NvmSettings& /*settings*/) noexcept {}

}  // namespace unimoc::hardware
