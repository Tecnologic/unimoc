/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file can_transceiver.cpp
 *    @brief Standby pin of the CAN transceiver of an STM32H7 board.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "can_transceiver.hpp"
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(unimoc_cyphal, CONFIG_UNIMOC_LOG_LEVEL);

namespace unimoc::hardware {

namespace {

const struct gpio_dt_spec kCanStandby = GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), can_standby_gpios, {});

}  // namespace

void InitCanTransceiver() noexcept {
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

}  // namespace unimoc::hardware
