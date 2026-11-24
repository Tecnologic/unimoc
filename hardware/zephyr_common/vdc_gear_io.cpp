/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file vdc_gear_io.cpp
 *    @brief Gear buttons and their lamps on the GPIOs of the VDC.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "vdc_gear_io.hpp"
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <array>
#include <cstddef>

namespace unimoc::hardware {

namespace {

// NOLINTBEGIN(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr): the devicetree macros of
// GPIO_DT_SPEC_GET_BY_IDX_OR take the device address with a C-style cast.

const std::array<gpio_dt_spec, vehicle::kGearButtonCount> kButtons{{
    GPIO_DT_SPEC_GET_BY_IDX_OR(DT_PATH(zephyr_user), gear_button_gpios, 0, {}),
    GPIO_DT_SPEC_GET_BY_IDX_OR(DT_PATH(zephyr_user), gear_button_gpios, 1, {}),
    GPIO_DT_SPEC_GET_BY_IDX_OR(DT_PATH(zephyr_user), gear_button_gpios, 2, {}),
    GPIO_DT_SPEC_GET_BY_IDX_OR(DT_PATH(zephyr_user), gear_button_gpios, 3, {}),
    GPIO_DT_SPEC_GET_BY_IDX_OR(DT_PATH(zephyr_user), gear_button_gpios, 4, {}),
}};

const std::array<gpio_dt_spec, vehicle::kGearButtonCount> kLamps{{
    GPIO_DT_SPEC_GET_BY_IDX_OR(DT_PATH(zephyr_user), gear_lamp_gpios, 0, {}),
    GPIO_DT_SPEC_GET_BY_IDX_OR(DT_PATH(zephyr_user), gear_lamp_gpios, 1, {}),
    GPIO_DT_SPEC_GET_BY_IDX_OR(DT_PATH(zephyr_user), gear_lamp_gpios, 2, {}),
    GPIO_DT_SPEC_GET_BY_IDX_OR(DT_PATH(zephyr_user), gear_lamp_gpios, 3, {}),
    GPIO_DT_SPEC_GET_BY_IDX_OR(DT_PATH(zephyr_user), gear_lamp_gpios, 4, {}),
}};
// NOLINTEND(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)

std::array<bool, vehicle::kGearButtonCount> g_button_ok{};
std::array<bool, vehicle::kGearButtonCount> g_lamp_ok{};

}  // namespace

bool InitGearIo() noexcept {
  bool all_ok = true;
  for (std::size_t i = 0U; i < vehicle::kGearButtonCount; ++i) {
    if (kButtons[i].port != nullptr) {
      g_button_ok[i] = gpio_is_ready_dt(&kButtons[i]) && gpio_pin_configure_dt(&kButtons[i], GPIO_INPUT) == 0;
      all_ok = all_ok && g_button_ok[i];
    }
    if (kLamps[i].port != nullptr) {
      g_lamp_ok[i] =
          gpio_is_ready_dt(&kLamps[i]) && gpio_pin_configure_dt(&kLamps[i], GPIO_OUTPUT_INACTIVE) == 0;
      all_ok = all_ok && g_lamp_ok[i];
    }
  }
  return all_ok;
}

vehicle::GearButtonStates ReadGearButtons() noexcept {
  vehicle::GearButtonStates states{};
  for (std::size_t i = 0U; i < vehicle::kGearButtonCount; ++i) {
    states[i] = g_button_ok[i] && gpio_pin_get_dt(&kButtons[i]) > 0;
  }
  return states;
}

void WriteGearLamps(const vehicle::PerGearButton<bool>& lamps) noexcept {
  for (std::size_t i = 0U; i < vehicle::kGearButtonCount; ++i) {
    if (g_lamp_ok[i]) {
      (void)gpio_pin_set_dt(&kLamps[i], lamps[i] ? 1 : 0);
    }
  }
}

}  // namespace unimoc::hardware
