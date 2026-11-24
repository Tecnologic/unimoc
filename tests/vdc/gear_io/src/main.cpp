/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file main.cpp
 *    @brief ztest suite of the gear buttons and lamps of the VDC on gpio_emul.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <zephyr/device.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/ztest.h>
#include "vdc_gear_io.hpp"

namespace unimoc::hardware::test {

namespace {

constexpr gpio_pin_t kFirstButtonPin = 16U;
constexpr gpio_pin_t kFirstLampPin = 24U;

const struct device* Gpio() { return DEVICE_DT_GET(DT_NODELABEL(gpio0)); }

}  // namespace

ZTEST(vdc_gear_io, test_buttons_read_pressed_one_by_one) {
  zassert_true(InitGearIo());
  for (std::size_t i = 0U; i < vehicle::kGearButtonCount; ++i) {
    const auto kPin = static_cast<gpio_pin_t>(kFirstButtonPin + i);
    zassert_ok(gpio_emul_input_set(Gpio(), kPin, 1));
    const vehicle::GearButtonStates kStates = ReadGearButtons();
    for (std::size_t j = 0U; j < vehicle::kGearButtonCount; ++j) {
      zexpect_equal(kStates[j], i == j);
    }
    zassert_ok(gpio_emul_input_set(Gpio(), kPin, 0));
  }
}

ZTEST(vdc_gear_io, test_lamps_follow_the_request) {
  zassert_true(InitGearIo());
  vehicle::PerGearButton<bool> lamps{};
  lamps[vehicle::GearButton(1)] = true;
  WriteGearLamps(lamps);
  for (std::size_t i = 0U; i < vehicle::kGearButtonCount; ++i) {
    const auto kPin = static_cast<gpio_pin_t>(kFirstLampPin + i);
    zexpect_equal(gpio_emul_output_get(Gpio(), kPin), (i == vehicle::GearButton(1)) ? 1 : 0);
  }
  WriteGearLamps(vehicle::PerGearButton<bool>{});
  zexpect_equal(gpio_emul_output_get(Gpio(), static_cast<gpio_pin_t>(kFirstLampPin + vehicle::GearButton(1))),
                0);
}

ZTEST_SUITE(vdc_gear_io, nullptr, nullptr, nullptr, nullptr, nullptr);

}  // namespace unimoc::hardware::test
