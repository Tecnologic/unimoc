/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file status_leds.cpp
 *    @brief NUCLEO-H743ZI user LEDs: heartbeat of the bring-up build.
 *
 *    Green LD1 and yellow LD2 flash in antiphase at 1 Hz from a low-priority
 *    thread: a visible sign that the kernel runs. Not part of the drive.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

namespace unimoc::hardware {

namespace {

constexpr int kBlinkPeriodMs = 500;
constexpr int kThreadStackSize = 512;

const gpio_dt_spec kGreenLed = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
const gpio_dt_spec kYellowLed = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

// Green LD1 and yellow LD2 flash in antiphase at 1 Hz while the scheduler runs.
void BlinkThread(void* /*p1*/, void* /*p2*/, void* /*p3*/) {
  if (!gpio_is_ready_dt(&kGreenLed) || !gpio_is_ready_dt(&kYellowLed) ||
      gpio_pin_configure_dt(&kGreenLed, GPIO_OUTPUT_ACTIVE) != 0 ||
      gpio_pin_configure_dt(&kYellowLed, GPIO_OUTPUT_INACTIVE) != 0) {
    return;
  }
  for (;;) {
    k_msleep(kBlinkPeriodMs);
    gpio_pin_toggle_dt(&kGreenLed);
    gpio_pin_toggle_dt(&kYellowLed);
  }
}

}  // namespace

}  // namespace unimoc::hardware

// NOLINTNEXTLINE(readability-identifier-naming,cppcoreguidelines-avoid-non-const-global-variables): Zephyr
// macro defines the thread objects.
K_THREAD_DEFINE(unimoc_status_leds,
                unimoc::hardware::kThreadStackSize,
                unimoc::hardware::BlinkThread,
                nullptr,
                nullptr,
                nullptr,
                K_LOWEST_APPLICATION_THREAD_PRIO,
                0,
                0);
