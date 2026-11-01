/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file status_leds.cpp
 *    @brief Zephyr thread that drives the status LEDs.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "status_leds.hpp"
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include "power_stage_interface.hpp"

namespace unimoc::hardware {

// NOLINTBEGIN(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay):
// K_THREAD_STACK_DEFINE declares a C array inside the Zephyr macro.

namespace {

constexpr int kTickMs = 100;
constexpr uint32_t kRunHalfPeriodTicks = 5;  // 1 Hz
constexpr uint32_t kWarningBlinkTicks = 1;   // 5 Hz
constexpr uint32_t kWarningHoldTicks = 20;   // two seconds
constexpr int kStatusLedsStackSize = 512;

// An alias that the board does not define leaves its LED out.
#define UNIMOC_OPTIONAL_LED(name, alias)                                                    \
  COND_CODE_1(DT_NODE_EXISTS(DT_ALIAS(alias)),                                              \
              (const gpio_dt_spec k##name##Spec = GPIO_DT_SPEC_GET(DT_ALIAS(alias), gpios); \
               const gpio_dt_spec* const k##name##Led = &k##name##Spec;),                   \
              (const gpio_dt_spec* const k##name##Led = nullptr;))

UNIMOC_OPTIONAL_LED(Run, unimoc_led_run)
UNIMOC_OPTIONAL_LED(Pwm, unimoc_led_pwm)
UNIMOC_OPTIONAL_LED(Error, unimoc_led_error)

#undef UNIMOC_OPTIONAL_LED

K_THREAD_STACK_DEFINE(g_status_leds_stack, kStatusLedsStackSize);
struct k_thread g_status_leds_thread;
OverloadCounterCallback g_overload_counter = nullptr;

bool Configure(const gpio_dt_spec* led) {
  return led == nullptr || (gpio_is_ready_dt(led) && gpio_pin_configure_dt(led, GPIO_OUTPUT_INACTIVE) == 0);
}

void Set(const gpio_dt_spec* led, bool on) {
  if (led != nullptr) {
    (void)gpio_pin_set_dt(led, on ? 1 : 0);
  }
}

void StatusLedsThread(void* /*unused*/, void* /*unused*/, void* /*unused*/) {
  if (!Configure(kRunLed) || !Configure(kPwmLed) || !Configure(kErrorLed)) {
    return;
  }
  uint32_t last_overloads = g_overload_counter();
  uint32_t warning_ticks = 0;
  for (uint32_t tick = 0;; ++tick) {
    const uint32_t kOverloads = g_overload_counter();
    if (kOverloads != last_overloads) {
      last_overloads = kOverloads;
      warning_ticks = kWarningHoldTicks;
    } else if (warning_ticks > 0) {
      --warning_ticks;
    }

    const PowerStageState kState = kPowerStage.GetState();
    Set(kRunLed, (tick / kRunHalfPeriodTicks) % 2 == 0);
    Set(kPwmLed, kState == PowerStageState::kEnabled);
    Set(kErrorLed,
        kState == PowerStageState::kFault || (warning_ticks > 0 && (tick / kWarningBlinkTicks) % 2 == 0));
    k_msleep(kTickMs);
  }
}

}  // namespace

void StartStatusLeds(OverloadCounterCallback overload_counter) noexcept {
  g_overload_counter = overload_counter;
  k_thread_create(&g_status_leds_thread,
                  g_status_leds_stack,
                  K_THREAD_STACK_SIZEOF(g_status_leds_stack),
                  StatusLedsThread,
                  nullptr,
                  nullptr,
                  nullptr,
                  K_LOWEST_APPLICATION_THREAD_PRIO,
                  0,
                  K_NO_WAIT);
  k_thread_name_set(&g_status_leds_thread, "status_leds");
}

// NOLINTEND(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay)

}  // namespace unimoc::hardware
