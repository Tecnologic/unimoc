/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file gear_buttons_test.cpp
 *    @brief ztest coverage of the gear buttons: debounce, one button at a time, stuck buttons, lamps.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "gear_buttons.hpp"
#include <zephyr/ztest.h>
#include <optional>

namespace unimoc::vehicle::test {

namespace {

constexpr GearButtonStates kNone{};

GearButtonStates Pressed(const Gear gear) {
  GearButtonStates states{};
  states[GearButton(gear)] = true;
  return states;
}

/// Feeds @p states every millisecond from @p from_us for @p duration_us; returns the last press seen.
std::optional<Gear> Hold(GearButtons& buttons,
                         const GearButtonStates& states,
                         const uint64_t from_us,
                         const uint64_t duration_us) {
  std::optional<Gear> press;
  for (uint64_t t = from_us; t <= from_us + duration_us; t += 1000U) {
    const std::optional<Gear> kPress = buttons.Update(states, t);
    if (kPress.has_value()) {
      press = kPress;
    }
  }
  return press;
}

}  // namespace

ZTEST(gear_buttons, test_button_order) {
  zexpect_equal(ButtonGear(0U), kReverse);
  zexpect_equal(ButtonGear(1U), kNeutral);
  zexpect_equal(ButtonGear(4U), kTopGear);
  zexpect_equal(GearButton(kReverse), 0U);
  zexpect_equal(GearButton(2), 3U);
}

ZTEST(gear_buttons, test_press_counts_after_the_debounce_time) {
  GearButtons buttons;
  zexpect_false(buttons.Update(Pressed(1), 0U).has_value());
  zexpect_false(buttons.Update(Pressed(1), 10000U).has_value());
  const std::optional<Gear> kPress = buttons.Update(Pressed(1), 20000U);
  zassert_true(kPress.has_value());
  zexpect_equal(*kPress, 1);
  // Held: no second press.
  zexpect_false(Hold(buttons, Pressed(1), 21000U, 500000U).has_value());
}

ZTEST(gear_buttons, test_bounce_shorter_than_the_debounce_time_is_ignored) {
  GearButtons buttons;
  zexpect_false(Hold(buttons, Pressed(2), 0U, 10000U).has_value());
  zexpect_false(Hold(buttons, kNone, 11000U, 100000U).has_value());
}

ZTEST(gear_buttons, test_two_buttons_at_once_select_nothing) {
  GearButtons buttons;
  GearButtonStates both = Pressed(1);
  both[GearButton(kReverse)] = true;
  zexpect_false(Hold(buttons, both, 0U, 100000U).has_value());
}

ZTEST(gear_buttons, test_stuck_button_is_ignored_and_reported) {
  GearButtons buttons;
  zexpect_true(Hold(buttons, Pressed(kNeutral), 0U, 50000U).has_value());
  zexpect_false(buttons.Stuck());
  (void)Hold(buttons, Pressed(kNeutral), 51000U, 2100000U);
  zexpect_true(buttons.Stuck());
  // Another button works while the stuck one is still held.
  GearButtonStates with_first = Pressed(kNeutral);
  with_first[GearButton(1)] = true;
  const std::optional<Gear> kPress = Hold(buttons, with_first, 2200000U, 50000U);
  zassert_true(kPress.has_value());
  zexpect_equal(*kPress, 1);
  // Released: no longer stuck.
  (void)Hold(buttons, kNone, 2300000U, 50000U);
  zexpect_false(buttons.Stuck());
}

ZTEST(gear_buttons, test_lamp_states) {
  const PerGearButton<LampState> kLamps = GearLampStates(kReverse, kNeutral, 2);
  zexpect_equal(kLamps[GearButton(kReverse)], LampState::kWaiting);
  zexpect_equal(kLamps[GearButton(kNeutral)], LampState::kOn);
  zexpect_equal(kLamps[GearButton(1)], LampState::kOff);
  zexpect_equal(kLamps[GearButton(2)], LampState::kRefused);
  zexpect_equal(kLamps[GearButton(3)], LampState::kOff);
}

ZTEST(gear_buttons, test_lamp_blinking) {
  zexpect_true(LampLit(LampState::kOn, 123456U));
  zexpect_false(LampLit(LampState::kOff, 0U));
  zexpect_true(LampLit(LampState::kWaiting, 100000U));
  zexpect_false(LampLit(LampState::kWaiting, 600000U));
  zexpect_true(LampLit(LampState::kRefused, 50000U));
  zexpect_false(LampLit(LampState::kRefused, 150000U));
}

ZTEST_SUITE(gear_buttons, nullptr, nullptr, nullptr, nullptr, nullptr);

}  // namespace unimoc::vehicle::test
