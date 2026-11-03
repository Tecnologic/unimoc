/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file control_timing_test.cpp
 *    @brief PWM frequency suggestion, control divider and observer rate of the settings.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "control_timing.hpp"
#include <zephyr/ztest.h>
#include "settings_store.hpp"
#include "unimoc_test.hpp"

namespace unimoc::settings::test {

using namespace unimoc::unit;

namespace {

// A frequency in kHz as the plain Frequency the settings hold.
constexpr Frequency Khz(const float khz) { return 1.0_kHz * khz; }

// 48 V drive (55 V highest operating voltage), 50 A motor: 15 A peak-to-peak allowed.
NvmSettings Motor(const Inductance inductance) {
  NvmSettings settings{};
  settings.motor_i_max = 50.0_A;
  settings.dc_link_regen_derate_start = 55.0_V;
  settings.dc_link_trip = 58.0_V;
  settings.l_d = inductance;
  settings.l_q = inductance;
  return settings;
}

HardwareCapabilities Board(const Frequency max_pwm, const Frequency max_control) {
  HardwareCapabilities limits{};
  limits.max_pwm_frequency = max_pwm;
  limits.max_control_frequency = max_control;
  return limits;
}

}  // namespace

ZTEST(control_timing, test_ripple_follows_the_inductance_and_the_frequency) {
  // 0.15 * 48 V / (10 uH * 40 kHz) = 18 A peak to peak.
  UNIMOC_EXPECT_NEAR(PwmRipple(48.0_V, 10e-6_H, Khz(40.0F)), 18.0_A, 1e-3_A);
  UNIMOC_EXPECT_NEAR(PwmRipple(48.0_V, 20e-6_H, Khz(40.0F)), 9.0_A, 1e-3_A);
  UNIMOC_EXPECT_NEAR(PwmRipple(48.0_V, 10e-6_H, Khz(20.0F)), 36.0_A, 1e-3_A);

  // The smaller of the two inductances sets the ripple.
  NvmSettings settings = Motor(40e-6_H);
  settings.l_d = 20e-6_H;
  UNIMOC_EXPECT_NEAR(PwmRipple(settings, Khz(40.0F)), PwmRipple(55.0_V, 20e-6_H, Khz(40.0F)), 1e-3_A);
}

ZTEST(control_timing, test_suggestion_is_the_lowest_frequency_within_the_ripple_target) {
  const HardwareCapabilities kBoard = Board(Khz(64.0F), Khz(64.0F));
  // 30 uH: 0.15 * 55 V / (30 uH * 15 A) = 18.3 kHz needed: the 20 kHz floor.
  zexpect_equal(SuggestedPwmFrequency(Motor(30e-6_H), kBoard), Khz(20.0F));
  // 20 uH: 27.5 kHz needed: 28 kHz.
  zexpect_equal(SuggestedPwmFrequency(Motor(20e-6_H), kBoard), Khz(28.0F));
  // 10 uH: 55 kHz needed: 64 kHz.
  zexpect_equal(SuggestedPwmFrequency(Motor(10e-6_H), kBoard), Khz(64.0F));
  // A hub motor never goes below the audible floor.
  zexpect_equal(SuggestedPwmFrequency(Motor(350e-6_H), kBoard), Khz(20.0F));
  // 5 uH needs 110 kHz: the board's maximum, with the ripple above the target.
  const NvmSettings kTiny = Motor(5e-6_H);
  zexpect_equal(SuggestedPwmFrequency(kTiny, kBoard), Khz(64.0F));
  zexpect_true(PwmRipple(kTiny, Khz(64.0F)) > kTiny.motor_i_max * kPwmRippleTarget);
  // A board limited to 40 kHz.
  zexpect_equal(SuggestedPwmFrequency(Motor(10e-6_H), Board(Khz(40.0F), Khz(40.0F))), Khz(40.0F));
}

ZTEST(control_timing, test_automatic_divider_is_the_smallest_the_board_runs) {
  NvmSettings settings = Motor(100e-6_H);
  settings.pwm_frequency = Khz(32.0F);
  settings.control_divider = 0U;

  // 32 kHz PWM on a 64 kHz processor: a step every half period, the valley sampling not needed.
  ControlTiming timing = ResolveControlTiming(settings, Board(Khz(64.0F), Khz(64.0F)));
  zexpect_equal(timing.divider, 1U);
  zexpect_true(timing.EveryHalfPeriod());
  zexpect_equal(timing.CallsPerStep(), 1U);
  zexpect_equal(timing.steps_per_cycle, 4U);  // 16 kHz observer
  UNIMOC_EXPECT_FLOAT_EQ(timing.StepFrequency(), Khz(64.0F));
  UNIMOC_EXPECT_FLOAT_EQ(timing.StepPeriod(), 15.625e-6_s);
  UNIMOC_EXPECT_FLOAT_EQ(timing.OutputDelaySteps(), 1.5F);

  // 64 kHz PWM: a half period step would be 128 kHz, so a step every period at the valley.
  settings.pwm_frequency = Khz(64.0F);
  timing = ResolveControlTiming(settings, Board(Khz(64.0F), Khz(64.0F)));
  zexpect_equal(timing.divider, 2U);
  zexpect_false(timing.EveryHalfPeriod());
  zexpect_equal(timing.CallsPerStep(), 1U);
  zexpect_equal(timing.steps_per_cycle, 4U);
  UNIMOC_EXPECT_FLOAT_EQ(timing.StepFrequency(), Khz(64.0F));
  UNIMOC_EXPECT_FLOAT_EQ(timing.OutputDelaySteps(), 1.0F);

  // A processor that sustains 20 kHz control steps: every fourth period.
  timing = ResolveControlTiming(settings, Board(Khz(64.0F), Khz(20.0F)));
  zexpect_equal(timing.divider, 8U);
  zexpect_equal(timing.CallsPerStep(), 4U);
  zexpect_equal(timing.steps_per_cycle, 1U);
  UNIMOC_EXPECT_FLOAT_EQ(timing.StepFrequency(), Khz(16.0F));
  UNIMOC_EXPECT_FLOAT_EQ(timing.OutputDelaySteps(), 0.625F);  // T_h + 4 T_h = 5 T_h = 0.625 steps

  // Not even every fourth period: not runnable.
  zexpect_false(ResolveControlTiming(settings, Board(Khz(64.0F), Khz(10.0F))).Valid());
}

ZTEST(control_timing, test_explicit_divider_and_observer_rate) {
  NvmSettings settings = Motor(100e-6_H);
  settings.pwm_frequency = Khz(40.0F);
  settings.control_divider = 4U;  // every second period
  const ControlTiming kTiming = ResolveControlTiming(settings, Board(Khz(64.0F), Khz(64.0F)));
  zexpect_equal(kTiming.divider, 4U);
  zexpect_equal(kTiming.CallsPerStep(), 2U);
  zexpect_equal(kTiming.steps_per_cycle, 2U);  // 20 kHz steps, 10 kHz observer
  UNIMOC_EXPECT_FLOAT_EQ(kTiming.StepPeriod(), 50e-6_s);
  UNIMOC_EXPECT_FLOAT_EQ(kTiming.CyclePeriod(), 100e-6_s);

  // An explicit divider the processor cannot run is refused, not changed.
  zexpect_false(ResolveControlTiming(settings, Board(Khz(64.0F), Khz(16.0F))).Valid());
  settings.control_divider = 3U;
  zexpect_false(ResolveControlTiming(settings, Board(Khz(64.0F), Khz(64.0F))).Valid());
}

ZTEST(control_timing, test_automatic_pwm_frequency_resolves_from_the_inductance) {
  NvmSettings settings = Motor(20e-6_H);
  settings.pwm_frequency = Frequency{};
  const ControlTiming kTiming = ResolveControlTiming(settings, Board(Khz(64.0F), Khz(64.0F)));
  zexpect_true(kTiming.Valid());
  zexpect_equal(kTiming.pwm_frequency, Khz(28.0F));
  zexpect_equal(kTiming.divider, 1U);
}

ZTEST(control_timing, test_frequency_above_the_board_or_off_the_list_is_refused) {
  NvmSettings settings = Motor(100e-6_H);
  settings.pwm_frequency = Khz(64.0F);
  zexpect_false(ResolveControlTiming(settings, Board(Khz(40.0F), Khz(64.0F))).Valid());
  settings.pwm_frequency = Khz(30.0F);
  zexpect_false(ResolveControlTiming(settings, Board(Khz(64.0F), Khz(64.0F))).Valid());
}

ZTEST(control_timing, test_validation_refuses_what_the_board_cannot_run) {
  const HardwareCapabilities kBoard = Board(Khz(40.0F), Khz(40.0F));
  NvmSettings settings{};
  settings.pwm_frequency = Khz(40.0F);
  zexpect_equal(settings_store_internal::Validate(settings, kBoard), SettingsStatus::kSuccess);
  settings.pwm_frequency = Khz(48.0F);
  zexpect_equal(settings_store_internal::Validate(settings, kBoard), SettingsStatus::kOutOfRange);
  settings.pwm_frequency = Khz(40.0F);
  settings.control_divider = 3U;
  zexpect_equal(settings_store_internal::Validate(settings, kBoard), SettingsStatus::kInvalidSettings);
  settings.control_divider = 4U;
  zexpect_equal(settings_store_internal::Validate(settings, kBoard), SettingsStatus::kSuccess);
}

ZTEST_SUITE(control_timing, nullptr, nullptr, nullptr, nullptr, nullptr);

}  // namespace unimoc::settings::test
