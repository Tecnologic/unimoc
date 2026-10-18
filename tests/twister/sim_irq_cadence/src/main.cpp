// SPDX-License-Identifier: GPL-3.0-or-later
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "hardware_interface.hpp"
#include "power_stage_interface.hpp"
#include "sim_control.hpp"

namespace {

using unimoc::hardware::kSlowUpdateDivider;
using unimoc::hardware::motor;

volatile uint32_t g_fast_count = 0U;
volatile uint32_t g_slow_count = 0U;
volatile uint32_t g_misaligned_slow = 0U;  // slow interrupts not directly after a multiple of N fast ones

void OnFast() noexcept { g_fast_count = g_fast_count + 1U; }

void OnSlow() noexcept {
  g_slow_count = g_slow_count + 1U;
  if (g_fast_count % kSlowUpdateDivider != 0U || g_fast_count / kSlowUpdateDivider != g_slow_count) {
    g_misaligned_slow = g_misaligned_slow + 1U;
  }
}

}  // namespace

ZTEST(sim_irq_cadence, test_rejects_zero_frequency) {
  zassert_false(motor[0].Initialize(unimoc::unit::Frequency{0.0F}, 0U, OnFast, OnSlow));
}

ZTEST(sim_irq_cadence, test_slow_irq_every_fourth_fast_irq) {
  zassert_true(motor[0].Initialize(unimoc::unit::Frequency{20'000.0F}, 0U, OnFast, OnSlow));
  k_sleep(K_MSEC(100));

  const unsigned int key = irq_lock();
  const uint32_t fast = g_fast_count;
  const uint32_t slow = g_slow_count;
  const uint32_t misaligned = g_misaligned_slow;
  irq_unlock(key);

  // 100 ms at 20 kHz PWM, two fast steps per period.
  zassert_between_inclusive(fast, 3999U, 4001U, "fast=%u", fast);
  zassert_equal(slow, fast / kSlowUpdateDivider, "fast=%u slow=%u", fast, slow);
  zassert_equal(misaligned, 0U);
}

ZTEST(sim_power_stage, test_enable_needs_external_enable) {
  using unimoc::hardware::PowerStageState;
  using unimoc::hardware::power_stage;
  unimoc::hardware::sim::SetExternalEnable(false);
  power_stage.ClearFault();
  zassert_equal(power_stage.GetState(), PowerStageState::kBlocked);
  zassert_false(power_stage.Enable());

  unimoc::hardware::sim::SetExternalEnable(true);
  zassert_equal(power_stage.GetState(), PowerStageState::kFault, "break from the removed enable stays latched");
  zassert_false(power_stage.Enable());
  power_stage.ClearFault();
  zassert_equal(power_stage.GetState(), PowerStageState::kDisabled);
  zassert_true(power_stage.Enable());
  zassert_equal(power_stage.GetState(), PowerStageState::kEnabled);
}

ZTEST(sim_power_stage, test_trip_latches_until_cleared) {
  using unimoc::hardware::PowerStageState;
  using unimoc::hardware::power_stage;
  unimoc::hardware::sim::SetExternalEnable(true);
  power_stage.ClearFault();
  zassert_true(power_stage.Enable());

  power_stage.Trip();
  zassert_equal(power_stage.GetState(), PowerStageState::kFault);
  zassert_false(power_stage.Enable());
  power_stage.ClearFault();
  zassert_true(power_stage.Enable());

  power_stage.Disable();
  zassert_equal(power_stage.GetState(), PowerStageState::kDisabled);
}

ZTEST(sim_power_stage, test_enable_removed_while_running_breaks) {
  using unimoc::hardware::PowerStageState;
  using unimoc::hardware::power_stage;
  unimoc::hardware::sim::SetExternalEnable(true);
  power_stage.ClearFault();
  zassert_true(power_stage.Enable());
  unimoc::hardware::sim::SetExternalEnable(false);
  zassert_equal(power_stage.GetState(), PowerStageState::kBlocked);
  unimoc::hardware::sim::SetExternalEnable(true);
  zassert_equal(power_stage.GetState(), PowerStageState::kFault);
}

ZTEST_SUITE(sim_power_stage, nullptr, nullptr, nullptr, nullptr, nullptr);

ZTEST_SUITE(sim_irq_cadence, nullptr, nullptr, nullptr, nullptr, nullptr);
