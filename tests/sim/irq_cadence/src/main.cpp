/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file main.cpp
 *    @brief Cycle cadence, duty-plan timing and power stage of the native_sim platform.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include "hardware_interface.hpp"
#include "hot_path_interface.hpp"
#include "power_stage_interface.hpp"
#include "sim_control.hpp"

namespace {

using unimoc::hardware::DutyPlan;
using unimoc::hardware::kMotors;
using unimoc::unit::Ratio;
namespace sim = unimoc::hardware::sim;

volatile uint32_t g_control_count = 0U;
volatile uint32_t g_observer_count = 0U;
volatile uint32_t g_misordered = 0U;  // observer callbacks not directly after a control callback

// Plan of cycle n: phase A duty of entry k is 0.1 * (k + 1) + 0.01 * n.
float PlanDuty(const uint32_t cycle, const uint8_t entry) {
  return (0.1F * static_cast<float>(entry + 1U)) + (0.01F * static_cast<float>(cycle));
}

}  // namespace

// The platform calls these directly (hot_path_interface.hpp).
void unimoc::hardware::ControlCycleHandler() noexcept {
  DutyPlan plan{};
  for (uint8_t k = 0U; k < unimoc::hardware::kHalfPeriodsPerCycle; ++k) {
    plan.duties[k].a = Ratio{PlanDuty(g_control_count, k)};
  }
  unimoc::hardware::hot_path::SetDutyPlan(plan);
  g_control_count = g_control_count + 1U;
}

void unimoc::hardware::ObserverHandler() noexcept {
  if (g_observer_count + 1U != g_control_count) {
    g_misordered = g_misordered + 1U;
  }
  g_observer_count = g_observer_count + 1U;
}

namespace {

void Reset() {
  g_control_count = 0U;
  g_observer_count = 0U;
  g_misordered = 0U;
}

}  // namespace

ZTEST(sim_irq_cadence, test_rejects_zero_frequency) {
  zassert_false(kMotors[0].Initialize(unimoc::unit::Frequency{0.0F}, 0U));
}

ZTEST(sim_irq_cadence, test_one_control_cycle_per_four_half_periods) {
  Reset();
  zassert_true(kMotors[0].Initialize(unimoc::unit::Frequency{32'000.0F}, 0U));
  k_sleep(K_MSEC(100));

  const unsigned int kIrqKey = irq_lock();
  const uint32_t kControl = g_control_count;
  const uint32_t kObserver = g_observer_count;
  const uint32_t kMisordered = g_misordered;
  irq_unlock(kIrqKey);
  sim::SetFreeRunning(false);

  // 100 ms at 32 kHz PWM: 64000 half periods, 16000 control cycles per second.
  zassert_between_inclusive(kControl, 1599U, 1601U, "control=%u", kControl);
  zassert_equal(kObserver, kControl, "control=%u observer=%u", kControl, kObserver);
  zassert_equal(kMisordered, 0U);
}

ZTEST(sim_irq_cadence, test_plan_entries_reach_the_following_half_periods) {
  Reset();
  zassert_true(kMotors[0].Initialize(unimoc::unit::Frequency{32'000.0F}, 0U));
  sim::SetFreeRunning(false);

  sim::StepCycle();  // control writes plan 0
  auto applied = sim::AppliedDuties();
  // The half period right after the control boundary still runs the old
  // plan (neutral here); entries 0..2 follow.
  zassert_within(applied[0].a.Value(), 0.5F, 1e-6F);
  for (uint8_t k = 1U; k < 4U; ++k) {
    zassert_within(applied[k].a.Value(), PlanDuty(0U, k - 1U), 1e-6F);
  }

  sim::StepCycle();  // plan 1; entry 3 of plan 0 carries over
  applied = sim::AppliedDuties();
  zassert_within(applied[0].a.Value(), PlanDuty(0U, 3U), 1e-6F);
  for (uint8_t k = 1U; k < 4U; ++k) {
    zassert_within(applied[k].a.Value(), PlanDuty(1U, k - 1U), 1e-6F);
  }
  zassert_equal(sim::CycleCount(), 2U);
}

ZTEST(sim_power_stage, test_enable_needs_external_enable) {
  using unimoc::hardware::kPowerStage;
  using unimoc::hardware::PowerStageState;
  sim::SetExternalEnable(false);
  kPowerStage.ClearFault();
  zassert_equal(kPowerStage.GetState(), PowerStageState::kBlocked);
  zassert_false(kPowerStage.Enable());

  sim::SetExternalEnable(true);
  zassert_equal(kPowerStage.GetState(),
                PowerStageState::kFault,
                "break from the removed enable stays latched");
  zassert_false(kPowerStage.Enable());
  kPowerStage.ClearFault();
  zassert_equal(kPowerStage.GetState(), PowerStageState::kDisabled);
  zassert_true(kPowerStage.Enable());
  zassert_equal(kPowerStage.GetState(), PowerStageState::kEnabled);
}

ZTEST(sim_power_stage, test_trip_latches_until_cleared) {
  using unimoc::hardware::kPowerStage;
  using unimoc::hardware::PowerStageState;
  sim::SetExternalEnable(true);
  kPowerStage.ClearFault();
  zassert_true(kPowerStage.Enable());

  kPowerStage.Trip();
  zassert_equal(kPowerStage.GetState(), PowerStageState::kFault);
  zassert_false(kPowerStage.Enable());
  kPowerStage.ClearFault();
  zassert_true(kPowerStage.Enable());

  kPowerStage.Disable();
  zassert_equal(kPowerStage.GetState(), PowerStageState::kDisabled);
}

ZTEST(sim_power_stage, test_enable_removed_while_running_breaks) {
  using unimoc::hardware::kPowerStage;
  using unimoc::hardware::PowerStageState;
  sim::SetExternalEnable(true);
  kPowerStage.ClearFault();
  zassert_true(kPowerStage.Enable());
  sim::SetExternalEnable(false);
  zassert_equal(kPowerStage.GetState(), PowerStageState::kBlocked);
  sim::SetExternalEnable(true);
  zassert_equal(kPowerStage.GetState(), PowerStageState::kFault);
}

ZTEST_SUITE(sim_power_stage, nullptr, nullptr, nullptr, nullptr, nullptr);

ZTEST_SUITE(sim_irq_cadence, nullptr, nullptr, nullptr, nullptr, nullptr);
