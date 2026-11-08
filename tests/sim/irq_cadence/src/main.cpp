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
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <cstdint>
#include "hardware_interface.hpp"
#include "hot_path_interface.hpp"
#include "hot_path_timing.hpp"
#include "power_stage_interface.hpp"
#include "sim_control.hpp"

namespace {

using unimoc::hardware::kMotors;
constexpr auto kValleys = unimoc::hardware::ControlSampling::kValleys;
constexpr auto kEveryHalfPeriod = unimoc::hardware::ControlSampling::kEveryHalfPeriod;
using namespace unimoc::unit;
namespace sim = unimoc::hardware::sim;

volatile uint32_t g_sample_count = 0U;
volatile uint32_t g_control_count = 0U;  // control cycles: every second sample, one per sim cycle
volatile uint32_t g_observer_count = 0U;
volatile uint32_t g_misordered = 0U;  // observer callbacks not directly after the end of a control cycle

// Samples from this one on read the sample (hot_path::GetSample()) and count its trips.
volatile uint32_t g_read_from = UINT32_MAX;
volatile uint32_t g_overcurrent_reports = 0U;

// Duties written by the control step of sample n: phase A 0.1 + 0.01 * n.
Ratio StepDuty(const uint32_t sample) { return 0.1_ratio + (0.01_ratio * static_cast<float>(sample)); }

}  // namespace

// The platform calls these directly (hot_path_interface.hpp): once per PWM period, after the valley.
bool unimoc::hardware::SampleHandler() noexcept {
  if (g_sample_count >= g_read_from && unimoc::hardware::hot_path::GetSample().overcurrent) {
    g_overcurrent_reports = g_overcurrent_reports + 1U;
  }
  unimoc::hardware::hot_path::SetDuties({StepDuty(g_sample_count), 0.5_ratio, 0.5_ratio});
  g_sample_count = g_sample_count + 1U;
  if (g_sample_count % 2U != 0U) {
    return false;
  }
  g_control_count = g_control_count + 1U;
  return true;
}

void unimoc::hardware::ObserverHandler() noexcept {
  if (g_observer_count + 1U != g_control_count) {
    g_misordered = g_misordered + 1U;
  }
  g_observer_count = g_observer_count + 1U;
}

namespace {

void Reset() {
  g_sample_count = 0U;
  g_control_count = 0U;
  g_observer_count = 0U;
  g_misordered = 0U;
  g_read_from = UINT32_MAX;
  g_overcurrent_reports = 0U;
}

}  // namespace

ZTEST(sim_irq_cadence, test_rejects_zero_frequency) {
  zassert_false(kMotors[0].Initialize(0.0_Hz, kValleys, 0U));
}

ZTEST(sim_irq_cadence, test_one_sample_per_pwm_period_and_an_observer_per_cycle) {
  Reset();
  zassert_true(kMotors[0].Initialize(unimoc::unit::Frequency{32'000.0F}, kValleys, 0U));
  k_sleep(K_MSEC(100));

  const unsigned int kIrqKey = irq_lock();
  const uint32_t kSamples = g_sample_count;
  const uint32_t kControl = g_control_count;
  const uint32_t kObserver = g_observer_count;
  const uint32_t kMisordered = g_misordered;
  irq_unlock(kIrqKey);
  sim::SetFreeRunning(false);

  // 100 ms at 32 kHz PWM: 3200 valleys (samples), 1600 sim cycles of two periods.
  zassert_equal(kSamples, 2U * kControl, "samples=%u control=%u", kSamples, kControl);
  zassert_between_inclusive(kControl, 1599U, 1601U, "control=%u", kControl);
  zassert_equal(kObserver, kControl, "control=%u observer=%u", kControl, kObserver);
  zassert_equal(kMisordered, 0U);
}

ZTEST(sim_irq_cadence, test_duties_of_a_step_take_effect_at_the_next_peak) {
  Reset();
  zassert_true(kMotors[0].Initialize(unimoc::unit::Frequency{32'000.0F}, kValleys, 0U));
  sim::SetFreeRunning(false);

  // A sim cycle starts at a valley: half periods 0 and 2 count up to a peak,
  // 1 and 3 down to a valley. Sample n is taken at the valley that ends half
  // period 2n + 1; its duties take effect at the next peak, so half periods
  // 2n + 3 and 2n + 4 run them: the pulse about every valley is symmetric.
  sim::StepCycle();
  auto applied = sim::AppliedDuties();
  for (uint8_t k = 0U; k < 3U; ++k) {
    zassert_within(applied[k].a, 0.5_ratio, 1e-6_ratio);
  }
  zassert_within(applied[3].a, StepDuty(0U), 1e-6_ratio);

  sim::StepCycle();
  applied = sim::AppliedDuties();
  zassert_within(applied[0].a, StepDuty(0U), 1e-6_ratio);
  zassert_within(applied[1].a, StepDuty(1U), 1e-6_ratio);
  zassert_within(applied[2].a, StepDuty(1U), 1e-6_ratio);
  zassert_within(applied[3].a, StepDuty(2U), 1e-6_ratio);
  zassert_equal(sim::CycleCount(), 2U);
  zassert_equal(g_sample_count, 4U);
}

ZTEST(sim_irq_cadence, test_every_half_period_takes_every_sample_and_updates_at_every_boundary) {
  Reset();
  zassert_true(kMotors[0].Initialize(32.0_kHz, kEveryHalfPeriod, 0U));
  sim::SetFreeRunning(false);

  // Sample n is taken at boundary n, the end of half period n. Its control
  // step runs after that boundary, so its duties take effect at boundary
  // n + 1: half period n + 2 runs them. The first two run neutral duties.
  sim::StepCycle();
  auto applied = sim::AppliedDuties();
  zassert_within(applied[0].a, 0.5_ratio, 1e-6_ratio);
  zassert_within(applied[1].a, 0.5_ratio, 1e-6_ratio);
  for (uint8_t k = 2U; k < 4U; ++k) {
    zassert_within(applied[k].a, StepDuty(k - 2U), 1e-6_ratio);
  }

  sim::StepCycle();
  applied = sim::AppliedDuties();
  for (uint8_t k = 0U; k < 4U; ++k) {
    zassert_within(applied[k].a, StepDuty(k + 2U), 1e-6_ratio);
  }
  zassert_equal(sim::CycleCount(), 2U);
  zassert_equal(g_sample_count, 8U);
}

ZTEST(sim_irq_cadence, test_trips_of_unread_samples_are_reported_once_with_the_next_read) {
  using unimoc::hardware::kPowerStage;
  Reset();
  sim::SetExternalEnable(true);
  kPowerStage.ClearFault();
  zassert_true(kMotors[0].Initialize(32.0_kHz, kValleys, 0U));
  sim::SetFreeRunning(false);
  kMotors[0].SetOvercurrentLimit(10.0_A);

  // The first cycle trips at every sample, but its two valleys are not read
  // (as between two control steps); the third sample reads them, the fourth
  // reads a clean sample.
  g_read_from = 2U;
  sim::SetPhaseCurrentOverride(20.0F, -10.0F, -10.0F);
  sim::StepCycle();
  sim::ClearPhaseCurrentOverride();
  zassert_equal(g_overcurrent_reports, 0U);
  sim::StepCycle();
  zassert_equal(g_overcurrent_reports, 1U, "reported with the next sample read, once");

  kMotors[0].SetOvercurrentLimit(1000.0_A);  // beyond any test current
  kPowerStage.ClearFault();
}

ZTEST(sim_irq_cadence, test_stop_halts_the_cycle_source_until_the_next_initialize) {
  Reset();
  kMotors[0].Stop();  // nothing running yet: no effect
  zassert_true(kMotors[0].Initialize(unimoc::unit::Frequency{32'000.0F}, kValleys, 0U));
  k_sleep(K_MSEC(10));
  kMotors[0].Stop();
  const uint32_t kStopped = g_control_count;
  zassert_true(kStopped > 0U);
  k_sleep(K_MSEC(20));
  zassert_equal(g_control_count, kStopped, "no control cycle after Stop()");
  zassert_equal(g_observer_count, kStopped, "no observer after Stop()");

  // A restart with another PWM frequency runs from cycle 0 again.
  zassert_true(kMotors[0].Initialize(unimoc::unit::Frequency{16'000.0F}, kValleys, 0U));
  sim::SetFreeRunning(false);
  sim::StepCycle();
  zassert_equal(sim::CycleCount(), 1U);
  zassert_equal(g_control_count, kStopped + 1U);
  zassert_equal(g_sample_count, 2U * (kStopped + 1U));
}

ZTEST(sim_irq_cadence, test_timing_probe_keeps_min_mean_max_and_resets_at_the_next_record) {
  unimoc::hardware::timing::Probe probe{};
  zassert_equal(probe.Read().count, 0U);
  zassert_equal(probe.Read().MeanCycles(), 0U);
  probe.Record(30U);
  probe.Record(10U);
  probe.Record(20U);
  auto stats = probe.Read();
  zassert_equal(stats.min_cycles, 10U);
  zassert_equal(stats.max_cycles, 30U);
  zassert_equal(stats.MeanCycles(), 20U);
  zassert_equal(stats.count, 3U);

  probe.Reset();
  zassert_equal(probe.Read().count, 3U, "the writer carries out the reset");
  probe.Record(5U);
  stats = probe.Read();
  zassert_equal(stats.count, 1U);
  zassert_equal(stats.min_cycles, 5U);
  zassert_equal(stats.max_cycles, 5U);
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
