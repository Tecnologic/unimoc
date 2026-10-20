/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file main.cpp
 *    @brief Per-sample overcurrent trip, loop freeze and startup acknowledgement on native_sim.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <zephyr/ztest.h>
#include "current_control_isr.hpp"
#include "hardware_interface.hpp"
#include "hot_path_interface.hpp"
#include "hw_startup.hpp"
#include "power_stage_interface.hpp"
#include "settings_store.hpp"
#include "sim_control.hpp"
#include "slow_update.hpp"

namespace {

using unimoc::current_control::CurrentControlIsr;
using unimoc::current_control::SlowUpdate;
using unimoc::hardware::kPowerStage;
using unimoc::hardware::PowerStageState;
namespace sim = unimoc::hardware::sim;

constexpr float kLimitA = 50.0F;

}  // namespace

// The platform calls these directly (hot_path_interface.hpp).
void unimoc::hardware::ControlCycleHandler() noexcept { CurrentControlIsr::Instance().OnControlCycle(); }
void unimoc::hardware::ObserverHandler() noexcept { SlowUpdate::Instance().RunOnce(); }

struct sim_overcurrent_fixture {
  unimoc::settings::SettingsStore store{unimoc::hardware::kSettings.GetSettingsProfile(),
                                        unimoc::hardware::kSettings.GetSettingsStorage()};
};

namespace {

// One control cycle covers four half periods; the samples of a cycle reach
// the controller in the next one.
void RunCycles(const unsigned count) {
  for (unsigned i = 0; i < count; ++i) {
    sim::StepCycle();
  }
}

void* Setup() {
  static sim_overcurrent_fixture fixture;
  zassert_equal(fixture.store.Load(), unimoc::settings::SettingsStatus::kFactoryDefaults);
  const auto kSnapshot = fixture.store.GetSnapshot();
  const auto& settings = kSnapshot.Get();
  auto& isr = CurrentControlIsr::Instance();
  isr.Init(settings, unimoc::hardware::kMotors[0], 170'000'000U);
  SlowUpdate::Instance().Init(settings, isr);
  isr.SetOvercurrentLimit(unimoc::unit::Current{kLimitA});
  zassert_true(unimoc::hardware::kMotors[0].Initialize(settings.pwm_frequency, 0U));
  sim::SetFreeRunning(false);
  return &fixture;
}

void Before(void* /*unused*/) {
  // Flush the samples of the previous test with the stage off.
  sim::SetPhaseCurrentOverride(0.0F, 0.0F, 0.0F);
  sim::SetExternalEnable(true);
  kPowerStage.Disable();
  RunCycles(1);
  kPowerStage.ClearFault();
  CurrentControlIsr::Instance().ClearOvercurrentTrip();
}

}  // namespace

ZTEST_F(sim_overcurrent, test_below_limit_keeps_stage_enabled) {
  zassert_true(kPowerStage.Enable());
  sim::SetPhaseCurrentOverride(40.0F, -20.0F, -20.0F);
  RunCycles(3);
  zassert_equal(kPowerStage.GetState(), PowerStageState::kEnabled);
  zassert_false(CurrentControlIsr::Instance().state.overcurrent_tripped);
}

ZTEST_F(sim_overcurrent, test_sample_check_trips_at_once_and_freezes_loop) {
  zassert_true(kPowerStage.Enable());
  sim::SetPhaseCurrentOverride(0.0F, 60.0F, -60.0F);
  RunCycles(1);
  // The per-sample check of the hardware layer, before any control cycle saw it.
  zassert_equal(kPowerStage.GetState(), PowerStageState::kFault);
  zassert_false(CurrentControlIsr::Instance().state.overcurrent_tripped);

  RunCycles(1);  // the control cycle with these samples latches the trip
  zassert_true(CurrentControlIsr::Instance().state.overcurrent_tripped);
  for (const auto& duties : CurrentControlIsr::Instance().state.duty_plan.duties) {
    zassert_within(duties.a.Value(), 0.5F, 1e-6F);
    zassert_within(duties.b.Value(), 0.5F, 1e-6F);
    zassert_within(duties.c.Value(), 0.5F, 1e-6F);
  }

  // The latched fault survives the current going away.
  sim::SetPhaseCurrentOverride(0.0F, 0.0F, 0.0F);
  RunCycles(2);
  zassert_false(kPowerStage.Enable());
}

ZTEST_F(sim_overcurrent, test_startup_needs_external_enable) {
  sim::SetExternalEnable(false);
  unimoc::startup::HwStartup startup{CurrentControlIsr::Instance(), fixture->store.GetOperations()};
  startup.RequestRun();
  zassert_equal(startup.State(), unimoc::startup::FsmState::kIdle);
  zassert_not_equal(kPowerStage.GetState(), PowerStageState::kEnabled);
}

ZTEST_F(sim_overcurrent, test_startup_run_releases_stage_and_fault_cuts_it) {
  unimoc::startup::HwStartup startup{CurrentControlIsr::Instance(), fixture->store.GetOperations()};
  startup.RequestRun();
  zassert_equal(startup.State(), unimoc::startup::FsmState::kPwmDisable);
  zassert_equal(kPowerStage.GetState(), PowerStageState::kEnabled);

  sim::SetPhaseCurrentOverride(70.0F, -35.0F, -35.0F);
  RunCycles(2);  // trip at the sample, then samples_ready for the startup FSM
  startup.RunOnce();
  zassert_equal(startup.State(), unimoc::startup::FsmState::kFault);
  zassert_not_equal(kPowerStage.GetState(), PowerStageState::kEnabled);
}

ZTEST_SUITE(sim_overcurrent, nullptr, Setup, Before, nullptr, nullptr);
