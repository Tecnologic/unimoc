// SPDX-License-Identifier: GPL-3.0-or-later
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
using unimoc::hardware::power_stage;
using unimoc::hardware::PowerStageState;
namespace sim = unimoc::hardware::sim;

constexpr float kLimitA = 50.0F;

}  // namespace

// The platform calls these directly (hot_path_interface.hpp).
void unimoc::hardware::ControlCycleHandler() noexcept { CurrentControlIsr::instance().on_control_cycle(); }
void unimoc::hardware::ObserverHandler() noexcept { SlowUpdate::instance().run_once(); }

struct sim_overcurrent_fixture {
  unimoc::settings::SettingsStore store{unimoc::hardware::settings.GetSettingsProfile(),
                                        unimoc::hardware::settings.GetSettingsStorage()};
};

namespace {

// One control cycle covers four half periods; the samples of a cycle reach
// the controller in the next one.
void RunCycles(const unsigned count) {
  for (unsigned i = 0; i < count; ++i) sim::StepCycle();
}

void* Setup() {
  static sim_overcurrent_fixture fixture;
  zassert_equal(fixture.store.Load(), unimoc::settings::SettingsStatus::kFactoryDefaults);
  const auto snapshot = fixture.store.GetSnapshot();
  const auto& settings = snapshot.Get();
  auto& isr = CurrentControlIsr::instance();
  isr.init(settings, unimoc::hardware::motor[0], 170'000'000U);
  SlowUpdate::instance().init(settings, isr);
  isr.set_overcurrent_limit(unimoc::unit::Current{kLimitA});
  zassert_true(unimoc::hardware::motor[0].Initialize(settings.pwm_frequency, 0U));
  sim::SetFreeRunning(false);
  return &fixture;
}

void Before(void*) {
  // Flush the samples of the previous test with the stage off.
  sim::SetPhaseCurrentOverride(0.0F, 0.0F, 0.0F);
  sim::SetExternalEnable(true);
  power_stage.Disable();
  RunCycles(1);
  power_stage.ClearFault();
  CurrentControlIsr::instance().clear_overcurrent_trip();
}

}  // namespace

ZTEST_F(sim_overcurrent, test_below_limit_keeps_stage_enabled) {
  zassert_true(power_stage.Enable());
  sim::SetPhaseCurrentOverride(40.0F, -20.0F, -20.0F);
  RunCycles(3);
  zassert_equal(power_stage.GetState(), PowerStageState::kEnabled);
  zassert_false(CurrentControlIsr::instance().state.overcurrent_tripped);
}

ZTEST_F(sim_overcurrent, test_sample_check_trips_at_once_and_freezes_loop) {
  zassert_true(power_stage.Enable());
  sim::SetPhaseCurrentOverride(0.0F, 60.0F, -60.0F);
  RunCycles(1);
  // The per-sample check of the hardware layer, before any control cycle saw it.
  zassert_equal(power_stage.GetState(), PowerStageState::kFault);
  zassert_false(CurrentControlIsr::instance().state.overcurrent_tripped);

  RunCycles(1);  // the control cycle with these samples latches the trip
  zassert_true(CurrentControlIsr::instance().state.overcurrent_tripped);
  for (const auto& duties : CurrentControlIsr::instance().state.duty_plan.duties) {
    zassert_within(duties.a.Value(), 0.5F, 1e-6F);
    zassert_within(duties.b.Value(), 0.5F, 1e-6F);
    zassert_within(duties.c.Value(), 0.5F, 1e-6F);
  }

  // The latched fault survives the current going away.
  sim::SetPhaseCurrentOverride(0.0F, 0.0F, 0.0F);
  RunCycles(2);
  zassert_false(power_stage.Enable());
}

ZTEST_F(sim_overcurrent, test_startup_needs_external_enable) {
  sim::SetExternalEnable(false);
  unimoc::startup::HwStartup startup{CurrentControlIsr::instance(), fixture->store.GetOperations()};
  startup.request_run();
  zassert_equal(startup.state(), unimoc::startup::FsmState::IDLE);
  zassert_not_equal(power_stage.GetState(), PowerStageState::kEnabled);
}

ZTEST_F(sim_overcurrent, test_startup_run_releases_stage_and_fault_cuts_it) {
  unimoc::startup::HwStartup startup{CurrentControlIsr::instance(), fixture->store.GetOperations()};
  startup.request_run();
  zassert_equal(startup.state(), unimoc::startup::FsmState::PWM_DISABLE);
  zassert_equal(power_stage.GetState(), PowerStageState::kEnabled);

  sim::SetPhaseCurrentOverride(70.0F, -35.0F, -35.0F);
  RunCycles(2);  // trip at the sample, then samples_ready for the startup FSM
  startup.run_once();
  zassert_equal(startup.state(), unimoc::startup::FsmState::FAULT);
  zassert_not_equal(power_stage.GetState(), PowerStageState::kEnabled);
}

ZTEST_SUITE(sim_overcurrent, nullptr, Setup, Before, nullptr, nullptr);
