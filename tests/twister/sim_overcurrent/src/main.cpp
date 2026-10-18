// SPDX-License-Identifier: GPL-3.0-or-later
#include <zephyr/ztest.h>

#include "current_control_isr.hpp"
#include "hardware_interface.hpp"
#include "hw_startup.hpp"
#include "power_stage_interface.hpp"
#include "settings_store.hpp"
#include "sim_control.hpp"

namespace {

using unimoc::current_control::CurrentControlIsr;
using unimoc::hardware::PowerStageState;
using unimoc::hardware::power_stage;
namespace sim = unimoc::hardware::sim;

constexpr float kLimitA = 50.0F;

}  // namespace

struct sim_overcurrent_fixture {
  unimoc::settings::SettingsStore store{unimoc::hardware::settings.GetSettingsProfile(),
                                        unimoc::hardware::settings.GetSettingsStorage()};
};

namespace {

// One control sample is a fast interrupt; four of them complete the sub-step cycle.
void RunSamples(const unsigned count) {
  for (unsigned i = 0; i < count; ++i) CurrentControlIsr::instance().on_jeoc();
}

void *Setup() {
  static sim_overcurrent_fixture fixture;
  zassert_equal(fixture.store.Load(), unimoc::settings::SettingsStatus::kFactoryDefaults);
  auto& isr = CurrentControlIsr::instance();
  isr.init(fixture.store.GetSnapshot().Get(), unimoc::hardware::motor[0], 170'000'000U);
  isr.set_overcurrent_limit(unimoc::unit::Current{kLimitA});
  return &fixture;
}

void Before(void *) {
  sim::SetPhaseCurrents(0.0F, 0.0F, 0.0F);
  sim::SetExternalEnable(true);
  power_stage.Disable();
  power_stage.ClearFault();
  CurrentControlIsr::instance().clear_overcurrent_trip();
}

}  // namespace

ZTEST_F(sim_overcurrent, test_below_limit_keeps_stage_enabled) {
  zassert_true(power_stage.Enable());
  sim::SetPhaseCurrents(40.0F, -20.0F, -20.0F);
  RunSamples(8);
  zassert_equal(power_stage.GetState(), PowerStageState::kEnabled);
  zassert_false(CurrentControlIsr::instance().state.overcurrent_tripped);
}

ZTEST_F(sim_overcurrent, test_overcurrent_trips_and_freezes_loop) {
  zassert_true(power_stage.Enable());
  sim::SetPhaseCurrents(0.0F, 60.0F, -60.0F);
  RunSamples(1);
  zassert_equal(power_stage.GetState(), PowerStageState::kFault);
  zassert_true(CurrentControlIsr::instance().state.overcurrent_tripped);
  const auto duties = CurrentControlIsr::instance().state.phase_duties;
  zassert_within(duties.a.Value(), 0.5F, 1e-6F);
  zassert_within(duties.b.Value(), 0.5F, 1e-6F);
  zassert_within(duties.c.Value(), 0.5F, 1e-6F);

  // The latched fault survives the current going away.
  sim::SetPhaseCurrents(0.0F, 0.0F, 0.0F);
  RunSamples(4);
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

  sim::SetPhaseCurrents(70.0F, -35.0F, -35.0F);
  RunSamples(4);  // trip, then samples_ready for the startup FSM
  startup.run_once();
  zassert_equal(startup.state(), unimoc::startup::FsmState::FAULT);
  zassert_not_equal(power_stage.GetState(), PowerStageState::kEnabled);
}

ZTEST_SUITE(sim_overcurrent, nullptr, Setup, Before, nullptr, nullptr);
