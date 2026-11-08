/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file main.cpp
 *    @brief Motor identification M1 and M2 against the simulated motor.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <zephyr/ztest.h>
#include "current_control_isr.hpp"
#include "current_control_runtime.hpp"
#include "hardware_interface.hpp"
#include "hot_path_interface.hpp"
#include "motor_identification.hpp"
#include "power_stage_interface.hpp"
#include "settings_store.hpp"
#include "sim_control.hpp"
#include "sim_test_motor.hpp"
#include "slow_update.hpp"
#include "unimoc_test.hpp"

namespace {

using unimoc::current_control::CurrentControlIsr;
using unimoc::current_control::SlowUpdate;
using unimoc::hardware::kPowerStage;
using unimoc::hardware::PowerStageState;
using unimoc::identification::Fault;
using unimoc::identification::MotorIdentification;
using unimoc::identification::StartResult;
using unimoc::identification::Step;
using namespace unimoc::unit;
namespace sim = unimoc::hardware::sim;

// The truth of the simulated motor; the settings keep the test motor (0.1 Ohm, 1 mH) as the prior.
constexpr Resistance kResistance = 0.12_Ohm;
constexpr Inductance kLd = 0.8_mH;
constexpr Inductance kLq = 1.2_mH;
constexpr Time kDeadTime = 0.5e-6_s;
constexpr Voltage kDcLink = 48.0_V;

}  // namespace

// The platform calls these directly (hot_path_interface.hpp).
bool unimoc::hardware::SampleHandler() noexcept { return CurrentControlIsr::Instance().OnSample(); }
void unimoc::hardware::ObserverHandler() noexcept { SlowUpdate::Instance().RunOnce(); }

struct sim_identification_fixture {
  unimoc::settings::SettingsStore store{unimoc::test::SimTestMotorProfile(
                                            unimoc::hardware::kSettings.GetSettingsProfile()),
                                        unimoc::hardware::kSettings.GetSettingsStorage(),
                                        unimoc::settings::SettingsSaveGate::Always()};
  unimoc::settings::NvmSettings settings{};
};

namespace {

void* Setup() {
  static sim_identification_fixture fixture;
  zassert_equal(fixture.store.Load(), unimoc::settings::SettingsStatus::kFactoryDefaults);
  fixture.settings = fixture.store.GetSnapshot().Get();
  zassert_true(unimoc::current_control::StartRuntime(fixture.settings,
                                                     unimoc::hardware::kMotors[0],
                                                     fixture.settings.motor_i_max * 1.2F));
  sim::SetFreeRunning(false);
  return &fixture;
}

void Before(void* data) {
  auto* fixture = static_cast<sim_identification_fixture*>(data);
  kPowerStage.Disable();
  sim::Reset();
  unimoc::sim::SimulationParameters parameters = unimoc::sim::ParametersFromSettings(fixture->settings);
  parameters.pmsm.stator_resistance = kResistance;
  parameters.pmsm.l_d = kLd;
  parameters.pmsm.l_q = kLq;
  parameters.pmsm.flux_pm = 0.01_Wb;
  parameters.inverter.dead_time = kDeadTime * fixture->settings.pwm_frequency;
  parameters.dc_link_voltage = kDcLink;
  zassert_equal(sim::Plant().Configure(parameters), unimoc::sim::Status::kOk);
  // Standstill measurements on a held shaft; the rotor's d axis at angle 0.
  sim::Plant().SetShaftMode(unimoc::sim::ShaftMode::kImposedSpeed, AngularVelocity{});
  sim::SetExternalEnable(true);
  CurrentControlIsr::Instance().ClearTrips();
  kPowerStage.ClearFault();
}

/// Runs the identification to its end; false if it is still active after @p max_cycles.
bool RunToEnd(MotorIdentification& identification, const uint32_t max_cycles) {
  for (uint32_t i = 0; i < max_cycles && identification.IsActive(); ++i) {
    sim::StepCycle();
    identification.RunOnce();
  }
  return !identification.IsActive();
}

}  // namespace

ZTEST_F(sim_identification, test_m1_measures_resistance_and_dead_time) {
  MotorIdentification identification{CurrentControlIsr::Instance(), SlowUpdate::Instance()};
  identification.Configure(fixture->settings);
  zassert_equal(identification.Start(unimoc::identification::kMeasureResistance), StartResult::kStarted);
  zassert_equal(kPowerStage.GetState(), PowerStageState::kEnabled);
  zassert_equal(identification.GetStatus().step, Step::kResistance);

  zassert_true(RunToEnd(identification, 200000U));

  const auto& results = identification.GetResults();
  zassert_equal(identification.GetStatus().step, Step::kDone);
  zassert_true(results.resistance_valid);
  zassert_false(results.inductance_valid);
  UNIMOC_EXPECT_NEAR(results.resistance, kResistance, kResistance * 0.02F);
  UNIMOC_EXPECT_NEAR(results.dead_time, kDeadTime, kDeadTime * 0.05F);
  zexpect_not_equal(identification.GetStatus().passed & (1U << static_cast<uint8_t>(Step::kResistance)), 0U);
  zexpect_not_equal(kPowerStage.GetState(), PowerStageState::kEnabled, "the outputs are off again");
}

ZTEST_F(sim_identification, test_m2_measures_both_inductances) {
  MotorIdentification identification{CurrentControlIsr::Instance(), SlowUpdate::Instance()};
  identification.Configure(fixture->settings);
  zassert_equal(identification.Start(unimoc::identification::kMeasureResistance |
                                     unimoc::identification::kMeasureInductance),
                StartResult::kStarted);

  zassert_true(RunToEnd(identification, 400000U));

  const auto& results = identification.GetResults();
  const auto kStatus = identification.GetStatus();
  zassert_equal(kStatus.step, Step::kDone);
  zassert_true(results.inductance_valid);
  UNIMOC_EXPECT_NEAR(results.l_d, kLd, kLd * 0.02F);
  UNIMOC_EXPECT_NEAR(results.l_q, kLq, kLq * 0.02F);
  UNIMOC_EXPECT_NEAR(results.l_mean, (kLd + kLq) * 0.5F, kLd * 0.03F);
  UNIMOC_EXPECT_NEAR(results.resistance_ac, kResistance, kResistance * 0.2F);
  zexpect_equal(kStatus.failed, 0U);
}

ZTEST_F(sim_identification, test_abort_and_a_lost_enable_end_the_run) {
  MotorIdentification identification{CurrentControlIsr::Instance(), SlowUpdate::Instance()};
  identification.Configure(fixture->settings);
  zassert_equal(identification.Start(unimoc::identification::kMeasureResistance), StartResult::kStarted);
  zassert_equal(identification.Start(unimoc::identification::kMeasureResistance), StartResult::kRunning);
  sim::StepCycle();
  identification.RunOnce();
  identification.Abort();
  zexpect_equal(identification.GetStatus().step, Step::kFault);
  zexpect_equal(identification.GetStatus().fault, Fault::kAborted);
  zexpect_not_equal(kPowerStage.GetState(), PowerStageState::kEnabled);

  zassert_equal(identification.Start(unimoc::identification::kMeasureInductance), StartResult::kStarted);
  sim::SetExternalEnable(false);  // the E-stop loop opens
  sim::StepCycle();
  identification.RunOnce();
  zexpect_equal(identification.GetStatus().step, Step::kFault);
  zexpect_equal(identification.GetStatus().fault, Fault::kEnableLost);

  zexpect_equal(identification.Start(0U), StartResult::kNoMeasurement);
  zexpect_equal(identification.Start(unimoc::identification::kMeasureResistance), StartResult::kNotReleased);
}

ZTEST_SUITE(sim_identification, nullptr, Setup, Before, nullptr, nullptr);
