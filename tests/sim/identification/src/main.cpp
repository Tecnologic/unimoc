/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file main.cpp
 *    @brief Motor identification M1, M2 and M3 against the simulated motor.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <zephyr/ztest.h>
#include <numbers>
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
using unimoc::identification::ConfirmResult;
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
constexpr MagneticFlux kFlux = 0.01_Wb;

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
  parameters.pmsm.flux_pm = kFlux;
  // Dry friction slows the free shaft of M3 down (2.5e-2 N m on 1e-4 kg m^2: 250 rad/s^2).
  parameters.mechanics.coulomb_friction = 0.025_Nm;
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

ZTEST_F(sim_identification, test_m3_measures_the_flux_linkage_by_a_coast_down) {
  MotorIdentification identification{CurrentControlIsr::Instance(), SlowUpdate::Instance()};
  identification.Configure(fixture->settings);
  sim::Plant().SetShaftMode(unimoc::sim::ShaftMode::kFree);
  zassert_equal(identification.Start(unimoc::identification::kMeasureFluxLinkage), StartResult::kStarted);
  zassert_equal(identification.GetStatus().step, Step::kFluxLinkage);

  // The outputs are on at zero current until the operator confirms the free shaft.
  for (uint32_t i = 0; i < 1000U; ++i) {
    sim::StepCycle();
    identification.RunOnce();
  }
  zexpect_true(identification.GetStatus().needs_shaft_free);
  zexpect_equal(identification.GetStatus().progress, 0U);
  zexpect_within(sim::Plant().Output().electrical_speed, AngularVelocity{}, 1.0_rad_per_s);
  zexpect_equal(identification.Confirm(false), ConfirmResult::kWrongConfirmation);
  zassert_equal(identification.Confirm(true), ConfirmResult::kAccepted);
  zexpect_false(identification.GetStatus().needs_shaft_free);

  // Spin-up, then the coast with the outputs open.
  AngularVelocity top_speed{};
  bool coasted = false;
  for (uint32_t i = 0; i < 600000U && identification.IsActive(); ++i) {
    sim::StepCycle();
    identification.RunOnce();
    top_speed = Max(top_speed, sim::Plant().Output().electrical_speed);
    coasted = coasted || kPowerStage.GetState() != PowerStageState::kEnabled;
  }
  zassert_false(identification.IsActive());
  zexpect_true(coasted);

  const auto& results = identification.GetResults();
  const auto kStatus = identification.GetStatus();
  zassert_equal(kStatus.step, Step::kDone);
  zassert_true(results.flux_valid);
  UNIMOC_EXPECT_NEAR(results.flux_pm, kFlux, kFlux * 0.02F);
  zexpect_true(results.flux_spread < 0.03_ratio);
  zexpect_true(results.flux_periods >= MotorIdentification::kMinFluxPeriods);
  zexpect_not_equal(kStatus.passed & (1U << static_cast<uint8_t>(Step::kFluxLinkage)), 0U);
  zexpect_equal(kStatus.failed, 0U);
  // The spin-up ends at the back-EMF target, 25 % of the DC link line to line, within the
  // windows that average the swing of the rotor about the forced angle (about 0.1 s).
  const AngularVelocity kTarget = ((kDcLink * 0.25F / std::numbers::sqrt3_v<float>) / kFlux) * 1.0_rad;
  zexpect_true(top_speed > kTarget * 0.9F && top_speed < kTarget * 1.5F, "the top speed misses the target");
  UNIMOC_EXPECT_NEAR(results.coast_speed, top_speed, top_speed * 0.05F);
  zexpect_not_equal(kPowerStage.GetState(), PowerStageState::kEnabled, "the outputs stay off");
  zexpect_equal(identification.Confirm(true), ConfirmResult::kNotWaiting);
}

ZTEST_F(sim_identification, test_m3_with_a_flux_in_the_settings_follows_the_coast) {
  // With psi in the settings the decoupling feedforward and the open-output tracking take part:
  // the feedforward must use the forced speed (the observer's runs off in the background), and
  // the observer must follow the coast from where the forced angle left it.
  unimoc::settings::NvmSettings settings = fixture->settings;
  settings.flux_pm = kFlux;
  settings.motor_i_max = 50.0_A;
  settings.current_kp_q = 0.6_V_per_A;
  zassert_true(unimoc::current_control::StartRuntime(settings, unimoc::hardware::kMotors[0], 60.0_A));
  sim::SetFreeRunning(false);
  unimoc::sim::SimulationParameters parameters = unimoc::sim::ParametersFromSettings(settings);
  parameters.dc_link_voltage = kDcLink;
  zassert_equal(sim::Plant().Configure(parameters), unimoc::sim::Status::kOk);
  sim::Plant().SetShaftMode(unimoc::sim::ShaftMode::kFree);
  MotorIdentification identification{CurrentControlIsr::Instance(), SlowUpdate::Instance()};
  identification.Configure(settings);
  zassert_equal(identification.Start(unimoc::identification::kMeasureFluxLinkage), StartResult::kStarted);
  identification.RunOnce();
  zassert_equal(identification.Confirm(true), ConfirmResult::kAccepted);

  bool loaded = false;
  uint32_t coast_cycles = 0U;
  for (uint32_t i = 0; i < 600000U && identification.IsActive(); ++i) {
    sim::StepCycle();
    identification.RunOnce();
    if (kPowerStage.GetState() != PowerStageState::kEnabled && !loaded) {
      // No friction in this plant: a load slows the coast down (only now, it would turn the free shaft back).
      sim::Plant().SetInput(unimoc::sim::SimulationInput{0.01_Nm, Voltage{}});
      loaded = true;
    }
    if (loaded && ++coast_cycles == 4800U) {
      // 0.3 s into the coast the estimate is the shaft's.
      UNIMOC_EXPECT_NEAR(CurrentControlIsr::Instance().mech_obs.omega,
                         sim::Plant().Output().electrical_speed,
                         Abs(sim::Plant().Output().electrical_speed) * 0.01F);
    }
  }
  sim::Plant().SetInput(unimoc::sim::SimulationInput{});
  zassert_true(loaded, "the run reached the coast");
  zexpect_true(coast_cycles > 4800U);
  zexpect_false(CurrentControlIsr::Instance().state.overcurrent_tripped);
  zexpect_equal(identification.GetStatus().step, Step::kDone);
  UNIMOC_EXPECT_NEAR(identification.GetResults().flux_pm, kFlux, kFlux * 0.02F);
  zassert_true(unimoc::current_control::StartRuntime(fixture->settings,
                                                     unimoc::hardware::kMotors[0],
                                                     fixture->settings.motor_i_max * 1.2F));
  sim::SetFreeRunning(false);
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
