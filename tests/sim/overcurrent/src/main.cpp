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
#include "current_control_runtime.hpp"
#include "hardware_interface.hpp"
#include "hot_path_interface.hpp"
#include "hw_startup.hpp"
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
using namespace unimoc::unit;
namespace sim = unimoc::hardware::sim;

constexpr float kLimitA = 50.0F;

}  // namespace

// The platform calls these directly (hot_path_interface.hpp).
void unimoc::hardware::ControlCycleHandler() noexcept { CurrentControlIsr::Instance().OnControlCycle(); }
void unimoc::hardware::ObserverHandler() noexcept { SlowUpdate::Instance().RunOnce(); }

struct sim_overcurrent_fixture {
  unimoc::settings::SettingsStore store{unimoc::test::SimTestMotorProfile(
                                            unimoc::hardware::kSettings.GetSettingsProfile()),
                                        unimoc::hardware::kSettings.GetSettingsStorage(),
                                        unimoc::settings::SettingsSaveGate::Always()};
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
  zassert_equal(sim::Plant().Configure(settings), unimoc::sim::Status::kOk);
  zassert_true(unimoc::current_control::StartRuntime(settings,
                                                     unimoc::hardware::kMotors[0],
                                                     unimoc::unit::Current{kLimitA}));
  sim::SetFreeRunning(false);
  return &fixture;
}

void Before(void* /*unused*/) {
  // Flush the samples of the previous test with the stage off.
  sim::SetPhaseCurrentOverride(0.0F, 0.0F, 0.0F);
  sim::SetExternalEnable(true);
  sim::Plant().SetDcLinkVoltage(48.0_V);
  kPowerStage.Disable();
  RunCycles(1);
  kPowerStage.ClearFault();
  CurrentControlIsr::Instance().ClearTrips();
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
    zassert_within(duties.a, 0.5_ratio, 1e-6_ratio);
    zassert_within(duties.b, 0.5_ratio, 1e-6_ratio);
    zassert_within(duties.c, 0.5_ratio, 1e-6_ratio);
  }

  // The latched fault survives the current going away.
  sim::SetPhaseCurrentOverride(0.0F, 0.0F, 0.0F);
  RunCycles(2);
  zassert_false(kPowerStage.Enable());
}

ZTEST_F(sim_overcurrent, test_forced_duties_replace_the_loop_until_released) {
  CurrentControlIsr& isr = CurrentControlIsr::Instance();
  const Ratio kDutyMax = isr.svm.duty_max;
  isr.ForceDuty(0.25_ratio, 0.5_ratio, 2.0_ratio);  // phase C clamped to the SVM limit
  RunCycles(1);
  for (const auto& duties : isr.state.duty_plan.duties) {
    zassert_within(duties.a, 0.25_ratio, 1e-6_ratio);
    zassert_within(duties.b, 0.5_ratio, 1e-6_ratio);
    zassert_within(duties.c, kDutyMax, 1e-6_ratio);
  }

  // Released, the loop runs again: zero current and reference give neutral duties.
  isr.ReleaseForceDuty();
  RunCycles(2);
  for (const auto& duties : isr.state.duty_plan.duties) {
    zassert_within(duties.a, 0.5_ratio, 1e-3_ratio);
    zassert_within(duties.c, 0.5_ratio, 1e-3_ratio);
  }
}

ZTEST_F(sim_overcurrent, test_dc_link_overvoltage_trips_and_freezes_the_loop) {
  CurrentControlIsr& isr = CurrentControlIsr::Instance();
  zassert_true(kPowerStage.Enable());
  sim::Plant().SetDcLinkVoltage(57.9_V);  // below the 58 V trip of the sim profile
  RunCycles(2);
  zassert_false(isr.state.overvoltage_tripped);
  zassert_equal(kPowerStage.GetState(), PowerStageState::kEnabled);

  sim::Plant().SetDcLinkVoltage(58.5_V);
  RunCycles(2);  // the samples of a cycle reach the control cycle in the next one
  zassert_true(isr.state.overvoltage_tripped);
  zassert_true(isr.Tripped());
  zassert_equal(kPowerStage.GetState(), PowerStageState::kFault);
  for (const auto& duties : isr.state.duty_plan.duties) {
    zassert_within(duties.a, 0.5_ratio, 1e-6_ratio);
  }

  sim::Plant().SetDcLinkVoltage(48.0_V);
  RunCycles(2);
  zassert_true(isr.state.overvoltage_tripped, "latched until acknowledged");
  isr.ClearTrips();
  zassert_false(isr.Tripped());
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

namespace {

/// One sample for the startup FSM: phase currents a ± noise, b ± noise (alternating sign), c balancing.
void StartupSample(unimoc::startup::HwStartup& startup,
                   const float a,
                   const float b,
                   const float noise,
                   const unsigned index) {
  const float kNoise = (index % 2U == 0U) ? noise : -noise;
  sim::SetPhaseCurrentOverride(a + kNoise, b + kNoise, -(a + b) - (2.0F * kNoise));
  RunCycles(1);
  startup.RunOnce();
}

/// Runs @p samples samples of one step, then advances to the next step.
void StartupStep(unimoc::startup::HwStartup& startup,
                 const unsigned samples,
                 const float a,
                 const float b,
                 const float noise) {
  for (unsigned i = 0U; i < samples; ++i) {
    StartupSample(startup, a, b, noise, i);
  }
  startup.RequestNextStep();
  StartupSample(startup, a, b, noise, samples);
}

}  // namespace

ZTEST_F(sim_overcurrent, test_startup_results_are_in_units) {
  using unimoc::startup::FsmState;
  using unimoc::startup::HwStartup;
  CurrentControlIsr& isr = CurrentControlIsr::Instance();
  const uint32_t kTriggerOffset = isr.state.adc_trigger_offset;
  HwStartup startup{isr, fixture->store.GetOperations()};
  startup.RequestRun();
  zassert_equal(startup.State(), FsmState::kPwmDisable);

  // Phase 1 without a motor: a zero offset of 0.2 A and -0.1 A with 10 mA noise.
  StartupStep(startup, 1U, 0.2F, -0.1F, 0.01F);
  zassert_equal(startup.State(), FsmState::kAdcOffsetCal);
  StartupStep(startup, HwStartup::kNCal, 0.2F, -0.1F, 0.01F);
  UNIMOC_EXPECT_NEAR(startup.results.adc_offset_a, 0.2_A, 1e-5_A);
  UNIMOC_EXPECT_NEAR(startup.results.adc_offset_b, -0.1_A, 1e-5_A);
  zexpect_true(startup.results.passed[static_cast<uint8_t>(FsmState::kAdcOffsetCal)]);
  StartupStep(startup, HwStartup::kNCal, 0.2F, -0.1F, 0.01F);
  UNIMOC_EXPECT_NEAR(startup.results.adc_noise_rms_a, 0.01_A, 1e-5_A);
  UNIMOC_EXPECT_NEAR(startup.results.adc_noise_rms_b, 0.01_A, 1e-5_A);
  zexpect_true(startup.results.passed[static_cast<uint8_t>(FsmState::kAdcNoiseFloor)]);
  for (unsigned i = 0U; i < 3U; ++i) {  // DUTY_FORCE_LOW .. HIGH
    StartupStep(startup, HwStartup::kHoldSamples + 1U, 0.2F, -0.1F, 0.01F);
  }
  zassert_equal(startup.State(), FsmState::kDcLinkVoltageCheck);
  startup.SetExtVdc(48.0_V);  // the sim DC link
  StartupStep(startup, HwStartup::kNCal, 0.2F, -0.1F, 0.01F);
  UNIMOC_EXPECT_NEAR(startup.results.gain_vdc, 1.0_ratio, 1e-3_ratio);
  zexpect_true(startup.results.passed[static_cast<uint8_t>(FsmState::kDcLinkVoltageCheck)]);
  StartupStep(startup, 2U * 512U, 0.2F, -0.1F, 0.01F);  // GATE_DRIVER_ENABLE_CHECK
  zassert_equal(startup.State(), FsmState::kConnectMotor);
  StartupStep(startup, 1U, 0.0F, 0.0F, 0.0F);
  zassert_equal(startup.State(), FsmState::kPhaseAdcAlignment);

  // Phase 2: 2 A through phase A with the same noise. The RMS of each sweep
  // position needs the double accumulators: in float the 2 A would bury it.
  StartupStep(startup, (HwStartup::kNSweepSteps * HwStartup::kNAlignSamples) + 1U, 2.0F, -1.0F, 0.01F);
  zexpect_true(startup.results.passed[static_cast<uint8_t>(FsmState::kPhaseAdcAlignment)]);
  zassert_equal(startup.State(), FsmState::kCurrentSenseCalibration);

  // 1 A on alpha (phase B carries -0.5 A), the clamp meter reads 1.25 A.
  startup.SetExtCurrent(1.25_A);
  for (unsigned i = 0U; i < HwStartup::kNCal; ++i) {
    StartupSample(startup, 1.0F, -0.5F, 0.01F, i);
  }
  // The samples reach the control cycle one cycle late: the first one of the
  // step is still the last one of the alignment (1/1024 of 1 A off).
  UNIMOC_EXPECT_NEAR(startup.results.gain_a, 0.8_ratio, 1e-3_ratio);
  UNIMOC_EXPECT_NEAR(startup.results.gain_b, 0.8_ratio, 1e-3_ratio);
  zexpect_false(startup.results.passed[static_cast<uint8_t>(FsmState::kCurrentSenseCalibration)],
                "20 %% gain error");

  startup.RequestAbort();
  zexpect_equal(startup.State(), FsmState::kFault);
  isr.SetAdcTriggerOffset(kTriggerOffset);  // the sweep moved it
}

ZTEST_SUITE(sim_overcurrent, nullptr, Setup, Before, nullptr, nullptr);
