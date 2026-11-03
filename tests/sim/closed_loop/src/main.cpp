/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file main.cpp
 *    @brief Closed-loop current steps of the control core against the simulated drive.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <zephyr/ztest.h>
#include <cmath>
#include <cstdint>
#include <numbers>
#include "current_control_isr.hpp"
#include "current_control_runtime.hpp"
#include "hardware_interface.hpp"
#include "hot_path_interface.hpp"
#include "motor_simulation.hpp"
#include "power_stage_interface.hpp"
#include "settings_store.hpp"
#include "sim_control.hpp"
#include "sin_cos.hpp"
#include "slow_update.hpp"

namespace {

using unimoc::current_control::CurrentControlIsr;
using unimoc::current_control::SlowUpdate;
using unimoc::hardware::kPowerStage;
using namespace unimoc::unit;
namespace sim = unimoc::hardware::sim;

constexpr unsigned kCyclesPerMs = 16U;  // 32 kHz PWM: sim cycles of two PWM periods

// Small outrunner: 50 mOhm, 100 uH, 7 pole pairs. PI tuned for a 500 Hz
// current loop (kp = L * wc, ki = R * wc).
unimoc::settings::NvmSettings MotorSettings(const unimoc::settings::NvmSettings& factory) {
  unimoc::settings::NvmSettings settings = factory;
  constexpr float kWc = 2.0F * std::numbers::pi_v<float> * 500.0F;
  settings.pole_pairs = 7U;
  settings.stator_r = 0.05_Ohm;
  settings.l_d = 100e-6_H;
  settings.l_q = 100e-6_H;
  settings.flux_pm = 5e-3_Wb;
  settings.current_kp_d = VoltagePerCurrent{100e-6F * kWc};
  settings.current_kp_q = VoltagePerCurrent{100e-6F * kWc};
  settings.current_ki_d = VoltagePerCurrentTime{0.05F * kWc};
  settings.current_ki_q = VoltagePerCurrentTime{0.05F * kWc};
  settings.dtc_dead_time = 0.0_s;  // the plant has no dead time
  // The rotor is locked: for the observer an inertia the current cannot move
  // (its torque model would otherwise accelerate the estimate).
  settings.motor_j = 1.0_kg_m2;
  return settings;
}

// Plant current in the frame the controller works in (its estimated angle).
unimoc::system::Rotor<Current> PlantCurrentInControllerFrame() {
  const auto kCurrentAlphaBeta = sim::Plant().Output().phase_currents.ToStator();
  const unimoc::system::SinCos<Ratio> kSc(CurrentControlIsr::Instance().mech_obs.theta);
  return kCurrentAlphaBeta.ToRotor(kSc);
}

bool Finite(const unimoc::system::Rotor<Current>& current) {
  return IsFinite(current.d) && IsFinite(current.q);
}

// Restarts the controller with @p pwm_frequency and @p divider (the power stage is off meanwhile).
void Restart(unimoc::settings::NvmSettings settings, const Frequency pwm_frequency, const uint8_t divider) {
  kPowerStage.Disable();
  settings.pwm_frequency = pwm_frequency;
  settings.control_divider = divider;
  zassert_true(unimoc::current_control::StartRuntime(settings, unimoc::hardware::kMotors[0], 60.0_A));
  sim::SetFreeRunning(false);
  zassert_true(kPowerStage.Enable());
}

// PI gains for 60 degrees phase margin at the output delay of the running timing:
// w_c = (pi / 6) / delay, kp = L w_c, ki = R w_c.
void TuneToDelay() {
  auto& isr = CurrentControlIsr::Instance();
  const InverseTime kWc =
      std::numbers::pi_v<float> / (6.0F * isr.state.dt_fast * isr.state.output_delay_steps);
  const VoltagePerCurrent kKp = 100e-6_H * kWc;
  const VoltagePerCurrentTime kKi = 0.05_Ohm * kWc;
  isr.SetCurrentGains({.kp_d = kKp, .ki_d = kKi, .kp_q = kKp, .ki_q = kKi});
}

}  // namespace

// The platform calls these directly (hot_path_interface.hpp).
bool unimoc::hardware::SampleHandler() noexcept { return CurrentControlIsr::Instance().OnSample(); }
void unimoc::hardware::ObserverHandler() noexcept { SlowUpdate::Instance().RunOnce(); }

struct sim_closed_loop_fixture {
  unimoc::settings::SettingsStore store{unimoc::hardware::kSettings.GetSettingsProfile(),
                                        unimoc::hardware::kSettings.GetSettingsStorage(),
                                        unimoc::settings::SettingsSaveGate::Always()};
  unimoc::settings::NvmSettings settings{};
};

namespace {

void* Setup() {
  static sim_closed_loop_fixture fixture;
  zassert_equal(fixture.store.Load(), unimoc::settings::SettingsStatus::kFactoryDefaults);
  fixture.settings = MotorSettings(fixture.store.GetSnapshot().Get());
  return &fixture;
}

void Before(void* data) {
  auto* fixture = static_cast<sim_closed_loop_fixture*>(data);
  zassert_equal(sim::Plant().Configure(fixture->settings), unimoc::sim::Status::kOk);
  sim::Plant().SetShaftMode(unimoc::sim::ShaftMode::kImposedSpeed, AngularVelocity{});  // locked rotor
  sim::ClearPhaseCurrentOverride();
  zassert_true(
      unimoc::current_control::StartRuntime(fixture->settings, unimoc::hardware::kMotors[0], 60.0_A));
  sim::SetFreeRunning(false);
  sim::SetExternalEnable(true);
  kPowerStage.ClearFault();
  zassert_true(kPowerStage.Enable());
}

void After(void* /*unused*/) {
  kPowerStage.Disable();
  CurrentControlIsr::Instance().SetCurrentRef({Current{}, Current{}});
}

}  // namespace

ZTEST_F(sim_closed_loop, test_q_current_step_settles) {
  auto& isr = CurrentControlIsr::Instance();
  isr.SetCurrentRef({0.0_A, 10.0_A});

  // 500 Hz loop: well above 60 % after 1 ms, settled after 5 ms.
  for (unsigned i = 0; i < kCyclesPerMs; ++i) {
    sim::StepCycle();
  }
  const auto kEarly = PlantCurrentInControllerFrame();
  zassert_true(kEarly.q.Value() > 6.0F, "i_q after 1 ms = %f A", static_cast<double>(kEarly.q.Value()));

  for (unsigned i = 0; i < 4U * kCyclesPerMs; ++i) {
    sim::StepCycle();
  }
  const auto kSettled = PlantCurrentInControllerFrame();
  zassert_true(Finite(kSettled));
  zassert_within(kSettled.q, 10.0_A, 0.3_A, "i_q = %f A", static_cast<double>(kSettled.q.Value()));
  zassert_within(kSettled.d, 0.0_A, 0.3_A, "i_d = %f A", static_cast<double>(kSettled.d.Value()));
  zassert_equal(isr.state.observer_overruns, 0U);
  zassert_false(isr.state.overcurrent_tripped);
}

ZTEST_F(sim_closed_loop, test_q_current_step_at_the_bandwidth_of_the_half_period_step) {
  // The factory 32 kHz PWM resolves to a step every half period (64 kHz):
  // 1.5 half periods (23.4 us) from a sample to the middle of its duties.
  // 60 degrees phase margin: w_c = (pi / 6) / 23.4 us, 22.3 krad/s (3.6 kHz).
  auto& isr = CurrentControlIsr::Instance();
  zassert_equal(isr.state.timing.divider, 1U);
  zassert_within(isr.state.output_delay_steps, 1.5F, 1e-6F);
  TuneToDelay();
  isr.SetCurrentRef({0.0_A, 10.0_A});

  // Two sim cycles, eight control steps (125 us): within 5 % of the step.
  sim::StepCycle();
  sim::StepCycle();
  const auto kEarly = PlantCurrentInControllerFrame();
  zassert_within(kEarly.q, 10.0_A, 0.5_A, "i_q after 125 us");

  Current peak{};
  for (unsigned i = 0; i < kCyclesPerMs; ++i) {
    sim::StepCycle();
    peak = Max(peak, PlantCurrentInControllerFrame().q);
  }
  zassert_true(peak < 10.8_A, "overshoot");
  const auto kSettled = PlantCurrentInControllerFrame();
  zassert_within(kSettled.q, 10.0_A, 0.1_A);
  zassert_within(kSettled.d, 0.0_A, 0.1_A);
  zassert_false(isr.state.overcurrent_tripped);
}

ZTEST_F(sim_closed_loop, test_q_current_step_at_the_bandwidth_of_64_khz_pwm) {
  // A control step every PWM period at 64 kHz: one period (15.6 us) from a
  // sample to the middle of its duties. 60 degrees phase margin: w_c =
  // (pi / 6) / 15.6 us, 33.5 krad/s (5.3 kHz).
  Restart(fixture->settings, 64.0_kHz, 2U);
  auto& isr = CurrentControlIsr::Instance();
  zassert_equal(isr.state.timing.divider, 2U);
  zassert_within(isr.state.output_delay_steps, 1.0F, 1e-6F);
  TuneToDelay();
  isr.SetCurrentRef({0.0_A, 10.0_A});

  // Four sim cycles, eight control steps (125 us): within 5 % of the step.
  for (unsigned i = 0; i < 4U; ++i) {
    sim::StepCycle();
  }
  const auto kEarly = PlantCurrentInControllerFrame();
  zassert_within(kEarly.q, 10.0_A, 0.5_A, "i_q after 125 us");

  Current peak{};
  for (unsigned i = 0; i < 2U * kCyclesPerMs; ++i) {
    sim::StepCycle();
    peak = Max(peak, PlantCurrentInControllerFrame().q);
  }
  zassert_true(peak < 10.8_A, "overshoot");
  const auto kSettled = PlantCurrentInControllerFrame();
  zassert_within(kSettled.q, 10.0_A, 0.1_A);
  zassert_within(kSettled.d, 0.0_A, 0.1_A);
  zassert_false(isr.state.overcurrent_tripped);
}

ZTEST_F(sim_closed_loop, test_control_divider_runs_a_step_every_second_period) {
  // 64 kHz PWM, divider 4 half periods: 32 kHz control steps, the observer
  // after every second one (16 kHz). The duties hold for two periods: 1.5
  // periods delay.
  Restart(fixture->settings, 64.0_kHz, 4U);
  auto& isr = CurrentControlIsr::Instance();
  zassert_equal(isr.state.timing.divider, 4U);
  zassert_equal(isr.state.timing.steps_per_cycle, 2U);
  zassert_within(isr.state.output_delay_steps, 0.75F, 1e-6F);
  TuneToDelay();
  isr.SetCurrentRef({0.0_A, 10.0_A});

  // A sim cycle is two PWM periods: one control step each.
  for (uint32_t i = 1U; i <= 4U * kCyclesPerMs; ++i) {
    sim::StepCycle();
    zassert_equal(isr.state.periods, 2U * i);
    zassert_equal(isr.state.samples, i);
    zassert_equal(isr.state.control_cycles, i / 2U);
  }
  const auto kSettled = PlantCurrentInControllerFrame();
  zassert_within(kSettled.q, 10.0_A, 0.1_A);
  zassert_within(kSettled.d, 0.0_A, 0.1_A);
  zassert_equal(isr.state.observer_overruns, 0U);
}

ZTEST_F(sim_closed_loop, test_d_current_step_settles) {
  auto& isr = CurrentControlIsr::Instance();
  isr.SetCurrentRef({-8.0_A, 0.0_A});
  for (unsigned i = 0; i < 5U * kCyclesPerMs; ++i) {
    sim::StepCycle();
  }
  const auto kSettled = PlantCurrentInControllerFrame();
  zassert_within(kSettled.d, -8.0_A, 0.3_A, "i_d = %f A", static_cast<double>(kSettled.d.Value()));
  zassert_within(kSettled.q, 0.0_A, 0.3_A, "i_q = %f A", static_cast<double>(kSettled.q.Value()));
}

ZTEST_F(sim_closed_loop, test_duties_stay_in_range) {
  auto& isr = CurrentControlIsr::Instance();
  isr.SetCurrentRef({0.0_A, 20.0_A});
  for (unsigned i = 0; i < 2U * kCyclesPerMs; ++i) {
    sim::StepCycle();
    for (const auto& d : sim::AppliedDuties()) {
      for (const Ratio kDuty : {d.a, d.b, d.c}) {
        zassert_true(kDuty >= fixture->settings.svm_duty_min - 1e-6_ratio &&
                         kDuty <= fixture->settings.svm_duty_max + 1e-6_ratio,
                     "duty %f",
                     static_cast<double>(kDuty.Value()));
      }
    }
  }
}

ZTEST_F(sim_closed_loop, test_slow_update_runs_once_per_control_cycle) {
  auto& isr = CurrentControlIsr::Instance();
  isr.SetCurrentRef({0.0_A, 5.0_A});
  constexpr uint32_t kCycles = 1000U;
  for (uint32_t i = 1U; i <= kCycles; ++i) {
    sim::StepCycle();
    // The observer ran directly after each control cycle and left nothing pending.
    zassert_equal(isr.state.control_cycles, i);
    zassert_equal(sim::CycleCount(), i);
    zassert_false(isr.state.observer_pending.load());
    zassert_equal(isr.state.angles.active.load(), i % 2U, "cycle %u", i);
    zassert_equal(isr.state.samples, 4U * i, "one control step per half period");
  }
  zassert_equal(isr.state.observer_overruns, 0U);
  // 16 kHz control cycle: exactly 62.5 us each, no rounding drift.
  zassert_equal(sim::VirtualTimeNs(), uint64_t{kCycles} * 62'500U);
}

ZTEST_F(sim_closed_loop, test_advance_cycles_matches_single_steps) {
  sim::AdvanceCycles(7U);
  zassert_equal(sim::CycleCount(), 7U);
  zassert_equal(CurrentControlIsr::Instance().state.control_cycles, 7U);
  sim::AdvanceCycles(0U);
  zassert_equal(sim::CycleCount(), 7U);
}

ZTEST_F(sim_closed_loop, test_state_stays_finite_under_aggressive_reference) {
  auto& isr = CurrentControlIsr::Instance();
  // Reference far beyond what the DC link can deliver: the loop saturates.
  isr.SetCurrentRef({-30.0_A, 55.0_A});
  for (unsigned i = 0; i < 20U * kCyclesPerMs; ++i) {
    sim::StepCycle();
    zassert_true(Finite(PlantCurrentInControllerFrame()), "plant current at cycle %u", i);
    zassert_true(IsFinite(isr.state.u_dq_last.d) && IsFinite(isr.state.u_dq_last.q),
                 "voltage demand at cycle %u",
                 i);
    zassert_true(IsFinite(isr.mech_obs.theta) && IsFinite(isr.mech_obs.omega), "observer at cycle %u", i);
    if (isr.state.overcurrent_tripped) {
      break;  // a trip is a defined outcome; the state must still be finite up to it
    }
  }
}

ZTEST_F(sim_closed_loop, test_reset_makes_a_run_repeatable) {
  const auto kRun = [&]() {
    // Same start as after power-up: platform at rest, controller from its initial state.
    sim::Reset();
    zassert_true(
        unimoc::current_control::StartRuntime(fixture->settings, unimoc::hardware::kMotors[0], 60.0_A));
    sim::SetFreeRunning(false);
    kPowerStage.ClearFault();
    zassert_true(kPowerStage.Enable());
    CurrentControlIsr::Instance().SetCurrentRef({-3.0_A, 8.0_A});
    sim::AdvanceCycles(3U * kCyclesPerMs);
    return sim::TakeSnapshot();
  };
  const auto kFirst = kRun();
  const auto kSecond = kRun();

  zassert_equal(kFirst.cycle_count, 3U * kCyclesPerMs);
  zassert_equal(kSecond.cycle_count, kFirst.cycle_count);
  zassert_equal(kSecond.virtual_time_ns, kFirst.virtual_time_ns);
  // Bit-identical: the simulation is deterministic.
  zassert_equal(kSecond.plant.phase_currents.a, kFirst.plant.phase_currents.a);
  zassert_equal(kSecond.plant.phase_currents.b, kFirst.plant.phase_currents.b);
  zassert_equal(kSecond.plant.rotor_current.q, kFirst.plant.rotor_current.q);
  zassert_equal(kSecond.applied[3].a, kFirst.applied[3].a);
  zassert_true(kFirst.plant.rotor_current.q > 5.0_A, "the run did something");
}

ZTEST_F(sim_closed_loop, test_reset_returns_the_plant_to_rest) {
  CurrentControlIsr::Instance().SetCurrentRef({0.0_A, 10.0_A});
  sim::AdvanceCycles(2U * kCyclesPerMs);
  zassert_true(sim::TakeSnapshot().plant.rotor_current.q > 1.0_A);

  sim::Reset();
  const auto kSnapshot = sim::TakeSnapshot();
  zassert_equal(kSnapshot.cycle_count, 0U);
  zassert_equal(kSnapshot.virtual_time_ns, uint64_t{0U});
  zassert_equal(kSnapshot.plant.phase_currents.a, 0.0_A);
  zassert_equal(kSnapshot.plant.rotor_current.q, 0.0_A);
  zassert_equal(kPowerStage.GetState(), unimoc::hardware::PowerStageState::kDisabled);
}

ZTEST_SUITE(sim_closed_loop, nullptr, Setup, Before, After, nullptr);
