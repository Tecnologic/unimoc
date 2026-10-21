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

constexpr unsigned kCyclesPerMs = 16U;  // 32 kHz PWM, 16 kHz control cycle

// Small outrunner: 50 mOhm, 100 uH, 7 pole pairs. PI tuned for a 500 Hz
// current loop (kp = L * wc, ki = R * wc).
unimoc::settings::NvmSettings MotorSettings(const unimoc::settings::NvmSettings& factory) {
  unimoc::settings::NvmSettings settings = factory;
  constexpr float kWc = 2.0F * std::numbers::pi_v<float> * 500.0F;
  settings.pole_pairs = 7U;
  settings.stator_r = Resistance{0.05F};
  settings.l_d = Inductance{100e-6F};
  settings.l_q = Inductance{100e-6F};
  settings.flux_pm = MagneticFlux{5e-3F};
  settings.current_kp_d = VoltagePerCurrent{100e-6F * kWc};
  settings.current_kp_q = VoltagePerCurrent{100e-6F * kWc};
  settings.current_ki_d = VoltagePerCurrentTime{0.05F * kWc};
  settings.current_ki_q = VoltagePerCurrentTime{0.05F * kWc};
  settings.dtc_dead_time = Time{0.0F};  // the plant has no dead time
  // The rotor is locked: for the observer an inertia the current cannot move
  // (its torque model would otherwise accelerate the estimate).
  settings.motor_j = Inertia{1.0F};
  return settings;
}

// Plant current in the frame the controller works in (its estimated angle).
unimoc::system::Rotor<Current> PlantCurrentInControllerFrame() {
  const auto kCurrentAlphaBeta = sim::Plant().Output().phase_currents.ToStator();
  const unimoc::system::SinCos<Ratio> kSc(CurrentControlIsr::Instance().mech_obs.theta);
  return kCurrentAlphaBeta.ToRotor(kSc);
}

bool Finite(const unimoc::system::Rotor<Current>& current) {
  return std::isfinite(current.d.Value()) && std::isfinite(current.q.Value());
}

}  // namespace

// The platform calls these directly (hot_path_interface.hpp).
void unimoc::hardware::ControlCycleHandler() noexcept { CurrentControlIsr::Instance().OnControlCycle(); }
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
      unimoc::current_control::StartRuntime(fixture->settings, unimoc::hardware::kMotors[0], Current{60.0F}));
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
  isr.SetCurrentRef({Current{0.0F}, Current{10.0F}});

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
  zassert_within(kSettled.q.Value(), 10.0F, 0.3F, "i_q = %f A", static_cast<double>(kSettled.q.Value()));
  zassert_within(kSettled.d.Value(), 0.0F, 0.3F, "i_d = %f A", static_cast<double>(kSettled.d.Value()));
  zassert_equal(isr.state.observer_overruns, 0U);
  zassert_false(isr.state.overcurrent_tripped);
}

ZTEST_F(sim_closed_loop, test_d_current_step_settles) {
  auto& isr = CurrentControlIsr::Instance();
  isr.SetCurrentRef({Current{-8.0F}, Current{0.0F}});
  for (unsigned i = 0; i < 5U * kCyclesPerMs; ++i) {
    sim::StepCycle();
  }
  const auto kSettled = PlantCurrentInControllerFrame();
  zassert_within(kSettled.d.Value(), -8.0F, 0.3F, "i_d = %f A", static_cast<double>(kSettled.d.Value()));
  zassert_within(kSettled.q.Value(), 0.0F, 0.3F, "i_q = %f A", static_cast<double>(kSettled.q.Value()));
}

ZTEST_F(sim_closed_loop, test_duties_stay_in_range) {
  auto& isr = CurrentControlIsr::Instance();
  isr.SetCurrentRef({Current{0.0F}, Current{20.0F}});
  for (unsigned i = 0; i < 2U * kCyclesPerMs; ++i) {
    sim::StepCycle();
    for (const auto& d : sim::AppliedDuties()) {
      for (const float kDuty : {d.a.Value(), d.b.Value(), d.c.Value()}) {
        zassert_true(kDuty >= fixture->settings.svm_duty_min.Value() - 1e-6F &&
                         kDuty <= fixture->settings.svm_duty_max.Value() + 1e-6F,
                     "duty %f",
                     static_cast<double>(kDuty));
      }
    }
  }
}

ZTEST_F(sim_closed_loop, test_slow_update_runs_once_per_control_cycle) {
  auto& isr = CurrentControlIsr::Instance();
  isr.SetCurrentRef({Current{0.0F}, Current{5.0F}});
  constexpr uint32_t kCycles = 1000U;
  for (uint32_t i = 1U; i <= kCycles; ++i) {
    sim::StepCycle();
    // The observer ran directly after each control cycle and left nothing pending.
    zassert_equal(isr.state.control_cycles, i);
    zassert_equal(sim::CycleCount(), i);
    zassert_false(isr.state.observer_pending.load());
    zassert_equal(isr.state.double_buf.active.load(), i % 2U, "cycle %u", i);
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
  isr.SetCurrentRef({Current{-30.0F}, Current{55.0F}});
  for (unsigned i = 0; i < 20U * kCyclesPerMs; ++i) {
    sim::StepCycle();
    zassert_true(Finite(PlantCurrentInControllerFrame()), "plant current at cycle %u", i);
    zassert_true(std::isfinite(isr.state.u_dq_last.d.Value()) && std::isfinite(isr.state.u_dq_last.q.Value()),
                 "voltage demand at cycle %u",
                 i);
    zassert_true(std::isfinite(isr.mech_obs.theta.Value()) && std::isfinite(isr.mech_obs.omega.Value()),
                 "observer at cycle %u",
                 i);
    if (isr.state.overcurrent_tripped) {
      break;  // a trip is a defined outcome; the state must still be finite up to it
    }
  }
}

ZTEST_F(sim_closed_loop, test_reset_makes_a_run_repeatable) {
  const auto kRun = [&]() {
    // Same start as after power-up: platform at rest, controller from its initial state.
    sim::Reset();
    zassert_true(unimoc::current_control::StartRuntime(fixture->settings,
                                                       unimoc::hardware::kMotors[0],
                                                       Current{60.0F}));
    sim::SetFreeRunning(false);
    kPowerStage.ClearFault();
    zassert_true(kPowerStage.Enable());
    CurrentControlIsr::Instance().SetCurrentRef({Current{-3.0F}, Current{8.0F}});
    sim::AdvanceCycles(3U * kCyclesPerMs);
    return sim::TakeSnapshot();
  };
  const auto kFirst = kRun();
  const auto kSecond = kRun();

  zassert_equal(kFirst.cycle_count, 3U * kCyclesPerMs);
  zassert_equal(kSecond.cycle_count, kFirst.cycle_count);
  zassert_equal(kSecond.virtual_time_ns, kFirst.virtual_time_ns);
  // Bit-identical: the simulation is deterministic.
  zassert_equal(kSecond.plant.phase_currents.a.Value(), kFirst.plant.phase_currents.a.Value());
  zassert_equal(kSecond.plant.phase_currents.b.Value(), kFirst.plant.phase_currents.b.Value());
  zassert_equal(kSecond.plant.rotor_current.q.Value(), kFirst.plant.rotor_current.q.Value());
  zassert_equal(kSecond.applied[3].a.Value(), kFirst.applied[3].a.Value());
  zassert_true(kFirst.plant.rotor_current.q.Value() > 5.0F, "the run did something");
}

ZTEST_F(sim_closed_loop, test_reset_returns_the_plant_to_rest) {
  CurrentControlIsr::Instance().SetCurrentRef({Current{0.0F}, Current{10.0F}});
  sim::AdvanceCycles(2U * kCyclesPerMs);
  zassert_true(sim::TakeSnapshot().plant.rotor_current.q.Value() > 1.0F);

  sim::Reset();
  const auto kSnapshot = sim::TakeSnapshot();
  zassert_equal(kSnapshot.cycle_count, 0U);
  zassert_equal(kSnapshot.virtual_time_ns, uint64_t{0U});
  zassert_equal(kSnapshot.plant.phase_currents.a.Value(), 0.0F);
  zassert_equal(kSnapshot.plant.rotor_current.q.Value(), 0.0F);
  zassert_equal(kPowerStage.GetState(), unimoc::hardware::PowerStageState::kDisabled);
}

ZTEST_SUITE(sim_closed_loop, nullptr, Setup, Before, After, nullptr);
