// SPDX-License-Identifier: GPL-3.0-or-later
#include <zephyr/ztest.h>
#include <cmath>
#include "current_control_isr.hpp"
#include "hardware_interface.hpp"
#include "motor_simulation.hpp"
#include "power_stage_interface.hpp"
#include "settings_store.hpp"
#include "sim_control.hpp"
#include "sin_cos.hpp"
#include "slow_update.hpp"

namespace {

using unimoc::current_control::CurrentControlIsr;
using unimoc::current_control::SlowUpdate;
using unimoc::hardware::power_stage;
using namespace unimoc::unit;
namespace sim = unimoc::hardware::sim;

constexpr unsigned kCyclesPerMs = 16U;  // 32 kHz PWM, 16 kHz control cycle

void OnControl() noexcept { CurrentControlIsr::instance().on_control_cycle(); }
void OnObserver() noexcept { SlowUpdate::instance().run_once(); }

// Small outrunner: 50 mOhm, 100 uH, 7 pole pairs. PI tuned for a 500 Hz
// current loop (kp = L * wc, ki = R * wc).
unimoc::settings::NvmSettings MotorSettings(const unimoc::settings::NvmSettings& factory) {
  unimoc::settings::NvmSettings s = factory;
  constexpr float kWc = 2.0F * 3.14159265F * 500.0F;
  s.pole_pairs = 7U;
  s.stator_r = Resistance{0.05F};
  s.l_d = Inductance{100e-6F};
  s.l_q = Inductance{100e-6F};
  s.flux_pm = MagneticFlux{5e-3F};
  s.current_kp_d = VoltagePerCurrent{100e-6F * kWc};
  s.current_kp_q = VoltagePerCurrent{100e-6F * kWc};
  s.current_ki_d = VoltagePerCurrentTime{0.05F * kWc};
  s.current_ki_q = VoltagePerCurrentTime{0.05F * kWc};
  s.dtc_dead_time = Time{0.0F};  // the plant has no dead time
  return s;
}

// Plant current in the frame the controller works in (its estimated angle).
unimoc::system::Rotor<Current> PlantCurrentInControllerFrame() {
  const auto i_ab = sim::Plant().Output().phase_currents.ToStator();
  const unimoc::system::SinCos<Ratio> sc(CurrentControlIsr::instance().mech_obs.theta);
  return i_ab.ToRotor(sc);
}

bool Finite(const unimoc::system::Rotor<Current>& i) {
  return std::isfinite(i.d.Value()) && std::isfinite(i.q.Value());
}

}  // namespace

struct sim_closed_loop_fixture {
  unimoc::settings::SettingsStore store{unimoc::hardware::settings.GetSettingsProfile(),
                                        unimoc::hardware::settings.GetSettingsStorage()};
  unimoc::settings::NvmSettings settings{};
};

namespace {

void* Setup() {
  static sim_closed_loop_fixture fixture;
  zassert_equal(fixture.store.Load(), unimoc::settings::SettingsStatus::kFactoryDefaults);
  fixture.settings = MotorSettings(fixture.store.GetSnapshot().Get());
  return &fixture;
}

void Before(void* f) {
  auto* fixture = static_cast<sim_closed_loop_fixture*>(f);
  auto& isr = CurrentControlIsr::instance();
  isr.init(fixture->settings, unimoc::hardware::motor[0], 170'000'000U);
  SlowUpdate::instance().init(fixture->settings, isr);
  isr.set_overcurrent_limit(Current{60.0F});
  zassert_equal(sim::Plant().Configure(fixture->settings), unimoc::sim::Status::kOk);
  sim::Plant().SetShaftMode(unimoc::sim::ShaftMode::kImposedSpeed, AngularVelocity{});  // locked rotor
  sim::ClearPhaseCurrentOverride();
  zassert_true(
      unimoc::hardware::motor[0].Initialize(fixture->settings.pwm_frequency, 0U, OnControl, OnObserver));
  sim::SetFreeRunning(false);
  sim::SetExternalEnable(true);
  power_stage.ClearFault();
  zassert_true(power_stage.Enable());
}

void After(void*) {
  power_stage.Disable();
  CurrentControlIsr::instance().set_current_ref({Current{}, Current{}});
}

}  // namespace

ZTEST_F(sim_closed_loop, test_q_current_step_settles) {
  auto& isr = CurrentControlIsr::instance();
  isr.set_current_ref({Current{0.0F}, Current{10.0F}});

  // 500 Hz loop: well above 60 % after 1 ms, settled after 5 ms.
  for (unsigned i = 0; i < kCyclesPerMs; ++i) sim::StepCycle();
  const auto early = PlantCurrentInControllerFrame();
  zassert_true(early.q.Value() > 6.0F, "i_q after 1 ms = %f A", static_cast<double>(early.q.Value()));

  for (unsigned i = 0; i < 4U * kCyclesPerMs; ++i) sim::StepCycle();
  const auto settled = PlantCurrentInControllerFrame();
  zassert_true(Finite(settled));
  zassert_within(settled.q.Value(), 10.0F, 0.3F, "i_q = %f A", static_cast<double>(settled.q.Value()));
  zassert_within(settled.d.Value(), 0.0F, 0.3F, "i_d = %f A", static_cast<double>(settled.d.Value()));
  zassert_equal(isr.state.observer_overruns, 0U);
  zassert_false(isr.state.overcurrent_tripped);
}

ZTEST_F(sim_closed_loop, test_d_current_step_settles) {
  auto& isr = CurrentControlIsr::instance();
  isr.set_current_ref({Current{-8.0F}, Current{0.0F}});
  for (unsigned i = 0; i < 5U * kCyclesPerMs; ++i) sim::StepCycle();
  const auto settled = PlantCurrentInControllerFrame();
  zassert_within(settled.d.Value(), -8.0F, 0.3F, "i_d = %f A", static_cast<double>(settled.d.Value()));
  zassert_within(settled.q.Value(), 0.0F, 0.3F, "i_q = %f A", static_cast<double>(settled.q.Value()));
}

ZTEST_F(sim_closed_loop, test_duties_stay_in_range) {
  auto& isr = CurrentControlIsr::instance();
  isr.set_current_ref({Current{0.0F}, Current{20.0F}});
  for (unsigned i = 0; i < 2U * kCyclesPerMs; ++i) {
    sim::StepCycle();
    for (const auto& d : sim::AppliedDuties()) {
      for (const float v : {d.a.Value(), d.b.Value(), d.c.Value()}) {
        zassert_true(v >= fixture->settings.svm_duty_min.Value() - 1e-6F &&
                         v <= fixture->settings.svm_duty_max.Value() + 1e-6F,
                     "duty %f",
                     static_cast<double>(v));
      }
    }
  }
}

ZTEST_SUITE(sim_closed_loop, nullptr, Setup, Before, After, nullptr);
