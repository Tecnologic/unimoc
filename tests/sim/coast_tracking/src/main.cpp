/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file main.cpp
 *    @brief Outputs open: the observer follows a coasting motor from its phase voltages.
 *
 *    Without Hall sensors, the flux observer and the back-EMF speed measurement
 *    take the phase voltages of the open terminals (the plant's back-EMF), so
 *    the estimate follows the shaft while the drive is disengaged or tripped,
 *    and the drive engages on the spinning motor without a current jump.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <zephyr/ztest.h>
#include <algorithm>
#include "angle_wrap.hpp"
#include "current_control_isr.hpp"
#include "current_control_runtime.hpp"
#include "hardware_interface.hpp"
#include "hot_path_interface.hpp"
#include "motor_simulation.hpp"
#include "power_stage_interface.hpp"
#include "settings_store.hpp"
#include "sim_control.hpp"
#include "slow_update.hpp"
#include "unimoc_test.hpp"

namespace {

using unimoc::current_control::CurrentControlIsr;
using unimoc::current_control::SlowUpdate;
using unimoc::hardware::kPowerStage;
using unimoc::hardware::PowerStageState;
using namespace unimoc::unit;
namespace sim = unimoc::hardware::sim;

constexpr unsigned kCyclesPerMs = 16U;  // 32 kHz PWM, 16 kHz control cycle
constexpr uint8_t kPolePairs = 7U;
constexpr float kPoles = static_cast<float>(kPolePairs);

/// The outrunner of sim_speed_control, sensorless (no Hall fusion).
unimoc::settings::NvmSettings MotorSettings(const unimoc::settings::NvmSettings& factory) {
  unimoc::settings::NvmSettings settings = factory;
  constexpr float kWc = 2.0F * std::numbers::pi_v<float> * 500.0F;
  settings.pole_pairs = kPolePairs;
  settings.stator_r = 0.05_Ohm;
  settings.l_d = 100e-6_H;
  settings.l_q = 100e-6_H;
  settings.flux_pm = 5e-3_Wb;
  settings.motor_j = 1e-4_kg_m2;
  // A current loop of 500 Hz: kp = L wc, ki = R wc.
  constexpr InverseTime kBandwidth = 1.0_per_s * kWc;
  settings.current_kp_d = 100e-6_H * kBandwidth;
  settings.current_kp_q = 100e-6_H * kBandwidth;
  settings.current_ki_d = 0.05_Ohm * kBandwidth;
  settings.current_ki_q = 0.05_Ohm * kBandwidth;
  settings.dtc_dead_time = 0.0_s;
  settings.hall_enable = 0U;
  settings.pmsm_flux_obs_r_speed = 1.0_rad4_per_s2;
  settings.motor_i_max = 20.0_A;
  return settings;
}

void RunMs(const unsigned milliseconds) { sim::AdvanceCycles(milliseconds * kCyclesPerMs); }

/// Imposes the shaft speed of the electrical speed @p electrical.
void ImposeSpeed(const AngularVelocity electrical) {
  sim::Plant().SetShaftMode(unimoc::sim::ShaftMode::kImposedSpeed, electrical / kPoles);
}

/// Angle error of the estimate against the plant's electrical angle. After a
/// cycle the estimate is the angle at the first sample of the next one: one
/// control step after the plant's present state.
Angle AngleError() {
  const auto& plant = sim::Plant().Output();
  const auto& isr = CurrentControlIsr::Instance();
  return unimoc::observer::WrapAngle(isr.mech_obs.theta - plant.electrical_angle.AngleInRevolution() -
                                     (plant.electrical_speed * isr.state.dt_fast));
}

/// The estimate follows the plant: speed within 2 %, angle within @p angle.
void ExpectTracking(const AngularVelocity electrical, const Angle angle) {
  UNIMOC_EXPECT_NEAR(CurrentControlIsr::Instance().mech_obs.omega,
                     electrical,
                     (Abs(electrical) * 0.02F) + 2.0_rad_per_s);
  zexpect_true(Abs(AngleError()) < angle, "the angle estimate is off");
}

/// Largest phase current over @p milliseconds.
Current PeakCurrent(const unsigned milliseconds) {
  Current peak{};
  for (unsigned cycle = 0U; cycle < milliseconds * kCyclesPerMs; ++cycle) {
    sim::StepCycle();
    const auto& currents = sim::Plant().Output().phase_currents;
    peak = Max(peak, Max(Abs(currents.a), Max(Abs(currents.b), Abs(currents.c))));
  }
  return peak;
}

}  // namespace

// The platform calls these directly (hot_path_interface.hpp).
bool unimoc::hardware::SampleHandler() noexcept { return CurrentControlIsr::Instance().OnSample(); }
void unimoc::hardware::ObserverHandler() noexcept { SlowUpdate::Instance().RunOnce(); }

struct sim_coast_tracking_fixture {
  unimoc::settings::SettingsStore store{unimoc::hardware::kSettings.GetSettingsProfile(),
                                        unimoc::hardware::kSettings.GetSettingsStorage(),
                                        unimoc::settings::SettingsSaveGate::Always()};
  unimoc::settings::NvmSettings settings{};
};

namespace {

void* Setup() {
  static sim_coast_tracking_fixture fixture;
  zassert_equal(fixture.store.Load(), unimoc::settings::SettingsStatus::kFactoryDefaults);
  fixture.settings = MotorSettings(fixture.store.GetSnapshot().Get());
  return &fixture;
}

void Before(void* data) {
  const auto& settings = static_cast<sim_coast_tracking_fixture*>(data)->settings;
  sim::ClearPhaseCurrentOverride();
  sim::SetExternalEnable(true);
  kPowerStage.Disable();
  zassert_equal(sim::Plant().Configure(settings), unimoc::sim::Status::kOk);
  sim::Plant().SetInput(unimoc::sim::SimulationInput{});
  sim::SetSenseFilters(unimoc::hardware::SenseDelays{});
  sim::Reset();
  zassert_true(unimoc::current_control::StartRuntime(settings, unimoc::hardware::kMotors[0], 60.0_A));
  sim::SetFreeRunning(false);
  CurrentControlIsr::Instance().ClearTrips();
  kPowerStage.ClearFault();
}

void After(void* /*unused*/) { kPowerStage.Disable(); }

}  // namespace

ZTEST_F(sim_coast_tracking, test_the_estimate_follows_the_open_terminals) {
  // The estimate starts at standstill, the shaft turns at 1000 rad/s electrical
  // (5 V back-EMF); from scratch the flux observer settles with its own gains.
  ImposeSpeed(1000.0_rad_per_s);
  RunMs(300U);
  zassert_not_equal(kPowerStage.GetState(), PowerStageState::kEnabled);
  ExpectTracking(1000.0_rad_per_s, 0.01_rad);

  // A reversal: the sense of rotation of the back-EMF turns the estimate round.
  ImposeSpeed(-700.0_rad_per_s);
  RunMs(300U);
  ExpectTracking(-700.0_rad_per_s, 0.01_rad);

  // The shaft stops: below 0.3 V the voltages only bound the speed (0.3 V / psi = 60 rad/s),
  // and the estimate is pulled within the bound.
  ImposeSpeed(AngularVelocity{});
  RunMs(200U);
  zexpect_within(CurrentControlIsr::Instance().mech_obs.omega, AngularVelocity{}, 65.0_rad_per_s);
}

ZTEST_F(sim_coast_tracking, test_engages_on_the_coasting_motor_without_a_current_jump) {
  ImposeSpeed(1200.0_rad_per_s);
  RunMs(300U);
  ExpectTracking(1200.0_rad_per_s, 0.01_rad);

  // Engaged at zero current: the decoupling applies the back-EMF in the right frame.
  CurrentControlIsr::Instance().ClearTrips();
  zassert_true(kPowerStage.Enable());
  const Current kPeak = PeakCurrent(20U);
  zexpect_true(kPeak < 2.0_A, "a current jump at the engage");
  zexpect_false(CurrentControlIsr::Instance().Tripped());
  // Engaged, the flux observer runs on the voltage demand again (its usual lag at speed).
  ExpectTracking(1200.0_rad_per_s, 0.15_rad);

  // Disengaged again: the estimate stays with the shaft, the next engage is as smooth.
  kPowerStage.Disable();
  ImposeSpeed(900.0_rad_per_s);
  RunMs(150U);
  ExpectTracking(900.0_rad_per_s, 0.01_rad);
  CurrentControlIsr::Instance().ClearTrips();
  zassert_true(kPowerStage.Enable());
  zexpect_true(PeakCurrent(20U) < 2.0_A);
}

ZTEST_F(sim_coast_tracking, test_the_estimate_follows_after_a_trip) {
  ImposeSpeed(800.0_rad_per_s);
  zassert_true(kPowerStage.Enable());
  // An overcurrent trip: the outputs are off and stay latched until the next engage.
  sim::SetPhaseCurrentOverride(80.0F, -40.0F, -40.0F);
  RunMs(1U);
  sim::ClearPhaseCurrentOverride();
  zassert_true(CurrentControlIsr::Instance().Tripped());

  ImposeSpeed(-500.0_rad_per_s);
  RunMs(300U);
  ExpectTracking(-500.0_rad_per_s, 0.01_rad);
}

namespace {

/// Restarts the controller with sense filters in the plant; it reads their delays.
void StartWithSenseFilters(const unimoc::settings::NvmSettings& settings,
                           const unimoc::hardware::SenseDelays& filters) {
  sim::SetSenseFilters(filters);
  sim::Reset();
  zassert_true(unimoc::current_control::StartRuntime(settings, unimoc::hardware::kMotors[0], 60.0_A));
  sim::SetFreeRunning(false);
}

}  // namespace

ZTEST_F(sim_coast_tracking, test_the_phase_voltage_filter_delay_is_compensated) {
  // The Jacobi phase-voltage filters, about 7 us: uncompensated 0.014 rad at 2000 rad/s.
  StartWithSenseFilters(fixture->settings, unimoc::hardware::SenseDelays{Time{}, 7.2e-6_s});
  ImposeSpeed(2000.0_rad_per_s);
  RunMs(300U);
  ExpectTracking(2000.0_rad_per_s, 0.003_rad);
}

namespace {

/// The plant's d current at 1000 rad/s, engaged with a q current of 5 A behind a current filter.
Current DCurrentBehind(const unimoc::settings::NvmSettings& settings, const Time filter) {
  StartWithSenseFilters(settings, unimoc::hardware::SenseDelays{filter, Time{}});
  ImposeSpeed(1000.0_rad_per_s);
  RunMs(300U);
  CurrentControlIsr::Instance().ClearTrips();
  zassert_true(kPowerStage.Enable());
  CurrentControlIsr::Instance().SetCurrentRef(unimoc::system::Rotor<Current>{Current{}, 5.0_A});
  RunMs(50U);
  CurrentControlIsr::Instance().SetCurrentRef(unimoc::system::Rotor<Current>{});
  UNIMOC_EXPECT_NEAR(sim::Plant().Output().rotor_current.q, 5.0_A, 0.1_A);
  kPowerStage.Disable();
  return sim::Plant().Output().rotor_current.d;
}

}  // namespace

ZTEST_F(sim_coast_tracking, test_the_current_filter_delay_is_compensated) {
  // A slow current filter, 40 us: uncompensated the Park angle lags 0.04 rad at 1000 rad/s and the
  // plant carries 0.2 A of d current more than without the filter (measured: 0.21 A).
  const Current kWithout = DCurrentBehind(fixture->settings, Time{});
  const Current kBehind = DCurrentBehind(fixture->settings, 40.0e-6_s);
  UNIMOC_EXPECT_NEAR(kBehind, kWithout, 0.05_A);
}

ZTEST_SUITE(sim_coast_tracking, nullptr, Setup, Before, After, nullptr);
