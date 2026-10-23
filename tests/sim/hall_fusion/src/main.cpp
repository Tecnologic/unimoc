/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file main.cpp
 *    @brief Hall fusion of the observer against the simulated sensors: standstill
 *           without drift, constant speed, ramps and a Hall failure.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <zephyr/ztest.h>
#include <cmath>
#include <numbers>
#include "angle_wrap.hpp"
#include "current_control_isr.hpp"
#include "current_control_runtime.hpp"
#include "hardware_interface.hpp"
#include "hot_path_interface.hpp"
#include "motor_simulation.hpp"
#include "power_stage_interface.hpp"
#include "rotor_angle.hpp"
#include "settings_store.hpp"
#include "sim_control.hpp"
#include "slow_update.hpp"

namespace {

using unimoc::current_control::CurrentControlIsr;
using unimoc::current_control::SlowUpdate;
using unimoc::hardware::kPowerStage;
using unimoc::observer::WrapAngle;
using namespace unimoc::unit;
namespace sim = unimoc::hardware::sim;

constexpr unsigned kCyclesPerMs = 16U;  // 32 kHz PWM, 16 kHz control cycle
constexpr float kPi = std::numbers::pi_v<float>;
constexpr Angle kDegree = 1.0_deg;
constexpr uint8_t kPolePairs = 7U;

// The outrunner of sim_closed_loop with Hall sensors, and the flux observer
// weighted down at low speed.
unimoc::settings::NvmSettings MotorSettings(const unimoc::settings::NvmSettings& factory) {
  unimoc::settings::NvmSettings settings = factory;
  constexpr float kWc = 2.0F * kPi * 500.0F;
  settings.pole_pairs = kPolePairs;
  settings.stator_r = 0.05_Ohm;
  settings.l_d = 100e-6_H;
  settings.l_q = 100e-6_H;
  settings.flux_pm = 5e-3_Wb;
  settings.motor_j = 1e-4_kg_m2;
  settings.current_kp_d = VoltagePerCurrent{100e-6F * kWc};
  settings.current_kp_q = VoltagePerCurrent{100e-6F * kWc};
  settings.current_ki_d = VoltagePerCurrentTime{0.05F * kWc};
  settings.current_ki_q = VoltagePerCurrentTime{0.05F * kWc};
  settings.dtc_dead_time = 0.0_s;
  settings.hall_enable = 1U;
  settings.hall_offset = Angle{};  // the simulated sensors are aligned
  settings.pmsm_flux_obs_r_speed = 1.0F;
  return settings;
}

/// Electrical angle error of the observer. Its estimate refers to the first
/// sample of the cycle that the plant has just run, three half periods ago.
Angle AngleError() {
  const auto& plant = sim::Plant().Output();
  const auto& isr = CurrentControlIsr::Instance();
  const Angle kTrue =
      plant.electrical_angle.AngleInRevolution() - plant.electrical_speed * (isr.state.dt_fast * 3.0F);
  return WrapAngle(isr.mech_obs.theta - kTrue);
}

void RunMs(const unsigned milliseconds) { sim::AdvanceCycles(milliseconds * kCyclesPerMs); }

/// Places the shaft at an electrical angle and restarts the controller (estimate at zero).
void StartAt(const unimoc::settings::NvmSettings& settings, const Angle electrical_angle) {
  sim::Reset();
  sim::Plant().SetMechanicalState(unimoc::system::RotorAngle::FromAngle(electrical_angle /
                                                                        static_cast<float>(kPolePairs)),
                                  AngularVelocity{});
  zassert_true(unimoc::current_control::StartRuntime(settings, unimoc::hardware::kMotors[0], 60.0_A));
  sim::SetFreeRunning(false);
}

}  // namespace

// The platform calls these directly (hot_path_interface.hpp).
void unimoc::hardware::ControlCycleHandler() noexcept { CurrentControlIsr::Instance().OnControlCycle(); }
void unimoc::hardware::ObserverHandler() noexcept { SlowUpdate::Instance().RunOnce(); }

struct sim_hall_fusion_fixture {
  unimoc::settings::SettingsStore store{unimoc::hardware::kSettings.GetSettingsProfile(),
                                        unimoc::hardware::kSettings.GetSettingsStorage(),
                                        unimoc::settings::SettingsSaveGate::Always()};
  unimoc::settings::NvmSettings settings{};
};

namespace {

void* Setup() {
  static sim_hall_fusion_fixture fixture;
  zassert_equal(fixture.store.Load(), unimoc::settings::SettingsStatus::kFactoryDefaults);
  fixture.settings = MotorSettings(fixture.store.GetSnapshot().Get());
  return &fixture;
}

void Before(void* data) {
  auto* fixture = static_cast<sim_hall_fusion_fixture*>(data);
  zassert_equal(sim::Plant().Configure(fixture->settings), unimoc::sim::Status::kOk);
  sim::ClearPhaseCurrentOverride();
  sim::ClearHallCodeOverride();
  sim::SetExternalEnable(true);
  kPowerStage.ClearFault();
}

void After(void* /*unused*/) {
  kPowerStage.Disable();
  sim::ClearHallCodeOverride();
  CurrentControlIsr::Instance().SetCurrentRef({Current{}, Current{}});
}

}  // namespace

ZTEST_F(sim_hall_fusion, test_standstill_estimate_finds_the_sector_and_does_not_drift) {
  sim::Plant().SetShaftMode(unimoc::sim::ShaftMode::kImposedSpeed, AngularVelocity{});  // locked
  for (const Angle kAngle : {100.0F * kDegree, -150.0F * kDegree, 10.0F * kDegree}) {
    StartAt(fixture->settings, kAngle);
    RunMs(500U);
    // The estimate starts at zero; the sector pulls it within 30 degrees (plus margin).
    const Angle kSettled = AngleError();
    zassert_true(Abs(kSettled) < 32.0F * kDegree,
                 "start %f deg: error %f deg",
                 static_cast<double>((kAngle / kDegree).Value()),
                 static_cast<double>((kSettled / kDegree).Value()));
    RunMs(1000U);
    zexpect_within(AngleError(), kSettled, 1.0F * kDegree, "drift at standstill");
    zexpect_within(CurrentControlIsr::Instance().mech_obs.omega, 0.0_rad_per_s, 1.0_rad_per_s);
    zexpect_equal(SlowUpdate::Instance().hall.Edges(), 0U);
  }
}

ZTEST_F(sim_hall_fusion, test_constant_speed_is_tracked_from_the_edges) {
  StartAt(fixture->settings, 0.0_rad);
  constexpr float kMechanicalSpeed = 30.0F;  // 210 rad/s electrical, 33 Hz
  sim::Plant().SetShaftMode(unimoc::sim::ShaftMode::kImposedSpeed, AngularVelocity{kMechanicalSpeed});
  RunMs(300U);
  const float kElectricalSpeed = kMechanicalSpeed * static_cast<float>(kPolePairs);
  zexpect_within(CurrentControlIsr::Instance().mech_obs.omega,
                 kElectricalSpeed * 1.0_rad_per_s,
                 (0.03F * kElectricalSpeed) * 1.0_rad_per_s,
                 "omega %f",
                 static_cast<double>(CurrentControlIsr::Instance().mech_obs.omega.Value()));
  zexpect_within(AngleError(),
                 0.0_rad,
                 5.0F * kDegree,
                 "error %f deg",
                 static_cast<double>((AngleError() / kDegree).Value()));
  zexpect_true(SlowUpdate::Instance().hall.Edges() > 50U);
  zexpect_equal(SlowUpdate::Instance().hall.InvalidTransitions(), 0U);
}

ZTEST_F(sim_hall_fusion, test_speed_ramp_is_tracked) {
  StartAt(fixture->settings, 0.0_rad);
  // 0 to 60 rad/s mechanical (420 rad/s electrical) in one second, in both
  // directions, without current: the observer learns the acceleration from
  // the edges alone. At low speed the edges are rare and only the sector is
  // known (about +-30 degrees); once they come every few milliseconds, the
  // estimate follows within 10 degrees.
  for (const float kSign : {1.0F, -1.0F}) {
    Angle slow_error;
    Angle fast_error;
    for (unsigned ms = 0U; ms <= 1000U; ++ms) {
      sim::Plant().SetShaftMode(unimoc::sim::ShaftMode::kImposedSpeed,
                                AngularVelocity{kSign * 0.06F * static_cast<float>(ms)});
      RunMs(1U);
      Angle& error = (ms < 400U) ? slow_error : fast_error;
      error = Max(error, Abs(AngleError()));
    }
    zexpect_true(slow_error < 45.0F * kDegree,
                 "low-speed error %f deg",
                 static_cast<double>((slow_error / kDegree).Value()));
    zexpect_true(fast_error < 10.0F * kDegree,
                 "error %f deg",
                 static_cast<double>((fast_error / kDegree).Value()));
    sim::Plant().SetShaftMode(unimoc::sim::ShaftMode::kImposedSpeed, AngularVelocity{});
    RunMs(200U);
  }
}

ZTEST_F(sim_hall_fusion, test_hall_failure_falls_back_to_the_flux_observer) {
  StartAt(fixture->settings, 0.0_rad);
  constexpr float kMechanicalSpeed = 50.0F;  // 350 rad/s electrical: back-EMF 2.6 V
  sim::Plant().SetShaftMode(unimoc::sim::ShaftMode::kImposedSpeed, AngularVelocity{kMechanicalSpeed});
  // The current loop holds zero current, so its voltage demand is the back-EMF.
  zassert_true(kPowerStage.Enable());
  CurrentControlIsr::Instance().SetCurrentRef({Current{}, Current{}});
  RunMs(300U);
  zassert_within(AngleError(), 0.0_rad, 5.0F * kDegree);

  sim::SetHallCodeOverride(0U);  // all three sensors low: unpowered or disconnected
  RunMs(500U);
  zexpect_true(SlowUpdate::Instance().hall.InvalidCodes() >= 1U);
  zexpect_within(AngleError(),
                 0.0_rad,
                 10.0F * kDegree,
                 "error %f deg",
                 static_cast<double>((AngleError() / kDegree).Value()));
  zexpect_within(CurrentControlIsr::Instance().mech_obs.omega,
                 (kMechanicalSpeed * static_cast<float>(kPolePairs)) * 1.0_rad_per_s,
                 (0.05F * kMechanicalSpeed * static_cast<float>(kPolePairs)) * 1.0_rad_per_s);
  zexpect_false(CurrentControlIsr::Instance().state.overcurrent_tripped);
}

ZTEST_SUITE(sim_hall_fusion, nullptr, Setup, Before, After, nullptr);
