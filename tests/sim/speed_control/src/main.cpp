/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file main.cpp
 *    @brief Outer loop against the simulated drive: speed steps, the current
 *           limit, load rejection, position steps and the inactive state.
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
#include "outer_loop.hpp"
#include "power_stage_interface.hpp"
#include "settings_store.hpp"
#include "sim_control.hpp"
#include "slow_update.hpp"

namespace {

using unimoc::current_control::CurrentControlIsr;
using unimoc::current_control::OuterLoop;
using unimoc::current_control::OuterSetpoint;
using unimoc::current_control::SlowUpdate;
using unimoc::cyphal::ControlMode;
using unimoc::hardware::kPowerStage;
using namespace unimoc::unit;
namespace sim = unimoc::hardware::sim;

constexpr unsigned kCyclesPerMs = 16U;  // 32 kHz PWM, 16 kHz control cycle
constexpr float kPi = std::numbers::pi_v<float>;
constexpr uint8_t kPolePairs = 7U;

// The outrunner of sim_closed_loop with Hall sensors and a speed loop of about
// 30 Hz on 1e-4 kg m^2 (kp = J * wc, integral corner at wc / 4).
unimoc::settings::NvmSettings MotorSettings(const unimoc::settings::NvmSettings& factory) {
  unimoc::settings::NvmSettings settings = factory;
  constexpr float kWc = 2.0F * kPi * 500.0F;
  constexpr float kSpeedWc = 2.0F * kPi * 30.0F;
  settings.pole_pairs = kPolePairs;
  settings.stator_r = Resistance{0.05F};
  settings.l_d = Inductance{100e-6F};
  settings.l_q = Inductance{100e-6F};
  settings.flux_pm = MagneticFlux{5e-3F};
  settings.motor_j = Inertia{1e-4F};
  settings.current_kp_d = VoltagePerCurrent{100e-6F * kWc};
  settings.current_kp_q = VoltagePerCurrent{100e-6F * kWc};
  settings.current_ki_d = VoltagePerCurrentTime{0.05F * kWc};
  settings.current_ki_q = VoltagePerCurrentTime{0.05F * kWc};
  settings.dtc_dead_time = Time{0.0F};
  settings.hall_enable = 1U;
  settings.pmsm_flux_obs_r_speed = 1.0F;
  settings.speed_kp = TorquePerAngularVelocity{1e-4F * kSpeedWc};
  settings.speed_ki = InverseTime{kSpeedWc / 4.0F};
  settings.speed_accel_limit = AngularAcceleration{2000.0F};
  settings.motor_i_max = Current{20.0F};
  settings.pos_kp_pos = AngularVelocityPerAngle{20.0F};
  settings.pos_speed_limit = AngularVelocity{40.0F};
  settings.pos_accel_limit = AngularAcceleration{1000.0F};
  settings.pos_position_tolerance = Angle{0.02F};
  settings.pos_speed_tolerance = AngularVelocity{1.0F};
  return settings;
}

float MechanicalSpeed() { return sim::Plant().Output().mechanical_speed.Value(); }
float MechanicalAngle() { return sim::Plant().Output().mechanical_angle.AbsoluteAngle().Value(); }

void RunMs(const unsigned milliseconds) { sim::AdvanceCycles(milliseconds * kCyclesPerMs); }

OuterSetpoint Speed(const float speed) {
  OuterSetpoint setpoint{};
  setpoint.active = true;
  setpoint.mode = ControlMode::kSpeed;
  setpoint.speed = AngularVelocity{speed};
  return setpoint;
}

OuterSetpoint Position(const float position) {
  OuterSetpoint setpoint{};
  setpoint.active = true;
  setpoint.mode = ControlMode::kPosition;
  setpoint.position = Angle{position};
  return setpoint;
}

}  // namespace

// The platform calls these directly (hot_path_interface.hpp). The control
// thread runs synchronously here, every OuterLoop::kCyclesPerStep cycles.
void unimoc::hardware::ControlCycleHandler() noexcept { CurrentControlIsr::Instance().OnControlCycle(); }
void unimoc::hardware::ObserverHandler() noexcept {
  SlowUpdate::Instance().RunOnce();
  if (OuterLoop::Instance().OnObserverCycle()) {
    OuterLoop::Instance().Step();
  }
}

struct sim_speed_control_fixture {
  unimoc::settings::SettingsStore store{unimoc::hardware::kSettings.GetSettingsProfile(),
                                        unimoc::hardware::kSettings.GetSettingsStorage(),
                                        unimoc::settings::SettingsSaveGate::Always()};
  unimoc::settings::NvmSettings settings{};
};

namespace {

void* Setup() {
  static sim_speed_control_fixture fixture;
  zassert_equal(fixture.store.Load(), unimoc::settings::SettingsStatus::kFactoryDefaults);
  fixture.settings = MotorSettings(fixture.store.GetSnapshot().Get());
  return &fixture;
}

void Start(const unimoc::settings::NvmSettings& settings) {
  OuterLoop::Instance().SetSetpoint(OuterSetpoint{});
  zassert_equal(sim::Plant().Configure(settings), unimoc::sim::Status::kOk);
  sim::Plant().SetShaftMode(unimoc::sim::ShaftMode::kFree);
  sim::Plant().SetInput(unimoc::sim::SimulationInput{});
  sim::Reset();
  zassert_true(
      unimoc::current_control::StartRuntime(settings, unimoc::hardware::kMotors[0], Current{60.0F}));
  sim::SetFreeRunning(false);
  kPowerStage.ClearFault();
  zassert_true(kPowerStage.Enable());
}

void Before(void* data) {
  sim::ClearPhaseCurrentOverride();
  sim::SetExternalEnable(true);
  Start(static_cast<sim_speed_control_fixture*>(data)->settings);
}

void After(void* /*unused*/) {
  OuterLoop::Instance().SetSetpoint(OuterSetpoint{});
  kPowerStage.Disable();
}

}  // namespace

ZTEST_F(sim_speed_control, test_inactive_setpoint_commands_zero_current) {
  RunMs(10U);
  const auto kStatus = OuterLoop::Instance().Status();
  zexpect_true(kStatus.steps >= 10U * kCyclesPerMs / OuterLoop::kCyclesPerStep - 1U);
  zexpect_false(kStatus.active);
  zexpect_equal(CurrentControlIsr::Instance().state.i_ref.q.Value(), 0.0F);
  zexpect_within(MechanicalSpeed(), 0.0F, 0.1F);
}

ZTEST_F(sim_speed_control, test_speed_step_reaches_the_setpoint) {
  OuterLoop::Instance().SetSetpoint(Speed(30.0F));
  RunMs(400U);
  zexpect_within(MechanicalSpeed(), 30.0F, 0.6F, "speed %f", static_cast<double>(MechanicalSpeed()));
  const auto kStatus = OuterLoop::Instance().Status();
  zexpect_true(kStatus.active);
  zexpect_equal(kStatus.mode, ControlMode::kSpeed);
  zexpect_within(kStatus.speed.Value(), 30.0F, 0.6F);

  // And back through zero to the reverse direction.
  OuterLoop::Instance().SetSetpoint(Speed(-20.0F));
  RunMs(400U);
  zexpect_within(MechanicalSpeed(), -20.0F, 0.6F, "speed %f", static_cast<double>(MechanicalSpeed()));
  zexpect_false(CurrentControlIsr::Instance().state.overcurrent_tripped);
}

ZTEST_F(sim_speed_control, test_load_torque_is_rejected) {
  OuterLoop::Instance().SetSetpoint(Speed(25.0F));
  RunMs(300U);
  sim::Plant().SetInput(unimoc::sim::SimulationInput{Torque{0.2F}, Voltage{}});
  RunMs(500U);
  zexpect_within(MechanicalSpeed(), 25.0F, 0.6F, "speed %f", static_cast<double>(MechanicalSpeed()));
  // 0.2 N m at 3/2 * 7 * 5 mWb = 52.5 mN m/A: about 3.8 A of q current.
  zexpect_within(OuterLoop::Instance().Status().current_reference.q.Value(), 0.2F / 0.0525F, 0.4F);
}

ZTEST_F(sim_speed_control, test_current_limit_takes_effect) {
  auto settings = fixture->settings;
  settings.motor_i_max = Current{5.0F};
  settings.speed_accel_limit = AngularAcceleration{};  // a step: the controller saturates
  Start(settings);
  OuterLoop::Instance().SetSetpoint(Speed(200.0F));
  float largest = 0.0F;
  for (unsigned ms = 0U; ms < 50U; ++ms) {
    RunMs(1U);
    const auto& reference = CurrentControlIsr::Instance().state.i_ref;
    largest = std::max(largest, std::hypot(reference.d.Value(), reference.q.Value()));
  }
  zexpect_true(largest <= 5.0F + 1e-4F, "largest reference %f A", static_cast<double>(largest));
  zexpect_within(largest, 5.0F, 1e-3F);
  // Limited torque: 5 A * 52.5 mN m/A on 1e-4 kg m^2 is 2625 rad/s^2, so 50 ms reach at most 131 rad/s.
  zexpect_true(MechanicalSpeed() > 50.0F && MechanicalSpeed() < 135.0F,
               "speed %f",
               static_cast<double>(MechanicalSpeed()));
}

ZTEST_F(sim_speed_control, test_position_step_settles_in_position) {
  const float kStart = MechanicalAngle();
  OuterLoop::Instance().SetSetpoint(Position(2.0F));
  RunMs(1000U);
  zexpect_within(MechanicalAngle() - kStart,
                 2.0F,
                 0.03F,
                 "moved %f rad",
                 static_cast<double>(MechanicalAngle() - kStart));
  const auto kStatus = OuterLoop::Instance().Status();
  zexpect_equal(kStatus.mode, ControlMode::kPosition);
  zexpect_within(kStatus.position.Value(), 2.0F, 0.03F);
  zexpect_true(kStatus.in_position);

  OuterLoop::Instance().SetSetpoint(Position(-1.0F));
  RunMs(1000U);
  zexpect_within(MechanicalAngle() - kStart,
                 -1.0F,
                 0.03F,
                 "moved %f rad, estimate %f rad",
                 static_cast<double>(MechanicalAngle() - kStart),
                 static_cast<double>(OuterLoop::Instance().Status().position.Value()));
}

ZTEST_F(sim_speed_control, test_position_speed_stays_below_the_limit) {
  OuterLoop::Instance().SetSetpoint(Position(20.0F));
  float fastest = 0.0F;
  float fastest_demand = 0.0F;
  for (unsigned ms = 0U; ms < 300U; ++ms) {
    RunMs(1U);
    fastest = std::max(fastest, std::abs(MechanicalSpeed()));
    fastest_demand = std::max(fastest_demand, std::abs(OuterLoop::Instance().Status().speed_reference.Value()));
  }
  // The position loop demands at most the 40 rad/s limit; the shaft follows
  // with the overshoot of the speed loop.
  zexpect_true(fastest_demand <= 40.0F + 1e-3F, "demand %f", static_cast<double>(fastest_demand));
  zexpect_true(fastest > 35.0F && fastest < 1.35F * 40.0F, "fastest %f", static_cast<double>(fastest));
}

ZTEST_SUITE(sim_speed_control, nullptr, Setup, Before, After, nullptr);
