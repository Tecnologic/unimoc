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
#include "angle_wrap.hpp"
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
  settings.pmsm_flux_obs_r_speed = 1.0_rad4_per_s2;
  settings.speed_kp = TorquePerAngularVelocity{1e-4F * kSpeedWc};
  settings.speed_ki = InverseTime{kSpeedWc / 4.0F};
  settings.speed_accel_limit = 2000.0_rad_per_s2;
  settings.motor_i_max = 20.0_A;
  settings.pos_kp_pos = 20.0_rad_per_s_per_rad;
  settings.pos_speed_limit = 40.0_rad_per_s;
  settings.pos_accel_limit = 1000.0_rad_per_s2;
  settings.pos_position_tolerance = 0.02_rad;
  settings.pos_speed_tolerance = 1.0_rad_per_s;
  return settings;
}

AngularVelocity MechanicalSpeed() { return sim::Plant().Output().mechanical_speed; }
Angle MechanicalAngle() { return sim::Plant().Output().mechanical_angle.AbsoluteAngle(); }

/// Half a Hall sector in mechanical angle: the uncertainty of the angle at standstill.
constexpr Angle kHalfSector = (kPi / 6.0F / static_cast<float>(kPolePairs)) * 1.0_rad;

/// True shaft angle of the drive's home (position zero). The drive counts the
/// position from the estimate at its start, which the Hall sector places only
/// within half a sector of the shaft; the edges correct the angle estimate
/// later, but not the home. Valid at standstill.
Angle TrueHome() {
  const auto& plant = sim::Plant().Output();
  const Angle kEstimateError = unimoc::observer::WrapAngle(plant.electrical_angle.AngleInRevolution() -
                                                           CurrentControlIsr::Instance().mech_obs.theta);
  return MechanicalAngle() - OuterLoop::Instance().Status().position -
         (kEstimateError / static_cast<float>(kPolePairs));
}

void RunMs(const unsigned milliseconds) { sim::AdvanceCycles(milliseconds * kCyclesPerMs); }

OuterSetpoint Speed(const AngularVelocity speed) {
  OuterSetpoint setpoint{};
  setpoint.active = true;
  setpoint.mode = ControlMode::kSpeed;
  setpoint.speed = speed;
  return setpoint;
}

OuterSetpoint Position(const Angle position) {
  OuterSetpoint setpoint{};
  setpoint.active = true;
  setpoint.mode = ControlMode::kPosition;
  setpoint.position = position;
  return setpoint;
}

/// The reference of an external trajectory at this instant.
OuterSetpoint Trajectory(const Angle position, const AngularVelocity speed, const Torque torque) {
  OuterSetpoint setpoint = Position(position);
  setpoint.trajectory = true;
  setpoint.speed = speed;
  setpoint.torque = torque;
  return setpoint;
}

}  // namespace

// The platform calls these directly (hot_path_interface.hpp). The control
// thread runs synchronously here, every OuterLoop::kCyclesPerStep cycles.
bool unimoc::hardware::SampleHandler() noexcept { return CurrentControlIsr::Instance().OnSample(); }
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
  zassert_true(unimoc::current_control::StartRuntime(settings, unimoc::hardware::kMotors[0], 60.0_A));
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
  zexpect_equal(CurrentControlIsr::Instance().state.i_ref.q, 0.0_A);
  zexpect_within(MechanicalSpeed(), 0.0_rad_per_s, 0.1_rad_per_s);
}

ZTEST_F(sim_speed_control, test_speed_step_reaches_the_setpoint) {
  OuterLoop::Instance().SetSetpoint(Speed(30.0_rad_per_s));
  RunMs(400U);
  zexpect_within(MechanicalSpeed(),
                 30.0_rad_per_s,
                 0.6_rad_per_s,
                 "speed %f",
                 static_cast<double>(MechanicalSpeed().Value()));
  const auto kStatus = OuterLoop::Instance().Status();
  zexpect_true(kStatus.active);
  zexpect_equal(kStatus.mode, ControlMode::kSpeed);
  zexpect_within(kStatus.speed, 30.0_rad_per_s, 0.6_rad_per_s);

  // And back through zero to the reverse direction.
  OuterLoop::Instance().SetSetpoint(Speed(-20.0_rad_per_s));
  RunMs(400U);
  zexpect_within(MechanicalSpeed(),
                 -20.0_rad_per_s,
                 0.6_rad_per_s,
                 "speed %f",
                 static_cast<double>(MechanicalSpeed().Value()));
  zexpect_false(CurrentControlIsr::Instance().state.overcurrent_tripped);
}

ZTEST_F(sim_speed_control, test_load_torque_is_rejected) {
  OuterLoop::Instance().SetSetpoint(Speed(25.0_rad_per_s));
  RunMs(300U);
  sim::Plant().SetInput(unimoc::sim::SimulationInput{0.2_Nm, Voltage{}});
  RunMs(500U);
  zexpect_within(MechanicalSpeed(),
                 25.0_rad_per_s,
                 0.6_rad_per_s,
                 "speed %f",
                 static_cast<double>(MechanicalSpeed().Value()));
  // 0.2 N m at 3/2 * 7 * 5 mWb = 52.5 mN m/A: about 3.8 A of q current.
  zexpect_within(OuterLoop::Instance().Status().current_reference.q, (0.2F / 0.0525F) * 1.0_A, 0.4_A);
}

ZTEST_F(sim_speed_control, test_current_limit_takes_effect) {
  auto settings = fixture->settings;
  settings.motor_i_max = 5.0_A;
  settings.speed_accel_limit = AngularAcceleration{};  // a step: the controller saturates
  Start(settings);
  OuterLoop::Instance().SetSetpoint(Speed(200.0_rad_per_s));
  Current largest;
  for (unsigned ms = 0U; ms < 50U; ++ms) {
    RunMs(1U);
    const auto& reference = CurrentControlIsr::Instance().state.i_ref;
    largest = Max(largest, reference.Length());
  }
  zexpect_true(largest <= 5.0_A + 1e-4_A, "largest reference %f A", static_cast<double>(largest.Value()));
  zexpect_within(largest, 5.0_A, 1e-3_A);
  // Limited torque: 5 A * 52.5 mN m/A on 1e-4 kg m^2 is 2625 rad/s^2, so 50 ms reach at most 131 rad/s.
  zexpect_true(MechanicalSpeed() > 50.0_rad_per_s && MechanicalSpeed() < 135.0_rad_per_s,
               "speed %f",
               static_cast<double>(MechanicalSpeed().Value()));
}

ZTEST_F(sim_speed_control, test_position_step_settles_in_position) {
  const Angle kStart = MechanicalAngle();
  // The planned move arrives within about 0.1 s. The hold is checked long
  // after the Hall observer has taken the standstill (0.5 s without an edge):
  // the held shaft must not start to drift or oscillate then.
  OuterLoop::Instance().SetSetpoint(Position(2.0_rad));
  RunMs(1000U);
  const Angle kHome = TrueHome();
  zexpect_within(kHome,
                 kStart,
                 kHalfSector + 0.005_rad,
                 "home %f rad from the start",
                 static_cast<double>((kHome - kStart).Value()));
  zexpect_within(MechanicalAngle() - kHome,
                 2.0_rad,
                 0.03_rad,
                 "moved %f rad",
                 static_cast<double>((MechanicalAngle() - kHome).Value()));
  const auto kStatus = OuterLoop::Instance().Status();
  zexpect_equal(kStatus.mode, ControlMode::kPosition);
  zexpect_within(kStatus.position, 2.0_rad, 0.03_rad);
  zexpect_true(kStatus.in_position);

  OuterLoop::Instance().SetSetpoint(Position(-1.0_rad));
  RunMs(1000U);
  zexpect_within(MechanicalAngle() - kHome,
                 -1.0_rad,
                 0.03_rad,
                 "moved %f rad, estimate %f rad",
                 static_cast<double>((MechanicalAngle() - kHome).Value()),
                 static_cast<double>(OuterLoop::Instance().Status().position.Value()));
}

ZTEST_F(sim_speed_control, test_position_speed_stays_below_the_limit) {
  OuterLoop::Instance().SetSetpoint(Position(20.0_rad));
  AngularVelocity fastest;
  AngularVelocity fastest_demand;
  for (unsigned ms = 0U; ms < 300U; ++ms) {
    RunMs(1U);
    fastest = Max(fastest, Abs(MechanicalSpeed()));
    fastest_demand = Max(fastest_demand, Abs(OuterLoop::Instance().Status().speed_reference));
  }
  // The planned move demands at most the 40 rad/s limit; the shaft follows
  // with the overshoot of the speed loop. At 1000 rad/s^2 the Hall observer
  // lags the shaft by up to 10 rad/s, so the shaft overshoots by about a third.
  zexpect_true(fastest_demand <= 40.0_rad_per_s + 1e-3_rad_per_s,
               "demand %f",
               static_cast<double>(fastest_demand.Value()));
  zexpect_true(fastest > 35.0_rad_per_s && fastest < 1.4F * 40.0_rad_per_s,
               "fastest %f",
               static_cast<double>(fastest.Value()));
}

ZTEST_F(sim_speed_control, test_planned_move_keeps_the_positioning_limits) {
  // 0.05 N m on 1e-4 kg m^2: the move accelerates with 500 rad/s^2, below the
  // 1000 rad/s^2 of the settings; it cruises at 40 rad/s.
  auto settings = fixture->settings;
  settings.pos_torque_limit = 0.05_Nm;
  Start(settings);
  OuterLoop::Instance().SetSetpoint(Position(20.0_rad));
  AngularVelocity fastest_demand;
  Torque largest_torque;
  unsigned arrival_ms = 0U;
  for (unsigned ms = 0U; ms < 1200U; ++ms) {
    RunMs(1U);
    const auto kStatus = OuterLoop::Instance().Status();
    fastest_demand = Max(fastest_demand, Abs(kStatus.speed_reference));
    largest_torque = Max(largest_torque, Abs(kStatus.torque_reference));
    if (arrival_ms == 0U && kStatus.in_position) {
      arrival_ms = ms;
    }
  }
  zexpect_true(fastest_demand <= 40.0_rad_per_s + 1e-3_rad_per_s,
               "demand %f",
               static_cast<double>(fastest_demand.Value()));
  zexpect_true(largest_torque <= 0.05_Nm + 1e-4_Nm, "torque %f", static_cast<double>(largest_torque.Value()));
  // 20 rad / 40 rad/s + 40 rad/s / 500 rad/s^2 = 0.58 s, plus the settling.
  zexpect_true(arrival_ms > 550U && arrival_ms < 800U, "in position after %u ms", arrival_ms);
  const auto kStatus = OuterLoop::Instance().Status();
  zexpect_within(kStatus.position, 20.0_rad, 0.03_rad);
  zexpect_true(kStatus.in_position);
}

ZTEST_F(sim_speed_control, test_external_trajectory_is_followed) {
  // The master ramps at 300 rad/s^2 to 30 rad/s and sends position, speed and
  // the torque J a of every instant; the drive follows without planning.
  constexpr AngularAcceleration kAccel = 300.0_rad_per_s2;
  constexpr AngularVelocity kCruise = 30.0_rad_per_s;
  constexpr Torque kAccelTorque = 1e-4_kg_m2 * kAccel;
  constexpr Time kStep = 1e-3_s;
  Angle position;
  AngularVelocity speed;
  Angle largest_error;
  for (unsigned ms = 0U; ms < 500U; ++ms) {
    const bool kAccelerating = speed < kCruise;
    speed = kAccelerating ? Min(kCruise, speed + (kAccel * kStep)) : speed;
    position += speed * kStep;
    OuterLoop::Instance().SetSetpoint(Trajectory(position, speed, kAccelerating ? kAccelTorque : Torque{}));
    RunMs(1U);
    if (ms > 400U) {
      largest_error = Max(largest_error, Abs(OuterLoop::Instance().Status().position - position));
    }
  }
  // In the cruise the shaft runs on the trajectory (the start carries the lag
  // of the Hall observer at low speed).
  zexpect_true(largest_error < 0.05_rad, "error %f rad", static_cast<double>(largest_error.Value()));
  zexpect_within(MechanicalSpeed(), kCruise, 1.0_rad_per_s);
  zexpect_equal(OuterLoop::Instance().Status().mode, ControlMode::kPosition);
}

ZTEST_F(sim_speed_control, test_homing_takes_the_stop_as_zero) {
  using unimoc::control::HomingState;
  auto settings = fixture->settings;
  settings.pos_homing_speed = 5.0_rad_per_s;
  settings.pos_homing_current = 4.0_A;
  Start(settings);
  OuterLoop::Instance().SetSetpoint(Position(0.0_rad));
  RunMs(200U);
  zexpect_equal(OuterLoop::Instance().Status().homing, HomingState::kIdle);

  OuterLoop::Instance().StartHoming();
  RunMs(300U);
  zexpect_equal(OuterLoop::Instance().Status().homing, HomingState::kSearching);
  zexpect_within(MechanicalSpeed(), 5.0_rad_per_s, 0.5_rad_per_s);

  // The stop: the shaft locks, the run pushes with at most the homing current.
  sim::Plant().SetShaftMode(unimoc::sim::ShaftMode::kImposedSpeed);
  const Angle kStop = MechanicalAngle();
  Current largest;
  for (unsigned ms = 0U; ms < 300U; ++ms) {
    RunMs(1U);
    largest = Max(largest, CurrentControlIsr::Instance().state.i_ref.Length());
  }
  zexpect_true(largest <= 4.0_A + 1e-3_A, "the homing current bounds the push");
  const auto kHomed = OuterLoop::Instance().Status();
  zexpect_equal(kHomed.homing, HomingState::kDone);
  zexpect_within(kHomed.position, 0.0_rad, 0.01_rad);

  // Held at the home while the setpoint stays; then in the frame of the stop.
  sim::Plant().SetShaftMode(unimoc::sim::ShaftMode::kFree);
  RunMs(300U);
  zexpect_within(MechanicalAngle() - kStop, 0.0_rad, kHalfSector + 0.03_rad);
  OuterLoop::Instance().SetSetpoint(Position(-2.0_rad));
  RunMs(1000U);
  zexpect_within(MechanicalAngle() - kStop, -2.0_rad, kHalfSector + 0.03_rad);
  zexpect_equal(OuterLoop::Instance().Status().homing, HomingState::kDone);
}

ZTEST_F(sim_speed_control, test_homing_needs_the_position_mode) {
  using unimoc::control::HomingState;
  OuterLoop::Instance().SetSetpoint(Speed(10.0_rad_per_s));
  RunMs(50U);
  OuterLoop::Instance().StartHoming();
  RunMs(10U);
  zexpect_equal(OuterLoop::Instance().Status().homing, HomingState::kFault);

  // A disengage ends a run; the next one starts again.
  OuterLoop::Instance().SetSetpoint(Position(0.0_rad));
  RunMs(50U);
  OuterLoop::Instance().StartHoming();
  RunMs(50U);
  zexpect_equal(OuterLoop::Instance().Status().homing, HomingState::kSearching);
  OuterLoop::Instance().SetSetpoint(OuterSetpoint{});
  RunMs(10U);
  zexpect_equal(OuterLoop::Instance().Status().homing, HomingState::kFault);
  zexpect_equal(OuterLoop::Instance().Status().current_reference.q, 0.0_A);
}

ZTEST_SUITE(sim_speed_control, nullptr, Setup, Before, After, nullptr);
