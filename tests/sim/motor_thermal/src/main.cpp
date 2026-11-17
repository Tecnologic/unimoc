/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file main.cpp
 *    @brief The winding temperature estimate of the outer loop against the heated plant.
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
#include "motor_simulation.hpp"
#include "outer_loop.hpp"
#include "power_stage_interface.hpp"
#include "settings_store.hpp"
#include "sim_control.hpp"
#include "slow_update.hpp"
#include "unimoc_test.hpp"

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

/// A motor that heats a hundred times faster than the hub motor (tau_w about
/// 0.6 s, tau_s about 12 s), so the scenarios run in seconds of virtual time.
unimoc::settings::NvmSettings MotorSettings(const unimoc::settings::NvmSettings& factory) {
  unimoc::settings::NvmSettings settings = factory;
  constexpr float kWc = 2.0F * kPi * 500.0F;
  settings.pole_pairs = 7U;
  settings.stator_r = 0.05_Ohm;
  settings.stator_r_temperature = 25.0_degC;
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
  settings.motor_i_max = 40.0_A;
  settings.thermal_ambient = 25.0_degC;
  settings.thermal_c_winding = 1.5_J_per_K;
  settings.thermal_c_stator = 15.0_J_per_K;
  settings.thermal_r_winding = 0.4_K_per_W;
  settings.thermal_r_stator = 0.8_K_per_W;
  settings.thermal_derate_start = 150.0_degC;
  settings.thermal_limit = 180.0_degC;
  settings.thermal_rs_speed_max = 100.0_rad_per_s;
  settings.thermal_rs_current_min = 3.0_A;
  settings.thermal_rs_correction_time = 0.0_s;
  return settings;
}

/// The plant of the settings with the winding heating, its network that of the settings.
unimoc::sim::SimulationParameters HeatedPlant(const unimoc::settings::NvmSettings& settings) {
  unimoc::sim::SimulationParameters parameters = unimoc::sim::ParametersFromSettings(settings);
  parameters.thermal.enabled = true;
  parameters.thermal.network.reference_temperature = settings.stator_r_temperature;
  parameters.thermal.network.winding_capacity = settings.thermal_c_winding;
  parameters.thermal.network.stator_capacity = settings.thermal_c_stator;
  parameters.thermal.network.winding_to_stator = settings.thermal_r_winding;
  parameters.thermal.network.stator_to_ambient = settings.thermal_r_stator;
  parameters.thermal.ambient = settings.thermal_ambient;
  parameters.thermal.start = settings.thermal_ambient;
  return parameters;
}

void RunMs(const unsigned milliseconds) { sim::AdvanceCycles(milliseconds * kCyclesPerMs); }

/// Runs for a time in blocks of a second (one advance is limited in length by nothing but its count).
void RunSeconds(const unsigned seconds) {
  for (unsigned second = 0U; second < seconds; ++second) {
    RunMs(1000U);
  }
}

/// A constant q current on the locked rotor.
OuterSetpoint Hold(const Current current) {
  OuterSetpoint setpoint{};
  setpoint.active = true;
  setpoint.mode = ControlMode::kTorque;
  setpoint.current = {Current{}, current};
  return setpoint;
}

Temperature PlantWinding() { return sim::Plant().Output().winding_temperature; }
Temperature EstimatedWinding() { return OuterLoop::Instance().Status().winding_temperature; }

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

struct sim_motor_thermal_fixture {
  unimoc::settings::SettingsStore store{unimoc::hardware::kSettings.GetSettingsProfile(),
                                        unimoc::hardware::kSettings.GetSettingsStorage(),
                                        unimoc::settings::SettingsSaveGate::Always()};
  unimoc::settings::NvmSettings settings{};
};

namespace {

void* Setup() {
  static sim_motor_thermal_fixture fixture;
  zassert_equal(fixture.store.Load(), unimoc::settings::SettingsStatus::kFactoryDefaults);
  fixture.settings = MotorSettings(fixture.store.GetSnapshot().Get());
  return &fixture;
}

/// Starts plant and controller; the estimate at the ambient, the plant at @p plant_temperature.
void Start(const unimoc::settings::NvmSettings& settings, const Temperature plant_temperature) {
  OuterLoop::Instance().SetSetpoint(OuterSetpoint{});
  kPowerStage.Disable();
  zassert_equal(unimoc::settings::settings_store_internal::Validate(settings,
                                                                    unimoc::hardware::kSettings
                                                                        .GetSettingsProfile()
                                                                        .capabilities),
                unimoc::settings::SettingsStatus::kSuccess);
  zassert_equal(sim::Plant().Configure(HeatedPlant(settings)), unimoc::sim::Status::kOk);
  sim::Plant().SetShaftMode(unimoc::sim::ShaftMode::kImposedSpeed, AngularVelocity{});
  sim::Plant().SetInput(unimoc::sim::SimulationInput{});
  sim::Reset();
  sim::Plant().SetMotorTemperature(plant_temperature);
  OuterLoop::Instance().RestartThermal();
  zassert_true(unimoc::current_control::StartRuntime(settings, unimoc::hardware::kMotors[0], 80.0_A));
  sim::SetFreeRunning(false);
  kPowerStage.ClearFault();
  zassert_true(kPowerStage.Enable());
  RunMs(10U);
}

void Before(void* /*unused*/) {
  sim::SetMotorSensor(false);
  OuterLoop::Instance().SetMotorSensorTemperature(std::nullopt);
  sim::ClearPhaseCurrentOverride();
  sim::SetExternalEnable(true);
}

void After(void* /*unused*/) {
  OuterLoop::Instance().SetSetpoint(OuterSetpoint{});
  kPowerStage.Disable();
}

}  // namespace

ZTEST_F(sim_motor_thermal, test_estimate_follows_the_heated_winding) {
  Start(fixture->settings, 25.0_degC);
  UNIMOC_EXPECT_NEAR(EstimatedWinding(), 25.0_degC, 0.5_degC);
  // 30 A on the locked rotor: about 68 W cold, a steady rise of about 100 K.
  OuterLoop::Instance().SetSetpoint(Hold(30.0_A));
  RunSeconds(3U);
  zexpect_true(PlantWinding() > 60.0_degC, "the winding heats");
  UNIMOC_EXPECT_NEAR(EstimatedWinding(), PlantWinding(), 2.0_degC);
  RunSeconds(10U);
  UNIMOC_EXPECT_NEAR(EstimatedWinding(), PlantWinding(), 3.0_degC);
  // The model's resistance is the plant's.
  UNIMOC_EXPECT_NEAR(OuterLoop::Instance().Status().stator_resistance,
                     sim::Plant().Output().stator_resistance,
                     1.5e-3_Ohm);
  // The flux observer works with the warm resistance, not with that of the settings.
  UNIMOC_EXPECT_NEAR(SlowUpdate::Instance().flux_obs.rs,
                     OuterLoop::Instance().Status().stator_resistance,
                     1e-4_Ohm);
  zexpect_true(SlowUpdate::Instance().flux_obs.rs > fixture->settings.stator_r * 1.1F, "the warm winding");
  // And it cools down with the plant.
  OuterLoop::Instance().SetSetpoint(OuterSetpoint{});
  RunSeconds(5U);
  UNIMOC_EXPECT_NEAR(EstimatedWinding(), PlantWinding(), 3.0_degC);
  zexpect_false(CurrentControlIsr::Instance().state.overcurrent_tripped);
}

ZTEST_F(sim_motor_thermal, test_resistance_pulls_a_cold_estimate_to_a_warm_motor) {
  // The motor is still warm from an earlier run; the drive starts at the ambient.
  unimoc::settings::NvmSettings settings = fixture->settings;
  settings.thermal_rs_correction_time = 1.0_s;
  Start(settings, 90.0_degC);
  UNIMOC_EXPECT_NEAR(EstimatedWinding(), 25.0_degC, 0.5_degC);
  // 10 A at standstill: little heating, enough current for the resistance.
  OuterLoop::Instance().SetSetpoint(Hold(10.0_A));
  RunSeconds(6U);
  UNIMOC_EXPECT_NEAR(EstimatedWinding(), PlantWinding(), 5.0_degC);
}

ZTEST_F(sim_motor_thermal, test_without_correction_a_cold_estimate_stays_cold) {
  Start(fixture->settings, 90.0_degC);
  OuterLoop::Instance().SetSetpoint(Hold(10.0_A));
  RunSeconds(6U);
  zexpect_true(PlantWinding() - EstimatedWinding() > 30.0_degC,
               "the model alone does not see the warm start");
}

ZTEST_F(sim_motor_thermal, test_a_motor_sensor_pulls_the_estimate_to_a_warm_motor) {
  // The motor is warm; the drive starts at the ambient, the stator resistance
  // correction is off: only the sensor on the stator can find the temperature.
  unimoc::settings::NvmSettings settings = fixture->settings;
  settings.thermal_sensor_correction_time = 1.0_s;
  Start(settings, 70.0_degC);
  sim::SetMotorSensor(true);
  UNIMOC_EXPECT_NEAR(EstimatedWinding(), 25.0_degC, 0.5_degC);
  for (unsigned step = 0U; step < 50U; ++step) {
    // As the drive does in its spin: the reading of the platform to the outer loop.
    OuterLoop::Instance().SetMotorSensorTemperature(
        unimoc::hardware::kMotors[0].GetTemperatureMeasurements().motor);
    RunMs(100U);
  }
  UNIMOC_EXPECT_NEAR(EstimatedWinding(), PlantWinding(), 2.0_degC);
  UNIMOC_EXPECT_NEAR(OuterLoop::Instance().Status().stator_temperature,
                     sim::Plant().Output().stator_temperature,
                     2.0_degC);
  // Without a reading the model runs on alone.
  sim::SetMotorSensor(false);
  OuterLoop::Instance().SetMotorSensorTemperature(
      unimoc::hardware::kMotors[0].GetTemperatureMeasurements().motor);
  RunMs(500U);
  UNIMOC_EXPECT_NEAR(EstimatedWinding(), PlantWinding(), 2.0_degC);
}

ZTEST_F(sim_motor_thermal, test_hot_winding_derates_the_current) {
  unimoc::settings::NvmSettings settings = fixture->settings;
  settings.thermal_derate_start = 50.0_degC;
  settings.thermal_limit = 70.0_degC;
  Start(settings, 25.0_degC);
  OuterLoop::Instance().SetSetpoint(Hold(30.0_A));
  RunSeconds(15U);
  const auto kStatus = OuterLoop::Instance().Status();
  zexpect_true(kStatus.thermally_derated);
  zexpect_true(kStatus.current_reference.q < 25.0_A, "the reference is derated");
  // The derating holds the winding in its band.
  zexpect_true(PlantWinding() > 50.0_degC);
  zexpect_true(PlantWinding() < 70.0_degC, "the winding stays below the limit");
  // The torque limits of the drive follow the derated current.
  zexpect_true(kStatus.torque_max < OuterLoop::MotorTorqueConstant(settings, MagneticFlux{}) * 25.0_A);
}

ZTEST_SUITE(sim_motor_thermal, nullptr, Setup, Before, After, nullptr);
