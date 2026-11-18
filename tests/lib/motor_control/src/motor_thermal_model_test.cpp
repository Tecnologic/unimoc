/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file motor_thermal_model_test.cpp
 *    @brief Tests for the two-mass thermal model of the motor winding.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "motor_thermal_model.hpp"
#include <zephyr/ztest.h>
#include <limits>
#include "unimoc_test.hpp"

namespace unimoc::control::test {

using namespace unit;

namespace {

/// The defaults of the plan: tau_w about 60 s, tau_s about 20 min.
constexpr MotorThermalModel::Parameters kParameters{.stator_resistance = 0.15_Ohm,
                                                    .reference_temperature = 25.0_degC,
                                                    .temperature_coefficient = kCopperTemperatureCoefficient,
                                                    .winding_capacity = 150.0_J_per_K,
                                                    .stator_capacity = 1500.0_J_per_K,
                                                    .winding_to_stator = 0.4_K_per_W,
                                                    .stator_to_ambient = 0.8_K_per_W,
                                                    .derate_start = 100.0_degC,
                                                    .limit = 130.0_degC};

/// A model with a resistance that does not follow the temperature.
constexpr MotorThermalModel::Parameters ConstantResistance() {
  MotorThermalModel::Parameters parameters = kParameters;
  parameters.temperature_coefficient = 0.0_per_K;
  return parameters;
}

constexpr MotorThermalModel::CurrentSquared Squared(const Current current) { return current * current; }

/// Runs the model for a time in steps of a given length.
void Run(MotorThermalModel& model, const Current current, const Time duration, const Time dt) {
  for (Time time{}; time + (dt * 0.5F) < duration; time += dt) {
    model.Step(Squared(current), 25.0_degC, dt);
  }
}

}  // namespace

ZTEST(motor_thermal_model, test_stator_resistance_follows_the_winding_temperature) {
  MotorThermalModel model{kParameters, 25.0_degC};
  UNIMOC_EXPECT_FLOAT_EQ(model.StatorResistance(), 0.15_Ohm);
  // 100 K warmer: 39.3 % more.
  UNIMOC_EXPECT_NEAR(model.StatorResistanceAt(125.0_degC), 0.20895_Ohm, 1e-6_Ohm);
  // The inverse gives the temperature back.
  UNIMOC_EXPECT_NEAR(model.TemperatureFromResistance(0.20895_Ohm), 125.0_degC, 1e-3_degC);
  UNIMOC_EXPECT_NEAR(model.TemperatureFromResistance(0.15_Ohm), 25.0_degC, 1e-5_degC);
  // Never below zero, even far below the range of the coefficient.
  UNIMOC_EXPECT_FLOAT_EQ(model.StatorResistanceAt(-500.0_degC), 0.0_Ohm);
}

ZTEST(motor_thermal_model, test_copper_loss_of_three_phases) {
  const MotorThermalModel kModel{kParameters, 25.0_degC};
  // 3/2 * 0.15 Ohm * (15 A)^2 = 50.625 W.
  UNIMOC_EXPECT_NEAR(kModel.CopperLoss(Squared(15.0_A)), 50.625_W, 1e-3_W);
}

ZTEST(motor_thermal_model, test_without_current_the_temperature_stays) {
  MotorThermalModel model{kParameters, 25.0_degC};
  Run(model, 0.0_A, 100.0_s, MotorThermalModel::kStepPeriod);
  UNIMOC_EXPECT_FLOAT_EQ(model.Winding(), 25.0_degC);
  UNIMOC_EXPECT_FLOAT_EQ(model.Stator(), 25.0_degC);
}

ZTEST(motor_thermal_model, test_steady_state_is_the_series_thermal_resistance) {
  MotorThermalModel model{ConstantResistance(), 25.0_degC};
  // 50.625 W: the winding settles 1.2 K/W above, the stator 0.8 K/W above the ambient.
  Run(model, 15.0_A, 20000.0_s, 1.0_s);
  UNIMOC_EXPECT_NEAR(model.Winding(), 25.0_degC + (50.625_W * 1.2_K_per_W), 0.05_degC);
  UNIMOC_EXPECT_NEAR(model.Stator(), 25.0_degC + (50.625_W * 0.8_K_per_W), 0.05_degC);
}

ZTEST(motor_thermal_model, test_hot_winding_settles_higher_with_the_resistance_coefficient) {
  MotorThermalModel model{kParameters, 25.0_degC};
  Run(model, 15.0_A, 20000.0_s, 1.0_s);
  // dT = 1.2 K/W * 50.625 W * (1 + alpha dT) = 60.75 K (1 + alpha dT),
  // so dT = 60.75 K / (1 - 60.75 K * alpha).
  const Temperature kExpected = 25.0_degC + (60.75_degC / (1.0F - (60.75F * 3.93e-3F)));
  UNIMOC_EXPECT_NEAR(model.Winding(), kExpected, 0.1_degC);
  zexpect_true(model.Winding() > 25.0_degC + 60.75_degC, "the warm winding loses more");
}

ZTEST(motor_thermal_model, test_winding_heats_with_its_own_time_constant_first) {
  MotorThermalModel model{ConstantResistance(), 25.0_degC};
  // A short heavy load: the winding heats far faster than the stator.
  Run(model, 40.0_A, 10.0_s, MotorThermalModel::kStepPeriod);
  // 3/2 * 0.15 * 1600 = 360 W into 150 J/K: at most 24 K in 10 s, a little
  // less through the coupling to the stator.
  zexpect_true(model.Winding() > 25.0_degC + 20.0_degC, "the winding heats fast");
  zexpect_true(model.Winding() < 25.0_degC + 24.0_degC, "the stator takes some heat");
  zexpect_true(model.Stator() < 25.0_degC + 2.0_degC, "the housing hardly moves");
}

ZTEST(motor_thermal_model, test_long_steps_agree_with_short_steps) {
  MotorThermalModel fine{kParameters, 25.0_degC};
  MotorThermalModel coarse{kParameters, 25.0_degC};
  Run(fine, 20.0_A, 600.0_s, MotorThermalModel::kStepPeriod);
  Run(coarse, 20.0_A, 600.0_s, 5.0_s);
  // A step of 5 s (a twelfth of tau_w) stays stable and close.
  UNIMOC_EXPECT_NEAR(coarse.Winding(), fine.Winding(), 1.5_degC);
  // Even one step far longer than every time constant stays bounded:
  // backward Euler goes to the steady state (within tau/dt), it does not overshoot.
  MotorThermalModel jump{ConstantResistance(), 25.0_degC};
  jump.Step(Squared(15.0_A), 25.0_degC, 1.0e6_s);
  UNIMOC_EXPECT_NEAR(jump.Winding(), 25.0_degC + (50.625_W * 1.2_K_per_W), 0.1_degC);
  zexpect_true(jump.Winding() < 25.0_degC + (50.625_W * 1.2_K_per_W), "no overshoot");
}

ZTEST(motor_thermal_model, test_cooling_back_to_the_ambient) {
  MotorThermalModel model{kParameters, 25.0_degC};
  Run(model, 15.0_A, 3000.0_s, 1.0_s);
  zexpect_true(model.Winding() > 60.0_degC, "heated");
  Run(model, 0.0_A, 20000.0_s, 1.0_s);
  UNIMOC_EXPECT_NEAR(model.Winding(), 25.0_degC, 0.05_degC);
  UNIMOC_EXPECT_NEAR(model.Stator(), 25.0_degC, 0.05_degC);
}

ZTEST(motor_thermal_model, test_correction_moves_both_masses) {
  MotorThermalModel model{kParameters, 25.0_degC};
  Run(model, 15.0_A, 60.0_s, MotorThermalModel::kStepPeriod);
  const Temperature kGradient = model.Winding() - model.Stator();
  // The winding measured 20 K warmer; half of it is taken over.
  model.Correct(model.Winding() + 20.0_degC, 0.5_ratio);
  UNIMOC_EXPECT_NEAR(model.Winding() - model.Stator(), kGradient, 1e-4_degC);
  // The weight is clamped to one: the measurement at most replaces the estimate.
  model.Correct(90.0_degC, 3.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(model.Winding(), 90.0_degC);
  // A negative weight changes nothing.
  model.Correct(10.0_degC, -1.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(model.Winding(), 90.0_degC);
}

ZTEST(motor_thermal_model, test_stator_correction_keeps_the_gradient) {
  MotorThermalModel model{kParameters, 25.0_degC};
  Run(model, 15.0_A, 60.0_s, MotorThermalModel::kStepPeriod);
  const Temperature kGradient = model.Winding() - model.Stator();
  zexpect_true(kGradient > 5.0_degC, "the winding leads the stator");
  // A sensor on the housing reads 40 degC: the whole estimate moves with it.
  model.CorrectStator(40.0_degC, 1.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(model.Stator(), 40.0_degC);
  UNIMOC_EXPECT_NEAR(model.Winding(), 40.0_degC + kGradient, 1e-4_degC);
  // A partial weight takes a share; NaN changes nothing.
  model.CorrectStator(50.0_degC, 0.25_ratio);
  UNIMOC_EXPECT_NEAR(model.Stator(), 42.5_degC, 1e-4_degC);
  model.CorrectStator(Temperature{std::numeric_limits<float>::quiet_NaN()}, 1.0_ratio);
  UNIMOC_EXPECT_NEAR(model.Stator(), 42.5_degC, 1e-4_degC);
}

ZTEST(motor_thermal_model, test_current_limit_derating) {
  MotorThermalModel model{kParameters, 25.0_degC};
  UNIMOC_EXPECT_FLOAT_EQ(model.CurrentLimitScale(), 1.0_ratio);
  model.Reset(100.0_degC);
  UNIMOC_EXPECT_FLOAT_EQ(model.CurrentLimitScale(), 1.0_ratio);
  model.Reset(115.0_degC);
  UNIMOC_EXPECT_NEAR(model.CurrentLimitScale(), 0.5_ratio, 1e-5_ratio);
  model.Reset(130.0_degC);
  UNIMOC_EXPECT_FLOAT_EQ(model.CurrentLimitScale(), 0.0_ratio);
  model.Reset(200.0_degC);
  UNIMOC_EXPECT_FLOAT_EQ(model.CurrentLimitScale(), 0.0_ratio);
}

ZTEST(motor_thermal_model, test_invalid_parameters_or_steps_keep_the_state) {
  MotorThermalModel::Parameters parameters = kParameters;
  parameters.winding_capacity = HeatCapacity{};
  MotorThermalModel model{parameters, 40.0_degC};
  zexpect_false(parameters.Valid());
  model.Step(Squared(15.0_A), 25.0_degC, 1.0_s);
  UNIMOC_EXPECT_FLOAT_EQ(model.Winding(), 40.0_degC);

  MotorThermalModel valid{kParameters, 40.0_degC};
  valid.Step(Squared(15.0_A), 25.0_degC, 0.0_s);
  valid.Step(Squared(15.0_A), 25.0_degC, -1.0_s);
  UNIMOC_EXPECT_FLOAT_EQ(valid.Winding(), 40.0_degC);
}

ZTEST_SUITE(motor_thermal_model, nullptr, nullptr, nullptr, nullptr, nullptr);

}  // namespace unimoc::control::test
