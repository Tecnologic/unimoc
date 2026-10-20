/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file pmsm_model_test.cpp
 *	@brief Tests of the electrical PMSM model.
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */

#include "pmsm_model.hpp"
#include <zephyr/ztest.h>
#include "unimoc_test.hpp"

using namespace unimoc::unit;
using unimoc::sim::ElectricalInput;
using unimoc::sim::PmsmModel;
using unimoc::sim::PmsmParameters;
using unimoc::sim::Status;
using unimoc::system::Rotor;
using unimoc::system::RotorAngle;
using unimoc::system::Stator;

namespace {

constexpr Time kDt{25.0e-6F};

PmsmParameters TestParameters() {
  PmsmParameters parameters{};
  parameters.pole_pairs = 4U;
  parameters.stator_resistance = 0.1_Ohm;
  parameters.l_d = 0.2_mH;
  parameters.l_q = 0.3_mH;
  parameters.flux_pm = 10.0_mWb;
  return parameters;
}

/// Applies a rotor-frame voltage at a fixed angle and speed for a number of steps.
void RunSteps(PmsmModel& model,
              const Rotor<Voltage>& voltage,
              const RotorAngle& angle,
              AngularVelocity speed,
              int steps) {
  const ElectricalInput kInput{voltage.ToStator(angle), angle, speed, true, Voltage{}};
  for (int step = 0; step < steps; ++step) {
    model.Step(kInput, kDt);
  }
}

}  // namespace

ZTEST(pmsm_model, test_validate_rejects_invalid_parameters) {
  zexpect_equal(PmsmModel(TestParameters()).Validate(), Status::kOk);

  PmsmParameters parameters = TestParameters();
  parameters.pole_pairs = 0U;
  zexpect_equal(PmsmModel(parameters).Validate(), Status::kInvalidPolePairs);

  parameters = TestParameters();
  parameters.stator_resistance = 0.0_Ohm;
  zexpect_equal(PmsmModel(parameters).Validate(), Status::kInvalidResistance);

  parameters = TestParameters();
  parameters.l_q = Inductance{-1.0e-3F};
  zexpect_equal(PmsmModel(parameters).Validate(), Status::kInvalidInductance);

  parameters = TestParameters();
  parameters.flux_pm = MagneticFlux{-0.1F};
  zexpect_equal(PmsmModel(parameters).Validate(), Status::kInvalidFlux);
}

ZTEST(pmsm_model, test_locked_rotor_settles_at_ohmic_current) {
  PmsmModel model(TestParameters());
  const RotorAngle kAngle = RotorAngle::FromAngle(0.7_rad);
  // Ten time constants of the slower q axis (3 ms).
  RunSteps(model, Rotor<Voltage>(1.0_V, -0.5_V), kAngle, AngularVelocity{}, 1200);

  UNIMOC_EXPECT_NEAR(model.Output().rotor_current.d.Value(), 10.0F, 1.0e-2F);
  UNIMOC_EXPECT_NEAR(model.Output().rotor_current.q.Value(), -5.0F, 1.0e-2F);
}

ZTEST(pmsm_model, test_step_response_follows_electrical_time_constant) {
  PmsmModel model(TestParameters());
  // tau_d = L_d / R = 2 ms = 80 steps.
  RunSteps(model, Rotor<Voltage>(1.0_V, 0.0_V), RotorAngle{}, AngularVelocity{}, 80);

  // 1 - e^-1 of the final 10 A; the implicit step lags slightly.
  UNIMOC_EXPECT_NEAR(model.Output().rotor_current.d.Value(), 6.32F, 0.05F);
  UNIMOC_EXPECT_FLOAT_EQ(model.Output().rotor_current.q.Value(), 0.0F);
}

ZTEST(pmsm_model, test_steady_state_operating_point_is_stationary) {
  const PmsmParameters kParameters = TestParameters();
  PmsmModel model(kParameters);
  const AngularVelocity kOmega{1000.0F};
  const Rotor<Current> kCurrent(-5.0_A, 20.0_A);
  model.SetCurrent(kCurrent);

  // v_d = R i_d - w L_q i_q, v_q = R i_q + w (L_d i_d + psi).
  const Voltage kVd =
      (kCurrent.d * kParameters.stator_resistance) - (kOmega * (kParameters.l_q * kCurrent.q));
  const Voltage kVq = (kCurrent.q * kParameters.stator_resistance) +
                      (kOmega * ((kParameters.l_d * kCurrent.d) + kParameters.flux_pm));
  RunSteps(model, Rotor<Voltage>(kVd, kVq), RotorAngle::FromAngle(-2.0_rad), kOmega, 400);

  UNIMOC_EXPECT_NEAR(model.Output().rotor_current.d.Value(), kCurrent.d.Value(), 0.05F);
  UNIMOC_EXPECT_NEAR(model.Output().rotor_current.q.Value(), kCurrent.q.Value(), 0.05F);
}

ZTEST(pmsm_model, test_torque_includes_magnet_and_reluctance_parts) {
  PmsmModel model(TestParameters());
  model.SetCurrent(Rotor<Current>(-10.0_A, 20.0_A));

  // 3/2 * 4 * (0.01 + (0.2e-3 - 0.3e-3) * -10) * 20 = 1.32 Nm
  UNIMOC_EXPECT_NEAR(model.Output().torque.Value(), 1.32F, 1.0e-5F);
}

ZTEST(pmsm_model, test_open_stator_forces_zero_current) {
  PmsmModel model(TestParameters());
  model.SetCurrent(Rotor<Current>(3.0_A, 4.0_A));
  model.Step(ElectricalInput{Stator<Voltage>(5.0_V, 5.0_V),
                             RotorAngle{},
                             AngularVelocity{500.0F},
                             false,
                             Voltage{}},
             kDt);

  zexpect_equal(model.Output().rotor_current, Rotor<Current>{});
  UNIMOC_EXPECT_FLOAT_EQ(model.Output().torque.Value(), 0.0F);
}

ZTEST(pmsm_model, test_short_circuit_current_matches_steady_state_solution) {
  PmsmModel model(TestParameters());
  RunSteps(model, Rotor<Voltage>{}, RotorAngle{}, AngularVelocity{5000.0F}, 4000);

  // i_d = -w^2 L_q psi / (R^2 + w^2 L_d L_q), i_q = -w R psi / (R^2 + w^2 L_d L_q)
  constexpr float kDenominator = (0.1F * 0.1F) + (5000.0F * 5000.0F * 0.2e-3F * 0.3e-3F);
  UNIMOC_EXPECT_NEAR(model.Output().rotor_current.d.Value(),
                     -(5000.0F * 5000.0F * 0.3e-3F * 0.01F) / kDenominator,
                     0.05F);
  UNIMOC_EXPECT_NEAR(model.Output().rotor_current.q.Value(), -(5000.0F * 0.1F * 0.01F) / kDenominator, 0.05F);
}

ZTEST_SUITE(pmsm_model, nullptr, nullptr, nullptr, nullptr, nullptr);
