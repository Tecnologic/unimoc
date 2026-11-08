/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file eesm_model_test.cpp
 *	@brief Tests of the electrical EESM model: field build-up and decay, torque with and without
 * excitation.
 *
 *	SPDX-License-Identifier: GPL-3.0-or-later
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */
#include "eesm_model.hpp"
#include <zephyr/ztest.h>
#include "unimoc_test.hpp"

using namespace unimoc::unit;
using unimoc::sim::EesmModel;
using unimoc::sim::EesmParameters;
using unimoc::sim::ElectricalInput;
using unimoc::sim::Status;
using unimoc::system::MotorType;
using unimoc::system::Rotor;
using unimoc::system::RotorAngle;
using unimoc::system::Stator;

namespace {

constexpr Time kDt = 25.0_us;

/// Field time constant L_f / R_f = 25 ms.
EesmParameters TestParameters() {
  EesmParameters parameters{};
  parameters.pole_pairs = 4U;
  parameters.stator_resistance = 0.1_Ohm;
  parameters.l_d = 0.3_mH;
  parameters.l_q = 0.2_mH;
  parameters.l_m = 2.0_mH;
  parameters.field_resistance = 2.0_Ohm;
  parameters.field_inductance = 50.0_mH;
  return parameters;
}

int Steps(const float seconds) { return static_cast<int>(seconds / 25.0e-6F); }

void Run(EesmModel& model, const ElectricalInput& input, const int steps) {
  for (int step = 0; step < steps; ++step) {
    model.Step(input, kDt);
  }
}

ElectricalInput OpenStator(const Voltage field_voltage) {
  return ElectricalInput{Stator<Voltage>{}, RotorAngle{}, AngularVelocity{}, false, field_voltage};
}

/// A standing rotor with a rotor-frame stator voltage and a field voltage.
ElectricalInput Standing(const Rotor<Voltage>& voltage, const Voltage field_voltage) {
  return ElectricalInput{voltage.ToStator(RotorAngle{}),
                         RotorAngle{},
                         AngularVelocity{},
                         true,
                         field_voltage};
}

}  // namespace

ZTEST(eesm_model, test_validate_rejects_invalid_parameters) {
  zexpect_equal(EesmModel(TestParameters()).Validate(), Status::kOk);
  zexpect_equal(EesmModel::Type(), MotorType::kEesm);
  EesmParameters parameters = TestParameters();
  parameters.field_resistance = Resistance{};
  zexpect_equal(EesmModel(parameters).Validate(), Status::kInvalidResistance);
  parameters = TestParameters();
  parameters.field_inductance = 1.0_mH;  // 3/2 L_m^2 > L_d L_f
  zexpect_equal(EesmModel(parameters).Validate(), Status::kInvalidInductance);
}

ZTEST(eesm_model, test_the_field_builds_up_and_decays_with_its_time_constant) {
  EesmModel model(TestParameters());
  // 10 V on 2 Ohm: 5 A after the build-up, 63 % after one time constant (25 ms).
  Run(model, OpenStator(10.0_V), Steps(0.025F));
  UNIMOC_EXPECT_NEAR(model.Output().field_current, 5.0_A * 0.632F, 0.02_A);
  Run(model, OpenStator(10.0_V), Steps(0.5F));
  UNIMOC_EXPECT_NEAR(model.Output().field_current, 5.0_A, 1e-3_A);
  zexpect_equal(model.Output().rotor_current, Rotor<Current>{});
  Run(model, OpenStator(Voltage{}), Steps(0.025F));
  UNIMOC_EXPECT_NEAR(model.Output().field_current, 5.0_A * 0.368F, 0.02_A);
}

ZTEST(eesm_model, test_the_torque_needs_the_excitation) {
  // q current 10 A (1 V on 0.1 Ohm) at a standing rotor.
  EesmModel excited(TestParameters());
  excited.SetFieldCurrent(5.0_A);
  Run(excited, Standing({Voltage{}, 1.0_V}, 10.0_V), Steps(0.2F));
  UNIMOC_EXPECT_NEAR(excited.Output().rotor_current.q, 10.0_A, 1e-3_A);
  UNIMOC_EXPECT_NEAR(excited.Output().field_current, 5.0_A, 1e-3_A);
  // 3/2 p L_m i_f i_q = 1.5 * 4 * 2 mH * 5 A * 10 A = 0.6 N m.
  UNIMOC_EXPECT_NEAR(excited.Output().torque, 0.6_Nm, 1e-3_Nm);

  EesmModel unexcited(TestParameters());
  Run(unexcited, Standing({Voltage{}, 1.0_V}, Voltage{}), Steps(0.2F));
  UNIMOC_EXPECT_NEAR(unexcited.Output().rotor_current.q, 10.0_A, 1e-3_A);
  UNIMOC_EXPECT_NEAR(unexcited.Output().torque, Torque{}, 1e-4_Nm);
}

ZTEST(eesm_model, test_a_d_current_step_couples_into_the_field_winding) {
  EesmModel model(TestParameters());
  model.SetFieldCurrent(5.0_A);
  // A d current step induces a transient opposite field current; the field returns to v_f / R_f.
  const ElectricalInput kStep = Standing({1.0_V, Voltage{}}, 10.0_V);
  Run(model, kStep, Steps(0.002F));
  zexpect_true(model.Output().field_current < 5.0_A - 0.1_A, "the flux of the d axis is kept at first");
  Run(model, kStep, Steps(0.5F));
  UNIMOC_EXPECT_NEAR(model.Output().field_current, 5.0_A, 1e-3_A);
  UNIMOC_EXPECT_NEAR(model.Output().rotor_current.d, 10.0_A, 1e-3_A);
}

ZTEST(eesm_model, test_the_back_emf_follows_the_field_current) {
  // Turning at 500 rad/s electrical with psi_f = L_m i_f = 10 mWb: a q voltage of 5 V cancels the back-EMF.
  EesmModel model(TestParameters());
  model.SetFieldCurrent(5.0_A);
  const AngularVelocity kSpeed = 500.0_rad_per_s;
  const RotorAngle kAngle{};
  const ElectricalInput kInput{Rotor<Voltage>{Voltage{}, 5.0_V}.ToStator(kAngle),
                               kAngle,
                               kSpeed,
                               true,
                               10.0_V};
  Run(model, kInput, Steps(0.2F));
  UNIMOC_EXPECT_NEAR(model.Output().rotor_current.d, Current{}, 1e-3_A);
  UNIMOC_EXPECT_NEAR(model.Output().rotor_current.q, Current{}, 1e-3_A);
}

ZTEST_SUITE(eesm_model, nullptr, nullptr, nullptr, nullptr, nullptr);
