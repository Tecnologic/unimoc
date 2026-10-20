/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file mechanical_model_test.cpp
 *	@brief Tests of the rigid shaft model.
 *
 *	SPDX-License-Identifier: GPL-3.0-or-later
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */

#include "mechanical_model.hpp"
#include <zephyr/ztest.h>
#include "unimoc_test.hpp"

using namespace unimoc::unit;
using unimoc::sim::MechanicalModel;
using unimoc::sim::MechanicalParameters;
using unimoc::sim::ShaftMode;
using unimoc::sim::Status;
using unimoc::system::RotorAngle;

namespace {

constexpr Time kDt{1.0e-4F};

MechanicalParameters Parameters(TorquePerAngularVelocity viscous, Torque coulomb) {
  return MechanicalParameters{1.0e-3_kg_m2, viscous, coulomb};
}

}  // namespace

ZTEST(mechanical_model, test_validate_rejects_invalid_parameters) {
  zexpect_equal(MechanicalModel(Parameters({}, {})).Validate(), Status::kOk);
  zexpect_equal(MechanicalModel(MechanicalParameters{Inertia{}, {}, {}}).Validate(), Status::kInvalidInertia);
  zexpect_equal(MechanicalModel(Parameters(TorquePerAngularVelocity{-1.0F}, {})).Validate(),
                Status::kInvalidFriction);
  zexpect_equal(MechanicalModel(Parameters({}, Torque{-1.0F})).Validate(), Status::kInvalidFriction);
}

ZTEST(mechanical_model, test_constant_torque_accelerates_uniformly) {
  MechanicalModel model(Parameters({}, {}));
  for (int step = 0; step < 1000; ++step) {
    model.Step(0.1_Nm, Torque{}, kDt);
  }
  // a = 100 rad/s^2 for 0.1 s.
  UNIMOC_EXPECT_NEAR(model.Speed().Value(), 10.0F, 1.0e-3F);
  UNIMOC_EXPECT_NEAR(model.Angle().AbsoluteAngle().Value(), 0.5F, 1.0e-3F);
}

ZTEST(mechanical_model, test_load_torque_opposes_motor_torque) {
  MechanicalModel model(Parameters({}, {}));
  for (int step = 0; step < 100; ++step) {
    model.Step(0.1_Nm, 0.3_Nm, kDt);
  }
  UNIMOC_EXPECT_NEAR(model.Speed().Value(), -2.0F, 1.0e-4F);
}

ZTEST(mechanical_model, test_viscous_friction_limits_speed) {
  MechanicalModel model(Parameters(1.0e-3_Nm_s_per_rad, {}));
  // tau = J / B = 1 s; run ten time constants.
  for (int step = 0; step < 100'000; ++step) {
    model.Step(0.1_Nm, Torque{}, kDt);
  }
  UNIMOC_EXPECT_NEAR(model.Speed().Value(), 100.0F, 0.1F);
}

ZTEST(mechanical_model, test_coulomb_friction_holds_and_stops_the_shaft) {
  MechanicalModel model(Parameters({}, 0.05_Nm));
  for (int step = 0; step < 100; ++step) {
    model.Step(0.04_Nm, Torque{}, kDt);
  }
  UNIMOC_EXPECT_FLOAT_EQ(model.Speed().Value(), 0.0F);

  model.SetState(RotorAngle{}, 1.0_rad_per_s);
  // Deceleration 50 rad/s^2: the shaft stops after 20 ms and stays at rest.
  for (int step = 0; step < 1000; ++step) {
    model.Step(Torque{}, Torque{}, kDt);
  }
  UNIMOC_EXPECT_FLOAT_EQ(model.Speed().Value(), 0.0F);
  UNIMOC_EXPECT_NEAR(model.Angle().AbsoluteAngle().Value(), 0.01F, 1.0e-4F);
}

ZTEST(mechanical_model, test_imposed_speed_ignores_torque) {
  MechanicalModel model(Parameters({}, {}));
  model.SetShaftMode(ShaftMode::kImposedSpeed, 20.0_rad_per_s);
  for (int step = 0; step < 1000; ++step) {
    model.Step(5.0_Nm, Torque{}, kDt);
  }
  UNIMOC_EXPECT_FLOAT_EQ(model.Speed().Value(), 20.0F);
  UNIMOC_EXPECT_NEAR(model.Angle().AbsoluteAngle().Value(), 2.0F, 1.0e-4F);

  model.SetShaftMode(ShaftMode::kImposedSpeed, AngularVelocity{});
  model.Step(5.0_Nm, Torque{}, kDt);
  UNIMOC_EXPECT_FLOAT_EQ(model.Speed().Value(), 0.0F);
}

ZTEST(mechanical_model, test_electrical_angle_is_mechanical_angle_times_pole_pairs) {
  MechanicalModel model(Parameters({}, {}));
  model.SetState(RotorAngle::FromAngle(Angle{7.0F}), AngularVelocity{});
  const RotorAngle kElectrical = model.ElectricalAngle(5U);
  zexpect_equal(kElectrical.AbsoluteRaw(), model.Angle().AbsoluteRaw() * 5);
  UNIMOC_EXPECT_NEAR(kElectrical.AbsoluteAngle().Value(), 35.0F, 1.0e-4F);
}

ZTEST_SUITE(mechanical_model, nullptr, nullptr, nullptr, nullptr, nullptr);
