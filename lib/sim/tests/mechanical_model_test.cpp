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
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */

#include "mechanical_model.hpp"
#include <gtest/gtest.h>

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

TEST(MechanicalModelTest, ValidateRejectsInvalidParameters) {
  EXPECT_EQ(MechanicalModel(Parameters({}, {})).Validate(), Status::kOk);
  EXPECT_EQ(MechanicalModel(MechanicalParameters{Inertia{}, {}, {}}).Validate(), Status::kInvalidInertia);
  EXPECT_EQ(MechanicalModel(Parameters(TorquePerAngularVelocity{-1.0F}, {})).Validate(),
            Status::kInvalidFriction);
  EXPECT_EQ(MechanicalModel(Parameters({}, Torque{-1.0F})).Validate(), Status::kInvalidFriction);
}

TEST(MechanicalModelTest, ConstantTorqueAcceleratesUniformly) {
  MechanicalModel model(Parameters({}, {}));
  for (int step = 0; step < 1000; ++step) {
    model.Step(0.1_Nm, Torque{}, kDt);
  }
  // a = 100 rad/s^2 for 0.1 s.
  EXPECT_NEAR(model.Speed().Value(), 10.0F, 1.0e-3F);
  EXPECT_NEAR(model.Angle().AbsoluteAngle().Value(), 0.5F, 1.0e-3F);
}

TEST(MechanicalModelTest, LoadTorqueOpposesMotorTorque) {
  MechanicalModel model(Parameters({}, {}));
  for (int step = 0; step < 100; ++step) {
    model.Step(0.1_Nm, 0.3_Nm, kDt);
  }
  EXPECT_NEAR(model.Speed().Value(), -2.0F, 1.0e-4F);
}

TEST(MechanicalModelTest, ViscousFrictionLimitsSpeed) {
  MechanicalModel model(Parameters(1.0e-3_Nm_s_per_rad, {}));
  // tau = J / B = 1 s; run ten time constants.
  for (int step = 0; step < 100'000; ++step) {
    model.Step(0.1_Nm, Torque{}, kDt);
  }
  EXPECT_NEAR(model.Speed().Value(), 100.0F, 0.1F);
}

TEST(MechanicalModelTest, CoulombFrictionHoldsAndStopsTheShaft) {
  MechanicalModel model(Parameters({}, 0.05_Nm));
  for (int step = 0; step < 100; ++step) {
    model.Step(0.04_Nm, Torque{}, kDt);
  }
  EXPECT_FLOAT_EQ(model.Speed().Value(), 0.0F);

  model.SetState(RotorAngle{}, 1.0_rad_per_s);
  // Deceleration 50 rad/s^2: the shaft stops after 20 ms and stays at rest.
  for (int step = 0; step < 1000; ++step) {
    model.Step(Torque{}, Torque{}, kDt);
  }
  EXPECT_FLOAT_EQ(model.Speed().Value(), 0.0F);
  EXPECT_NEAR(model.Angle().AbsoluteAngle().Value(), 0.01F, 1.0e-4F);
}

TEST(MechanicalModelTest, ImposedSpeedIgnoresTorque) {
  MechanicalModel model(Parameters({}, {}));
  model.SetShaftMode(ShaftMode::kImposedSpeed, 20.0_rad_per_s);
  for (int step = 0; step < 1000; ++step) {
    model.Step(5.0_Nm, Torque{}, kDt);
  }
  EXPECT_FLOAT_EQ(model.Speed().Value(), 20.0F);
  EXPECT_NEAR(model.Angle().AbsoluteAngle().Value(), 2.0F, 1.0e-4F);

  model.SetShaftMode(ShaftMode::kImposedSpeed, AngularVelocity{});
  model.Step(5.0_Nm, Torque{}, kDt);
  EXPECT_FLOAT_EQ(model.Speed().Value(), 0.0F);
}

TEST(MechanicalModelTest, ElectricalAngleIsMechanicalAngleTimesPolePairs) {
  MechanicalModel model(Parameters({}, {}));
  model.SetState(RotorAngle::FromAngle(Angle{7.0F}), AngularVelocity{});
  const RotorAngle kElectrical = model.ElectricalAngle(5U);
  EXPECT_EQ(kElectrical.AbsoluteRaw(), model.Angle().AbsoluteRaw() * 5);
  EXPECT_NEAR(kElectrical.AbsoluteAngle().Value(), 35.0F, 1.0e-4F);
}
