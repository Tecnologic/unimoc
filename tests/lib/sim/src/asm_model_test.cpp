/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file asm_model_test.cpp
 *	@brief Tests of the electrical induction motor model: flux build-up, slip and torque direction.
 *
 *	SPDX-License-Identifier: GPL-3.0-or-later
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */
#include "asm_model.hpp"
#include <zephyr/ztest.h>
#include "sin_cos.hpp"
#include "unimoc_test.hpp"

using namespace unimoc::unit;
using unimoc::sim::AsmModel;
using unimoc::sim::AsmParameters;
using unimoc::sim::ElectricalInput;
using unimoc::sim::Status;
using unimoc::system::MotorType;
using unimoc::system::Rotor;
using unimoc::system::RotorAngle;
using unimoc::system::Stator;

namespace {

constexpr Time kDt = 25.0_us;

/// A small induction motor: rotor time constant L_r / R_r = 0.167 s.
AsmParameters TestParameters() {
  AsmParameters parameters{};
  parameters.pole_pairs = 2U;
  parameters.stator_resistance = 0.5_Ohm;
  parameters.rotor_resistance = 0.3_Ohm;
  parameters.l_s = 50.0_mH;
  parameters.l_r = 50.0_mH;
  parameters.l_m = 47.0_mH;
  return parameters;
}

/// Steps with a rotor-frame voltage at a standing rotor.
void RunStanding(AsmModel& model, const Rotor<Voltage>& voltage, const int steps) {
  const RotorAngle kAngle{};
  const ElectricalInput kInput{voltage.ToStator(kAngle), kAngle, AngularVelocity{}, true, Voltage{}};
  for (int step = 0; step < steps; ++step) {
    model.Step(kInput, kDt);
  }
}

/**
 * Feeds a stator voltage of @p amplitude that turns at @p field_speed (electrical, stationary frame) into
 * a rotor turning at @p rotor_speed (electrical) for @p seconds; returns the mean torque of the last 0.1 s.
 */
Torque MeanTorque(const Voltage amplitude,
                  const AngularVelocity field_speed,
                  const AngularVelocity rotor_speed,
                  const float seconds) {
  AsmModel model(TestParameters());
  const int kSteps = static_cast<int>(seconds / 25.0e-6F);
  const int kAverage = static_cast<int>(0.1F / 25.0e-6F);
  Torque sum;
  for (int step = 0; step < kSteps; ++step) {
    const float kTime = (static_cast<float>(step) + 0.5F) * 25.0e-6F;
    const unimoc::system::SinCos<Ratio> kField(field_speed * (1.0_s * kTime));
    const Stator<Voltage> kVoltage{amplitude * kField.cos, amplitude * kField.sin};
    const RotorAngle kAngle = RotorAngle::FromAngle(rotor_speed * (1.0_s * kTime));
    model.Step(ElectricalInput{kVoltage, kAngle, rotor_speed, true, Voltage{}}, kDt);
    if (step >= kSteps - kAverage) {
      sum += model.Output().torque;
    }
  }
  return sum / static_cast<float>(kAverage);
}

}  // namespace

ZTEST(asm_model, test_validate_rejects_invalid_parameters) {
  zexpect_equal(AsmModel(TestParameters()).Validate(), Status::kOk);
  zexpect_equal(AsmModel::Type(), MotorType::kAsm);
  AsmParameters parameters = TestParameters();
  parameters.pole_pairs = 0U;
  zexpect_equal(AsmModel(parameters).Validate(), Status::kInvalidPolePairs);
  parameters = TestParameters();
  parameters.rotor_resistance = Resistance{};
  zexpect_equal(AsmModel(parameters).Validate(), Status::kInvalidResistance);
  parameters = TestParameters();
  parameters.l_m = 50.0_mH;  // no leakage: L_m^2 = L_s L_r
  zexpect_equal(AsmModel(parameters).Validate(), Status::kInvalidInductance);
}

ZTEST(asm_model, test_a_dc_current_magnetizes_the_rotor_without_torque) {
  AsmModel model(TestParameters());
  // 1 V on d: 2 A of stator current; the rotor flux builds up to L_m i_s = 94 mWb with about L_r / R_r.
  RunStanding(model, {1.0_V, Voltage{}}, static_cast<int>(0.05F / 25.0e-6F));
  const MagneticFlux kEarly = model.RotorFlux().d;
  RunStanding(model, {1.0_V, Voltage{}}, static_cast<int>(2.0F / 25.0e-6F));
  const MagneticFlux kSettled = model.RotorFlux().d;
  zexpect_true(kEarly > MagneticFlux{} && kEarly < kSettled * 0.6F,
               "the flux builds up with the rotor time constant");
  // The stator current is the difference of two fluxes over the small leakage: 0.1 % float error.
  UNIMOC_EXPECT_NEAR(model.Output().rotor_current.d, 2.0_A, 0.01_A);
  const MagneticFlux kExpected = 47.0_mH * 2.0_A;
  UNIMOC_EXPECT_NEAR(kSettled, kExpected, 5e-4_Wb);
  UNIMOC_EXPECT_NEAR(model.Output().torque, Torque{}, 1e-4_Nm);
}

ZTEST(asm_model, test_the_rotor_flux_decays_with_an_open_stator) {
  AsmModel model(TestParameters());
  RunStanding(model, {1.0_V, Voltage{}}, static_cast<int>(2.0F / 25.0e-6F));
  const MagneticFlux kStart = model.RotorFlux().d;
  const ElectricalInput kOpen{Stator<Voltage>{}, RotorAngle{}, AngularVelocity{}, false, Voltage{}};
  // One rotor time constant (50 mH / 0.3 Ohm = 0.167 s): down to 1/e.
  for (int step = 0; step < static_cast<int>((1.0F / 6.0F) / 25.0e-6F); ++step) {
    model.Step(kOpen, kDt);
  }
  UNIMOC_EXPECT_NEAR(model.RotorFlux().d, kStart * 0.3679F, kStart * 0.01F);
  UNIMOC_EXPECT_NEAR(model.Output().rotor_current.d, Current{}, 1e-6_A);
}

ZTEST(asm_model, test_the_torque_follows_the_slip) {
  // A field at 2 x 50 rad/s electrical: motoring below, generating above, nothing at synchronous speed.
  const AngularVelocity kField = 100.0_rad_per_s;
  const Torque kMotoring = MeanTorque(10.0_V, kField, 90.0_rad_per_s, 1.5F);
  const Torque kSynchronous = MeanTorque(10.0_V, kField, kField, 1.5F);
  const Torque kGenerating = MeanTorque(10.0_V, kField, 110.0_rad_per_s, 1.5F);
  zexpect_true(kMotoring > 0.05_Nm, "below the field speed the motor drives");
  zexpect_true(kGenerating < -0.05_Nm, "above it the motor brakes");
  UNIMOC_EXPECT_NEAR(kSynchronous, Torque{}, 1e-3_Nm);
}

ZTEST(asm_model, test_reset_clears_the_fluxes) {
  AsmModel model(TestParameters());
  RunStanding(model, {1.0_V, 1.0_V}, 1000);
  model.Reset();
  zexpect_equal(model.Output().rotor_current, Rotor<Current>{});
  zexpect_equal(model.RotorFlux(), Rotor<MagneticFlux>{});
}

ZTEST_SUITE(asm_model, nullptr, nullptr, nullptr, nullptr, nullptr);
