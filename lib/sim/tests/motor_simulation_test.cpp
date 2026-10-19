/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file motor_simulation_test.cpp
 *	@brief Tests of the motor plant simulation facade.
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */

#include "motor_simulation.hpp"
#include <gtest/gtest.h>
#include <array>
#include <cstdint>
#include <vector>

using namespace unimoc::unit;
using unimoc::settings::NvmSettings;
using unimoc::sim::MotorSimulation;
using unimoc::sim::ParametersFromSettings;
using unimoc::sim::ShaftMode;
using unimoc::sim::SimulationInput;
using unimoc::sim::SimulationParameters;
using unimoc::sim::Status;
using unimoc::system::MotorType;
using unimoc::system::Rotor;
using unimoc::system::RotorAngle;
using unimoc::system::Stator;
using unimoc::system::ThreePhase;

namespace {

/// Half period of a 20 kHz PWM: one current-control sample.
constexpr Time kDt{25.0e-6F};

NvmSettings TestSettings() {
  NvmSettings settings{};
  settings.motor_type = MotorType::kPmsm;
  settings.pole_pairs = 4U;
  settings.stator_r = 0.1_Ohm;
  settings.l_d = 0.2_mH;
  settings.l_q = 0.2_mH;
  settings.flux_pm = 10.0_mWb;
  settings.motor_j = 1.0e-4_kg_m2;
  return settings;
}

/// Converts a stator voltage into centred phase duties.
ThreePhase<Ratio> Duties(const Stator<Voltage>& voltage, Voltage dc_link) {
  const ThreePhase<Voltage> kPhase = voltage.ToThreePhase();
  return {Ratio{0.5F} + (kPhase.a / dc_link),
          Ratio{0.5F} + (kPhase.b / dc_link),
          Ratio{0.5F} + (kPhase.c / dc_link)};
}

/// Advances the simulation with the held duties; returns false when a step fails.
bool RunSteps(MotorSimulation& simulation, int steps) {
  for (int step = 0; step < steps; ++step) {
    if (simulation.Step(kDt) != Status::kOk) {
      return false;
    }
  }
  return true;
}

/// Applies a rotor-frame voltage at the true rotor angle (ideal sensor commutation).
bool RunCommutated(MotorSimulation& simulation, const Rotor<Voltage>& voltage, int steps) {
  for (int step = 0; step < steps; ++step) {
    const auto& output = simulation.Output();
    const RotorAngle kAngle = output.electrical_angle + (output.electrical_speed * kDt * 0.5F);
    simulation.SetPhaseDuties(Duties(voltage.ToStator(kAngle), output.dc_link_voltage));
    if (simulation.Step(kDt) != Status::kOk) {
      return false;
    }
  }
  return true;
}

/// Runs commutated blocks and records every change of the Hall code.
std::vector<std::uint8_t> RecordHallCodes(MotorSimulation& simulation,
                                          const Rotor<Voltage>& voltage,
                                          int blocks) {
  std::vector<std::uint8_t> codes{simulation.Output().hall.code};
  for (int block = 0; block < blocks; ++block) {
    if (!RunCommutated(simulation, voltage, 10)) {
      return {};
    }
    if (simulation.Output().hall.code != codes.back()) {
      codes.push_back(simulation.Output().hall.code);
    }
  }
  return codes;
}

/// Returns whether every recorded code follows its predecessor in the forward sequence.
bool IsForwardHallSequence(const std::vector<std::uint8_t>& codes) {
  // Successor of each code in the forward sequence 5, 1, 3, 2, 6, 4.
  constexpr std::array<std::uint8_t, 8> kNext = {0U, 3U, 6U, 2U, 5U, 1U, 4U, 0U};
  for (std::size_t index = 1U; index < codes.size(); ++index) {
    if (codes.at(index) != kNext.at(codes.at(index - 1U))) {
      return false;
    }
  }
  return true;
}

}  // namespace

TEST(MotorSimulationTest, ParametersFollowTheSettings) {
  const NvmSettings kSettings = TestSettings();
  const SimulationParameters kParameters = ParametersFromSettings(kSettings);
  EXPECT_EQ(kParameters.motor_type, kSettings.motor_type);
  EXPECT_EQ(kParameters.pmsm.pole_pairs, kSettings.pole_pairs);
  EXPECT_EQ(kParameters.pmsm.stator_resistance, kSettings.stator_r);
  EXPECT_EQ(kParameters.pmsm.l_d, kSettings.l_d);
  EXPECT_EQ(kParameters.pmsm.l_q, kSettings.l_q);
  EXPECT_EQ(kParameters.pmsm.flux_pm, kSettings.flux_pm);
  EXPECT_EQ(kParameters.mechanics.inertia, kSettings.motor_j);
}

TEST(MotorSimulationTest, DefaultMotorTypeIsTheConfiguredOne) {
  MotorSimulation simulation;
  EXPECT_FALSE(simulation.IsConfigured());
  EXPECT_EQ(simulation.Step(kDt), Status::kUnsupportedMotorType);

  ASSERT_EQ(simulation.Configure(NvmSettings{}), Status::kOk);
  EXPECT_TRUE(simulation.IsConfigured());
  EXPECT_EQ(simulation.Type(), NvmSettings{}.motor_type);
}

TEST(MotorSimulationTest, MotorTypesWithoutModelAreRejectedAndKeepTheConfiguration) {
  MotorSimulation simulation;
  ASSERT_EQ(simulation.Configure(TestSettings()), Status::kOk);

  for (const MotorType kType : {MotorType::kAsm, MotorType::kEesm}) {
    NvmSettings settings = TestSettings();
    settings.motor_type = kType;
    EXPECT_EQ(simulation.Configure(settings), Status::kUnsupportedMotorType);
    EXPECT_EQ(simulation.Type(), MotorType::kPmsm);
  }
}

TEST(MotorSimulationTest, InvalidParametersAreRejected) {
  MotorSimulation simulation;
  NvmSettings settings = TestSettings();
  settings.pole_pairs = 0U;
  EXPECT_EQ(simulation.Configure(settings), Status::kInvalidPolePairs);

  settings = TestSettings();
  settings.motor_j = Inertia{};
  EXPECT_EQ(simulation.Configure(settings), Status::kInvalidInertia);

  SimulationParameters parameters = ParametersFromSettings(TestSettings());
  parameters.dc_link_voltage = Voltage{};
  EXPECT_EQ(simulation.Configure(parameters), Status::kInvalidDcLinkVoltage);
  EXPECT_FALSE(simulation.IsConfigured());

  ASSERT_EQ(simulation.Configure(TestSettings()), Status::kOk);
  EXPECT_EQ(simulation.Step(Time{}), Status::kInvalidTimeStep);
}

TEST(MotorSimulationTest, CentredDutiesApplyNoVoltage) {
  MotorSimulation simulation;
  ASSERT_EQ(simulation.Configure(TestSettings()), Status::kOk);
  simulation.SetBridgeEnabled(true);
  ASSERT_TRUE(RunSteps(simulation, 100));
  const auto& output = simulation.Output();
  EXPECT_EQ(output.phase_currents, ThreePhase<Current>{});
  EXPECT_EQ(output.phase_voltages, ThreePhase<Voltage>{});
  EXPECT_FLOAT_EQ(output.mechanical_speed.Value(), 0.0F);
}

TEST(MotorSimulationTest, LockedRotorPhaseCurrentFollowsOhmsLaw) {
  MotorSimulation simulation;
  ASSERT_EQ(simulation.Configure(TestSettings()), Status::kOk);
  simulation.SetShaftMode(ShaftMode::kImposedSpeed, AngularVelocity{});
  simulation.SetBridgeEnabled(true);
  // 1 V along phase A: duty difference of 1.5 V / 48 V between phase A and B/C.
  simulation.SetPhaseDuties(Duties(Stator<Voltage>(1.0_V, 0.0_V), Voltage{48.0F}));
  ASSERT_TRUE(RunSteps(simulation, 2000));
  const auto& output = simulation.Output();
  EXPECT_NEAR(output.phase_voltages.a.Value(), 1.0F, 1.0e-5F);
  EXPECT_NEAR(output.phase_currents.a.Value(), 10.0F, 1.0e-2F);
  EXPECT_NEAR(output.phase_currents.b.Value(), -5.0F, 1.0e-2F);
  EXPECT_NEAR(output.phase_currents.c.Value(), -5.0F, 1.0e-2F);
  EXPECT_FLOAT_EQ(output.mechanical_speed.Value(), 0.0F);
}

TEST(MotorSimulationTest, BridgeOffDisconnectsTheStator) {
  MotorSimulation simulation;
  ASSERT_EQ(simulation.Configure(TestSettings()), Status::kOk);
  simulation.SetShaftMode(ShaftMode::kImposedSpeed, 100.0_rad_per_s);
  simulation.SetPhaseDuties({Ratio{1.0F}, Ratio{0.0F}, Ratio{0.0F}});
  ASSERT_TRUE(RunSteps(simulation, 100));
  EXPECT_EQ(simulation.Output().phase_currents, ThreePhase<Current>{});
  EXPECT_EQ(simulation.Output().phase_voltages, ThreePhase<Voltage>{});
}

TEST(MotorSimulationTest, CommutatedQVoltageSpinsForwardWithHallSequence) {
  MotorSimulation simulation;
  ASSERT_EQ(simulation.Configure(TestSettings()), Status::kOk);
  simulation.SetBridgeEnabled(true);

  const std::vector<std::uint8_t> kCodes = RecordHallCodes(simulation, Rotor<Voltage>(0.0_V, 2.0_V), 400);

  const auto& output = simulation.Output();
  EXPECT_GT(output.mechanical_speed.Value(), 10.0F);
  EXPECT_NEAR(output.electrical_speed.Value(), output.mechanical_speed.Value() * 4.0F, 1.0e-3F);

  EXPECT_GT(kCodes.size(), 6U);
  EXPECT_TRUE(IsForwardHallSequence(kCodes));
}

TEST(MotorSimulationTest, NoLoadSpeedSettlesAtBackEmfBalance) {
  MotorSimulation simulation;
  ASSERT_EQ(simulation.Configure(TestSettings()), Status::kOk);
  simulation.SetBridgeEnabled(true);
  // Without friction or load the q current decays to zero at omega_e = v_q / psi.
  ASSERT_TRUE(RunCommutated(simulation, Rotor<Voltage>(0.0_V, 2.0_V), 40'000));
  EXPECT_NEAR(simulation.Output().electrical_speed.Value(), 200.0F, 1.0F);
  EXPECT_NEAR(simulation.Output().rotor_current.q.Value(), 0.0F, 0.1F);
}

TEST(MotorSimulationTest, LoadTorqueIsBalancedByQCurrent) {
  MotorSimulation simulation;
  ASSERT_EQ(simulation.Configure(TestSettings()), Status::kOk);
  simulation.SetBridgeEnabled(true);
  simulation.SetInput(SimulationInput{0.06_Nm, Voltage{}});
  ASSERT_TRUE(RunCommutated(simulation, Rotor<Voltage>(0.0_V, 2.0_V), 40'000));

  // T = 3/2 p psi i_q -> i_q = 0.06 / (1.5 * 4 * 0.01) = 1 A
  EXPECT_NEAR(simulation.Output().rotor_current.q.Value(), 1.0F, 0.02F);
  EXPECT_NEAR(simulation.Output().electromagnetic_torque.Value(), 0.06F, 1.0e-3F);
}

TEST(MotorSimulationTest, PhaseCurrentsSumToZero) {
  MotorSimulation simulation;
  ASSERT_EQ(simulation.Configure(TestSettings()), Status::kOk);
  simulation.SetBridgeEnabled(true);
  ASSERT_TRUE(RunCommutated(simulation, Rotor<Voltage>(-0.5_V, 2.0_V), 2000));
  const auto& currents = simulation.Output().phase_currents;
  EXPECT_NEAR((currents.a + currents.b + currents.c).Value(), 0.0F, 1.0e-4F);
}

TEST(MotorSimulationTest, SetMechanicalStateMovesWithoutHallEdge) {
  MotorSimulation simulation;
  ASSERT_EQ(simulation.Configure(TestSettings()), Status::kOk);
  // 100 deg electrical = 25 deg mechanical with four pole pairs.
  simulation.SetMechanicalState(RotorAngle::FromAngle(25.0_deg), AngularVelocity{});
  EXPECT_NEAR(simulation.Output().electrical_angle.AngleInRevolution().Value(), (100.0_deg).Value(), 1.0e-5F);
  EXPECT_EQ(simulation.Output().hall.code, 1U);
  EXPECT_FALSE(simulation.Output().hall.edge);
}

TEST(MotorSimulationTest, IdenticalRunsAreBitExact) {
  MotorSimulation first;
  MotorSimulation second;
  ASSERT_EQ(first.Configure(TestSettings()), Status::kOk);
  ASSERT_EQ(second.Configure(TestSettings()), Status::kOk);
  first.SetBridgeEnabled(true);
  second.SetBridgeEnabled(true);
  ASSERT_TRUE(RunCommutated(first, Rotor<Voltage>(0.3_V, 1.5_V), 3000));
  ASSERT_TRUE(RunCommutated(second, Rotor<Voltage>(0.3_V, 1.5_V), 3000));

  EXPECT_EQ(first.Output().phase_currents, second.Output().phase_currents);
  EXPECT_EQ(first.Output().mechanical_angle, second.Output().mechanical_angle);
  EXPECT_EQ(first.Output().hall.code, second.Output().hall.code);
}

TEST(MotorSimulationTest, ResetReturnsToStandstill) {
  MotorSimulation simulation;
  ASSERT_EQ(simulation.Configure(TestSettings()), Status::kOk);
  simulation.SetBridgeEnabled(true);
  ASSERT_TRUE(RunCommutated(simulation, Rotor<Voltage>(0.0_V, 2.0_V), 1000));
  simulation.Reset();

  const auto& output = simulation.Output();
  EXPECT_EQ(output.phase_currents, ThreePhase<Current>{});
  EXPECT_EQ(output.mechanical_angle, RotorAngle{});
  EXPECT_FLOAT_EQ(output.mechanical_speed.Value(), 0.0F);
  EXPECT_EQ(output.hall.code, 5U);
}
