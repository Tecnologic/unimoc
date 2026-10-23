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
 *	SPDX-License-Identifier: GPL-3.0-or-later
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */

#include "motor_simulation.hpp"
#include <zephyr/ztest.h>
#include <array>
#include <cstdint>
#include <vector>
#include "unimoc_test.hpp"

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
  return {0.5_ratio + (kPhase.a / dc_link),
          0.5_ratio + (kPhase.b / dc_link),
          0.5_ratio + (kPhase.c / dc_link)};
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

ZTEST(motor_simulation, test_parameters_follow_the_settings) {
  const NvmSettings kSettings = TestSettings();
  const SimulationParameters kParameters = ParametersFromSettings(kSettings);
  zexpect_equal(kParameters.motor_type, kSettings.motor_type);
  zexpect_equal(kParameters.pmsm.pole_pairs, kSettings.pole_pairs);
  zexpect_equal(kParameters.pmsm.stator_resistance, kSettings.stator_r);
  zexpect_equal(kParameters.pmsm.l_d, kSettings.l_d);
  zexpect_equal(kParameters.pmsm.l_q, kSettings.l_q);
  zexpect_equal(kParameters.pmsm.flux_pm, kSettings.flux_pm);
  zexpect_equal(kParameters.mechanics.inertia, kSettings.motor_j);
}

ZTEST(motor_simulation, test_default_motor_type_is_the_configured_one) {
  MotorSimulation simulation;
  zexpect_false(simulation.IsConfigured());
  zexpect_equal(simulation.Step(kDt), Status::kUnsupportedMotorType);

  zassert_equal(simulation.Configure(NvmSettings{}), Status::kOk);
  zexpect_true(simulation.IsConfigured());
  zexpect_equal(simulation.Type(), NvmSettings{}.motor_type);
}

ZTEST(motor_simulation, test_motor_types_without_model_are_rejected_and_keep_the_configuration) {
  MotorSimulation simulation;
  zassert_equal(simulation.Configure(TestSettings()), Status::kOk);

  for (const MotorType kType : {MotorType::kAsm, MotorType::kEesm}) {
    NvmSettings settings = TestSettings();
    settings.motor_type = kType;
    zexpect_equal(simulation.Configure(settings), Status::kUnsupportedMotorType);
    zexpect_equal(simulation.Type(), MotorType::kPmsm);
  }
}

ZTEST(motor_simulation, test_invalid_parameters_are_rejected) {
  MotorSimulation simulation;
  NvmSettings settings = TestSettings();
  settings.pole_pairs = 0U;
  zexpect_equal(simulation.Configure(settings), Status::kInvalidPolePairs);

  settings = TestSettings();
  settings.motor_j = Inertia{};
  zexpect_equal(simulation.Configure(settings), Status::kInvalidInertia);

  SimulationParameters parameters = ParametersFromSettings(TestSettings());
  parameters.dc_link_voltage = Voltage{};
  zexpect_equal(simulation.Configure(parameters), Status::kInvalidDcLinkVoltage);
  zexpect_false(simulation.IsConfigured());

  zassert_equal(simulation.Configure(TestSettings()), Status::kOk);
  zexpect_equal(simulation.Step(Time{}), Status::kInvalidTimeStep);
}

ZTEST(motor_simulation, test_centred_duties_apply_no_voltage) {
  MotorSimulation simulation;
  zassert_equal(simulation.Configure(TestSettings()), Status::kOk);
  simulation.SetBridgeEnabled(true);
  zassert_true(RunSteps(simulation, 100));
  const auto& output = simulation.Output();
  zexpect_equal(output.phase_currents, ThreePhase<Current>{});
  zexpect_equal(output.phase_voltages, ThreePhase<Voltage>{});
  UNIMOC_EXPECT_FLOAT_EQ(output.mechanical_speed, 0.0_rad_per_s);
}

ZTEST(motor_simulation, test_locked_rotor_phase_current_follows_ohms_law) {
  MotorSimulation simulation;
  zassert_equal(simulation.Configure(TestSettings()), Status::kOk);
  simulation.SetShaftMode(ShaftMode::kImposedSpeed, AngularVelocity{});
  simulation.SetBridgeEnabled(true);
  // 1 V along phase A: duty difference of 1.5 V / 48 V between phase A and B/C.
  simulation.SetPhaseDuties(Duties(Stator<Voltage>(1.0_V, 0.0_V), 48.0_V));
  zassert_true(RunSteps(simulation, 2000));
  const auto& output = simulation.Output();
  UNIMOC_EXPECT_NEAR(output.phase_voltages.a, 1.0_V, 1.0e-5_V);
  UNIMOC_EXPECT_NEAR(output.phase_currents.a, 10.0_A, 1.0e-2_A);
  UNIMOC_EXPECT_NEAR(output.phase_currents.b, -5.0_A, 1.0e-2_A);
  UNIMOC_EXPECT_NEAR(output.phase_currents.c, -5.0_A, 1.0e-2_A);
  UNIMOC_EXPECT_FLOAT_EQ(output.mechanical_speed, 0.0_rad_per_s);
}

ZTEST(motor_simulation, test_bridge_off_disconnects_the_stator) {
  MotorSimulation simulation;
  zassert_equal(simulation.Configure(TestSettings()), Status::kOk);
  simulation.SetShaftMode(ShaftMode::kImposedSpeed, 100.0_rad_per_s);
  simulation.SetPhaseDuties({1.0_ratio, 0.0_ratio, 0.0_ratio});
  zassert_true(RunSteps(simulation, 100));
  zexpect_equal(simulation.Output().phase_currents, ThreePhase<Current>{});
  zexpect_equal(simulation.Output().phase_voltages, ThreePhase<Voltage>{});
}

ZTEST(motor_simulation, test_commutated_q_voltage_spins_forward_with_hall_sequence) {
  MotorSimulation simulation;
  zassert_equal(simulation.Configure(TestSettings()), Status::kOk);
  simulation.SetBridgeEnabled(true);

  const std::vector<std::uint8_t> kCodes = RecordHallCodes(simulation, Rotor<Voltage>(0.0_V, 2.0_V), 400);

  const auto& output = simulation.Output();
  zexpect_true(output.mechanical_speed > 10.0_rad_per_s);
  UNIMOC_EXPECT_NEAR(output.electrical_speed, output.mechanical_speed * 4.0F, 1.0e-3_rad_per_s);

  zexpect_true((kCodes.size()) > (6U));
  zexpect_true(IsForwardHallSequence(kCodes));
}

ZTEST(motor_simulation, test_no_load_speed_settles_at_back_emf_balance) {
  MotorSimulation simulation;
  zassert_equal(simulation.Configure(TestSettings()), Status::kOk);
  simulation.SetBridgeEnabled(true);
  // Without friction or load the q current decays to zero at omega_e = v_q / psi.
  zassert_true(RunCommutated(simulation, Rotor<Voltage>(0.0_V, 2.0_V), 40'000));
  UNIMOC_EXPECT_NEAR(simulation.Output().electrical_speed, 200.0_rad_per_s, 1.0_rad_per_s);
  UNIMOC_EXPECT_NEAR(simulation.Output().rotor_current.q, 0.0_A, 0.1_A);
}

ZTEST(motor_simulation, test_load_torque_is_balanced_by_q_current) {
  MotorSimulation simulation;
  zassert_equal(simulation.Configure(TestSettings()), Status::kOk);
  simulation.SetBridgeEnabled(true);
  simulation.SetInput(SimulationInput{0.06_Nm, Voltage{}});
  zassert_true(RunCommutated(simulation, Rotor<Voltage>(0.0_V, 2.0_V), 40'000));

  // T = 3/2 p psi i_q -> i_q = 0.06 / (1.5 * 4 * 0.01) = 1 A
  UNIMOC_EXPECT_NEAR(simulation.Output().rotor_current.q, 1.0_A, 0.02_A);
  UNIMOC_EXPECT_NEAR(simulation.Output().electromagnetic_torque, 0.06_Nm, 1.0e-3_Nm);
}

ZTEST(motor_simulation, test_phase_currents_sum_to_zero) {
  MotorSimulation simulation;
  zassert_equal(simulation.Configure(TestSettings()), Status::kOk);
  simulation.SetBridgeEnabled(true);
  zassert_true(RunCommutated(simulation, Rotor<Voltage>(-0.5_V, 2.0_V), 2000));
  const auto& currents = simulation.Output().phase_currents;
  UNIMOC_EXPECT_NEAR((currents.a + currents.b + currents.c), 0.0_A, 1.0e-4_A);
}

ZTEST(motor_simulation, test_set_mechanical_state_moves_without_hall_edge) {
  MotorSimulation simulation;
  zassert_equal(simulation.Configure(TestSettings()), Status::kOk);
  // 100 deg electrical = 25 deg mechanical with four pole pairs.
  simulation.SetMechanicalState(RotorAngle::FromAngle(25.0_deg), AngularVelocity{});
  UNIMOC_EXPECT_NEAR(simulation.Output().electrical_angle.AngleInRevolution(), (100.0_deg), 1.0e-5_rad);
  zexpect_equal(simulation.Output().hall.code, 1U);
  zexpect_false(simulation.Output().hall.edge);
}

ZTEST(motor_simulation, test_identical_runs_are_bit_exact) {
  MotorSimulation first;
  MotorSimulation second;
  zassert_equal(first.Configure(TestSettings()), Status::kOk);
  zassert_equal(second.Configure(TestSettings()), Status::kOk);
  first.SetBridgeEnabled(true);
  second.SetBridgeEnabled(true);
  zassert_true(RunCommutated(first, Rotor<Voltage>(0.3_V, 1.5_V), 3000));
  zassert_true(RunCommutated(second, Rotor<Voltage>(0.3_V, 1.5_V), 3000));

  zexpect_equal(first.Output().phase_currents, second.Output().phase_currents);
  zexpect_equal(first.Output().mechanical_angle, second.Output().mechanical_angle);
  zexpect_equal(first.Output().hall.code, second.Output().hall.code);
}

ZTEST(motor_simulation, test_reset_returns_to_standstill) {
  MotorSimulation simulation;
  zassert_equal(simulation.Configure(TestSettings()), Status::kOk);
  simulation.SetBridgeEnabled(true);
  zassert_true(RunCommutated(simulation, Rotor<Voltage>(0.0_V, 2.0_V), 1000));
  simulation.Reset();

  const auto& output = simulation.Output();
  zexpect_equal(output.phase_currents, ThreePhase<Current>{});
  zexpect_equal(output.mechanical_angle, RotorAngle{});
  UNIMOC_EXPECT_FLOAT_EQ(output.mechanical_speed, 0.0_rad_per_s);
  zexpect_equal(output.hall.code, 5U);
}

ZTEST_SUITE(motor_simulation, nullptr, nullptr, nullptr, nullptr, nullptr);
