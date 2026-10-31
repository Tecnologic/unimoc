/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file dc_link_model_test.cpp
 *    @brief ztest coverage of the DC link, the dead time and the body diodes of the plant.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "dc_link_model.hpp"
#include <zephyr/ztest.h>
#include "inverter_model.hpp"
#include "motor_simulation.hpp"
#include "unimoc_test.hpp"

using namespace unimoc::unit;
using unimoc::sim::DcLinkModel;
using unimoc::sim::DcLinkParameters;
using unimoc::sim::InverterModel;
using unimoc::sim::MotorSimulation;
using unimoc::sim::ShaftMode;
using unimoc::sim::SimulationParameters;
using unimoc::sim::Status;
using unimoc::system::ThreePhase;

namespace {

constexpr Time kDt = 15.625e-6_s;  // half period of 32 kHz PWM

DcLinkParameters Capacitor() {
  DcLinkParameters parameters;
  parameters.capacitance = 2.0_mF;
  parameters.battery_resistance = 0.05_Ohm;
  return parameters;
}

/// A PMSM with a strong magnet: 0.05 Wb, so 1000 rad/s electrical is 50 V peak, 87 V line to line.
SimulationParameters Generator() {
  SimulationParameters parameters;
  parameters.pmsm.pole_pairs = 1U;
  parameters.pmsm.stator_resistance = 0.1_Ohm;
  parameters.pmsm.l_d = 0.2_mH;
  parameters.pmsm.l_q = 0.2_mH;
  parameters.pmsm.flux_pm = 0.05_Wb;
  parameters.dc_link_voltage = 48.0_V;
  return parameters;
}

}  // namespace

ZTEST(dc_link_model, test_ideal_source_holds_the_battery_voltage) {
  DcLinkModel dc_link(DcLinkParameters{}, 48.0_V);
  zassert_equal(dc_link.Validate(), Status::kOk);
  zexpect_true(dc_link.Ideal());
  dc_link.SetContactorClosed(false);
  dc_link.Step(100.0_A, kDt);
  UNIMOC_EXPECT_FLOAT_EQ(dc_link.Voltage(), 48.0_V);
  UNIMOC_EXPECT_FLOAT_EQ(dc_link.BatteryCurrent(), 100.0_A);
  dc_link.SetBatteryVoltage(50.0_V);
  UNIMOC_EXPECT_FLOAT_EQ(dc_link.Voltage(), 50.0_V);
}

ZTEST(dc_link_model, test_open_contactor_leaves_the_capacitor_to_the_inverter) {
  DcLinkModel dc_link(Capacitor(), 48.0_V);
  zassert_equal(dc_link.Validate(), Status::kOk);
  dc_link.SetContactorClosed(false);
  // Returning 10 A for 1 ms charges 2 mF by 5 V.
  for (int step = 0; step < 64; ++step) {
    dc_link.Step(-10.0_A, kDt);
  }
  UNIMOC_EXPECT_NEAR(dc_link.Voltage(), 53.0_V, 0.01_V);
  UNIMOC_EXPECT_FLOAT_EQ(dc_link.BatteryCurrent(), Current{});

  // Drawn empty, the low-side diodes clamp it at zero.
  for (int step = 0; step < 20000; ++step) {
    dc_link.Step(50.0_A, kDt);
  }
  UNIMOC_EXPECT_FLOAT_EQ(dc_link.Voltage(), Voltage{});
}

ZTEST(dc_link_model, test_battery_resistance_sets_the_steady_state) {
  DcLinkModel dc_link(Capacitor(), 48.0_V);
  // Time constant R C = 0.1 ms: settled after 2 ms.
  for (int step = 0; step < 128; ++step) {
    dc_link.Step(20.0_A, kDt);
  }
  UNIMOC_EXPECT_NEAR(dc_link.Voltage(), 47.0_V, 1.0e-3_V);  // 48 V - 20 A * 0.05 Ohm
  UNIMOC_EXPECT_NEAR(dc_link.BatteryCurrent(), 20.0_A, 0.02_A);

  dc_link.SetContactorClosed(false);
  dc_link.Step(Current{}, kDt);
  dc_link.SetContactorClosed(true);
  dc_link.Reset();
  UNIMOC_EXPECT_FLOAT_EQ(dc_link.Voltage(), 48.0_V);
  zexpect_true(dc_link.ContactorClosed());
}

ZTEST(dc_link_model, test_invalid_parameters_are_rejected) {
  DcLinkParameters negative;
  negative.capacitance = -1.0_mF;
  zexpect_equal(DcLinkModel(negative, 48.0_V).Validate(), Status::kInvalidCapacitance);
  DcLinkParameters no_resistance = Capacitor();
  no_resistance.battery_resistance = Resistance{};
  zexpect_equal(DcLinkModel(no_resistance, 48.0_V).Validate(), Status::kInvalidResistance);

  SimulationParameters parameters = Generator();
  parameters.inverter.dead_time = 0.5_ratio;
  MotorSimulation simulation;
  zexpect_equal(simulation.Configure(parameters), Status::kInvalidDeadTime);
}

ZTEST(dc_link_model, test_dead_time_lowers_the_terminal_with_the_current) {
  InverterModel inverter;
  inverter.SetEnabled(true);
  inverter.SetDeadTime(0.02_ratio);
  // Phase a carries +10 A out of the bridge, b and c return it.
  const ThreePhase<Current> kCurrents{10.0_A, -5.0_A, -5.0_A};
  const auto kVoltages = inverter.PhaseVoltages(48.0_V, kCurrents);
  // Terminal errors -D, +D, +D of 48 V; without the common mode a gets -4/3 D V_dc.
  UNIMOC_EXPECT_NEAR(kVoltages.a, -48.0_V * 0.02F * (4.0F / 3.0F), 1.0e-4_V);
  UNIMOC_EXPECT_NEAR(kVoltages.b, 48.0_V * 0.02F * (2.0F / 3.0F), 1.0e-4_V);
  const auto kIdeal = inverter.PhaseVoltages(48.0_V);
  UNIMOC_EXPECT_FLOAT_EQ(kIdeal.a, Voltage{});
}

ZTEST(dc_link_model, test_open_bridge_charges_the_dc_link_above_the_back_emf_limit) {
  SimulationParameters parameters = Generator();
  parameters.dc_link = Capacitor();
  MotorSimulation simulation;
  zassert_equal(simulation.Configure(parameters), Status::kOk);
  simulation.SetContactorClosed(false);

  // 400 rad/s: 35 V line to line, below 48 V: the diodes block.
  simulation.SetShaftMode(ShaftMode::kImposedSpeed, 400.0_rad_per_s);
  for (int step = 0; step < 640; ++step) {
    zassert_equal(simulation.Step(kDt), Status::kOk);
  }
  zexpect_false(simulation.Output().diodes_conducting);
  UNIMOC_EXPECT_FLOAT_EQ(simulation.Output().dc_link_voltage, 48.0_V);

  // 1000 rad/s: 87 V line to line: the motor charges the capacitor through the diodes.
  simulation.SetShaftMode(ShaftMode::kImposedSpeed, 1000.0_rad_per_s);
  Torque torque_sum{};
  bool conducted = false;
  for (int step = 0; step < 640; ++step) {
    zassert_equal(simulation.Step(kDt), Status::kOk);
    torque_sum = torque_sum + simulation.Output().electromagnetic_torque;
    conducted = conducted || simulation.Output().diodes_conducting;
    zexpect_false(simulation.Output().dc_link_current > Current{}, "diodes only return energy");
  }
  zexpect_true(conducted);
  zexpect_true(simulation.Output().dc_link_voltage > 55.0_V, "the DC link rises with the bridge off");
  zexpect_true(torque_sum < Torque{}, "the rectifier brakes the shaft");
  // Charged to the line-to-line peak, the diodes block again.
  zexpect_false(simulation.Output().dc_link_voltage > 1000.0_rad_per_s * 0.05_Wb * 1.7320508F + 1.0_V);
}

ZTEST(dc_link_model, test_ideal_source_keeps_the_open_stator) {
  MotorSimulation simulation;
  zassert_equal(simulation.Configure(Generator()), Status::kOk);
  simulation.SetShaftMode(ShaftMode::kImposedSpeed, 1000.0_rad_per_s);
  for (int step = 0; step < 640; ++step) {
    zassert_equal(simulation.Step(kDt), Status::kOk);
  }
  zexpect_false(simulation.Output().diodes_conducting);
  UNIMOC_EXPECT_FLOAT_EQ(simulation.Output().phase_currents.a, Current{});
  UNIMOC_EXPECT_FLOAT_EQ(simulation.Output().dc_link_voltage, 48.0_V);
}

ZTEST(dc_link_model, test_motoring_draws_from_the_battery) {
  SimulationParameters parameters = Generator();
  parameters.dc_link = Capacitor();
  MotorSimulation simulation;
  zassert_equal(simulation.Configure(parameters), Status::kOk);
  simulation.SetShaftMode(ShaftMode::kImposedSpeed, AngularVelocity{});
  simulation.SetBridgeEnabled(true);
  // A fixed alpha voltage of 2.4 V on the held rotor: 24 A in phase a, 2.4 V * 24 A / 48 V = 1.2 A from the
  // link.
  simulation.SetPhaseDuties({0.55_ratio, 0.475_ratio, 0.475_ratio});
  for (int step = 0; step < 6400; ++step) {
    zassert_equal(simulation.Step(kDt), Status::kOk);
  }
  zexpect_true(simulation.Output().battery_current > Current{});
  UNIMOC_EXPECT_NEAR(simulation.Output().dc_link_current * simulation.Output().dc_link_voltage,
                     simulation.Output().phase_currents.a * simulation.Output().phase_voltages.a * 1.5F,
                     1.0_W);
}

ZTEST_SUITE(dc_link_model, nullptr, nullptr, nullptr, nullptr, nullptr);
