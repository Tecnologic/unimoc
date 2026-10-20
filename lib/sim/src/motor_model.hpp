/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file motor_model.hpp
 *	@brief Common contract of the electrical motor models of the simulation.
 *
 *	Every motor type of the project (PMSM, ASM, EESM) is simulated by one
 *	electrical model that satisfies the MotorModel concept. The models see
 *	the same inputs (stator voltage, rotor position and speed) and produce the
 *	same outputs (stator current, electromagnetic torque), so MotorSimulation
 *	can host any of them behind one API.
 *
 *	SPDX-License-Identifier: GPL-3.0-or-later
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */
#pragma once

#include <concepts>
#include <cstdint>
#include "motor_type.hpp"
#include "rotor_angle.hpp"
#include "rotor_system.hpp"
#include "stator_system.hpp"
#include "units.hpp"

/**
 * @namespace unimoc::sim Deterministic motor plant simulation.
 */
namespace unimoc::sim {

/**
 * @brief Result of a parameter check or a configuration request.
 */
enum class Status : std::uint8_t {
  /// The parameters are valid and were applied.
  kOk,
  /// The selected motor type has no model in the simulation yet.
  kUnsupportedMotorType,
  /// The pole-pair count is zero.
  kInvalidPolePairs,
  /// A resistance is not positive.
  kInvalidResistance,
  /// An inductance is not positive.
  kInvalidInductance,
  /// A flux linkage is negative.
  kInvalidFlux,
  /// The rotational inertia is not positive.
  kInvalidInertia,
  /// A friction coefficient is negative.
  kInvalidFriction,
  /// The DC-link voltage is not positive.
  kInvalidDcLinkVoltage,
  /// The integration step is not positive.
  kInvalidTimeStep,
};

/**
 * @brief Inputs of one electrical model step.
 *
 * The mechanical state is held constant over the step. The stator voltage is
 * the average voltage the inverter applies during the step.
 */
struct ElectricalInput {
  /// Average stator voltage in the stationary alpha/beta frame.
  system::Stator<unit::Voltage> stator_voltage;
  /// Electrical rotor angle at the middle of the step.
  system::RotorAngle electrical_angle;
  /// Electrical rotor speed.
  unit::AngularVelocity electrical_speed;
  /// `false` when the inverter bridge is switched off and the stator is open.
  bool stator_connected{false};
  /// Field winding voltage. Only used by models with a field winding (EESM).
  unit::Voltage field_voltage;
};

/**
 * @brief Outputs of an electrical model after a step.
 */
struct ElectricalOutput {
  /// Stator current in the rotating d/q frame of the rotor.
  system::Rotor<unit::Current> rotor_current;
  /// Electromagnetic torque on the shaft.
  unit::Torque torque;
  /// Field winding current. Zero for models without a field winding.
  unit::Current field_current;
};

/**
 * @brief Contract every electrical motor model of the simulation fulfils.
 *
 * - `Type()` names the simulated MotorType.
 * - `Validate()` checks the parameters without changing the model.
 * - `Reset()` returns all electrical states to zero.
 * - `Step()` integrates the electrical states over one time step.
 * - `Output()` returns the states at the end of the last step.
 */
template <typename M>
concept MotorModel =
    requires(M model, const M& const_model, const ElectricalInput& input, unit::Time time_step) {
      { const_model.Type() } -> std::same_as<system::MotorType>;
      { const_model.Validate() } -> std::same_as<Status>;
      { model.Reset() };
      { model.Step(input, time_step) };
      { const_model.Output() } -> std::same_as<ElectricalOutput>;
    };

}  // namespace unimoc::sim
