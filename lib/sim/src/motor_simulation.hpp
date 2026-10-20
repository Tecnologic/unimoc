/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file motor_simulation.hpp
 *	@brief Motor plant simulation: inverter, motor, shaft, and position sensor.
 *
 *	SPDX-License-Identifier: GPL-3.0-or-later
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include <variant>
#include "hall_sensor.hpp"
#include "inverter_model.hpp"
#include "mechanical_model.hpp"
#include "motor_model.hpp"
#include "motor_type.hpp"
#include "nvm_settings.hpp"
#include "pmsm_model.hpp"
#include "three_phase_system.hpp"
#include "units.hpp"

namespace unimoc::sim {

/**
 * @brief Complete parameter set of one simulated drive.
 *
 * The motor type selects the electrical model. Parameters of the other motor
 * types are carried along but ignored.
 */
struct SimulationParameters {
  /// Simulated motor type.
  system::MotorType motor_type{system::MotorType::kPmsm};
  /// PMSM parameters, used for MotorType::kPmsm.
  PmsmParameters pmsm;
  /// Shaft and load parameters, used for every motor type.
  MechanicalParameters mechanics;
  /// Hall sensor parameters.
  HallParameters hall;
  /// DC-link voltage of the ideal supply.
  unit::Voltage dc_link_voltage{unit::Voltage{48.0F}};
};

/**
 * @brief Derives simulation parameters from the drive settings.
 * @param settings Drive settings; the motor type, pole pairs, motor and inertia parameters are used.
 * @return Parameters of a motor of the configured type. Friction, Hall offset and the DC-link
 * voltage keep their defaults, because the settings do not describe them.
 */
[[nodiscard]] constexpr SimulationParameters ParametersFromSettings(
    const settings::NvmSettings& settings) noexcept {
  SimulationParameters parameters;
  parameters.motor_type = settings.motor_type;
  parameters.pmsm.pole_pairs = settings.pole_pairs;
  parameters.pmsm.stator_resistance = settings.stator_r;
  parameters.pmsm.l_d = settings.l_d;
  parameters.pmsm.l_q = settings.l_q;
  parameters.pmsm.flux_pm = settings.flux_pm;
  parameters.mechanics.inertia = settings.motor_j;
  return parameters;
}

/**
 * @brief Inputs that a test or the simulated hardware applies in addition to the PWM.
 */
struct SimulationInput {
  /// External load torque on the shaft; positive opposes positive rotation.
  unit::Torque load_torque;
  /// Field winding voltage for motors with a field winding (EESM).
  unit::Voltage field_voltage;
};

/**
 * @brief Measurable state of the simulated drive at the end of the last step.
 */
struct SimulationOutput {
  /// Phase currents, as the current sensors measure them.
  system::ThreePhase<unit::Current> phase_currents;
  /// Phase-to-star-point voltages averaged over the last step.
  system::ThreePhase<unit::Voltage> phase_voltages;
  /// DC-link voltage.
  unit::Voltage dc_link_voltage;
  /// Stator current in the d/q frame of the rotor (simulation truth).
  system::Rotor<unit::Current> rotor_current;
  /// Field winding current (EESM), zero otherwise.
  unit::Current field_current;
  /// Electromagnetic torque.
  unit::Torque electromagnetic_torque;
  /// Mechanical shaft angle.
  system::RotorAngle mechanical_angle;
  /// Mechanical shaft speed.
  unit::AngularVelocity mechanical_speed;
  /// Electrical rotor angle (simulation truth).
  system::RotorAngle electrical_angle;
  /// Electrical rotor speed (simulation truth).
  unit::AngularVelocity electrical_speed;
  /// Hall sensor signals.
  HallSignals hall;
};

/**
 * @brief Deterministic simulation of one motor with its inverter, shaft and Hall sensors.
 *
 * The simulation advances in explicit steps, normally one per current-control
 * sample (half a PWM period). Inside a step:
 *
 * 1. The inverter applies the held duties as an average stator voltage.
 * 2. The electrical model integrates the motor currents with the mechanical
 *    state of the start of the step.
 * 3. The shaft integrates the resulting torque.
 * 4. The Hall sensors evaluate the new electrical angle.
 *
 * Output() then returns the sample at the end of the step. The motor type is
 * taken from the parameters; ParametersFromSettings() makes it follow the
 * configured `NvmSettings::motor_type`. Each motor type maps to one
 * electrical model behind the MotorModel concept; the public API is the same
 * for all of them.
 *
 * The class has no dynamic allocation, no locks and no floating-point library
 * calls, so it can run inside a simulated interrupt.
 */
class MotorSimulation {
 public:
  /** @brief Constructs an unconfigured simulation; Configure() must succeed before Step(). */
  constexpr MotorSimulation() = default;

  /**
   * @brief Validates and applies parameters and resets all states.
   * @param parameters Parameters of the drive; the motor type selects the model.
   * @return Status::kOk, or the reason the parameters were rejected. A rejected
   * configuration leaves the previous one active.
   */
  constexpr Status Configure(const SimulationParameters& parameters) noexcept {
    if (parameters.dc_link_voltage <= unit::Voltage{}) {
      return Status::kInvalidDcLinkVoltage;
    }
    const MechanicalModel kMechanics(parameters.mechanics);
    const Status kMechanicsStatus = kMechanics.Validate();
    if (kMechanicsStatus != Status::kOk) {
      return kMechanicsStatus;
    }

    Model model;
    std::uint8_t pole_pairs = 0U;
    switch (parameters.motor_type) {
      case system::MotorType::kPmsm:
        model = PmsmModel(parameters.pmsm);
        pole_pairs = parameters.pmsm.pole_pairs;
        break;
      case system::MotorType::kAsm:
      case system::MotorType::kEesm:
      default:
        return Status::kUnsupportedMotorType;
    }
    const Status kModelStatus = std::visit(ValidateVisitor{}, model);
    if (kModelStatus != Status::kOk) {
      return kModelStatus;
    }

    parameters_ = parameters;
    model_ = model;
    pole_pairs_ = pole_pairs;
    mechanics_ = kMechanics;
    hall_ = HallSensor(parameters.hall);
    Reset();
    return Status::kOk;
  }

  /**
   * @brief Configures the simulation for the motor type and parameters of the settings.
   * @param settings Drive settings.
   * @return The result of Configure().
   */
  constexpr Status Configure(const settings::NvmSettings& settings) noexcept {
    return Configure(ParametersFromSettings(settings));
  }

  /**
   * @brief Returns whether a configuration was applied.
   * @return `true` after a successful Configure().
   */
  [[nodiscard]] constexpr bool IsConfigured() const noexcept {
    return !std::holds_alternative<std::monostate>(model_);
  }

  /**
   * @brief Returns the simulated motor type.
   * @return The motor type of the active configuration.
   */
  [[nodiscard]] constexpr system::MotorType Type() const noexcept { return parameters_.motor_type; }

  /**
   * @brief Returns the active parameters.
   * @return The parameters of the last successful Configure().
   */
  [[nodiscard]] constexpr const SimulationParameters& Parameters() const noexcept { return parameters_; }

  /**
   * @brief Returns all states to standstill at zero angle, with zero current and the bridge off.
   *
   * The configuration, the shaft mode and the load input are kept.
   */
  constexpr void Reset() noexcept {
    std::visit(ResetVisitor{}, model_);
    mechanics_.Reset();
    inverter_ = InverterModel{};
    UpdateOutput();
    hall_.Reset(output_.electrical_angle);
    output_.hall = hall_.Signals();
  }

  /**
   * @brief Holds new phase duties until the next call.
   * @param duties Duty of each half-bridge.
   */
  constexpr void SetPhaseDuties(const system::ThreePhase<unit::Ratio>& duties) noexcept {
    inverter_.SetDuties(duties);
  }

  /**
   * @brief Returns the held phase duties.
   * @return The clamped duties.
   */
  [[nodiscard]] constexpr const system::ThreePhase<unit::Ratio>& PhaseDuties() const noexcept {
    return inverter_.Duties();
  }

  /**
   * @brief Switches the inverter bridge on or off.
   * @param enabled `false` disconnects the stator.
   */
  constexpr void SetBridgeEnabled(bool enabled) noexcept { inverter_.SetEnabled(enabled); }

  /**
   * @brief Sets the DC-link voltage of the ideal supply.
   * @param voltage DC-link voltage; non-positive values are ignored.
   */
  constexpr void SetDcLinkVoltage(unit::Voltage voltage) noexcept {
    if (voltage > unit::Voltage{}) {
      parameters_.dc_link_voltage = voltage;
      output_.dc_link_voltage = voltage;
    }
  }

  /**
   * @brief Sets the external inputs.
   * @param input Load torque and field voltage, held until the next call.
   */
  constexpr void SetInput(const SimulationInput& input) noexcept { input_ = input; }

  /**
   * @brief Selects how the shaft speed is determined.
   * @param mode Free shaft or imposed speed.
   * @param imposed_speed Mechanical speed used in ShaftMode::kImposedSpeed.
   */
  constexpr void SetShaftMode(ShaftMode mode, unit::AngularVelocity imposed_speed = {}) noexcept {
    mechanics_.SetShaftMode(mode, imposed_speed);
    UpdateOutput();
  }

  /**
   * @brief Places the shaft at an angle and speed without reporting a Hall edge.
   * @param mechanical_angle Mechanical shaft angle.
   * @param mechanical_speed Mechanical speed; ignored in ShaftMode::kImposedSpeed.
   */
  constexpr void SetMechanicalState(const system::RotorAngle& mechanical_angle,
                                    unit::AngularVelocity mechanical_speed) noexcept {
    mechanics_.SetState(mechanical_angle, mechanical_speed);
    UpdateOutput();
    hall_.Reset(output_.electrical_angle);
    output_.hall = hall_.Signals();
  }

  /**
   * @brief Advances the simulation by one step.
   * @param time_step Step size, normally half a PWM period.
   * @return Status::kOk, Status::kInvalidTimeStep for a non-positive step, or
   * Status::kUnsupportedMotorType when the simulation is not configured.
   */
  constexpr Status Step(unit::Time time_step) noexcept {
    if (!IsConfigured()) {
      return Status::kUnsupportedMotorType;
    }
    if (time_step <= unit::Time{}) {
      return Status::kInvalidTimeStep;
    }

    const unit::AngularVelocity kStartSpeed = ElectricalSpeed();
    // Hold the mechanical state over the step and sample the rotor angle at its middle.
    const ElectricalInput kElectricalInput{
        inverter_.StatorVoltage(parameters_.dc_link_voltage),
        output_.electrical_angle + (kStartSpeed * time_step * 0.5F),
        kStartSpeed,
        inverter_.Enabled(),
        input_.field_voltage,
    };
    std::visit([&kElectricalInput, time_step](auto& model) { StepModel(model, kElectricalInput, time_step); },
               model_);

    const ElectricalOutput kElectrical = std::visit(OutputVisitor{}, model_);
    mechanics_.Step(kElectrical.torque, input_.load_torque, time_step);

    UpdateOutput();
    hall_.Update(output_.electrical_angle, (kStartSpeed + output_.electrical_speed) * 0.5F, time_step);
    output_.hall = hall_.Signals();
    return Status::kOk;
  }

  /**
   * @brief Returns the measurable state at the end of the last step.
   * @return The simulation output.
   */
  [[nodiscard]] constexpr const SimulationOutput& Output() const noexcept { return output_; }

 private:
  /// Electrical models of all motor types; std::monostate while unconfigured.
  using Model = std::variant<std::monostate, PmsmModel>;

  /// Validates the parameters of the held model.
  struct ValidateVisitor {
    constexpr Status operator()(const std::monostate& /*unused*/) const noexcept {
      return Status::kUnsupportedMotorType;
    }
    template <MotorModel M>
    constexpr Status operator()(const M& model) const noexcept {
      return model.Validate();
    }
  };

  /// Resets the states of the held model.
  struct ResetVisitor {
    constexpr void operator()(std::monostate& /*unused*/) const noexcept {}
    template <MotorModel M>
    constexpr void operator()(M& model) const noexcept {
      model.Reset();
    }
  };

  /// Returns the outputs of the held model.
  struct OutputVisitor {
    constexpr ElectricalOutput operator()(const std::monostate& /*unused*/) const noexcept { return {}; }
    template <MotorModel M>
    constexpr ElectricalOutput operator()(const M& model) const noexcept {
      return model.Output();
    }
  };

  /**
   * @brief Steps a motor model; does nothing for the unconfigured state.
   * @param model Model to step.
   * @param input Electrical inputs of the step.
   * @param time_step Step size.
   */
  template <typename M>
  static constexpr void StepModel(M& model, const ElectricalInput& input, unit::Time time_step) noexcept {
    if constexpr (MotorModel<M>) {
      model.Step(input, time_step);
    }
  }

  /**
   * @brief Returns the electrical speed of the shaft.
   * @return The mechanical speed times the pole-pair count.
   */
  [[nodiscard]] constexpr unit::AngularVelocity ElectricalSpeed() const noexcept {
    return mechanics_.Speed() * static_cast<float>(pole_pairs_);
  }

  /** @brief Recomputes the output sample from the states. */
  constexpr void UpdateOutput() noexcept {
    const ElectricalOutput kElectrical = std::visit(OutputVisitor{}, model_);
    output_.electrical_angle = mechanics_.ElectricalAngle(pole_pairs_);
    output_.electrical_speed = ElectricalSpeed();
    output_.mechanical_angle = mechanics_.Angle();
    output_.mechanical_speed = mechanics_.Speed();
    output_.rotor_current = kElectrical.rotor_current;
    output_.phase_currents = kElectrical.rotor_current.ToStator(output_.electrical_angle).ToThreePhase();
    output_.phase_voltages = inverter_.PhaseVoltages(parameters_.dc_link_voltage);
    output_.dc_link_voltage = parameters_.dc_link_voltage;
    output_.field_current = kElectrical.field_current;
    output_.electromagnetic_torque = kElectrical.torque;
  }

  /// Active parameters.
  SimulationParameters parameters_;
  /// Electrical model of the configured motor type.
  Model model_;
  /// Pole-pair count of the configured motor.
  std::uint8_t pole_pairs_{0U};
  /// Inverter bridge.
  InverterModel inverter_;
  /// Shaft and load.
  MechanicalModel mechanics_;
  /// Hall sensors.
  HallSensor hall_;
  /// External inputs.
  SimulationInput input_;
  /// Output sample at the end of the last step.
  SimulationOutput output_;
};

}  // namespace unimoc::sim
