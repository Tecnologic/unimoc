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
#include <numbers>
#include <variant>
#include "dc_link_model.hpp"
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
  /// DC-link voltage of the ideal supply; the open-circuit battery voltage with a DC-link capacitor.
  unit::Voltage dc_link_voltage{unit::Voltage{48.0F}};
  /// Dead time of the bridge.
  InverterParameters inverter;
  /// DC-link capacitor and battery resistance; the default (no capacitor) is the ideal source.
  DcLinkParameters dc_link;
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
  /// Terminal voltages against the negative DC rail at the end of the step, as
  /// the phase-voltage sense measures them: the averaged bridge or diode output
  /// while the stator is connected; with the bridge open the back-EMF, its
  /// lowest phase at the negative rail (the low-side body diode carries the
  /// small current of the voltage dividers; its drop is not modelled).
  system::ThreePhase<unit::Voltage> terminal_voltages;
  /// DC-link voltage.
  unit::Voltage dc_link_voltage;
  /// Mean current the bridge took from the DC link over the last step; negative while it charges it.
  unit::Current dc_link_current;
  /// Battery current of the last step; negative while the battery is charged.
  unit::Current battery_current;
  /// True while the body diodes of the switched-off bridge rectify the back-EMF into the DC link.
  bool diodes_conducting{false};
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
    if (parameters.inverter.dead_time < unit::Ratio{} || !(parameters.inverter.dead_time < kMaxDeadTime)) {
      return Status::kInvalidDeadTime;
    }
    const DcLinkModel kDcLink(parameters.dc_link, parameters.dc_link_voltage);
    const Status kDcLinkStatus = kDcLink.Validate();
    if (kDcLinkStatus != Status::kOk) {
      return kDcLinkStatus;
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
    dc_link_ = kDcLink;
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
   * The configuration, the shaft mode and the load input are kept; the
   * DC-link capacitor is charged to the battery voltage, the contactor closed.
   */
  constexpr void Reset() noexcept {
    std::visit(ResetVisitor{}, model_);
    mechanics_.Reset();
    inverter_ = InverterModel{};
    inverter_.SetDeadTime(parameters_.inverter.dead_time);
    dc_link_.Reset();
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
   * @brief Sets the DC-link voltage of the ideal supply, or the battery voltage behind the capacitor.
   * @param voltage Voltage; non-positive values are ignored.
   */
  constexpr void SetDcLinkVoltage(unit::Voltage voltage) noexcept {
    if (voltage > unit::Voltage{}) {
      parameters_.dc_link_voltage = voltage;
      dc_link_.SetBatteryVoltage(voltage);
      output_.dc_link_voltage = dc_link_.Voltage();
    }
  }

  /**
   * @brief Opens or closes the contactor between the battery and the DC-link capacitor.
   *
   * No effect on the ideal source (no capacitor configured).
   *
   * @param closed `false` leaves the capacitor alone with the inverter.
   */
  constexpr void SetContactorClosed(bool closed) noexcept { dc_link_.SetContactorClosed(closed); }

  /**
   * @brief Returns whether the contactor is closed.
   * @return `true` while the battery is connected.
   */
  [[nodiscard]] constexpr bool ContactorClosed() const noexcept { return dc_link_.ContactorClosed(); }

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
    const unit::Voltage kDcLink = dc_link_.Voltage();
    const system::ThreePhase<unit::Current> kStartCurrents = output_.phase_currents;
    // Hold the mechanical state over the step and sample the rotor angle at its middle.
    const system::RotorAngle kAngle = output_.electrical_angle + (kStartSpeed * time_step * 0.5F);
    // Duties that connect the phases to the DC link: the bridge's, the body
    // diodes' of an open bridge, or none (the stator is open).
    system::ThreePhase<unit::Ratio> conduction{};
    bool connected = inverter_.Enabled();
    system::Stator<unit::Voltage> stator_voltage = inverter_.StatorVoltage(kDcLink, kStartCurrents);
    output_.diodes_conducting = false;
    if (connected) {
      conduction = inverter_.Duties();
    } else if (BodyDiodesConduct(kDcLink, kStartSpeed)) {
      conduction = DiodeDuties(kStartSpeed, kAngle);
      InverterModel diodes;
      diodes.SetEnabled(true);
      diodes.SetDuties(conduction);
      stator_voltage = diodes.StatorVoltage(kDcLink);
      connected = true;
      output_.diodes_conducting = true;
    }
    const ElectricalInput kElectricalInput{
        stator_voltage,
        kAngle,
        kStartSpeed,
        connected,
        input_.field_voltage,
    };
    const Model kBefore = model_;
    std::visit([&kElectricalInput, time_step](auto& model) { StepModel(model, kElectricalInput, time_step); },
               model_);
    if (output_.diodes_conducting && DcCurrent(conduction, kStartCurrents, kAngle) > unit::Current{}) {
      // The diodes would feed the motor from the DC link: they block, the stator is open.
      model_ = kBefore;
      ElectricalInput open = kElectricalInput;
      open.stator_connected = false;
      std::visit([&open, time_step](auto& model) { StepModel(model, open, time_step); }, model_);
      conduction = {};
      output_.diodes_conducting = false;
    }

    const ElectricalOutput kElectrical = std::visit(OutputVisitor{}, model_);
    mechanics_.Step(kElectrical.torque, input_.load_torque, time_step);

    UpdateOutput();
    output_.terminal_voltages = TerminalVoltages(conduction, connected, kDcLink);
    // The DC link carries the mean of the phase currents through the conducting switches.
    const auto kMean = [](unit::Current start, unit::Current end) { return (start + end) * 0.5F; };
    const unit::Current kDcCurrent = (conduction.a * kMean(kStartCurrents.a, output_.phase_currents.a)) +
                                     (conduction.b * kMean(kStartCurrents.b, output_.phase_currents.b)) +
                                     (conduction.c * kMean(kStartCurrents.c, output_.phase_currents.c));
    dc_link_.Step(kDcCurrent, time_step);
    output_.dc_link_voltage = dc_link_.Voltage();
    output_.dc_link_current = kDcCurrent;
    output_.battery_current = dc_link_.BatteryCurrent();
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

  /// Largest dead time accepted: half a PWM period.
  static constexpr unit::Ratio kMaxDeadTime = unit::operator""_ratio(0.5L);

  /**
   * @brief Whether the body diodes of the open bridge rectify the back-EMF.
   *
   * Only with a DC-link capacitor (the ideal source keeps the behavior of an
   * open stator) and a permanent magnet: the line-to-line peak sqrt(3) |w psi|
   * exceeds the DC link.
   */
  [[nodiscard]] constexpr bool BodyDiodesConduct(unit::Voltage dc_link,
                                                 unit::AngularVelocity speed) const noexcept {
    if (dc_link_.Ideal() || parameters_.motor_type != system::MotorType::kPmsm) {
      return false;
    }
    return unit::Abs(speed * parameters_.pmsm.flux_pm) * std::numbers::sqrt3_v<float> > dc_link;
  }

  /**
   * @brief Duties of the conducting diodes: each phase on the rail its back-EMF points to.
   *
   * An averaged six-step rectifier: a phase with positive back-EMF conducts
   * through its high-side diode (duty 1), else through its low-side diode. A
   * step in which this would take power from the DC link is repeated with the
   * stator open, so the diodes only return energy; the DC link charges up to
   * about pi/2 times the phase back-EMF peak (the line-to-line peak, sqrt(3)
   * times it, in the hardware). Low fidelity: enough for the overvoltage trip.
   */
  [[nodiscard]] constexpr system::ThreePhase<unit::Ratio> DiodeDuties(
      unit::AngularVelocity speed, const system::RotorAngle& angle) const noexcept {
    const system::Rotor<unit::Voltage> kBackEmf{unit::Voltage{}, speed * parameters_.pmsm.flux_pm};
    const system::ThreePhase<unit::Voltage> kPhases = kBackEmf.ToStator(angle).ToThreePhase();
    const auto kRail = [](unit::Voltage emf) {
      using unit::operator""_ratio;
      return emf > unit::Voltage{} ? 1.0_ratio : 0.0_ratio;
    };
    return {kRail(kPhases.a), kRail(kPhases.b), kRail(kPhases.c)};
  }

  /**
   * @brief Mean DC-link current of a step, from the model's present stator current.
   * @param conduction Duties connecting the phases to the DC link.
   * @param start Phase currents at the start of the step.
   * @param angle Electrical angle of the step (its middle).
   */
  [[nodiscard]] constexpr unit::Current DcCurrent(const system::ThreePhase<unit::Ratio>& conduction,
                                                  const system::ThreePhase<unit::Current>& start,
                                                  const system::RotorAngle& angle) const noexcept {
    const ElectricalOutput kElectrical = std::visit(OutputVisitor{}, model_);
    const system::ThreePhase<unit::Current> kEnd = kElectrical.rotor_current.ToStator(angle).ToThreePhase();
    return (conduction.a * (start.a + kEnd.a) * 0.5F) + (conduction.b * (start.b + kEnd.b) * 0.5F) +
           (conduction.c * (start.c + kEnd.c) * 0.5F);
  }

  /**
   * @brief Terminal voltages against the negative rail at the end of the step.
   * @param conduction Duties of the conducting body diodes (used when the bridge is off).
   * @param connected The bridge or the diodes connect the stator to the DC link.
   * @param dc_link DC-link voltage of the step.
   */
  [[nodiscard]] constexpr system::ThreePhase<unit::Voltage> TerminalVoltages(
      const system::ThreePhase<unit::Ratio>& conduction,
      bool connected,
      unit::Voltage dc_link) const noexcept {
    if (inverter_.Enabled()) {
      return inverter_.TerminalVoltages(dc_link, output_.phase_currents);
    }
    if (connected) {
      return {dc_link * conduction.a, dc_link * conduction.b, dc_link * conduction.c};
    }
    if (parameters_.motor_type != system::MotorType::kPmsm) {
      return {};
    }
    // Open stator, no current: the windings carry the back-EMF alone.
    const system::Rotor<unit::Voltage> kBackEmf{unit::Voltage{},
                                                output_.electrical_speed * parameters_.pmsm.flux_pm};
    const system::ThreePhase<unit::Voltage> kPhases =
        kBackEmf.ToStator(output_.electrical_angle).ToThreePhase();
    const unit::Voltage kLowest = unit::Min(kPhases.a, unit::Min(kPhases.b, kPhases.c));
    return {kPhases.a - kLowest, kPhases.b - kLowest, kPhases.c - kLowest};
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
    output_.phase_voltages = inverter_.PhaseVoltages(dc_link_.Voltage(), output_.phase_currents);
    output_.dc_link_voltage = dc_link_.Voltage();
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
  /// DC link: capacitor, battery and contactor.
  DcLinkModel dc_link_;
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
