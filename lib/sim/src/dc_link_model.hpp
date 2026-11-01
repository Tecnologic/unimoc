/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file dc_link_model.hpp
 *	@brief DC link of the simulated drive: capacitor, battery and contactor.
 *
 *	SPDX-License-Identifier: GPL-3.0-or-later
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */
#pragma once

#include "motor_model.hpp"
#include "units.hpp"

namespace unimoc::sim {

using unit::operator""_Ohm;
using unit::operator""_V;

/**
 * @brief Parameters of the DC link.
 *
 * A capacitance of zero selects the ideal source: the DC link is the battery
 * voltage, whatever current the inverter takes or returns, and the contactor
 * has no effect (the behavior before the DC-link model).
 */
struct DcLinkParameters {
  /// DC-link capacitance; zero selects the ideal source.
  unit::Capacitance capacitance;
  /// Internal resistance of the battery, including the cables and the contactor.
  unit::Resistance battery_resistance{0.05_Ohm};
};

/**
 * @brief DC link with a capacitor, a battery with internal resistance and a contactor.
 *
 * The battery is an open-circuit voltage behind its internal resistance; the
 * contactor connects it to the capacitor. The inverter takes the current
 * i_dc from the capacitor (positive: the motor draws power). One step solves
 *
 *   C dv/dt = (V_bat - v) / R_bat - i_dc   (contactor closed)
 *   C dv/dt = -i_dc                        (contactor open)
 *
 * with backward Euler, which is stable for every step size. The battery
 * accepts charge as readily as it delivers it (no charge limit, no BMS). With
 * the contactor open the capacitor voltage cannot fall below zero: the body
 * diodes of the low-side switches clamp it.
 */
class DcLinkModel {
 public:
  /** @brief Constructs an ideal source at the default battery voltage. */
  constexpr DcLinkModel() = default;

  /**
   * @brief Constructs a DC link with the given parameters, charged to the battery voltage.
   * @param parameters Capacitance and battery resistance; check them with Validate().
   * @param battery_voltage Open-circuit voltage of the battery.
   */
  constexpr DcLinkModel(const DcLinkParameters& parameters, unit::Voltage battery_voltage) noexcept
      : parameters_(parameters), battery_voltage_(battery_voltage), voltage_(battery_voltage) {}

  /**
   * @brief Checks the parameters.
   * @return Status::kOk, kInvalidCapacitance for a negative capacitance or
   * kInvalidResistance for a non-positive battery resistance with a capacitor.
   */
  [[nodiscard]] constexpr Status Validate() const noexcept {
    if (parameters_.capacitance < unit::Capacitance{}) {
      return Status::kInvalidCapacitance;
    }
    if (!Ideal() && parameters_.battery_resistance <= unit::Resistance{}) {
      return Status::kInvalidResistance;
    }
    return Status::kOk;
  }

  /**
   * @brief Returns whether the DC link is the ideal source.
   * @return `true` for a capacitance of zero.
   */
  [[nodiscard]] constexpr bool Ideal() const noexcept {
    return parameters_.capacitance <= unit::Capacitance{};
  }

  /**
   * @brief Returns the parameters.
   * @return The capacitance and the battery resistance.
   */
  [[nodiscard]] constexpr const DcLinkParameters& Parameters() const noexcept { return parameters_; }

  /** @brief Charges the capacitor to the battery voltage and closes the contactor. */
  constexpr void Reset() noexcept {
    voltage_ = battery_voltage_;
    battery_current_ = {};
    contactor_closed_ = true;
  }

  /**
   * @brief Sets the open-circuit voltage of the battery.
   *
   * The ideal source follows at once; the capacitor follows through the
   * battery resistance while the contactor is closed.
   *
   * @param voltage Battery voltage; non-positive values are ignored.
   */
  constexpr void SetBatteryVoltage(unit::Voltage voltage) noexcept {
    if (voltage <= unit::Voltage{}) {
      return;
    }
    battery_voltage_ = voltage;
    if (Ideal()) {
      voltage_ = voltage;
    }
  }

  /**
   * @brief Returns the open-circuit voltage of the battery.
   * @return The battery voltage.
   */
  [[nodiscard]] constexpr unit::Voltage BatteryVoltage() const noexcept { return battery_voltage_; }

  /**
   * @brief Opens or closes the contactor between the battery and the capacitor.
   * @param closed `false` leaves the capacitor alone with the inverter.
   */
  constexpr void SetContactorClosed(bool closed) noexcept { contactor_closed_ = closed; }

  /**
   * @brief Returns whether the contactor is closed.
   * @return `true` while the battery is connected.
   */
  [[nodiscard]] constexpr bool ContactorClosed() const noexcept { return contactor_closed_; }

  /**
   * @brief Advances the DC link by one step.
   * @param inverter_current Mean current the inverter takes from the DC link over the step.
   * @param time_step Step size; must be positive.
   */
  constexpr void Step(unit::Current inverter_current, unit::Time time_step) noexcept {
    if (Ideal()) {
      voltage_ = battery_voltage_;
      battery_current_ = inverter_current;
      return;
    }
    const auto kCapacitiveConductance = parameters_.capacitance / time_step;
    if (contactor_closed_) {
      const auto kBatteryConductance = 1.0F / parameters_.battery_resistance;
      voltage_ = ((kCapacitiveConductance * voltage_) + (kBatteryConductance * battery_voltage_) -
                  inverter_current) /
                 (kCapacitiveConductance + kBatteryConductance);
      battery_current_ = (battery_voltage_ - voltage_) * kBatteryConductance;
    } else {
      voltage_ = voltage_ - (inverter_current / kCapacitiveConductance);
      battery_current_ = {};
    }
    if (voltage_ < unit::Voltage{}) {
      voltage_ = {};
    }
  }

  /**
   * @brief Returns the DC-link voltage at the end of the last step.
   * @return The capacitor voltage, or the battery voltage of the ideal source.
   */
  [[nodiscard]] constexpr unit::Voltage Voltage() const noexcept { return voltage_; }

  /**
   * @brief Returns the battery current of the last step.
   * @return The current out of the battery; negative while it is charged.
   */
  [[nodiscard]] constexpr unit::Current BatteryCurrent() const noexcept { return battery_current_; }

 private:
  /// Capacitance and battery resistance.
  DcLinkParameters parameters_;
  /// Open-circuit voltage of the battery.
  unit::Voltage battery_voltage_{48.0_V};
  /// Capacitor voltage (the DC link).
  unit::Voltage voltage_{48.0_V};
  /// Battery current of the last step.
  unit::Current battery_current_;
  /// Contactor state.
  bool contactor_closed_{true};
};

}  // namespace unimoc::sim
