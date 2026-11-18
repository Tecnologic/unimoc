/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file inverter_model.hpp
 *	@brief Averaged two-level three-phase inverter model.
 *
 *	SPDX-License-Identifier: GPL-3.0-or-later
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */
#pragma once

#include "stator_system.hpp"
#include "three_phase_system.hpp"
#include "units.hpp"

namespace unimoc::sim {

/**
 * @brief Non-ideal properties of the inverter bridge.
 */
struct InverterParameters {
  /// Dead time as a fraction of the PWM period, t_dead f_pwm (0 = ideal bridge, below 0.5).
  unit::Ratio dead_time;
};

/**
 * @brief Averaged model of a two-level three-phase voltage-source inverter.
 *
 * Each half-bridge applies its duty times the DC-link voltage to its phase
 * terminal, averaged over the step. The motor star point is isolated, so the
 * common-mode part of the terminal voltages does not reach the windings.
 *
 * The dead time shortens the pulse of a phase whose current flows out of the
 * bridge into the winding and lengthens it for the opposite current: the
 * terminal duty is d - sign(i) t_dead f_pwm (the error the dead-time
 * compensation of the drive corrects). Switching ripple, switch and diode
 * drops are not modelled. Duties are clamped to [0, 1].
 */
class InverterModel {
 public:
  /**
   * @brief Holds new phase duties until the next change, as the PWM shadow registers do.
   * @param duties Duty of each half-bridge, 0 = low side on, 1 = high side on.
   */
  constexpr void SetDuties(const system::ThreePhase<unit::Ratio>& duties) noexcept {
    duties_ = {Clamp(duties.a), Clamp(duties.b), Clamp(duties.c)};
  }

  /**
   * @brief Returns the held phase duties.
   * @return The clamped duties.
   */
  [[nodiscard]] constexpr const system::ThreePhase<unit::Ratio>& Duties() const noexcept { return duties_; }

  /**
   * @brief Switches the bridge on or off.
   * @param enabled `false` opens all switches; the stator is then disconnected.
   */
  constexpr void SetEnabled(bool enabled) noexcept { enabled_ = enabled; }

  /**
   * @brief Sets the dead time as a fraction of the PWM period.
   * @param dead_time t_dead f_pwm; zero for an ideal bridge.
   */
  constexpr void SetDeadTime(unit::Ratio dead_time) noexcept { dead_time_ = dead_time; }

  /**
   * @brief Returns the dead time as a fraction of the PWM period.
   * @return t_dead f_pwm.
   */
  [[nodiscard]] constexpr unit::Ratio DeadTime() const noexcept { return dead_time_; }

  /**
   * @brief Returns whether the bridge is switched on.
   * @return `true` while the bridge drives the stator.
   */
  [[nodiscard]] constexpr bool Enabled() const noexcept { return enabled_; }

  /**
   * @brief Returns the phase-to-star-point voltages.
   * @param dc_link_voltage DC-link voltage.
   * @param currents Phase currents into the winding; their signs set the dead-time error.
   * @return The averaged winding voltages, zero while the bridge is off.
   */
  [[nodiscard]] constexpr system::ThreePhase<unit::Voltage> PhaseVoltages(
      unit::Voltage dc_link_voltage, const system::ThreePhase<unit::Current>& currents = {}) const noexcept {
    if (!enabled_) {
      return {};
    }
    const system::ThreePhase<unit::Voltage> kTerminal = TerminalVoltages(dc_link_voltage, currents);
    const unit::Voltage kCommonMode = (kTerminal.a + kTerminal.b + kTerminal.c) / 3.0F;
    return {kTerminal.a - kCommonMode, kTerminal.b - kCommonMode, kTerminal.c - kCommonMode};
  }

  /**
   * @brief Returns the averaged terminal voltages against the negative DC rail.
   * @param dc_link_voltage DC-link voltage.
   * @param currents Phase currents into the winding; their signs set the dead-time error.
   * @return Each phase's duty (less its dead-time error) times the DC link, as
   *         the bridge drives it; the switch state alone, whether or not it is enabled.
   */
  [[nodiscard]] constexpr system::ThreePhase<unit::Voltage> TerminalVoltages(
      unit::Voltage dc_link_voltage, const system::ThreePhase<unit::Current>& currents = {}) const noexcept {
    return {dc_link_voltage * (duties_.a - DeadTimeError(currents.a)),
            dc_link_voltage * (duties_.b - DeadTimeError(currents.b)),
            dc_link_voltage * (duties_.c - DeadTimeError(currents.c))};
  }

  /**
   * @brief Returns the stator voltage vector.
   * @param dc_link_voltage DC-link voltage.
   * @param currents Phase currents into the winding (dead time).
   * @return The averaged voltage in the alpha/beta frame, zero while the bridge is off.
   */
  [[nodiscard]] constexpr system::Stator<unit::Voltage> StatorVoltage(
      unit::Voltage dc_link_voltage, const system::ThreePhase<unit::Current>& currents = {}) const noexcept {
    return PhaseVoltages(dc_link_voltage, currents).ToStator();
  }

 private:
  /**
   * @brief Limits a duty to the physical range.
   * @param duty Requested duty.
   * @return The duty clamped to [0, 1].
   */
  static constexpr unit::Ratio Clamp(unit::Ratio duty) noexcept {
    return duty.Clamp(unit::Ratio{0.0F}, unit::Ratio{1.0F});
  }

  /**
   * @brief Duty lost to the dead time for one phase current.
   * @param current Phase current into the winding.
   * @return dead time times the sign of the current; zero at zero current.
   */
  [[nodiscard]] constexpr unit::Ratio DeadTimeError(unit::Current current) const noexcept {
    if (current > unit::Current{}) {
      return dead_time_;
    }
    if (current < unit::Current{}) {
      return -dead_time_;
    }
    return {};
  }

  /// Held phase duties.
  system::ThreePhase<unit::Ratio> duties_{unit::Ratio{0.5F}, unit::Ratio{0.5F}, unit::Ratio{0.5F}};
  /// Dead time as a fraction of the PWM period.
  unit::Ratio dead_time_;
  /// Bridge state.
  bool enabled_{false};
};

}  // namespace unimoc::sim
