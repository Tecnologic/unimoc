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
 * @brief Averaged model of a two-level three-phase voltage-source inverter.
 *
 * Each half-bridge applies its duty times the DC-link voltage to its phase
 * terminal, averaged over the step. The motor star point is isolated, so the
 * common-mode part of the terminal voltages does not reach the windings.
 *
 * Switching ripple, dead time, switch and diode drops, and DC-link ripple are
 * not modelled. Duties are clamped to [0, 1].
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
   * @brief Returns whether the bridge is switched on.
   * @return `true` while the bridge drives the stator.
   */
  [[nodiscard]] constexpr bool Enabled() const noexcept { return enabled_; }

  /**
   * @brief Returns the phase-to-star-point voltages.
   * @param dc_link_voltage DC-link voltage.
   * @return The averaged winding voltages, zero while the bridge is off.
   */
  [[nodiscard]] constexpr system::ThreePhase<unit::Voltage> PhaseVoltages(
      unit::Voltage dc_link_voltage) const noexcept {
    if (!enabled_) {
      return {};
    }
    const unit::Voltage kTerminalA = dc_link_voltage * duties_.a;
    const unit::Voltage kTerminalB = dc_link_voltage * duties_.b;
    const unit::Voltage kTerminalC = dc_link_voltage * duties_.c;
    const unit::Voltage kCommonMode = (kTerminalA + kTerminalB + kTerminalC) / 3.0F;
    return {kTerminalA - kCommonMode, kTerminalB - kCommonMode, kTerminalC - kCommonMode};
  }

  /**
   * @brief Returns the stator voltage vector.
   * @param dc_link_voltage DC-link voltage.
   * @return The averaged voltage in the alpha/beta frame, zero while the bridge is off.
   */
  [[nodiscard]] constexpr system::Stator<unit::Voltage> StatorVoltage(
      unit::Voltage dc_link_voltage) const noexcept {
    return PhaseVoltages(dc_link_voltage).ToStator();
  }

 private:
  /**
   * @brief Limits a duty to the physical range.
   * @param duty Requested duty.
   * @return The duty clamped to [0, 1].
   */
  static constexpr unit::Ratio Clamp(unit::Ratio duty) noexcept { return duty.Clamp(0.0F, 1.0F); }

  /// Held phase duties.
  system::ThreePhase<unit::Ratio> duties_{unit::Ratio{0.5F}, unit::Ratio{0.5F}, unit::Ratio{0.5F}};
  /// Bridge state.
  bool enabled_{false};
};

}  // namespace unimoc::sim
