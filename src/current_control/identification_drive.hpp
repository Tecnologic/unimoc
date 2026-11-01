/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file identification_drive.hpp
 *    @brief What the motor identification asks of the control cycle and the observer.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include <numbers>
#include "impedance_estimator.hpp"
#include "rotor_system.hpp"
#include "sin_cos.hpp"
#include "sub_step_buffer.hpp"
#include "units.hpp"

namespace unimoc::current_control {

/// Axis of the injected sine of M2.
enum class InjectionAxis : uint8_t {
  kD = 0,
  kQ = 1,
};

/**
 * @brief The control cycle under the motor identification (docs/motor_identification_plan.md, section 5).
 *
 * While active, the control cycle takes this current reference instead of the
 * outer loop's; the angle comes from the forced angle of SlowUpdate.
 */
struct IdentificationDrive {
  /// Current reference in the forced rotor frame.
  system::Rotor<unit::Current> current_ref;
  /// true: the current PI is frozen and its last output is held (open loop).
  bool hold_voltage{false};
  /// false: no dead-time compensation (M1 measures the dead-time voltage).
  bool dead_time_compensation{true};
  /// Axis of the injected sine.
  InjectionAxis injection_axis{InjectionAxis::kD};
  /// Amplitude of the injected sine; zero: none.
  unit::Voltage injection_amplitude;
};

/**
 * @brief Phase of the injected sine, 2^32 counts per period.
 *
 * The sine has a fixed period of kInjectionHalfPeriods PWM half periods
 * (f_pwm / 64, 500 Hz at 32 kHz), counted from the first control cycle after
 * Init(): exact and drift-free in integer counts, the same in the control
 * cycle that generates it and in the observer that demodulates it.
 */
inline constexpr uint32_t kInjectionHalfPeriods = 128U;
/// Control cycles of one injection period.
inline constexpr uint32_t kInjectionPeriodCycles = kInjectionHalfPeriods / kNumSubSteps;
/// Phase advance of one PWM half period.
inline constexpr uint32_t kInjectionPhaseStep = 0x1'0000'0000ULL / kInjectionHalfPeriods;

/**
 * @brief Phase at the middle of duty entry @p entry of control cycle @p cycle.
 *
 * Entry k takes effect k + 1 half periods after the control boundary and
 * holds for one half period (hardware::DutyPlan); the boundary of cycle n is
 * half period 4 n.
 */
[[nodiscard]] constexpr uint32_t InjectionEntryPhase(const uint32_t cycle, const uint8_t entry) noexcept {
  return ((cycle * kNumSubSteps) + entry + 1U) * kInjectionPhaseStep + (kInjectionPhaseStep / 2U);
}

/**
 * @brief Phase at current sample @p sample of the samples control cycle @p cycle works on.
 *
 * Sample k was taken at the (k + 1)-th boundary after the previous control
 * boundary (hardware::ControlCycleSamples).
 */
[[nodiscard]] constexpr uint32_t InjectionSamplePhase(const uint32_t cycle, const uint8_t sample) noexcept {
  return ((cycle * kNumSubSteps) + sample + 1U - kNumSubSteps) * kInjectionPhaseStep;
}

/// Sine and cosine of an injection phase (no libm: the table of the hot path).
[[nodiscard]] constexpr system::SinCos<unit::Ratio> InjectionSinCos(const uint32_t phase) noexcept {
  return system::PortableSinCosProvider::Calculate(static_cast<int32_t>(phase));
}

/// Angular frequency of the injection at a PWM frequency.
[[nodiscard]] constexpr unit::AngularVelocity InjectionAngularFrequency(const unit::Frequency pwm) noexcept {
  using unit::operator""_rad;
  return pwm * 2.0F / static_cast<float>(kInjectionHalfPeriods) *
         (2.0F * std::numbers::pi_v<float> * 1.0_rad);
}

/// Sums of one measurement window of the observer.
struct IdentificationWindowResult {
  uint32_t cycles{0U};                        ///< Control cycles in the window.
  system::Rotor<unit::Current> current_mean;  ///< Mean current in the forced frame.
  system::Rotor<unit::Voltage> voltage_mean;  ///< Mean voltage demand (PI or held, without the sine).
  unit::Voltage dc_link_mean;                 ///< Mean DC-link voltage.
  identification::PhasorAccumulator<unit::Voltage> voltage_phasor{};  ///< Injected voltage of the axis.
  identification::PhasorAccumulator<unit::Current> current_phasor{};  ///< Current of the axis.
};

}  // namespace unimoc::current_control
