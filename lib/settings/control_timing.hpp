/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file control_timing.hpp
 *    @brief PWM frequency, control divider and observer rate resolved from the settings.
 *
 *    The current control samples once per PWM period at the valley (the
 *    middle of the high-side pulses) and writes the duties into the CCR
 *    preload, which the timer takes over at the next peak only: every pulse
 *    stays symmetric, the sample is free of ripple. The control step runs on
 *    every `divider`-th PWM period; the observer after every
 *    `steps_per_cycle` control steps (docs/hotpath_plan.md, section 2).
 *
 *    `pwm_frequency` 0 selects the frequency from the inductance (the
 *    smallest one that keeps the current ripple within kPwmRippleTarget of
 *    the motor current limit), `control_divider` 0 the smallest divider the
 *    board's control-rate budget allows.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include "nvm_settings.hpp"
#include "settings_profile.hpp"
#include "units.hpp"

namespace unimoc::settings {

/// PWM frequencies the timer runs; `pwm_frequency` is one of them or 0 (automatic).
inline constexpr std::array<unit::Frequency, 8> kPwmFrequencies{
    16.0_kHz,
    20.0_kHz,
    24.0_kHz,
    28.0_kHz,
    32.0_kHz,
    40.0_kHz,
    48.0_kHz,
    64.0_kHz,
};

/// Control dividers: the control step runs on every first, second or fourth PWM period.
inline constexpr std::array<uint8_t, 3> kControlDividers{1U, 2U, 4U};

/// Control steps per observer cycle: the observer runs after 1, 2, 4 or 8 steps.
inline constexpr std::array<uint8_t, 4> kStepsPerCycle{1U, 2U, 4U, 8U};

/// Most control steps of one observer cycle (CycleRecord capacity).
inline constexpr uint8_t kMaxStepsPerCycle = 8U;

/// Fastest observer: the rate measured on the STM32H7 (docs/hotpath_plan.md, section 4).
inline constexpr unit::Frequency kMaxObserverFrequency{16000.0F};

/// Lowest frequency the automatic choice takes: above the audible range.
inline constexpr unit::Frequency kMinAutomaticPwmFrequency{20000.0F};

/// Peak-to-peak current ripple the automatic choice allows, a share of motor_i_max.
inline constexpr unit::Ratio kPwmRippleTarget{0.3_ratio};

/**
 * @brief Worst-case peak-to-peak phase current ripple of the space vector modulation.
 *
 * ΔI_pp ≈ 0.15 V_dc / (L f_pwm): the largest ripple over the hexagon, for a
 * star-connected motor with the inductance @p inductance per phase.
 */
[[nodiscard]] constexpr unit::Current PwmRipple(const unit::Voltage dc_link,
                                                const unit::Inductance inductance,
                                                const unit::Frequency pwm_frequency) noexcept {
  constexpr float kRippleFactor = 0.15F;
  return dc_link * kRippleFactor / (inductance * pwm_frequency);
}

/// Ripple of the settings at @p pwm_frequency: the smaller inductance, the highest operating voltage.
[[nodiscard]] constexpr unit::Current PwmRipple(const NvmSettings& settings,
                                                const unit::Frequency pwm_frequency) noexcept {
  return PwmRipple(settings.dc_link_regen_derate_start, unit::Min(settings.l_d, settings.l_q), pwm_frequency);
}

/**
 * @brief The PWM frequency for the motor of @p settings.
 *
 * The smallest frequency of kPwmFrequencies from kMinAutomaticPwmFrequency
 * up to the board's maximum that keeps PwmRipple() within kPwmRippleTarget ×
 * motor_i_max; the board's maximum when none does (the ripple is above the
 * target then).
 */
[[nodiscard]] constexpr unit::Frequency SuggestedPwmFrequency(const NvmSettings& settings,
                                                              const HardwareCapabilities& limits) noexcept {
  const unit::Current kAllowed = settings.motor_i_max * kPwmRippleTarget;
  unit::Frequency best{};
  for (const unit::Frequency kFrequency : kPwmFrequencies) {
    if (kFrequency < kMinAutomaticPwmFrequency || kFrequency > limits.max_pwm_frequency) {
      continue;
    }
    best = kFrequency;
    if (!(PwmRipple(settings, kFrequency) > kAllowed)) {
      break;
    }
  }
  return best;
}

/**
 * @brief The timing of the current control: PWM frequency, divider, steps per observer cycle.
 *
 * A zero divider marks settings the board cannot run.
 */
struct ControlTiming {
  /// PWM frequency of the timer.
  unit::Frequency pwm_frequency;
  /// PWM periods per control step (1, 2 or 4); 0: not runnable.
  uint8_t divider{0U};
  /// Control steps per observer cycle (1, 2, 4 or 8).
  uint8_t steps_per_cycle{1U};

  /// True when the board runs this timing.
  [[nodiscard]] constexpr bool Valid() const noexcept { return divider != 0U; }

  /// Period of one control step: divider PWM periods.
  [[nodiscard]] constexpr unit::Time StepPeriod() const noexcept {
    return static_cast<float>(divider) / pwm_frequency;
  }

  /// Rate of the control steps.
  [[nodiscard]] constexpr unit::Frequency StepFrequency() const noexcept {
    return pwm_frequency / static_cast<float>(divider);
  }

  /// Period of one observer cycle: steps_per_cycle control steps.
  [[nodiscard]] constexpr unit::Time CyclePeriod() const noexcept {
    return StepPeriod() * static_cast<float>(steps_per_cycle);
  }

  /**
   * @brief Delay from a sample to the middle of the interval its duties apply to, in control steps.
   *
   * The duties take effect at the peak half a PWM period after the sample
   * and hold for `divider` periods: 0.5 T + divider T / 2.
   */
  [[nodiscard]] constexpr float OutputDelaySteps() const noexcept {
    return 0.5F + (0.5F / static_cast<float>(divider));
  }
};

/// True when @p frequency is one of kPwmFrequencies.
[[nodiscard]] constexpr bool IsPwmFrequency(const unit::Frequency frequency) noexcept {
  return std::ranges::any_of(kPwmFrequencies, [frequency](const unit::Frequency kFrequency) {
    return kFrequency == frequency;
  });
}

/**
 * @brief Resolves the automatic choices of @p settings on the board of @p limits.
 *
 * Returns an invalid timing (divider 0) for a PWM frequency outside
 * kPwmFrequencies or above the board's maximum, a divider that is not one of
 * kControlDividers, or a control step faster than the board's
 * max_control_frequency.
 */
[[nodiscard]] constexpr ControlTiming ResolveControlTiming(const NvmSettings& settings,
                                                           const HardwareCapabilities& limits) noexcept {
  ControlTiming timing{};
  timing.pwm_frequency = (settings.pwm_frequency == unit::Frequency{})
                             ? SuggestedPwmFrequency(settings, limits)
                             : settings.pwm_frequency;
  if (!IsPwmFrequency(timing.pwm_frequency) || timing.pwm_frequency > limits.max_pwm_frequency) {
    return ControlTiming{};
  }
  for (const uint8_t kDivider : kControlDividers) {
    const bool kSelected = (settings.control_divider == 0U) || (settings.control_divider == kDivider);
    if (kSelected && !(timing.pwm_frequency / static_cast<float>(kDivider) > limits.max_control_frequency)) {
      timing.divider = kDivider;
      break;
    }
  }
  if (!timing.Valid()) {
    return ControlTiming{};
  }
  for (const uint8_t kSteps : kStepsPerCycle) {
    timing.steps_per_cycle = kSteps;
    if (!(timing.StepFrequency() / static_cast<float>(kSteps) > kMaxObserverFrequency)) {
      break;
    }
  }
  return timing;
}

}  // namespace unimoc::settings
