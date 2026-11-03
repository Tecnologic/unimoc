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
 *    The control step runs every `divider` PWM half periods and always writes
 *    its duties into the CCR preload (docs/hotpath_plan.md, section 2):
 *
 *     - divider 1: a step after every peak and every valley sample; the timer
 *       takes the preload over at every boundary (RCR = 0). The fastest loop;
 *       the duties change in the middle of a pulse.
 *     - divider 2, 4 or 8: a step after every first, second or fourth valley
 *       sample (the middle of the high-side pulses); the timer takes the
 *       preload over at the peak only (RCR = 1). Every pulse stays symmetric,
 *       the sample is the mean of the ripple.
 *
 *    The observer runs after every `steps_per_cycle` control steps.
 *
 *    Low-side shunts (HardwareCapabilities::current_sense) measure a phase
 *    only while its low side conducts. The STM32H7 then inverts the PWM (mode
 *    2: the low-side pulses centered on the valley), so the valley sample is
 *    the middle of the low-side pulses; the peak sample measures nothing.
 *    Divider 1 is not runnable, and the duty is capped so every phase keeps
 *    min_low_side_time of low-side on time per period (`duty_max`).
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

/// Control dividers in PWM half periods per control step: every half period, every period, every second or
/// fourth.
inline constexpr std::array<uint8_t, 4> kControlDividers{1U, 2U, 4U, 8U};

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
  /// PWM half periods per control step (1, 2, 4 or 8); 0: not runnable.
  uint8_t divider{0U};
  /// Control steps per observer cycle (1, 2, 4 or 8).
  uint8_t steps_per_cycle{1U};
  /// Highest duty the current sensing allows: below 1 for low-side shunts.
  unit::Ratio duty_max{1.0_ratio};

  /// True when the board runs this timing.
  [[nodiscard]] constexpr bool Valid() const noexcept { return divider != 0U; }

  /// True for a control step after every peak and valley sample (divider 1).
  [[nodiscard]] constexpr bool EveryHalfPeriod() const noexcept { return divider == 1U; }

  /// Calls of the sample handler per control step: every sample at divider 1, else divider / 2 valleys.
  [[nodiscard]] constexpr uint8_t CallsPerStep() const noexcept {
    return EveryHalfPeriod() ? 1U : static_cast<uint8_t>(divider / 2U);
  }

  /// Period of one control step: divider PWM half periods.
  [[nodiscard]] constexpr unit::Time StepPeriod() const noexcept {
    return static_cast<float>(divider) / (2.0F * pwm_frequency);
  }

  /// Rate of the control steps.
  [[nodiscard]] constexpr unit::Frequency StepFrequency() const noexcept {
    return 2.0F * pwm_frequency / static_cast<float>(divider);
  }

  /// Period of one observer cycle: steps_per_cycle control steps.
  [[nodiscard]] constexpr unit::Time CyclePeriod() const noexcept {
    return StepPeriod() * static_cast<float>(steps_per_cycle);
  }

  /**
   * @brief Delay from a sample to the middle of the interval its duties apply to, in control steps.
   *
   * The duties take effect at the next update, one half period T_h after the
   * sample (the next boundary at divider 1, the peak after the valley else),
   * and hold for divider half periods: T_h + divider T_h / 2, 1.5 steps at
   * divider 1, one step at divider 2.
   */
  [[nodiscard]] constexpr float OutputDelaySteps() const noexcept {
    return 0.5F + (1.0F / static_cast<float>(divider));
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
 * kControlDividers, a control step faster than the board's
 * max_control_frequency, divider 1 on low-side shunts, or a low-side on time
 * that leaves no duty above 50 %.
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
  const bool kLowSide = limits.current_sense == CurrentSense::kLowSide;
  if (kLowSide) {
    timing.duty_max = 1.0_ratio - (timing.pwm_frequency * limits.min_low_side_time);
    if (!(timing.duty_max > 0.5_ratio)) {
      return ControlTiming{};
    }
  }
  for (const uint8_t kDivider : kControlDividers) {
    const bool kSelected = (settings.control_divider == 0U) || (settings.control_divider == kDivider);
    const bool kSampled = !kLowSide || kDivider != 1U;  // low-side shunts: the valley only
    if (kSelected && kSampled &&
        !(2.0F * timing.pwm_frequency / static_cast<float>(kDivider) > limits.max_control_frequency)) {
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
