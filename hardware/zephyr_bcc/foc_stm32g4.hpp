/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file foc_stm32g4.hpp
 *    @brief TIM8 / ADC1..5 power stage driver of the battery case controller.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>

#include "hardware_interface.hpp"

namespace unimoc::hardware::foc_stm32g4 {

/**
 * @brief Brings up the pins, TIM8, ADC1..5 and both zero-latency interrupts.
 *
 * Timing, all derived from the PWM frequency:
 *  - TIM8 runs center aligned (mode 3). One PWM period is one up/down cycle.
 *  - CC4 toggles OC4REF once per period; OC4REF is TRGO and starts the
 *    injected sequence of ADC1..5 (JEXTSEL 9, rising edge).
 *  - The end of the ADC1 injected sequence raises the fast interrupt.
 *  - With RCR = 2 * kSlowUpdateDivider - 1 the TIM8 update event, and with it
 *    the slow interrupt, occurs every kSlowUpdateDivider PWM periods.
 *
 * Both interrupts are zero-latency IRQs: they run outside of the kernel and
 * the callbacks must not use any kernel API (no logging, no k_* calls).
 *
 * @param pwm_frequency        PWM frequency (one fast interrupt per period).
 * @param adc_trigger_offset   CC4 value in timer ticks.
 * @param current_control      Fast callback.
 * @param slow_update          Slow callback.
 * @return false on invalid arguments or if an ADC fails to calibrate or enable.
 */
[[nodiscard]] bool Initialize(unit::Frequency pwm_frequency,
                              uint32_t adc_trigger_offset,
                              HardwareInterface::CurrentControlCallback current_control,
                              HardwareInterface::SlowUpdateCallback slow_update) noexcept;

[[nodiscard]] uint32_t TimerClockFrequency() noexcept;

/// Auto-reload value of the running timer (full scale of the duty compare values).
[[nodiscard]] uint32_t TimerAutoReload() noexcept;

void SetCompare(uint32_t channel, uint32_t ticks) noexcept;  ///< channel 1..3: phase A..C, 4: ADC trigger
[[nodiscard]] uint32_t GetCompare(uint32_t channel) noexcept;

/// Raw injected results (12 bit, unsigned, right aligned) of the last sequence.
struct RawSamples {
  uint32_t ia, va, ib, vb, ic, vc, vdc, crank_torque, motor_temperature, bridge_temperature;
};

[[nodiscard]] RawSamples ReadRaw() noexcept;

}  // namespace unimoc::hardware::foc_stm32g4
