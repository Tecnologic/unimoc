/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file foc_stm32h7.hpp
 *    @brief TIM1/TIM8 / ADC1..3 power stage driver on the STM32H7.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cstdint>
#include "hardware_interface.hpp"
#include "power_stage_interface.hpp"

namespace unimoc::hardware::foc_stm32h7 {

/**
 * @brief Brings up the pins, the timer, ADC1..3 and the interrupts.
 *
 * Timing and interrupt layout of docs/hotpath_plan.md (taken over from the
 * former STM32G4 driver of the BCC); the timer is TIM1 or TIM8 of the
 * foc0 node and the ADCs run three injected conversions each.
 *
 * Timing, all derived from the PWM frequency (T = one PWM period):
 *  - The timer runs center aligned (mode 3) with RCR = 0. The counter reverses at
 *    the peak (CNT = ARR) and at the valley (CNT = 0); both are update events.
 *  - CCR1..3 are preloaded: a written duty takes effect at the next update.
 *  - The ADC1..3 injected sequences are triggered twice per period by TRGO2,
 *    a lead of adc_trigger_offset ticks before each boundary: CC4 before the
 *    peak (up-counting) and CC6 before the valley (down-counting).
 *  - The end of the phase A ADC injected sequence raises the sample interrupt (zero
 *    latency). It copies the raw phase currents and compares them with the
 *    overcurrent window; a violation trips the power stage at once. Then it
 *    calls SampleHandler() directly, which writes the duties of the next half
 *    period into the CCR preload (SetDutyTicks()), and pends the observer
 *    interrupt when the handler ended a control cycle.
 *  - The observer interrupt (timer update vector, pended by software; the
 *    update interrupt enable UIE stays 0) is a normal Zephyr interrupt at the
 *    highest normal priority; it calls ObserverHandler(). The sample interrupt
 *    preempts it.
 *
 * Why around the boundaries: the samples are centered on the boundary, where
 * the phases with a mid-range duty do not switch. A phase with an extreme
 * duty switches close to the sample; the hardware layer then reconstructs it
 * from the other two (see SampleAtPeak()/AppliedTicks()).
 *
 * The end of the injected sequence must come after the boundary of its
 * trigger (trigger lead shorter than the sequence): the duties written then
 * take effect at the next boundary. With a longer lead they take effect half a
 * period earlier.
 *
 * The outputs are held off (idle level, MOE = 0) until PowerStageEnable().
 *
 * The sample interrupt is a zero-latency IRQ: it runs outside of the kernel
 * and the handler must not use any kernel API (no logging, no k_* calls).
 *
 * @param pwm_frequency        PWM frequency.
 * @param adc_trigger_offset   Trigger lead before each boundary [timer ticks].
 * @return false on invalid arguments or if an ADC fails to calibrate or enable.
 */
[[nodiscard]] bool Initialize(unit::Frequency pwm_frequency, uint32_t adc_trigger_offset) noexcept;

/**
 * @brief Stops the sample and observer interrupts and the hardware they run on.
 *
 * Disables both interrupts, stops the timer (outputs off), stops
 * the injected sequences (JADSTP) and disables the ADCs (ADDIS), so that
 * Initialize() may calibrate and reprogram them: the reference manual allows
 * ADCAL only with ADEN = 0 and a JSQR write only with JADSTART = 0. Thread
 * context, power stage disabled. Does nothing before the first Initialize().
 */
void Stop() noexcept;

/**
 * @name Power stage release
 * The outputs stay off after Initialize(). The external enable is the break
 * input TIMx_BKIN (active low, internal pull-down): when it drops, the timer
 * clears MOE in hardware and latches BIF. PowerStageTrip() generates the same
 * break from software. All of these are plain register accesses.
 * @{
 */
[[nodiscard]] bool PowerStageEnable() noexcept;
void PowerStageDisable() noexcept;
void PowerStageTrip() noexcept;
void PowerStageClearFault() noexcept;
[[nodiscard]] PowerStageState GetPowerStageState() noexcept;
/** @} */

[[nodiscard]] uint32_t TimerClockFrequency() noexcept;

/// Auto-reload value of the running timer (full scale of the duty compare values).
[[nodiscard]] uint32_t TimerAutoReload() noexcept;

/// Trigger lead [ticks] before the peak and the valley; clamped to 1..ARR/2.
void SetTriggerLead(uint32_t ticks) noexcept;

/// Applied trigger lead [ticks].
[[nodiscard]] uint32_t TriggerLead() noexcept;

/// Compare values of CCR1..3 for one half period.
using PhaseTicks = std::array<uint32_t, 3>;

/**
 * @brief Writes the compare values of the next half period into the CCR preload.
 *
 * Call only from the sample handler, after the boundary of the sample: the
 * values take effect at the next boundary. A write after that boundary
 * counts as an overrun (DutyOverruns()).
 */
void SetDutyTicks(const PhaseTicks& ticks) noexcept;

/// Samples whose duties missed their half period.
[[nodiscard]] uint32_t DutyOverruns() noexcept;

/// Raw values of one sample (valid in the sample handler).
struct SampleRaw {
  uint16_t ia, ib, ic;  ///< phase currents, 12 bit, right aligned
  uint32_t vdc;         ///< DC link
  bool overcurrent;     ///< a phase left the window
  bool overvoltage;     ///< the DC link reached the limit
  uint32_t timestamp;   ///< Timestamp() at the sample interrupt
};

[[nodiscard]] const SampleRaw& Sample() noexcept;

/**
 * @brief The timestamp clock of the drive: the DWT cycle counter.
 *
 * Counts core clock cycles (kTimestampFrequency) and wraps every 10.7 s at 400 MHz;
 * Initialize() starts it. A plain register read, usable in every context.
 */
[[nodiscard]] uint32_t Timestamp() noexcept;

/// Compare values that were active while the sample was taken.
[[nodiscard]] const PhaseTicks& AppliedTicks() noexcept;

/// true when the sample belongs to a peak, false when to a valley.
[[nodiscard]] bool SampleAtPeak() noexcept;

/**
 * @brief Raw window of the per-sample overcurrent check.
 *
 * A phase conversion below @p low or above @p high trips the power stage.
 * low = 0 and high = 4095 disable the check.
 */
void SetOvercurrentWindow(uint32_t low, uint32_t high) noexcept;

/// Raw DC-link count of SetOvervoltageCounts() that never trips (above the 12-bit range).
inline constexpr uint32_t kOvervoltageCheckOff = 0xFFFFFFFFU;

/**
 * @brief Raw limit of the per-sample DC-link overvoltage check.
 *
 * A DC-link conversion at or above @p counts trips the power stage in the
 * sample interrupt, like the overcurrent window. kOvervoltageCheckOff
 * disables the check.
 */
void SetOvervoltageCounts(uint32_t counts) noexcept;

/// Raw injected results (12 bit, unsigned, right aligned) of the last sequence.
struct RawSamples {
  uint32_t ia, va, bridge_temperature, ib, vb, motor_temperature, ic, vdc, vc;
};

[[nodiscard]] RawSamples ReadRaw() noexcept;

}  // namespace unimoc::hardware::foc_stm32h7
