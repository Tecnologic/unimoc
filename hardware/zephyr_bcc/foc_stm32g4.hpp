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

#include <array>
#include <cstdint>
#include "hardware_interface.hpp"
#include "power_stage_interface.hpp"

namespace unimoc::hardware::foc_stm32g4 {

/**
 * @brief Brings up the pins, TIM8, ADC1..5, the duty DMA and the interrupts.
 *
 * Timing, all derived from the PWM frequency (T = one PWM period):
 *  - TIM8 runs center aligned (mode 3) with RCR = 0. The counter reverses at
 *    the peak (CNT = ARR) and at the valley (CNT = 0); both are update events.
 *  - CCR1..3 are preloaded: a written duty takes effect at the next update.
 *  - The ADC1..5 injected sequence is triggered twice per period by TRGO2,
 *    a lead of adc_trigger_offset ticks before each boundary: CC4 before the
 *    peak (up-counting) and CC6 before the valley (down-counting).
 *  - The end of the ADC1 injected sequence raises the sample interrupt (zero
 *    latency). It copies the raw phase currents into the cycle array and
 *    compares them with the overcurrent window; a violation trips the power
 *    stage at once. Every kHalfPeriodsPerCycle-th sample it calls
 *    ControlCycleHandler() directly and pends the observer interrupt.
 *  - Every update event requests a DMA burst of three words (TIM8_DCR/DMAR)
 *    into CCR1..3 from a circular buffer of kHalfPeriodsPerCycle slots. The
 *    control cycle writes duty entry 0 directly into the preload (after the
 *    burst of the control boundary) and entries 1..3 into the slots of the
 *    next three update events, see SetDutyTicks().
 *  - The observer interrupt (TIM8 update vector, pended by software; the
 *    update interrupt enable UIE stays 0) is a normal Zephyr interrupt at the
 *    highest normal priority; it calls ObserverHandler(). The sample interrupt
 *    preempts it.
 *
 * Why around the boundaries: the samples are centered on the boundary, where
 * the phases with a mid-range duty do not switch. A phase with an extreme
 * duty switches close to the sample; the hardware layer then reconstructs it
 * from the other two (see SampleAtPeak()/AppliedTicks()).
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
 * @name Power stage release
 * The outputs stay off after Initialize(). The external enable is the break
 * input TIM8_BKIN (active low, internal pull-down): when it drops, the timer
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

/// Compare values of the kHalfPeriodsPerCycle half periods after a control boundary.
using TickPlan = std::array<PhaseTicks, kHalfPeriodsPerCycle>;

/**
 * @brief Writes the compare values of the next kHalfPeriodsPerCycle half periods.
 *
 * Call only from the control cycle. Entries 1..3 go into the DMA slots of
 * the next three update events. Entry 0 goes directly into the CCR preload
 * when the burst of the control boundary is done (normal case), or into its
 * DMA slot when the control cycle runs before that boundary (long trigger lead).
 * Anything else counts as an overrun (DutyOverruns()).
 */
void SetDutyTicks(const TickPlan& ticks) noexcept;

/// Control cycles whose entry 0 missed its half period.
[[nodiscard]] uint32_t DutyOverruns() noexcept;

/// Raw phase-current conversions (12 bit, right aligned) of one sample.
struct RawCurrents {
  uint16_t a, b, c;
};

/// Raw values of the control cycle that just ended (valid in the control cycle).
struct CycleRaw {
  std::array<RawCurrents, kHalfPeriodsPerCycle> currents;  ///< oldest first
  uint32_t vdc;                                            ///< of the last sample
  bool overcurrent;                                        ///< a sample left the window
};

[[nodiscard]] const CycleRaw& CycleSamples() noexcept;

/// Compare values that were active while sample k of the current cycle was taken.
[[nodiscard]] const PhaseTicks& AppliedTicks(uint8_t sample) noexcept;

/// true when sample k of the current cycle was taken before a peak, false before a valley.
[[nodiscard]] bool SampleAtPeak(uint8_t sample) noexcept;

/**
 * @brief Raw window of the per-sample overcurrent check.
 *
 * A phase conversion below @p low or above @p high trips the power stage.
 * low = 0 and high = 4095 disable the check.
 */
void SetOvercurrentWindow(uint32_t low, uint32_t high) noexcept;

/// Raw injected results (12 bit, unsigned, right aligned) of the last sequence.
struct RawSamples {
  uint32_t ia, va, ib, vb, ic, vc, vdc, crank_torque, motor_temperature, bridge_temperature;
};

[[nodiscard]] RawSamples ReadRaw() noexcept;

}  // namespace unimoc::hardware::foc_stm32g4
