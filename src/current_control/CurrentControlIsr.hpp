/*
       __  ___   ________  _______  ______
      / / / / | / /  _/  |/  / __ \/ ____/
     / / / /  |/ // // /|_/ / / / / /
    / /_/ / /|  // // /  / / /_/ / /___
    \____/_/ |_/___/_/  /_/\____/\____/

    Universal Motor Control  2026 Alexander <tecnologic86@gmail.com> Evers

    This file is part of UNIMOC.

    UNIMOC is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#ifndef UNIMOC_CURRENT_CONTROL_CURRENT_CONTROL_ISR_H_
#define UNIMOC_CURRENT_CONTROL_CURRENT_CONTROL_ISR_H_

#include <atomic>
#include <cstdint>
#include "NvmSettings.hpp"
#include "RotorReference.hpp"
#include "StatorReference.hpp"
#include "SubStepBuffer.hpp"
#include "MechanicalObserver.hpp"
#include "CurrentController.hpp"
#include "DeadTimeCompensation.hpp"
#include "Svm.hpp"
#include "Hfi.hpp"

/**
 * @namespace unimoc global namespace
 */
namespace unimoc
{
/**
 * @namespace current_control current-control subsystem namespace
 */
namespace current_control
{

/**
 * @brief Timer period in seconds derived from a PwmFrequency enumerator.
 *
 * Returns the period of one PWM half-period (i.e. the ISR interval), which
 * is 1 / (2 × f_pwm).
 *
 * @param freq  PWM frequency selection.
 * @return      dt_fast [s].
 */
[[nodiscard]] constexpr float
dt_fast_from_pwm_frequency(system::PwmFrequency freq) noexcept
{
    const auto f_khz = static_cast<uint32_t>(freq);
    return 1.0f / (2.0f * static_cast<float>(f_khz) * 1000.0f);
}

/**
 * @brief Slow-update period [s] = NUM_SUB_STEPS × dt_fast.
 */
[[nodiscard]] constexpr float
dt_slow_from_pwm_frequency(system::PwmFrequency freq) noexcept
{
    return static_cast<float>(NUM_SUB_STEPS) * dt_fast_from_pwm_frequency(freq);
}

/**
 * @brief Shared state block for the current-control ISR and the slow-update task.
 *
 * A single instance of this struct is owned by `CurrentControlIsr`.  Both the
 * ISR (on_jeoc) and the SlowUpdate task access it; the access discipline is
 * documented per member.
 *
 * Thread-safety model
 * -------------------
 *  - The ISR runs at the highest IRQ priority and is never preempted by the
 *    slow-update task.
 *  - `double_buf`  : atomic active index; SubStepBuffer access follows the
 *                    ownership protocol described in SubStepBuffer.hpp.
 *  - `i_ref`       : written by the outer control loop (lower priority);
 *                    read by the ISR.  A two-word RotorReference<float> on
 *                    Cortex-M is not atomically accessible; the outer loop
 *                    should use a critical section or double-buffered update.
 *  - `u_dq_last`   : written by the ISR; read by the slow-update task.
 *                    Protected by `samples_ready`: the slow-update task reads
 *                    it only after `samples_ready` is set, which happens at
 *                    sub-step 3 — after the last write.
 *  - `samples_ready`: set by the ISR at sub-step 3; cleared by SlowUpdate
 *                    at the start of its cycle.  Declared volatile so the
 *                    compiler does not optimise away the polling loop.
 */
struct CurrentControlState
{
    /// Double-buffered sin/cos precomputation + current sample storage.
    DoubleBuffer double_buf{};

    /// Current setpoint in the rotor frame [A].  Written by the outer loop.
    system::RotorReference<float> i_ref{0.0f, 0.0f};

    /// Last voltage demand computed by the ISR, rotor frame [V].
    /// Read by SlowUpdate for the PMSM flux observer.
    system::RotorReference<float> u_dq_last{0.0f, 0.0f};

    /// Set to true by the ISR at sub-step 3; cleared by SlowUpdate.
    volatile bool samples_ready{false};

    /// dt for one ISR call [s] — set by init().
    float dt_fast{1.0f / (2.0f * 20000.0f)};

    /// dt for one slow-update cycle [s] — set by init().
    float dt_slow{static_cast<float>(NUM_SUB_STEPS) / (2.0f * 20000.0f)};

    /// Timer auto-reload register value — set by init().
    uint32_t arr{4199u};

    /// ADC trigger CC offset in timer ticks (1 µs) — set by init().
    uint32_t adc_trigger_offset{168u};
};

/**
 * @brief Current-control ISR driver.
 *
 * Overview
 * --------
 * `CurrentControlIsr` wraps the hardware-facing current-control loop.  It
 * owns the algorithm instances (current controller, SVM, dead-time
 * compensation, HFI) and the shared state that `SlowUpdate` uses to run the
 * flux observer and pre-compute the next set of Park transforms.
 *
 * Execution model
 * ---------------
 * The PWM timer runs in centre-aligned mode.  One CC event is configured
 * 1 µs before the counter peak, another 1 µs before the counter trough.
 * Each CC event triggers an ADC injected sequence (I_a, I_b, V_dc).  The ADC
 * fires a JEOC interrupt upon completion, which calls `on_jeoc()`.
 *
 * Thus `on_jeoc()` fires at 2 × f_pwm, advancing through sub-steps 0–3.
 * At the end of sub-step 3 the `samples_ready` flag is set to wake the
 * `SlowUpdate` task, which runs the observers in the background.
 *
 * Boundary guard
 * --------------
 * When any PWM duty cycle is below 5 % or above 95 % of the timer period the
 * ADC injected sample may be corrupted (switching noise or insufficient
 * settling time).  `on_jeoc()` detects this condition and writes safe neutral
 * duties without running the PI controller or updating the integrators.
 *
 * CCR preload
 * -----------
 * The timer CCR preload registers are disabled (`OCxPE = 0`) so that each
 * write to CCR1/2/3 takes effect within the same half-period.  This is
 * essential for the 4-step HFI injection to work correctly.
 *
 * Usage
 * -----
 * @code
 *   // One-time setup (before enabling the timer/ADC):
 *   auto& isr = unimoc::current_control::CurrentControlIsr::instance();
 *   isr.init(settings);
 *
 *   // From ADC JEOC IRQ handler (highest priority):
 *   isr.on_jeoc();
 *
 *   // From outer control loop to update the setpoint:
 *   isr.set_current_ref({i_d_ref, i_q_ref});
 * @endcode
 */
class CurrentControlIsr
{
public:
    // =========================================================================
    // Singleton access
    // =========================================================================

    /// Return the single global instance.
    static CurrentControlIsr& instance() noexcept
    {
        static CurrentControlIsr obj;
        return obj;
    }

    // =========================================================================
    // Initialisation
    // =========================================================================

    /**
     * @brief Configure the current-control ISR from NVM settings.
     *
     * Computes timing parameters (ARR, ADC trigger offset, dt_fast, dt_slow)
     * from `settings.pwm_frequency` and loads all algorithm parameters.
     *
     * The caller is responsible for configuring the hardware timer and ADC
     * injected sequence using the values stored in `state` after this call:
     *   - `state.arr`                 → TIMx ARR register
     *   - `state.adc_trigger_offset`  → TIMx CCR trigger offset
     *   - `state.dt_fast`             → confirmed ISR period
     *
     * @note Pre-fills both double-buffer sin/cos entries with zero-angle
     *       (identity) so the ISR can run immediately even before SlowUpdate
     *       has had a chance to compute proper values.
     *
     * @param settings  NVM settings loaded and validated by the boot sequence.
     * @param timer_clock_hz  Timer peripheral clock frequency in Hz
     *                        (e.g. 168 000 000 for a 168 MHz APB2 timer).
     */
    void init(const system::NvmSettings& settings,
              uint32_t timer_clock_hz = 168'000'000u) noexcept;

    // =========================================================================
    // ISR entry point
    // =========================================================================

    /**
     * @brief ADC JEOC (end-of-injected-conversion) interrupt handler body.
     *
     * Must be called from the ADC JEOC ISR at the highest configured IRQ
     * priority level.  The function reads the three injected ADC results
     * (I_a, I_b, V_dc), runs one sub-step of the current control loop, and
     * writes the new duty cycles directly to the timer CCR registers.
     *
     * Implementation is in CurrentControlIsr.cpp where the platform-specific
     * ADC/timer register access is defined.
     */
    void on_jeoc() noexcept;

    // =========================================================================
    // Setpoint interface (called from lower-priority context)
    // =========================================================================

    /**
     * @brief Update the rotor-frame current reference.
     *
     * @note The caller must ensure mutual exclusion (e.g. disable the ADC JEOC
     *       IRQ while writing, or use a double-buffered update) because
     *       RotorReference<float> is not atomically writable on Cortex-M.
     *
     * @param ref  New d/q current reference [A].
     */
    void set_current_ref(const system::RotorReference<float>& ref) noexcept
    {
        state.i_ref = ref;
    }

    // =========================================================================
    // Shared state (accessed by SlowUpdate)
    // =========================================================================

    /// Shared state between the ISR and the slow-update task.
    CurrentControlState state{};

    // =========================================================================
    // Algorithm instances (owned here; parameters loaded by init())
    // =========================================================================

    /// Kalman mechanical observer — state updated by SlowUpdate; omega/sin/cos
    /// read by the ISR for Park transforms and feedforward.
    observer::MechanicalObserver<float> mech_obs{};

    /// d/q-axis PI current controller.
    control::CurrentController<float> cc{};

    /// HFI (high-frequency injection) observer.
    observer::Hfi<float> hfi{};

    /// Dead-time compensation (α/β frame, added to modulator input).
    control::DeadTimeCompensation<float> dtc{};

    /// Space-vector modulator.
    control::Svm<float> svm{};

    /// True when HFI injection is active.
    bool hfi_active{false};

private:
    CurrentControlIsr() = default;

    /// Current sub-step index (0–3), ISR-private.
    uint8_t sub_step_{0u};

    /// Active-buffer index snapshot taken at sub-step 0 and used for the
    /// whole 4-step cycle so that a buffer flip mid-cycle is handled
    /// gracefully (the ISR finishes with the old buffer; SlowUpdate writes
    /// new sc into the other one).
    uint8_t active_buf_snapshot_{0u};
};

}  // namespace current_control
}  // namespace unimoc

#endif /* UNIMOC_CURRENT_CONTROL_CURRENT_CONTROL_ISR_H_ */
