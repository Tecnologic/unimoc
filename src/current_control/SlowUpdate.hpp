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

#ifndef UNIMOC_CURRENT_CONTROL_SLOW_UPDATE_H_
#define UNIMOC_CURRENT_CONTROL_SLOW_UPDATE_H_

#include "NvmSettings.hpp"
#include "RotorReference.hpp"
#include "PmsmFluxObserver.hpp"
#include "CurrentControlIsr.hpp"

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
 * @brief Background observer update that runs outside the ISR context.
 *
 * Overview
 * --------
 * `SlowUpdate` owns the `PmsmFluxObserver` and drives the `MechanicalObserver`
 * prediction step.  It wakes whenever the ISR signals that a new set of four
 * current samples is ready (via `CurrentControlIsr::state.samples_ready`),
 * then:
 *
 *  1. Snapshots the four I_αβ samples and pre-computed sin/cos pairs from the
 *     active double-buffer — the buffer that the ISR was writing to during
 *     the previous four sub-steps.
 *  2. Applies the per-sub-step angle to each sample (angle-advance correction)
 *     and computes the mean d/q current.
 *  3. Calls the PMSM flux observer and the mechanical Kalman predict step.
 *  4. Computes the next four sin/cos pairs (projected forward by `omega × dt_fast`)
 *     and stores them in the **inactive** double-buffer.
 *  5. Atomically flips the double-buffer so the ISR picks up the new values
 *     at the start of the next 4-step cycle.
 *
 * Execution model
 * ---------------
 * The caller is responsible for scheduling `run()` in a loop at a priority
 * lower than the ADC JEOC ISR.  `run()` polls `samples_ready` and does
 * nothing until it is set.  On a FreeRTOS system the caller can yield between
 * polls; on a bare-metal system a simple busy wait is safe because the ISR
 * will still preempt it.
 *
 * Thread-safety
 * -------------
 * See the ownership protocol in `SubStepBuffer.hpp`.  After `run()` flips
 * the active buffer the ISR begins reading from the new (formerly inactive)
 * buffer.  The old buffer (now inactive) is then owned by SlowUpdate and is
 * safe to write without synchronisation until the next flip.
 *
 * Usage
 * -----
 * @code
 *   // One-time setup:
 *   auto& isr   = unimoc::current_control::CurrentControlIsr::instance();
 *   auto& slow  = unimoc::current_control::SlowUpdate::instance();
 *   slow.init(settings, isr);
 *
 *   // Background loop (low-priority task or main thread):
 *   for (;;) {
 *       slow.run_once();   // returns immediately if samples not yet ready
 *       // yield / sleep here if available
 *   }
 * @endcode
 */
class SlowUpdate
{
public:
    // =========================================================================
    // Singleton access
    // =========================================================================

    /// Return the single global instance.
    static SlowUpdate& instance() noexcept
    {
        static SlowUpdate obj;
        return obj;
    }

    // =========================================================================
    // Initialisation
    // =========================================================================

    /**
     * @brief Initialise the slow-update task from NVM settings.
     *
     * Must be called after `CurrentControlIsr::init()`.
     *
     * @param settings  NVM settings (motor params, observer gains).
     * @param isr       Reference to the ISR instance whose shared state and
     *                  `MechanicalObserver` this task will update.
     */
    void init(const system::NvmSettings& settings,
              CurrentControlIsr&         isr) noexcept;

    // =========================================================================
    // Setpoint interface (called from outer control loop)
    // =========================================================================

    /**
     * @brief Update the flux setpoint used by the PMSM flux observer.
     *
     * Typically set_flux.d = ψ_PM (nominal permanent-magnet flux) and
     * set_flux.q = 0 for field-oriented control without field weakening.
     *
     * @param sp  Flux setpoint in the rotor frame [Wb].
     */
    void set_flux_setpoint(const system::RotorReference<float>& sp) noexcept
    {
        flux_setpoint_ = sp;
    }

    // =========================================================================
    // Main execution
    // =========================================================================

    /**
     * @brief Attempt one observer update cycle.
     *
     * Returns immediately (no-op) if `samples_ready` is not set.  Otherwise
     * runs the full observer pipeline and prepares the next set of sub-step
     * sin/cos values.
     *
     * The caller should call this in a tight loop (yielding between calls when
     * possible) at a priority lower than the ADC JEOC ISR.
     *
     * @return true   if an update was performed this call.
     * @return false  if samples were not yet ready.
     */
    bool run_once() noexcept;

    // =========================================================================
    // Observer instance (public so outer loop can read psi_pm_d etc.)
    // =========================================================================

    /// PMSM voltage-model flux observer owned by the slow-update task.
    observer::PmsmFluxObserver<float> flux_obs{};

private:
    SlowUpdate() = default;

    /// Pointer to the ISR instance set by init().
    CurrentControlIsr* isr_{nullptr};

    /// Flux setpoint used by flux_obs.calculate() — written by outer loop.
    system::RotorReference<float> flux_setpoint_{0.0f, 0.0f};
};

}  // namespace current_control
}  // namespace unimoc

#endif /* UNIMOC_CURRENT_CONTROL_SLOW_UPDATE_H_ */
