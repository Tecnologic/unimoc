/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file slow_update.hpp
 *    @brief Background current-control update interface.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include "current_control_isr.hpp"
#include "nvm_settings.hpp"
#include "pmsm_flux_observer.hpp"
#include "rotor_system.hpp"

/**
 * @namespace unimoc global namespace
 */

/**
 * @namespace current_control current-control subsystem namespace
 */
namespace unimoc::current_control {

/**
 * @brief Observer update that runs after each control cycle, below the control cycle.
 *
 * Overview
 * --------
 * `SlowUpdate` owns the `PmsmFluxObserver` and drives the `MechanicalObserver`
 * prediction step. After every control cycle (signalled through
 * `CurrentControlIsr::state.observer_pending`) it:
 *
 *  1. Takes the mean d/q current and the voltage demand of the cycle from the
 *     buffer the control cycle used.
 *  2. Calls the PMSM flux observer and the mechanical Kalman predict step,
 *     and feeds the four samples to the HFI observer when HFI is active.
 *  3. Computes the angles of the next cycle (Park angle of each sample and
 *     inverse-Park angle of each duty entry, projected forward by
 *     `omega × dt_fast`) into the **inactive** buffer.
 *  4. Atomically flips the double buffer so the next control cycle uses them.
 *
 * Execution model
 * ---------------
 * The platform runs `RunOnce()` after each control cycle at a priority
 * below the sampling interrupt (BCC: a software-pended normal-priority
 * interrupt; native_sim: directly after the control callback). It must finish
 * before the next control cycle, one cycle (62.5 µs at 16 kHz) later;
 * `state.observer_overruns` counts the cycles where it did not.
 *
 * Usage
 * -----
 * @code
 *   auto& isr   = unimoc::current_control::CurrentControlIsr::Instance();
 *   auto& slow  = unimoc::current_control::SlowUpdate::Instance();
 *   slow.Init(settings, isr);
 *
 *   // Observer interrupt, after each control cycle:
 *   slow.RunOnce();
 * @endcode
 */
class SlowUpdate {
 public:
  // =========================================================================
  // Singleton access
  // =========================================================================

  /// Return the single global instance.
  static SlowUpdate& Instance() noexcept {
    static SlowUpdate obj;
    return obj;
  }

  // =========================================================================
  // Initialisation
  // =========================================================================

  /**
   * @brief Initialise the slow-update task from NVM settings.
   *
   * Must be called after `CurrentControlIsr::Init()`.
   *
   * @param settings  NVM settings (motor params, observer gains).
   * @param isr       Reference to the ISR instance whose shared state and
   *                  `MechanicalObserver` this task will update.
   */
  void Init(const settings::NvmSettings& settings, CurrentControlIsr& isr) noexcept;

  // =========================================================================
  // Setpoint interface (called from outer control loop)
  // =========================================================================

  /**
   * @brief Update the flux setpoint used by the PMSM flux observer.
   *
   * Typically set_flux.d = ψ_PM (nominal permanent-magnet flux) and
   * set_flux.q = 0 for field-oriented control without field weakening.
   *
   * @param setpoint  Flux setpoint in the rotor frame [Wb].
   */
  void SetFluxSetpoint(const system::Rotor<unit::MagneticFlux>& setpoint) noexcept {
    flux_setpoint_ = setpoint;
  }

  // =========================================================================
  // Main execution
  // =========================================================================

  /**
   * @brief Run one observer update for the control cycle that just finished.
   *
   * Returns immediately (no-op) if no cycle is pending. Otherwise runs the
   * observer pipeline and prepares the angles of the next cycle.
   *
   * @return true   if an update was performed this call.
   * @return false  if no control cycle was pending.
   */
  bool RunOnce() noexcept;

  // =========================================================================
  // Observer instance (public so outer loop can read psi_pm_d etc.)
  // =========================================================================

  /// PMSM voltage-model flux observer owned by the slow-update task.
  // NOLINTNEXTLINE(cppcoreguidelines-non-private-member-variables-in-classes): read by the outer loop.
  observer::PmsmFluxObserver<float> flux_obs{};

 private:
  SlowUpdate() = default;

  /// Pointer to the ISR instance set by Init().
  CurrentControlIsr* isr_{nullptr};

  /// Flux setpoint used by flux_obs.calculate() — written by outer loop.
  system::Rotor<unit::MagneticFlux> flux_setpoint_{0.0F, 0.0F};
};

}  // namespace unimoc::current_control
