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
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include "current_control_isr.hpp"
#include "hall_fusion.hpp"
#include "hardware_interface.hpp"
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
 * `SlowUpdate` owns the `PmsmFluxObserver` and the Hall fusion and drives
 * the `MechanicalObserver` (docs/hotpath_plan.md, D2 and section 5). After
 * every control cycle (signalled through
 * `CurrentControlIsr::state.observer_pending`) it:
 *
 *  1. Takes the mean d/q current and the voltage demand of the cycle from the
 *     buffer the control cycle used.
 *  2. Predicts the covariance of the mechanical Kalman filter for the cycle.
 *  3. Updates it with the angle measurements of the cycle, each with its own
 *     noise: the PMSM flux observer (weighted down at low speed, where it
 *     has no back-EMF) and, when enabled, the Hall edges captured by the
 *     platform and the Hall sector.
 *  4. Predicts the state to the next cycle and feeds the four samples to the
 *     HFI observer when HFI is active.
 *  5. Computes the angles of the next cycle (Park angle of each sample and
 *     inverse-Park angle of each duty entry, projected forward by
 *     `omega × dt_fast`) into the **inactive** buffer.
 *  6. Atomically flips the double buffer so the next control cycle uses them.
 *
 * The estimate before the state prediction refers to the first sample of the
 * cycle that just finished; the Hall edges are projected to that time.
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
 *   slow.Init(settings, isr, motor);
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
   * @param hardware  Hardware access of the motor (the captured Hall events).
   */
  void Init(const settings::NvmSettings& settings,
            CurrentControlIsr& isr,
            const hardware::HardwareInterface& hardware) noexcept;

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

  // NOLINTBEGIN(cppcoreguidelines-non-private-member-variables-in-classes): read
  // by the outer loop, the telemetry and the tests; written only here.

  /// PMSM voltage-model flux observer owned by the slow-update task.
  observer::PmsmFluxObserver<float> flux_obs{};

  /// Hall sensor fusion (edges, sector, counters).
  observer::HallFusion<float> hall{};

  // NOLINTEND(cppcoreguidelines-non-private-member-variables-in-classes)

 private:
  SlowUpdate() = default;

  /// Hall events taken per cycle at most; more wait for the next cycle.
  static constexpr std::size_t kMaxHallEvents = 8U;

  /// Measurement noise of the flux angle at an electrical speed.
  [[nodiscard]] unit::AngleVariance FluxNoise(unit::AngularVelocity omega) const noexcept;

  /// Fuses the Hall events of the platform; drains them when Hall is off.
  void FuseHall(const SubStepBuffer& cycle) noexcept;

  /// Pointer to the ISR instance set by Init().
  CurrentControlIsr* isr_{nullptr};

  /// Hardware of the motor set by Init().
  const hardware::HardwareInterface* hardware_{nullptr};

  /// Speed weighting k of the flux measurement noise (R + k / omega^2).
  unit::AngleVarianceSpeedWeight flux_r_speed_;

  /// Timestamp ticks from the first to the last sample of a cycle.
  uint32_t reference_offset_ticks_{0U};

  /// Flux setpoint used by flux_obs.Calculate() — written by outer loop.
  system::Rotor<unit::MagneticFlux> flux_setpoint_{0.0F, 0.0F};
};

}  // namespace unimoc::current_control
