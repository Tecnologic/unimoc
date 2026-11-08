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
#include "asm_flux_observer.hpp"
#include "back_emf_speed.hpp"
#include "coast_down_meter.hpp"
#include "current_control_isr.hpp"
#include "hall_fusion.hpp"
#include "hardware_interface.hpp"
#include "mailbox.hpp"
#include "motor_type.hpp"
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
 * every control cycle of timing.steps_per_cycle control steps (signalled through
 * `CurrentControlIsr::state.observer_pending`) it:
 *
 *  1. Takes the mean d/q current and the mean voltage demand of the cycle
 *     from the record the control steps filled.
 *  2. Predicts the covariance of the mechanical Kalman filter for the cycle.
 *  3. Updates it with the angle measurements of the cycle, each with its own
 *     noise: the PMSM flux observer (weighted down at low speed, where it
 *     has no back-EMF) and, when enabled, the Hall edges captured by the
 *     platform and the Hall sector.
 *  4. Predicts the state to the next cycle.
 *  5. Writes the angle reference of the next cycle (the angle at its first
 *     sample, the rotation of one half period and of the output delay) into
 *     the **inactive** buffer.
 *  6. Atomically flips the buffer; the control steps advance the angle from
 *     the new reference.
 *
 * The estimate before the state prediction refers to the first sample of the
 * cycle that just finished; the Hall edges are projected to that time.
 *
 * Motor types (`unimoc.motor.type`)
 * ---------------------------------
 *  - PMSM: the flux observer measures the rotor angle against psi_PM.
 *  - EESM: as the PMSM, with the field flux L_m i_f of the excitation module
 *    (SetFieldFlux()) in place of the magnet.
 *  - ASM (sensorless): the `AsmFluxObserver` runs once per control step at
 *    the rotor speed (the estimated synchronous speed less the slip); the
 *    angle of its rotor flux (Atan2, no libm) is the field frame and its
 *    filtered rate of change the synchronous speed, which the mechanical
 *    observer only carries (its shaft model does not describe the field).
 *    Until the flux reaches half its reference the frame stands. The Hall
 *    sensors measure the rotor, not the field: they are not fused.
 *
 * Outputs open (PMSM, EESM; disengaged, tripped, the coast of M3): the voltage demand
 * does not reach the motor, but on a platform with a phase-voltage sense
 * the terminals carry the back-EMF. The flux observer then integrates the
 * measured voltages instead of the demand, and `BackEmfSpeed` measures the
 * speed (magnitude and sense of rotation of the back-EMF), so the estimate
 * follows a coasting motor and the drive engages on it without a jump
 * (docs/hotpath_plan.md, "Outputs open"). Without the sense the flux
 * observer starts anew on every such cycle, as before.
 *
 * Execution model
 * ---------------
 * The platform runs `RunOnce()` after each control cycle at a priority
 * below the sampling interrupt (STM32H7: a software-pended normal-priority
 * interrupt; native_sim: directly after the sample handler). It should
 * finish before the first sample of the next cycle (one half period; else
 * the control steps extrapolate a little longer) and must finish before the
 * end of the next cycle (62.5 µs at 16 kHz); `state.observer_overruns`
 * counts the cycles where it did not.
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

  /**
   * @brief EESM: the field flux psi_f = L_m i_f of the measured field current; thread context.
   *
   * The flux observer takes it as its flux setpoint and the mechanical
   * observer as the flux of its torque model, from the next cycle on.
   */
  void SetFieldFlux(const unit::MagneticFlux flux) noexcept { field_flux_.Write(flux); }

  /// The motor type the observer runs for (from Init()).
  [[nodiscard]] system::MotorType MotorType() const noexcept { return motor_type_; }

  /// ASM: slip speed omega_slip = R_r / L_r * L_m i_q / |psi_r| of the last cycle (electrical); zero
  /// otherwise.
  [[nodiscard]] unit::AngularVelocity SlipSpeed() const noexcept { return slip_.Read(); }

  /// ASM: estimated rotor flux magnitude of the last cycle; zero otherwise.
  [[nodiscard]] unit::MagneticFlux RotorFlux() const noexcept { return rotor_flux_.Read(); }

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
  // Motor identification (docs/motor_identification_plan.md, section 5)
  // =========================================================================

  /**
   * @brief Replaces the observer's angle in the angle buffers by a forced one.
   *
   * From the next cycle on the control cycle works at @p angle, which then
   * turns at @p speed (zero: a fixed angle, M1 and M2; a ramp: I/f). The
   * mechanical observer keeps running in the background. One writer thread.
   *
   * @param angle Electrical angle from the next cycle on.
   * @param speed Electrical speed of the forced angle.
   * @param acceleration Change of @p speed per second (the I/f ramp of M3); zero: a constant speed.
   */
  void SetForcedAngle(unit::Angle angle,
                      unit::AngularVelocity speed,
                      unit::AngularAcceleration acceleration = unit::AngularAcceleration{}) noexcept {
    forced_.Write(ForcedAngle{angle, speed, acceleration});
    forced_state_.store(kForcedNew, std::memory_order_release);
  }

  /// Ends the ramp of the forced angle: it turns on at the speed it has reached.
  void HoldForcedSpeed() noexcept { forced_hold_.store(true, std::memory_order_release); }

  /// Speed of the forced angle of the last cycle (zero while it is off).
  [[nodiscard]] unit::AngularVelocity ForcedSpeed() const noexcept { return forced_published_.Read().speed; }

  /// Back to the observer's angle from the next cycle on.
  void ReleaseForcedAngle() noexcept { forced_state_.store(kForcedOff, std::memory_order_release); }

  /**
   * @brief Starts a measurement window of @p cycles control cycles.
   *
   * The observer sums the means and the phasors of the identification
   * (IdentificationWindowResult) over the next @p cycles cycles; TakeWindow()
   * returns them once complete. A new request replaces a running window. One
   * thread (the identification FSM) requests and takes.
   */
  void RequestWindow(uint32_t cycles) noexcept {
    window_cycles_ = cycles;
    window_generation_.fetch_add(1U, std::memory_order_release);
  }

  /// Takes the result of the requested window; false while it is not complete.
  [[nodiscard]] bool TakeWindow(IdentificationWindowResult& result) noexcept {
    const uint32_t kRequested = window_generation_.load(std::memory_order_relaxed);
    if (window_done_.load(std::memory_order_acquire) != kRequested) {
      return false;
    }
    result = window_result_;
    window_done_.store(kRequested - 1U, std::memory_order_relaxed);  // taken: not done again
    return true;
  }

  /// Ends the requested window without a result.
  void CancelWindow() noexcept {
    window_cycles_ = 0U;
    window_generation_.fetch_add(1U, std::memory_order_release);
  }

  /// State of the coast-down meter of M3 (StartCoast()).
  struct CoastStatus {
    uint32_t generation{0U};                          ///< The StartCoast() this status belongs to.
    bool running{false};                              ///< The meter takes the samples of the open outputs.
    bool standstill{false};                           ///< The back-EMF stayed below its minimum.
    identification::CoastDownMeter::Result result{};  ///< The periods measured so far.
  };

  /**
   * @brief Starts the coast-down meter on the phase voltages (M3).
   *
   * From the next cycle on, every cycle with the outputs open feeds its phase
   * voltages to a CoastDownMeter started with @p parameters;
   * GetCoast() returns its result. One thread starts, reads and stops.
   *
   * @return The generation of this coast: a status of GetCoast() with another
   *         generation is older (the observer has not run since).
   */
  uint32_t StartCoast(const identification::CoastDownMeter::Parameters& parameters) noexcept {
    coast_parameters_.Write(parameters);
    uint32_t generation = coast_generation_.fetch_add(1U, std::memory_order_release) + 1U;
    if (generation == 0U) {
      generation = coast_generation_.fetch_add(1U, std::memory_order_release) + 1U;  // 0 means none
    }
    return generation;
  }

  /// Stops the coast-down meter; its last result stays readable.
  void StopCoast() noexcept { coast_stop_.store(coast_generation_.load(std::memory_order_relaxed)); }

  /// The coast-down meter's state, last published by the observer.
  [[nodiscard]] CoastStatus GetCoast() const noexcept { return coast_status_.Read(); }

  // =========================================================================
  // Observer instance (public so outer loop can read psi_pm_d etc.)
  // =========================================================================

  // NOLINTBEGIN(cppcoreguidelines-non-private-member-variables-in-classes): read
  // by the outer loop, the telemetry and the tests; written only here.

  /// PMSM voltage-model flux observer owned by the slow-update task.
  observer::PmsmFluxObserver<float> flux_obs{};

  /// Hall sensor fusion (edges, sector, counters).
  observer::HallFusion<float> hall{};

  /// ASM rotor flux observer (the angle measurement of an induction motor).
  observer::AsmFluxObserver<float> asm_obs{};

  // NOLINTEND(cppcoreguidelines-non-private-member-variables-in-classes)

 private:
  SlowUpdate() = default;

  /// Hall events taken per cycle at most; more wait for the next cycle.
  static constexpr std::size_t kMaxHallEvents = 8U;

  /// Measurement noise of the flux angle at an electrical speed.
  [[nodiscard]] unit::AngleVariance FluxNoise(unit::AngularVelocity omega) const noexcept;

  /// Fuses the Hall events of the platform; drains them when Hall is off.
  void FuseHall(const CycleRecord& cycle) noexcept;

  /// ASM: runs the rotor flux observer over the steps of the cycle and updates the angle with its flux angle.
  void UpdateAsm(const CycleRecord& cycle, bool powered) noexcept;

  /// Adds the cycle to the requested measurement window; completes it after its cycles.
  void AccumulateWindow(const CycleRecord& cycle) noexcept;

  /// Outputs open: flux observer and speed from the phase voltages of the cycle.
  void TrackBackEmf(const CycleRecord& cycle, const system::Rotor<unit::Current>& current_dq) noexcept;

  /// Feeds the phase voltages of an open-output cycle to the coast-down meter.
  void MeasureCoast(const CycleRecord& cycle, bool outputs_open) noexcept;

  /// The angle of the next cycle, the observer's or the forced one.
  struct ForcedAngle {
    unit::Angle angle;
    unit::AngularVelocity speed;
    unit::AngularAcceleration acceleration;
  };
  static constexpr uint8_t kForcedOff = 0U;
  static constexpr uint8_t kForcedNew = 1U;
  static constexpr uint8_t kForcedOn = 2U;
  Mailbox<ForcedAngle> forced_{};
  std::atomic<uint8_t> forced_state_{kForcedOff};
  std::atomic<bool> forced_hold_{false};
  ForcedAngle forced_now_{};                 ///< Observer only: the forced angle of the next cycle.
  bool forced_last_{false};                  ///< Observer only: the last cycle used the forced angle.
  SeqLock<ForcedAngle> forced_published_{};  ///< forced_now_ for the identification thread.

  /// Measurement window: requested by generation, done when window_done_ equals it.
  uint32_t window_cycles_{0U};
  std::atomic<uint32_t> window_generation_{0U};
  std::atomic<uint32_t> window_done_{0xFFFFFFFFU};
  uint32_t window_running_{0U};  ///< Observer only: the generation being summed.
  IdentificationWindowResult window_sums_{};
  IdentificationWindowResult window_result_{};

  /// Speed of the coasting motor from its back-EMF (outputs open).
  observer::BackEmfSpeed emf_speed_;

  /// Coast-down meter of M3: started by generation, stopped when coast_stop_ equals it.
  Mailbox<identification::CoastDownMeter::Parameters> coast_parameters_{};
  std::atomic<uint32_t> coast_generation_{0U};
  std::atomic<uint32_t> coast_stop_{0U};
  uint32_t coast_running_{0U};  ///< Observer only: the generation being measured.
  identification::CoastDownMeter coast_meter_;
  SeqLock<CoastStatus> coast_status_{};

  /// Pointer to the ISR instance set by Init().
  CurrentControlIsr* isr_{nullptr};

  /// Hardware of the motor set by Init().
  const hardware::HardwareInterface* hardware_{nullptr};

  /// Delays of the measured currents and phase voltages, compensated (from the hardware at Init()).
  hardware::SenseDelays sense_delays_;

  /// Speed weighting k of the flux measurement noise (R + k / omega^2).
  unit::AngleVarianceSpeedWeight flux_r_speed_;

  /// Timestamp ticks from the first to the last sample of a cycle.
  uint32_t reference_offset_ticks_{0U};

  /// Flux setpoint used by flux_obs.Calculate() — written by outer loop.
  system::Rotor<unit::MagneticFlux> flux_setpoint_{0.0F, 0.0F};

  system::MotorType motor_type_{system::MotorType::kPmsm};
  Mailbox<unit::MagneticFlux> field_flux_{};    ///< EESM: L_m i_f, from the drive thread.
  Mailbox<unit::AngularVelocity> slip_{};       ///< ASM: slip of the last cycle, for the outer loop.
  Mailbox<unit::MagneticFlux> rotor_flux_{};    ///< ASM: |psi_r| of the last cycle.
  unit::MagneticFlux asm_flux_reference_;       ///< ASM: |psi_r*| (bounds the slip at a small flux).
  unit::Angle last_flux_angle_;                 ///< ASM: flux angle of the last cycle.
  unit::AngularVelocity field_speed_;           ///< ASM: synchronous speed (rotor speed plus slip).
  unit::AngularVelocity rotor_speed_integral_;  ///< ASM: integral part of the adapted rotor speed.
  /// ASM: gains of the speed adaptation, rad/s per (A Wb) and per (A Wb s).
  decltype(unit::AngularVelocity{} / unit::Torque{}) speed_kp_{200.0F};
  decltype(unit::AngularVelocity{} / (unit::Torque{} * unit::Time{})) speed_ki_{20000.0F};
};

}  // namespace unimoc::current_control
