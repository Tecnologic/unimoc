/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file current_control_isr.hpp
 *    @brief Current-control ISR interface and state.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <limits>
#include "current_controller.hpp"
#include "dead_time_compensation.hpp"
#include "hardware_interface.hpp"
#include "hfi.hpp"
#include "mailbox.hpp"
#include "mechanical_observer.hpp"
#include "nvm_settings.hpp"
#include "rotor_system.hpp"
#include "stator_system.hpp"
#include "sub_step_buffer.hpp"
#include "svm.hpp"

/**
 * @namespace unimoc global namespace
 */

/**
 * @namespace current_control current-control subsystem namespace
 */
namespace unimoc::current_control {

using namespace unit;

static_assert(kNumSubSteps == hardware::kHalfPeriodsPerCycle,
              "one control cycle covers the half periods of one hardware cycle");

/**
 * @brief Timer period in seconds derived from a PWM frequency.
 *
 * Returns the period of one PWM half-period (i.e. the sample interval), which
 * is 1 / (2 × f_pwm).
 *
 * @param freq  PWM frequency selection.
 * @return      dt_fast.
 */
[[nodiscard]] constexpr unit::Time DtFastFromPwmFrequency(unit::Frequency freq) noexcept {
  return 1.0F / (2.0F * freq);
}

/**
 * @brief Control-cycle period = kNumSubSteps × dt_fast.
 */
[[nodiscard]] constexpr unit::Time DtSlowFromPwmFrequency(unit::Frequency freq) noexcept {
  return DtFastFromPwmFrequency(freq) * static_cast<float>(kNumSubSteps);
}

/// PWM frequency the timing of the state assumes until Init() sets it.
inline constexpr unit::Frequency kInitialPwmFrequency{32000.0F};

/**
 * @brief Shared state block for the control cycle and the observer.
 *
 * A single instance of this struct is owned by `CurrentControlIsr`.  The
 * control cycle (zero latency, once per kNumSubSteps PWM half periods), the
 * observer (`SlowUpdate`, lower-priority interrupt after each control cycle)
 * and the startup FSM access it; the access discipline is documented per
 * member.
 *
 * Thread-safety model
 * -------------------
 *  - The control cycle preempts the observer and every thread.
 *  - `double_buf`  : atomic active index; SubStepBuffer access follows the
 *                    ownership protocol described in sub_step_buffer.hpp.
 *  - `i_ref_mailbox`: written by the outer loop (control thread, one
 *                    writer) through SetCurrentRef(); the control cycle copies
 *                    it into `i_ref` at the start of every cycle.
 *  - `observer_pending`: set by the control cycle after it filled the active
 *                    buffer, cleared by the observer after the flip.
 *  - `samples_ready`: set by the control cycle once per cycle, consumed by the
 *                    startup FSM.
 */
struct CurrentControlState {
  /// Double-buffered angle precomputation + current sample storage.
  DoubleBuffer double_buf{};

  /// Current reference in the rotor frame [A] the last control cycle worked on.
  /// Written only by the control cycle (from `i_ref_mailbox`).
  system::Rotor<unit::Current> i_ref{0.0F, 0.0F};

  /// Current reference handed over by SetCurrentRef(); the control cycle reads it.
  Mailbox<system::Rotor<unit::Current>> i_ref_mailbox{};

  /// Mean measured current of the last controlled cycle, rotor frame [A].
  system::Rotor<unit::Current> i_dq_last{0.0F, 0.0F};

  /// Last voltage demand computed by the control cycle, rotor frame [V].
  system::Rotor<unit::Voltage> u_dq_last{0.0F, 0.0F};

  /// Buffer index the last control cycle used; read by the observer.
  uint8_t observer_buffer{0U};

  /// Set by the control cycle when the observer has a cycle to process.
  std::atomic<bool> observer_pending{false};

  /// Control cycles that started while the observer still processed the previous one.
  uint32_t observer_overruns{0U};

  /// Control cycles run since Init().
  uint32_t control_cycles{0U};

  /// Set to true by the control cycle once per cycle; consumed by the startup FSM.
  volatile bool samples_ready{false};

  /// Duration of one PWM half period (one current sample) — set by Init().
  unit::Time dt_fast{DtFastFromPwmFrequency(kInitialPwmFrequency)};

  /// Duration of one control cycle — set by Init().
  unit::Time dt_slow{DtSlowFromPwmFrequency(kInitialPwmFrequency)};

  /// Delay from the control boundary to the middle of duty entry 0, in PWM
  /// half periods (D4). Entry 0 is written directly into the CCR preload and
  /// takes effect one half period after the boundary: 1.5. Tuned on the board.
  float output_delay_half_periods{1.5F};

  /// Timer auto-reload value used for timing calculations — set by Init().
  uint32_t arr{2655U};

  /// ADC trigger compare offset in timer ticks — set by Init().
  uint32_t adc_trigger_offset{170U};

  /// Duties of the last control cycle; entry 0 is the first half period.
  hardware::DutyPlan duty_plan{};

  /// Most recently requested normalized phase duties (entry 0 of duty_plan).
  system::ThreePhase<unit::Ratio> phase_duties{0.5_ratio, 0.5_ratio, 0.5_ratio};

  /// Newest raw ADC sample of the last cycle; updated by every control cycle
  /// regardless of control mode. Safe to read from lower-priority contexts
  /// (e.g. the startup FSM). The phase-current vector preserves ia + ib + ic = 0.
  system::ThreePhase<unit::Current> raw_phase_currents;
  unit::Voltage raw_vdc;

  /// Hard phase-current limit [A]. The hardware checks every sample against
  /// it and trips the power stage; the control cycle checks all samples of the
  /// cycle again. Infinity disables the check.
  unit::Current overcurrent_limit{std::numeric_limits<float>::infinity()};

  /// Set by the control cycle when the limit was exceeded; the control loop
  /// stays frozen at neutral duties until ClearTrips().
  volatile bool overcurrent_tripped{false};

  /// DC-link voltage at which the control cycle trips the power stage; set by
  /// Init() from the settings. Infinity disables the check.
  unit::Voltage dc_link_trip{std::numeric_limits<float>::infinity()};

  /// Set by the control cycle when the DC link reached `dc_link_trip`; the
  /// control loop stays frozen at neutral duties until ClearTrips().
  volatile bool overvoltage_tripped{false};
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
 * The PWM timer runs in centre-aligned mode and the hardware samples the
 * phase currents once per half period (at every peak and valley). Every
 * kNumSubSteps samples it calls `OnControlCycle()` in the zero-latency
 * context, which
 *
 *  1. reads the kNumSubSteps samples of the cycle,
 *  2. parks each sample with the angle it was taken at and averages them,
 *  3. runs the PI current controller once,
 *  4. computes kNumSubSteps duty entries, each with its own inverse-Park
 *     angle, HFI injection step and dead-time compensation, and
 *  5. writes them to the hardware as one `DutyPlan` and signals the observer.
 *
 * At 32 kHz PWM the cycle runs at 16 kHz. One HFI injection period (four
 * steps) fits into one cycle, so the mean current seen by the PI is free of
 * the injection response.
 *
 * Duty range
 * ----------
 * The duties take effect through the PWM preload at the boundaries, so they
 * are not limited by the ISR timing; only `svm.duty_min` / `svm.duty_max`
 * apply (sampling window and minimum pulse of the power stage).
 *
 * Usage
 * -----
 * @code
 *   // One-time setup (before enabling the timer/ADC):
 *   auto& isr = unimoc::current_control::CurrentControlIsr::Instance();
 *   isr.Init(settings, hardware, hardware.GetTimerClockFrequency());
 *
 *   // From hardware::ControlCycleHandler() (zero latency):
 *   isr.OnControlCycle();
 *
 *   // From outer control loop to update the setpoint:
 *   isr.SetCurrentRef({i_d_ref, i_q_ref});
 * @endcode
 */
class CurrentControlIsr {
 public:
  // =========================================================================
  // Singleton access
  // =========================================================================

  /// Return the single global instance.
  static CurrentControlIsr& Instance() noexcept {
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
   * The hardware interface owns timer and ADC configuration. The calculated
   * values remain in `state` for diagnostics and runtime timing.
   *
   * @note Pre-fills both double-buffer sin/cos entries with zero-angle
   *       (identity) so the ISR can run immediately even before SlowUpdate
   *       has had a chance to compute proper values.
   *
   * @param settings  NVM settings loaded and validated by the boot sequence.
   * @param hardware  Hardware callbacks for ADC samples and PWM output.
   * @param timer_clock_hz  Timer peripheral clock frequency in Hz
   *                        (e.g. 168 000 000 for a 168 MHz APB2 timer).
   */
  void Init(const settings::NvmSettings& settings,
            const hardware::HardwareInterface& hardware,
            uint32_t timer_clock_hz) noexcept;

  // =========================================================================
  // ISR entry point
  // =========================================================================

  /**
   * @brief Control-cycle handler body.
   *
   * Called from hardware::ControlCycleHandler() (zero latency) once per
   * kNumSubSteps PWM half periods. Reads the samples of the cycle, runs the
   * current controller once and writes the duty plan of the next
   * kNumSubSteps half periods through hardware::hot_path. Does nothing before
   * Init().
   */
  void OnControlCycle() noexcept;

  // =========================================================================
  // Setpoint interface (called from lower-priority context)
  // =========================================================================

  /**
   * @brief Update the rotor-frame current reference.
   *
   * Hands the reference to the control cycle through a mailbox; the next
   * control cycle works on it. One writer: the outer loop (control thread),
   * or a test or bring-up context while no outer loop runs.
   *
   * @param ref  New d/q current reference [A].
   */
  void SetCurrentRef(const system::Rotor<unit::Current>& ref) noexcept { state.i_ref_mailbox.Write(ref); }

  // =========================================================================
  // Startup / bring-up interface
  // =========================================================================

  /**
   * @brief Override PWM output with fixed duty cycles, bypassing the PI loop.
   *
   * When active, OnControlCycle() still reads and stores the raw ADC samples in
   * state.raw_phase_currents / raw_vdc, but skips Clarke / Park / PI / SVM and
   * writes the requested duties through the hardware interface.
   *
   * Duties are clamped to [svm.duty_min, svm.duty_max]. The control cycle sees
   * either the previous or all three new duties, never a mix. One writer
   * thread only (the startup FSM). To ensure the ADC
   * sampling window is never corrupted, consider keeping duties well within
   * this range during calibration steps.
   *
   * @param da  Phase-A duty cycle [0, 1].
   * @param db  Phase-B duty cycle [0, 1].
   * @param dc  Phase-C duty cycle [0, 1].
   */
  void ForceDuty(unit::Ratio da, unit::Ratio db, unit::Ratio dc) noexcept;

  /**
   * @brief Release force-duty mode and return to normal closed-loop PI control.
   *
   * Resets the current-controller integrators so the PI loop resumes from a
   * clean state rather than inheriting stale winding-up values accumulated
   * while force-duty was active.
   */
  void ReleaseForceDuty() noexcept;

  /**
   * @brief Update the ADC injection trigger offset in timer ticks.
   *
   * Used by the startup alignment sweep to find the optimal ADC sampling
   * point within the PWM half-period.  Updates state.adc_trigger_offset;
   * on real hardware the caller must also write the new value to the timer
   * hardware compare register that triggers the ADC injected sequence.
   *
   * @param offset  New trigger offset [timer ticks from PWM peak/trough].
   */
  void SetAdcTriggerOffset(uint32_t offset) noexcept;

  /**
   * @brief Set the hard phase-current limit that trips the power stage.
   *
   * The hardware checks every sample against it and trips the power stage at
   * once; OnControlCycle() checks the samples of the cycle again, calls
   * hardware::kPowerStage.Trip() when needed and freezes the control loop at
   * neutral duties.
   */
  void SetOvercurrentLimit(unit::Current limit) noexcept;

  /// Acknowledges an overcurrent or overvoltage trip. The power stage fault is
  /// cleared separately (kPowerStage.ClearFault()).
  void ClearTrips() noexcept {
    cc.Reset();
    state.overcurrent_tripped = false;
    state.overvoltage_tripped = false;
  }

  /// True while a trip of the control cycle holds the loop at neutral duties.
  [[nodiscard]] bool Tripped() const noexcept {
    return state.overcurrent_tripped || state.overvoltage_tripped;
  }

  /// Name of the latched trip for the log; nullptr without one.
  [[nodiscard]] const char* TripName() const noexcept {
    if (state.overcurrent_tripped) {
      return "overcurrent trip";
    }
    if (state.overvoltage_tripped) {
      return "DC-link overvoltage trip";
    }
    return nullptr;
  }

  // =========================================================================
  // Shared state (accessed by SlowUpdate)
  // =========================================================================

  // NOLINTBEGIN(cppcoreguidelines-non-private-member-variables-in-classes): the
  // observer (SlowUpdate) and the startup FSM work on the shared state and the
  // algorithm instances of the singleton directly; the access protocol is
  // documented at CurrentControlState and SubStepBuffer.

  /// Shared state between the ISR and the slow-update task.
  CurrentControlState state{};

  // =========================================================================
  // Algorithm instances (owned here; parameters loaded by Init())
  // =========================================================================

  /// Kalman mechanical observer — state updated by SlowUpdate; omega
  /// read by the control cycle for the decoupling feedforward.
  observer::MechanicalObserver<float> mech_obs{};

  /// d/q-axis PI current controller.
  control::CurrentController<float> cc{};

  /// HFI (high-frequency injection) observer.
  observer::Hfi<float> hfi{};

  /// Dead-time compensation (α/β frame, added to modulator input).
  control::DeadTimeCompensation<float> dtc{};

  /// Space-vector modulator.
  control::Svm svm{};

  /// True when HFI injection is active.
  bool hfi_active{false};

  // NOLINTEND(cppcoreguidelines-non-private-member-variables-in-classes)

 private:
  CurrentControlIsr() = default;

  /// Writes the same duties for all entries of the plan.
  void SetUniformDuties(const system::ThreePhase<unit::Ratio>& duties) noexcept;

  void SetDutyPlan(const hardware::DutyPlan& plan) noexcept;

  /// Ends a cycle that did not run the controller: FSM signalled, observer not run.
  void FinishCycleWithoutControl() noexcept;

  const hardware::HardwareInterface* hardware_{nullptr};

  /// Fixed duty cycles applied to CCR1/2/3 while force_duty_active_ is true.
  /// Clamped to [svm.duty_min, svm.duty_max] by ForceDuty(), which publishes
  /// them before it sets the flag.
  Mailbox<system::ThreePhase<unit::Ratio>> forced_duties_{};

  /// When true, OnControlCycle() writes forced_duties_ as the duty plan and
  /// bypasses the PI control loop. Set by ForceDuty() after the duties
  /// (release), read by the control cycle (acquire); cleared by
  /// ReleaseForceDuty().
  std::atomic<bool> force_duty_active_{false};
};

}  // namespace unimoc::current_control
