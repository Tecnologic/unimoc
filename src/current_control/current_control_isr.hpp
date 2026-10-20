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
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <limits>
#include "SubStepBuffer.hpp"
#include "current_controller.hpp"
#include "dead_time_compensation.hpp"
#include "hardware_interface.hpp"
#include "hfi.hpp"
#include "mechanical_observer.hpp"
#include "nvm_settings.hpp"
#include "rotor_system.hpp"
#include "stator_system.hpp"
#include "svm.hpp"

/**
 * @namespace unimoc global namespace
 */
namespace unimoc {
/**
 * @namespace current_control current-control subsystem namespace
 */
namespace current_control {

using namespace unit;

static_assert(NUM_SUB_STEPS == hardware::kHalfPeriodsPerCycle,
              "one control cycle covers the half periods of one hardware cycle");

/**
 * @brief Timer period in seconds derived from a PWM frequency.
 *
 * Returns the period of one PWM half-period (i.e. the sample interval), which
 * is 1 / (2 × f_pwm).
 *
 * @param freq  PWM frequency selection.
 * @return      dt_fast [s].
 */
[[nodiscard]] constexpr float dt_fast_from_pwm_frequency(unit::Frequency freq) noexcept {
  return 1.0f / (2.0f * freq.Value());
}

/**
 * @brief Control-cycle period [s] = NUM_SUB_STEPS × dt_fast.
 */
[[nodiscard]] constexpr float dt_slow_from_pwm_frequency(unit::Frequency freq) noexcept {
  return static_cast<float>(NUM_SUB_STEPS) * dt_fast_from_pwm_frequency(freq);
}

/**
 * @brief Shared state block for the control cycle and the observer.
 *
 * A single instance of this struct is owned by `CurrentControlIsr`.  The
 * control cycle (zero latency, once per NUM_SUB_STEPS PWM half periods), the
 * observer (`SlowUpdate`, lower-priority interrupt after each control cycle)
 * and the startup FSM access it; the access discipline is documented per
 * member.
 *
 * Thread-safety model
 * -------------------
 *  - The control cycle preempts the observer and every thread.
 *  - `double_buf`  : atomic active index; SubStepBuffer access follows the
 *                    ownership protocol described in SubStepBuffer.hpp.
 *  - `i_ref`       : written by the outer control loop (lower priority);
 *                    read by the control cycle.  A two-word Rotor<float> on
 *                    Cortex-M is not atomically accessible; the outer loop
 *                    should use a critical section or double-buffered update.
 *  - `observer_pending`: set by the control cycle after it filled the active
 *                    buffer, cleared by the observer after the flip.
 *  - `samples_ready`: set by the control cycle once per cycle, consumed by the
 *                    startup FSM.
 */
struct CurrentControlState {
  /// Double-buffered angle precomputation + current sample storage.
  DoubleBuffer double_buf{};

  /// Current setpoint in the rotor frame [A].  Written by the outer loop.
  system::Rotor<unit::Current> i_ref{0.0f, 0.0f};

  /// Last voltage demand computed by the control cycle, rotor frame [V].
  system::Rotor<unit::Voltage> u_dq_last{0.0f, 0.0f};

  /// Buffer index the last control cycle used; read by the observer.
  uint8_t observer_buffer{0u};

  /// Set by the control cycle when the observer has a cycle to process.
  std::atomic<bool> observer_pending{false};

  /// Control cycles that started while the observer still processed the previous one.
  uint32_t observer_overruns{0u};

  /// Control cycles run since init().
  uint32_t control_cycles{0u};

  /// Set to true by the control cycle once per cycle; consumed by the startup FSM.
  volatile bool samples_ready{false};

  /// Duration of one PWM half period (one current sample) [s] — set by init().
  float dt_fast{1.0f / (2.0f * 32000.0f)};

  /// Duration of one control cycle [s] — set by init().
  float dt_slow{static_cast<float>(NUM_SUB_STEPS) / (2.0f * 32000.0f)};

  /// Delay from the control boundary to the middle of duty entry 0, in PWM
  /// half periods (D4). Entry 0 is written directly into the CCR preload and
  /// takes effect one half period after the boundary: 1.5. Tuned on the board.
  float output_delay_half_periods{1.5f};

  /// Timer auto-reload value used for timing calculations — set by init().
  uint32_t arr{2655u};

  /// ADC trigger compare offset in timer ticks — set by init().
  uint32_t adc_trigger_offset{170u};

  /// Duties of the last control cycle; entry 0 is the first half period.
  hardware::DutyPlan duty_plan{};

  /// Most recently requested normalized phase duties (entry 0 of duty_plan).
  system::ThreePhase<unit::Ratio> phase_duties{0.5_ratio, 0.5_ratio, 0.5_ratio};

  /// Newest raw ADC sample of the last cycle; updated by every control cycle
  /// regardless of control mode. Safe to read from lower-priority contexts
  /// (e.g. the startup FSM). The phase-current vector preserves ia + ib + ic = 0.
  system::ThreePhase<unit::Current> raw_phase_currents{};
  unit::Voltage raw_vdc{};

  /// Hard phase-current limit [A]. The hardware checks every sample against
  /// it and trips the power stage; the control cycle checks all samples of the
  /// cycle again. Infinity disables the check.
  float overcurrent_limit_A{std::numeric_limits<float>::infinity()};

  /// Set by the control cycle when the limit was exceeded; the control loop
  /// stays frozen at neutral duties until clear_overcurrent_trip().
  volatile bool overcurrent_tripped{false};
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
 * NUM_SUB_STEPS samples it calls `on_control_cycle()` in the zero-latency
 * context, which
 *
 *  1. reads the NUM_SUB_STEPS samples of the cycle,
 *  2. parks each sample with the angle it was taken at and averages them,
 *  3. runs the PI current controller once,
 *  4. computes NUM_SUB_STEPS duty entries, each with its own inverse-Park
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
 *   auto& isr = unimoc::current_control::CurrentControlIsr::instance();
 *   isr.init(settings, hardware, hardware.GetTimerClockFrequency());
 *
 *   // From hardware::ControlCycleHandler() (zero latency):
 *   isr.on_control_cycle();
 *
 *   // From outer control loop to update the setpoint:
 *   isr.set_current_ref({i_d_ref, i_q_ref});
 * @endcode
 */
class CurrentControlIsr {
 public:
  // =========================================================================
  // Singleton access
  // =========================================================================

  /// Return the single global instance.
  static CurrentControlIsr& instance() noexcept {
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
  void init(const settings::NvmSettings& settings,
            hardware::HardwareInterface& hardware,
            uint32_t timer_clock_hz) noexcept;

  // =========================================================================
  // ISR entry point
  // =========================================================================

  /**
   * @brief Control-cycle handler body.
   *
   * Called from hardware::ControlCycleHandler() (zero latency) once per
   * NUM_SUB_STEPS PWM half periods. Reads the samples of the cycle, runs the
   * current controller once and writes the duty plan of the next
   * NUM_SUB_STEPS half periods through hardware::hot_path. Does nothing before
   * init().
   */
  void on_control_cycle() noexcept;

  // =========================================================================
  // Setpoint interface (called from lower-priority context)
  // =========================================================================

  /**
   * @brief Update the rotor-frame current reference.
   *
   * @note The caller must ensure mutual exclusion (e.g. disable the ADC JEOC
   *       IRQ while writing, or use a double-buffered update) because
   *       Rotor<float> is not atomically writable on Cortex-M.
   *
   * @param ref  New d/q current reference [A].
   */
  void set_current_ref(const system::Rotor<unit::Current>& ref) noexcept { state.i_ref = ref; }

  // =========================================================================
  // Startup / bring-up interface
  // =========================================================================

  /**
   * @brief Override PWM output with fixed duty cycles, bypassing the PI loop.
   *
   * When active, on_control_cycle() still reads and stores the raw ADC samples in
   * state.raw_phase_currents / raw_vdc, but skips Clarke / Park / PI / SVM and
   * writes the requested duties through the hardware interface.
   *
   * Duties are clamped to [svm.duty_min, svm.duty_max].  To ensure the ADC
   * sampling window is never corrupted, consider keeping duties well within
   * this range during calibration steps.
   *
   * @param da  Phase-A duty cycle [0, 1].
   * @param db  Phase-B duty cycle [0, 1].
   * @param dc  Phase-C duty cycle [0, 1].
   */
  void force_duty(float da, float db, float dc) noexcept;

  /**
   * @brief Release force-duty mode and return to normal closed-loop PI control.
   *
   * Resets the current-controller integrators so the PI loop resumes from a
   * clean state rather than inheriting stale winding-up values accumulated
   * while force-duty was active.
   */
  void release_force_duty() noexcept;

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
  void set_adc_trigger_offset(uint32_t offset) noexcept;

  /**
   * @brief Set the hard phase-current limit that trips the power stage.
   *
   * The hardware checks every sample against it and trips the power stage at
   * once; on_control_cycle() checks the samples of the cycle again, calls
   * hardware::power_stage.Trip() when needed and freezes the control loop at
   * neutral duties.
   */
  void set_overcurrent_limit(unit::Current limit) noexcept;

  /// Acknowledges an overcurrent trip. The power stage fault is cleared separately
  /// (power_stage.ClearFault()).
  void clear_overcurrent_trip() noexcept {
    cc.reset();
    state.overcurrent_tripped = false;
  }

  // =========================================================================
  // Shared state (accessed by SlowUpdate)
  // =========================================================================

  /// Shared state between the ISR and the slow-update task.
  CurrentControlState state{};

  // =========================================================================
  // Algorithm instances (owned here; parameters loaded by init())
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

  /// When true, on_control_cycle() writes forced_duties as the duty plan and
  /// bypasses the PI control loop.  Set by force_duty(); cleared by
  /// release_force_duty().
  bool force_duty_active{false};

  /// Fixed duty cycles applied to CCR1/2/3 while force_duty_active is true.
  /// Clamped to [svm.duty_min, svm.duty_max] by force_duty().
  system::ThreePhase<unit::Ratio> forced_duties{0.5_ratio, 0.5_ratio, 0.5_ratio};

 private:
  CurrentControlIsr() = default;

  /// Writes the same duties for all entries of the plan.
  void set_uniform_duties(const system::ThreePhase<unit::Ratio>& duties) noexcept;

  void set_duty_plan(const hardware::DutyPlan& plan) noexcept;

  /// Ends a cycle that did not run the controller: FSM signalled, observer not run.
  void finish_cycle_without_control() noexcept;

  hardware::HardwareInterface* hardware_{nullptr};
};

}  // namespace current_control
}  // namespace unimoc
