/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file hw_startup.hpp
 *    @brief Hardware startup-aid FSM for UNIMOC boards.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

/**
 * The FSM is split into two phases: safe ADC/PWM checks with no motor
 * connected, followed by low-voltage motor-connected checks with over-current
 * protection. It never advances automatically; each step waits for an
 * explicit operation request from the application or Cyphal service.
 *
 * The drive runs it from the Cyphal thread (drive::DriveNode, registers
 * `unimoc.startup.*`, drive::StartupRegisters): RunOnce() once per spin
 * while IsActive(), one ADC sample per call. The drive refuses to engage
 * while it runs and a run while engaged.
 *
 * @code
 * HwStartup hw_startup{cc_isr, settings_store.GetOperations()};
 * if (hw_startup.IsActive())
 *     hw_startup.RunOnce();
 * @endcode
 */

#include "current_control_isr.hpp"
#include "settings_store.hpp"
#include "startup_results.hpp"

namespace unimoc::startup {

// =============================================================================
// HwStartup
// =============================================================================

/// Hardware bring-up FSM.
class HwStartup {
 public:
  // =========================================================================
  // Configuration thresholds (public; override before calling RequestRun())
  // =========================================================================

  // NOLINTBEGIN(cppcoreguidelines-non-private-member-variables-in-classes): the
  // thresholds are plain configuration, set by the application or a Cyphal
  // register before RequestRun(); the results are read back the same way.

  /// Maximum acceptable current-sense zero offset.  Default 0.5 A.
  unit::Current offset_threshold{0.5_A};

  /// Maximum acceptable ADC noise RMS.  Default 0.1 A.
  unit::Current noise_threshold{0.1_A};

  /// Current-sense gain tolerance (± from 1.0).  Default 0.05 = 5 %.
  unit::Ratio gain_tolerance{0.05_ratio};

  /// V_dc gain tolerance.  Default 0.02 = 2 %.
  unit::Ratio vdc_gain_tolerance{0.02_ratio};

  /// Minimum V_dc reading considered valid.
  unit::Voltage vdc_min_valid{1.0_V};

  /// Fraction of max_phase_current_A used as OC trip limit during Phase 2.
  unit::Ratio oc_fraction{0.10_ratio};

  /// Duty fraction applied in forced-duty steps (offset from 0.5).
  ///   LOW  → 0.5 − duty_step_fraction
  ///   HIGH → 0.5 + duty_step_fraction
  unit::Ratio duty_step_fraction{0.45_ratio};  ///< gives 5 % / 95 % by default

  /// Voltage injection fraction for CURRENT_SENSE_CALIBRATION
  /// (fraction of ARR; tiny to avoid destroying low-Rs windings).
  unit::Ratio v_cal_fraction{0.002F};

  /// Number of ADC samples to average for offset/noise/gain calibrations.
  static constexpr uint32_t kNCal = 1024U;

  /// Number of samples to hold for each DUTY_FORCE_* step.
  static constexpr uint32_t kHoldSamples = 1000U;

  /// Samples per alignment sweep position.
  static constexpr uint32_t kNAlignSamples = 256U;

  /// Number of ADC-trigger sweep positions (±5 µs in 0.5 µs steps = 21).
  static constexpr uint32_t kNSweepSteps = 21U;

  // =========================================================================
  // External measurement inputs
  // =========================================================================

  /// Set by the Cyphal handler for `unimoc.startup.ext_current_A`.
  void SetExtCurrent(const unit::Current current) noexcept { ext_current_ = current; }

  /// Set by the Cyphal handler for `unimoc.startup.ext_vdc_V`.
  void SetExtVdc(const unit::Voltage voltage) noexcept { ext_vdc_ = voltage; }

  // =========================================================================
  // Results (populated during the FSM run; persistent until next request_run)
  // =========================================================================
  StartupResults results{};

  // NOLINTEND(cppcoreguidelines-non-private-member-variables-in-classes)

  // =========================================================================
  // Construction
  // =========================================================================

  /// Construct an HwStartup instance bound to a CurrentControlIsr and the
  /// authorized settings operation capability.
  ///
  /// \param current_control      The active current-control ISR; must outlive the FSM.
  /// \param settings_operations  Authorized settings operations (calibration commit).
  HwStartup(current_control::CurrentControlIsr& current_control,
            settings::SettingsOperations settings_operations) noexcept
      : cc_{&current_control}, settings_operations_{settings_operations} {}

  // =========================================================================
  // Control interface (called from Cyphal register handlers)
  // =========================================================================

  /// Start the FSM from IDLE, DONE or FAULT (a new run).  No-op while running.
  void RequestRun() noexcept;

  /// Advance the FSM one step (only valid while step_done_ is true).
  void RequestNextStep() noexcept;

  /// Abort a running FSM immediately; enter FAULT state.  No-op in IDLE, DONE and FAULT.
  void RequestAbort() noexcept;

  // =========================================================================
  // Polling interface (called from the main loop)
  // =========================================================================

  /// Returns true while the FSM is actively running.
  /// Returns false when the FSM is IDLE (never started), DONE (completed),
  /// or FAULT (aborted), so that normal application tasks (SlowUpdate) can
  /// resume after the startup sequence finishes.
  [[nodiscard]] bool IsActive() const noexcept {
    return state_ != FsmState::kIdle && state_ != FsmState::kDone && state_ != FsmState::kFault;
  }

  /// Returns the current FSM state.
  [[nodiscard]] FsmState State() const noexcept { return state_; }

  /// Polling function: consumes one samples_ready pulse of the control cycle
  /// per call (the drive calls it once per Cyphal spin, about 1 kHz).
  void RunOnce() noexcept;

 private:
  // =========================================================================
  // Internal helpers
  // =========================================================================

  void TransitionTo(FsmState next) noexcept;
  void AdvanceToNextState() noexcept;
  bool ValidateCurrentState() noexcept;
  void EnterFault(const char* reason) noexcept;

  void RunAdcOffsetCal() noexcept;
  void RunAdcNoiseFloor() noexcept;
  void RunDutyForce(unit::Ratio duty) noexcept;
  void RunDcLinkVoltageCheck() noexcept;
  void RunGateDriverEnableCheck() noexcept;
  void RunConnectMotorWait() noexcept;
  void RunPhaseAdcAlignment() noexcept;
  void RunCurrentSenseCalibration() noexcept;
  void RunDone() noexcept;

  bool CheckMotorOc() noexcept;
  bool CollectSample(uint32_t n_samples) noexcept;

  [[nodiscard]] static unit::Current MeanCurrent(double sum, uint32_t n_samples) noexcept;
  [[nodiscard]] static unit::Voltage MeanVoltage(double sum, uint32_t n_samples) noexcept;
  [[nodiscard]] static unit::Current RmsCurrent(double sum,
                                                double sum_of_squares,
                                                uint32_t n_samples) noexcept;

  static const char* StateName(FsmState state) noexcept;
  void LogSummary() noexcept;

  // =========================================================================
  // Data members
  // =========================================================================

  current_control::CurrentControlIsr* cc_;
  settings::SettingsOperations settings_operations_;

  FsmState state_{FsmState::kIdle};

  bool step_done_{false};            ///< Current step has finished collecting data.
  bool next_step_requested_{false};  ///< User requested advancement.

  uint32_t sample_count_{0U};  ///< Samples accumulated for current step.

  // Accumulators for running statistics. Plain doubles (CONTEXT.md, "Unit
  // Boundaries"): in float the variance E[x^2] - E[x]^2 loses most of its
  // digits when a current flows (alignment sweep).
  double accum_a_{0.0};
  double accum_b_{0.0};
  double accum_sq_a_{0.0};
  double accum_sq_b_{0.0};

  // Alignment sweep
  uint32_t sweep_pos_{0U};
  std::array<unit::Current, kNSweepSteps> sweep_noise_a_{};
  uint32_t base_trigger_offset_{0U};  ///< Initialised from cc_.state.adc_trigger_offset at sweep start.

  // Gate-driver enable-check accumulators
  unit::Current gate_mean_disabled_;
  unit::Current gate_mean_enabled_;
  bool gate_phase_disabled_{true};  ///< true = collecting disabled phase

  // External measurements
  unit::Current ext_current_;
  unit::Voltage ext_vdc_;

  // OC limit (computed once at Phase 2 entry)
  unit::Current oc_limit_{10.0_A};
};

}  // namespace unimoc::startup
