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
 * Usage from the main application loop:
 * @code
 * HwStartup hw_startup{cc_isr, settings_store.GetOperations()};
 * if (hw_startup.IsActive())
 *     hw_startup.RunOnce();
 * else
 *     slow_update.RunOnce();
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

  /// Maximum acceptable current-sense zero offset [A].  Default 0.5 A.
  float offset_threshold{0.5F};

  /// Maximum acceptable ADC noise RMS [A].  Default 0.1 A.
  float noise_threshold{0.1F};

  /// Current-sense gain tolerance (± from 1.0).  Default 0.05 = 5 %.
  float gain_tolerance{0.05F};

  /// V_dc gain tolerance.  Default 0.02 = 2 %.
  float vdc_gain_tolerance{0.02F};

  /// Minimum V_dc reading considered valid [V].
  float vdc_min_valid{1.0F};

  /// Fraction of max_phase_current_A used as OC trip limit during Phase 2.
  float oc_fraction{0.10F};

  /// Duty fraction applied in forced-duty steps (offset from 0.5).
  ///   LOW  → 0.5 − duty_step_fraction
  ///   HIGH → 0.5 + duty_step_fraction
  float duty_step_fraction{0.45F};  ///< gives 5 % / 95 % by default

  /// Voltage injection fraction for CURRENT_SENSE_CALIBRATION
  /// (fraction of ARR; tiny to avoid destroying low-Rs windings).
  float v_cal_fraction{0.002F};

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
  void SetExtCurrent(float amps) noexcept { ext_current_ = amps; }

  /// Set by the Cyphal handler for `unimoc.startup.ext_vdc_V`.
  void SetExtVdc(float volts) noexcept { ext_vdc_ = volts; }

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

  /// Start the FSM from IDLE.  No-op if already running.
  void RequestRun() noexcept;

  /// Advance the FSM one step (only valid while step_done_ is true).
  void RequestNextStep() noexcept;

  /// Abort the FSM immediately; enter FAULT state.
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

  /// Main polling function — call once per samples_ready pulse
  /// (i.e. at the same rate as SlowUpdate::RunOnce(); ~10 kHz).
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
  void RunDutyForce(float duty) noexcept;
  void RunDcLinkVoltageCheck() noexcept;
  void RunGateDriverEnableCheck() noexcept;
  void RunConnectMotorWait() noexcept;
  void RunPhaseAdcAlignment() noexcept;
  void RunCurrentSenseCalibration() noexcept;
  void RunDone() noexcept;

  bool CheckMotorOc() noexcept;
  bool CollectSample(uint32_t n_samples) noexcept;

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

  // Accumulators for running statistics
  double accum_a_{0.0};
  double accum_b_{0.0};
  double accum_sq_a_{0.0};
  double accum_sq_b_{0.0};

  // Alignment sweep
  uint32_t sweep_pos_{0U};
  std::array<float, kNSweepSteps> sweep_noise_a_{};
  uint32_t base_trigger_offset_{0U};  ///< Initialised from cc_.state.adc_trigger_offset at sweep start.

  // Gate-driver enable-check accumulators
  float gate_mean_disabled_{0.0F};
  float gate_mean_enabled_{0.0F};
  bool gate_phase_disabled_{true};  ///< true = collecting disabled phase

  // External measurements
  float ext_current_{0.0F};
  float ext_vdc_{0.0F};

  // OC limit (computed once at Phase 2 entry)
  float oc_limit_{10.0F};
};

}  // namespace unimoc::startup
