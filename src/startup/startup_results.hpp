/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file startup_results.hpp
 *    @brief Result struct and FSM-state enum of the hardware startup aid.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cstdint>

namespace unimoc::startup {

// =============================================================================
// FSM state enumeration
// =============================================================================

/// States of the hardware bring-up FSM.
///
/// Phase 1 (no motor connected): IDLE … GATE_DRIVER_ENABLE_CHECK
/// Phase 2 (motor connected, caution!): CONNECT_MOTOR … DONE
enum class FsmState : uint8_t {
  kIdle = 0U,                      ///< Not running; FSM is standby.
  kPwmDisable = 1U,                ///< Gate drivers disabled; safe baseline.
  kAdcOffsetCal = 2U,              ///< Phase 1: measure current-sense zero offsets.
  kAdcNoiseFloor = 3U,             ///< Phase 1: measure ADC noise RMS.
  kDutyForceLow = 4U,              ///< Phase 1: apply 5 % duty on all phases.
  kDutyForceMid = 5U,              ///< Phase 1: apply 50 % duty on all phases.
  kDutyForceHigh = 6U,             ///< Phase 1: apply 95 % duty on all phases.
  kDcLinkVoltageCheck = 7U,        ///< Phase 1: compare V_dc ADC to external meter.
  kGateDriverEnableCheck = 8U,     ///< Phase 1: toggle gate-enable and verify ADC response.
  kConnectMotor = 9U,              ///< Barrier: prompt user to connect motor; wait for step.
  kPhaseAdcAlignment = 10U,        ///< Phase 2: sweep ADC trigger offset; find optimal point.
  kCurrentSenseCalibration = 11U,  ///< Phase 2: force I_α; compare ADC vs external clamp meter.
  kDone = 12U,                     ///< All steps completed; results are logged and available.
  kFault = 13U,                    ///< Aborted due to OC, overrange, or user abort.
  kNumStates = 14U
};

// =============================================================================
// Per-step pass/fail
// =============================================================================

/// Total number of FSM states (includes IDLE, DONE, and FAULT).
/// The \c passed[] array is sized and indexed by FsmState value (0 … NUM_STATES-1);
/// IDLE, DONE, and FAULT entries are skipped when reporting results.
inline constexpr uint8_t kNumStartupSteps = static_cast<uint8_t>(FsmState::kNumStates);

// =============================================================================
// StartupResults
// =============================================================================

/// Flat POD holding pass/fail flags and measured values for every startup step.
///
/// Written by HwStartup during each step.  Read-back is available via the
/// Cyphal `unimoc.startup.results` register and via the Zephyr log at DONE.
struct StartupResults {
  // ---- per-step pass/fail (indexed by FsmState value) --------------------
  std::array<bool, kNumStartupSteps> passed{};  ///< passed[i] = true iff step i passed.

  // ---- ADC_OFFSET_CAL results --------------------------------------------
  float adc_offset_a{0.0F};  ///< Measured phase-A zero offset [A].
  float adc_offset_b{0.0F};  ///< Measured phase-B zero offset [A].

  // ---- ADC_NOISE_FLOOR results -------------------------------------------
  float adc_noise_rms_a{0.0F};  ///< Phase-A ADC noise RMS [A].
  float adc_noise_rms_b{0.0F};  ///< Phase-B ADC noise RMS [A].

  // ---- DC_LINK_VOLTAGE_CHECK results -------------------------------------
  float gain_vdc{1.0F};  ///< Computed V_dc ADC gain correction [dimensionless].

  // ---- CURRENT_SENSE_CALIBRATION results ---------------------------------
  float gain_a{1.0F};  ///< Phase-A current-sense gain correction [dimensionless].
  float gain_b{1.0F};  ///< Phase-B current-sense gain correction [dimensionless].

  // ---- PHASE_ADC_ALIGNMENT results ----------------------------------------
  uint32_t adc_trigger_offset_optimal{168U};  ///< Optimal trigger offset [timer ticks].

  // ---- FSM bookkeeping ---------------------------------------------------
  FsmState current_state{FsmState::kIdle};  ///< Snapshot of last known state.
};

}  // namespace unimoc::startup
