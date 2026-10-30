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
#include "units.hpp"

namespace unimoc::startup {

using unit::operator""_A;
using unit::operator""_ratio;
using unit::operator""_V;

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

/// Why a run ended in FAULT (unimoc.drive.CommissioningStatus.0.1 FAULT_*).
enum class StartupFault : uint8_t {
  kNone = 0U,
  kAborted = 1U,        ///< The abort command or a restart.
  kEmergencyStop = 2U,  ///< The emergency stop command.
  kOvercurrent = 3U,    ///< A phase current above the limit of the run.
  kTrip = 4U,           ///< A trip of the control cycle opened the power stage.
  kEnableLost = 5U,     ///< The power stage is off or was not released (external enable, break).
  kApplyFailed = 6U,    ///< The calibration did not pass the settings validation at DONE.
};

/// What a running step waits for before it can measure on or continue.
enum class StartupNeeds : uint8_t {
  kNone = 0U,
  kDcLinkVoltage = 1U,   ///< The multimeter reading (DC_LINK_VOLTAGE_CHECK).
  kPhaseCurrent = 2U,    ///< The clamp-meter reading (CURRENT_SENSE_CALIBRATION).
  kMotorConnected = 3U,  ///< The confirmation that the motor is connected (CONNECT_MOTOR).
};

/// Answer to RequestRun().
enum class StartupStart : uint8_t { kStarted, kRunning, kNotReleased };

/// Answer to RequestNextStep().
enum class StartupContinue : uint8_t {
  kAccepted,  ///< The FSM continues at its next run.
  kNotRunning,
  kMeasuring,               ///< The step has not measured yet.
  kNeedsReading,            ///< The step waits for a meter reading first.
  kNeedsConfirmation,       ///< CONNECT_MOTOR: the motor-connected confirmation is missing.
  kUnexpectedConfirmation,  ///< A confirmation the step does not wait for.
};

/// The state of a run as the drive reports it (CommissioningStatus).
struct StartupStatus {
  FsmState step{FsmState::kIdle};
  bool active{false};
  bool measured{false};  ///< The present step has measured (it waits for the continue).
  StartupNeeds needs{StartupNeeds::kNone};
  uint8_t progress{0U};  ///< Of the present step [%].
  uint32_t passed{0U};   ///< One bit per FsmState.
  uint32_t failed{0U};   ///< Finished steps that did not pass.
  StartupFault fault{StartupFault::kNone};
  bool started{false};  ///< A run was started since the boot.
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
  unit::Current adc_offset_a;  ///< Measured phase-A zero offset.
  unit::Current adc_offset_b;  ///< Measured phase-B zero offset.
  unit::Current adc_offset_c;  ///< Measured phase-C zero offset.

  // ---- ADC_NOISE_FLOOR results -------------------------------------------
  unit::Current adc_noise_rms_a;  ///< Phase-A ADC noise RMS.
  unit::Current adc_noise_rms_b;  ///< Phase-B ADC noise RMS.

  // ---- DC_LINK_VOLTAGE_CHECK results -------------------------------------
  unit::Ratio gain_vdc{1.0_ratio};  ///< Computed V_dc ADC gain correction.

  // ---- CURRENT_SENSE_CALIBRATION results ---------------------------------
  unit::Ratio gain_a{1.0_ratio};  ///< Phase-A current-sense gain correction.
  unit::Ratio gain_b{1.0_ratio};  ///< Phase-B current-sense gain correction.
  unit::Ratio gain_c{1.0_ratio};  ///< Phase-C current-sense gain correction.

  // ---- PHASE_ADC_ALIGNMENT results ----------------------------------------
  uint32_t adc_trigger_offset_optimal{168U};  ///< Optimal trigger offset [timer ticks].

  // ---- FSM bookkeeping ---------------------------------------------------
  FsmState current_state{FsmState::kIdle};  ///< Snapshot of last known state.
};

}  // namespace unimoc::startup
