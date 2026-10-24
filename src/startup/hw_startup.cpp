/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file hw_startup.cpp
 *    @brief Hardware startup-aid FSM implementation.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "hw_startup.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include "hardware_interface.hpp"
#include "power_stage_interface.hpp"

namespace unimoc::startup {

namespace {

using unit::operator""_ratio;

/// Both switches of a leg for the same time: no voltage across the motor.
constexpr unit::Ratio kNeutralDuty = 0.5_ratio;

}  // namespace

void LogInfo(const char* const message) noexcept {
  hardware::kRuntime.Log(hardware::LogLevel::kInfo, message);
}

void LogWarning(const char* const message) noexcept {
  hardware::kRuntime.Log(hardware::LogLevel::kWarning, message);
}

#if defined(__GNUC__) || defined(__clang__)
void LogFormatted(hardware::LogLevel level, const char* format, ...) noexcept
    __attribute__((format(printf, 2, 3)));
#else
void LogFormatted(const hardware::LogLevel level, const char* const format, ...) noexcept;
#endif

void LogFormatted(const hardware::LogLevel level, const char* const format, ...) noexcept {
  std::array<char, 512> message{};
  // va_list is an array type on some ABIs (x86-64); the va_* macros and
  // vsnprintf need it as it is.
  std::va_list arguments;
  va_start(arguments, format);  // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
  const int kResult = std::vsnprintf(message.data(), message.size(), format, arguments);
  va_end(arguments);  // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
  if (kResult >= 0) {
    hardware::kRuntime.Log(level, message.data());
  }
}

// =============================================================================
// request_run
// =============================================================================

void HwStartup::RequestRun() noexcept {
  if (IsActive()) {
    return;  // a run is in progress; DONE and FAULT start a new one
  }
  state_ = FsmState::kIdle;
  results.current_state = FsmState::kIdle;

  // Reset all results and counters
  results = StartupResults{};
  sample_count_ = 0U;
  sweep_pos_ = 0U;
  step_done_ = false;
  next_step_requested_ = false;
  ext_current_ = 0.0F;
  ext_vdc_ = 0.0F;

  // A new run is the explicit acknowledgement of a latched break and of an
  // overcurrent trip. The external enable must still be present (Enable() checks).
  if (hardware::kPowerStage.GetState() == hardware::PowerStageState::kFault) {
    hardware::kPowerStage.ClearFault();
  }
  cc_->ClearTrips();
  cc_->ForceDuty(kNeutralDuty, kNeutralDuty, kNeutralDuty);  // neutral before the outputs are released

  LogInfo(
      "[STARTUP] ================================================\n"
      "[STARTUP] Hardware bring-up sequence started.\n"
      "[STARTUP] Phase 1 - NO MOTOR CONNECTED.\n"
      "[STARTUP] Advance each step by writing 1 to\n"
      "[STARTUP]   unimoc.startup.step\n"
      "[STARTUP] ================================================\n");

  // Release the outputs at neutral duty. Without the external enable (jumper,
  // e-stop) this fails.
  // The run does not start in that case; the FSM stays IDLE so it can be requested again.
  if (!hardware::kPowerStage.Enable()) {
    cc_->ReleaseForceDuty();
    LogFormatted(hardware::LogLevel::kError,
                 "[STARTUP] Power stage not released (external enable missing or break active). Not "
                 "started.\n");
    return;
  }

  TransitionTo(FsmState::kPwmDisable);
}

// =============================================================================
// request_next_step
// =============================================================================

void HwStartup::RequestNextStep() noexcept {
  if (!IsActive()) {
    return;
  }
  next_step_requested_ = true;
}

// =============================================================================
// request_abort
// =============================================================================

void HwStartup::RequestAbort() noexcept {
  // No-op if the FSM was never started or has already completed.
  if (state_ == FsmState::kIdle || state_ == FsmState::kDone) {
    return;
  }
  EnterFault("user abort");
}

// =============================================================================
// run_once  —  called at the samples_ready rate (~10 kHz)
// =============================================================================

void HwStartup::RunOnce() noexcept {
  // Consume the samples_ready flag (mirrors SlowUpdate behaviour)
  if (!cc_->state.samples_ready) {
    return;
  }
  cc_->state.samples_ready = false;

  // OC check applies to all Phase-2 states
  // The stage is released for the whole run. If it is not any more (hard
  // overcurrent trip in the ISR, external break, e-stop), stop here.
  if (state_ != FsmState::kIdle && state_ != FsmState::kDone && state_ != FsmState::kFault &&
      hardware::kPowerStage.GetState() != hardware::PowerStageState::kEnabled) {
    const char* const kTrip = cc_->TripName();
    EnterFault(kTrip != nullptr ? kTrip : "power stage broken or blocked");
    return;
  }

  if (state_ >= FsmState::kConnectMotor && state_ < FsmState::kDone) {
    if (CheckMotorOc()) {
      return;  // enter_fault already called
    }
  }

  switch (state_) {
    case FsmState::kIdle:
      break;

    case FsmState::kPwmDisable:
      // Force 50 % neutral duty on all phases; wait for user step.
      cc_->ForceDuty(kNeutralDuty, kNeutralDuty, kNeutralDuty);
      if (!step_done_) {
        LogInfo(
            "[STARTUP] Step PWM_DISABLE: 50 % duty applied on all phases.\n"
            "[STARTUP] Verify gate-driver outputs are toggling. Press NEXT when ready.\n");
        step_done_ = true;
      }
      if (step_done_ && next_step_requested_ && ValidateCurrentState()) {
        results.passed[static_cast<uint8_t>(FsmState::kPwmDisable)] = true;
        AdvanceToNextState();
      }
      break;

    case FsmState::kAdcOffsetCal:
      RunAdcOffsetCal();
      break;

    case FsmState::kAdcNoiseFloor:
      RunAdcNoiseFloor();
      break;

    case FsmState::kDutyForceLow:
      RunDutyForce(kNeutralDuty - duty_step_fraction);
      break;

    case FsmState::kDutyForceMid:
      RunDutyForce(kNeutralDuty);
      break;

    case FsmState::kDutyForceHigh:
      RunDutyForce(kNeutralDuty + duty_step_fraction);
      break;

    case FsmState::kDcLinkVoltageCheck:
      RunDcLinkVoltageCheck();
      break;

    case FsmState::kGateDriverEnableCheck:
      RunGateDriverEnableCheck();
      break;

    case FsmState::kConnectMotor:
      RunConnectMotorWait();
      break;

    case FsmState::kPhaseAdcAlignment:
      RunPhaseAdcAlignment();
      break;

    case FsmState::kCurrentSenseCalibration:
      RunCurrentSenseCalibration();
      break;

    case FsmState::kDone:
      // Nothing to do; remain in DONE until reset.
      break;

    case FsmState::kFault:
      // Remain in FAULT; safe 50 % duty is applied in EnterFault().
      break;

    default:
      break;
  }
}

// =============================================================================
// transition_to
// =============================================================================

void HwStartup::TransitionTo(FsmState next) noexcept {
  state_ = next;
  step_done_ = false;
  next_step_requested_ = false;
  sample_count_ = 0U;
  accum_a_ = 0.0;
  accum_b_ = 0.0;
  accum_sq_a_ = 0.0;
  accum_sq_b_ = 0.0;

  results.current_state = next;

  LogFormatted(hardware::LogLevel::kInfo, "[STARTUP] Entering state: %s\n", StateName(next));
}

// =============================================================================
// advance_to_next_state
// =============================================================================

void HwStartup::AdvanceToNextState() noexcept {
  next_step_requested_ = false;

  const auto kState = static_cast<uint8_t>(state_);
  const auto kNextRaw = static_cast<uint8_t>(kState + 1U);

  if (kNextRaw >= static_cast<uint8_t>(FsmState::kNumStates)) {
    RunDone();
    return;
  }

  TransitionTo(static_cast<FsmState>(kNextRaw));
}

// =============================================================================
// validate_current_state
// =============================================================================

bool HwStartup::ValidateCurrentState() noexcept {
  if (state_ == FsmState::kCurrentSenseCalibration) {
    if (ext_current_ < 1e-3F) {
      LogWarning(
          "[STARTUP] CURRENT_SENSE_CALIBRATION: ext_current_A is zero. "
          "Please enter the clamp-meter reading via unimoc.startup.ext_current_A before advancing.\n");
      return false;
    }
  }
  if (state_ == FsmState::kDcLinkVoltageCheck) {
    if (ext_vdc_ < vdc_min_valid) {
      LogWarning(
          "[STARTUP] DC_LINK_VOLTAGE_CHECK: ext_vdc_V is not set. "
          "Please enter the multimeter reading via unimoc.startup.ext_vdc_V before advancing.\n");
      return false;
    }
  }
  return true;
}

// =============================================================================
// enter_fault
// =============================================================================

void HwStartup::EnterFault(const char* reason) noexcept {
  state_ = FsmState::kFault;
  results.current_state = FsmState::kFault;
  cc_->ForceDuty(kNeutralDuty, kNeutralDuty, kNeutralDuty);  // safe neutral
  hardware::kPowerStage.Disable();                           // outputs off, whatever the duties do
  LogFormatted(hardware::LogLevel::kError, "[STARTUP] *** FAULT: %s ***\n", reason);
}

// =============================================================================
// check_motor_oc
// =============================================================================

bool HwStartup::CheckMotorOc() noexcept {
  const auto& phase_currents = cc_->state.raw_phase_currents;
  const float kIa = phase_currents.a.Value();
  const float kIb = phase_currents.b.Value();
  const float kIc = phase_currents.c.Value();
  if ((std::abs(kIa) > oc_limit_) || (std::abs(kIb) > oc_limit_) || (std::abs(kIc) > oc_limit_)) {
    EnterFault("over-current detected");
    return true;
  }
  return false;
}

// =============================================================================
// collect_sample  — accumulate one sample; return true when n_samples reached
// =============================================================================

bool HwStartup::CollectSample(uint32_t n_samples) noexcept {
  const auto kIa = static_cast<double>(cc_->state.raw_phase_currents.a.Value());
  const auto kIb = static_cast<double>(cc_->state.raw_phase_currents.b.Value());
  accum_a_ += kIa;
  accum_b_ += kIb;
  accum_sq_a_ += kIa * kIa;
  accum_sq_b_ += kIb * kIb;
  ++sample_count_;
  return (sample_count_ >= n_samples);
}

// =============================================================================
// run_adc_offset_cal
// =============================================================================

void HwStartup::RunAdcOffsetCal() noexcept {
  // Ensure neutral duty while collecting
  cc_->ForceDuty(kNeutralDuty, kNeutralDuty, kNeutralDuty);

  if (!step_done_) {
    if (CollectSample(kNCal)) {
      const auto kSampleCount = static_cast<float>(kNCal);
      const float kMeanA = static_cast<float>(accum_a_) / kSampleCount;
      const float kMeanB = static_cast<float>(accum_b_) / kSampleCount;

      results.adc_offset_a = kMeanA;
      results.adc_offset_b = kMeanB;

      const bool kPass = (std::abs(kMeanA) < offset_threshold) && (std::abs(kMeanB) < offset_threshold);

      results.passed[static_cast<uint8_t>(FsmState::kAdcOffsetCal)] = kPass;

      LogFormatted(hardware::LogLevel::kInfo,
                   "[STARTUP] ADC_OFFSET_CAL: offset_a=%f A, offset_b=%f A  %s\n",
                   static_cast<double>(kMeanA),
                   static_cast<double>(kMeanB),
                   kPass ? "PASS" : "FAIL");

      if (!kPass) {
        LogFormatted(hardware::LogLevel::kWarning,
                     "[STARTUP] ADC offset exceeds threshold (%f A). Check op-amp supply, resistors, and PCB "
                     "connections.\n",
                     static_cast<double>(offset_threshold));
      }

      step_done_ = true;
      LogInfo("[STARTUP] Press NEXT to continue.\n");
    }
  }

  if (step_done_ && next_step_requested_ && ValidateCurrentState()) {
    AdvanceToNextState();
  }
}

// =============================================================================
// run_adc_noise_floor
// =============================================================================

void HwStartup::RunAdcNoiseFloor() noexcept {
  cc_->ForceDuty(kNeutralDuty, kNeutralDuty, kNeutralDuty);

  if (!step_done_) {
    if (CollectSample(kNCal)) {
      const auto kSampleCount = static_cast<double>(kNCal);
      // variance = E[x²] - E[x]²
      const auto kVarA = static_cast<float>(accum_sq_a_ / kSampleCount -
                                            (accum_a_ / kSampleCount) * (accum_a_ / kSampleCount));
      const auto kVarB = static_cast<float>(accum_sq_b_ / kSampleCount -
                                            (accum_b_ / kSampleCount) * (accum_b_ / kSampleCount));

      results.adc_noise_rms_a = (kVarA > 0.0F) ? std::sqrt(kVarA) : 0.0F;
      results.adc_noise_rms_b = (kVarB > 0.0F) ? std::sqrt(kVarB) : 0.0F;

      const bool kPass =
          (results.adc_noise_rms_a < noise_threshold) && (results.adc_noise_rms_b < noise_threshold);

      results.passed[static_cast<uint8_t>(FsmState::kAdcNoiseFloor)] = kPass;

      LogFormatted(hardware::LogLevel::kInfo,
                   "[STARTUP] ADC_NOISE_FLOOR: rms_a=%f A, rms_b=%f A  %s\n",
                   static_cast<double>(results.adc_noise_rms_a),
                   static_cast<double>(results.adc_noise_rms_b),
                   kPass ? "PASS" : "FAIL");

      if (!kPass) {
        LogFormatted(hardware::LogLevel::kWarning,
                     "[STARTUP] ADC noise exceeds threshold (%f A). Check decoupling caps, layout, and "
                     "ground paths.\n",
                     static_cast<double>(noise_threshold));
      }

      step_done_ = true;
      LogInfo("[STARTUP] Press NEXT to continue.\n");
    }
  }

  if (step_done_ && next_step_requested_ && ValidateCurrentState()) {
    AdvanceToNextState();
  }
}

// =============================================================================
// run_duty_force  —  used by DUTY_FORCE_LOW / MID / HIGH
// =============================================================================

void HwStartup::RunDutyForce(const unit::Ratio duty) noexcept {
  cc_->ForceDuty(duty, duty, duty);

  if (!step_done_) {
    if (sample_count_ < kHoldSamples) {
      ++sample_count_;
      return;
    }

    // Manual confirmation only; always PASS-MANUAL
    results.passed[static_cast<uint8_t>(state_)] = true;

    LogFormatted(hardware::LogLevel::kInfo,
                 "[STARTUP] %s: duty=%f applied for %u samples. PASS-MANUAL (verify with scope).\n"
                 "[STARTUP] Press NEXT to continue.\n",
                 StateName(state_),
                 static_cast<double>(duty.Value()),  // the log takes plain numbers
                 static_cast<unsigned int>(kHoldSamples));

    step_done_ = true;
  }

  if (step_done_ && next_step_requested_ && ValidateCurrentState()) {
    AdvanceToNextState();
  }
}

// =============================================================================
// run_dc_link_voltage_check
// =============================================================================

void HwStartup::RunDcLinkVoltageCheck() noexcept {
  cc_->ForceDuty(kNeutralDuty, kNeutralDuty, kNeutralDuty);

  if (step_done_) {
    if (next_step_requested_ && ValidateCurrentState()) {
      AdvanceToNextState();
    }
    return;
  }

  // Accumulate raw_vdc directly - do NOT use CollectSample() which
  // accumulates phase currents and would give a meaningless mean here.
  if (sample_count_ < kNCal) {
    accum_a_ += static_cast<double>(cc_->state.raw_vdc.Value());
    ++sample_count_;
    if (sample_count_ < kNCal) {
      return;
    }
    // kNCal samples just collected — fall through to evaluate.
  }

  // kNCal samples collected; compute the kNCal-sample average V_dc reading.
  const float kMeasuredVdc = static_cast<float>(accum_a_) / static_cast<float>(kNCal);

  if (ext_vdc_ > vdc_min_valid) {
    results.gain_vdc = kMeasuredVdc / ext_vdc_;
    const bool kPass = std::abs(results.gain_vdc - 1.0F) < vdc_gain_tolerance;

    results.passed[static_cast<uint8_t>(FsmState::kDcLinkVoltageCheck)] = kPass;

    LogFormatted(hardware::LogLevel::kInfo,
                 "[STARTUP] DC_LINK_VOLTAGE_CHECK: adc_vdc=%f V, ext=%f V, gain=%f  %s\n",
                 static_cast<double>(kMeasuredVdc),
                 static_cast<double>(ext_vdc_),
                 static_cast<double>(results.gain_vdc),
                 kPass ? "PASS" : "FAIL");

    if (!kPass) {
      LogFormatted(hardware::LogLevel::kWarning,
                   "[STARTUP] V_dc gain error exceeds %f %%. Check voltage-divider resistors on V_dc sense "
                   "circuit.\n"
                   "[STARTUP] Suggested correction factor: %f\n",
                   static_cast<double>(vdc_gain_tolerance * 100.0F),
                   static_cast<double>(results.gain_vdc));
    }

    step_done_ = true;
    LogInfo("[STARTUP] Press NEXT to continue.\n");
  } else if (sample_count_ == kNCal) {
    // Log the "waiting" prompt exactly once (bump sample_count_ as sentinel).
    LogFormatted(hardware::LogLevel::kInfo,
                 "[STARTUP] DC_LINK_VOLTAGE_CHECK: adc_vdc=%f V. Enter multimeter reading via "
                 "unimoc.startup.ext_vdc_V, "
                 "then press NEXT.\n",
                 static_cast<double>(kMeasuredVdc));
    ++sample_count_;
    // step_done_ stays false; keep polling until ext_vdc_ is provided.
  }
  // else: sample_count_ > kNCal, ext_vdc_ still not valid — silent wait.
}

// =============================================================================
// run_gate_driver_enable_check
// =============================================================================

void HwStartup::RunGateDriverEnableCheck() noexcept {
  // Phase 1 of this step: collect with 50 % duty (baseline noise).
  // Phase 2: collect with a small 60 % / 40 % imbalance (simulates gate on).
  // On real hardware the gate-enable GPIO would be toggled here; on the
  // current MCU-agnostic model we simply observe the ADC response delta
  // between neutral (50 %) and a slightly off-neutral duty.

  static constexpr unit::Ratio kGateTestDutyHigh = 0.55_ratio;
  static constexpr unit::Ratio kGateTestDutyLow = 0.45_ratio;
  static constexpr uint32_t kNGate = 512U;

  // Keep the step idle once results have been collected.
  if (step_done_) {
    if (next_step_requested_ && ValidateCurrentState()) {
      AdvanceToNextState();
    }
    return;
  }

  if (gate_phase_disabled_) {
    // Collecting disabled (neutral) baseline
    cc_->ForceDuty(kNeutralDuty, kNeutralDuty, kNeutralDuty);
    if (CollectSample(kNGate)) {
      gate_mean_disabled_ = static_cast<float>(accum_a_) / static_cast<float>(kNGate);
      // Reset accumulators for enabled phase
      sample_count_ = 0U;
      accum_a_ = 0.0;
      accum_b_ = 0.0;
      accum_sq_a_ = 0.0;
      accum_sq_b_ = 0.0;
      gate_phase_disabled_ = false;
      LogFormatted(hardware::LogLevel::kInfo,
                   "[STARTUP] GATE_DRIVER_ENABLE_CHECK: baseline mean_ia=%f A\n",
                   static_cast<double>(gate_mean_disabled_));
    }
    return;  // continue next call
  }

  // Collecting "enabled" phase (small imbalance duty)
  cc_->ForceDuty(kGateTestDutyHigh, kGateTestDutyLow, kGateTestDutyLow);
  if (CollectSample(kNGate)) {
    gate_mean_enabled_ = static_cast<float>(accum_a_) / static_cast<float>(kNGate);

    const float kDelta = std::abs(gate_mean_enabled_ - gate_mean_disabled_);
    const float kThreshold = 2.0F * results.adc_noise_rms_a;
    const bool kPass = (kDelta > kThreshold);

    results.passed[static_cast<uint8_t>(FsmState::kGateDriverEnableCheck)] = kPass;

    LogFormatted(hardware::LogLevel::kInfo,
                 "[STARTUP] GATE_DRIVER_ENABLE_CHECK: delta_ia=%f A, threshold=%f A  %s\n",
                 static_cast<double>(kDelta),
                 static_cast<double>(kThreshold),
                 kPass ? "PASS" : "FAIL");

    if (!kPass) {
      LogWarning(
          "[STARTUP] Gate-driver response smaller than 2x noise floor. Check gate-enable GPIO and driver "
          "power supply.\n");
    }

    // Reset flag for potential re-run
    gate_phase_disabled_ = true;
    step_done_ = true;
    LogInfo("[STARTUP] Press NEXT to continue.\n");
  }
}

// =============================================================================
// run_connect_motor_wait  —  barrier state; user must confirm motor is connected
// =============================================================================

void HwStartup::RunConnectMotorWait() noexcept {
  cc_->ForceDuty(kNeutralDuty, kNeutralDuty, kNeutralDuty);

  if (!step_done_) {
    oc_limit_ = settings_operations_.GetHardwareCapabilities().max_phase_current.Value() * oc_fraction;

    LogWarning(
        "\n"
        "[STARTUP] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
        "[STARTUP] !!  PHASE 1 COMPLETE - PREPARE FOR PHASE 2       !!\n"
        "[STARTUP] !!                                                !!\n"
        "[STARTUP] !!  ACTION REQUIRED:                              !!\n"
        "[STARTUP] !!    1. POWER OFF the drive now.                 !!\n"
        "[STARTUP] !!    2. CONNECT the motor (UVW + PE).            !!\n"
        "[STARTUP] !!    3. POWER ON again.                          !!\n"
        "[STARTUP] !!    4. Write 1 to unimoc.startup.step           !!\n"
        "[STARTUP] !!       ONLY after motor is securely connected.  !!\n"
        "[STARTUP] !!                                                !!\n"
        "[STARTUP] !!  CAUTION: Phase 2 injects LIVE voltages.       !!\n"
        "[STARTUP] !!  LOW-Rs WINDINGS: even tiny duty changes cause !!\n"
        "[STARTUP] !!  large currents. OC limit is set to            !!\n");
    LogFormatted(hardware::LogLevel::kWarning,
                 "[STARTUP] !!  %f A (%f%% of max).\n"
                 "[STARTUP] !!  Ensure motor shaft is FREE to rotate.        !!\n"
                 "[STARTUP] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n",
                 static_cast<double>(oc_limit_),
                 static_cast<double>(oc_fraction * 100.0F));

    results.passed[static_cast<uint8_t>(FsmState::kConnectMotor)] = true;
    step_done_ = true;
  }

  if (step_done_ && next_step_requested_ && ValidateCurrentState()) {
    AdvanceToNextState();
  }
}

// =============================================================================
// run_phase_adc_alignment
// =============================================================================

void HwStartup::RunPhaseAdcAlignment() noexcept {
  // Force a known asymmetric duty so phase A carries more current than B/C.
  static constexpr unit::Ratio kDa = 0.8_ratio;
  static constexpr unit::Ratio kDb = 0.2_ratio;
  static constexpr unit::Ratio kDc = 0.2_ratio;
  cc_->ForceDuty(kDa, kDb, kDc);

  if (step_done_) {
    if (next_step_requested_ && ValidateCurrentState()) {
      AdvanceToNextState();
    }
    return;
  }

  // Initialise base_trigger_offset_ from the actual hardware value on first
  // entry (sweep_pos_ == 0) so the sweep range and step size are correct for
  // any timer clock, not just 168 MHz.
  if (sweep_pos_ == 0U && sample_count_ == 0U) {
    base_trigger_offset_ = cc_->state.adc_trigger_offset;
  }

  if (sweep_pos_ >= kNSweepSteps) {
    // Find sweep position with minimum noise
    uint32_t best_pos = 0U;
    float best_noise = sweep_noise_a_[0U];
    for (uint32_t i = 1U; i < kNSweepSteps; ++i) {
      if (sweep_noise_a_[i] < best_noise) {
        best_noise = sweep_noise_a_[i];
        best_pos = i;
      }
    }

    // Convert position to timer ticks (keep signed to detect underflow).
    // position 0 = base - N/2 * half_tick, step = 0.5 µs = base_trigger_offset_/2
    const auto kHalfTick = static_cast<int32_t>(base_trigger_offset_ / 2U);
    const int32_t kOffsetSteps = static_cast<int32_t>(best_pos) - static_cast<int32_t>(kNSweepSteps / 2U);
    const int32_t kTicksSigned = static_cast<int32_t>(base_trigger_offset_) + kOffsetSteps * kHalfTick;
    // Clamp to a non-negative value before storing as uint32_t.
    const uint32_t kOptimalTicks = (kTicksSigned > 0) ? static_cast<uint32_t>(kTicksSigned) : 0U;

    results.adc_trigger_offset_optimal = kOptimalTicks;

    // Pass: min noise < 2× noise floor from ADC_NOISE_FLOOR step
    const float kNoiseFloor = results.adc_noise_rms_a;
    const bool kPass = (best_noise < 2.0F * kNoiseFloor);
    results.passed[static_cast<uint8_t>(FsmState::kPhaseAdcAlignment)] = kPass;

    LogFormatted(hardware::LogLevel::kInfo,
                 "[STARTUP] PHASE_ADC_ALIGNMENT: best_pos=%u, optimal_offset=%u ticks, min_noise=%f A  %s\n",
                 static_cast<unsigned int>(best_pos),
                 static_cast<unsigned int>(kOptimalTicks),
                 static_cast<double>(best_noise),
                 kPass ? "PASS" : "FAIL");

    if (!kPass) {
      LogWarning(
          "[STARTUP] Could not find a clean ADC sampling window. Check PWM frequency, ADC trigger timing, "
          "and hardware layout.\n");
    }

    LogFormatted(hardware::LogLevel::kInfo,
                 "[STARTUP] Suggest writing adc_trigger_offset=%u to NvmSettings to persist the optimal "
                 "offset.\n"
                 "[STARTUP] Press NEXT to continue.\n",
                 static_cast<unsigned int>(kOptimalTicks));

    cc_->SetAdcTriggerOffset(kOptimalTicks);
    step_done_ = true;
    return;
  }

  // Collecting kNAlignSamples at the current sweep position
  if (CollectSample(kNAlignSamples)) {
    const auto kSampleCount = static_cast<double>(kNAlignSamples);
    const auto kVar = static_cast<float>(accum_sq_a_ / kSampleCount -
                                         (accum_a_ / kSampleCount) * (accum_a_ / kSampleCount));
    sweep_noise_a_[sweep_pos_] = (kVar > 0.0F) ? std::sqrt(kVar) : 0.0F;

    // Advance sweep
    ++sweep_pos_;
    sample_count_ = 0U;
    accum_a_ = 0.0;
    accum_b_ = 0.0;
    accum_sq_a_ = 0.0;
    accum_sq_b_ = 0.0;

    // Set trigger offset for next sweep position (clamp to non-negative).
    const auto kHalfTick = static_cast<int32_t>(base_trigger_offset_ / 2U);
    const int32_t kOffsetSteps = static_cast<int32_t>(sweep_pos_) - static_cast<int32_t>(kNSweepSteps / 2U);
    const int32_t kNewSigned = static_cast<int32_t>(base_trigger_offset_) + kOffsetSteps * kHalfTick;
    const uint32_t kNewOffset = (kNewSigned > 0) ? static_cast<uint32_t>(kNewSigned) : 0U;
    cc_->SetAdcTriggerOffset(kNewOffset);
  }
}

// =============================================================================
// run_current_sense_calibration
// =============================================================================

void HwStartup::RunCurrentSenseCalibration() noexcept {
  // Force pure alpha-axis current injection (small voltage differential)
  const unit::Ratio kDa = kNeutralDuty + v_cal_fraction;
  const unit::Ratio kDb = kNeutralDuty - v_cal_fraction * 0.5F;
  const unit::Ratio kDc = kNeutralDuty - v_cal_fraction * 0.5F;
  cc_->ForceDuty(kDa, kDb, kDc);

  if (!step_done_ && !CollectSample(kNCal)) {
    return;  // still collecting
  }

  if (!step_done_) {
    // Measurement complete
    const auto kSampleCount = static_cast<float>(kNCal);
    const float kMeanIa = static_cast<float>(accum_a_) / kSampleCount;
    const float kMeanIb = static_cast<float>(accum_b_) / kSampleCount;

    LogFormatted(hardware::LogLevel::kInfo,
                 "[STARTUP] CURRENT_SENSE_CALIBRATION: mean_ia=%f A, mean_ib=%f A\n",
                 static_cast<double>(kMeanIa),
                 static_cast<double>(kMeanIb));

    if (ext_current_ >= 1e-3F) {
      // gain = adc_reading / ext_reference; ideal = 1.0
      results.gain_a = (std::abs(kMeanIa) > 1e-4F) ? (kMeanIa / ext_current_) : 0.0F;
      results.gain_b = (std::abs(kMeanIb) > 1e-4F) ? (kMeanIb / (-ext_current_ * 0.5F)) : 0.0F;

      const bool kPass = (std::abs(results.gain_a - 1.0F) < gain_tolerance) &&
                         (results.gain_b > 1e-4F && std::abs(results.gain_b - 1.0F) < gain_tolerance);

      results.passed[static_cast<uint8_t>(FsmState::kCurrentSenseCalibration)] = kPass;

      LogFormatted(hardware::LogLevel::kInfo,
                   "[STARTUP] CURRENT_SENSE_CALIBRATION: ext=%f A, gain_a=%f, gain_b=%f  %s\n",
                   static_cast<double>(ext_current_),
                   static_cast<double>(results.gain_a),
                   static_cast<double>(results.gain_b),
                   kPass ? "PASS" : "FAIL");

      if (!kPass) {
        LogFormatted(hardware::LogLevel::kWarning,
                     "[STARTUP] Gain error exceeds %f %%. Check voltage-divider and op-amp gain resistors "
                     "for current-sense channels.\n"
                     "[STARTUP] Correction factors: adc_gain_a=%f, adc_gain_b=%f\n",
                     static_cast<double>(gain_tolerance * 100.0F),
                     static_cast<double>(1.0F / results.gain_a),
                     static_cast<double>(1.0F / results.gain_b));
      }

    } else {
      LogInfo(
          "[STARTUP] CURRENT_SENSE_CALIBRATION: waiting for ext_current_A. "
          "Enter clamp-meter reading via unimoc.startup.ext_current_A, then press NEXT.\n");
    }

    step_done_ = true;
    LogInfo("[STARTUP] Press NEXT to continue.\n");
  }

  if (step_done_ && next_step_requested_ && ValidateCurrentState()) {
    AdvanceToNextState();
  }
}

// =============================================================================
// run_done
// =============================================================================

void HwStartup::RunDone() noexcept {
  // The bring-up ends with the outputs off; the application releases them again.
  hardware::kPowerStage.Disable();
  cc_->ReleaseForceDuty();

  const float kGainA = (results.gain_a > 1e-4F) ? (1.0F / results.gain_a) : 1.0F;
  const float kGainB = (results.gain_b > 1e-4F) ? (1.0F / results.gain_b) : 1.0F;
  const float kGainVdc = (results.gain_vdc > 1e-4F) ? (1.0F / results.gain_vdc) : 1.0F;

  const settings::SettingsStatus kCommitStatus =
      settings_operations_.ApplyAdcCalibration(unit::Current{results.adc_offset_a},
                                               unit::Current{results.adc_offset_b},
                                               unit::Ratio{kGainA},
                                               unit::Ratio{kGainB},
                                               unit::Ratio{kGainVdc});
  if (kCommitStatus != settings::SettingsStatus::kSuccess) {
    EnterFault("failed to apply calibration settings");
    return;
  }

  state_ = FsmState::kDone;
  results.current_state = FsmState::kDone;

  LogSummary();
}

// =============================================================================
// log_summary
// =============================================================================

void HwStartup::LogSummary() noexcept {
  LogInfo(
      "\n[STARTUP] ====================================================\n"
      "[STARTUP] HARDWARE STARTUP AID - SUMMARY\n"
      "[STARTUP] ====================================================\n");

  for (uint8_t i = 0U; i < kNumStartupSteps; ++i) {
    const auto kState = static_cast<FsmState>(i);
    if (kState == FsmState::kIdle || kState == FsmState::kDone || kState == FsmState::kFault) {
      continue;
    }
    LogFormatted(hardware::LogLevel::kInfo,
                 "[STARTUP]   %s: %s\n",
                 StateName(kState),
                 results.passed[i] ? "PASS" : "FAIL/PENDING");
  }

  LogFormatted(hardware::LogLevel::kInfo,
               "[STARTUP] adc_offset_a  = %f A\n"
               "[STARTUP] adc_offset_b  = %f A\n"
               "[STARTUP] adc_noise_a   = %f A\n"
               "[STARTUP] adc_noise_b   = %f A\n"
               "[STARTUP] gain_vdc      = %f\n"
               "[STARTUP] gain_a        = %f\n"
               "[STARTUP] gain_b        = %f\n"
               "[STARTUP] adc_trig_opt  = %u ticks\n"
               "[STARTUP] NVM calibration fields updated.\n"
               "[STARTUP] ====================================================\n\n",
               static_cast<double>(results.adc_offset_a),
               static_cast<double>(results.adc_offset_b),
               static_cast<double>(results.adc_noise_rms_a),
               static_cast<double>(results.adc_noise_rms_b),
               static_cast<double>(results.gain_vdc),
               static_cast<double>(results.gain_a),
               static_cast<double>(results.gain_b),
               static_cast<unsigned int>(results.adc_trigger_offset_optimal));
}

// =============================================================================
// state_name
// =============================================================================

const char* HwStartup::StateName(FsmState state) noexcept {
  switch (state) {
    case FsmState::kIdle:
      return "IDLE";
    case FsmState::kPwmDisable:
      return "PWM_DISABLE";
    case FsmState::kAdcOffsetCal:
      return "ADC_OFFSET_CAL";
    case FsmState::kAdcNoiseFloor:
      return "ADC_NOISE_FLOOR";
    case FsmState::kDutyForceLow:
      return "DUTY_FORCE_LOW";
    case FsmState::kDutyForceMid:
      return "DUTY_FORCE_MID";
    case FsmState::kDutyForceHigh:
      return "DUTY_FORCE_HIGH";
    case FsmState::kDcLinkVoltageCheck:
      return "DC_LINK_VOLTAGE_CHECK";
    case FsmState::kGateDriverEnableCheck:
      return "GATE_DRIVER_ENABLE_CHECK";
    case FsmState::kConnectMotor:
      return "CONNECT_MOTOR";
    case FsmState::kPhaseAdcAlignment:
      return "PHASE_ADC_ALIGNMENT";
    case FsmState::kCurrentSenseCalibration:
      return "CURRENT_SENSE_CALIBRATION";
    case FsmState::kDone:
      return "DONE";
    case FsmState::kFault:
      return "FAULT";
    default:
      return "UNKNOWN";
  }
}

}  // namespace unimoc::startup
