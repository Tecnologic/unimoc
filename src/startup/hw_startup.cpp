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

/// Both switches of a leg for the same time: no voltage across the motor.
constexpr unit::Ratio kNeutralDuty = 0.5_ratio;

/// The log boundary: a `%f` argument of LogFormatted() in the base unit.
template <unit::UnitLike U>
[[nodiscard]] double LogValue(const U value) noexcept {
  return static_cast<double>(value.Value());
}

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

StartupStart HwStartup::RequestRun() noexcept {
  if (IsActive()) {
    return StartupStart::kRunning;  // a run is in progress; DONE and FAULT start a new one
  }
  state_ = FsmState::kIdle;
  results.current_state = FsmState::kIdle;

  // Reset all results and counters
  results = StartupResults{};
  sample_count_ = 0U;
  sweep_pos_ = 0U;
  step_done_ = false;
  next_step_requested_ = false;
  ext_current_ = unit::Current{};
  ext_vdc_ = unit::Voltage{};
  reading_prompted_ = false;
  completed_ = 0U;
  fault_ = StartupFault::kNone;
  oc_limit_ = settings_operations_.GetHardwareCapabilities().max_phase_current * oc_fraction;

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
      "[STARTUP] Continue each step with the commissioning\n"
      "[STARTUP]   command CONTINUE (0x0202)\n"
      "[STARTUP] ================================================\n");

  // Release the outputs at neutral duty. Without the external enable (jumper,
  // e-stop) this fails.
  // The run does not start in that case; the FSM stays IDLE so it can be requested again.
  if (!hardware::kPowerStage.Enable()) {
    cc_->ReleaseForceDuty();
    LogFormatted(hardware::LogLevel::kError,
                 "[STARTUP] Power stage not released (external enable missing or break active). Not "
                 "started.\n");
    return StartupStart::kNotReleased;
  }

  started_ = true;
  TransitionTo(FsmState::kPwmDisable);
  return StartupStart::kStarted;
}

// =============================================================================
// request_next_step
// =============================================================================

StartupContinue HwStartup::RequestNextStep(const bool motor_connected) noexcept {
  if (!IsActive()) {
    return StartupContinue::kNotRunning;
  }
  switch (Needs()) {
    case StartupNeeds::kDcLinkVoltage:
    case StartupNeeds::kPhaseCurrent:
      return StartupContinue::kNeedsReading;
    case StartupNeeds::kMotorConnected:
      if (!motor_connected) {
        return StartupContinue::kNeedsConfirmation;
      }
      break;
    case StartupNeeds::kNone:
      if (motor_connected) {
        return StartupContinue::kUnexpectedConfirmation;
      }
      break;
  }
  // A request before the step has measured would continue on its own later:
  // refused, so every continue is a decision on a measured step.
  if (!step_done_) {
    return StartupContinue::kMeasuring;
  }
  next_step_requested_ = true;
  return StartupContinue::kAccepted;
}

// =============================================================================
// status — for the commissioning status message
// =============================================================================

StartupNeeds HwStartup::Needs() const noexcept {
  if (!IsActive()) {
    return StartupNeeds::kNone;
  }
  switch (state_) {
    case FsmState::kDcLinkVoltageCheck:
      // Measured, the reading still missing (the step completes with it).
      return (!step_done_ && sample_count_ >= kNCal) ? StartupNeeds::kDcLinkVoltage : StartupNeeds::kNone;
    case FsmState::kCurrentSenseCalibration:
      return (!step_done_ && sample_count_ >= kNCal) ? StartupNeeds::kPhaseCurrent : StartupNeeds::kNone;
    case FsmState::kConnectMotor:
      return step_done_ ? StartupNeeds::kMotorConnected : StartupNeeds::kNone;
    default:
      return StartupNeeds::kNone;
  }
}

uint8_t HwStartup::Progress() const noexcept {
  if (!IsActive()) {
    return state_ == FsmState::kDone ? 100U : 0U;
  }
  if (step_done_) {
    return 100U;
  }
  uint32_t done = sample_count_;
  uint32_t total = 1U;
  switch (state_) {
    case FsmState::kAdcOffsetCal:
    case FsmState::kAdcNoiseFloor:
    case FsmState::kDcLinkVoltageCheck:
    case FsmState::kCurrentSenseCalibration:
      total = kNCal;
      break;
    case FsmState::kDutyForceLow:
    case FsmState::kDutyForceMid:
    case FsmState::kDutyForceHigh:
      total = kHoldSamples;
      break;
    case FsmState::kGateDriverEnableCheck:
      done = gate_phase_disabled_ ? sample_count_ : kNGate + sample_count_;
      total = 2U * kNGate;
      break;
    case FsmState::kPhaseAdcAlignment:
      done = (sweep_pos_ * kNAlignSamples) + sample_count_;
      total = kNSweepSteps * kNAlignSamples;
      break;
    default:
      break;
  }
  return static_cast<uint8_t>(std::min<uint32_t>(done, total) * 100U / total);
}

StartupStatus HwStartup::Status() const noexcept {
  StartupStatus status{};
  status.step = state_;
  status.active = IsActive();
  status.measured = status.active && step_done_;
  status.needs = Needs();
  status.progress = Progress();
  for (uint8_t i = 0U; i < kNumStartupSteps; ++i) {
    if (results.passed[i]) {
      status.passed |= 1U << i;
    }
  }
  uint32_t finished = completed_;
  if (status.measured) {
    finished |= 1U << static_cast<uint8_t>(state_);
  }
  status.failed = finished & ~status.passed;
  status.fault = fault_;
  status.started = started_;
  return status;
}

// =============================================================================
// request_abort
// =============================================================================

void HwStartup::RequestAbort(const StartupFault reason) noexcept {
  // No-op unless a run is active: in IDLE, DONE and FAULT the outputs are
  // already the drive's, and a second EnterFault() would force the duties
  // again with nobody left to release them.
  if (!IsActive()) {
    return;
  }
  EnterFault(reason == StartupFault::kEmergencyStop ? "emergency stop" : "user abort", reason);
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
  // CONNECT_MOTOR holds the outputs off while the motor is connected.
  if (IsActive() && state_ != FsmState::kConnectMotor &&
      hardware::kPowerStage.GetState() != hardware::PowerStageState::kEnabled) {
    const char* const kTrip = cc_->TripName();
    EnterFault(kTrip != nullptr ? kTrip : "power stage broken or blocked",
               kTrip != nullptr ? StartupFault::kTrip : StartupFault::kEnableLost);
    return;
  }

  // In every step: in phase 1 a current means a motor is still connected.
  if (IsActive()) {
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
            "[STARTUP] Verify gate-driver outputs are toggling, then continue.\n");
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
  accum_c_ = 0.0;
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
  completed_ |= 1U << static_cast<uint8_t>(state_);

  const auto kState = static_cast<uint8_t>(state_);
  const auto kNextRaw = static_cast<uint8_t>(kState + 1U);

  // The last measuring step ends in DONE: commit the calibration, outputs off.
  if (kNextRaw >= static_cast<uint8_t>(FsmState::kDone)) {
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
    if (ext_current_ < 1e-3_A) {
      LogWarning(
          "[STARTUP] CURRENT_SENSE_CALIBRATION: enter the clamp-meter reading (SET_READING current=) "
          "before continuing.\n");
      return false;
    }
  }
  if (state_ == FsmState::kDcLinkVoltageCheck) {
    if (ext_vdc_ < vdc_min_valid) {
      LogWarning(
          "[STARTUP] DC_LINK_VOLTAGE_CHECK: enter the multimeter reading (SET_READING vdc=) "
          "before continuing.\n");
      return false;
    }
  }
  return true;
}

// =============================================================================
// enter_fault
// =============================================================================

void HwStartup::EnterFault(const char* reason, const StartupFault fault) noexcept {
  fault_ = fault;
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
  if ((unit::Abs(phase_currents.a) > oc_limit_) || (unit::Abs(phase_currents.b) > oc_limit_) ||
      (unit::Abs(phase_currents.c) > oc_limit_)) {
    EnterFault("over-current detected", StartupFault::kOvercurrent);
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
  accum_c_ += static_cast<double>(cc_->state.raw_phase_currents.c.Value());
  accum_sq_a_ += kIa * kIa;
  accum_sq_b_ += kIb * kIb;
  ++sample_count_;
  return (sample_count_ >= n_samples);
}

// =============================================================================
// mean_current, mean_voltage, rms_current  — the accumulators back into units
// =============================================================================

unit::Current HwStartup::MeanCurrent(const double sum, const uint32_t n_samples) noexcept {
  return unit::Current{static_cast<float>(sum) / static_cast<float>(n_samples)};
}

unit::Voltage HwStartup::MeanVoltage(const double sum, const uint32_t n_samples) noexcept {
  return unit::Voltage{static_cast<float>(sum) / static_cast<float>(n_samples)};
}

unit::Current HwStartup::RmsCurrent(const double sum,
                                    const double sum_of_squares,
                                    const uint32_t n_samples) noexcept {
  const auto kSampleCount = static_cast<double>(n_samples);
  // variance = E[x²] - E[x]², in double (see the accumulators)
  const auto kVariance =
      static_cast<float>(sum_of_squares / kSampleCount - (sum / kSampleCount) * (sum / kSampleCount));
  return unit::Current{(kVariance > 0.0F) ? std::sqrt(kVariance) : 0.0F};
}

// =============================================================================
// run_adc_offset_cal
// =============================================================================

void HwStartup::RunAdcOffsetCal() noexcept {
  // Ensure neutral duty while collecting
  cc_->ForceDuty(kNeutralDuty, kNeutralDuty, kNeutralDuty);

  if (!step_done_) {
    if (CollectSample(kNCal)) {
      const unit::Current kMeanA = MeanCurrent(accum_a_, kNCal);
      const unit::Current kMeanB = MeanCurrent(accum_b_, kNCal);
      const unit::Current kMeanC = MeanCurrent(accum_c_, kNCal);

      results.adc_offset_a = kMeanA;
      results.adc_offset_b = kMeanB;
      results.adc_offset_c = kMeanC;

      const bool kPass = (unit::Abs(kMeanA) < offset_threshold) && (unit::Abs(kMeanB) < offset_threshold) &&
                         (unit::Abs(kMeanC) < offset_threshold);

      results.passed[static_cast<uint8_t>(FsmState::kAdcOffsetCal)] = kPass;

      LogFormatted(hardware::LogLevel::kInfo,
                   "[STARTUP] ADC_OFFSET_CAL: offset_a=%f A, offset_b=%f A, offset_c=%f A  %s\n",
                   LogValue(kMeanA),
                   LogValue(kMeanB),
                   LogValue(kMeanC),
                   kPass ? "PASS" : "FAIL");

      if (!kPass) {
        LogFormatted(hardware::LogLevel::kWarning,
                     "[STARTUP] ADC offset exceeds threshold (%f A). Check op-amp supply, resistors, and PCB "
                     "connections.\n",
                     LogValue(offset_threshold));
      }

      step_done_ = true;
      LogInfo("[STARTUP] Continue with the next step.\n");
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
      results.adc_noise_rms_a = RmsCurrent(accum_a_, accum_sq_a_, kNCal);
      results.adc_noise_rms_b = RmsCurrent(accum_b_, accum_sq_b_, kNCal);

      const bool kPass =
          (results.adc_noise_rms_a < noise_threshold) && (results.adc_noise_rms_b < noise_threshold);

      results.passed[static_cast<uint8_t>(FsmState::kAdcNoiseFloor)] = kPass;

      LogFormatted(hardware::LogLevel::kInfo,
                   "[STARTUP] ADC_NOISE_FLOOR: rms_a=%f A, rms_b=%f A  %s\n",
                   LogValue(results.adc_noise_rms_a),
                   LogValue(results.adc_noise_rms_b),
                   kPass ? "PASS" : "FAIL");

      if (!kPass) {
        LogFormatted(hardware::LogLevel::kWarning,
                     "[STARTUP] ADC noise exceeds threshold (%f A). Check decoupling caps, layout, and "
                     "ground paths.\n",
                     LogValue(noise_threshold));
      }

      step_done_ = true;
      LogInfo("[STARTUP] Continue with the next step.\n");
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
                 "[STARTUP] Continue with the next step.\n",
                 StateName(state_),
                 LogValue(duty),
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
  const unit::Voltage kMeasuredVdc = MeanVoltage(accum_a_, kNCal);

  if (ext_vdc_ > vdc_min_valid) {
    results.gain_vdc = kMeasuredVdc / ext_vdc_;
    const bool kPass = unit::Abs(results.gain_vdc - 1.0_ratio) < vdc_gain_tolerance;

    results.passed[static_cast<uint8_t>(FsmState::kDcLinkVoltageCheck)] = kPass;

    LogFormatted(hardware::LogLevel::kInfo,
                 "[STARTUP] DC_LINK_VOLTAGE_CHECK: adc_vdc=%f V, ext=%f V, gain=%f  %s\n",
                 LogValue(kMeasuredVdc),
                 LogValue(ext_vdc_),
                 LogValue(results.gain_vdc),
                 kPass ? "PASS" : "FAIL");

    if (!kPass) {
      LogFormatted(hardware::LogLevel::kWarning,
                   "[STARTUP] V_dc gain error exceeds %f %%. Check voltage-divider resistors on V_dc sense "
                   "circuit.\n"
                   "[STARTUP] Suggested correction factor: %f\n",
                   LogValue(vdc_gain_tolerance * 100.0F),
                   LogValue(results.gain_vdc));
    }

    step_done_ = true;
    LogInfo("[STARTUP] Continue with the next step.\n");
  } else if (sample_count_ == kNCal) {
    // Log the "waiting" prompt exactly once (bump sample_count_ as sentinel).
    LogFormatted(hardware::LogLevel::kInfo,
                 "[STARTUP] DC_LINK_VOLTAGE_CHECK: adc_vdc=%f V. Enter the multimeter reading "
                 "(SET_READING vdc=).\n",
                 LogValue(kMeasuredVdc));
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
      gate_mean_disabled_ = MeanCurrent(accum_a_, kNGate);
      // Reset accumulators for enabled phase
      sample_count_ = 0U;
      accum_a_ = 0.0;
      accum_b_ = 0.0;
      accum_c_ = 0.0;
      accum_c_ = 0.0;
      accum_c_ = 0.0;
      accum_sq_a_ = 0.0;
      accum_sq_b_ = 0.0;
      gate_phase_disabled_ = false;
      LogFormatted(hardware::LogLevel::kInfo,
                   "[STARTUP] GATE_DRIVER_ENABLE_CHECK: baseline mean_ia=%f A\n",
                   LogValue(gate_mean_disabled_));
    }
    return;  // continue next call
  }

  // Collecting "enabled" phase (small imbalance duty)
  cc_->ForceDuty(kGateTestDutyHigh, kGateTestDutyLow, kGateTestDutyLow);
  if (CollectSample(kNGate)) {
    gate_mean_enabled_ = MeanCurrent(accum_a_, kNGate);

    const unit::Current kDelta = unit::Abs(gate_mean_enabled_ - gate_mean_disabled_);
    const unit::Current kThreshold = 2.0F * results.adc_noise_rms_a;
    const bool kPass = (kDelta > kThreshold);

    results.passed[static_cast<uint8_t>(FsmState::kGateDriverEnableCheck)] = kPass;

    LogFormatted(hardware::LogLevel::kInfo,
                 "[STARTUP] GATE_DRIVER_ENABLE_CHECK: delta_ia=%f A, threshold=%f A  %s\n",
                 LogValue(kDelta),
                 LogValue(kThreshold),
                 kPass ? "PASS" : "FAIL");

    if (!kPass) {
      LogWarning(
          "[STARTUP] Gate-driver response smaller than 2x noise floor. Check gate-enable GPIO and driver "
          "power supply.\n");
    }

    // Reset flag for potential re-run
    gate_phase_disabled_ = true;
    step_done_ = true;
    LogInfo("[STARTUP] Continue with the next step.\n");
  }
}

// =============================================================================
// run_connect_motor_wait  —  barrier state; user must confirm motor is connected
// =============================================================================

void HwStartup::RunConnectMotorWait() noexcept {
  cc_->ForceDuty(kNeutralDuty, kNeutralDuty, kNeutralDuty);

  if (!step_done_) {
    // The motor is connected with the outputs off and the logic powered: a
    // power cycle would restart the drive and end the run.
    hardware::kPowerStage.Disable();
    oc_limit_ = settings_operations_.GetHardwareCapabilities().max_phase_current * oc_fraction;

    LogWarning(
        "\n"
        "[STARTUP] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
        "[STARTUP] !!  PHASE 1 COMPLETE - PREPARE FOR PHASE 2       !!\n"
        "[STARTUP] !!                                                !!\n"
        "[STARTUP] !!  The outputs are OFF now.                      !!\n"
        "[STARTUP] !!  ACTION REQUIRED:                              !!\n"
        "[STARTUP] !!    1. CONNECT the motor (UVW + PE). Keep the   !!\n"
        "[STARTUP] !!       logic powered: a power cycle ends the    !!\n"
        "[STARTUP] !!       run. Switch the DC link off only if the  !!\n"
        "[STARTUP] !!       logic has a supply of its own.           !!\n"
        "[STARTUP] !!    2. CONTINUE with motor-connected            !!\n"
        "[STARTUP] !!       ONLY after motor is securely connected.  !!\n"
        "[STARTUP] !!                                                !!\n"
        "[STARTUP] !!  CAUTION: Phase 2 injects LIVE voltages.       !!\n"
        "[STARTUP] !!  LOW-Rs WINDINGS: even tiny duty changes cause !!\n"
        "[STARTUP] !!  large currents. OC limit is set to            !!\n");
    LogFormatted(hardware::LogLevel::kWarning,
                 "[STARTUP] !!  %f A (%f%% of max).\n"
                 "[STARTUP] !!  Ensure motor shaft is FREE to rotate.        !!\n"
                 "[STARTUP] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n",
                 LogValue(oc_limit_),
                 LogValue(oc_fraction * 100.0F));

    results.passed[static_cast<uint8_t>(FsmState::kConnectMotor)] = true;
    step_done_ = true;
  }

  if (step_done_ && next_step_requested_ && ValidateCurrentState()) {
    // Release the outputs again at neutral duty for phase 2. The continue is
    // the acknowledgement of a break latched while the outputs were off (the
    // external enable pulled while the motor was connected), as RequestRun().
    if (hardware::kPowerStage.GetState() == hardware::PowerStageState::kFault) {
      hardware::kPowerStage.ClearFault();
    }
    if (!hardware::kPowerStage.Enable()) {
      EnterFault("power stage not released for phase 2 (external enable missing or break active)",
                 StartupFault::kEnableLost);
      return;
    }
    AdvanceToNextState();
  }
}

// =============================================================================
// inject_alpha_current  — a small voltage on the alpha axis (phase A out, B and C back)
// =============================================================================

void HwStartup::InjectAlphaCurrent() noexcept {
  const unit::Ratio kDa = kNeutralDuty + v_cal_fraction;
  const unit::Ratio kDb = kNeutralDuty - v_cal_fraction * 0.5F;
  const unit::Ratio kDc = kNeutralDuty - v_cal_fraction * 0.5F;
  cc_->ForceDuty(kDa, kDb, kDc);
}

// =============================================================================
// run_phase_adc_alignment
// =============================================================================

void HwStartup::RunPhaseAdcAlignment() noexcept {
  // A small current in phase A, the injection of CURRENT_SENSE_CALIBRATION:
  // a large asymmetric duty would drive the motor into the overcurrent limit.
  InjectAlphaCurrent();

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
    unit::Current best_noise = sweep_noise_a_[0U];
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
    const unit::Current kNoiseFloor = results.adc_noise_rms_a;
    const bool kPass = (best_noise < 2.0F * kNoiseFloor);
    results.passed[static_cast<uint8_t>(FsmState::kPhaseAdcAlignment)] = kPass;

    LogFormatted(hardware::LogLevel::kInfo,
                 "[STARTUP] PHASE_ADC_ALIGNMENT: best_pos=%u, optimal_offset=%u ticks, min_noise=%f A  %s\n",
                 static_cast<unsigned int>(best_pos),
                 static_cast<unsigned int>(kOptimalTicks),
                 LogValue(best_noise),
                 kPass ? "PASS" : "FAIL");

    if (!kPass) {
      LogWarning(
          "[STARTUP] Could not find a clean ADC sampling window. Check PWM frequency, ADC trigger timing, "
          "and hardware layout.\n");
    }

    LogFormatted(hardware::LogLevel::kInfo,
                 "[STARTUP] Suggest writing adc_trigger_offset=%u to NvmSettings to persist the optimal "
                 "offset.\n"
                 "[STARTUP] Continue with the next step.\n",
                 static_cast<unsigned int>(kOptimalTicks));

    cc_->SetAdcTriggerOffset(kOptimalTicks);
    step_done_ = true;
    return;
  }

  // Collecting kNAlignSamples at the current sweep position
  if (CollectSample(kNAlignSamples)) {
    sweep_noise_a_[sweep_pos_] = RmsCurrent(accum_a_, accum_sq_a_, kNAlignSamples);

    // Advance sweep
    ++sweep_pos_;
    sample_count_ = 0U;
    accum_a_ = 0.0;
    accum_b_ = 0.0;
    accum_c_ = 0.0;
    accum_c_ = 0.0;
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
  InjectAlphaCurrent();

  if (!step_done_) {
    if (sample_count_ < kNCal && !CollectSample(kNCal)) {
      return;  // still collecting
    }
    // Measured; the gains need the clamp-meter reading (SET_READING), which may come later.
    if (ext_current_ < 1e-3_A) {
      if (!reading_prompted_) {
        reading_prompted_ = true;
        LogInfo("[STARTUP] CURRENT_SENSE_CALIBRATION: measured. Enter the clamp-meter reading of phase A.\n");
      }
      return;
    }
    FinishCurrentSenseCalibration();
    step_done_ = true;
    LogInfo("[STARTUP] Continue with the next step.\n");
  }

  if (step_done_ && next_step_requested_ && ValidateCurrentState()) {
    AdvanceToNextState();
  }
}

void HwStartup::FinishCurrentSenseCalibration() noexcept {
  const unit::Current kMeanIa = MeanCurrent(accum_a_, kNCal);
  const unit::Current kMeanIb = MeanCurrent(accum_b_, kNCal);
  const unit::Current kMeanIc = MeanCurrent(accum_c_, kNCal);

  // gain = adc_reading / ext_reference; ideal = 1.0. The reading less the
  // zero offset of ADC_OFFSET_CAL: the raw samples do not carry the
  // calibration. Phases B and C each carry -I/2.
  const unit::Current kSignalA = kMeanIa - results.adc_offset_a;
  const unit::Current kSignalB = kMeanIb - results.adc_offset_b;
  const unit::Current kSignalC = kMeanIc - results.adc_offset_c;
  const unit::Current kHalfReturn = -ext_current_ * 0.5F;
  results.gain_a = (unit::Abs(kSignalA) > 1e-4_A) ? (kSignalA / ext_current_) : unit::Ratio{};
  results.gain_b = (unit::Abs(kSignalB) > 1e-4_A) ? (kSignalB / kHalfReturn) : unit::Ratio{};
  results.gain_c = (unit::Abs(kSignalC) > 1e-4_A) ? (kSignalC / kHalfReturn) : unit::Ratio{};

  const auto kWithin = [this](const unit::Ratio gain) {
    return gain > 1e-4_ratio && unit::Abs(gain - 1.0_ratio) < gain_tolerance;
  };
  const bool kPass = kWithin(results.gain_a) && kWithin(results.gain_b) && kWithin(results.gain_c);
  results.passed[static_cast<uint8_t>(FsmState::kCurrentSenseCalibration)] = kPass;

  LogFormatted(hardware::LogLevel::kInfo,
               "[STARTUP] CURRENT_SENSE_CALIBRATION: mean_ia=%f A, mean_ib=%f A, mean_ic=%f A, ext=%f A, "
               "gain_a=%f, gain_b=%f, gain_c=%f  %s\n",
               LogValue(kMeanIa),
               LogValue(kMeanIb),
               LogValue(kMeanIc),
               LogValue(ext_current_),
               LogValue(results.gain_a),
               LogValue(results.gain_b),
               LogValue(results.gain_c),
               kPass ? "PASS" : "FAIL");
  if (!kPass) {
    LogFormatted(hardware::LogLevel::kWarning,
                 "[STARTUP] Gain error exceeds %f %%. Check voltage-divider and op-amp gain resistors "
                 "for current-sense channels.\n"
                 "[STARTUP] Correction factors: adc_gain_a=%f, adc_gain_b=%f, adc_gain_c=%f\n",
                 LogValue(gain_tolerance * 100.0F),
                 LogValue(1.0_ratio / results.gain_a),
                 LogValue(1.0_ratio / results.gain_b),
                 LogValue(1.0_ratio / results.gain_c));
  }
}

// =============================================================================
// run_done
// =============================================================================

void HwStartup::RunDone() noexcept {
  // The bring-up ends with the outputs off; the application releases them again.
  hardware::kPowerStage.Disable();
  cc_->ReleaseForceDuty();

  // A gain that was not measured (0) keeps the correction at 1.
  const auto kCorrection = [](const unit::Ratio gain) {
    return (gain > 1e-4_ratio) ? (1.0_ratio / gain) : 1.0_ratio;
  };
  const settings::SettingsStatus kCommitStatus = settings_operations_.ApplyAdcCalibration(
      settings::AdcCalibration{.offset_a = results.adc_offset_a,
                               .offset_b = results.adc_offset_b,
                               .offset_c = results.adc_offset_c,
                               .gain_a = kCorrection(results.gain_a),
                               .gain_b = kCorrection(results.gain_b),
                               .gain_c = kCorrection(results.gain_c),
                               .gain_vdc = kCorrection(results.gain_vdc)});
  if (kCommitStatus != settings::SettingsStatus::kSuccess) {
    EnterFault("failed to apply calibration settings", StartupFault::kApplyFailed);
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
               "[STARTUP] adc_offset_c  = %f A\n"
               "[STARTUP] adc_noise_a   = %f A\n"
               "[STARTUP] adc_noise_b   = %f A\n"
               "[STARTUP] gain_vdc      = %f\n"
               "[STARTUP] gain_a        = %f\n"
               "[STARTUP] gain_b        = %f\n"
               "[STARTUP] gain_c        = %f\n"
               "[STARTUP] adc_trig_opt  = %u ticks\n"
               "[STARTUP] NVM calibration fields updated.\n"
               "[STARTUP] ====================================================\n\n",
               LogValue(results.adc_offset_a),
               LogValue(results.adc_offset_b),
               LogValue(results.adc_offset_c),
               LogValue(results.adc_noise_rms_a),
               LogValue(results.adc_noise_rms_b),
               LogValue(results.gain_vdc),
               LogValue(results.gain_a),
               LogValue(results.gain_b),
               LogValue(results.gain_c),
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
