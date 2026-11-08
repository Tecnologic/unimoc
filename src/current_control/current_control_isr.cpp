/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file current_control_isr.cpp
 *    @brief Current-control ISR implementation.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "current_control_isr.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include "hardware_interface.hpp"
#include "hot_path_interface.hpp"
#include "nvm_settings.hpp"
#include "sin_cos.hpp"
#include "three_phase_system.hpp"
#include "units.hpp"

namespace unimoc::current_control {

using namespace unit;

// =============================================================================
// init
// =============================================================================

bool CurrentControlIsr::Init(const settings::NvmSettings& settings,
                             const hardware::HardwareInterface& hardware,
                             const uint32_t timer_clock_hz) noexcept {
  // --- Timing: the PWM frequency and the divider for this board ---
  const settings::HardwareCapabilities& limits = hardware::kSettings.GetSettingsProfile().capabilities;
  const settings::ControlTiming kTiming = settings::ResolveControlTiming(settings, limits);
  if (!kTiming.Valid()) {
    hardware_ = nullptr;  // OnSample() does nothing
    return false;
  }
  hardware_ = &hardware;
  state.timing = kTiming;
  const unit::Frequency kSuggested = settings::SuggestedPwmFrequency(settings, limits);
  state.pwm_suggestion = PwmSuggestion{
      .pwm_frequency = kSuggested,
      .ripple = settings::PwmRipple(settings, kSuggested),
      .running_ripple = settings::PwmRipple(settings, kTiming.pwm_frequency),
  };

  // Centre-aligned PWM: ARR = timer_clock / (2 × f_pwm) − 1
  const auto kPwmHz = static_cast<uint32_t>(kTiming.pwm_frequency.Value());
  state.arr = timer_clock_hz / (2U * kPwmHz) - 1U;
  state.dt_fast = kTiming.StepPeriod();
  state.dt_slow = kTiming.CyclePeriod();
  state.output_delay_steps = kTiming.OutputDelaySteps();
  state.adc_trigger_offset = static_cast<uint32_t>(std::roundf(1e-6F * static_cast<float>(timer_clock_hz)));

  state.dc_link_trip = settings.dc_link_trip;
  state.calibration = settings::AdcCalibration{.offset_a = settings.adc_offset_a,
                                               .offset_b = settings.adc_offset_b,
                                               .offset_c = settings.adc_offset_c,
                                               .gain_a = settings.adc_gain_a,
                                               .gain_b = settings.adc_gain_b,
                                               .gain_c = settings.adc_gain_c,
                                               .gain_vdc = settings.adc_gain_vdc};
  // The hardware checks every uncalibrated DC-link sample: the trip level before the calibration.
  hardware.SetOvervoltageLimit(state.calibration.gain_vdc > Ratio{}
                                   ? state.dc_link_trip / state.calibration.gain_vdc
                                   : state.dc_link_trip);
  dtc.Init(settings);
  svm.Init(settings);
  // Low-side shunts: every phase keeps the low-side on time of a valid sample.
  svm.duty_max = Min(svm.duty_max, kTiming.duty_max);
  svm.duty_min = Min(svm.duty_min, svm.duty_max);

  // --- Current controller parameters ---
  cc.kp_d = settings.current_kp_d;
  cc.ki_d = settings.current_ki_d;
  cc.kp_q = settings.current_kp_q;
  cc.ki_q = settings.current_ki_q;
  gains_pending_.store(false, std::memory_order_relaxed);  // the settings are newer than any live gains
  cc.L_d = settings.l_d;
  cc.L_q = settings.l_q;
  cc.psi = settings.flux_pm;
  if (settings.motor_type == system::MotorType::kAsm && settings.asm_l_r > unit::Inductance{} &&
      settings.asm_l_s > unit::Inductance{}) {
    // In the rotor flux frame the stator sees its transient inductance sigma L_s on both axes and the
    // back-EMF of (L_m / L_r) psi_r, at the flux reference.
    const unit::Inductance kTransient =
        settings.asm_l_s - (settings.asm_l_m * (settings.asm_l_m / settings.asm_l_r));
    cc.L_d = kTransient;
    cc.L_q = kTransient;
    cc.psi = settings.asm_flux_psi_r * (settings.asm_l_m / settings.asm_l_r);
  } else if (settings.motor_type == system::MotorType::kEesm) {
    cc.psi = unit::MagneticFlux{};  // the field builds up after the engage: the integrators take its back-EMF
  }
  cc.v_max = settings.current_v_max;

  // --- Mechanical observer parameters ---
  mech_obs.Init(settings);

  // --- Zero angle, standing still, until the observer hands over a reference ---
  for (auto& reference : state.angles.buf) {
    reference = AngleReference{};
  }
  state.angles.active.store(0U, std::memory_order_release);
  angle_ = system::SinCos<unit::Ratio>{};
  speed_ = unit::AngularVelocity{};
  reference_sample_ = 0U;
  record_index_ = 0U;
  cycle_controlled_ = false;
  cycle_ended_ = false;

  // --- Reset algorithm state ---
  cc.Reset();
  state.samples_ready = false;
  state.observer_pending.store(false, std::memory_order_release);
  state.observer_overruns = 0U;
  state.samples = 0U;
  state.periods = 0U;
  state.control_cycles = 0U;
  state.phase_duties = system::ThreePhase<unit::Ratio>{0.5_ratio, 0.5_ratio, 0.5_ratio};
  // A restart leaves a bring-up mode: the control loop runs again, from zero
  // current until the outer loop hands over a reference.
  force_duty_active_.store(false, std::memory_order_release);
  holding_ = false;
  state.i_ref = system::Rotor<unit::Current>{0.0F, 0.0F};
  state.i_ref_mailbox.Write(state.i_ref);
  state.i_dq_last = system::Rotor<unit::Current>{0.0F, 0.0F};
  state.u_dq_last = system::Rotor<unit::Voltage>{0.0F, 0.0F};
  return true;
}

// =============================================================================
// on_sample — after every handed-over sample, a control step every CallsPerStep() calls, zero latency
// =============================================================================
//
// Must finish before the next peak: the duties go directly into the CCR
// preload registers, which the timer takes over there. It reaches the
// hardware only through hot_path:: (bound at link time, no function pointers)
// and calls no libm function (built with -fno-math-errno).
//
static bool Exceeds(const system::ThreePhase<unit::Current>& currents, const unit::Current limit) noexcept {
  return unit::Abs(currents.a) > limit || unit::Abs(currents.b) > limit || unit::Abs(currents.c) > limit;
}

/// The sample with the calibration of the startup aid: i = (i_raw - offset) * gain, v = v_raw * gain.
static hardware::CurrentSample Calibrated(const hardware::CurrentSample& raw,
                                          const settings::AdcCalibration& calibration) noexcept {
  hardware::CurrentSample sample = raw;
  sample.phase_currents.a = (raw.phase_currents.a - calibration.offset_a) * calibration.gain_a;
  sample.phase_currents.b = (raw.phase_currents.b - calibration.offset_b) * calibration.gain_b;
  sample.phase_currents.c = (raw.phase_currents.c - calibration.offset_c) * calibration.gain_c;
  sample.dc_link = raw.dc_link * calibration.gain_vdc;
  // The phase voltages use the divider and the reference of the DC link.
  sample.phase_voltages = {raw.phase_voltages.a * calibration.gain_vdc,
                           raw.phase_voltages.b * calibration.gain_vdc,
                           raw.phase_voltages.c * calibration.gain_vdc};
  return sample;
}

bool CurrentControlIsr::OnSample() noexcept {
  cycle_ended_ = false;
  if (hardware_ == nullptr) {
    return false;
  }
  // The samples between two control steps: the duties of the last step hold,
  // the platform keeps the trip flags of their samples for the next step.
  if (state.periods++ % state.timing.CallsPerStep() != 0U) {
    return false;
  }
  const uint32_t kSampleIndex = ++state.samples;
  const uint8_t kStepsPerCycle = state.timing.steps_per_cycle;
  const auto kStep = static_cast<uint8_t>((kSampleIndex - 1U) % kStepsPerCycle);
  const bool kCycleEnd = kStep == kStepsPerCycle - 1U;
  cycle_ended_ = kCycleEnd;

  // -------------------------------------------------------------------------
  // 1. The sample, raw and calibrated — always, regardless of control mode.
  //    The startup FSM reads the raw one from a lower-priority context.
  // -------------------------------------------------------------------------
  const hardware::CurrentSample kRaw = hardware::hot_path::GetSample();
  const hardware::CurrentSample kSample = Calibrated(kRaw, state.calibration);
  state.raw_phase_currents = kRaw.phase_currents;
  state.raw_vdc = kRaw.dc_link;
  state.phase_currents = kSample.phase_currents;
  state.dc_link = kSample.dc_link;
  AdvanceAngle(kSampleIndex);

  // The identification's drive holds for a whole cycle (the observer
  // demodulates the cycle with it); the reference and the gains every step.
  CycleRecord& record = state.records[record_index_];
  if (kStep == 0U) {
    drive_active_ = identification_active_.load(std::memory_order_acquire);
    if (drive_active_) {
      drive_ = identification_.Read();
    }
    cycle_controlled_ = true;
    i_dq_sum_ = system::Rotor<unit::Current>{0.0F, 0.0F};
    u_dq_sum_ = system::Rotor<unit::Voltage>{0.0F, 0.0F};
    record.first_sample = kSampleIndex;
    record.injection_amplitude = drive_active_ ? drive_.injection_amplitude : unit::Voltage{};
    record.injection_axis = drive_active_ ? static_cast<uint8_t>(drive_.injection_axis) : 0U;
  }
  const IdentificationDrive* const kDrive = drive_active_ ? &drive_ : nullptr;
  state.i_ref = (kDrive != nullptr) ? kDrive->current_ref : state.i_ref_mailbox.Read();
  if (gains_pending_.load(std::memory_order_acquire)) {
    gains_pending_.store(false, std::memory_order_relaxed);
    const CurrentGains kGains = gains_.Read();
    cc.kp_d = kGains.kp_d;
    cc.ki_d = kGains.ki_d;
    cc.kp_q = kGains.kp_q;
    cc.ki_q = kGains.ki_q;
  }

  // -------------------------------------------------------------------------
  // 2. Clarke and Park with the angle of the sample, and the phase voltages
  //    (the back-EMF while the outputs are open, which the observer tracks).
  // -------------------------------------------------------------------------
  const system::Stator<unit::Current> kCurrentAlphaBeta = kSample.phase_currents.ToStator();
  const system::Rotor<unit::Current> kCurrentDq = kCurrentAlphaBeta.ToRotor(angle_);
  record.sc[kStep] = angle_;
  record.i_ab_samples[kStep] = kCurrentAlphaBeta;
  record.u_ab_samples[kStep] = kSample.phase_voltages.ToStator();
  record.phase_voltages_valid = kSample.phase_voltages_valid;
  i_dq_sum_.d += kCurrentDq.d;
  i_dq_sum_.q += kCurrentDq.q;
  state.i_dq_last = kCurrentDq;

  // -------------------------------------------------------------------------
  // 3. Hard overcurrent limit. The hardware has checked the sample and
  //    tripped the stage already; this repeats the check on the calibrated
  //    currents. The loop stays frozen at neutral duties until the trip is
  //    acknowledged. DC-link overvoltage: no braking current can be taken any
  //    more; the outer loop has derated regenerative braking before this level.
  // -------------------------------------------------------------------------
  if (!state.overcurrent_tripped &&
      (kSample.overcurrent || Exceeds(kSample.phase_currents, state.overcurrent_limit))) {
    hardware::hot_path::TripPowerStage();
    state.overcurrent_tripped = true;
  }
  if (!state.overvoltage_tripped && (kSample.overvoltage || !(kSample.dc_link < state.dc_link_trip))) {
    hardware::hot_path::TripPowerStage();
    state.overvoltage_tripped = true;
  }
  if (state.overcurrent_tripped || state.overvoltage_tripped) {
    // The outputs are off: no voltage demand, the observer keeps tracking the
    // coasting motor from the phase voltages of the record.
    SetDuties(system::ThreePhase<unit::Ratio>{0.5_ratio, 0.5_ratio, 0.5_ratio});
    state.u_dq_last = system::Rotor<unit::Voltage>{0.0F, 0.0F};
    return kCycleEnd && FinishCycle(record, kSample);
  }

  // Force-duty mode: fixed duties, no control loop.
  if (force_duty_active_.load(std::memory_order_acquire)) {
    SetDuties(forced_duties_.Read());
    return FinishStepWithoutControl(kCycleEnd);
  }

  // -------------------------------------------------------------------------
  // 4. Current PI with decoupling feedforward over one control step; frozen
  //    (its last output held) when the identification asks for it.
  // -------------------------------------------------------------------------
  const system::Rotor<unit::Voltage> kVoltageDq = ControlVoltage(kDrive, kCurrentDq, kSample.dc_link);
  u_dq_sum_.d += kVoltageDq.d;
  u_dq_sum_.q += kVoltageDq.q;
  state.u_dq_last = kVoltageDq;

  // -------------------------------------------------------------------------
  // 5. Duties from the next peak on: inverse Park with the angle at the middle
  //    of the interval they apply to, dead-time compensation from this
  //    sample's current, SVM.
  // -------------------------------------------------------------------------
  const AngleReference& reference = state.angles.buf[state.angles.active.load(std::memory_order_acquire)];
  const system::SinCos<unit::Ratio> kAngleOut = system::Rotated(angle_, reference.out);
  const system::Stator<unit::Voltage> kVoltageAlphaBeta =
      WithInjection(kVoltageDq, kDrive, kSampleIndex, state.timing.divider).ToStator(kAngleOut);
  SetDuties(svm.CalculateWithDeadTimeCompensation(kVoltageAlphaBeta,
                                                  kSample.dc_link,
                                                  DeadTimeRatio(kDrive, kCurrentAlphaBeta)));

  return kCycleEnd && FinishCycle(record, kSample);
}

void CurrentControlIsr::AdvanceAngle(const uint32_t sample) noexcept {
  const AngleReference& reference = state.angles.buf[state.angles.active.load(std::memory_order_acquire)];
  if (reference.sample != reference_sample_ && sample >= reference.sample) {
    // A new reference: normally of this very sample, later when the observer was late.
    reference_sample_ = reference.sample;
    angle_ = reference.sc;
    speed_ = reference.speed;
    for (uint32_t k = reference.sample; k < sample && k < reference.sample + state.timing.steps_per_cycle;
         ++k) {
      angle_ = system::Rotated(angle_, reference.step);
    }
  } else {
    angle_ = system::Rotated(angle_, reference.step);
  }
  angle_ = system::Renormalized(angle_);
}

bool CurrentControlIsr::FinishStepWithoutControl(const bool cycle_end) noexcept {
  cycle_controlled_ = false;
  if (cycle_end) {
    ++state.control_cycles;
    state.samples_ready = true;
  }
  return false;  // the observer runs only on cycles of the control loop
}

bool CurrentControlIsr::FinishCycle(CycleRecord& record, const hardware::CurrentSample& sample) noexcept {
  ++state.control_cycles;
  state.samples_ready = true;
  if (!cycle_controlled_) {
    return false;  // the cycle started without the control loop
  }
  const float kInverseCount = 1.0F / static_cast<float>(state.timing.steps_per_cycle);
  record.i_dq_mean = system::Rotor<unit::Current>{i_dq_sum_.d * kInverseCount, i_dq_sum_.q * kInverseCount};
  record.u_dq = system::Rotor<unit::Voltage>{u_dq_sum_.d * kInverseCount, u_dq_sum_.q * kInverseCount};
  record.timestamp = sample.timestamp;
  record.dc_link = sample.dc_link;
  if (state.observer_pending.load(std::memory_order_acquire)) {
    ++state.observer_overruns;  // the observer has not finished the previous cycle
  }
  state.observer_buffer = record_index_;
  state.observer_pending.store(true, std::memory_order_release);
  record_index_ = static_cast<uint8_t>(1U - record_index_);
  return true;
}

// =============================================================================
// force_duty
// =============================================================================

void CurrentControlIsr::ForceDuty(const unit::Ratio da, const unit::Ratio db, const unit::Ratio dc) noexcept {
  const auto kClampDuty = [this](const unit::Ratio duty) noexcept {
    return duty.Clamp(svm.duty_min, svm.duty_max);
  };
  forced_duties_.Write(system::ThreePhase<unit::Ratio>{kClampDuty(da), kClampDuty(db), kClampDuty(dc)});
  force_duty_active_.store(true, std::memory_order_release);
}

// =============================================================================
// release_force_duty
// =============================================================================

void CurrentControlIsr::ReleaseForceDuty() noexcept {
  // Clear the integrators while the control cycle still bypasses the PI
  // loop, then hand the loop back (release: the reset is seen first).
  cc.Reset();
  force_duty_active_.store(false, std::memory_order_release);
}

// =============================================================================
// set_adc_trigger_offset / set_overcurrent_limit
// =============================================================================

void CurrentControlIsr::SetAdcTriggerOffset(uint32_t offset) noexcept {
  state.adc_trigger_offset = offset;
  if (hardware_ != nullptr) {
    hardware_->SetAdcTriggerOffset(offset);
  }
}

void CurrentControlIsr::SetOvercurrentLimit(const unit::Current limit) noexcept {
  state.overcurrent_limit = limit;
  if (hardware_ != nullptr) {
    hardware_->SetOvercurrentLimit(limit);
  }
}

// =============================================================================
// duty output helpers
// =============================================================================

void CurrentControlIsr::SetDuties(const system::ThreePhase<unit::Ratio>& duties) noexcept {
  state.phase_duties = duties;
  hardware::hot_path::SetDuties(duties);
}

// =============================================================================
// motor identification helpers of the control cycle
// =============================================================================

system::Rotor<unit::Voltage> CurrentControlIsr::ControlVoltage(const IdentificationDrive* const drive,
                                                               const system::Rotor<unit::Current>& i_dq,
                                                               const unit::Voltage dc_link) noexcept {
  if (drive != nullptr && drive->hold_voltage) {
    if (!holding_) {
      held_voltage_ = state.u_dq_last;  // the PI's last output; its integrators keep their state
      holding_ = true;
    }
    return held_voltage_;
  }
  holding_ = false;
  // The speed of the angle reference: the forced speed during the
  // identification (the observer's estimate is not the frame's speed then).
  return cc.Update(state.i_ref, i_dq, speed_, state.dt_fast, dc_link);
}

system::Stator<unit::Ratio> CurrentControlIsr::DeadTimeRatio(
    const IdentificationDrive* const drive, const system::Stator<unit::Current>& i_ab) const noexcept {
  if (drive != nullptr && !drive->dead_time_compensation) {
    return system::Stator<unit::Ratio>{0.0F, 0.0F};
  }
  return dtc.Calculate(i_ab);
}

system::Rotor<unit::Voltage> CurrentControlIsr::WithInjection(system::Rotor<unit::Voltage> u_dq,
                                                              const IdentificationDrive* const drive,
                                                              const uint32_t sample,
                                                              const uint8_t divider) noexcept {
  if (drive == nullptr || drive->injection_amplitude == unit::Voltage{}) {
    return u_dq;
  }
  // The sine of M2 at the middle of the interval of the duties (no libm: the table).
  const unit::Voltage kSine =
      drive->injection_amplitude * InjectionSinCos(InjectionEntryPhase(sample, divider)).sin;
  if (drive->injection_axis == InjectionAxis::kD) {
    u_dq.d += kSine;
  } else {
    u_dq.q += kSine;
  }
  return u_dq;
}

}  // namespace unimoc::current_control
