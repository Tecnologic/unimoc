/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file motor_identification.cpp
 *    @brief Motor identification FSM: stator resistance, dead time, L_d and L_q.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "motor_identification.hpp"
#include <algorithm>
#include "power_stage_interface.hpp"

namespace unimoc::identification {

using namespace unit;
using current_control::IdentificationDrive;
using current_control::IdentificationWindowResult;
using current_control::InjectionAngularFrequency;
using current_control::InjectionAxis;

namespace {

constexpr uint32_t kLevelsPerAngle = 2U * MotorIdentification::kResistanceLevels.size();
constexpr uint32_t kResistancePoints = kLevelsPerAngle * MotorIdentification::kResistanceAngles.size();

/// Progress of an M2 axis in its phases [%].
template <typename P>
constexpr uint8_t InductanceProgress(const P phase) noexcept {
  switch (phase) {
    case P::kBias:
      return 0U;
    case P::kSettle:
      return 33U;
    default:
      return 66U;
  }
}

constexpr uint32_t Bit(const Step step) noexcept { return 1U << static_cast<uint8_t>(step); }

/// |a - b| <= tolerance * |b|.
template <typename T>
constexpr bool Within(const T a, const T b, const float tolerance) noexcept {
  return Abs(a - b) <= Abs(b) * tolerance;
}

}  // namespace

void MotorIdentification::Configure(const settings::NvmSettings& settings) noexcept {
  if (IsActive()) {
    return;
  }
  current_max_ = settings.motor_i_max;
  resistance_prior_ = settings.stator_r;
  inductance_prior_ = Max(settings.l_d, settings.l_q);
  pwm_frequency_ = settings.pwm_frequency;
}

StartResult MotorIdentification::Start(const uint8_t measurements) noexcept {
  if (IsActive()) {
    return StartResult::kRunning;
  }
  const uint8_t kRequested = measurements & (kMeasureResistance | kMeasureInductance);
  if (kRequested == 0U) {
    return StartResult::kNoMeasurement;
  }
  // A new run is the acknowledgement of a latched trip (as the startup aid).
  hardware::kPowerStage.ClearFault();
  isr_->ClearTrips();
  isr_->ReleaseForceDuty();
  Drive(Current{}, Angle{}, false, Voltage{}, 0U);
  if (!hardware::kPowerStage.Enable()) {
    isr_->ClearIdentificationDrive();
    slow_->ReleaseForcedAngle();
    return StartResult::kNotReleased;
  }

  requested_ = kRequested;
  started_ = true;
  fault_ = Fault::kNone;
  passed_ = 0U;
  failed_ = 0U;
  results_ = Results{};
  all_points_.Reset();
  for (auto& regression : per_angle_) {
    regression.Reset();
  }
  step_ = Step::kIdle;
  Next();
  return StartResult::kStarted;
}

void MotorIdentification::Abort(const Fault reason) noexcept {
  if (IsActive()) {
    Finish(Step::kFault, reason);
  }
}

void MotorIdentification::Finish(const Step step, const Fault fault) noexcept {
  hardware::kPowerStage.Disable();
  isr_->ClearIdentificationDrive();
  slow_->ReleaseForcedAngle();
  slow_->CancelWindow();
  step_ = step;
  fault_ = fault;
}

void MotorIdentification::Drive(const Current d_current,
                                const Angle angle,
                                const bool hold,
                                const Voltage amplitude,
                                const uint8_t axis) noexcept {
  IdentificationDrive drive{};
  drive.current_ref = system::Rotor<Current>{d_current, Current{}};
  drive.hold_voltage = hold;
  // The dead-time voltage is what M1 measures; with the DC bias of M2 it is a constant.
  drive.dead_time_compensation = false;
  drive.injection_axis = axis == 0U ? InjectionAxis::kD : InjectionAxis::kQ;
  drive.injection_amplitude = amplitude;
  isr_->SetIdentificationDrive(drive);
  slow_->SetForcedAngle(angle, AngularVelocity{});
}

Current MotorIdentification::Level(const uint32_t point) const noexcept {
  const uint32_t kIndex = point % kLevelsPerAngle;
  const float kFraction = kResistanceLevels[kIndex % kResistanceLevels.size()];
  const float kSign = kIndex < kResistanceLevels.size() ? 1.0F : -1.0F;
  return current_max_ * (kFraction * kSign);
}

uint32_t MotorIdentification::SettleCycles() const noexcept {
  // Five time constants L / R of the present settings.
  const Time kSettle = (inductance_prior_ / resistance_prior_) * 5.0F;
  const Time kCycle = isr_->state.dt_slow;
  if (!(kSettle > Time{}) || !(kCycle > Time{})) {
    return kMinSettleCycles;
  }
  const float kCycles = (kSettle / kCycle).Value();  // a count of control cycles
  if (!(kCycles < static_cast<float>(kMaxSettleCycles))) {
    return kMaxSettleCycles;
  }
  return std::max(kMinSettleCycles, static_cast<uint32_t>(kCycles));
}

void MotorIdentification::Next() noexcept {
  switch (step_) {
    case Step::kIdle:
    case Step::kDone:
    case Step::kFault:
      if ((requested_ & kMeasureResistance) != 0U) {
        step_ = Step::kResistance;
        point_ = 0U;
        StartResistancePoint();
        return;
      }
      StartInductanceAxis(Step::kInductanceD);
      return;
    case Step::kResistance:
      FinishResistance();
      if ((requested_ & kMeasureInductance) != 0U) {
        StartInductanceAxis(Step::kInductanceD);
        return;
      }
      Finish(Step::kDone, Fault::kNone);
      return;
    case Step::kInductanceD:
      StartInductanceAxis(Step::kInductanceQ);
      return;
    case Step::kInductanceQ:
      FinishInductance();
      Finish(Step::kDone, Fault::kNone);
      return;
  }
}

// ----- M1 --------------------------------------------------------------------

void MotorIdentification::StartResistancePoint() noexcept {
  const uint32_t kAngle = point_ / kLevelsPerAngle;
  Drive(Level(point_), 1.0_deg * kResistanceAngles[kAngle], false, Voltage{}, 0U);
  phase_ = Phase::kSettle;
  slow_->RequestWindow(SettleCycles());
}

void MotorIdentification::OnResistanceWindow(const IdentificationWindowResult& window) noexcept {
  const uint32_t kAngle = point_ / kLevelsPerAngle;
  per_angle_[kAngle].Add(window.current_mean.d, window.voltage_mean.d);
  all_points_.Add(window.current_mean.d, window.voltage_mean.d);
  dc_link_ = window.dc_link_mean;
  ++point_;
  if (point_ < kResistancePoints) {
    StartResistancePoint();
    return;
  }
  Next();
}

void MotorIdentification::FinishResistance() noexcept {
  const auto kFit = all_points_.Solve();
  if (!kFit.has_value() || !(kFit->resistance > Resistance{})) {
    failed_ |= Bit(Step::kResistance);
    return;
  }
  results_.resistance_valid = true;
  results_.resistance = kFit->resistance;
  results_.dead_time_voltage = kFit->dead_time_voltage;
  // The dead-time compensation adds sign(i) t_dead f_pwm per phase (Clarke,
  // amplitude invariant); with the current along one phase (the angles of M1)
  // that is 4/3 t_dead f_pwm V_dc on the d axis.
  if (dc_link_ > Voltage{} && pwm_frequency_ > Frequency{}) {
    results_.dead_time = Max(Time{}, kFit->dead_time_voltage * 0.75F / (dc_link_ * pwm_frequency_));
  }
  bool agree = true;
  for (const auto& regression : per_angle_) {
    const auto kAngleFit = regression.Solve();
    agree =
        agree && kAngleFit.has_value() && Within(kAngleFit->resistance, kFit->resistance, kSlopeTolerance);
  }
  if (agree) {
    passed_ |= Bit(Step::kResistance);
  } else {
    failed_ |= Bit(Step::kResistance);  // a bad connection or a broken strand
  }
}

// ----- M2 --------------------------------------------------------------------

void MotorIdentification::StartInductanceAxis(const Step step) noexcept {
  step_ = step;
  point_ = 0U;
  // The bias first, with the current PI; then it is held and the sine added.
  const Current kTarget = current_max_ * kInjectionCurrent;
  const Resistance kReactance = (InjectionAngularFrequency(pwm_frequency_) / 1.0_rad) * inductance_prior_;
  amplitude_ = kTarget * (resistance_prior_ + kReactance);
  Drive(current_max_ * kInductanceBias, Angle{}, false, Voltage{}, 0U);
  phase_ = Phase::kBias;
  slow_->RequestWindow(SettleCycles());
}

void MotorIdentification::OnInductanceWindow(const IdentificationWindowResult& window) noexcept {
  const Current kTarget = current_max_ * kInjectionCurrent;
  const Current kAmplitude = Sqrt(window.current_phasor.MagnitudeSquared());
  ++point_;
  if (!Within(kAmplitude, kTarget, kInjectionTolerance) && point_ < kMaxInjectionTries &&
      kAmplitude > Current{}) {
    // Scale the voltage to the target current and measure again.
    amplitude_ = amplitude_ * (kTarget / kAmplitude);
    Drive(current_max_ * kInductanceBias, Angle{}, true, amplitude_, step_ == Step::kInductanceD ? 0U : 1U);
    phase_ = Phase::kSettle;
    slow_->RequestWindow(SettleCycles());
    return;
  }
  const auto kImpedance = EstimateImpedance(window.voltage_phasor,
                                            window.current_phasor,
                                            InjectionAngularFrequency(pwm_frequency_));
  if (!kImpedance.has_value() || !(kImpedance->inductance > Inductance{})) {
    failed_ |= Bit(step_);
  } else if (step_ == Step::kInductanceD) {
    results_.l_d = kImpedance->inductance;
    results_.resistance_ac = kImpedance->resistance;
    passed_ |= Bit(step_);
  } else {
    results_.l_q = kImpedance->inductance;
    results_.resistance_ac = (results_.resistance_ac + kImpedance->resistance) * 0.5F;
    passed_ |= Bit(step_);
  }
  // Back to the PI for the bias of the next axis.
  Drive(current_max_ * kInductanceBias, Angle{}, false, Voltage{}, 0U);
  Next();
}

void MotorIdentification::FinishInductance() noexcept {
  const uint32_t kBoth = Bit(Step::kInductanceD) | Bit(Step::kInductanceQ);
  if ((passed_ & kBoth) != kBoth) {
    return;  // the failed axis carries its bit
  }
  results_.inductance_valid = true;
  results_.l_mean = (results_.l_d + results_.l_q) * 0.5F;
  // R from the impedance and from M1 measure the same winding.
  if (results_.resistance_valid &&
      !Within(results_.resistance_ac, results_.resistance, kAcResistanceTolerance)) {
    passed_ &= ~kBoth;
    failed_ |= kBoth;
  }
}

// ----- the run ---------------------------------------------------------------

void MotorIdentification::RunOnce() noexcept {
  if (!IsActive()) {
    return;
  }
  if (hardware::kPowerStage.GetState() != hardware::PowerStageState::kEnabled) {
    Fault fault = Fault::kEnableLost;
    if (isr_->state.overcurrent_tripped) {
      fault = Fault::kOvercurrent;
    } else if (isr_->Tripped()) {
      fault = Fault::kTrip;
    }
    Finish(Step::kFault, fault);
    return;
  }
  IdentificationWindowResult window{};
  if (!slow_->TakeWindow(window)) {
    return;
  }
  switch (phase_) {
    case Phase::kBias:
      // The bias stands: hold the PI's voltage and add the sine.
      Drive(current_max_ * kInductanceBias, Angle{}, true, amplitude_, step_ == Step::kInductanceD ? 0U : 1U);
      phase_ = Phase::kSettle;
      slow_->RequestWindow(SettleCycles());
      return;
    case Phase::kSettle:
      phase_ = Phase::kMeasure;
      slow_->RequestWindow(kWindowCycles);
      return;
    case Phase::kMeasure:
      if (step_ == Step::kResistance) {
        OnResistanceWindow(window);
      } else {
        OnInductanceWindow(window);
      }
      return;
  }
}

Status MotorIdentification::GetStatus() const noexcept {
  Status status{};
  status.step = step_;
  status.active = IsActive();
  status.passed = passed_;
  status.failed = failed_;
  status.fault = fault_;
  status.counter = point_;
  if (step_ == Step::kResistance) {
    status.progress = static_cast<uint8_t>((100U * point_) / kResistancePoints);
  } else if (step_ == Step::kInductanceD || step_ == Step::kInductanceQ) {
    status.progress = InductanceProgress(phase_);
  } else if (step_ == Step::kDone) {
    status.progress = 100U;
  }
  return status;
}

}  // namespace unimoc::identification
