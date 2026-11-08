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
#include <numbers>
#include "power_stage_interface.hpp"

namespace unimoc::identification {

using namespace unit;
using current_control::IdentificationDrive;
using current_control::IdentificationWindowResult;
using current_control::InjectionAngularFrequency;
using current_control::InjectionAxis;

/// Below the minimum back-EMF this long the coasting motor stands [control cycles].
constexpr uint32_t kStandstillCycles = 1600U;

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

/// Progress of M3 in its phases [%].
template <typename P>
constexpr uint8_t FluxLinkageProgress(const P phase) noexcept {
  switch (phase) {
    case P::kAlign:
      return 10U;
    case P::kRamp:
      return 30U;
    case P::kHold:
      return 50U;
    case P::kCoast:
      return 75U;
    default:
      return 0U;
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
  speed_limit_ = settings.motor_omega_max;
  resistance_prior_ = settings.stator_r;
  inductance_prior_ = Max(settings.l_d, settings.l_q);
}

StartResult MotorIdentification::Start(const uint8_t measurements) noexcept {
  if (IsActive()) {
    return StartResult::kRunning;
  }
  const uint8_t kRequested = measurements & (kMeasureResistance | kMeasureInductance | kMeasureFluxLinkage);
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
  slow_->StopCoast();
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

uint32_t MotorIdentification::WindowCycles() const noexcept {
  return kWindowPeriods * current_control::InjectionPeriodCycles(isr_->state.timing.divider,
                                                                 isr_->state.timing.steps_per_cycle);
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
      if ((requested_ & kMeasureInductance) != 0U) {
        StartInductanceAxis(Step::kInductanceD);
        return;
      }
      StartFluxLinkage();
      return;
    case Step::kResistance:
      FinishResistance();
      if ((requested_ & kMeasureInductance) != 0U) {
        StartInductanceAxis(Step::kInductanceD);
        return;
      }
      break;
    case Step::kInductanceD:
      StartInductanceAxis(Step::kInductanceQ);
      return;
    case Step::kInductanceQ:
      FinishInductance();
      break;
    case Step::kFluxLinkage:
      Finish(Step::kDone, Fault::kNone);
      return;
  }
  // The standstill measurements ended: the coast-down needs the free shaft.
  if ((requested_ & kMeasureFluxLinkage) != 0U) {
    StartFluxLinkage();
    return;
  }
  Finish(Step::kDone, Fault::kNone);
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
  const Frequency kPwmFrequency = isr_->state.timing.pwm_frequency;
  if (dc_link_ > Voltage{} && kPwmFrequency > Frequency{}) {
    results_.dead_time = Max(Time{}, kFit->dead_time_voltage * 0.75F / (dc_link_ * kPwmFrequency));
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
  const Resistance kReactance =
      (InjectionAngularFrequency(isr_->state.timing.pwm_frequency) / 1.0_rad) * inductance_prior_;
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
                                            InjectionAngularFrequency(isr_->state.timing.pwm_frequency));
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

// ----- M3 --------------------------------------------------------------------

void MotorIdentification::StartFluxLinkage() noexcept {
  step_ = Step::kFluxLinkage;
  point_ = 0U;
  phase_ = Phase::kWaitShaftFree;
  // The outputs stay on at zero current until the operator confirms the free shaft.
  slow_->CancelWindow();
  Drive(Current{}, Angle{}, false, Voltage{}, 0U);
}

ConfirmResult MotorIdentification::Confirm(const bool shaft_free) noexcept {
  if (!IsActive() || step_ != Step::kFluxLinkage || phase_ != Phase::kWaitShaftFree) {
    return ConfirmResult::kNotWaiting;
  }
  if (!shaft_free) {
    return ConfirmResult::kWrongConfirmation;
  }
  DriveSpin(Angle{}, AngularAcceleration{});
  phase_ = Phase::kAlign;
  slow_->RequestWindow(kAlignCycles);
  return ConfirmResult::kAccepted;
}

void MotorIdentification::DriveSpin(const Angle angle, const AngularAcceleration acceleration) noexcept {
  IdentificationDrive drive{};
  drive.current_ref = system::Rotor<Current>{Current{}, current_max_ * kSpinUpCurrent};
  drive.dead_time_compensation = true;
  isr_->SetIdentificationDrive(drive);
  slow_->SetForcedAngle(angle, AngularVelocity{}, acceleration);
}

void MotorIdentification::RunFluxLinkage() noexcept {
  if (phase_ != Phase::kCoast && hardware::kPowerStage.GetState() != hardware::PowerStageState::kEnabled) {
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
  switch (phase_) {
    case Phase::kAlign:
      if (slow_->TakeWindow(window)) {
        // The rotor stands at the current: the forced angle ramps up from there.
        DriveSpin(Angle{}, (speed_limit_ * kSpinUpSpeedFraction) / (1.0_s * kSpinUpRampSeconds));
        spin_up_hits_ = 0U;
        phase_ = Phase::kRamp;
        slow_->RequestWindow(kRampWindowCycles);
      }
      return;
    case Phase::kRamp:
      if (slow_->TakeWindow(window)) {
        OnSpinUpWindow(window);
      }
      return;
    case Phase::kHold:
      if (slow_->TakeWindow(window)) {
        StartCoast();
      }
      return;
    case Phase::kCoast: {
      const bool kTimedOut = slow_->TakeWindow(window);
      const current_control::SlowUpdate::CoastStatus kCoast = slow_->GetCoast();
      if (kCoast.generation != coast_generation_ && !kTimedOut) {
        return;  // the observer has not started this coast's meter yet
      }
      const bool kSlow = kCoast.result.periods > 0U &&
                         Abs(kCoast.result.last_speed) <
                             Abs(kCoast.result.start_speed) * CoastDownMeter{}.min_speed_fraction;
      // A free coast does not reverse: a period the other way round means a load drives the shaft.
      const bool kReversed =
          kCoast.result.periods > 0U &&
          (kCoast.result.last_speed < AngularVelocity{}) != (kCoast.result.start_speed < AngularVelocity{});
      if (kTimedOut || kCoast.standstill || kSlow || kReversed) {
        FinishFluxLinkage();
      }
      return;
    }
    default:
      return;  // waits for the confirmation
  }
}

void MotorIdentification::OnSpinUpWindow(const IdentificationWindowResult& window) noexcept {
  // The back-EMF in the forced frame: the demand less the drop of R and L at the forced speed.
  const AngularVelocity kSpeed = slow_->ForcedSpeed();
  const Resistance kResistance = results_.resistance_valid ? results_.resistance : resistance_prior_;
  const Inductance kInductance = results_.inductance_valid ? results_.l_mean : inductance_prior_;
  const Voltage kBackEmfD = window.voltage_mean.d - (kResistance * window.current_mean.d) +
                            (kSpeed * (kInductance * window.current_mean.q));
  const Voltage kBackEmfQ = window.voltage_mean.q - (kResistance * window.current_mean.q) -
                            (kSpeed * (kInductance * window.current_mean.d));
  // The line-to-line peak is sqrt(3) times the phase peak (the amplitude-invariant magnitude).
  const Voltage kTarget = window.dc_link_mean * (kSpinUpBackEmf / std::numbers::sqrt3_v<float>);
  spin_up_hits_ = Hypot(kBackEmfD, kBackEmfQ) < kTarget ? 0U : static_cast<uint8_t>(spin_up_hits_ + 1U);
  const bool kSpeedReached = !(Abs(kSpeed) < Abs(speed_limit_ * kSpinUpSpeedFraction));
  ++point_;
  if (spin_up_hits_ >= kSpinUpHits || kSpeedReached) {
    slow_->HoldForcedSpeed();
    phase_ = Phase::kHold;
    slow_->RequestWindow(kHoldCycles);
    return;
  }
  slow_->RequestWindow(kRampWindowCycles);
}

void MotorIdentification::StartCoast() noexcept {
  // All switches open: the terminals carry the back-EMF alone.
  hardware::kPowerStage.Disable();
  isr_->ClearIdentificationDrive();
  slow_->ReleaseForcedAngle();
  CoastDownMeter meter{};
  meter.standstill_samples = kStandstillCycles * isr_->state.timing.steps_per_cycle;
  coast_generation_ = slow_->StartCoast(meter);
  phase_ = Phase::kCoast;
  slow_->RequestWindow(kCoastTimeoutCycles);
}

void MotorIdentification::FinishFluxLinkage() noexcept {
  const current_control::SlowUpdate::CoastStatus kCoast = slow_->GetCoast();
  const CoastDownMeter::Result kResult =
      kCoast.generation == coast_generation_ ? kCoast.result : CoastDownMeter::Result{};
  slow_->StopCoast();
  results_.flux_periods = kResult.periods;
  if (kResult.periods > 0U) {
    results_.flux_valid = true;
    results_.flux_pm = kResult.flux;
    results_.flux_spread = kResult.spread;
    results_.coast_speed = kResult.start_speed;
  }
  if (kResult.periods >= kMinFluxPeriods && !(kResult.spread > kFluxSpreadTolerance)) {
    passed_ |= Bit(Step::kFluxLinkage);
  } else {
    failed_ |=
        Bit(Step::kFluxLinkage);  // no back-EMF (no phase-voltage sense, a stalled spin-up) or a noisy one
  }
  Next();
}

// ----- the run ---------------------------------------------------------------

void MotorIdentification::RunOnce() noexcept {
  if (!IsActive()) {
    return;
  }
  if (step_ == Step::kFluxLinkage) {
    RunFluxLinkage();
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
      slow_->RequestWindow(WindowCycles());
      return;
    case Phase::kMeasure:
      if (step_ == Step::kResistance) {
        OnResistanceWindow(window);
      } else {
        OnInductanceWindow(window);
      }
      return;
    default:
      return;  // the phases of M3 run in RunFluxLinkage()
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
  } else if (step_ == Step::kFluxLinkage) {
    status.needs_shaft_free = phase_ == Phase::kWaitShaftFree;
    status.progress = FluxLinkageProgress(phase_);
    const current_control::SlowUpdate::CoastStatus kCoast = slow_->GetCoast();
    if (phase_ == Phase::kCoast && kCoast.generation == coast_generation_) {
      status.counter = kCoast.result.periods;
    }
  } else if (step_ == Step::kDone) {
    status.progress = 100U;
  }
  return status;
}

}  // namespace unimoc::identification
