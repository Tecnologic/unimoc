/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file slow_update.cpp
 *    @brief Background current-control update implementation.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "slow_update.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <span>
#include "angle_wrap.hpp"
#include "atan2.hpp"
#include "hall_capture.hpp"
#include "power_stage_interface.hpp"
#include "rotor_system.hpp"
#include "sin_cos.hpp"
#include "stator_system.hpp"
#include "units.hpp"

namespace unimoc::current_control {

// =============================================================================
// init
// =============================================================================

void SlowUpdate::Init(const settings::NvmSettings& settings,
                      CurrentControlIsr& isr,
                      const hardware::HardwareInterface& hardware) noexcept {
  isr_ = &isr;
  hardware_ = &hardware;
  sense_delays_ = hardware.GetSenseDelays();

  // --- PMSM flux observer parameters ---
  flux_obs.Init(settings);
  flux_r_speed_ = settings.pmsm_flux_obs_r_speed;

  // --- Hall fusion: the estimate refers to the first sample of a cycle,
  //     the platform timestamps the last one. ---
  hall.Init(settings);
  const unit::Time kOffset = isr.state.dt_fast * static_cast<float>(isr.state.timing.steps_per_cycle - 1U);
  reference_offset_ticks_ =
      static_cast<uint32_t>((kOffset.Value() * static_cast<float>(hardware::kTimestampFrequency)) + 0.5F);

  // Initial flux setpoint: d-axis = nominal ψ_PM, q-axis = 0
  flux_setpoint_ = {settings.flux_pm, unit::MagneticFlux{}};

  motor_type_ = settings.motor_type;
  // EESM: no field until the excitation module reports one.
  field_flux_.Write(unit::MagneticFlux{});
  slip_.Write(unit::AngularVelocity{});
  rotor_flux_.Write(unit::MagneticFlux{});
  asm_obs.R_s = settings.asm_r_s;
  asm_obs.R_r = settings.asm_r_r;
  asm_obs.L_s = settings.asm_l_s;
  asm_obs.L_r = settings.asm_l_r;
  asm_obs.L_m = settings.asm_l_m;
  asm_obs.g_i = settings.asm_obs_g_i;
  asm_obs.g_flux = settings.asm_obs_g_flux;
  asm_obs.Reset();
  asm_flux_reference_ = settings.asm_flux_psi_r;
  field_speed_ = unit::AngularVelocity{};
  last_flux_angle_ = unit::Angle{};
  rotor_speed_integral_ = unit::AngularVelocity{};
  if (motor_type_ == system::MotorType::kAsm) {
    // No torque model for the field frame (UpdateAsm()): no flux, no reluctance.
    isr.mech_obs.L_d = settings.asm_l_s;
    isr.mech_obs.L_q = settings.asm_l_s;
    isr.mech_obs.psi = unit::MagneticFlux{};
  } else if (motor_type_ == system::MotorType::kEesm) {
    isr.mech_obs.psi = unit::MagneticFlux{};
  }
}

// =============================================================================
// run_once — after each control cycle, lower priority than the control cycle
// =============================================================================

bool SlowUpdate::RunOnce() noexcept {
  if (isr_ == nullptr) {
    return false;
  }

  CurrentControlState& state = isr_->state;
  auto& mech_obs = isr_->mech_obs;

  // -------------------------------------------------------------------------
  // Nothing to do until the control steps have filled a record.
  // -------------------------------------------------------------------------
  if (!state.observer_pending.load(std::memory_order_acquire)) {
    return false;
  }

  const unit::Time kDtFast = state.dt_fast;
  const unit::Time kDtSlow = state.dt_slow;

  // -------------------------------------------------------------------------
  // 1. The record of the cycle that just finished.
  // -------------------------------------------------------------------------
  const CycleRecord& cycle = state.records[state.observer_buffer];
  const system::Rotor<unit::Current> kCurrentDqMean = cycle.i_dq_mean;
  AccumulateWindow(cycle);

  // -------------------------------------------------------------------------
  // 2. Covariance prediction of the cycle, then the angle measurements, each
  //    with its own noise (D2). The flux observer reads sin/cos of the
  //    estimate the cycle used.
  // -------------------------------------------------------------------------
  mech_obs.PredictCovariance(kDtSlow);
  // The voltage demand reaches the motor only while the outputs are released.
  // With the outputs open the terminals carry the back-EMF: where the platform
  // measures them, the flux observer integrates those voltages instead and the
  // speed follows from them; otherwise it has nothing to integrate and starts anew.
  const bool kOutputsOpen = hardware::kPowerStage.GetState() != hardware::PowerStageState::kEnabled;
  if (motor_type_ == system::MotorType::kAsm) {
    UpdateAsm(cycle, !kOutputsOpen);
  } else {
    if (motor_type_ == system::MotorType::kEesm) {
      const unit::MagneticFlux kField = field_flux_.Read();
      flux_setpoint_ = {kField, unit::MagneticFlux{}};
      mech_obs.psi = kField;
    }
    if (!kOutputsOpen) {
      emf_speed_.Reset();
      const std::optional<unit::Angle> kFluxError =
          flux_obs.Calculate(cycle.u_dq, kCurrentDqMean, flux_setpoint_, kDtSlow, mech_obs);
      if (kFluxError.has_value()) {
        mech_obs.UpdateAngle(*kFluxError, FluxNoise(mech_obs.omega));
      }
    } else if (cycle.phase_voltages_valid && flux_setpoint_.d > unit::MagneticFlux{}) {
      TrackBackEmf(cycle, kCurrentDqMean);  // an EESM only with its field on
    } else {
      emf_speed_.Reset();
      flux_obs.Reset();
    }
  }
  MeasureCoast(cycle, kOutputsOpen);
  FuseHall(cycle);

  // -------------------------------------------------------------------------
  // 3. Mechanical Kalman predict step (propagates omega and theta forward by
  //    one cycle). Afterwards theta is the estimate for the first sample of
  //    the next cycle.
  // -------------------------------------------------------------------------
  mech_obs.Predict(kCurrentDqMean, kDtSlow);

  // -------------------------------------------------------------------------
  // 4. Angle reference of the next cycle into the inactive buffer: the angle
  //    at its first sample, the rotation of one control step, and the rotation
  //    from a sample to the middle of the interval its duties apply to
  //    (output delay, D4). The control steps advance the angle themselves.
  // -------------------------------------------------------------------------
  AngleBuffer& angles = state.angles;
  const auto kInactive = static_cast<uint8_t>(1U - angles.active.load(std::memory_order_relaxed));
  AngleReference& next = angles.buf[kInactive];

  // The identification's forced angle replaces the estimate (plan section 5.1).
  const uint8_t kForced = forced_state_.load(std::memory_order_acquire);
  if (kForced == kForcedNew) {
    forced_hold_.store(false, std::memory_order_relaxed);
    forced_now_ = forced_.Read();
    forced_state_.store(kForcedOn, std::memory_order_relaxed);
  } else if (kForced == kForcedOn) {
    if (forced_hold_.exchange(false, std::memory_order_acquire)) {
      forced_now_.acceleration = unit::AngularAcceleration{};
    }
    forced_now_.angle = observer::WrapAngle(forced_now_.angle + (forced_now_.speed * kDtSlow) +
                                            (forced_now_.acceleration * kDtSlow * kDtSlow * 0.5F));
    forced_now_.speed = forced_now_.speed + (forced_now_.acceleration * kDtSlow);
  }
  const bool kUseForced = kForced != kForcedOff;
  if (!kUseForced && forced_last_) {
    // Back from a forced angle: the estimate ran in the background on currents
    // in a frame that was not its own (and learned a load torque from them). It
    // restarts where the forced angle left the rotor, the flux observer anew.
    mech_obs.Reset(observer::WrapAngle(forced_now_.angle + (forced_now_.speed * kDtSlow)), forced_now_.speed);
    flux_obs.Reset();
    emf_speed_.Reset();
  }
  forced_last_ = kUseForced;
  forced_published_.Write(kUseForced ? forced_now_ : ForcedAngle{});
  const unit::AngularVelocity kOmega = kUseForced ? forced_now_.speed : mech_obs.omega;
  const unit::Angle kStep = kOmega * kDtFast;

  next.sample = cycle.first_sample + state.timing.steps_per_cycle;
  // The measured currents lag by their filter delay: the Park angle is the
  // angle they show, and the rotation to the output interval longer by as much.
  const unit::Angle kCurrentLag = kOmega * sense_delays_.current;
  next.sc = system::SinCos<unit::Ratio>((kUseForced ? forced_now_.angle : mech_obs.theta) - kCurrentLag);
  next.step = system::SinCos<unit::Ratio>(kStep);
  next.out = system::SinCos<unit::Ratio>((kStep * state.output_delay_steps) + kCurrentLag);
  next.speed = kOmega;

  // -------------------------------------------------------------------------
  // 5. Publish: the control steps take the new reference.
  // -------------------------------------------------------------------------
  angles.active.store(kInactive, std::memory_order_release);
  state.observer_pending.store(false, std::memory_order_release);
  return true;
}

// =============================================================================
// update_asm — the rotor flux frame of an induction motor
// =============================================================================

void SlowUpdate::UpdateAsm(const CycleRecord& cycle, const bool powered) noexcept {
  auto& mech_obs = isr_->mech_obs;
  const CurrentControlState& state = isr_->state;
  if (!powered) {
    asm_obs.Reset();
    field_speed_ = unit::AngularVelocity{};
    last_flux_angle_ = unit::Angle{};
    rotor_speed_integral_ = unit::AngularVelocity{};
    slip_.Write(unit::AngularVelocity{});
    rotor_flux_.Write(unit::MagneticFlux{});
    return;
  }
  // Slip from the last flux estimate: omega_slip = R_r / L_r * L_m i_q / |psi_r| (electrical). While the
  // flux builds up, at least half the reference counts, so a small estimate gives no runaway slip.
  constexpr unit::MagneticFlux kMinFlux = 1.0_mWb;
  const unit::MagneticFlux kSlipFlux = unit::Max(asm_obs.flux_magnitude, asm_flux_reference_ * 0.5F);
  unit::AngularVelocity slip;
  if (kSlipFlux > kMinFlux) {
    slip = ((asm_obs.R_r / asm_obs.L_r) * ((asm_obs.L_m * cycle.i_dq_mean.q) / kSlipFlux)) * 1.0_rad;
  }
  // Speed adaptation of the full-order observer (Kubota): the current error crossed with the estimated rotor
  // flux drives the rotor speed estimate, so the rotor speed is observed instead of assumed. The field turns
  // at the rotor speed plus the slip; its angle is the flux angle (Atan2, no libm).
  const uint8_t kSteps = state.timing.steps_per_cycle;
  for (uint8_t k = 0U; k < kSteps; ++k) {
    const system::Stator<unit::Current>& measured = cycle.i_ab_samples[k];
    const unit::Torque kCross = ((measured.alpha - asm_obs.i_alpha_hat) * asm_obs.psi_r_beta) -
                                ((measured.beta - asm_obs.i_beta_hat) * asm_obs.psi_r_alpha);
    rotor_speed_integral_ = rotor_speed_integral_ + (kCross * speed_ki_ * state.dt_fast);
    const unit::AngularVelocity kRotorSpeed = rotor_speed_integral_ + (kCross * speed_kp_);
    asm_obs.Estimate(cycle.u_dq.ToStator(cycle.sc[k]), measured, kRotorSpeed, state.dt_fast);
  }
  const unit::AngularVelocity kRotorSpeed = rotor_speed_integral_;
  if (asm_obs.flux_magnitude > asm_flux_reference_ * 0.5F && asm_obs.flux_magnitude > kMinFlux) {
    field_speed_ = kRotorSpeed + slip;
  } else {
    // While the flux builds up its angle is the direction of the magnetizing current.
    field_speed_ = kRotorSpeed;
  }
  last_flux_angle_ = asm_obs.flux_angle;
  const unit::Angle kFrame = (asm_obs.flux_magnitude > kMinFlux) ? asm_obs.flux_angle : mech_obs.theta;
  // The observer state refers to the start of the next cycle; Predict() then adds omega dt_slow.
  mech_obs.Reset(observer::WrapAngle(kFrame - (field_speed_ * state.dt_slow)), field_speed_);
  slip = field_speed_ - kRotorSpeed;
  slip_.Write(slip);
  rotor_flux_.Write(asm_obs.flux_magnitude);
}

// =============================================================================
// accumulate_window — the measurement windows of the motor identification
// =============================================================================

void SlowUpdate::AccumulateWindow(const CycleRecord& cycle) noexcept {
  const uint32_t kGeneration = window_generation_.load(std::memory_order_acquire);
  if (kGeneration != window_running_) {
    window_running_ = kGeneration;
    window_sums_ = IdentificationWindowResult{};
  }
  const uint32_t kCycles = window_cycles_;
  if (kCycles == 0U || window_sums_.cycles >= kCycles) {
    return;  // cancelled, or complete and not yet requested again
  }
  window_sums_.current_mean = window_sums_.current_mean + cycle.i_dq_mean;
  window_sums_.voltage_mean = window_sums_.voltage_mean + cycle.u_dq;
  window_sums_.dc_link_mean = window_sums_.dc_link_mean + cycle.dc_link;
  // The phasors of the sine: the currents at the phases of their samples, the
  // voltage of each step's duties at the phase of the middle of their interval.
  const bool kAxisD = cycle.injection_axis == static_cast<uint8_t>(InjectionAxis::kD);
  const settings::ControlTiming& timing = isr_->state.timing;
  for (uint8_t k = 0U; k < timing.steps_per_cycle; ++k) {
    const system::Rotor<unit::Current> kCurrent = cycle.i_ab_samples[k].ToRotor(cycle.sc[k]);
    window_sums_.current_phasor.Add(kAxisD ? kCurrent.d : kCurrent.q,
                                    InjectionSinCos(
                                        InjectionSamplePhase(cycle.first_sample + k, timing.divider)));
    const system::SinCos<unit::Ratio> kEntry =
        InjectionSinCos(InjectionEntryPhase(cycle.first_sample + k, timing.divider));
    window_sums_.voltage_phasor.Add(cycle.injection_amplitude * kEntry.sin, kEntry);
  }
  ++window_sums_.cycles;
  if (window_sums_.cycles < kCycles) {
    return;
  }
  const float kInverse = 1.0F / static_cast<float>(kCycles);
  window_result_ = window_sums_;
  window_result_.current_mean = window_sums_.current_mean * kInverse;
  window_result_.voltage_mean = window_sums_.voltage_mean * kInverse;
  window_result_.dc_link_mean = window_sums_.dc_link_mean * kInverse;
  window_done_.store(kGeneration, std::memory_order_release);
}

// =============================================================================
// track_back_emf — outputs open: the observer follows the coasting motor
// =============================================================================

void SlowUpdate::TrackBackEmf(const CycleRecord& cycle,
                              const system::Rotor<unit::Current>& current_dq) noexcept {
  auto& mech_obs = isr_->mech_obs;
  const uint8_t kSteps = isr_->state.timing.steps_per_cycle;
  const std::span<const system::Stator<unit::Voltage>> kSamples{cycle.u_ab_samples.data(), kSteps};

  const std::optional<observer::BackEmfSpeed::Measurement> kSpeed =
      emf_speed_.Update(kSamples, flux_setpoint_.d);
  if (!kSpeed.has_value()) {
    return;
  }
  if (kSpeed->upper_bound) {
    // Too little back-EMF for an angle: the flux observer starts anew, as
    // without the sense; the voltages still bound the speed, so an estimate
    // beyond the bound is pulled to it.
    flux_obs.Reset();
    if (Abs(mech_obs.omega) > kSpeed->speed) {
      const unit::AngularVelocity kBound =
          mech_obs.omega > unit::AngularVelocity{} ? kSpeed->speed : -kSpeed->speed;
      mech_obs.UpdateSpeed(kBound - mech_obs.omega, kSpeed->variance);
    }
    return;
  }

  // The flux observer turns its voltage into the stator frame with the
  // estimate it reads now, the angle at the cycle's first sample, and compares
  // the integrated flux with that angle. The samples are in the stator frame
  // already: their sum is the flux change of the cycle up to half a step after
  // its last sample, (N - 1/2) steps after the first. Their mean turned into
  // the frame of the estimate at that instant makes the flux the observer
  // integrates the flux at the first sample, so the estimate settles on the
  // angle at the samples. The measured voltages lag by their filter delay:
  // their flux is that much earlier.
  system::Stator<unit::Voltage> voltage_sum{0.0F, 0.0F};
  for (const auto& sample : kSamples) {
    voltage_sum = voltage_sum + sample;
  }
  const unit::Time kFluxTime =
      (isr_->state.dt_fast * (static_cast<float>(kSteps) - 0.5F)) - sense_delays_.phase_voltage;
  const system::Rotor<unit::Voltage> kVoltageDq =
      (voltage_sum / static_cast<float>(kSteps))
          .ToRotor(system::SinCos<unit::Ratio>(mech_obs.theta + (mech_obs.omega * kFluxTime)));
  const std::optional<unit::Angle> kFluxError =
      flux_obs.Calculate(kVoltageDq, current_dq, flux_setpoint_, isr_->state.dt_slow, mech_obs);
  mech_obs.UpdateSpeed(kSpeed->speed - mech_obs.omega, kSpeed->variance);
  if (kFluxError.has_value()) {
    mech_obs.UpdateAngle(*kFluxError, FluxNoise(mech_obs.omega));
  }
}

// =============================================================================
// measure_coast — the coast-down meter of M3
// =============================================================================

void SlowUpdate::MeasureCoast(const CycleRecord& cycle, const bool outputs_open) noexcept {
  const uint32_t kGeneration = coast_generation_.load(std::memory_order_acquire);
  if (kGeneration == 0U || coast_stop_.load(std::memory_order_relaxed) == kGeneration) {
    if (coast_running_ != 0U) {
      CoastStatus status = coast_status_.Read();
      status.running = false;
      coast_status_.Write(status);
      coast_running_ = 0U;
    }
    return;
  }
  if (kGeneration != coast_running_) {
    coast_running_ = kGeneration;
    coast_meter_ = identification::CoastDownMeter{coast_parameters_.Read()};
  }
  if (outputs_open && cycle.phase_voltages_valid) {
    const uint8_t kSteps = isr_->state.timing.steps_per_cycle;
    for (uint8_t k = 0U; k < kSteps; ++k) {
      coast_meter_.Add(cycle.u_ab_samples[k], isr_->state.dt_fast);
    }
  }
  coast_status_.Write(CoastStatus{kGeneration, true, coast_meter_.Standstill(), coast_meter_.GetResult()});
}

// =============================================================================
// flux_noise — the back-EMF carries no angle information at standstill
// =============================================================================

unit::AngleVariance SlowUpdate::FluxNoise(const unit::AngularVelocity omega) const noexcept {
  const unit::AngleVariance kNoise = isr_->mech_obs.R;
  if (!(flux_r_speed_ > unit::AngleVarianceSpeedWeight{})) {
    return kNoise;
  }
  // At most k (below 1 rad/s): the measurement is all but ignored there.
  constexpr unit::AngularVelocityVariance kMinSpeedSquared = 1.0_rad_per_s * 1.0_rad_per_s;
  const unit::AngularVelocityVariance kSpeedSquared = unit::Max(omega * omega, kMinSpeedSquared);
  return kNoise + (flux_r_speed_ / kSpeedSquared);
}

// =============================================================================
// fuse_hall — edges and sector of the Hall sensors
// =============================================================================

void SlowUpdate::FuseHall(const CycleRecord& cycle) noexcept {
  if (hardware_ == nullptr) {
    return;
  }
  std::array<observer::HallEvent, kMaxHallEvents> events{};
  std::size_t count = 0U;
  while (count < events.size() && hardware_->PopHallEvent(events[count])) {
    ++count;
  }
  if (!hall.Enabled() || motor_type_ == system::MotorType::kAsm) {
    return;  // taken anyway, so the queue of the platform never overflows; an ASM's Hall is not its field
  }
  constexpr unit::Time kTickPeriod = 1.0_s / static_cast<float>(hardware::kTimestampFrequency);
  hall.Update(std::span<const observer::HallEvent>{events.data(), count},
              cycle.timestamp - reference_offset_ticks_,
              kTickPeriod,
              isr_->mech_obs);
}

}  // namespace unimoc::current_control
