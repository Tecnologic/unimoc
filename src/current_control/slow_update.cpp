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
  if (!kOutputsOpen) {
    emf_speed_.Reset();
    const std::optional<unit::Angle> kFluxError =
        flux_obs.Calculate(cycle.u_dq, kCurrentDqMean, flux_setpoint_, kDtSlow, mech_obs);
    if (kFluxError.has_value()) {
      mech_obs.UpdateAngle(*kFluxError, FluxNoise(mech_obs.omega));
    }
  } else if (cycle.phase_voltages_valid && flux_setpoint_.d > unit::MagneticFlux{}) {
    TrackBackEmf(cycle, kCurrentDqMean);
  } else {
    emf_speed_.Reset();
    flux_obs.Reset();
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
  forced_published_.Write(kUseForced ? forced_now_ : ForcedAngle{});
  const unit::AngularVelocity kOmega = kUseForced ? forced_now_.speed : mech_obs.omega;
  const unit::Angle kStep = kOmega * kDtFast;

  next.sample = cycle.first_sample + state.timing.steps_per_cycle;
  next.sc = system::SinCos<unit::Ratio>(kUseForced ? forced_now_.angle : mech_obs.theta);
  next.step = system::SinCos<unit::Ratio>(kStep);
  next.out = system::SinCos<unit::Ratio>(kStep * state.output_delay_steps);

  // -------------------------------------------------------------------------
  // 5. Publish: the control steps take the new reference.
  // -------------------------------------------------------------------------
  angles.active.store(kInactive, std::memory_order_release);
  state.observer_pending.store(false, std::memory_order_release);
  return true;
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

  // The measured voltage in the frame of the estimate of each sample, as the
  // demand of a controlled cycle: the flux observer runs on, unchanged.
  system::Rotor<unit::Voltage> voltage_sum{0.0F, 0.0F};
  for (uint8_t k = 0U; k < kSteps; ++k) {
    const system::Rotor<unit::Voltage> kVoltage = kSamples[k].ToRotor(cycle.sc[k]);
    voltage_sum.d += kVoltage.d;
    voltage_sum.q += kVoltage.q;
  }
  const float kInverse = 1.0F / static_cast<float>(kSteps);
  const system::Rotor<unit::Voltage> kVoltageDq{voltage_sum.d * kInverse, voltage_sum.q * kInverse};

  const std::optional<observer::BackEmfSpeed::Measurement> kSpeed =
      emf_speed_.Update(kSamples, flux_setpoint_.d);
  if (kSpeed.has_value()) {
    mech_obs.UpdateSpeed(kSpeed->speed - mech_obs.omega, kSpeed->variance);
  }
  const std::optional<unit::Angle> kFluxError =
      flux_obs.Calculate(kVoltageDq, current_dq, flux_setpoint_, isr_->state.dt_slow, mech_obs);
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
    coast_meter_ = coast_parameters_.Read();
    coast_meter_.Start();
  }
  if (outputs_open && cycle.phase_voltages_valid) {
    const uint8_t kSteps = isr_->state.timing.steps_per_cycle;
    for (uint8_t k = 0U; k < kSteps; ++k) {
      coast_meter_.Add(cycle.u_ab_samples[k], isr_->state.dt_fast);
    }
  }
  coast_status_.Write(CoastStatus{true, coast_meter_.Standstill(), coast_meter_.GetResult()});
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
  if (!hall.Enabled()) {
    return;  // taken anyway, so the queue of the platform never overflows
  }
  constexpr unit::Time kTickPeriod = 1.0_s / static_cast<float>(hardware::kTimestampFrequency);
  hall.Update(std::span<const observer::HallEvent>{events.data(), count},
              cycle.timestamp - reference_offset_ticks_,
              kTickPeriod,
              isr_->mech_obs);
}

}  // namespace unimoc::current_control
