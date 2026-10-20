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

void CurrentControlIsr::Init(const settings::NvmSettings& settings,
                             const hardware::HardwareInterface& hardware,
                             const uint32_t timer_clock_hz) noexcept {
  hardware_ = &hardware;

  // --- Timing parameters ---
  const auto kPwmHz = static_cast<uint32_t>(settings.pwm_frequency.Value());
  if (kPwmHz == 0U) {
    return;
  }

  // Centre-aligned PWM: ARR = timer_clock / (2 × f_pwm) − 1
  state.arr = timer_clock_hz / (2U * kPwmHz) - 1U;
  state.dt_fast = 1.0F / (2.0F * static_cast<float>(kPwmHz));
  state.dt_slow = static_cast<float>(kNumSubSteps) * state.dt_fast;
  state.adc_trigger_offset = static_cast<uint32_t>(std::roundf(1e-6F * static_cast<float>(timer_clock_hz)));

  hfi.Init(settings);
  dtc.Init(settings);
  svm.Init(settings);

  // --- Current controller parameters ---
  cc.kp_d = settings.current_kp_d;
  cc.ki_d = settings.current_ki_d;
  cc.kp_q = settings.current_kp_q;
  cc.ki_q = settings.current_ki_q;
  cc.L_d = settings.l_d;
  cc.L_q = settings.l_q;
  cc.psi = settings.flux_pm;
  cc.v_max = settings.current_v_max;

  // --- Mechanical observer parameters ---
  mech_obs.Init(settings);

  // --- Pre-fill both double-buffer halves with zero-angle sin/cos ---
  for (auto& buf : state.double_buf.buf) {
    for (uint8_t k = 0U; k < kNumSubSteps; ++k) {
      buf.sc[k] = system::SinCos<unit::Ratio>(0.0_rad);
      buf.sc_out[k] = system::SinCos<unit::Ratio>(0.0_rad);
    }
  }
  state.double_buf.active.store(0U, std::memory_order_release);

  // --- Reset algorithm state ---
  cc.Reset();
  hfi.Reset();
  // The observer feeds sample 0 of a cycle first; it answers the last
  // injection step of the previous plan (docs/hotpath_plan.md, section 4).
  hfi.step = kNumSubSteps - 1;
  state.samples_ready = false;
  state.observer_pending.store(false, std::memory_order_release);
  state.observer_overruns = 0U;
  state.control_cycles = 0U;
  state.duty_plan = hardware::DutyPlan{};
  state.phase_duties = system::ThreePhase<unit::Ratio>{0.5_ratio, 0.5_ratio, 0.5_ratio};
}

// =============================================================================
// on_control_cycle — once per kNumSubSteps PWM half periods, zero latency
// =============================================================================
//
// Must finish before the first PWM boundary after the control boundary: duty
// entry 0 goes directly into the CCR preload registers. It reaches the
// hardware only through hot_path:: (bound at link time, no function
// pointers) and calls no libm function (built with -fno-math-errno).
//
static bool Exceeds(const system::ThreePhase<unit::Current>& currents, const float limit) noexcept {
  return std::abs(currents.a.Value()) > limit || std::abs(currents.b.Value()) > limit ||
         std::abs(currents.c.Value()) > limit;
}

void CurrentControlIsr::OnControlCycle() noexcept {
  if (hardware_ == nullptr) {
    return;
  }
  ++state.control_cycles;

  // -------------------------------------------------------------------------
  // 1. Samples of the cycle (oldest first) from the hardware boundary.
  // -------------------------------------------------------------------------
  const hardware::ControlCycleSamples kSamples = hardware::hot_path::GetControlCycleSamples();
  const float kDcLinkVoltage = kSamples.dc_link.Value();

  // -------------------------------------------------------------------------
  // 1a. Store the newest raw sample — always, regardless of control mode.
  //     The startup FSM reads it from a lower-priority context.
  // -------------------------------------------------------------------------
  state.raw_phase_currents = kSamples.phase_currents[kNumSubSteps - 1U];
  state.raw_vdc = unit::Voltage{kDcLinkVoltage};

  // -------------------------------------------------------------------------
  // 1b. Hard overcurrent limit. The hardware has checked every sample and
  //     tripped the stage already; this repeats the check on the scaled
  //     currents. The loop stays frozen at neutral duties until the trip is
  //     acknowledged.
  // -------------------------------------------------------------------------
  if (!state.overcurrent_tripped) {
    bool over = kSamples.overcurrent;
    for (const auto& currents : kSamples.phase_currents) {
      over = over || Exceeds(currents, state.overcurrent_limit);
    }
    if (over) {
      hardware::hot_path::TripPowerStage();
      state.overcurrent_tripped = true;
    }
  }
  if (state.overcurrent_tripped) {
    SetUniformDuties(system::ThreePhase<unit::Ratio>{0.5_ratio, 0.5_ratio, 0.5_ratio});
    FinishCycleWithoutControl();
    return;
  }

  // -------------------------------------------------------------------------
  // 1c. Force-duty mode: fixed duties, no control loop.
  // -------------------------------------------------------------------------
  if (force_duty_active) {
    SetUniformDuties(forced_duties);
    FinishCycleWithoutControl();
    return;
  }

  // -------------------------------------------------------------------------
  // 2. Angles of this cycle. The observer prepared them for this cycle; if it
  //    is still busy with the previous one, it is late: count and reuse.
  // -------------------------------------------------------------------------
  if (state.observer_pending.load(std::memory_order_acquire)) {
    ++state.observer_overruns;
  }
  const uint8_t kActiveBuffer = state.double_buf.active.load(std::memory_order_acquire);
  SubStepBuffer& buffer = state.double_buf.buf[kActiveBuffer];

  // -------------------------------------------------------------------------
  // 3. Clarke per sample, Park with the angle of the sample, mean d/q current.
  // -------------------------------------------------------------------------
  system::Rotor<unit::Current> i_dq_mean{0.0F, 0.0F};
  for (uint8_t k = 0U; k < kNumSubSteps; ++k) {
    const system::Stator<unit::Current> kCurrentAlphaBeta = kSamples.phase_currents[k].ToStator();
    buffer.i_ab_samples[k] = kCurrentAlphaBeta;
    const system::Rotor<unit::Current> kCurrentDq = kCurrentAlphaBeta.ToRotor(buffer.sc[k]);
    i_dq_mean.d += kCurrentDq.d;
    i_dq_mean.q += kCurrentDq.q;
  }
  constexpr float kInverseCount = 1.0F / static_cast<float>(kNumSubSteps);
  i_dq_mean.d *= kInverseCount;
  i_dq_mean.q *= kInverseCount;
  buffer.i_dq_mean = i_dq_mean;

  // -------------------------------------------------------------------------
  // 4. Current PI with decoupling feedforward, once per cycle.
  // -------------------------------------------------------------------------
  const system::Rotor<unit::Voltage> kVoltageDq = cc.Update(state.i_ref,
                                                            i_dq_mean,
                                                            mech_obs.omega,
                                                            unit::Time{state.dt_slow},
                                                            unit::Voltage{kDcLinkVoltage});
  buffer.u_dq = kVoltageDq;
  state.u_dq_last = kVoltageDq;

  // -------------------------------------------------------------------------
  // 5. One duty entry per half period: inverse Park with the angle of the
  //    entry, HFI step k, dead-time compensation from the newest current, SVM.
  //    One reciprocal of V_dc for all entries (M2).
  // -------------------------------------------------------------------------
  const system::Stator<unit::Ratio> kDtcRatio = dtc.Calculate(buffer.i_ab_samples[kNumSubSteps - 1U]);
  const float kInverseVDc = control::Svm::InverseDcLinkVoltage(kSamples.dc_link);
  hardware::DutyPlan plan{};
  for (uint8_t k = 0U; k < kNumSubSteps; ++k) {
    const system::SinCos<unit::Ratio>& sc_out = buffer.sc_out[k];
    system::Stator<unit::Voltage> u_ab = kVoltageDq.ToStator(sc_out);
    if (hfi_active) {
      u_ab = u_ab + hfi.GetInjectionVoltage(static_cast<int>(k), sc_out.sin, sc_out.cos);
    }
    plan.duties[k] = svm.CalculateWithInverseDcLinkVoltage(u_ab, kInverseVDc, kDtcRatio);
  }

  // -------------------------------------------------------------------------
  // 6. Write the plan; the observer runs next.
  // -------------------------------------------------------------------------
  SetDutyPlan(plan);
  state.observer_buffer = kActiveBuffer;
  state.observer_pending.store(true, std::memory_order_release);
  state.samples_ready = true;
}

// =============================================================================
// force_duty
// =============================================================================

void CurrentControlIsr::ForceDuty(float da, float db, float dc) noexcept {
  const auto kClampDuty = [this](float duty) noexcept {
    return unit::Ratio{std::clamp(duty, svm.duty_min.Value(), svm.duty_max.Value())};
  };
  forced_duties.a = kClampDuty(da);
  forced_duties.b = kClampDuty(db);
  forced_duties.c = kClampDuty(dc);
  force_duty_active = true;
}

// =============================================================================
// release_force_duty
// =============================================================================

void CurrentControlIsr::ReleaseForceDuty() noexcept {
  force_duty_active = false;
  cc.Reset();  // clear integrators before resuming closed-loop control
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
  state.overcurrent_limit = limit.Value();
  if (hardware_ != nullptr) {
    hardware_->SetOvercurrentLimit(limit);
  }
}

// =============================================================================
// duty output helpers
// =============================================================================

void CurrentControlIsr::SetUniformDuties(const system::ThreePhase<unit::Ratio>& duties) noexcept {
  hardware::DutyPlan plan{};
  for (auto& entry : plan.duties) {
    entry = duties;
  }
  SetDutyPlan(plan);
}

void CurrentControlIsr::SetDutyPlan(const hardware::DutyPlan& plan) noexcept {
  state.duty_plan = plan;
  state.phase_duties = plan.duties[0];
  hardware::hot_path::SetDutyPlan(plan);
}

void CurrentControlIsr::FinishCycleWithoutControl() noexcept { state.samples_ready = true; }

}  // namespace unimoc::current_control
