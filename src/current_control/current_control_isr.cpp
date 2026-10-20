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
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "current_control_isr.hpp"

#include <algorithm>
#include <cmath>
#include "hardware_interface.hpp"
#include "power_stage_interface.hpp"
#include "nvm_settings.hpp"
#include "sin_cos.hpp"
#include "three_phase_system.hpp"
#include "units.hpp"

namespace unimoc {
namespace current_control {

using namespace unit;

// =============================================================================
// init
// =============================================================================

void CurrentControlIsr::init(const settings::NvmSettings& settings,
                             hardware::HardwareInterface& hardware,
                             const uint32_t timer_clock_hz) noexcept {
  hardware_ = &hardware;

  // --- Timing parameters ---
  const uint32_t f_hz = static_cast<uint32_t>(settings.pwm_frequency.Value());
  if (f_hz == 0u) return;

  // Centre-aligned PWM: ARR = timer_clock / (2 × f_pwm) − 1
  state.arr = timer_clock_hz / (2u * f_hz) - 1u;
  state.dt_fast = 1.0f / (2.0f * static_cast<float>(f_hz));
  state.dt_slow = static_cast<float>(NUM_SUB_STEPS) * state.dt_fast;
  state.adc_trigger_offset = static_cast<uint32_t>(std::roundf(1e-6f * static_cast<float>(timer_clock_hz)));

  hfi.init(settings);
  dtc.init(settings);
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
  mech_obs.init(settings);

  // --- Pre-fill both double-buffer halves with zero-angle sin/cos ---
  for (auto& buf : state.double_buf.buf) {
    for (uint8_t k = 0u; k < NUM_SUB_STEPS; ++k) {
      buf.sc[k] = system::SinCos<unit::Ratio>(0.0_rad);
      buf.sc_out[k] = system::SinCos<unit::Ratio>(0.0_rad);
    }
  }
  state.double_buf.active.store(0u, std::memory_order_release);

  // --- Reset algorithm state ---
  cc.reset();
  hfi.reset();
  // The observer feeds sample 0 of a cycle first; it answers the last
  // injection step of the previous plan (docs/hotpath_plan.md, section 4).
  hfi.step = NUM_SUB_STEPS - 1;
  state.samples_ready = false;
  state.observer_pending.store(false, std::memory_order_release);
  state.observer_overruns = 0u;
  state.control_cycles = 0u;
  state.duty_plan = hardware::DutyPlan{};
  state.phase_duties = system::ThreePhase<unit::Ratio>{0.5_ratio, 0.5_ratio, 0.5_ratio};
}

// =============================================================================
// on_control_cycle — once per NUM_SUB_STEPS PWM half periods, zero latency
// =============================================================================
//
// Must finish before the first PWM boundary after the control boundary: duty
// entry 0 goes directly into the CCR preload registers.
//
static bool exceeds(const system::ThreePhase<unit::Current>& i, const float limit) noexcept {
  return std::abs(i.a.Value()) > limit || std::abs(i.b.Value()) > limit || std::abs(i.c.Value()) > limit;
}

void CurrentControlIsr::on_control_cycle() noexcept {
  if (hardware_ == nullptr) return;
  ++state.control_cycles;

  // -------------------------------------------------------------------------
  // 1. Samples of the cycle (oldest first) from the hardware boundary.
  // -------------------------------------------------------------------------
  const hardware::ControlCycleSamples samples = hardware_->GetControlCycleSamples();
  const float v_dc = samples.dc_link.Value();

  // -------------------------------------------------------------------------
  // 1a. Store the newest raw sample — always, regardless of control mode.
  //     The startup FSM reads it from a lower-priority context.
  // -------------------------------------------------------------------------
  state.raw_phase_currents = samples.phase_currents[NUM_SUB_STEPS - 1u];
  state.raw_vdc = unit::Voltage{v_dc};

  // -------------------------------------------------------------------------
  // 1b. Hard overcurrent limit. The hardware has checked every sample and
  //     tripped the stage already; this repeats the check on the scaled
  //     currents. The loop stays frozen at neutral duties until the trip is
  //     acknowledged.
  // -------------------------------------------------------------------------
  if (!state.overcurrent_tripped) {
    bool over = samples.overcurrent;
    for (const auto& i : samples.phase_currents) over = over || exceeds(i, state.overcurrent_limit_A);
    if (over) {
      hardware::power_stage.Trip();
      state.overcurrent_tripped = true;
    }
  }
  if (state.overcurrent_tripped) {
    set_uniform_duties(system::ThreePhase<unit::Ratio>{0.5_ratio, 0.5_ratio, 0.5_ratio});
    finish_cycle_without_control();
    return;
  }

  // -------------------------------------------------------------------------
  // 1c. Force-duty mode: fixed duties, no control loop.
  // -------------------------------------------------------------------------
  if (force_duty_active) {
    set_uniform_duties(forced_duties);
    finish_cycle_without_control();
    return;
  }

  // -------------------------------------------------------------------------
  // 2. Angles of this cycle. The observer prepared them for this cycle; if it
  //    is still busy with the previous one, it is late: count and reuse.
  // -------------------------------------------------------------------------
  if (state.observer_pending.load(std::memory_order_acquire)) ++state.observer_overruns;
  const uint8_t ab = state.double_buf.active.load(std::memory_order_acquire);
  SubStepBuffer& sb = state.double_buf.buf[ab];

  // -------------------------------------------------------------------------
  // 3. Clarke per sample, Park with the angle of the sample, mean d/q current.
  // -------------------------------------------------------------------------
  system::Rotor<unit::Current> i_dq_mean{0.0f, 0.0f};
  for (uint8_t k = 0u; k < NUM_SUB_STEPS; ++k) {
    const system::Stator<unit::Current> i_ab = samples.phase_currents[k].ToStator();
    sb.i_ab_samples[k] = i_ab;
    const system::Rotor<unit::Current> i_dq = i_ab.ToRotor(sb.sc[k]);
    i_dq_mean.d += i_dq.d;
    i_dq_mean.q += i_dq.q;
  }
  constexpr float inv_n = 1.0f / static_cast<float>(NUM_SUB_STEPS);
  i_dq_mean.d *= inv_n;
  i_dq_mean.q *= inv_n;
  sb.i_dq_mean = i_dq_mean;

  // -------------------------------------------------------------------------
  // 4. Current PI with decoupling feedforward, once per cycle.
  // -------------------------------------------------------------------------
  const system::Rotor<unit::Voltage> u_dq =
      cc.update(state.i_ref, i_dq_mean, mech_obs.omega, unit::Time{state.dt_slow}, unit::Voltage{v_dc});
  sb.u_dq = u_dq;
  state.u_dq_last = u_dq;

  // -------------------------------------------------------------------------
  // 5. One duty entry per half period: inverse Park with the angle of the
  //    entry, HFI step k, dead-time compensation from the newest current, SVM.
  // -------------------------------------------------------------------------
  const system::Stator<unit::Ratio> dtc_ratio = dtc.calculate(sb.i_ab_samples[NUM_SUB_STEPS - 1u]);
  hardware::DutyPlan plan{};
  for (uint8_t k = 0u; k < NUM_SUB_STEPS; ++k) {
    const system::SinCos<unit::Ratio>& sc_out = sb.sc_out[k];
    system::Stator<unit::Voltage> u_ab = u_dq.ToStator(sc_out);
    if (hfi_active) {
      u_ab = u_ab + hfi.get_injection_voltage(static_cast<int>(k), sc_out.sin, sc_out.cos);
    }
    plan.duties[k] = svm.CalculateWithDeadTimeCompensation(u_ab, unit::Voltage{v_dc}, dtc_ratio);
  }

  // -------------------------------------------------------------------------
  // 6. Write the plan; the observer runs next.
  // -------------------------------------------------------------------------
  set_duty_plan(plan);
  state.observer_buffer = ab;
  state.observer_pending.store(true, std::memory_order_release);
  state.samples_ready = true;
}

// =============================================================================
// force_duty
// =============================================================================

void CurrentControlIsr::force_duty(float da, float db, float dc) noexcept {
  const auto clamp_duty = [this](float duty) noexcept {
    return unit::Ratio{std::clamp(duty, svm.duty_min.Value(), svm.duty_max.Value())};
  };
  forced_duties.a = clamp_duty(da);
  forced_duties.b = clamp_duty(db);
  forced_duties.c = clamp_duty(dc);
  force_duty_active = true;
}

// =============================================================================
// release_force_duty
// =============================================================================

void CurrentControlIsr::release_force_duty() noexcept {
  force_duty_active = false;
  cc.reset();  // clear integrators before resuming closed-loop control
}

// =============================================================================
// set_adc_trigger_offset / set_overcurrent_limit
// =============================================================================

void CurrentControlIsr::set_adc_trigger_offset(uint32_t offset) noexcept {
  state.adc_trigger_offset = offset;
  if (hardware_ != nullptr) hardware_->SetAdcTriggerOffset(offset);
}

void CurrentControlIsr::set_overcurrent_limit(const unit::Current limit) noexcept {
  state.overcurrent_limit_A = limit.Value();
  if (hardware_ != nullptr) hardware_->SetOvercurrentLimit(limit);
}

// =============================================================================
// duty output helpers
// =============================================================================

void CurrentControlIsr::set_uniform_duties(const system::ThreePhase<unit::Ratio>& duties) noexcept {
  hardware::DutyPlan plan{};
  for (auto& entry : plan.duties) entry = duties;
  set_duty_plan(plan);
}

void CurrentControlIsr::set_duty_plan(const hardware::DutyPlan& plan) noexcept {
  state.duty_plan = plan;
  state.phase_duties = plan.duties[0];
  if (hardware_ != nullptr) hardware_->SetDutyPlan(plan);
}

void CurrentControlIsr::finish_cycle_without_control() noexcept { state.samples_ready = true; }

}  // namespace current_control
}  // namespace unimoc
