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
#include <cmath>
#include "hardware_interface.hpp"
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

void CurrentControlIsr::init(const settings::NvmSettings& settings, hardware::HardwareInterface& hardware, const uint32_t timer_clock_hz) noexcept {
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
      // phi_k = 0 + k × omega_init × dt_fast = 0 (omega = 0 at startup)
      buf.sc[k] = system::SinCos<unit::DimensionlessRatio>(0.0_rad);
    }
  }

  // --- Reset algorithm state ---
  cc.reset();
  hfi.reset();
  sub_step_ = 0u;
  active_buf_snapshot_ = 0u;
  state.samples_ready = false;
  state.phase_duties = system::ThreePhase<unit::DimensionlessRatio>{0.5_ratio, 0.5_ratio, 0.5_ratio};
}

// =============================================================================
// on_jeoc — ADC end-of-injected-conversion handler
// =============================================================================
//
// This function is called from the ADC JEOC ISR at the highest configured IRQ
// priority.  It must complete in well under one PWM half-period.
//
static bool duty_in_bounds(const unit::DimensionlessRatio duty,
                           const unit::DimensionlessRatio duty_min,
                           const unit::DimensionlessRatio duty_max) noexcept {
  return duty >= duty_min && duty <= duty_max;
}

// -----------------------------------------------------------------------------
// on_jeoc
// -----------------------------------------------------------------------------

void CurrentControlIsr::on_jeoc() noexcept {
  if (hardware_ == nullptr) return;

  // -------------------------------------------------------------------------
  // 1. Read ADC samples through the hardware boundary.
  // -------------------------------------------------------------------------
  const hardware::CurrentControlSamples samples = hardware_->GetCurrentControlSamples();
  const system::ThreePhase<unit::Current> phase_currents = samples.phase_currents;
  const float v_dc = samples.dc_link.Value();

  // -------------------------------------------------------------------------
  // 1a. Store raw ADC samples — always, regardless of control mode.
  //     The startup FSM reads these from a lower-priority context.
  // -------------------------------------------------------------------------
  state.raw_phase_currents = phase_currents;
  state.raw_vdc = unit::Voltage{v_dc};

  // -------------------------------------------------------------------------
  // 1b. Force-duty mode: write fixed CCR values and skip the control loop.
  //     ADC samples are stored above so the startup FSM can still monitor
  //     them for over-current protection.
  // -------------------------------------------------------------------------
  if (force_duty_active) {
    set_phase_duties(forced_duties);
    sub_step_ = (sub_step_ + 1u) & 3u;
    if (sub_step_ == 0u) {
      state.samples_ready = true;
    }
    return;
  }

  // -------------------------------------------------------------------------
  // 2. Snapshot the active-buffer index at sub-step 0 and reuse it for the
  //    whole 4-step cycle so that a buffer flip mid-cycle is handled safely.
  // -------------------------------------------------------------------------
  if (sub_step_ == 0u) {
    active_buf_snapshot_ = state.double_buf.active.load(std::memory_order_acquire);
  }
  const uint8_t ab = active_buf_snapshot_;
  SubStepBuffer& sb = state.double_buf.buf[ab];
  const system::SinCos<unit::DimensionlessRatio>& sc = sb.sc[sub_step_];

  // -------------------------------------------------------------------------
  // 3. Boundary guard — skip current PI and write neutral duties when any
  //    duty is outside the 5–95 % window (corrupted ADC samples or
  //    insufficient PWM headroom for voltage injection).
  // -------------------------------------------------------------------------
  {
    const system::ThreePhase<unit::DimensionlessRatio> applied_duties = hardware_->GetPhaseDuties();
    if (!duty_in_bounds(applied_duties.a, svm.duty_min, svm.duty_max) || !duty_in_bounds(applied_duties.b, svm.duty_min, svm.duty_max) ||
        !duty_in_bounds(applied_duties.c, svm.duty_min, svm.duty_max)) {
      // Write safe neutral duties (50 %) and skip this control update.
      set_phase_duties(system::ThreePhase<unit::DimensionlessRatio>{0.5_ratio, 0.5_ratio, 0.5_ratio});
      sub_step_ = (sub_step_ + 1u) & 3u;
      if (sub_step_ == 0u) {
        state.samples_ready = true;
      }
      return;
    }
  }

  // -------------------------------------------------------------------------
  // 4. Clarke transform: I_a, I_b, I_c -> I_alpha, I_beta
  // -------------------------------------------------------------------------
  const system::Stator<unit::Current> i_ab = phase_currents.ToStator();

  // -------------------------------------------------------------------------
  // 5. Store current sample in the active buffer for SlowUpdate
  // -------------------------------------------------------------------------
  sb.i_ab_samples[sub_step_] = i_ab;

  // -------------------------------------------------------------------------
  // 6. Park transform: I_α, I_β → I_d, I_q
  // -------------------------------------------------------------------------
  const system::Rotor<unit::Current> i_dq = i_ab.ToRotor(sc);

  // -------------------------------------------------------------------------
  // 7. Current PI with decoupling feedforward
  // -------------------------------------------------------------------------
  const system::Rotor<unit::Voltage> u_dq = cc.update(state.i_ref, i_dq, mech_obs.omega, unit::Time{state.dt_fast}, unit::Voltage{v_dc});

  // Store for SlowUpdate (flux observer needs last voltage)
  state.u_dq_last = u_dq;

  // -------------------------------------------------------------------------
  // 8. HFI voltage injection (α/β frame, added before inverse Park)
  //    The injection voltage is computed from the current step's sin/cos.
  // -------------------------------------------------------------------------
  system::Stator<unit::Voltage> v_inj{0.0f, 0.0f};
  if (hfi_active) {
    v_inj = hfi.get_injection_voltage(sc.sin, sc.cos);
  }

  // -------------------------------------------------------------------------
  // 9. Inverse Park: U_d, U_q → U_α, U_β
  // -------------------------------------------------------------------------
  system::Stator<unit::Voltage> u_ab = u_dq.ToStator(sc);

  // Add HFI injection in the α/β frame
  u_ab = u_ab + v_inj;

  // -------------------------------------------------------------------------
  // 10. Dead-time compensation (adds a correction in the α/β frame)
  // -------------------------------------------------------------------------
  const system::Stator<unit::DimensionlessRatio> dtc_ratio = dtc.calculate(i_ab);

  // -------------------------------------------------------------------------
  // 11. Space-vector modulation → normalised duties [0, 1]
  //     SVM normalises by V_dc and adds the (already normalised) DTC term.
  // -------------------------------------------------------------------------
  const system::ThreePhase<unit::DimensionlessRatio> duties = svm.CalculateWithDeadTimeCompensation(u_ab, unit::Voltage{v_dc}, dtc_ratio);

  // -------------------------------------------------------------------------
  // 12. Write normalized duties through the hardware boundary.
  // -------------------------------------------------------------------------
  set_phase_duties(duties);

  // -------------------------------------------------------------------------
  // 13. Advance sub-step; at wrap-around signal the slow-update task
  // -------------------------------------------------------------------------
  sub_step_ = (sub_step_ + 1u) & 3u;
  if (sub_step_ == 0u) {
    state.samples_ready = true;
  }
}

// =============================================================================
// force_duty
// =============================================================================

void CurrentControlIsr::force_duty(float da, float db, float dc) noexcept {
  forced_duties.a = unit::DimensionlessRatio{da}.Clamp(svm.duty_min.Value(), svm.duty_max.Value());
  forced_duties.b = unit::DimensionlessRatio{db}.Clamp(svm.duty_min.Value(), svm.duty_max.Value());
  forced_duties.c = unit::DimensionlessRatio{dc}.Clamp(svm.duty_min.Value(), svm.duty_max.Value());
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
// set_adc_trigger_offset
// =============================================================================

void CurrentControlIsr::set_adc_trigger_offset(uint32_t offset) noexcept {
  state.adc_trigger_offset = offset;
  if (hardware_ != nullptr) hardware_->SetAdcTriggerOffset(offset);
}

void CurrentControlIsr::set_phase_duties(const system::ThreePhase<unit::DimensionlessRatio>& duties) noexcept {
  state.phase_duties = duties;
  if (hardware_ != nullptr) hardware_->SetPhaseDuties(duties);
}

}  // namespace current_control
}  // namespace unimoc
