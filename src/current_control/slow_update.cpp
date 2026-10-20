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
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "slow_update.hpp"
#include <cmath>
#include <cstdint>
#include "rotor_system.hpp"
#include "sin_cos.hpp"
#include "stator_system.hpp"
#include "units.hpp"

namespace unimoc::current_control {

// =============================================================================
// init
// =============================================================================

void SlowUpdate::Init(const settings::NvmSettings& settings, CurrentControlIsr& isr) noexcept {
  isr_ = &isr;

  // --- PMSM flux observer parameters ---
  flux_obs.init(settings);

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
  // Nothing to do until the control cycle has filled a buffer.
  // -------------------------------------------------------------------------
  if (!state.observer_pending.load(std::memory_order_acquire)) {
    return false;
  }

  const float kDtFast = state.dt_fast;
  const float kDtSlow = state.dt_slow;

  // -------------------------------------------------------------------------
  // 1. The buffer of the cycle that just finished.
  // -------------------------------------------------------------------------
  const uint8_t kFinishedBuffer = state.observer_buffer;
  const SubStepBuffer& cycle = state.double_buf.buf[kFinishedBuffer];
  const system::Rotor<unit::Current> kCurrentDqMean = cycle.i_dq_mean;

  // -------------------------------------------------------------------------
  // 2. PMSM flux observer. calculate() reads mech_obs.sin_theta / cos_theta
  //    and calls mech_obs.inject_angle_error() at the end.
  // -------------------------------------------------------------------------
  flux_obs.calculate(cycle.u_dq, kCurrentDqMean, flux_setpoint_, unit::Time{kDtSlow}, mech_obs);

  // -------------------------------------------------------------------------
  // 3. Mechanical Kalman predict step (propagates omega and theta forward by
  //    one cycle). Afterwards theta is the estimate for the first sample of
  //    the next cycle.
  // -------------------------------------------------------------------------
  mech_obs.predict(kCurrentDqMean, unit::Time{kDtSlow});

  // -------------------------------------------------------------------------
  // 4. HFI: sample k + 1 answers injection step k of the plan before it, so
  //    sample 0 completes the injection period (step 3) and samples 1..3 start
  //    the next one (steps 0..2). update() advances its step itself.
  // -------------------------------------------------------------------------
  if (isr_->hfi_active) {
    for (uint8_t k = 0U; k < kNumSubSteps; ++k) {
      isr_->hfi.update(cycle.i_ab_samples[k],
                       cycle.sc[k].sin,
                       cycle.sc[k].cos,
                       unit::Time{kDtSlow},
                       mech_obs);
    }
  }

  // -------------------------------------------------------------------------
  // 5. Angles of the next cycle into the inactive buffer:
  //      sample k:     phi = theta + k × omega × dt_fast
  //      duty entry k: phi = theta + (kNumSubSteps − 1 + delay + k) × omega × dt_fast
  //    The next control boundary is the last sample of the next cycle; entry
  //    k takes effect `delay` + k half periods after it (D4).
  // -------------------------------------------------------------------------
  SubStepBuffer& next = state.double_buf.buf[1U - kFinishedBuffer];

  const float kThetaNow = mech_obs.theta.Value();
  const float kStep = mech_obs.omega.Value() * kDtFast;
  const float kOutOffset = static_cast<float>(kNumSubSteps - 1U) + state.output_delay_half_periods;

  for (uint8_t k = 0U; k < kNumSubSteps; ++k) {
    const auto kIndex = static_cast<float>(k);
    next.sc[k] = system::SinCos<unit::Ratio>(unit::Angle{kThetaNow + kIndex * kStep});
    next.sc_out[k] = system::SinCos<unit::Ratio>(unit::Angle{kThetaNow + (kOutOffset + kIndex) * kStep});
  }

  // -------------------------------------------------------------------------
  // 6. Publish: the next control cycle uses the new buffer.
  // -------------------------------------------------------------------------
  state.double_buf.active.store(static_cast<uint8_t>(1U - kFinishedBuffer), std::memory_order_release);
  state.observer_pending.store(false, std::memory_order_release);
  return true;
}

}  // namespace unimoc::current_control
