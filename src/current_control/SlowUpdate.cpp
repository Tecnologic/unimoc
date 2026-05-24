/*
       __  ___   ________  _______  ______
      / / / / | / /  _/  |/  / __ \/ ____/
     / / / /  |/ // // /|_/ / / / / /
    / /_/ / /|  // // /  / / /_/ / /___
    \____/_/ |_/___/_/  /_/\____/\____/

    Universal Motor Control  2026 Alexander <tecnologic86@gmail.com> Evers

    This file is part of UNIMOC.

    UNIMOC is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "SlowUpdate.hpp"
#include <cmath>
#include <cstdint>
#include "SinCos.hpp"
#include "RotorReference.hpp"
#include "StatorReference.hpp"

namespace unimoc
{
namespace current_control
{

// =============================================================================
// init
// =============================================================================

void SlowUpdate::init(const system::NvmSettings& settings,
                      CurrentControlIsr&          isr) noexcept
{
    isr_ = &isr;

    // --- PMSM flux observer parameters ---
    flux_obs.rs  = settings.stator_R;
    flux_obs.L_d = settings.L_d;
    flux_obs.L_q = settings.L_q;
    flux_obs.C_d = settings.pmsm_flux_obs_C_d;
    flux_obs.C_q = settings.pmsm_flux_obs_C_q;

    // Initial flux setpoint: d-axis = nominal ψ_PM, q-axis = 0
    flux_setpoint_ = {settings.flux_pm, 0.0f};

    flux_obs.reset();
}

// =============================================================================
// run_once
// =============================================================================

bool SlowUpdate::run_once() noexcept
{
    if (!isr_)
        return false;

    CurrentControlState& state     = isr_->state;
    auto&                mech_obs  = isr_->mech_obs;

    // -------------------------------------------------------------------------
    // Poll: nothing to do until the ISR has completed a 4-step cycle
    // -------------------------------------------------------------------------
    if (!state.samples_ready)
        return false;

    // -------------------------------------------------------------------------
    // 1. Clear the flag so the ISR can set it again for the next cycle
    // -------------------------------------------------------------------------
    state.samples_ready = false;

    const float dt_fast = state.dt_fast;
    const float dt_slow = state.dt_slow;

    // -------------------------------------------------------------------------
    // 2. Snapshot the buffer that the ISR was using.
    //    We read `active` with acquire ordering so all ISR writes to
    //    i_ab_samples[0..3] are visible before we read them.
    //
    //    Because `samples_ready` is only set at sub-step 3 (after the fourth
    //    write), and the ISR will now be on sub-step 0 of the *next* cycle
    //    (writing to the same active buffer again), we have a full set of
    //    valid samples in old_active.  We copy them to the stack before the
    //    ISR overwrites any of them.
    // -------------------------------------------------------------------------
    const uint8_t old_active =
        state.double_buf.active.load(std::memory_order_acquire);

    const SubStepBuffer& old_buf = state.double_buf.buf[old_active];

    // Stack copies — safe to read even while the ISR may overwrite
    // i_ab_samples[0] for the next cycle (the ISR writes sequentially, so
    // samples 1..3 remain valid until the next wrap).  To be conservative we
    // copy all four before proceeding.
    system::StatorReference<float> i_ab_snap[NUM_SUB_STEPS];
    system::SinCos<float>          sc_snap[NUM_SUB_STEPS];

    for (uint8_t k = 0u; k < NUM_SUB_STEPS; ++k)
    {
        i_ab_snap[k] = old_buf.i_ab_samples[k];
        sc_snap[k]   = old_buf.sc[k];
    }

    // Snapshot last voltage demand (written by ISR before samples_ready was set)
    const system::RotorReference<float> u_dq_last = state.u_dq_last;

    // -------------------------------------------------------------------------
    // 3. Angle-advance correction: re-Park each sample with the sin/cos that
    //    was active when it was captured.  This corrects for the changing
    //    electrical angle across the four sub-steps.
    // -------------------------------------------------------------------------
    system::RotorReference<float> i_dq[NUM_SUB_STEPS];
    for (uint8_t k = 0u; k < NUM_SUB_STEPS; ++k)
    {
        i_dq[k] = i_ab_snap[k].park(sc_snap[k]);
    }

    // -------------------------------------------------------------------------
    // 4. Mean d/q current over the four sub-steps
    // -------------------------------------------------------------------------
    system::RotorReference<float> i_dq_mean{0.0f, 0.0f};
    for (uint8_t k = 0u; k < NUM_SUB_STEPS; ++k)
    {
        i_dq_mean.d += i_dq[k].d;
        i_dq_mean.q += i_dq[k].q;
    }
    constexpr float inv_n = 1.0f / static_cast<float>(NUM_SUB_STEPS);
    i_dq_mean.d *= inv_n;
    i_dq_mean.q *= inv_n;

    // -------------------------------------------------------------------------
    // 5. PMSM flux observer
    //    calculate() internally reads mech_obs.sin_theta / cos_theta and
    //    calls mech_obs.inject_angle_error() at the end.
    // -------------------------------------------------------------------------
    flux_obs.calculate(u_dq_last, i_dq_mean, flux_setpoint_, dt_slow, mech_obs);

    // -------------------------------------------------------------------------
    // 6. Mechanical Kalman predict step (propagates omega and theta forward)
    // -------------------------------------------------------------------------
    mech_obs.predict(i_dq_mean, dt_slow);

    // -------------------------------------------------------------------------
    // 7. HFI update (if active) — updates step counter and feeds error into
    //    the Kalman filter via inject_angle_error().
    //    The HFI observer processes the mean α/β current; it needs the mean
    //    sin/cos for the Park transform inside update().  We use sc_snap[0]
    //    as a representative angle for the slow-update epoch.
    // -------------------------------------------------------------------------
    if (isr_->hfi_active)
    {
        // Compute mean stator-frame current for HFI
        system::StatorReference<float> i_ab_mean{0.0f, 0.0f};
        for (uint8_t k = 0u; k < NUM_SUB_STEPS; ++k)
        {
            i_ab_mean.alpha += i_ab_snap[k].alpha;
            i_ab_mean.beta  += i_ab_snap[k].beta;
        }
        i_ab_mean.alpha *= inv_n;
        i_ab_mean.beta  *= inv_n;

        isr_->hfi.update(
            i_ab_mean,
            mech_obs.sin_theta,
            mech_obs.cos_theta,
            dt_slow,
            mech_obs);
    }

    // -------------------------------------------------------------------------
    // 8. Precompute sc[0..3] for the next 4-step cycle and store them in the
    //    INACTIVE buffer.
    //    phi_k = theta + k × omega × dt_fast
    //    (theta and omega are current Kalman estimates, updated by predict()
    //    and inject_angle_error() just above.)
    // -------------------------------------------------------------------------
    const uint8_t new_active = 1u - old_active;
    SubStepBuffer& new_buf   = state.double_buf.buf[new_active];

    const float theta_now = mech_obs.theta;
    const float omega_now = mech_obs.omega;

    for (uint8_t k = 0u; k < NUM_SUB_STEPS; ++k)
    {
        const float phi_k = theta_now + static_cast<float>(k) * omega_now * dt_fast;
        new_buf.sc[k] = system::SinCos<float>(phi_k);
    }

    // -------------------------------------------------------------------------
    // 9. Atomic flip — from this point the ISR reads the newly prepared buffer.
    //    memory_order_release ensures all stores to new_buf.sc[k] are visible
    //    to any thread that subsequently reads active with memory_order_acquire.
    // -------------------------------------------------------------------------
    state.double_buf.active.store(new_active, std::memory_order_release);

    return true;
}

}  // namespace current_control
}  // namespace unimoc
