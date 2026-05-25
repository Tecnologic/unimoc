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
#include "CurrentControlIsr.hpp"
#include <cmath>
#include <algorithm>
#include "ThreePhase.hpp"
#include "SinCos.hpp"
#include "NvmSettings.hpp"

namespace unimoc
{
namespace current_control
{

// =============================================================================
// init
// =============================================================================

void CurrentControlIsr::init(const system::NvmSettings& settings,
                              const uint32_t             timer_clock_hz) noexcept
{
    // --- Timing parameters ---
    const auto f_khz = static_cast<uint32_t>(settings.pwm_frequency);
    const auto f_hz  = f_khz * 1000u;

    // Centre-aligned PWM: ARR = timer_clock / (2 × f_pwm) − 1
    state.arr                = timer_clock_hz / (2u * f_hz) - 1u;
    state.dt_fast            = 1.0f / (2.0f * static_cast<float>(f_hz));
    state.dt_slow            = static_cast<float>(NUM_SUB_STEPS) * state.dt_fast;
    state.adc_trigger_offset = static_cast<uint32_t>(
        std::roundf(1e-6f * static_cast<float>(timer_clock_hz)));

    // --- Current controller parameters ---
    cc.kp_d  = settings.current_kp_d;
    cc.ki_d  = settings.current_ki_d;
    cc.kp_q  = settings.current_kp_q;
    cc.ki_q  = settings.current_ki_q;
    const float L_d = settings.L_d;
    const float L_q = settings.L_q;
    const float psi = settings.flux_pm;

    cc.L_d   = L_d;
    cc.L_q   = L_q;
    cc.psi   = psi;
    cc.v_max = settings.current_v_max;

    // --- Mechanical observer parameters ---
    mech_obs.psi       = psi;
    mech_obs.L_d       = L_d;
    mech_obs.L_q       = L_q;
    mech_obs.J         = settings.motor_J;
    mech_obs.omega_max = settings.motor_omega_max;
    mech_obs.omega_min = settings.motor_omega_min;
    mech_obs.Q         = settings.mech_obs_Q;
    mech_obs.R         = settings.mech_obs_R;

    // --- Pre-fill both double-buffer halves with zero-angle sin/cos ---
    for (auto& buf : state.double_buf.buf)
    {
        for (uint8_t k = 0u; k < NUM_SUB_STEPS; ++k)
        {
            // phi_k = 0 + k × omega_init × dt_fast = 0 (omega = 0 at startup)
            buf.sc[k] = system::SinCos<float>(0.0f);
        }
    }

    // --- Reset algorithm state ---
    cc.reset();
    hfi.reset();
    sub_step_             = 0u;
    active_buf_snapshot_  = 0u;
    state.samples_ready   = false;
}

// =============================================================================
// on_jeoc — ADC end-of-injected-conversion handler
// =============================================================================
//
// This function is called from the ADC JEOC ISR at the highest configured IRQ
// priority.  It must complete in well under one PWM half-period.
//
// Platform-specific ADC/timer register access is handled through thin inline
// helpers defined below.  On a real target these would read the actual
// peripheral registers; the portable stubs used here allow the code to compile
// and be unit-tested on a host.
//
// =============================================================================

// -----------------------------------------------------------------------------
// Platform stubs — replace with real register reads on target hardware.
// -----------------------------------------------------------------------------

/// @cond INTERNAL
#ifndef UNIMOC_TARGET_HW
// Host / test build: provide neutral stubs.

static uint32_t timer_ccr_shadow[3] = {2099u, 2099u, 2099u};

/// Read an ADC injected data register (0-based index 0..2).
static float adc_read_injected([[maybe_unused]] uint8_t rank)
{
    return 0.0f;  // Ia, Ib, Vdc all return 0 in stub
}

/// Write a timer compare register (1-based channel index 1..3).
static void timer_set_ccr([[maybe_unused]] uint8_t channel,
                          [[maybe_unused]] uint32_t value)
{
    if ((channel >= 1u) && (channel <= 3u))
    {
        timer_ccr_shadow[channel - 1u] = value;
    }
}

/// Read a timer compare register (1-based channel index 1..3).
static uint32_t timer_get_ccr([[maybe_unused]] uint8_t channel)
{
    if ((channel >= 1u) && (channel <= 3u))
    {
        return timer_ccr_shadow[channel - 1u];
    }
    return 0u;
}

#endif  // UNIMOC_TARGET_HW
/// @endcond

// -----------------------------------------------------------------------------
// Boundary guard threshold (5 % and 95 % of ARR)
// -----------------------------------------------------------------------------

static inline bool duty_in_bounds(const uint32_t ccr, const uint32_t arr) noexcept
{
    const uint32_t low  = arr / 20u;        // 5 %
    const uint32_t high = arr - arr / 20u;  // 95 %
    return (ccr >= low) && (ccr <= high);
}

// -----------------------------------------------------------------------------
// on_jeoc
// -----------------------------------------------------------------------------

void CurrentControlIsr::on_jeoc() noexcept
{
    // -------------------------------------------------------------------------
    // 1. Read ADC injected results (platform-specific on real target)
    // -------------------------------------------------------------------------
    const float i_a = adc_read_injected(0u);
    const float i_b = adc_read_injected(1u);
    const float v_dc = adc_read_injected(2u);

    // -------------------------------------------------------------------------
    // 2. Snapshot the active-buffer index at sub-step 0 and reuse it for the
    //    whole 4-step cycle so that a buffer flip mid-cycle is handled safely.
    // -------------------------------------------------------------------------
    if (sub_step_ == 0u)
    {
        active_buf_snapshot_ =
            state.double_buf.active.load(std::memory_order_acquire);
    }
    const uint8_t   ab      = active_buf_snapshot_;
    SubStepBuffer&  sb      = state.double_buf.buf[ab];
    const system::SinCos<float>& sc = sb.sc[sub_step_];

    // -------------------------------------------------------------------------
    // 3. Boundary guard — skip current PI and write neutral duties when any
    //    duty is outside the 5–95 % window (corrupted ADC samples or
    //    insufficient PWM headroom for voltage injection).
    // -------------------------------------------------------------------------
    const uint32_t arr = state.arr;
    {
        // Peek at the current CCR values to assess headroom.
        // On target this should map to TIMx->CCR1/2/3.
        const uint32_t ccr1 = timer_get_ccr(1u);
        const uint32_t ccr2 = timer_get_ccr(2u);
        const uint32_t ccr3 = timer_get_ccr(3u);

        if (!duty_in_bounds(ccr1, arr) ||
            !duty_in_bounds(ccr2, arr) ||
            !duty_in_bounds(ccr3, arr))
        {
            // Write safe neutral duties (50 %) and skip this control update.
            const uint32_t neutral = arr / 2u;
            timer_set_ccr(1u, neutral);
            timer_set_ccr(2u, neutral);
            timer_set_ccr(3u, neutral);
            sub_step_ = (sub_step_ + 1u) & 3u;
            if (sub_step_ == 0u)
            {
                state.samples_ready = true;
            }
            return;
        }
    }

    // -------------------------------------------------------------------------
    // 4. Clarke transform: I_a, I_b → I_α, I_β
    //    Using the two-sensor variant: I_c = −I_a − I_b
    // -------------------------------------------------------------------------
    const system::ThreePhase<float> i_abc{i_a, i_b, -i_a - i_b};
    const system::StatorReference<float> i_ab = i_abc.clark();

    // -------------------------------------------------------------------------
    // 5. Store current sample in the active buffer for SlowUpdate
    // -------------------------------------------------------------------------
    sb.i_ab_samples[sub_step_] = i_ab;

    // -------------------------------------------------------------------------
    // 6. Park transform: I_α, I_β → I_d, I_q
    // -------------------------------------------------------------------------
    const system::RotorReference<float> i_dq = i_ab.park(sc);

    // -------------------------------------------------------------------------
    // 7. Current PI with decoupling feedforward
    // -------------------------------------------------------------------------
    const system::RotorReference<float> u_dq =
        cc.update(state.i_ref, i_dq, mech_obs.omega, state.dt_fast);

    // Store for SlowUpdate (flux observer needs last voltage)
    state.u_dq_last = u_dq;

    // -------------------------------------------------------------------------
    // 8. HFI voltage injection (α/β frame, added before inverse Park)
    //    The injection voltage is computed from the current step's sin/cos.
    // -------------------------------------------------------------------------
    system::StatorReference<float> v_inj{0.0f, 0.0f};
    if (hfi_active)
    {
        v_inj = hfi.get_injection_voltage(sc.sin, sc.cos);
    }

    // -------------------------------------------------------------------------
    // 9. Inverse Park: U_d, U_q → U_α, U_β
    // -------------------------------------------------------------------------
    system::StatorReference<float> u_ab = u_dq.inverse_park(sc);

    // Add HFI injection in the α/β frame
    u_ab = u_ab + v_inj;

    // -------------------------------------------------------------------------
    // 10. Dead-time compensation (adds a correction in the α/β frame)
    // -------------------------------------------------------------------------
    u_ab = u_ab + dtc.calculate(i_ab);

    // -------------------------------------------------------------------------
    // 11. Space-vector modulation → normalised duties [0, 1]
    //     SVM expects the voltage vector normalised by V_dc.
    // -------------------------------------------------------------------------
    const float v_dc_safe = (v_dc > 1.0f) ? v_dc : 1.0f;  // prevent /0
    system::StatorReference<float> u_ab_norm{u_ab.alpha / v_dc_safe,
                                             u_ab.beta  / v_dc_safe};
    const system::ThreePhase<float> duties = svm.calculate(u_ab_norm);

    // -------------------------------------------------------------------------
    // 12. Write CCR registers directly (preload disabled, takes effect now)
    //     Duty is in [0, 1]; CCR = round(duty × ARR)
    // -------------------------------------------------------------------------
    const auto to_ccr = [arr](float duty) -> uint32_t
    {
        const float d = std::clamp(duty, 0.0f, 1.0f);
        return static_cast<uint32_t>(std::roundf(d * static_cast<float>(arr)));
    };

    timer_set_ccr(1u, to_ccr(duties.a));
    timer_set_ccr(2u, to_ccr(duties.b));
    timer_set_ccr(3u, to_ccr(duties.c));

    // -------------------------------------------------------------------------
    // 13. Advance sub-step; at wrap-around signal the slow-update task
    // -------------------------------------------------------------------------
    sub_step_ = (sub_step_ + 1u) & 3u;
    if (sub_step_ == 0u)
    {
        state.samples_ready = true;
    }
}

}  // namespace current_control
}  // namespace unimoc
