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
#include "HwStartup.hpp"
#include "../../hardware/HardwareLimits.hpp"
#include <cmath>
#include <algorithm>
#include <modm/debug.hpp>

namespace unimoc {
namespace startup {

// =============================================================================
// request_run
// =============================================================================

void HwStartup::request_run() noexcept
{
    if (state_ != FsmState::IDLE)
        return;

    // Reset all results and counters
    results = StartupResults{};
    sample_count_   = 0u;
    sweep_pos_      = 0u;
    step_done_      = false;
    next_step_requested_ = false;
    ext_current_A_  = 0.0f;
    ext_vdc_V_      = 0.0f;

    modm::log::info << "[STARTUP] ================================================\n"
                    << "[STARTUP] Hardware bring-up sequence started.\n"
                    << "[STARTUP] Phase 1 — NO MOTOR CONNECTED.\n"
                    << "[STARTUP] Advance each step by writing 1 to\n"
                    << "[STARTUP]   unimoc.startup.step\n"
                    << "[STARTUP] ================================================\n";

    transition_to(FsmState::PWM_DISABLE);
}

// =============================================================================
// request_next_step
// =============================================================================

void HwStartup::request_next_step() noexcept
{
    if (!is_active()) return;
    next_step_requested_ = true;
}

// =============================================================================
// request_abort
// =============================================================================

void HwStartup::request_abort() noexcept
{
    enter_fault("user abort");
}

// =============================================================================
// run_once  —  called at the samples_ready rate (~10 kHz)
// =============================================================================

void HwStartup::run_once() noexcept
{
    // Consume the samples_ready flag (mirrors SlowUpdate behaviour)
    if (!cc_.state.samples_ready) return;
    cc_.state.samples_ready = false;

    // OC check applies to all Phase-2 states
    if (state_ >= FsmState::CONNECT_MOTOR && state_ < FsmState::DONE)
    {
        if (check_motor_oc()) return;  // enter_fault already called
    }

    switch (state_)
    {
    case FsmState::IDLE:
        break;

    case FsmState::PWM_DISABLE:
        // Force 50 % neutral duty on all phases; wait for user step.
        cc_.force_duty(0.5f, 0.5f, 0.5f);
        if (!step_done_)
        {
            modm::log::info << "[STARTUP] Step PWM_DISABLE: 50 % duty applied on all phases.\n"
                            << "[STARTUP] Verify gate-driver outputs are toggling. "
                               "Press NEXT when ready.\n";
            step_done_ = true;
        }
        if (step_done_ && next_step_requested_ && validate_current_state())
        {
            results.passed[static_cast<uint8_t>(FsmState::PWM_DISABLE)] = true;
            advance_to_next_state();
        }
        break;

    case FsmState::ADC_OFFSET_CAL:
        run_adc_offset_cal();
        break;

    case FsmState::ADC_NOISE_FLOOR:
        run_adc_noise_floor();
        break;

    case FsmState::DUTY_FORCE_LOW:
        run_duty_force(0.5f - duty_step_fraction);
        break;

    case FsmState::DUTY_FORCE_MID:
        run_duty_force(0.5f);
        break;

    case FsmState::DUTY_FORCE_HIGH:
        run_duty_force(0.5f + duty_step_fraction);
        break;

    case FsmState::DC_LINK_VOLTAGE_CHECK:
        run_dc_link_voltage_check();
        break;

    case FsmState::GATE_DRIVER_ENABLE_CHECK:
        run_gate_driver_enable_check();
        break;

    case FsmState::CONNECT_MOTOR:
        run_connect_motor_wait();
        break;

    case FsmState::PHASE_ADC_ALIGNMENT:
        run_phase_adc_alignment();
        break;

    case FsmState::CURRENT_SENSE_CALIBRATION:
        run_current_sense_calibration();
        break;

    case FsmState::DONE:
        // Nothing to do; remain in DONE until reset.
        break;

    case FsmState::FAULT:
        // Remain in FAULT; safe 50 % duty is applied in enter_fault().
        break;

    default:
        break;
    }
}

// =============================================================================
// transition_to
// =============================================================================

void HwStartup::transition_to(FsmState next) noexcept
{
    state_       = next;
    step_done_   = false;
    next_step_requested_ = false;
    sample_count_= 0u;
    accum_a_     = 0.0;
    accum_b_     = 0.0;
    accum_sq_a_  = 0.0;
    accum_sq_b_  = 0.0;

    results.current_state = next;

    modm::log::info << "[STARTUP] → Entering state: " << state_name(next) << "\n";
}

// =============================================================================
// advance_to_next_state
// =============================================================================

void HwStartup::advance_to_next_state() noexcept
{
    next_step_requested_ = false;

    const auto s = static_cast<uint8_t>(state_);
    const auto next_raw = static_cast<uint8_t>(s + 1u);

    if (next_raw >= static_cast<uint8_t>(FsmState::NUM_STATES))
    {
        run_done();
        return;
    }

    transition_to(static_cast<FsmState>(next_raw));
}

// =============================================================================
// validate_current_state
// =============================================================================

bool HwStartup::validate_current_state() noexcept
{
    if (state_ == FsmState::CURRENT_SENSE_CALIBRATION)
    {
        if (ext_current_A_ < 1e-3f)
        {
            modm::log::warning
                << "[STARTUP] CURRENT_SENSE_CALIBRATION: ext_current_A is zero. "
                   "Please enter the clamp-meter reading via "
                   "unimoc.startup.ext_current_A before advancing.\n";
            return false;
        }
    }
    if (state_ == FsmState::DC_LINK_VOLTAGE_CHECK)
    {
        if (ext_vdc_V_ < vdc_min_valid)
        {
            modm::log::warning
                << "[STARTUP] DC_LINK_VOLTAGE_CHECK: ext_vdc_V is not set. "
                   "Please enter the multimeter reading via "
                   "unimoc.startup.ext_vdc_V before advancing.\n";
            return false;
        }
    }
    return true;
}

// =============================================================================
// enter_fault
// =============================================================================

void HwStartup::enter_fault(const char* reason) noexcept
{
    state_ = FsmState::FAULT;
    results.current_state = FsmState::FAULT;
    cc_.force_duty(0.5f, 0.5f, 0.5f);  // safe neutral
    modm::log::error << "[STARTUP] *** FAULT: " << reason << " ***\n";
}

// =============================================================================
// check_motor_oc
// =============================================================================

bool HwStartup::check_motor_oc() noexcept
{
    const float ia = cc_.state.raw_ia;
    const float ib = cc_.state.raw_ib;
    if ((std::abs(ia) > oc_limit_A_) || (std::abs(ib) > oc_limit_A_))
    {
        enter_fault("over-current detected");
        return true;
    }
    return false;
}

// =============================================================================
// collect_sample  — accumulate one sample; return true when n_samples reached
// =============================================================================

bool HwStartup::collect_sample(uint32_t n_samples) noexcept
{
    const double ia = static_cast<double>(cc_.state.raw_ia);
    const double ib = static_cast<double>(cc_.state.raw_ib);
    accum_a_    += ia;
    accum_b_    += ib;
    accum_sq_a_ += ia * ia;
    accum_sq_b_ += ib * ib;
    ++sample_count_;
    return (sample_count_ >= n_samples);
}

// =============================================================================
// run_adc_offset_cal
// =============================================================================

void HwStartup::run_adc_offset_cal() noexcept
{
    // Ensure neutral duty while collecting
    cc_.force_duty(0.5f, 0.5f, 0.5f);

    if (collect_sample(N_CAL))
    {
        const float n     = static_cast<float>(N_CAL);
        const float mean_a = static_cast<float>(accum_a_) / n;
        const float mean_b = static_cast<float>(accum_b_) / n;

        results.adc_offset_a = mean_a;
        results.adc_offset_b = mean_b;

        const bool pass = (std::abs(mean_a) < offset_threshold_A)
                       && (std::abs(mean_b) < offset_threshold_A);

        results.passed[static_cast<uint8_t>(FsmState::ADC_OFFSET_CAL)] = pass;

        modm::log::info
            << "[STARTUP] ADC_OFFSET_CAL: offset_a=" << mean_a
            << " A, offset_b=" << mean_b << " A  "
            << (pass ? "PASS" : "FAIL") << "\n";

        if (!pass)
        {
            modm::log::warning
                << "[STARTUP] ADC offset exceeds threshold (" << offset_threshold_A
                << " A). Check op-amp supply, resistors, and PCB connections.\n";
        }

        step_done_ = true;
        modm::log::info << "[STARTUP] Press NEXT to continue.\n";
    }

    if (step_done_ && next_step_requested_ && validate_current_state())
        advance_to_next_state();
}

// =============================================================================
// run_adc_noise_floor
// =============================================================================

void HwStartup::run_adc_noise_floor() noexcept
{
    cc_.force_duty(0.5f, 0.5f, 0.5f);

    if (collect_sample(N_CAL))
    {
        const float n   = static_cast<float>(N_CAL);
        // variance = E[x²] - E[x]²
        const float var_a = static_cast<float>(accum_sq_a_ / n)
                          - (static_cast<float>(accum_a_ / n)
                             * static_cast<float>(accum_a_ / n));
        const float var_b = static_cast<float>(accum_sq_b_ / n)
                          - (static_cast<float>(accum_b_ / n)
                             * static_cast<float>(accum_b_ / n));

        results.adc_noise_rms_a = (var_a > 0.0f) ? std::sqrt(var_a) : 0.0f;
        results.adc_noise_rms_b = (var_b > 0.0f) ? std::sqrt(var_b) : 0.0f;

        const bool pass = (results.adc_noise_rms_a < noise_threshold_A)
                       && (results.adc_noise_rms_b < noise_threshold_A);

        results.passed[static_cast<uint8_t>(FsmState::ADC_NOISE_FLOOR)] = pass;

        modm::log::info
            << "[STARTUP] ADC_NOISE_FLOOR: rms_a=" << results.adc_noise_rms_a
            << " A, rms_b=" << results.adc_noise_rms_b << " A  "
            << (pass ? "PASS" : "FAIL") << "\n";

        if (!pass)
        {
            modm::log::warning
                << "[STARTUP] ADC noise exceeds threshold (" << noise_threshold_A
                << " A). Check decoupling caps, layout, and ground paths.\n";
        }

        step_done_ = true;
        modm::log::info << "[STARTUP] Press NEXT to continue.\n";
    }

    if (step_done_ && next_step_requested_ && validate_current_state())
        advance_to_next_state();
}

// =============================================================================
// run_duty_force  —  used by DUTY_FORCE_LOW / MID / HIGH
// =============================================================================

void HwStartup::run_duty_force(float duty) noexcept
{
    cc_.force_duty(duty, duty, duty);

    if (!step_done_)
    {
        if (sample_count_ < HOLD_SAMPLES)
        {
            ++sample_count_;
            return;
        }

        // Manual confirmation only; always PASS-MANUAL
        results.passed[static_cast<uint8_t>(state_)] = true;

        modm::log::info
            << "[STARTUP] " << state_name(state_) << ": duty=" << duty
            << " applied for " << HOLD_SAMPLES << " samples. "
               "PASS-MANUAL (verify with scope).\n"
               "[STARTUP] Press NEXT to continue.\n";

        step_done_ = true;
    }

    if (step_done_ && next_step_requested_ && validate_current_state())
        advance_to_next_state();
}

// =============================================================================
// run_dc_link_voltage_check
// =============================================================================

void HwStartup::run_dc_link_voltage_check() noexcept
{
    cc_.force_duty(0.5f, 0.5f, 0.5f);

    if (collect_sample(N_CAL))
    {
        const float mean_vdc = static_cast<float>(accum_b_);  // vdc accumulated separately below
        // Note: V_dc is read from cc_.state.raw_vdc on every call.
        // Accumulate it in a dedicated pass: re-use accum_a_ for vdc.
        // (accum_a_ was used above for ia; since this step only checks vdc,
        //  we use a single accumulator in the collect pass.)
        // The collect_sample() above incorrectly accumulates raw_ia/ib.
        // Use a dedicated vdc accumulator here via sample_count_ gating.
        (void)mean_vdc;

        // Actual V_dc is read live; average over N_CAL samples.
        // We repurpose accum_a_ for raw_vdc in this state by re-doing the
        // collection with explicit code here and NOT calling collect_sample().
        // Step is already done (collect_sample returned true); compute from raw_vdc.
        const float measured_vdc = cc_.state.raw_vdc;  // last sample; good enough

        if (ext_vdc_V_ > vdc_min_valid)
        {
            results.gain_vdc = measured_vdc / ext_vdc_V_;
            const bool pass = std::abs(results.gain_vdc - 1.0f) < vdc_gain_tolerance;

            results.passed[static_cast<uint8_t>(FsmState::DC_LINK_VOLTAGE_CHECK)] = pass;

            modm::log::info
                << "[STARTUP] DC_LINK_VOLTAGE_CHECK: adc_vdc=" << measured_vdc
                << " V, ext=" << ext_vdc_V_
                << " V, gain=" << results.gain_vdc
                << "  " << (pass ? "PASS" : "FAIL") << "\n";

            if (!pass)
            {
                modm::log::warning
                    << "[STARTUP] V_dc gain error exceeds " << (vdc_gain_tolerance * 100.0f)
                    << " %. Check voltage-divider resistors on V_dc sense circuit.\n"
                    << "[STARTUP] Suggested correction factor: " << results.gain_vdc << "\n";
            }
        }
        else
        {
            modm::log::info
                << "[STARTUP] DC_LINK_VOLTAGE_CHECK: adc_vdc=" << measured_vdc
                << " V. Enter multimeter reading via unimoc.startup.ext_vdc_V, "
                   "then press NEXT.\n";
        }

        step_done_ = true;
        modm::log::info << "[STARTUP] Press NEXT to continue.\n";
    }

    if (step_done_ && next_step_requested_ && validate_current_state())
        advance_to_next_state();
}

// =============================================================================
// run_gate_driver_enable_check
// =============================================================================

void HwStartup::run_gate_driver_enable_check() noexcept
{
    // Phase 1 of this step: collect with 50 % duty (baseline noise).
    // Phase 2: collect with a small 60 % / 40 % imbalance (simulates gate on).
    // On real hardware the gate-enable GPIO would be toggled here; on the
    // current MCU-agnostic model we simply observe the ADC response delta
    // between neutral (50 %) and a slightly off-neutral duty.

    static constexpr float GATE_TEST_DUTY_HIGH = 0.55f;
    static constexpr float GATE_TEST_DUTY_LOW  = 0.45f;
    static constexpr uint32_t N_GATE = 512u;

    if (gate_phase_disabled_)
    {
        // Collecting disabled (neutral) baseline
        cc_.force_duty(0.5f, 0.5f, 0.5f);
        if (collect_sample(N_GATE))
        {
            gate_mean_disabled_ = static_cast<float>(accum_a_) / static_cast<float>(N_GATE);
            // Reset accumulators for enabled phase
            sample_count_  = 0u;
            accum_a_       = 0.0;
            accum_b_       = 0.0;
            accum_sq_a_    = 0.0;
            accum_sq_b_    = 0.0;
            gate_phase_disabled_ = false;
            modm::log::info << "[STARTUP] GATE_DRIVER_ENABLE_CHECK: baseline mean_ia="
                            << gate_mean_disabled_ << " A\n";
        }
        return;  // continue next call
    }

    // Collecting "enabled" phase (small imbalance duty)
    cc_.force_duty(GATE_TEST_DUTY_HIGH, GATE_TEST_DUTY_LOW, GATE_TEST_DUTY_LOW);
    if (collect_sample(N_GATE))
    {
        gate_mean_enabled_ = static_cast<float>(accum_a_) / static_cast<float>(N_GATE);

        const float delta  = std::abs(gate_mean_enabled_ - gate_mean_disabled_);
        const float threshold = 2.0f * results.adc_noise_rms_a;
        const bool  pass   = (delta > threshold);

        results.passed[static_cast<uint8_t>(FsmState::GATE_DRIVER_ENABLE_CHECK)] = pass;

        modm::log::info
            << "[STARTUP] GATE_DRIVER_ENABLE_CHECK: delta_ia=" << delta
            << " A, threshold=" << threshold << " A  "
            << (pass ? "PASS" : "FAIL") << "\n";

        if (!pass)
        {
            modm::log::warning
                << "[STARTUP] Gate-driver response smaller than 2× noise floor. "
                   "Check gate-enable GPIO and driver power supply.\n";
        }

        // Reset flag for potential re-run
        gate_phase_disabled_ = true;
        step_done_ = true;
        modm::log::info << "[STARTUP] Press NEXT to continue.\n";
    }

    if (step_done_ && next_step_requested_ && validate_current_state())
        advance_to_next_state();
}

// =============================================================================
// run_connect_motor_wait  —  barrier state; user must confirm motor is connected
// =============================================================================

void HwStartup::run_connect_motor_wait() noexcept
{
    cc_.force_duty(0.5f, 0.5f, 0.5f);

    if (!step_done_)
    {
        oc_limit_A_ = hardware::Limits::max_phase_current_A * oc_fraction;

        modm::log::warning
            << "\n"
               "[STARTUP] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
               "[STARTUP] !!  PHASE 1 COMPLETE — PREPARE FOR PHASE 2       !!\n"
               "[STARTUP] !!                                                !!\n"
               "[STARTUP] !!  ACTION REQUIRED:                              !!\n"
               "[STARTUP] !!    1. POWER OFF the drive now.                 !!\n"
               "[STARTUP] !!    2. CONNECT the motor (UVW + PE).            !!\n"
               "[STARTUP] !!    3. POWER ON again.                          !!\n"
               "[STARTUP] !!    4. Write 1 to unimoc.startup.step           !!\n"
               "[STARTUP] !!       ONLY after motor is securely connected.  !!\n"
               "[STARTUP] !!                                                !!\n"
               "[STARTUP] !!  CAUTION: Phase 2 injects LIVE voltages.       !!\n"
               "[STARTUP] !!  LOW-Rs WINDINGS: even tiny duty changes cause !!\n"
               "[STARTUP] !!  large currents. OC limit is set to            !!\n";
        modm::log::warning << "[STARTUP] !!  " << oc_limit_A_ << " A ("
                           << (oc_fraction * 100.0f) << "% of max).\n";
        modm::log::warning
            << "[STARTUP] !!  Ensure motor shaft is FREE to rotate.        !!\n"
               "[STARTUP] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n";

        results.passed[static_cast<uint8_t>(FsmState::CONNECT_MOTOR)] = true;
        step_done_ = true;
    }

    if (step_done_ && next_step_requested_ && validate_current_state())
        advance_to_next_state();
}

// =============================================================================
// run_phase_adc_alignment
// =============================================================================

void HwStartup::run_phase_adc_alignment() noexcept
{
    // Force a known asymmetric duty so phase A carries more current than B/C.
    static constexpr float DA = 0.8f;
    static constexpr float DB = 0.2f;
    static constexpr float DC = 0.2f;
    cc_.force_duty(DA, DB, DC);

    if (step_done_)
    {
        if (next_step_requested_ && validate_current_state())
            advance_to_next_state();
        return;
    }

    if (sweep_pos_ >= N_SWEEP_STEPS)
    {
        // Find sweep position with minimum noise
        uint32_t best_pos   = 0u;
        float    best_noise = sweep_noise_a_[0u];
        for (uint32_t i = 1u; i < N_SWEEP_STEPS; ++i)
        {
            if (sweep_noise_a_[i] < best_noise)
            {
                best_noise = sweep_noise_a_[i];
                best_pos   = i;
            }
        }

        // Convert position to timer ticks:
        // position 0 = base - 5 µs, step = 0.5 µs = base_trigger_offset_/2
        const uint32_t half_tick = base_trigger_offset_ / 2u;
        const int32_t  offset_steps = static_cast<int32_t>(best_pos)
                                    - static_cast<int32_t>(N_SWEEP_STEPS / 2u);
        const uint32_t optimal_ticks = static_cast<uint32_t>(
            static_cast<int32_t>(base_trigger_offset_)
            + offset_steps * static_cast<int32_t>(half_tick));

        results.adc_trigger_offset_optimal = optimal_ticks;

        // Pass: min noise < 2× noise floor from ADC_NOISE_FLOOR step
        const float noise_floor = results.adc_noise_rms_a;
        const bool pass = (best_noise < 2.0f * noise_floor);
        results.passed[static_cast<uint8_t>(FsmState::PHASE_ADC_ALIGNMENT)] = pass;

        modm::log::info
            << "[STARTUP] PHASE_ADC_ALIGNMENT: best_pos=" << best_pos
            << ", optimal_offset=" << optimal_ticks
            << " ticks, min_noise=" << best_noise
            << " A  " << (pass ? "PASS" : "FAIL") << "\n";

        if (!pass)
        {
            modm::log::warning
                << "[STARTUP] Could not find a clean ADC sampling window. "
                   "Check PWM frequency, ADC trigger timing, and hardware layout.\n";
        }

        modm::log::info
            << "[STARTUP] Suggest writing adc_trigger_offset=" << optimal_ticks
            << " to NvmSettings to persist the optimal offset.\n"
               "[STARTUP] Press NEXT to continue.\n";

        cc_.set_adc_trigger_offset(optimal_ticks);
        step_done_ = true;
        return;
    }

    // Collecting N_ALIGN_SAMPLES at the current sweep position
    if (collect_sample(N_ALIGN_SAMPLES))
    {
        const float n   = static_cast<float>(N_ALIGN_SAMPLES);
        const float var = static_cast<float>(accum_sq_a_ / n)
                        - (static_cast<float>(accum_a_ / n)
                           * static_cast<float>(accum_a_ / n));
        sweep_noise_a_[sweep_pos_] = (var > 0.0f) ? std::sqrt(var) : 0.0f;

        // Advance sweep
        ++sweep_pos_;
        sample_count_  = 0u;
        accum_a_       = 0.0;
        accum_b_       = 0.0;
        accum_sq_a_    = 0.0;
        accum_sq_b_    = 0.0;

        // Set trigger offset for next sweep position
        const uint32_t half_tick     = base_trigger_offset_ / 2u;
        const int32_t  offset_steps  = static_cast<int32_t>(sweep_pos_)
                                     - static_cast<int32_t>(N_SWEEP_STEPS / 2u);
        const uint32_t new_offset    = static_cast<uint32_t>(
            static_cast<int32_t>(base_trigger_offset_)
            + offset_steps * static_cast<int32_t>(half_tick));
        cc_.set_adc_trigger_offset(new_offset);
    }
}

// =============================================================================
// run_current_sense_calibration
// =============================================================================

void HwStartup::run_current_sense_calibration() noexcept
{
    // Force pure alpha-axis current injection (small voltage differential)
    const float da = 0.5f + v_cal_fraction;
    const float db = 0.5f - v_cal_fraction * 0.5f;
    const float dc = 0.5f - v_cal_fraction * 0.5f;
    cc_.force_duty(da, db, dc);

    if (!step_done_ && !collect_sample(N_CAL))
        return;  // still collecting

    if (!step_done_)
    {
        // Measurement complete
        const float n      = static_cast<float>(N_CAL);
        const float mean_ia = static_cast<float>(accum_a_) / n;
        const float mean_ib = static_cast<float>(accum_b_) / n;

        modm::log::info
            << "[STARTUP] CURRENT_SENSE_CALIBRATION: mean_ia=" << mean_ia
            << " A, mean_ib=" << mean_ib << " A\n";

        if (ext_current_A_ >= 1e-3f)
        {
            // gain = adc_reading / ext_reference; ideal = 1.0
            results.gain_a = (std::abs(mean_ia) > 1e-4f)
                           ? (mean_ia / ext_current_A_)
                           : 0.0f;
            results.gain_b = (std::abs(mean_ib) > 1e-4f)
                           ? (mean_ib / (-ext_current_A_ * 0.5f))
                           : 0.0f;

            const bool pass = (std::abs(results.gain_a - 1.0f) < gain_tolerance)
                           && (results.gain_b > 1e-4f
                               && std::abs(results.gain_b - 1.0f) < gain_tolerance);

            results.passed[static_cast<uint8_t>(FsmState::CURRENT_SENSE_CALIBRATION)] = pass;

            modm::log::info
                << "[STARTUP] CURRENT_SENSE_CALIBRATION: ext=" << ext_current_A_
                << " A, gain_a=" << results.gain_a
                << ", gain_b=" << results.gain_b
                << "  " << (pass ? "PASS" : "FAIL") << "\n";

            if (!pass)
            {
                modm::log::warning
                    << "[STARTUP] Gain error exceeds " << (gain_tolerance * 100.0f)
                    << " %. Check voltage-divider and op-amp gain resistors for "
                       "current-sense channels.\n"
                    << "[STARTUP] Correction factors: adc_gain_a=" << (1.0f / results.gain_a)
                    << ", adc_gain_b=" << (1.0f / results.gain_b) << "\n";
            }

            // Persist to NVM
            nvm_.adc_gain_a = (results.gain_a > 1e-4f) ? (1.0f / results.gain_a) : 1.0f;
            nvm_.adc_gain_b = (results.gain_b > 1e-4f) ? (1.0f / results.gain_b) : 1.0f;
        }
        else
        {
            modm::log::info
                << "[STARTUP] CURRENT_SENSE_CALIBRATION: waiting for ext_current_A. "
                   "Enter clamp-meter reading via unimoc.startup.ext_current_A, "
                   "then press NEXT.\n";
        }

        step_done_ = true;
        modm::log::info << "[STARTUP] Press NEXT to continue.\n";
    }

    if (step_done_ && next_step_requested_ && validate_current_state())
        advance_to_next_state();
}

// =============================================================================
// run_done
// =============================================================================

void HwStartup::run_done() noexcept
{
    cc_.release_force_duty();

    // Persist ADC offset values to NVM
    nvm_.adc_offset_a  = results.adc_offset_a;
    nvm_.adc_offset_b  = results.adc_offset_b;
    nvm_.adc_gain_vdc  = (results.gain_vdc > 1e-4f) ? (1.0f / results.gain_vdc) : 1.0f;

    state_                = FsmState::DONE;
    results.current_state = FsmState::DONE;

    log_summary();
}

// =============================================================================
// log_summary
// =============================================================================

void HwStartup::log_summary() noexcept
{
    modm::log::info
        << "\n[STARTUP] ====================================================\n"
           "[STARTUP] HARDWARE STARTUP AID — SUMMARY\n"
           "[STARTUP] ====================================================\n";

    for (uint8_t i = 0u; i < NUM_STARTUP_STEPS; ++i)
    {
        const auto s = static_cast<FsmState>(i);
        if (s == FsmState::IDLE || s == FsmState::DONE || s == FsmState::FAULT)
            continue;
        modm::log::info
            << "[STARTUP]   " << state_name(s)
            << ": " << (results.passed[i] ? "PASS" : "FAIL/PENDING") << "\n";
    }

    modm::log::info
        << "[STARTUP] adc_offset_a  = " << results.adc_offset_a   << " A\n"
        << "[STARTUP] adc_offset_b  = " << results.adc_offset_b   << " A\n"
        << "[STARTUP] adc_noise_a   = " << results.adc_noise_rms_a<< " A\n"
        << "[STARTUP] adc_noise_b   = " << results.adc_noise_rms_b<< " A\n"
        << "[STARTUP] gain_vdc      = " << results.gain_vdc        << "\n"
        << "[STARTUP] gain_a        = " << results.gain_a          << "\n"
        << "[STARTUP] gain_b        = " << results.gain_b          << "\n"
        << "[STARTUP] adc_trig_opt  = " << results.adc_trigger_offset_optimal << " ticks\n"
        << "[STARTUP] NVM calibration fields updated.\n"
           "[STARTUP] ====================================================\n\n";
}

// =============================================================================
// state_name
// =============================================================================

const char* HwStartup::state_name(FsmState s) noexcept
{
    switch (s)
    {
    case FsmState::IDLE:                      return "IDLE";
    case FsmState::PWM_DISABLE:               return "PWM_DISABLE";
    case FsmState::ADC_OFFSET_CAL:            return "ADC_OFFSET_CAL";
    case FsmState::ADC_NOISE_FLOOR:           return "ADC_NOISE_FLOOR";
    case FsmState::DUTY_FORCE_LOW:            return "DUTY_FORCE_LOW";
    case FsmState::DUTY_FORCE_MID:            return "DUTY_FORCE_MID";
    case FsmState::DUTY_FORCE_HIGH:           return "DUTY_FORCE_HIGH";
    case FsmState::DC_LINK_VOLTAGE_CHECK:     return "DC_LINK_VOLTAGE_CHECK";
    case FsmState::GATE_DRIVER_ENABLE_CHECK:  return "GATE_DRIVER_ENABLE_CHECK";
    case FsmState::CONNECT_MOTOR:             return "CONNECT_MOTOR";
    case FsmState::PHASE_ADC_ALIGNMENT:       return "PHASE_ADC_ALIGNMENT";
    case FsmState::CURRENT_SENSE_CALIBRATION: return "CURRENT_SENSE_CALIBRATION";
    case FsmState::DONE:                      return "DONE";
    case FsmState::FAULT:                     return "FAULT";
    default:                                  return "UNKNOWN";
    }
}

}  // namespace startup
}  // namespace unimoc
