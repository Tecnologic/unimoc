/// \file StartupResults.hpp
/// POD result struct and FSM-state enum for the hardware startup aid.
///
/// This file has no platform dependencies — it may be included from any
/// translation unit on both target and hosted environments.
///
/// \copyright (c) Tecnologic SL
#pragma once

#include <cstdint>

namespace unimoc {
namespace startup {

// =============================================================================
// FSM state enumeration
// =============================================================================

/// States of the hardware bring-up FSM.
///
/// Phase 1 (no motor connected): IDLE … GATE_DRIVER_ENABLE_CHECK
/// Phase 2 (motor connected, caution!): CONNECT_MOTOR … DONE
enum class FsmState : uint8_t
{
    IDLE                     = 0u,  ///< Not running; FSM is standby.
    PWM_DISABLE              = 1u,  ///< Gate drivers disabled; safe baseline.
    ADC_OFFSET_CAL           = 2u,  ///< Phase 1: measure current-sense zero offsets.
    ADC_NOISE_FLOOR          = 3u,  ///< Phase 1: measure ADC noise RMS.
    DUTY_FORCE_LOW           = 4u,  ///< Phase 1: apply 5 % duty on all phases.
    DUTY_FORCE_MID           = 5u,  ///< Phase 1: apply 50 % duty on all phases.
    DUTY_FORCE_HIGH          = 6u,  ///< Phase 1: apply 95 % duty on all phases.
    DC_LINK_VOLTAGE_CHECK    = 7u,  ///< Phase 1: compare V_dc ADC to external meter.
    GATE_DRIVER_ENABLE_CHECK = 8u,  ///< Phase 1: toggle gate-enable and verify ADC response.
    CONNECT_MOTOR            = 9u,  ///< Barrier: prompt user to connect motor; wait for step.
    PHASE_ADC_ALIGNMENT      = 10u, ///< Phase 2: sweep ADC trigger offset; find optimal point.
    CURRENT_SENSE_CALIBRATION= 11u, ///< Phase 2: force I_α; compare ADC vs external clamp meter.
    DONE                     = 12u, ///< All steps completed; results are logged and available.
    FAULT                    = 13u, ///< Aborted due to OC, overrange, or user abort.
    NUM_STATES               = 14u
};

// =============================================================================
// Per-step pass/fail
// =============================================================================

/// Total number of FSM states (includes IDLE, DONE, and FAULT).
/// The \c passed[] array is sized and indexed by FsmState value (0 … NUM_STATES-1);
/// IDLE, DONE, and FAULT entries are skipped when reporting results.
inline constexpr uint8_t NUM_STARTUP_STEPS = static_cast<uint8_t>(FsmState::NUM_STATES);

// =============================================================================
// StartupResults
// =============================================================================

/// Flat POD holding pass/fail flags and measured values for every startup step.
///
/// Written by HwStartup during each step.  Read-back is available via the
/// Cyphal `unimoc.startup.results` register and via RTT logging at DONE.
struct StartupResults
{
    // ---- per-step pass/fail (indexed by FsmState value) --------------------
    bool passed[NUM_STARTUP_STEPS]{};  ///< passed[i] = true iff step i passed.

    // ---- ADC_OFFSET_CAL results --------------------------------------------
    float adc_offset_a{0.0f};  ///< Measured phase-A zero offset [A].
    float adc_offset_b{0.0f};  ///< Measured phase-B zero offset [A].

    // ---- ADC_NOISE_FLOOR results -------------------------------------------
    float adc_noise_rms_a{0.0f};  ///< Phase-A ADC noise RMS [A].
    float adc_noise_rms_b{0.0f};  ///< Phase-B ADC noise RMS [A].

    // ---- DC_LINK_VOLTAGE_CHECK results -------------------------------------
    float gain_vdc{1.0f};  ///< Computed V_dc ADC gain correction [dimensionless].

    // ---- CURRENT_SENSE_CALIBRATION results ---------------------------------
    float gain_a{1.0f};  ///< Phase-A current-sense gain correction [dimensionless].
    float gain_b{1.0f};  ///< Phase-B current-sense gain correction [dimensionless].

    // ---- PHASE_ADC_ALIGNMENT results ----------------------------------------
    uint32_t adc_trigger_offset_optimal{168u};  ///< Optimal trigger offset [timer ticks].

    // ---- FSM bookkeeping ---------------------------------------------------
    FsmState current_state{FsmState::IDLE};  ///< Snapshot of last known state.
};

}  // namespace startup
}  // namespace unimoc
