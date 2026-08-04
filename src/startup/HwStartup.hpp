/// \file HwStartup.hpp
/// Hardware startup-aid FSM for bring-up of new UNIMOC boards.
///
/// The FSM is split into two phases:
///   Phase 1 — No motor connected (safe ADC/PWM checks).
///   Phase 2 — Motor connected; uses very small voltages; OC-protected.
///
/// The FSM **never advances automatically**.  Each step waits until the user
/// explicitly calls request_next_step() (or writes 1 to the Cyphal register
/// `unimoc.startup.step`).  This gives the user time to observe scope
/// traces, enter external measurements, etc.
///
/// Usage (from the main application loop):
/// \code
///   // 1. Instantiate (once, alongside CurrentControlIsr)
///   HwStartup hw_startup{cc_isr, nvm_settings};
///
///   // 2. In the main loop, instead of slow_update.run_once():
///   if (hw_startup.is_active())
///       hw_startup.run_once();
///   else
///       slow_update.run_once();
///
///   // 3. Wire Cyphal handlers:
///   //    STARTUP_RUN write  → hw_startup.request_run() / request_abort()
///   //    STARTUP_STEP write → hw_startup.request_next_step()
///   //    STARTUP_EXT_CURRENT_A write → hw_startup.set_ext_current(value)
///   //    STARTUP_EXT_VDC_V write → hw_startup.set_ext_vdc(value)
/// \endcode
///
/// \copyright (c) Tecnologic SL
#pragma once

#include "StartupResults.hpp"
#include "../current_control/CurrentControlIsr.hpp"
#include "../../lib/system/NvmSettings.hpp"

namespace unimoc {
namespace startup {

// =============================================================================
// HwStartup
// =============================================================================

/// Hardware bring-up FSM.
class HwStartup
{
public:
    // =========================================================================
    // Configuration thresholds (public; override before calling request_run())
    // =========================================================================

    /// Maximum acceptable current-sense zero offset [A].  Default 0.5 A.
    float offset_threshold_A{0.5f};

    /// Maximum acceptable ADC noise RMS [A].  Default 0.1 A.
    float noise_threshold_A{0.1f};

    /// Current-sense gain tolerance (± from 1.0).  Default 0.05 = 5 %.
    float gain_tolerance{0.05f};

    /// V_dc gain tolerance.  Default 0.02 = 2 %.
    float vdc_gain_tolerance{0.02f};

    /// Minimum V_dc reading considered valid [V].
    float vdc_min_valid{1.0f};

    /// Fraction of max_phase_current_A used as OC trip limit during Phase 2.
    float oc_fraction{0.10f};

    /// Duty fraction applied in forced-duty steps (offset from 0.5).
    ///   LOW  → 0.5 − duty_step_fraction
    ///   HIGH → 0.5 + duty_step_fraction
    float duty_step_fraction{0.45f};  ///< gives 5 % / 95 % by default

    /// Voltage injection fraction for CURRENT_SENSE_CALIBRATION
    /// (fraction of ARR; tiny to avoid destroying low-Rs windings).
    float v_cal_fraction{0.002f};

    /// Number of ADC samples to average for offset/noise/gain calibrations.
    static constexpr uint32_t N_CAL = 1024u;

    /// Number of samples to hold for each DUTY_FORCE_* step.
    static constexpr uint32_t HOLD_SAMPLES = 1000u;

    /// Samples per alignment sweep position.
    static constexpr uint32_t N_ALIGN_SAMPLES = 256u;

    /// Number of ADC-trigger sweep positions (±5 µs in 0.5 µs steps = 21).
    static constexpr uint32_t N_SWEEP_STEPS = 21u;

    // =========================================================================
    // External measurement inputs
    // =========================================================================

    /// Set by the Cyphal handler for `unimoc.startup.ext_current_A`.
    void set_ext_current(float amps) noexcept { ext_current_A_ = amps; }

    /// Set by the Cyphal handler for `unimoc.startup.ext_vdc_V`.
    void set_ext_vdc(float volts) noexcept    { ext_vdc_V_     = volts; }

    // =========================================================================
    // Results (populated during the FSM run; persistent until next request_run)
    // =========================================================================
    StartupResults results{};

    // =========================================================================
    // Construction
    // =========================================================================

    /// Construct an HwStartup instance bound to a CurrentControlIsr and the
    /// live NvmSettings.
    ///
    /// \param cc   Reference to the active current-control ISR.
    /// \param nvm  Reference to the active NVM settings (updated with calibration
    ///             values at DONE).
    HwStartup(current_control::CurrentControlIsr& cc,
              system::NvmSettings&                nvm) noexcept
        : cc_{cc}, nvm_{nvm}
    {}

    // =========================================================================
    // Control interface (called from Cyphal register handlers)
    // =========================================================================

    /// Start the FSM from IDLE.  No-op if already running.
    void request_run() noexcept;

    /// Advance the FSM one step (only valid while step_done_ is true).
    void request_next_step() noexcept;

    /// Abort the FSM immediately; enter FAULT state.
    void request_abort() noexcept;

    // =========================================================================
    // Polling interface (called from the main loop)
    // =========================================================================

    /// Returns true while the FSM is actively running.
    /// Returns false when the FSM is IDLE (never started), DONE (completed),
    /// or FAULT (aborted), so that normal application tasks (SlowUpdate) can
    /// resume after the startup sequence finishes.
    bool is_active() const noexcept
    {
        return state_ != FsmState::IDLE
            && state_ != FsmState::DONE
            && state_ != FsmState::FAULT;
    }

    /// Returns the current FSM state.
    FsmState state() const noexcept { return state_; }

    /// Main polling function — call once per samples_ready pulse
    /// (i.e. at the same rate as SlowUpdate::run_once(); ~10 kHz).
    void run_once() noexcept;

private:
    // =========================================================================
    // Internal helpers
    // =========================================================================

    void transition_to(FsmState next) noexcept;
    void advance_to_next_state() noexcept;
    bool validate_current_state() noexcept;
    void enter_fault(const char* reason) noexcept;

    void run_adc_offset_cal() noexcept;
    void run_adc_noise_floor() noexcept;
    void run_duty_force(float duty) noexcept;
    void run_dc_link_voltage_check() noexcept;
    void run_gate_driver_enable_check() noexcept;
    void run_connect_motor_wait() noexcept;
    void run_phase_adc_alignment() noexcept;
    void run_current_sense_calibration() noexcept;
    void run_done() noexcept;

    bool check_motor_oc() noexcept;
    bool collect_sample(uint32_t n_samples) noexcept;

    static const char* state_name(FsmState s) noexcept;
    void               log_summary() noexcept;

    // =========================================================================
    // Data members
    // =========================================================================

    current_control::CurrentControlIsr& cc_;
    system::NvmSettings&                nvm_;

    FsmState state_{FsmState::IDLE};

    bool step_done_{false};          ///< Current step has finished collecting data.
    bool next_step_requested_{false};///< User requested advancement.

    uint32_t sample_count_{0u};      ///< Samples accumulated for current step.

    // Accumulators for running statistics
    double accum_a_{0.0};
    double accum_b_{0.0};
    double accum_sq_a_{0.0};
    double accum_sq_b_{0.0};

    // Alignment sweep
    uint32_t sweep_pos_{0u};
    float    sweep_noise_a_[N_SWEEP_STEPS]{};
    uint32_t base_trigger_offset_{0u};    ///< Initialised from cc_.state.adc_trigger_offset at sweep start.

    // Gate-driver enable-check accumulators
    float    gate_mean_disabled_{0.0f};
    float    gate_mean_enabled_{0.0f};
    bool     gate_phase_disabled_{true};  ///< true = collecting disabled phase

    // External measurements
    float    ext_current_A_{0.0f};
    float    ext_vdc_V_{0.0f};

    // OC limit (computed once at Phase 2 entry)
    float    oc_limit_A_{10.0f};
};

}  // namespace startup
}  // namespace unimoc
