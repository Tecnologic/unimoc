/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file nvm_settings.hpp
 *    @brief Typed persistent settings for UNIMOC.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include "cyphal/control_mode.hpp"
#include "cyphal/node_identity.hpp"
#include "motor_type.hpp"
#include "units.hpp"

// ============================================================================
// Standalone enums (declared before NvmSettings so they can be used as
// member types with in-class initialisers).
// ============================================================================

/**
 * @namespace unimoc global namespace
 */
namespace unimoc {
/**
 * @namespace settings persistent motor-control settings
 */
namespace settings {

using namespace unit;

/// Magic number stored at the start of every NvmSettings block.
/// Used to detect an uninitialised or corrupt NVM image.
inline constexpr uint32_t kNvmMagic = 0x554D4F43u;  // 'UMOC'

/// Layout version of the NvmSettings struct.
/// Increment whenever the struct layout changes incompatibly.
inline constexpr uint16_t kNvmVersion = 3u;

/**
 * @brief Aggregate of all persistent (NVM-backed) UNIMOC configuration.
 *
 * Every member that a user can configure via Cyphal registers and that must
 * survive a power cycle lives in this struct.  The hardware NVM driver
 * stores and retrieves this struct atomically (e.g., via a CRC-protected
 * flash page or EEPROM block).
 *
 * Cyphal register mapping
 * -----------------------
 * Each field corresponds to a named Cyphal register (see cyphal_interface.hpp
 * for the canonical register name strings).  The Cyphal register service
 * (`uavcan.register.Access`) is the write path; on successful write the
 * application flushes the updated NvmSettings to non-volatile storage.
 *
 * Node ID and plug-and-play
 * -------------------------
 * `node_id == 0` is the sentinel value meaning "not assigned".  When the
 * firmware boots with `node_id == 0` it initiates the Cyphal plug-and-play
 * allocation sequence (`uavcan.pnp.NodeIDAllocationData`) and stores the
 * allocated ID in this field for future boots.
 *
 * Default values
 * --------------
 * All fields are initialised to sensible defaults so that an unprogrammed
 * device boots and operates in a known state without manual configuration.
 */
struct NvmSettings {
  // =========================================================================
  // Header — must be first
  // =========================================================================

  /// Magic word — set to kNvmMagic by factory defaults.
  uint32_t magic{kNvmMagic};

  /// Layout version — set to kNvmVersion.
  uint16_t version{kNvmVersion};

  // =========================================================================
  // Node identification
  // =========================================================================

  /// Cyphal node ID [1..127].  0 = not assigned → use PnP allocation.
  /// Register: `uavcan.node.id`
  uint8_t node_id{0};

  /// Human-readable node identity (name + hw/sw versions).
  /// Register: `uavcan.node.description` (name field only over Cyphal).
  cyphal::NodeIdentity identity{};

  // =========================================================================
  // System
  // =========================================================================

  /// Active motor type.
  /// Register: `unimoc.motor.type`
  system::MotorType motor_type{system::MotorType::PMSM};

  /// Motor pole-pair count.
  /// Register: `unimoc.motor.pole_pairs`
  uint8_t pole_pairs{1};

  /// Initial control mode after boot (overridden at runtime via Cyphal).
  /// Register: `unimoc.control.mode`
  cyphal::ControlMode control_mode{cyphal::ControlMode::TORQUE};

  // =========================================================================
  // Stator / winding parameters (all motor types)
  // =========================================================================

  /// Stator phase resistance [Ω].
  /// Register: `unimoc.motor.stator.R`
  unit::Resistance stator_r{0.1_Ohm};

  /// Stator inductance [H] (L_q for IPMSM; average (L_d+L_q)/2 otherwise).
  /// Register: `unimoc.motor.stator.L`
  unit::Inductance stator_l{1.0_mH};

  // =========================================================================
  // Motor operating limits
  //
  // These are runtime-configurable limits; they must never exceed the
  // immutable capabilities supplied by the selected hardware profile.
  // =========================================================================

  /// Maximum motor (resultant stator vector) current [A].
  /// Hard upper bound: hardware profile's maximum motor current.
  /// Register: `unimoc.motor.limits.i_max`
  unit::Current motor_i_max{40.0_A};

  /// Maximum electrical angular velocity in the forward direction [rad/s].
  /// Register: `unimoc.motor.limits.omega_max`
  unit::AngularVelocity motor_omega_max{2000.0_rad_per_s};

  /// Maximum electrical angular velocity in the reverse direction [rad/s]
  /// (stored as a negative value).
  /// Register: `unimoc.motor.limits.omega_min`
  unit::AngularVelocity motor_omega_min{-2000.0_rad_per_s};

  /// Maximum battery discharge (drive) current [A].
  /// Hard upper bound: hardware profile's maximum drive current.
  /// Register: `unimoc.battery.limits.drive_current`
  unit::Current battery_drive_current_max{15.0_A};

  /// Maximum battery charge (regenerative braking) current [A].
  /// Hard upper bound: hardware profile's maximum charge current.
  /// Register: `unimoc.battery.limits.charge_current`
  unit::Current battery_charge_current_max{5.0_A};

  // =========================================================================
  // PMSM / EESM parameters
  // =========================================================================

  /// Permanent-magnet flux linkage ψ_PM [Wb].
  /// For EESM, this is the initial/nominal ψ_f used until the excitation
  /// observer provides a measured value.
  /// Register: `unimoc.motor.pmsm.flux_pm`
  unit::MagneticFlux flux_pm{0.0_Wb};

  /// d-axis inductance L_d [H].
  /// Register: `unimoc.motor.pmsm.L_d`
  unit::Inductance l_d{1.0_mH};

  /// q-axis inductance L_q [H].
  /// Register: `unimoc.motor.pmsm.L_q`
  unit::Inductance l_q{1.0_mH};

  // =========================================================================
  // Asynchronous motor (ASM) parameters
  // =========================================================================

  /// Rotor resistance R_r [Ω].
  /// Register: `unimoc.motor.asm.R_r`
  unit::Resistance asm_r_r{0.3_Ohm};

  /// Stator resistance R_s [Ω].
  /// Register: `unimoc.motor.asm.R_s`
  unit::Resistance asm_r_s{0.5_Ohm};

  /// Stator self-inductance L_s [H].
  /// Register: `unimoc.motor.asm.L_s`
  unit::Inductance asm_l_s{50.0_mH};

  /// Rotor self-inductance L_r [H].
  /// Register: `unimoc.motor.asm.L_r`
  unit::Inductance asm_l_r{50.0_mH};

  /// Mutual (magnetising) inductance L_m [H].
  /// Register: `unimoc.motor.asm.L_m`
  unit::Inductance asm_l_m{47.0_mH};

  // =========================================================================
  // Mechanical observer (Kalman filter) parameters
  // =========================================================================

  /// Rotor + load inertia J [kg·m²].
  /// Used by MechanicalObserver::predict() to integrate the torque equation.
  /// Register: `unimoc.motor.mechanics.J`
  unit::Inertia motor_j{1.0e-4_kg_m2};

  /// Kalman filter process noise variance Q.
  /// Increase to let the filter track faster but noisier.
  /// Register: `unimoc.observer.mech.Q`
  float mech_obs_q{1e-5F};

  /// Kalman filter measurement noise variance R.
  /// Increase to smooth angle corrections at the cost of slower tracking.
  /// Register: `unimoc.observer.mech.R`
  float mech_obs_r{1e-4F};

  // =========================================================================
  // PMSM flux observer (PmsmFluxObserver) gains
  // =========================================================================

  /// d-axis anti-drift feedback gain C_d [1/s].
  /// Register: `unimoc.observer.pmsm_flux.C_d`
  unit::InverseTime pmsm_flux_obs_c_d{50.0_per_s};

  /// q-axis anti-drift feedback gain C_q [1/s].
  /// Register: `unimoc.observer.pmsm_flux.C_q`
  unit::InverseTime pmsm_flux_obs_c_q{1.0_per_s};

  // =========================================================================
  // ASM flux observer gains
  // =========================================================================

  /// Stator-current correction gain g_i [1/s].
  /// Register: `unimoc.observer.asm_flux.g_i`
  unit::InverseTime asm_obs_g_i{500.0_per_s};

  /// Rotor-flux correction gain g_flux [Wb/(A·s)].
  /// Register: `unimoc.observer.asm_flux.g_flux`
  unit::MagneticFluxPerCurrentTime asm_obs_g_flux{5000.0_Wb_per_A_s};

  // =========================================================================
  // ASM flux controller
  // =========================================================================

  /// Proportional gain [A/Wb].
  /// Register: `unimoc.control.asm_flux.kp`
  unit::CurrentPerMagneticFlux asm_flux_kp{10.0_A_per_Wb};

  /// Integral gain [A/(Wb·s)].
  /// Register: `unimoc.control.asm_flux.ki`
  unit::CurrentPerMagneticFluxTime asm_flux_ki{50.0_A_per_Wb_s};

  /// Minimum d-axis current [A].
  /// Register: `unimoc.control.asm_flux.i_d_min`
  unit::Current asm_flux_i_d_min{0.0_A};

  /// Maximum d-axis current [A].
  /// Register: `unimoc.control.asm_flux.i_d_max`
  unit::Current asm_flux_i_d_max{10.0_A};

  // =========================================================================
  // Field weakening
  // =========================================================================

  /// Maximum voltage vector magnitude (normalised, range (0,1]).
  /// Register: `unimoc.control.fw.v_max`
  unit::Voltage fw_v_max{0.9_V};

  /// Integrator gain [A/(V·s)].
  /// Register: `unimoc.control.fw.ki`
  unit::CurrentPerVoltageTime fw_ki{10.0_A_per_V_s};

  /// Most negative i_d allowed [A].
  /// Register: `unimoc.control.fw.i_d_min`
  unit::Current fw_i_d_min{-10.0_A};

  // =========================================================================
  // PWM frequency
  // =========================================================================

  /// PWM switching frequency for the current-control timer.
  ///
  /// The ISR fires at 2× this frequency (once at peak, once at trough of the
  /// centre-aligned timer) so the effective current-loop rate is 2 × f_pwm.
  /// The slow-update task runs at (2 × f_pwm) / 4 = f_pwm / 2.
  ///
  /// Register: `unimoc.control.pwm_frequency`
  unit::Frequency pwm_frequency{20.0_kHz};

  // =========================================================================
  // Current controller (d/q-axis PI with cross-coupling feedforward)
  // =========================================================================

  /// d-axis proportional gain [V/A].
  /// Register: `unimoc.control.current.kp_d`
  unit::VoltagePerCurrent current_kp_d{1.0_V_per_A};

  /// d-axis integral gain [V/(A·s)].
  /// Register: `unimoc.control.current.ki_d`
  unit::VoltagePerCurrentTime current_ki_d{100.0_V_per_A_s};

  /// q-axis proportional gain [V/A].
  /// Register: `unimoc.control.current.kp_q`
  unit::VoltagePerCurrent current_kp_q{1.0_V_per_A};

  /// q-axis integral gain [V/(A·s)].
  /// Register: `unimoc.control.current.ki_q`
  unit::VoltagePerCurrentTime current_ki_q{100.0_V_per_A_s};

  /// Maximum voltage vector magnitude (normalised by V_dc, range (0, 1]).
  /// Limits the current controller output before it reaches the SVM
  /// modulator.  Typically set slightly below the SVM duty_max to preserve
  /// headroom for dead-time compensation and ADC sampling.
  /// Register: `unimoc.control.current.v_max`
  unit::Ratio current_v_max{0.9_ratio};

  // =========================================================================
  // SVM modulator
  // =========================================================================

  /// Minimum duty cycle (headroom for ADC + dead-time).
  /// Register: `unimoc.control.svm.duty_min`
  unit::Ratio svm_duty_min{0.05_ratio};

  /// Maximum duty cycle.
  /// Register: `unimoc.control.svm.duty_max`
  unit::Ratio svm_duty_max{0.95_ratio};

  // =========================================================================
  // Dead-time compensation
  // =========================================================================

  /// Gate-driver dead time [s].
  /// Register: `unimoc.control.dtc.dead_time`
  unit::Time dtc_dead_time{0.0_s};

  /// PWM switching frequency [Hz].
  /// Register: `unimoc.control.dtc.f_pwm`
  unit::Frequency dtc_f_pwm{10.0_kHz};

  /// Phase-current zero-crossing threshold [A].
  /// Register: `unimoc.control.dtc.i_threshold`
  unit::Current dtc_i_threshold{0.1_A};

  // =========================================================================
  // HFI (high-frequency injection) observer
  // =========================================================================

  /// HFI injection voltage [V].
  /// Register: `unimoc.observer.hfi.v_inject`
  unit::Voltage hfi_v_inject{0.0_V};

  /// Current error to PLL angle scaling gain [rad/A].
  /// Register: `unimoc.observer.hfi.error_gain`
  unit::AnglePerCurrent hfi_error_gain{1.0_rad_per_A};

  // =========================================================================
  // EESM excitation controller
  // =========================================================================

  /// Excitation mode: 0 = CurrentMode, 1 = FluxMode.
  /// Register: `unimoc.control.excitation.mode`
  uint8_t excitation_mode{0};

  /// Mutual inductance L_m [H] for flux↔current conversion.
  /// Register: `unimoc.control.excitation.L_m`
  unit::Inductance excitation_l_m{47.0_mH};

  /// Proportional gain [V/A].
  /// Register: `unimoc.control.excitation.kp`
  unit::Ratio excitation_kp{5.0_ratio};

  /// Integral gain [V/(A·s)].
  /// Register: `unimoc.control.excitation.ki`
  unit::InverseTime excitation_ki{50.0_per_s};

  /// Minimum rotor excitation current [A].
  /// Register: `unimoc.control.excitation.i_f_min`
  unit::Current excitation_i_f_min{0.0_A};

  /// Maximum rotor excitation current [A].
  /// Register: `unimoc.control.excitation.i_f_max`
  unit::Current excitation_i_f_max{10.0_A};

  // =========================================================================
  // EESM excitation observer
  // =========================================================================

  /// Low-pass filter time constant [s].
  /// Register: `unimoc.observer.excitation.tau`
  unit::Time excitation_obs_tau{2.0_ms};

  /// Mutual inductance L_m [H] used by the observer.
  /// Register: `unimoc.observer.excitation.L_m`
  unit::Inductance excitation_obs_l_m{47.0_mH};

  // =========================================================================
  // Position controller
  // =========================================================================

  /// Position loop proportional gain [rad/s per rad].
  /// Register: `unimoc.control.pos.kp`
  unit::AngularVelocityPerAngle pos_kp_pos{10.0_rad_per_s_per_rad};

  /// Speed loop proportional gain.
  /// Register: `unimoc.control.pos.kp_speed`
  unit::Ratio pos_kp_speed{5.0_ratio};

  /// Speed loop integral gain.
  /// Register: `unimoc.control.pos.ki_speed`
  unit::InverseTime pos_ki_speed{20.0_per_s};

  /// Maximum mechanical angular velocity [rad/s].
  /// Register: `unimoc.control.pos.speed_limit`
  unit::AngularVelocity pos_speed_limit{100.0_rad_per_s};

  /// Maximum speed-demand rate of change [rad/s²].
  /// Register: `unimoc.control.pos.accel_limit`
  unit::AngularAcceleration pos_accel_limit{500.0_rad_per_s2};

  /// In-position position tolerance [rad].
  /// Register: `unimoc.control.pos.position_tolerance`
  unit::Angle pos_position_tolerance{0.01_rad};

  /// In-position speed tolerance [rad/s].
  /// Register: `unimoc.control.pos.speed_tolerance`
  unit::AngularVelocity pos_speed_tolerance{1.0_rad_per_s};

  /// Constant homing velocity [rad/s].
  /// Register: `unimoc.control.pos.homing_speed`
  unit::AngularVelocity pos_homing_speed{5.0_rad_per_s};

  // =========================================================================
  // ADC calibration (populated by the hardware startup aid)
  //
  // These values are measured by HwStartup and stored here for persistence.
  // The hardware-specific adc_read_injected() implementation should apply
  // them as:  i_cal = (i_raw - adc_offset_X) * adc_gain_X
  //           v_cal = v_raw * adc_gain_vdc
  // =========================================================================

  /// Phase-A current-sense ADC zero offset [A].
  /// Register: `unimoc.startup.adc_offset_a`
  unit::Current adc_offset_a{0.0_A};

  /// Phase-B current-sense ADC zero offset [A].
  /// Register: `unimoc.startup.adc_offset_b`
  unit::Current adc_offset_b{0.0_A};

  /// Phase-A current-sense ADC gain correction factor [dimensionless].
  /// Apply as: i_cal_a = (i_raw_a - adc_offset_a) * adc_gain_a.
  /// Register: `unimoc.startup.gain_a`
  unit::Ratio adc_gain_a{1.0_ratio};

  /// Phase-B current-sense ADC gain correction factor [dimensionless].
  /// Register: `unimoc.startup.gain_b`
  unit::Ratio adc_gain_b{1.0_ratio};

  /// DC-link voltage ADC gain correction factor [dimensionless].
  /// Apply as: v_cal = v_raw * adc_gain_vdc.
  /// Register: `unimoc.startup.gain_vdc`
  unit::Ratio adc_gain_vdc{1.0_ratio};

  // =========================================================================
  // Phase current balance correction
  //
  // Per-phase ADC gain correction factors, dimensionless (nominal value 1.0).
  // Applied to the raw phase-current ADC reading of each channel to
  // compensate for hardware gain/offset mismatch between the three current
  // sense paths.  Identified by CMD_MEASURE_BALANCE.
  // =========================================================================

  /// Phase-A ADC gain correction factor [dimensionless, ≈ 1.0].
  /// Register: `unimoc.motor.balance.gain_a`
  unit::Ratio phase_balance_a{1.0_ratio};

  /// Phase-B ADC gain correction factor [dimensionless, ≈ 1.0].
  /// Register: `unimoc.motor.balance.gain_b`
  unit::Ratio phase_balance_b{1.0_ratio};

  /// Phase-C ADC gain correction factor [dimensionless, ≈ 1.0].
  /// Register: `unimoc.motor.balance.gain_c`
  unit::Ratio phase_balance_c{1.0_ratio};

  // =========================================================================
  // Validation and safety clamping
  // =========================================================================

  /**
   * @brief Return true if the magic word and version match expected values.
   *
   * Call this after loading from NVM.  If it returns false the block is
   * uninitialised or corrupt — reset to defaults and re-save.
   * @return `true` when the image header matches the current settings format.
   */
  [[nodiscard]] constexpr bool IsValid() const noexcept { return magic == kNvmMagic && version == kNvmVersion; }

  /// Restore all fields to factory defaults.
  void ResetToDefaults() noexcept { *this = NvmSettings{}; }
};

}  // namespace settings
}  // namespace unimoc
