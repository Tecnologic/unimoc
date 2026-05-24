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
#pragma once

#ifndef UNIMOC_SYSTEM_NVM_SETTINGS_H_
#define UNIMOC_SYSTEM_NVM_SETTINGS_H_

#include <algorithm>
#include <cstdint>
#include "ControlMode.hpp"
#include "MotorType.hpp"
#include "NodeIdentity.hpp"
#include "HardwareLimits.hpp"

/**
 * @namespace unimoc global namespace
 */
namespace unimoc
{
/**
 * @namespace system coordinate and motor type definitions
 */
namespace system
{

/// Magic number stored at the start of every NvmSettings block.
/// Used to detect an uninitialised or corrupt NVM image.
inline constexpr uint32_t NVM_MAGIC = 0x554D4F43u;  // 'UMOC'

/// Layout version of the NvmSettings struct.
/// Increment whenever the struct layout changes incompatibly.
inline constexpr uint16_t NVM_VERSION = 1u;

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
 * Each field corresponds to a named Cyphal register (see CyphalInterface.hpp
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
struct NvmSettings
{
    // =========================================================================
    // Header — must be first
    // =========================================================================

    /// Magic word — set to NVM_MAGIC by factory defaults.
    uint32_t magic{NVM_MAGIC};

    /// Layout version — set to NVM_VERSION.
    uint16_t version{NVM_VERSION};

    // =========================================================================
    // Node identification
    // =========================================================================

    /// Cyphal node ID [1..127].  0 = not assigned → use PnP allocation.
    /// Register: `uavcan.node.id`
    uint8_t node_id{0};

    /// Human-readable node identity (name + hw/sw versions).
    /// Register: `uavcan.node.description` (name field only over Cyphal).
    NodeIdentity identity{};

    // =========================================================================
    // System
    // =========================================================================

    /// Active motor type.
    /// Register: `unimoc.motor.type`
    MotorType motor_type{MotorType::PMSM};

    /// Motor pole-pair count.
    /// Register: `unimoc.motor.pole_pairs`
    uint8_t pole_pairs{1};

    /// Initial control mode after boot (overridden at runtime via Cyphal).
    /// Register: `unimoc.control.mode`
    ControlMode control_mode{ControlMode::TORQUE};

    // =========================================================================
    // Stator / winding parameters (all motor types)
    // =========================================================================

    /// Stator phase resistance [Ω].
    /// Register: `unimoc.motor.stator.R`
    float stator_R{0.1f};

    /// Stator inductance [H] (L_q for IPMSM; average (L_d+L_q)/2 otherwise).
    /// Register: `unimoc.motor.stator.L`
    float stator_L{1e-3f};

    // =========================================================================
    // Motor operating limits
    //
    // These are runtime-configurable limits; they must never exceed the
    // compile-time hardware limits declared in hardware::Limits.
    // Call clamp_to_hardware_limits() after loading from NVM to enforce this.
    // =========================================================================

    /// Maximum motor (resultant stator vector) current [A].
    /// Hard upper bound: hardware::Limits::max_motor_current_A.
    /// Register: `unimoc.motor.limits.i_max`
    float motor_i_max{hardware::Limits::max_motor_current_A};

    /// Maximum electrical angular velocity in the forward direction [rad/s].
    /// Register: `unimoc.motor.limits.omega_max`
    float motor_omega_max{2000.0f};

    /// Maximum electrical angular velocity in the reverse direction [rad/s]
    /// (stored as a negative value).
    /// Register: `unimoc.motor.limits.omega_min`
    float motor_omega_min{-2000.0f};

    /// Maximum battery discharge (drive) current [A].
    /// Hard upper bound: hardware::Limits::max_battery_drive_current_A.
    /// Register: `unimoc.battery.limits.drive_current`
    float battery_drive_current_max{hardware::Limits::max_battery_drive_current_A};

    /// Maximum battery charge (regenerative braking) current [A].
    /// Hard upper bound: hardware::Limits::max_battery_charge_current_A.
    /// Register: `unimoc.battery.limits.charge_current`
    float battery_charge_current_max{hardware::Limits::max_battery_charge_current_A};

    // =========================================================================
    // PMSM / EESM parameters
    // =========================================================================

    /// Permanent-magnet flux linkage ψ_PM [Wb].
    /// For EESM, this is the initial/nominal ψ_f used until the excitation
    /// observer provides a measured value.
    /// Register: `unimoc.motor.pmsm.flux_pm`
    float flux_pm{0.0f};

    /// d-axis inductance L_d [H].
    /// Register: `unimoc.motor.pmsm.L_d`
    float L_d{1e-3f};

    /// q-axis inductance L_q [H].
    /// Register: `unimoc.motor.pmsm.L_q`
    float L_q{1e-3f};

    // =========================================================================
    // Asynchronous motor (ASM) parameters
    // =========================================================================

    /// Rotor resistance R_r [Ω].
    /// Register: `unimoc.motor.asm.R_r`
    float asm_R_r{0.3f};

    /// Stator resistance R_s [Ω].
    /// Register: `unimoc.motor.asm.R_s`
    float asm_R_s{0.5f};

    /// Stator self-inductance L_s [H].
    /// Register: `unimoc.motor.asm.L_s`
    float asm_L_s{50e-3f};

    /// Rotor self-inductance L_r [H].
    /// Register: `unimoc.motor.asm.L_r`
    float asm_L_r{50e-3f};

    /// Mutual (magnetising) inductance L_m [H].
    /// Register: `unimoc.motor.asm.L_m`
    float asm_L_m{47e-3f};

    // =========================================================================
    // Mechanical observer (Kalman filter) parameters
    // =========================================================================

    /// Rotor + load inertia J [kg·m²].
    /// Used by MechanicalObserver::predict() to integrate the torque equation.
    /// Register: `unimoc.motor.mechanics.J`
    float motor_J{1e-4f};

    /// Kalman filter process noise variance Q.
    /// Increase to let the filter track faster but noisier.
    /// Register: `unimoc.observer.mech.Q`
    float mech_obs_Q{1e-5f};

    /// Kalman filter measurement noise variance R.
    /// Increase to smooth angle corrections at the cost of slower tracking.
    /// Register: `unimoc.observer.mech.R`
    float mech_obs_R{1e-4f};

    // =========================================================================
    // PMSM flux observer (PmsmFluxObserver) gains
    // =========================================================================

    /// d-axis anti-drift feedback gain C_d [1/s].
    /// Register: `unimoc.observer.pmsm_flux.C_d`
    float pmsm_flux_obs_C_d{50.0f};

    /// q-axis anti-drift feedback gain C_q [1/s].
    /// Register: `unimoc.observer.pmsm_flux.C_q`
    float pmsm_flux_obs_C_q{1.0f};

    // =========================================================================
    // ASM flux observer gains
    // =========================================================================

    /// Stator-current correction gain g_i [1/s].
    /// Register: `unimoc.observer.asm_flux.g_i`
    float asm_obs_g_i{500.0f};

    /// Rotor-flux correction gain g_flux [Wb/(A·s)].
    /// Register: `unimoc.observer.asm_flux.g_flux`
    float asm_obs_g_flux{5000.0f};

    // =========================================================================
    // ASM flux controller
    // =========================================================================

    /// Proportional gain [A/Wb].
    /// Register: `unimoc.control.asm_flux.kp`
    float asm_flux_kp{10.0f};

    /// Integral gain [A/(Wb·s)].
    /// Register: `unimoc.control.asm_flux.ki`
    float asm_flux_ki{50.0f};

    /// Minimum d-axis current [A].
    /// Register: `unimoc.control.asm_flux.i_d_min`
    float asm_flux_i_d_min{0.0f};

    /// Maximum d-axis current [A].
    /// Register: `unimoc.control.asm_flux.i_d_max`
    float asm_flux_i_d_max{10.0f};

    // =========================================================================
    // Field weakening
    // =========================================================================

    /// Maximum voltage vector magnitude (normalised, range (0,1]).
    /// Register: `unimoc.control.fw.v_max`
    float fw_v_max{0.9f};

    /// Integrator gain [A/(V·s)].
    /// Register: `unimoc.control.fw.ki`
    float fw_ki{10.0f};

    /// Most negative i_d allowed [A].
    /// Register: `unimoc.control.fw.i_d_min`
    float fw_i_d_min{-10.0f};

    // =========================================================================
    // SVM modulator
    // =========================================================================

    /// Minimum duty cycle (headroom for ADC + dead-time).
    /// Register: `unimoc.control.svm.duty_min`
    float svm_duty_min{0.05f};

    /// Maximum duty cycle.
    /// Register: `unimoc.control.svm.duty_max`
    float svm_duty_max{0.95f};

    // =========================================================================
    // Dead-time compensation
    // =========================================================================

    /// Gate-driver dead time [s].
    /// Register: `unimoc.control.dtc.dead_time`
    float dtc_dead_time{0.0f};

    /// PWM switching frequency [Hz].
    /// Register: `unimoc.control.dtc.f_pwm`
    float dtc_f_pwm{10000.0f};

    /// Phase-current zero-crossing threshold [A].
    /// Register: `unimoc.control.dtc.i_threshold`
    float dtc_i_threshold{0.1f};

    // =========================================================================
    // HFI (high-frequency injection) observer
    // =========================================================================

    /// HFI injection voltage [V].
    /// Register: `unimoc.observer.hfi.v_inject`
    float hfi_v_inject{0.0f};

    /// Angle error to PLL scaling gain [1/V].
    /// Register: `unimoc.observer.hfi.error_gain`
    float hfi_error_gain{1.0f};

    // =========================================================================
    // EESM excitation controller
    // =========================================================================

    /// Excitation mode: 0 = CurrentMode, 1 = FluxMode.
    /// Register: `unimoc.control.excitation.mode`
    uint8_t excitation_mode{0};

    /// Mutual inductance L_m [H] for flux↔current conversion.
    /// Register: `unimoc.control.excitation.L_m`
    float excitation_L_m{47e-3f};

    /// Proportional gain [V/A].
    /// Register: `unimoc.control.excitation.kp`
    float excitation_kp{5.0f};

    /// Integral gain [V/(A·s)].
    /// Register: `unimoc.control.excitation.ki`
    float excitation_ki{50.0f};

    /// Minimum rotor excitation current [A].
    /// Register: `unimoc.control.excitation.i_f_min`
    float excitation_i_f_min{0.0f};

    /// Maximum rotor excitation current [A].
    /// Register: `unimoc.control.excitation.i_f_max`
    float excitation_i_f_max{10.0f};

    // =========================================================================
    // EESM excitation observer
    // =========================================================================

    /// Low-pass filter time constant [s].
    /// Register: `unimoc.observer.excitation.tau`
    float excitation_obs_tau{2e-3f};

    /// Mutual inductance L_m [H] used by the observer.
    /// Register: `unimoc.observer.excitation.L_m`
    float excitation_obs_L_m{47e-3f};

    // =========================================================================
    // Position controller
    // =========================================================================

    /// Position loop proportional gain [rad/s per rad].
    /// Register: `unimoc.control.pos.kp`
    float pos_kp_pos{10.0f};

    /// Speed loop proportional gain.
    /// Register: `unimoc.control.pos.kp_speed`
    float pos_kp_speed{5.0f};

    /// Speed loop integral gain.
    /// Register: `unimoc.control.pos.ki_speed`
    float pos_ki_speed{20.0f};

    /// Maximum mechanical angular velocity [rad/s].
    /// Register: `unimoc.control.pos.speed_limit`
    float pos_speed_limit{100.0f};

    /// Maximum speed-demand rate of change [rad/s²].
    /// Register: `unimoc.control.pos.accel_limit`
    float pos_accel_limit{500.0f};

    /// In-position position tolerance [rad].
    /// Register: `unimoc.control.pos.position_tolerance`
    float pos_position_tolerance{0.01f};

    /// In-position speed tolerance [rad/s].
    /// Register: `unimoc.control.pos.speed_tolerance`
    float pos_speed_tolerance{1.0f};

    /// Constant homing velocity [rad/s].
    /// Register: `unimoc.control.pos.homing_speed`
    float pos_homing_speed{5.0f};

    // =========================================================================
    // Validation and safety clamping
    // =========================================================================

    /**
     * @brief Return true if the magic word and version match expected values.
     *
     * Call this after loading from NVM.  If it returns false the block is
     * uninitialised or corrupt — reset to defaults and re-save.
     */
    [[nodiscard]] constexpr bool
    is_valid() const noexcept
    {
        return magic == NVM_MAGIC && version == NVM_VERSION;
    }

    /**
     * @brief Clamp all runtime current limits to the compile-time hardware maxima.
     *
     * Call this immediately after loading settings from NVM (and before
     * applying any Cyphal register write) to ensure that stored values can
     * never drive hardware beyond its safe operating envelope even if the NVM
     * image was written by firmware targeting a different hardware revision.
     */
    constexpr void
    clamp_to_hardware_limits() noexcept
    {
        motor_i_max =
            hardware::Limits::clamp_motor_current(motor_i_max);
        battery_drive_current_max =
            hardware::Limits::clamp_battery_drive_current(battery_drive_current_max);
        battery_charge_current_max =
            hardware::Limits::clamp_battery_charge_current(battery_charge_current_max);
    }

    /// Restore all fields to factory defaults.
    void
    reset_to_defaults() noexcept
    {
        *this = NvmSettings{};
    }
};

}  // namespace system
}  // namespace unimoc

#endif /* UNIMOC_SYSTEM_NVM_SETTINGS_H_ */
