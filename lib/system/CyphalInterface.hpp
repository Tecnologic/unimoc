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

#ifndef UNIMOC_SYSTEM_CYPHAL_INTERFACE_H_
#define UNIMOC_SYSTEM_CYPHAL_INTERFACE_H_

#include <cstdint>

/**
 * @file CyphalInterface.hpp
 * @brief Canonical UNIMOC Cyphal register names and subject port IDs.
 *
 * Overview
 * ========
 * UNIMOC exposes its complete configuration and runtime interface over the
 * Cyphal (UAVCAN v1) network.  The two main mechanisms are:
 *
 *  1. **Registers** (`uavcan.register.Access` / `uavcan.register.List`)
 *     — Named key/value pairs for persistent settings.  Writing a register
 *     updates the in-RAM copy and triggers an NVM flush.
 *
 *  2. **Subjects** (publisher/subscriber port IDs)
 *     — Real-time data exchange: setpoints flow *in*, status/telemetry flow
 *     *out*.  Port IDs are themselves registers (`uavcan.pub.*`,
 *     `uavcan.sub.*`) so they can be reconfigured without firmware changes.
 *
 * Node ID and plug-and-play
 * =========================
 * If `uavcan.node.id` register equals 0 (NVM default for a fresh device)
 * the firmware participates in the Cyphal plug-and-play node-ID allocation
 * protocol (`uavcan.pnp.NodeIDAllocationData.2`).  Once an ID is allocated
 * it is written back to `uavcan.node.id` and persisted to NVM so that the
 * same ID is used after every subsequent reset.
 *
 * Node identity
 * =============
 * `uavcan.node.description` (string, read/write) stores the human-readable
 * node name returned in `uavcan.node.GetInfo` responses.  Set it once via
 * Cyphal to give the drive a meaningful identity in the network (e.g.,
 * "unimoc.propulsion.left").  The value is stored in NvmSettings::identity.
 * The hardware unique-ID used for GetInfo/PnP is sourced directly from the
 * hardware layer at runtime (not from NvmSettings).
 *
 * Setpoints and control mode
 * ==========================
 * Runtime setpoints are **not** persisted to NVM — they are ephemeral and
 * sourced from Cyphal subjects each cycle.  For UDRAL servo-style interfaces
 * (`reg.udral.service.actuator.servo/_.0.1`) the active control mode is
 * selected from the first finite kinematics field of the setpoint
 * (`position -> POSITION`, `velocity -> SPEED`, else `torque -> TORQUE`).
 * The *initial* control mode after boot is still stored in
 * NvmSettings::control_mode and can be changed by writing
 * the `unimoc.control.mode` register.
 *
 * All settings available via Cyphal
 * ==================================
 * Every field of NvmSettings has a corresponding register entry listed in the
 * `reg` namespace below.  Writing any of those registers over Cyphal updates
 * the parameter in RAM and schedules an NVM flush.  This means the complete
 * drive configuration (motor parameters, controller gains, limits, identity)
 * is accessible and persistent without physical access to the hardware.
 */
namespace unimoc
{
namespace system
{
namespace cyphal
{

// ============================================================================
// Standard Cyphal node registers
// ============================================================================

/// @defgroup std_registers Standard Cyphal registers
/// @{

namespace reg
{
    // --- Node identity and topology ---

    /// Cyphal node ID [1..127].  0 triggers PnP allocation at boot.
    /// Type: natural16[1] (read/write, persistent)
    inline constexpr const char* NODE_ID           = "uavcan.node.id";

    /// Human-readable UTF-8 node description / name (max 50 bytes).
    /// Maps to NodeIdentity::name and returned in GetInfo.
    /// Type: string (read/write, persistent)
    inline constexpr const char* NODE_DESCRIPTION  = "uavcan.node.description";

    // --- System parameters ---

    /// Active motor type: 0=PMSM, 1=ASM, 2=EESM.
    /// Type: natural8[1] (read/write, persistent)
    inline constexpr const char* MOTOR_TYPE        = "unimoc.motor.type";

    /// Motor pole-pair count.
    /// Type: natural8[1] (read/write, persistent)
    inline constexpr const char* MOTOR_POLE_PAIRS  = "unimoc.motor.pole_pairs";

    /// Active control mode on boot: 0=TORQUE, 1=SPEED, 2=POSITION.
    /// Type: natural8[1] (read/write, persistent)
    inline constexpr const char* CONTROL_MODE      = "unimoc.control.mode";

    // --- Stator winding parameters ---

    /// Stator phase resistance [Ω].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* STATOR_R          = "unimoc.motor.stator.R";

    /// Stator inductance [H].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* STATOR_L          = "unimoc.motor.stator.L";

    // --- PMSM / EESM parameters ---

    /// Permanent-magnet flux linkage ψ_PM [Wb].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* PMSM_FLUX_PM      = "unimoc.motor.pmsm.flux_pm";

    /// d-axis inductance L_d [H].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* PMSM_L_D          = "unimoc.motor.pmsm.L_d";

    /// q-axis inductance L_q [H].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* PMSM_L_Q          = "unimoc.motor.pmsm.L_q";

    // --- ASM parameters ---

    /// ASM rotor resistance [Ω].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* ASM_R_R           = "unimoc.motor.asm.R_r";

    /// ASM stator resistance [Ω].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* ASM_R_S           = "unimoc.motor.asm.R_s";

    /// ASM stator self-inductance [H].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* ASM_L_S           = "unimoc.motor.asm.L_s";

    /// ASM rotor self-inductance [H].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* ASM_L_R           = "unimoc.motor.asm.L_r";

    /// ASM mutual (magnetising) inductance [H].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* ASM_L_M           = "unimoc.motor.asm.L_m";

    // --- Mechanical observer (Kalman filter) ---

    /// Rotor + load inertia J [kg·m²].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* MOTOR_J           = "unimoc.motor.mechanics.J";

    /// Kalman filter process noise variance Q.
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* MECH_OBS_Q        = "unimoc.observer.mech.Q";

    /// Kalman filter measurement noise variance R.
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* MECH_OBS_R        = "unimoc.observer.mech.R";

    // --- PMSM flux observer ---

    /// PMSM flux observer d-axis anti-drift feedback gain C_d [1/s].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* PMSM_FLUX_OBS_C_D = "unimoc.observer.pmsm_flux.C_d";

    /// PMSM flux observer q-axis anti-drift feedback gain C_q [1/s].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* PMSM_FLUX_OBS_C_Q = "unimoc.observer.pmsm_flux.C_q";

    // --- ASM flux observer gains ---

    /// ASM stator-current correction gain g_i [1/s].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* ASM_OBS_G_I       = "unimoc.observer.asm_flux.g_i";

    /// ASM rotor-flux correction gain g_flux [Wb/(A·s)].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* ASM_OBS_G_FLUX    = "unimoc.observer.asm_flux.g_flux";

    // --- ASM flux controller ---

    /// ASM flux controller proportional gain [A/Wb].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* ASM_FLUX_KP       = "unimoc.control.asm_flux.kp";

    /// ASM flux controller integral gain [A/(Wb·s)].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* ASM_FLUX_KI       = "unimoc.control.asm_flux.ki";

    /// ASM flux controller minimum d-axis current [A].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* ASM_FLUX_I_D_MIN  = "unimoc.control.asm_flux.i_d_min";

    /// ASM flux controller maximum d-axis current [A].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* ASM_FLUX_I_D_MAX  = "unimoc.control.asm_flux.i_d_max";

    // --- Field weakening ---

    /// Maximum voltage vector magnitude (normalised).
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* FW_V_MAX          = "unimoc.control.fw.v_max";

    /// Field-weakening integrator gain [A/(V·s)].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* FW_KI             = "unimoc.control.fw.ki";

    /// Most negative i_d allowed [A].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* FW_I_D_MIN        = "unimoc.control.fw.i_d_min";

    // --- SVM modulator ---

    /// Minimum PWM duty cycle.
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* SVM_DUTY_MIN      = "unimoc.control.svm.duty_min";

    /// Maximum PWM duty cycle.
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* SVM_DUTY_MAX      = "unimoc.control.svm.duty_max";

    // --- Dead-time compensation ---

    /// Gate-driver dead time [s].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* DTC_DEAD_TIME     = "unimoc.control.dtc.dead_time";

    /// PWM switching frequency [Hz].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* DTC_F_PWM         = "unimoc.control.dtc.f_pwm";

    /// Phase-current zero-crossing threshold [A].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* DTC_I_THRESHOLD   = "unimoc.control.dtc.i_threshold";

    // --- HFI observer ---

    /// HFI injection voltage [V].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* HFI_V_INJECT      = "unimoc.observer.hfi.v_inject";

    /// HFI angle-error scaling gain [1/V].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* HFI_ERROR_GAIN    = "unimoc.observer.hfi.error_gain";

    // --- EESM excitation controller ---

    /// Excitation mode: 0=CurrentMode, 1=FluxMode.
    /// Type: natural8[1] (read/write, persistent)
    inline constexpr const char* EXCIT_MODE        = "unimoc.control.excitation.mode";

    /// Excitation controller L_m [H].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* EXCIT_L_M         = "unimoc.control.excitation.L_m";

    /// Excitation controller proportional gain [V/A].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* EXCIT_KP          = "unimoc.control.excitation.kp";

    /// Excitation controller integral gain [V/(A·s)].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* EXCIT_KI          = "unimoc.control.excitation.ki";

    /// Minimum rotor excitation current [A].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* EXCIT_I_F_MIN     = "unimoc.control.excitation.i_f_min";

    /// Maximum rotor excitation current [A].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* EXCIT_I_F_MAX     = "unimoc.control.excitation.i_f_max";

    // --- EESM excitation observer ---

    /// Excitation observer LPF time constant [s].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* EXCIT_OBS_TAU     = "unimoc.observer.excitation.tau";

    /// Excitation observer L_m [H].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* EXCIT_OBS_L_M     = "unimoc.observer.excitation.L_m";

    // --- Position controller ---

    /// Position loop proportional gain [rad/s per rad].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* POS_KP            = "unimoc.control.pos.kp";

    /// Speed loop proportional gain.
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* POS_KP_SPEED      = "unimoc.control.pos.kp_speed";

    /// Speed loop integral gain.
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* POS_KI_SPEED      = "unimoc.control.pos.ki_speed";

    /// Maximum mechanical angular velocity [rad/s].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* POS_SPEED_LIMIT   = "unimoc.control.pos.speed_limit";

    /// Maximum speed-demand acceleration [rad/s²].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* POS_ACCEL_LIMIT   = "unimoc.control.pos.accel_limit";

    /// In-position position tolerance [rad].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* POS_POS_TOL       = "unimoc.control.pos.position_tolerance";

    /// In-position speed tolerance [rad/s].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* POS_SPEED_TOL     = "unimoc.control.pos.speed_tolerance";

    /// Constant homing search velocity [rad/s].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* POS_HOMING_SPEED  = "unimoc.control.pos.homing_speed";

    // --- Motor operating limits ---

    /// Maximum motor (resultant stator vector) current [A].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* MOTOR_I_MAX            = "unimoc.motor.limits.i_max";

    /// Maximum electrical angular velocity (forward) [rad/s].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* MOTOR_OMEGA_MAX        = "unimoc.motor.limits.omega_max";

    /// Maximum electrical angular velocity (reverse, negative) [rad/s].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* MOTOR_OMEGA_MIN        = "unimoc.motor.limits.omega_min";

    /// Maximum battery discharge (drive) current [A].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* BATTERY_DRIVE_I_MAX    = "unimoc.battery.limits.drive_current";

    /// Maximum battery charge (regen) current [A].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* BATTERY_CHARGE_I_MAX   = "unimoc.battery.limits.charge_current";

    // --- PWM frequency ---

    /// PWM switching frequency selection: 16/20/24/28/32 kHz.
    /// Stored as the frequency value in kHz (uint8).
    /// Type: natural8[1] (read/write, persistent)
    inline constexpr const char* PWM_FREQUENCY          = "unimoc.control.pwm_frequency";

    // --- d/q-axis current controller ---

    /// d-axis proportional gain [V/A].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* CURRENT_KP_D           = "unimoc.control.current.kp_d";

    /// d-axis integral gain [V/(A·s)].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* CURRENT_KI_D           = "unimoc.control.current.ki_d";

    /// q-axis proportional gain [V/A].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* CURRENT_KP_Q           = "unimoc.control.current.kp_q";

    /// q-axis integral gain [V/(A·s)].
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* CURRENT_KI_Q           = "unimoc.control.current.ki_q";

    /// Maximum voltage vector magnitude (normalised by V_dc).
    /// Type: real32[1] (read/write, persistent)
    inline constexpr const char* CURRENT_V_MAX          = "unimoc.control.current.v_max";

}  // namespace reg

// ============================================================================
// Cyphal subject port IDs
// ============================================================================
//
// These are the *default* port IDs.  Each is also a port-ID register
// (uavcan.sub.<name>.id or uavcan.pub.<name>.id) so they can be reconfigured
// at runtime without firmware changes.
//
// Convention:
//   UNSET_PORT_ID  — a port ID of 65535 means the port is disabled.
//   Subject names follow snake_case and map to the Cyphal data type used.
// ============================================================================

/// Port ID value meaning "port not configured / disabled".
inline constexpr uint16_t UNSET_PORT_ID = 0xFFFFu;

/// @defgroup sub_ports Subscriber (input) subjects — setpoints received by UNIMOC
/// @{

/// Torque (q-axis current) setpoint [A·m or normalised, depends on scale].
/// Data type: uavcan.si.unit.torque.Scalar.1.0 or reg.udral.physics.dynamics.rotation.PlanarTs
/// Register: `uavcan.sub.torque_sp.id`
inline constexpr uint16_t SUB_TORQUE_SETPOINT    = 100u;

/// Speed (mechanical angular velocity) setpoint [rad/s].
/// Data type: uavcan.si.unit.angular_velocity.Scalar.1.0
/// Register: `uavcan.sub.speed_sp.id`
inline constexpr uint16_t SUB_SPEED_SETPOINT     = 101u;

/// Position setpoint [rad], referenced to home.
/// Data type: uavcan.si.unit.angle.Scalar.1.0
/// Register: `uavcan.sub.position_sp.id`
inline constexpr uint16_t SUB_POSITION_SETPOINT  = 102u;

/// Legacy explicit control mode selection: 0=TORQUE, 1=SPEED, 2=POSITION.
/// UDRAL servo mode selection should follow setpoint field precedence instead.
/// Data type: uavcan.primitive.scalar.Natural8.1.0 (legacy)
/// Register: `uavcan.sub.control_mode.id`
inline constexpr uint16_t SUB_CONTROL_MODE       = 103u;

/// EESM rotor excitation setpoint (current [A] or flux [Wb] per mode).
/// Data type: uavcan.primitive.scalar.Real32.1.0
/// Register: `uavcan.sub.excitation_sp.id`
inline constexpr uint16_t SUB_EXCITATION_SETPOINT = 104u;

/// Homing trigger: any value on this subject starts the homing sequence.
/// Data type: uavcan.primitive.scalar.Bit.1.0
/// Register: `uavcan.sub.homing_trigger.id`
inline constexpr uint16_t SUB_HOMING_TRIGGER     = 105u;

/// @}

/// @defgroup pub_ports Publisher (output) subjects — telemetry from UNIMOC
/// @{

/// Estimated rotor electrical angle [rad].
/// Data type: uavcan.si.unit.angle.Scalar.1.0
/// Register: `uavcan.pub.rotor_angle.id`
inline constexpr uint16_t PUB_ROTOR_ANGLE        = 200u;

/// Estimated electrical angular velocity [rad/s].
/// Data type: uavcan.si.unit.angular_velocity.Scalar.1.0
/// Register: `uavcan.pub.rotor_speed.id`
inline constexpr uint16_t PUB_ROTOR_SPEED        = 201u;

/// Absolute shaft position [rad], referenced to home.
/// Data type: uavcan.si.unit.angle.Scalar.1.0
/// Register: `uavcan.pub.shaft_position.id`
inline constexpr uint16_t PUB_SHAFT_POSITION     = 202u;

/// In-position flag.
/// Data type: uavcan.primitive.scalar.Bit.1.0
/// Register: `uavcan.pub.in_position.id`
inline constexpr uint16_t PUB_IN_POSITION        = 203u;

/// Homing state: 0=IDLE, 1=SEARCHING, 2=ZEROING, 3=DONE, 4=FAULT.
/// Data type: uavcan.primitive.scalar.Natural8.1.0
/// Register: `uavcan.pub.homing_state.id`
inline constexpr uint16_t PUB_HOMING_STATE       = 204u;

/// DC-link voltage [V].
/// Data type: uavcan.si.unit.voltage.Scalar.1.0
/// Register: `uavcan.pub.dc_voltage.id`
inline constexpr uint16_t PUB_DC_VOLTAGE         = 205u;

/// Phase current magnitude [A].
/// Data type: uavcan.si.unit.electric_current.Scalar.1.0
/// Register: `uavcan.pub.phase_current.id`
inline constexpr uint16_t PUB_PHASE_CURRENT      = 206u;

/// Estimated rotor excitation current î_f [A] (EESM only).
/// Data type: uavcan.si.unit.electric_current.Scalar.1.0
/// Register: `uavcan.pub.excitation_current.id`
inline constexpr uint16_t PUB_EXCITATION_CURRENT = 207u;

/// @}

}  // namespace cyphal
}  // namespace system
}  // namespace unimoc

#endif /* UNIMOC_SYSTEM_CYPHAL_INTERFACE_H_ */
