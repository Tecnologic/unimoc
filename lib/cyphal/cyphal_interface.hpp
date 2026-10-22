/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file cyphal_interface.hpp
 *    @brief Cyphal register, subject, and execute-command identifiers.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>

/**
 * @details Canonical UNIMOC Cyphal register names and subject port IDs.
 *
 * Overview
 * ========
 * UNIMOC exposes its complete configuration and runtime interface over the
 * Cyphal (UAVCAN v1) network.  The two main mechanisms are:
 *
 *  1. **Registers** (`uavcan.register.Access` / `uavcan.register.List`)
 *     — Named key/value pairs for persistent settings.  Writing a register
 *     updates the in-RAM copy; ExecuteCommand STORE_PERSISTENT_STATES saves
 *     the changed registers to flash.
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
 * protocol (`uavcan.pnp.NodeIDAllocationData.1.0`, the Cyphal/CAN version).
 * Once an ID is allocated it is written back to `uavcan.node.id` and
 * persisted to NVM so that the same ID is used after every subsequent reset.
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
 * `reg` namespace below (the table is `ForEachRegister()` in
 * settings_registers.hpp).  Writing any of those registers over Cyphal
 * updates the parameter in RAM; a store command persists it.  The complete
 * drive configuration (motor parameters, controller gains, limits, identity,
 * ports) is accessible without physical access to the hardware. The
 * `unimoc.startup.*` run/step/result registers of the startup FSM are served
 * by the drive (src/drive/startup_registers.hpp); they are not persistent.
 */

namespace unimoc::cyphal {

// ============================================================================
// Standard Cyphal node registers
// ============================================================================

/// @defgroup std_registers Standard Cyphal registers
/// @{

namespace reg {
// --- Node identity and topology ---

/// Cyphal node ID [1..127].  0 triggers PnP allocation at boot.
/// Type: natural16[1] (read/write, persistent)
inline constexpr const char* kNodeId = "uavcan.node.id";

/// Human-readable UTF-8 node description / name (max 50 bytes).
/// Maps to node_identity::name and returned in GetInfo.
/// Type: string (read/write, persistent)
inline constexpr const char* kNodeDescription = "uavcan.node.description";

// --- System parameters ---

/// Active motor type: 0=PMSM, 1=ASM, 2=EESM.
/// Type: natural8[1] (read/write, persistent)
inline constexpr const char* kMotorType = "unimoc.motor.type";

/// Motor pole-pair count.
/// Type: natural8[1] (read/write, persistent)
inline constexpr const char* kMotorPolePairs = "unimoc.motor.pole_pairs";

/// Active control mode on boot: 0=TORQUE, 1=SPEED, 2=POSITION.
/// Type: natural8[1] (read/write, persistent)
inline constexpr const char* kControlMode = "unimoc.control.mode";

// --- Stator winding parameters ---

/// Stator phase resistance [Ω].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kStatorR = "unimoc.motor.stator.R";

/// Stator inductance [H].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kStatorL = "unimoc.motor.stator.L";

// --- PMSM / EESM parameters ---

/// Permanent-magnet flux linkage ψ_PM [Wb].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kPmsmFluxPm = "unimoc.motor.pmsm.flux_pm";

/// d-axis inductance L_d [H].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kPmsmLD = "unimoc.motor.pmsm.L_d";

/// q-axis inductance L_q [H].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kPmsmLQ = "unimoc.motor.pmsm.L_q";

// --- ASM parameters ---

/// ASM rotor resistance [Ω].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kAsmRR = "unimoc.motor.asm.R_r";

/// ASM stator resistance [Ω].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kAsmRS = "unimoc.motor.asm.R_s";

/// ASM stator self-inductance [H].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kAsmLS = "unimoc.motor.asm.L_s";

/// ASM rotor self-inductance [H].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kAsmLR = "unimoc.motor.asm.L_r";

/// ASM mutual (magnetising) inductance [H].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kAsmLM = "unimoc.motor.asm.L_m";

// --- Mechanical observer (Kalman filter) ---

/// Rotor + load inertia J [kg·m²].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kMotorJ = "unimoc.motor.mechanics.J";

/// Kalman filter process noise variance Q.
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kMechObsQ = "unimoc.observer.mech.Q";

/// Kalman filter measurement noise variance R.
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kMechObsR = "unimoc.observer.mech.R";

// --- PMSM flux observer ---

/// PMSM flux observer d-axis anti-drift feedback gain C_d [1/s].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kPmsmFluxObsCD = "unimoc.observer.pmsm_flux.C_d";

/// PMSM flux observer q-axis anti-drift feedback gain C_q [1/s].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kPmsmFluxObsCQ = "unimoc.observer.pmsm_flux.C_q";

/// Speed weighting k of the flux angle measurement, R_flux = R + k / omega^2 [rad^2 (rad/s)^2].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kPmsmFluxObsRSpeed = "unimoc.observer.pmsm_flux.R_speed";

// --- Hall sensors ---

/// 1 fuses the Hall sensors into the observer, 0 ignores them.
/// Type: natural8[1] (read/write, persistent)
inline constexpr const char* kHallEnable = "unimoc.observer.hall.enable";

/// Electrical angle at which Hall A rises [rad].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kHallOffset = "unimoc.observer.hall.offset";

/// Measurement noise of a Hall edge [rad^2].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kHallREdge = "unimoc.observer.hall.R_edge";

/// Measurement noise of the Hall sector bound [rad^2].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kHallRSector = "unimoc.observer.hall.R_sector";

// --- ASM flux observer gains ---

/// ASM stator-current correction gain g_i [1/s].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kAsmObsGI = "unimoc.observer.asm_flux.g_i";

/// ASM rotor-flux correction gain g_flux [Wb/(A·s)].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kAsmObsGFlux = "unimoc.observer.asm_flux.g_flux";

// --- ASM flux controller ---

/// ASM flux controller proportional gain [A/Wb].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kAsmFluxKp = "unimoc.control.asm_flux.kp";

/// ASM flux controller integral gain [A/(Wb·s)].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kAsmFluxKi = "unimoc.control.asm_flux.ki";

/// ASM flux controller minimum d-axis current [A].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kAsmFluxIDMin = "unimoc.control.asm_flux.i_d_min";

/// ASM flux controller maximum d-axis current [A].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kAsmFluxIDMax = "unimoc.control.asm_flux.i_d_max";

// --- Field weakening ---

/// Maximum voltage vector magnitude (normalised).
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kFwVMax = "unimoc.control.fw.v_max";

/// Field-weakening integrator gain [A/(V·s)].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kFwKi = "unimoc.control.fw.ki";

/// Most negative i_d allowed [A].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kFwIDMin = "unimoc.control.fw.i_d_min";

// --- SVM modulator ---

/// Minimum PWM duty cycle.
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kSvmDutyMin = "unimoc.control.svm.duty_min";

/// Maximum PWM duty cycle.
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kSvmDutyMax = "unimoc.control.svm.duty_max";

// --- Dead-time compensation ---

/// Gate-driver dead time [s].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kDtcDeadTime = "unimoc.control.dtc.dead_time";

/// PWM switching frequency [Hz].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kDtcFPwm = "unimoc.control.dtc.f_pwm";

/// Phase-current zero-crossing threshold [A].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kDtcIThreshold = "unimoc.control.dtc.i_threshold";

// --- HFI observer ---

/// HFI injection voltage [V].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kHfiVInject = "unimoc.observer.hfi.v_inject";

/// HFI current-error to angle scaling gain [rad/A].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kHfiErrorGain = "unimoc.observer.hfi.error_gain";

// --- EESM excitation controller ---

/// Excitation mode: 0=CurrentMode, 1=FluxMode.
/// Type: natural8[1] (read/write, persistent)
inline constexpr const char* kExcitMode = "unimoc.control.excitation.mode";

/// Excitation controller L_m [H].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kExcitLM = "unimoc.control.excitation.L_m";

/// Excitation controller proportional gain [V/A].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kExcitKp = "unimoc.control.excitation.kp";

/// Excitation controller integral gain [V/(A·s)].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kExcitKi = "unimoc.control.excitation.ki";

/// Minimum rotor excitation current [A].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kExcitIFMin = "unimoc.control.excitation.i_f_min";

/// Maximum rotor excitation current [A].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kExcitIFMax = "unimoc.control.excitation.i_f_max";

// --- EESM excitation observer ---

/// Excitation observer LPF time constant [s].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kExcitObsTau = "unimoc.observer.excitation.tau";

/// Excitation observer L_m [H].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kExcitObsLM = "unimoc.observer.excitation.L_m";

// --- Speed controller ---

/// Speed loop proportional gain [N m per rad/s of mechanical speed].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kSpeedKp = "unimoc.control.speed.kp";

/// Speed loop integral gain 1/T_i [1/s].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kSpeedKi = "unimoc.control.speed.ki";

/// Largest rate of change of the speed reference [rad/s^2]; 0: no ramp.
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kSpeedAccelLimit = "unimoc.control.speed.accel_limit";

// --- Position controller ---

/// Position loop proportional gain [rad/s per rad].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kPosKp = "unimoc.control.pos.kp";

/// Speed loop proportional gain.
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kPosKpSpeed = "unimoc.control.pos.kp_speed";

/// Speed loop integral gain.
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kPosKiSpeed = "unimoc.control.pos.ki_speed";

/// Maximum mechanical angular velocity [rad/s].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kPosSpeedLimit = "unimoc.control.pos.speed_limit";

/// Maximum speed-demand acceleration [rad/s²].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kPosAccelLimit = "unimoc.control.pos.accel_limit";

/// In-position position tolerance [rad].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kPosPosTol = "unimoc.control.pos.position_tolerance";

/// In-position speed tolerance [rad/s].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kPosSpeedTol = "unimoc.control.pos.speed_tolerance";

/// Constant homing search velocity [rad/s].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kPosHomingSpeed = "unimoc.control.pos.homing_speed";

// --- Motor operating limits ---

/// Maximum motor (resultant stator vector) current [A].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kMotorIMax = "unimoc.motor.limits.i_max";

/// Maximum electrical angular velocity (forward) [rad/s].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kMotorOmegaMax = "unimoc.motor.limits.omega_max";

/// Maximum electrical angular velocity (reverse, negative) [rad/s].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kMotorOmegaMin = "unimoc.motor.limits.omega_min";

/// Maximum battery discharge (drive) current [A].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kBatteryDriveIMax = "unimoc.battery.limits.drive_current";

/// Maximum battery charge (regen) current [A].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kBatteryChargeIMax = "unimoc.battery.limits.charge_current";

/// DC-link voltage above which regenerative braking is derated [V].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kBatteryRegenDerateVoltage = "unimoc.battery.limits.regen_derate_voltage";

/// DC-link voltage at which the drive trips the power stage [V].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kBatteryTripVoltage = "unimoc.battery.limits.trip_voltage";

// --- PWM frequency ---

/// PWM switching frequency selection: 16/20/24/28/32 kHz.
/// Stored as the frequency value in kHz (uint8).
/// Type: natural8[1] (read/write, persistent)
inline constexpr const char* kPwmFrequency = "unimoc.control.pwm_frequency";

// --- d/q-axis current controller ---

/// d-axis proportional gain [V/A].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kCurrentKpD = "unimoc.control.current.kp_d";

/// d-axis integral gain [V/(A·s)].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kCurrentKiD = "unimoc.control.current.ki_d";

/// q-axis proportional gain [V/A].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kCurrentKpQ = "unimoc.control.current.kp_q";

/// q-axis integral gain [V/(A·s)].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kCurrentKiQ = "unimoc.control.current.ki_q";

/// Maximum voltage vector magnitude (normalised by V_dc).
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kCurrentVMax = "unimoc.control.current.v_max";

// =========================================================================
// Hardware startup aid
// =========================================================================

/// Write 1 to start the startup FSM; write 0 to abort.
/// Type: uint8[1] (write-only)
inline constexpr const char* kStartupRun = "unimoc.startup.run";

/// Current FSM step (read) or step to advance to (write 1 = next step).
/// Type: uint8[1] (read/write)
inline constexpr const char* kStartupStep = "unimoc.startup.step";

/// User-entered external current measurement [A] (clamp-meter reading).
/// Used by CURRENT_SENSE_CALIBRATION step to compute gain_a / gain_b.
/// Type: real32[1] (read/write)
inline constexpr const char* kStartupExtCurrentA = "unimoc.startup.ext_current_A";

/// User-entered external DC-link voltage measurement [V] (multimeter).
/// Used by DC_LINK_VOLTAGE_CHECK step to compute gain_vdc.
/// Type: real32[1] (read/write)
inline constexpr const char* kStartupExtVdcV = "unimoc.startup.ext_vdc_V";

/// Read-back of pass/fail status flags, one byte per FSM step (1=pass, 0=fail/pending).
/// Type: uint8[<=14] (read-only)
inline constexpr const char* kStartupResults = "unimoc.startup.results";

/// Computed phase-A current-sense gain correction factor [dimensionless].
/// Type: real32[1] (read-only after CURRENT_SENSE_CALIBRATION step)
inline constexpr const char* kStartupGainA = "unimoc.startup.gain_a";

/// Computed phase-B current-sense gain correction factor [dimensionless].
/// Type: real32[1] (read-only after CURRENT_SENSE_CALIBRATION step)
inline constexpr const char* kStartupGainB = "unimoc.startup.gain_b";

/// Computed DC-link voltage ADC gain correction factor [dimensionless].
/// Type: real32[1] (read-only after DC_LINK_VOLTAGE_CHECK step)
inline constexpr const char* kStartupGainVdc = "unimoc.startup.gain_vdc";

/// Measured phase-A current-sense zero offset [A].
/// Type: real32[1] (read-only after ADC_OFFSET_CAL step)
inline constexpr const char* kStartupAdcOffsetA = "unimoc.startup.adc_offset_a";

/// Measured phase-B current-sense zero offset [A].
/// Type: real32[1] (read-only after ADC_OFFSET_CAL step)
inline constexpr const char* kStartupAdcOffsetB = "unimoc.startup.adc_offset_b";

// =========================================================================
// Execution times of the interrupts (M0, CONFIG_UNIMOC_HOT_PATH_TIMING)
// =========================================================================

/// Whole sample interrupt of a control boundary, including the control cycle:
/// min, mean, max [timestamp-clock cycles] and the number of records.
/// Type: natural32[4] (read-only)
inline constexpr const char* kTimingSampleIsr = "unimoc.timing.sample_isr";

/// The control cycle alone: min, mean, max [cycles], records.
/// Type: natural32[4] (read-only)
inline constexpr const char* kTimingControlCycle = "unimoc.timing.control_cycle";

/// The observer alone: min, mean, max [cycles], records.
/// Type: natural32[4] (read-only)
inline constexpr const char* kTimingObserver = "unimoc.timing.observer";

/// Duty plans applied late and control cycles that started while the
/// observer still ran, since start.
/// Type: natural32[2] (read-only)
inline constexpr const char* kTimingOverruns = "unimoc.timing.overruns";

/// Frequency of the timestamp clock the times count in [Hz].
/// Type: natural32[1] (read-only)
inline constexpr const char* kTimingClockHz = "unimoc.timing.clock_hz";

/// Write 1: start new statistics (the overrun counters keep counting).
/// Type: natural8[1] (read 0, write 1)
inline constexpr const char* kTimingReset = "unimoc.timing.reset";

// --- Phase current balance correction ---

/// Phase-A ADC gain correction factor [dimensionless, ≈ 1.0].
/// Applied to the raw phase-A current sample to compensate hardware offset/gain
/// mismatch between ADC channels.  Identified by CMD_MEASURE_BALANCE.
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kBalanceGainA = "unimoc.motor.balance.gain_a";

/// Phase-B ADC gain correction factor [dimensionless, ≈ 1.0].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kBalanceGainB = "unimoc.motor.balance.gain_b";

/// Phase-C ADC gain correction factor [dimensionless, ≈ 1.0].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kBalanceGainC = "unimoc.motor.balance.gain_c";

// --- Cyphal ports of the drive (standard port-ID registers) ---
//
// Port-ID registers follow the Cyphal convention `uavcan.<pub|sub>.<name>.id`
// (natural16[1], 65535 = disabled) with a read-only `.type` companion that
// names the data type. A changed port-ID takes effect after a restart.

/// Readiness (arming) of the drive: SLEEP/STANDBY disable, ENGAGED enables.
/// Data type: reg.udral.service.common.Readiness.0.1
/// Type: natural16[1] (read/write, persistent)
inline constexpr const char* kSubReadinessId = "uavcan.sub.readiness.id";
/// Type name of the readiness subscription. Type: string (read-only)
inline constexpr const char* kSubReadinessType = "uavcan.sub.readiness.type";

/// d/q current setpoint of the drive.
/// Data type: unimoc.drive.CurrentSetpoint.0.1
/// Type: natural16[1] (read/write, persistent)
inline constexpr const char* kSubCurrentSetpointId = "uavcan.sub.current_setpoint.id";
/// Type name of the current setpoint subscription. Type: string (read-only)
inline constexpr const char* kSubCurrentSetpointType = "uavcan.sub.current_setpoint.type";

/// Torque setpoint, converted to i_q with the PMSM torque constant.
/// Data type: uavcan.si.unit.torque.Scalar.1.0
/// Type: natural16[1] (read/write, persistent)
inline constexpr const char* kSubTorqueSetpointId = "uavcan.sub.torque_setpoint.id";
/// Type name of the torque setpoint subscription. Type: string (read-only)
inline constexpr const char* kSubTorqueSetpointType = "uavcan.sub.torque_setpoint.type";

/// Mechanical speed setpoint (speed mode).
/// Data type: uavcan.si.unit.angular_velocity.Scalar.1.0
/// Type: natural16[1] (read/write, persistent)
inline constexpr const char* kSubSpeedSetpointId = "uavcan.sub.speed_setpoint.id";
/// Type name of the speed setpoint subscription. Type: string (read-only)
inline constexpr const char* kSubSpeedSetpointType = "uavcan.sub.speed_setpoint.type";

/// Mechanical position setpoint relative to home (position mode).
/// Data type: uavcan.si.unit.angle.Scalar.1.0
/// Type: natural16[1] (read/write, persistent)
inline constexpr const char* kSubPositionSetpointId = "uavcan.sub.position_setpoint.id";
/// Type name of the position setpoint subscription. Type: string (read-only)
inline constexpr const char* kSubPositionSetpointType = "uavcan.sub.position_setpoint.type";

/// UDRAL servo setpoint; the first finite kinematic field selects the mode.
/// Data type: reg.udral.physics.dynamics.rotation.Planar.0.1
/// Type: natural16[1] (read/write, persistent)
inline constexpr const char* kSubServoSetpointId = "uavcan.sub.servo_setpoint.id";
/// Type name of the servo setpoint subscription. Type: string (read-only)
inline constexpr const char* kSubServoSetpointType = "uavcan.sub.servo_setpoint.type";

/// Current-controller trace of the drive.
/// Data type: unimoc.drive.Telemetry.0.1
/// Type: natural16[1] (read/write, persistent)
inline constexpr const char* kPubTelemetryId = "uavcan.pub.telemetry.id";
/// Type name of the telemetry publication. Type: string (read-only)
inline constexpr const char* kPubTelemetryType = "uavcan.pub.telemetry.type";

/// Publication rate of the telemetry [Hz]; 0 stops it.
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kTelemetryRate = "unimoc.telemetry.rate";

/// Time without a new readiness or setpoint message after which the drive
/// disengages (readiness) or falls back to zero current (setpoints) [s].
/// Type: real32[1] (read/write, persistent)
inline constexpr const char* kCommandTimeout = "unimoc.control.command_timeout";

// ---- Traces (cymon, docs/cymon_integration_plan.md) ----
//
// The cymon services have no fixed port-IDs. The host tool finds them through
// uavcan.node.port.List and the `.type` registers.

/// Variable list of the traces. Data type: cymon.GetVariableList.0.1
/// Type: natural16[1] (read/write, persistent)
inline constexpr const char* kSrvCymonVariablesId = "uavcan.srv.cymon_variables.id";
/// Type name of the variable list service. Type: string (read-only)
inline constexpr const char* kSrvCymonVariablesType = "uavcan.srv.cymon_variables.type";

/// Capture limits. Data type: cymon.GetBufferInfo.0.1
/// Type: natural16[1] (read/write, persistent)
inline constexpr const char* kSrvCymonBufferInfoId = "uavcan.srv.cymon_buffer_info.id";
/// Type name of the buffer info service. Type: string (read-only)
inline constexpr const char* kSrvCymonBufferInfoType = "uavcan.srv.cymon_buffer_info.type";

/// Capture setup. Data type: cymon.SetupCapture.0.1
/// Type: natural16[1] (read/write, persistent)
inline constexpr const char* kSrvCymonSetupId = "uavcan.srv.cymon_setup.id";
/// Type name of the capture setup service. Type: string (read-only)
inline constexpr const char* kSrvCymonSetupType = "uavcan.srv.cymon_setup.type";

/// Arming of the capture trigger. Data type: cymon.ArmTrigger.0.1
/// Type: natural16[1] (read/write, persistent)
inline constexpr const char* kSrvCymonArmId = "uavcan.srv.cymon_arm.id";
/// Type name of the arm service. Type: string (read-only)
inline constexpr const char* kSrvCymonArmType = "uavcan.srv.cymon_arm.type";

/// Capture status and read-out. Data type: cymon.ReadSamples.0.1
/// Type: natural16[1] (read/write, persistent)
inline constexpr const char* kSrvCymonReadId = "uavcan.srv.cymon_read.id";
/// Type name of the read service. Type: string (read-only)
inline constexpr const char* kSrvCymonReadType = "uavcan.srv.cymon_read.type";

/// Trigger event of this node's captures. Data type: cymon.TriggerEvent.0.1
/// Type: natural16[1] (read/write, persistent)
inline constexpr const char* kPubCymonTriggerId = "uavcan.pub.cymon_trigger.id";
/// Type name of the trigger publication. Type: string (read-only)
inline constexpr const char* kPubCymonTriggerType = "uavcan.pub.cymon_trigger.type";

/// Trigger events of other nodes that trigger this node's armed capture.
/// Data type: cymon.TriggerEvent.0.1
/// Type: natural16[1] (read/write, persistent)
inline constexpr const char* kSubCymonTriggerId = "uavcan.sub.cymon_trigger.id";
/// Type name of the trigger subscription. Type: string (read-only)
inline constexpr const char* kSubCymonTriggerType = "uavcan.sub.cymon_trigger.type";

/// Hardware unique-ID of the node (MCU UID). Type: unstructured (read-only)
inline constexpr const char* kNodeUniqueId = "uavcan.node.unique_id";

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
inline constexpr uint16_t kUnsetPortId = 0xFFFFU;

/// Largest Cyphal service-ID (0..255 unregulated, 256..511 regulated).
inline constexpr uint16_t kMaxServiceId = 511U;

/// @defgroup sub_ports Subscriber (input) subjects — setpoints received by UNIMOC
/// @{

/// Torque setpoint [N m]; the drive converts it to i_q (PMSM torque constant).
/// Data type: uavcan.si.unit.torque.Scalar.1.0
/// Register: `uavcan.sub.torque_setpoint.id`
inline constexpr uint16_t kSubTorqueSetpoint = 100U;

/// Speed (mechanical angular velocity) setpoint [rad/s].
/// Data type: uavcan.si.unit.angular_velocity.Scalar.1.0
/// Register: `uavcan.sub.speed_setpoint.id`
inline constexpr uint16_t kSubSpeedSetpoint = 101U;

/// Position setpoint [rad], referenced to home.
/// Data type: uavcan.si.unit.angle.Scalar.1.0
/// Register: `uavcan.sub.position_setpoint.id`
inline constexpr uint16_t kSubPositionSetpoint = 102U;

/// Legacy explicit control mode selection: 0=TORQUE, 1=SPEED, 2=POSITION.
/// UDRAL servo mode selection should follow setpoint field precedence instead.
/// Data type: uavcan.primitive.scalar.Natural8.1.0 (legacy)
/// Register: `uavcan.sub.control_mode.id`
inline constexpr uint16_t kSubControlMode = 103U;

/// EESM rotor excitation setpoint (current [A] or flux [Wb] per mode).
/// Data type: uavcan.primitive.scalar.Real32.1.0
/// Register: `uavcan.sub.excitation_sp.id`
inline constexpr uint16_t kSubExcitationSetpoint = 104U;

/// Readiness (arming) of the drive.
/// Data type: reg.udral.service.common.Readiness.0.1
/// Register: `uavcan.sub.readiness.id`
inline constexpr uint16_t kSubReadiness = 106U;

/// d/q current setpoint.
/// Data type: unimoc.drive.CurrentSetpoint.0.1
/// Register: `uavcan.sub.current_setpoint.id`
inline constexpr uint16_t kSubCurrentSetpoint = 107U;

/// UDRAL servo setpoint (mechanical position, speed or torque).
/// Data type: reg.udral.physics.dynamics.rotation.Planar.0.1
/// Register: `uavcan.sub.servo_setpoint.id`
inline constexpr uint16_t kSubServoSetpoint = 108U;

/// Homing trigger: any value on this subject starts the homing sequence.
/// Data type: uavcan.primitive.scalar.Bit.1.0
/// Register: `uavcan.sub.homing_trigger.id`
inline constexpr uint16_t kSubHomingTrigger = 105U;

/// @}

/// @defgroup pub_ports Publisher (output) subjects — telemetry from UNIMOC
/// @{

/// Estimated rotor electrical angle [rad].
/// Data type: uavcan.si.unit.angle.Scalar.1.0
/// Register: `uavcan.pub.rotor_angle.id`
inline constexpr uint16_t kPubRotorAngle = 200U;

/// Estimated electrical angular velocity [rad/s].
/// Data type: uavcan.si.unit.angular_velocity.Scalar.1.0
/// Register: `uavcan.pub.rotor_speed.id`
inline constexpr uint16_t kPubRotorSpeed = 201U;

/// Absolute shaft position [rad], referenced to home.
/// Data type: uavcan.si.unit.angle.Scalar.1.0
/// Register: `uavcan.pub.shaft_position.id`
inline constexpr uint16_t kPubShaftPosition = 202U;

/// In-position flag.
/// Data type: uavcan.primitive.scalar.Bit.1.0
/// Register: `uavcan.pub.in_position.id`
inline constexpr uint16_t kPubInPosition = 203U;

/// Homing state: 0=IDLE, 1=SEARCHING, 2=ZEROING, 3=DONE, 4=FAULT.
/// Data type: uavcan.primitive.scalar.Natural8.1.0
/// Register: `uavcan.pub.homing_state.id`
inline constexpr uint16_t kPubHomingState = 204U;

/// DC-link voltage [V].
/// Data type: uavcan.si.unit.voltage.Scalar.1.0
/// Register: `uavcan.pub.dc_voltage.id`
inline constexpr uint16_t kPubDcVoltage = 205U;

/// Phase current magnitude [A].
/// Data type: uavcan.si.unit.electric_current.Scalar.1.0
/// Register: `uavcan.pub.phase_current.id`
inline constexpr uint16_t kPubPhaseCurrent = 206U;

/// Estimated rotor excitation current î_f [A] (EESM only).
/// Data type: uavcan.si.unit.electric_current.Scalar.1.0
/// Register: `uavcan.pub.excitation_current.id`
inline constexpr uint16_t kPubExcitationCurrent = 207U;

// ---- Motor identification / self-test result subjects ----
//
// These subjects carry the result of an identification run triggered by a
// CMD_MEASURE_* execute command.  Each subject is published exactly once at
// the end of the measurement; periodic publication is not used.
//
// If the port is disabled (UNSET_PORT_ID) the result is still applied to
// RAM / NVM according to the command `parameter` bitmask, but no Cyphal
// message is emitted.

/// Measured stator resistance R_s [Ω].
/// Published at the end of a CMD_MEASURE_RS run.
/// Data type: uavcan.primitive.scalar.Real32.1.0
/// Register: `uavcan.pub.meas_rs.id`
inline constexpr uint16_t kPubMeasRs = 208U;

/// Measured inductance pair [L_d, L_q] [H] as a 2-element real array.
/// Published at the end of a CMD_MEASURE_LD_LQ run.
/// Data type: uavcan.primitive.array.Real32.1.0 (length 2: [L_d, L_q])
/// Register: `uavcan.pub.meas_ld_lq.id`
inline constexpr uint16_t kPubMeasLdLq = 209U;

/// Measured permanent-magnet (or rotor) flux linkage ψ_PM [Wb].
/// Published at the end of a CMD_MEASURE_PSI run.
/// Data type: uavcan.primitive.scalar.Real32.1.0
/// Register: `uavcan.pub.meas_psi.id`
inline constexpr uint16_t kPubMeasPsi = 210U;

/// Measured phase current balance correction factors [gain_a, gain_b, gain_c]
/// [dimensionless, ≈ 1.0] as a 3-element real array.
/// Published at the end of a CMD_MEASURE_BALANCE run.
/// Data type: uavcan.primitive.array.Real32.1.0 (length 3: [gain_a, gain_b, gain_c])
/// Register: `uavcan.pub.meas_balance.id`
inline constexpr uint16_t kPubMeasBalance = 211U;

/// Current-controller trace.
/// Data type: unimoc.drive.Telemetry.0.1
/// Register: `uavcan.pub.telemetry.id`
inline constexpr uint16_t kPubTelemetry = 212U;

/// Plant state of the native_sim drive (simulation truth).
/// Data type: unimoc.sim.Plant.0.1
/// Register: `uavcan.pub.sim_plant.id` (native_sim only, not persistent)
inline constexpr uint16_t kPubSimPlant = 213U;

/// @}

/// @defgroup trace_ports Trace services (cymon), unregulated service-IDs
/// @{

/// Register: `uavcan.srv.cymon_variables.id` (cymon.GetVariableList.0.1)
inline constexpr uint16_t kSrvCymonVariables = 120U;
/// Register: `uavcan.srv.cymon_buffer_info.id` (cymon.GetBufferInfo.0.1)
inline constexpr uint16_t kSrvCymonBufferInfo = 121U;
/// Register: `uavcan.srv.cymon_setup.id` (cymon.SetupCapture.0.1)
inline constexpr uint16_t kSrvCymonSetup = 122U;
/// Register: `uavcan.srv.cymon_arm.id` (cymon.ArmTrigger.0.1)
inline constexpr uint16_t kSrvCymonArm = 123U;
/// Register: `uavcan.srv.cymon_read.id` (cymon.ReadSamples.0.1)
inline constexpr uint16_t kSrvCymonRead = 124U;

/// @}

// ============================================================================
// Execute Command — uavcan.node.ExecuteCommand.1.1
// ============================================================================
//
// The ExecuteCommand service (fixed service-ID 435 per Cyphal specification
// §5.3.4) lets a network manager send a command to this node.
//
// Response status codes
// ---------------------
// The response `status` field uses the values below.  Any code not listed
// here is vendor-specific and may carry additional meaning.
//
// Standard command codes
// ----------------------
// Codes 0xFF00..0xFFFF are reserved by the Cyphal specification; all other
// values below 0x8000 are vendor-specific and available to UNIMOC.
//
// Vendor-specific telemetry commands (0x0001..0x00FF)
// ---------------------------------------------------
// Sending any CMD_GET_* command causes the node to publish a single
// on-demand measurement frame on the corresponding subject even when the
// periodic publication interval is set to zero (port disabled).  The command
// parameter field is ignored.  The node responds with STATUS_SUCCESS once the
// measurement has been enqueued for transmission, or STATUS_BAD_COMMAND if
// the measurement is not available (e.g. EESM-only measurement on a PMSM
// node).
//
// Vendor-specific motor identification commands (0x0100..0x01FF)
// --------------------------------------------------------------
// Sending any CMD_MEASURE_* command starts an automated measurement
// procedure (the motor must be at rest unless otherwise specified).  The
// node responds with STATUS_SUCCESS immediately to acknowledge that the
// procedure has been started, or one of the STATUS_BAD_* codes if it cannot
// be started (e.g. motor still running).
//
// The `parameter` field of the request is a bitmask that controls what
// happens with the measurement result:
//
//   Bit 0  PARAM_APPLY_TO_RAM   — apply the result to in-RAM parameters
//                                 (takes effect immediately for the next
//                                 control cycle).
//   Bit 1  PARAM_PERSIST_TO_NVM — flush the in-RAM parameters to NVM after
//                                 applying (only meaningful when bit 0 is set).
//
// Typical usage:
//   parameter == 0x0000  → measure, publish on PUB_MEAS_* subject only;
//                           do not change any parameter.
//   parameter == 0x0001  → measure, publish, and apply to RAM (live effect)
//                           without persisting to NVM.
//   parameter == 0x0003  → measure, publish, apply to RAM, and flush to NVM.
//
// At the end of the procedure the result is published on the corresponding
// PUB_MEAS_* subject (if that port is configured / not UNSET_PORT_ID).
// ============================================================================

/// @defgroup exec_commands uavcan.node.ExecuteCommand constants
/// @{

namespace cmd {

// -------------------------------------------------------------------------
// Response status codes (uavcan.node.ExecuteCommand.1.1 Response.status)
// -------------------------------------------------------------------------

/// The command was accepted and is being executed (or has completed).
inline constexpr uint8_t kStatusSuccess = 0U;

/// The command could not be started or did not reach its outcome.
inline constexpr uint8_t kStatusFailure = 1U;

/// The command was denied for lack of authorization.
inline constexpr uint8_t kStatusNotAuthorized = 2U;

/// The command code is unknown to or not supported by this node.
inline constexpr uint8_t kStatusBadCommand = 3U;

/// The supplied parameter is out of range or malformed.
inline constexpr uint8_t kStatusBadParameter = 4U;

/// The node is not in a state that allows the command (e.g. the drive is
/// engaged when a flash save is requested).
inline constexpr uint8_t kStatusBadState = 5U;

/// The command should have succeeded but an unexpected failure occurred.
inline constexpr uint8_t kStatusInternalError = 6U;

// -------------------------------------------------------------------------
// Standard Cyphal execute command codes (Cyphal specification §5.3.4)
// -------------------------------------------------------------------------

/// Restart the node.
inline constexpr uint16_t kCommandRestart = 65535U;

/// Power off the node.
inline constexpr uint16_t kCommandPowerOff = 65534U;

/// Begin a software update via the Cyphal file-transfer protocol.
inline constexpr uint16_t kCommandBeginSoftwareUpdate = 65533U;

/// Reset all persistent parameters to factory defaults and restart.
inline constexpr uint16_t kCommandFactoryReset = 65532U;

/// Stop the motor immediately (coast to a stop, disarm the drive).
inline constexpr uint16_t kCommandEmergencyStop = 65531U;

/// Persist all current register values to non-volatile memory.
inline constexpr uint16_t kCommandStorePersistentStates = 65530U;

// -------------------------------------------------------------------------
// Vendor-specific measurement commands (on-demand single-shot publications)
// -------------------------------------------------------------------------

/// Request a single publication of the estimated rotor electrical angle [rad].
/// Subject: PUB_ROTOR_ANGLE (port 200).
inline constexpr uint16_t kCmdGetRotorAngle = 0x0001U;

/// Request a single publication of the estimated electrical angular velocity
/// [rad/s].
/// Subject: PUB_ROTOR_SPEED (port 201).
inline constexpr uint16_t kCmdGetRotorSpeed = 0x0002U;

/// Request a single publication of the absolute shaft position [rad],
/// referenced to home.
/// Subject: PUB_SHAFT_POSITION (port 202).
inline constexpr uint16_t kCmdGetShaftPosition = 0x0003U;

/// Request a single publication of the in-position flag.
/// Subject: PUB_IN_POSITION (port 203).
inline constexpr uint16_t kCmdGetInPosition = 0x0004U;

/// Request a single publication of the homing state
/// (0=IDLE, 1=SEARCHING, 2=ZEROING, 3=DONE, 4=FAULT).
/// Subject: PUB_HOMING_STATE (port 204).
inline constexpr uint16_t kCmdGetHomingState = 0x0005U;

/// Request a single publication of the DC-link voltage [V].
/// Subject: PUB_DC_VOLTAGE (port 205).
inline constexpr uint16_t kCmdGetDcVoltage = 0x0006U;

/// Request a single publication of the phase current magnitude [A].
/// Subject: PUB_PHASE_CURRENT (port 206).
inline constexpr uint16_t kCmdGetPhaseCurrent = 0x0007U;

/// Request a single publication of the estimated rotor excitation current
/// î_f [A] (EESM only; returns STATUS_BAD_STATE for PMSM/ASM nodes).
/// Subject: PUB_EXCITATION_CURRENT (port 207).
inline constexpr uint16_t kCmdGetExcitationCurrent = 0x0008U;

// -------------------------------------------------------------------------
// Vendor-specific motor identification / self-test commands (0x0100..0x01FF)
//
// All CMD_MEASURE_* commands accept a `parameter` bitmask:
//   PARAM_APPLY_TO_RAM    (bit 0) — write result into in-RAM parameters
//   PARAM_PERSIST_TO_NVM  (bit 1) — additionally flush RAM to NVM
// -------------------------------------------------------------------------

/// Bitmask bit 0: apply measurement result to in-RAM parameters.
inline constexpr uint16_t kParamApplyToRam = 0x0001U;

/// Bitmask bit 1: persist in-RAM parameters to NVM after applying.
/// Only meaningful when PARAM_APPLY_TO_RAM is also set.
inline constexpr uint16_t kParamPersistToNvm = 0x0002U;

/// Measure stator resistance R_s via DC injection (motor at rest).
/// Result published on PUB_MEAS_RS (port 208).
/// If successful, `stator_R` is updated per `parameter` bitmask.
inline constexpr uint16_t kCmdMeasureRs = 0x0100U;

/// Measure d/q-axis inductances L_d and L_q via AC impedance sweep
/// (HFI-style, motor at rest).
/// Result published on PUB_MEAS_LD_LQ (port 209) as [L_d, L_q].
/// If successful, `L_d` and `L_q` are updated per `parameter` bitmask.
inline constexpr uint16_t kCmdMeasureLdLq = 0x0101U;

/// Measure permanent-magnet (or rotor) flux linkage ψ_PM via back-EMF
/// observation (motor must be spinning at a known speed; ensure the shaft
/// is free to rotate before issuing this command).
/// Result published on PUB_MEAS_PSI (port 210).
/// If successful, `flux_pm` is updated per `parameter` bitmask.
inline constexpr uint16_t kCmdMeasurePsi = 0x0102U;

/// Measure phase current balance by injecting equal test currents and
/// comparing ADC readings across all three phases (motor at rest).
/// Result published on PUB_MEAS_BALANCE (port 211) as
/// [gain_a, gain_b, gain_c].
/// If successful, `phase_balance_a/b/c` are updated per `parameter` bitmask.
inline constexpr uint16_t kCmdMeasureBalance = 0x0103U;

}  // namespace cmd

/// @}

}  // namespace unimoc::cyphal
