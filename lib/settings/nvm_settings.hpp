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
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include "cyphal/control_mode.hpp"
#include "cyphal/cyphal_interface.hpp"
#include "cyphal/node_identity.hpp"
#include "motor_type.hpp"
#include "units.hpp"

// ============================================================================
// Standalone enums (declared before NvmSettings so they can be used as
// member types with in-class initialisers).
// ============================================================================

/**
 * @namespace settings persistent motor-control settings
 */
namespace unimoc::settings {

using namespace unit;

/// Rate of change of a torque [N m/s].
using TorqueRate = decltype(Torque{} / Time{});

/// Phase resistance of the default motor (6.5" hoverboard hub motor).
inline constexpr unit::Resistance kHoverboardR{0.15_Ohm};

/// Phase inductance of the default motor; surface magnets, so L_d = L_q.
inline constexpr unit::Inductance kHoverboardL{0.35_mH};

/// Bandwidth of the default current PI, about 320 Hz (pole-zero cancellation:
/// kp = L * w_c, ki = R * w_c).
inline constexpr unit::InverseTime kHoverboardCurrentBandwidth{2000.0_per_s};

/**
 * @brief Aggregate of all persistent (NVM-backed) UNIMOC configuration.
 *
 * Every member that a user can configure via Cyphal registers and that must
 * survive a power cycle lives in this struct.  The struct is the live (RAM)
 * copy; `SettingsStore` persists it one register per key
 * (settings_registers.hpp), only the registers that differ from the factory
 * profile.
 *
 * Cyphal register mapping
 * -----------------------
 * Each field corresponds to a named Cyphal register (see cyphal_interface.hpp
 * for the canonical register name strings).  The Cyphal register service
 * (`uavcan.register.Access`) is the write path into the live copy; saving to
 * non-volatile storage is a separate, explicit step (`SettingsOperations::SaveToFlash`).
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
 * The motor defaults describe the cart wheel: a 6.5" hoverboard hub motor
 * (36 V, 350 W, 15 pole pairs, about 16 rpm/V) with a 3.50-6 pneumatic tyre
 * (about 300 mm diameter); `kHoverboard*` below. These are typical values of
 * that motor class, not a measurement: identify the actual motor before
 * closing the loop at full current.
 */
struct NvmSettings {
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
  system::MotorType motor_type{system::MotorType::kPmsm};

  /// Motor pole-pair count.
  /// Register: `unimoc.motor.pole_pairs`
  uint8_t pole_pairs{15};

  /// Initial control mode after boot (overridden at runtime via Cyphal).
  /// Register: `unimoc.control.mode`
  cyphal::ControlMode control_mode{cyphal::ControlMode::kTorque};

  // =========================================================================
  // Stator / winding parameters (all motor types)
  // =========================================================================

  /// Stator phase resistance [Ω].
  /// Register: `unimoc.motor.stator.R`
  unit::Resistance stator_r{kHoverboardR};

  /// Stator inductance [H] (L_q for IPMSM; average (L_d+L_q)/2 otherwise).
  /// Register: `unimoc.motor.stator.L`
  unit::Inductance stator_l{kHoverboardL};

  // =========================================================================
  // Motor operating limits
  //
  // These are runtime-configurable limits; they must never exceed the
  // immutable capabilities supplied by the selected hardware profile.
  // =========================================================================

  /// Maximum motor (resultant stator vector) current [A]. 15 A (about 7.8 N m)
  /// is the usual motor current limit of a hoverboard hub motor.
  /// Hard upper bound: hardware profile's maximum motor current.
  /// Register: `unimoc.motor.limits.i_max`
  unit::Current motor_i_max{15.0_A};

  /// Maximum electrical angular velocity in the forward direction [rad/s].
  /// 1500 rad/s is 100 rad/s mechanical (955 rpm, about 54 km/h on the
  /// 3.50-6 tyre), just above the no-load speed at 54.6 V (about 91 rad/s).
  /// A motor limit; the cart speed limit belongs to the VDC.
  /// Register: `unimoc.motor.limits.omega_max`
  unit::AngularVelocity motor_omega_max{1500.0_rad_per_s};

  /// Maximum electrical angular velocity in the reverse direction [rad/s]
  /// (stored as a negative value).
  /// Register: `unimoc.motor.limits.omega_min`
  unit::AngularVelocity motor_omega_min{-1500.0_rad_per_s};

  /// Maximum battery discharge (drive) current [A].
  /// Hard upper bound: hardware profile's maximum drive current.
  /// Register: `unimoc.battery.limits.drive_current`
  unit::Current battery_drive_current_max{15.0_A};

  /// Maximum battery charge (regenerative braking) current [A].
  /// Hard upper bound: hardware profile's maximum charge current.
  /// Register: `unimoc.battery.limits.charge_current`
  unit::Current battery_charge_current_max{5.0_A};

  /// DC-link voltage above which regenerative braking is derated [V]; the
  /// charge limit falls linearly to zero at `dc_link_trip`. Above the voltage
  /// of a full battery (13s: 54.6 V).
  /// Register: `unimoc.battery.limits.regen_derate_voltage`
  unit::Voltage dc_link_regen_derate_start{55.0_V};

  /// DC-link voltage at which the control cycle trips the power stage [V].
  /// Hard upper bound: hardware profile's maximum DC-link voltage.
  /// Register: `unimoc.battery.limits.trip_voltage`
  unit::Voltage dc_link_trip{58.0_V};

  // =========================================================================
  // PMSM / EESM parameters
  // =========================================================================

  /// Permanent-magnet flux linkage ψ_PM [Wb].
  /// For EESM, this is the initial/nominal ψ_f used until the excitation
  /// observer provides a measured value.
  /// 23 mWb from 16 rpm/V at 15 pole pairs (amplitude invariant), a torque
  /// constant of 1.5 * 15 * 23 mWb = 0.52 N m/A.
  /// Register: `unimoc.motor.pmsm.flux_pm`
  unit::MagneticFlux flux_pm{23.0_mWb};

  /// d-axis inductance L_d [H].
  /// Register: `unimoc.motor.pmsm.L_d`
  unit::Inductance l_d{kHoverboardL};

  /// q-axis inductance L_q [H].
  /// Register: `unimoc.motor.pmsm.L_q`
  unit::Inductance l_q{kHoverboardL};

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
  /// The hub motor with the 3.50-6 tyre and tube alone (estimate: rotor
  /// 0.007, tyre 0.017); the share of the cart mass on the wheel is far larger
  /// (25 kg at 150 mm: 0.56 kg·m²).
  /// Register: `unimoc.motor.mechanics.J`
  unit::Inertia motor_j{2.5e-2_kg_m2};

  /// Kalman filter process noise variance Q.
  /// Increase to let the filter track faster but noisier.
  /// Register: `unimoc.observer.mech.Q`
  float mech_obs_q{1e-5F};

  /// Kalman filter measurement noise variance R.
  /// Increase to smooth angle corrections at the cost of slower tracking.
  /// Register: `unimoc.observer.mech.R`
  unit::AngleVariance mech_obs_r{1e-4_rad2};

  // =========================================================================
  // PMSM flux observer (PmsmFluxObserver) gains
  // =========================================================================

  /// d-axis anti-drift feedback gain C_d [1/s].
  /// Register: `unimoc.observer.pmsm_flux.C_d`
  unit::InverseTime pmsm_flux_obs_c_d{50.0_per_s};

  /// q-axis anti-drift feedback gain C_q [1/s].
  /// Register: `unimoc.observer.pmsm_flux.C_q`
  unit::InverseTime pmsm_flux_obs_c_q{1.0_per_s};

  /// Speed weighting of the flux angle measurement k [rad² · (rad/s)²].
  /// The flux observer has no angle information at standstill (no back-EMF);
  /// its measurement noise is R_flux = mech_obs_r + k / ω² (electrical ω).
  /// 0 keeps R_flux = mech_obs_r at every speed.
  /// Register: `unimoc.observer.pmsm_flux.R_speed`
  unit::AngleVarianceSpeedWeight pmsm_flux_obs_r_speed{0.0_rad4_per_s2};

  // =========================================================================
  // Hall sensors (fused into the mechanical observer)
  // =========================================================================

  /// 1 fuses the Hall sensors (edges and sectors) into the observer, 0 ignores them.
  /// Register: `unimoc.observer.hall.enable`
  uint8_t hall_enable{0U};

  /// Electrical angle at which Hall A rises (start of the sector of code 5).
  /// Determined once with an open-loop run (docs/hotpath_plan.md, section 5).
  /// Register: `unimoc.observer.hall.offset`
  unit::Angle hall_offset{0.0_rad};

  /// Measurement noise of a Hall edge [rad²] (alignment and capture jitter).
  /// Register: `unimoc.observer.hall.R_edge`
  unit::AngleVariance hall_r_edge{3e-3_rad2};

  /// Measurement noise of the Hall sector bound [rad²].
  /// Register: `unimoc.observer.hall.R_sector`
  unit::AngleVariance hall_r_sector{3e-2_rad2};

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
  unit::VoltagePerCurrent current_kp_d{kHoverboardL * kHoverboardCurrentBandwidth};

  /// d-axis integral gain [V/(A·s)].
  /// Register: `unimoc.control.current.ki_d`
  unit::VoltagePerCurrentTime current_ki_d{kHoverboardR * kHoverboardCurrentBandwidth};

  /// q-axis proportional gain [V/A].
  /// Register: `unimoc.control.current.kp_q`
  unit::VoltagePerCurrent current_kp_q{kHoverboardL * kHoverboardCurrentBandwidth};

  /// q-axis integral gain [V/(A·s)].
  /// Register: `unimoc.control.current.ki_q`
  unit::VoltagePerCurrentTime current_ki_q{kHoverboardR * kHoverboardCurrentBandwidth};

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
  // Speed controller (outer loop, mechanical speed)
  // =========================================================================

  /// Proportional gain [N m per rad/s of mechanical speed]. 0.3 gives
  /// about 12 rad/s bandwidth on the wheel alone (`motor_j`).
  /// Register: `unimoc.control.speed.kp`
  unit::TorquePerAngularVelocity speed_kp{0.3_Nm_s_per_rad};

  /// Integral gain 1/T_i [1/s] (T = kp · (e + ki · ∫e dt)).
  /// Register: `unimoc.control.speed.ki`
  unit::InverseTime speed_ki{5.0_per_s};

  /// Largest rate of change of the speed reference [rad/s²]; 0: no ramp.
  /// Register: `unimoc.control.speed.accel_limit`
  unit::AngularAcceleration speed_accel_limit{1000.0_rad_per_s2};

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
  // The control cycle applies them to every sample (CurrentControlIsr):
  //   i_cal = (i_raw - adc_offset_X) * adc_gain_X   (X = a, b, c)
  //   v_cal = v_raw * adc_gain_vdc
  // HwStartup itself measures the uncalibrated values.
  // =========================================================================

  /// Phase-A current-sense ADC zero offset [A].
  /// Register: `unimoc.startup.adc_offset_a`
  unit::Current adc_offset_a{0.0_A};

  /// Phase-B current-sense ADC zero offset [A].
  /// Register: `unimoc.startup.adc_offset_b`
  unit::Current adc_offset_b{0.0_A};

  /// Phase-C current-sense ADC zero offset [A].
  /// Register: `unimoc.startup.adc_offset_c`
  unit::Current adc_offset_c{0.0_A};

  /// Phase-A current-sense ADC gain correction factor [dimensionless].
  /// Apply as: i_cal_a = (i_raw_a - adc_offset_a) * adc_gain_a.
  /// Register: `unimoc.startup.gain_a`
  unit::Ratio adc_gain_a{1.0_ratio};

  /// Phase-B current-sense ADC gain correction factor [dimensionless].
  /// Register: `unimoc.startup.gain_b`
  unit::Ratio adc_gain_b{1.0_ratio};

  /// Phase-C current-sense ADC gain correction factor [dimensionless].
  /// Register: `unimoc.startup.gain_c`
  unit::Ratio adc_gain_c{1.0_ratio};

  /// DC-link voltage ADC gain correction factor [dimensionless].
  /// Apply as: v_cal = v_raw * adc_gain_vdc.
  /// Register: `unimoc.startup.gain_vdc`
  unit::Ratio adc_gain_vdc{1.0_ratio};

  // =========================================================================
  // Cyphal ports and timing
  //
  // Port-IDs are applied at the next start; 65535 (cyphal::kUnsetPortId)
  // disables the port.
  // =========================================================================

  /// Readiness subscription (reg.udral.service.common.Readiness.0.1); by
  /// default it follows the node-ID (cyphal::EffectiveReadinessId()).
  /// Register: `uavcan.sub.readiness.id`
  uint16_t sub_readiness_id{cyphal::kReadinessFollowsNodeId};

  /// Current setpoint subscription (unimoc.drive.CurrentSetpoint.0.1).
  /// Register: `uavcan.sub.current_setpoint.id`
  uint16_t sub_current_setpoint_id{cyphal::kSubCurrentSetpoint};

  /// Torque setpoint subscription (uavcan.si.unit.torque.Scalar.1.0).
  /// Register: `uavcan.sub.torque_setpoint.id`
  uint16_t sub_torque_setpoint_id{cyphal::kSubTorqueSetpoint};

  /// Speed setpoint subscription (uavcan.si.unit.angular_velocity.Scalar.1.0).
  /// Register: `uavcan.sub.speed_setpoint.id`
  uint16_t sub_speed_setpoint_id{cyphal::kSubSpeedSetpoint};

  /// Position setpoint subscription (uavcan.si.unit.angle.Scalar.1.0).
  /// Register: `uavcan.sub.position_setpoint.id`
  uint16_t sub_position_setpoint_id{cyphal::kSubPositionSetpoint};

  /// UDRAL servo setpoint subscription (reg.udral.physics.dynamics.rotation.Planar.0.1).
  /// Register: `uavcan.sub.servo_setpoint.id`
  uint16_t sub_servo_setpoint_id{cyphal::kSubServoSetpoint};

  /// Telemetry publication (unimoc.drive.Telemetry.0.1).
  /// Register: `uavcan.pub.telemetry.id`
  uint16_t pub_telemetry_id{cyphal::kPubTelemetry};

  /// Trace variable list service (cymon.GetVariableList.0.1).
  /// Register: `uavcan.srv.cymon_variables.id`
  uint16_t srv_cymon_variables_id{cyphal::kSrvCymonVariables};

  /// Trace buffer info service (cymon.GetBufferInfo.0.1).
  /// Register: `uavcan.srv.cymon_buffer_info.id`
  uint16_t srv_cymon_buffer_info_id{cyphal::kSrvCymonBufferInfo};

  /// Trace capture setup service (cymon.SetupCapture.0.1).
  /// Register: `uavcan.srv.cymon_setup.id`
  uint16_t srv_cymon_setup_id{cyphal::kSrvCymonSetup};

  /// Trace trigger arm service (cymon.ArmTrigger.0.1).
  /// Register: `uavcan.srv.cymon_arm.id`
  uint16_t srv_cymon_arm_id{cyphal::kSrvCymonArm};

  /// Trace read-out service (cymon.ReadSamples.0.1).
  /// Register: `uavcan.srv.cymon_read.id`
  uint16_t srv_cymon_read_id{cyphal::kSrvCymonRead};

  /// Trigger event publication of the traces (cymon.TriggerEvent.0.1); disabled by default.
  /// Register: `uavcan.pub.cymon_trigger.id`
  uint16_t pub_cymon_trigger_id{cyphal::kUnsetPortId};

  /// Trigger event subscription of the traces (cymon.TriggerEvent.0.1); disabled by default.
  /// Register: `uavcan.sub.cymon_trigger.id`
  uint16_t sub_cymon_trigger_id{cyphal::kUnsetPortId};

  /// Telemetry publication rate; 0 stops it.
  /// Register: `unimoc.telemetry.rate`
  unit::Frequency telemetry_rate{20.0_Hz};

  /// Readiness and setpoint timeout.
  /// Register: `unimoc.control.command_timeout`
  unit::Time command_timeout{0.5_s};

  // =========================================================================
  // Cart (docs/system_plan.md, sections 6.2 and 7)
  //
  // The drive becomes a cart wheel drive with the first wheel command after
  // it engaged; without one it follows the setpoint subjects above.
  // =========================================================================

  /// Wheel command subscription (unimoc.cart.WheelCommand.0.1).
  /// Register: `uavcan.sub.wheel_command.id`
  uint16_t sub_wheel_command_id{cyphal::kSubWheelCommand};

  /// Pedal state subscription (unimoc.cart.PedalState.0.1), for limp-home mode.
  /// Register: `uavcan.sub.pedal_state.id`
  uint16_t sub_pedal_state_id{cyphal::kSubPedalState};

  /// Battery status subscription (unimoc.cart.BatteryStatus.0.1).
  /// Register: `uavcan.sub.battery_status.id`
  uint16_t sub_battery_status_id{cyphal::kSubBatteryStatus};

  /// Drive status publication (unimoc.cart.DriveStatus.0.1).
  /// Register: `uavcan.pub.drive_status.id`
  uint16_t pub_drive_status_id{cyphal::kPubDriveStatus};

  /// Commissioning status publication (unimoc.drive.CommissioningStatus.0.1).
  /// Register: `uavcan.pub.commissioning.id`
  uint16_t pub_commissioning_id{cyphal::kPubCommissioning};

  /// Wheel of this drive (unimoc.cart.WheelCommand.WHEEL_*), 0 to 3.
  /// Register: `unimoc.cart.wheel`
  uint8_t cart_wheel{0U};

  /// 1: positive rotor torque drives the cart backward.
  /// Register: `unimoc.cart.reversed`
  uint8_t cart_reversed{0U};

  /// Timeout of the wheel command, the pedal state and the battery status;
  /// at most `command_timeout`, so limp-home starts before the readiness times out.
  /// Register: `unimoc.cart.timeout`
  unit::Time cart_timeout{0.1_s};

  /// Drive status publication rate; 0 stops it.
  /// Register: `unimoc.cart.status_rate`
  unit::Frequency cart_status_rate{50.0_Hz};

  /// Torque ramp when the source of the cart torque changes.
  /// Register: `unimoc.cart.torque_ramp`
  TorqueRate cart_torque_ramp{50.0_Nm / 1.0_s};

  /// Wheel torque of a fully pressed pedal in limp-home mode.
  /// Register: `unimoc.cart.limp_home_torque`
  unit::Torque cart_limp_home_torque{5.0_Nm};

  /// Wheel speed below which the cart stands still.
  /// Register: `unimoc.cart.standstill_speed`
  unit::AngularVelocity cart_standstill_speed{1.0_rad_per_s};

  /// Share of the BMS current limits this drive may use, 0 to 1.
  /// Register: `unimoc.cart.battery_share`
  unit::Ratio cart_battery_share{1.0_ratio};

  // =========================================================================
  // E-bike (docs/ebike_plan.md): the assist law, the sensor and the display.
  // Used with CONFIG_UNIMOC_EBIKE; untuned starting values.
  // =========================================================================

  /// Cadence pulses per crank revolution (poles of the sensor ring), 2 to 64.
  /// Register: `unimoc.ebike.pulses_per_rev`
  uint8_t ebike_pulses_per_rev{36U};

  /// Rolling circumference of the wheel [m]; the wheel size of the display has to match it.
  /// Register: `unimoc.ebike.wheel_circumference`
  float ebike_wheel_circumference{2.1F};

  /// Motor shaft revolutions per wheel revolution (1: hub motor).
  /// Register: `unimoc.ebike.gear_ratio`
  unit::Ratio ebike_gear_ratio{1.0_ratio};

  /// Series cells of the battery (battery bars of the display), 1 to 30.
  /// Register: `unimoc.ebike.battery_cells`
  uint8_t ebike_battery_cells{10U};

  /// Nominal battery voltage [V] the display is set to (24, 36, 48).
  /// Register: `unimoc.ebike.nominal_voltage`
  uint8_t ebike_nominal_voltage{36U};

  /// Assist ratio (motor power / pedal power) of display level 1.
  /// Register: `unimoc.ebike.gain_1`
  unit::Ratio ebike_gain_1{0.3_ratio};

  /// Assist ratio of display level 2.
  /// Register: `unimoc.ebike.gain_2`
  unit::Ratio ebike_gain_2{0.6_ratio};

  /// Assist ratio of display level 3.
  /// Register: `unimoc.ebike.gain_3`
  unit::Ratio ebike_gain_3{1.0_ratio};

  /// Assist ratio of display level 4.
  /// Register: `unimoc.ebike.gain_4`
  unit::Ratio ebike_gain_4{1.5_ratio};

  /// Assist ratio of display level 5; levels 6 and 7 use it too.
  /// Register: `unimoc.ebike.gain_5`
  unit::Ratio ebike_gain_5{2.0_ratio};

  /// The assist ratio grows with exp(pedal power / power_scale).
  /// Register: `unimoc.ebike.power_scale`
  unit::Power ebike_power_scale{300.0_W};

  /// Upper limit of the total assist ratio.
  /// Register: `unimoc.ebike.max_ratio`
  unit::Ratio ebike_max_ratio{4.0_ratio};

  /// Motor torque per pedal torque at a standing motor (low-speed torque assist).
  /// Register: `unimoc.ebike.torque_ratio`
  unit::Ratio ebike_torque_ratio{0.5_ratio};

  /// Pedal torque below this gives no assist.
  /// Register: `unimoc.ebike.torque_deadband`
  unit::Torque ebike_torque_deadband{2.0_Nm};

  /// No assist below this crank speed.
  /// Register: `unimoc.ebike.min_cadence`
  unit::AngularVelocity ebike_min_cadence{2.0_rad_per_s};

  /// The assist fades out over this motor speed range below the speed limit of the display.
  /// Register: `unimoc.ebike.speed_taper`
  unit::AngularVelocity ebike_speed_taper{20.0_rad_per_s};

  /// Time to ramp the assist torque from zero to the torque limit.
  /// Register: `unimoc.ebike.rise_time`
  unit::Time ebike_rise_time{0.5_s};

  /// Time to ramp the assist torque from the torque limit to zero.
  /// Register: `unimoc.ebike.fall_time`
  unit::Time ebike_fall_time{0.15_s};

  /// The drive stays engaged this long after the assist ended.
  /// Register: `unimoc.ebike.engage_hold`
  unit::Time ebike_engage_hold{2.0_s};

  /// After a fault the pedals must stand still this long before the drive engages again.
  /// Register: `unimoc.ebike.rearm_time`
  unit::Time ebike_rearm_time{1.0_s};

  /// Without a display request for this long the display counts as lost: no assist.
  /// Register: `unimoc.ebike.display_timeout`
  unit::Time ebike_display_timeout{2.0_s};

  /// Restore all fields to factory defaults.
  void ResetToDefaults() noexcept { *this = NvmSettings{}; }
};

}  // namespace unimoc::settings
