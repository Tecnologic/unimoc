/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file settings_registers.hpp
 *    @brief Register table of NvmSettings and the byte encoding of one register value.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <string_view>
#include <type_traits>
#include "cyphal/cyphal_interface.hpp"
#include "cyphal/register_description.hpp"
#include "nvm_settings.hpp"
#include "unit_symbol.hpp"
#include "units.hpp"

namespace unimoc::settings {

/// Largest encoded register value: the node name with its terminator.
inline constexpr std::size_t kMaxRegisterValueSize = cyphal::kNodeNameMaxLen + 1U;

/**
 * @brief A register of ForEachRegister(): its name and its description.
 *
 * It converts to its name, so a visitor that only needs the name takes a
 * `std::string_view`; one that needs the unit, limits, effect or help takes
 * a `const RegisterEntry&`.
 */
struct RegisterEntry {
  /// Cyphal register name and storage key.
  std::string_view name;
  /// Unit (empty: the symbol of the field's unit type), limits, effect and help.
  cyphal::RegisterDescription description;

  constexpr operator std::string_view() const noexcept { return name; }
};

/// Limits of a setting: the smallest and the largest value the node may accept.
struct RegisterLimits {
  float minimum{-std::numeric_limits<float>::infinity()};
  float maximum{std::numeric_limits<float>::infinity()};
};

namespace limits {
/// No limit beyond a finite number.
inline constexpr RegisterLimits kAny{};
[[nodiscard]] constexpr RegisterLimits AtLeast(const float minimum) noexcept {
  return {minimum, std::numeric_limits<float>::infinity()};
}
[[nodiscard]] constexpr RegisterLimits AtMost(const float maximum) noexcept {
  return {-std::numeric_limits<float>::infinity(), maximum};
}
[[nodiscard]] constexpr RegisterLimits Between(const float minimum, const float maximum) noexcept {
  return {minimum, maximum};
}
/// A port-ID: a subject-ID 0..8191 or a service-ID 0..511, 65535 disables the port.
inline constexpr RegisterLimits kPortId{0.0F, 65535.0F};
}  // namespace limits

/// A setting the drive applies while it is disengaged; a write while engaged waits for the disengage.
[[nodiscard]] constexpr RegisterEntry Setting(const std::string_view name,
                                              const std::string_view help,
                                              const RegisterLimits limits = limits::kAny,
                                              const std::string_view unit = {}) noexcept {
  return {name, {unit, limits.minimum, limits.maximum, cyphal::RegisterEffect::kDisengaged, help}};
}

/// A setting the drive takes at once, also while engaged (live tuning).
[[nodiscard]] constexpr RegisterEntry Tunable(const std::string_view name,
                                              const std::string_view help,
                                              const RegisterLimits limits = limits::kAny,
                                              const std::string_view unit = {}) noexcept {
  return {name, {unit, limits.minimum, limits.maximum, cyphal::RegisterEffect::kImmediate, help}};
}

/// A setting that takes effect after a restart (node-ID, port-IDs, name).
[[nodiscard]] constexpr RegisterEntry AfterRestart(const std::string_view name,
                                                   const std::string_view help,
                                                   const RegisterLimits limits = limits::kAny) noexcept {
  return {name, {{}, limits.minimum, limits.maximum, cyphal::RegisterEffect::kRestart, help}};
}

/// The description of a register with the unit of its field when the entry names none.
template <typename Field>
[[nodiscard]] constexpr cyphal::RegisterDescription DescriptionOf(const RegisterEntry& entry,
                                                                  const Field& /*field*/) noexcept {
  cyphal::RegisterDescription description = entry.description;
  if (description.unit.empty()) {
    description.unit = UnitSymbol<Field>();
  }
  return description;
}

/**
 * @brief Calls @p visit(entry, field...) for every persistent register of NvmSettings.
 *
 * The register name is the Cyphal register name (`cyphal::reg`) and also the
 * storage key; the entry carries the description of the register next to it
 * (unit, limits, effect, help: the register description file,
 * `docs/gui_plan.md` section 4). Several settings objects may be passed to
 * visit the same register of all of them at once. This is the one place that
 * lists the persistent settings: adding a register means adding a line here.
 *
 * The limits are those of the validation (`settings_store_internal::Validate()`)
 * that hold for the register alone; limits that depend on another register or
 * on the hardware profile are named in the help. A ztest checks that a value
 * just outside each limit is rejected.
 *
 * @tparam Visitor Callable taking the entry (or its name) and one reference per settings object.
 * @tparam Settings `NvmSettings` or `const NvmSettings`.
 */
template <typename Visitor, typename... Settings>
void ForEachRegister(const Visitor& visit, Settings&... settings) {
  namespace reg = cyphal::reg;
  using limits::AtLeast;
  using limits::AtMost;
  using limits::Between;
  using limits::kAny;
  using limits::kPortId;
  // clang-format off
  visit(AfterRestart(reg::kNodeId, "Node-ID; 0: plug-and-play allocation", Between(0.0F, 127.0F)), settings.node_id...);
  visit(AfterRestart(reg::kNodeDescription, "Node name in GetInfo: a role label, e.g. unimoc.propulsion.left"), settings.identity.name...);
  visit(Setting(reg::kMotorType, "Motor type: 0 PMSM, 1 ASM, 2 EESM", Between(0.0F, 2.0F)), settings.motor_type...);
  visit(Setting(reg::kMotorPolePairs, "Pole pairs of the motor", Between(1.0F, 255.0F)), settings.pole_pairs...);
  visit(Setting(reg::kControlMode, "Control mode after boot: 0 torque, 1 speed, 2 position", Between(0.0F, 2.0F)), settings.control_mode...);
  visit(Setting(reg::kStatorR, "Stator phase resistance", AtLeast(0.0F)), settings.stator_r...);
  visit(Setting(reg::kStatorL, "Stator inductance (L_q of an IPMSM, else the mean of L_d and L_q)", AtLeast(0.0F)), settings.stator_l...);
  visit(Setting(reg::kMotorIMax, "Largest motor current (stator vector), up to the hardware limit", AtLeast(0.0F)), settings.motor_i_max...);
  visit(Setting(reg::kMotorOmegaMax, "Largest electrical speed forward", AtLeast(0.0F)), settings.motor_omega_max...);
  visit(Setting(reg::kMotorOmegaMin, "Largest electrical speed in reverse, negative", AtMost(0.0F)), settings.motor_omega_min...);
  visit(Setting(reg::kBatteryDriveIMax, "Largest battery discharge current, up to the hardware limit", AtLeast(0.0F)), settings.battery_drive_current_max...);
  visit(Setting(reg::kBatteryChargeIMax, "Largest regenerative charge current, up to the hardware limit", AtLeast(0.0F)), settings.battery_charge_current_max...);
  visit(Setting(reg::kBatteryRegenDerateVoltage, "DC-link voltage from which regenerative braking derates to zero at the trip voltage", AtLeast(0.0F)), settings.dc_link_regen_derate_start...);
  visit(Setting(reg::kBatteryTripVoltage, "DC-link voltage that trips the power stage; above the derate voltage, up to the hardware limit", AtLeast(0.0F)), settings.dc_link_trip...);
  visit(Setting(reg::kPmsmFluxPm, "Permanent-magnet flux linkage psi_PM (EESM: the nominal excitation flux)", AtLeast(0.0F)), settings.flux_pm...);
  visit(Setting(reg::kPmsmLD, "d-axis inductance L_d", AtLeast(0.0F)), settings.l_d...);
  visit(Setting(reg::kPmsmLQ, "q-axis inductance L_q", AtLeast(0.0F)), settings.l_q...);
  visit(Setting(reg::kAsmRR, "ASM rotor resistance R_r", AtLeast(0.0F)), settings.asm_r_r...);
  visit(Setting(reg::kAsmRS, "ASM stator resistance R_s", AtLeast(0.0F)), settings.asm_r_s...);
  visit(Setting(reg::kAsmLS, "ASM stator self-inductance L_s", AtLeast(0.0F)), settings.asm_l_s...);
  visit(Setting(reg::kAsmLR, "ASM rotor self-inductance L_r", AtLeast(0.0F)), settings.asm_l_r...);
  visit(Setting(reg::kAsmLM, "ASM mutual inductance L_m", AtLeast(0.0F)), settings.asm_l_m...);
  visit(Setting(reg::kMotorJ, "Inertia of rotor and load", AtLeast(0.0F)), settings.motor_j...);
  visit(Setting(reg::kMechObsQ, "Process noise variance Q of the mechanical observer; larger tracks faster and noisier"), settings.mech_obs_q...);
  visit(Setting(reg::kMechObsR, "Angle measurement noise variance R of the mechanical observer", AtLeast(0.0F), "rad^2"), settings.mech_obs_r...);
  visit(Setting(reg::kPmsmFluxObsCD, "d-axis anti-drift gain C_d of the PMSM flux observer"), settings.pmsm_flux_obs_c_d...);
  visit(Setting(reg::kPmsmFluxObsCQ, "q-axis anti-drift gain C_q of the PMSM flux observer"), settings.pmsm_flux_obs_c_q...);
  visit(Setting(reg::kPmsmFluxObsRSpeed, "Speed weight k of the flux angle noise, R_flux = R + k / omega^2; 0: R at every speed", AtLeast(0.0F), "rad^4/s^2"), settings.pmsm_flux_obs_r_speed...);
  visit(Setting(reg::kHallEnable, "1 fuses the Hall sensors into the observer", Between(0.0F, 1.0F)), settings.hall_enable...);
  visit(Setting(reg::kHallOffset, "Electrical angle at which Hall A rises"), settings.hall_offset...);
  visit(Setting(reg::kHallREdge, "Measurement noise of a Hall edge", AtLeast(0.0F), "rad^2"), settings.hall_r_edge...);
  visit(Setting(reg::kHallRSector, "Measurement noise of the Hall sector bound", AtLeast(0.0F), "rad^2"), settings.hall_r_sector...);
  visit(Setting(reg::kAsmObsGI, "Stator current correction gain g_i of the ASM flux observer"), settings.asm_obs_g_i...);
  visit(Setting(reg::kAsmObsGFlux, "Rotor flux correction gain g_flux of the ASM flux observer", kAny, "Wb/(A s)"), settings.asm_obs_g_flux...);
  visit(Setting(reg::kAsmFluxKp, "Proportional gain of the ASM flux controller", kAny, "A/Wb"), settings.asm_flux_kp...);
  visit(Setting(reg::kAsmFluxKi, "Integral gain of the ASM flux controller", kAny, "A/(Wb s)"), settings.asm_flux_ki...);
  visit(Setting(reg::kAsmFluxIDMin, "Smallest d-axis current of the ASM flux controller", AtLeast(0.0F)), settings.asm_flux_i_d_min...);
  visit(Setting(reg::kAsmFluxIDMax, "Largest d-axis current of the ASM flux controller, at least i_d_min", AtLeast(0.0F)), settings.asm_flux_i_d_max...);
  visit(Setting(reg::kFwVMax, "Voltage vector magnitude from which field weakening starts", AtLeast(0.0F)), settings.fw_v_max...);
  visit(Setting(reg::kFwKi, "Integral gain of the field weakening", kAny, "A/(V s)"), settings.fw_ki...);
  visit(Setting(reg::kFwIDMin, "Most negative d-axis current of the field weakening"), settings.fw_i_d_min...);
  visit(Setting(reg::kPwmFrequency, "PWM frequency: 16, 20, 24, 28 or 32 kHz; the control cycle runs at half of it", Between(16000.0F, 32000.0F), "Hz"), settings.pwm_frequency...);
  visit(Tunable(reg::kCurrentKpD, "Proportional gain of the d-axis current controller", kAny, "V/A"), settings.current_kp_d...);
  visit(Tunable(reg::kCurrentKiD, "Integral gain of the d-axis current controller", kAny, "V/(A s)"), settings.current_ki_d...);
  visit(Tunable(reg::kCurrentKpQ, "Proportional gain of the q-axis current controller", kAny, "V/A"), settings.current_kp_q...);
  visit(Tunable(reg::kCurrentKiQ, "Integral gain of the q-axis current controller", kAny, "V/(A s)"), settings.current_ki_q...);
  visit(Setting(reg::kCurrentVMax, "Largest voltage vector of the current controller, a share of the DC link", Between(0.0F, 1.0F)), settings.current_v_max...);
  visit(Setting(reg::kSvmDutyMin, "Smallest duty cycle, at most duty_max", Between(0.0F, 1.0F)), settings.svm_duty_min...);
  visit(Setting(reg::kSvmDutyMax, "Largest duty cycle", Between(0.0F, 1.0F)), settings.svm_duty_max...);
  visit(Setting(reg::kDtcDeadTime, "Gate driver dead time of the dead-time compensation", AtLeast(0.0F)), settings.dtc_dead_time...);
  visit(Setting(reg::kDtcFPwm, "PWM frequency of the dead-time compensation", AtLeast(0.0F), "Hz"), settings.dtc_f_pwm...);
  visit(Setting(reg::kDtcIThreshold, "Phase current below which the dead-time compensation counts a zero crossing"), settings.dtc_i_threshold...);
  visit(Setting(reg::kHfiVInject, "Injection voltage of the HFI observer"), settings.hfi_v_inject...);
  visit(Setting(reg::kHfiErrorGain, "Current error to angle gain of the HFI observer", kAny, "rad/A"), settings.hfi_error_gain...);
  visit(Setting(reg::kExcitMode, "EESM excitation mode: 0 current, 1 flux", Between(0.0F, 1.0F)), settings.excitation_mode...);
  visit(Setting(reg::kExcitLM, "Mutual inductance L_m of the excitation controller"), settings.excitation_l_m...);
  visit(Setting(reg::kExcitKp, "Proportional gain of the excitation controller", kAny, "V/A"), settings.excitation_kp...);
  visit(Setting(reg::kExcitKi, "Integral gain of the excitation controller", kAny, "V/(A s)"), settings.excitation_ki...);
  visit(Setting(reg::kExcitIFMin, "Smallest rotor excitation current", AtLeast(0.0F)), settings.excitation_i_f_min...);
  visit(Setting(reg::kExcitIFMax, "Largest rotor excitation current, at least i_f_min", AtLeast(0.0F)), settings.excitation_i_f_max...);
  visit(Setting(reg::kExcitObsTau, "Low-pass time constant of the excitation observer", AtLeast(0.0F)), settings.excitation_obs_tau...);
  visit(Setting(reg::kExcitObsLM, "Mutual inductance L_m of the excitation observer"), settings.excitation_obs_l_m...);
  visit(Tunable(reg::kSpeedKp, "Proportional gain of the speed controller (mechanical speed)", AtLeast(0.0F), "N m s/rad"), settings.speed_kp...);
  visit(Tunable(reg::kSpeedKi, "Integral gain 1/T_i of the speed controller", AtLeast(0.0F)), settings.speed_ki...);
  visit(Tunable(reg::kSpeedAccelLimit, "Largest rate of change of the speed reference; 0: no ramp", AtLeast(0.0F)), settings.speed_accel_limit...);
  visit(Tunable(reg::kPosKp, "Proportional gain of the position loop (rad/s per rad)"), settings.pos_kp_pos...);
  visit(Setting(reg::kPosKpSpeed, "Proportional gain of the speed loop of the position controller"), settings.pos_kp_speed...);
  visit(Setting(reg::kPosKiSpeed, "Integral gain of the speed loop of the position controller"), settings.pos_ki_speed...);
  visit(Tunable(reg::kPosSpeedLimit, "Largest mechanical speed of the position controller"), settings.pos_speed_limit...);
  visit(Tunable(reg::kPosAccelLimit, "Largest rate of change of the speed demand of the position controller"), settings.pos_accel_limit...);
  visit(Tunable(reg::kPosPosTol, "Position tolerance of in-position", AtLeast(0.0F)), settings.pos_position_tolerance...);
  visit(Tunable(reg::kPosSpeedTol, "Speed tolerance of in-position", AtLeast(0.0F)), settings.pos_speed_tolerance...);
  visit(Setting(reg::kPosHomingSpeed, "Speed of the homing run"), settings.pos_homing_speed...);
  visit(Setting(reg::kStartupAdcOffsetA, "Zero offset of the phase A current sense (from the startup aid)"), settings.adc_offset_a...);
  visit(Setting(reg::kStartupAdcOffsetB, "Zero offset of the phase B current sense (from the startup aid)"), settings.adc_offset_b...);
  visit(Setting(reg::kStartupAdcOffsetC, "Zero offset of the phase C current sense (from the startup aid)"), settings.adc_offset_c...);
  visit(Setting(reg::kStartupGainA, "Gain correction of the phase A current sense", AtLeast(0.0F)), settings.adc_gain_a...);
  visit(Setting(reg::kStartupGainB, "Gain correction of the phase B current sense", AtLeast(0.0F)), settings.adc_gain_b...);
  visit(Setting(reg::kStartupGainC, "Gain correction of the phase C current sense", AtLeast(0.0F)), settings.adc_gain_c...);
  visit(Setting(reg::kStartupGainVdc, "Gain correction of the DC-link voltage sense", AtLeast(0.0F)), settings.adc_gain_vdc...);
  visit(Setting(reg::kBalanceGainA, "Balance correction of phase A, about 1", AtLeast(0.0F)), settings.phase_balance_a...);
  visit(Setting(reg::kBalanceGainB, "Balance correction of phase B, about 1", AtLeast(0.0F)), settings.phase_balance_b...);
  visit(Setting(reg::kBalanceGainC, "Balance correction of phase C, about 1", AtLeast(0.0F)), settings.phase_balance_c...);
  visit(AfterRestart(reg::kSubReadinessId, "Readiness subject; 65534: 1024 + node-ID, 65535: off", kPortId), settings.sub_readiness_id...);
  visit(AfterRestart(reg::kSubCurrentSetpointId, "Current setpoint subject; 65535: off", kPortId), settings.sub_current_setpoint_id...);
  visit(AfterRestart(reg::kSubTorqueSetpointId, "Torque setpoint subject; 65535: off", kPortId), settings.sub_torque_setpoint_id...);
  visit(AfterRestart(reg::kSubSpeedSetpointId, "Speed setpoint subject; 65535: off", kPortId), settings.sub_speed_setpoint_id...);
  visit(AfterRestart(reg::kSubPositionSetpointId, "Position setpoint subject; 65535: off", kPortId), settings.sub_position_setpoint_id...);
  visit(AfterRestart(reg::kSubServoSetpointId, "UDRAL servo setpoint subject; 65535: off", kPortId), settings.sub_servo_setpoint_id...);
  visit(AfterRestart(reg::kPubTelemetryId, "Telemetry subject; 65535: off", kPortId), settings.pub_telemetry_id...);
  visit(AfterRestart(reg::kSrvCymonVariablesId, "Trace variable list service; 65535: off", kPortId), settings.srv_cymon_variables_id...);
  visit(AfterRestart(reg::kSrvCymonBufferInfoId, "Trace buffer info service; 65535: off", kPortId), settings.srv_cymon_buffer_info_id...);
  visit(AfterRestart(reg::kSrvCymonSetupId, "Trace capture setup service; 65535: off", kPortId), settings.srv_cymon_setup_id...);
  visit(AfterRestart(reg::kSrvCymonArmId, "Trace trigger arm service; 65535: off", kPortId), settings.srv_cymon_arm_id...);
  visit(AfterRestart(reg::kSrvCymonReadId, "Trace read-out service; 65535: off", kPortId), settings.srv_cymon_read_id...);
  visit(AfterRestart(reg::kPubCymonTriggerId, "Trace trigger event subject (published); 65535: off", kPortId), settings.pub_cymon_trigger_id...);
  visit(AfterRestart(reg::kSubCymonTriggerId, "Trace trigger event subject (subscribed); 65535: off", kPortId), settings.sub_cymon_trigger_id...);
  visit(Tunable(reg::kTelemetryRate, "Telemetry rate; 0 stops it", Between(0.0F, 1000.0F), "Hz"), settings.telemetry_rate...);
  visit(Tunable(reg::kCommandTimeout, "Readiness and setpoint timeout", Between(0.0F, 10.0F)), settings.command_timeout...);
  visit(AfterRestart(reg::kSubWheelCommandId, "Cart wheel command subject; 65535: off", kPortId), settings.sub_wheel_command_id...);
  visit(AfterRestart(reg::kSubPedalStateId, "Cart pedal state subject (limp-home); 65535: off", kPortId), settings.sub_pedal_state_id...);
  visit(AfterRestart(reg::kSubBatteryStatusId, "Cart battery status subject; 65535: off", kPortId), settings.sub_battery_status_id...);
  visit(AfterRestart(reg::kPubDriveStatusId, "Cart drive status subject; 65535: off", kPortId), settings.pub_drive_status_id...);
  visit(Setting(reg::kCartWheel, "Cart wheel of this drive (WheelCommand.WHEEL_*)", Between(0.0F, 3.0F)), settings.cart_wheel...);
  visit(Setting(reg::kCartReversed, "1: positive rotor torque drives the cart backward", Between(0.0F, 1.0F)), settings.cart_reversed...);
  visit(Setting(reg::kCartTimeout, "Timeout of wheel command, pedal state and battery status; at most the command timeout", AtLeast(0.0F)), settings.cart_timeout...);
  visit(Tunable(reg::kCartStatusRate, "Drive status rate; 0 stops it", Between(0.0F, 1000.0F), "Hz"), settings.cart_status_rate...);
  visit(Tunable(reg::kCartTorqueRamp, "Torque ramp when the source of the cart torque changes", AtLeast(0.0F), "N m/s"), settings.cart_torque_ramp...);
  visit(Tunable(reg::kCartLimpHomeTorque, "Wheel torque of a fully pressed pedal in limp-home mode", AtLeast(0.0F)), settings.cart_limp_home_torque...);
  visit(Tunable(reg::kCartStandstillSpeed, "Wheel speed below which the cart stands still", AtLeast(0.0F)), settings.cart_standstill_speed...);
  visit(Setting(reg::kCartBatteryShare, "Share of the BMS current limits this drive may use", Between(0.0F, 1.0F)), settings.cart_battery_share...);
  visit(Setting(reg::kEbikePulsesPerRev, "E-bike cadence pulses per crank revolution", Between(2.0F, 64.0F)), settings.ebike_pulses_per_rev...);
  visit(Setting(reg::kEbikeWheelCircumference, "E-bike rolling circumference of the wheel; the display wheel size has to match", Between(0.5F, 4.0F), "m"), settings.ebike_wheel_circumference...);
  visit(Setting(reg::kEbikeGearRatio, "E-bike motor revolutions per wheel revolution (1: hub motor)", Between(0.0F, 100.0F)), settings.ebike_gear_ratio...);
  visit(Setting(reg::kEbikeBatteryCells, "E-bike series cells of the battery (battery bars of the display)", Between(1.0F, 30.0F)), settings.ebike_battery_cells...);
  visit(Setting(reg::kEbikeNominalVoltage, "E-bike nominal battery voltage of the display (24, 36, 48)", Between(1.0F, 255.0F), "V"), settings.ebike_nominal_voltage...);
  visit(Setting(reg::kEbikeGain1, "E-bike assist ratio (motor / pedal power) of level 1", Between(0.0F, 20.0F)), settings.ebike_gain_1...);
  visit(Setting(reg::kEbikeGain2, "E-bike assist ratio of level 2", Between(0.0F, 20.0F)), settings.ebike_gain_2...);
  visit(Setting(reg::kEbikeGain3, "E-bike assist ratio of level 3", Between(0.0F, 20.0F)), settings.ebike_gain_3...);
  visit(Setting(reg::kEbikeGain4, "E-bike assist ratio of level 4", Between(0.0F, 20.0F)), settings.ebike_gain_4...);
  visit(Setting(reg::kEbikeGain5, "E-bike assist ratio of level 5 (also 6 and 7)", Between(0.0F, 20.0F)), settings.ebike_gain_5...);
  visit(Setting(reg::kEbikePowerScale, "E-bike: the assist ratio grows with exp(pedal power / power_scale)", AtLeast(0.0F)), settings.ebike_power_scale...);
  visit(Setting(reg::kEbikeMaxRatio, "E-bike upper limit of the total assist ratio", Between(0.0F, 20.0F)), settings.ebike_max_ratio...);
  visit(Setting(reg::kEbikeTorqueRatio, "E-bike motor torque per pedal torque at a standing motor", Between(0.0F, 20.0F)), settings.ebike_torque_ratio...);
  visit(Setting(reg::kEbikeTorqueDeadband, "E-bike pedal torque below which there is no assist", AtLeast(0.0F)), settings.ebike_torque_deadband...);
  visit(Setting(reg::kEbikeMinCadence, "E-bike crank speed below which there is no assist", AtLeast(0.0F)), settings.ebike_min_cadence...);
  visit(Setting(reg::kEbikeSpeedTaper, "E-bike motor speed range over which the assist fades out below the speed limit", AtLeast(0.0F)), settings.ebike_speed_taper...);
  visit(Setting(reg::kEbikeRiseTime, "E-bike time to ramp the assist torque up to the limit", AtLeast(0.0F)), settings.ebike_rise_time...);
  visit(Setting(reg::kEbikeFallTime, "E-bike time to ramp the assist torque down to zero", AtLeast(0.0F)), settings.ebike_fall_time...);
  visit(Setting(reg::kEbikeEngageHold, "E-bike: the drive stays engaged this long after the assist ended", AtLeast(0.0F)), settings.ebike_engage_hold...);
  visit(Setting(reg::kEbikeRearmTime, "E-bike: after a fault the pedals stand still this long before the drive engages again", AtLeast(0.0F)), settings.ebike_rearm_time...);
  visit(Setting(reg::kEbikeDisplayTimeout, "E-bike: without a display request for this long the display counts as lost", AtLeast(0.0F)), settings.ebike_display_timeout...);
  // clang-format on
}

namespace settings_registers_internal {

template <typename T>
inline constexpr bool kIsEnumByte = std::is_enum_v<T> && sizeof(T) == 1U;

inline void PutFloat(const float value, std::span<std::byte, 4> out) noexcept {
  const auto kBits = std::bit_cast<uint32_t>(value);
  for (std::size_t i = 0; i < 4U; ++i) {
    out[i] = static_cast<std::byte>((kBits >> (8U * i)) & 0xFFU);
  }
}

[[nodiscard]] inline float GetFloat(std::span<const std::byte, 4> bytes) noexcept {
  uint32_t bits = 0U;
  for (std::size_t i = 0; i < 4U; ++i) {
    bits |= static_cast<uint32_t>(bytes[i]) << (8U * i);
  }
  return std::bit_cast<float>(bits);
}

}  // namespace settings_registers_internal

/**
 * @brief Encodes one register value (little endian).
 *
 * Units and floats are 4 byte IEEE floats, `uint8_t` and 8 bit enums one
 * byte, `uint16_t` (port-IDs) two bytes, the node name its characters and a
 * terminator. No value is empty:
 * the Zephyr settings subsystem treats an empty value as a deletion.
 *
 * @param value Register field.
 * @param out Destination, at least `kMaxRegisterValueSize` bytes.
 * @return Number of bytes written.
 */
template <typename T>
[[nodiscard]] std::size_t EncodeRegisterValue(const T& value, std::span<std::byte> out) noexcept {
  using settings_registers_internal::kIsEnumByte;
  if constexpr (unit::UnitLike<T>) {
    static_assert(std::is_same_v<std::remove_cvref_t<decltype(value.Value())>, float>);
    settings_registers_internal::PutFloat(value.Value(), std::span<std::byte, 4>{out.first<4>()});
    return 4U;
  } else if constexpr (std::is_same_v<T, float>) {
    settings_registers_internal::PutFloat(value, std::span<std::byte, 4>{out.first<4>()});
    return 4U;
  } else if constexpr (std::is_same_v<T, uint8_t>) {
    out[0] = static_cast<std::byte>(value);
    return 1U;
  } else if constexpr (kIsEnumByte<T>) {
    out[0] = static_cast<std::byte>(value);
    return 1U;
  } else if constexpr (std::is_same_v<T, uint16_t>) {
    out[0] = static_cast<std::byte>(value & 0xFFU);
    out[1] = static_cast<std::byte>(value >> 8U);
    return 2U;
  } else {
    // Node name: characters up to the first terminator, then the terminator.
    static_assert(std::is_same_v<T, std::array<char, cyphal::kNodeNameMaxLen + 1U>>);
    std::size_t size = 0U;
    while (size < cyphal::kNodeNameMaxLen && value[size] != '\0') {
      out[size] = static_cast<std::byte>(value[size]);
      ++size;
    }
    out[size] = std::byte{0};
    return size + 1U;
  }
}

/**
 * @brief Decodes one register value written by EncodeRegisterValue().
 * @param stored Stored bytes.
 * @param value Register field; unchanged when the bytes do not fit.
 * @return False when the size is wrong for the register type or a name is
 *         not terminated. The value is
 *         not range checked here, SettingsStore validates the whole set.
 */
template <typename T>
[[nodiscard]] bool DecodeRegisterValue(std::span<const std::byte> stored, T& value) noexcept {
  using settings_registers_internal::kIsEnumByte;
  if constexpr (unit::UnitLike<T>) {
    if (stored.size() != 4U) {
      return false;
    }
    value = T{settings_registers_internal::GetFloat(stored.first<4>())};
    return true;
  } else if constexpr (std::is_same_v<T, float>) {
    if (stored.size() != 4U) {
      return false;
    }
    value = settings_registers_internal::GetFloat(stored.first<4>());
    return true;
  } else if constexpr (std::is_same_v<T, uint8_t> || kIsEnumByte<T>) {
    if (stored.size() != 1U) {
      return false;
    }
    value = static_cast<T>(stored[0]);
    return true;
  } else if constexpr (std::is_same_v<T, uint16_t>) {
    if (stored.size() != 2U) {
      return false;
    }
    value =
        static_cast<uint16_t>(static_cast<uint16_t>(stored[0]) | (static_cast<uint16_t>(stored[1]) << 8U));
    return true;
  } else {
    static_assert(std::is_same_v<T, std::array<char, cyphal::kNodeNameMaxLen + 1U>>);
    if (stored.empty() || stored.size() > kMaxRegisterValueSize || stored.back() != std::byte{0}) {
      return false;
    }
    value.fill('\0');
    for (std::size_t i = 0; i + 1U < stored.size(); ++i) {
      value[i] = static_cast<char>(stored[i]);
    }
    return true;
  }
}

}  // namespace unimoc::settings
