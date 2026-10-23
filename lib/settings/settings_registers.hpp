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
#include <span>
#include <string_view>
#include <type_traits>
#include "cyphal/cyphal_interface.hpp"
#include "nvm_settings.hpp"
#include "units.hpp"

namespace unimoc::settings {

/// Largest encoded register value: the node name with its terminator.
inline constexpr std::size_t kMaxRegisterValueSize = cyphal::kNodeNameMaxLen + 1U;

/**
 * @brief Calls @p visit(name, field...) for every persistent register of NvmSettings.
 *
 * The register name is the Cyphal register name (`cyphal::reg`) and also the
 * storage key. Several settings objects may be passed to visit the same
 * register of all of them at once. This is the one place that lists the
 * persistent settings: adding a register means adding a line here.
 *
 * @tparam Visitor Callable taking the name and one reference per settings object.
 * @tparam Settings `NvmSettings` or `const NvmSettings`.
 */
template <typename Visitor, typename... Settings>
void ForEachRegister(const Visitor& visit, Settings&... settings) {
  namespace reg = cyphal::reg;
  visit(reg::kNodeId, settings.node_id...);
  visit(reg::kNodeDescription, settings.identity.name...);
  visit(reg::kMotorType, settings.motor_type...);
  visit(reg::kMotorPolePairs, settings.pole_pairs...);
  visit(reg::kControlMode, settings.control_mode...);
  visit(reg::kStatorR, settings.stator_r...);
  visit(reg::kStatorL, settings.stator_l...);
  visit(reg::kMotorIMax, settings.motor_i_max...);
  visit(reg::kMotorOmegaMax, settings.motor_omega_max...);
  visit(reg::kMotorOmegaMin, settings.motor_omega_min...);
  visit(reg::kBatteryDriveIMax, settings.battery_drive_current_max...);
  visit(reg::kBatteryChargeIMax, settings.battery_charge_current_max...);
  visit(reg::kBatteryRegenDerateVoltage, settings.dc_link_regen_derate_start...);
  visit(reg::kBatteryTripVoltage, settings.dc_link_trip...);
  visit(reg::kPmsmFluxPm, settings.flux_pm...);
  visit(reg::kPmsmLD, settings.l_d...);
  visit(reg::kPmsmLQ, settings.l_q...);
  visit(reg::kAsmRR, settings.asm_r_r...);
  visit(reg::kAsmRS, settings.asm_r_s...);
  visit(reg::kAsmLS, settings.asm_l_s...);
  visit(reg::kAsmLR, settings.asm_l_r...);
  visit(reg::kAsmLM, settings.asm_l_m...);
  visit(reg::kMotorJ, settings.motor_j...);
  visit(reg::kMechObsQ, settings.mech_obs_q...);
  visit(reg::kMechObsR, settings.mech_obs_r...);
  visit(reg::kPmsmFluxObsCD, settings.pmsm_flux_obs_c_d...);
  visit(reg::kPmsmFluxObsCQ, settings.pmsm_flux_obs_c_q...);
  visit(reg::kPmsmFluxObsRSpeed, settings.pmsm_flux_obs_r_speed...);
  visit(reg::kHallEnable, settings.hall_enable...);
  visit(reg::kHallOffset, settings.hall_offset...);
  visit(reg::kHallREdge, settings.hall_r_edge...);
  visit(reg::kHallRSector, settings.hall_r_sector...);
  visit(reg::kAsmObsGI, settings.asm_obs_g_i...);
  visit(reg::kAsmObsGFlux, settings.asm_obs_g_flux...);
  visit(reg::kAsmFluxKp, settings.asm_flux_kp...);
  visit(reg::kAsmFluxKi, settings.asm_flux_ki...);
  visit(reg::kAsmFluxIDMin, settings.asm_flux_i_d_min...);
  visit(reg::kAsmFluxIDMax, settings.asm_flux_i_d_max...);
  visit(reg::kFwVMax, settings.fw_v_max...);
  visit(reg::kFwKi, settings.fw_ki...);
  visit(reg::kFwIDMin, settings.fw_i_d_min...);
  visit(reg::kPwmFrequency, settings.pwm_frequency...);
  visit(reg::kCurrentKpD, settings.current_kp_d...);
  visit(reg::kCurrentKiD, settings.current_ki_d...);
  visit(reg::kCurrentKpQ, settings.current_kp_q...);
  visit(reg::kCurrentKiQ, settings.current_ki_q...);
  visit(reg::kCurrentVMax, settings.current_v_max...);
  visit(reg::kSvmDutyMin, settings.svm_duty_min...);
  visit(reg::kSvmDutyMax, settings.svm_duty_max...);
  visit(reg::kDtcDeadTime, settings.dtc_dead_time...);
  visit(reg::kDtcFPwm, settings.dtc_f_pwm...);
  visit(reg::kDtcIThreshold, settings.dtc_i_threshold...);
  visit(reg::kHfiVInject, settings.hfi_v_inject...);
  visit(reg::kHfiErrorGain, settings.hfi_error_gain...);
  visit(reg::kExcitMode, settings.excitation_mode...);
  visit(reg::kExcitLM, settings.excitation_l_m...);
  visit(reg::kExcitKp, settings.excitation_kp...);
  visit(reg::kExcitKi, settings.excitation_ki...);
  visit(reg::kExcitIFMin, settings.excitation_i_f_min...);
  visit(reg::kExcitIFMax, settings.excitation_i_f_max...);
  visit(reg::kExcitObsTau, settings.excitation_obs_tau...);
  visit(reg::kExcitObsLM, settings.excitation_obs_l_m...);
  visit(reg::kSpeedKp, settings.speed_kp...);
  visit(reg::kSpeedKi, settings.speed_ki...);
  visit(reg::kSpeedAccelLimit, settings.speed_accel_limit...);
  visit(reg::kPosKp, settings.pos_kp_pos...);
  visit(reg::kPosKpSpeed, settings.pos_kp_speed...);
  visit(reg::kPosKiSpeed, settings.pos_ki_speed...);
  visit(reg::kPosSpeedLimit, settings.pos_speed_limit...);
  visit(reg::kPosAccelLimit, settings.pos_accel_limit...);
  visit(reg::kPosPosTol, settings.pos_position_tolerance...);
  visit(reg::kPosSpeedTol, settings.pos_speed_tolerance...);
  visit(reg::kPosHomingSpeed, settings.pos_homing_speed...);
  visit(reg::kStartupAdcOffsetA, settings.adc_offset_a...);
  visit(reg::kStartupAdcOffsetB, settings.adc_offset_b...);
  visit(reg::kStartupGainA, settings.adc_gain_a...);
  visit(reg::kStartupGainB, settings.adc_gain_b...);
  visit(reg::kStartupGainVdc, settings.adc_gain_vdc...);
  visit(reg::kBalanceGainA, settings.phase_balance_a...);
  visit(reg::kBalanceGainB, settings.phase_balance_b...);
  visit(reg::kBalanceGainC, settings.phase_balance_c...);
  visit(reg::kSubReadinessId, settings.sub_readiness_id...);
  visit(reg::kSubCurrentSetpointId, settings.sub_current_setpoint_id...);
  visit(reg::kSubTorqueSetpointId, settings.sub_torque_setpoint_id...);
  visit(reg::kSubSpeedSetpointId, settings.sub_speed_setpoint_id...);
  visit(reg::kSubPositionSetpointId, settings.sub_position_setpoint_id...);
  visit(reg::kSubServoSetpointId, settings.sub_servo_setpoint_id...);
  visit(reg::kPubTelemetryId, settings.pub_telemetry_id...);
  visit(reg::kSrvCymonVariablesId, settings.srv_cymon_variables_id...);
  visit(reg::kSrvCymonBufferInfoId, settings.srv_cymon_buffer_info_id...);
  visit(reg::kSrvCymonSetupId, settings.srv_cymon_setup_id...);
  visit(reg::kSrvCymonArmId, settings.srv_cymon_arm_id...);
  visit(reg::kSrvCymonReadId, settings.srv_cymon_read_id...);
  visit(reg::kPubCymonTriggerId, settings.pub_cymon_trigger_id...);
  visit(reg::kSubCymonTriggerId, settings.sub_cymon_trigger_id...);
  visit(reg::kTelemetryRate, settings.telemetry_rate...);
  visit(reg::kCommandTimeout, settings.command_timeout...);
  visit(reg::kSubWheelCommandId, settings.sub_wheel_command_id...);
  visit(reg::kSubPedalStateId, settings.sub_pedal_state_id...);
  visit(reg::kSubBatteryStatusId, settings.sub_battery_status_id...);
  visit(reg::kPubDriveStatusId, settings.pub_drive_status_id...);
  visit(reg::kCartWheel, settings.cart_wheel...);
  visit(reg::kCartReversed, settings.cart_reversed...);
  visit(reg::kCartTimeout, settings.cart_timeout...);
  visit(reg::kCartStatusRate, settings.cart_status_rate...);
  visit(reg::kCartTorqueRamp, settings.cart_torque_ramp...);
  visit(reg::kCartLimpHomeTorque, settings.cart_limp_home_torque...);
  visit(reg::kCartStandstillSpeed, settings.cart_standstill_speed...);
  visit(reg::kCartBatteryShare, settings.cart_battery_share...);
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
