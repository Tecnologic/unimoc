/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file settings_store.hpp
 *    @brief Read-only settings snapshots and authorized operations.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <span>
#include <string_view>
#include <utility>
#include "settings_profile.hpp"
#include "settings_registers.hpp"
#include "settings_storage.hpp"

namespace unimoc::settings {

/**
 * @brief Result of loading or changing settings.
 */
enum class SettingsStatus : uint8_t {
  /// The operation completed successfully.
  kSuccess,
  /// Nothing is stored; the factory settings are active.
  kFactoryDefaults,
  /// A stored entry could not be decoded; the factory settings are active.
  kInvalidImage,
  /// Settings failed structural or enum validation.
  kInvalidSettings,
  /// A setting exceeded a configured safety or range limit.
  kOutOfRange,
  /// The storage adapter is not available on this target.
  kStorageUnavailable,
  /// The storage adapter reported an I/O failure.
  kStorageError,
  /// Saving is refused because the power stage is not disabled.
  kPersistBlocked,
};

/// Calibration of the current and DC-link sense (the startup aid, `unimoc.startup.*`).
struct AdcCalibration {
  unit::Current offset_a;           ///< Zero offset of phase A.
  unit::Current offset_b;           ///< Zero offset of phase B.
  unit::Current offset_c;           ///< Zero offset of phase C.
  unit::Ratio gain_a{1.0_ratio};    ///< Gain correction of phase A.
  unit::Ratio gain_b{1.0_ratio};    ///< Gain correction of phase B.
  unit::Ratio gain_c{1.0_ratio};    ///< Gain correction of phase C.
  unit::Ratio gain_vdc{1.0_ratio};  ///< Gain correction of the DC link.
};

namespace settings_store_internal {

/// The zero of a value's unit, for sign checks without Value().
template <unit::UnitLike U>
[[nodiscard]] constexpr U ZeroOf(const U& /*unused*/) noexcept {
  return U{};
}

/// The e-bike registers hold finite numbers.
[[nodiscard]] inline bool IsFiniteEbike(const NvmSettings& settings) noexcept {
  return std::isfinite(settings.ebike_wheel_circumference) && unit::IsFinite(settings.ebike_gear_ratio) &&
         unit::IsFinite(settings.ebike_gain_1) && unit::IsFinite(settings.ebike_gain_2) &&
         unit::IsFinite(settings.ebike_gain_3) && unit::IsFinite(settings.ebike_gain_4) &&
         unit::IsFinite(settings.ebike_gain_5) && unit::IsFinite(settings.ebike_power_scale) &&
         unit::IsFinite(settings.ebike_max_ratio) && unit::IsFinite(settings.ebike_torque_ratio) &&
         unit::IsFinite(settings.ebike_torque_deadband) && unit::IsFinite(settings.ebike_min_cadence) &&
         unit::IsFinite(settings.ebike_speed_taper) && unit::IsFinite(settings.ebike_rise_time) &&
         unit::IsFinite(settings.ebike_fall_time) && unit::IsFinite(settings.ebike_engage_hold) &&
         unit::IsFinite(settings.ebike_rearm_time) && unit::IsFinite(settings.ebike_display_timeout);
}

[[nodiscard]] inline bool IsFinite(const NvmSettings& settings) noexcept {
  return unit::IsFinite(settings.stator_r) && unit::IsFinite(settings.stator_l) &&
         unit::IsFinite(settings.motor_i_max) && unit::IsFinite(settings.motor_omega_max) &&
         unit::IsFinite(settings.motor_omega_min) && unit::IsFinite(settings.battery_drive_current_max) &&
         unit::IsFinite(settings.battery_charge_current_max) && unit::IsFinite(settings.flux_pm) &&
         unit::IsFinite(settings.l_d) && unit::IsFinite(settings.l_q) && unit::IsFinite(settings.asm_r_r) &&
         unit::IsFinite(settings.asm_r_s) && unit::IsFinite(settings.asm_l_s) &&
         unit::IsFinite(settings.asm_l_r) && unit::IsFinite(settings.asm_l_m) &&
         unit::IsFinite(settings.motor_j) && std::isfinite(settings.mech_obs_q) &&
         unit::IsFinite(settings.mech_obs_r) && unit::IsFinite(settings.pmsm_flux_obs_c_d) &&
         unit::IsFinite(settings.pmsm_flux_obs_c_q) && unit::IsFinite(settings.pmsm_flux_obs_r_speed) &&
         unit::IsFinite(settings.hall_offset) && unit::IsFinite(settings.hall_r_edge) &&
         unit::IsFinite(settings.hall_r_sector) && unit::IsFinite(settings.asm_obs_g_i) &&
         unit::IsFinite(settings.asm_obs_g_flux) && unit::IsFinite(settings.asm_flux_kp) &&
         unit::IsFinite(settings.asm_flux_ki) && unit::IsFinite(settings.asm_flux_i_d_min) &&
         unit::IsFinite(settings.asm_flux_i_d_max) && unit::IsFinite(settings.fw_v_max) &&
         unit::IsFinite(settings.fw_ki) && unit::IsFinite(settings.fw_i_d_min) &&
         unit::IsFinite(settings.current_kp_d) && unit::IsFinite(settings.current_ki_d) &&
         unit::IsFinite(settings.current_kp_q) && unit::IsFinite(settings.current_ki_q) &&
         unit::IsFinite(settings.current_v_max) && unit::IsFinite(settings.svm_duty_min) &&
         unit::IsFinite(settings.svm_duty_max) && unit::IsFinite(settings.dtc_dead_time) &&
         unit::IsFinite(settings.dtc_f_pwm) && unit::IsFinite(settings.dtc_i_threshold) &&
         unit::IsFinite(settings.hfi_v_inject) && unit::IsFinite(settings.hfi_error_gain) &&
         unit::IsFinite(settings.excitation_l_m) && unit::IsFinite(settings.excitation_kp) &&
         unit::IsFinite(settings.excitation_ki) && unit::IsFinite(settings.excitation_i_f_min) &&
         unit::IsFinite(settings.excitation_i_f_max) && unit::IsFinite(settings.excitation_obs_tau) &&
         unit::IsFinite(settings.excitation_obs_l_m) && unit::IsFinite(settings.speed_kp) &&
         unit::IsFinite(settings.speed_ki) && unit::IsFinite(settings.speed_accel_limit) &&
         unit::IsFinite(settings.pos_kp_pos) && unit::IsFinite(settings.pos_kp_speed) &&
         unit::IsFinite(settings.pos_ki_speed) && unit::IsFinite(settings.pos_speed_limit) &&
         unit::IsFinite(settings.pos_accel_limit) && unit::IsFinite(settings.pos_position_tolerance) &&
         unit::IsFinite(settings.pos_speed_tolerance) && unit::IsFinite(settings.pos_homing_speed) &&
         unit::IsFinite(settings.pos_homing_current) && unit::IsFinite(settings.adc_offset_a) &&
         unit::IsFinite(settings.adc_offset_b) && unit::IsFinite(settings.adc_offset_c) &&
         unit::IsFinite(settings.adc_gain_c) && unit::IsFinite(settings.adc_gain_a) &&
         unit::IsFinite(settings.adc_gain_b) && unit::IsFinite(settings.adc_gain_vdc) &&
         unit::IsFinite(settings.telemetry_rate) && unit::IsFinite(settings.command_timeout) &&
         unit::IsFinite(settings.dc_link_regen_derate_start) && unit::IsFinite(settings.dc_link_trip) &&
         unit::IsFinite(settings.cart_timeout) && unit::IsFinite(settings.cart_status_rate) &&
         unit::IsFinite(settings.cart_torque_ramp) && unit::IsFinite(settings.cart_limp_home_torque) &&
         unit::IsFinite(settings.cart_standstill_speed) && unit::IsFinite(settings.cart_battery_share) &&
         IsFiniteEbike(settings);
}

[[nodiscard]] inline bool IsValidEnumValues(const NvmSettings& settings) noexcept {
  const bool kValidMotorType = settings.motor_type == system::MotorType::kPmsm ||
                               settings.motor_type == system::MotorType::kAsm ||
                               settings.motor_type == system::MotorType::kEesm;
  const bool kValidControlMode = settings.control_mode == cyphal::ControlMode::kTorque ||
                                 settings.control_mode == cyphal::ControlMode::kSpeed ||
                                 settings.control_mode == cyphal::ControlMode::kPosition;
  constexpr std::array<unit::Frequency, 5> kPwmFrequencies{unit::Frequency{16000.0F},
                                                           unit::Frequency{20000.0F},
                                                           unit::Frequency{24000.0F},
                                                           unit::Frequency{28000.0F},
                                                           unit::Frequency{32000.0F}};
  const bool kValidPwmFrequency =
      std::find(kPwmFrequencies.begin(), kPwmFrequencies.end(), settings.pwm_frequency) !=
      kPwmFrequencies.end();
  return kValidMotorType && kValidControlMode && kValidPwmFrequency && settings.excitation_mode <= 1U &&
         settings.hall_enable <= 1U && settings.cart_wheel <= 3U && settings.cart_reversed <= 1U;
}

[[nodiscard]] inline bool IsValidProfile(const SettingsProfile& profile) noexcept {
  const auto& limits = profile.capabilities;
  return IsFinite(profile.factory_settings) && unit::IsFinite(limits.max_phase_current) &&
         unit::IsFinite(limits.max_motor_current) && unit::IsFinite(limits.max_battery_drive_current) &&
         unit::IsFinite(limits.max_battery_charge_current) &&
         limits.max_phase_current > ZeroOf(limits.max_phase_current) &&
         limits.max_motor_current > ZeroOf(limits.max_motor_current) &&
         limits.max_motor_current <= limits.max_phase_current &&
         limits.max_battery_drive_current > ZeroOf(limits.max_battery_drive_current) &&
         limits.max_battery_charge_current >= ZeroOf(limits.max_battery_charge_current) &&
         unit::IsFinite(limits.max_dc_link_voltage) &&
         limits.max_dc_link_voltage > ZeroOf(limits.max_dc_link_voltage);
}

/// Largest Cyphal subject-ID.
inline constexpr uint16_t kMaxSubjectId = 8191U;
/// Highest telemetry rate the Cyphal thread serves.
inline constexpr unit::Frequency kMaxTelemetryRate{1000.0F};
/// Longest readiness and setpoint timeout.
inline constexpr unit::Time kMaxCommandTimeout{10.0F};
/// E-bike limits: ratios (assist, gear), pulses of the sensor ring, wheel circumference [m], series cells.
inline constexpr unit::Ratio kMaxEbikeRatio{20.0F};
inline constexpr uint8_t kMinEbikePulses = 2U;
inline constexpr uint8_t kMaxEbikePulses = 64U;
inline constexpr float kMinWheelCircumference = 0.5F;
inline constexpr float kMaxWheelCircumference = 4.0F;
inline constexpr uint8_t kMaxBatteryCells = 30U;

/// True when two enabled ports in @p port_ids share one port-ID.
template <std::size_t kCount>
[[nodiscard]] constexpr bool HasDuplicatePort(const std::array<uint16_t, kCount>& port_ids) noexcept {
  for (std::size_t i = 0U; i < kCount; ++i) {
    for (std::size_t j = i + 1U; j < kCount; ++j) {
      if (port_ids[i] != cyphal::kUnsetPortId && port_ids[i] == port_ids[j]) {
        return true;
      }
    }
  }
  return false;
}

/**
 * @brief True when two enabled ports of the drive would share one port-ID.
 *
 * The drive dispatches a received message by its subject-ID, so two
 * subscriptions on one subject would give the message to the first match
 * only, and a publication on a subscribed subject mixes two data types on
 * one subject. Subject-IDs and service-IDs are separate spaces. The one
 * allowed overlap is the cymon trigger: publishing and subscribing the same
 * `cymon.TriggerEvent` subject lets every node of a bus trigger the others
 * (a node does not receive its own transfers).
 */
[[nodiscard]] inline bool HasPortIdCollision(const NvmSettings& settings) noexcept {
  // A readiness that follows the node-ID is checked on its subject; without a
  // node-ID (0: plug-and-play) it has none yet.
  const uint16_t kReadiness =
      (settings.node_id == 0U && settings.sub_readiness_id == cyphal::kReadinessFollowsNodeId)
          ? cyphal::kUnsetPortId
          : cyphal::EffectiveReadinessId(settings.sub_readiness_id, settings.node_id);
  const std::array<uint16_t, 13U> kSubjects{kReadiness,
                                            settings.sub_current_setpoint_id,
                                            settings.sub_torque_setpoint_id,
                                            settings.sub_speed_setpoint_id,
                                            settings.sub_position_setpoint_id,
                                            settings.sub_servo_setpoint_id,
                                            settings.pub_telemetry_id,
                                            settings.sub_wheel_command_id,
                                            settings.sub_pedal_state_id,
                                            settings.sub_battery_status_id,
                                            settings.pub_drive_status_id,
                                            settings.pub_commissioning_id,
                                            settings.pub_homing_state_id};
  const std::array<uint16_t, 5U> kServices{settings.srv_cymon_variables_id,
                                           settings.srv_cymon_buffer_info_id,
                                           settings.srv_cymon_setup_id,
                                           settings.srv_cymon_arm_id,
                                           settings.srv_cymon_read_id};
  const auto kOnSubject = [&kSubjects](const uint16_t port_id) {
    return port_id != cyphal::kUnsetPortId &&
           std::find(kSubjects.begin(), kSubjects.end(), port_id) != kSubjects.end();
  };
  return HasDuplicatePort(kSubjects) || HasDuplicatePort(kServices) ||
         kOnSubject(settings.pub_cymon_trigger_id) || kOnSubject(settings.sub_cymon_trigger_id);
}

[[nodiscard]] inline SettingsStatus Validate(const NvmSettings& settings,
                                             const HardwareCapabilities& limits) noexcept {
  if (!IsFinite(settings) || !IsValidEnumValues(settings) || settings.node_id > 127U ||
      settings.pole_pairs == 0U) {
    return SettingsStatus::kInvalidSettings;
  }

  if (settings.motor_i_max < ZeroOf(settings.motor_i_max) ||
      settings.motor_i_max > limits.max_motor_current ||
      settings.battery_drive_current_max < ZeroOf(settings.battery_drive_current_max) ||
      settings.battery_drive_current_max > limits.max_battery_drive_current ||
      settings.battery_charge_current_max < ZeroOf(settings.battery_charge_current_max) ||
      settings.battery_charge_current_max > limits.max_battery_charge_current ||
      settings.motor_omega_min > ZeroOf(settings.motor_omega_min) ||
      settings.motor_omega_max < ZeroOf(settings.motor_omega_max) ||
      settings.dc_link_regen_derate_start <= ZeroOf(settings.dc_link_regen_derate_start) ||
      settings.dc_link_trip <= settings.dc_link_regen_derate_start ||
      settings.dc_link_trip > limits.max_dc_link_voltage) {
    return SettingsStatus::kOutOfRange;
  }

  if (settings.stator_r <= ZeroOf(settings.stator_r) || settings.stator_l <= ZeroOf(settings.stator_l) ||
      settings.flux_pm < ZeroOf(settings.flux_pm) || settings.l_d <= ZeroOf(settings.l_d) ||
      settings.l_q <= ZeroOf(settings.l_q) || settings.asm_r_r <= ZeroOf(settings.asm_r_r) ||
      settings.asm_r_s <= ZeroOf(settings.asm_r_s) || settings.asm_l_s <= ZeroOf(settings.asm_l_s) ||
      settings.asm_l_r <= ZeroOf(settings.asm_l_r) || settings.asm_l_m <= ZeroOf(settings.asm_l_m) ||
      settings.motor_j <= ZeroOf(settings.motor_j) ||
      settings.asm_flux_i_d_min < ZeroOf(settings.asm_flux_i_d_min) ||
      settings.asm_flux_i_d_max < settings.asm_flux_i_d_min ||
      settings.excitation_i_f_min < ZeroOf(settings.excitation_i_f_min) ||
      settings.excitation_i_f_max < settings.excitation_i_f_min ||
      settings.dtc_dead_time < ZeroOf(settings.dtc_dead_time) ||
      settings.dtc_f_pwm <= ZeroOf(settings.dtc_f_pwm) ||
      settings.excitation_obs_tau <= ZeroOf(settings.excitation_obs_tau) ||
      settings.pos_position_tolerance < ZeroOf(settings.pos_position_tolerance) ||
      settings.pos_speed_tolerance < ZeroOf(settings.pos_speed_tolerance) ||
      settings.pos_homing_current < ZeroOf(settings.pos_homing_current)) {
    return SettingsStatus::kOutOfRange;
  }

  // Noise variances are positive; gains, weights and limits are not negative.
  if (settings.mech_obs_r <= ZeroOf(settings.mech_obs_r) ||
      settings.pmsm_flux_obs_r_speed < ZeroOf(settings.pmsm_flux_obs_r_speed) ||
      settings.hall_r_edge <= ZeroOf(settings.hall_r_edge) ||
      settings.hall_r_sector <= ZeroOf(settings.hall_r_sector) ||
      settings.speed_kp < ZeroOf(settings.speed_kp) || settings.speed_ki < ZeroOf(settings.speed_ki) ||
      settings.speed_accel_limit < ZeroOf(settings.speed_accel_limit)) {
    return SettingsStatus::kOutOfRange;
  }

  const auto kIsRatio = [](const unit::Ratio value) {
    return value >= unit::Ratio{0.0F} && value <= unit::Ratio{1.0F};
  };
  if (settings.fw_v_max < ZeroOf(settings.fw_v_max) || !kIsRatio(settings.current_v_max) ||
      !kIsRatio(settings.svm_duty_min) || !kIsRatio(settings.svm_duty_max) ||
      settings.svm_duty_min > settings.svm_duty_max || settings.adc_gain_a <= ZeroOf(settings.adc_gain_a) ||
      settings.adc_gain_b <= ZeroOf(settings.adc_gain_b) ||
      settings.adc_gain_c <= ZeroOf(settings.adc_gain_c) ||
      settings.adc_gain_vdc <= ZeroOf(settings.adc_gain_vdc)) {
    return SettingsStatus::kOutOfRange;
  }

  // Cyphal subject-IDs are 0..8191; 65535 disables the port.
  const auto kIsSubjectId = [](const uint16_t port_id) {
    return port_id <= kMaxSubjectId || port_id == cyphal::kUnsetPortId;
  };
  if ((!kIsSubjectId(settings.sub_readiness_id) &&
       settings.sub_readiness_id != cyphal::kReadinessFollowsNodeId) ||
      !kIsSubjectId(settings.sub_current_setpoint_id) || !kIsSubjectId(settings.sub_torque_setpoint_id) ||
      !kIsSubjectId(settings.sub_speed_setpoint_id) || !kIsSubjectId(settings.sub_position_setpoint_id) ||
      !kIsSubjectId(settings.sub_servo_setpoint_id) || !kIsSubjectId(settings.pub_telemetry_id) ||
      settings.telemetry_rate < ZeroOf(settings.telemetry_rate) ||
      settings.telemetry_rate > kMaxTelemetryRate ||
      settings.command_timeout <= ZeroOf(settings.command_timeout) ||
      settings.command_timeout > kMaxCommandTimeout) {
    return SettingsStatus::kOutOfRange;
  }

  // Cart: limp-home must start before the readiness times out.
  if (!kIsSubjectId(settings.sub_wheel_command_id) || !kIsSubjectId(settings.sub_pedal_state_id) ||
      !kIsSubjectId(settings.sub_battery_status_id) || !kIsSubjectId(settings.pub_drive_status_id) ||
      !kIsSubjectId(settings.pub_commissioning_id) || !kIsSubjectId(settings.pub_homing_state_id) ||
      settings.cart_timeout <= ZeroOf(settings.cart_timeout) ||
      settings.cart_timeout > settings.command_timeout ||
      settings.cart_status_rate < ZeroOf(settings.cart_status_rate) ||
      settings.cart_status_rate > kMaxTelemetryRate ||
      settings.cart_torque_ramp <= ZeroOf(settings.cart_torque_ramp) ||
      settings.cart_limp_home_torque < ZeroOf(settings.cart_limp_home_torque) ||
      settings.cart_standstill_speed < ZeroOf(settings.cart_standstill_speed) ||
      !kIsRatio(settings.cart_battery_share)) {
    return SettingsStatus::kOutOfRange;
  }

  // E-bike: sensor, wheel, battery and assist parameters.
  const auto kIsGain = [](const unit::Ratio value) {
    return value >= ZeroOf(value) && value <= kMaxEbikeRatio;
  };
  if (settings.ebike_pulses_per_rev < kMinEbikePulses || settings.ebike_pulses_per_rev > kMaxEbikePulses ||
      settings.ebike_wheel_circumference < kMinWheelCircumference ||
      settings.ebike_wheel_circumference > kMaxWheelCircumference ||
      settings.ebike_gear_ratio <= ZeroOf(settings.ebike_gear_ratio) ||
      settings.ebike_gear_ratio > kMaxEbikeRatio * 5.0F || settings.ebike_battery_cells < 1U ||
      settings.ebike_battery_cells > kMaxBatteryCells || settings.ebike_nominal_voltage == 0U ||
      !kIsGain(settings.ebike_gain_1) || !kIsGain(settings.ebike_gain_2) || !kIsGain(settings.ebike_gain_3) ||
      !kIsGain(settings.ebike_gain_4) || !kIsGain(settings.ebike_gain_5) ||
      !kIsGain(settings.ebike_max_ratio) || !kIsGain(settings.ebike_torque_ratio) ||
      settings.ebike_power_scale <= ZeroOf(settings.ebike_power_scale) ||
      settings.ebike_torque_deadband < ZeroOf(settings.ebike_torque_deadband) ||
      settings.ebike_min_cadence < ZeroOf(settings.ebike_min_cadence) ||
      settings.ebike_speed_taper <= ZeroOf(settings.ebike_speed_taper) ||
      settings.ebike_rise_time <= ZeroOf(settings.ebike_rise_time) ||
      settings.ebike_fall_time <= ZeroOf(settings.ebike_fall_time) ||
      settings.ebike_engage_hold < ZeroOf(settings.ebike_engage_hold) ||
      settings.ebike_rearm_time < ZeroOf(settings.ebike_rearm_time) ||
      settings.ebike_display_timeout <= ZeroOf(settings.ebike_display_timeout)) {
    return SettingsStatus::kOutOfRange;
  }

  // Service-IDs are 0..511; 65535 disables the service.
  const auto kIsServiceId = [](const uint16_t port_id) {
    return port_id <= cyphal::kMaxServiceId || port_id == cyphal::kUnsetPortId;
  };
  if (!kIsServiceId(settings.srv_cymon_variables_id) || !kIsServiceId(settings.srv_cymon_buffer_info_id) ||
      !kIsServiceId(settings.srv_cymon_setup_id) || !kIsServiceId(settings.srv_cymon_arm_id) ||
      !kIsServiceId(settings.srv_cymon_read_id) || !kIsSubjectId(settings.pub_cymon_trigger_id) ||
      !kIsSubjectId(settings.sub_cymon_trigger_id)) {
    return SettingsStatus::kOutOfRange;
  }

  if (HasPortIdCollision(settings)) {
    return SettingsStatus::kInvalidSettings;
  }

  return SettingsStatus::kSuccess;
}

/// Number of bytes of the largest encoded register value, as a buffer type.
using RegisterBytes = std::array<std::byte, kMaxRegisterValueSize>;

/// True when two register values have the same stored representation.
template <typename T>
[[nodiscard]] bool SameEncoding(const T& first, const T& second) noexcept {
  RegisterBytes first_bytes{};
  RegisterBytes second_bytes{};
  const std::size_t kFirstSize = EncodeRegisterValue(first, first_bytes);
  const std::size_t kSecondSize = EncodeRegisterValue(second, second_bytes);
  return kFirstSize == kSecondSize &&
         std::equal(first_bytes.begin(),
                    first_bytes.begin() + static_cast<std::ptrdiff_t>(kFirstSize),
                    second_bytes.begin());
}

[[nodiscard]] constexpr SettingsStatus ToSettingsStatus(const SettingsStorageStatus status) noexcept {
  switch (status) {
    case SettingsStorageStatus::kSuccess:
    case SettingsStorageStatus::kNotFound:
      return SettingsStatus::kSuccess;
    case SettingsStorageStatus::kUnavailable:
      return SettingsStatus::kStorageUnavailable;
    case SettingsStorageStatus::kIoError:
      break;
  }
  return SettingsStatus::kStorageError;
}

}  // namespace settings_store_internal

/**
 * @brief Decides whether settings may be written to persistent storage now.
 *
 * Writing flash can stall the CPU for milliseconds, which the zero-latency
 * control cycle cannot tolerate, so the application allows it only while the
 * power stage is disabled. The gate brackets a whole save: TryBegin() checks
 * the condition and, on success, holds it (the power stage cannot be enabled)
 * until End(). A default-constructed gate never allows saving.
 */
class SettingsSaveGate {
 public:
  /// Returns true and holds the save condition when saving is allowed now.
  using BeginCallback = bool (*)(void* context);
  /// Releases the save condition taken by a successful BeginCallback.
  using EndCallback = void (*)(void* context);

  constexpr SettingsSaveGate() = default;

  /**
   * @brief Constructs a gate from platform callbacks.
   * @param context Opaque callback context.
   * @param begin Returns true and holds the condition while saving is safe.
   * @param end Releases the condition; may be null when nothing is held.
   */
  constexpr SettingsSaveGate(void* context, BeginCallback begin, EndCallback end = nullptr) noexcept
      : context_{context}, begin_{begin}, end_{end} {}

  /// A gate that always allows saving (host tests, tools without a power stage).
  [[nodiscard]] static constexpr SettingsSaveGate Always() noexcept {
    return SettingsSaveGate{nullptr, [](void* /*unused*/) { return true; }};
  }

  /// Returns true and holds the save condition until End() when saving is allowed now.
  [[nodiscard]] bool TryBegin() const noexcept { return begin_ != nullptr && begin_(context_); }

  /// Releases the condition taken by a successful TryBegin().
  void End() const noexcept {
    if (end_ != nullptr) {
      end_(context_);
    }
  }

 private:
  void* context_{nullptr};
  BeginCallback begin_{nullptr};
  EndCallback end_{nullptr};
};

/**
 * @brief Immutable copy of the active settings for application consumers.
 *
 * The only settings access available through this type is a const reference.
 * It cannot be used to mutate the store or obtain a mutable settings object.
 */
class SettingsSnapshot {
 public:
  /**
   * @brief Returns the active settings as a read-only record.
   * @return Const reference to the snapshot's immutable settings record.
   */
  [[nodiscard]] const NvmSettings& Get() const noexcept { return settings_; }

 private:
  explicit SettingsSnapshot(const NvmSettings& settings) : settings_{settings} {}

  NvmSettings settings_{};
  friend class SettingsStore;
};

class SettingsStore;

/**
 * @brief Authorized mutation capability for a SettingsStore.
 *
 * This object should be passed only to operation endpoints such as Cyphal,
 * startup calibration, or a manufacturing tool. Application control code
 * should receive SettingsSnapshot instead.
 */
class SettingsOperations {
 public:
  /**
   * @brief Applies a validated transaction to the live settings.
   *
   * The live settings are RAM only and take effect at once (live tuning).
   * Nothing is written to flash until SaveToFlash().
   *
   * @tparam Operation Callable accepting `NvmSettings&`.
   * @param operation Operation-specific mutation applied to a copy. It runs
   *                  under the store lock and must not call into the store.
   * @return The validation result.
   */
  template <typename Operation>
  SettingsStatus Update(Operation&& operation) const;

  /**
   * @brief Changes the Cyphal node ID.
   * @param node_id Node-ID in the range [0, 127], where 0 requests PnP
   *                allocation.
   * @return The validation result.
   */
  [[nodiscard]] SettingsStatus SetNodeId(uint8_t node_id) const;

  /**
   * @brief Changes the configured motor current limit.
   * @param current New maximum resultant motor current.
   * @return The validation result.
   */
  [[nodiscard]] SettingsStatus SetMotorCurrentLimit(unit::Current current) const;

  /**
   * @brief Changes the persisted node name.
   * @param name New UTF-8 node name; it is truncated to the protocol limit.
   * @return The validation result.
   */
  [[nodiscard]] SettingsStatus SetNodeName(std::string_view name) const;

  /**
   * @brief Commits ADC calibration results as one transaction.
   * @param calibration Offsets and gain corrections of the startup aid.
   * @return The validation result.
   */
  [[nodiscard]] SettingsStatus ApplyAdcCalibration(const AdcCalibration& calibration) const;

  /**
   * @brief Returns the immutable hardware capabilities used for validation.
   * @return Const reference to the target hardware capability limits.
   */
  [[nodiscard]] const HardwareCapabilities& GetHardwareCapabilities() const noexcept;

  /**
   * @brief Replaces the live settings with the immutable target factory profile.
   *
   * Flash is untouched until SaveToFlash(), which then erases the stored
   * registers.
   *
   * @return The validation result.
   */
  [[nodiscard]] SettingsStatus ResetToFactoryDefaults() const;

  /**
   * @brief Writes the registers that changed since the last load or save to flash.
   *
   * Only registers whose live value differs from the stored one are written;
   * a register that is back at its factory value is erased instead. Refused
   * with `kPersistBlocked` unless the save gate allows it (power stage
   * disabled); the gate is held for the whole save, so the power stage cannot
   * be enabled while flash is written. Registers saved before an error stay
   * saved.
   *
   * @return `kSuccess`, `kPersistBlocked`, or the storage error.
   */
  [[nodiscard]] SettingsStatus SaveToFlash() const;

  /**
   * @brief Counts the registers whose live value differs from the stored one.
   * @return Number of registers SaveToFlash() would write or erase.
   */
  [[nodiscard]] std::size_t UnsavedCount() const;

 private:
  explicit SettingsOperations(SettingsStore& store) noexcept : store_{&store} {}

  SettingsStore* store_;
  friend class SettingsStore;
};

/**
 * @brief Owns the live settings and mediates all changes and their persistence.
 *
 * Two copies are kept:
 *
 * - the live settings (RAM), which the controller uses and which operations
 *   change at once and without any flash access (live tuning);
 * - a shadow of what the storage holds, which tells SaveToFlash() what changed.
 *
 * Storage is one entry per register, only for registers that differ from the
 * factory profile: a register that is not stored has its factory value, so a
 * new firmware can add registers without invalidating stored settings.
 *
 * Every public operation runs under the SettingsLock given at construction,
 * so several threads may share the store. Without a lock, serialize the calls
 * (one settings thread).
 */
class SettingsStore {
 public:
  /**
   * @brief Constructs a store from a target profile and storage adapter.
   * @param profile Immutable target capabilities and factory values.
   * @param storage Platform key/value storage.
   * @param save_gate Decides when flash may be written (power stage disabled).
   * @param lock Serializes the threads that use the store; none by default.
   */
  SettingsStore(const SettingsProfile& profile,
                const SettingsStorage& storage,
                const SettingsSaveGate& save_gate,
                const SettingsLock& lock = SettingsLock{}) noexcept
      : profile_{profile},
        storage_{storage},
        save_gate_{save_gate},
        lock_{lock},
        settings_{profile.factory_settings},
        stored_{profile.factory_settings} {}

  /**
   * @brief Loads the stored registers over the factory profile.
   *
   * Unknown keys are ignored and missing keys keep their factory value. If
   * an entry cannot be decoded or the result is invalid, the live settings
   * are the factory profile and the cause is returned.
   *
   * @return `kSuccess` when registers were loaded, `kFactoryDefaults` when
   *         nothing is stored, or the cause of a fallback or storage error.
   */
  SettingsStatus Load() noexcept {
    const Guard kGuard{lock_};
    if (!settings_store_internal::IsValidProfile(profile_)) {
      return SettingsStatus::kInvalidSettings;
    }

    settings_ = profile_.factory_settings;
    stored_ = profile_.factory_settings;
    erase_defaults_ = false;

    NvmSettings loaded = profile_.factory_settings;
    std::size_t loaded_count = 0U;
    bool undecodable = false;
    const SettingsStorageStatus kStorageStatus =
        storage_.LoadAll([&](std::string_view key, std::span<const std::byte> value) {
          ForEachRegister(
              [&](std::string_view name, auto& field) {
                if (name != key) {
                  return;
                }
                ++loaded_count;
                undecodable = !DecodeRegisterValue(value, field) || undecodable;
              },
              loaded);
        });
    if (kStorageStatus != SettingsStorageStatus::kSuccess &&
        kStorageStatus != SettingsStorageStatus::kNotFound) {
      return settings_store_internal::ToSettingsStatus(kStorageStatus);
    }

    stored_ = loaded;
    // An undecodable entry cannot be mirrored: the next save erases the
    // registers that are at factory value, which removes it.
    erase_defaults_ = undecodable;
    const SettingsStatus kValidation = undecodable
                                           ? SettingsStatus::kInvalidImage
                                           : settings_store_internal::Validate(loaded, profile_.capabilities);
    if (kValidation != SettingsStatus::kSuccess) {
      return kValidation;
    }

    settings_ = loaded;
    return loaded_count == 0U ? SettingsStatus::kFactoryDefaults : SettingsStatus::kSuccess;
  }

  /**
   * @brief Returns a read-only copy of the live settings.
   * @return Snapshot containing the current settings.
   */
  [[nodiscard]] SettingsSnapshot GetSnapshot() const {
    const Guard kGuard{lock_};
    return SettingsSnapshot{settings_};
  }

  /**
   * @brief Creates the authorized operation capability.
   * @return Operations object that can commit validated changes.
   */
  [[nodiscard]] SettingsOperations GetOperations() noexcept { return SettingsOperations{*this}; }

  /**
   * @brief Returns the immutable target profile.
   * @return Const reference to the profile used by this store.
   */
  [[nodiscard]] const SettingsProfile& GetProfile() const noexcept { return profile_; }

 private:
  /// Holds the store lock for one public operation.
  class Guard {
   public:
    explicit Guard(const SettingsLock& lock) noexcept : lock_{&lock} { lock_->Lock(); }
    ~Guard() { lock_->Unlock(); }
    Guard(const Guard&) = delete;
    Guard& operator=(const Guard&) = delete;
    Guard(Guard&&) = delete;
    Guard& operator=(Guard&&) = delete;

   private:
    const SettingsLock* lock_;
  };

  SettingsStatus Commit(const NvmSettings& candidate) noexcept {
    const SettingsStatus kValidation = settings_store_internal::Validate(candidate, profile_.capabilities);
    if (kValidation != SettingsStatus::kSuccess) {
      return kValidation;
    }
    settings_ = candidate;
    return SettingsStatus::kSuccess;
  }

  SettingsStatus SaveChanged() noexcept {
    SettingsStatus result = SettingsStatus::kSuccess;
    bool saving = false;  // the gate is held from the first write to the end
    ForEachRegister(
        [&](std::string_view key, const auto& live, auto& stored, const auto& factory) {
          if (result != SettingsStatus::kSuccess) {
            return;
          }
          const bool kAtFactory = settings_store_internal::SameEncoding(live, factory);
          const bool kChanged = !settings_store_internal::SameEncoding(live, stored);
          if (!kChanged && !(erase_defaults_ && kAtFactory)) {
            return;
          }
          // Taken before the first write and held to the end: the power stage
          // cannot be enabled while flash is written.
          if (!saving) {
            if (!save_gate_.TryBegin()) {
              result = SettingsStatus::kPersistBlocked;
              return;
            }
            saving = true;
          }
          SettingsStorageStatus status = SettingsStorageStatus::kSuccess;
          if (kAtFactory) {
            status = storage_.Erase(key);
          } else {
            settings_store_internal::RegisterBytes bytes{};
            const std::size_t kSize = EncodeRegisterValue(live, bytes);
            status = storage_.Write(key, std::span<const std::byte>{bytes.data(), kSize});
          }
          result = settings_store_internal::ToSettingsStatus(status);
          if (result == SettingsStatus::kSuccess) {
            stored = live;
          }
        },
        std::as_const(settings_),
        stored_,
        std::as_const(profile_.factory_settings));
    if (saving) {
      save_gate_.End();
    }
    if (result == SettingsStatus::kSuccess) {
      erase_defaults_ = false;
    }
    return result;
  }

  [[nodiscard]] std::size_t CountUnsaved() const noexcept {
    std::size_t count = 0U;
    ForEachRegister(
        [&](std::string_view /*key*/, const auto& live, const auto& stored) {
          count += settings_store_internal::SameEncoding(live, stored) ? 0U : 1U;
        },
        settings_,
        stored_);
    return count;
  }

  SettingsProfile profile_{};
  SettingsStorage storage_;
  SettingsSaveGate save_gate_;
  SettingsLock lock_;
  NvmSettings settings_{};      ///< Live settings (RAM).
  NvmSettings stored_{};        ///< What the storage holds, as far as known.
  bool erase_defaults_{false};  ///< Erase registers at factory value on the next save.

  friend class SettingsOperations;
};

template <typename Operation>
SettingsStatus SettingsOperations::Update(Operation&& operation) const {
  const SettingsStore::Guard kGuard{store_->lock_};
  NvmSettings candidate = store_->settings_;
  std::forward<Operation>(operation)(candidate);
  return store_->Commit(candidate);
}

inline SettingsStatus SettingsOperations::SetNodeId(uint8_t node_id) const {
  return Update([node_id](NvmSettings& settings) { settings.node_id = node_id; });
}

inline const HardwareCapabilities& SettingsOperations::GetHardwareCapabilities() const noexcept {
  return store_->profile_.capabilities;
}

inline SettingsStatus SettingsOperations::SetMotorCurrentLimit(unit::Current current) const {
  return Update([current](NvmSettings& settings) { settings.motor_i_max = current; });
}

inline SettingsStatus SettingsOperations::SetNodeName(std::string_view name) const {
  return Update([name](NvmSettings& settings) { settings.identity.SetName(name); });
}

inline SettingsStatus SettingsOperations::ApplyAdcCalibration(const AdcCalibration& calibration) const {
  return Update([&calibration](NvmSettings& settings) {
    settings.adc_offset_a = calibration.offset_a;
    settings.adc_offset_b = calibration.offset_b;
    settings.adc_offset_c = calibration.offset_c;
    settings.adc_gain_a = calibration.gain_a;
    settings.adc_gain_b = calibration.gain_b;
    settings.adc_gain_c = calibration.gain_c;
    settings.adc_gain_vdc = calibration.gain_vdc;
  });
}

inline SettingsStatus SettingsOperations::ResetToFactoryDefaults() const {
  const SettingsStore::Guard kGuard{store_->lock_};
  return store_->Commit(store_->profile_.factory_settings);
}

inline SettingsStatus SettingsOperations::SaveToFlash() const {
  const SettingsStore::Guard kGuard{store_->lock_};
  return store_->SaveChanged();
}

inline std::size_t SettingsOperations::UnsavedCount() const {
  const SettingsStore::Guard kGuard{store_->lock_};
  return store_->CountUnsaved();
}

}  // namespace unimoc::settings
