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
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <string_view>
#include <utility>
#include "settings_codec.hpp"
#include "settings_profile.hpp"
#include "settings_storage.hpp"

namespace unimoc::settings {

/**
 * @brief Result of loading or changing settings.
 */
enum class SettingsStatus : uint8_t {
  /// The operation completed successfully.
  kSuccess,
  /// Factory settings were restored and persisted.
  kFactoryDefaults,
  /// The stored image could not be decoded.
  kInvalidImage,
  /// Settings failed structural or enum validation.
  kInvalidSettings,
  /// A setting exceeded a configured safety or range limit.
  kOutOfRange,
  /// The storage adapter is not available on this target.
  kStorageUnavailable,
  /// The storage adapter reported an I/O failure.
  kStorageError,
};

namespace settings_store_internal {

[[nodiscard]] inline bool IsFinite(const NvmSettings& settings) noexcept {
  return std::isfinite(settings.stator_r.Value()) && std::isfinite(settings.stator_l.Value()) && std::isfinite(settings.motor_i_max.Value()) &&
         std::isfinite(settings.motor_omega_max.Value()) && std::isfinite(settings.motor_omega_min.Value()) &&
         std::isfinite(settings.battery_drive_current_max.Value()) && std::isfinite(settings.battery_charge_current_max.Value()) &&
         std::isfinite(settings.flux_pm.Value()) && std::isfinite(settings.l_d.Value()) && std::isfinite(settings.l_q.Value()) &&
         std::isfinite(settings.asm_r_r.Value()) && std::isfinite(settings.asm_r_s.Value()) && std::isfinite(settings.asm_l_s.Value()) &&
         std::isfinite(settings.asm_l_r.Value()) && std::isfinite(settings.asm_l_m.Value()) && std::isfinite(settings.motor_j.Value()) &&
         std::isfinite(settings.mech_obs_q) && std::isfinite(settings.mech_obs_r) && std::isfinite(settings.pmsm_flux_obs_c_d.Value()) &&
         std::isfinite(settings.pmsm_flux_obs_c_q.Value()) && std::isfinite(settings.asm_obs_g_i.Value()) &&
         std::isfinite(settings.asm_obs_g_flux.Value()) && std::isfinite(settings.asm_flux_kp.Value()) &&
         std::isfinite(settings.asm_flux_ki.Value()) && std::isfinite(settings.asm_flux_i_d_min.Value()) &&
         std::isfinite(settings.asm_flux_i_d_max.Value()) && std::isfinite(settings.fw_v_max.Value()) && std::isfinite(settings.fw_ki.Value()) &&
         std::isfinite(settings.fw_i_d_min.Value()) && std::isfinite(settings.current_kp_d.Value()) && std::isfinite(settings.current_ki_d.Value()) &&
         std::isfinite(settings.current_kp_q.Value()) && std::isfinite(settings.current_ki_q.Value()) &&
         std::isfinite(settings.current_v_max.Value()) && std::isfinite(settings.svm_duty_min.Value()) &&
         std::isfinite(settings.svm_duty_max.Value()) && std::isfinite(settings.dtc_dead_time.Value()) && std::isfinite(settings.dtc_f_pwm.Value()) &&
         std::isfinite(settings.dtc_i_threshold.Value()) && std::isfinite(settings.hfi_v_inject.Value()) &&
         std::isfinite(settings.hfi_error_gain.Value()) && std::isfinite(settings.excitation_l_m.Value()) &&
         std::isfinite(settings.excitation_kp.Value()) && std::isfinite(settings.excitation_ki.Value()) &&
         std::isfinite(settings.excitation_i_f_min.Value()) && std::isfinite(settings.excitation_i_f_max.Value()) &&
         std::isfinite(settings.excitation_obs_tau.Value()) && std::isfinite(settings.excitation_obs_l_m.Value()) &&
         std::isfinite(settings.pos_kp_pos.Value()) && std::isfinite(settings.pos_kp_speed.Value()) && std::isfinite(settings.pos_ki_speed.Value()) &&
         std::isfinite(settings.pos_speed_limit.Value()) && std::isfinite(settings.pos_accel_limit.Value()) &&
         std::isfinite(settings.pos_position_tolerance.Value()) && std::isfinite(settings.pos_speed_tolerance.Value()) &&
         std::isfinite(settings.pos_homing_speed.Value()) && std::isfinite(settings.adc_offset_a.Value()) &&
         std::isfinite(settings.adc_offset_b.Value()) && std::isfinite(settings.adc_gain_a.Value()) && std::isfinite(settings.adc_gain_b.Value()) &&
         std::isfinite(settings.adc_gain_vdc.Value()) && std::isfinite(settings.phase_balance_a.Value()) &&
         std::isfinite(settings.phase_balance_b.Value()) && std::isfinite(settings.phase_balance_c.Value());
}

[[nodiscard]] inline bool IsValidEnumValues(const NvmSettings& settings) noexcept {
  const bool valid_motor_type = settings.motor_type == system::MotorType::PMSM || settings.motor_type == system::MotorType::ASM ||
                                settings.motor_type == system::MotorType::EESM;
  const bool valid_control_mode = settings.control_mode == cyphal::ControlMode::TORQUE || settings.control_mode == cyphal::ControlMode::SPEED ||
                                  settings.control_mode == cyphal::ControlMode::POSITION;
  const float pwm_frequency_hz = settings.pwm_frequency.Value();
  const bool valid_pwm_frequency = pwm_frequency_hz == 16000.0F || pwm_frequency_hz == 20000.0F || pwm_frequency_hz == 24000.0F ||
                                   pwm_frequency_hz == 28000.0F || pwm_frequency_hz == 32000.0F;
  return valid_motor_type && valid_control_mode && valid_pwm_frequency && settings.excitation_mode <= 1U;
}

[[nodiscard]] inline bool IsValidProfile(const SettingsProfile& profile) noexcept {
  const auto& limits = profile.capabilities;
  return profile.factory_settings.IsValid() && IsFinite(profile.factory_settings) && std::isfinite(limits.max_phase_current.Value()) &&
         std::isfinite(limits.max_motor_current.Value()) && std::isfinite(limits.max_battery_drive_current.Value()) &&
         std::isfinite(limits.max_battery_charge_current.Value()) && limits.max_phase_current.Value() > 0.0F &&
         limits.max_motor_current.Value() > 0.0F && limits.max_motor_current.Value() <= limits.max_phase_current.Value() &&
         limits.max_battery_drive_current.Value() > 0.0F && limits.max_battery_charge_current.Value() >= 0.0F;
}

[[nodiscard]] inline SettingsStatus Validate(const NvmSettings& settings, const HardwareCapabilities& limits) noexcept {
  if (!settings.IsValid() || !IsFinite(settings) || !IsValidEnumValues(settings) || settings.node_id > 127U || settings.pole_pairs == 0U)
    return SettingsStatus::kInvalidSettings;

  if (settings.motor_i_max.Value() < 0.0F || settings.motor_i_max.Value() > limits.max_motor_current.Value() ||
      settings.battery_drive_current_max.Value() < 0.0F || settings.battery_drive_current_max.Value() > limits.max_battery_drive_current.Value() ||
      settings.battery_charge_current_max.Value() < 0.0F || settings.battery_charge_current_max.Value() > limits.max_battery_charge_current.Value() ||
      settings.motor_omega_min.Value() > 0.0F || settings.motor_omega_max.Value() < 0.0F)
    return SettingsStatus::kOutOfRange;

  if (settings.stator_r.Value() <= 0.0F || settings.stator_l.Value() <= 0.0F || settings.flux_pm.Value() < 0.0F || settings.l_d.Value() <= 0.0F ||
      settings.l_q.Value() <= 0.0F || settings.asm_r_r.Value() <= 0.0F || settings.asm_r_s.Value() <= 0.0F || settings.asm_l_s.Value() <= 0.0F ||
      settings.asm_l_r.Value() <= 0.0F || settings.asm_l_m.Value() <= 0.0F || settings.motor_j.Value() <= 0.0F ||
      settings.asm_flux_i_d_min.Value() < 0.0F || settings.asm_flux_i_d_max.Value() < settings.asm_flux_i_d_min.Value() ||
      settings.excitation_i_f_min.Value() < 0.0F || settings.excitation_i_f_max.Value() < settings.excitation_i_f_min.Value() ||
      settings.dtc_dead_time.Value() < 0.0F || settings.dtc_f_pwm.Value() <= 0.0F || settings.excitation_obs_tau.Value() <= 0.0F ||
      settings.pos_position_tolerance.Value() < 0.0F || settings.pos_speed_tolerance.Value() < 0.0F)
    return SettingsStatus::kOutOfRange;

  const auto IsRatio = [](const unit::Ratio value) { return value.Value() >= 0.0F && value.Value() <= 1.0F; };
  if (settings.fw_v_max.Value() < 0.0F || !IsRatio(settings.current_v_max) || !IsRatio(settings.svm_duty_min) || !IsRatio(settings.svm_duty_max) ||
      settings.svm_duty_min > settings.svm_duty_max || settings.adc_gain_a.Value() <= 0.0F || settings.adc_gain_b.Value() <= 0.0F ||
      settings.adc_gain_vdc.Value() <= 0.0F || settings.phase_balance_a.Value() <= 0.0F || settings.phase_balance_b.Value() <= 0.0F ||
      settings.phase_balance_c.Value() <= 0.0F)
    return SettingsStatus::kOutOfRange;

  return SettingsStatus::kSuccess;
}

}  // namespace settings_store_internal

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
   * @brief Applies a validated transaction to a candidate settings record.
   * @tparam Operation Callable accepting `NvmSettings&`.
   * @param operation Operation-specific mutation applied to a copy.
   * @return The validation or persistence result.
   */
  template <typename Operation>
  SettingsStatus Update(Operation&& operation) const;

  /**
   * @brief Changes the Cyphal node ID.
   * @param node_id Node-ID in the range [0, 127], where 0 requests PnP
   *                allocation.
   * @return The validation and persistence result.
   */
  SettingsStatus SetNodeId(uint8_t node_id) const;

  /**
   * @brief Changes the configured motor current limit.
   * @param current New maximum resultant motor current.
   * @return The validation and persistence result.
   */
  SettingsStatus SetMotorCurrentLimit(unit::Current current) const;

  /**
   * @brief Changes the persisted node name.
   * @param name New UTF-8 node name; it is truncated to the protocol limit.
   * @return The validation and persistence result.
   */
  SettingsStatus SetNodeName(std::string_view name) const;

  /**
   * @brief Commits ADC calibration results as one transaction.
   * @param offset_a Phase-A current offset.
   * @param offset_b Phase-B current offset.
   * @param gain_a Phase-A current-sense gain.
   * @param gain_b Phase-B current-sense gain.
   * @param gain_vdc DC-link voltage-sense gain.
   * @return The validation and persistence result.
   */
  SettingsStatus ApplyAdcCalibration(unit::Current offset_a,
                                     unit::Current offset_b,
                                     unit::Ratio gain_a,
                                     unit::Ratio gain_b,
                                     unit::Ratio gain_vdc) const;

  /**
   * @brief Returns the immutable hardware capabilities used for validation.
   * @return Const reference to the target hardware capability limits.
   */
  [[nodiscard]] const HardwareCapabilities& GetHardwareCapabilities() const noexcept;

  /**
   * @brief Replaces settings with the immutable target factory profile.
   * @return The validation and persistence result.
   */
  SettingsStatus ResetToFactoryDefaults() const;

 private:
  explicit SettingsOperations(SettingsStore& store) noexcept : store_{&store} {}

  SettingsStore* store_;
  friend class SettingsStore;
};

/**
 * @brief Owns the active settings and mediates all persistent changes.
 */
class SettingsStore {
 public:
  /**
   * @brief Constructs a store from a target profile and storage adapter.
   * @param profile Immutable target capabilities and factory values.
   * @param storage Platform storage callbacks for the serialized image.
   */
  SettingsStore(const SettingsProfile& profile, const SettingsStorage& storage) noexcept
      : profile_{profile}, storage_{storage}, settings_{profile.factory_settings} {}

  /**
   * @brief Loads settings, falling back to the target factory profile.
   * @return The load, fallback, or storage result.
   */
  SettingsStatus Load() noexcept {
    if (!settings_store_internal::IsValidProfile(profile_)) return SettingsStatus::kInvalidSettings;

    std::array<std::byte, kSettingsImageSize> image{};
    const SettingsStorageStatus storage_status = storage_.Load(image);
    if (storage_status == SettingsStorageStatus::kSuccess) {
      NvmSettings loaded{};
      if (!SettingsCodec::Decode(image, loaded)) return RestoreFactory(SettingsStatus::kInvalidImage);

      const SettingsStatus validation = settings_store_internal::Validate(loaded, profile_.capabilities);
      if (validation != SettingsStatus::kSuccess) return RestoreFactory(validation);

      settings_ = loaded;
      return SettingsStatus::kSuccess;
    }

    if (storage_status == SettingsStorageStatus::kNotFound) return RestoreFactory(SettingsStatus::kFactoryDefaults);
    if (storage_status == SettingsStorageStatus::kUnavailable) return SettingsStatus::kStorageUnavailable;
    return SettingsStatus::kStorageError;
  }

  /**
   * @brief Returns a read-only copy of the active settings.
   * @return Snapshot containing the current settings.
   */
  [[nodiscard]] SettingsSnapshot GetSnapshot() const { return SettingsSnapshot{settings_}; }

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
  SettingsStatus RestoreFactory(const SettingsStatus fallback) noexcept {
    const SettingsStatus validation = settings_store_internal::Validate(profile_.factory_settings, profile_.capabilities);
    if (validation != SettingsStatus::kSuccess) return validation;

    const SettingsStatus save_status = Save(profile_.factory_settings);
    if (save_status != SettingsStatus::kSuccess) return save_status;
    settings_ = profile_.factory_settings;
    return fallback;
  }

  SettingsStatus Save(const NvmSettings& settings) const noexcept {
    std::array<std::byte, kSettingsImageSize> image{};
    if (!SettingsCodec::Encode(settings, image)) return SettingsStatus::kInvalidSettings;

    const SettingsStorageStatus status = storage_.Save(image);
    if (status == SettingsStorageStatus::kSuccess) return SettingsStatus::kSuccess;
    if (status == SettingsStorageStatus::kUnavailable) return SettingsStatus::kStorageUnavailable;
    return SettingsStatus::kStorageError;
  }

  SettingsStatus Commit(const NvmSettings& candidate) noexcept {
    const SettingsStatus validation = settings_store_internal::Validate(candidate, profile_.capabilities);
    if (validation != SettingsStatus::kSuccess) return validation;

    const SettingsStatus save_status = Save(candidate);
    if (save_status != SettingsStatus::kSuccess) return save_status;

    settings_ = candidate;
    return SettingsStatus::kSuccess;
  }

  SettingsProfile profile_{};
  SettingsStorage storage_{};
  NvmSettings settings_{};

  friend class SettingsOperations;
};

template <typename Operation>
SettingsStatus SettingsOperations::Update(Operation&& operation) const {
  NvmSettings candidate = store_->settings_;
  std::forward<Operation>(operation)(candidate);
  return store_->Commit(candidate);
}

inline SettingsStatus SettingsOperations::SetNodeId(uint8_t node_id) const {
  return Update([node_id](NvmSettings& settings) { settings.node_id = node_id; });
}

inline const HardwareCapabilities& SettingsOperations::GetHardwareCapabilities() const noexcept { return store_->profile_.capabilities; }

inline SettingsStatus SettingsOperations::SetMotorCurrentLimit(unit::Current current) const {
  return Update([current](NvmSettings& settings) { settings.motor_i_max = current; });
}

inline SettingsStatus SettingsOperations::SetNodeName(std::string_view name) const {
  return Update([name](NvmSettings& settings) { settings.identity.set_name(name); });
}

inline SettingsStatus SettingsOperations::ApplyAdcCalibration(unit::Current offset_a,
                                                              unit::Current offset_b,
                                                              unit::Ratio gain_a,
                                                              unit::Ratio gain_b,
                                                              unit::Ratio gain_vdc) const {
  return Update([=](NvmSettings& settings) {
    settings.adc_offset_a = offset_a;
    settings.adc_offset_b = offset_b;
    settings.adc_gain_a = gain_a;
    settings.adc_gain_b = gain_b;
    settings.adc_gain_vdc = gain_vdc;
  });
}

inline SettingsStatus SettingsOperations::ResetToFactoryDefaults() const { return store_->Commit(store_->profile_.factory_settings); }

}  // namespace unimoc::settings
