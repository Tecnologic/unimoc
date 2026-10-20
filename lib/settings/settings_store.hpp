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

namespace settings_store_internal {

[[nodiscard]] inline bool IsFinite(const NvmSettings& settings) noexcept {
  return std::isfinite(settings.stator_r.Value()) && std::isfinite(settings.stator_l.Value()) &&
         std::isfinite(settings.motor_i_max.Value()) && std::isfinite(settings.motor_omega_max.Value()) &&
         std::isfinite(settings.motor_omega_min.Value()) &&
         std::isfinite(settings.battery_drive_current_max.Value()) &&
         std::isfinite(settings.battery_charge_current_max.Value()) &&
         std::isfinite(settings.flux_pm.Value()) && std::isfinite(settings.l_d.Value()) &&
         std::isfinite(settings.l_q.Value()) && std::isfinite(settings.asm_r_r.Value()) &&
         std::isfinite(settings.asm_r_s.Value()) && std::isfinite(settings.asm_l_s.Value()) &&
         std::isfinite(settings.asm_l_r.Value()) && std::isfinite(settings.asm_l_m.Value()) &&
         std::isfinite(settings.motor_j.Value()) && std::isfinite(settings.mech_obs_q) &&
         std::isfinite(settings.mech_obs_r) && std::isfinite(settings.pmsm_flux_obs_c_d.Value()) &&
         std::isfinite(settings.pmsm_flux_obs_c_q.Value()) && std::isfinite(settings.asm_obs_g_i.Value()) &&
         std::isfinite(settings.asm_obs_g_flux.Value()) && std::isfinite(settings.asm_flux_kp.Value()) &&
         std::isfinite(settings.asm_flux_ki.Value()) && std::isfinite(settings.asm_flux_i_d_min.Value()) &&
         std::isfinite(settings.asm_flux_i_d_max.Value()) && std::isfinite(settings.fw_v_max.Value()) &&
         std::isfinite(settings.fw_ki.Value()) && std::isfinite(settings.fw_i_d_min.Value()) &&
         std::isfinite(settings.current_kp_d.Value()) && std::isfinite(settings.current_ki_d.Value()) &&
         std::isfinite(settings.current_kp_q.Value()) && std::isfinite(settings.current_ki_q.Value()) &&
         std::isfinite(settings.current_v_max.Value()) && std::isfinite(settings.svm_duty_min.Value()) &&
         std::isfinite(settings.svm_duty_max.Value()) && std::isfinite(settings.dtc_dead_time.Value()) &&
         std::isfinite(settings.dtc_f_pwm.Value()) && std::isfinite(settings.dtc_i_threshold.Value()) &&
         std::isfinite(settings.hfi_v_inject.Value()) && std::isfinite(settings.hfi_error_gain.Value()) &&
         std::isfinite(settings.excitation_l_m.Value()) && std::isfinite(settings.excitation_kp.Value()) &&
         std::isfinite(settings.excitation_ki.Value()) &&
         std::isfinite(settings.excitation_i_f_min.Value()) &&
         std::isfinite(settings.excitation_i_f_max.Value()) &&
         std::isfinite(settings.excitation_obs_tau.Value()) &&
         std::isfinite(settings.excitation_obs_l_m.Value()) && std::isfinite(settings.pos_kp_pos.Value()) &&
         std::isfinite(settings.pos_kp_speed.Value()) && std::isfinite(settings.pos_ki_speed.Value()) &&
         std::isfinite(settings.pos_speed_limit.Value()) && std::isfinite(settings.pos_accel_limit.Value()) &&
         std::isfinite(settings.pos_position_tolerance.Value()) &&
         std::isfinite(settings.pos_speed_tolerance.Value()) &&
         std::isfinite(settings.pos_homing_speed.Value()) && std::isfinite(settings.adc_offset_a.Value()) &&
         std::isfinite(settings.adc_offset_b.Value()) && std::isfinite(settings.adc_gain_a.Value()) &&
         std::isfinite(settings.adc_gain_b.Value()) && std::isfinite(settings.adc_gain_vdc.Value()) &&
         std::isfinite(settings.phase_balance_a.Value()) && std::isfinite(settings.phase_balance_b.Value()) &&
         std::isfinite(settings.phase_balance_c.Value());
}

[[nodiscard]] inline bool IsValidEnumValues(const NvmSettings& settings) noexcept {
  const bool kValidMotorType = settings.motor_type == system::MotorType::kPmsm ||
                               settings.motor_type == system::MotorType::kAsm ||
                               settings.motor_type == system::MotorType::kEesm;
  const bool kValidControlMode = settings.control_mode == cyphal::ControlMode::kTorque ||
                                 settings.control_mode == cyphal::ControlMode::kSpeed ||
                                 settings.control_mode == cyphal::ControlMode::kPosition;
  const float kPwmFrequencyHz = settings.pwm_frequency.Value();
  const bool kValidPwmFrequency = kPwmFrequencyHz == 16000.0F || kPwmFrequencyHz == 20000.0F ||
                                  kPwmFrequencyHz == 24000.0F || kPwmFrequencyHz == 28000.0F ||
                                  kPwmFrequencyHz == 32000.0F;
  return kValidMotorType && kValidControlMode && kValidPwmFrequency && settings.excitation_mode <= 1U;
}

[[nodiscard]] inline bool IsValidProfile(const SettingsProfile& profile) noexcept {
  const auto& limits = profile.capabilities;
  return IsFinite(profile.factory_settings) && std::isfinite(limits.max_phase_current.Value()) &&
         std::isfinite(limits.max_motor_current.Value()) &&
         std::isfinite(limits.max_battery_drive_current.Value()) &&
         std::isfinite(limits.max_battery_charge_current.Value()) &&
         limits.max_phase_current.Value() > 0.0F && limits.max_motor_current.Value() > 0.0F &&
         limits.max_motor_current.Value() <= limits.max_phase_current.Value() &&
         limits.max_battery_drive_current.Value() > 0.0F && limits.max_battery_charge_current.Value() >= 0.0F;
}

[[nodiscard]] inline SettingsStatus Validate(const NvmSettings& settings,
                                             const HardwareCapabilities& limits) noexcept {
  if (!IsFinite(settings) || !IsValidEnumValues(settings) || settings.node_id > 127U ||
      settings.pole_pairs == 0U) {
    return SettingsStatus::kInvalidSettings;
  }

  if (settings.motor_i_max.Value() < 0.0F ||
      settings.motor_i_max.Value() > limits.max_motor_current.Value() ||
      settings.battery_drive_current_max.Value() < 0.0F ||
      settings.battery_drive_current_max.Value() > limits.max_battery_drive_current.Value() ||
      settings.battery_charge_current_max.Value() < 0.0F ||
      settings.battery_charge_current_max.Value() > limits.max_battery_charge_current.Value() ||
      settings.motor_omega_min.Value() > 0.0F || settings.motor_omega_max.Value() < 0.0F) {
    return SettingsStatus::kOutOfRange;
  }

  if (settings.stator_r.Value() <= 0.0F || settings.stator_l.Value() <= 0.0F ||
      settings.flux_pm.Value() < 0.0F || settings.l_d.Value() <= 0.0F || settings.l_q.Value() <= 0.0F ||
      settings.asm_r_r.Value() <= 0.0F || settings.asm_r_s.Value() <= 0.0F ||
      settings.asm_l_s.Value() <= 0.0F || settings.asm_l_r.Value() <= 0.0F ||
      settings.asm_l_m.Value() <= 0.0F || settings.motor_j.Value() <= 0.0F ||
      settings.asm_flux_i_d_min.Value() < 0.0F ||
      settings.asm_flux_i_d_max.Value() < settings.asm_flux_i_d_min.Value() ||
      settings.excitation_i_f_min.Value() < 0.0F ||
      settings.excitation_i_f_max.Value() < settings.excitation_i_f_min.Value() ||
      settings.dtc_dead_time.Value() < 0.0F || settings.dtc_f_pwm.Value() <= 0.0F ||
      settings.excitation_obs_tau.Value() <= 0.0F || settings.pos_position_tolerance.Value() < 0.0F ||
      settings.pos_speed_tolerance.Value() < 0.0F) {
    return SettingsStatus::kOutOfRange;
  }

  const auto kIsRatio = [](const unit::Ratio value) {
    return value.Value() >= 0.0F && value.Value() <= 1.0F;
  };
  if (settings.fw_v_max.Value() < 0.0F || !kIsRatio(settings.current_v_max) ||
      !kIsRatio(settings.svm_duty_min) || !kIsRatio(settings.svm_duty_max) ||
      settings.svm_duty_min > settings.svm_duty_max || settings.adc_gain_a.Value() <= 0.0F ||
      settings.adc_gain_b.Value() <= 0.0F || settings.adc_gain_vdc.Value() <= 0.0F ||
      settings.phase_balance_a.Value() <= 0.0F || settings.phase_balance_b.Value() <= 0.0F ||
      settings.phase_balance_c.Value() <= 0.0F) {
    return SettingsStatus::kOutOfRange;
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
   * @param offset_a Phase-A current offset.
   * @param offset_b Phase-B current offset.
   * @param gain_a Phase-A current-sense gain.
   * @param gain_b Phase-B current-sense gain.
   * @param gain_vdc DC-link voltage-sense gain.
   * @return The validation result.
   */
  [[nodiscard]] SettingsStatus ApplyAdcCalibration(unit::Current offset_a,
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
