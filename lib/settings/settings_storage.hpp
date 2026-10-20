/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file settings_storage.hpp
 *    @brief Platform boundary for the persistent key/value settings storage.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace unimoc::settings {

/// Result of a platform storage operation.
enum class SettingsStorageStatus : uint8_t {
  /// The operation completed.
  kSuccess,
  /// The key does not exist.
  kNotFound,
  /// There is no persistent storage on this target.
  kUnavailable,
  /// The storage reported an I/O failure.
  kIoError,
};

/**
 * @brief Platform adapter for persistent key/value storage.
 *
 * One entry per register: the key is the register name, the value its
 * encoded bytes (settings_registers.hpp). The Zephyr platforms back it with
 * the settings subsystem on ZMS (hardware/zephyr_common). All callbacks run
 * in thread context and may block; they are never called from the hot path.
 */
class SettingsStorage {
 public:
  /// Receives one stored entry while the storage is enumerated.
  using EntryCallback = void (*)(const void* entry_context,
                                 std::string_view key,
                                 std::span<const std::byte> value);
  /// Calls @p entry for every stored entry.
  using LoadAllCallback = SettingsStorageStatus (*)(void* context,
                                                    EntryCallback entry,
                                                    const void* entry_context);
  /// Stores or replaces one entry.
  using WriteCallback = SettingsStorageStatus (*)(void* context,
                                                  std::string_view key,
                                                  std::span<const std::byte> value);
  /// Removes one entry; removing a missing entry succeeds.
  using EraseCallback = SettingsStorageStatus (*)(void* context, std::string_view key);

  constexpr SettingsStorage() = default;

  /**
   * @brief Constructs a storage adapter from platform callbacks.
   * @param context Opaque platform-owned callback context.
   * @param load_all Enumerates all stored entries.
   * @param write Writes one entry.
   * @param erase Erases one entry.
   */
  constexpr SettingsStorage(void* context,
                            LoadAllCallback load_all,
                            WriteCallback write,
                            EraseCallback erase) noexcept
      : context_{context}, load_all_{load_all}, write_{write}, erase_{erase} {}

  /**
   * @brief Enumerates all stored entries.
   * @param visitor Callable `void(std::string_view key, std::span<const std::byte> value)`.
   * @return The platform operation result.
   */
  template <typename Visitor>
  [[nodiscard]] SettingsStorageStatus LoadAll(const Visitor& visitor) const {
    const EntryCallback kEntry =
        [](const void* entry_context, std::string_view key, std::span<const std::byte> value) {
          (*static_cast<const Visitor*>(entry_context))(key, value);
        };
    return load_all_ == nullptr ? SettingsStorageStatus::kUnavailable
                                : load_all_(context_, kEntry, static_cast<const void*>(&visitor));
  }

  /**
   * @brief Stores or replaces one entry.
   * @param key Register name.
   * @param value Encoded register value.
   * @return The platform operation result.
   */
  [[nodiscard]] SettingsStorageStatus Write(std::string_view key,
                                            std::span<const std::byte> value) const noexcept {
    return write_ == nullptr ? SettingsStorageStatus::kUnavailable : write_(context_, key, value);
  }

  /**
   * @brief Removes one entry.
   * @param key Register name.
   * @return The platform operation result.
   */
  [[nodiscard]] SettingsStorageStatus Erase(std::string_view key) const noexcept {
    return erase_ == nullptr ? SettingsStorageStatus::kUnavailable : erase_(context_, key);
  }

 private:
  void* context_{nullptr};
  LoadAllCallback load_all_{nullptr};
  WriteCallback write_{nullptr};
  EraseCallback erase_{nullptr};
};

}  // namespace unimoc::settings
