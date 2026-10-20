/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file ram_settings_storage.hpp
 *    @brief In-memory key/value settings storage with fault injection for host tests.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include "settings_storage.hpp"

namespace unimoc::test {

/// Key/value storage in RAM that counts operations and can fail them on demand.
class RamSettingsStorage {
 public:
  static constexpr std::size_t kMaxEntries = 96U;
  static constexpr std::size_t kMaxKeySize = 48U;
  static constexpr std::size_t kMaxValueSize = 64U;

  /// Returns the adapter for `SettingsStore`; the object must outlive it.
  [[nodiscard]] settings::SettingsStorage Adapter() noexcept {
    return settings::SettingsStorage{this, &LoadAll, &Write, &Erase};
  }

  /// Stores an entry directly, bypassing the counters (prepares a test).
  void Put(const std::string_view key, const std::span<const std::byte> value) noexcept {
    (void)Store(key, value);
  }

  /// Returns true when an entry exists.
  [[nodiscard]] bool Contains(const std::string_view key) const noexcept { return Find(key) != nullptr; }

  /// Returns the stored value of an entry, empty when it does not exist.
  [[nodiscard]] std::span<const std::byte> Value(const std::string_view key) const noexcept {
    const Entry* entry = Find(key);
    return entry == nullptr ? std::span<const std::byte>{}
                            : std::span<const std::byte>{entry->value.data(), entry->size};
  }

  /// Number of stored entries.
  [[nodiscard]] std::size_t Count() const noexcept {
    return static_cast<std::size_t>(
        std::count_if(entries_.begin(), entries_.end(), [](const Entry& entry) { return entry.used; }));
  }

  /// Write() calls so far.
  [[nodiscard]] uint32_t Writes() const noexcept { return writes_; }
  /// Erase() calls so far.
  [[nodiscard]] uint32_t Erases() const noexcept { return erases_; }
  /// Makes every Write() fail.
  void FailWrites(const bool fail) noexcept { fail_writes_ = fail; }
  /// Makes every operation report that there is no storage.
  void SetUnavailable(const bool unavailable) noexcept { unavailable_ = unavailable; }
  /// Fails every Write() after @p count successful ones; negative: never.
  void FailWritesAfter(const int count) noexcept { fail_writes_after_ = count; }

 private:
  struct Entry {
    bool used{false};
    std::array<char, kMaxKeySize> key{};
    std::size_t key_size{0U};
    std::array<std::byte, kMaxValueSize> value{};
    std::size_t size{0U};
  };

  [[nodiscard]] const Entry* Find(const std::string_view key) const noexcept {
    for (const Entry& entry : entries_) {
      if (entry.used && std::string_view{entry.key.data(), entry.key_size} == key) {
        return &entry;
      }
    }
    return nullptr;
  }

  [[nodiscard]] bool Store(const std::string_view key, const std::span<const std::byte> value) noexcept {
    if (key.size() > kMaxKeySize || value.size() > kMaxValueSize) {
      return false;
    }
    Entry* target = nullptr;
    for (Entry& entry : entries_) {
      if (entry.used && std::string_view{entry.key.data(), entry.key_size} == key) {
        target = &entry;
        break;
      }
      if (!entry.used && target == nullptr) {
        target = &entry;
      }
    }
    if (target == nullptr) {
      return false;
    }
    target->used = true;
    std::copy(key.begin(), key.end(), target->key.begin());
    target->key_size = key.size();
    std::copy(value.begin(), value.end(), target->value.begin());
    target->size = value.size();
    return true;
  }

  static settings::SettingsStorageStatus LoadAll(void* context,
                                                 const settings::SettingsStorage::EntryCallback entry,
                                                 const void* const entry_context) noexcept {
    const auto& self = *static_cast<RamSettingsStorage*>(context);
    if (self.unavailable_) {
      return settings::SettingsStorageStatus::kUnavailable;
    }
    for (const Entry& stored : self.entries_) {
      if (stored.used) {
        entry(entry_context,
              std::string_view{stored.key.data(), stored.key_size},
              std::span<const std::byte>{stored.value.data(), stored.size});
      }
    }
    return settings::SettingsStorageStatus::kSuccess;
  }

  static settings::SettingsStorageStatus Write(void* context,
                                               const std::string_view key,
                                               const std::span<const std::byte> value) noexcept {
    auto& self = *static_cast<RamSettingsStorage*>(context);
    if (self.unavailable_) {
      return settings::SettingsStorageStatus::kUnavailable;
    }
    const bool kFailNow =
        self.fail_writes_ ||
        (self.fail_writes_after_ >= 0 && self.writes_ >= static_cast<uint32_t>(self.fail_writes_after_));
    ++self.writes_;
    if (kFailNow || !self.Store(key, value)) {
      return settings::SettingsStorageStatus::kIoError;
    }
    return settings::SettingsStorageStatus::kSuccess;
  }

  static settings::SettingsStorageStatus Erase(void* context, const std::string_view key) noexcept {
    auto& self = *static_cast<RamSettingsStorage*>(context);
    if (self.unavailable_) {
      return settings::SettingsStorageStatus::kUnavailable;
    }
    ++self.erases_;
    for (Entry& entry : self.entries_) {
      if (entry.used && std::string_view{entry.key.data(), entry.key_size} == key) {
        entry = Entry{};
      }
    }
    return settings::SettingsStorageStatus::kSuccess;
  }

  std::array<Entry, kMaxEntries> entries_{};
  uint32_t writes_{0U};
  uint32_t erases_{0U};
  bool fail_writes_{false};
  bool unavailable_{false};
  int fail_writes_after_{-1};
};

}  // namespace unimoc::test
