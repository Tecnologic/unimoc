/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file zephyr_settings_storage.cpp
 *    @brief Settings storage adapter on the Zephyr settings subsystem.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "zephyr_settings_storage.hpp"
#include <zephyr/settings/settings.h>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstring>
#include <string_view>
#include "settings_registers.hpp"

namespace unimoc::hardware {

namespace {

using settings::SettingsStorage;
using settings::SettingsStorageStatus;

constexpr std::string_view kSubtree = "unimoc";
// "unimoc/" and the longest register name, with room to spare.
constexpr std::size_t kNameBufferSize = 64U;

bool g_initialized = false;

bool EnsureInitialized() noexcept {
  if (!g_initialized) {
    g_initialized = settings_subsys_init() == 0;
  }
  return g_initialized;
}

// Builds "unimoc/<key>"; false when the key does not fit.
bool MakeName(const std::string_view key, std::array<char, kNameBufferSize>& name) noexcept {
  if (kSubtree.size() + 1U + key.size() + 1U > name.size()) {
    return false;
  }
  std::size_t length = 0U;
  std::memcpy(name.data(), kSubtree.data(), kSubtree.size());
  length += kSubtree.size();
  name[length++] = '/';
  std::memcpy(&name[length], key.data(), key.size());
  length += key.size();
  name[length] = '\0';
  return true;
}

struct LoadContext {
  SettingsStorage::EntryCallback entry;
  const void* entry_context;
};

int OnEntry(const char* key, const size_t length, settings_read_cb read_cb, void* cb_arg, void* param) {
  const auto& context = *static_cast<const LoadContext*>(param);
  // One byte more than any register value: an oversized entry is passed on
  // with a size no decoder accepts instead of being truncated into a valid one.
  std::array<std::byte, settings::kMaxRegisterValueSize + 1U> buffer{};
  const size_t kWanted = std::min(length, buffer.size());
  const ssize_t kRead = read_cb(cb_arg, buffer.data(), kWanted);
  if (kRead < 0) {
    return static_cast<int>(kRead);
  }
  context.entry(context.entry_context,
                key,
                std::span<const std::byte>{buffer.data(), static_cast<size_t>(kRead)});
  return 0;
}

SettingsStorageStatus LoadAll(void* /*context*/,
                              const SettingsStorage::EntryCallback entry,
                              const void* const entry_context) noexcept {
  if (!EnsureInitialized()) {
    return SettingsStorageStatus::kIoError;
  }
  LoadContext context{entry, entry_context};
  const std::array<char, 7> kSubtreeName{'u', 'n', 'i', 'm', 'o', 'c', '\0'};
  return settings_load_subtree_direct(kSubtreeName.data(), OnEntry, &context) == 0
             ? SettingsStorageStatus::kSuccess
             : SettingsStorageStatus::kIoError;
}

SettingsStorageStatus Write(void* /*context*/,
                            const std::string_view key,
                            const std::span<const std::byte> value) noexcept {
  std::array<char, kNameBufferSize> name{};
  // An empty value would delete the entry in the settings subsystem.
  if (value.empty() || !MakeName(key, name) || !EnsureInitialized()) {
    return SettingsStorageStatus::kIoError;
  }
  return settings_save_one(name.data(), value.data(), value.size()) == 0 ? SettingsStorageStatus::kSuccess
                                                                         : SettingsStorageStatus::kIoError;
}

SettingsStorageStatus Erase(void* /*context*/, const std::string_view key) noexcept {
  std::array<char, kNameBufferSize> name{};
  if (!MakeName(key, name) || !EnsureInitialized()) {
    return SettingsStorageStatus::kIoError;
  }
  return settings_delete(name.data()) == 0 ? SettingsStorageStatus::kSuccess
                                           : SettingsStorageStatus::kIoError;
}

}  // namespace

settings::SettingsStorage ZephyrSettingsStorage::Adapter() noexcept {
  return SettingsStorage{nullptr, LoadAll, Write, Erase};
}

}  // namespace unimoc::hardware
