/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file zephyr_settings_storage.hpp
 *    @brief Persistent settings storage on the Zephyr settings subsystem (ZMS).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include "settings_storage.hpp"

namespace unimoc::hardware {

/**
 * @brief Key/value storage of the settings on the Zephyr settings subsystem.
 *
 * Every register is one settings entry `unimoc/<register name>` in the
 * `storage_partition` (ZMS on the BCC flash, on the flash simulator on
 * native_sim). The settings subsystem is initialized on first use. Calls
 * block and must come from a thread, never from a zero-latency interrupt.
 */
class ZephyrSettingsStorage {
 public:
  /// Returns the storage adapter for `SettingsStore`.
  [[nodiscard]] static settings::SettingsStorage Adapter() noexcept;

  /// Returns the lock for `SettingsStore`: one Zephyr mutex, threads only.
  [[nodiscard]] static settings::SettingsLock Lock() noexcept;
};

}  // namespace unimoc::hardware
