/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file ram_settings_storage.hpp
 *    @brief Volatile settings image shared by the Zephyr hardware layers.
 *
 *    Placeholder until a persistent backend (flash, file) exists. The image
 *    survives only until reset, so the node starts from factory settings.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <span>

#include "settings_codec.hpp"
#include "settings_storage.hpp"

namespace unimoc::hardware {

class RamSettingsStorage {
 public:
  [[nodiscard]] settings::SettingsStorage Adapter() noexcept {
    return settings::SettingsStorage{this, &Load, &Save};
  }

 private:
  static settings::SettingsStorageStatus Load(void* const context, const std::span<std::byte> image) noexcept {
    const auto& self = *static_cast<RamSettingsStorage*>(context);
    if (!self.valid_) return settings::SettingsStorageStatus::kNotFound;
    std::copy_n(self.image_.begin(), std::min(image.size(), self.image_.size()), image.begin());
    return settings::SettingsStorageStatus::kSuccess;
  }

  static settings::SettingsStorageStatus Save(void* const context,
                                              const std::span<const std::byte> image) noexcept {
    auto& self = *static_cast<RamSettingsStorage*>(context);
    if (image.size() > self.image_.size()) return settings::SettingsStorageStatus::kIoError;
    std::copy(image.begin(), image.end(), self.image_.begin());
    self.valid_ = true;
    return settings::SettingsStorageStatus::kSuccess;
  }

  std::array<std::byte, settings::kSettingsImageSize> image_{};
  bool valid_{false};
};

}  // namespace unimoc::hardware
