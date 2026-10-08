/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file settings_storage.hpp
 *    @brief Platform-neutral settings storage callbacks.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace unimoc::settings {

/**
 * @brief Result of a platform settings storage operation.
 */
enum class SettingsStorageStatus : uint8_t {
  kSuccess,
  kNotFound,
  kUnavailable,
  kIoError,
};

/**
 * @brief Non-owning load/save adapter for a serialized settings image.
 *
 * The context and callbacks are supplied by the hardware layer. The settings
 * service owns no file, flash, or EEPROM resources and can therefore be used
 * by hosted and embedded targets alike.
 */
class SettingsStorage {
 public:
  using LoadCallback = SettingsStorageStatus (*)(void* context, std::span<std::byte> image);
  using SaveCallback = SettingsStorageStatus (*)(void* context, std::span<const std::byte> image);

  constexpr SettingsStorage() = default;

  /**
   * @brief Constructs a storage adapter from platform callbacks.
   * @param context Opaque platform-owned callback context.
   * @param load Reads exactly the requested image size.
   * @param save Atomically writes the supplied image when possible.
   */
  constexpr SettingsStorage(void* context, LoadCallback load, SaveCallback save) noexcept
      : context_{context}, load_{load}, save_{save} {}

  /**
   * @brief Loads a serialized settings image.
   * @param image Destination buffer with the required image size.
   * @return The platform operation result.
   */
  [[nodiscard]] SettingsStorageStatus Load(std::span<std::byte> image) const noexcept {
    return load_ == nullptr ? SettingsStorageStatus::kUnavailable : load_(context_, image);
  }

  /**
   * @brief Saves a serialized settings image.
   * @param image Complete serialized settings image.
   * @return The platform operation result.
   */
  [[nodiscard]] SettingsStorageStatus Save(std::span<const std::byte> image) const noexcept {
    return save_ == nullptr ? SettingsStorageStatus::kUnavailable : save_(context_, image);
  }

 private:
  void* context_{nullptr};
  LoadCallback load_{nullptr};
  SaveCallback save_{nullptr};
};

}  // namespace unimoc::settings
