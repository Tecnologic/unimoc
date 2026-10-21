/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file settings_register_provider.hpp
 *    @brief Cyphal registers of the persistent settings (ForEachRegister).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cstddef>
#include <string_view>
#include "cyphal/register_value.hpp"
#include "settings_store.hpp"

namespace unimoc::drive {

/// Number of persistent registers of NvmSettings.
[[nodiscard]] std::size_t SettingsRegisterCount() noexcept;

/**
 * @brief Exposes every register of `ForEachRegister()` over uavcan.register.
 *
 * Reads take the live (RAM) settings, writes go through
 * `SettingsOperations::Update()` and its validation; they change the RAM
 * copy only. ExecuteCommand STORE_PERSISTENT_STATES saves them to flash.
 *
 * Value types: units and floats real32[1], `uint8_t` and enums natural8[1]
 * (`uavcan.node.id` natural16[1] as the standard demands), port-IDs
 * natural16[1], the node name a string. Writes accept any numeric type.
 */
class SettingsRegisterProvider final : public cyphal::RegisterProvider {
 public:
  /// Largest number of registers the provider can hold.
  static constexpr std::size_t kCapacity = 96U;

  /// @param store Settings store shared with the rest of the firmware; must outlive the provider.
  explicit SettingsRegisterProvider(settings::SettingsStore& store) noexcept;

  [[nodiscard]] std::size_t Count() const noexcept override { return count_; }
  [[nodiscard]] std::string_view Name(std::size_t index) const noexcept override;
  [[nodiscard]] cyphal::RegisterFlags Read(std::size_t index,
                                           cyphal::RegisterValue& value) const noexcept override;
  [[nodiscard]] cyphal::RegisterWriteResult Write(std::size_t index,
                                                  const cyphal::RegisterValue& value) noexcept override;

  /// Number of accepted writes since construction; tells the application to apply them.
  [[nodiscard]] uint32_t WriteCount() const noexcept { return write_count_; }

 private:
  settings::SettingsStore* store_;
  std::array<std::string_view, kCapacity> names_{};
  std::size_t count_{0U};
  uint32_t write_count_{0U};
};

}  // namespace unimoc::drive
