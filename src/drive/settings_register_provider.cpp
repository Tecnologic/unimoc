/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file settings_register_provider.cpp
 *    @brief Cyphal registers of the persistent settings (ForEachRegister).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "settings_register_provider.hpp"
#include <cmath>
#include <limits>
#include <type_traits>
#include "settings_registers.hpp"

namespace unimoc::drive {

namespace {

using cyphal::RegisterValue;
using cyphal::RegisterWriteResult;

template <typename T>
inline constexpr bool kIsEnumByte = std::is_enum_v<T> && sizeof(T) == 1U;

using NodeName = std::array<char, cyphal::kNodeNameMaxLen + 1U>;

template <typename Field>
void ToValue(const std::string_view name, const Field& field, RegisterValue& value) noexcept {
  if constexpr (unit::UnitLike<Field>) {
    cyphal::SetReal32(value, field.Value());
  } else if constexpr (std::is_same_v<Field, float>) {
    cyphal::SetReal32(value, field);
  } else if constexpr (std::is_same_v<Field, uint8_t>) {
    if (name == cyphal::reg::kNodeId) {
      cyphal::SetNatural16(value, field);  // uavcan.node.id is natural16 by the standard
    } else {
      cyphal::SetNatural8(value, field);
    }
  } else if constexpr (kIsEnumByte<Field>) {
    cyphal::SetNatural8(value, static_cast<uint8_t>(field));
  } else if constexpr (std::is_same_v<Field, uint16_t>) {
    cyphal::SetNatural16(value, field);
  } else {
    static_assert(std::is_same_v<Field, NodeName>);
    std::size_t length = 0U;
    while (length < cyphal::kNodeNameMaxLen && field[length] != '\0') {
      ++length;
    }
    cyphal::SetString(value, std::string_view{field.data(), length});
  }
}

template <typename Integer>
[[nodiscard]] bool ToInteger(const RegisterValue& value, Integer& out) noexcept {
  const auto kNumber = cyphal::GetInteger(value);
  if (!kNumber.has_value() || *kNumber < 0 ||
      *kNumber > static_cast<int64_t>(std::numeric_limits<Integer>::max())) {
    return false;
  }
  out = static_cast<Integer>(*kNumber);
  return true;
}

template <typename Field>
[[nodiscard]] RegisterWriteResult FromValue(const RegisterValue& value, Field& field) noexcept {
  if constexpr (unit::UnitLike<Field> || std::is_same_v<Field, float>) {
    const auto kNumber = cyphal::GetNumber(value);
    if (!kNumber.has_value()) {
      return RegisterWriteResult::kTypeMismatch;
    }
    if (!std::isfinite(*kNumber)) {
      return RegisterWriteResult::kOutOfRange;
    }
    field = Field{*kNumber};
    return RegisterWriteResult::kAccepted;
  } else if constexpr (std::is_same_v<Field, uint8_t> || std::is_same_v<Field, uint16_t>) {
    if (!cyphal::GetNumber(value).has_value()) {
      return RegisterWriteResult::kTypeMismatch;
    }
    return ToInteger(value, field) ? RegisterWriteResult::kAccepted : RegisterWriteResult::kOutOfRange;
  } else if constexpr (kIsEnumByte<Field>) {
    uint8_t raw = 0U;
    if (!cyphal::GetNumber(value).has_value()) {
      return RegisterWriteResult::kTypeMismatch;
    }
    if (!ToInteger(value, raw)) {
      return RegisterWriteResult::kOutOfRange;
    }
    field = static_cast<Field>(raw);  // the settings validation rejects unknown values
    return RegisterWriteResult::kAccepted;
  } else {
    static_assert(std::is_same_v<Field, NodeName>);
    const auto kText = cyphal::GetString(value);
    if (!kText.has_value()) {
      return RegisterWriteResult::kTypeMismatch;
    }
    if (kText->size() > cyphal::kNodeNameMaxLen) {
      return RegisterWriteResult::kOutOfRange;
    }
    field.fill('\0');
    for (std::size_t i = 0U; i < kText->size(); ++i) {
      field[i] = (*kText)[i];
    }
    return RegisterWriteResult::kAccepted;
  }
}

}  // namespace

std::size_t SettingsRegisterCount() noexcept {
  std::size_t count = 0U;
  const settings::NvmSettings kSettings{};
  settings::ForEachRegister([&](std::string_view /*name*/, const auto& /*field*/) { ++count; }, kSettings);
  return count;
}

SettingsRegisterProvider::SettingsRegisterProvider(settings::SettingsStore& store) noexcept
    : store_{&store} {}

std::string_view SettingsRegisterProvider::Name(const std::size_t index) const noexcept {
  std::string_view found{};
  std::size_t position = 0U;
  const settings::NvmSettings kSettings{};
  settings::ForEachRegister(
      [&](const std::string_view name, const auto& /*field*/) {
        if (position++ == index) {
          found = name;
        }
      },
      kSettings);
  return found;  // the names are string literals of cyphal::reg
}

cyphal::RegisterDescription SettingsRegisterProvider::Describe(const std::size_t index) const noexcept {
  cyphal::RegisterDescription description{};
  std::size_t position = 0U;
  const settings::NvmSettings kSettings{};
  settings::ForEachRegister(
      [&](const settings::RegisterEntry& entry, const auto& field) {
        if (position++ == index) {
          description = settings::DescriptionOf(entry, field);
        }
      },
      kSettings);
  return description;
}

cyphal::RegisterFlags SettingsRegisterProvider::Read(const std::size_t index,
                                                     RegisterValue& value) const noexcept {
  const settings::SettingsSnapshot kSnapshot = store_->GetSnapshot();
  std::size_t position = 0U;
  settings::ForEachRegister(
      [&](const std::string_view name, const auto& field) {
        if (position++ == index) {
          ToValue(name, field, value);
        }
      },
      kSnapshot.Get());
  return {true, true};
}

RegisterWriteResult SettingsRegisterProvider::Write(const std::size_t index,
                                                    const RegisterValue& value) noexcept {
  RegisterWriteResult result = RegisterWriteResult::kTypeMismatch;
  const settings::SettingsStatus kStatus =
      store_->GetOperations().Update([&](settings::NvmSettings& candidate) {
        std::size_t position = 0U;
        settings::ForEachRegister(
            [&](std::string_view /*name*/, auto& field) {
              if (position++ == index) {
                result = FromValue(value, field);
              }
            },
            candidate);
      });
  if (result != RegisterWriteResult::kAccepted) {
    return result;  // the field was left unchanged, the commit was a no-op
  }
  if (kStatus != settings::SettingsStatus::kSuccess) {
    return RegisterWriteResult::kOutOfRange;
  }
  ++write_count_;
  return RegisterWriteResult::kAccepted;
}

}  // namespace unimoc::drive
