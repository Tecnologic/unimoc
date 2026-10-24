/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file register_value.cpp
 *    @brief Helpers for uavcan.register.Value.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "cyphal/register_value.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace unimoc::cyphal {

namespace {

/// Copies up to the capacity of @p array and sets its count.
template <typename Array, typename Element>
void Assign(Array& array, std::span<const Element> elements) noexcept {
  constexpr std::size_t kCapacity = sizeof(array.elements) / sizeof(array.elements[0]);
  const std::size_t kCount = std::min(elements.size(), kCapacity);
  std::copy_n(elements.begin(), kCount, std::begin(array.elements));
  array.count = kCount;
}

template <typename Array>
[[nodiscard]] std::optional<float> First(const Array& array) noexcept {
  if (array.count == 0U) {
    return std::nullopt;
  }
  return static_cast<float>(array.elements[0]);
}

}  // namespace

void SetEmpty(RegisterValue& value) noexcept { uavcan_register_Value_1_0_select_empty_(&value); }

void SetString(RegisterValue& value, const std::string_view text) noexcept {
  uavcan_register_Value_1_0_select_string_(&value);
  const auto* const kBytes = reinterpret_cast<const uint8_t*>(text.data());
  Assign(value._string.value, std::span<const uint8_t>{kBytes, text.size()});
}

void SetUnstructured(RegisterValue& value, const std::span<const uint8_t> bytes) noexcept {
  uavcan_register_Value_1_0_select_unstructured_(&value);
  Assign(value.unstructured.value, bytes);
}

void SetBit(RegisterValue& value, const bool bit) noexcept {
  uavcan_register_Value_1_0_select_bit_(&value);
  value.bit.value.bitpacked[0] = bit ? 1U : 0U;
  value.bit.value.count = 1U;
}

void SetNatural8(RegisterValue& value, const uint8_t number) noexcept {
  uavcan_register_Value_1_0_select_natural8_(&value);
  value.natural8.value.elements[0] = number;
  value.natural8.value.count = 1U;
}

void SetNatural16(RegisterValue& value, const uint16_t number) noexcept {
  uavcan_register_Value_1_0_select_natural16_(&value);
  value.natural16.value.elements[0] = number;
  value.natural16.value.count = 1U;
}

void SetNatural32(RegisterValue& value, const uint32_t number) noexcept {
  uavcan_register_Value_1_0_select_natural32_(&value);
  value.natural32.value.elements[0] = number;
  value.natural32.value.count = 1U;
}

void SetReal32(RegisterValue& value, const float number) noexcept {
  uavcan_register_Value_1_0_select_real32_(&value);
  value.real32.value.elements[0] = number;
  value.real32.value.count = 1U;
}

bool IsEmpty(const RegisterValue& value) noexcept { return uavcan_register_Value_1_0_is_empty_(&value); }

std::optional<float> GetNumber(const RegisterValue& value) noexcept {
  if (uavcan_register_Value_1_0_is_bit_(&value)) {
    if (value.bit.value.count == 0U) {
      return std::nullopt;
    }
    return ((value.bit.value.bitpacked[0] & 1U) != 0U) ? 1.0F : 0.0F;
  }
  if (uavcan_register_Value_1_0_is_integer64_(&value)) {
    return First(value.integer64.value);
  }
  if (uavcan_register_Value_1_0_is_integer32_(&value)) {
    return First(value.integer32.value);
  }
  if (uavcan_register_Value_1_0_is_integer16_(&value)) {
    return First(value.integer16.value);
  }
  if (uavcan_register_Value_1_0_is_integer8_(&value)) {
    return First(value.integer8.value);
  }
  if (uavcan_register_Value_1_0_is_natural64_(&value)) {
    return First(value.natural64.value);
  }
  if (uavcan_register_Value_1_0_is_natural32_(&value)) {
    return First(value.natural32.value);
  }
  if (uavcan_register_Value_1_0_is_natural16_(&value)) {
    return First(value.natural16.value);
  }
  if (uavcan_register_Value_1_0_is_natural8_(&value)) {
    return First(value.natural8.value);
  }
  if (uavcan_register_Value_1_0_is_real64_(&value)) {
    return First(value.real64.value);
  }
  if (uavcan_register_Value_1_0_is_real32_(&value)) {
    return First(value.real32.value);
  }
  if (uavcan_register_Value_1_0_is_real16_(&value)) {
    return First(value.real16.value);
  }
  return std::nullopt;
}

std::optional<int64_t> GetInteger(const RegisterValue& value) noexcept {
  // 64 bit integers are read exactly; everything else fits into a float.
  if (uavcan_register_Value_1_0_is_integer64_(&value)) {
    if (value.integer64.value.count == 0U) {
      return std::nullopt;
    }
    return value.integer64.value.elements[0];
  }
  if (uavcan_register_Value_1_0_is_natural64_(&value)) {
    if (value.natural64.value.count == 0U ||
        value.natural64.value.elements[0] > static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) {
      return std::nullopt;
    }
    return static_cast<int64_t>(value.natural64.value.elements[0]);
  }
  if (uavcan_register_Value_1_0_is_natural32_(&value)) {
    if (value.natural32.value.count == 0U) {
      return std::nullopt;
    }
    return static_cast<int64_t>(value.natural32.value.elements[0]);
  }
  if (uavcan_register_Value_1_0_is_integer32_(&value)) {
    if (value.integer32.value.count == 0U) {
      return std::nullopt;
    }
    return static_cast<int64_t>(value.integer32.value.elements[0]);
  }
  const std::optional<float> kNumber = GetNumber(value);
  if (!kNumber.has_value() || !std::isfinite(*kNumber) || std::trunc(*kNumber) != *kNumber ||
      std::fabs(*kNumber) > 9.0e15F) {
    return std::nullopt;
  }
  return static_cast<int64_t>(*kNumber);
}

std::optional<std::string_view> GetString(const RegisterValue& value) noexcept {
  if (!uavcan_register_Value_1_0_is_string_(&value)) {
    return std::nullopt;
  }
  return std::string_view{reinterpret_cast<const char*>(value._string.value.elements),
                          value._string.value.count};
}

}  // namespace unimoc::cyphal
