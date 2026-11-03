/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file settings_registers_test.cpp
 *    @brief ztest suite of the register table and the register value encoding.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "settings_registers.hpp"
#include <zephyr/ztest.h>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string_view>
#include <type_traits>
#include "settings_store.hpp"
#include "unimoc_test.hpp"

namespace unimoc::settings::test {

using namespace unimoc::unit;

ZTEST(settings_registers, test_every_register_has_a_unique_short_name) {
  std::array<std::string_view, 160> names{};
  std::size_t count = 0U;
  const NvmSettings kSettings{};
  ForEachRegister(
      [&](std::string_view name, const auto& /*field*/) {
        zassert_true(count < names.size());
        zassert_true(name.size() <= 48U, "name too long: %s", name.data());
        for (std::size_t i = 0; i < count; ++i) {
          zassert_true(names[i] != name, "duplicate register %s", name.data());
        }
        names[count++] = name;
      },
      kSettings);
  zexpect_equal(count, 130U);
}

ZTEST(settings_registers, test_every_register_round_trips_and_fits_the_buffer) {
  NvmSettings source{};
  // Make every register differ from its default so a lost field shows up.
  ForEachRegister(
      [&](std::string_view /*name*/, auto& field) {
        using Field = std::remove_cvref_t<decltype(field)>;
        if constexpr (unit::UnitLike<Field>) {
          field = Field{field.Value() * 1.5F + 0.25F};
        } else if constexpr (std::is_same_v<Field, float>) {
          field = field * 1.5F + 0.25F;
        } else if constexpr (std::is_same_v<Field, uint8_t>) {
          field = static_cast<uint8_t>(field + 3U);
        } else if constexpr (std::is_same_v<Field, uint16_t>) {
          field = static_cast<uint16_t>(field + 0x0101U);
        }
      },
      source);

  NvmSettings decoded{};
  std::size_t visited = 0U;
  ForEachRegister(
      [&](std::string_view name, const auto& field, auto& target) {
        std::array<std::byte, kMaxRegisterValueSize> bytes{};
        const std::size_t kSize = EncodeRegisterValue(field, bytes);
        zassert_true(kSize >= 1U && kSize <= kMaxRegisterValueSize, "%s size %u", name.data(), kSize);
        zassert_true(DecodeRegisterValue(std::span<const std::byte>{bytes.data(), kSize}, target),
                     "%s does not decode",
                     name.data());
        ++visited;
      },
      source,
      decoded);
  zexpect_equal(visited, 130U);

  ForEachRegister(
      [&](std::string_view name, const auto& expected, const auto& actual) {
        std::array<std::byte, kMaxRegisterValueSize> a{};
        std::array<std::byte, kMaxRegisterValueSize> b{};
        const std::size_t kSizeA = EncodeRegisterValue(expected, a);
        const std::size_t kSizeB = EncodeRegisterValue(actual, b);
        zassert_equal(kSizeA, kSizeB, "%s", name.data());
        zassert_mem_equal(a.data(), b.data(), kSizeA, "%s", name.data());
      },
      source,
      decoded);
}

ZTEST(settings_registers, test_every_register_is_described) {
  const NvmSettings kSettings{};
  ForEachRegister(
      [&](const RegisterEntry& entry, const auto& field) {
        using Field = std::remove_cvref_t<decltype(field)>;
        const cyphal::RegisterDescription kDescription = DescriptionOf(entry, field);
        zassert_false(kDescription.help.empty(), "%s has no help", entry.name.data());
        zassert_true(kDescription.minimum <= kDescription.maximum, "%s: limits swapped", entry.name.data());
        if constexpr (unit::UnitLike<Field>) {
          if constexpr (!std::is_same_v<typename Field::dimension, unit::dim::Dimensionless>) {
            zassert_false(kDescription.unit.empty(), "%s has no unit", entry.name.data());
          }
        }
        // The drive subscribes and publishes its ports and takes its node-ID at the start.
        const bool kPort = entry.name.starts_with("uavcan.sub.") || entry.name.starts_with("uavcan.pub.") ||
                           entry.name.starts_with("uavcan.srv.") || entry.name == cyphal::reg::kNodeId;
        if (kPort) {
          zassert_equal(kDescription.effect, cyphal::RegisterEffect::kRestart, "%s", entry.name.data());
        }
      },
      kSettings);
}

/// The value just beyond @p bound (above it or below it), when the field type can hold one.
template <typename Field>
[[nodiscard]] std::optional<Field> Beyond(const float bound, const bool above) {
  const float kDirection =
      above ? std::numeric_limits<float>::infinity() : -std::numeric_limits<float>::infinity();
  if constexpr (unit::UnitLike<Field>) {
    return Field{std::nextafter(bound, kDirection)};
  } else if constexpr (std::is_same_v<Field, float>) {
    return std::nextafter(bound, kDirection);
  } else if constexpr (std::is_same_v<Field, uint8_t> || std::is_same_v<Field, uint16_t> ||
                       (std::is_enum_v<Field> && sizeof(Field) == 1U)) {
    using Raw = std::conditional_t<std::is_same_v<Field, uint16_t>, uint16_t, uint8_t>;
    const float kValue = above ? bound + 1.0F : bound - 1.0F;
    if (kValue < 0.0F || kValue > static_cast<float>(std::numeric_limits<Raw>::max())) {
      return std::nullopt;
    }
    return static_cast<Field>(static_cast<Raw>(kValue));
  } else {
    return std::nullopt;  // the node name
  }
}

ZTEST(settings_registers, test_a_value_beyond_each_limit_is_rejected) {
  // The limits of the description are a hint for tools; they must not be
  // tighter than the validation, so a value just outside is always rejected.
  const HardwareCapabilities kCapabilities{};
  NvmSettings candidate{};
  zassert_equal(settings_store_internal::Validate(candidate, kCapabilities), SettingsStatus::kSuccess);
  std::size_t checked = 0U;
  ForEachRegister(
      [&](const RegisterEntry& entry, auto& field) {
        using Field = std::remove_cvref_t<decltype(field)>;
        const Field kOriginal = field;
        for (const bool kAbove : {false, true}) {
          const float kBound = kAbove ? entry.description.maximum : entry.description.minimum;
          if (!std::isfinite(kBound)) {
            continue;
          }
          const std::optional<Field> kOutside = Beyond<Field>(kBound, kAbove);
          if (!kOutside.has_value()) {
            continue;
          }
          field = *kOutside;
          zassert_not_equal(settings_store_internal::Validate(candidate, kCapabilities),
                            SettingsStatus::kSuccess,
                            "%s accepts a value %s its limit",
                            entry.name.data(),
                            kAbove ? "above" : "below");
          field = kOriginal;
          ++checked;
        }
      },
      candidate);
  zexpect_true(checked > 95U, "only %u limits checked", static_cast<unsigned>(checked));
}

ZTEST(settings_registers, test_values_are_little_endian_ieee_floats) {
  std::array<std::byte, kMaxRegisterValueSize> bytes{};
  zexpect_equal(EncodeRegisterValue(1.0F, bytes), 4U);
  zexpect_equal(bytes[0], std::byte{0x00});
  zexpect_equal(bytes[1], std::byte{0x00});
  zexpect_equal(bytes[2], std::byte{0x80});
  zexpect_equal(bytes[3], std::byte{0x3F});
}

ZTEST(settings_registers, test_decode_rejects_a_wrong_size) {
  float value = 0.0F;
  const std::array<std::byte, 3> kShort{};
  zexpect_false(DecodeRegisterValue(std::span<const std::byte>{kShort}, value));
  uint8_t small = 0U;
  const std::array<std::byte, 2> kLong{};
  zexpect_false(DecodeRegisterValue(std::span<const std::byte>{kLong}, small));
  zexpect_false(DecodeRegisterValue(std::span<const std::byte>{}, small));
}

ZTEST(settings_registers, test_node_name_needs_its_terminator_and_fits_the_limit) {
  cyphal::NodeIdentity identity{};
  const std::array<std::byte, 3> kUnterminated{std::byte{'a'}, std::byte{'b'}, std::byte{'c'}};
  zexpect_false(DecodeRegisterValue(std::span<const std::byte>{kUnterminated}, identity.name));
  zexpect_false(DecodeRegisterValue(std::span<const std::byte>{}, identity.name));

  const std::array<std::byte, 4> kTerminated{std::byte{'a'}, std::byte{'b'}, std::byte{'c'}, std::byte{0}};
  zexpect_true(DecodeRegisterValue(std::span<const std::byte>{kTerminated}, identity.name));
  zexpect_equal(identity.GetName(), std::string_view{"abc"});

  identity.SetName("x");
  std::array<std::byte, kMaxRegisterValueSize> bytes{};
  zexpect_equal(EncodeRegisterValue(identity.name, bytes), 2U);
}

}  // namespace unimoc::settings::test

ZTEST_SUITE(settings_registers, nullptr, nullptr, nullptr, nullptr, nullptr);
