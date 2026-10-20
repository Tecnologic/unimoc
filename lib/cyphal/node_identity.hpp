/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file node_identity.hpp
 *    @brief Persistent Cyphal node identity and version information.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <string_view>
#include "cyphal/sw_version.hpp"

/**
 * @namespace unimoc::cyphal Cyphal protocol and application definitions
 */
namespace unimoc::cyphal {

/// Maximum length of the node name string (bytes, excluding NUL terminator).
/// Matches the UAVCAN/Cyphal GetInfo response limit.
inline constexpr uint8_t kNodeNameMaxLen = 50;

/// Length of the hardware unique-ID array (bytes).
/// Matches the UAVCAN/Cyphal GetInfo response layout.
inline constexpr uint8_t kUniqueIdLen = 16;

/**
 * @brief Node identity record for a UNIMOC drive node.
 *
 * This struct is the authoritative source for the values returned by the
 * Cyphal `uavcan.node.GetInfo` response.  It is persisted in NVM
 * (via NvmSettings::identity) so that the node name survives power cycles.
 *
 * Cyphal interface
 * ----------------
 * - The `name` field maps to `uavcan.node.description` register (read/write,
 *   string).  Writing this register over Cyphal updates the name in RAM; the
 *   application should persist it to NVM when the `uavcan.node.description`
 *   register write is acknowledged.
 * - `hw_version_major` / `hw_version_minor` and `sw_version_major` /
 *   `sw_version_minor` are read-only registers exposed as
 *   `unimoc.hw.version` and `unimoc.sw.version`.
 * - The 16-byte hardware unique-ID is read directly from hardware by the
 *   platform layer and attached to `uavcan.node.GetInfo` responses at runtime.
 *
 * Hardware unique ID
 * ------------------
 * The 16-byte unique-ID is not stored in this persisted struct.  It must be
 * read by the hardware layer at startup (e.g., from STM32 UID registers) and
 * provided directly to Cyphal GetInfo / PnP handling.
 *
 * Application identity
 * --------------------
 * Set `name` via Cyphal to give the node a human-readable identity that
 * describes its role in the system (e.g., "unimoc.propulsion.left").
 * The name is preserved in NVM and is broadcast as part of the heartbeat
 * NodeInfo.
 */
struct NodeIdentity {
  /// Human-readable UTF-8 node name (NUL-terminated).
  /// Matches the `uavcan.node.description` register.
  /// Maximum NODE_NAME_MAX_LEN meaningful characters.
  std::array<char, kNodeNameMaxLen + 1> name{'u', 'n', 'i', 'm', 'o', 'c'};

  /// Hardware version — major component (read-only, set at compile time).
  uint8_t hw_version_major{1};
  /// Hardware version — minor component.
  uint8_t hw_version_minor{0};

  /// Software version — major component (read-only, set from git version).
  uint8_t sw_version_major{static_cast<uint8_t>(kSwVersionMajor)};
  /// Software version — minor component.
  uint8_t sw_version_minor{static_cast<uint8_t>(kSwVersionMinor)};

  // -------------------------------------------------------------------------
  // Helpers
  // -------------------------------------------------------------------------

  /**
   * @brief Set the node name from a string_view.
   *
   * Copies at most NODE_NAME_MAX_LEN characters and ensures NUL-termination.
   *
   * @param new_name  New name (truncated if longer than NODE_NAME_MAX_LEN).
   */
  constexpr void SetName(std::string_view new_name) noexcept {
    const std::size_t kLen = std::min(new_name.size(), static_cast<std::size_t>(kNodeNameMaxLen));
    for (std::size_t i = 0; i < kLen; ++i) {
      name[i] = new_name[i];
    }
    name[kLen] = '\0';
  }

  /**
   * @brief Return the node name as a string_view.
   * @return The NUL-terminated name without the trailing NUL character.
   */
  [[nodiscard]] constexpr std::string_view GetName() const noexcept {
    // Find actual length without relying on strlen (constexpr friendly)
    std::size_t len = 0;
    while (len <= kNodeNameMaxLen && name[len] != '\0') {
      ++len;
    }
    return {name.data(), len};
  }

  /**
   * @brief Compare two identity records for equality (name + versions).
   * @param other Identity record to compare with this record.
   * @return `true` when the name and all version fields match.
   */
  [[nodiscard]] constexpr bool operator==(const NodeIdentity& other) const noexcept {
    return GetName() == other.GetName() && hw_version_major == other.hw_version_major &&
           hw_version_minor == other.hw_version_minor && sw_version_major == other.sw_version_major &&
           sw_version_minor == other.sw_version_minor;
  }

  /**
   * @brief Compare two identity records for inequality.
   * @param other Identity record to compare with this record.
   * @return `true` when at least one name or version field differs.
   */
  [[nodiscard]] constexpr bool operator!=(const NodeIdentity& other) const noexcept {
    return !(*this == other);
  }
};

}  // namespace unimoc::cyphal
