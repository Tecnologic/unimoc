/*
       __  ___   ________  _______  ______
      / / / / | / /  _/  |/  / __ \/ ____/
     / / / /  |/ // // /|_/ / / / / /
    / /_/ / /|  // // /  / / /_/ / /___
    \____/_/ |_/___/_/  /_/\____/\____/

    Universal Motor Control  2026 Alexander <tecnologic86@gmail.com> Evers

    This file is part of UNIMOC.

    UNIMOC is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#ifndef UNIMOC_SYSTEM_NODE_IDENTITY_H_
#define UNIMOC_SYSTEM_NODE_IDENTITY_H_

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string_view>

/**
 * @namespace unimoc global namespace
 */
namespace unimoc
{
/**
 * @namespace system coordinate and motor type definitions
 */
namespace system
{

/// Maximum length of the node name string (bytes, excluding NUL terminator).
/// Matches the UAVCAN/Cyphal GetInfo response limit.
inline constexpr uint8_t NODE_NAME_MAX_LEN = 50;

/// Length of the hardware unique-ID array (bytes).
/// Matches the UAVCAN/Cyphal GetInfo response layout.
inline constexpr uint8_t UNIQUE_ID_LEN = 16;

#ifndef UNIMOC_SW_VERSION_MAJOR
#define UNIMOC_SW_VERSION_MAJOR 1
#endif

#ifndef UNIMOC_SW_VERSION_MINOR
#define UNIMOC_SW_VERSION_MINOR 0
#endif

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
struct NodeIdentity
{
    /// Human-readable UTF-8 node name (NUL-terminated).
    /// Matches the `uavcan.node.description` register.
    /// Maximum NODE_NAME_MAX_LEN meaningful characters.
    char name[NODE_NAME_MAX_LEN + 1]{"unimoc"};

    /// Hardware version — major component (read-only, set at compile time).
    uint8_t hw_version_major{1};
    /// Hardware version — minor component.
    uint8_t hw_version_minor{0};

    /// Software version — major component (read-only, set from git version).
    uint8_t sw_version_major{static_cast<uint8_t>(UNIMOC_SW_VERSION_MAJOR)};
    /// Software version — minor component.
    uint8_t sw_version_minor{static_cast<uint8_t>(UNIMOC_SW_VERSION_MINOR)};

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    /**
     * @brief Set the node name from a string_view.
     *
     * Copies at most NODE_NAME_MAX_LEN characters and ensures NUL-termination.
     *
     * @param s  New name (truncated if longer than NODE_NAME_MAX_LEN).
     */
    constexpr void
    set_name(std::string_view s) noexcept
    {
        const std::size_t len = std::min(s.size(),
                                         static_cast<std::size_t>(NODE_NAME_MAX_LEN));
        for (std::size_t i = 0; i < len; ++i)
            name[i] = s[i];
        name[len] = '\0';
    }

    /**
     * @brief Return the node name as a string_view.
     */
    [[nodiscard]] constexpr std::string_view
    get_name() const noexcept
    {
        // Find actual length without relying on strlen (constexpr friendly)
        std::size_t len = 0;
        while (len <= NODE_NAME_MAX_LEN && name[len] != '\0')
            ++len;
        return std::string_view(name, len);
    }

    /**
     * @brief Compare two identity records for equality (name + versions).
     */
    [[nodiscard]] constexpr bool
    operator==(const NodeIdentity& other) const noexcept
    {
        return get_name() == other.get_name()
            && hw_version_major == other.hw_version_major
            && hw_version_minor == other.hw_version_minor
            && sw_version_major == other.sw_version_major
            && sw_version_minor == other.sw_version_minor;
    }

    [[nodiscard]] constexpr bool
    operator!=(const NodeIdentity& other) const noexcept
    {
        return !(*this == other);
    }
};

}  // namespace system
}  // namespace unimoc

#endif /* UNIMOC_SYSTEM_NODE_IDENTITY_H_ */
