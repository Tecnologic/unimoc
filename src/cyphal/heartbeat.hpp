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

#include <array>
#include <cstdint>
#include <gitversion/version.h>

namespace unimoc::cyphal {

/// Fixed subject-ID for uavcan.node.Heartbeat.1.0 (Cyphal specification §5.3.2)
static constexpr uint16_t HEARTBEAT_SUBJECT_ID = 7509;

/// Maximum allowed publication period in seconds (Cyphal specification §5.3.2)
static constexpr uint8_t HEARTBEAT_MAX_PERIOD_S = 1;

/// Node health values for uavcan.node.Heartbeat.1.0
enum class Health : uint8_t {
    NOMINAL  = 0,  ///< The node is functioning properly.
    ADVISORY = 1,  ///< A non-critical anomaly has been detected.
    CAUTION  = 2,  ///< A critical anomaly has been detected; recovery may be possible.
    WARNING  = 3,  ///< The node is about to fail or has already failed.
};

/// Node operating mode values for uavcan.node.Heartbeat.1.0
enum class Mode : uint8_t {
    OPERATIONAL     = 0,  ///< Normal operation.
    INITIALIZATION  = 1,  ///< Initialisation in progress; not yet ready for normal operation.
    MAINTENANCE     = 2,  ///< Under maintenance; non-operational.
    SOFTWARE_UPDATE = 3,  ///< Performing a software update; non-operational.
};

/// Serialized size of uavcan.node.Heartbeat.1.0 in bytes (7 bytes = 56 bits)
static constexpr std::size_t HEARTBEAT_PAYLOAD_SIZE = 7;

namespace detail {

/// Compile-time conversion of a decimal string to an integer.
constexpr uint32_t str_to_uint(const char* s)
{
    uint32_t result = 0;
    while (*s >= '0' && *s <= '9') {
        result = result * 10u + static_cast<uint32_t>(*s - '0');
        ++s;
    }
    return result;
}

/// Parse the major version component from a version string (e.g. "v1.2" → 1).
constexpr uint8_t parse_major(const char* s)
{
    if (*s == 'v') { ++s; }
    return static_cast<uint8_t>(str_to_uint(s));
}

/// Parse the minor version component from a version string (e.g. "v1.2" → 2).
constexpr uint8_t parse_minor(const char* s)
{
    if (*s == 'v') { ++s; }
    while (*s != '.' && *s != '\0') { ++s; }
    if (*s == '.') { ++s; }
    return static_cast<uint8_t>(str_to_uint(s));
}

} // namespace detail

/// Software major version derived from the git tag via gitversion.
static constexpr uint8_t SW_VERSION_MAJOR = detail::parse_major(version::VERSION_STRING);

/// Software minor version derived from the git tag via gitversion.
static constexpr uint8_t SW_VERSION_MINOR = detail::parse_minor(version::VERSION_STRING);

/// Vendor-specific status code that encodes the software version.
/// Bit layout: bits[18:10] = MAJOR (9 bits), bits[9:0] = MINOR (10 bits).
static constexpr uint32_t VERSION_VSSC =
    (static_cast<uint32_t>(SW_VERSION_MAJOR) << 10u) |
    static_cast<uint32_t>(SW_VERSION_MINOR);

/// Serialize a uavcan.node.Heartbeat.1.0 message into a 7-byte buffer.
///
/// The payload is DSDL bit-packed in little-endian order:
///   bytes [3:0]  uptime   (uint32, seconds since node start)
///   bits  [33:32] health   (uint2)
///   bits  [36:34] mode     (uint3)
///   bits  [55:37] vssc     (uint19, vendor-specific status code)
///
/// @param uptime_s  Seconds elapsed since the node was started.
/// @param health    Node health status (default: NOMINAL).
/// @param mode      Node operating mode (default: OPERATIONAL).
/// @param vssc      Vendor-specific status code (default: VERSION_VSSC with
///                  the software version encoded as major/minor).
/// @return 7-byte serialized heartbeat payload ready for transmission.
[[nodiscard]] constexpr std::array<uint8_t, HEARTBEAT_PAYLOAD_SIZE> serialize_heartbeat(
    uint32_t uptime_s,
    Health   health = Health::NOMINAL,
    Mode     mode   = Mode::OPERATIONAL,
    uint32_t vssc   = VERSION_VSSC)
{
    std::array<uint8_t, HEARTBEAT_PAYLOAD_SIZE> buf{};

    // uptime: 4 bytes, little-endian
    buf[0] = static_cast<uint8_t>(uptime_s);
    buf[1] = static_cast<uint8_t>(uptime_s >> 8u);
    buf[2] = static_cast<uint8_t>(uptime_s >> 16u);
    buf[3] = static_cast<uint8_t>(uptime_s >> 24u);

    // Remaining 24 bits: health[1:0] | mode[2:0] | vssc[18:0]
    const uint32_t tail =
        (static_cast<uint32_t>(health) & 0x3u) |
        ((static_cast<uint32_t>(mode)  & 0x7u) << 2u) |
        ((vssc                         & 0x7FFFFu) << 5u);

    buf[4] = static_cast<uint8_t>(tail);
    buf[5] = static_cast<uint8_t>(tail >> 8u);
    buf[6] = static_cast<uint8_t>(tail >> 16u);

    return buf;
}

} // namespace unimoc::cyphal
