/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file firmware_image.hpp
 *    @brief The UNIMOC firmware image header and its CRC32 check (docs/system_plan.md, firmware update).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace unimoc::cyphal {

/**
 * @brief Header in front of the payload of a firmware image (little endian, 40 bytes).
 *
 * | Offset | Size | Field |
 * |---|---|---|
 * | 0 | 4 | magic "UNIM" |
 * | 4 | 2 | header version (1) |
 * | 6 | 1 | software major |
 * | 7 | 1 | software minor |
 * | 8 | 8 | VCS revision |
 * | 16 | 4 | payload size [bytes] |
 * | 20 | 4 | CRC32 (IEEE 802.3, reflected) of the payload |
 * | 24 | 16 | board name, zero padded (`unimoc_jacobi`, `nucleo_h743zi`, `native_sim`) |
 *
 * `scripts/make_firmware_image.py` writes it. The payload is the image of the bootloader
 * (STM32H7: the MCUboot image `zephyr.signed.bin`, signed; the bootloader checks hash and
 * signature at the swap). native_sim: any file.
 */
struct FirmwareImageHeader {
  static constexpr std::size_t kSize = 40U;
  static constexpr std::array<uint8_t, 4> kMagic{'U', 'N', 'I', 'M'};
  static constexpr uint16_t kVersion = 1U;
  static constexpr std::size_t kBoardNameSize = 16U;

  uint8_t software_major{0U};
  uint8_t software_minor{0U};
  uint64_t vcs_revision{0U};
  uint32_t payload_size{0U};
  uint32_t payload_crc{0U};
  std::array<char, kBoardNameSize> board{};

  /// The board name up to its first zero.
  [[nodiscard]] std::string_view Board() const noexcept {
    std::size_t length = 0U;
    while (length < board.size() && board.at(length) != '\0') {
      ++length;
    }
    return std::string_view{board.data(), length};
  }
};

/// Why an image was refused.
enum class ImageStatus : uint8_t {
  kOk,
  kTooShort,     ///< Shorter than the header.
  kBadMagic,     ///< No UNIMOC image.
  kBadVersion,   ///< An unknown header version.
  kWrongBoard,   ///< Built for another board.
  kWrongSize,    ///< The file is not header plus payload.
  kTooLarge,     ///< The payload does not fit the bank.
  kBadChecksum,  ///< The CRC32 of the payload does not match.
  kNoBootImage,  ///< The payload is no image of the bootloader (flash boards: no MCUboot header).
  kNotNewer,     ///< The bootloader would not take it over: its version is not higher than the running one.
};

/// Incremental CRC32 (IEEE 802.3, polynomial 0xEDB88320 reflected): Crc32Update(Crc32Init(), data) ... Final.
[[nodiscard]] constexpr uint32_t Crc32Init() noexcept { return 0xFFFFFFFFU; }

[[nodiscard]] constexpr uint32_t Crc32Update(uint32_t crc, const std::span<const std::byte> data) noexcept {
  for (const std::byte kByte : data) {
    crc ^= static_cast<uint32_t>(kByte);
    for (int bit = 0; bit < 8; ++bit) {
      crc = ((crc & 1U) != 0U) ? ((crc >> 1U) ^ 0xEDB88320U) : (crc >> 1U);
    }
  }
  return crc;
}

[[nodiscard]] constexpr uint32_t Crc32Final(const uint32_t crc) noexcept { return crc ^ 0xFFFFFFFFU; }

namespace image_detail {

[[nodiscard]] constexpr uint32_t Le32(const std::span<const std::byte> bytes,
                                      const std::size_t offset) noexcept {
  return static_cast<uint32_t>(bytes[offset]) | (static_cast<uint32_t>(bytes[offset + 1U]) << 8U) |
         (static_cast<uint32_t>(bytes[offset + 2U]) << 16U) |
         (static_cast<uint32_t>(bytes[offset + 3U]) << 24U);
}

}  // namespace image_detail

/**
 * @brief Reads the header from the first FirmwareImageHeader::kSize bytes of an image.
 * @param bytes At least the header.
 * @param header Receives the fields.
 * @return kOk, kTooShort, kBadMagic or kBadVersion.
 */
[[nodiscard]] constexpr ImageStatus ParseImageHeader(const std::span<const std::byte> bytes,
                                                     FirmwareImageHeader& header) noexcept {
  if (bytes.size() < FirmwareImageHeader::kSize) {
    return ImageStatus::kTooShort;
  }
  for (std::size_t i = 0U; i < FirmwareImageHeader::kMagic.size(); ++i) {
    if (static_cast<uint8_t>(bytes[i]) != FirmwareImageHeader::kMagic.at(i)) {
      return ImageStatus::kBadMagic;
    }
  }
  const auto kVersion =
      static_cast<uint16_t>(static_cast<uint32_t>(bytes[4]) | (static_cast<uint32_t>(bytes[5]) << 8U));
  if (kVersion != FirmwareImageHeader::kVersion) {
    return ImageStatus::kBadVersion;
  }
  header.software_major = static_cast<uint8_t>(bytes[6]);
  header.software_minor = static_cast<uint8_t>(bytes[7]);
  header.vcs_revision = static_cast<uint64_t>(image_detail::Le32(bytes, 8U)) |
                        (static_cast<uint64_t>(image_detail::Le32(bytes, 12U)) << 32U);
  header.payload_size = image_detail::Le32(bytes, 16U);
  header.payload_crc = image_detail::Le32(bytes, 20U);
  for (std::size_t i = 0U; i < FirmwareImageHeader::kBoardNameSize; ++i) {
    header.board.at(i) = static_cast<char>(bytes[24U + i]);
  }
  return ImageStatus::kOk;
}

}  // namespace unimoc::cyphal
