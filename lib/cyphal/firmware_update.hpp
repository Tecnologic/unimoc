/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file firmware_update.hpp
 *    @brief Firmware download over uavcan.file.Read into the second flash bank and its check.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include "firmware_image.hpp"

namespace unimoc::cyphal {

/**
 * @brief The bank an update is written into (STM32H7: slot 1 of MCUboot in flash bank 2; native_sim: a
 * buffer).
 *
 * The bank holds the payload of the image (the file without the UNIMOC header), from offset 0. Thread
 * context.
 */
struct FirmwareBank {
  /// Prepares a sequential write from offset 0; false: no bank (the update is refused).
  /// The flash bank erases the sectors as the writes reach them, so that this does not stall the node.
  bool (*begin)() noexcept;
  /// Writes @p data at @p offset (sequential, from 0).
  bool (*write)(uint32_t offset, std::span<const std::byte> data) noexcept;
  /// Writes out what write() still buffers (called once, after the last write).
  bool (*finish)() noexcept;
  /// Reads back @p data.size() bytes at @p offset.
  bool (*read)(uint32_t offset, std::span<std::byte> data) noexcept;
  /// Makes the bootloader take the checked image over at the next start (it runs once, and confirms
  /// itself, or the bootloader swaps back). kOk, or why not (kNoBootImage, kNotNewer). Null: nothing to do.
  ImageStatus (*activate)() noexcept;
  /// Bytes the bank holds.
  uint32_t capacity;
};

/// One uavcan.file.Read request the update wants sent.
struct FileReadRequest {
  uint8_t server{0U};
  uint64_t offset{0U};
  std::string_view path;
};

/**
 * @brief Downloads an image from a file server (the node that sent BEGIN_SOFTWARE_UPDATE) and checks it.
 *
 * Begin() prepares the bank; Poll() then hands out one uavcan.file.Read request
 * at a time (256 bytes each), OnResponse() writes its data. The first answer
 * carries the header (FirmwareImageHeader), checked against the board before
 * anything is written; the bank gets the payload behind it. A short answer is
 * the end of the file: the size is checked and the CRC32 of the payload read
 * back from the bank. A request without an answer is repeated after
 * kTimeoutUs, up to kRetries times. kReady means the bank holds a checked
 * image and the bootloader was told to take it over at the next restart (the
 * swap is the bootloader's, not on native_sim).
 */
class FirmwareUpdate {
 public:
  enum class State : uint8_t {
    kIdle = 0,
    kDownloading = 1,
    kReady = 2,   ///< A checked image waits in the bank.
    kFailed = 3,  ///< See Result() and FileError().
  };

  static constexpr std::size_t kMaxPath = 255U;
  static constexpr std::size_t kChunk = 256U;
  static constexpr uint64_t kTimeoutUs = 1'000'000U;
  static constexpr uint8_t kRetries = 3U;

  FirmwareUpdate(const FirmwareBank& bank, std::string_view board) noexcept : bank_{&bank}, board_{board} {}

  /**
   * @brief Starts a download of @p path from @p server.
   * @return False while a download runs, without a bank, or for an empty or too long path.
   */
  [[nodiscard]] bool Begin(const uint8_t server, const std::string_view path) noexcept {
    if (state_ == State::kDownloading || path.empty() || path.size() > kMaxPath || bank_->begin == nullptr ||
        !bank_->begin()) {
      return false;
    }
    std::copy(path.begin(), path.end(), path_.begin());
    path_size_ = path.size();
    server_ = server;
    received_ = 0U;
    retries_ = 0U;
    result_ = ImageStatus::kOk;
    file_error_ = 0U;
    state_ = State::kDownloading;
    pending_ = false;
    return true;
  }

  /// Stops a running download (kFailed).
  void Abort() noexcept {
    if (state_ == State::kDownloading) {
      state_ = State::kFailed;
    }
  }

  /// The request to send now, if any; after it was sent, the update waits for its answer.
  [[nodiscard]] std::optional<FileReadRequest> Poll(const uint64_t now_us) noexcept {
    if (state_ != State::kDownloading) {
      return std::nullopt;
    }
    if (pending_) {
      if (now_us - sent_us_ < kTimeoutUs) {
        return std::nullopt;
      }
      if (++retries_ > kRetries) {
        Fail(ImageStatus::kOk, kErrorTimeout);
        return std::nullopt;
      }
    }
    pending_ = true;
    sent_us_ = now_us;
    return FileReadRequest{server_, received_, std::string_view{path_.data(), path_size_}};
  }

  /**
   * @brief The answer to the last request.
   * @param error uavcan.file.Error value (0: OK).
   * @param data The bytes at the requested offset; fewer than kChunk: the end of the file.
   */
  void OnResponse(const uint16_t error, const std::span<const std::byte> data) noexcept {
    if (state_ != State::kDownloading || !pending_) {
      return;
    }
    pending_ = false;
    retries_ = 0U;
    if (error != 0U) {
      Fail(ImageStatus::kOk, error);
      return;
    }
    std::span<const std::byte> payload = data;
    if (received_ == 0U) {
      // The header leads the file: it is checked before the bank is touched.
      const ImageStatus kChecked = CheckHeader(data);
      if (kChecked != ImageStatus::kOk) {
        Fail(kChecked, 0U);
        return;
      }
      payload = data.subspan(FirmwareImageHeader::kSize);
    }
    const uint64_t kBankOffset = (received_ == 0U) ? 0U : received_ - FirmwareImageHeader::kSize;
    if (kBankOffset + payload.size() > bank_->capacity) {
      Fail(ImageStatus::kTooLarge, 0U);
      return;
    }
    if (!payload.empty() && !bank_->write(static_cast<uint32_t>(kBankOffset), payload)) {
      Fail(ImageStatus::kOk, kErrorBank);
      return;
    }
    received_ += data.size();
    if (data.size() < kChunk) {
      Verify();
    }
  }

  [[nodiscard]] State GetState() const noexcept { return state_; }
  [[nodiscard]] ImageStatus Result() const noexcept { return result_; }
  /// uavcan.file.Error of a failed read, kErrorTimeout or kErrorBank; 0 otherwise.
  [[nodiscard]] uint16_t FileError() const noexcept { return file_error_; }
  [[nodiscard]] uint64_t Received() const noexcept { return received_; }
  [[nodiscard]] const FirmwareImageHeader& Header() const noexcept { return header_; }

  static constexpr uint16_t kErrorTimeout = 0xFFFFU;  ///< No answer from the file server.
  static constexpr uint16_t kErrorBank = 0xFFFEU;     ///< The bank refused a write or a read.

 private:
  void Fail(const ImageStatus result, const uint16_t file_error) noexcept {
    result_ = result;
    file_error_ = file_error;
    state_ = State::kFailed;
  }

  /// Parses the header at the start of the file and checks it against the board and the bank.
  [[nodiscard]] ImageStatus CheckHeader(const std::span<const std::byte> first) noexcept {
    const ImageStatus kParsed = ParseImageHeader(first, header_);
    if (kParsed != ImageStatus::kOk) {
      return kParsed;
    }
    if (header_.Board() != board_) {
      return ImageStatus::kWrongBoard;
    }
    if (header_.payload_size > bank_->capacity) {
      return ImageStatus::kTooLarge;
    }
    return ImageStatus::kOk;
  }

  void Verify() noexcept {
    std::array<std::byte, kChunk> chunk{};
    if (received_ < FirmwareImageHeader::kSize) {
      Fail(ImageStatus::kTooShort, 0U);
      return;
    }
    if (static_cast<uint64_t>(header_.payload_size) + FirmwareImageHeader::kSize != received_) {
      Fail(ImageStatus::kWrongSize, 0U);
      return;
    }
    if (bank_->finish != nullptr && !bank_->finish()) {
      Fail(ImageStatus::kOk, kErrorBank);
      return;
    }
    uint32_t crc = Crc32Init();
    for (uint64_t offset = 0U; offset < header_.payload_size; offset += kChunk) {
      const auto kLength =
          static_cast<std::size_t>(std::min<uint64_t>(kChunk, header_.payload_size - offset));
      const std::span<std::byte> kPart{chunk.data(), kLength};
      if (!bank_->read(static_cast<uint32_t>(offset), kPart)) {
        Fail(ImageStatus::kOk, kErrorBank);
        return;
      }
      crc = Crc32Update(crc, kPart);
    }
    if (Crc32Final(crc) != header_.payload_crc) {
      Fail(ImageStatus::kBadChecksum, 0U);
      return;
    }
    if (bank_->activate != nullptr) {
      const ImageStatus kActivated = bank_->activate();
      if (kActivated != ImageStatus::kOk) {
        Fail(kActivated, 0U);
        return;
      }
    }
    state_ = State::kReady;
  }

  const FirmwareBank* bank_;
  std::string_view board_;
  std::array<char, kMaxPath> path_{};
  std::size_t path_size_{0U};
  uint8_t server_{0U};
  uint64_t received_{0U};
  uint8_t retries_{0U};
  bool pending_{false};
  uint64_t sent_us_{0U};
  State state_{State::kIdle};
  ImageStatus result_{ImageStatus::kOk};
  uint16_t file_error_{0U};
  FirmwareImageHeader header_{};
};

}  // namespace unimoc::cyphal
