/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file file_service.hpp
 *    @brief uavcan.file.Read on both sides: a file server and the firmware update client.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include "cyphal/firmware_update.hpp"
#include "cyphal/node.hpp"
#include "cyphal/register_value.hpp"

namespace unimoc::cyphal {

namespace port {
/// uavcan.file.Read.1.1.
inline constexpr uint16_t kFileRead = 408U;
}  // namespace port

/// Values of uavcan.file.Error.1.0.
namespace file_error {
inline constexpr uint16_t kOk = 0U;
inline constexpr uint16_t kNotFound = 2U;
inline constexpr uint16_t kIoError = 5U;
}  // namespace file_error

/// The outcome of one read of a FileSource.
struct FileReadResult {
  uint16_t error{file_error::kOk};  ///< uavcan.file.Error value.
  std::size_t size{0U};             ///< Bytes written into the buffer; fewer than asked: the end of the file.
};

/// The files a FileServer serves.
class FileSource {
 public:
  FileSource() = default;
  virtual ~FileSource() = default;
  FileSource(const FileSource&) = delete;
  FileSource& operator=(const FileSource&) = delete;
  FileSource(FileSource&&) = delete;
  FileSource& operator=(FileSource&&) = delete;

  /**
   * @brief Reads up to @p data.size() bytes of @p path from @p offset.
   * @return kNotFound for an unknown path; an offset at or past the end reads 0 bytes.
   */
  [[nodiscard]] virtual FileReadResult Read(std::string_view path,
                                            uint64_t offset,
                                            std::span<std::byte> data) noexcept = 0;
};

/**
 * @brief Serves uavcan.file.Read.1.1 from a FileSource (native_sim: the register description file, F3).
 *
 * Add it with CyphalNode::AddServer(port::kFileRead, FileServer::kRequestExtent, server).
 */
class FileServer final : public ServiceHandler {
 public:
  /// Extent of uavcan.file.Read.1.1 Request [bytes].
  static constexpr std::size_t kRequestExtent = 300U;
  /// Bytes of one answer (uavcan.primitive.Unstructured.1.0).
  static constexpr std::size_t kChunk = 256U;

  /// @param source The files; must outlive the server.
  explicit FileServer(FileSource& source) noexcept : source_{&source} {}

  [[nodiscard]] std::optional<std::size_t> OnRequest(const Transfer& request,
                                                     std::span<std::byte> response) noexcept override;

 private:
  FileSource* source_;
};

/**
 * @brief The firmware update on the bus: BEGIN_SOFTWARE_UPDATE starts a download with uavcan.file.Read
 *        from the commanding node into the bank (FirmwareUpdate), its state is in `unimoc.update.*`.
 *
 * Registers (read-only): `unimoc.update.state` (FirmwareUpdate::State), `unimoc.update.result`
 * (ImageStatus), `unimoc.update.file_error` (uavcan.file.Error or the FirmwareUpdate::kError* codes),
 * `unimoc.update.received` (bytes) and `unimoc.update.version` (major, minor of the image in the bank).
 * Thread context, from the Spin() of the node's thread.
 */
class FirmwareUpdateService final : public ResponseListener, public RegisterProvider {
 public:
  /// Extent of uavcan.file.Read.1.1 Response [bytes].
  static constexpr std::size_t kResponseExtent = 300U;

  /**
   * @param bank The bank to write; erase == nullptr: the board has none and every update is refused.
   * @param board Board name the image must carry (CONFIG_BOARD).
   */
  FirmwareUpdateService(const FirmwareBank& bank, std::string_view board) noexcept : update_{bank, board} {}

  /// Adds the uavcan.file.Read client and the registers to @p node; false when a slot is missing.
  [[nodiscard]] bool Start(CyphalNode& node) noexcept;

  /**
   * @brief Starts a download (BEGIN_SOFTWARE_UPDATE).
   * @param server The node that sent the command, which serves the file.
   * @param path The command parameter.
   * @return The ExecuteCommand answer.
   */
  [[nodiscard]] CommandResult Begin(uint8_t server, std::span<const uint8_t> path) noexcept;

  /// Sends the next request, if one is due.
  void Spin(uint64_t now_us) noexcept;

  /// True while a download runs (the heartbeat mode is SOFTWARE_UPDATE, the drive does not engage).
  [[nodiscard]] bool Downloading() const noexcept {
    return update_.GetState() == FirmwareUpdate::State::kDownloading;
  }

  [[nodiscard]] const FirmwareUpdate& Update() const noexcept { return update_; }

  void OnResponse(const Transfer& response, uint64_t now_us) noexcept override;

  [[nodiscard]] std::size_t Count() const noexcept override;
  [[nodiscard]] std::string_view Name(std::size_t index) const noexcept override;
  [[nodiscard]] RegisterFlags Read(std::size_t index, RegisterValue& value) const noexcept override;
  [[nodiscard]] RegisterWriteResult Write(std::size_t index, const RegisterValue& value) noexcept override;
  [[nodiscard]] RegisterDescription Describe(std::size_t index) const noexcept override;

 private:
  FirmwareUpdate update_;
  CyphalNode* node_{nullptr};
  uint8_t server_{kNodeIdUnset};
  uint8_t transfer_id_{0U};
  bool awaiting_{false};
};

}  // namespace unimoc::cyphal
