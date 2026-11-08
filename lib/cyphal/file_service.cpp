/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file file_service.cpp
 *    @brief uavcan.file.Read server and the firmware update client.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "cyphal/file_service.hpp"
#include <uavcan/file/Read_1_1.h>
#include <algorithm>
#include <array>
#include "cyphal/cyphal_interface.hpp"

namespace unimoc::cyphal {

namespace {

static_assert(FileServer::kRequestExtent == uavcan_file_Read_Request_1_1_EXTENT_BYTES_);
static_assert(FirmwareUpdateService::kResponseExtent == uavcan_file_Read_Response_1_1_EXTENT_BYTES_);
static_assert(FileServer::kChunk == uavcan_primitive_Unstructured_1_0_value_ARRAY_CAPACITY_);
static_assert(FirmwareUpdate::kChunk == FileServer::kChunk);
static_assert(FirmwareUpdate::kMaxPath == uavcan_file_Path_2_0_path_ARRAY_CAPACITY_);

[[nodiscard]] uint8_t* OutputBytes(std::span<std::byte> buffer) noexcept {
  return reinterpret_cast<uint8_t*>(buffer.data());
}

[[nodiscard]] const uint8_t* InputBytes(std::span<const std::byte> buffer) noexcept {
  return reinterpret_cast<const uint8_t*>(buffer.data());
}

enum class UpdateRegister : uint8_t { kState, kResult, kFileError, kReceived, kVersion, kCount };

constexpr std::array<std::string_view, static_cast<std::size_t>(UpdateRegister::kCount)> kNames{
    "unimoc.update.state",
    "unimoc.update.result",
    "unimoc.update.file_error",
    "unimoc.update.received",
    "unimoc.update.version",
};

/// A read-only register: unit and help only.
[[nodiscard]] constexpr RegisterDescription ReadOnly(const std::string_view unit,
                                                     const std::string_view help) noexcept {
  RegisterDescription description{};
  description.unit = unit;
  description.help = help;
  return description;
}

}  // namespace

// ----- FileServer ---------------------------------------------------------------

std::optional<std::size_t> FileServer::OnRequest(const Transfer& request,
                                                 const std::span<std::byte> response) noexcept {
  uavcan_file_Read_Request_1_1 read{};
  std::size_t request_size = request.payload.size();
  if (uavcan_file_Read_Request_1_1_deserialize_(&read, InputBytes(request.payload), &request_size) < 0) {
    return std::nullopt;
  }
  const std::string_view kPath{reinterpret_cast<const char*>(read.path.path.elements), read.path.path.count};
  uavcan_file_Read_Response_1_1 answer{};
  std::span<std::byte> data{reinterpret_cast<std::byte*>(answer.data.value.elements), kChunk};
  const FileReadResult kResult = source_->Read(kPath, read.offset, data);
  answer._error.value = kResult.error;
  answer.data.value.count = (kResult.error == file_error::kOk) ? std::min(kResult.size, kChunk) : 0U;
  std::size_t size = response.size();
  if (uavcan_file_Read_Response_1_1_serialize_(&answer, OutputBytes(response), &size) < 0) {
    return std::nullopt;
  }
  return size;
}

// ----- FirmwareUpdateService ------------------------------------------------------

bool FirmwareUpdateService::Start(CyphalNode& node) noexcept {
  node_ = &node;
  const bool kClient = node.AddClient(port::kFileRead, kResponseExtent, *this);
  return node.AddRegisterProvider(*this) && kClient;
}

CommandResult FirmwareUpdateService::Begin(const uint8_t server,
                                           const std::span<const uint8_t> path) noexcept {
  if (server > kNodeIdMax) {
    return {cmd::kStatusBadParameter, "no file server"};
  }
  if (Downloading()) {
    return {cmd::kStatusBadState, "update running"};
  }
  const std::string_view kPath{reinterpret_cast<const char*>(path.data()), path.size()};
  if (kPath.empty() || kPath.size() > FirmwareUpdate::kMaxPath) {
    return {cmd::kStatusBadParameter, "no image path"};
  }
  if (!update_.Begin(server, kPath)) {
    return {cmd::kStatusFailure, "no update bank"};
  }
  server_ = server;
  awaiting_ = false;
  return {cmd::kStatusSuccess, ""};
}

void FirmwareUpdateService::Spin(const uint64_t now_us) noexcept {
  if (node_ == nullptr) {
    return;
  }
  const std::optional<FileReadRequest> kRequest = update_.Poll(now_us);
  if (!kRequest.has_value()) {
    return;
  }
  uavcan_file_Read_Request_1_1 read{};
  read.offset = kRequest->offset;
  read.path.path.count = kRequest->path.size();
  std::copy(kRequest->path.begin(), kRequest->path.end(), std::begin(read.path.path.elements));
  std::array<std::byte, uavcan_file_Read_Request_1_1_SERIALIZATION_BUFFER_SIZE_BYTES_> buffer{};
  std::size_t size = buffer.size();
  if (uavcan_file_Read_Request_1_1_serialize_(&read, OutputBytes(buffer), &size) < 0) {
    return;
  }
  ++transfer_id_;
  awaiting_ = true;
  // A request the transport refuses is repeated after FirmwareUpdate::kTimeoutUs.
  (void)node_->Request(port::kFileRead,
                       kRequest->server,
                       transfer_id_,
                       std::span<const std::byte>{buffer.data(), size});
}

void FirmwareUpdateService::OnResponse(const Transfer& response, const uint64_t /*now_us*/) noexcept {
  if (!awaiting_ || response.source_node_id != server_ || response.transfer_id != transfer_id_) {
    return;  // another node, or the answer to a repeated request
  }
  uavcan_file_Read_Response_1_1 answer{};
  std::size_t size = response.payload.size();
  if (uavcan_file_Read_Response_1_1_deserialize_(&answer, InputBytes(response.payload), &size) < 0) {
    return;
  }
  awaiting_ = false;
  update_.OnResponse(answer._error.value,
                     std::span<const std::byte>{reinterpret_cast<const std::byte*>(
                                                    answer.data.value.elements),
                                                answer.data.value.count});
}

std::size_t FirmwareUpdateService::Count() const noexcept { return kNames.size(); }

std::string_view FirmwareUpdateService::Name(const std::size_t index) const noexcept {
  return index < kNames.size() ? kNames.at(index) : std::string_view{};
}

RegisterFlags FirmwareUpdateService::Read(const std::size_t index, RegisterValue& value) const noexcept {
  switch (static_cast<UpdateRegister>(index)) {
    case UpdateRegister::kState:
      SetNatural8(value, static_cast<uint8_t>(update_.GetState()));
      break;
    case UpdateRegister::kResult:
      SetNatural8(value, static_cast<uint8_t>(update_.Result()));
      break;
    case UpdateRegister::kFileError:
      SetNatural16(value, update_.FileError());
      break;
    case UpdateRegister::kReceived:
      SetNatural32(value, static_cast<uint32_t>(update_.Received()));
      break;
    case UpdateRegister::kVersion: {
      const bool kReady = update_.GetState() == FirmwareUpdate::State::kReady;
      const std::array<uint8_t, 2> kVersion{kReady ? update_.Header().software_major : uint8_t{0U},
                                            kReady ? update_.Header().software_minor : uint8_t{0U}};
      uavcan_register_Value_1_0_select_natural8_(&value);
      value.natural8.value.count = kVersion.size();
      std::copy(kVersion.begin(), kVersion.end(), std::begin(value.natural8.value.elements));
      break;
    }
    case UpdateRegister::kCount:
    default:
      SetEmpty(value);
      break;
  }
  return RegisterFlags{};
}

RegisterWriteResult FirmwareUpdateService::Write(const std::size_t /*index*/,
                                                 const RegisterValue& /*value*/) noexcept {
  return RegisterWriteResult::kReadOnly;
}

RegisterDescription FirmwareUpdateService::Describe(const std::size_t index) const noexcept {
  switch (static_cast<UpdateRegister>(index)) {
    case UpdateRegister::kState:
      return ReadOnly("", "Firmware update: 0 idle, 1 downloading, 2 a checked image waits, 3 failed");
    case UpdateRegister::kResult:
      return ReadOnly("",
                      "Image check: 0 ok, 1 too short, 2 no UNIMOC image, 3 header version, 4 board, 5 size, "
                      "6 too large, 7 CRC");
    case UpdateRegister::kFileError:
      return ReadOnly("", "uavcan.file.Error of a failed read; 65535 no answer, 65534 bank error");
    case UpdateRegister::kReceived:
      return ReadOnly("B", "Bytes of the image written into the update bank");
    case UpdateRegister::kVersion:
      return ReadOnly("", "Software version (major, minor) of the checked image in the bank");
    case UpdateRegister::kCount:
    default:
      return {};
  }
}

}  // namespace unimoc::cyphal
