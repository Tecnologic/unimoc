/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file node.cpp
 *    @brief Transport-injected Cyphal node: heartbeat, GetInfo, registers,
 *           ExecuteCommand, plug-and-play, port list and diagnostics.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "cyphal/node.hpp"
#include <uavcan/_register/Access_1_0.h>
#include <uavcan/_register/List_1_0.h>
#include <uavcan/diagnostic/Record_1_1.h>
#include <uavcan/node/ExecuteCommand_1_3.h>
#include <uavcan/node/GetInfo_1_0.h>
#include <uavcan/node/Heartbeat_1_0.h>
#include <uavcan/pnp/NodeIDAllocationData_1_0.h>
#include <algorithm>
#include <cstring>

namespace unimoc::cyphal {

namespace {

/// Mask of the 48-bit plug-and-play hash.
constexpr uint64_t kUniqueIdHashMask = (uint64_t{1} << 48U) - 1U;

/// Services this node serves, for the port list.
constexpr std::array<uint16_t, 4> kServedServices{port::kRegisterAccess,
                                                  port::kRegisterList,
                                                  port::kGetInfo,
                                                  port::kExecuteCommand};

[[nodiscard]] uint8_t* OutputBytes(std::span<std::byte> buffer) noexcept {
  return reinterpret_cast<uint8_t*>(buffer.data());
}

[[nodiscard]] const uint8_t* InputBytes(std::span<const std::byte> buffer) noexcept {
  return reinterpret_cast<const uint8_t*>(buffer.data());
}

/// Little-endian writer for the hand-serialized port list.
class Writer {
 public:
  explicit Writer(std::span<std::byte> buffer) noexcept : buffer_{buffer} {}

  void U8(const uint8_t value) noexcept {
    if (size_ < buffer_.size()) {
      buffer_[size_] = static_cast<std::byte>(value);
    }
    ++size_;
  }
  void U16(const uint16_t value) noexcept {
    U8(static_cast<uint8_t>(value & 0xFFU));
    U8(static_cast<uint8_t>(value >> 8U));
  }
  void U32(const uint32_t value) noexcept {
    U16(static_cast<uint16_t>(value & 0xFFFFU));
    U16(static_cast<uint16_t>(value >> 16U));
  }
  [[nodiscard]] std::size_t Size() const noexcept { return size_; }
  [[nodiscard]] bool Fits() const noexcept { return size_ <= buffer_.size(); }

 private:
  std::span<std::byte> buffer_;
  std::size_t size_{0U};
};

/// uavcan.node.port.SubjectIDList.1.0, delimited, as a sparse list.
void WriteSubjectList(Writer& writer, const std::span<const uint16_t> subjects) noexcept {
  const auto kCount = static_cast<uint8_t>(std::min<std::size_t>(subjects.size(), 255U));
  writer.U32(2U + (2U * static_cast<uint32_t>(kCount)));  // delimiter header
  writer.U8(1U);                                          // union tag: sparse_list
  writer.U8(kCount);
  for (std::size_t i = 0U; i < kCount; ++i) {
    writer.U16(static_cast<uint16_t>(subjects[i] & 0x1FFFU));
  }
}

/// uavcan.node.port.ServiceIDList.1.0, delimited: a 512 bit mask.
void WriteServiceList(Writer& writer, const std::span<const uint16_t> services) noexcept {
  constexpr uint32_t kMaskBytes = 64U;
  writer.U32(kMaskBytes);
  for (uint32_t byte = 0U; byte < kMaskBytes; ++byte) {
    uint8_t bits = 0U;
    for (const uint16_t kService : services) {
      if ((kService / 8U) == byte) {
        bits = static_cast<uint8_t>(bits | (1U << (kService % 8U)));
      }
    }
    writer.U8(bits);
  }
}

}  // namespace

bool CyphalNode::AddRegisterProvider(RegisterProvider& provider) noexcept {
  if (provider_count_ >= kMaxRegisterProviders) {
    return false;
  }
  providers_[provider_count_++] = &provider;
  return true;
}

bool CyphalNode::Subscribe(const uint16_t subject_id,
                           const std::size_t extent,
                           MessageListener& listener) noexcept {
  if (subject_id == kUnsetPortId) {
    return true;
  }
  if (listener_count_ >= kMaxListeners ||
      !transport_->Subscribe(TransferKind::kMessage, subject_id, extent)) {
    return false;
  }
  listeners_[listener_count_++] = Listener{subject_id, &listener};
  return true;
}

bool CyphalNode::AddPublisher(const Publisher& publisher) noexcept {
  if (publisher.port_id == kUnsetPortId) {
    return true;
  }
  if (published_count_ >= kMaxPublishers) {
    return false;
  }
  published_subjects_[published_count_++] = publisher.port_id;
  return true;
}

bool CyphalNode::Start(const uint8_t node_id, const uint64_t now_us) noexcept {
  start_us_ = now_us;
  next_heartbeat_us_ = now_us;
  next_port_list_us_ = now_us + kPortListPeriodUs;
  allocation_seed_ = static_cast<uint32_t>(UniqueIdHash() ^ (UniqueIdHash() >> 32U));
  next_allocation_us_ = now_us + (allocation_seed_ % 1'000'000U);
  transport_->SetLocalNodeId(node_id <= kNodeIdMax ? node_id : kNodeIdUnset);

  bool subscribed = transport_->Subscribe(TransferKind::kServiceRequest,
                                          port::kGetInfo,
                                          uavcan_node_GetInfo_Request_1_0_EXTENT_BYTES_);
  subscribed = transport_->Subscribe(TransferKind::kServiceRequest,
                                     port::kRegisterList,
                                     uavcan_register_List_Request_1_0_EXTENT_BYTES_) &&
               subscribed;
  subscribed = transport_->Subscribe(TransferKind::kServiceRequest,
                                     port::kRegisterAccess,
                                     uavcan_register_Access_Request_1_0_EXTENT_BYTES_) &&
               subscribed;
  subscribed = transport_->Subscribe(TransferKind::kServiceRequest,
                                     port::kExecuteCommand,
                                     uavcan_node_ExecuteCommand_Request_1_3_EXTENT_BYTES_) &&
               subscribed;
  if (NodeId() == kNodeIdUnset) {
    subscribed = transport_->Subscribe(TransferKind::kMessage,
                                       port::kNodeIdAllocation,
                                       uavcan_pnp_NodeIDAllocationData_1_0_EXTENT_BYTES_) &&
                 subscribed;
  }
  started_ = true;
  return subscribed;
}

void CyphalNode::Spin(const uint64_t now_us) noexcept {
  transport_->Process(now_us);
  if (!started_) {
    return;
  }
  for (std::size_t i = 0U; i < kMaxTransfersPerSpin; ++i) {
    Transfer transfer{};
    if (transport_->Receive(rx_buffer_, transfer) != ReceiveResult::kReceived) {
      break;
    }
    Dispatch(transfer, now_us);
  }

  if (NodeId() == kNodeIdUnset) {
    if (now_us >= next_allocation_us_) {
      RequestNodeId(now_us);
    }
  } else {
    if (now_us >= next_heartbeat_us_) {
      PublishHeartbeat(now_us);
      next_heartbeat_us_ += kHeartbeatPeriodUs;
      if (next_heartbeat_us_ <= now_us) {
        next_heartbeat_us_ = now_us + kHeartbeatPeriodUs;  // after a stall: no burst
      }
    }
    if (now_us >= next_port_list_us_) {
      PublishPortList();
      next_port_list_us_ = now_us + kPortListPeriodUs;
    }
  }
  transport_->Process(now_us);
}

void CyphalNode::Dispatch(const Transfer& transfer, const uint64_t now_us) noexcept {
  if (transfer.kind == TransferKind::kServiceRequest) {
    if (NodeId() == kNodeIdUnset) {
      return;
    }
    switch (transfer.port_id) {
      case port::kGetInfo:
        HandleGetInfo(transfer);
        break;
      case port::kRegisterList:
        HandleRegisterList(transfer);
        break;
      case port::kRegisterAccess:
        HandleRegisterAccess(transfer);
        break;
      case port::kExecuteCommand:
        HandleExecuteCommand(transfer);
        break;
      default:
        break;
    }
    return;
  }
  if (transfer.kind != TransferKind::kMessage) {
    return;
  }
  if (transfer.port_id == port::kNodeIdAllocation) {
    HandleAllocation(transfer);
    return;
  }
  for (std::size_t i = 0U; i < listener_count_; ++i) {
    if (listeners_[i].subject_id == transfer.port_id) {
      listeners_[i].listener->OnMessage(transfer, now_us);
    }
  }
}

void CyphalNode::Respond(const Transfer& request, const std::size_t size) noexcept {
  const Transfer kResponse{
      .port_id = request.port_id,
      .source_node_id = NodeId(),
      .destination_node_id = request.source_node_id,
      .transfer_id = request.transfer_id,
      .kind = TransferKind::kServiceResponse,
      .priority = request.priority,
      .payload = std::span<const std::byte>{tx_buffer_.data(), size},
  };
  (void)transport_->Send(kResponse);
}

void CyphalNode::HandleGetInfo(const Transfer& request) noexcept {
  uavcan_node_GetInfo_Response_1_0 response{};
  response.protocol_version.major = 1U;
  response.protocol_version.minor = 0U;
  response.hardware_version.major = info_.hardware_major;
  response.hardware_version.minor = info_.hardware_minor;
  response.software_version.major = info_.software_major;
  response.software_version.minor = info_.software_minor;
  response.software_vcs_revision_id = info_.vcs_revision_id;
  std::copy(info_.unique_id.begin(), info_.unique_id.end(), std::begin(response.unique_id));
  const std::size_t kNameSize = std::min<std::size_t>(info_.name.size(), kNodeNameMaxLen);
  std::copy_n(info_.name.begin(), kNameSize, std::begin(response.name.elements));
  response.name.count = kNameSize;

  std::size_t size = tx_buffer_.size();
  if (uavcan_node_GetInfo_Response_1_0_serialize_(&response, OutputBytes(tx_buffer_), &size) >= 0) {
    Respond(request, size);
  }
}

bool CyphalNode::FindRegister(const std::string_view name,
                              RegisterProvider*& provider,
                              std::size_t& index) const noexcept {
  for (std::size_t provider_index = 0U; provider_index < provider_count_; ++provider_index) {
    RegisterProvider* const kProvider = providers_[provider_index];
    for (std::size_t i = 0U; i < kProvider->Count(); ++i) {
      if (kProvider->Name(i) == name) {
        provider = kProvider;
        index = i;
        return true;
      }
    }
  }
  return false;
}

void CyphalNode::HandleRegisterList(const Transfer& request) noexcept {
  uavcan_register_List_Request_1_0 list_request{};
  std::size_t request_size = request.payload.size();
  if (uavcan_register_List_Request_1_0_deserialize_(&list_request,
                                                    InputBytes(request.payload),
                                                    &request_size) < 0) {
    return;
  }
  uavcan_register_List_Response_1_0 response{};
  std::size_t index = list_request.index;
  for (std::size_t provider_index = 0U; provider_index < provider_count_; ++provider_index) {
    const std::size_t kCount = providers_[provider_index]->Count();
    if (index < kCount) {
      const std::string_view kName = providers_[provider_index]->Name(index);
      const std::size_t kSize =
          std::min<std::size_t>(kName.size(), uavcan_register_Name_1_0_name_ARRAY_CAPACITY_);
      std::copy_n(kName.begin(), kSize, std::begin(response.name.name.elements));
      response.name.name.count = kSize;
      break;
    }
    index -= kCount;
  }
  std::size_t size = tx_buffer_.size();
  if (uavcan_register_List_Response_1_0_serialize_(&response, OutputBytes(tx_buffer_), &size) >= 0) {
    Respond(request, size);
  }
}

void CyphalNode::HandleRegisterAccess(const Transfer& request) noexcept {
  uavcan_register_Access_Request_1_0 access{};
  std::size_t request_size = request.payload.size();
  if (uavcan_register_Access_Request_1_0_deserialize_(&access, InputBytes(request.payload), &request_size) <
      0) {
    return;
  }
  const std::string_view kName{reinterpret_cast<const char*>(access.name.name.elements),
                               access.name.name.count};

  uavcan_register_Access_Response_1_0 response{};
  RegisterProvider* provider = nullptr;
  std::size_t index = 0U;
  if (FindRegister(kName, provider, index)) {
    RegisterValue current{};
    const RegisterFlags kFlags = provider->Read(index, current);
    if (!IsEmpty(access.value) && kFlags.is_mutable) {
      (void)provider->Write(index, access.value);
    }
    (void)provider->Read(index, response.value);
    response._mutable = kFlags.is_mutable;
    response.persistent = kFlags.is_persistent;
  }
  std::size_t size = tx_buffer_.size();
  if (uavcan_register_Access_Response_1_0_serialize_(&response, OutputBytes(tx_buffer_), &size) >= 0) {
    Respond(request, size);
  }
}

void CyphalNode::HandleExecuteCommand(const Transfer& request) noexcept {
  uavcan_node_ExecuteCommand_Request_1_3 command{};
  std::size_t request_size = request.payload.size();
  if (uavcan_node_ExecuteCommand_Request_1_3_deserialize_(&command,
                                                          InputBytes(request.payload),
                                                          &request_size) < 0) {
    return;
  }
  uavcan_node_ExecuteCommand_Response_1_3 response{};
  response.status = cmd::kStatusBadCommand;
  if (command_handler_ != nullptr) {
    response.status =
        command_handler_->OnCommand(command.command,
                                    std::span<const uint8_t>{std::begin(command.parameter.elements),
                                                             command.parameter.count});
  }
  std::size_t size = tx_buffer_.size();
  if (uavcan_node_ExecuteCommand_Response_1_3_serialize_(&response, OutputBytes(tx_buffer_), &size) >= 0) {
    Respond(request, size);
  }
}

uint64_t CyphalNode::UniqueIdHash() const noexcept {
  // FNV-1a 64 over the unique-ID, truncated to 48 bits.
  uint64_t hash = 0xCBF29CE484222325ULL;
  for (const uint8_t kByte : info_.unique_id) {
    hash ^= kByte;
    hash *= 0x100000001B3ULL;
  }
  return hash & kUniqueIdHashMask;
}

void CyphalNode::RequestNodeId(const uint64_t now_us) noexcept {
  uavcan_pnp_NodeIDAllocationData_1_0 request{};
  request.unique_id_hash = UniqueIdHash();
  request.allocated_node_id.count = 0U;
  std::size_t size = tx_buffer_.size();
  if (uavcan_pnp_NodeIDAllocationData_1_0_serialize_(&request, OutputBytes(tx_buffer_), &size) >= 0) {
    (void)PublishInternal(allocation_, size);
  }
  // Next request after 0.5 to 1.5 s (linear congruential sequence from the hash).
  allocation_seed_ = (allocation_seed_ * 1'664'525U) + 1'013'904'223U;
  next_allocation_us_ = now_us + 500'000U + (allocation_seed_ % 1'000'000U);
}

void CyphalNode::HandleAllocation(const Transfer& message) noexcept {
  if (NodeId() != kNodeIdUnset || message.source_node_id == kNodeIdUnset) {
    return;  // already allocated, or the anonymous request of another node
  }
  uavcan_pnp_NodeIDAllocationData_1_0 response{};
  std::size_t size = message.payload.size();
  if (uavcan_pnp_NodeIDAllocationData_1_0_deserialize_(&response, InputBytes(message.payload), &size) < 0) {
    return;
  }
  if (response.allocated_node_id.count != 1U || response.unique_id_hash != UniqueIdHash() ||
      response.allocated_node_id.elements[0].value > kNodeIdMax) {
    return;
  }
  const auto kNodeId = static_cast<uint8_t>(response.allocated_node_id.elements[0].value);
  transport_->SetLocalNodeId(kNodeId);
  next_heartbeat_us_ = 0U;
  if (node_id_listener_ != nullptr) {
    node_id_listener_->OnNodeIdAllocated(kNodeId);
  }
}

uint32_t CyphalNode::UptimeSeconds(const uint64_t now_us) const noexcept {
  return static_cast<uint32_t>((now_us - start_us_) / 1'000'000U);
}

void CyphalNode::PublishHeartbeat(const uint64_t now_us) noexcept {
  uavcan_node_Heartbeat_1_0 heartbeat{};
  heartbeat.uptime = UptimeSeconds(now_us);
  heartbeat.health.value = static_cast<uint8_t>(health_);
  heartbeat.mode.value = static_cast<uint8_t>(mode_);
  heartbeat.vendor_specific_status_code = vendor_status_;
  std::size_t size = tx_buffer_.size();
  if (uavcan_node_Heartbeat_1_0_serialize_(&heartbeat, OutputBytes(tx_buffer_), &size) >= 0) {
    (void)PublishInternal(heartbeat_, size);
  }
}

void CyphalNode::PublishPortList() noexcept {
  std::array<uint16_t, kMaxPublishers + 3U> publishers{};
  std::size_t publisher_count = 0U;
  publishers[publisher_count++] = port::kHeartbeat;
  publishers[publisher_count++] = port::kPortList;
  publishers[publisher_count++] = port::kDiagnosticRecord;
  for (std::size_t i = 0U; i < published_count_; ++i) {
    publishers[publisher_count++] = published_subjects_[i];
  }
  std::array<uint16_t, kMaxListeners> subscribers{};
  for (std::size_t i = 0U; i < listener_count_; ++i) {
    subscribers[i] = listeners_[i].subject_id;
  }

  Writer writer{tx_buffer_};
  WriteSubjectList(writer, std::span<const uint16_t>{publishers.data(), publisher_count});
  WriteSubjectList(writer, std::span<const uint16_t>{subscribers.data(), listener_count_});
  WriteServiceList(writer, {});
  WriteServiceList(writer, kServedServices);
  if (writer.Fits()) {
    (void)PublishInternal(port_list_, writer.Size());
  }
}

bool CyphalNode::Publish(Publisher& publisher, const std::span<const std::byte> payload) noexcept {
  if (publisher.port_id == kUnsetPortId || NodeId() == kNodeIdUnset) {
    return false;
  }
  const Transfer kTransfer{
      .port_id = publisher.port_id,
      .source_node_id = NodeId(),
      .destination_node_id = kNodeIdUnset,
      .transfer_id = publisher.next_transfer_id,
      .kind = TransferKind::kMessage,
      .priority = publisher.priority,
      .payload = payload,
  };
  publisher.next_transfer_id = static_cast<uint8_t>((publisher.next_transfer_id + 1U) & 0x1FU);
  return transport_->Send(kTransfer);
}

bool CyphalNode::PublishInternal(Publisher& publisher, const std::size_t size) noexcept {
  const Transfer kTransfer{
      .port_id = publisher.port_id,
      .source_node_id = NodeId(),
      .destination_node_id = kNodeIdUnset,
      .transfer_id = publisher.next_transfer_id,
      .kind = TransferKind::kMessage,
      .priority = publisher.priority,
      .payload = std::span<const std::byte>{tx_buffer_.data(), size},
  };
  publisher.next_transfer_id = static_cast<uint8_t>((publisher.next_transfer_id + 1U) & 0x1FU);
  return transport_->Send(kTransfer);
}

bool CyphalNode::Log(const Severity severity, const std::string_view text) noexcept {
  if (NodeId() == kNodeIdUnset) {
    return false;
  }
  uavcan_diagnostic_Record_1_1 record{};
  record.severity.value = static_cast<uint8_t>(severity);
  const std::size_t kSize =
      std::min<std::size_t>(text.size(), uavcan_diagnostic_Record_1_1_text_ARRAY_CAPACITY_);
  std::copy_n(text.begin(), kSize, std::begin(record.text.elements));
  record.text.count = kSize;
  std::size_t size = tx_buffer_.size();
  if (uavcan_diagnostic_Record_1_1_serialize_(&record, OutputBytes(tx_buffer_), &size) < 0) {
    return false;
  }
  return PublishInternal(diagnostic_, size);
}

}  // namespace unimoc::cyphal
