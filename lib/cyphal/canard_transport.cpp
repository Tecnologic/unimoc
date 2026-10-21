/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file canard_transport.cpp
 *    @brief Cyphal/CAN transport on libcanard 4.x with an O1Heap arena.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "cyphal/canard_transport.hpp"
#include <algorithm>
#include <cstring>

namespace unimoc::cyphal {

namespace {

[[nodiscard]] constexpr CanardTransferKind ToCanard(const TransferKind kind) noexcept {
  switch (kind) {
    case TransferKind::kMessage:
      return CanardTransferKindMessage;
    case TransferKind::kServiceRequest:
      return CanardTransferKindRequest;
    case TransferKind::kServiceResponse:
      return CanardTransferKindResponse;
  }
  return CanardTransferKindMessage;
}

[[nodiscard]] constexpr TransferKind FromCanard(const CanardTransferKind kind) noexcept {
  switch (kind) {
    case CanardTransferKindMessage:
      return TransferKind::kMessage;
    case CanardTransferKindRequest:
      return TransferKind::kServiceRequest;
    case CanardTransferKindResponse:
      return TransferKind::kServiceResponse;
  }
  return TransferKind::kMessage;
}

}  // namespace

CanardTransport::CanardTransport(const std::span<std::byte> arena,
                                 const std::span<CanMedia* const> interfaces) noexcept
    : heap_{o1heapInit(arena.data(), arena.size())} {
  const CanardMemoryResource kMemory{this, Deallocate, Allocate};
  canard_ = canardInit(kMemory);
  canard_.node_id = CANARD_NODE_ID_UNSET;
  for (CanMedia* media : interfaces) {
    if (media == nullptr || interface_count_ >= kMaxInterfaces) {
      continue;
    }
    interfaces_[interface_count_] = media;
    tx_queues_[interface_count_] = canardTxInit(kTxQueueCapacity, media->Mtu(), kMemory);
    ++interface_count_;
  }
}

void* CanardTransport::Allocate(void* const user_reference, const std::size_t size) {
  auto* self = static_cast<CanardTransport*>(user_reference);
  void* const kPointer = o1heapAllocate(self->heap_, size);
  if (kPointer == nullptr) {
    ++self->out_of_memory_;
  }
  return kPointer;
}

void CanardTransport::Deallocate(void* const user_reference,
                                 const std::size_t /*size*/,
                                 void* const pointer) {
  auto* self = static_cast<CanardTransport*>(user_reference);
  o1heapFree(self->heap_, pointer);
}

bool CanardTransport::Send(const Transfer& transfer) noexcept {
  if (!IsValid()) {
    return false;
  }
  const CanardTransferMetadata kMetadata{
      .priority = static_cast<CanardPriority>(transfer.priority),
      .transfer_kind = ToCanard(transfer.kind),
      .port_id = transfer.port_id,
      .remote_node_id = static_cast<CanardNodeID>(
          (transfer.kind == TransferKind::kMessage) ? CANARD_NODE_ID_UNSET : transfer.destination_node_id),
      .transfer_id = transfer.transfer_id,
  };
  const CanardPayload kPayload{transfer.payload.size(), transfer.payload.data()};
  bool accepted = false;
  for (std::size_t i = 0U; i < interface_count_; ++i) {
    uint64_t expired = 0U;
    const int32_t kResult = canardTxPush(&tx_queues_[i],
                                         &canard_,
                                         now_us_ + kDefaultTxTimeoutUs,
                                         &kMetadata,
                                         kPayload,
                                         now_us_,
                                         &expired);
    tx_dropped_ += expired;
    if (kResult > 0) {
      accepted = true;
    } else if (kResult == -CANARD_ERROR_OUT_OF_MEMORY) {
      ++tx_dropped_;
    }
  }
  return accepted;
}

ReceiveResult CanardTransport::Receive(const std::span<std::byte> payload_buffer,
                                       Transfer& transfer) noexcept {
  if (!IsValid()) {
    return ReceiveResult::kError;
  }
  CanFrame frame{};
  std::size_t idle_interfaces = 0U;
  for (std::size_t attempt = 0U; attempt < kMaxFramesPerReceive && idle_interfaces < interface_count_;
       ++attempt) {
    const std::size_t kIface = next_rx_interface_;
    next_rx_interface_ = (next_rx_interface_ + 1U) % interface_count_;
    if (!interfaces_[kIface]->Pop(frame)) {
      ++idle_interfaces;
      continue;
    }
    idle_interfaces = 0U;

    const CanardFrame kCanardFrame{frame.extended_can_id, {frame.size, frame.data.data()}};
    CanardRxTransfer received{};
    CanardRxSubscription* subscription = nullptr;
    const int8_t kResult = canardRxAccept(&canard_,
                                          now_us_,
                                          &kCanardFrame,
                                          static_cast<uint8_t>(kIface),
                                          &received,
                                          &subscription);
    if (kResult == -CANARD_ERROR_OUT_OF_MEMORY) {
      ++out_of_memory_;
      continue;
    }
    if (kResult != 1) {
      continue;  // not complete yet, a duplicate, or not subscribed
    }

    const std::size_t kSize = std::min(received.payload.size, payload_buffer.size());
    if (kSize > 0U) {
      std::memcpy(payload_buffer.data(), received.payload.data, kSize);
    }
    canard_.memory.deallocate(canard_.memory.user_reference,
                              received.payload.allocated_size,
                              received.payload.data);

    transfer.port_id = received.metadata.port_id;
    transfer.kind = FromCanard(received.metadata.transfer_kind);
    transfer.priority = static_cast<Priority>(received.metadata.priority);
    transfer.transfer_id = received.metadata.transfer_id;
    transfer.source_node_id = received.metadata.remote_node_id;
    transfer.destination_node_id = (transfer.kind == TransferKind::kMessage) ? kNodeIdUnset : LocalNodeId();
    transfer.payload = payload_buffer.first(kSize);
    return ReceiveResult::kReceived;
  }
  return ReceiveResult::kNoTransfer;
}

bool CanardTransport::Subscribe(const TransferKind kind,
                                const uint16_t port_id,
                                const std::size_t extent) noexcept {
  if (subscription_count_ >= kMaxSubscriptions) {
    return false;
  }
  const int8_t kResult = canardRxSubscribe(&canard_,
                                           ToCanard(kind),
                                           port_id,
                                           extent,
                                           kTransferIdTimeoutUs,
                                           &subscriptions_[subscription_count_]);
  if (kResult < 0) {
    return false;
  }
  if (kResult > 0) {
    ++subscription_count_;  // a new subscription; 0 replaced an existing one in place
  }
  return true;
}

void CanardTransport::SetLocalNodeId(const uint8_t node_id) noexcept {
  canard_.node_id = (node_id <= kNodeIdMax) ? node_id : CANARD_NODE_ID_UNSET;
}

uint8_t CanardTransport::LocalNodeId() const noexcept {
  return (canard_.node_id <= kNodeIdMax) ? canard_.node_id : kNodeIdUnset;
}

void CanardTransport::Process(const uint64_t now_us) noexcept {
  now_us_ = now_us;
  for (std::size_t i = 0U; i < interface_count_; ++i) {
    CanardTxQueue& queue = tx_queues_[i];
    CanMedia& media = *interfaces_[i];
    for (CanardTxQueueItem* item = canardTxPeek(&queue); item != nullptr; item = canardTxPeek(&queue)) {
      if (item->tx_deadline_usec < now_us) {
        ++tx_dropped_;
      } else {
        CanFrame frame{};
        frame.extended_can_id = item->frame.extended_can_id;
        frame.size = static_cast<uint8_t>(std::min(item->frame.payload.size, frame.data.size()));
        std::memcpy(frame.data.data(), item->frame.payload.data, frame.size);
        if (!media.Push(frame)) {
          break;  // the interface is busy; retry with the next Process()
        }
      }
      canardTxFree(&queue, &canard_, canardTxPop(&queue, item));
    }
  }
}

}  // namespace unimoc::cyphal
