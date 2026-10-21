/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file canard_transport.hpp
 *    @brief Cyphal/CAN transport on libcanard 4.x with an O1Heap arena.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <canard.h>
#include <o1heap.h>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include "cyphal/can_media.hpp"
#include "cyphal/transport.hpp"

namespace unimoc::cyphal {

/// Alignment the memory arena of CanardTransport needs (O1Heap).
inline constexpr std::size_t kCanardArenaAlignment = O1HEAP_ALIGNMENT;

/**
 * @brief Memory arena for CanardTransport, aligned for O1Heap.
 * @tparam Size Arena size in bytes. libcanard allocates the TX queue items,
 *              the RX sessions (one per remote node and port) and the payload
 *              of multi-frame transfers from it.
 */
template <std::size_t Size>
struct CanardArena {
  alignas(kCanardArenaAlignment) std::array<std::byte, Size> bytes{};
};

/**
 * @brief Cyphal/CAN transport over one or two (redundant) CAN interfaces.
 *
 * Every interface has its own TX queue with the MTU of the interface, so a
 * CAN FD interface and a classic CAN interface can be used side by side; the
 * RX path removes the duplicates of redundant interfaces (libcanard). All
 * calls come from the Cyphal thread; nothing here is reachable from the
 * zero-latency hot path.
 */
class CanardTransport final : public Transport {
 public:
  /// Largest number of CAN interfaces of one node.
  static constexpr std::size_t kMaxInterfaces = 2U;
  /// Largest number of subscribed ports (messages, requests and responses).
  static constexpr std::size_t kMaxSubscriptions = 16U;
  /// Frames held per TX queue before Send() refuses a transfer.
  static constexpr std::size_t kTxQueueCapacity = 64U;
  /// Default time a frame may wait in the TX queue [us].
  static constexpr uint64_t kDefaultTxTimeoutUs = 100'000U;
  /// Transfer-ID timeout of the subscriptions [us] (Cyphal default 2 s).
  static constexpr uint64_t kTransferIdTimeoutUs = 2'000'000U;
  /// Largest number of frames Receive() looks at per call.
  static constexpr std::size_t kMaxFramesPerReceive = 64U;

  /**
   * @brief Creates the transport.
   * @param arena Memory for libcanard, aligned at kCanardArenaAlignment (use
   *              CanardArena). It must outlive the transport.
   * @param interfaces CAN interfaces; the first kMaxInterfaces non-null
   *                   entries are used. They must outlive the transport.
   */
  CanardTransport(std::span<std::byte> arena, std::span<CanMedia* const> interfaces) noexcept;

  /// False when the arena could not be used (alignment, size) or no interface was given.
  [[nodiscard]] bool IsValid() const noexcept { return heap_ != nullptr && interface_count_ > 0U; }

  [[nodiscard]] bool Send(const Transfer& transfer) noexcept override;
  [[nodiscard]] ReceiveResult Receive(std::span<std::byte> payload_buffer,
                                      Transfer& transfer) noexcept override;
  [[nodiscard]] bool Subscribe(TransferKind kind, uint16_t port_id, std::size_t extent) noexcept override;
  void SetLocalNodeId(uint8_t node_id) noexcept override;
  [[nodiscard]] uint8_t LocalNodeId() const noexcept override;
  void Process(uint64_t now_us) noexcept override;

  /// Number of CAN interfaces in use.
  [[nodiscard]] std::size_t InterfaceCount() const noexcept { return interface_count_; }

  /// Frames dropped because their deadline passed or the queue was full.
  [[nodiscard]] uint64_t TxDropped() const noexcept { return tx_dropped_; }

  /// Frames or transfers lost to a full arena.
  [[nodiscard]] uint64_t OutOfMemory() const noexcept { return out_of_memory_; }

 private:
  static void* Allocate(void* user_reference, std::size_t size);
  static void Deallocate(void* user_reference, std::size_t size, void* pointer);

  O1HeapInstance* heap_{nullptr};
  CanardInstance canard_{};
  std::array<CanMedia*, kMaxInterfaces> interfaces_{};
  std::array<CanardTxQueue, kMaxInterfaces> tx_queues_{};
  std::size_t interface_count_{0U};
  std::array<CanardRxSubscription, kMaxSubscriptions> subscriptions_{};
  std::size_t subscription_count_{0U};
  std::size_t next_rx_interface_{0U};
  uint64_t now_us_{0U};
  uint64_t tx_dropped_{0U};
  uint64_t out_of_memory_{0U};
};

}  // namespace unimoc::cyphal
