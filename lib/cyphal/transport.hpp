/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file transport.hpp
 *    @brief Transport-independent Cyphal transfer contract.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace unimoc::cyphal {

/// Node-ID value of an anonymous sender or a broadcast destination.
inline constexpr uint8_t kNodeIdUnset = 255U;

/// Largest valid Cyphal/CAN node-ID.
inline constexpr uint8_t kNodeIdMax = 127U;

/// Identifies the kind of Cyphal transfer carried by a transport frame.
enum class TransferKind : uint8_t {
  /// Broadcast or point-to-point subject message.
  kMessage,
  /// Request sent to a Cyphal service server.
  kServiceRequest,
  /// Response sent by a Cyphal service server.
  kServiceResponse,
};

/// Cyphal transfer priority; a lower value wins the bus arbitration.
enum class Priority : uint8_t {
  kExceptional = 0,
  kImmediate = 1,
  kFast = 2,
  kHigh = 3,
  kNominal = 4,
  kLow = 5,
  kSlow = 6,
  kOptional = 7,
};

/**
 * @brief Metadata and payload for one transport-independent Cyphal transfer.
 *
 * The payload is a non-owning view. Its storage must remain valid for the
 * duration of the send operation or until the received transfer is consumed.
 */
struct Transfer {
  /// Subject or service port-ID carried by the transfer.
  uint16_t port_id{0U};
  /// Node-ID of the sender, kNodeIdUnset for an anonymous message.
  uint8_t source_node_id{kNodeIdUnset};
  /// Node-ID of the recipient, or kNodeIdUnset for a broadcast message.
  uint8_t destination_node_id{kNodeIdUnset};
  /// Cyphal transfer-ID assigned by the application.
  uint8_t transfer_id{0U};
  /// Transfer category used by the transport adapter.
  TransferKind kind{TransferKind::kMessage};
  /// Arbitration priority; a response uses the priority of its request.
  Priority priority{Priority::kNominal};
  /// Serialized DSDL payload; this view does not own its storage.
  std::span<const std::byte> payload;
};

/// Result of polling an injected transport for one transfer.
enum class ReceiveResult : uint8_t {
  /// No transfer was available before the transport poll returned.
  kNoTransfer,
  /// A transfer was written to the caller-provided buffer.
  kReceived,
  /// The transport could not produce a valid transfer.
  kError,
};

/**
 * @brief Transport boundary used by the Cyphal application layer.
 *
 * Implementations own the CAN controller, the UDP socket or the test queue.
 * The Cyphal layer only exchanges transfer metadata and serialized payloads,
 * so the same node runs on Cyphal/CAN, a later Cyphal/UDP or a fake in
 * tests. All methods are called from one thread.
 */
class Transport {
 public:
  Transport() = default;
  virtual ~Transport() = default;

  Transport(const Transport&) = delete;
  Transport& operator=(const Transport&) = delete;
  Transport(Transport&&) = delete;
  Transport& operator=(Transport&&) = delete;

  /**
   * @brief Sends one serialized transfer through the concrete transport.
   * @param transfer Transfer metadata and non-owning payload view. The source
   *                 node-ID is the local one (SetLocalNodeId()).
   * @return `true` when the transport accepted the transfer.
   */
  [[nodiscard]] virtual bool Send(const Transfer& transfer) noexcept = 0;

  /**
   * @brief Polls for one transfer and writes its payload into a caller buffer.
   *
   * Only transfers of a subscribed port arrive (Subscribe()). A payload longer
   * than the buffer is truncated, which DSDL deserialization accepts (implicit
   * zero extension and truncation).
   *
   * @param payload_buffer Storage owned by the caller for the received payload.
   * @param transfer Output metadata; on success its payload points into
   *                 `payload_buffer`.
   * @return The receive outcome, including an empty-poll result.
   */
  [[nodiscard]] virtual ReceiveResult Receive(std::span<std::byte> payload_buffer,
                                              Transfer& transfer) noexcept = 0;

  /**
   * @brief Accepts transfers of one port.
   * @param kind Message, or service request (server) or response (client).
   * @param port_id Subject-ID or service-ID.
   * @param extent Largest payload kept of a transfer (the DSDL extent).
   * @return `false` when no subscription slot is left.
   */
  [[nodiscard]] virtual bool Subscribe(TransferKind kind, uint16_t port_id, std::size_t extent) noexcept = 0;

  /**
   * @brief Sets the node-ID used as the source of sent transfers.
   * @param node_id Node-ID 0..kNodeIdMax, or kNodeIdUnset for an anonymous node.
   */
  virtual void SetLocalNodeId(uint8_t node_id) noexcept = 0;

  /// Node-ID of this node, or kNodeIdUnset while it is anonymous.
  [[nodiscard]] virtual uint8_t LocalNodeId() const noexcept = 0;

  /**
   * @brief Moves queued frames to the bus and advances the transport clock.
   * @param now_us Monotonic time [us]; deadlines and timeouts use it.
   */
  virtual void Process(uint64_t now_us) noexcept = 0;
};

}  // namespace unimoc::cyphal
