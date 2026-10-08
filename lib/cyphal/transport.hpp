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
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace unimoc::cyphal {

/// Identifies the kind of Cyphal transfer carried by a transport frame.
enum class TransferKind : uint8_t {
  /// Broadcast or point-to-point subject message.
  kMessage,
  /// Request sent to a Cyphal service server.
  kServiceRequest,
  /// Response sent by a Cyphal service server.
  kServiceResponse,
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
  /// Node-ID of the sender.
  uint8_t source_node_id{0U};
  /// Node-ID of the recipient, or 255 for a broadcast message.
  uint8_t destination_node_id{255U};
  /// Cyphal transfer-ID assigned by the application.
  uint8_t transfer_id{0U};
  /// Transfer category used by the transport adapter.
  TransferKind kind{TransferKind::kMessage};
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
 * Implementations own the UDP socket, CAN controller, or test queue.  The
 * Cyphal layer only exchanges transfer metadata and serialized payloads, so
 * the same node can be used with different transports or a fake in tests.
 * `Receive()` must point `transfer.payload` into `payload_buffer` when it
 * returns `kReceived`.
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
   * @param transfer Transfer metadata and non-owning payload view.
   * @return `true` when the transport accepted the transfer.
   */
  [[nodiscard]] virtual bool Send(const Transfer& transfer) noexcept = 0;

  /**
   * @brief Polls for one transfer and writes its payload into a caller buffer.
   * @param payload_buffer Storage owned by the caller for the received payload.
   * @param transfer Output metadata; on success its payload points into
   *                 `payload_buffer`.
   * @return The receive outcome, including an empty-poll result.
   */
  [[nodiscard]] virtual ReceiveResult Receive(std::span<std::byte> payload_buffer,
                                              Transfer& transfer) noexcept = 0;
};

}  // namespace unimoc::cyphal