/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file node.hpp
 *    @brief Transport-injected Cyphal application node shell.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <span>
#include "cyphal/transport.hpp"

namespace unimoc::cyphal {

/**
 * @brief Cyphal application shell with an injected transport.
 *
 * This class deliberately stops at the transfer boundary.  Register,
 * subject, and service handlers can be layered on top once the DSDL stack is
 * selected, while UDP, CAN, and unit-test transports remain interchangeable.
 */
class CyphalNode {
 public:
  /**
   * @brief Binds the node to a caller-owned transport.
   * @param transport Transport used for all send and receive operations. It
   *                  must outlive this node.
   */
  explicit CyphalNode(Transport& transport) noexcept : transport_{&transport} {}

  /**
   * @brief Sends one transfer through the injected transport.
   * @param transfer Transfer metadata and payload view to send.
   * @return `true` when the transport accepts the transfer.
   */
  [[nodiscard]] bool Send(const Transfer& transfer) noexcept { return transport_->Send(transfer); }

  /**
   * @brief Polls the injected transport for one transfer.
   * @param payload_buffer Caller-owned storage for the received payload.
   * @param transfer Output transfer whose payload points into `payload_buffer`
   *                 when a transfer is received.
   * @return The transport receive result.
   */
  [[nodiscard]] ReceiveResult Poll(std::span<std::byte> payload_buffer, Transfer& transfer) noexcept {
    return transport_->Receive(payload_buffer, transfer);
  }

 private:
  Transport* transport_;
};

}  // namespace unimoc::cyphal