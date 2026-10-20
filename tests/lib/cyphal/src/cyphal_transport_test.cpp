/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file cyphal_transport_test.cpp
 *    @brief ztest coverage of the injected Cyphal transport boundary.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <zephyr/ztest.h>
#include <array>
#include <cstddef>
#include <cstring>
#include <span>
#include "cyphal/node.hpp"
#include "unimoc_test.hpp"

namespace unimoc::cyphal::test {
namespace {

class FakeTransport final : public Transport {
 public:
  bool Send(const Transfer& transfer) noexcept override {
    sent_transfer = transfer;
    return send_result;
  }

  ReceiveResult Receive(std::span<std::byte> payload_buffer, Transfer& transfer) noexcept override {
    if (receive_result != ReceiveResult::kReceived) {
      return receive_result;
    }
    if (payload.size() > payload_buffer.size()) {
      return ReceiveResult::kError;
    }
    std::memcpy(payload_buffer.data(), payload.data(), payload.size());
    transfer = received_transfer;
    transfer.payload = payload_buffer.first(payload.size());
    return ReceiveResult::kReceived;
  }

  bool send_result{true};
  ReceiveResult receive_result{ReceiveResult::kNoTransfer};
  Transfer sent_transfer{};
  Transfer received_transfer{};
  std::span<const std::byte> payload;
};

ZTEST(cyphal_node, test_sends_through_injected_transport) {
  FakeTransport fake;
  CyphalNode node{fake};
  const std::array<std::byte, 2U> kPayload{std::byte{0x12}, std::byte{0x34}};
  const Transfer kTransfer{
      .port_id = 42U,
      .source_node_id = 10U,
      .destination_node_id = 20U,
      .transfer_id = 3U,
      .kind = TransferKind::kServiceRequest,
      .payload = kPayload,
  };

  zassert_true(node.Send(kTransfer));
  zexpect_equal(fake.sent_transfer.port_id, 42U);
  zexpect_equal(fake.sent_transfer.source_node_id, 10U);
  zexpect_equal(fake.sent_transfer.destination_node_id, 20U);
  zexpect_equal(fake.sent_transfer.kind, TransferKind::kServiceRequest);
  zassert_equal(fake.sent_transfer.payload.size(), kPayload.size());
  zexpect_equal(std::memcmp(fake.sent_transfer.payload.data(), kPayload.data(), kPayload.size()), 0);
}

ZTEST(cyphal_node, test_polls_into_caller_owned_buffer) {
  FakeTransport fake;
  CyphalNode node{fake};
  const std::array<std::byte, 3U> kInput{std::byte{0x01}, std::byte{0x02}, std::byte{0x03}};
  fake.receive_result = ReceiveResult::kReceived;
  fake.received_transfer = Transfer{
      .port_id = 100U,
      .source_node_id = 7U,
      .destination_node_id = 255U,
      .transfer_id = 9U,
      .kind = TransferKind::kMessage,
      .payload = {},
  };
  fake.payload = kInput;
  std::array<std::byte, 8U> k_buffer{};
  Transfer received{};

  zexpect_equal(node.Poll(k_buffer, received), ReceiveResult::kReceived);
  zexpect_equal(received.port_id, 100U);
  zexpect_equal(received.source_node_id, 7U);
  zassert_equal(received.payload.size(), kInput.size());
  zexpect_equal(std::memcmp(received.payload.data(), kInput.data(), kInput.size()), 0);
  zexpect_equal(std::memcmp(k_buffer.data(), kInput.data(), kInput.size()), 0);
}

}  // namespace
}  // namespace unimoc::cyphal::test

ZTEST_SUITE(cyphal_node, nullptr, nullptr, nullptr, nullptr, nullptr);
