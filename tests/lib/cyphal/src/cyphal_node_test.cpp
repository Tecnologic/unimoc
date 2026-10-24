/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file cyphal_node_test.cpp
 *    @brief Tests of the libcanard transport and the Cyphal node services
 *           against a second libcanard instance on an in-memory CAN bus.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <uavcan/_register/Access_1_0.h>
#include <uavcan/_register/List_1_0.h>
#include <uavcan/diagnostic/Record_1_1.h>
#include <uavcan/node/ExecuteCommand_1_3.h>
#include <uavcan/node/GetInfo_1_0.h>
#include <uavcan/node/Heartbeat_1_0.h>
#include <uavcan/node/port/List_1_0.h>
#include <uavcan/pnp/NodeIDAllocationData_1_0.h>
#include <zephyr/ztest.h>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstring>
#include <new>
#include <optional>
#include <string_view>
#include "can_test_bus.hpp"
#include "cyphal/canard_transport.hpp"
#include "cyphal/node.hpp"
#include "cyphal/register_value.hpp"
#include "unimoc_test.hpp"

namespace unimoc::cyphal::test {
namespace {

using unimoc::test::CanTestBus;
using unimoc::test::CanTestMedia;

constexpr uint8_t kServerId = 42U;
constexpr uint8_t kClientId = 10U;

uint8_t* Out(std::span<std::byte> buffer) { return reinterpret_cast<uint8_t*>(buffer.data()); }
const uint8_t* In(std::span<const std::byte> buffer) {
  return reinterpret_cast<const uint8_t*>(buffer.data());
}

/// Two registers: a writable real32 and a read-only string.
class TestRegisters final : public RegisterProvider {
 public:
  [[nodiscard]] std::size_t Count() const noexcept override { return 2U; }
  [[nodiscard]] std::string_view Name(const std::size_t index) const noexcept override {
    return index == 0U ? std::string_view{"test.gain"} : std::string_view{"test.label"};
  }
  [[nodiscard]] RegisterFlags Read(const std::size_t index, RegisterValue& value) const noexcept override {
    if (index == 0U) {
      SetReal32(value, gain);
      return {true, true};
    }
    SetString(value, "fixed");
    return {false, false};
  }
  [[nodiscard]] RegisterWriteResult Write(const std::size_t index,
                                          const RegisterValue& value) noexcept override {
    const auto kNumber = GetNumber(value);
    if (index != 0U || !kNumber.has_value()) {
      return RegisterWriteResult::kTypeMismatch;
    }
    if (*kNumber < 0.0F) {
      return RegisterWriteResult::kOutOfRange;
    }
    gain = *kNumber;
    return RegisterWriteResult::kAccepted;
  }

  float gain{1.5F};
};

class TestCommands final : public CommandHandler {
 public:
  [[nodiscard]] uint8_t OnCommand(const uint16_t command,
                                  std::span<const uint8_t> parameter) noexcept override {
    last_command = command;
    last_parameter_size = parameter.size();
    return command == 7U ? cmd::kStatusSuccess : cmd::kStatusBadCommand;
  }
  uint16_t last_command{0U};
  std::size_t last_parameter_size{0U};
};

class TestNodeIdListener final : public NodeIdListener {
 public:
  void OnNodeIdAllocated(const uint8_t node_id) noexcept override { allocated = node_id; }
  uint8_t allocated{kNodeIdUnset};
};

class TestMessages final : public MessageListener {
 public:
  void OnMessage(const Transfer& transfer, uint64_t /*now_us*/) noexcept override {
    ++count;
    last_source = transfer.source_node_id;
    last_size = transfer.payload.size();
  }
  std::size_t count{0U};
  uint8_t last_source{0U};
  std::size_t last_size{0U};
};

/// Answers with the request reversed; an empty request gets no response.
class TestService final : public ServiceHandler {
 public:
  [[nodiscard]] std::optional<std::size_t> OnRequest(const Transfer& request,
                                                     std::span<std::byte> response) noexcept override {
    ++count;
    last_port = request.port_id;
    if (request.payload.empty() || request.payload.size() > response.size()) {
      return std::nullopt;
    }
    std::reverse_copy(request.payload.begin(), request.payload.end(), response.begin());
    return request.payload.size();
  }
  std::size_t count{0U};
  uint16_t last_port{kUnsetPortId};
};

/// A node under test and a client on the same bus, one or two interfaces each.
struct Rig {
  explicit Rig(const std::size_t mtu, const bool redundant = false)
      : server_media_a{bus_a, mtu},
        server_media_b{bus_b, mtu},
        client_media_a{bus_a, mtu},
        client_media_b{bus_b, mtu},
        server_interfaces{&server_media_a, redundant ? &server_media_b : nullptr},
        client_interfaces{&client_media_a, redundant ? &client_media_b : nullptr},
        server_transport{server_arena.bytes, server_interfaces},
        client{client_arena.bytes, client_interfaces},
        node{server_transport} {
    client.SetLocalNodeId(kClientId);
    client.Process(now);  // the transport clock sets the deadline of the first transfer
    NodeInfo info{};
    info.name = "org.unimoc.test";
    info.software_major = 3U;
    info.software_minor = 4U;
    info.vcs_revision_id = 0x1234ABCDULL;
    for (std::size_t i = 0U; i < info.unique_id.size(); ++i) {
      info.unique_id[i] = static_cast<uint8_t>(i + 1U);
    }
    node.SetInfo(info);
  }

  /// Advances both sides by @p step_us and moves the frames.
  void Run(const uint64_t step_us = 1000U, const unsigned rounds = 3U) {
    for (unsigned i = 0U; i < rounds; ++i) {
      now += step_us;
      node.Spin(now);
      client.Process(now);
    }
  }

  bool Request(const uint16_t service, std::span<const std::byte> payload) {
    const Transfer kTransfer{
        .port_id = service,
        .source_node_id = kClientId,
        .destination_node_id = kServerId,
        .transfer_id = next_transfer_id++,
        .kind = TransferKind::kServiceRequest,
        .priority = Priority::kNominal,
        .payload = payload,
    };
    return client.Send(kTransfer);
  }

  /// Waits for one transfer of @p port at the client.
  bool Await(const uint16_t port, Transfer& transfer) {
    for (unsigned i = 0U; i < 20U; ++i) {
      Run(1000U, 1U);
      while (client.Receive(rx, transfer) == ReceiveResult::kReceived) {
        if (transfer.port_id == port) {
          return true;
        }
      }
    }
    return false;
  }

  CanardArena<32768> server_arena;
  CanardArena<32768> client_arena;
  CanTestBus bus_a;
  CanTestBus bus_b;
  CanTestMedia server_media_a;
  CanTestMedia server_media_b;
  CanTestMedia client_media_a;
  CanTestMedia client_media_b;
  std::array<CanMedia*, 2> server_interfaces;
  std::array<CanMedia*, 2> client_interfaces;
  CanardTransport server_transport;
  CanardTransport client;
  CyphalNode node;
  std::array<std::byte, 1024> rx{};
  std::array<std::byte, 600> tx{};
  uint64_t now{1'000'000U};
  uint8_t next_transfer_id{0U};
};

/// The rig is large: keep it off the ztest stack.
Rig& MakeRig(const std::size_t mtu, const bool redundant = false) {
  alignas(Rig) static std::array<std::byte, sizeof(Rig)> storage;
  static Rig* rig = nullptr;
  if (rig != nullptr) {
    rig->~Rig();
  }
  // NOLINTNEXTLINE(cppcoreguidelines-owning-memory): placement new into static storage.
  rig = new (storage.data()) Rig{mtu, redundant};
  return *rig;
}

ZTEST(cyphal_stack, test_arena_and_interfaces_make_a_valid_transport) {
  Rig& rig = MakeRig(kCanFdMtu);
  zexpect_true(rig.server_transport.IsValid());
  zexpect_equal(rig.server_transport.InterfaceCount(), 1U);
  std::array<std::byte, 64> misaligned{};
  const std::array<CanMedia*, 1> kNone{nullptr};
  CanardTransport invalid{std::span<std::byte>{misaligned}.subspan(1U), kNone};
  zexpect_false(invalid.IsValid());
}

ZTEST(cyphal_stack, test_heartbeat_once_per_second_after_start) {
  Rig& rig = MakeRig(kCanFdMtu);
  zassert_true(rig.client.Subscribe(TransferKind::kMessage, port::kHeartbeat, 12U));
  zassert_true(rig.node.Start(kServerId, rig.now));
  rig.node.SetMode(Mode::kOperational);
  rig.node.SetHealth(Health::kAdvisory);

  Transfer transfer{};
  zassert_true(rig.Await(port::kHeartbeat, transfer));
  zexpect_equal(transfer.source_node_id, kServerId);
  uavcan_node_Heartbeat_1_0 heartbeat{};
  std::size_t size = transfer.payload.size();
  zassert_equal(uavcan_node_Heartbeat_1_0_deserialize_(&heartbeat, In(transfer.payload), &size), 0);
  zexpect_equal(heartbeat.uptime, 0U);
  zexpect_equal(heartbeat.mode.value, 0U);
  zexpect_equal(heartbeat.health.value, 1U);

  // No second heartbeat within the second, one after it with uptime 1.
  rig.Run(100'000U, 5U);
  zexpect_equal(rig.client.Receive(rig.rx, transfer), ReceiveResult::kNoTransfer);
  rig.Run(100'000U, 5U);
  zassert_equal(rig.client.Receive(rig.rx, transfer), ReceiveResult::kReceived);
  size = transfer.payload.size();
  zassert_equal(uavcan_node_Heartbeat_1_0_deserialize_(&heartbeat, In(transfer.payload), &size), 0);
  zexpect_equal(heartbeat.uptime, 1U);
}

ZTEST(cyphal_stack, test_get_info_over_classic_can) {
  Rig& rig = MakeRig(kCanClassicMtu);
  zassert_true(rig.client.Subscribe(TransferKind::kServiceResponse, port::kGetInfo, 448U));
  zassert_true(rig.node.Start(kServerId, rig.now));
  zassert_true(rig.Request(port::kGetInfo, {}));

  Transfer transfer{};
  zassert_true(rig.Await(port::kGetInfo, transfer));
  zexpect_equal(transfer.kind, TransferKind::kServiceResponse);
  zexpect_equal(rig.server_media_a.largest_sent, kCanClassicMtu);  // multi-frame on classic CAN
  uavcan_node_GetInfo_Response_1_0 info{};
  std::size_t size = transfer.payload.size();
  zassert_equal(uavcan_node_GetInfo_Response_1_0_deserialize_(&info, In(transfer.payload), &size), 0);
  zexpect_equal(info.protocol_version.major, 1U);
  zexpect_equal(info.software_version.major, 3U);
  zexpect_equal(info.software_version.minor, 4U);
  zexpect_equal(info.software_vcs_revision_id, 0x1234ABCDULL);
  zexpect_equal(info.unique_id[0], 1U);
  zexpect_equal(info.unique_id[15], 16U);
  zexpect_equal(std::string_view(reinterpret_cast<const char*>(info.name.elements), info.name.count),
                std::string_view{"org.unimoc.test"});
}

ZTEST(cyphal_stack, test_register_list_spans_providers) {
  Rig& rig = MakeRig(kCanFdMtu);
  TestRegisters first;
  TestRegisters second;
  zassert_true(rig.node.AddRegisterProvider(first));
  zassert_true(rig.node.AddRegisterProvider(second));
  zassert_true(rig.client.Subscribe(TransferKind::kServiceResponse, port::kRegisterList, 256U));
  zassert_true(rig.node.Start(kServerId, rig.now));

  const std::array<uint16_t, 5> kIndices{0U, 1U, 2U, 3U, 4U};
  const std::array<std::string_view, 5> kExpected{"test.gain", "test.label", "test.gain", "test.label", ""};
  for (std::size_t i = 0U; i < kIndices.size(); ++i) {
    uavcan_register_List_Request_1_0 request{kIndices[i]};
    std::size_t size = rig.tx.size();
    zassert_equal(uavcan_register_List_Request_1_0_serialize_(&request, Out(rig.tx), &size), 0);
    zassert_true(rig.Request(port::kRegisterList, std::span<const std::byte>{rig.tx.data(), size}));
    Transfer transfer{};
    zassert_true(rig.Await(port::kRegisterList, transfer));
    uavcan_register_List_Response_1_0 response{};
    size = transfer.payload.size();
    zassert_equal(uavcan_register_List_Response_1_0_deserialize_(&response, In(transfer.payload), &size), 0);
    zexpect_equal(std::string_view(reinterpret_cast<const char*>(response.name.name.elements),
                                   response.name.name.count),
                  kExpected[i]);
  }
}

uavcan_register_Access_Response_1_0 Access(Rig& rig, std::string_view name, const RegisterValue& value) {
  uavcan_register_Access_Request_1_0 request{};
  std::memcpy(std::begin(request.name.name.elements), name.data(), name.size());
  request.name.name.count = name.size();
  request.value = value;
  std::size_t size = rig.tx.size();
  zassert_equal(uavcan_register_Access_Request_1_0_serialize_(&request, Out(rig.tx), &size), 0);
  zassert_true(rig.Request(port::kRegisterAccess, std::span<const std::byte>{rig.tx.data(), size}));
  Transfer transfer{};
  zassert_true(rig.Await(port::kRegisterAccess, transfer));
  uavcan_register_Access_Response_1_0 response{};
  size = transfer.payload.size();
  zassert_equal(uavcan_register_Access_Response_1_0_deserialize_(&response, In(transfer.payload), &size), 0);
  return response;
}

ZTEST(cyphal_stack, test_register_access_reads_writes_and_rejects) {
  Rig& rig = MakeRig(kCanFdMtu);
  TestRegisters registers;
  zassert_true(rig.node.AddRegisterProvider(registers));
  zassert_true(rig.client.Subscribe(TransferKind::kServiceResponse, port::kRegisterAccess, 267U));
  zassert_true(rig.node.Start(kServerId, rig.now));

  RegisterValue empty{};
  auto response = Access(rig, "test.gain", empty);
  zexpect_true(response._mutable);
  zexpect_true(response.persistent);
  zexpect_equal(GetNumber(response.value).value_or(0.0F), 1.5F);

  // A natural16 write to a real32 register is converted.
  RegisterValue write{};
  SetNatural16(write, 7U);
  response = Access(rig, "test.gain", write);
  zexpect_equal(GetNumber(response.value).value_or(0.0F), 7.0F);
  zexpect_equal(registers.gain, 7.0F);

  // Rejected value: the response holds the unchanged value.
  SetReal32(write, -1.0F);
  response = Access(rig, "test.gain", write);
  zexpect_equal(GetNumber(response.value).value_or(0.0F), 7.0F);

  // Read-only register ignores the write.
  SetString(write, "other");
  response = Access(rig, "test.label", write);
  zexpect_false(response._mutable);
  zexpect_equal(GetString(response.value).value_or(""), std::string_view{"fixed"});

  // Unknown register: empty value.
  response = Access(rig, "test.unknown", empty);
  zexpect_true(IsEmpty(response.value));
}

ZTEST(cyphal_stack, test_execute_command_reaches_the_handler) {
  Rig& rig = MakeRig(kCanFdMtu);
  TestCommands commands;
  zassert_true(rig.client.Subscribe(TransferKind::kServiceResponse, port::kExecuteCommand, 48U));
  zassert_true(rig.node.Start(kServerId, rig.now));

  const auto kExecute = [&](const uint16_t code) {
    uavcan_node_ExecuteCommand_Request_1_3 request{};
    request.command = code;
    request.parameter.count = 3U;
    std::size_t size = rig.tx.size();
    zassert_equal(uavcan_node_ExecuteCommand_Request_1_3_serialize_(&request, Out(rig.tx), &size), 0);
    zassert_true(rig.Request(port::kExecuteCommand, std::span<const std::byte>{rig.tx.data(), size}));
    Transfer transfer{};
    zassert_true(rig.Await(port::kExecuteCommand, transfer));
    uavcan_node_ExecuteCommand_Response_1_3 response{};
    size = transfer.payload.size();
    zassert_equal(uavcan_node_ExecuteCommand_Response_1_3_deserialize_(&response,
                                                                       In(transfer.payload),
                                                                       &size),
                  0);
    return response.status;
  };

  zexpect_equal(kExecute(7U), cmd::kStatusBadCommand);  // no handler yet
  rig.node.SetCommandHandler(commands);
  zexpect_equal(kExecute(7U), cmd::kStatusSuccess);
  zexpect_equal(commands.last_command, 7U);
  zexpect_equal(commands.last_parameter_size, 3U);
  zexpect_equal(kExecute(cmd::kCommandPowerOff), cmd::kStatusBadCommand);
}

ZTEST(cyphal_stack, test_application_servers_answer_their_requests) {
  Rig& rig = MakeRig(kCanFdMtu);
  TestService service;
  zassert_true(rig.node.AddServer(kUnsetPortId, 8U, service));  // disabled: ignored
  zassert_true(rig.node.AddServer(120U, 8U, service));
  zassert_true(rig.node.AddServer(124U, 8U, service));
  zassert_false(rig.node.AddServer(124U, 8U, service));            // taken
  zassert_false(rig.node.AddServer(port::kGetInfo, 8U, service));  // a standard service
  zassert_false(rig.node.AddServer(512U, 8U, service));            // no service-ID
  zassert_true(rig.client.Subscribe(TransferKind::kServiceResponse, 120U, 8U));
  zassert_true(rig.client.Subscribe(TransferKind::kServiceResponse, 124U, 8U));
  zassert_true(rig.node.Start(kServerId, rig.now));

  const std::array<std::byte, 3> kRequest{std::byte{1}, std::byte{2}, std::byte{3}};
  for (const uint16_t kService : {uint16_t{120U}, uint16_t{124U}}) {
    zassert_true(rig.Request(kService, kRequest));
    Transfer transfer{};
    zassert_true(rig.Await(kService, transfer));
    zexpect_equal(service.last_port, kService);
    zassert_equal(transfer.payload.size(), 3U);
    zexpect_equal(transfer.payload[0], std::byte{3});
    zexpect_equal(transfer.payload[2], std::byte{1});
  }

  // No response when the handler declines.
  zassert_true(rig.Request(120U, std::span<const std::byte>{}));
  Transfer transfer{};
  zexpect_false(rig.Await(120U, transfer));
  zexpect_equal(service.count, 3U);
}

ZTEST(cyphal_stack, test_plug_and_play_allocation) {
  Rig& rig = MakeRig(kCanClassicMtu);
  TestNodeIdListener listener;
  rig.node.SetNodeIdListener(listener);
  zassert_true(rig.client.Subscribe(TransferKind::kMessage, port::kNodeIdAllocation, 9U));
  zassert_true(rig.client.Subscribe(TransferKind::kMessage, port::kHeartbeat, 12U));
  zassert_true(rig.node.Start(kNodeIdUnset, rig.now));
  zexpect_equal(rig.node.NodeId(), kNodeIdUnset);

  // The anonymous request arrives within the first 1.5 s; no heartbeat meanwhile.
  Transfer transfer{};
  bool requested = false;
  for (unsigned i = 0U; i < 40U && !requested; ++i) {
    rig.Run(50'000U, 1U);
    while (rig.client.Receive(rig.rx, transfer) == ReceiveResult::kReceived) {
      zassert_not_equal(transfer.port_id, port::kHeartbeat);
      requested = transfer.port_id == port::kNodeIdAllocation;
    }
  }
  zassert_true(requested);
  zexpect_equal(transfer.source_node_id, kNodeIdUnset);
  uavcan_pnp_NodeIDAllocationData_1_0 request{};
  std::size_t size = transfer.payload.size();
  zassert_equal(uavcan_pnp_NodeIDAllocationData_1_0_deserialize_(&request, In(transfer.payload), &size), 0);
  zexpect_equal(request.unique_id_hash, rig.node.UniqueIdHash());
  zexpect_equal(request.allocated_node_id.count, 0U);

  // A response for another hash is ignored, the matching one is taken.
  uavcan_pnp_NodeIDAllocationData_1_0 response{};
  response.allocated_node_id.count = 1U;
  response.allocated_node_id.elements[0].value = 77U;
  for (const uint64_t kHash : {request.unique_id_hash ^ 1U, request.unique_id_hash}) {
    response.unique_id_hash = kHash;
    size = rig.tx.size();
    zassert_equal(uavcan_pnp_NodeIDAllocationData_1_0_serialize_(&response, Out(rig.tx), &size), 0);
    const Transfer kMessage{.port_id = port::kNodeIdAllocation,
                            .source_node_id = kClientId,
                            .destination_node_id = kNodeIdUnset,
                            .transfer_id = rig.next_transfer_id++,
                            .kind = TransferKind::kMessage,
                            .priority = Priority::kSlow,
                            .payload = std::span<const std::byte>{rig.tx.data(), size}};
    zassert_true(rig.client.Send(kMessage));
    rig.Run();
    if (kHash != request.unique_id_hash) {
      zexpect_equal(rig.node.NodeId(), kNodeIdUnset);
    }
  }
  zexpect_equal(rig.node.NodeId(), 77U);
  zexpect_equal(listener.allocated, 77U);
  zassert_true(rig.Await(port::kHeartbeat, transfer));
  zexpect_equal(transfer.source_node_id, 77U);
}

ZTEST(cyphal_stack, test_port_list_names_ports_and_services) {
  Rig& rig = MakeRig(kCanFdMtu);
  TestMessages messages;
  TestService service;
  zassert_true(rig.node.Subscribe(300U, 8U, messages));
  zassert_true(rig.node.AddServer(130U, 8U, service));
  zassert_true(rig.node.AddPublisher(Publisher{301U, Priority::kNominal, 0U}));
  zassert_true(rig.client.Subscribe(TransferKind::kMessage, port::kPortList, 8466U));
  zassert_true(rig.node.Start(kServerId, rig.now));

  Transfer transfer{};
  bool received = false;
  for (unsigned i = 0U; i < 70U && !received; ++i) {
    rig.Run(100'000U, 1U);
    while (rig.client.Receive(rig.rx, transfer) == ReceiveResult::kReceived) {
      received = received || transfer.port_id == port::kPortList;
    }
  }
  zassert_true(received);
  static uavcan_node_port_List_1_0 list{};
  std::size_t size = transfer.payload.size();
  zassert_equal(uavcan_node_port_List_1_0_deserialize_(&list, In(transfer.payload), &size), 0);
  zassert_true(uavcan_node_port_SubjectIDList_1_0_is_sparse_list_(&list.publishers));
  zexpect_equal(list.publishers.sparse_list.count, 4U);
  zexpect_equal(list.publishers.sparse_list.elements[0].value, port::kHeartbeat);
  zexpect_equal(list.publishers.sparse_list.elements[3].value, 301U);
  zassert_true(uavcan_node_port_SubjectIDList_1_0_is_sparse_list_(&list.subscribers));
  zexpect_equal(list.subscribers.sparse_list.count, 1U);
  zexpect_equal(list.subscribers.sparse_list.elements[0].value, 300U);
  const auto kServes = [&](const uint16_t service) {
    return (list.servers.mask_bitpacked_[service / 8U] & (1U << (service % 8U))) != 0U;
  };
  zexpect_true(kServes(port::kGetInfo));
  zexpect_true(kServes(port::kRegisterAccess));
  zexpect_true(kServes(port::kRegisterList));
  zexpect_true(kServes(port::kExecuteCommand));
  zexpect_true(kServes(130U));
  zexpect_false(kServes(port::kGetInfo + 1U));
}

ZTEST(cyphal_stack, test_messages_reach_listeners_and_logs_are_published) {
  Rig& rig = MakeRig(kCanFdMtu);
  TestMessages messages;
  zassert_true(rig.node.Subscribe(300U, 8U, messages));
  zassert_true(rig.node.Subscribe(kUnsetPortId, 8U, messages));  // a disabled port is accepted
  zassert_true(rig.client.Subscribe(TransferKind::kMessage, port::kDiagnosticRecord, 300U));
  zassert_true(rig.node.Start(kServerId, rig.now));

  const std::array<std::byte, 4> kPayload{};
  const Transfer kMessage{.port_id = 300U,
                          .source_node_id = kClientId,
                          .destination_node_id = kNodeIdUnset,
                          .transfer_id = 0U,
                          .kind = TransferKind::kMessage,
                          .priority = Priority::kNominal,
                          .payload = kPayload};
  zassert_true(rig.client.Send(kMessage));
  rig.Run();
  zexpect_equal(messages.count, 1U);
  zexpect_equal(messages.last_source, kClientId);
  zexpect_equal(messages.last_size, 4U);

  zassert_true(rig.node.Log(Severity::kWarning, "overcurrent trip"));
  Transfer transfer{};
  zassert_true(rig.Await(port::kDiagnosticRecord, transfer));
  uavcan_diagnostic_Record_1_1 record{};
  std::size_t size = transfer.payload.size();
  zassert_equal(uavcan_diagnostic_Record_1_1_deserialize_(&record, In(transfer.payload), &size), 0);
  zexpect_equal(record.severity.value, 4U);
  zexpect_equal(std::string_view(reinterpret_cast<const char*>(record.text.elements), record.text.count),
                std::string_view{"overcurrent trip"});
}

ZTEST(cyphal_stack, test_a_subject_subscribed_twice_keeps_one_subscription) {
  Rig& rig = MakeRig(kCanFdMtu);
  TestMessages first;
  TestMessages second;
  TestMessages other;
  // Two registers on the same subject, the second with the larger extent,
  // then another port: its subscription must not reuse a linked object.
  zassert_true(rig.node.Subscribe(300U, 2U, first));
  zassert_true(rig.node.Subscribe(300U, 8U, second));
  zassert_true(rig.node.Subscribe(301U, 8U, other));
  zassert_true(rig.node.Start(kServerId, rig.now));

  const std::array<std::byte, 6> kPayload{};
  Transfer message{.port_id = 300U,
                   .source_node_id = kClientId,
                   .destination_node_id = kNodeIdUnset,
                   .transfer_id = 0U,
                   .kind = TransferKind::kMessage,
                   .priority = Priority::kNominal,
                   .payload = kPayload};
  zassert_true(rig.client.Send(message));
  message.port_id = 301U;
  zassert_true(rig.client.Send(message));
  rig.Run();
  zexpect_equal(first.count, 1U);
  zexpect_equal(second.count, 1U);
  zexpect_equal(second.last_size, kPayload.size());  // the larger extent applies
  zexpect_equal(other.count, 1U);
}

ZTEST(cyphal_stack, test_redundant_interfaces_deliver_once_and_survive_a_dead_bus) {
  Rig& rig = MakeRig(kCanFdMtu, true);
  zassert_equal(rig.server_transport.InterfaceCount(), 2U);
  zassert_true(rig.client.Subscribe(TransferKind::kServiceResponse, port::kGetInfo, 448U));
  zassert_true(rig.node.Start(kServerId, rig.now));

  zassert_true(rig.Request(port::kGetInfo, {}));
  Transfer transfer{};
  zassert_true(rig.Await(port::kGetInfo, transfer));
  rig.Run();
  zexpect_equal(rig.client.Receive(rig.rx, transfer),
                ReceiveResult::kNoTransfer);  // the duplicate is dropped
  zexpect_true(rig.server_media_a.sent > 0U);
  zexpect_true(rig.server_media_b.sent > 0U);

  // Bus A refuses frames: the response arrives over bus B. The receiver
  // follows the other interface after the transfer-ID timeout (libcanard).
  rig.server_media_a.accept_tx = false;
  rig.Run(CanardTransport::kTransferIdTimeoutUs / 4U, 5U);
  zassert_true(rig.Request(port::kGetInfo, {}));
  zassert_true(rig.Await(port::kGetInfo, transfer));
  rig.Run(200'000U, 2U);
  zexpect_true(rig.server_transport.TxDropped() > 0U);
}

ZTEST(cyphal_stack, test_register_value_helpers) {
  RegisterValue value{};
  zexpect_true(IsEmpty(value));
  zexpect_false(GetNumber(value).has_value());
  SetBit(value, true);
  zexpect_equal(GetNumber(value).value_or(0.0F), 1.0F);
  SetNatural8(value, 200U);
  zexpect_equal(GetInteger(value).value_or(0), 200);
  SetReal32(value, 2.5F);
  zexpect_false(GetInteger(value).has_value());  // not integral
  SetNatural32(value, 70000U);
  zexpect_equal(GetInteger(value).value_or(0), 70000);
  SetString(value, "abc");
  zexpect_false(GetNumber(value).has_value());
  zexpect_equal(GetString(value).value_or(""), std::string_view{"abc"});
  const std::array<uint8_t, 2> kBytes{1U, 2U};
  SetUnstructured(value, kBytes);
  zexpect_equal(value.unstructured.value.count, 2U);
  SetEmpty(value);
  zexpect_true(IsEmpty(value));
}

}  // namespace
}  // namespace unimoc::cyphal::test

ZTEST_SUITE(cyphal_stack, nullptr, nullptr, nullptr, nullptr, nullptr);
