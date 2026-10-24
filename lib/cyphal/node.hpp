/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file node.hpp
 *    @brief Transport-injected Cyphal node: heartbeat, GetInfo, registers,
 *           ExecuteCommand, plug-and-play, port list and diagnostics.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include "cyphal/cyphal_interface.hpp"
#include "cyphal/node_identity.hpp"
#include "cyphal/register_value.hpp"
#include "cyphal/transport.hpp"

namespace unimoc::cyphal {

/// Node health of uavcan.node.Heartbeat.1.0.
enum class Health : uint8_t {
  kNominal = 0,   ///< The node is functioning properly.
  kAdvisory = 1,  ///< A non-critical anomaly; the node still works.
  kCaution = 2,   ///< A critical anomaly; recovery may be possible.
  kWarning = 3,   ///< The node is about to fail or has failed.
};

/// Node mode of uavcan.node.Heartbeat.1.0.
enum class Mode : uint8_t {
  kOperational = 0,     ///< Normal operation.
  kInitialization = 1,  ///< Starting up.
  kMaintenance = 2,     ///< Under maintenance, not operational.
  kSoftwareUpdate = 3,  ///< Updating its software.
};

/// Severity of uavcan.diagnostic.Record.1.1.
enum class Severity : uint8_t {
  kTrace = 0,
  kDebug = 1,
  kInfo = 2,
  kNotice = 3,
  kWarning = 4,
  kError = 5,
  kCritical = 6,
  kAlert = 7,
};

/// Fixed port-IDs of the standard services and messages the node uses.
namespace port {
inline constexpr uint16_t kHeartbeat = 7509U;
inline constexpr uint16_t kPortList = 7510U;
inline constexpr uint16_t kNodeIdAllocation = 8166U;
inline constexpr uint16_t kDiagnosticRecord = 8184U;
inline constexpr uint16_t kRegisterAccess = 384U;
inline constexpr uint16_t kRegisterList = 385U;
inline constexpr uint16_t kGetInfo = 430U;
inline constexpr uint16_t kExecuteCommand = 435U;
}  // namespace port

/// Values reported by uavcan.node.GetInfo.
struct NodeInfo {
  /// Node name (at most kNodeNameMaxLen characters); the view must outlive the node.
  std::string_view name{"unimoc"};
  uint8_t hardware_major{0U};  ///< Hardware version, major.
  uint8_t hardware_minor{0U};  ///< Hardware version, minor.
  uint8_t software_major{0U};  ///< Software version, major.
  uint8_t software_minor{0U};  ///< Software version, minor.
  /// Version control revision of the build (the first 16 hex digits of the git hash), 0 if unknown.
  uint64_t vcs_revision_id{0U};
  /// Hardware unique-ID (MCU UID); also the source of the plug-and-play hash.
  std::array<uint8_t, kUniqueIdLen> unique_id{};
};

/// Receives the messages of one subscribed subject.
class MessageListener {
 public:
  MessageListener() = default;
  virtual ~MessageListener() = default;
  MessageListener(const MessageListener&) = delete;
  MessageListener& operator=(const MessageListener&) = delete;
  MessageListener(MessageListener&&) = delete;
  MessageListener& operator=(MessageListener&&) = delete;

  /**
   * @brief Handles one received message.
   * @param transfer The message; its payload is valid during the call only.
   * @param now_us Time of the CyphalNode::Spin() call that received it [us].
   */
  virtual void OnMessage(const Transfer& transfer, uint64_t now_us) noexcept = 0;
};

/// Executes uavcan.node.ExecuteCommand requests.
class CommandHandler {
 public:
  CommandHandler() = default;
  virtual ~CommandHandler() = default;
  CommandHandler(const CommandHandler&) = delete;
  CommandHandler& operator=(const CommandHandler&) = delete;
  CommandHandler(CommandHandler&&) = delete;
  CommandHandler& operator=(CommandHandler&&) = delete;

  /**
   * @brief Executes or starts one command; the response is sent right after.
   * @param command Standard (cmd::kCommand*) or vendor command code.
   * @param parameter Command parameter.
   * @return Response status (cmd::kStatus*).
   */
  [[nodiscard]] virtual uint8_t OnCommand(uint16_t command, std::span<const uint8_t> parameter) noexcept = 0;
};

/// Told when plug-and-play allocated a node-ID.
class NodeIdListener {
 public:
  NodeIdListener() = default;
  virtual ~NodeIdListener() = default;
  NodeIdListener(const NodeIdListener&) = delete;
  NodeIdListener& operator=(const NodeIdListener&) = delete;
  NodeIdListener(NodeIdListener&&) = delete;
  NodeIdListener& operator=(NodeIdListener&&) = delete;

  /// Called once with the allocated node-ID; the node uses it from then on.
  virtual void OnNodeIdAllocated(uint8_t node_id) noexcept = 0;
};

/// Serves the requests of one application service (a port-ID from a register).
class ServiceHandler {
 public:
  ServiceHandler() = default;
  virtual ~ServiceHandler() = default;
  ServiceHandler(const ServiceHandler&) = delete;
  ServiceHandler& operator=(const ServiceHandler&) = delete;
  ServiceHandler(ServiceHandler&&) = delete;
  ServiceHandler& operator=(ServiceHandler&&) = delete;

  /**
   * @brief Handles one request; the node sends the response right after.
   * @param request The request; its port-ID tells the services of one handler
   *                apart, its payload is valid during the call only.
   * @param response Buffer for the serialized response.
   * @return Size of the response in @p response, or std::nullopt to send none
   *         (the request could not be deserialized).
   */
  [[nodiscard]] virtual std::optional<std::size_t> OnRequest(const Transfer& request,
                                                             std::span<std::byte> response) noexcept = 0;
};

/// Port and transfer-ID counter of one published subject.
struct Publisher {
  uint16_t port_id{kUnsetPortId};         ///< Subject-ID; kUnsetPortId disables the publisher.
  Priority priority{Priority::kNominal};  ///< Arbitration priority of the messages.
  uint8_t next_transfer_id{0U};           ///< Transfer-ID of the next message.
};

/**
 * @brief Cyphal application node on an injected transport.
 *
 * Implements the standard node services and leaves the application data
 * (setpoints, telemetry, settings) to the caller:
 *
 * - uavcan.node.Heartbeat (1 Hz) and uavcan.node.port.List (every 5 s)
 * - uavcan.node.GetInfo with NodeInfo
 * - uavcan.register.List / Access over the added RegisterProviders
 * - uavcan.node.ExecuteCommand, forwarded to the CommandHandler
 * - application services on register-assigned port-IDs (AddServer())
 * - uavcan.pnp.NodeIDAllocationData.1.0 (Cyphal/CAN) while the node has no ID
 * - uavcan.diagnostic.Record through Log()
 *
 * Everything runs in the thread that calls Spin(); nothing here is reachable
 * from the zero-latency hot path.
 */
class CyphalNode {
 public:
  /// Largest number of register providers (drive: settings, info, startup,
  /// timing; the platform: one more).
  static constexpr std::size_t kMaxRegisterProviders = 6U;
  /// Largest number of application subscriptions (drive: 6 setpoint and
  /// readiness subjects, 3 cart subjects; traces: the trigger).
  static constexpr std::size_t kMaxListeners = 12U;
  /// Largest number of application publishers reported in the port list.
  static constexpr std::size_t kMaxPublishers = 8U;
  /// Largest number of application services.
  static constexpr std::size_t kMaxServers = 8U;
  /// Heartbeat period [us].
  static constexpr uint64_t kHeartbeatPeriodUs = 1'000'000U;
  /// Port list period [us] (the standard allows up to 10 s).
  static constexpr uint64_t kPortListPeriodUs = 5'000'000U;
  /// Largest number of transfers Spin() handles per call.
  static constexpr std::size_t kMaxTransfersPerSpin = 32U;

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
   * @brief Polls the injected transport for one transfer, bypassing the dispatch of Spin().
   * @param payload_buffer Caller-owned storage for the received payload.
   * @param transfer Output transfer whose payload points into `payload_buffer`
   *                 when a transfer is received.
   * @return The transport receive result.
   */
  [[nodiscard]] ReceiveResult Poll(std::span<std::byte> payload_buffer, Transfer& transfer) noexcept {
    return transport_->Receive(payload_buffer, transfer);
  }

  /// Sets the values of GetInfo; call before Start().
  void SetInfo(const NodeInfo& info) noexcept { info_ = info; }

  /// Adds a register provider; returns false when all slots are used.
  [[nodiscard]] bool AddRegisterProvider(RegisterProvider& provider) noexcept;

  /// Sets the handler of ExecuteCommand; without one every command is answered kStatusBadCommand.
  void SetCommandHandler(CommandHandler& handler) noexcept { command_handler_ = &handler; }

  /// Sets the listener told about a plug-and-play allocation.
  void SetNodeIdListener(NodeIdListener& listener) noexcept { node_id_listener_ = &listener; }

  /**
   * @brief Subscribes the application to a subject.
   * @param subject_id Subject-ID; kUnsetPortId is ignored (returns true).
   * @param extent DSDL extent of the message type [bytes].
   * @param listener Receives the messages; must outlive the node.
   * @return False when no slot is left or the transport refused it.
   */
  [[nodiscard]] bool Subscribe(uint16_t subject_id, std::size_t extent, MessageListener& listener) noexcept;

  /**
   * @brief Serves an application service: subscribes its requests and lists it in the port list.
   * @param service_id Service-ID 0..511; kUnsetPortId is ignored (returns true).
   * @param request_extent DSDL extent of the request type [bytes].
   * @param handler Answers the requests; must outlive the node.
   * @return False when the service-ID is invalid or taken, no slot is left or
   *         the transport refused the subscription.
   */
  [[nodiscard]] bool AddServer(uint16_t service_id,
                               std::size_t request_extent,
                               ServiceHandler& handler) noexcept;

  /**
   * @brief Registers a publisher for the port list. Publish() works without it.
   * @return False when all slots are used.
   */
  [[nodiscard]] bool AddPublisher(const Publisher& publisher) noexcept;

  /**
   * @brief Starts the node services.
   * @param node_id Node-ID 0..127, or a larger value to request one by plug-and-play.
   * @param now_us Current time [us]; the uptime counts from here.
   * @return False when the transport refused a subscription.
   */
  [[nodiscard]] bool Start(uint8_t node_id, uint64_t now_us) noexcept;

  /**
   * @brief Handles received transfers and the periodic node messages.
   * @param now_us Monotonic time [us].
   */
  void Spin(uint64_t now_us) noexcept;

  /**
   * @brief Publishes one message.
   * @param publisher Port, priority and transfer-ID counter; the counter advances.
   * @param payload Serialized message.
   * @return False when the node has no node-ID yet, the port is unset or the
   *         transport refused the transfer.
   */
  [[nodiscard]] bool Publish(Publisher& publisher, std::span<const std::byte> payload) noexcept;

  /**
   * @brief Publishes a uavcan.diagnostic.Record.
   * @param severity Severity of the record.
   * @param text Text, cut at 255 bytes.
   * @return False when it could not be queued.
   */
  [[nodiscard]] bool Log(Severity severity, std::string_view text) noexcept;

  /// Health and mode reported by the heartbeat.
  void SetHealth(Health health) noexcept { health_ = health; }
  void SetMode(Mode mode) noexcept { mode_ = mode; }
  /// Vendor-specific status code of the heartbeat.
  void SetVendorStatus(uint8_t status) noexcept { vendor_status_ = status; }

  /// Node-ID, or kNodeIdUnset while plug-and-play runs.
  [[nodiscard]] uint8_t NodeId() const noexcept { return transport_->LocalNodeId(); }

  /// Bus state of the transport (CAN fault confinement).
  [[nodiscard]] CanBusStatus BusStatus() const noexcept { return transport_->BusStatus(); }

  /// Seconds since Start().
  [[nodiscard]] uint32_t UptimeSeconds(uint64_t now_us) const noexcept;

  /// 48-bit plug-and-play hash of the unique-ID.
  [[nodiscard]] uint64_t UniqueIdHash() const noexcept;

  /// Register lookup by name; returns false when no provider has it.
  [[nodiscard]] bool FindRegister(std::string_view name,
                                  RegisterProvider*& provider,
                                  std::size_t& index) const noexcept;

  /// The added register providers, in the order uavcan.register.List names their registers.
  [[nodiscard]] std::span<RegisterProvider* const> RegisterProviders() const noexcept {
    return std::span<RegisterProvider* const>{providers_.data(), provider_count_};
  }

 private:
  struct Listener {
    uint16_t subject_id{kUnsetPortId};
    MessageListener* listener{nullptr};
  };

  struct Server {
    uint16_t service_id{kUnsetPortId};
    ServiceHandler* handler{nullptr};
  };

  void Dispatch(const Transfer& transfer, uint64_t now_us) noexcept;
  void HandleGetInfo(const Transfer& request) noexcept;
  void HandleRegisterList(const Transfer& request) noexcept;
  void HandleRegisterAccess(const Transfer& request) noexcept;
  void HandleExecuteCommand(const Transfer& request) noexcept;
  void HandleAllocation(const Transfer& message) noexcept;
  void HandleApplicationRequest(const Transfer& request) noexcept;
  void Respond(const Transfer& request, std::size_t size) noexcept;
  void PublishHeartbeat(uint64_t now_us) noexcept;
  void PublishPortList() noexcept;
  void RequestNodeId(uint64_t now_us) noexcept;
  [[nodiscard]] bool PublishInternal(Publisher& publisher, std::size_t size) noexcept;

  Transport* transport_;
  NodeInfo info_{};
  CommandHandler* command_handler_{nullptr};
  NodeIdListener* node_id_listener_{nullptr};
  std::array<RegisterProvider*, kMaxRegisterProviders> providers_{};
  std::size_t provider_count_{0U};
  std::array<Listener, kMaxListeners> listeners_{};
  std::size_t listener_count_{0U};
  std::array<Server, kMaxServers> servers_{};
  std::size_t server_count_{0U};
  std::array<uint16_t, kMaxPublishers> published_subjects_{};
  std::size_t published_count_{0U};

  Health health_{Health::kNominal};
  Mode mode_{Mode::kInitialization};
  uint8_t vendor_status_{0U};
  bool started_{false};
  uint64_t start_us_{0U};
  uint64_t next_heartbeat_us_{0U};
  uint64_t next_port_list_us_{0U};
  uint64_t next_allocation_us_{0U};
  uint32_t allocation_seed_{0U};

  Publisher heartbeat_{port::kHeartbeat, Priority::kNominal, 0U};
  Publisher port_list_{port::kPortList, Priority::kOptional, 0U};
  Publisher allocation_{port::kNodeIdAllocation, Priority::kSlow, 0U};
  Publisher diagnostic_{port::kDiagnosticRecord, Priority::kOptional, 0U};

  /// Serialization buffers: the largest request (register.Access, 515 bytes)
  /// and the port list (sparse lists of a few ports plus two 64 byte masks).
  std::array<std::byte, 600U> rx_buffer_{};
  std::array<std::byte, 600U> tx_buffer_{};
};

}  // namespace unimoc::cyphal
