/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file drive_node.hpp
 *    @brief Cyphal application of the drive: arming, setpoints, telemetry,
 *           settings registers and node commands.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include <span>
#include <string_view>
#include "cyphal/node.hpp"
#include "hardware_interface.hpp"
#include "settings_register_provider.hpp"
#include "settings_store.hpp"

namespace unimoc::drive {

/// Platform services the drive node needs from the Zephyr glue.
struct DriveHooks {
  /// Restarts the firmware (sys_reboot); called after the response went out.
  void (*restart)() noexcept {nullptr};
  /// Called after new settings were applied to the controller (native_sim: the plant follows); may be null.
  void (*settings_applied)(const settings::NvmSettings& settings) noexcept {nullptr};
};

/**
 * @brief Read-only registers of the drive: the data type names of its ports
 *        and the hardware unique-ID.
 */
class DriveInfoRegisters final : public cyphal::RegisterProvider {
 public:
  /// @param unique_id Unique-ID reported by `uavcan.node.unique_id`; must outlive the provider.
  explicit DriveInfoRegisters(std::span<const uint8_t> unique_id) noexcept : unique_id_{unique_id} {}

  [[nodiscard]] std::size_t Count() const noexcept override;
  [[nodiscard]] std::string_view Name(std::size_t index) const noexcept override;
  [[nodiscard]] cyphal::RegisterFlags Read(std::size_t index,
                                           cyphal::RegisterValue& value) const noexcept override;
  [[nodiscard]] cyphal::RegisterWriteResult Write(std::size_t index,
                                                  const cyphal::RegisterValue& value) noexcept override;

 private:
  std::span<const uint8_t> unique_id_;
};

/**
 * @brief Cyphal application of one drive (motor 0).
 *
 * Ports (IDs from the settings, applied at start):
 *
 * | Register | Type | Effect |
 * |---|---|---|
 * | `uavcan.sub.readiness.id` | reg.udral.service.common.Readiness.0.1 | ENGAGED releases the power stage,
 * SLEEP/STANDBY or a timeout disables it | | `uavcan.sub.current_setpoint.id` |
 * unimoc.drive.CurrentSetpoint.0.1 | d/q current setpoint | | `uavcan.sub.torque_setpoint.id` |
 * uavcan.si.unit.torque.Scalar.1.0 | torque, converted to i_q (PMSM) | | `uavcan.pub.telemetry.id` |
 * unimoc.drive.Telemetry.0.1 | controller trace at `unimoc.telemetry.rate` |
 *
 * Setpoints apply only while engaged and fall back to zero current after
 * `unimoc.control.command_timeout`; the current is limited to
 * `unimoc.motor.limits.i_max`. A power stage fault or an overcurrent trip
 * while engaged disengages the drive; it engages again only after a
 * readiness other than ENGAGED (an explicit re-arm). STANDBY while
 * disengaged acknowledges a latched trip or fault without releasing the
 * outputs (a still missing external enable latches it again).
 *
 * Settings registers change the RAM settings. They are applied to the
 * controller (`current_control::StartRuntime()`) while the drive is
 * disengaged, at the latest when it engages; ExecuteCommand
 * STORE_PERSISTENT_STATES saves them to flash.
 *
 * Runs in the Cyphal thread; it reaches the control cycle only through
 * `CurrentControlIsr::SetCurrentRef()` and reads its state for the trace.
 */
class DriveNode final : public cyphal::MessageListener,
                        public cyphal::CommandHandler,
                        public cyphal::NodeIdListener {
 public:
  /// Delay between the RESTART response and the restart [us].
  static constexpr uint64_t kRestartDelayUs = 200'000U;

  /**
   * @param node Cyphal node on the platform transport.
   * @param store Settings store of the firmware.
   * @param motor Hardware access of the motor the drive controls.
   * @param hooks Platform services.
   * @param unique_id Hardware unique-ID for `uavcan.node.unique_id`; must outlive the drive node.
   */
  DriveNode(cyphal::CyphalNode& node,
            settings::SettingsStore& store,
            const hardware::HardwareInterface& motor,
            const DriveHooks& hooks,
            std::span<const uint8_t> unique_id) noexcept;

  /**
   * @brief Adds the registers, subscribes the ports and starts the node.
   * @param now_us Monotonic time [us].
   * @return False when a port could not be subscribed.
   */
  [[nodiscard]] bool Start(uint64_t now_us) noexcept;

  /// Runs the node and the drive logic; call every millisecond or faster.
  void Spin(uint64_t now_us) noexcept;

  /// True while the power stage is released for the setpoint.
  [[nodiscard]] bool IsEngaged() const noexcept { return engaged_; }

  void OnMessage(const cyphal::Transfer& transfer, uint64_t now_us) noexcept override;
  [[nodiscard]] uint8_t OnCommand(uint16_t command, std::span<const uint8_t> parameter) noexcept override;
  void OnNodeIdAllocated(uint8_t node_id) noexcept override;

 private:
  void OnReadiness(const cyphal::Transfer& transfer, uint64_t now_us) noexcept;
  void OnCurrentSetpoint(const cyphal::Transfer& transfer, uint64_t now_us) noexcept;
  void OnTorqueSetpoint(const cyphal::Transfer& transfer, uint64_t now_us) noexcept;
  void SetSetpoint(float i_d, float i_q, uint64_t now_us) noexcept;
  void Engage() noexcept;
  void Disengage(std::string_view reason) noexcept;
  /// Takes the timing and limits of the drive from the live settings.
  void LoadParameters(const settings::NvmSettings& settings) noexcept;
  /// Restarts the controller with the live settings; refused while the outputs are released.
  [[nodiscard]] bool ApplySettings() noexcept;
  void PublishTelemetry() noexcept;
  void UpdateStatus() noexcept;

  cyphal::CyphalNode* node_;
  settings::SettingsStore* store_;
  const hardware::HardwareInterface* motor_;
  DriveHooks hooks_;
  SettingsRegisterProvider settings_registers_;
  DriveInfoRegisters info_registers_;

  // Settings captured at start or at the last apply.
  uint16_t readiness_port_{cyphal::kUnsetPortId};
  uint16_t current_setpoint_port_{cyphal::kUnsetPortId};
  uint16_t torque_setpoint_port_{cyphal::kUnsetPortId};
  uint64_t command_timeout_us_{500'000U};
  uint64_t telemetry_period_us_{0U};
  float current_limit_{0.0F};
  float torque_constant_{0.0F};
  uint32_t applied_writes_{0U};
  bool settings_pending_{false};

  cyphal::Publisher telemetry_{};
  uint64_t next_telemetry_us_{0U};

  bool engaged_{false};
  bool rearm_required_{false};
  bool emergency_stop_{false};
  bool engage_requested_{false};
  uint64_t last_readiness_us_{0U};
  uint64_t last_setpoint_us_{0U};
  bool setpoint_active_{false};
  float setpoint_d_{0.0F};
  float setpoint_q_{0.0F};
  bool restart_requested_{false};
  uint64_t restart_at_us_{0U};
  uint64_t now_us_{0U};
};

}  // namespace unimoc::drive
