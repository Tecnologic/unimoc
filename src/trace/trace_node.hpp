/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file trace_node.hpp
 *    @brief Cyphal services of the traces: the cymon protocol on the drive node.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include "cymon/device.hpp"
#include "cyphal/node.hpp"
#include "nvm_settings.hpp"

namespace unimoc::trace {

/**
 * @brief Serves the cymon trace protocol of one trace device on the drive's node.
 *
 * | Register | Type | Effect |
 * |---|---|---|
 * | `uavcan.srv.cymon_variables.id` | cymon.GetVariableList.0.1 | variable `index` -> name and unit |
 * | `uavcan.srv.cymon_buffer_info.id` | cymon.GetBufferInfo.0.1 | capture limits and tick period |
 * | `uavcan.srv.cymon_setup.id` | cymon.SetupCapture.0.1 | channels, sample period, samples |
 * | `uavcan.srv.cymon_arm.id` | cymon.ArmTrigger.0.1 | arm or disarm the trigger |
 * | `uavcan.srv.cymon_read.id` | cymon.ReadSamples.0.1 | state, then the frames page by page |
 * | `uavcan.pub.cymon_trigger.id` | cymon.TriggerEvent.0.1 | published when a capture triggers |
 * | `uavcan.sub.cymon_trigger.id` | cymon.TriggerEvent.0.1 | triggers the armed capture |
 *
 * The port-IDs are taken from the settings at Start(); a changed ID takes
 * effect after a restart. The device is sampled by the control cycle
 * (OnControlCycle()); everything here runs in the Cyphal thread.
 */
class TraceNode final : public cyphal::ServiceHandler, public cyphal::MessageListener {
 public:
  /**
   * @param node Cyphal node of the drive.
   * @param device Trace device; its variables are registered before Start().
   */
  TraceNode(cyphal::CyphalNode& node, cymon::Device& device) noexcept : node_{&node}, device_{&device} {}

  /**
   * @brief Adds the servers, the trigger subscription and the trigger publisher to the node.
   * @param settings Live settings (port-IDs).
   * @param tick_period_us Period of the control cycle that samples the device [us].
   * @return False when a port could not be added.
   */
  [[nodiscard]] bool Start(const settings::NvmSettings& settings, float tick_period_us) noexcept;

  /**
   * @brief Sets the sampling period after the controller restarted with another PWM frequency [us].
   *
   * Applies to the next capture setup; a capture in progress keeps its decimation.
   */
  void SetTickPeriod(float tick_period_us) noexcept { device_->SetTickPeriod(tick_period_us); }

  /// Publishes the event of a capture that triggered since the last call; call after each node spin.
  void Spin() noexcept;

  [[nodiscard]] std::optional<std::size_t> OnRequest(const cyphal::Transfer& request,
                                                     std::span<std::byte> response) noexcept override;
  void OnMessage(const cyphal::Transfer& transfer, uint64_t now_us) noexcept override;

 private:
  [[nodiscard]] std::optional<std::size_t> OnGetVariableList(std::span<const std::byte> request,
                                                             std::span<std::byte> response) const noexcept;
  [[nodiscard]] std::optional<std::size_t> OnGetBufferInfo(std::span<std::byte> response) const noexcept;
  [[nodiscard]] std::optional<std::size_t> OnSetupCapture(std::span<const std::byte> request,
                                                          std::span<std::byte> response) noexcept;
  [[nodiscard]] std::optional<std::size_t> OnArmTrigger(std::span<const std::byte> request,
                                                        std::span<std::byte> response) noexcept;
  [[nodiscard]] std::optional<std::size_t> OnReadSamples(std::span<const std::byte> request,
                                                         std::span<std::byte> response) const noexcept;

  cyphal::CyphalNode* node_;
  cymon::Device* device_;
  uint16_t variables_port_{cyphal::kUnsetPortId};
  uint16_t buffer_info_port_{cyphal::kUnsetPortId};
  uint16_t setup_port_{cyphal::kUnsetPortId};
  uint16_t arm_port_{cyphal::kUnsetPortId};
  uint16_t read_port_{cyphal::kUnsetPortId};
  uint16_t trigger_subject_{cyphal::kUnsetPortId};
  cyphal::Publisher trigger_publisher_{};
};

}  // namespace unimoc::trace
