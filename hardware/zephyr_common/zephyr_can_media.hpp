/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file zephyr_can_media.hpp
 *    @brief Cyphal CAN interface on a Zephyr CAN controller.
 *
 *    Shared by the BCC (STM32G4 FDCAN) and native_sim (SocketCAN through
 *    the native Linux CAN driver).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <array>
#include "cyphal/can_media.hpp"

namespace unimoc::hardware {

/**
 * @brief CAN controller of the Zephyr CAN API as a Cyphal CAN interface.
 *
 * Receives all extended data frames (Cyphal/CAN uses no standard IDs) into a
 * message queue from the driver callback and wakes the Cyphal thread through
 * a semaphore. Frames are sent without blocking; a full controller makes
 * Push() fail and the transport retries.
 */
class ZephyrCanMedia final : public cyphal::CanMedia {
 public:
  /// Received frames buffered between two runs of the Cyphal thread.
  static constexpr std::size_t kRxQueueLength = 32U;

  /**
   * @brief Configures and starts the controller.
   * @param device CAN controller.
   * @param wake Semaphore given for every received frame (the Cyphal thread waits on it).
   * @param use_fd Use CAN FD frames (64 bytes) when the controller supports it.
   * @return 0, or a negative errno of the driver.
   */
  int Init(const struct device* device, struct k_sem* wake, bool use_fd) noexcept;

  [[nodiscard]] std::size_t Mtu() const noexcept override { return mtu_; }
  [[nodiscard]] bool Push(const cyphal::CanFrame& frame) noexcept override;
  [[nodiscard]] bool Pop(cyphal::CanFrame& frame) noexcept override;

  /// Frames dropped because the receive queue was full.
  [[nodiscard]] uint32_t RxOverruns() const noexcept { return rx_overruns_; }

 private:
  static void OnFrame(const struct device* device, struct can_frame* frame, void* user_data);

  const struct device* device_{nullptr};
  struct k_sem* wake_{nullptr};
  struct k_msgq rx_queue_{};
  alignas(4) std::array<char, kRxQueueLength * sizeof(struct can_frame)> rx_buffer_{};
  std::size_t mtu_{cyphal::kCanClassicMtu};
  bool brs_{false};
  uint32_t rx_overruns_{0U};
};

}  // namespace unimoc::hardware
