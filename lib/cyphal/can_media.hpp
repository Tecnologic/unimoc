/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file can_media.hpp
 *    @brief CAN interface boundary below the Cyphal/CAN transport.
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

namespace unimoc::cyphal {

/// Data bytes of a classic CAN frame.
inline constexpr std::size_t kCanClassicMtu = 8U;

/// Data bytes of a CAN FD frame.
inline constexpr std::size_t kCanFdMtu = 64U;

/// One CAN data frame with a 29-bit identifier, as Cyphal/CAN uses it.
struct CanFrame {
  /// Extended (29-bit) CAN identifier.
  uint32_t extended_can_id{0U};
  /// Number of valid bytes in `data` (a valid CAN or CAN FD length).
  uint8_t size{0U};
  /// Frame data; only the first `size` bytes are valid.
  std::array<std::byte, kCanFdMtu> data{};
};

/**
 * @brief One CAN interface of the node: a CAN controller, a SocketCAN device
 *        or a serial-line CAN adapter.
 *
 * Both calls are non-blocking and run in the Cyphal thread. The platform
 * implements this boundary (`hardware/zephyr_*`); the Cyphal layer stays
 * free of driver APIs.
 */
class CanMedia {
 public:
  CanMedia() = default;
  virtual ~CanMedia() = default;

  CanMedia(const CanMedia&) = delete;
  CanMedia& operator=(const CanMedia&) = delete;
  CanMedia(CanMedia&&) = delete;
  CanMedia& operator=(CanMedia&&) = delete;

  /// Largest data length per frame: kCanClassicMtu or kCanFdMtu.
  [[nodiscard]] virtual std::size_t Mtu() const noexcept = 0;

  /**
   * @brief Hands one frame to the interface for transmission.
   * @param frame Frame to send; its size does not exceed Mtu().
   * @return `false` when the interface cannot take the frame now (mailboxes
   *         full, bus off); the transport retries until the deadline.
   */
  [[nodiscard]] virtual bool Push(const CanFrame& frame) noexcept = 0;

  /**
   * @brief Takes one received extended data frame.
   * @param frame Output; valid when the call returns `true`.
   * @return `false` when no frame is pending.
   */
  [[nodiscard]] virtual bool Pop(CanFrame& frame) noexcept = 0;
};

}  // namespace unimoc::cyphal
