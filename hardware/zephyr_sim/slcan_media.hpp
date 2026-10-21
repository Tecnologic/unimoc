/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file slcan_media.hpp
 *    @brief Serial-line CAN interface (Lawicel/slcan) on native_sim pseudo terminals.
 *
 *    The simulated drive plays a USB-serial CAN adapter on several pseudo
 *    terminals, so pycyphal and yakut reach it with their python-can slcan
 *    media, without SocketCAN (WSL2 kernels without vcan). A pseudo terminal
 *    serves one process; the ports form one bus: a frame a tool sends on one
 *    port reaches the drive and the tools on the other ports. Classic CAN
 *    frames only (pycyphal runs slcan without CAN FD).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <zephyr/device.h>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include "cyphal/can_media.hpp"

namespace unimoc::hardware::sim {

/**
 * @brief Lawicel ASCII protocol on polled UARTs, as one CAN interface.
 *
 * Host to node: `O`/`L` open, `C` close, `Tiiiiiiiidhh..` extended frames
 * (`D`/`B` CAN FD frames are accepted as well); other commands are
 * acknowledged with CR. Node to host while open: `Tiiiiiiiidhh..\r`.
 * Frames for a closed port are dropped, so an unconnected pseudo terminal
 * does not fill up; a reader that falls behind loses whole frames, as on an
 * overloaded bus, never a part of a line.
 */
class SlcanMedia final : public cyphal::CanMedia {
 public:
  /// Largest number of pseudo terminals.
  static constexpr std::size_t kMaxPorts = 4U;
  /// Received frames buffered between two Pop() calls.
  static constexpr std::size_t kRxQueueLength = 16U;
  /// Longest line: 'D' + 8 id digits + DLC + 128 data digits.
  static constexpr std::size_t kMaxLine = 140U;
  /// Characters read per port and Pop() call at most.
  static constexpr std::size_t kMaxCharsPerPop = 1024U;

  /**
   * @brief Takes the UARTs of the ports; devices that are not ready are skipped.
   * @return The number of usable ports.
   */
  std::size_t Init(std::span<const struct device* const> uarts) noexcept;

  [[nodiscard]] std::size_t Mtu() const noexcept override { return cyphal::kCanClassicMtu; }
  [[nodiscard]] bool Push(const cyphal::CanFrame& frame) noexcept override;
  [[nodiscard]] bool Pop(cyphal::CanFrame& frame) noexcept override;

  /// Result of one host line.
  struct LineResult {
    std::string_view reply;                 ///< Reply to the host (CR, BEL), empty for none.
    std::optional<cyphal::CanFrame> frame;  ///< Frame the host sent.
    std::optional<bool> open;               ///< New open state of the port.
  };

  /// Parses one host line (without the CR).
  [[nodiscard]] static LineResult ParseLine(std::string_view line) noexcept;

  /// Formats a frame as an slcan line with its CR; returns the length.
  [[nodiscard]] static std::size_t FormatFrame(const cyphal::CanFrame& frame, std::span<char> text) noexcept;

 private:
  struct Port {
    const struct device* uart{nullptr};
    bool open{false};
    std::array<char, kMaxLine> line{};
    std::size_t line_length{0U};
    bool line_overflow{false};
    /// Rest of a line the pseudo terminal did not take; sent before anything else.
    std::array<char, kMaxLine> pending{};
    std::size_t pending_length{0U};
  };

  void Poll(std::size_t index) noexcept;
  /// Handles the complete line of a port: reply, open state, received frame.
  void Complete(std::size_t index) noexcept;
  void Send(std::size_t index, const cyphal::CanFrame& frame) noexcept;
  /// Writes whole lines only: a full pseudo terminal drops the next lines, never a part of one.
  static void Write(Port& port, std::string_view text) noexcept;
  /// Sends the pending rest of a line; false while some of it is still pending.
  static bool Flush(Port& port) noexcept;

  std::array<Port, kMaxPorts> ports_{};
  std::size_t port_count_{0U};
  std::array<cyphal::CanFrame, kRxQueueLength> rx_{};
  std::size_t rx_head_{0U};
  std::size_t rx_count_{0U};
};

}  // namespace unimoc::hardware::sim
