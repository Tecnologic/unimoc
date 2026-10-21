/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file slcan_media.cpp
 *    @brief Serial-line CAN interface (Lawicel/slcan) on native_sim pseudo terminals.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "slcan_media.hpp"
#include <zephyr/drivers/uart.h>

namespace unimoc::hardware::sim {

namespace {

constexpr std::string_view kOk{"\r"};
constexpr std::string_view kError{"\a"};
constexpr std::array<char, 16>
    kHex{'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
/// Data length of the CAN FD DLC codes.
constexpr std::array<uint8_t, 16>
    kDlcToLength{0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};
/// Longest slcan line of a classic frame: 'T', 8 id digits, DLC, 16 data digits, CR.
constexpr std::size_t kMaxFrameText = 1U + 8U + 1U + (2U * cyphal::kCanClassicMtu) + 1U;

[[nodiscard]] std::optional<uint32_t> ParseHex(const std::string_view digits) noexcept {
  uint32_t value = 0U;
  for (const char kDigit : digits) {
    uint32_t nibble = 0U;
    if (kDigit >= '0' && kDigit <= '9') {
      nibble = static_cast<uint32_t>(kDigit - '0');
    } else if (kDigit >= 'A' && kDigit <= 'F') {
      nibble = static_cast<uint32_t>(kDigit - 'A') + 10U;
    } else if (kDigit >= 'a' && kDigit <= 'f') {
      nibble = static_cast<uint32_t>(kDigit - 'a') + 10U;
    } else {
      return std::nullopt;
    }
    value = (value << 4U) | nibble;
  }
  return value;
}

}  // namespace

std::size_t SlcanMedia::Init(const std::span<const struct device* const> uarts) noexcept {
  port_count_ = 0U;
  for (const struct device* const kUart : uarts) {
    if (kUart != nullptr && device_is_ready(kUart) && port_count_ < kMaxPorts) {
      ports_[port_count_++].uart = kUart;
    }
  }
  return port_count_;
}

SlcanMedia::LineResult SlcanMedia::ParseLine(const std::string_view line) noexcept {
  LineResult result{};
  if (line.empty()) {
    result.reply = kOk;
    return result;
  }
  switch (line[0]) {
    case 'O':
    case 'L':
      result.open = true;
      result.reply = kOk;
      return result;
    case 'C':
      result.open = false;
      result.reply = kOk;
      return result;
    case 'T':
    case 'D':
    case 'B': {
      // Extended frame: 8 id digits, one DLC digit, the data digits.
      result.reply = kError;
      if (line.size() < 10U) {
        return result;
      }
      const auto kId = ParseHex(line.substr(1U, 8U));
      const auto kDlc = ParseHex(line.substr(9U, 1U));
      if (!kId.has_value() || !kDlc.has_value() || (line[0] == 'T' && *kDlc > 8U)) {
        return result;
      }
      cyphal::CanFrame frame{};
      frame.extended_can_id = *kId & 0x1FFFFFFFU;
      frame.size = kDlcToLength[*kDlc];
      if (line.size() < 10U + (2U * frame.size)) {
        return result;
      }
      for (std::size_t i = 0U; i < frame.size; ++i) {
        const auto kByte = ParseHex(line.substr(10U + (2U * i), 2U));
        if (!kByte.has_value()) {
          return result;
        }
        frame.data[i] = static_cast<std::byte>(*kByte);
      }
      result.frame = frame;
      result.reply = {};  // no acknowledgement: python-can does not expect one
      return result;
    }
    case 't':
    case 'r':
    case 'R':
      return result;  // standard and remote frames are not used by Cyphal/CAN
    default:
      result.reply = kOk;  // bit rate, version, timestamp and status commands
      return result;
  }
}

std::size_t SlcanMedia::FormatFrame(const cyphal::CanFrame& frame, const std::span<char> text) noexcept {
  if (text.size() < kMaxFrameText) {
    return 0U;
  }
  std::size_t length = 0U;
  text[length++] = 'T';
  for (uint32_t shift = 32U; shift > 0U; shift -= 4U) {
    text[length++] = kHex[(frame.extended_can_id >> (shift - 4U)) & 0xFU];
  }
  const std::size_t kSize = (frame.size > cyphal::kCanClassicMtu) ? cyphal::kCanClassicMtu : frame.size;
  text[length++] = kHex[kSize];
  for (std::size_t i = 0U; i < kSize; ++i) {
    const auto kByte = static_cast<uint8_t>(frame.data[i]);
    text[length++] = kHex[kByte >> 4U];
    text[length++] = kHex[kByte & 0xFU];
  }
  text[length++] = '\r';
  return length;
}

bool SlcanMedia::Flush(Port& port) noexcept {
  if (port.pending_length == 0U) {
    return true;
  }
  const int kWritten = uart_fifo_fill(port.uart,
                                      reinterpret_cast<const uint8_t*>(port.pending.data()),
                                      static_cast<int>(port.pending_length));
  const std::size_t kDone = (kWritten > 0) ? static_cast<std::size_t>(kWritten) : 0U;
  for (std::size_t i = kDone; i < port.pending_length; ++i) {
    port.pending[i - kDone] = port.pending[i];
  }
  port.pending_length -= kDone;
  return port.pending_length == 0U;
}

void SlcanMedia::Write(Port& port, const std::string_view text) noexcept {
  if (text.empty() || !Flush(port)) {
    return;  // the terminal is full: drop the line
  }
  const int kWritten =
      uart_fifo_fill(port.uart, reinterpret_cast<const uint8_t*>(text.data()), static_cast<int>(text.size()));
  const std::size_t kDone = (kWritten > 0) ? static_cast<std::size_t>(kWritten) : 0U;
  if (kDone == 0U) {
    return;  // nothing of the line went out: dropped whole
  }
  for (std::size_t i = kDone; i < text.size() && port.pending_length < port.pending.size(); ++i) {
    port.pending[port.pending_length++] = text[i];
  }
}

void SlcanMedia::Send(const std::size_t index, const cyphal::CanFrame& frame) noexcept {
  Port& port = ports_[index];
  if (!port.open) {
    return;  // nobody listens: drop instead of filling the pseudo terminal
  }
  std::array<char, kMaxFrameText> text{};
  const std::size_t kLength = FormatFrame(frame, text);
  Write(port, std::string_view{text.data(), kLength});
}

void SlcanMedia::Complete(const std::size_t index) noexcept {
  Port& port = ports_[index];
  const LineResult kResult = ParseLine(std::string_view{port.line.data(), port.line_length});
  if (kResult.open.has_value()) {
    port.open = *kResult.open;
  }
  Write(port, kResult.reply);
  if (!kResult.frame.has_value()) {
    return;
  }
  rx_[(rx_head_ + rx_count_) % kRxQueueLength] = *kResult.frame;
  ++rx_count_;
  // The ports form one bus: the other tools see the frame too.
  for (std::size_t other = 0U; other < port_count_; ++other) {
    if (other != index) {
      Send(other, *kResult.frame);
    }
  }
}

void SlcanMedia::Poll(const std::size_t index) noexcept {
  Port& port = ports_[index];
  (void)Flush(port);
  unsigned char received = 0U;
  for (std::size_t count = 0U; count < kMaxCharsPerPop && rx_count_ < kRxQueueLength; ++count) {
    if (uart_poll_in(port.uart, &received) != 0) {
      break;
    }
    const auto kChar = static_cast<char>(received);
    if (kChar != '\r' && kChar != '\n') {
      if (port.line_length < port.line.size()) {
        port.line[port.line_length++] = kChar;
      } else {
        port.line_overflow = true;
      }
      continue;
    }
    if (!port.line_overflow && port.line_length > 0U) {
      Complete(index);
    }
    port.line_length = 0U;
    port.line_overflow = false;
  }
}

bool SlcanMedia::Pop(cyphal::CanFrame& frame) noexcept {
  for (std::size_t i = 0U; i < port_count_; ++i) {
    Poll(i);
  }
  if (rx_count_ == 0U) {
    return false;
  }
  frame = rx_[rx_head_];
  rx_head_ = (rx_head_ + 1U) % kRxQueueLength;
  --rx_count_;
  return true;
}

bool SlcanMedia::Push(const cyphal::CanFrame& frame) noexcept {
  for (std::size_t i = 0U; i < port_count_; ++i) {
    Send(i, frame);
  }
  return true;
}

}  // namespace unimoc::hardware::sim
