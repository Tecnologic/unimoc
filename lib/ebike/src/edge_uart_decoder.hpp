/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file edge_uart_decoder.hpp
 *    @brief Software UART receiver (8N1) from timestamped edges of the line.
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

/**
 * @namespace unimoc::ebike e-bike application logic namespace
 */
namespace unimoc::ebike {

/// One edge of a serial line: the level after the edge and the time it was seen.
struct UartEdge {
  /// Timestamp in ticks of a free-running 32-bit counter (wraps).
  uint32_t ticks{0U};
  /// Level of the line after the edge (true: high, the idle level).
  bool level{true};
};

/**
 * @brief Decodes 8N1 serial bytes (LSB first, idle high) from the edges of a line.
 *
 * For a line that is not on a UART receive pin: the display receive line of the BCC sits on PB15, which has
 * no UART function. The platform stamps every edge of the pin in an interrupt (a counter that runs at a known
 * rate, at 9600 baud a bit lasts about 104 microseconds, so interrupt jitter of some microseconds does not
 * matter) and this class reconstructs the bytes in a thread.
 *
 * Each frame is sampled in the middle of its bits, timed from its own start edge, so the baud rate only has
 * to match within a few percent over the ten bits. A start bit that is no longer low in its middle is a
 * glitch and ignored. A frame whose stop bit is not high is dropped (framing error) and counted.
 *
 * The last bits of a frame can lie after the last edge (a byte that ends in idle-level bits), so the platform
 * also calls Poll() with the current time; it completes the frames whose sample times have passed.
 *
 * Call Push() and Poll() in time order from one thread (the platform copies the edges out of the interrupt).
 */
class EdgeUartDecoder {
 public:
  /// Received bytes the decoder holds until Pop().
  static constexpr std::size_t kFifoSize = 32U;

  /**
   * @param ticks_per_bit Counter ticks of one bit (counter rate / baud rate).
   */
  explicit EdgeUartDecoder(const uint32_t ticks_per_bit) noexcept : ticks_per_bit_(ticks_per_bit) {}

  /**
   * @brief Feeds the next edge.
   * @param edge Edge in time order.
   */
  void Push(const UartEdge edge) noexcept {
    SampleUntil(edge.ticks);
    if (!in_frame_ && level_ && !edge.level) {
      in_frame_ = true;
      start_ticks_ = edge.ticks;
      bit_ = 0U;
      data_ = 0U;
    }
    level_ = edge.level;
  }

  /**
   * @brief Completes the frames whose bits have all been on the line by @p now.
   * @param now Current value of the counter.
   */
  void Poll(const uint32_t now) noexcept { SampleUntil(now); }

  /**
   * @brief Takes the oldest received byte.
   * @param byte Receives the byte.
   * @return true when a byte was available.
   */
  bool Pop(uint8_t& byte) noexcept {
    if (count_ == 0U) {
      return false;
    }
    byte = fifo_[read_];
    read_ = (read_ + 1U) % kFifoSize;
    --count_;
    return true;
  }

  /// Frames dropped for a stop bit that was not high.
  [[nodiscard]] uint32_t FramingErrors() const noexcept { return framing_errors_; }
  /// Bytes dropped because the FIFO was full.
  [[nodiscard]] uint32_t Overruns() const noexcept { return overruns_; }

 private:
  static constexpr unsigned kStopBit = 9U;

  // Takes the samples of the open frame whose time lies before @p until, with the level the line had.
  void SampleUntil(const uint32_t until) noexcept {
    while (in_frame_) {
      // Middle of bit k: start + (k + 0.5) * ticks per bit.
      const uint32_t kSampleAt = start_ticks_ + (((2U * bit_) + 1U) * ticks_per_bit_) / 2U;
      if (static_cast<int32_t>(until - kSampleAt) <= 0) {
        return;
      }
      if (bit_ == 0U) {
        if (level_) {
          in_frame_ = false;  // not low in the middle of the start bit: a glitch
          return;
        }
      } else if (bit_ < kStopBit) {
        data_ = static_cast<uint8_t>(data_ | ((level_ ? 1U : 0U) << (bit_ - 1U)));
      } else {
        in_frame_ = false;
        if (level_) {
          Store(data_);
        } else {
          ++framing_errors_;
        }
        return;
      }
      ++bit_;
    }
  }

  void Store(const uint8_t byte) noexcept {
    if (count_ == kFifoSize) {
      ++overruns_;
      return;
    }
    fifo_[(read_ + count_) % kFifoSize] = byte;
    ++count_;
  }

  uint32_t ticks_per_bit_;
  bool level_{true};
  bool in_frame_{false};
  uint32_t start_ticks_{0U};
  unsigned bit_{0U};
  uint8_t data_{0U};

  std::array<uint8_t, kFifoSize> fifo_{};
  std::size_t read_{0U};
  std::size_t count_{0U};
  uint32_t framing_errors_{0U};
  uint32_t overruns_{0U};
};

}  // namespace unimoc::ebike
