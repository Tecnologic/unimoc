/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file mailbox.hpp
 *    @brief Lock-free hand-over of a value between two priority levels.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <type_traits>

/**
 * @namespace current_control current-control subsystem namespace
 */
namespace unimoc::current_control {

/**
 * @brief Double buffer from a lower-priority writer to a higher-priority reader.
 *
 * The writer fills the inactive slot and flips the index; the reader copies
 * the active slot. The reader always preempts the writer (control cycle over
 * the control thread, control thread over the Cyphal thread), so a read is
 * never interrupted by a write and never sees a half-written value. Neither
 * side blocks, which a zero-latency reader requires. One writer only.
 *
 * @tparam T Trivially copyable value.
 */
template <typename T>
class Mailbox {
  static_assert(std::is_trivially_copyable_v<T>);

 public:
  /// Publishes a value (writer).
  void Write(const T& value) noexcept {
    const auto kNext = static_cast<uint8_t>(1U - active_.load(std::memory_order_relaxed));
    slots_[kNext] = value;
    active_.store(kNext, std::memory_order_release);
  }

  /// Returns the last published value (reader).
  [[nodiscard]] T Read() const noexcept { return slots_[active_.load(std::memory_order_acquire)]; }

 private:
  std::array<T, 2> slots_{};
  std::atomic<uint8_t> active_{0U};
};

/**
 * @brief Sequence lock from a higher-priority writer to a lower-priority reader.
 *
 * The writer (the observer interrupt) never waits. The reader (the control
 * thread) retries when a write overlapped its copy; it cannot starve, the
 * writer runs once per control cycle and a copy takes far less. One writer only.
 *
 * @tparam T Trivially copyable value.
 */
template <typename T>
class SeqLock {
  static_assert(std::is_trivially_copyable_v<T>);

 public:
  /// Publishes a value (writer).
  void Write(const T& value) noexcept {
    const uint32_t kSequence = sequence_.load(std::memory_order_relaxed);
    sequence_.store(kSequence + 1U, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    value_ = value;
    std::atomic_thread_fence(std::memory_order_release);
    sequence_.store(kSequence + 2U, std::memory_order_relaxed);
  }

  /// Returns a consistent copy of the last published value (reader).
  [[nodiscard]] T Read() const noexcept {
    while (true) {
      const uint32_t kBefore = sequence_.load(std::memory_order_acquire);
      if ((kBefore & 1U) != 0U) {
        continue;  // only possible on a second core; here the writer has finished
      }
      const T kCopy = value_;
      std::atomic_thread_fence(std::memory_order_acquire);
      if (sequence_.load(std::memory_order_relaxed) == kBefore) {
        return kCopy;
      }
    }
  }

 private:
  T value_{};
  std::atomic<uint32_t> sequence_{0U};
};

}  // namespace unimoc::current_control
