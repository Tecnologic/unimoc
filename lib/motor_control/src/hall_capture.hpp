/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file hall_capture.hpp
 *    @brief Lock-free queue of captured Hall code changes.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

/**
 * @namespace unimoc::observer observer algorithms namespace
 */
namespace unimoc::observer {

/**
 * @brief One change of the Hall code with the time it was captured.
 *
 * The code has bit 0 = Hall A, bit 1 = Hall B, bit 2 = Hall C. The timestamp
 * counts ticks of the platform timestamp clock (BCC: the DWT cycle counter,
 * native_sim: nanoseconds of virtual time) and wraps; only differences of
 * timestamps are meaningful.
 */
struct HallEvent {
  /// Hall code after the change.
  uint8_t code{0U};
  /// Timestamp of the change in platform ticks.
  uint32_t timestamp{0U};
};

/**
 * @brief Single-producer, single-consumer queue of Hall events.
 *
 * The edge interrupt pushes, the observer pops. Neither side blocks or takes
 * a lock, so either may preempt the other. When the queue is full the newest
 * event is dropped and counted; the observer then still sees the newest code
 * through LatestCode().
 *
 * @tparam N Capacity; a power of two.
 */
template <std::size_t N = 16U>
class HallEventQueue {
  static_assert(N >= 2U && (N & (N - 1U)) == 0U, "the capacity is a power of two");

 public:
  /**
   * @brief Adds an event (producer side).
   * @param event Code and timestamp of the change.
   * @return False when the queue was full and the event was dropped.
   */
  bool Push(const HallEvent& event) noexcept {
    latest_code_.store(event.code, std::memory_order_relaxed);
    const uint32_t kHead = head_.load(std::memory_order_relaxed);
    const uint32_t kTail = tail_.load(std::memory_order_acquire);
    if (kHead - kTail >= N) {
      overflows_.fetch_add(1U, std::memory_order_relaxed);
      return false;
    }
    events_[kHead & (N - 1U)] = event;
    head_.store(kHead + 1U, std::memory_order_release);
    return true;
  }

  /**
   * @brief Removes the oldest event (consumer side).
   * @param event Receives the event.
   * @return False when the queue is empty.
   */
  bool Pop(HallEvent& event) noexcept {
    const uint32_t kTail = tail_.load(std::memory_order_relaxed);
    const uint32_t kHead = head_.load(std::memory_order_acquire);
    if (kHead == kTail) {
      return false;
    }
    event = events_[kTail & (N - 1U)];
    tail_.store(kTail + 1U, std::memory_order_release);
    return true;
  }

  /// Drops all queued events (consumer side).
  void Clear() noexcept { tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release); }

  /// Code of the newest pushed event, also of a dropped one.
  [[nodiscard]] uint8_t LatestCode() const noexcept { return latest_code_.load(std::memory_order_relaxed); }

  /// Events dropped because the queue was full.
  [[nodiscard]] uint32_t Overflows() const noexcept { return overflows_.load(std::memory_order_relaxed); }

 private:
  std::array<HallEvent, N> events_{};
  std::atomic<uint32_t> head_{0U};
  std::atomic<uint32_t> tail_{0U};
  std::atomic<uint8_t> latest_code_{0U};
  std::atomic<uint32_t> overflows_{0U};
};

}  // namespace unimoc::observer
