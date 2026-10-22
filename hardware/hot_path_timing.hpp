/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file hot_path_timing.hpp
 *    @brief Execution-time statistics of the interrupts (milestone M0).
 *
 *    The platform measures the sample interrupt, the control cycle and the
 *    observer in cycles of its timestamp clock (BCC: the DWT cycle counter)
 *    when CONFIG_UNIMOC_HOT_PATH_TIMING is enabled, and records them here.
 *    The Cyphal thread reads them (`unimoc.timing.*` registers).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>

/**
 * @namespace unimoc::hardware::timing
 * @brief Execution times of the hot path, measured by the platform.
 */
namespace unimoc::hardware::timing {

/// Execution times of one section since the last reset, in timestamp-clock cycles.
struct Stats {
  uint32_t min_cycles{std::numeric_limits<uint32_t>::max()};  ///< max() until the first record
  uint32_t max_cycles{0U};
  uint64_t sum_cycles{0U};
  uint32_t count{0U};  ///< Records since the reset; wraps after 2^32.

  /// Mean execution time; 0 without a record.
  [[nodiscard]] constexpr uint32_t MeanCycles() const noexcept {
    return (count == 0U) ? 0U : static_cast<uint32_t>(sum_cycles / count);
  }
};

/**
 * @brief Statistics of one section, written by one interrupt, read by a thread.
 *
 * Record() runs in the measured interrupt: no locks, no kernel calls, a fixed
 * handful of instructions. Read() retries while a record overlaps its copy
 * (the writer preempts the reader and finishes first). Reset() is a request
 * the writer carries out at its next record, so the writer stays the only one
 * that changes the statistics.
 */
class Probe {
 public:
  /// Adds one execution time (writer, the measured interrupt).
  void Record(const uint32_t cycles) noexcept {
    if (reset_requested_.load(std::memory_order_acquire)) {
      reset_requested_.store(false, std::memory_order_relaxed);
      accumulator_ = Stats{};
    }
    accumulator_.min_cycles = std::min(accumulator_.min_cycles, cycles);
    accumulator_.max_cycles = std::max(accumulator_.max_cycles, cycles);
    accumulator_.sum_cycles += cycles;
    ++accumulator_.count;

    const uint32_t kSequence = sequence_.load(std::memory_order_relaxed);
    sequence_.store(kSequence + 1U, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    published_ = accumulator_;
    std::atomic_thread_fence(std::memory_order_release);
    sequence_.store(kSequence + 2U, std::memory_order_relaxed);
  }

  /// Consistent copy of the statistics (reader, thread context).
  [[nodiscard]] Stats Read() const noexcept {
    while (true) {
      const uint32_t kBefore = sequence_.load(std::memory_order_acquire);
      if ((kBefore & 1U) != 0U) {
        continue;  // only possible on a second core; here the writer has finished
      }
      const Stats kCopy = published_;
      std::atomic_thread_fence(std::memory_order_acquire);
      if (sequence_.load(std::memory_order_relaxed) == kBefore) {
        return kCopy;
      }
    }
  }

  /// Starts new statistics with the next record (any context).
  void Reset() noexcept { reset_requested_.store(true, std::memory_order_release); }

 private:
  Stats accumulator_{};
  Stats published_{};
  std::atomic<uint32_t> sequence_{0U};
  std::atomic<bool> reset_requested_{false};
};

/// Measured sections.
enum class Section : uint8_t {
  kSampleIsr,     ///< Whole sample interrupt of a control boundary (includes the control cycle).
  kControlCycle,  ///< ControlCycleHandler() alone.
  kObserver,      ///< ObserverHandler() alone.
};

/// Number of measured sections.
inline constexpr std::size_t kSectionCount = 3U;

/// The probe of a section; the platform records, the application reads.
[[nodiscard]] inline Probe& ProbeOf(const Section section) noexcept {
  static constinit std::array<Probe, kSectionCount> probes{};
  return probes[static_cast<std::size_t>(section)];
}

/// Duty plans the hardware did not apply in time since start (BCC: the DMA burst ran ahead); 0 without such a check.
[[nodiscard]] uint32_t DutyOverruns() noexcept;

}  // namespace unimoc::hardware::timing
