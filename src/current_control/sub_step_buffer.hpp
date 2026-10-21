/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file sub_step_buffer.hpp
 *    @brief Double buffer shared by the control cycle and the observer.
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
#include "rotor_system.hpp"
#include "sin_cos.hpp"
#include "stator_system.hpp"

/**
 * @namespace unimoc global namespace
 */

/**
 * @namespace current_control current-control subsystem namespace
 */
namespace unimoc::current_control {

/// PWM half periods (current samples, duty entries, HFI steps) per control cycle.
inline constexpr uint8_t kNumSubSteps = 4U;

/**
 * @brief Data shared between the control cycle and the observer for one half
 *        of the double buffer.
 *
 * Index k of the sample arrays is the k-th current sample of the cycle (the
 * last one at the control boundary). Index k of the output arrays is duty
 * entry k, which applies to the half period starting k + 1 boundaries after
 * the control boundary.
 *
 * Ownership protocol
 * ------------------
 *  - The observer writes `sc` and `sc_out` into the **inactive** buffer, then
 *    flips `DoubleBuffer::active`.
 *  - The control cycle reads `sc` / `sc_out` of the **active** buffer and
 *    writes `i_ab_samples`, `i_dq_mean` and `u_dq` into it. The observer reads
 *    them after the control cycle signalled `observer_pending`.
 *
 * Because the observer fully prepares the inactive buffer before the atomic
 * flip, the control cycle never observes a half-written angle set.
 */
struct SubStepBuffer {
  /// Park angles of the current samples of the cycle (estimated angle at each sample).
  std::array<system::SinCos<unit::Ratio>, kNumSubSteps> sc{};

  /// Inverse-Park angles of the duty entries (estimated angle in the middle of
  /// the half period each entry applies to, docs/hotpath_plan.md D4).
  std::array<system::SinCos<unit::Ratio>, kNumSubSteps> sc_out{};

  /// Stator-frame (α/β) current samples of the cycle, written by the control cycle.
  std::array<system::Stator<unit::Current>, kNumSubSteps> i_ab_samples{};

  /// Mean rotor-frame current of the cycle, written by the control cycle.
  system::Rotor<unit::Current> i_dq_mean;

  /// Voltage demand of the cycle, rotor frame, written by the control cycle.
  system::Rotor<unit::Voltage> u_dq;

  /// Timestamp of the last sample (the control boundary), written by the
  /// control cycle; the clock of the Hall events (hardware::kTimestampFrequency).
  uint32_t timestamp{0U};
};

/**
 * @brief Ping-pong pair of SubStepBuffers with an atomic active-index.
 *
 * The control cycle uses `buf[active]`. The observer reads that buffer after
 * the cycle, writes the angles of the next cycle into `buf[1 − active]` and
 * then flips `active`.
 *
 * Memory-ordering
 * ---------------
 *  - The observer uses `memory_order_release` on the flip so that all writes
 *    to the inactive buffer are visible to the control cycle, which reads
 *    `active` with `memory_order_acquire`.
 *  - The control cycle preempts the observer. When the observer is late (it
 *    has not flipped when the next cycle starts), the control cycle counts an
 *    overrun and reuses the old angles.
 */
struct DoubleBuffer {
  /// The two ping-pong buffers.
  std::array<SubStepBuffer, 2> buf{};

  /**
   * @brief Index of the buffer used by the next control cycle (0 or 1).
   *
   * Write with `memory_order_release` (observer).
   * Read  with `memory_order_acquire` (control cycle).
   */
  std::atomic<uint8_t> active{0U};
};

}  // namespace unimoc::current_control
