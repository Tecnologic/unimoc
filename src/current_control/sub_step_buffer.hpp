/*
       __  ___   ________  _______  ______
      / / / / | / /  _/  |/  / __ \/ ____/
     / / / /  |/ // // /|_/ / / / / /
    / /_/ / /|  // // /  / / /_/ / /___
    \____/_/ |_/___/_/  /_/\____/\____/

    Universal Motor Control  2026 Alexander <tecnologic86@gmail.com> Evers

    This file is part of UNIMOC.

    UNIMOC is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
