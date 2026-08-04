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

#ifndef UNIMOC_CURRENT_CONTROL_SUB_STEP_BUFFER_H_
#define UNIMOC_CURRENT_CONTROL_SUB_STEP_BUFFER_H_

#include <atomic>
#include <cstdint>
#include "SinCos.hpp"
#include "StatorReference.hpp"

/**
 * @namespace unimoc global namespace
 */
namespace unimoc
{
/**
 * @namespace current_control current-control subsystem namespace
 */
namespace current_control
{

/// Number of HFI sub-steps per slow-update cycle.
inline constexpr uint8_t NUM_SUB_STEPS = 4u;

/**
 * @brief Data shared between the current-control ISR and the slow-update task
 *        for one half of the double buffer.
 *
 * Each element corresponds to one of the @p NUM_SUB_STEPS consecutive PWM
 * half-periods that make up one slow-update cycle.
 *
 * Layout
 * ------
 *  - **sc**            Pre-computed sin/cos for the Park transform at each
 *                      sub-step.  Written by the slow-update task; read by
 *                      the ISR.
 *  - **i_ab_samples**  Clarke-transformed stator current measured by the ISR
 *                      at each sub-step.  Written by the ISR; read by the
 *                      slow-update task.
 *
 * Ownership protocol
 * ------------------
 * Only one side writes each array:
 *  - SlowUpdate writes `sc[0..3]` into the **inactive** buffer, then flips
 *    `DoubleBuffer::active`.
 *  - The ISR writes `i_ab_samples[sub_step]` into the **active** buffer.
 *
 * Because SlowUpdate fully prepares the inactive buffer before the atomic
 * flip, the ISR never observes a half-written `sc` set.
 */
struct SubStepBuffer
{
    /// Pre-computed sin/cos for each sub-step Park transform.
    system::SinCos<float> sc[NUM_SUB_STEPS]{};

    /// Stator-frame (α/β) current samples recorded by the ISR at each sub-step.
    system::StatorReference<float> i_ab_samples[NUM_SUB_STEPS]{};
};

/**
 * @brief Ping-pong pair of SubStepBuffers with an atomic active-index.
 *
 * The ISR reads from `buf[active]` and writes current samples into it.
 * The slow-update task writes new sin/cos values into `buf[1 − active]`
 * and then atomically flips `active`.
 *
 * Memory-ordering
 * ---------------
 *  - The slow-update task uses `memory_order_release` on the flip so that all
 *    writes to the inactive buffer are visible to any thread that subsequently
 *    reads `active` with `memory_order_acquire`.
 *  - The ISR uses `memory_order_acquire` when reading `active` at the start
 *    of each sub-step.
 *
 * The ISR is not preempted by the slow-update task (it runs at a higher IRQ
 * priority), so there is no race on `i_ab_samples`: the ISR completes all
 * four writes within the same set of four consecutive half-periods; the
 * slow-update task only reads them after `samples_ready` is set (at the end
 * of sub-step 3), by which time the ISR has moved back to sub-step 0 and is
 * writing into the same buffer again — the slow-update task reads the
 * *previous* four samples.
 */
struct DoubleBuffer
{
    /// The two ping-pong buffers.
    SubStepBuffer buf[2]{};

    /**
     * @brief Index of the buffer currently in use by the ISR (0 or 1).
     *
     * Write with `memory_order_release` (slow-update task).
     * Read  with `memory_order_acquire` (ISR, beginning of each sub-step cycle).
     */
    std::atomic<uint8_t> active{0u};
};

}  // namespace current_control
}  // namespace unimoc

#endif /* UNIMOC_CURRENT_CONTROL_SUB_STEP_BUFFER_H_ */
