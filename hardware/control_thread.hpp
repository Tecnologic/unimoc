/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file control_thread.hpp
 *    @brief The control thread of the outer loop (speed, position, setpoints).
 *
 *    The observer interrupt wakes the thread every few control cycles; the
 *    thread runs the outer loop of the application, which hands the current
 *    reference to the control cycle through a mailbox (docs/hotpath_plan.md,
 *    M7). The implementation is shared by the Zephyr platforms
 *    (hardware/zephyr_common/control_thread.cpp).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

namespace unimoc::hardware {

/// Starts the control thread. It calls ControlThreadHandler() once per NotifyControlThread().
void StartControlThread() noexcept;

/**
 * @brief Wakes the control thread.
 *
 * Callable from the observer interrupt (a normal-priority interrupt, not
 * zero latency). Wake-ups the thread has not served yet are merged.
 */
void NotifyControlThread() noexcept;

/// Handler of the control thread, defined by the application.
void ControlThreadHandler() noexcept;

}  // namespace unimoc::hardware
