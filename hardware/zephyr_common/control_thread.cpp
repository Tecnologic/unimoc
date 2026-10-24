/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file control_thread.cpp
 *    @brief Zephyr thread of the outer loop, woken by the observer interrupt.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "control_thread.hpp"
#include <zephyr/kernel.h>

namespace unimoc::hardware {

// NOLINTBEGIN(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay):
// K_THREAD_STACK_DEFINE and K_SEM_DEFINE declare C arrays inside the Zephyr macros.

namespace {

K_THREAD_STACK_DEFINE(g_control_stack, CONFIG_UNIMOC_CONTROL_THREAD_STACK_SIZE);
struct k_thread g_control_thread;
// Limit 1: wake-ups the thread has not served yet are merged.
K_SEM_DEFINE(g_control_wake, 0, 1);

void ControlThread(void* /*unused*/, void* /*unused*/, void* /*unused*/) {
  while (true) {
    (void)k_sem_take(&g_control_wake, K_FOREVER);
    ControlThreadHandler();
  }
}

}  // namespace

void StartControlThread() noexcept {
  k_thread_create(&g_control_thread,
                  g_control_stack,
                  K_THREAD_STACK_SIZEOF(g_control_stack),
                  ControlThread,
                  nullptr,
                  nullptr,
                  nullptr,
                  CONFIG_UNIMOC_CONTROL_THREAD_PRIORITY,
                  K_FP_REGS,
                  K_NO_WAIT);
  k_thread_name_set(&g_control_thread, "control");
}

void NotifyControlThread() noexcept { k_sem_give(&g_control_wake); }

// NOLINTEND(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay)

}  // namespace unimoc::hardware
