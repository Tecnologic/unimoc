/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file board_inputs.cpp
 *    @brief Board inputs of the native_sim platform: set by the tests and the `unimoc.sim.*` registers.
 *
 *    The limits of the gate supply are those of Jacobi (unimoc_jacobi.dts).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include <zephyr/kernel.h>
#include "board_inputs_interface.hpp"
#include "sim_board.hpp"

namespace unimoc::hardware {

namespace {

using unit::operator""_V;

// The Cyphal thread reads, a test or the registers write.
struct k_spinlock g_lock;
BoardInputs g_inputs = sim::DefaultBoardInputs();

}  // namespace

BoardInputs ReadBoardInputs() noexcept { return sim::SimBoardInputs(); }

GateSupplyLimits GetGateSupplyLimits() noexcept { return GateSupplyLimits{10.0_V, 10.5_V}; }

void StartBoardInputs() noexcept {}

namespace sim {

// The accelerator and the brake lever at the released voltage of the factory calibration.
BoardInputs DefaultBoardInputs() noexcept { return BoardInputs{true, 12.0_V, 0.8_V, 0.8_V}; }

void SetBoardInputs(const BoardInputs& inputs) noexcept {
  const k_spinlock_key_t kKey = k_spin_lock(&g_lock);
  g_inputs = inputs;
  k_spin_unlock(&g_lock, kKey);
}

BoardInputs SimBoardInputs() noexcept {
  const k_spinlock_key_t kKey = k_spin_lock(&g_lock);
  const BoardInputs kInputs = g_inputs;
  k_spin_unlock(&g_lock, kKey);
  return kInputs;
}

}  // namespace sim

}  // namespace unimoc::hardware
