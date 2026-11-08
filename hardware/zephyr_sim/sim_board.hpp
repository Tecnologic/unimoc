/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file sim_board.hpp
 *    @brief The simulated board inputs of native_sim (`unimoc.sim.power_board`, `gate_supply`,
 *           `accelerator`, `brake`).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include "board_inputs_interface.hpp"

namespace unimoc::hardware::sim {

/// What ReadBoardInputs() returns from now on. At the start: power board present, 12 V gate supply,
/// accelerator and brake at 0.8 V (released, factory calibration).
void SetBoardInputs(const BoardInputs& inputs) noexcept;

/// The simulated board inputs.
[[nodiscard]] BoardInputs SimBoardInputs() noexcept;

/// The simulated board inputs at the start.
[[nodiscard]] BoardInputs DefaultBoardInputs() noexcept;

}  // namespace unimoc::hardware::sim
