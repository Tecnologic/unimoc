/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file board_inputs_interface.hpp
 *    @brief Platform boundary of the board inputs: power board detect, gate driver supply, accelerator
 *           and brake lever (thread context only).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <optional>
#include "units.hpp"

namespace unimoc::hardware {

/**
 * @brief The slow inputs of a board, read by the drive thread.
 *
 * Every input is optional: a board without it reports std::nullopt, and the
 * drive does not gate on it (the NUCLEO-H743ZI has none of them).
 */
struct BoardInputs {
  /// The power board is plugged in (Jacobi: PWR_DET on PE4, low = present).
  std::optional<bool> power_board;
  /// Gate driver supply +12P in front of its divider (Jacobi: PC5).
  std::optional<unit::Voltage> gate_supply;
  /// Output voltage of the accelerator (throttle) sensor (Jacobi: AIN_ACC on PC0, 0 to 3.3 V).
  std::optional<unit::Voltage> accelerator;
  /// Output voltage of the brake lever sensor (Jacobi: AIN_DCC on PC4, 0 to 3.3 V).
  std::optional<unit::Voltage> brake;
};

/**
 * @brief Limits of the gate driver supply of the board (devicetree `gate-supply-millivolts`).
 *
 * Below @c engage the drive does not engage; below @c trip it switches an
 * engaged power stage off with a latched fault. trip < engage gives the hysteresis.
 */
struct GateSupplyLimits {
  unit::Voltage trip;
  unit::Voltage engage;
};

/// Reads the board inputs; thread context. Analog inputs come from the injected sequences (valid once the
/// control runs, zero before).
[[nodiscard]] BoardInputs ReadBoardInputs() noexcept;

/// The gate supply limits of the board; meaningful only when ReadBoardInputs() reports a gate supply.
[[nodiscard]] GateSupplyLimits GetGateSupplyLimits() noexcept;

/// Configures the input pins; thread context, once, before the first ReadBoardInputs().
void StartBoardInputs() noexcept;

}  // namespace unimoc::hardware
