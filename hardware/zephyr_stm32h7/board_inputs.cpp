/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file board_inputs.cpp
 *    @brief Board inputs of the STM32H7 platform.
 *
 *    The power board detect is the optional power-board-gpios of zephyr,user
 *    (Jacobi: PWR_DET on PE4, active low). The gate driver supply, the
 *    accelerator and the brake lever are optional io-channels of foc0
 *    ("gate-supply", "accelerator", "brake"), converted with the injected
 *    sequences; the drive thread reads their last result. Not run on
 *    hardware yet.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include "analog_scaling.hpp"
#include "board_inputs_interface.hpp"
#include "foc_stm32h7.hpp"
#include "foc_stm32h7_dt.hpp"

namespace unimoc::hardware {

namespace {

using unit::operator""_V;

constexpr bool kHasPowerBoardDetect = DT_NODE_HAS_PROP(DT_PATH(zephyr_user), power_board_gpios);
const struct gpio_dt_spec kPowerBoardDetect =
    GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), power_board_gpios, {});

// Divider of the gate supply (foc0 gate-supply-divider-ohms).
constexpr float kGateSupplyDividerRatio =
    static_cast<float>(foc_stm32h7::kGateSupplyDividerTopOhm + foc_stm32h7::kGateSupplyDividerBottomOhm) /
    static_cast<float>(foc_stm32h7::kGateSupplyDividerBottomOhm);

/// Voltage at the ADC pin: the accelerator and brake sensors run from the ADC reference, without a divider.
unit::Voltage PinVoltage(const uint32_t raw) noexcept {
  return 1.0_V * analog::CountsToVolts(static_cast<float>(raw));
}

}  // namespace

BoardInputs ReadBoardInputs() noexcept {
  BoardInputs inputs{};
  if constexpr (kHasPowerBoardDetect) {
    const int kLevel = gpio_pin_get_dt(&kPowerBoardDetect);  // logical: 1 = present (active low)
    inputs.power_board = kLevel > 0;
  }
  const foc_stm32h7::RawSamples kRaw = foc_stm32h7::ReadRaw();
  if constexpr (foc_stm32h7::kHasGateSupply) {
    inputs.gate_supply = PinVoltage(kRaw.gate_supply) * kGateSupplyDividerRatio;
  }
  if constexpr (foc_stm32h7::kHasAccelerator) {
    inputs.accelerator = PinVoltage(kRaw.accelerator);
  }
  if constexpr (foc_stm32h7::kHasBrake) {
    inputs.brake = PinVoltage(kRaw.brake);
  }
  return inputs;
}

GateSupplyLimits GetGateSupplyLimits() noexcept {
  return GateSupplyLimits{0.001_V * static_cast<float>(foc_stm32h7::kGateSupplyTripMillivolts),
                          0.001_V * static_cast<float>(foc_stm32h7::kGateSupplyEngageMillivolts)};
}

void StartBoardInputs() noexcept {
  if constexpr (kHasPowerBoardDetect) {
    // The pull-up is on the board (Jacobi: R612); a read error reads as "missing".
    (void)gpio_pin_configure_dt(&kPowerBoardDetect, GPIO_INPUT);
  }
}

}  // namespace unimoc::hardware
