/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file vdc_gear_io.hpp
 *    @brief Gear buttons and their lamps on the GPIOs of the VDC.
 *
 *    The pins come from the devicetree node `zephyr,user`:
 *    `gear-button-gpios` and `gear-lamp-gpios`, five entries each in the
 *    order R, N, 1, 2, 3 (docs/vdc_hardware_plan.md). A board without them
 *    reads no button and drives no lamp. Shared by the STM32H7 and
 *    native_sim (gpio_emul).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include "gear_buttons.hpp"

namespace unimoc::hardware {

/// Configures the button inputs and the lamp outputs; false when a configured pin failed.
[[nodiscard]] bool InitGearIo() noexcept;

/// The raw button states, true while pressed (the active level comes from the devicetree flags).
[[nodiscard]] vehicle::GearButtonStates ReadGearButtons() noexcept;

/// Switches the lamps.
void WriteGearLamps(const vehicle::PerGearButton<bool>& lamps) noexcept;

}  // namespace unimoc::hardware
