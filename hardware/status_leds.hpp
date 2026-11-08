/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file status_leds.hpp
 *    @brief Status LEDs: code running, power stage enabled, error and load warning.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>

namespace unimoc::hardware {

/// Counter that rises whenever the controller missed a deadline (late duty plan, observer overrun).
using OverloadCounterCallback = uint32_t (*)() noexcept;

/**
 * @brief Starts the status LED thread.
 *
 * The LEDs are the optional devicetree aliases `unimoc-led-run`,
 * `unimoc-led-pwm` and `unimoc-led-error`; a board without one just has no
 * such LED. The thread polls at 10 Hz and touches no hot-path state:
 *  - RUN flashes at 1 Hz while the scheduler runs.
 *  - PWM is on while the power stage outputs are released.
 *  - ERROR is on while a power stage fault is latched, and flashes at 5 Hz
 *    for two seconds after the overload counter has risen (high load warning).
 *
 * @param overload_counter Counter polled for the high load warning.
 */
void StartStatusLeds(OverloadCounterCallback overload_counter) noexcept;

}  // namespace unimoc::hardware
