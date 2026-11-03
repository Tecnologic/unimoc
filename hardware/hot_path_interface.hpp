/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file hot_path_interface.hpp
 *    @brief Link-time bound hardware access of the zero-latency hot path.
 *
 *    The zero-latency contexts (sample ISR and control cycle) must not reach
 *    the hardware through function pointers (docs/hotpath_plan.md, M3). The
 *    platform selected by the build (hardware/zephyr_stm32h7, hardware/zephyr_sim)
 *    defines the functions of `hot_path`; the application defines the two
 *    handlers, which the platform calls directly. Everything outside the hot
 *    path keeps using HardwareInterface.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include "hardware_interface.hpp"

namespace unimoc::hardware {

/**
 * @namespace unimoc::hardware::hot_path
 * @brief Hardware access of the sample handler; defined by the platform.
 *
 * Only call these from the sample handler. They take no locks and call
 * no kernel API.
 */
namespace hot_path {

/**
 * @brief Current sample of the valley that just passed.
 *
 * Its trip flags also carry the trips of the samples that were not read (the
 * peaks, and the valleys between two control steps); they are reported once.
 */
[[nodiscard]] CurrentSample GetSample() noexcept;

/**
 * @brief Writes the duties of the next PWM periods.
 *
 * Must be called before the next peak: the duties go directly into the PWM
 * preload registers and take effect at that peak, half a period after the
 * sample. They hold until the next write.
 */
void SetDuties(const system::ThreePhase<unit::Ratio>& duties) noexcept;

/// Switches the outputs off at once and latches the fault, like kPowerStage.Trip().
void TripPowerStage() noexcept;

}  // namespace hot_path

/**
 * @brief Sample handler, defined by the application: the fast current control.
 *
 * The platform calls it once per PWM period, after the sample of the valley,
 * in the zero-latency context (STM32H7: inside the sample interrupt). The
 * application runs its control step on every control_divider-th call.
 *
 * @return true when the sample ended a control cycle: the platform then runs
 *         ObserverHandler() below the sample interrupt.
 */
[[nodiscard]] bool SampleHandler() noexcept;

/**
 * @brief Observer handler, defined by the application.
 *
 * The platform calls it after each control cycle at a priority below the
 * sample interrupt (STM32H7: software-pended normal-priority interrupt).
 */
void ObserverHandler() noexcept;

}  // namespace unimoc::hardware
