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
 *    platform selected by the build (hardware/zephyr_bcc, hardware/zephyr_sim)
 *    defines the functions of `hot_path`; the application defines the two
 *    handlers, which the platform calls directly. Everything outside the hot
 *    path keeps using HardwareInterface.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include "hardware_interface.hpp"

namespace unimoc::hardware {

/**
 * @namespace unimoc::hardware::hot_path
 * @brief Hardware access of the control cycle; defined by the platform.
 *
 * Only call these from the control-cycle handler. They take no locks and call
 * no kernel API.
 */
namespace hot_path {

/// Samples of the control cycle that just ended (four half periods, oldest first).
[[nodiscard]] ControlCycleSamples GetControlCycleSamples() noexcept;

/**
 * @brief Writes the duties of the next kHalfPeriodsPerCycle half periods.
 *
 * Must be called before the first PWM boundary after the control boundary:
 * entry 0 goes directly into the PWM preload registers.
 */
void SetDutyPlan(const DutyPlan& plan) noexcept;

/// Switches the outputs off at once and latches the fault, like power_stage.Trip().
void TripPowerStage() noexcept;

}  // namespace hot_path

/**
 * @brief Control-cycle handler, defined by the application.
 *
 * The platform calls it once per control cycle in the zero-latency context
 * (BCC: inside every kHalfPeriodsPerCycle-th sample interrupt).
 */
void ControlCycleHandler() noexcept;

/**
 * @brief Observer handler, defined by the application.
 *
 * The platform calls it after each control cycle at a priority below the
 * sample interrupt (BCC: software-pended normal-priority interrupt).
 */
void ObserverHandler() noexcept;

}  // namespace unimoc::hardware
