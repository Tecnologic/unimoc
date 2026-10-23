/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file battery_limits.hpp
 *    @brief Battery current limits that replace those of the settings (BMS).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include "units.hpp"

namespace unimoc::current_control {

/**
 * @brief Battery current limits that replace those of the settings.
 *
 * Written by the drive from the battery status of the BMS (Cyphal thread).
 * Without valid limits the outer loop uses `unimoc.battery.limits.*`.
 */
struct BatteryLimits {
  /// The limits below apply; false: the limits of the settings apply.
  bool valid{false};
  /// Largest battery discharge (motoring) current.
  unit::Current drive_current;
  /// Largest battery charge (regenerative) current.
  unit::Current charge_current;
};

}  // namespace unimoc::current_control
