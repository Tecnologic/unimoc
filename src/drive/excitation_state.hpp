/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file excitation_state.hpp
 *    @brief The state of the excitation module of an EESM drive (unimoc.excitation.Status.0.1).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include "units.hpp"

namespace unimoc::drive {

/// The state of the excitation module of an EESM (unimoc.excitation.Status.0.1).
struct ExcitationState {
  unit::Current field_current;  ///< Measured field current.
  unit::Voltage field_voltage;  ///< Mean field voltage the module applies.
  bool enabled{false};          ///< The field current loop runs.
  bool fault{false};            ///< The module switched its output off.
};

}  // namespace unimoc::drive
