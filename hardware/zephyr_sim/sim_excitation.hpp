/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file sim_excitation.hpp
 *    @brief The simulated excitation module of an EESM on native_sim: the field current loop on the
 *           field winding of the plant (docs/system_plan.md, section 5.2.1).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include "excitation_state.hpp"
#include "units.hpp"

namespace unimoc::hardware::sim {

/**
 * @brief Takes a field current command of the drive (DriveHooks::excitation_command) and runs one step of
 *        the module's field current loop.
 *
 * A PI with the pole of the plant's field winding cancelled (5 ms closed-loop
 * time constant) sets the field voltage of the plant, between 0 and the DC
 * link (one half bridge: the field only freewheels down). Disabled, the
 * voltage is zero and the field decays with its own time constant.
 */
void ExcitationCommand(unit::Current field_current, bool enable, uint64_t now_us) noexcept;

/// The state of the module (DriveHooks::excitation_status); always available.
[[nodiscard]] bool ExcitationStatus(drive::ExcitationState& state) noexcept;

/// Back to the state at the start: disabled, no field voltage.
void ResetExcitation() noexcept;

}  // namespace unimoc::hardware::sim
