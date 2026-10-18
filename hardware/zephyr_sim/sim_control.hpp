/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file sim_control.hpp
 *    @brief Inputs of the native_sim hardware layer that a test or shell can set.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

namespace unimoc::hardware::sim {

/// External enable (jumper, e-stop). Dropping it while enabled breaks like the hardware does.
void SetExternalEnable(bool present) noexcept;

}  // namespace unimoc::hardware::sim
