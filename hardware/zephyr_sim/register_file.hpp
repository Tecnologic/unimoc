/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file register_file.hpp
 *    @brief The register description file (registers.json) of the simulated drive.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include "cyphal/node.hpp"

namespace unimoc::hardware::sim {

/**
 * @brief Writes the description of every register of @p node as JSON (`docs/gui_plan.md`, section 4).
 *
 * One entry per register in the order of uavcan.register.List: name, value
 * type and size, the flags, the value now (the default of a fresh node) and
 * the provider's description. Run at the start, before anything is written.
 *
 * @param node The node with all its register providers added.
 * @param info GetInfo values (software version and VCS revision).
 * @param path File to write (the host file system).
 * @return False when the file could not be written.
 */
[[nodiscard]] bool WriteRegisterFile(const cyphal::CyphalNode& node,
                                     const cyphal::NodeInfo& info,
                                     const char* path) noexcept;

}  // namespace unimoc::hardware::sim
