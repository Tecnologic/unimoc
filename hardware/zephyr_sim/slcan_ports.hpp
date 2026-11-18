/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file slcan_ports.hpp
 *    @brief The slcan pseudo terminals of a native_sim application as one CAN interface.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <span>
#include "cyphal/can_media.hpp"

namespace unimoc::hardware::sim {

/**
 * @brief Opens the slcan pseudo terminals of the `slcan-uarts` property of zephyr,user.
 *
 * Shared by the drive and the VDC on native_sim (CONFIG_UNIMOC_CYPHAL_SLCAN).
 * Call once from the Cyphal thread before the transport starts.
 *
 * @return The slcan interface, or an empty span without terminals.
 */
[[nodiscard]] std::span<cyphal::CanMedia* const> StartSlcanPorts() noexcept;

}  // namespace unimoc::hardware::sim
