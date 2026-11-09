/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file vdc_platform.cpp
 *    @brief The VDC on an STM32H7 board: the CAN transceiver, the node-ID of the build.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "vdc_platform.hpp"
#include "can_transceiver.hpp"

namespace unimoc::hardware {

void VdcPlatformInit() noexcept { InitCanTransceiver(); }

std::span<cyphal::CanMedia* const> VdcPlatformInterfaces() noexcept { return {}; }

uint8_t VdcPlatformNodeId(const uint8_t configured) noexcept { return configured; }

}  // namespace unimoc::hardware
