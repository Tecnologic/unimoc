/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file vdc_platform.cpp
 *    @brief The VDC on native_sim: the slcan pseudo terminals and the node-ID of the command line.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "vdc_platform.hpp"
#include <algorithm>
#include "sim_cmdline.h"
#include "slcan_ports.hpp"

namespace unimoc::hardware {

namespace {

std::span<cyphal::CanMedia* const> g_interfaces;

}  // namespace

void VdcPlatformInit() noexcept { g_interfaces = sim::StartSlcanPorts(); }

std::span<cyphal::CanMedia* const> VdcPlatformInterfaces() noexcept { return g_interfaces; }

uint8_t VdcPlatformNodeId(const uint8_t configured) noexcept {
  // --node-id=<n> as for the simulated drive; a value above 127 requests plug-and-play.
  const int32_t kNodeId = UnimocSimNodeIdOption();
  return (kNodeId >= 0) ? static_cast<uint8_t>(std::min<int32_t>(kNodeId, 255)) : configured;
}

}  // namespace unimoc::hardware
