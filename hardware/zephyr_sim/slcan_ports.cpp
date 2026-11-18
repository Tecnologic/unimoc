/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file slcan_ports.cpp
 *    @brief The slcan pseudo terminals of a native_sim application as one CAN interface.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "slcan_ports.hpp"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <array>
#include "slcan_media.hpp"

LOG_MODULE_DECLARE(unimoc_cyphal, CONFIG_UNIMOC_LOG_LEVEL);

namespace unimoc::hardware::sim {

namespace {

// The slcan pseudo terminals: the slcan-uarts phandles of zephyr,user (app overlay).
#define UNIMOC_SLCAN_NODE DT_PATH(zephyr_user)
#if defined(CONFIG_UNIMOC_CYPHAL_SLCAN) && DT_NODE_HAS_PROP(UNIMOC_SLCAN_NODE, slcan_uarts)
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage): DT_FOREACH_PROP_ELEM takes a macro.
#define UNIMOC_SLCAN_UART(node_id, prop, idx) DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx)),
const std::array kSlcanUarts{DT_FOREACH_PROP_ELEM(UNIMOC_SLCAN_NODE, slcan_uarts, UNIMOC_SLCAN_UART)};
#undef UNIMOC_SLCAN_UART
SlcanMedia g_slcan;
std::array<cyphal::CanMedia*, 1> g_interfaces{};
#else
std::array<cyphal::CanMedia*, 0> g_interfaces{};
#endif

}  // namespace

std::span<cyphal::CanMedia* const> StartSlcanPorts() noexcept {
#if defined(CONFIG_UNIMOC_CYPHAL_SLCAN) && DT_NODE_HAS_PROP(UNIMOC_SLCAN_NODE, slcan_uarts)
  const std::size_t kPorts = g_slcan.Init(kSlcanUarts);
  if (kPorts == 0U) {
    return {};
  }
  g_interfaces[0] = &g_slcan;
  // NOLINTBEGIN(cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays):
  // the arrays are inside the Zephyr LOG macro.
  LOG_INF("slcan CAN interface on %u pseudo terminals (uart_1.. in the lines above)",
          static_cast<unsigned>(kPorts));
  // NOLINTEND(cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays)
#endif
  return g_interfaces;
}

}  // namespace unimoc::hardware::sim
