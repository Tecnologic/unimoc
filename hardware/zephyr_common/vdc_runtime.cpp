/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file vdc_runtime.cpp
 *    @brief Cyphal thread of the vehicle dynamics controller (VDC): CAN interfaces,
 *           transport, node info, the VDC node and the log.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <array>
#include "cyphal/canard_transport.hpp"
#include "cyphal/node.hpp"
#include "cyphal/sw_version.hpp"
#include "cyphal_log_backend.h"
#include "hardware_interface_config.hpp"
#include "vdc_node.hpp"
#include "vdc_platform.hpp"
#include "zephyr_can_media.hpp"

LOG_MODULE_REGISTER(unimoc_cyphal, CONFIG_UNIMOC_LOG_LEVEL);

#ifndef UNIMOC_VCS_REVISION
#define UNIMOC_VCS_REVISION 0ULL
#endif

namespace unimoc::hardware {

// NOLINTBEGIN(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay,clang-diagnostic-vla-cxx-extension):
// LOG_* packs its arguments into a stack buffer (a VLA in deferred mode) and
// K_THREAD_STACK_DEFINE / K_SEM_DEFINE declare C arrays, inside the Zephyr macros.

namespace {

constexpr std::size_t kMaxInterfaces = cyphal::CanardTransport::kMaxInterfaces;
constexpr std::string_view kNodeName = "org.unimoc.vdc";

K_THREAD_STACK_DEFINE(g_cyphal_stack, CONFIG_UNIMOC_CYPHAL_THREAD_STACK_SIZE);
struct k_thread g_cyphal_thread;
K_SEM_DEFINE(g_cyphal_wake, 0, 1000);

ZephyrCanMedia g_can_media;
cyphal::CanardArena<CONFIG_UNIMOC_CYPHAL_ARENA_SIZE> g_arena;
std::array<uint8_t, cyphal::kUniqueIdLen> g_unique_id{};

[[nodiscard]] uint64_t NowUs() noexcept { return k_ticks_to_us_floor64(k_uptime_ticks()); }

[[nodiscard]] cyphal::Severity ToSeverity(const uint8_t level) noexcept {
  switch (level) {
    case LOG_LEVEL_ERR:
      return cyphal::Severity::kError;
    case LOG_LEVEL_WRN:
      return cyphal::Severity::kWarning;
    case LOG_LEVEL_INF:
      return cyphal::Severity::kInfo;
    default:
      return cyphal::Severity::kDebug;
  }
}

/// Collects the CAN interfaces: the chosen controller first, then the platform extras.
std::size_t CollectInterfaces(std::array<cyphal::CanMedia*, kMaxInterfaces>& interfaces) noexcept {
  std::size_t count = 0U;
  const struct device* const kCan = DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_canbus));
  const int kResult = g_can_media.Init(kCan, &g_cyphal_wake, IS_ENABLED(CONFIG_UNIMOC_CYPHAL_CAN_FD));
  if (kResult == 0) {
    interfaces[count++] = &g_can_media;
    LOG_INF("CAN %s up, MTU %u", kCan->name, static_cast<unsigned>(g_can_media.Mtu()));
  } else {
    LOG_WRN("CAN controller not available (%d)", kResult);
  }
  for (cyphal::CanMedia* media : VdcPlatformInterfaces()) {
    if (media != nullptr && count < interfaces.size()) {
      interfaces[count++] = media;
    }
  }
  return count;
}

void CyphalThread(void* /*unused*/, void* /*unused*/, void* /*unused*/) {
  VdcPlatformInit();
  std::array<cyphal::CanMedia*, kMaxInterfaces> interfaces{};
  const std::size_t kCount = CollectInterfaces(interfaces);
  if (kCount == 0U) {
    LOG_ERR("no CAN interface: Cyphal disabled");
    return;
  }

  static cyphal::CanardTransport transport{g_arena.bytes,
                                           std::span<cyphal::CanMedia* const>{interfaces.data(), kCount}};
  if (!transport.IsValid()) {
    LOG_ERR("Cyphal transport not usable (arena)");
    return;
  }
  static cyphal::CyphalNode node{transport};

  (void)hwinfo_get_device_id(g_unique_id.data(), g_unique_id.size());
  cyphal::NodeInfo info{};
  info.name = kNodeName;
  info.hardware_major = kVersionMajor;
  info.hardware_minor = kVersionMinor;
  info.software_major = static_cast<uint8_t>(cyphal::kSwVersionMajor);
  info.software_minor = static_cast<uint8_t>(cyphal::kSwVersionMinor);
  info.vcs_revision_id = UNIMOC_VCS_REVISION;
  info.unique_id = g_unique_id;
  node.SetInfo(info);

  static vdc::VdcNode vdc{node};
  if (!vdc.Start(VdcPlatformNodeId(CONFIG_UNIMOC_VDC_NODE_ID), NowUs())) {
    LOG_WRN("not all Cyphal ports could be added");
  }
  LOG_INF("VDC node %u on %u interface(s)",
          static_cast<unsigned>(node.NodeId()),
          static_cast<unsigned>(kCount));

  std::array<char, 128> text{};
  while (true) {
    // Woken by every received frame; the timeout bounds the jitter of the wheel command.
    (void)k_sem_take(&g_cyphal_wake, K_MSEC(1));
    vdc.Spin(NowUs());
    uint8_t level = 0U;
    while (UnimocCyphalLogGet(&level, text.data(), text.size())) {
      (void)node.Log(ToSeverity(level), std::string_view{text.data()});
    }
  }
}

}  // namespace

void StartVdc() noexcept {
  k_thread_create(&g_cyphal_thread,
                  g_cyphal_stack,
                  K_THREAD_STACK_SIZEOF(g_cyphal_stack),
                  CyphalThread,
                  nullptr,
                  nullptr,
                  nullptr,
                  CONFIG_UNIMOC_CYPHAL_THREAD_PRIORITY,
                  0,
                  K_NO_WAIT);
  k_thread_name_set(&g_cyphal_thread, "cyphal");
}

// NOLINTEND(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay,clang-diagnostic-vla-cxx-extension)

}  // namespace unimoc::hardware
