/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file cyphal_runtime.cpp
 *    @brief Cyphal thread of the drive, shared by all Zephyr platforms.
 *
 *    Owns the CAN interfaces (the chosen `zephyr,canbus` plus the platform
 *    extras), the libcanard transport, the node and the drive application.
 *    The thread wakes on every received frame and at least once per
 *    millisecond. It is a normal preemptible thread below the hot path.
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
#include <zephyr/sys/reboot.h>
#include <algorithm>
#include <array>
#include "current_control_isr.hpp"
#include "cyphal/canard_transport.hpp"
#include "cyphal/node.hpp"
#include "cyphal/sw_version.hpp"
#include "cyphal_log_backend.h"
#include "cyphal_platform.hpp"
#include "drive_node.hpp"
#include "hardware_interface.hpp"
#include "hardware_interface_config.hpp"
#include "trace_node.hpp"
#include "trace_signals.hpp"
#include "zephyr_can_media.hpp"

LOG_MODULE_REGISTER(unimoc_cyphal, CONFIG_UNIMOC_LOG_LEVEL);

#ifndef UNIMOC_VCS_REVISION
#define UNIMOC_VCS_REVISION 0ULL
#endif

namespace unimoc::hardware {

// NOLINTBEGIN(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay,clang-diagnostic-vla-cxx-extension,readability-function-size):
// LOG_* packs its arguments into a stack buffer (a VLA in deferred mode) and
// K_THREAD_STACK_DEFINE / K_SEM_DEFINE declare C arrays, inside the Zephyr macros.

namespace {

constexpr std::size_t kMaxInterfaces = cyphal::CanardTransport::kMaxInterfaces;

K_THREAD_STACK_DEFINE(g_cyphal_stack, CONFIG_UNIMOC_CYPHAL_THREAD_STACK_SIZE);
struct k_thread g_cyphal_thread;
K_SEM_DEFINE(g_cyphal_wake, 0, 1000);

settings::SettingsStore* g_store = nullptr;
ZephyrCanMedia g_can_media;
cyphal::CanardArena<CONFIG_UNIMOC_CYPHAL_ARENA_SIZE> g_arena;
std::array<uint8_t, cyphal::kUniqueIdLen> g_unique_id{};
std::array<char, cyphal::kNodeNameMaxLen + 1U> g_name{};

[[nodiscard]] uint64_t NowUs() noexcept { return k_ticks_to_us_floor64(k_uptime_ticks()); }

void Restart() noexcept {
  LOG_INF("restart requested over Cyphal");
  k_msleep(50);  // let the log thread drain
  sys_reboot(SYS_REBOOT_COLD);
}

trace::TraceNode* g_trace = nullptr;

/// Period of the control cycle, which samples the traces [us].
[[nodiscard]] float TraceTickPeriodUs() noexcept {
  return current_control::CurrentControlIsr::Instance().state.dt_slow * 1.0e6F;
}

void SettingsApplied(const settings::NvmSettings& settings) noexcept {
  if (g_trace != nullptr) {
    g_trace->SetTickPeriod(TraceTickPeriodUs());
  }
  CyphalPlatformSettingsApplied(settings);
}

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
  for (cyphal::CanMedia* media : CyphalPlatformInterfaces()) {
    if (media != nullptr && count < interfaces.size()) {
      interfaces[count++] = media;
    }
  }
  return count;
}

void CyphalThread(void* /*unused*/, void* /*unused*/, void* /*unused*/) {
  CyphalPlatformInit();
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

  const uint8_t kNodeIdOverride = CyphalPlatformNodeIdOverride();
  if (kNodeIdOverride <= cyphal::kNodeIdMax &&
      g_store->GetOperations().SetNodeId(kNodeIdOverride) != settings::SettingsStatus::kSuccess) {
    LOG_WRN("node-ID %u from the command line rejected", static_cast<unsigned>(kNodeIdOverride));
  }
  (void)hwinfo_get_device_id(g_unique_id.data(), g_unique_id.size());
  const settings::SettingsSnapshot kSnapshot = g_store->GetSnapshot();
  const std::string_view kName = kSnapshot.Get().identity.GetName();
  std::copy(kName.begin(), kName.end(), g_name.begin());
  cyphal::NodeInfo info{};
  info.name = std::string_view{g_name.data(), kName.size()};
  info.hardware_major = kVersionMajor;
  info.hardware_minor = kVersionMinor;
  info.software_major = static_cast<uint8_t>(cyphal::kSwVersionMajor);
  info.software_minor = static_cast<uint8_t>(cyphal::kSwVersionMinor);
  info.vcs_revision_id = UNIMOC_VCS_REVISION;
  info.unique_id = g_unique_id;
  node.SetInfo(info);

  static drive::DriveNode drive{node,
                                *g_store,
                                kMotors[0],
                                drive::DriveHooks{Restart, SettingsApplied},
                                std::span<const uint8_t>{g_unique_id}};
  CyphalPlatformStart(node, kSnapshot.Get());

  // Traces: the control signals first (IDs 0..11), then the platform's.
  static trace::TraceNode trace_node{node, trace::TraceDevice()};
  if (!trace::RegisterControlSignals()) {
    LOG_WRN("trace registry full");
  }
  CyphalPlatformTraceSignals(trace::TraceDevice());
  if (!trace_node.Start(kSnapshot.Get(), TraceTickPeriodUs())) {
    LOG_WRN("not all trace ports could be added");
  }
  g_trace = &trace_node;

  if (!drive.Start(NowUs())) {
    LOG_WRN("not all Cyphal ports could be subscribed");
  }
  LOG_INF("Cyphal node %u on %u interface(s)",
          static_cast<unsigned>(node.NodeId()),
          static_cast<unsigned>(kCount));

  std::array<char, 128> text{};
  while (true) {
    (void)k_sem_take(&g_cyphal_wake, K_MSEC(1));
    const uint64_t kNow = NowUs();
    drive.Spin(kNow);
    trace_node.Spin();
    CyphalPlatformSpin(node, kNow);
    uint8_t level = 0U;
    while (UnimocCyphalLogGet(&level, text.data(), text.size())) {
      (void)node.Log(ToSeverity(level), std::string_view{text.data()});
    }
  }
}

}  // namespace

void StartCyphal(settings::SettingsStore& store) noexcept {
  g_store = &store;
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

// NOLINTEND(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay,clang-diagnostic-vla-cxx-extension,readability-function-size)

}  // namespace unimoc::hardware
