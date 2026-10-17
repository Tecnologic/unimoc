/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file system_interface.cpp
 *    @brief Zephyr implementation of the platform system services.
 *
 *    Shared by all Zephyr platforms. The platform is selected by the CMake
 *    build, not by the preprocessor.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "hardware_interface.hpp"

LOG_MODULE_REGISTER(unimoc, CONFIG_UNIMOC_LOG_LEVEL);

namespace unimoc::hardware {

namespace {

void LogMessage(const LogLevel level, const char* const message) noexcept {
  switch (level) {
    case LogLevel::kDebug:
      LOG_DBG("%s", message);
      break;
    case LogLevel::kInfo:
      LOG_INF("%s", message);
      break;
    case LogLevel::kWarning:
      LOG_WRN("%s", message);
      break;
    case LogLevel::kError:
      LOG_ERR("%s", message);
      break;
  }
}

/// Background wait of the main thread. The control loops run in interrupts, so
/// the main thread only has to yield to the logging and shell threads.
void WaitForInterrupt() noexcept { k_msleep(1); }

}  // namespace

HardwareSystemInterface runtime{LogMessage, WaitForInterrupt};

}  // namespace unimoc::hardware
