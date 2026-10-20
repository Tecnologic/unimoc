/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file main.cpp
 *    @brief Firmware application entry point.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "current_control_runtime.hpp"
#include "hardware_interface.hpp"
#include "hot_path_interface.hpp"
#include "settings_save_gate.hpp"
#include "settings_store.hpp"
#include "slow_update.hpp"

// Called directly by the platform (hot_path_interface.hpp), no registration.
void unimoc::hardware::ControlCycleHandler() noexcept {
  unimoc::current_control::CurrentControlIsr::Instance().OnControlCycle();
}

void unimoc::hardware::ObserverHandler() noexcept {
  unimoc::current_control::SlowUpdate::Instance().RunOnce();
}

// Sets up the drive and returns. Afterwards the control runs in the platform
// interrupts and everything else in Zephyr threads; the main thread is not
// needed any more (Zephyr ends it when main() returns).
int main() {
  // Static: outlives main() for the threads that read or commit settings later.
  // Settings are saved to flash only while the power stage is disabled, and
  // the platform lock serializes the threads that share the store.
  static unimoc::settings::SettingsStore settings_store{unimoc::hardware::kSettings.GetSettingsProfile(),
                                                        unimoc::hardware::kSettings.GetSettingsStorage(),
                                                        unimoc::settings::PowerStageSaveGate(
                                                            unimoc::hardware::kPowerStage),
                                                        unimoc::hardware::kSettings.GetSettingsLock()};
  const auto kSettingsStatus = settings_store.Load();
  switch (kSettingsStatus) {
    case unimoc::settings::SettingsStatus::kSuccess:
    case unimoc::settings::SettingsStatus::kFactoryDefaults:
      break;
    case unimoc::settings::SettingsStatus::kInvalidImage:
    case unimoc::settings::SettingsStatus::kInvalidSettings:
    case unimoc::settings::SettingsStatus::kOutOfRange:
      // The stored registers are unusable; the factory profile is active.
      unimoc::hardware::kRuntime.Log(unimoc::hardware::LogLevel::kWarning,
                                     "Stored settings rejected, using factory settings\n");
      break;
    default:
      unimoc::hardware::kRuntime.Log(unimoc::hardware::LogLevel::kError, "Failed to load motor settings\n");
      return 1;
  }

  const auto kSettingsSnapshot = settings_store.GetSnapshot();
  for (const auto& motor : unimoc::hardware::kMotors) {
    if (!unimoc::current_control::StartRuntime(kSettingsSnapshot.Get(),
                                               motor,
                                               unimoc::hardware::kSettings.GetSettingsProfile()
                                                   .capabilities.max_phase_current)) {
      unimoc::hardware::kRuntime.Log(unimoc::hardware::LogLevel::kError,
                                     "Failed to initialize motor interface\n");
      return 1;  // Exit if initialization fails
    }
  }

  return 0;
}