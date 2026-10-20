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
#include "current_control_isr.hpp"
#include "hardware_interface.hpp"
#include "hot_path_interface.hpp"
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
  static unimoc::settings::SettingsStore settings_store{unimoc::hardware::kSettings.GetSettingsProfile(),
                                                        unimoc::hardware::kSettings.GetSettingsStorage()};
  const auto kSettingsStatus = settings_store.Load();
  if (kSettingsStatus != unimoc::settings::SettingsStatus::kSuccess &&
      kSettingsStatus != unimoc::settings::SettingsStatus::kFactoryDefaults) {
    unimoc::hardware::kRuntime.Log(unimoc::hardware::LogLevel::kError, "Failed to load motor settings\n");
    return 1;
  }

  const auto kSettingsSnapshot = settings_store.GetSnapshot();
  for (const auto& motor : unimoc::hardware::kMotors) {
    auto& current_control = unimoc::current_control::CurrentControlIsr::Instance();
    auto& slow_update = unimoc::current_control::SlowUpdate::Instance();
    current_control.Init(kSettingsSnapshot.Get(), motor, motor.GetTimerClockFrequency());
    slow_update.Init(kSettingsSnapshot.Get(), current_control);
    current_control.SetOvercurrentLimit(
        unimoc::hardware::kSettings.GetSettingsProfile().capabilities.max_phase_current);
    if (!motor.Initialize(kSettingsSnapshot.Get().pwm_frequency, current_control.state.adc_trigger_offset)) {
      unimoc::hardware::kRuntime.Log(unimoc::hardware::LogLevel::kError,
                                     "Failed to initialize motor interface\n");
      return 1;  // Exit if initialization fails
    }
  }

  return 0;
}