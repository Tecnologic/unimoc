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
#include "control_thread.hpp"
#include "current_control_runtime.hpp"
#include "cyphal_platform.hpp"
#include "hardware_interface.hpp"
#include "hot_path_interface.hpp"
#include "outer_loop.hpp"
#include "settings_save_gate.hpp"
#include "settings_store.hpp"
#include "slow_update.hpp"
#include "trace_signals.hpp"

// Called directly by the platform (hot_path_interface.hpp), no registration.
// The traces sample the state of the cycle right after it (docs/cymon_integration_plan.md).
void unimoc::hardware::ControlCycleHandler() noexcept {
  auto& current_control = unimoc::current_control::CurrentControlIsr::Instance();
  current_control.OnControlCycle();
  unimoc::trace::OnControlCycle(current_control);
}

// The observer interrupt wakes the control thread every
// OuterLoop::kCyclesPerStep control cycles (2 kHz at a 16 kHz control cycle).
void unimoc::hardware::ObserverHandler() noexcept {
  unimoc::current_control::SlowUpdate::Instance().RunOnce();
  if (unimoc::current_control::OuterLoop::Instance().OnObserverCycle()) {
    unimoc::hardware::NotifyControlThread();
  }
}

void unimoc::hardware::ControlThreadHandler() noexcept {
  unimoc::current_control::OuterLoop::Instance().Step();
}

// Sets up the drive and returns. Afterwards the control runs in the platform
// interrupts and everything else in Zephyr threads (the outer loop in the
// control thread, the Cyphal node in its own thread); the main thread is not
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

  // Outer loop (speed, position, setpoints) of motor 0, below the observer.
  unimoc::hardware::StartControlThread();

  // Cyphal/CAN: setpoints, registers, telemetry and the log (motor 0).
  unimoc::hardware::StartCyphal(settings_store);
  return 0;
}