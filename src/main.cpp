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
  unimoc::current_control::CurrentControlIsr::instance().on_control_cycle();
}

void unimoc::hardware::ObserverHandler() noexcept {
  unimoc::current_control::SlowUpdate::instance().run_once();
}

int main() {
  unimoc::settings::SettingsStore settings_store{unimoc::hardware::settings.GetSettingsProfile(),
                                                 unimoc::hardware::settings.GetSettingsStorage()};
  const auto settings_status = settings_store.Load();
  if (settings_status != unimoc::settings::SettingsStatus::kSuccess &&
      settings_status != unimoc::settings::SettingsStatus::kFactoryDefaults) {
    unimoc::hardware::runtime.Log(unimoc::hardware::LogLevel::kError, "Failed to load motor settings\n");
    return 1;
  }

  const auto settings_snapshot = settings_store.GetSnapshot();
  for (auto& motor : unimoc::hardware::motor) {
    auto& current_control = unimoc::current_control::CurrentControlIsr::instance();
    auto& slow_update = unimoc::current_control::SlowUpdate::instance();
    current_control.init(settings_snapshot.Get(), motor, motor.GetTimerClockFrequency());
    slow_update.init(settings_snapshot.Get(), current_control);
    current_control.set_overcurrent_limit(
        unimoc::hardware::settings.GetSettingsProfile().capabilities.max_phase_current);
    if (!motor.Initialize(settings_snapshot.Get().pwm_frequency, current_control.state.adc_trigger_offset)) {
      unimoc::hardware::runtime.Log(unimoc::hardware::LogLevel::kError,
                                    "Failed to initialize motor interface\n");
      return 1;  // Exit if initialization fails
    }
  }

  while (true) {
    unimoc::hardware::runtime.WaitForInterrupt();
  }
}