/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file hardware_interface.cpp
 *    @brief Hosted hardware interface implementation.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "hardware_interface.hpp"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <thread>
#include "hot_path_interface.hpp"

namespace unimoc {
namespace hardware {

using namespace unit;

namespace {

const char* GetSettingsPath() noexcept {
  const char* const path = std::getenv("UNIMOC_SETTINGS_FILE");
  return path == nullptr ? "unimoc_settings.bin" : path;
}

settings::SettingsStorageStatus LoadSettings(void* const, const std::span<std::byte> image) noexcept {
  std::ifstream input{GetSettingsPath(), std::ios::binary};
  if (!input.is_open()) return settings::SettingsStorageStatus::kNotFound;

  input.read(reinterpret_cast<char*>(image.data()), static_cast<std::streamsize>(image.size()));
  return input.gcount() == static_cast<std::streamsize>(image.size())
             ? settings::SettingsStorageStatus::kSuccess
             : settings::SettingsStorageStatus::kIoError;
}

settings::SettingsStorageStatus SaveSettings(void* const, const std::span<const std::byte> image) noexcept {
  std::ofstream output{GetSettingsPath(), std::ios::binary | std::ios::trunc};
  if (!output.is_open()) return settings::SettingsStorageStatus::kIoError;

  output.write(reinterpret_cast<const char*>(image.data()), static_cast<std::streamsize>(image.size()));
  return output.good() ? settings::SettingsStorageStatus::kSuccess
                       : settings::SettingsStorageStatus::kIoError;
}

const settings::SettingsProfile kSettingsProfile = [] {
  settings::SettingsProfile profile{};
  profile.factory_settings.motor_i_max = 80.0_A;
  profile.factory_settings.battery_drive_current_max = 50.0_A;
  profile.factory_settings.battery_charge_current_max = 20.0_A;
  profile.capabilities.max_phase_current = 100.0_A;
  profile.capabilities.max_motor_current = 80.0_A;
  profile.capabilities.max_battery_drive_current = 50.0_A;
  profile.capabilities.max_battery_charge_current = 20.0_A;
  return profile;
}();

const settings::SettingsStorage kSettingsStorage{nullptr, LoadSettings, SaveSettings};

}  // namespace

HardwareSettingsInterface settings{kSettingsProfile, kSettingsStorage};

// Stub function implementations
static bool InitializeStub([[maybe_unused]] const unit::Frequency pwm_frequency,
                           [[maybe_unused]] const uint32_t adc_trigger_offset) noexcept {
  // Stub: Always return successful initialization
  return true;
}

static system::ThreePhase<unit::Current> GetPhaseCurrentsStub() noexcept {
  // Stub: Return zero currents
  return system::ThreePhase<unit::Current>{0.0_A, 0.0_A, 0.0_A};
}

static system::ThreePhase<unit::Voltage> GetPhaseVoltagesStub() noexcept {
  // Stub: Return zero voltages
  return system::ThreePhase<unit::Voltage>{0.0_V, 0.0_V, 0.0_V};
}

static TemperatureMeasurements GetTemperatureMeasurementsStub() noexcept { return TemperatureMeasurements{}; }

static void SetOvercurrentLimitStub([[maybe_unused]] unit::Current limit) noexcept {}

static void SetAdcTriggerOffsetStub([[maybe_unused]] uint32_t offset) noexcept {}

static uint32_t GetTimerClockFrequencyStub() noexcept { return 168'000'000u; }

static void WaitForInterruptStub() noexcept { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }

static void LogStub([[maybe_unused]] const LogLevel level, const char* const message) noexcept {
  std::fputs(message, stderr);
}

// Initialize the motor array with stub implementations
HardwareInterface motor[MOTORS] = {HardwareInterface(InitializeStub,
                                                     GetPhaseCurrentsStub,
                                                     GetPhaseVoltagesStub,
                                                     GetTemperatureMeasurementsStub,
                                                     SetOvercurrentLimitStub,
                                                     SetAdcTriggerOffsetStub,
                                                     GetTimerClockFrequencyStub)};

HardwareSystemInterface runtime{LogStub, WaitForInterruptStub};

// The hosted stubs have no interrupts; the hot path sees zero samples.
namespace hot_path {

ControlCycleSamples GetControlCycleSamples() noexcept { return ControlCycleSamples{}; }

void SetDutyPlan([[maybe_unused]] const DutyPlan& plan) noexcept {}

void TripPowerStage() noexcept {}

}  // namespace hot_path

}  // namespace hardware
}  // namespace unimoc