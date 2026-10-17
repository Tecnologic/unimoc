/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file hardware_interface.cpp
 *    @brief Battery case controller (STM32G473) hardware interface.
 *
 *    Skeleton: the motor callbacks are placeholders. The next step brings up
 *    TIM8 and ADC1..ADC5 from the foc0 devicetree node, with the ADC1 JEOC
 *    interrupt and the TIM8 update interrupt as zero-latency IRQs.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "hardware_interface.hpp"

namespace unimoc::hardware {

using namespace unit;

namespace {

settings::SettingsStorageStatus LoadSettings(void* const, const std::span<std::byte>) noexcept {
  return settings::SettingsStorageStatus::kUnavailable;
}

settings::SettingsStorageStatus SaveSettings(void* const, const std::span<const std::byte>) noexcept {
  return settings::SettingsStorageStatus::kUnavailable;
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

bool Initialize(Frequency, uint32_t, HardwareInterface::CurrentControlCallback,
                HardwareInterface::SlowUpdateCallback) noexcept {
  return true;
}

system::ThreePhase<Current> GetPhaseCurrents() noexcept { return {}; }

system::ThreePhase<Voltage> GetPhaseVoltages() noexcept { return {}; }

CurrentControlSamples GetCurrentControlSamples() noexcept { return {}; }

TemperatureMeasurements GetTemperatureMeasurements() noexcept { return {}; }

system::ThreePhase<Ratio> phase_duties{0.5_ratio, 0.5_ratio, 0.5_ratio};

system::ThreePhase<Ratio> GetPhaseDuties() noexcept { return phase_duties; }

void SetPhaseDuties(const system::ThreePhase<Ratio>& duties) noexcept { phase_duties = duties; }

void SetAdcTriggerOffset(uint32_t) noexcept {}

uint32_t GetTimerClockFrequency() noexcept { return 170'000'000U; }

}  // namespace

HardwareSettingsInterface settings{kSettingsProfile, kSettingsStorage};

HardwareInterface motor[MOTORS] = {HardwareInterface(Initialize,
                                                     GetPhaseCurrents,
                                                     GetPhaseVoltages,
                                                     GetCurrentControlSamples,
                                                     GetTemperatureMeasurements,
                                                     GetPhaseDuties,
                                                     SetPhaseDuties,
                                                     SetAdcTriggerOffset,
                                                     GetTimerClockFrequency)};

}  // namespace unimoc::hardware
