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
 *    Glue between the portable HardwareInterface and the TIM8 / ADC1..5 driver
 *    (foc_stm32g4). Pins, timer and ADC channels come from the foc0 node of the
 *    board devicetree.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "hardware_interface.hpp"
#include "ram_settings_storage.hpp"

#include <algorithm>

#include "analog_scaling.hpp"
#include "current_control_isr.hpp"
#include "foc_stm32g4.hpp"
#include "foc_stm32g4_dt.hpp"

namespace unimoc::hardware {

using namespace unit;

// The slow interrupt consumes one complete sub-step cycle of the fast one.
static_assert(kSlowUpdateDivider == current_control::NUM_SUB_STEPS,
              "slow update must run once per current control sub-step cycle");

namespace {

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

RamSettingsStorage g_ram_settings_storage;
const settings::SettingsStorage kSettingsStorage = g_ram_settings_storage.Adapter();

bool Initialize(Frequency pwm_frequency,
                uint32_t adc_trigger_offset,
                HardwareInterface::CurrentControlCallback current_control,
                HardwareInterface::SlowUpdateCallback slow_update) noexcept {
  return foc_stm32g4::Initialize(pwm_frequency, adc_trigger_offset, current_control, slow_update);
}

// The public vector has an exact zero-sequence component: the common-mode
// error of the three sensors is removed and the last phase is reconstructed.
system::ThreePhase<Current> PhaseCurrents(const foc_stm32g4::RawSamples& raw) noexcept {
  const float a = analog::CountsToCurrent(raw.ia);
  const float b = analog::CountsToCurrent(raw.ib);
  const float c = analog::CountsToCurrent(raw.ic);
  const float zero_sequence = (a + b + c) / 3.0F;
  const Current corrected_a{a - zero_sequence};
  const Current corrected_b{b - zero_sequence};
  return {corrected_a, corrected_b, Current{-corrected_a.Value() - corrected_b.Value()}};
}

system::ThreePhase<Current> GetPhaseCurrents() noexcept { return PhaseCurrents(foc_stm32g4::ReadRaw()); }

system::ThreePhase<Voltage> GetPhaseVoltages() noexcept {
  const auto raw = foc_stm32g4::ReadRaw();
  return {Voltage{analog::CountsToVoltage(raw.va)},
          Voltage{analog::CountsToVoltage(raw.vb)},
          Voltage{analog::CountsToVoltage(raw.vc)}};
}

CurrentControlSamples GetCurrentControlSamples() noexcept {
  const auto raw = foc_stm32g4::ReadRaw();
  return CurrentControlSamples{PhaseCurrents(raw), Voltage{analog::CountsToVoltage(raw.vdc)}};
}

TemperatureMeasurements GetTemperatureMeasurements() noexcept {
  const auto raw = foc_stm32g4::ReadRaw();
  TemperatureMeasurements measurements{};
  if constexpr (kBridgeTemperatureAvailable) {
    measurements.bridge = Temperature{analog::CountsToTemperature(raw.bridge_temperature)};
  }
  if constexpr (kMotorTemperatureAvailable) {
    measurements.motor = Temperature{analog::CountsToTemperature(raw.motor_temperature)};
  }
  return measurements;
}

system::ThreePhase<Ratio> GetPhaseDuties() noexcept {
  const uint32_t reload = foc_stm32g4::TimerAutoReload();
  if (reload == 0U) return {0.5_ratio, 0.5_ratio, 0.5_ratio};
  const float scale = 1.0F / static_cast<float>(reload);
  return {Ratio{static_cast<float>(foc_stm32g4::GetCompare(1U)) * scale},
          Ratio{static_cast<float>(foc_stm32g4::GetCompare(2U)) * scale},
          Ratio{static_cast<float>(foc_stm32g4::GetCompare(3U)) * scale}};
}

uint32_t DutyToTicks(const Ratio duty, const float reload) noexcept {
  return static_cast<uint32_t>(std::clamp(duty.Value(), 0.0F, 1.0F) * reload);
}

void SetPhaseDuties(const system::ThreePhase<Ratio>& duties) noexcept {
  const auto reload = static_cast<float>(foc_stm32g4::TimerAutoReload());
  foc_stm32g4::SetCompare(1U, DutyToTicks(duties.a, reload));
  foc_stm32g4::SetCompare(2U, DutyToTicks(duties.b, reload));
  foc_stm32g4::SetCompare(3U, DutyToTicks(duties.c, reload));
}

void SetAdcTriggerOffset(uint32_t offset) noexcept { foc_stm32g4::SetTriggerLead(offset); }

uint32_t GetTimerClockFrequency() noexcept {
  // Needed before Initialize() to derive the PWM period; fixed by the clock tree in bcc.dts.
  const uint32_t running = foc_stm32g4::TimerClockFrequency();
  return running != 0U ? running : static_cast<uint32_t>(DT_PROP(DT_NODELABEL(rcc), clock_frequency));
}

}  // namespace

HardwareSettingsInterface settings{kSettingsProfile, kSettingsStorage};

PowerStageInterface power_stage{foc_stm32g4::PowerStageEnable,
                                foc_stm32g4::PowerStageDisable,
                                foc_stm32g4::PowerStageTrip,
                                foc_stm32g4::PowerStageClearFault,
                                foc_stm32g4::GetPowerStageState};

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
