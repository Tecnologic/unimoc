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
 *    Glue between the portable HardwareInterface and the TIM8 / ADC1..5 / DMA driver
 *    (foc_stm32g4). Pins, timer and ADC channels come from the foc0 node of the
 *    board devicetree.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "hardware_interface.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include "analog_scaling.hpp"
#include "current_control_isr.hpp"
#include "foc_stm32g4.hpp"
#include "foc_stm32g4_dt.hpp"
#include "hot_path_interface.hpp"
#include "ram_settings_storage.hpp"

namespace unimoc::hardware {

using namespace unit;

// One control cycle consumes the samples of one hardware cycle.
static_assert(kHalfPeriodsPerCycle == current_control::kNumSubSteps,
              "control cycle and hardware cycle cover the same half periods");

namespace {

const settings::SettingsProfile kSettingsProfile = [] {
  settings::SettingsProfile profile{};
  profile.factory_settings.pwm_frequency = 32.0_kHz;  // 16 kHz control cycle
  // Duty range: the minimum off/on time per PWM period is 3 % of 31.25 us =
  // 0.94 us, above the 400 ns dead time (bcc.dts). The sampling window no
  // longer limits it (the phase that switches next to a sample is
  // reconstructed). Estimate, to be checked on the scope with the bootstrap
  // supply of the gate driver.
  profile.factory_settings.svm_duty_min = 0.03_ratio;
  profile.factory_settings.svm_duty_max = 0.97_ratio;
  profile.factory_settings.motor_i_max = 50.0_A;  // current controller limit
  profile.factory_settings.battery_drive_current_max = 50.0_A;
  profile.factory_settings.battery_charge_current_max = 20.0_A;
  profile.capabilities.max_phase_current = 60.0_A;  // hard trip limit of the power stage
  profile.capabilities.max_motor_current = 50.0_A;
  profile.capabilities.max_battery_drive_current = 50.0_A;
  profile.capabilities.max_battery_charge_current = 20.0_A;
  return profile;
}();

RamSettingsStorage g_ram_settings_storage;
const settings::SettingsStorage kSettingsStorage = g_ram_settings_storage.Adapter();

// Settling after a switching edge before a sample is clean again [s]. Estimate.
constexpr float kSwitchingSettleTime = 1.0e-6F;

// A phase that switches within this distance [ticks] of a boundary disturbs
// the sample taken at that boundary.
uint32_t g_guard_ticks = 0U;

void UpdateGuard() noexcept {
  const auto kSettle =
      static_cast<uint32_t>(kSwitchingSettleTime * static_cast<float>(foc_stm32g4::TimerClockFrequency()));
  g_guard_ticks = foc_stm32g4::TriggerLead() + kSettle;
}

bool Initialize(Frequency pwm_frequency, uint32_t adc_trigger_offset) noexcept {
  if (!foc_stm32g4::Initialize(pwm_frequency, adc_trigger_offset)) {
    return false;
  }
  UpdateGuard();
  return true;
}

// The public vector has an exact zero-sequence component: the common-mode
// error of the three sensors is removed and the last phase is reconstructed.
system::ThreePhase<Current> PhaseCurrents(const foc_stm32g4::RawSamples& raw) noexcept {
  const float kCurrentA = analog::CountsToCurrent(raw.ia);
  const float kCurrentB = analog::CountsToCurrent(raw.ib);
  const float kCurrentC = analog::CountsToCurrent(raw.ic);
  const float kZeroSequence = (kCurrentA + kCurrentB + kCurrentC) / 3.0F;
  const Current kCorrectedA{kCurrentA - kZeroSequence};
  const Current kCorrectedB{kCurrentB - kZeroSequence};
  return {kCorrectedA, kCorrectedB, Current{-kCorrectedA.Value() - kCorrectedB.Value()}};
}

system::ThreePhase<Current> GetPhaseCurrents() noexcept { return PhaseCurrents(foc_stm32g4::ReadRaw()); }

// Phase currents of a sample of the cycle. At a peak the phase with the
// highest duty switches closest to the sample, at a valley the one with the
// lowest. When that edge falls into the guard window around the sample, the
// phase is reconstructed from the other two (ia + ib + ic = 0); otherwise
// the zero-sequence error of all three sensors is removed.
system::ThreePhase<Current> CycleSampleCurrents(const uint8_t sample,
                                                const foc_stm32g4::RawCurrents& raw) noexcept {
  const std::array<float, 3> kPhaseCurrent = {analog::CountsToCurrent(raw.a),
                                              analog::CountsToCurrent(raw.b),
                                              analog::CountsToCurrent(raw.c)};
  const foc_stm32g4::PhaseTicks& ticks = foc_stm32g4::AppliedTicks(sample);
  const uint32_t kReload = foc_stm32g4::TimerAutoReload();

  uint8_t edge = 0U;
  bool disturbed = false;
  if (foc_stm32g4::SampleAtPeak(sample)) {
    for (uint8_t phase = 1U; phase < 3U; ++phase) {
      edge = ticks[phase] > ticks[edge] ? phase : edge;
    }
    disturbed = kReload - std::min(ticks[edge], kReload) < g_guard_ticks;
  } else {
    for (uint8_t phase = 1U; phase < 3U; ++phase) {
      edge = ticks[phase] < ticks[edge] ? phase : edge;
    }
    disturbed = ticks[edge] < g_guard_ticks;
  }

  std::array<float, 3> out{};
  if (disturbed) {
    const uint8_t kOther1 = (edge + 1U) % 3U;
    const uint8_t kOther2 = (edge + 2U) % 3U;
    out[kOther1] = kPhaseCurrent[kOther1];
    out[kOther2] = kPhaseCurrent[kOther2];
    out[edge] = -kPhaseCurrent[kOther1] - kPhaseCurrent[kOther2];
  } else {
    const float kZeroSequence = (kPhaseCurrent[0] + kPhaseCurrent[1] + kPhaseCurrent[2]) / 3.0F;
    out[0] = kPhaseCurrent[0] - kZeroSequence;
    out[1] = kPhaseCurrent[1] - kZeroSequence;
    out[2] = -out[0] - out[1];
  }
  return {Current{out[0]}, Current{out[1]}, Current{out[2]}};
}

system::ThreePhase<Voltage> GetPhaseVoltages() noexcept {
  const auto kRaw = foc_stm32g4::ReadRaw();
  return {Voltage{analog::CountsToVoltage(kRaw.va)},
          Voltage{analog::CountsToVoltage(kRaw.vb)},
          Voltage{analog::CountsToVoltage(kRaw.vc)}};
}

TemperatureMeasurements GetTemperatureMeasurements() noexcept {
  const auto kRaw = foc_stm32g4::ReadRaw();
  TemperatureMeasurements measurements{};
  if constexpr (kBridgeTemperatureAvailable) {
    measurements.bridge = Temperature{analog::CountsToTemperature(kRaw.bridge_temperature)};
  }
  if constexpr (kMotorTemperatureAvailable) {
    measurements.motor = Temperature{analog::CountsToTemperature(kRaw.motor_temperature)};
  }
  return measurements;
}

uint32_t DutyToTicks(const Ratio duty, const float reload) noexcept {
  return static_cast<uint32_t>(std::clamp(duty.Value(), 0.0F, 1.0F) * reload);
}

// The per-sample check compares raw counts in the sample interrupt.
void SetOvercurrentLimit(const Current limit) noexcept {
  const float kAmpsPerCount = analog::CountsToCurrent(static_cast<uint32_t>(analog::kCurrentZeroCounts + 1));
  const float kCounts = limit.Value() / kAmpsPerCount;
  if (!(kCounts < static_cast<float>(analog::kCurrentZeroCounts))) {
    foc_stm32g4::SetOvercurrentWindow(0U, 0xFFFU);  // beyond the measuring range: no check
    return;
  }
  const auto kWindow = static_cast<uint32_t>(std::floor(kCounts));
  const auto kZero = static_cast<uint32_t>(analog::kCurrentZeroCounts);
  foc_stm32g4::SetOvercurrentWindow(kZero - kWindow, kZero + kWindow);
}

void SetAdcTriggerOffset(uint32_t offset) noexcept {
  foc_stm32g4::SetTriggerLead(offset);
  UpdateGuard();
}

uint32_t GetTimerClockFrequency() noexcept {
  // Needed before Initialize() to derive the PWM period; fixed by the clock tree in bcc.dts.
  const uint32_t kRunning = foc_stm32g4::TimerClockFrequency();
  return kRunning != 0U ? kRunning : static_cast<uint32_t>(DT_PROP(DT_NODELABEL(rcc), clock_frequency));
}

}  // namespace

namespace hot_path {

ControlCycleSamples GetControlCycleSamples() noexcept {
  const foc_stm32g4::CycleRaw& raw = foc_stm32g4::CycleSamples();
  ControlCycleSamples samples{};
  for (uint8_t k = 0U; k < kHalfPeriodsPerCycle; ++k) {
    samples.phase_currents[k] = CycleSampleCurrents(k, raw.currents[k]);
  }
  samples.dc_link = Voltage{analog::CountsToVoltage(raw.vdc)};
  samples.overcurrent = raw.overcurrent;
  return samples;
}

void SetDutyPlan(const DutyPlan& plan) noexcept {
  const auto kReload = static_cast<float>(foc_stm32g4::TimerAutoReload());
  foc_stm32g4::TickPlan ticks{};
  for (uint8_t k = 0U; k < kHalfPeriodsPerCycle; ++k) {
    ticks[k] = {DutyToTicks(plan.duties[k].a, kReload),
                DutyToTicks(plan.duties[k].b, kReload),
                DutyToTicks(plan.duties[k].c, kReload)};
  }
  foc_stm32g4::SetDutyTicks(ticks);
}

void TripPowerStage() noexcept { foc_stm32g4::PowerStageTrip(); }

}  // namespace hot_path

const HardwareSettingsInterface kSettings{kSettingsProfile, kSettingsStorage};

const PowerStageInterface kPowerStage{foc_stm32g4::PowerStageEnable,
                                      foc_stm32g4::PowerStageDisable,
                                      foc_stm32g4::PowerStageTrip,
                                      foc_stm32g4::PowerStageClearFault,
                                      foc_stm32g4::GetPowerStageState};

const std::array<HardwareInterface, kMotorCount> kMotors = {HardwareInterface(Initialize,
                                                                              GetPhaseCurrents,
                                                                              GetPhaseVoltages,
                                                                              GetTemperatureMeasurements,
                                                                              SetOvercurrentLimit,
                                                                              SetAdcTriggerOffset,
                                                                              GetTimerClockFrequency)};

}  // namespace unimoc::hardware
