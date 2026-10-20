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
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "hardware_interface.hpp"
#include <algorithm>
#include <cmath>
#include "analog_scaling.hpp"
#include "current_control_isr.hpp"
#include "foc_stm32g4.hpp"
#include "foc_stm32g4_dt.hpp"
#include "ram_settings_storage.hpp"

namespace unimoc::hardware {

using namespace unit;

// One control cycle consumes the samples of one hardware cycle.
static_assert(kHalfPeriodsPerCycle == current_control::NUM_SUB_STEPS,
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
  const auto settle =
      static_cast<uint32_t>(kSwitchingSettleTime * static_cast<float>(foc_stm32g4::TimerClockFrequency()));
  g_guard_ticks = foc_stm32g4::TriggerLead() + settle;
}

bool Initialize(Frequency pwm_frequency,
                uint32_t adc_trigger_offset,
                HardwareInterface::ControlCycleCallback control_cycle,
                HardwareInterface::ObserverCallback observer) noexcept {
  if (!foc_stm32g4::Initialize(pwm_frequency, adc_trigger_offset, control_cycle, observer)) return false;
  UpdateGuard();
  return true;
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

// Phase currents of sample k of the cycle. At a peak the phase with the
// highest duty switches closest to the sample, at a valley the one with the
// lowest. When that edge falls into the guard window around the sample, the
// phase is reconstructed from the other two (ia + ib + ic = 0); otherwise
// the zero-sequence error of all three sensors is removed.
system::ThreePhase<Current> CycleSampleCurrents(const uint8_t k,
                                                const foc_stm32g4::RawCurrents& raw) noexcept {
  const float i[3] = {analog::CountsToCurrent(raw.a),
                      analog::CountsToCurrent(raw.b),
                      analog::CountsToCurrent(raw.c)};
  const foc_stm32g4::PhaseTicks& ticks = foc_stm32g4::AppliedTicks(k);
  const uint32_t reload = foc_stm32g4::TimerAutoReload();

  uint8_t edge = 0U;
  bool disturbed = false;
  if (foc_stm32g4::SampleAtPeak(k)) {
    for (uint8_t p = 1U; p < 3U; ++p) edge = ticks[p] > ticks[edge] ? p : edge;
    disturbed = reload - std::min(ticks[edge], reload) < g_guard_ticks;
  } else {
    for (uint8_t p = 1U; p < 3U; ++p) edge = ticks[p] < ticks[edge] ? p : edge;
    disturbed = ticks[edge] < g_guard_ticks;
  }

  float out[3];
  if (disturbed) {
    const uint8_t p1 = (edge + 1U) % 3U;
    const uint8_t p2 = (edge + 2U) % 3U;
    out[p1] = i[p1];
    out[p2] = i[p2];
    out[edge] = -i[p1] - i[p2];
  } else {
    const float zero_sequence = (i[0] + i[1] + i[2]) / 3.0F;
    out[0] = i[0] - zero_sequence;
    out[1] = i[1] - zero_sequence;
    out[2] = -out[0] - out[1];
  }
  return {Current{out[0]}, Current{out[1]}, Current{out[2]}};
}

system::ThreePhase<Voltage> GetPhaseVoltages() noexcept {
  const auto raw = foc_stm32g4::ReadRaw();
  return {Voltage{analog::CountsToVoltage(raw.va)},
          Voltage{analog::CountsToVoltage(raw.vb)},
          Voltage{analog::CountsToVoltage(raw.vc)}};
}

ControlCycleSamples GetControlCycleSamples() noexcept {
  const foc_stm32g4::CycleRaw& raw = foc_stm32g4::CycleSamples();
  ControlCycleSamples samples{};
  for (uint8_t k = 0U; k < kHalfPeriodsPerCycle; ++k)
    samples.phase_currents[k] = CycleSampleCurrents(k, raw.currents[k]);
  samples.dc_link = Voltage{analog::CountsToVoltage(raw.vdc)};
  samples.overcurrent = raw.overcurrent;
  return samples;
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

uint32_t DutyToTicks(const Ratio duty, const float reload) noexcept {
  return static_cast<uint32_t>(std::clamp(duty.Value(), 0.0F, 1.0F) * reload);
}

void SetDutyPlan(const DutyPlan& plan) noexcept {
  const auto reload = static_cast<float>(foc_stm32g4::TimerAutoReload());
  foc_stm32g4::TickPlan ticks{};
  for (uint8_t k = 0U; k < kHalfPeriodsPerCycle; ++k) {
    ticks[k] = {DutyToTicks(plan.duties[k].a, reload),
                DutyToTicks(plan.duties[k].b, reload),
                DutyToTicks(plan.duties[k].c, reload)};
  }
  foc_stm32g4::SetDutyTicks(ticks);
}

// The per-sample check compares raw counts in the sample interrupt.
void SetOvercurrentLimit(const Current limit) noexcept {
  const float amps_per_count = analog::CountsToCurrent(static_cast<uint32_t>(analog::kCurrentZeroCounts + 1));
  const float counts = limit.Value() / amps_per_count;
  if (!(counts < static_cast<float>(analog::kCurrentZeroCounts))) {
    foc_stm32g4::SetOvercurrentWindow(0U, 0xFFFU);  // beyond the measuring range: no check
    return;
  }
  const auto window = static_cast<uint32_t>(std::floor(counts));
  const auto zero = static_cast<uint32_t>(analog::kCurrentZeroCounts);
  foc_stm32g4::SetOvercurrentWindow(zero - window, zero + window);
}

void SetAdcTriggerOffset(uint32_t offset) noexcept {
  foc_stm32g4::SetTriggerLead(offset);
  UpdateGuard();
}

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
                                                     GetControlCycleSamples,
                                                     GetTemperatureMeasurements,
                                                     SetDutyPlan,
                                                     SetOvercurrentLimit,
                                                     SetAdcTriggerOffset,
                                                     GetTimerClockFrequency)};

}  // namespace unimoc::hardware
