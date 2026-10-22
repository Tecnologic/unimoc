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
#include <zephyr/drivers/gpio.h>
#include <algorithm>
#include <array>
#include <cmath>
#include "analog_scaling.hpp"
#include "current_control_isr.hpp"
#include "foc_stm32g4.hpp"
#include "foc_stm32g4_dt.hpp"
#include "hot_path_interface.hpp"
#include "hot_path_timing.hpp"
#include "zephyr_settings_storage.hpp"

namespace unimoc::hardware {

using namespace unit;

// One control cycle consumes the samples of one hardware cycle.
static_assert(kHalfPeriodsPerCycle == current_control::kNumSubSteps,
              "control cycle and hardware cycle cover the same half periods");
// The DWT cycle counter runs at the core clock (HCLK).
static_assert(DT_PROP(DT_NODELABEL(rcc), clock_frequency) == kTimestampFrequency,
              "the timestamp clock is the core clock");

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

const settings::SettingsStorage kSettingsStorage = ZephyrSettingsStorage::Adapter();

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

// ----- Hall sensors ----------------------------------------------------------
//
// Hall A, B, C on PC13..PC15 (zephyr,user hall-gpios, RC filtered, about
// 7 us). Every edge raises the EXTI interrupt (normal priority, preempted by
// the zero-latency interrupts); the handler only reads the code and the DWT
// timestamp and queues them for the observer. Not run on the board yet: the
// code order (bit 0 = A) and the edge directions follow the convention of
// observer::HallDecoder; the offset is calibrated (unimoc.observer.hall.offset).

const std::array<struct gpio_dt_spec, 3> kHallInputs = {{
    GPIO_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), hall_gpios, 0),
    GPIO_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), hall_gpios, 1),
    GPIO_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), hall_gpios, 2),
}};

observer::HallEventQueue<16U> g_hall_events;
struct gpio_callback g_hall_callback;
bool g_hall_ready = false;

uint8_t ReadHallCode() noexcept {
  uint8_t code = 0U;
  for (std::size_t sensor = 0U; sensor < kHallInputs.size(); ++sensor) {
    if (gpio_pin_get_dt(&kHallInputs[sensor]) > 0) {
      code = static_cast<uint8_t>(code | (1U << sensor));
    }
  }
  return code;
}

void OnHallEdge(const struct device* /*port*/, struct gpio_callback* /*callback*/, uint32_t /*pins*/) {
  const uint32_t kTimestamp = foc_stm32g4::Timestamp();
  (void)g_hall_events.Push(observer::HallEvent{ReadHallCode(), kTimestamp});
}

bool InitHall() noexcept {
  if (!g_hall_ready) {
    gpio_port_pins_t pins = 0U;
    for (const auto& input : kHallInputs) {
      if (!gpio_is_ready_dt(&input) || input.port != kHallInputs[0].port ||
          gpio_pin_configure_dt(&input, GPIO_INPUT) != 0 ||
          gpio_pin_interrupt_configure_dt(&input, GPIO_INT_EDGE_BOTH) != 0) {
        return false;
      }
      pins |= BIT(input.pin);
    }
    gpio_init_callback(&g_hall_callback, OnHallEdge, pins);
    if (gpio_add_callback(kHallInputs[0].port, &g_hall_callback) != 0) {
      return false;
    }
    g_hall_ready = true;
  }
  // The observer starts from the present code.
  g_hall_events.Clear();
  (void)g_hall_events.Push(observer::HallEvent{ReadHallCode(), foc_stm32g4::Timestamp()});
  return true;
}

// The observer interrupt is the only consumer.
bool PopHallEvent(observer::HallEvent& event) noexcept { return g_hall_events.Pop(event); }

bool Initialize(Frequency pwm_frequency, uint32_t adc_trigger_offset) noexcept {
  if (!foc_stm32g4::Initialize(pwm_frequency, adc_trigger_offset)) {
    return false;
  }
  if (!InitHall()) {
    return false;
  }
  UpdateGuard();
  return true;
}

// The public vector has an exact zero-sequence component: the common-mode
// error of the three sensors is removed and the last phase is reconstructed.
system::ThreePhase<Current> PhaseCurrents(const foc_stm32g4::RawSamples& raw) noexcept {
  const Current kCurrentA = analog::CountsToCurrent(raw.ia);
  const Current kCurrentB = analog::CountsToCurrent(raw.ib);
  const Current kCurrentC = analog::CountsToCurrent(raw.ic);
  const Current kZeroSequence = (kCurrentA + kCurrentB + kCurrentC) / 3.0F;
  const Current kCorrectedA = kCurrentA - kZeroSequence;
  const Current kCorrectedB = kCurrentB - kZeroSequence;
  return {kCorrectedA, kCorrectedB, -kCorrectedA - kCorrectedB};
}

system::ThreePhase<Current> GetPhaseCurrents() noexcept { return PhaseCurrents(foc_stm32g4::ReadRaw()); }

// Phase currents of a sample of the cycle. At a peak the phase with the
// highest duty switches closest to the sample, at a valley the one with the
// lowest. When that edge falls into the guard window around the sample, the
// phase is reconstructed from the other two (ia + ib + ic = 0); otherwise
// the zero-sequence error of all three sensors is removed.
system::ThreePhase<Current> CycleSampleCurrents(const uint8_t sample,
                                                const foc_stm32g4::RawCurrents& raw) noexcept {
  const std::array<Current, 3> kPhaseCurrent = {analog::CountsToCurrent(raw.a),
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

  std::array<Current, 3> out{};
  if (disturbed) {
    const uint8_t kOther1 = (edge + 1U) % 3U;
    const uint8_t kOther2 = (edge + 2U) % 3U;
    out[kOther1] = kPhaseCurrent[kOther1];
    out[kOther2] = kPhaseCurrent[kOther2];
    out[edge] = -kPhaseCurrent[kOther1] - kPhaseCurrent[kOther2];
  } else {
    const Current kZeroSequence = (kPhaseCurrent[0] + kPhaseCurrent[1] + kPhaseCurrent[2]) / 3.0F;
    out[0] = kPhaseCurrent[0] - kZeroSequence;
    out[1] = kPhaseCurrent[1] - kZeroSequence;
    out[2] = -out[0] - out[1];
  }
  return {out[0], out[1], out[2]};
}

system::ThreePhase<Voltage> GetPhaseVoltages() noexcept {
  const auto kRaw = foc_stm32g4::ReadRaw();
  return {analog::CountsToVoltage(kRaw.va),
          analog::CountsToVoltage(kRaw.vb),
          analog::CountsToVoltage(kRaw.vc)};
}

TemperatureMeasurements GetTemperatureMeasurements() noexcept {
  const auto kRaw = foc_stm32g4::ReadRaw();
  TemperatureMeasurements measurements{};
  if constexpr (kBridgeTemperatureAvailable) {
    measurements.bridge = analog::CountsToTemperature(kRaw.bridge_temperature);
  }
  if constexpr (kMotorTemperatureAvailable) {
    measurements.motor = analog::CountsToTemperature(kRaw.motor_temperature);
  }
  return measurements;
}

uint32_t DutyToTicks(const Ratio duty, const float reload) noexcept {
  return static_cast<uint32_t>(std::clamp(duty.Value(), 0.0F, 1.0F) * reload);
}

// The per-sample check compares raw counts in the sample interrupt.
void SetOvercurrentLimit(const Current limit) noexcept {
  const float kCounts = (limit / analog::kCurrentPerCount).Value();  // a Ratio: counts from zero
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

uint32_t timing::DutyOverruns() noexcept { return foc_stm32g4::DutyOverruns(); }

namespace hot_path {

ControlCycleSamples GetControlCycleSamples() noexcept {
  const foc_stm32g4::CycleRaw& raw = foc_stm32g4::CycleSamples();
  ControlCycleSamples samples{};
  for (uint8_t k = 0U; k < kHalfPeriodsPerCycle; ++k) {
    samples.phase_currents[k] = CycleSampleCurrents(k, raw.currents[k]);
  }
  samples.dc_link = analog::CountsToVoltage(raw.vdc);
  samples.overcurrent = raw.overcurrent;
  samples.timestamp = raw.timestamp;
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

const HardwareSettingsInterface kSettings{kSettingsProfile, kSettingsStorage, ZephyrSettingsStorage::Lock()};

const PowerStageInterface kPowerStage{foc_stm32g4::PowerStageEnable,
                                      foc_stm32g4::PowerStageDisable,
                                      foc_stm32g4::PowerStageTrip,
                                      foc_stm32g4::PowerStageClearFault,
                                      foc_stm32g4::GetPowerStageState};

const std::array<HardwareInterface, kMotorCount> kMotors = {HardwareInterface(Initialize,
                                                                              foc_stm32g4::Stop,
                                                                              GetPhaseCurrents,
                                                                              GetPhaseVoltages,
                                                                              GetTemperatureMeasurements,
                                                                              SetOvercurrentLimit,
                                                                              SetAdcTriggerOffset,
                                                                              GetTimerClockFrequency,
                                                                              PopHallEvent)};

}  // namespace unimoc::hardware
