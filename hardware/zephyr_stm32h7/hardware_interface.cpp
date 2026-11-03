/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file hardware_interface.cpp
 *    @brief STM32H7 hardware interface (Jacobi, NUCLEO-H743ZI).
 *
 *    Glue between the portable HardwareInterface and the TIM1/TIM8 / ADC1..3
 *    driver (foc_stm32h7). Pins, timer and ADC channels come from the foc0
 *    node of the board devicetree. Counterpart of hardware/zephyr_sim.
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
#include "foc_stm32h7.hpp"
#include "foc_stm32h7_dt.hpp"
#include "hot_path_interface.hpp"
#include "hot_path_timing.hpp"
#include "zephyr_settings_storage.hpp"

namespace unimoc::hardware {

using namespace unit;

// The DWT cycle counter runs at the core clock.
static_assert(foc_stm32h7::kCoreClockHz == kTimestampFrequency, "the timestamp clock is the core clock");

namespace {

// Low-side shunts: settling after the low side turned on plus the trigger
// lead (1 us, CurrentControlIsr::Init()) before the valley [s]. Estimate.
constexpr Time kLowSideSampleWindow = 2.0e-6_s;

const settings::SettingsProfile kSettingsProfile = [] {
  settings::SettingsProfile profile{};
  // 32 kHz PWM, a control step every half period (divider automatic, 64 kHz),
  // the observer at 16 kHz. With the 1 us dead time of the 4850 power board higher
  // frequencies cost too much voltage (docs/hotpath_plan.md, section 3).
  profile.factory_settings.pwm_frequency = 32.0_kHz;
  // Duty range: the minimum off/on time per PWM period is 4 % of 31.25 us =
  // 1.25 us, above the 1 us dead time of the 4850 power board (legacy
  // firmware). Estimate, to be checked on the scope with the bootstrap
  // supply of the LM5109B.
  profile.factory_settings.svm_duty_min = 0.04_ratio;
  profile.factory_settings.svm_duty_max = 0.96_ratio;
  profile.factory_settings.battery_drive_current_max = 50.0_A;
  profile.factory_settings.battery_charge_current_max = 20.0_A;
  profile.capabilities.max_phase_current = 60.0_A;  // hard trip limit of the power stage
  profile.capabilities.max_motor_current = 50.0_A;
  profile.capabilities.max_battery_drive_current = 50.0_A;
  profile.capabilities.max_battery_charge_current = 20.0_A;
  // DC link: 48 V battery (13s, 54.6 V full). Regenerative braking derates
  // from 55 V and the drive trips at 58 V. The 4850 power board uses the
  // BSC040N08 (80 V); the capacitor rating is not checked yet. Currents:
  // the 4850 is a 50 A drive.
  profile.factory_settings.dc_link_regen_derate_start = 55.0_V;
  profile.factory_settings.dc_link_trip = 58.0_V;
  profile.capabilities.max_dc_link_voltage = 60.0_V;
  // Control steps up to 64 kHz: a step every half period at 32 kHz PWM, every
  // period at 64 kHz; about 2070 cycles worst at 400 MHz from ITCM
  // (docs/hotpath_plan.md, section 4).
  profile.capabilities.max_pwm_frequency = 64.0_kHz;
  profile.capabilities.max_control_frequency = 64.0_kHz;
  // foc0 current-sense. Low-side shunts: the control after the valleys only,
  // and each low side on for the dead time, the settling and the trigger lead
  // on both sides of the valley sample (UpdateGuard(); about 6 us with the
  // 1 us dead time, a duty cap of 0.81 at 32 kHz). Estimate, to be measured
  // on the first low-side board.
  if constexpr (foc_stm32h7::kLowSideCurrentSense) {
    profile.capabilities.current_sense = settings::CurrentSense::kLowSide;
    profile.capabilities.min_low_side_time =
        2.0F * (1.0e-9_s * static_cast<float>(foc_stm32h7::kDeadTimeNs) + kLowSideSampleWindow);
  }
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
      static_cast<uint32_t>(kSwitchingSettleTime * static_cast<float>(foc_stm32h7::TimerClockFrequency()));
  g_guard_ticks = foc_stm32h7::TriggerLead() + kSettle;
}

// ----- Hall sensors ----------------------------------------------------------
//
// Hall A, B, C from zephyr,user hall-gpios (one port). Every edge raises the EXTI interrupt (normal priority,
// preempted by the zero-latency interrupts); the handler only reads the code and the DWT timestamp and queues
// them for the observer. Not run on the board yet: the code order (bit 0 = A) and the edge directions follow
// the convention of observer::HallDecoder; the offset is calibrated (unimoc.observer.hall.offset).

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
  const uint32_t kTimestamp = foc_stm32h7::Timestamp();
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
  (void)g_hall_events.Push(observer::HallEvent{ReadHallCode(), foc_stm32h7::Timestamp()});
  return true;
}

// The observer interrupt is the only consumer.
bool PopHallEvent(observer::HallEvent& event) noexcept { return g_hall_events.Pop(event); }

bool Initialize(Frequency pwm_frequency, ControlSampling sampling, uint32_t adc_trigger_offset) noexcept {
  if (!foc_stm32h7::Initialize(pwm_frequency,
                               sampling == ControlSampling::kEveryHalfPeriod,
                               adc_trigger_offset)) {
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
system::ThreePhase<Current> PhaseCurrents(const foc_stm32h7::RawSamples& raw) noexcept {
  const Current kCurrentA = analog::CountsToCurrent(raw.ia);
  const Current kCurrentB = analog::CountsToCurrent(raw.ib);
  const Current kCurrentC = analog::CountsToCurrent(raw.ic);
  const Current kZeroSequence = (kCurrentA + kCurrentB + kCurrentC) / 3.0F;
  const Current kCorrectedA = kCurrentA - kZeroSequence;
  const Current kCorrectedB = kCurrentB - kZeroSequence;
  return {kCorrectedA, kCorrectedB, -kCorrectedA - kCorrectedB};
}

// The last conversion, of a peak or a valley; with low-side shunts only a
// valley conversion measures the phases.
system::ThreePhase<Current> GetPhaseCurrents() noexcept { return PhaseCurrents(foc_stm32h7::ReadRaw()); }

// Phase currents of the sample (the control samples the valleys). At a peak
// the phase with the highest duty switches closest to the sample, at a valley
// the one with the lowest; with low-side shunts (PWM mode 2) the low-side
// pulses are centered on the valley and the highest duty switches closest.
// When that edge falls into the guard window around the sample, the phase is
// reconstructed from the other two (ia + ib + ic = 0); otherwise the
// zero-sequence error of all three sensors is removed. The duty cap of the
// low-side shunts (min_low_side_time) keeps every low side on for a sample.
system::ThreePhase<Current> SampleCurrents(const foc_stm32h7::SampleRaw& raw) noexcept {
  const std::array<Current, 3> kPhaseCurrent = {
      analog::CountsToCurrent(raw.ia),
      analog::CountsToCurrent(raw.ib),
      analog::CountsToCurrent(raw.ic),
  };
  const foc_stm32h7::PhaseTicks& ticks = foc_stm32h7::AppliedTicks();
  const uint32_t kReload = foc_stm32h7::TimerAutoReload();

  uint8_t edge = 0U;
  bool disturbed = false;
  if (foc_stm32h7::SampleAtPeak() != foc_stm32h7::kLowSideCurrentSense) {
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
  const auto kRaw = foc_stm32h7::ReadRaw();
  return {analog::CountsToVoltage(kRaw.va),
          analog::CountsToVoltage(kRaw.vb),
          analog::CountsToVoltage(kRaw.vc)};
}

TemperatureMeasurements GetTemperatureMeasurements() noexcept {
  const auto kRaw = foc_stm32h7::ReadRaw();
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
    foc_stm32h7::SetOvercurrentWindow(0U, 0xFFFU);  // beyond the measuring range: no check
    return;
  }
  const auto kWindow = static_cast<uint32_t>(std::floor(kCounts));
  const auto kZero = static_cast<uint32_t>(analog::kCurrentZeroCounts);
  foc_stm32h7::SetOvercurrentWindow(kZero - kWindow, kZero + kWindow);
}

// The per-sample check compares raw counts in the sample interrupt: a count at
// or above the limit trips. Beyond the measuring range nothing can trip.
void SetOvervoltageLimit(const Voltage limit) noexcept {
  const float kCounts = (limit / analog::kVoltagePerCount).Value();  // a Ratio: counts from zero
  if (!(kCounts < analog::kAdcResolution)) {
    foc_stm32h7::SetOvervoltageCounts(foc_stm32h7::kOvervoltageCheckOff);
    return;
  }
  foc_stm32h7::SetOvervoltageCounts(static_cast<uint32_t>(std::ceil(std::max(kCounts, 0.0F))));
}

void SetAdcTriggerOffset(uint32_t offset) noexcept {
  foc_stm32h7::SetTriggerLead(offset);
  UpdateGuard();
}

uint32_t GetTimerClockFrequency() noexcept {
  // Fixed by the clock tree of the board devicetree, also before Initialize().
  return foc_stm32h7::TimerClockFrequency();
}

}  // namespace

uint32_t timing::DutyOverruns() noexcept { return foc_stm32h7::DutyOverruns(); }

namespace hot_path {

CurrentSample GetSample() noexcept {
  const foc_stm32h7::SampleRaw& raw = foc_stm32h7::Sample();
  CurrentSample sample{};
  sample.phase_currents = SampleCurrents(raw);
  sample.dc_link = analog::CountsToVoltage(raw.vdc);
  sample.overcurrent = raw.overcurrent;
  sample.overvoltage = raw.overvoltage;
  sample.timestamp = raw.timestamp;
  return sample;
}

void SetDuties(const system::ThreePhase<Ratio>& duties) noexcept {
  const auto kReload = static_cast<float>(foc_stm32h7::TimerAutoReload());
  foc_stm32h7::SetDutyTicks(
      {DutyToTicks(duties.a, kReload), DutyToTicks(duties.b, kReload), DutyToTicks(duties.c, kReload)});
}

void TripPowerStage() noexcept { foc_stm32h7::PowerStageTrip(); }

}  // namespace hot_path

const HardwareSettingsInterface kSettings{kSettingsProfile, kSettingsStorage, ZephyrSettingsStorage::Lock()};

const PowerStageInterface kPowerStage{foc_stm32h7::PowerStageEnable,
                                      foc_stm32h7::PowerStageDisable,
                                      foc_stm32h7::PowerStageTrip,
                                      foc_stm32h7::PowerStageClearFault,
                                      foc_stm32h7::GetPowerStageState};

const std::array<HardwareInterface, kMotorCount> kMotors = {HardwareInterface(Initialize,
                                                                              foc_stm32h7::Stop,
                                                                              GetPhaseCurrents,
                                                                              GetPhaseVoltages,
                                                                              GetTemperatureMeasurements,
                                                                              SetOvercurrentLimit,
                                                                              SetOvervoltageLimit,
                                                                              SetAdcTriggerOffset,
                                                                              GetTimerClockFrequency,
                                                                              PopHallEvent)};

}  // namespace unimoc::hardware
