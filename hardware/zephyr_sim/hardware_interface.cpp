/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file hardware_interface.cpp
 *    @brief Simulation (native_sim) hardware interface.
 *
 *    Counterpart of hardware/zephyr_bcc. A kernel timer in simulated time
 *    stands in for the PWM period: every expiry runs the fast callback (BCC:
 *    ADC1 end of injected sequence) and every kSlowUpdateDivider-th expiry the
 *    slow callback (BCC: TIM8 update event with repetition counter), the fast
 *    one first, as on the target. The analog side is a constant stub for now:
 *    zero phase currents and a fixed DC link voltage.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "hardware_interface.hpp"
#include "ram_settings_storage.hpp"

#include <zephyr/kernel.h>

namespace unimoc::hardware {

using namespace unit;

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

HardwareInterface::CurrentControlCallback g_fast_callback = nullptr;
HardwareInterface::SlowUpdateCallback g_slow_callback = nullptr;
uint32_t g_fast_count = 0U;
struct k_timer g_pwm_period_timer;

// Runs in the (simulated) timer interrupt, like the BCC interrupts: no kernel
// calls, no logging in the callbacks.
void OnPwmPeriod(struct k_timer*) {
  g_fast_callback();
  if (++g_fast_count >= kSlowUpdateDivider) {
    g_fast_count = 0U;
    g_slow_callback();
  }
}

bool Initialize(Frequency pwm_frequency,
                uint32_t,
                HardwareInterface::CurrentControlCallback current_control,
                HardwareInterface::SlowUpdateCallback slow_update) noexcept {
  const auto pwm_hz = static_cast<uint64_t>(pwm_frequency.Value());
  if (pwm_hz == 0U || current_control == nullptr || slow_update == nullptr) return false;
  g_fast_callback = current_control;
  g_slow_callback = slow_update;
  g_fast_count = 0U;

  const k_timeout_t period = K_NSEC(1'000'000'000ULL / pwm_hz);
  k_timer_init(&g_pwm_period_timer, OnPwmPeriod, nullptr);
  k_timer_start(&g_pwm_period_timer, period, period);
  return true;
}

system::ThreePhase<Current> GetPhaseCurrents() noexcept { return {}; }

system::ThreePhase<Voltage> GetPhaseVoltages() noexcept { return {}; }

constexpr float kSimDcLinkVoltage = 48.0F;

CurrentControlSamples GetCurrentControlSamples() noexcept {
  return CurrentControlSamples{{}, Voltage{kSimDcLinkVoltage}};
}

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
