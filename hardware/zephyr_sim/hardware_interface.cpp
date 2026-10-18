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
 *    stands in for the PWM half period: every expiry runs the fast callback (BCC:
 *    ADC1 end of injected sequence) and every kSlowUpdateDivider-th expiry the
 *    slow callback (BCC: TIM8 update event with repetition counter), the fast
 *    one first, as on the target. The analog side is a constant stub for now:
 *    zero phase currents and a fixed DC link voltage.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "hardware_interface.hpp"
#include "power_stage_interface.hpp"
#include "ram_settings_storage.hpp"
#include "sim_control.hpp"

#include <zephyr/kernel.h>

namespace unimoc::hardware {

using namespace unit;

namespace {

const settings::SettingsProfile kSettingsProfile = [] {
  settings::SettingsProfile profile{};
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

HardwareInterface::CurrentControlCallback g_fast_callback = nullptr;
HardwareInterface::SlowUpdateCallback g_slow_callback = nullptr;
uint32_t g_fast_count = 0U;
struct k_timer g_pwm_half_period_timer;

// Runs in the (simulated) timer interrupt, like the BCC interrupts: no kernel
// calls, no logging in the callbacks.
void OnPwmHalfPeriod(struct k_timer*) {
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

  // One fast step per half PWM period, as on the BCC.
  const k_timeout_t period = K_NSEC(500'000'000ULL / pwm_hz);
  k_timer_init(&g_pwm_half_period_timer, OnPwmHalfPeriod, nullptr);
  k_timer_start(&g_pwm_half_period_timer, period, period);
  return true;
}

system::ThreePhase<Current> GetPhaseCurrents() noexcept { return {}; }

system::ThreePhase<Voltage> GetPhaseVoltages() noexcept { return {}; }

constexpr float kSimDcLinkVoltage = 48.0F;

system::ThreePhase<Current> g_stub_currents{};

CurrentControlSamples GetCurrentControlSamples() noexcept {
  return CurrentControlSamples{g_stub_currents, Voltage{kSimDcLinkVoltage}};
}

TemperatureMeasurements GetTemperatureMeasurements() noexcept { return {}; }

system::ThreePhase<Ratio> phase_duties{0.5_ratio, 0.5_ratio, 0.5_ratio};

system::ThreePhase<Ratio> GetPhaseDuties() noexcept { return phase_duties; }

void SetPhaseDuties(const system::ThreePhase<Ratio>& duties) noexcept { phase_duties = duties; }

void SetAdcTriggerOffset(uint32_t) noexcept {}

uint32_t GetTimerClockFrequency() noexcept { return 170'000'000U; }

}  // namespace

namespace {

// Mirrors the BCC: the external enable only allows the release, a break
// switches off and latches until it is acknowledged.
struct SimPowerStage {
  volatile bool external_enable = true;
  volatile bool enabled = false;
  volatile bool fault = false;
} g_power_stage;

bool PowerStageEnable() noexcept {
  if (g_power_stage.enabled) return true;
  if (!g_power_stage.external_enable || g_power_stage.fault) return false;
  g_power_stage.enabled = true;
  return true;
}

void PowerStageDisable() noexcept { g_power_stage.enabled = false; }

void PowerStageTrip() noexcept {
  g_power_stage.enabled = false;
  g_power_stage.fault = true;
}

// An active break input latches again, as BIF does on the hardware.
void PowerStageClearFault() noexcept { g_power_stage.fault = !g_power_stage.external_enable; }

PowerStageState GetPowerStageState() noexcept {
  if (g_power_stage.enabled) return PowerStageState::kEnabled;
  if (!g_power_stage.external_enable) return PowerStageState::kBlocked;
  return g_power_stage.fault ? PowerStageState::kFault : PowerStageState::kDisabled;
}

}  // namespace

namespace sim {

void SetExternalEnable(const bool present) noexcept {
  g_power_stage.external_enable = present;
  if (!present) PowerStageTrip();  // the break input latches, like BIF
}

void SetPhaseCurrents(const float a, const float b, const float c) noexcept {
  g_stub_currents = system::ThreePhase<Current>{Current{a}, Current{b}, Current{c}};
}

}  // namespace sim

PowerStageInterface power_stage{
    PowerStageEnable, PowerStageDisable, PowerStageTrip, PowerStageClearFault, GetPowerStageState};

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
