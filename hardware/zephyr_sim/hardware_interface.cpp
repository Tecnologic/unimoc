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
 *    Counterpart of hardware/zephyr_bcc, closed around the motor plant of
 *    lib/sim. A kernel timer in simulated time stands in for the control
 *    cycle (kHalfPeriodsPerCycle PWM half periods, 16 kHz at 32 kHz PWM).
 *    Every expiry
 *
 *     1. calls ControlCycleHandler() (BCC: inside the last sample interrupt
 *        of the cycle) and then ObserverHandler() (BCC: pended interrupt) on
 *        the samples of the cycle that just ended, and
 *     2. advances the plant over the next kHalfPeriodsPerCycle half periods
 *        with the timing of the BCC: the first half period still uses the
 *        last entry of the previous plan, the next three use entries 0..2 of
 *        the new plan, entry 3 carries over. Each half period ends with a
 *        current sample and the per-sample overcurrent check.
 *
 *    Timestamps are nanoseconds of virtual time. A change of the simulated
 *    Hall code is queued with the time of the edge, as the EXTI capture of
 *    the BCC does (HardwareInterface::PopHallEvent()).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "hardware_interface.hpp"
#include <zephyr/kernel.h>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include "hot_path_interface.hpp"
#include "motor_simulation.hpp"
#include "power_stage_interface.hpp"
#include "sim_control.hpp"
#include "zephyr_settings_storage.hpp"

namespace unimoc::hardware {

using namespace unit;

namespace {

static_assert(kHalfPeriodsPerCycle == 4U, "sim::AppliedDuties() returns four entries");

const settings::SettingsProfile kSettingsProfile = [] {
  settings::SettingsProfile profile{};
  // A fixed Cyphal node-ID, so tools reach a fresh simulation without a
  // plug-and-play allocator (--node-id overrides it per process).
  profile.factory_settings.node_id = 10U;
  profile.factory_settings.pwm_frequency = 32.0_kHz;  // 16 kHz control cycle
  // Same duty range as the BCC (sampling window and minimum pulse, see there).
  profile.factory_settings.svm_duty_min = 0.03_ratio;
  profile.factory_settings.svm_duty_max = 0.97_ratio;
  // The simulated Hall sensors are ideal and aligned (offset 0): fuse them,
  // and weight the flux observer down at low speed.
  profile.factory_settings.hall_enable = 1U;
  profile.factory_settings.pmsm_flux_obs_r_speed = 1.0F;
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

// ----- Power stage ----------------------------------------------------------

// Mirrors the BCC: the external enable only allows the release, a break
// switches off and latches until it is acknowledged.
struct SimPowerStage {
  volatile bool external_enable = true;
  volatile bool enabled = false;
  volatile bool fault = false;
} g_power_stage;

bool PowerStageEnable() noexcept {
  if (g_power_stage.enabled) {
    return true;
  }
  if (!g_power_stage.external_enable || g_power_stage.fault) {
    return false;
  }
  g_power_stage.enabled = true;
  // As on the BCC: a trip of the cycle interrupt between the check and the
  // release must not be undone.
  if (g_power_stage.fault) {
    g_power_stage.enabled = false;
    return false;
  }
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
  if (g_power_stage.enabled) {
    return PowerStageState::kEnabled;
  }
  if (!g_power_stage.external_enable) {
    return PowerStageState::kBlocked;
  }
  return g_power_stage.fault ? PowerStageState::kFault : PowerStageState::kDisabled;
}

// ----- Plant and control cycle ----------------------------------------------

using Duties = system::ThreePhase<Ratio>;
constexpr Duties kNeutral{0.5_ratio, 0.5_ratio, 0.5_ratio};

::unimoc::sim::MotorSimulation g_plant;
sim::PlantTrace g_plant_trace;

bool g_initialized = false;
Time g_half_period{};

ControlCycleSamples g_samples{};  // samples of the cycle that just ended
DutyPlan g_plan{};                // written by the control cycle
Duties g_carry = kNeutral;        // entry 3 of the last plan: first half period of the next cycle
std::array<Duties, kHalfPeriodsPerCycle> g_applied{kNeutral, kNeutral, kNeutral, kNeutral};
float g_overcurrent_limit = std::numeric_limits<float>::infinity();
bool g_current_override = false;
system::ThreePhase<Current> g_override_currents{};
uint32_t g_cycle_count = 0U;

observer::HallEventQueue<16U> g_hall_events;
uint8_t g_last_hall_code = 0U;             // last queued code
std::optional<uint8_t> g_hall_override{};  // fault injection: the code the sensors report
uint64_t g_half_period_ns = 0U;

struct k_timer g_cycle_timer;
bool g_free_running = false;
uint64_t g_cycle_ns = 0U;    // exact cycle period
uint64_t g_elapsed_ns = 0U;  // ideal time of the last expiry since the start
uint64_t g_elapsed_us = 0U;  // the same, rounded down to the 1 us kernel tick

system::ThreePhase<Current> MeasuredCurrents() noexcept {
  return g_current_override ? g_override_currents : g_plant.Output().phase_currents;
}

uint8_t MeasuredHallCode() noexcept { return g_hall_override.value_or(g_plant.Output().hall.code); }

/// Queues the Hall code when it changed; @p time_ns is the end of the half period just simulated.
void CaptureHall(const uint64_t time_ns) noexcept {
  const uint8_t kCode = MeasuredHallCode();
  if (kCode == g_last_hall_code) {
    return;
  }
  uint64_t edge_ns = time_ns;
  const auto& hall = g_plant.Output().hall;
  if (!g_hall_override.has_value() && hall.edge) {
    edge_ns -= static_cast<uint64_t>(hall.edge_age.Value() * 1.0e9F);
  }
  g_last_hall_code = kCode;
  (void)g_hall_events.Push(observer::HallEvent{kCode, static_cast<uint32_t>(edge_ns)});
}

/// Starts the Hall capture again: the queue holds only the present code.
void RestartHallCapture() noexcept {
  g_hall_events.Clear();
  g_last_hall_code = 0U;
  CaptureHall(static_cast<uint64_t>(g_cycle_count) * g_cycle_ns);
}

bool Exceeds(const system::ThreePhase<Current>& currents) noexcept {
  const float kLimit = g_overcurrent_limit;
  return std::abs(currents.a.Value()) > kLimit || std::abs(currents.b.Value()) > kLimit ||
         std::abs(currents.c.Value()) > kLimit;
}

/// Copies the plant truth for the traces; the next control cycle samples it with its own state.
void UpdatePlantTrace() noexcept {
  const auto& output = g_plant.Output();
  g_plant_trace.i_a = output.phase_currents.a.Value();
  g_plant_trace.i_b = output.phase_currents.b.Value();
  g_plant_trace.i_c = output.phase_currents.c.Value();
  g_plant_trace.i_d = output.rotor_current.d.Value();
  g_plant_trace.i_q = output.rotor_current.q.Value();
  g_plant_trace.electrical_angle = output.electrical_angle.AngleInRevolution().Value();
  g_plant_trace.electrical_speed = output.electrical_speed.Value();
  g_plant_trace.mechanical_speed = output.mechanical_speed.Value();
  g_plant_trace.torque = output.electromagnetic_torque.Value();
}

// Runs in the (simulated) timer interrupt, like the BCC interrupts: no kernel
// calls and no logging in the handlers.
void RunCycle() noexcept {
  if (!g_initialized) {
    return;
  }
  ControlCycleHandler();
  ObserverHandler();

  const uint64_t kStartNs = static_cast<uint64_t>(g_cycle_count) * g_cycle_ns;
  ControlCycleSamples next{};
  for (uint8_t k = 0U; k < kHalfPeriodsPerCycle; ++k) {
    const Duties& duty = (k == 0U) ? g_carry : g_plan.duties[k - 1U];
    g_applied[k] = duty;
    g_plant.SetBridgeEnabled(g_power_stage.enabled);
    g_plant.SetPhaseDuties(duty);
    (void)g_plant.Step(g_half_period);
    CaptureHall(kStartNs + (k + 1U) * g_half_period_ns);

    // Sample at the end of the half period, then the per-sample check.
    next.phase_currents[k] = MeasuredCurrents();
    if (Exceeds(next.phase_currents[k])) {
      PowerStageTrip();
      next.overcurrent = true;
    }
  }
  next.dc_link = g_plant.Output().dc_link_voltage;
  next.timestamp = static_cast<uint32_t>(kStartNs + kHalfPeriodsPerCycle * g_half_period_ns);
  g_carry = g_plan.duties[kHalfPeriodsPerCycle - 1U];
  g_samples = next;
  UpdatePlantTrace();
  ++g_cycle_count;
}

// The cycle (62.5 us at 32 kHz PWM) is not a whole number of 1 us ticks:
// schedule each expiry from the exact elapsed time so the mean rate is exact.
void ScheduleNext() noexcept {
  g_elapsed_ns += g_cycle_ns;
  const uint64_t kDueUs = g_elapsed_ns / 1000U;
  k_timer_start(&g_cycle_timer, K_USEC(kDueUs - g_elapsed_us), K_NO_WAIT);
  g_elapsed_us = kDueUs;
}

void OnCycleTimer(struct k_timer* /*unused*/) {
  RunCycle();
  if (g_free_running) {
    ScheduleNext();
  }
}

void StartTimer() noexcept {
  g_free_running = true;
  g_elapsed_ns = 0U;
  g_elapsed_us = 0U;
  ScheduleNext();
}

void StopTimer() noexcept {
  g_free_running = false;
  k_timer_stop(&g_cycle_timer);
}

// Back to the first cycle: no plan, neutral duties, samples of the plant at rest.
void ResetCycleState() noexcept {
  g_samples = ControlCycleSamples{};
  g_samples.dc_link = g_plant.Output().dc_link_voltage;
  g_plan = DutyPlan{};
  g_carry = kNeutral;
  g_applied = {kNeutral, kNeutral, kNeutral, kNeutral};
  g_cycle_count = 0U;
}

bool Initialize(Frequency pwm_frequency, uint32_t /*unused*/) noexcept {
  const auto kPwmHz = static_cast<uint64_t>(pwm_frequency.Value());
  if (kPwmHz == 0U) {
    return false;
  }

  static bool timer_initialized = false;
  if (!timer_initialized) {
    k_timer_init(&g_cycle_timer, OnCycleTimer, nullptr);
    timer_initialized = true;
  }
  StopTimer();

  if (!g_plant.IsConfigured()) {
    (void)g_plant.Configure(kSettingsProfile.factory_settings);
  }

  g_initialized = true;
  g_half_period = Time{1.0F / (2.0F * pwm_frequency.Value())};
  g_cycle_ns = (kHalfPeriodsPerCycle * 1'000'000'000ULL) / (2U * kPwmHz);
  g_half_period_ns = 1'000'000'000ULL / (2U * kPwmHz);
  ResetCycleState();
  RestartHallCapture();

  StartTimer();
  return true;
}

// The cycle timer runs the control cycle and the observer: stopped, neither runs.
void Stop() noexcept {
  if (g_initialized) {
    StopTimer();
  }
}

system::ThreePhase<Current> GetPhaseCurrents() noexcept { return MeasuredCurrents(); }

system::ThreePhase<Voltage> GetPhaseVoltages() noexcept { return g_plant.Output().phase_voltages; }

TemperatureMeasurements GetTemperatureMeasurements() noexcept { return {}; }

void SetOvercurrentLimit(const Current limit) noexcept { g_overcurrent_limit = limit.Value(); }

void SetAdcTriggerOffset(uint32_t /*unused*/) noexcept {}

uint32_t GetTimerClockFrequency() noexcept { return 170'000'000U; }

// The observer is the only consumer; it runs in the cycle timer like the producer.
bool PopHallEvent(observer::HallEvent& event) noexcept { return g_hall_events.Pop(event); }

}  // namespace

namespace hot_path {

ControlCycleSamples GetControlCycleSamples() noexcept { return g_samples; }

void SetDutyPlan(const DutyPlan& plan) noexcept { g_plan = plan; }

void TripPowerStage() noexcept { PowerStageTrip(); }

}  // namespace hot_path

namespace sim {

void SetExternalEnable(const bool present) noexcept {
  g_power_stage.external_enable = present;
  if (!present) {
    PowerStageTrip();  // the break input latches, like BIF
  }
}

void SetPhaseCurrentOverride(const float a, const float b, const float c) noexcept {
  const unsigned int kIrqKey = irq_lock();
  g_override_currents = system::ThreePhase<Current>{Current{a}, Current{b}, Current{c}};
  g_current_override = true;
  irq_unlock(kIrqKey);
}

void ClearPhaseCurrentOverride() noexcept { g_current_override = false; }

void SetHallCodeOverride(const uint8_t code) noexcept {
  const unsigned int kIrqKey = irq_lock();
  g_hall_override = code;
  irq_unlock(kIrqKey);
}

void ClearHallCodeOverride() noexcept {
  const unsigned int kIrqKey = irq_lock();
  g_hall_override.reset();
  irq_unlock(kIrqKey);
}

::unimoc::sim::MotorSimulation& Plant() noexcept { return g_plant; }

const PlantTrace& PlantTraceSignals() noexcept { return g_plant_trace; }

void SetFreeRunning(const bool running) noexcept {
  if (running == g_free_running) {
    return;
  }
  if (running) {
    StartTimer();
  } else {
    StopTimer();
  }
}

void StepCycle() noexcept {
  const unsigned int kIrqKey = irq_lock();
  RunCycle();
  irq_unlock(kIrqKey);
}

uint64_t VirtualTimeNs() noexcept { return static_cast<uint64_t>(g_cycle_count) * g_cycle_ns; }

void Reset() noexcept {
  const unsigned int kIrqKey = irq_lock();
  g_plant.Reset();
  g_current_override = false;
  g_power_stage.enabled = false;
  g_power_stage.fault = !g_power_stage.external_enable;
  ResetCycleState();
  RestartHallCapture();
  irq_unlock(kIrqKey);
}

Snapshot TakeSnapshot() noexcept {
  const unsigned int kIrqKey = irq_lock();
  const Snapshot kSnapshot{g_cycle_count, VirtualTimeNs(), g_plant.Output(), g_applied};
  irq_unlock(kIrqKey);
  return kSnapshot;
}

void AdvanceCycles(const uint32_t count) noexcept {
  for (uint32_t i = 0U; i < count; ++i) {
    StepCycle();
  }
}

std::array<system::ThreePhase<unit::Ratio>, 4> AppliedDuties() noexcept { return g_applied; }

uint32_t CycleCount() noexcept { return g_cycle_count; }

}  // namespace sim

const PowerStageInterface kPowerStage{PowerStageEnable,
                                      PowerStageDisable,
                                      PowerStageTrip,
                                      PowerStageClearFault,
                                      GetPowerStageState};

const HardwareSettingsInterface kSettings{kSettingsProfile, kSettingsStorage, ZephyrSettingsStorage::Lock()};

const std::array<HardwareInterface, kMotorCount> kMotors = {HardwareInterface(Initialize,
                                                                              Stop,
                                                                              GetPhaseCurrents,
                                                                              GetPhaseVoltages,
                                                                              GetTemperatureMeasurements,
                                                                              SetOvercurrentLimit,
                                                                              SetAdcTriggerOffset,
                                                                              GetTimerClockFrequency,
                                                                              PopHallEvent)};

}  // namespace unimoc::hardware
