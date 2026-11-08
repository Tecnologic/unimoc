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
 *    Counterpart of hardware/zephyr_stm32h7, closed around the motor plant of
 *    lib/sim. A kernel timer in simulated time runs kHalfPeriodsPerCycle PWM
 *    half periods per expiry, starting at a valley (two PWM periods, 62.5 us
 *    at 32 kHz PWM). Each half period
 *
 *     1. advances the plant with the duties in effect,
 *     2. ends with a current sample and the overcurrent and DC-link
 *        overvoltage checks, and
 *     3. at an update (the peaks; every boundary in
 *        ControlSampling::kEveryHalfPeriod) the duties written by the last
 *        control step take effect (the CCR preload of the STM32H7); after a
 *        valley (every sample in kEveryHalfPeriod) it calls SampleHandler()
 *        (STM32H7: the sample interrupt), which writes the duties on its
 *        control steps, and ObserverHandler() (STM32H7: pended interrupt)
 *        when the handler ended a control cycle.
 *
 *    With CONFIG_UNIMOC_SIM_LOW_SIDE_CURRENT_SENSE the samples measure as
 *    low-side shunts in PWM mode 2 do (the low-side pulses centered on the
 *    valley): nothing at the peak, and at the valley a phase only while its
 *    low side is on for at least kLowSideValidTime.
 *
 *    Timestamps are nanoseconds of virtual time. A change of the simulated
 *    Hall code is queued with the time of the edge, as the EXTI capture of
 *    the STM32H7 does (HardwareInterface::PopHallEvent()).
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
#include "hot_path_timing.hpp"
#include "motor_simulation.hpp"
#include "power_stage_interface.hpp"
#include "sim_control.hpp"
#include "zephyr_settings_storage.hpp"

namespace unimoc::hardware {

using namespace unit;

namespace {

static_assert(kHalfPeriodsPerCycle == 4U, "sim::AppliedDuties() returns four entries");

constexpr bool kLowSide = IS_ENABLED(CONFIG_UNIMOC_SIM_LOW_SIDE_CURRENT_SENSE);

// Low-side shunts: the low-side on time per PWM period the profile asks for
// a valid sample, and the shortest one the simulated shunt still measures.
constexpr Time kLowSideRequiredTime = 4.0e-6_s;
constexpr Time kLowSideValidTime = 2.0e-6_s;

const settings::SettingsProfile kSettingsProfile = [] {
  settings::SettingsProfile profile{};
  // A fixed Cyphal node-ID, so tools reach a fresh simulation without a
  // plug-and-play allocator (--node-id overrides it per process).
  profile.factory_settings.node_id = 10U;
  profile.factory_settings.pwm_frequency =
      32.0_kHz;  // a control step every half period, the observer at 16 kHz
  // Duty range of the sampling window and the minimum pulse.
  profile.factory_settings.svm_duty_min = 0.03_ratio;
  profile.factory_settings.svm_duty_max = 0.97_ratio;
  // The simulated Hall sensors are ideal and aligned (offset 0): fuse them,
  // and weight the flux observer down at low speed.
  profile.factory_settings.hall_enable = 1U;
  profile.factory_settings.pmsm_flux_obs_r_speed = 1.0_rad4_per_s2;
  profile.factory_settings.battery_drive_current_max = 50.0_A;
  profile.factory_settings.battery_charge_current_max = 20.0_A;
  profile.capabilities.max_phase_current = 60.0_A;  // hard trip limit of the power stage
  profile.capabilities.max_motor_current = 50.0_A;
  profile.capabilities.max_battery_drive_current = 50.0_A;
  profile.capabilities.max_battery_charge_current = 20.0_A;
  profile.factory_settings.dc_link_regen_derate_start = 55.0_V;  // as the STM32H7
  profile.factory_settings.dc_link_trip = 58.0_V;
  profile.capabilities.max_dc_link_voltage = 60.0_V;
  profile.capabilities.max_pwm_frequency = 64.0_kHz;  // as the STM32H7
  profile.capabilities.max_control_frequency = 64.0_kHz;
  if constexpr (kLowSide) {
    profile.capabilities.current_sense = settings::CurrentSense::kLowSide;
    profile.capabilities.min_low_side_time = kLowSideRequiredTime;
  }
  return profile;
}();

const settings::SettingsStorage kSettingsStorage = ZephyrSettingsStorage::Adapter();

// ----- Power stage ----------------------------------------------------------

// Mirrors the STM32H7: the external enable only allows the release, a break
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
  // As on the STM32H7: a trip of the cycle interrupt between the check and the
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
constexpr bool kTiming = IS_ENABLED(CONFIG_UNIMOC_HOT_PATH_TIMING);

::unimoc::sim::MotorSimulation g_plant;
sim::PlantTrace g_plant_trace;

bool g_initialized = false;
bool g_every_half_period = false;  // ControlSampling::kEveryHalfPeriod
Time g_half_period{};

CurrentSample g_sample{};    // the sample of the last valley
Duties g_active = kNeutral;  // duties in effect (the active compare values)
Duties g_next = kNeutral;    // written by the last control step: in effect from the next peak
// Trips of the samples since the control step last read one (as on the STM32H7).
bool g_pending_overcurrent = false;
bool g_pending_overvoltage = false;
std::array<Duties, kHalfPeriodsPerCycle> g_applied{kNeutral, kNeutral, kNeutral, kNeutral};
Current g_overcurrent_limit{std::numeric_limits<float>::infinity()};
Voltage g_overvoltage_limit{std::numeric_limits<float>::infinity()};
bool g_current_override = false;
system::ThreePhase<Current> g_override_currents{};
uint32_t g_cycle_count = 0U;

observer::HallEventQueue<16U> g_hall_events;
uint8_t g_last_hall_code = 0U;             // last queued code
std::optional<uint8_t> g_hall_override{};  // fault injection: the code the sensors report
uint64_t g_half_period_ns = 0U;

struct k_timer g_cycle_timer;
bool g_free_running = false;
bool g_paused = false;       // SetPaused(): Initialize() leaves the timer stopped
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

/// What low-side shunts read of @p currents with @p duties: at a valley the
/// phases whose low side is on long enough, nothing at a peak.
system::ThreePhase<Current> LowSideShunts(const system::ThreePhase<Current>& currents,
                                          const Duties& duties,
                                          const bool peak) noexcept {
  if (peak) {
    return {};
  }
  const Time kPeriod = 2.0F * g_half_period;
  const auto kRead = [kPeriod](const Current current, const Ratio duty) noexcept {
    return ((1.0_ratio - duty) * kPeriod < kLowSideValidTime) ? Current{} : current;
  };
  return {kRead(currents.a, duties.a), kRead(currents.b, duties.b), kRead(currents.c, duties.c)};
}

bool Exceeds(const system::ThreePhase<Current>& currents) noexcept {
  const Current kLimit = g_overcurrent_limit;
  return Abs(currents.a) > kLimit || Abs(currents.b) > kLimit || Abs(currents.c) > kLimit;
}

/// Copies the plant truth for the traces; the control step of the sample samples it with its own state.
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

/// The control step of the sample, then the observer when the step ended a control cycle.
void RunHandlers() noexcept {
  if constexpr (kTiming) {
    // Simulated time stands still inside a cycle: the times are 0, the counts are real.
    const uint32_t kStart = k_cycle_get_32();
    const bool kObserver = SampleHandler();
    const uint32_t kControlEnd = k_cycle_get_32();
    timing::ProbeOf(timing::Section::kControlCycle).Record(kControlEnd - kStart);
    timing::ProbeOf(timing::Section::kSampleIsr).Record(kControlEnd - kStart);
    if (kObserver) {
      ObserverHandler();
      timing::ProbeOf(timing::Section::kObserver).Record(k_cycle_get_32() - kControlEnd);
    }
  } else if (SampleHandler()) {
    ObserverHandler();
  }
}

// Runs in the (simulated) timer interrupt, like the STM32H7 interrupts: no kernel
// calls and no logging in the handlers.
void RunCycle() noexcept {
  if (!g_initialized) {
    return;
  }
  const uint64_t kStartNs = static_cast<uint64_t>(g_cycle_count) * g_cycle_ns;
  for (uint8_t k = 0U; k < kHalfPeriodsPerCycle; ++k) {
    g_applied[k] = g_active;
    g_plant.SetBridgeEnabled(g_power_stage.enabled);
    g_plant.SetPhaseDuties(g_active);
    (void)g_plant.Step(g_half_period);
    const uint64_t kBoundaryNs = kStartNs + ((k + 1U) * g_half_period_ns);
    CaptureHall(kBoundaryNs);
    UpdatePlantTrace();

    // Sample at the end of the half period, then the checks of every sample.
    const bool kPeak = k % 2U == 0U;
    CurrentSample sample{};
    sample.phase_currents = MeasuredCurrents();
    if constexpr (kLowSide) {
      sample.phase_currents = LowSideShunts(sample.phase_currents, g_applied[k], kPeak);
    }
    if (Exceeds(sample.phase_currents)) {
      PowerStageTrip();
      g_pending_overcurrent = true;
    }
    if (!(g_plant.Output().dc_link_voltage < g_overvoltage_limit)) {
      PowerStageTrip();
      g_pending_overvoltage = true;
    }
    if (kPeak || g_every_half_period) {
      g_active = g_next;  // the update: the preload of the last control step takes effect
    }
    if (kPeak && !g_every_half_period) {
      continue;
    }
    sample.overcurrent = g_pending_overcurrent;
    sample.overvoltage = g_pending_overvoltage;
    sample.dc_link = g_plant.Output().dc_link_voltage;
    sample.phase_voltages = g_plant.Output().terminal_voltages;
    sample.phase_voltages_valid = true;
    sample.timestamp = static_cast<uint32_t>(kBoundaryNs);
    g_sample = sample;
    RunHandlers();
  }
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

// Back to the first cycle: neutral duties, the sample of the plant at rest.
void ResetCycleState() noexcept {
  g_sample = CurrentSample{};
  g_sample.dc_link = g_plant.Output().dc_link_voltage;
  g_active = kNeutral;
  g_next = kNeutral;
  g_pending_overcurrent = false;
  g_pending_overvoltage = false;
  g_applied = {kNeutral, kNeutral, kNeutral, kNeutral};
  g_cycle_count = 0U;
}

bool Initialize(Frequency pwm_frequency, ControlSampling sampling, uint32_t /*unused*/) noexcept {
  const auto kPwmHz = static_cast<uint64_t>(pwm_frequency.Value());
  if (kPwmHz == 0U || (kLowSide && sampling == ControlSampling::kEveryHalfPeriod)) {
    return false;  // low-side shunts: no valid sample at the peak
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
  g_every_half_period = sampling == ControlSampling::kEveryHalfPeriod;
  g_half_period = Time{1.0F / (2.0F * pwm_frequency.Value())};
  g_cycle_ns = (kHalfPeriodsPerCycle * 1'000'000'000ULL) / (2U * kPwmHz);
  g_half_period_ns = 1'000'000'000ULL / (2U * kPwmHz);
  ResetCycleState();
  RestartHallCapture();

  if (!g_paused) {
    StartTimer();
  }
  return true;
}

// The cycle timer runs the control steps and the observer: stopped, neither runs.
void Stop() noexcept {
  if (g_initialized) {
    StopTimer();
  }
}

system::ThreePhase<Current> GetPhaseCurrents() noexcept { return MeasuredCurrents(); }

// The terminals against the negative rail, as the phase-voltage sense of the STM32H7 boards.
system::ThreePhase<Voltage> GetPhaseVoltages() noexcept { return g_plant.Output().terminal_voltages; }

TemperatureMeasurements GetTemperatureMeasurements() noexcept { return {}; }

void SetOvercurrentLimit(const Current limit) noexcept { g_overcurrent_limit = limit; }

void SetOvervoltageLimit(const Voltage limit) noexcept { g_overvoltage_limit = limit; }

void SetAdcTriggerOffset(uint32_t /*unused*/) noexcept {}

uint32_t GetTimerClockFrequency() noexcept { return 170'000'000U; }

// The observer is the only consumer; it runs in the cycle timer like the producer.
bool PopHallEvent(observer::HallEvent& event) noexcept { return g_hall_events.Pop(event); }

}  // namespace

// The simulation applies every duty in time.
uint32_t timing::DutyOverruns() noexcept { return 0U; }

namespace hot_path {

// The trips are reported once: with this sample, not again with the next.
CurrentSample GetSample() noexcept {
  g_pending_overcurrent = false;
  g_pending_overvoltage = false;
  return g_sample;
}

void SetDuties(const Duties& duties) noexcept { g_next = duties; }

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

void SetPaused(const bool paused) noexcept {
  g_paused = paused;
  SetFreeRunning(!paused && g_initialized);
}

bool IsPaused() noexcept { return g_paused; }

uint64_t CyclePeriodNs() noexcept { return g_cycle_ns; }

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

uint32_t RunUntil(const uint32_t cycle) noexcept {
  uint32_t run = 0U;
  while (g_cycle_count < cycle) {
    const bool kWasEnabled = g_power_stage.enabled;
    StepCycle();
    ++run;
    if (kWasEnabled && !g_power_stage.enabled) {
      break;
    }
  }
  return run;
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
                                                                              SetOvervoltageLimit,
                                                                              SetAdcTriggerOffset,
                                                                              GetTimerClockFrequency,
                                                                              PopHallEvent)};

}  // namespace unimoc::hardware
