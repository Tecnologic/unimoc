/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file hardware_interface.hpp
 *    @brief Hardware callbacks for motor input and output.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include "hardware_interface_config.hpp"
#include "settings_profile.hpp"
#include "settings_storage.hpp"
#include "three_phase_system.hpp"

namespace unimoc::hardware {

/**
 * @brief ADC values of one control cycle (kHalfPeriodsPerCycle PWM half periods).
 *
 * The hardware samples the phase currents once per PWM half period, at every
 * peak and valley of the center-aligned counter. Sample k was taken at the
 * (k + 1)-th boundary after the previous control boundary, so the last sample
 * belongs to the boundary of this control cycle.
 */
struct ControlCycleSamples {
  /// Phase currents in A/B/C order, oldest first. The expected invariant is ia + ib + ic = 0.
  std::array<system::ThreePhase<unit::Current>, kHalfPeriodsPerCycle> phase_currents{};
  /// DC-link voltage of the last sample.
  unit::Voltage dc_link{};
  /// A sample of this cycle exceeded the overcurrent limit. The hardware layer
  /// has already tripped the power stage when this is set.
  bool overcurrent{false};
};

/**
 * @brief Phase duties for the kHalfPeriodsPerCycle PWM half periods after a control boundary.
 *
 * Entry k applies to the half period that starts k + 1 boundaries after the
 * control boundary: entry 0 is written directly into the PWM preload
 * registers, entries 1 to 3 are delivered by DMA at the following update events.
 */
struct DutyPlan {
  std::array<system::ThreePhase<unit::Ratio>, kHalfPeriodsPerCycle>
      duties{system::ThreePhase<unit::Ratio>{unit::Ratio{0.5F}, unit::Ratio{0.5F}, unit::Ratio{0.5F}},
             system::ThreePhase<unit::Ratio>{unit::Ratio{0.5F}, unit::Ratio{0.5F}, unit::Ratio{0.5F}},
             system::ThreePhase<unit::Ratio>{unit::Ratio{0.5F}, unit::Ratio{0.5F}, unit::Ratio{0.5F}},
             system::ThreePhase<unit::Ratio>{unit::Ratio{0.5F}, unit::Ratio{0.5F}, unit::Ratio{0.5F}}};
};

static_assert(kHalfPeriodsPerCycle == 4U, "DutyPlan initializer lists four entries");

/**
 * @brief Optional temperature measurements provided by a hardware target.
 */
struct TemperatureMeasurements {
  std::optional<unit::Temperature> cpu{};
  std::optional<unit::Temperature> bridge{};
  std::optional<unit::Temperature> motor{};
};

/**
 * @brief Severity used by the platform logging callback.
 */
enum class LogLevel : uint8_t { kDebug, kInfo, kWarning, kError };

/**
 * @brief Platform services used by the application runtime.
 */
class HardwareSystemInterface {
 public:
  using LogCallback = void (*)(LogLevel, const char*) noexcept;
  using WaitForInterruptCallback = void (*)() noexcept;

  /**
   * @brief Constructs the system hardware boundary.
   * @param log Emits a message using the target logging device.
   * @param wait_for_interrupt Suspends the CPU until an interrupt is pending.
   */
  HardwareSystemInterface(LogCallback log, WaitForInterruptCallback wait_for_interrupt) noexcept
      : log_(log), wait_for_interrupt_(wait_for_interrupt) {}

  /**
   * @brief Suspends execution until an interrupt wakes the CPU.
   */
  void WaitForInterrupt() const noexcept { wait_for_interrupt_(); }

  /**
   * @brief Emits a platform log message.
   */
  void Log(LogLevel level, const char* message) const noexcept { log_(level, message); }

 private:
  LogCallback log_;
  WaitForInterruptCallback wait_for_interrupt_;
};

/**
 * @brief Callback-based hardware interface for one motor.
 *
 * The hardware target supplies the callbacks; they are static platform
 * bindings. Interrupt layout (see docs/hotpath_plan.md):
 *  - The hardware samples the currents once per PWM half period and checks
 *    each sample against the overcurrent limit itself.
 *  - After kHalfPeriodsPerCycle samples it calls the control-cycle callback in
 *    the zero-latency context. The callback reads GetControlCycleSamples() and
 *    writes SetDutyPlan().
 *  - Then it runs the observer callback in a lower-priority interrupt that
 *    may be preempted by the sampling interrupt.
 */
class HardwareInterface {
 public:
  using ControlCycleCallback = void (*)() noexcept;
  using ObserverCallback = void (*)() noexcept;
  using InitializeCallback = bool (*)(unit::Frequency,
                                      uint32_t,
                                      ControlCycleCallback,
                                      ObserverCallback) noexcept;
  using GetPhaseCurrentsCallback = system::ThreePhase<unit::Current> (*)() noexcept;
  using GetPhaseVoltagesCallback = system::ThreePhase<unit::Voltage> (*)() noexcept;
  using GetControlCycleSamplesCallback = ControlCycleSamples (*)() noexcept;
  using GetTemperatureMeasurementsCallback = TemperatureMeasurements (*)() noexcept;
  using SetDutyPlanCallback = void (*)(const DutyPlan&) noexcept;
  using SetOvercurrentLimitCallback = void (*)(unit::Current) noexcept;
  using SetAdcTriggerOffsetCallback = void (*)(uint32_t) noexcept;
  using GetTimerClockFrequencyCallback = uint32_t (*)() noexcept;

  /**
   * @brief Constructs a motor hardware interface.
   * @param initialize Initializes the hardware for the requested PWM frequency
   *                   and ADC trigger offset, then registers both callbacks.
   * @param get_phase_currents Reads the three phase currents.
   * @param get_phase_voltages Reads the three phase voltages.
   * @param get_control_cycle_samples Returns the samples of the finished control cycle.
   * @param get_temperature_measurements Reads optional temperature channels.
   * @param set_duty_plan Writes the duties of the next kHalfPeriodsPerCycle half periods.
   * @param set_overcurrent_limit Sets the per-sample overcurrent limit.
   * @param set_adc_trigger_offset Updates the ADC trigger compare value.
   * @param get_timer_clock_frequency Returns the timer input clock in Hz.
   */
  HardwareInterface(InitializeCallback initialize,
                    GetPhaseCurrentsCallback get_phase_currents,
                    GetPhaseVoltagesCallback get_phase_voltages,
                    GetControlCycleSamplesCallback get_control_cycle_samples,
                    GetTemperatureMeasurementsCallback get_temperature_measurements,
                    SetDutyPlanCallback set_duty_plan,
                    SetOvercurrentLimitCallback set_overcurrent_limit,
                    SetAdcTriggerOffsetCallback set_adc_trigger_offset,
                    GetTimerClockFrequencyCallback get_timer_clock_frequency)
      : initialize_(initialize),
        get_phase_currents_(get_phase_currents),
        get_phase_voltages_(get_phase_voltages),
        get_control_cycle_samples_(get_control_cycle_samples),
        get_temperature_measurements_(get_temperature_measurements),
        set_duty_plan_(set_duty_plan),
        set_overcurrent_limit_(set_overcurrent_limit),
        set_adc_trigger_offset_(set_adc_trigger_offset),
        get_timer_clock_frequency_(get_timer_clock_frequency) {}

  /**
   * @brief Initializes the hardware interface and its interrupt sources.
   * @param pwm_frequency Requested PWM switching frequency.
   * @param adc_trigger_offset ADC trigger compare offset in timer ticks.
   * @param control_cycle_callback Zero-latency callback once per control cycle.
   * @param observer_callback Lower-priority callback after each control cycle.
   * @return True when initialization succeeds.
   */
  [[nodiscard]] bool Initialize(unit::Frequency pwm_frequency,
                                uint32_t adc_trigger_offset,
                                ControlCycleCallback control_cycle_callback,
                                ObserverCallback observer_callback) const noexcept {
    return initialize_(pwm_frequency, adc_trigger_offset, control_cycle_callback, observer_callback);
  }

  /**
   * @brief Reads the three phase currents.
   * @return The measured phase currents.
   */
  [[nodiscard]] system::ThreePhase<unit::Current> GetPhaseCurrents() const noexcept {
    return get_phase_currents_();
  }

  /**
   * @brief Reads the three phase voltages.
   * @return The measured phase voltages.
   */
  [[nodiscard]] system::ThreePhase<unit::Voltage> GetPhaseVoltages() const noexcept {
    return get_phase_voltages_();
  }

  /**
   * @brief Returns the samples of the control cycle that just finished.
   *
   * Only valid inside the control-cycle callback.
   */
  [[nodiscard]] ControlCycleSamples GetControlCycleSamples() const noexcept {
    return get_control_cycle_samples_();
  }

  /**
   * @brief Reads the optional CPU, bridge, and motor temperatures.
   */
  [[nodiscard]] TemperatureMeasurements GetTemperatureMeasurements() const noexcept {
    return get_temperature_measurements_();
  }

  /**
   * @brief Writes the phase duties of the next kHalfPeriodsPerCycle half periods.
   *
   * Must be called inside the control-cycle callback, before the first
   * boundary after the control boundary.
   *
   * @param plan Normalized duties, entry 0 for the first half period.
   */
  void SetDutyPlan(const DutyPlan& plan) const noexcept { set_duty_plan_(plan); }

  /**
   * @brief Sets the limit of the per-sample overcurrent check.
   *
   * The hardware compares every sample against it (in raw ADC counts where
   * possible) and trips the power stage at once when a phase exceeds it.
   *
   * @param limit Phase-current magnitude; infinity disables the check.
   */
  void SetOvercurrentLimit(unit::Current limit) const noexcept { set_overcurrent_limit_(limit); }

  /**
   * @brief Updates the timer compare value that triggers ADC conversion.
   * @param offset Trigger offset in timer ticks.
   */
  void SetAdcTriggerOffset(uint32_t offset) const noexcept { set_adc_trigger_offset_(offset); }

  /**
   * @brief Returns the timer input clock used for PWM timing calculations.
   */
  [[nodiscard]] uint32_t GetTimerClockFrequency() const noexcept { return get_timer_clock_frequency_(); }

 private:
  InitializeCallback initialize_;
  GetPhaseCurrentsCallback get_phase_currents_;
  GetPhaseVoltagesCallback get_phase_voltages_;
  GetControlCycleSamplesCallback get_control_cycle_samples_;
  GetTemperatureMeasurementsCallback get_temperature_measurements_;
  SetDutyPlanCallback set_duty_plan_;
  SetOvercurrentLimitCallback set_overcurrent_limit_;
  SetAdcTriggerOffsetCallback set_adc_trigger_offset_;
  GetTimerClockFrequencyCallback get_timer_clock_frequency_;
};

/**
 * @brief Hardware-owned settings profile and persistence boundary.
 *
 * The profile is immutable and normally resides in target read-only memory.
 * The storage callbacks address the separate writable settings image.
 */
class HardwareSettingsInterface {
 public:
  /**
   * @brief Constructs the settings hardware boundary.
   * @param profile Immutable factory settings and hardware capabilities.
   * @param storage Platform callbacks for the writable settings image.
   */
  HardwareSettingsInterface(const settings::SettingsProfile& profile,
                            settings::SettingsStorage storage) noexcept
      : profile_{profile}, storage_{storage} {}

  /**
   * @brief Returns the immutable target profile.
   */
  [[nodiscard]] const settings::SettingsProfile& GetSettingsProfile() const noexcept { return profile_; }

  /**
   * @brief Returns the platform settings storage callbacks.
   */
  [[nodiscard]] const settings::SettingsStorage& GetSettingsStorage() const noexcept { return storage_; }

 private:
  const settings::SettingsProfile& profile_;
  settings::SettingsStorage storage_;
};

extern HardwareSettingsInterface settings;  ///< Node settings hardware boundary.
extern HardwareInterface motor[MOTORS];     ///< Global array of hardware interfaces for motors
extern HardwareSystemInterface runtime;     ///< Application system hardware boundary.

}  // namespace unimoc::hardware
