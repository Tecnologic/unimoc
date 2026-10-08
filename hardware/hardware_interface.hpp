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

#include <cstdint>
#include <optional>
#include "hardware_interface_config.hpp"
#include "settings_profile.hpp"
#include "settings_storage.hpp"
#include "three_phase_system.hpp"

namespace unimoc::hardware {

/**
 * @brief ADC values needed by the fast current-control loop.
 */
struct CurrentControlSamples {
  /// Phase currents in A/B/C order. The expected invariant is ia + ib + ic = 0.
  system::ThreePhase<unit::Current> phase_currents{};
  unit::Voltage dc_link{};
};

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
 * The hardware target supplies the callbacks. The interface owns the callback
 * callbacks are static platform bindings. The fast-loop callbacks are safe to
 * invoke from interrupt context.
 */
class HardwareInterface {
 public:
  using CurrentControlCallback = void (*)() noexcept;
  using SlowUpdateCallback = void (*)() noexcept;
  using InitializeCallback = bool (*)(unit::Frequency, uint32_t, CurrentControlCallback, SlowUpdateCallback) noexcept;
  using GetPhaseCurrentsCallback = system::ThreePhase<unit::Current> (*)() noexcept;
  using GetPhaseVoltagesCallback = system::ThreePhase<unit::Voltage> (*)() noexcept;
  using GetCurrentControlSamplesCallback = CurrentControlSamples (*)() noexcept;
  using GetTemperatureMeasurementsCallback = TemperatureMeasurements (*)() noexcept;
  using GetPhaseDutiesCallback = system::ThreePhase<unit::Ratio> (*)() noexcept;
  using SetPhaseDutiesCallback = void (*)(const system::ThreePhase<unit::Ratio>&) noexcept;
  using SetAdcTriggerOffsetCallback = void (*)(uint32_t) noexcept;
  using GetTimerClockFrequencyCallback = uint32_t (*)() noexcept;

  /**
   * @brief Constructs a motor hardware interface.
   * @param initialize Initializes the hardware for the requested PWM frequency
   *                   and ADC trigger offset, then registers the ADC callback.
   * @param get_phase_currents Reads the three phase currents.
   * @param get_phase_voltages Reads the three phase voltages.
   * @param get_current_control_samples Reads the fast-loop ADC samples.
   * @param get_temperature_measurements Reads optional temperature channels.
   * @param get_phase_duties Reads the currently applied PWM duties.
   * @param set_phase_duties Writes the three phase PWM duties.
   * @param set_adc_trigger_offset Updates the ADC trigger compare value.
   * @param get_timer_clock_frequency Returns the timer input clock in Hz.
   */
  HardwareInterface(InitializeCallback initialize,
                    GetPhaseCurrentsCallback get_phase_currents,
                    GetPhaseVoltagesCallback get_phase_voltages,
                    GetCurrentControlSamplesCallback get_current_control_samples,
                    GetTemperatureMeasurementsCallback get_temperature_measurements,
                    GetPhaseDutiesCallback get_phase_duties,
                    SetPhaseDutiesCallback set_phase_duties,
                    SetAdcTriggerOffsetCallback set_adc_trigger_offset,
                    GetTimerClockFrequencyCallback get_timer_clock_frequency)
      : initialize_(initialize),
        get_phase_currents_(get_phase_currents),
        get_phase_voltages_(get_phase_voltages),
        get_current_control_samples_(get_current_control_samples),
        get_temperature_measurements_(get_temperature_measurements),
        get_phase_duties_(get_phase_duties),
        set_phase_duties_(set_phase_duties),
        set_adc_trigger_offset_(set_adc_trigger_offset),
        get_timer_clock_frequency_(get_timer_clock_frequency) {}

  /**
   * @brief Initializes the hardware interface and ADC interrupt source.
   * @param pwm_frequency Requested PWM switching frequency.
   * @param adc_trigger_offset ADC trigger compare offset in timer ticks.
   * @param current_control_callback Callback invoked after an ADC sequence.
   * @param slow_update_callback Callback invoked by the lower-priority update interrupt.
   * @return True when initialization succeeds.
   */
  [[nodiscard]] bool Initialize(unit::Frequency pwm_frequency,
                                uint32_t adc_trigger_offset,
                                CurrentControlCallback current_control_callback,
                                SlowUpdateCallback slow_update_callback) const noexcept {
    return initialize_(pwm_frequency, adc_trigger_offset, current_control_callback, slow_update_callback);
  }

  /**
   * @brief Reads the three phase currents.
   * @return The measured phase currents.
   */
  [[nodiscard]] system::ThreePhase<unit::Current> GetPhaseCurrents() const noexcept { return get_phase_currents_(); }

  /**
   * @brief Reads the three phase voltages.
   * @return The measured phase voltages.
   */
  [[nodiscard]] system::ThreePhase<unit::Voltage> GetPhaseVoltages() const noexcept { return get_phase_voltages_(); }

  /**
   * @brief Reads the ADC values used by the fast current-control loop.
   */
  [[nodiscard]] CurrentControlSamples GetCurrentControlSamples() const noexcept { return get_current_control_samples_(); }

  /**
   * @brief Reads the optional CPU, bridge, and motor temperatures.
   */
  [[nodiscard]] TemperatureMeasurements GetTemperatureMeasurements() const noexcept { return get_temperature_measurements_(); }

  /**
   * @brief Reads the currently applied normalized PWM duties.
   */
  [[nodiscard]] system::ThreePhase<unit::Ratio> GetPhaseDuties() const noexcept { return get_phase_duties_(); }

  /**
   * @brief Writes the three phase PWM duties.
   * @param duties Normalized duties for phases A, B, and C.
   */
  void SetPhaseDuties(const system::ThreePhase<unit::Ratio>& duties) const noexcept { set_phase_duties_(duties); }

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
  GetCurrentControlSamplesCallback get_current_control_samples_;
  GetTemperatureMeasurementsCallback get_temperature_measurements_;
  GetPhaseDutiesCallback get_phase_duties_;
  SetPhaseDutiesCallback set_phase_duties_;
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
  HardwareSettingsInterface(const settings::SettingsProfile& profile, settings::SettingsStorage storage) noexcept
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
