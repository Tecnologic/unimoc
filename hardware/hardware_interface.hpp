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
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include "hall_capture.hpp"
#include "hardware_interface_config.hpp"
#include "settings_profile.hpp"
#include "settings_storage.hpp"
#include "three_phase_system.hpp"

namespace unimoc::hardware {

/**
 * @brief ADC values of one current sample.
 *
 * The hardware samples the phase currents once per half period, at every peak
 * and valley of the center-aligned counter (the boundaries), and checks each
 * sample. The sample handler works on the sample of the boundary that just
 * passed (ControlSampling).
 */
struct CurrentSample {
  /// Phase currents in A/B/C order. The expected invariant is ia + ib + ic = 0.
  system::ThreePhase<unit::Current> phase_currents;
  /// DC-link voltage of the sample.
  unit::Voltage dc_link;
  /// Terminal voltages of the phases against the negative DC rail, converted
  /// with the same trigger. Only the line-to-line part carries information: the
  /// common mode depends on the switch state. With the outputs open they are the
  /// back-EMF of the spinning motor (plus a common mode).
  system::ThreePhase<unit::Voltage> phase_voltages;
  /// The platform measures phase_voltages (a phase-voltage sense is fitted).
  bool phase_voltages_valid{false};
  /// This sample or one not read since (a peak, a skipped valley) exceeded the
  /// overcurrent limit. The hardware layer has already tripped the power stage
  /// when this is set.
  bool overcurrent{false};
  /// This DC-link sample or one not read since reached the overvoltage limit.
  /// The hardware layer has already tripped the power stage when this is set.
  bool overvoltage{false};
  /// Time of the sample (its boundary) in ticks of the timestamp clock
  /// (kTimestampFrequency), the clock of the Hall events.
  uint32_t timestamp{0U};
};

/// Which samples the platform hands to SampleHandler(), and where the duties take effect.
enum class ControlSampling : uint8_t {
  /// Every valley (the middle of the high-side pulses); the duties take
  /// effect at the next peak only, every pulse stays symmetric (RCR = 1).
  kValleys = 0,
  /// Every peak and every valley; the duties take effect at the next
  /// boundary (RCR = 0).
  kEveryHalfPeriod = 1,
};

/**
 * @brief How long the measured signals lag the motor at the sample instant.
 *
 * The sample instant is the boundary at which the currents are sampled. A
 * low pass delays its signal by its time constant (the group delay well below
 * its corner); a signal converted later in the ADC sequence lags less by that
 * time. The observer compensates both delays: the current delay in the Park
 * angle of the control step, the phase-voltage delay in the coast tracking.
 */
struct SenseDelays {
  unit::Time current;        ///< Amplifier and RC filter of the phase currents.
  unit::Time phase_voltage;  ///< Divider and RC filters of the phase voltages, less their later sampling.
};

/**
 * @brief Optional temperature measurements provided by a hardware target.
 */
struct TemperatureMeasurements {
  std::optional<unit::Temperature> cpu;
  std::optional<unit::Temperature> bridge;
  std::optional<unit::Temperature> motor;
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

  /**
   * @brief Constructs the system hardware boundary.
   * @param log Emits a message using the target logging device.
   */
  explicit HardwareSystemInterface(LogCallback log) noexcept : log_(log) {}

  /**
   * @brief Emits a platform log message.
   */
  void Log(LogLevel level, const char* message) const noexcept { log_(level, message); }

 private:
  LogCallback log_;
};

/**
 * @brief Callback-based hardware interface for one motor.
 *
 * The hardware target supplies the callbacks; they are static platform
 * bindings for the thread context (setup, startup FSM, monitoring). The
 * zero-latency hot path does not use them: it reaches the hardware through
 * hot_path_interface.hpp, bound at link time. Interrupt layout (see
 * docs/hotpath_plan.md):
 *  - The hardware samples the currents and the DC link once per PWM half
 *    period and checks each sample against the overcurrent and the
 *    overvoltage limit itself.
 *  - After every valley sample, or every sample (ControlSampling), it calls
 *    SampleHandler() in the zero-latency context, which writes the duties on
 *    its control steps; they take effect at the next peak, or the next
 *    boundary (settings::ControlTiming).
 *  - When SampleHandler() ends a control cycle, it runs ObserverHandler() in a
 *    lower-priority interrupt that may be preempted by the sampling interrupt.
 */
class HardwareInterface {
 public:
  using InitializeCallback = bool (*)(unit::Frequency, ControlSampling, uint32_t) noexcept;
  using StopCallback = void (*)() noexcept;
  using GetPhaseCurrentsCallback = system::ThreePhase<unit::Current> (*)() noexcept;
  using GetPhaseVoltagesCallback = system::ThreePhase<unit::Voltage> (*)() noexcept;
  using GetTemperatureMeasurementsCallback = TemperatureMeasurements (*)() noexcept;
  using SetOvercurrentLimitCallback = void (*)(unit::Current) noexcept;
  using SetOvervoltageLimitCallback = void (*)(unit::Voltage) noexcept;
  using SetAdcTriggerOffsetCallback = void (*)(uint32_t) noexcept;
  using GetTimerClockFrequencyCallback = uint32_t (*)() noexcept;
  using PopHallEventCallback = bool (*)(observer::HallEvent&) noexcept;
  using GetSenseDelaysCallback = SenseDelays (*)() noexcept;

  /**
   * @brief Constructs a motor hardware interface.
   * @param initialize Initializes the hardware for the requested PWM frequency
   *                   and ADC trigger offset and starts the interrupts.
   * @param stop Stops the interrupts and the cycle source (see Stop()).
   * @param get_phase_currents Reads the three phase currents.
   * @param get_phase_voltages Reads the three phase voltages.
   * @param get_temperature_measurements Reads optional temperature channels.
   * @param set_overcurrent_limit Sets the per-sample overcurrent limit.
   * @param set_overvoltage_limit Sets the per-sample DC-link overvoltage limit.
   * @param set_adc_trigger_offset Updates the ADC trigger compare value.
   * @param get_timer_clock_frequency Returns the timer input clock in Hz.
   * @param pop_hall_event Takes the oldest captured change of the Hall code.
   * @param get_sense_delays Returns the delays of the measured signals.
   */
  HardwareInterface(InitializeCallback initialize,
                    StopCallback stop,
                    GetPhaseCurrentsCallback get_phase_currents,
                    GetPhaseVoltagesCallback get_phase_voltages,
                    GetTemperatureMeasurementsCallback get_temperature_measurements,
                    SetOvercurrentLimitCallback set_overcurrent_limit,
                    SetOvervoltageLimitCallback set_overvoltage_limit,
                    SetAdcTriggerOffsetCallback set_adc_trigger_offset,
                    GetTimerClockFrequencyCallback get_timer_clock_frequency,
                    PopHallEventCallback pop_hall_event,
                    GetSenseDelaysCallback get_sense_delays)
      : initialize_(initialize),
        stop_(stop),
        get_phase_currents_(get_phase_currents),
        get_phase_voltages_(get_phase_voltages),
        get_temperature_measurements_(get_temperature_measurements),
        set_overcurrent_limit_(set_overcurrent_limit),
        set_overvoltage_limit_(set_overvoltage_limit),
        set_adc_trigger_offset_(set_adc_trigger_offset),
        get_timer_clock_frequency_(get_timer_clock_frequency),
        pop_hall_event_(pop_hall_event),
        get_sense_delays_(get_sense_delays) {}

  /// The delays of the measured signals (read when the controller starts).
  [[nodiscard]] SenseDelays GetSenseDelays() const noexcept { return get_sense_delays_(); }

  /**
   * @brief Initializes the hardware and starts its interrupts.
   *
   * From then on the platform calls SampleHandler() after every sample of
   * @p sampling and ObserverHandler() after each control cycle
   * (hot_path_interface.hpp).
   *
   * @param pwm_frequency Requested PWM switching frequency.
   * @param sampling The samples of the control and where its duties take effect.
   * @param adc_trigger_offset ADC trigger compare offset in timer ticks.
   * @return True when initialization succeeds.
   */
  [[nodiscard]] bool Initialize(unit::Frequency pwm_frequency,
                                ControlSampling sampling,
                                uint32_t adc_trigger_offset) const noexcept {
    return initialize_(pwm_frequency, sampling, adc_trigger_offset);
  }

  /**
   * @brief Stops the cycle source before a new Initialize().
   *
   * Afterwards neither SampleHandler() nor ObserverHandler() runs and
   * the hardware is in a state Initialize() may reprogram (STM32H7: interrupts
   * off, timer and ADC sequences stopped, ADCs disabled). The power
   * stage must be disabled. Does nothing before the first Initialize().
   */
  void Stop() const noexcept { stop_(); }

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
   * @brief Reads the optional CPU, bridge, and motor temperatures.
   */
  [[nodiscard]] TemperatureMeasurements GetTemperatureMeasurements() const noexcept {
    return get_temperature_measurements_();
  }

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
   * @brief Sets the limit of the per-sample DC-link overvoltage check.
   *
   * The hardware compares the DC-link sample of every half period against it
   * (in raw ADC counts where possible) and trips the power stage at once when
   * it reaches the limit, like the overcurrent check.
   *
   * @param limit Uncalibrated DC-link voltage; infinity disables the check.
   */
  void SetOvervoltageLimit(unit::Voltage limit) const noexcept { set_overvoltage_limit_(limit); }

  /**
   * @brief Updates the timer compare value that triggers ADC conversion.
   * @param offset Trigger offset in timer ticks.
   */
  void SetAdcTriggerOffset(uint32_t offset) const noexcept { set_adc_trigger_offset_(offset); }

  /**
   * @brief Returns the timer input clock used for PWM timing calculations.
   */
  [[nodiscard]] uint32_t GetTimerClockFrequency() const noexcept { return get_timer_clock_frequency_(); }

  /**
   * @brief Takes the oldest captured change of the Hall code.
   *
   * The platform captures every change of the Hall code with its timestamp
   * (STM32H7: EXTI interrupt and the DWT cycle counter; native_sim: the edges of
   * the simulated sensors in virtual time) and the code at Initialize().
   * The observer takes them after each control cycle; it is the only reader.
   *
   * @param event Receives the code and its timestamp (kTimestampFrequency ticks).
   * @return False when no change is pending.
   */
  [[nodiscard]] bool PopHallEvent(observer::HallEvent& event) const noexcept {
    return pop_hall_event_(event);
  }

 private:
  InitializeCallback initialize_;
  StopCallback stop_;
  GetPhaseCurrentsCallback get_phase_currents_;
  GetPhaseVoltagesCallback get_phase_voltages_;
  GetTemperatureMeasurementsCallback get_temperature_measurements_;
  SetOvercurrentLimitCallback set_overcurrent_limit_;
  SetOvervoltageLimitCallback set_overvoltage_limit_;
  SetAdcTriggerOffsetCallback set_adc_trigger_offset_;
  GetTimerClockFrequencyCallback get_timer_clock_frequency_;
  PopHallEventCallback pop_hall_event_;
  GetSenseDelaysCallback get_sense_delays_;
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
   * @param lock Serializes the threads that share the settings store.
   */
  HardwareSettingsInterface(const settings::SettingsProfile& profile,
                            settings::SettingsStorage storage,
                            settings::SettingsLock lock) noexcept
      : profile_{&profile}, storage_{storage}, lock_{lock} {}

  /**
   * @brief Returns the immutable target profile.
   */
  [[nodiscard]] const settings::SettingsProfile& GetSettingsProfile() const noexcept { return *profile_; }

  /**
   * @brief Returns the platform settings storage callbacks.
   */
  [[nodiscard]] const settings::SettingsStorage& GetSettingsStorage() const noexcept { return storage_; }

  /**
   * @brief Returns the platform lock of the settings store.
   */
  [[nodiscard]] const settings::SettingsLock& GetSettingsLock() const noexcept { return lock_; }

 private:
  const settings::SettingsProfile* profile_;
  settings::SettingsStorage storage_;
  settings::SettingsLock lock_;
};

// Platform boundary objects, defined once per platform (hardware/zephyr_*).
// All their methods are const: the objects only bind callbacks.
extern const HardwareSettingsInterface kSettings;                 ///< Node settings hardware boundary.
extern const std::array<HardwareInterface, kMotorCount> kMotors;  ///< Hardware interfaces of the motors.
extern const HardwareSystemInterface kRuntime;                    ///< Application system hardware boundary.

}  // namespace unimoc::hardware
