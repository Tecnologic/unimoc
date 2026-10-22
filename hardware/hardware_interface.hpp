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
  unit::Voltage dc_link;
  /// A sample of this cycle exceeded the overcurrent limit. The hardware layer
  /// has already tripped the power stage when this is set.
  bool overcurrent{false};
  /// Time of the last sample (the control boundary) in ticks of the
  /// timestamp clock (kTimestampFrequency), the clock of the Hall events.
  uint32_t timestamp{0U};
};

/**
 * @brief Phase duties for the kHalfPeriodsPerCycle PWM half periods after a control boundary.
 *
 * Entry k applies to the half period that starts k + 1 boundaries after the
 * control boundary. On the BCC entry 0 is written directly into the PWM
 * preload registers and entries 1 to 3 are delivered by DMA at the following
 * update events (hot_path::SetDutyPlan()).
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
 *  - The hardware samples the currents once per PWM half period and checks
 *    each sample against the overcurrent limit itself.
 *  - After kHalfPeriodsPerCycle samples it calls ControlCycleHandler() in the
 *    zero-latency context.
 *  - Then it runs ObserverHandler() in a lower-priority interrupt that may be
 *    preempted by the sampling interrupt.
 */
class HardwareInterface {
 public:
  using InitializeCallback = bool (*)(unit::Frequency, uint32_t) noexcept;
  using GetPhaseCurrentsCallback = system::ThreePhase<unit::Current> (*)() noexcept;
  using GetPhaseVoltagesCallback = system::ThreePhase<unit::Voltage> (*)() noexcept;
  using GetTemperatureMeasurementsCallback = TemperatureMeasurements (*)() noexcept;
  using SetOvercurrentLimitCallback = void (*)(unit::Current) noexcept;
  using SetAdcTriggerOffsetCallback = void (*)(uint32_t) noexcept;
  using GetTimerClockFrequencyCallback = uint32_t (*)() noexcept;
  using PopHallEventCallback = bool (*)(observer::HallEvent&) noexcept;

  /**
   * @brief Constructs a motor hardware interface.
   * @param initialize Initializes the hardware for the requested PWM frequency
   *                   and ADC trigger offset and starts the interrupts.
   * @param get_phase_currents Reads the three phase currents.
   * @param get_phase_voltages Reads the three phase voltages.
   * @param get_temperature_measurements Reads optional temperature channels.
   * @param set_overcurrent_limit Sets the per-sample overcurrent limit.
   * @param set_adc_trigger_offset Updates the ADC trigger compare value.
   * @param get_timer_clock_frequency Returns the timer input clock in Hz.
   * @param pop_hall_event Takes the oldest captured change of the Hall code.
   */
  HardwareInterface(InitializeCallback initialize,
                    GetPhaseCurrentsCallback get_phase_currents,
                    GetPhaseVoltagesCallback get_phase_voltages,
                    GetTemperatureMeasurementsCallback get_temperature_measurements,
                    SetOvercurrentLimitCallback set_overcurrent_limit,
                    SetAdcTriggerOffsetCallback set_adc_trigger_offset,
                    GetTimerClockFrequencyCallback get_timer_clock_frequency,
                    PopHallEventCallback pop_hall_event)
      : initialize_(initialize),
        get_phase_currents_(get_phase_currents),
        get_phase_voltages_(get_phase_voltages),
        get_temperature_measurements_(get_temperature_measurements),
        set_overcurrent_limit_(set_overcurrent_limit),
        set_adc_trigger_offset_(set_adc_trigger_offset),
        get_timer_clock_frequency_(get_timer_clock_frequency),
        pop_hall_event_(pop_hall_event) {}

  /**
   * @brief Initializes the hardware and starts its interrupts.
   *
   * From then on the platform calls ControlCycleHandler() and
   * ObserverHandler() (hot_path_interface.hpp) once per control cycle.
   *
   * @param pwm_frequency Requested PWM switching frequency.
   * @param adc_trigger_offset ADC trigger compare offset in timer ticks.
   * @return True when initialization succeeds.
   */
  [[nodiscard]] bool Initialize(unit::Frequency pwm_frequency, uint32_t adc_trigger_offset) const noexcept {
    return initialize_(pwm_frequency, adc_trigger_offset);
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
   * (BCC: EXTI interrupt and the DWT cycle counter; native_sim: the edges of
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
  GetPhaseCurrentsCallback get_phase_currents_;
  GetPhaseVoltagesCallback get_phase_voltages_;
  GetTemperatureMeasurementsCallback get_temperature_measurements_;
  SetOvercurrentLimitCallback set_overcurrent_limit_;
  SetAdcTriggerOffsetCallback set_adc_trigger_offset_;
  GetTimerClockFrequencyCallback get_timer_clock_frequency_;
  PopHallEventCallback pop_hall_event_;
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
