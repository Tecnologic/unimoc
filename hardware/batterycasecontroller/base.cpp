/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file base.cpp
 *    @brief BatteryCaseController hardware interface implementation.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include <modm/debug.hpp>
#include <modm/platform.hpp>
#include <modm/platform/rtt/rtt.hpp>
#include "analog.hpp"
#include "hardware_interface.hpp"
#include "pulse_width.hpp"

using namespace modm::platform;
using namespace std::chrono_literals;
using namespace modm::literals;

namespace unimoc::hardware {

using namespace unit;

namespace {

settings::SettingsStorageStatus LoadSettings(void* const, const std::span<std::byte>) noexcept {
  return settings::SettingsStorageStatus::kUnavailable;
}

settings::SettingsStorageStatus SaveSettings(void* const, const std::span<const std::byte>) noexcept {
  return settings::SettingsStorageStatus::kUnavailable;
}

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

const settings::SettingsStorage kSettingsStorage{nullptr, LoadSettings, SaveSettings};

void LogMessage(const LogLevel level, const char* const message) noexcept {
  switch (level) {
    case LogLevel::kDebug:
      modm::log::debug << message << modm::endl;
      break;
    case LogLevel::kInfo:
      modm::log::info << message << modm::endl;
      break;
    case LogLevel::kWarning:
      modm::log::warning << message << modm::endl;
      break;
    case LogLevel::kError:
      modm::log::error << message << modm::endl;
      break;
  }
}

void WaitForInterrupt() noexcept { __WFI(); }

}  // namespace

HardwareSettingsInterface settings{kSettingsProfile, kSettingsStorage};
HardwareSystemInterface runtime{LogMessage, WaitForInterrupt};

bool Initialize(unit::Frequency pwm_frequency,
                uint32_t adc_trigger_offset,
                HardwareInterface::CurrentControlCallback current_control_callback,
                HardwareInterface::SlowUpdateCallback slow_update_callback) noexcept;

HardwareInterface motor[MOTORS] = {{// Initialize function
                                    Initialize,

                                    // Get phase currents function
                                    analog::GetPhaseCurrents,

                                    // Get phase voltages function
                                    analog::GetPhaseVoltages,

                                    // Get current-control ADC samples function
                                    analog::GetCurrentControlSamples,

                                    // Get optional temperature measurements function
                                    analog::GetTemperatureMeasurements,

                                    // Get applied phase duties function
                                    pulse_width::GetPhaseDuties,

                                    // Set phase duties function
                                    pulse_width::SetPhaseDuties,

                                    // Set ADC trigger offset function
                                    pulse_width::SetAdcTriggerOffset,

                                    // Get timer clock frequency function
                                    pulse_width::GetTimerClockFrequency}};

/**
 * Initializes the hardware components of the battery case controller.
 *
 * This function sets up the system clock, initializes the ADCs, and configures
 * the pulse width modulation (PWM) for motor control.
 */
bool Initialize(const unit::Frequency pwm_frequency,
                const uint32_t adc_trigger_offset,
                const HardwareInterface::CurrentControlCallback current_control_callback,
                const HardwareInterface::SlowUpdateCallback slow_update_callback) noexcept {
  if (!SystemClock::enable()) {
    MODM_LOG_ERROR << "Failed to enable system clock." << modm::endl;
    return false;  // Return false if system clock initialization fails
  }

  // Initialize the pulse width modulation (PWM) for motor control
  if (!pulse_width::Initialize(pwm_frequency, slow_update_callback)) {
    MODM_LOG_ERROR << "Failed to initialize PWM." << modm::endl;
    return false;  // Return false if PWM initialization fails
  }

  pulse_width::SetAdcTriggerOffset(adc_trigger_offset);
  pulse_width::SetPhaseDuties(system::ThreePhase<unit::Ratio>{0.5_ratio, 0.5_ratio, 0.5_ratio});

  // Initialize the ADCs last so the timer trigger and control state are ready
  // before the first conversion interrupt can be delivered.
  if (!analog::Initialize(current_control_callback)) {
    MODM_LOG_ERROR << "Failed to initialize ADCs." << modm::endl;
    return false;  // Return false if ADC initialization fails
  }

  MODM_LOG_INFO << "Hardware initialization successful." << modm::endl;
  return true;  // Return true if all initializations are successful
}

}  // namespace unimoc::hardware

// Include the necessary headers for RTT logging
using LoggerDevice = modm::IODeviceWrapper<modm::platform::Rtt<0>, modm::IOBuffer::DiscardIfFull>;
static LoggerDevice rtt_device;
// Set all four logger streams to use RTT
modm::log::Logger modm::log::debug(rtt_device);
modm::log::Logger modm::log::info(rtt_device);
modm::log::Logger modm::log::warning(rtt_device);
modm::log::Logger modm::log::error(rtt_device);
