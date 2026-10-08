/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file analog.cpp
 *    @brief STM32 analog input implementation.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "analog.hpp"
#include <chrono>
#include <modm/platform.hpp>

using namespace modm::platform;
using namespace std::chrono_literals;

namespace unimoc::hardware::analog {

static_assert(kCurrentSensorCount == 2U || kCurrentSensorCount == 3U);
static_assert(!kCurrentSamplesArePwmWindowed || kCurrentSenseIsLowSide);

constexpr float ADC_RESOLUTION = 4096.0f;  // 12-bit ADC resolution
constexpr float VOLTAGE_REFERENCE = 3.3f;  // Voltage reference for the ADC

float torqueOffset = 0.0f;  // Torque offset in Nm
HardwareInterface::CurrentControlCallback current_control_callback = nullptr;

float adcToVoltage(uint32_t adcValue) noexcept;

void adcInterruptHandler() noexcept {
  Adc1::acknowledgeInterruptFlags(Adc1::InterruptFlag::EndOfInjectedConversion);
  if (current_control_callback != nullptr) current_control_callback();
}

/**
 * @brief Initializes the ADCs and GPIOs for analog input.
 *
 * This function sets up the GPIO pins for analog input, initializes the ADCs,
 * and configures the injected conversion channels.
 *
 * @return true if initialization is successful, false otherwise.
 */
bool Initialize(const HardwareInterface::CurrentControlCallback callback) noexcept {
  current_control_callback = callback;

  // Initialize the GPIOs for ADC channels
  GpioA0::setAnalogInput();
  GpioA2::setAnalogInput();
  GpioA1::setAnalogInput();
  GpioA6::setAnalogInput();
  GpioB1::setAnalogInput();
  GpioB13::setAnalogInput();
  GpioB12::setAnalogInput();
  GpioB14::setAnalogInput();
  if constexpr (kBridgeTemperatureAvailable || kMotorTemperatureAvailable) {
    GpioA8::setAnalogInput();
    GpioA9::setAnalogInput();
  }

  Adc1::initialize(Adc1::ClockMode::SynchronousPrescaler4,
                   Adc1::ClockSource::SystemClock,
                   Adc1::Prescaler::Disabled,
                   Adc1::CalibrationMode::SingleEndedInputsMode,
                   true);

  Adc1::connect<A1_IA, A1_VA>();
  if (!Adc1::setInjectedConversionSequenceLength(2)) {
    return false;
  }
  if (!Adc1::setInjectedConversionChannel<A1_IA>(0, Adc1::SampleTime::Cycles25)) {
    return false;
  }
  if (!Adc1::setInjectedConversionChannel<A1_VA>(1, Adc1::SampleTime::Cycles25)) {
    return false;
  }

  Adc2::initialize(Adc2::ClockMode::SynchronousPrescaler4,
                   Adc2::ClockSource::SystemClock,
                   Adc2::Prescaler::Disabled,
                   Adc2::CalibrationMode::SingleEndedInputsMode,
                   true);

  Adc2::connect<A2_IB, A2_VB>();
  if (!Adc2::setInjectedConversionSequenceLength(2)) {
    return false;
  }
  if (!Adc2::setInjectedConversionChannel<A2_IB>(0, Adc2::SampleTime::Cycles25)) {
    return false;
  }
  if (!Adc2::setInjectedConversionChannel<A2_VB>(1, Adc2::SampleTime::Cycles25)) {
    return false;
  }

  Adc3::initialize(Adc3::ClockMode::SynchronousPrescaler4,
                   Adc3::ClockSource::SystemClock,
                   Adc3::Prescaler::Disabled,
                   Adc3::CalibrationMode::SingleEndedInputsMode,
                   true);

  Adc3::connect<A3_IC, A3_VC>();
  if (!Adc3::setInjectedConversionSequenceLength(2)) {
    return false;
  }
  if (!Adc3::setInjectedConversionChannel<A3_IC>(0, Adc3::SampleTime::Cycles25)) {
    return false;
  }
  if (!Adc3::setInjectedConversionChannel<A3_VC>(1, Adc3::SampleTime::Cycles25)) {
    return false;
  }

  Adc4::initialize(Adc4::ClockMode::SynchronousPrescaler4,
                   Adc4::ClockSource::SystemClock,
                   Adc4::Prescaler::Disabled,
                   Adc4::CalibrationMode::SingleEndedInputsMode,
                   true);
  Adc4::connect<A4_VDC, A4_CRKT>();
  if (!Adc4::setInjectedConversionSequenceLength(2)) {
    return false;
  }
  if (!Adc4::setInjectedConversionChannel<A4_VDC>(0, Adc4::SampleTime::Cycles25)) {
    return false;
  }
  if (!Adc4::setInjectedConversionChannel<A4_CRKT>(1, Adc4::SampleTime::Cycles25)) {
    return false;
  }

  if constexpr (kBridgeTemperatureAvailable || kMotorTemperatureAvailable) {
    Adc5::initialize(Adc5::ClockMode::SynchronousPrescaler4,
                     Adc5::ClockSource::SystemClock,
                     Adc5::Prescaler::Disabled,
                     Adc5::CalibrationMode::SingleEndedInputsMode,
                     true);

    Adc5::connect<A5_MOTT, A5_BRDGT>();
    if (!Adc5::setInjectedConversionSequenceLength(2)) {
      return false;
    }
    if (!Adc5::setInjectedConversionChannel<A5_MOTT>(0, Adc5::SampleTime::Cycles25)) {
      return false;
    }
    if (!Adc5::setInjectedConversionChannel<A5_BRDGT>(1, Adc5::SampleTime::Cycles25)) {
      return false;
    }
  }

  // Enable ADC interrupt vector and set priority
  // use only one ADC interrupt vector for all ADCs
  Adc1::enableInterruptVector(kCurrentControlIrqPriority);  // Set priority for ADC1 interrupt
  Adc1::enableInterrupt(Adc1::Interrupt::EndOfInjectedConversion);
  AdcInterrupt1::attachInterruptHandler(adcInterruptHandler);

  // Set the ADCs to use the same trigger source and edge. On STM32G4,
  // injected Event9 is TIM8_CC4.
  Adc1::enableInjectedConversionExternalTrigger(Adc1::ExternalTriggerPolarity::RisingEdge,
                                                Adc1::ExternalTriggerEvent::Event9);

  Adc2::enableInjectedConversionExternalTrigger(Adc2::ExternalTriggerPolarity::RisingEdge,
                                                Adc2::ExternalTriggerEvent::Event9);
  Adc3::enableInjectedConversionExternalTrigger(Adc3::ExternalTriggerPolarity::RisingEdge,
                                                Adc3::ExternalTriggerEvent::Event9);
  Adc4::enableInjectedConversionExternalTrigger(Adc4::ExternalTriggerPolarity::RisingEdge,
                                                Adc4::ExternalTriggerEvent::Event9);
  if constexpr (kBridgeTemperatureAvailable || kMotorTemperatureAvailable) {
    Adc5::enableInjectedConversionExternalTrigger(Adc5::ExternalTriggerPolarity::RisingEdge,
                                                  Adc5::ExternalTriggerEvent::Event9);
  }
  if (!Adc1::enableChannelOffset<A1_IA>(Adc1::OffsetSlot::Slot0, 2048)) {
    return false;
  }
  if (!Adc2::enableChannelOffset<A2_IB>(Adc2::OffsetSlot::Slot0, 2048)) {
    return false;
  }
  if (!Adc3::enableChannelOffset<A3_IC>(Adc3::OffsetSlot::Slot0, 2048)) {
    return false;
  }

  Adc1::startInjectedConversionSequence();
  Adc2::startInjectedConversionSequence();
  Adc3::startInjectedConversionSequence();
  Adc4::startInjectedConversionSequence();
  if constexpr (kBridgeTemperatureAvailable || kMotorTemperatureAvailable)
    Adc5::startInjectedConversionSequence();

  return true;  // Return true if initialization is successful
}

/**
 * @brief Converts an ADC value to a current in Amperes.
 * @param adcValue The ADC value to convert.
 * @return The converted current in Amperes.
 */
constexpr float adcToCurrent(uint32_t adcValue) noexcept {
  constexpr float SHUNT_RESISTOR = 0.0005f;    // 0.5 mOhm shunt resistor
  constexpr float AMPLIFICATION_GAIN = 50.0f;  // Gain applied to the ADC signal
  // Calculate the current based on the ADC value, shunt resistor, and gain
  // Formula: Current (A) = (ADC Value * Voltage Reference / ADC Resolution) / (Shunt Resistor * Gain)
  float current = (static_cast<float>(adcValue) * VOLTAGE_REFERENCE / ADC_RESOLUTION) /
                  (SHUNT_RESISTOR * AMPLIFICATION_GAIN);

  return current;  // Convert ADC value to current in Amperes
}

system::ThreePhase<unit::Current> ReadPhaseCurrents() noexcept {
  const unit::Current phase_a{adcToCurrent(Adc1::getInjectedConversionValue(0))};
  const unit::Current phase_b{adcToCurrent(Adc2::getInjectedConversionValue(0))};
  unit::Current phase_c{};

  if constexpr (kCurrentSensorCount == 3U && kCurrentSenseHasPhaseMeasurements && !kCurrentSenseIsLowSide &&
                !kCurrentSamplesArePwmWindowed) {
    phase_c = unit::Current{adcToCurrent(Adc3::getInjectedConversionValue(0))};
  } else {
    phase_c = unit::Current{-phase_a.Value() - phase_b.Value()};
  }

  // Remove sensor common-mode error and reconstruct the final phase so the
  // public vector has an exact zero-sequence component.
  const float zero_sequence = (phase_a.Value() + phase_b.Value() + phase_c.Value()) / 3.0F;
  const unit::Current corrected_a{phase_a.Value() - zero_sequence};
  const unit::Current corrected_b{phase_b.Value() - zero_sequence};
  const unit::Current corrected_c{-corrected_a.Value() - corrected_b.Value()};
  return system::ThreePhase<unit::Current>{corrected_a, corrected_b, corrected_c};
}

/**
 * @brief Reads all three phase currents.
 */
system::ThreePhase<unit::Current> GetPhaseCurrents() noexcept { return ReadPhaseCurrents(); }

/**
 * @brief Converts an ADC value to a voltage in Volts.
 * @param adcValue The ADC value to convert.
 * @return The converted voltage in Volts.
 */
float adcToVoltage(uint32_t adcValue) noexcept {
  constexpr float VOLTAGE_DIVIDER_RATIO =
      (33.0f + 1.0f) / 1.0f;  // Voltage divider ratio for 33k to 1k resistor ratio

  // Convert the ADC value to voltage using the reference voltage and resolution
  float voltage = (static_cast<float>(adcValue) * VOLTAGE_REFERENCE / ADC_RESOLUTION) * VOLTAGE_DIVIDER_RATIO;
  return voltage;  // Return the converted voltage
}

CurrentControlSamples GetCurrentControlSamples() noexcept {
  return CurrentControlSamples{ReadPhaseCurrents(),
                               unit::Voltage{adcToVoltage(Adc4::getInjectedConversionValue(0))}};
}

/**
 * @brief Reads the voltage from the specified phase.
 * @param phase The phase to read the voltage from (0 for A, 1 for B, 2 for C).
 * @return The voltage value in Volts.
 */
system::ThreePhase<unit::Voltage> GetPhaseVoltages() noexcept {
  system::ThreePhase<unit::Voltage> voltages;

  // Read the injected conversion values for each phase voltage
  voltages.a = unit::Voltage{adcToVoltage(Adc1::getInjectedConversionValue(1))};
  voltages.b = unit::Voltage{adcToVoltage(Adc2::getInjectedConversionValue(1))};
  voltages.c = unit::Voltage{adcToVoltage(Adc3::getInjectedConversionValue(1))};

  return voltages;  // Return the phase voltages
}

/**
 * @brief Reads the DC link voltage.
 * @return The DC link voltage in Volts.
 */
float GetDcLinkVoltage() noexcept {
  // Read the injected conversion value for the DC link voltage
  uint32_t adcValue = Adc4::getInjectedConversionValue(0);
  return adcToVoltage(adcValue);  // Convert the ADC value to voltage and return it
}

float adcToTorque(uint32_t adcValue) noexcept {
  constexpr float TORQUE_SENSITIVITY = 70.0f;  // Torque sensitivity in Nm/V E-Rider T9
  return (static_cast<float>(adcValue) * VOLTAGE_REFERENCE / ADC_RESOLUTION) /
         TORQUE_SENSITIVITY;  // Convert ADC value to torque
}

/**
 * @brief Reads the crank torque.
 * @return The crank torque in Newton-meters.
 */
float getCrankTorque(void) noexcept {
  // Read the injected conversion value for the crank torque
  uint32_t adcValue = Adc4::getInjectedConversionValue(1);
  return adcToTorque(adcValue) - torqueOffset;  // Convert the ADC value to torque and subtract the offset
}

/**
 * @brief Converts an ADC value to a temperature in degrees Celsius.
 * @param adcValue The ADC value to convert.
 * @return The converted temperature in degrees Celsius.
 */
float adcToTemperature(uint32_t adcValue) noexcept {
  // Lookup table for 10k NTC temperature calculation
  constexpr size_t TABLE_SIZE = 128;
  constexpr float MIN_ADC_VALUE = 0.0f;
  constexpr float MAX_ADC_VALUE = ADC_RESOLUTION - 1.0f;
  constexpr float ADC_STEP = (MAX_ADC_VALUE - MIN_ADC_VALUE) / (TABLE_SIZE - 1);

  constexpr float BETA = 3950.0f;             // Beta value of the NTC
  constexpr float T0 = 298.15f;               // Reference temperature in Kelvin (25°C)
  constexpr float R0 = 10000.0f;              // Reference resistance at T0 (10k Ohm)
  constexpr float V_REF = VOLTAGE_REFERENCE;  // Voltage reference (3.3V)
  constexpr float R_PULLUP = 10000.0f;        // Pull-up resistor value (10k Ohm)

  // Precompute the lookup table
  constexpr std::array<float, TABLE_SIZE> temperatureLookup = []() {
    std::array<float, TABLE_SIZE> table{};
    for (size_t i = 1; i < TABLE_SIZE; ++i) {
      float tmpAdcValue = MIN_ADC_VALUE + static_cast<float>(i) * ADC_STEP;
      float voltage = tmpAdcValue * V_REF / ADC_RESOLUTION;
      float resistance = R_PULLUP * (V_REF / voltage - 1.0f);
      float temperatureK = 1.0f / (1.0f / T0 + (1.0f / BETA) * std::log(resistance / R0));
      table[i] = temperatureK - 273.15f;  // Convert Kelvin to Celsius
    }
    return table;
  }();

  // Perform linear interpolation
  float normalizedIndex = static_cast<float>(adcValue) / ADC_STEP;
  size_t lowerIndex = static_cast<size_t>(normalizedIndex);
  size_t upperIndex = lowerIndex + 1;

  if (upperIndex >= TABLE_SIZE) {
    upperIndex = TABLE_SIZE - 1;
  }

  float lowerValue = temperatureLookup[lowerIndex];
  float upperValue = temperatureLookup[upperIndex];
  float fraction = normalizedIndex - static_cast<float>(lowerIndex);

  return lowerValue + fraction * (upperValue - lowerValue);
}

/**
 * @brief Reads the motor temperature.
 * @return The motor temperature in degrees Celsius.
 */
float getMotorTemperature(void) noexcept {
  // Read the injected conversion value for the motor temperature
  uint32_t adcValue = Adc5::getInjectedConversionValue(0);
  return adcToTemperature(adcValue);  // Convert the ADC value to temperature and return it
}

/**
 * @brief Reads the bridge temperature.
 * @return The bridge temperature in degrees Celsius.
 */
float getBridgeTemperature(void) noexcept {
  // Read the injected conversion value for the bridge temperature
  uint32_t adcValue = Adc5::getInjectedConversionValue(1);
  return adcToTemperature(adcValue);  // Convert the ADC value to temperature and return it
}

TemperatureMeasurements GetTemperatureMeasurements() noexcept {
  TemperatureMeasurements measurements{};
  if constexpr (kBridgeTemperatureAvailable) measurements.bridge = unit::Temperature{getBridgeTemperature()};
  if constexpr (kMotorTemperatureAvailable) measurements.motor = unit::Temperature{getMotorTemperature()};
  return measurements;
}

}  // namespace unimoc::hardware::analog
