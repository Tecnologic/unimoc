/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file analog.hpp
 *    @brief Analog input and motor measurement API.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include <modm/platform.hpp>
#include "hardware_interface.hpp"
#include "three_phase_system.hpp"

/**
 * @namespace unimoc global namespace
 */
namespace unimoc {
/**
 * @namespace hardware hardware namespace
 */
namespace hardware {

/**
 * @namespace analog analog namespace
 * @brief Contains definitions and mappings for analog signals in the UNIMOC hardware.
 */
namespace analog {
//
// ADC Channel Mapping
// Pin		Channel			Signal
// ---------------------------------------------
// PA0		ADC1_IN1		Current IA
// PA2		ADC1_IN3		Voltage VA
//
using A1_IA = modm::platform::GpioA0::In1;
using A1_VA = modm::platform::GpioA2::In3;
// ---------------------------------------------
// PA1		ADC2_IN2		Current IB
// PA6		ADC2_IN3		Voltage VB
using A2_IB = modm::platform::GpioA1::In2;
using A2_VB = modm::platform::GpioA6::In3;
// ---------------------------------------------
// PB1		ADC3_IN1		Current IC
// PB13		ADC3_IN5		Voltage VC
using A3_IC = modm::platform::GpioB1::In1;
using A3_VC = modm::platform::GpioB13::In5;
// ---------------------------------------------
// PB12		ADC4_IN3		DC Link Voltage
// PB14		ADC4_IN4		Crank Torque
using A4_VDC = modm::platform::GpioB12::In3;
using A4_CRKT = modm::platform::GpioB14::In4;
// ---------------------------------------------
// PA8		ADC5_IN1		Motor Temperature
// PA9		ADC5_IN2		Bridge Temperature
using A5_MOTT = modm::platform::GpioA8::In1;
using A5_BRDGT = modm::platform::GpioA9::In2;

/**
 * @brief Initializes the analog subsystem.
 * This function sets up the necessary configurations for the analog inputs.
 *
 * @return true if initialization is successful, false otherwise.
 */
bool Initialize(HardwareInterface::CurrentControlCallback current_control_callback) noexcept;

/**
 * @brief Reads the ADC values needed by the fast current-control loop.
 */
CurrentControlSamples GetCurrentControlSamples() noexcept;

/**
 * @brief Reads the optional CPU, bridge, and motor temperatures.
 */
TemperatureMeasurements GetTemperatureMeasurements() noexcept;

/**
 * @brief Reads the current from the specified phase.
 * @param phase The phase to read the current from (0 for A, 1 for B, 2 for C).
 * @return The current value in Amperes.
 */
system::ThreePhase<unit::Current> GetPhaseCurrents() noexcept;

/**
 * @brief Reads the voltage from the specified phase.
 * @param phase The phase to read the voltage from (0 for A, 1 for B, 2 for C).
 * @return The voltage value in Volts.
 */
system::ThreePhase<unit::Voltage> GetPhaseVoltages() noexcept;

/**
 * @brief Reads the DC link voltage.
 * @return The DC link voltage in Volts.
 */
float GetDcLinkVoltage() noexcept;

/**
 * @brief Reads the crank torque.
 * @return The crank torque in Newton-meters.
 */
float GetCrankTorque() noexcept;

/**
 * @brief Reads the motor temperature.
 * @return The motor temperature in degrees Celsius.
 */
float GetMotorTemperature() noexcept;

/**
 * @brief Reads the bridge temperature.
 * @return The bridge temperature in degrees Celsius.
 */
float GetBridgeTemperature() noexcept;

/**
 * @brief Sets the phase duties for the motor control.
 * @param duties The phase duties to set, represented as a ThreePhase structure.
 */
void SetPhaseDuties(const system::ThreePhase<unit::Ratio>& duties) noexcept;

}  // namespace analog
}  // namespace hardware
}  // namespace unimoc
