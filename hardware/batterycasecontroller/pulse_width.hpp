/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file pulse_width.hpp
 *    @brief PWM output API for the motor power stage.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include <modm/platform.hpp>
#include "base.hpp"
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
 * @namespace pulse_width pulse_width namespace
 * @brief Contains functions and definitions for handling pulse width modulation and analog inputs.
 */
namespace pulse_width {

/**
 * @brief Initializes the pwm subsystem.
 * This function sets up the necessary configurations for the pwm outputs.
 *
 * @return true if initialization is successful, false otherwise.
 */
bool Initialize(unit::Frequency pwm_frequency,
                HardwareInterface::SlowUpdateCallback slow_update_callback) noexcept;

/**
 * @brief Sets the phase duties for the motor control.
 * @param duties The phase duties to set, represented as a ThreePhase structure.
 */
void SetPhaseDuties(const system::ThreePhase<unit::Ratio>& duties) noexcept;

/**
 * @brief Reads the currently applied normalized PWM duties.
 */
system::ThreePhase<unit::Ratio> GetPhaseDuties() noexcept;

/**
 * @brief Sets the timer compare value used to trigger ADC conversion.
 * @param offset Trigger offset in timer ticks.
 */
void SetAdcTriggerOffset(uint32_t offset) noexcept;

/**
 * @brief Returns the timer input clock frequency in Hz.
 */
uint32_t GetTimerClockFrequency() noexcept;

}  // namespace pulse_width
}  // namespace hardware
}  // namespace unimoc
