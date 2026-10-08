/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file pulse_width.cpp
 *    @brief STM32 PWM output implementation.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "pulse_width.hpp"
#include <algorithm>
#include <chrono>
#include <limits>
#include <modm/architecture/interface/interrupt.hpp>
#include <modm/debug/logger.hpp>
#include <modm/platform.hpp>

using namespace modm::platform;

namespace unimoc::hardware::pulse_width {

using namespace unimoc::unit;

HardwareInterface::SlowUpdateCallback slow_update_callback = nullptr;

void TimerUpdateInterruptHandler() noexcept {
  Timer8::acknowledgeInterruptFlags(Timer8::InterruptFlag::Update);
  if (slow_update_callback != nullptr) slow_update_callback();
}

MODM_ISR(TIM8_UP) { TimerUpdateInterruptHandler(); }

/**
 * \brief Initializes the PWM output for three-phase motor control.
 *
 * This function sets up the GPIO pins for PWM output, configures the timer,
 * and initializes the PWM channels for the three phases (A, B, C).
 *
 * \return true if initialization is successful, false otherwise.
 */
bool Initialize(const unit::Frequency pwm_frequency,
                const HardwareInterface::SlowUpdateCallback callback) noexcept {
  const uint32_t frequency_hz = static_cast<uint32_t>(pwm_frequency.Value());
  if (frequency_hz == 0u) return false;
  slow_update_callback = callback;

  // Initialize GPIO pins for PWM output
  // Phase A: GPIO B6, A7
  GpioB6::setAlternateFunction(5);                // Set GPIO B6 to alternate function mode (AF5 for TIM8)
  GpioA7::setAlternateFunction(4);                // Set GPIO A7 to alternate function mode (AF4 for TIM8)
  GpioB6::setOutput(Gpio::OutputType::PushPull);  // Set GPIO B6 to push-pull output type
  GpioA7::setOutput(Gpio::OutputType::PushPull);  // Set GPIO A7 to push-pull output type
  // Set GPIO speed to high for better PWM performance
  GpioB6::reset();  // Reset GPIO B6 to low state
  GpioA7::reset();  // Reset GPIO A7 to low state

  // Phase B: GPIO B8, B0
  GpioB8::setAlternateFunction(10);               // Set GPIO B8 to alternate function mode (AF10 for TIM8)
  GpioB0::setAlternateFunction(4);                // Set GPIO B0 to alternate function mode (AF4 for TIM8)
  GpioB8::setOutput(Gpio::OutputType::PushPull);  // Set GPIO B8 to push-pull output type
  GpioB0::setOutput(Gpio::OutputType::PushPull);  // Set GPIO B0 to push-pull output type
  GpioB8::reset();                                // Reset GPIO B8 to low state
  GpioB0::reset();                                // Reset GPIO B0 to low state

  // Phase C: GPIO B9, B5
  GpioB9::setAlternateFunction(10);               // Set GPIO B9 to alternate function mode (AF10 for TIM8)
  GpioB5::setAlternateFunction(3);                // Set GPIO B5 to alternate function mode (AF3 for TIM8)
  GpioB9::setOutput(Gpio::OutputType::PushPull);  // Set GPIO B9 to push-pull output type
  GpioB5::setOutput(Gpio::OutputType::PushPull);  // Set GPIO B5 to push-pull output type
  GpioB9::reset();                                // Reset GPIO B9 to low state
  GpioB5::reset();                                // Reset GPIO B5 to low state

  Timer8::connect<GpioOutputB6::Ch1,
                  GpioOutputA7::Ch1n,
                  GpioOutputB8::Ch2,
                  GpioOutputB0::Ch2n,
                  GpioOutputB9::Ch3,
                  GpioOutputB5::Ch3n>();
  Timer8::enable();  // Enable Timer 8 for PWM operation

  Timer8::setMode(Timer8::Mode::CenterAligned3,
                  Timer8::SlaveMode::Disabled,
                  Timer8::SlaveModeTrigger::Internal0,
                  Timer8::MasterMode::CompareOc4Ref,
                  false,
                  Timer8::MasterMode2::Update);

  Timer8::setPrescaler(1);  // Set prescaler to 1 for maximum frequency
  const auto period = std::chrono::duration<uint64_t, std::nano>{1'000'000'000ULL / frequency_hz};
  auto period_set = Timer8::setPeriod<SystemClock>(period, true);
  if (period_set == 0) {
    return false;
  }
  MODM_LOG_INFO << "Timer8 period set to: " << period_set << modm::endl;

  Timer8::configureOutputChannel(4u, Timer8::OutputCompareMode::Toggle, 0u, Timer8::PinState::Disable);

  Timer8::configureOutputChannel<GpioOutputB6::Ch1>(Timer8::OutputCompareMode::Pwm,
                                                    Timer8::PinState::Enable,
                                                    Timer8::OutputComparePolarity::ActiveHigh,
                                                    Timer8::PinState::Enable,
                                                    Timer8::OutputComparePolarity::ActiveHigh,
                                                    Timer8::OutputComparePreload::Disable);

  Timer8::configureOutputChannel<GpioOutputB8::Ch2>(Timer8::OutputCompareMode::Pwm,
                                                    Timer8::PinState::Enable,
                                                    Timer8::OutputComparePolarity::ActiveHigh,
                                                    Timer8::PinState::Enable,
                                                    Timer8::OutputComparePolarity::ActiveHigh,
                                                    Timer8::OutputComparePreload::Disable);

  Timer8::configureOutputChannel<GpioOutputB9::Ch3>(Timer8::OutputCompareMode::Pwm,
                                                    Timer8::PinState::Enable,
                                                    Timer8::OutputComparePolarity::ActiveHigh,
                                                    Timer8::PinState::Enable,
                                                    Timer8::OutputComparePolarity::ActiveHigh,
                                                    Timer8::OutputComparePreload::Disable);

  Timer8::enableInterruptVector(Timer8::Interrupt::Update, true, kSlowUpdateIrqPriority);
  Timer8::enableInterrupt(Timer8::Interrupt::Update);
  Timer8::start();

  return true;  // Return true if initialization is successful
}

//! \brief Converts a duty cycle (0.0 to 1.0) to a compare value based on the period.
constexpr uint16_t dutyCycleToCompareValue(float dutyCycle, uint16_t period) noexcept {
  // Convert duty cycle (0.0 to 1.0) to compare value based on the period
  return static_cast<uint16_t>(dutyCycle * period);
}

//! \brief Sets the PWM duty cycles for the three phases (A, B, C).
void SetPhaseDuties(const system::ThreePhase<unit::Ratio>& duties) noexcept {
  Timer8::setCompareValue<GpioOutputB6::Ch1>(
      dutyCycleToCompareValue(duties.a.Value(), Timer8::getOverflow()));
  Timer8::setCompareValue<GpioOutputB8::Ch2>(
      dutyCycleToCompareValue(duties.b.Value(), Timer8::getOverflow()));
  Timer8::setCompareValue<GpioOutputB9::Ch3>(
      dutyCycleToCompareValue(duties.c.Value(), Timer8::getOverflow()));
}

system::ThreePhase<unit::Ratio> GetPhaseDuties() noexcept {
  const auto overflow = Timer8::getOverflow();
  if (overflow == 0u) {
    return system::ThreePhase<unit::Ratio>{0.5_ratio, 0.5_ratio, 0.5_ratio};
  }
  const float scale = 1.0F / static_cast<float>(overflow);
  return system::ThreePhase<unit::Ratio>{unit::Ratio{static_cast<float>(Timer8::getCompareValue(1u)) * scale},
                                         unit::Ratio{static_cast<float>(Timer8::getCompareValue(2u)) * scale},
                                         unit::Ratio{static_cast<float>(Timer8::getCompareValue(3u)) *
                                                     scale}};
}

void SetAdcTriggerOffset(const uint32_t offset) noexcept {
  const uint32_t limited_offset = std::min<uint32_t>(offset, std::numeric_limits<uint16_t>::max());
  Timer8::setCompareValue(4u, static_cast<uint16_t>(limited_offset));
}

uint32_t GetTimerClockFrequency() noexcept { return Timer8::getClockFrequency<SystemClock>(); }
}  // namespace unimoc::hardware::pulse_width
