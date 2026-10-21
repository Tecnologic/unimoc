/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file current_control_runtime.hpp
 *    @brief Start-up of the current-control runtime for one motor.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include "current_control_isr.hpp"
#include "hardware_interface.hpp"
#include "nvm_settings.hpp"
#include "outer_loop.hpp"
#include "slow_update.hpp"
#include "units.hpp"

namespace unimoc::current_control {

/**
 * @brief Initializes the control cycle, the observer and the outer loop and starts the motor's cycle source.
 *
 * Shared by the application and the native_sim scenarios, so that both run
 * the same start sequence. Calling it again restarts the controller from its
 * initial state with new settings.
 *
 * @param settings Motor and controller settings.
 * @param motor Hardware access of the motor.
 * @param overcurrent_limit Phase current at which the control cycle trips the power stage.
 * @return False when the hardware layer rejects the PWM frequency.
 */
[[nodiscard]] inline bool StartRuntime(const settings::NvmSettings& settings,
                                       const hardware::HardwareInterface& motor,
                                       const unit::Current overcurrent_limit) noexcept {
  auto& current_control = CurrentControlIsr::Instance();
  auto& slow_update = SlowUpdate::Instance();
  current_control.Init(settings, motor, motor.GetTimerClockFrequency());
  slow_update.Init(settings, current_control, motor);
  OuterLoop::Instance().Init(settings, current_control);
  current_control.SetOvercurrentLimit(overcurrent_limit);
  return motor.Initialize(settings.pwm_frequency, current_control.state.adc_trigger_offset);
}

}  // namespace unimoc::current_control
