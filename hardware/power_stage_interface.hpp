/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file power_stage_interface.hpp
 *    @brief Hardware boundary of the power stage release, break and fault latch.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>

namespace unimoc::hardware {

/// State of the power stage outputs.
enum class PowerStageState : uint8_t {
  kDisabled,  ///< Outputs off, released from outside, no fault latched. Enable() is allowed.
  kEnabled,   ///< Outputs follow the PWM compare values.
  kFault,     ///< Outputs off after a break (Trip() or a break input); latched until ClearFault().
  kBlocked,   ///< Outputs off, the external enable is missing (jumper, e-stop).
};

/**
 * @brief Release and protection of the power stage.
 *
 * The release is a state machine on the controller; the external enable
 * only allows it, it never starts the power stage by itself. Every way to
 * switch off ends in the same hardware break: outputs go to their idle (off)
 * state without software in the loop.
 *
 * Disable() and Trip() are safe to call from any interrupt, including
 * zero-latency ones.
 */
class PowerStageInterface {
 public:
  using EnableCallback = bool (*)() noexcept;
  using ActionCallback = void (*)() noexcept;
  using StateCallback = PowerStageState (*)() noexcept;

  PowerStageInterface(EnableCallback enable,
                      ActionCallback disable,
                      ActionCallback trip,
                      ActionCallback clear_fault,
                      StateCallback get_state) noexcept
      : enable_(enable), disable_(disable), trip_(trip), clear_fault_(clear_fault), get_state_(get_state) {}

  /// Releases the outputs. Fails (false) while the external enable is missing or a fault is latched.
  [[nodiscard]] bool Enable() const noexcept { return enable_(); }

  /// Switches the outputs off. No fault is latched.
  void Disable() const noexcept { disable_(); }

  /// Switches the outputs off and latches a fault (overcurrent, runaway controller).
  void Trip() const noexcept { trip_(); }

  /// Acknowledges a latched fault. A still active break input latches it again.
  void ClearFault() const noexcept { clear_fault_(); }

  [[nodiscard]] PowerStageState GetState() const noexcept { return get_state_(); }

 private:
  EnableCallback enable_;
  ActionCallback disable_;
  ActionCallback trip_;
  ActionCallback clear_fault_;
  StateCallback get_state_;
};

extern const PowerStageInterface kPowerStage;  ///< Power stage hardware boundary.

}  // namespace unimoc::hardware
