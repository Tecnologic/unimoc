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

#include <atomic>
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
 * zero-latency ones. Enable() runs in thread context only.
 *
 * HoldOffEnable() keeps Enable() from releasing the outputs, for example
 * while flash is written: on the single-bank STM32G4 a flash write stalls the
 * zero-latency control cycle for milliseconds, which must not happen with the
 * outputs released. Enable() and the hold-off exclude each other, so a stage
 * found disabled after HoldOffEnable() stays disabled until ReleaseEnable().
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

  /**
   * @brief Releases the outputs.
   * @return False while the external enable is missing, a fault is latched or
   *         Enable() is held off (HoldOffEnable()); the outputs stay off then.
   */
  [[nodiscard]] bool Enable() const noexcept {
    if (enable_lock_.test_and_set(std::memory_order_acquire)) {
      return false;  // held off, or a concurrent Enable()
    }
    const bool kEnabled = enable_();
    enable_lock_.clear(std::memory_order_release);
    return kEnabled;
  }

  /**
   * @brief Keeps Enable() from releasing the outputs until ReleaseEnable().
   *
   * Thread context only. Check GetState() afterwards: the hold-off does not
   * switch an already enabled stage off.
   *
   * @return False when it is already held off or an Enable() is running.
   */
  [[nodiscard]] bool HoldOffEnable() const noexcept {
    return !enable_lock_.test_and_set(std::memory_order_acquire);
  }

  /// Ends a successful HoldOffEnable().
  void ReleaseEnable() const noexcept { enable_lock_.clear(std::memory_order_release); }

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
  /// Set while Enable() runs or while it is held off.
  mutable std::atomic_flag enable_lock_;
};

extern const PowerStageInterface kPowerStage;  ///< Power stage hardware boundary.

}  // namespace unimoc::hardware
