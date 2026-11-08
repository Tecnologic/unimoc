/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file drive_monitor.hpp
 *    @brief The state of the wheel drives as the VDC sees it from their drive status.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include "units.hpp"
#include "wheel.hpp"

namespace unimoc::vehicle {

/**
 * @brief One unimoc.cart.DriveStatus of a wheel drive, in the cart frame.
 *
 * Positive speed and torque drive the cart forward. A value the drive does
 * not measure is NaN.
 */
struct DriveReport {
  /// Angular velocity of the wheel.
  unit::AngularVelocity wheel_speed;
  /// Torque estimate at the wheel.
  unit::Torque estimated_torque;
  /// Largest torque in the direction of motion (at standstill: forward), >= 0.
  unit::Torque drive_torque_limit;
  /// Largest torque against the direction of motion (at standstill: backward), >= 0.
  unit::Torque brake_torque_limit;
  /// The drive is ENGAGED and follows the torque command.
  bool engaged{false};
  /// The power stage outputs are enabled (POWER_STAGE_ENABLED).
  bool power_stage_enabled{false};
  /// The drive takes its torque from the pedals, not from the VDC.
  bool limp_home{false};
  /// An overcurrent or overvoltage trip opened the power stage.
  bool tripped{false};
};

/// Torque a wheel can take now, in the cart frame.
struct TorqueWindow {
  /// Largest forward torque, >= 0.
  unit::Torque forward;
  /// Largest backward torque as a magnitude, >= 0.
  unit::Torque backward;
};

/**
 * @brief Collects the drive status of the four wheels and decides which drive follows the VDC.
 *
 * A wheel is available while its last report is fresh (not older than the
 * timeout), the drive is engaged with its power stage enabled, is not in
 * limp-home mode and has no trip. Only an available wheel gets torque; the
 * torque split removes the other wheel of its axle too
 * (docs/system_plan.md, section 7: one drive lost).
 *
 * Runs in a thread of the VDC; times are a monotonic clock in microseconds.
 */
class DriveMonitor {
 public:
  /// Sets the age after which a report counts as lost [us].
  void SetTimeout(const uint64_t timeout_us) noexcept { timeout_us_ = timeout_us; }

  /// Age after which a report counts as lost [us].
  [[nodiscard]] uint64_t Timeout() const noexcept { return timeout_us_; }

  /// Takes the drive status of @p wheel, received at @p now_us.
  void OnReport(const Wheel wheel, const DriveReport& report, const uint64_t now_us) noexcept {
    Entry& entry = entries_[Index(wheel)];
    entry.report = report;
    entry.received_us = now_us;
    entry.received = true;
  }

  /// Forgets every report.
  void Reset() noexcept { entries_ = {}; }

  /// A report of @p wheel arrived and is not older than the timeout.
  [[nodiscard]] bool Fresh(const Wheel wheel, const uint64_t now_us) const noexcept {
    const Entry& entry = entries_[Index(wheel)];
    return entry.received && now_us >= entry.received_us && now_us - entry.received_us <= timeout_us_;
  }

  /// The drive of @p wheel follows the torque of the VDC.
  [[nodiscard]] bool Available(const Wheel wheel, const uint64_t now_us) const noexcept {
    const DriveReport& report = entries_[Index(wheel)].report;
    return Fresh(wheel, now_us) && report.engaged && report.power_stage_enabled && !report.limp_home &&
           !report.tripped;
  }

  /// The last report of @p wheel (default values before the first one).
  [[nodiscard]] const DriveReport& Report(const Wheel wheel) const noexcept {
    return entries_[Index(wheel)].report;
  }

  /**
   * @brief The torque window of @p wheel in the cart frame; zero unless Available().
   *
   * The drive reports its limits relative to its direction of motion; a
   * standstill or an unknown speed counts as forward, as on the drive. A
   * limit that is not finite or negative counts as zero.
   */
  [[nodiscard]] TorqueWindow Window(const Wheel wheel, const uint64_t now_us) const noexcept {
    if (!Available(wheel, now_us)) {
      return {};
    }
    const DriveReport& report = entries_[Index(wheel)].report;
    const unit::Torque kDrive = Sanitized(report.drive_torque_limit);
    const unit::Torque kBrake = Sanitized(report.brake_torque_limit);
    const bool kBackward = report.wheel_speed < unit::AngularVelocity{};
    return kBackward ? TorqueWindow{kBrake, kDrive} : TorqueWindow{kDrive, kBrake};
  }

  /// The torque windows of all wheels.
  [[nodiscard]] PerWheel<TorqueWindow> Windows(const uint64_t now_us) const noexcept {
    PerWheel<TorqueWindow> windows{};
    for (const Wheel kWheel : kWheels) {
      windows[Index(kWheel)] = Window(kWheel, now_us);
    }
    return windows;
  }

 private:
  struct Entry {
    DriveReport report;
    uint64_t received_us{0U};
    bool received{false};
  };

  /// A limit as a finite value >= 0.
  [[nodiscard]] static unit::Torque Sanitized(const unit::Torque limit) noexcept {
    return (unit::IsFinite(limit) && limit > unit::Torque{}) ? limit : unit::Torque{};
  }

  PerWheel<Entry> entries_{};
  uint64_t timeout_us_{100'000U};
};

}  // namespace unimoc::vehicle
