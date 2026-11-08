/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file torque_split.hpp
 *    @brief Splits the torque request of the cart onto the four wheels.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include "drive_monitor.hpp"
#include "units.hpp"
#include "wheel.hpp"

namespace unimoc::vehicle {

/**
 * @brief Splits a cart torque request onto the wheels without a yaw moment.
 *
 * The request (the sum of the wheel torques, cart frame, positive forward)
 * goes to the axles by @p front_share (0: all to the rear axle, 1: all to
 * the front axle). Both wheels of an axle get the same torque, at most the
 * smaller window of the two, so the split produces no yaw moment. A wheel
 * without torque window (a lost or disengaged drive) therefore takes its
 * axle out (docs/system_plan.md, section 7: one drive lost). What one axle
 * cannot take goes to the other one up to its window; what neither can
 * take is not applied.
 *
 * A request or share that is not finite counts as zero torque or an equal
 * share.
 *
 * @param request Total torque at the wheels.
 * @param front_share Share of the front axle, 0 to 1.
 * @param windows Torque window of each wheel (DriveMonitor::Windows()).
 * @return The torque of each wheel, indexed by Index().
 */
[[nodiscard]] constexpr PerWheel<unit::Torque> SplitTorque(const unit::Torque request,
                                                           const unit::Ratio front_share,
                                                           const PerWheel<TorqueWindow>& windows) noexcept {
  using unit::operator""_ratio;
  PerWheel<unit::Torque> torques{};
  if (!unit::IsFinite(request) || request == unit::Torque{}) {
    return torques;
  }
  const bool kForward = request > unit::Torque{};
  const unit::Torque kMagnitude = unit::Abs(request);
  const unit::Ratio kShare = unit::IsFinite(front_share)
                                 ? unit::Min(unit::Max(front_share, unit::Ratio{}), 1.0_ratio)
                                 : 0.5_ratio;

  // Capacity of an axle in the requested direction: twice its smaller wheel window.
  const auto kCapacity = [&windows, kForward](const Axle axle) noexcept {
    const std::array<Wheel, 2> kAxleWheels = WheelsOf(axle);
    const TorqueWindow& left = windows[Index(kAxleWheels[0])];
    const TorqueWindow& right = windows[Index(kAxleWheels[1])];
    const unit::Torque kWheel =
        kForward ? unit::Min(left.forward, right.forward) : unit::Min(left.backward, right.backward);
    return 2.0F * unit::Max(kWheel, unit::Torque{});
  };
  const unit::Torque kFrontCapacity = kCapacity(Axle::kFront);
  const unit::Torque kRearCapacity = kCapacity(Axle::kRear);

  const unit::Torque kFrontWanted = kMagnitude * kShare;
  unit::Torque front = unit::Min(kFrontWanted, kFrontCapacity);
  unit::Torque rear = unit::Min(kMagnitude - kFrontWanted, kRearCapacity);
  // What one axle could not take goes to the other one.
  const unit::Torque kSpare = unit::Max(kMagnitude - front - rear, unit::Torque{});
  const unit::Torque kFrontExtra = unit::Min(kSpare, kFrontCapacity - front);
  front = front + kFrontExtra;
  rear = unit::Min(rear + (kSpare - kFrontExtra), kRearCapacity);

  const float kSign = kForward ? 1.0F : -1.0F;
  for (const Wheel kWheel : kWheels) {
    const unit::Torque kAxle = (AxleOf(kWheel) == Axle::kFront) ? front : rear;
    torques[Index(kWheel)] = kSign * (kAxle / 2.0F);
  }
  return torques;
}

}  // namespace unimoc::vehicle
