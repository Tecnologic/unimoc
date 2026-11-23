/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file pedal_map.hpp
 *    @brief Torque request of the cart from the pedals and the gear: brake
 *           first, a speed limit and a speed ramp per gear, reverse.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include "units.hpp"

namespace unimoc::vehicle {

using unit::operator""_m_per_s;
using unit::operator""_ratio;

/// One pedal of unimoc.cart.PedalState: two channels of pedal travel (0 to 1, NaN failed).
struct PedalChannels {
  std::array<unit::Ratio, 2> channels{};
  /// Both channels agree within the tolerance of the pedal unit.
  bool plausible{false};
};

/// Both pedals of unimoc.cart.PedalState.
struct Pedals {
  PedalChannels drive;
  PedalChannels brake;
};

/// Gear of the cart: -1 reverse, 0 neutral, 1 to kTopGear forward.
using Gear = int8_t;

/// The reverse gear.
inline constexpr Gear kReverse = -1;
/// Neutral: no drive torque, the brake still works.
inline constexpr Gear kNeutral = 0;
/// The first forward gear, the one that needs no release.
inline constexpr Gear kFirstGear = 1;
/// The highest forward gear.
inline constexpr Gear kTopGear = 3;
/// Number of gears with limits: reverse and the forward gears (neutral has none).
inline constexpr std::size_t kGearCount = 1U + static_cast<std::size_t>(kTopGear);

/// @p gear is reverse, neutral or a forward gear.
[[nodiscard]] constexpr bool IsGear(const int gear) noexcept { return gear >= kReverse && gear <= kTopGear; }

/// Index of @p gear in the per-gear limits: reverse 0, forward gear n at n; neutral has none (0).
[[nodiscard]] constexpr std::size_t GearIndex(const Gear gear) noexcept {
  return (gear > kNeutral) ? static_cast<std::size_t>(gear) : 0U;
}

/// Direction of @p gear: +1 forward, -1 reverse, 0 neutral.
[[nodiscard]] constexpr int Direction(const Gear gear) noexcept {
  return (gear > kNeutral) ? 1 : ((gear < kNeutral) ? -1 : 0);
}

/// Limits of one gear.
struct GearLimits {
  /// Largest speed in the direction of the gear.
  unit::Velocity speed_limit;
  /// Rise of the speed limit from the speed of the cart while driving (speed ramp); zero: no ramp.
  unit::Acceleration acceleration;
};

/// Parameters of the pedal map.
struct PedalMapParameters {
  /// Limits per gear, indexed by GearIndex().
  std::array<GearLimits, kGearCount> gears{};
  /// Total wheel torque at the fully pressed drive pedal, every gear.
  unit::Torque drive_torque;
  /// Total wheel torque at the fully pressed brake pedal, every gear.
  unit::Torque brake_torque;
  /// Pedal travel below which a pedal counts as released (sensor noise at rest).
  unit::Ratio dead_band{0.05_ratio};
  /// Width of the speed band below a limit, as a share of the gear limit, in which the drive torque fades
  /// out.
  unit::Ratio speed_band{0.1_ratio};
  /// Speed below which the cart stands still: the brake fades out, the direction may change.
  unit::Velocity standstill_speed{0.15_m_per_s};
  /// How far the ramped speed limit may run ahead of the cart; uphill the cart lags and keeps the full
  /// torque.
  unit::Velocity ramp_lead{0.3_m_per_s};
  /// Brake pedal travel used while the pedal state is lost (system plan section 7: brake gently).
  unit::Ratio lost_pedal_brake{0.2_ratio};
};

/// Travel of the drive pedal: the smaller channel; an implausible pedal counts as released.
[[nodiscard]] constexpr unit::Ratio DriveTravel(const PedalChannels& pedal) noexcept {
  if (!pedal.plausible || !unit::IsFinite(pedal.channels[0]) || !unit::IsFinite(pedal.channels[1])) {
    return unit::Ratio{};
  }
  return unit::Min(pedal.channels[0], pedal.channels[1]).Clamp(0.0_ratio, 1.0_ratio);
}

/// Travel of the brake pedal: the larger of the valid channels, plausible or not (PedalState.0.1).
[[nodiscard]] constexpr unit::Ratio BrakeTravel(const PedalChannels& pedal) noexcept {
  unit::Ratio travel{};
  for (const unit::Ratio kChannel : pedal.channels) {
    if (unit::IsFinite(kChannel)) {
      travel = unit::Max(travel, kChannel);
    }
  }
  return travel.Clamp(0.0_ratio, 1.0_ratio);
}

/// @p travel past the dead band, rescaled to 0 to 1.
[[nodiscard]] constexpr unit::Ratio PastDeadBand(const unit::Ratio travel,
                                                 const unit::Ratio dead_band) noexcept {
  if (travel <= dead_band || dead_band >= 1.0_ratio) {
    return unit::Ratio{};
  }
  return ((travel - dead_band) / (1.0_ratio - dead_band)).Clamp(0.0_ratio, 1.0_ratio);
}

/**
 * @brief Maps the pedals and the gear to the total torque request of the cart (cart frame).
 *
 * - **Brake first:** a pressed brake pedal brakes against the direction of
 *   motion with up to the brake torque, whatever the drive pedal says. The
 *   brake torque fades out below the standstill speed, so it never drives
 *   the cart the other way (the same rule as the limp-home mode of the drives).
 * - **Drive in the direction of the gear:** the drive pedal asks for up to
 *   the drive torque, forward in a forward gear, backward in reverse,
 *   nothing in neutral. The torque is the same in every gear, so the cart
 *   climbs a slope in the first gear too.
 * - **Speed limit and speed ramp per gear:** the request is at most a limit
 *   torque that falls from the drive torque at the bottom of the speed band
 *   to zero at the limit and brakes above it, down to minus the drive torque
 *   at the limit plus the band. With an acceleration, the limit of a gear
 *   starts at the speed of the cart when the drive pedal is pressed and
 *   rises with it, never more than the ramp lead ahead of the cart: the cart
 *   accelerates gently on the flat and keeps the full torque uphill, where it
 *   lags. Without an acceleration the gear limit applies at once. Moving
 *   against the gear or in neutral, the cart is held to the limit of its
 *   direction of motion (reverse, or the highest released forward gear) by
 *   braking only.
 * - **Gears and release:** a requested gear above the released one is
 *   lowered to it; the release is volatile and set from outside (display,
 *   phone). Neutral and a gear of the same direction apply at once; a gear
 *   of the other direction waits in neutral for the standstill.
 *
 * Pure logic in the thread of the VDC: no bus, no hardware.
 */
class PedalMap {
 public:
  /// Takes new parameters; the state is kept.
  void Configure(const PedalMapParameters& parameters) noexcept { parameters_ = parameters; }

  /// The parameters in effect.
  [[nodiscard]] const PedalMapParameters& Parameters() const noexcept { return parameters_; }

  /// Asks for @p gear; it applies as described in the class (a gear above the release is lowered to it).
  void RequestGear(const Gear gear) noexcept { requested_gear_ = std::clamp(gear, kReverse, kTopGear); }

  /// The requested gear.
  [[nodiscard]] Gear RequestedGear() const noexcept { return requested_gear_; }

  /// The gear in effect.
  [[nodiscard]] Gear ActiveGear() const noexcept { return active_gear_; }

  /// Releases the forward gears up to @p gear (at least the first); a lower release lowers the gear at once.
  void SetRelease(const Gear gear) noexcept {
    release_ = std::clamp(gear, kFirstGear, kTopGear);
    active_gear_ = std::min(active_gear_, release_);
  }

  /// The highest released forward gear.
  [[nodiscard]] Gear Release() const noexcept { return release_; }

  /// The speed limit in effect: the ramped limit of the active gear.
  [[nodiscard]] unit::Velocity RampedLimit() const noexcept { return ramped_limit_; }

  /// The last torque request.
  [[nodiscard]] unit::Torque Request() const noexcept { return torque_; }

  /// Forgets the request and the speed ramp.
  void Reset() noexcept {
    torque_ = unit::Torque{};
    ramped_limit_ = unit::Velocity{};
  }

  /**
   * @brief The torque request of one cycle.
   * @param pedals The fresh pedal state.
   * @param speed Speed of the cart (cart frame, positive forward).
   * @param dt Time since the last cycle.
   * @return Total wheel torque, cart frame.
   */
  [[nodiscard]] unit::Torque Update(const Pedals& pedals,
                                    const unit::Velocity speed,
                                    const unit::Time dt) noexcept {
    return Apply(PastDeadBand(DriveTravel(pedals.drive), parameters_.dead_band),
                 PastDeadBand(BrakeTravel(pedals.brake), parameters_.dead_band),
                 speed,
                 dt);
  }

  /**
   * @brief The torque request of one cycle without a pedal state: brake gently.
   * @param speed Speed of the cart (cart frame, positive forward).
   * @param dt Time since the last cycle.
   * @return Total wheel torque, cart frame.
   */
  [[nodiscard]] unit::Torque UpdateLost(const unit::Velocity speed, const unit::Time dt) noexcept {
    return Apply(unit::Ratio{}, parameters_.lost_pedal_brake.Clamp(0.0_ratio, 1.0_ratio), speed, dt);
  }

 private:
  [[nodiscard]] unit::Torque Apply(const unit::Ratio drive,
                                   const unit::Ratio brake,
                                   const unit::Velocity speed,
                                   const unit::Time dt) noexcept {
    const unit::Velocity kSpeed = unit::IsFinite(speed) ? speed : unit::Velocity{};
    const unit::Velocity kStandstill = unit::Abs(parameters_.standstill_speed);
    const bool kStanding = unit::Abs(kSpeed) <= kStandstill;
    SelectGear(kStanding);

    const float kGearDirection = static_cast<float>(Direction(active_gear_));
    const unit::Torque kDriveTorque = unit::Max(parameters_.drive_torque, unit::Torque{});

    unit::Torque target{};
    if (brake > unit::Ratio{}) {
      // Against the motion, fading out towards standstill.
      const unit::Ratio kFade = (kStandstill > unit::Velocity{})
                                    ? (unit::Abs(kSpeed) / kStandstill).Clamp(0.0_ratio, 1.0_ratio)
                                    : 1.0_ratio;
      const float kSign = (kSpeed < unit::Velocity{}) ? 1.0F : -1.0F;
      target = kSign * (brake * kFade * unit::Max(parameters_.brake_torque, unit::Torque{}));
    } else {
      target = kGearDirection * (drive * kDriveTorque);
    }

    UpdateRamp(drive > unit::Ratio{} && brake <= unit::Ratio{}, kGearDirection * kSpeed, dt);

    // Speed limit along the motion, at standstill along the gear (braking only against the gear and in
    // neutral).
    const float kMotion = kStanding ? kGearDirection : ((kSpeed > unit::Velocity{}) ? 1.0F : -1.0F);
    if (kMotion != 0.0F) {
      const bool kWithGear = kGearDirection == kMotion;
      const Gear kLimitGear = kWithGear ? active_gear_ : ((kMotion > 0.0F) ? release_ : kReverse);
      const unit::Velocity kGearLimit =
          unit::Max(parameters_.gears[GearIndex(kLimitGear)].speed_limit, unit::Velocity{});
      const unit::Velocity kLimit = kWithGear ? ramped_limit_ : kGearLimit;
      const unit::Velocity kAlong = unit::Max(kMotion * kSpeed, unit::Velocity{});
      target = kMotion * unit::Min(kMotion * target, LimitTorque(kAlong, kLimit, kGearLimit, kDriveTorque));
    }

    torque_ = unit::IsFinite(target) ? target : unit::Torque{};
    return torque_;
  }

  /// Applies the requested gear: lowered to the release; the other direction waits in neutral for the
  /// standstill.
  void SelectGear(const bool standing) noexcept {
    const Gear kWanted = std::min(requested_gear_, release_);
    if (standing) {
      direction_ = 0;
    }
    active_gear_ = (Direction(kWanted) * direction_ < 0) ? kNeutral : kWanted;
    if (Direction(active_gear_) != 0) {
      direction_ = Direction(active_gear_);
    }
  }

  /// The ramped limit of the active gear: rises from the speed of the cart while driving.
  void UpdateRamp(const bool driving, const unit::Velocity gear_speed, const unit::Time dt) noexcept {
    const GearLimits& limits = parameters_.gears[GearIndex(active_gear_)];
    const unit::Velocity kGearLimit = unit::Max(limits.speed_limit, unit::Velocity{});
    const unit::Velocity kFrom = unit::Max(gear_speed, unit::Velocity{});
    const bool kRamped = Direction(active_gear_) != 0 && limits.acceleration > unit::Acceleration{} &&
                         unit::IsFinite(dt) && dt >= unit::Time{};
    if (!kRamped) {
      ramped_limit_ = kGearLimit;
    } else if (driving) {
      const unit::Velocity kStart = unit::Max(unit::Min(ramped_limit_, kFrom + parameters_.ramp_lead), kFrom);
      ramped_limit_ = unit::Min(kStart + limits.acceleration * dt, kGearLimit);
    } else {
      ramped_limit_ = unit::Min(kFrom, kGearLimit);
    }
  }

  /// The largest torque along the motion at @p speed: the drive torque below the band, minus it above the
  /// limit plus the band. The band is a share of the gear limit, not of the ramped limit.
  [[nodiscard]] unit::Torque LimitTorque(const unit::Velocity speed,
                                         const unit::Velocity limit,
                                         const unit::Velocity gear_limit,
                                         const unit::Torque drive_torque) const noexcept {
    const unit::Velocity kBand =
        unit::Max(gear_limit * parameters_.speed_band.Clamp(0.0_ratio, 1.0_ratio), kMinimumBand);
    const unit::Ratio kHeadroom = ((limit - speed) / kBand).Clamp(-1.0_ratio, 1.0_ratio);
    return kHeadroom * drive_torque;
  }

  /// Smallest speed band, so a limit of zero still has a finite slope.
  static constexpr unit::Velocity kMinimumBand{0.02_m_per_s};

  PedalMapParameters parameters_;
  Gear requested_gear_{kNeutral};
  Gear active_gear_{kNeutral};
  Gear release_{kFirstGear};
  /// Direction of the last driving gear since the last standstill; a gear of the other direction waits.
  int direction_{0};
  unit::Velocity ramped_limit_;
  unit::Torque torque_;
};

}  // namespace unimoc::vehicle
