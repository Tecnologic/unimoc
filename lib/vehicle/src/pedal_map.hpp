/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file pedal_map.hpp
 *    @brief Torque request of the cart from the pedals: brake first, drive mode
 *           limits, speed limit and a soft start.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include "units.hpp"

namespace unimoc::vehicle {

using unit::operator""_ratio;
using unit::operator""_rad_per_s;

/// Rate of change of a torque [N m/(rad s)].
using TorqueRate = decltype(unit::Torque{} / unit::Time{});

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

/// Drive mode: who drives the cart.
enum class DriveMode : uint8_t {
  /// Low speed limit, little torque, a soft start.
  kChild = 0,
  /// The full range.
  kAdult = 1,
};

/// Number of drive modes.
inline constexpr std::size_t kDriveModeCount = 2U;

/// A value per drive mode, indexed by ModeIndex().
template <typename T>
using PerMode = std::array<T, kDriveModeCount>;

/// Index of @p mode in the per-mode arrays.
[[nodiscard]] constexpr std::size_t ModeIndex(const DriveMode mode) noexcept {
  return static_cast<std::size_t>(mode);
}

/// The drive mode of a register number; nothing but 1 is the adult mode.
[[nodiscard]] constexpr DriveMode ModeAt(const std::size_t index) noexcept {
  return (index == ModeIndex(DriveMode::kAdult)) ? DriveMode::kAdult : DriveMode::kChild;
}

/// Limits of one drive mode.
struct DriveModeLimits {
  /// Largest forward wheel speed (cart frame).
  unit::AngularVelocity speed_limit;
  /// Total wheel torque at the fully pressed drive pedal.
  unit::Torque drive_torque;
  /// Rise of the drive torque; the torque falls and brakes without a ramp.
  TorqueRate drive_ramp;
};

/// Parameters of the pedal map.
struct PedalMapParameters {
  /// Limits per drive mode.
  PerMode<DriveModeLimits> modes{};
  /// Total wheel torque at the fully pressed brake pedal, in every mode.
  unit::Torque brake_torque;
  /// Pedal travel below which a pedal counts as released (sensor noise at rest).
  unit::Ratio dead_band{0.05_ratio};
  /// Width of the speed band below the limit, as a share of the limit, in which the drive torque fades out.
  unit::Ratio speed_band{0.1_ratio};
  /// Wheel speed below which the cart stands still: the brake fades out, a new drive mode applies.
  unit::AngularVelocity standstill_speed{1.0_rad_per_s};
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
 * @brief Maps the pedals to the total torque request of the cart (cart frame).
 *
 * - **Brake first:** a pressed brake pedal brakes against the direction of
 *   motion with up to the brake torque, whatever the drive pedal says. The
 *   brake torque fades out below the standstill speed, so it never drives
 *   the cart backward (the same rule as the limp-home mode of the drives).
 * - **Drive forward only:** the drive pedal drives forward up to the drive
 *   torque of the active mode; reverse needs a direction selector (open).
 * - **Speed limit:** while the cart moves forward, the request is at most
 *   a limit torque that falls from the drive torque at the bottom of the
 *   speed band to zero at the speed limit and brakes above it, down to minus
 *   the drive torque at the speed limit plus the band. Rolling downhill above
 *   the limit with the pedals released therefore brakes regeneratively.
 * - **Soft start:** a rising drive torque follows the drive ramp of the
 *   mode; a falling torque and every braking torque apply at once.
 * - **Drive mode:** a requested mode applies at the next standstill, so the
 *   limits never jump while the cart moves.
 *
 * Pure logic in the thread of the VDC: no bus, no hardware.
 */
class PedalMap {
 public:
  /// Takes new parameters; the state is kept.
  void Configure(const PedalMapParameters& parameters) noexcept { parameters_ = parameters; }

  /// The parameters in effect.
  [[nodiscard]] const PedalMapParameters& Parameters() const noexcept { return parameters_; }

  /// Asks for @p mode; it applies at the next standstill.
  void RequestMode(const DriveMode mode) noexcept { requested_mode_ = mode; }

  /// The requested drive mode.
  [[nodiscard]] DriveMode RequestedMode() const noexcept { return requested_mode_; }

  /// The drive mode in effect.
  [[nodiscard]] DriveMode ActiveMode() const noexcept { return active_mode_; }

  /// The last torque request.
  [[nodiscard]] unit::Torque Request() const noexcept { return torque_; }

  /// Forgets the ramp state; the next request starts from zero.
  void Reset() noexcept { torque_ = unit::Torque{}; }

  /**
   * @brief The torque request of one cycle.
   * @param pedals The fresh pedal state.
   * @param speed Wheel speed of the cart (cart frame, positive forward).
   * @param dt Time since the last cycle.
   * @return Total wheel torque, cart frame.
   */
  [[nodiscard]] unit::Torque Update(const Pedals& pedals,
                                    const unit::AngularVelocity speed,
                                    const unit::Time dt) noexcept {
    return Apply(PastDeadBand(DriveTravel(pedals.drive), parameters_.dead_band),
                 PastDeadBand(BrakeTravel(pedals.brake), parameters_.dead_band),
                 speed,
                 dt);
  }

  /**
   * @brief The torque request of one cycle without a pedal state: brake gently.
   * @param speed Wheel speed of the cart (cart frame, positive forward).
   * @param dt Time since the last cycle.
   * @return Total wheel torque, cart frame.
   */
  [[nodiscard]] unit::Torque UpdateLost(const unit::AngularVelocity speed, const unit::Time dt) noexcept {
    return Apply(unit::Ratio{}, parameters_.lost_pedal_brake.Clamp(0.0_ratio, 1.0_ratio), speed, dt);
  }

 private:
  [[nodiscard]] unit::Torque Apply(const unit::Ratio drive,
                                   const unit::Ratio brake,
                                   const unit::AngularVelocity speed,
                                   const unit::Time dt) noexcept {
    const unit::AngularVelocity kSpeed = unit::IsFinite(speed) ? speed : unit::AngularVelocity{};
    const unit::AngularVelocity kStandstill = unit::Abs(parameters_.standstill_speed);
    if (unit::Abs(kSpeed) <= kStandstill) {
      active_mode_ = requested_mode_;
    }
    const DriveModeLimits& limits = parameters_.modes[ModeIndex(active_mode_)];
    const unit::Torque kDriveTorque = unit::Max(limits.drive_torque, unit::Torque{});

    unit::Torque target{};
    if (brake > unit::Ratio{}) {
      // Against the motion, fading out towards standstill.
      const unit::Ratio kFade = (kStandstill > unit::AngularVelocity{})
                                    ? (unit::Abs(kSpeed) / kStandstill).Clamp(0.0_ratio, 1.0_ratio)
                                    : 1.0_ratio;
      const float kSign = (kSpeed < unit::AngularVelocity{}) ? 1.0F : -1.0F;
      target = kSign * (brake * kFade * unit::Max(parameters_.brake_torque, unit::Torque{}));
    } else {
      target = drive * kDriveTorque;
    }

    if (kSpeed > unit::AngularVelocity{}) {
      target = unit::Min(target, SpeedLimitTorque(kSpeed, limits.speed_limit, kDriveTorque));
    }

    // Soft start: only a rising drive torque is ramped.
    const unit::Torque kFrom = unit::Max(torque_, unit::Torque{});
    if (target > kFrom && unit::IsFinite(dt) && dt >= unit::Time{}) {
      target = unit::Min(target, kFrom + unit::Max(limits.drive_ramp, TorqueRate{}) * dt);
    }
    torque_ = unit::IsFinite(target) ? target : unit::Torque{};
    return torque_;
  }

  /// The largest torque at @p speed: the drive torque below the band, minus it above the limit plus the band.
  [[nodiscard]] unit::Torque SpeedLimitTorque(const unit::AngularVelocity speed,
                                              const unit::AngularVelocity limit,
                                              const unit::Torque drive_torque) const noexcept {
    const unit::AngularVelocity kLimit = unit::Max(limit, unit::AngularVelocity{});
    const unit::AngularVelocity kBand =
        unit::Max(kLimit * parameters_.speed_band.Clamp(0.0_ratio, 1.0_ratio), kMinimumBand);
    const unit::Ratio kHeadroom = ((kLimit - speed) / kBand).Clamp(-1.0_ratio, 1.0_ratio);
    return kHeadroom * drive_torque;
  }

  /// Smallest speed band, so a limit of zero still has a finite slope.
  static constexpr unit::AngularVelocity kMinimumBand{0.1_rad_per_s};

  PedalMapParameters parameters_;
  DriveMode requested_mode_{DriveMode::kChild};
  DriveMode active_mode_{DriveMode::kChild};
  unit::Torque torque_;
};

}  // namespace unimoc::vehicle
