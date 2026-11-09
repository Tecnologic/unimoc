/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file wheel.hpp
 *    @brief The wheels of the cart, their axles and sides.
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

/**
 * @namespace unimoc::vehicle vehicle dynamics of the cart (the VDC application)
 */
namespace unimoc::vehicle {

/// A wheel of the cart; the values are the indices of unimoc.cart.WheelCommand.WHEEL_*.
enum class Wheel : uint8_t {
  kFrontLeft = 0,
  kFrontRight = 1,
  kRearLeft = 2,
  kRearRight = 3,
};

/// Number of wheels of the cart (the length of unimoc.cart.WheelCommand.wheel_torque).
inline constexpr std::size_t kWheelCount = 4U;

/// All wheels in index order.
inline constexpr std::array<Wheel, kWheelCount> kWheels{
    Wheel::kFrontLeft,
    Wheel::kFrontRight,
    Wheel::kRearLeft,
    Wheel::kRearRight,
};

/// An axle of the cart.
enum class Axle : uint8_t {
  kFront = 0,
  kRear = 1,
};

/// Index of @p wheel in the per-wheel arrays.
[[nodiscard]] constexpr std::size_t Index(const Wheel wheel) noexcept {
  return static_cast<std::size_t>(wheel);
}

/// The wheel of index @p index (0 to kWheelCount - 1).
[[nodiscard]] constexpr Wheel WheelAt(const std::size_t index) noexcept {
  return kWheels[index % kWheelCount];
}

/// Axle of @p wheel.
[[nodiscard]] constexpr Axle AxleOf(const Wheel wheel) noexcept {
  return (wheel == Wheel::kFrontLeft || wheel == Wheel::kFrontRight) ? Axle::kFront : Axle::kRear;
}

/// The other wheel of the axle of @p wheel.
[[nodiscard]] constexpr Wheel OppositeWheel(const Wheel wheel) noexcept {
  switch (wheel) {
    case Wheel::kFrontLeft:
      return Wheel::kFrontRight;
    case Wheel::kFrontRight:
      return Wheel::kFrontLeft;
    case Wheel::kRearLeft:
      return Wheel::kRearRight;
    case Wheel::kRearRight:
      return Wheel::kRearLeft;
  }
  return wheel;
}

/// Left and right wheel of @p axle.
[[nodiscard]] constexpr std::array<Wheel, 2> WheelsOf(const Axle axle) noexcept {
  return (axle == Axle::kFront) ? std::array<Wheel, 2>{Wheel::kFrontLeft, Wheel::kFrontRight}
                                : std::array<Wheel, 2>{Wheel::kRearLeft, Wheel::kRearRight};
}

/// A value per wheel, indexed by Index().
template <typename T>
using PerWheel = std::array<T, kWheelCount>;

}  // namespace unimoc::vehicle
