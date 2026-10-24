/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file unit_symbol.hpp
 *    @brief SI symbol of a unit type, for the register description file.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <string_view>
#include <type_traits>
#include "units.hpp"

namespace unimoc::settings {

/**
 * @brief SI symbol of a dimension of the drive, empty for a dimensionless one or an unnamed one.
 *
 * Torque is shown as N m and the inertia as kg m^2 (the radian they carry
 * in `unit::dim` is left out, as on a data sheet). A register whose type has
 * no symbol here names its unit in its description.
 */
template <typename Dim>
[[nodiscard]] constexpr std::string_view DimensionSymbol() noexcept {
  namespace dim = unit::dim;
  if constexpr (std::is_same_v<Dim, dim::Time>) {
    return "s";
  } else if constexpr (std::is_same_v<Dim, dim::Frequency>) {
    return "1/s";
  } else if constexpr (std::is_same_v<Dim, dim::Length>) {
    return "m";
  } else if constexpr (std::is_same_v<Dim, dim::Mass>) {
    return "kg";
  } else if constexpr (std::is_same_v<Dim, dim::Current>) {
    return "A";
  } else if constexpr (std::is_same_v<Dim, dim::Voltage>) {
    return "V";
  } else if constexpr (std::is_same_v<Dim, dim::Resistance>) {
    return "Ohm";
  } else if constexpr (std::is_same_v<Dim, dim::Inductance>) {
    return "H";
  } else if constexpr (std::is_same_v<Dim, dim::MagneticFlux>) {
    return "Wb";
  } else if constexpr (std::is_same_v<Dim, dim::Power>) {
    return "W";
  } else if constexpr (std::is_same_v<Dim, dim::Temperature>) {
    return "K";
  } else if constexpr (std::is_same_v<Dim, dim::Angle>) {
    return "rad";
  } else if constexpr (std::is_same_v<Dim, dim::AngularVelocity>) {
    return "rad/s";
  } else if constexpr (std::is_same_v<Dim, dim::AngularAcceleration>) {
    return "rad/s^2";
  } else if constexpr (std::is_same_v<Dim, dim::Torque>) {
    return "N m";
  } else if constexpr (std::is_same_v<Dim, dim::Inertia>) {
    return "kg m^2";
  } else {
    return {};
  }
}

/// SI symbol of a register field: of its unit type, empty for a plain number, an enum or text.
template <typename Field>
[[nodiscard]] constexpr std::string_view UnitSymbol() noexcept {
  if constexpr (unit::UnitLike<Field>) {
    return DimensionSymbol<typename Field::dimension>();
  } else {
    return {};
  }
}

}  // namespace unimoc::settings
