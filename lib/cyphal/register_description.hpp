/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file register_description.hpp
 *    @brief What a tool needs to show and edit a register beyond its name and value.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include <limits>
#include <string_view>

namespace unimoc::cyphal {

/// When a written register value takes effect.
enum class RegisterEffect : uint8_t {
  /// At once, also while the drive is engaged (live tuning, runtime controls).
  kImmediate,
  /// At once while the drive is disengaged; while engaged it waits for the disengage.
  kDisengaged,
  /// After a restart (node-ID, port-IDs).
  kRestart,
};

/// Name of an effect in the register description file (`registers.json`).
[[nodiscard]] constexpr std::string_view EffectName(const RegisterEffect effect) noexcept {
  switch (effect) {
    case RegisterEffect::kImmediate:
      return "immediate";
    case RegisterEffect::kDisengaged:
      return "disengaged";
    case RegisterEffect::kRestart:
      return "restart";
  }
  return "immediate";
}

/**
 * @brief Unit, limits, effect and help of one register (`docs/gui_plan.md`, section 4).
 *
 * The limits are a hint for an editor: the node's validation decides, also
 * on limits that depend on other registers or on the hardware. An infinite
 * limit means none. A read-only register keeps the default effect.
 */
struct RegisterDescription {
  /// SI unit symbol, empty for a count, a ratio or text.
  std::string_view unit{};
  /// Smallest value the node may accept.
  float minimum{-std::numeric_limits<float>::infinity()};
  /// Largest value the node may accept.
  float maximum{std::numeric_limits<float>::infinity()};
  /// When a written value takes effect.
  RegisterEffect effect{RegisterEffect::kImmediate};
  /// One line of help.
  std::string_view help{};
};

}  // namespace unimoc::cyphal
