/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file gear_buttons.hpp
 *    @brief Gear selection with one button per gear (R, N, 1, 2, 3) and a lamp
 *           in each: debounce, stuck buttons, the lamp states.
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
#include <optional>
#include "pedal_map.hpp"
#include "units.hpp"

namespace unimoc::vehicle {

using unit::operator""_ms;
using unit::operator""_s;

/// Number of gear buttons: reverse, neutral and the forward gears.
inline constexpr std::size_t kGearButtonCount = 2U + static_cast<std::size_t>(kTopGear);

/// A state per gear button, in the order R, N, 1, 2, 3.
template <typename T>
using PerGearButton = std::array<T, kGearButtonCount>;

/// Raw button inputs: true while pressed, in the order R, N, 1, 2, 3.
using GearButtonStates = PerGearButton<bool>;

/// The gear of button @p index (0 reverse, 1 neutral, 2 to 4 the forward gears).
[[nodiscard]] constexpr Gear ButtonGear(const std::size_t index) noexcept {
  return static_cast<Gear>(static_cast<int>(index) + kReverse);
}

/// The button of @p gear.
[[nodiscard]] constexpr std::size_t GearButton(const Gear gear) noexcept {
  return static_cast<std::size_t>(gear - kReverse);
}

/// A duration in microseconds as a time.
[[nodiscard]] constexpr unit::Time ElapsedTime(const uint64_t from_us, const uint64_t to_us) noexcept {
  return (to_us >= from_us) ? 1.0e-6_s * static_cast<float>(to_us - from_us) : unit::Time{};
}

/// Timing of the gear buttons.
struct GearButtonParameters {
  /// A raw input counts once it was stable this long.
  unit::Time debounce{20.0_ms};
  /// A button held longer counts as stuck and is ignored until it is released.
  unit::Time stuck{2.0_s};
};

/**
 * @brief Debounces the gear buttons and turns a press into a gear request.
 *
 * A press counts on the debounced edge, and only when it is the one button
 * held: two buttons at once select nothing. A button held longer than the
 * stuck time is ignored until released and reported by Stuck(), so a
 * jammed button does not block the others for good.
 */
class GearButtons {
 public:
  /// Takes new timing.
  void Configure(const GearButtonParameters& parameters) noexcept { parameters_ = parameters; }

  /**
   * @brief Feeds the raw inputs of one cycle.
   * @param raw Raw button states, true while pressed.
   * @param now_us Time of the sample.
   * @return The gear of a new press, if any.
   */
  [[nodiscard]] std::optional<Gear> Update(const GearButtonStates& raw, const uint64_t now_us) noexcept {
    std::optional<std::size_t> pressed;
    for (std::size_t i = 0U; i < kGearButtonCount; ++i) {
      Button& button = buttons_[i];
      if (raw[i] != button.raw) {
        button.raw = raw[i];
        button.raw_since_us = now_us;
      }
      if (button.raw != button.stable && ElapsedTime(button.raw_since_us, now_us) >= parameters_.debounce) {
        button.stable = button.raw;
        button.stable_since_us = now_us;
        button.stuck = false;
        if (button.stable) {
          pressed = i;
        }
      }
      if (button.stable && ElapsedTime(button.stable_since_us, now_us) > parameters_.stuck) {
        button.stuck = true;
      }
    }
    if (!pressed.has_value() || HeldCount() != 1U) {
      return std::nullopt;
    }
    return ButtonGear(*pressed);
  }

  /// A button is held longer than the stuck time.
  [[nodiscard]] bool Stuck() const noexcept {
    for (const Button& button : buttons_) {
      if (button.stuck) {
        return true;
      }
    }
    return false;
  }

 private:
  struct Button {
    bool raw{false};
    bool stable{false};
    bool stuck{false};
    uint64_t raw_since_us{0U};
    uint64_t stable_since_us{0U};
  };

  /// Debounced buttons held, not counting stuck ones.
  [[nodiscard]] std::size_t HeldCount() const noexcept {
    std::size_t count = 0U;
    for (const Button& button : buttons_) {
      if (button.stable && !button.stuck) {
        ++count;
      }
    }
    return count;
  }

  GearButtonParameters parameters_;
  PerGearButton<Button> buttons_{};
};

/// What the lamp of a gear button shows.
enum class LampState : uint8_t {
  /// Not the requested and not the active gear.
  kOff,
  /// The gear in effect.
  kOn,
  /// Requested, waiting (reverse while the cart rolls): slow blinking.
  kWaiting,
  /// Pressed but not released: fast blinking for a moment.
  kRefused,
};

/// No gear: no refused press to show.
inline constexpr Gear kNoGear = INT8_MIN;

/**
 * @brief The lamp states of the gear buttons.
 * @param requested The requested gear.
 * @param active The gear in effect.
 * @param refused The gear of a refused press that is still shown, kNoGear for none.
 * @return A state per button, R, N, 1, 2, 3.
 */
[[nodiscard]] constexpr PerGearButton<LampState> GearLampStates(const Gear requested,
                                                                const Gear active,
                                                                const Gear refused = kNoGear) noexcept {
  PerGearButton<LampState> lamps{};
  for (std::size_t i = 0U; i < kGearButtonCount; ++i) {
    const Gear kGear = ButtonGear(i);
    if (refused == kGear) {
      lamps[i] = LampState::kRefused;
    } else if (kGear == active) {
      lamps[i] = LampState::kOn;
    } else if (kGear == requested) {
      lamps[i] = LampState::kWaiting;
    } else {
      lamps[i] = LampState::kOff;
    }
  }
  return lamps;
}

/// Half period of the slow blinking (waiting) [us].
inline constexpr uint64_t kWaitingBlinkUs = 500000U;
/// Half period of the fast blinking (refused) [us].
inline constexpr uint64_t kRefusedBlinkUs = 100000U;

/// Whether a lamp in @p state is lit at @p now_us (blinking from time zero).
[[nodiscard]] constexpr bool LampLit(const LampState state, const uint64_t now_us) noexcept {
  switch (state) {
    case LampState::kOn:
      return true;
    case LampState::kWaiting:
      return ((now_us / kWaitingBlinkUs) % 2U) == 0U;
    case LampState::kRefused:
      return ((now_us / kRefusedBlinkUs) % 2U) == 0U;
    case LampState::kOff:
      break;
  }
  return false;
}

}  // namespace unimoc::vehicle
