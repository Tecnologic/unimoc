/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file rotor_angle.hpp
 *	@brief Fixed-point electrical rotor angle with revolution counter and sine/cosine lookup.
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include "sin_cos.hpp"
#include "units.hpp"

/**
 * @namespace unimoc Global UNIMOC namespace.
 */
/**
 * @namespace unimoc::system Coordinate-system data types.
 */
namespace unimoc::system {

/**
 * @brief Electrical rotor angle stored as a Q1.31 phase plus a revolution counter.
 *
 * The Q1.31 phase covers exactly one revolution, so the full 32-bit range maps to
 * [-pi, pi): `INT32_MIN` is -pi and the value wraps back to -pi after +pi. Wrapping is
 * therefore free and the accumulated position never drifts, because every advance is
 * applied in integer arithmetic. The revolution counter records how often the +pi/-pi
 * seam has been crossed.
 *
 * This type represents the electrical angle only. Converting to or from a mechanical
 * angle, including any pole-pair scaling, is the responsibility of the caller.
 */
class RotorAngle {
 public:
  /** @brief Constructs a zero angle with zero revolutions. */
  constexpr RotorAngle() noexcept = default;

  /**
   * @brief Constructs a rotor angle from an angle relative to zero.
   * @param angle Electrical angle; magnitudes beyond one revolution set the counter.
   * @return The corresponding rotor angle.
   */
  static constexpr RotorAngle FromAngle(unit::Angle angle) noexcept {
    RotorAngle result;
    result.Advance(angle);
    return result;
  }

  /**
   * @brief Constructs a rotor angle from its raw fixed-point state.
   * @param raw Q1.31 phase inside the current revolution.
   * @param revolutions Number of completed revolutions.
   * @return The corresponding rotor angle.
   */
  static constexpr RotorAngle FromRaw(std::int32_t raw, std::int32_t revolutions = 0) noexcept {
    RotorAngle result;
    result.raw_ = raw;
    result.revolutions_ = revolutions;
    result.UpdateSinCos();
    return result;
  }

  /**
   * @brief Advances the angle by an offset and updates the revolution counter.
   * @param offset Angle to add; may span any number of revolutions.
   *
   * @note The offset is converted from a float and rounded to the nearest Q1.31 count, so a
   * single call carries at most half a count of quantisation error. That error does not
   * compound, because the phase itself is accumulated in integer counts.
   */
  constexpr void Advance(unit::Angle offset) noexcept {
    const auto kRevolutions = offset.Value() * detail::kRevolutionsPerRadian;
    auto carry = static_cast<std::int64_t>(kRevolutions);
    const float kFraction = kRevolutions - static_cast<float>(carry);
    std::int64_t counts = detail::RoundToInt64(kFraction * detail::kCountsPerRevolutionF);

    // Keep the residual within half a revolution so it fits a single wrapping add.
    if (counts >= detail::kCountsPerHalfRevolution) {
      counts -= detail::kCountsPerRevolution;
      ++carry;
    } else if (counts < -detail::kCountsPerHalfRevolution) {
      counts += detail::kCountsPerRevolution;
      --carry;
    }

    revolutions_ += static_cast<std::int32_t>(carry);
    AddRaw(static_cast<std::int32_t>(counts));
    UpdateSinCos();
  }

  /**
   * @brief Returns the angle inside the current revolution.
   * @return The angle in [-pi, pi).
   */
  [[nodiscard]] constexpr unit::Angle AngleInRevolution() const noexcept { return unit::Angle(static_cast<float>(raw_) * detail::kRadiansPerCount); }

  /**
   * @brief Returns the number of completed revolutions.
   * @return The signed revolution count.
   */
  [[nodiscard]] constexpr std::int32_t Revolutions() const noexcept { return revolutions_; }

  /**
   * @brief Returns the absolute angle including all completed revolutions.
   * @return The absolute angle in radians.
   *
   * @note This getter is a float convenience. It loses resolution once the revolution
   * count grows large; use AbsoluteRaw() when exactness is required.
   */
  [[nodiscard]] constexpr unit::Angle AbsoluteAngle() const noexcept {
    return unit::Angle((static_cast<float>(revolutions_) * detail::kRadiansPerRevolution) + AngleInRevolution().Value());
  }

  /**
   * @brief Returns the Q1.31 phase inside the current revolution.
   * @return The raw fixed-point phase.
   */
  [[nodiscard]] constexpr std::int32_t Raw() const noexcept { return raw_; }

  /**
   * @brief Returns the exact absolute position in Q1.31 counts.
   * @return The absolute position, counting 2^32 counts per revolution.
   */
  [[nodiscard]] constexpr std::int64_t AbsoluteRaw() const noexcept {
    return (static_cast<std::int64_t>(revolutions_) * detail::kCountsPerRevolution) + static_cast<std::int64_t>(raw_);
  }

  /**
   * @brief Returns the sine of the angle inside the current revolution.
   * @return The interpolated sine value.
   */
  [[nodiscard]] constexpr unit::Ratio Sin() const noexcept { return sin_; }

  /**
   * @brief Returns the cosine of the angle inside the current revolution.
   * @return The interpolated cosine value.
   */
  [[nodiscard]] constexpr unit::Ratio Cos() const noexcept { return cos_; }

  /**
   * @brief Advances this angle by an offset.
   * @param offset Angle to add.
   * @return This angle after the advance.
   */
  constexpr RotorAngle& operator+=(unit::Angle offset) noexcept {
    Advance(offset);
    return *this;
  }

  /**
   * @brief Retards this angle by an offset.
   * @param offset Angle to subtract.
   * @return This angle after the advance.
   */
  constexpr RotorAngle& operator-=(unit::Angle offset) noexcept {
    Advance(-offset);
    return *this;
  }

  /**
   * @brief Returns this angle advanced by an offset.
   * @param offset Angle to add.
   * @return The advanced angle.
   */
  constexpr RotorAngle operator+(unit::Angle offset) const noexcept {
    RotorAngle result = *this;
    result.Advance(offset);
    return result;
  }

  /**
   * @brief Returns this angle retarded by an offset.
   * @param offset Angle to subtract.
   * @return The retarded angle.
   */
  constexpr RotorAngle operator-(unit::Angle offset) const noexcept {
    RotorAngle result = *this;
    result.Advance(-offset);
    return result;
  }

  /**
   * @brief Returns the shortest-path difference between two angles.
   * @param other Angle to subtract.
   * @return The difference in (-pi, pi], ignoring the revolution counters.
   */
  constexpr unit::Angle operator-(const RotorAngle& other) const noexcept {
    const auto kDelta = static_cast<std::int32_t>(static_cast<std::uint32_t>(raw_) - static_cast<std::uint32_t>(other.raw_));
    return unit::Angle(static_cast<float>(kDelta) * detail::kRadiansPerCount);
  }

  /**
   * @brief Compares two rotor angles for equality.
   * @param other Angle to compare.
   * @return `true` when phase and revolution count match.
   */
  constexpr bool operator==(const RotorAngle& other) const noexcept { return (raw_ == other.raw_) && (revolutions_ == other.revolutions_); }

  /**
   * @brief Compares two rotor angles for inequality.
   * @param other Angle to compare.
   * @return `true` when phase or revolution count differ.
   */
  constexpr bool operator!=(const RotorAngle& other) const noexcept { return !(*this == other); }

 private:
  /**
   * @brief Adds a raw phase delta and counts a seam crossing.
   * @param delta Q1.31 phase delta with a magnitude of at most half a revolution.
   */
  constexpr void AddRaw(std::int32_t delta) noexcept {
    // Wrapping is done unsigned because signed overflow is undefined behaviour.
    const auto kNext = static_cast<std::int32_t>(static_cast<std::uint32_t>(raw_) + static_cast<std::uint32_t>(delta));
    if (((raw_ ^ kNext) & (delta ^ kNext)) < 0) {
      revolutions_ += (delta >= 0) ? 1 : -1;
    }
    raw_ = kNext;
  }

  /** @brief Recomputes sine and cosine from the lookup table. */
  constexpr void UpdateSinCos() noexcept {
    const auto kSinCos = GenerateSinCos(PortableSinCosProvider{}, raw_);
    sin_ = unit::Ratio(kSinCos.sin);
    cos_ = unit::Ratio(kSinCos.cos);
  }

  /// Q1.31 phase inside the current revolution.
  std::int32_t raw_{0};
  /// Number of completed revolutions.
  std::int32_t revolutions_{0};
  /// Sine of the phase inside the current revolution.
  unit::Ratio sin_{0.0F};
  /// Cosine of the phase inside the current revolution.
  unit::Ratio cos_{1.0F};
};

}  // namespace unimoc::system
