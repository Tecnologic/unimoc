/*
 *     __  ___   ________  _______  ______
 *    / / / / | / /  _/  |/  / __ \/ ____/
 *   / / / /  |/ // // /|_/ / / / / /
 *  / /_/ / /|  // //  /  / / /_/ / /___
 *  \____/_/ |_/___/_/  /_/\____/\____/
 *
 *  @file sin_cos.hpp
 *  @brief Sine/cosine value type and replaceable angle providers.
 *
 *  This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *  See the repository LICENSE file for details.
 */
#pragma once

#include <concepts>
#include <cstdint>
#include <numbers>
#include "rotor_angle_sin_table.hpp"
#include "units.hpp"

/**
 * @namespace unimoc Global UNIMOC namespace.
 */
/**
 * @namespace unimoc::system Coordinate-system data types.
 */
namespace unimoc::system {

/**
 * @namespace unimoc::system::detail Implementation helpers for sine/cosine providers.
 */
namespace detail {

/// Number of low bits of the Q1.31 phase used as the interpolation fraction.
inline constexpr std::uint32_t kSinTableFractionBits = 23U;
/// Mask selecting the interpolation fraction bits of the Q1.31 phase.
inline constexpr std::uint32_t kSinTableFractionMask = (1U << kSinTableFractionBits) - 1U;
/// Scale converting the fraction bits into a normalised fraction in [0, 1).
inline constexpr float kSinTableFractionScale = 1.0F / static_cast<float>(1U << kSinTableFractionBits);

/// Number of Q1.31 counts per full revolution.
inline constexpr std::int64_t kCountsPerRevolution = 0x1'0000'0000LL;
/// Number of Q1.31 counts per half revolution, the positive wrap limit.
inline constexpr std::int64_t kCountsPerHalfRevolution = 0x8000'0000LL;
/// Number of Q1.31 counts per full revolution as a float.
inline constexpr float kCountsPerRevolutionF = static_cast<float>(kCountsPerRevolution);
/// Radians represented by a single Q1.31 count.
inline constexpr float kRadiansPerCount = static_cast<float>(2.0 * std::numbers::pi_v<double> / static_cast<double>(kCountsPerRevolution));
/// Revolutions represented by one radian.
inline constexpr float kRevolutionsPerRadian = static_cast<float>(1.0 / (2.0 * std::numbers::pi_v<double>));
/// Radians per full revolution.
inline constexpr float kRadiansPerRevolution = 2.0F * std::numbers::pi_v<float>;

/**
 * @brief Interpolates a table segment linearly.
 * @param functionValueStart Function value at the start of the segment.
 * @param functionValueEnd Function value at the end of the segment.
 * @param interpolationFactor Normalised position inside the segment, in [0, 1).
 * @return The interpolated function value.
 */
constexpr float LinearInterpolate(float functionValueStart, float functionValueEnd, float interpolationFactor) noexcept {
  return functionValueStart + (interpolationFactor * (functionValueEnd - functionValueStart));
}

/**
 * @brief Rounds a float to the nearest integer, away from zero on ties.
 * @param value Value to round.
 * @return The rounded value.
 */
constexpr std::int64_t RoundToInt64(float value) noexcept { return static_cast<std::int64_t>(value >= 0.0F ? value + 0.5F : value - 0.5F); }

}  // namespace detail

/**
 * @brief Holds the sine and cosine of one angle.
 * @tparam Representation Numeric representation used by both values.
 */
template <typename Representation = unit::Ratio>
struct SinCos {
  /// Sine of the angle.
  Representation sin{0.0F};
  /// Cosine of the angle.
  Representation cos{1.0F};

  /** @brief Constructs the zero-angle pair. */
  constexpr SinCos() noexcept = default;

  /**
   * @brief Constructs a pair from sine and cosine values.
   * @param sinValue Sine value.
   * @param cosValue Cosine value.
   */
  constexpr SinCos(Representation sinValue, Representation cosValue) noexcept : sin(sinValue), cos(cosValue) {}

  /**
   * @brief Calculates a portable sine/cosine pair from an angle.
   * @param angle Angle in radians.
   */
  explicit constexpr SinCos(unit::Angle angle) noexcept;
};

/**
 * @brief Portable sine/cosine provider backed by a 512-entry lookup table.
 *
 * The provider linearly interpolates adjacent table entries and applies a
 * first-order magnitude correction. It is constexpr, deterministic, and has
 * no hardware dependencies, so it is the default provider for RotorAngle and
 * remains available on MCUs without a mathematical accelerator.
 *
 * @note Provider comparison for the current 512-entry table:
 *
 * | Provider | Maximum value error | Approximate work |
 * | --- | ---: | ---: |
 * | Direct lower-entry lookup | 1.23e-2 | 2 table loads |
 * | Direct nearest-entry lookup | 6.14e-3 | 2 table loads plus index rounding |
 * | Linear interpolation | 1.88e-5 | 4 table loads and FPU arithmetic |
 * | Linear interpolation plus normalisation | 1.4e-7 | Linear work plus 7 FPU operations |
 * | Cubic Hermite interpolation | 1.5e-7 in float32 | 4 table loads and substantially more FPU arithmetic |
 * | STM32G4 CORDIC, 6-cycle Q1.31 | 2^-19 documented residual | Peripheral setup, transfers, and 6 CORDIC iterations |
 *
 * The instruction and peripheral-work figures are implementation estimates;
 * flash wait states, memory placement, compiler scheduling, and transfer mode
 * affect the measured cost. The CORDIC residual is the maximum expected error
 * documented by ST for its six-cycle Q1.31 sine/cosine configuration.
 *
 * @note Decision: linear interpolation plus normalisation is the portable
 * default. Sine/cosine generation runs in the slow control task and the fast
 * current loop consumes precomputed values, so the extra correction work does
 * not burden the timing-critical loop. It provides better practical float32
 * accuracy than the six-cycle CORDIC while preserving constexpr operation and
 * portability. A target-specific provider can implement the same Calculate()
 * contract when profiling shows that slow-task CPU time matters.
 */
struct PortableSinCosProvider {
  /// Result type returned by this provider.
  using Result = SinCos<unit::Ratio>;

  /**
   * @brief Calculates sine and cosine from a raw Q1.31 phase.
   * @param raw Phase where one full revolution is 2^32 counts.
   * @return The interpolated and normalised sine/cosine pair.
   */
  [[nodiscard]] static constexpr Result Calculate(std::int32_t raw) noexcept {
    const auto kPhase = static_cast<std::uint32_t>(raw);
    const auto kTableIndex = kPhase >> detail::kSinTableFractionBits;
    const float kInterpolationFactor = static_cast<float>(kPhase & detail::kSinTableFractionMask) * detail::kSinTableFractionScale;
    const float* const kTable = detail::kSinTable.data() + kTableIndex;

    const float kSinValue = detail::LinearInterpolate(kTable[0U], kTable[1U], kInterpolationFactor);
    const float kCosValue =
        detail::LinearInterpolate(kTable[detail::kSinTableQuarterSize], kTable[detail::kSinTableQuarterSize + 1U], kInterpolationFactor);
    const float kNormalization = 1.5F - (0.5F * ((kSinValue * kSinValue) + (kCosValue * kCosValue)));

    return Result{unit::Ratio{kSinValue * kNormalization}, unit::Ratio{kCosValue * kNormalization}};
  }

  /**
   * @brief Calculates sine and cosine from an angle in radians.
   * @param angle Electrical angle; complete revolutions are discarded.
   * @return The interpolated and normalised sine/cosine pair.
   */
  [[nodiscard]] static constexpr Result Calculate(unit::Angle angle) noexcept {
    const auto kRevolutions = angle.Value() * detail::kRevolutionsPerRadian;
    const auto kCarry = static_cast<std::int64_t>(kRevolutions);
    const float kFraction = kRevolutions - static_cast<float>(kCarry);
    std::int64_t counts = detail::RoundToInt64(kFraction * detail::kCountsPerRevolutionF);

    if (counts >= detail::kCountsPerHalfRevolution) {
      counts -= detail::kCountsPerRevolution;
    } else if (counts < -detail::kCountsPerHalfRevolution) {
      counts += detail::kCountsPerRevolution;
    }

    return Calculate(static_cast<std::int32_t>(counts));
  }
};

/**
 * @brief Concept implemented by synchronous sine/cosine providers.
 *
 * A hardware provider may use the same contract around a peripheral-backed
 * calculation. Asynchronous providers should publish their completed result
 * as a SinCos value before the control loop consumes it.
 */
template <typename Provider>
concept SinCosProvider = requires(const Provider kProvider, std::int32_t raw) {
  typename Provider::Result;
  requires std::same_as<typename Provider::Result, SinCos<unit::Ratio>>;
  { kProvider.Calculate(raw) } -> std::same_as<SinCos<unit::Ratio>>;
};

static_assert(SinCosProvider<PortableSinCosProvider>);

/**
 * @brief Calculates a sine/cosine pair through a provider.
 * @tparam Provider Provider implementation.
 * @param provider Provider instance; stateless providers are also supported.
 * @param raw Raw Q1.31 phase.
 * @return The provider result.
 */
template <SinCosProvider Provider>
[[nodiscard]] constexpr SinCos<unit::Ratio> GenerateSinCos(const Provider& provider, std::int32_t raw) noexcept {
  return provider.Calculate(raw);
}

template <typename Representation>
constexpr SinCos<Representation>::SinCos(unit::Angle angle) noexcept {
  const auto kResult = PortableSinCosProvider::Calculate(angle);
  sin = Representation{kResult.sin.Value()};
  cos = Representation{kResult.cos.Value()};
}

}  // namespace unimoc::system