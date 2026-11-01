/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file impedance_estimator.hpp
 *    @brief Phasors of a sinusoidal injection and the winding impedance they give.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include <optional>
#include "sin_cos.hpp"
#include "units.hpp"

namespace unimoc::identification {

/**
 * @brief Phasor of one frequency, accumulated sample by sample (a single DFT bin).
 *
 * Measurement M2 of the motor identification (docs/motor_identification_plan.md,
 * section 4.2) injects a sine whose phase the drive generates itself, so the
 * phase of every sample is known: the bin is the sum of x e^(-j phi) with that
 * phase, the evaluation of the Goertzel filter at the injection frequency
 * without its recursion. For x = A cos(phi + alpha) over whole periods the
 * phasor is A e^(j alpha); a constant part of x (the d bias) drops out.
 *
 * @tparam T Unit type of the samples.
 */
template <typename T>
class PhasorAccumulator {
 public:
  /** @brief Forgets all samples. */
  constexpr void Reset() noexcept { *this = PhasorAccumulator{}; }

  /**
   * @brief Adds one sample.
   * @param value Sample.
   * @param phase Sine and cosine of the injection phase at the sample.
   */
  constexpr void Add(T value, const system::SinCos<unit::Ratio>& phase) noexcept {
    real_ = real_ + (value * phase.cos);
    imaginary_ = imaginary_ - (value * phase.sin);
    ++count_;
  }

  /**
   * @brief Number of samples added.
   * @return The sample count.
   */
  [[nodiscard]] constexpr uint32_t Count() const noexcept { return count_; }

  /**
   * @brief Real part of the phasor, 2/N sum x cos(phi).
   * @return The in-phase amplitude; zero without samples.
   */
  [[nodiscard]] constexpr T Real() const noexcept { return Scaled(real_); }

  /**
   * @brief Imaginary part of the phasor, -2/N sum x sin(phi).
   * @return The quadrature amplitude; zero without samples.
   */
  [[nodiscard]] constexpr T Imaginary() const noexcept { return Scaled(imaginary_); }

  /**
   * @brief Squared magnitude of the phasor.
   * @return Real^2 + Imaginary^2.
   */
  [[nodiscard]] constexpr auto MagnitudeSquared() const noexcept {
    return (Real() * Real()) + (Imaginary() * Imaginary());
  }

 private:
  [[nodiscard]] constexpr T Scaled(T sum) const noexcept {
    return count_ == 0U ? T{} : sum * (2.0F / static_cast<float>(count_));
  }

  T real_{};
  T imaginary_{};
  uint32_t count_{0U};
};

/// Resistance and inductance of a winding at one frequency.
struct Impedance {
  unit::Resistance resistance;  ///< Real part of U / I.
  unit::Inductance inductance;  ///< Imaginary part of U / I over the angular frequency.
};

/**
 * @brief The impedance Z = U / I of the voltage and current phasors.
 * @param voltage Voltage phasor of the axis.
 * @param current Current phasor of the same axis and samples.
 * @param angular_frequency Angular frequency of the injection (rad/s, positive).
 * @return R = Re Z and L = Im Z / omega; empty when the current phasor is zero.
 */
[[nodiscard]] constexpr std::optional<Impedance> EstimateImpedance(
    const PhasorAccumulator<unit::Voltage>& voltage,
    const PhasorAccumulator<unit::Current>& current,
    unit::AngularVelocity angular_frequency) noexcept {
  const auto kMagnitude = current.MagnitudeSquared();
  if (!(kMagnitude > decltype(kMagnitude){}) || !(angular_frequency > unit::AngularVelocity{})) {
    return std::nullopt;
  }
  const unit::Resistance kReal =
      ((voltage.Real() * current.Real()) + (voltage.Imaginary() * current.Imaginary())) / kMagnitude;
  const unit::Resistance kReactance =
      ((voltage.Imaginary() * current.Real()) - (voltage.Real() * current.Imaginary())) / kMagnitude;
  // X = omega L; the radian of omega drops out (an inductance is V s / A).
  using unit::operator""_rad;
  return Impedance{kReal, kReactance / (angular_frequency / 1.0_rad)};
}

}  // namespace unimoc::identification
