/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file back_emf_speed.hpp
 *    @brief Speed of a PMSM from the back-EMF at its open terminals.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include "stator_system.hpp"
#include "units.hpp"

namespace unimoc::observer {

using namespace unit;

/**
 * @brief Speed measurement of a coasting PMSM from its phase voltages.
 *
 * While the outputs are open no current flows and the terminals carry the
 * back-EMF alone, e = omega psi (-sin theta, cos theta) in the stator frame.
 * Its magnitude gives the speed, |omega| = |e| / psi, and the sense in which
 * the vector turns gives the direction (the vector alone cannot tell omega
 * from -omega with the angle turned by half a revolution). The flux observer
 * gives the angle from the same voltages; this measurement lets the mechanical
 * observer find the speed of a motor whose estimate went stale while the
 * outputs were off, including a reversed or a stopped one, where the angle
 * measurement alone cannot pull the speed in.
 *
 * Below `min_back_emf` the voltages carry no usable direction, only a bound:
 * |omega| <= min_back_emf / psi (Measurement::upper_bound).
 *
 * The magnitude rests on psi of the settings: an error of psi scales the
 * measured speed, so its noise has a relative part and the flux observer's
 * angle measurement (with its smaller noise) keeps the speed right while the
 * motor turns.
 */
class BackEmfSpeed {
 public:
  /// One speed measurement for the mechanical observer.
  struct Measurement {
    unit::AngularVelocity speed;             ///< Electrical speed, signed; the bound when upper_bound.
    unit::AngularVelocityVariance variance;  ///< Its noise variance (Kalman R).
    /// Below the band the voltages only bound the speed: |omega| <= speed. Correct an estimate
    /// beyond it towards the bound; leave one within it (Hall or the prediction know better).
    bool upper_bound{false};
  };

  /// Tuning of the measurement.
  struct Parameters {
    /// Back-EMF below which the motor is taken to stand.
    unit::Voltage min_back_emf{0.3_V};
    /// Noise of the magnitude of the back-EMF (sensor noise, residual common mode).
    unit::Voltage noise{0.1_V};
    /// Relative noise of the speed: the uncertainty of psi.
    float relative_noise{0.05F};
    /// Weight of a new cycle in the direction filter (0 to 1).
    float direction_weight{0.2F};
  };

  /// With the default tuning.
  constexpr BackEmfSpeed() noexcept = default;
  /// @param parameters Tuning of the measurement.
  constexpr explicit BackEmfSpeed(const Parameters& parameters) noexcept : parameters_{parameters} {}

  /// The tuning of the measurement.
  [[nodiscard]] constexpr const Parameters& GetParameters() const noexcept { return parameters_; }

  /** @brief Forgets the previous sample and the direction. */
  constexpr void Reset() noexcept {
    has_last_ = false;
    rotation_ = unit::Ratio{};
  }

  /**
   * @brief Evaluates the samples of one control cycle.
   * @param samples Stator-frame phase voltages of the cycle, in sample order.
   * @param psi Permanent-magnet flux linkage of the motor; positive.
   * @return The measurement, or nothing without samples or without a positive psi.
   */
  [[nodiscard]] std::optional<Measurement> Update(std::span<const system::Stator<unit::Voltage>> samples,
                                                  const unit::MagneticFlux psi) noexcept {
    if (samples.empty() || !(psi > unit::MagneticFlux{})) {
      return std::nullopt;
    }
    using VoltageSquared = decltype(unit::Voltage{} * unit::Voltage{});
    VoltageSquared magnitude_sum{};
    unit::Ratio turn{};  // sum of sin(angle step) of consecutive samples
    for (const auto& sample : samples) {
      const VoltageSquared kSquare = (sample.alpha * sample.alpha) + (sample.beta * sample.beta);
      magnitude_sum += kSquare;
      if (has_last_) {
        const VoltageSquared kCross = (last_.alpha * sample.beta) - (last_.beta * sample.alpha);
        const VoltageSquared kNorm = Sqrt(kSquare * last_square_);
        if (kNorm > VoltageSquared{}) {
          turn += kCross / kNorm;
        }
      }
      last_ = sample;
      last_square_ = kSquare;
      has_last_ = true;
    }
    const unit::Voltage kMagnitude = Sqrt(magnitude_sum / static_cast<float>(samples.size()));
    const unit::Voltage kBand = Max(parameters_.min_back_emf, parameters_.noise);
    if (kMagnitude < kBand) {
      rotation_ = unit::Ratio{};
      const unit::AngularVelocity kBandSpeed = (kBand / psi) * 1.0_rad;
      return Measurement{kBandSpeed, kBandSpeed * kBandSpeed, true};
    }
    rotation_ = (rotation_ * (1.0F - parameters_.direction_weight)) + (turn * parameters_.direction_weight);
    const unit::AngularVelocity kMagnitudeSpeed = (kMagnitude / psi) * 1.0_rad;
    const unit::AngularVelocity kSpeed = rotation_ < unit::Ratio{} ? -kMagnitudeSpeed : kMagnitudeSpeed;
    const unit::AngularVelocity kNoiseSpeed =
        ((parameters_.noise / psi) * 1.0_rad) + (kMagnitudeSpeed * parameters_.relative_noise);
    return Measurement{kSpeed, kNoiseSpeed * kNoiseSpeed};
  }

  /// Filtered sense of rotation: positive counter-clockwise (increasing angle), zero while unknown.
  [[nodiscard]] constexpr unit::Ratio Rotation() const noexcept { return rotation_; }

 private:
  Parameters parameters_;                                    ///< Tuning of the measurement.
  system::Stator<unit::Voltage> last_;                       ///< Last sample of the previous cycle.
  decltype(unit::Voltage{} * unit::Voltage{}) last_square_;  ///< Its squared magnitude.
  bool has_last_{false};                                     ///< last_ is valid.
  unit::Ratio rotation_;                                     ///< Low-passed sense of rotation.
};

}  // namespace unimoc::observer
