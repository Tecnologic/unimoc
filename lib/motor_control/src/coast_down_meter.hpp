/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file coast_down_meter.hpp
 *    @brief Flux linkage of a PMSM from the back-EMF of a coast-down (identification M3).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include <numbers>
#include "stator_system.hpp"
#include "units.hpp"

namespace unimoc::identification {

using namespace unit;

/**
 * @brief Measures psi_PM period by period while the motor coasts with the outputs open.
 *
 * Measurement M3 of the motor identification (docs/motor_identification_plan.md,
 * section 4.3). The phase voltages of the open terminals are the back-EMF
 * alone; their stator-frame vector (Clarke, the common mode drops out) has the
 * magnitude omega psi. Per electrical period:
 *
 * - the period from two rising zero crossings of the alpha component
 *   (interpolated between the samples, so the speed needs neither the pole
 *   pairs nor the observer), with a hysteresis of 30 % of the magnitude;
 * - the mean magnitude of the vector over the period: the fundamental phase
 *   amplitude (the 5th and 7th harmonics of a real back-EMF ripple the
 *   magnitude at 6 omega and drop out of the mean over a whole period);
 * - psi = mean |e| / omega.
 *
 * The result is the mean psi of the periods above `min_speed_fraction` of the
 * speed of the first period, the spread their largest relative deviation from
 * that mean. A period in which the magnitude fell below `min_back_emf` is
 * dropped. The meter reports standstill once the magnitude stayed below
 * `min_back_emf` for `standstill_samples` samples.
 */
class CoastDownMeter {
 public:
  /// Result of the coast so far.
  struct Result {
    unit::MagneticFlux flux;            ///< Mean psi_PM of the counted periods.
    unit::Ratio spread;                 ///< Largest relative deviation of a counted period from the mean.
    unit::AngularVelocity start_speed;  ///< Electrical speed of the first period (signed).
    unit::AngularVelocity last_speed;   ///< Electrical speed of the last period (signed).
    uint32_t periods{0U};               ///< Counted periods.
  };

  /// Magnitude below which a sample carries no back-EMF.
  unit::Voltage min_back_emf{0.3_V};
  /// Periods count above this fraction of the speed of the first period.
  float min_speed_fraction{0.1F};
  /// Samples below min_back_emf that make a standstill.
  uint32_t standstill_samples{1000U};

  /** @brief Starts a new coast; keeps the parameters. */
  constexpr void Start() noexcept {
    const unit::Voltage kMin = min_back_emf;
    const float kFraction = min_speed_fraction;
    const uint32_t kStandstill = standstill_samples;
    *this = CoastDownMeter{};
    min_back_emf = kMin;
    min_speed_fraction = kFraction;
    standstill_samples = kStandstill;
  }

  /**
   * @brief Adds one sample.
   * @param voltage Stator-frame phase voltage of the open terminals.
   * @param sample_period Time from the previous sample.
   */
  void Add(const system::Stator<unit::Voltage>& voltage, const unit::Time sample_period) noexcept {
    ++sample_;
    const unit::Voltage kMagnitude = Hypot(voltage.alpha, voltage.beta);
    if (kMagnitude < min_back_emf) {
      ++below_;
      armed_ = false;
      period_valid_ = false;
      has_last_alpha_ = false;
      return;
    }
    below_ = 0U;
    if (voltage.alpha < -(kMagnitude * kHysteresis)) {
      armed_ = true;
    }
    const bool kCrossing =
        armed_ && has_last_alpha_ && last_alpha_ < unit::Voltage{} && !(voltage.alpha < unit::Voltage{});
    if (kCrossing) {
      // Fraction of the step from the previous sample at which alpha crosses zero (0, 1].
      const unit::Ratio kFraction = last_alpha_ / (last_alpha_ - voltage.alpha);
      const unit::Voltage kBeta = last_beta_ + ((voltage.beta - last_beta_) * kFraction);
      if (period_valid_ && has_crossing_ && magnitude_count_ > 0U) {
        const unit::Time kPeriod = (sample_period * static_cast<float>(sample_ - crossing_sample_)) +
                                   (sample_period * (kFraction - crossing_fraction_));
        // At a rising alpha crossing a positive (counter-clockwise) rotation has beta < 0.
        const unit::AngularVelocity kSpeedMagnitude = kFullTurn / kPeriod;
        const unit::AngularVelocity kSpeed = kBeta < unit::Voltage{} ? kSpeedMagnitude : -kSpeedMagnitude;
        const unit::Voltage kMean = magnitude_sum_ / static_cast<float>(magnitude_count_);
        AddPeriod(kMean / kSpeedMagnitude, kSpeed);
      }
      has_crossing_ = true;
      period_valid_ = true;
      armed_ = false;
      crossing_sample_ = sample_;
      crossing_fraction_ = kFraction;
      magnitude_sum_ = unit::Voltage{};
      magnitude_count_ = 0U;
    }
    magnitude_sum_ += kMagnitude;
    ++magnitude_count_;
    last_alpha_ = voltage.alpha;
    last_beta_ = voltage.beta;
    has_last_alpha_ = true;
  }

  /// True once the magnitude stayed below min_back_emf for standstill_samples samples.
  [[nodiscard]] constexpr bool Standstill() const noexcept { return below_ >= standstill_samples; }

  /**
   * @brief Returns the result of the counted periods.
   *
   * The spread needs a second pass over the periods, which are not stored: it
   * is the larger deviation of the smallest and the largest psi from the mean.
   */
  [[nodiscard]] constexpr Result GetResult() const noexcept {
    Result result{};
    result.start_speed = start_speed_;
    result.last_speed = last_speed_;
    result.periods = counted_;
    if (counted_ == 0U) {
      return result;
    }
    result.flux = flux_sum_ / static_cast<float>(counted_);
    const unit::MagneticFlux kDeviation = Max(Abs(flux_max_ - result.flux), Abs(result.flux - flux_min_));
    result.spread = kDeviation / result.flux;
    return result;
  }

 private:
  /// Hysteresis of the zero crossing relative to the magnitude.
  static constexpr float kHysteresis = 0.3F;
  /// One electrical revolution.
  static constexpr unit::Angle kFullTurn = 2.0F * std::numbers::pi_v<float> * 1.0_rad;

  /// Counts one measured period.
  constexpr void AddPeriod(const unit::MagneticFlux flux, const unit::AngularVelocity speed) noexcept {
    if (!has_start_) {
      start_speed_ = speed;
      has_start_ = true;
    }
    last_speed_ = speed;
    if (Abs(speed) < Abs(start_speed_) * min_speed_fraction) {
      return;
    }
    flux_sum_ += flux;
    flux_min_ = counted_ == 0U ? flux : Min(flux_min_, flux);
    flux_max_ = counted_ == 0U ? flux : Max(flux_max_, flux);
    ++counted_;
  }

  uint32_t sample_{0U};              ///< Samples since Start().
  uint32_t below_{0U};               ///< Consecutive samples below min_back_emf.
  bool armed_{false};                ///< Alpha was below the hysteresis since the last crossing.
  bool has_last_alpha_{false};       ///< last_alpha_ is the previous sample's.
  unit::Voltage last_alpha_{};       ///< Alpha of the previous sample.
  unit::Voltage last_beta_{};        ///< Beta of the previous sample.
  bool has_crossing_{false};         ///< A crossing was seen.
  bool period_valid_{false};         ///< The period since the last crossing kept its magnitude.
  uint32_t crossing_sample_{0U};     ///< Sample at (after) the last crossing.
  unit::Ratio crossing_fraction_{};  ///< Its position within the step before it.
  unit::Voltage magnitude_sum_{};    ///< Sum of the magnitudes since the last crossing.
  uint32_t magnitude_count_{0U};     ///< Their count.
  bool has_start_{false};            ///< start_speed_ is set.
  unit::AngularVelocity start_speed_{};
  unit::AngularVelocity last_speed_{};
  unit::MagneticFlux flux_sum_{};
  unit::MagneticFlux flux_min_{};
  unit::MagneticFlux flux_max_{};
  uint32_t counted_{0U};
};

}  // namespace unimoc::identification
