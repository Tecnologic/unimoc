/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file pedal_observer.hpp
 *    @brief Observer of the pedal torque and cadence sensor (crank angle, cadence, pedal torque and power).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <span>
#include "angle_wrap.hpp"
#include "units.hpp"

/**
 * @namespace unimoc::ebike e-bike application logic namespace
 */
namespace unimoc::ebike {

using namespace unit;

/// Largest supported number of cadence pulses per crank revolution.
inline constexpr std::size_t kMaxPedalPulses = 64U;

/**
 * @brief Parameters of the PedalObserver.
 *
 * The defaults fit the E-Rider T9N bottom bracket sensor (36 poles, 70 Nm/V
 * on a 1.5 V rest voltage; the conversion to torque is done by the platform).
 */
struct PedalObserverConfig {
  /// Cadence pulses per crank revolution (poles of the sensor ring).
  uint8_t pulses_per_rev{36U};
  /// Variance of the angle measurement of one pulse [rad^2] (edge jitter, ring tolerance).
  float angle_variance{3e-4F};
  /// Process noise of the cadence [rad^2/s^3]: how fast the rider may change the cadence.
  float cadence_noise{100.0F};
  /// No pulse for this long means the crank stands still.
  Time stop_timeout{0.5_s};
  /// Time constant of the fast (not ripple free) torque.
  Time torque_time_constant{20.0_ms};
  /// Initial offset of the torque signal (the sensor reading at zero load).
  Torque zero_offset{};
  /// The offset only follows readings this close to it (a foot resting on a pedal must not become the zero).
  Torque zero_window{3.0_Nm};
  /// The crank must stand still this long before the offset follows the reading.
  Time zero_hold{1.0_s};
  /// Time constant of the offset trim.
  Time zero_time_constant{30.0_s};
};

/**
 * @brief Observer of the crank: angle, cadence and pedal torque and power.
 *
 * The torque and cadence sensor gives a torque voltage and a pulse train of
 * a ring with N poles (one pulse per 2 pi / N of crank angle, no direction).
 * Both are noisy, and the torque of a rider is far from constant: it follows
 * the pedal stroke twice per revolution, from near zero at the dead centers
 * to a multiple of its mean. The assist is to follow the mean torque, not the
 * stroke.
 *
 * Cadence. A two-state Kalman filter (angle, cadence; constant cadence with
 * random-walk process noise) fuses the pulses. A pulse is an angle
 * measurement: the pulse index times the pulse angle, modulo one revolution,
 * as the Hall edges are fused in the motor observers. The pulse ages the
 * platform hands in (time between the edge and the end of the step) keep the
 * estimate independent of the step jitter. A pulse that is overdue limits the
 * cadence to what the elapsed time allows, so the estimate decays during a
 * stop instead of waiting for the timeout.
 *
 * Torque. The torque samples are averaged per pulse interval (bin), and the
 * mean torque is the mean of the bins of the last half revolution. Averaging
 * over exactly one stroke (180 degrees of crank angle) cancels the twice per
 * revolution ripple at any cadence, with a delay of up to half a stroke. The
 * mean runs on crank angle, not on time, so a slow crank is not averaged over
 * a long time. Until a half revolution is complete the mean uses the bins
 * available. The fast torque is a plain low-pass for display and diagnosis.
 *
 * Power. Mean torque times cadence.
 *
 * Offset. The torque has a sensor offset. While the crank stands still, the
 * offset follows the reading slowly, but only within a small window around
 * the current offset, so a rider standing on a pedal does not retrain it
 * (at most the window of bias). SetZeroOffset() restores a stored value;
 * a fresh calibration with an unloaded crank is the application's choice.
 *
 * Call Step() at a fixed rate (some 100 Hz to 1 kHz) from one thread; the
 * class does no allocation and calls nothing of the OS.
 */
class PedalObserver {
 public:
  /**
   * @brief Creates the observer.
   * @param config Sensor and filter parameters.
   */
  explicit PedalObserver(const PedalObserverConfig& config = PedalObserverConfig{}) noexcept
      : config_(config),
        pulses_per_rev_(std::clamp<std::size_t>(config.pulses_per_rev, 2U, kMaxPedalPulses)),
        pulse_angle_(2.0F * std::numbers::pi_v<float> * 1.0_rad / static_cast<float>(pulses_per_rev_)),
        zero_offset_(config.zero_offset) {
    ResetCrank();
  }

  /**
   * @brief Advances the observer by one step.
   * @param dt Time since the last step.
   * @param torque Mean torque reading of this step, offset not removed.
   * @param pulse_ages Age of every cadence pulse since the last step (time between the edge and the end
   * of the step), oldest first.
   */
  void Step(const Time dt, const Torque torque, const std::span<const Time> pulse_ages) noexcept {
    Time elapsed;
    for (const Time age : pulse_ages) {
      const Time kEdge = std::clamp(dt - age, elapsed, dt);
      Predict(kEdge - elapsed);
      elapsed = kEdge;
      OnPulse();
    }
    Predict(dt - elapsed);

    const Torque kCompensated = torque - zero_offset_;
    const Ratio kAlpha = dt / (config_.torque_time_constant + dt);
    torque_fast_ += (kCompensated - torque_fast_) * kAlpha;
    if (pulses_seen_ > 0U) {
      bin_sum_ += kCompensated;
      ++bin_count_;
    }

    TrimZero(dt, torque);
  }

  /// Estimated crank angle, wrapped to (-pi, pi]; the origin is the first pulse after a start.
  [[nodiscard]] Angle CrankAngle() const noexcept { return angle_; }
  /// Estimated cadence (crank speed), zero while the crank stands still.
  [[nodiscard]] AngularVelocity Cadence() const noexcept { return pedaling_ ? omega_ : AngularVelocity{}; }
  /// Mean pedal torque over the last half revolution, offset removed, ripple free.
  [[nodiscard]] Torque MeanTorque() const noexcept { return pedaling_ ? mean_torque_ : Torque{}; }
  /// Low-passed torque, offset removed; follows the pedal stroke and also reads a static load.
  [[nodiscard]] Torque FastTorque() const noexcept { return torque_fast_; }
  /// Mean pedal power: mean torque times cadence.
  [[nodiscard]] Power PedalPower() const noexcept { return MeanTorque() * Cadence(); }
  /// The crank turns: pulses are arriving and the cadence is known.
  [[nodiscard]] bool Pedaling() const noexcept { return pedaling_; }
  /// Current offset of the torque signal.
  [[nodiscard]] Torque ZeroOffset() const noexcept { return zero_offset_; }

  /**
   * @brief Sets the torque offset, for example restores a stored calibration.
   * @param offset Sensor reading at zero load.
   */
  void SetZeroOffset(const Torque offset) noexcept { zero_offset_ = offset; }

 private:
  static constexpr float kInitialCadenceVariance = 100.0F;  // (10 rad/s)^2, about 95 rpm
  static constexpr float kOverdueMargin = 1.25F;            // pulse jitter allowed before the cadence is cut

  // Covariance of (angle, cadence); plain numbers: the products of the unit types are not defined.
  struct Covariance {
    float angle{1.0F};
    float cross{0.0F};
    float cadence{kInitialCadenceVariance};
  };

  void ResetCrank() noexcept {
    pedaling_ = false;
    pulses_seen_ = 0U;
    omega_ = AngularVelocity{};
    p_ = Covariance{};
    time_since_pulse_ = Time{};
    bin_index_ = 0U;
    completed_bins_ = 0U;
    bin_sum_ = Torque{};
    bin_count_ = 0U;
    mean_torque_ = Torque{};
  }

  void Predict(const Time h) noexcept {
    if (pulses_seen_ == 0U) {
      still_time_ += h;
      return;
    }
    time_since_pulse_ += h;
    if (time_since_pulse_ > config_.stop_timeout) {
      const Time kStill = time_since_pulse_;
      ResetCrank();
      still_time_ = kStill;
      return;
    }
    angle_ = observer::WrapAngle(angle_ + omega_ * h);
    // Overdue pulse: the cadence cannot be much higher than one pulse angle per elapsed time.
    const Angle kLimit = pulse_angle_ * kOverdueMargin;
    if (omega_ * time_since_pulse_ > kLimit) {
      omega_ = kLimit / time_since_pulse_;
    }
    const float kH = h.Value();
    const float kQ = config_.cadence_noise;
    p_.angle += kH * (2.0F * p_.cross + kH * p_.cadence) + kQ * kH * kH * kH / 3.0F;
    p_.cross += kH * p_.cadence + kQ * kH * kH / 2.0F;
    p_.cadence += kQ * kH;
  }

  void OnPulse() noexcept {
    still_time_ = Time{};
    bin_index_ = (bin_index_ + 1U) % pulses_per_rev_;
    const Angle kMeasured = pulse_angle_ * static_cast<float>(bin_index_);
    if (pulses_seen_ == 0U) {
      // First pulse after a stop: the angle origin, the cadence is not known yet.
      bin_index_ = 0U;
      angle_ = Angle{};
      p_ = Covariance{};
      bin_sum_ = Torque{};
      bin_count_ = 0U;
      pedaling_ = false;
    } else {
      if (pulses_seen_ == 1U) {
        // The first interval is a direct speed measurement.
        omega_ = pulse_angle_ / time_since_pulse_;
        angle_ = kMeasured;
        p_ = Covariance{config_.angle_variance, 0.0F, kInitialCadenceVariance / 100.0F};
      } else {
        FuseAngle(observer::WrapAngle(kMeasured - angle_));
      }
      pedaling_ = true;
      FinishBin();
    }
    time_since_pulse_ = Time{};
    pulses_seen_ = std::min<uint32_t>(pulses_seen_ + 1U, 2U);
  }

  void FuseAngle(const Angle innovation) noexcept {
    const float kS = p_.angle + config_.angle_variance;
    const float kGainAngle = p_.angle / kS;
    const float kGainCadence = p_.cross / kS;
    angle_ = observer::WrapAngle(angle_ + innovation * kGainAngle);
    omega_ += innovation * Frequency{kGainCadence};
    p_.cadence -= kGainCadence * p_.cross;
    p_.angle *= (1.0F - kGainAngle);
    p_.cross *= (1.0F - kGainAngle);
  }

  // Closes the bin of the pulse interval that just ended and refreshes the half revolution mean.
  void FinishBin() noexcept {
    const std::size_t kClosed = (bin_index_ + pulses_per_rev_ - 1U) % pulses_per_rev_;
    bins_[kClosed] = bin_count_ > 0U ? bin_sum_ / static_cast<float>(bin_count_) : torque_fast_;
    bin_sum_ = Torque{};
    bin_count_ = 0U;
    const std::size_t kWindow = std::max<std::size_t>(pulses_per_rev_ / 2U, 1U);
    completed_bins_ = std::min(completed_bins_ + 1U, kWindow);
    Torque total;
    for (std::size_t i = 0U; i < completed_bins_; ++i) {
      total += bins_[(kClosed + pulses_per_rev_ - i) % pulses_per_rev_];
    }
    mean_torque_ = total / static_cast<float>(completed_bins_);
  }

  // Lets the offset follow the reading while the crank stands still and the reading is near the offset.
  void TrimZero(const Time dt, const Torque torque) noexcept {
    if (pedaling_ || still_time_ < config_.zero_hold) {
      return;
    }
    const Torque kError = torque - zero_offset_;
    if (kError > -config_.zero_window && kError < config_.zero_window) {
      zero_offset_ += kError * (dt / config_.zero_time_constant);
    }
  }

  PedalObserverConfig config_;
  std::size_t pulses_per_rev_;
  Angle pulse_angle_;

  Angle angle_;
  AngularVelocity omega_;
  Covariance p_;
  Time time_since_pulse_;
  Time still_time_;
  uint32_t pulses_seen_{0U};
  bool pedaling_{false};

  std::array<Torque, kMaxPedalPulses> bins_{};
  std::size_t bin_index_{0U};
  std::size_t completed_bins_{0U};
  Torque bin_sum_;
  uint32_t bin_count_{0U};
  Torque mean_torque_;
  Torque torque_fast_;
  Torque zero_offset_;
};

}  // namespace unimoc::ebike
