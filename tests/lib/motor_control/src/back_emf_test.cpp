/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file back_emf_test.cpp
 *    @brief ztest coverage of the back-EMF speed measurement and the coast-down meter (M3).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <zephyr/ztest.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include "back_emf_speed.hpp"
#include "coast_down_meter.hpp"
#include "stator_system.hpp"
#include "three_phase_system.hpp"
#include "unimoc_test.hpp"

using namespace unimoc::unit;
using unimoc::identification::CoastDownMeter;
using unimoc::observer::BackEmfSpeed;
using unimoc::system::Stator;
using unimoc::system::ThreePhase;

namespace {

constexpr float kPi = std::numbers::pi_v<float>;
constexpr MagneticFlux kFlux = 0.01_Wb;
constexpr float kSampleSeconds = 1.0F / 64000.0F;  // a control step at 64 kHz
constexpr Time kSample = 1.0_s * kSampleSeconds;
constexpr uint8_t kStepsPerCycle = 4U;

/**
 * @brief Phase back-EMF of a PMSM at electrical angle @p theta and speed @p omega_rad_s.
 *
 * e = omega psi (-sin theta, cos theta) in the stator frame, from the three
 * phases with an optional 5th harmonic and a common mode (both must not matter).
 */
Stator<Voltage> BackEmf(const float theta, const float omega_rad_s, const float fifth = 0.0F) {
  const Voltage kAmplitude = kFlux * (1.0_rad_per_s * omega_rad_s);
  const auto kPhase = [&](const float offset) {
    const float kAngle = theta - offset;
    return kAmplitude * (-std::sin(kAngle) - (fifth * std::sin(5.0F * kAngle))) + 7.0_V;
  };
  const ThreePhase<Voltage> kPhases{kPhase(0.0F), kPhase(2.0F * kPi / 3.0F), kPhase(-2.0F * kPi / 3.0F)};
  return kPhases.ToStator();
}

/// Feeds @p cycles control cycles of a motor at a constant speed; returns the last measurement.
BackEmfSpeed::Measurement RunSpeed(BackEmfSpeed& meter, const float omega_rad_s, const uint32_t cycles) {
  std::array<Stator<Voltage>, kStepsPerCycle> samples{};
  BackEmfSpeed::Measurement last{};
  float theta = 0.3F;
  for (uint32_t cycle = 0U; cycle < cycles; ++cycle) {
    for (auto& sample : samples) {
      theta += omega_rad_s * kSampleSeconds;
      sample = BackEmf(theta, omega_rad_s);
    }
    const auto kMeasurement = meter.Update(samples, kFlux);
    zassert_true(kMeasurement.has_value());
    last = *kMeasurement;
  }
  return last;
}

}  // namespace

ZTEST(back_emf_speed, test_measures_the_speed_and_its_sense) {
  BackEmfSpeed meter;
  const BackEmfSpeed::Measurement kForward = RunSpeed(meter, 500.0F, 40U);
  UNIMOC_EXPECT_NEAR(kForward.speed, 500.0_rad_per_s, 0.5_rad_per_s);

  meter.Reset();
  const BackEmfSpeed::Measurement kReverse = RunSpeed(meter, -800.0F, 40U);
  UNIMOC_EXPECT_NEAR(kReverse.speed, -800.0_rad_per_s, 0.8_rad_per_s);
  // The relative part of the noise: 5 % of the speed and the sensor noise.
  zexpect_true(kReverse.variance > kForward.variance);
}

ZTEST(back_emf_speed, test_follows_a_reversal) {
  BackEmfSpeed meter;
  (void)RunSpeed(meter, 600.0F, 20U);
  const BackEmfSpeed::Measurement kReversed = RunSpeed(meter, -600.0F, 20U);
  UNIMOC_EXPECT_NEAR(kReversed.speed, -600.0_rad_per_s, 0.6_rad_per_s);
}

ZTEST(back_emf_speed, test_below_the_band_the_motor_stands) {
  BackEmfSpeed meter;
  // 10 rad/s give 0.1 V, below the band of 0.3 V.
  const BackEmfSpeed::Measurement kSlow = RunSpeed(meter, 10.0F, 5U);
  zexpect_equal(kSlow.speed, AngularVelocity{});
  const AngularVelocity kBand = (meter.GetParameters().min_back_emf / kFlux) * 1.0_rad;
  UNIMOC_EXPECT_FLOAT_EQ(kSlow.variance, kBand * kBand);
}

ZTEST(back_emf_speed, test_needs_a_flux_and_samples) {
  BackEmfSpeed meter;
  const std::array<Stator<Voltage>, 1> kSamples{BackEmf(0.0F, 100.0F)};
  zexpect_false(meter.Update(kSamples, MagneticFlux{}).has_value());
  zexpect_false(meter.Update(std::span<const Stator<Voltage>>{}, kFlux).has_value());
}

namespace {

/// A coast from @p start_rad_s with a constant deceleration; returns the meter after @p seconds.
CoastDownMeter Coast(const float start_rad_s,
                     const float deceleration,
                     const float seconds,
                     const float fifth = 0.0F) {
  CoastDownMeter meter;
  meter.Start();
  float omega = start_rad_s;
  float theta = 1.0F;
  const auto kSamples = static_cast<uint32_t>(seconds / kSampleSeconds);
  for (uint32_t k = 0U; k < kSamples; ++k) {
    theta += omega * kSampleSeconds;
    const float kStep = deceleration * kSampleSeconds;
    omega = omega > 0.0F ? std::max(0.0F, omega - kStep) : std::min(0.0F, omega + kStep);
    meter.Add(BackEmf(theta, omega, fifth), kSample);
  }
  return meter;
}

}  // namespace

ZTEST(coast_down_meter, test_flux_of_a_decelerating_motor) {
  // 700 rad/s down to 50 rad/s in 2 s, with a 5 % 5th harmonic and a common mode.
  const CoastDownMeter kMeter = Coast(700.0F, 325.0F, 2.0F, 0.05F);
  const CoastDownMeter::Result kResult = kMeter.GetResult();
  zassert_true(kResult.periods > 50U, "periods %u", kResult.periods);
  UNIMOC_EXPECT_NEAR(kResult.flux, kFlux, kFlux * 0.003F);
  zexpect_true(kResult.spread < 0.01_ratio);
  UNIMOC_EXPECT_NEAR(kResult.start_speed, 700.0_rad_per_s, 5.0_rad_per_s);
  zexpect_true(kResult.last_speed < 100.0_rad_per_s);
  zexpect_false(kMeter.Standstill());
}

ZTEST(coast_down_meter, test_reverse_rotation_and_standstill) {
  // From -500 rad/s to standstill in 1 s, then 0.2 s at rest.
  const CoastDownMeter kMeter = Coast(-500.0F, 500.0F, 1.2F);
  const CoastDownMeter::Result kResult = kMeter.GetResult();
  zassert_true(kResult.periods > 10U);
  UNIMOC_EXPECT_NEAR(kResult.flux, kFlux, kFlux * 0.005F);
  zexpect_true(kResult.start_speed < -450.0_rad_per_s);
  zexpect_true(kMeter.Standstill(), "the back-EMF stays below its minimum");
}

ZTEST(coast_down_meter, test_a_reversal_is_not_counted) {
  // A load stops the shaft and drives it back: only the periods of the coast count.
  CoastDownMeter meter;
  meter.Start();
  float omega = 600.0F;
  float theta = 0.0F;
  for (uint32_t k = 0U; k < static_cast<uint32_t>(4.0F / kSampleSeconds); ++k) {
    theta += omega * kSampleSeconds;
    omega -= 600.0F * kSampleSeconds;  // through zero after 1 s, then -1800 rad/s after 4 s
    meter.Add(BackEmf(theta, omega), kSample);
  }
  const CoastDownMeter::Result kResult = meter.GetResult();
  zexpect_true(kResult.start_speed > 500.0_rad_per_s);
  zexpect_true(kResult.last_speed < -1500.0_rad_per_s, "the last period runs backwards");
  UNIMOC_EXPECT_NEAR(kResult.flux, kFlux, kFlux * 0.005F);
  zexpect_true(kResult.periods < 100U, "periods %u: the backward ones are not counted", kResult.periods);
}

ZTEST(coast_down_meter, test_no_periods_without_back_emf) {
  CoastDownMeter meter;
  meter.Start();
  for (uint32_t k = 0U; k < 2000U; ++k) {
    meter.Add(Stator<Voltage>{0.01_V, -0.02_V}, kSample);
  }
  zexpect_equal(meter.GetResult().periods, 0U);
  zexpect_equal(meter.GetResult().flux, MagneticFlux{});
  zexpect_true(meter.Standstill());
}

ZTEST_SUITE(back_emf_speed, nullptr, nullptr, nullptr, nullptr, nullptr);
ZTEST_SUITE(coast_down_meter, nullptr, nullptr, nullptr, nullptr, nullptr);
