/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file mechanical_observer_test.cpp
 *    @brief ztest coverage of the observer API (M4): covariance prediction,
 *           angle updates with their own noise, innovation wrap, flux error.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "mechanical_observer.hpp"
#include <zephyr/ztest.h>
#include <cmath>
#include <limits>
#include <numbers>
#include "angle_wrap.hpp"
#include "pmsm_flux_observer.hpp"
#include "unimoc_test.hpp"

namespace unimoc::observer::test {

using namespace unit;

namespace {

constexpr float kPi = std::numbers::pi_v<float>;
constexpr Time kDt{62.5e-6F};
const system::Rotor<Current> kNoCurrent{Current{}, Current{}};

MechanicalObserver<float> MakeObserver() {
  settings::NvmSettings settings{};
  settings.motor_j = 1e-3_kg_m2;
  settings.motor_omega_max = 10000.0_rad_per_s;
  settings.motor_omega_min = -10000.0_rad_per_s;
  settings.mech_obs_q = 1e-5F;
  settings.mech_obs_r = 1e-4_rad2;
  MechanicalObserver<float> observer;
  observer.Init(settings);
  return observer;
}

}  // namespace

ZTEST(mechanical_observer, test_inject_equals_prediction_plus_update) {
  MechanicalObserver<float> combined = MakeObserver();
  MechanicalObserver<float> split = MakeObserver();
  for (int i = 0; i < 50; ++i) {
    const Angle kError{0.01F * static_cast<float>(i % 7) - 0.03F};
    combined.InjectAngleError(kError, kDt);
    combined.Predict(kNoCurrent, kDt);
    split.PredictCovariance(kDt);
    split.UpdateAngle(kError, split.R);
    split.Predict(kNoCurrent, kDt);
  }
  UNIMOC_EXPECT_FLOAT_EQ(split.theta, combined.theta);
  UNIMOC_EXPECT_FLOAT_EQ(split.omega, combined.omega);
  UNIMOC_EXPECT_FLOAT_EQ(split.m_l, combined.m_l);
  UNIMOC_EXPECT_FLOAT_EQ(split.p[1][1], combined.p[1][1]);
}

ZTEST(mechanical_observer, test_innovation_is_wrapped) {
  MechanicalObserver<float> observer = MakeObserver();
  observer.Reset(3.1_rad);
  for (int i = 0; i < 20; ++i) {
    observer.PredictCovariance(kDt);
  }
  // Measured -3.1 rad is 0.083 rad ahead of 3.1 rad, not 6.2 rad behind.
  observer.UpdateAngle(-3.1_rad - observer.theta, observer.R);
  const Angle kStep = WrapAngle(observer.theta - 3.1_rad);
  zexpect_true(kStep > 0.0_rad && kStep < 0.09_rad, "step %f", static_cast<double>(kStep.Value()));
  zexpect_true(observer.theta <= kPi * 1.0_rad && observer.theta > -kPi * 1.0_rad);
}

ZTEST(mechanical_observer, test_measurement_noise_sets_the_weight) {
  MechanicalObserver<float> precise = MakeObserver();
  MechanicalObserver<float> noisy = MakeObserver();
  for (int i = 0; i < 20; ++i) {
    precise.PredictCovariance(kDt);
    noisy.PredictCovariance(kDt);
  }
  precise.UpdateAngle(0.1_rad, 1e-5_rad2);
  noisy.UpdateAngle(0.1_rad, 1e-1_rad2);
  zexpect_true(precise.theta > 10.0F * noisy.theta);
  zexpect_true(noisy.theta > 0.0_rad);
  // A second, independent measurement in the same cycle refines the estimate further.
  const Angle kAfterOne = precise.theta;
  precise.UpdateAngle(0.1_rad - precise.theta, 1e-5_rad2);
  zexpect_true(precise.theta > kAfterOne && precise.theta <= 0.1_rad + 1e-6_rad);
}

ZTEST(mechanical_observer, test_speed_noise_sets_the_weight) {
  MechanicalObserver<float> precise = MakeObserver();
  MechanicalObserver<float> noisy = MakeObserver();
  for (int i = 0; i < 20; ++i) {
    precise.PredictCovariance(kDt);
    noisy.PredictCovariance(kDt);
  }
  // The noise of a speed measurement is a variance of the speed.
  constexpr AngularVelocityVariance kPreciseNoise = 1e-3_rad_per_s * 1e-3_rad_per_s;
  constexpr AngularVelocityVariance kNoisyNoise = 1.0_rad_per_s * 1.0_rad_per_s;
  precise.UpdateSpeed(10.0_rad_per_s, kPreciseNoise);
  noisy.UpdateSpeed(10.0_rad_per_s, kNoisyNoise);
  zexpect_true(precise.omega > 10.0F * noisy.omega);
  zexpect_true(noisy.omega > 0.0_rad_per_s);
  zexpect_true(precise.omega <= 10.0_rad_per_s);
}

ZTEST(mechanical_observer, test_tracks_a_synthetic_speed_ramp) {
  // Angle measurements of a rotor that accelerates at 2000 rad/s^2 (electrical)
  // from 50 rad/s: the speed and the load-torque state follow the ramp.
  MechanicalObserver<float> observer = MakeObserver();
  constexpr AngularAcceleration kAcceleration = 2000.0_rad_per_s2;
  AngularVelocity speed = 50.0_rad_per_s;
  Angle angle;
  for (int i = 0; i < 8000; ++i) {  // 0.5 s
    observer.PredictCovariance(kDt);
    observer.UpdateAngle(WrapAngle(angle - observer.theta), 1e-4_rad2);
    observer.Predict(kNoCurrent, kDt);
    angle = WrapAngle(angle + (speed * kDt));
    speed += kAcceleration * kDt;
  }
  UNIMOC_EXPECT_NEAR(observer.omega, speed, 0.02F * speed);
  UNIMOC_EXPECT_NEAR(WrapAngle(observer.theta - angle), 0.0_rad, 0.02_rad);
}

ZTEST(mechanical_observer, test_pmsm_flux_error_is_an_angle_or_nothing) {
  settings::NvmSettings settings{};
  settings.stator_r = 0.05_Ohm;
  settings.l_d = 100e-6_H;
  settings.l_q = 100e-6_H;
  MechanicalObserver<float> observer = MakeObserver();
  PmsmFluxObserver<float> flux;
  flux.Init(settings);

  // No set flux (no permanent magnet configured): no angle information.
  const auto kNone = flux.Calculate(system::Rotor<Voltage>{Voltage{}, Voltage{}},
                                    kNoCurrent,
                                    system::Rotor<MagneticFlux>{MagneticFlux{}, MagneticFlux{}},
                                    kDt,
                                    observer);
  zexpect_false(kNone.has_value());

  // A flux vector that leads the estimate by 0.1 rad gives +0.1 rad, independent of its size.
  for (const float kFlux : {5e-3F, 50e-3F}) {
    flux.Reset();
    flux.psi_alpha = MagneticFlux{kFlux * std::cos(0.1F)};
    flux.psi_beta = MagneticFlux{kFlux * std::sin(0.1F)};
    const auto kError = flux.Calculate(system::Rotor<Voltage>{Voltage{}, Voltage{}},
                                       kNoCurrent,
                                       system::Rotor<MagneticFlux>{MagneticFlux{kFlux}, MagneticFlux{}},
                                       Time{},
                                       observer);
    zassert_true(kError.has_value());
    UNIMOC_EXPECT_NEAR(kError->Value(), 0.1F, 1e-3F);
  }
}

ZTEST(mechanical_observer, test_sin_cos_come_from_the_table_provider) {
  MechanicalObserver<float> obs = MakeObserver();
  constexpr Ratio kTolerance{1e-6F};

  obs.Reset(30.0_deg);
  UNIMOC_EXPECT_NEAR(obs.sin_theta, 0.5_ratio, kTolerance);
  UNIMOC_EXPECT_NEAR(obs.cos_theta, 0.8660254_ratio, kTolerance);

  obs.Reset(-135.0_deg);
  UNIMOC_EXPECT_NEAR(obs.sin_theta, -0.70710678_ratio, kTolerance);
  UNIMOC_EXPECT_NEAR(obs.cos_theta, -0.70710678_ratio, kTolerance);

  // After a prediction the pair belongs to the new angle estimate.
  obs.Reset(0.0_rad, 1000.0_rad_per_s);
  obs.Predict(kNoCurrent, kDt);
  const system::SinCos<Ratio> kExpected{obs.theta};
  UNIMOC_EXPECT_NEAR(obs.sin_theta, kExpected.sin, kTolerance);
  UNIMOC_EXPECT_NEAR(obs.cos_theta, kExpected.cos, kTolerance);
  UNIMOC_EXPECT_NEAR((obs.sin_theta * obs.sin_theta) + (obs.cos_theta * obs.cos_theta),
                     1.0_ratio,
                     kTolerance);
}

ZTEST(mechanical_observer, test_non_finite_angle_restarts_at_zero) {
  MechanicalObserver<float> obs = MakeObserver();
  obs.Reset(Angle{} * std::numeric_limits<float>::quiet_NaN());
  UNIMOC_EXPECT_FLOAT_EQ(obs.theta, 0.0_rad);
  UNIMOC_EXPECT_FLOAT_EQ(obs.sin_theta, 0.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(obs.cos_theta, 1.0_ratio);
}

}  // namespace unimoc::observer::test

ZTEST_SUITE(mechanical_observer, nullptr, nullptr, nullptr, nullptr, nullptr);
