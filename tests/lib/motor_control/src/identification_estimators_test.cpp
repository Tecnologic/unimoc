/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file identification_estimators_test.cpp
 *    @brief ztest coverage of the estimators of the motor identification.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <zephyr/ztest.h>
#include <array>
#include <cstdint>
#include "impedance_estimator.hpp"
#include "resistance_regression.hpp"
#include "sin_cos.hpp"
#include "unimoc_test.hpp"

using namespace unimoc::unit;
using unimoc::identification::EstimateImpedance;
using unimoc::identification::PhasorAccumulator;
using unimoc::identification::ResistanceRegression;
using unimoc::system::PortableSinCosProvider;

namespace {

constexpr Resistance kResistance = 0.12_Ohm;
constexpr Voltage kDeadTimeVoltage = 0.35_V;

/// The voltage demand of a winding with dead time: u = R i + U_dt sign(i), plus a disturbance.
Voltage Demand(const Current current, const Voltage disturbance = Voltage{}) {
  const Voltage kDeadTime = current > Current{} ? kDeadTimeVoltage : -kDeadTimeVoltage;
  return (current * kResistance) + kDeadTime + disturbance;
}

}  // namespace

ZTEST(resistance_regression, test_separates_the_slope_from_the_dead_time_step) {
  ResistanceRegression regression;
  for (const float kLevel : {1.0F, 2.0F, 3.0F, 4.0F, 5.0F}) {
    regression.Add(1.0_A * kLevel, Demand(1.0_A * kLevel));
    regression.Add(-1.0_A * kLevel, Demand(-1.0_A * kLevel));
  }

  const auto kFit = regression.Solve();

  zassert_true(kFit.has_value());
  zexpect_equal(regression.Count(), 10U);
  UNIMOC_EXPECT_NEAR(kFit->resistance, kResistance, 1.0e-5_Ohm);
  UNIMOC_EXPECT_NEAR(kFit->dead_time_voltage, kDeadTimeVoltage, 1.0e-5_V);
}

ZTEST(resistance_regression, test_tolerates_a_small_disturbance) {
  ResistanceRegression regression;
  const std::array<float, 10>
      kNoise{0.01F, -0.02F, 0.015F, -0.005F, 0.0F, 0.02F, -0.01F, 0.005F, -0.015F, 0.01F};
  for (std::size_t k = 0; k < kNoise.size(); ++k) {
    const Current kCurrent = 1.0_A * (static_cast<float>(k % 5U) + 1.0F) * (k < 5U ? 1.0F : -1.0F);
    regression.Add(kCurrent, Demand(kCurrent, 1.0_V * kNoise[k]));
  }

  const auto kFit = regression.Solve();

  zassert_true(kFit.has_value());
  UNIMOC_EXPECT_NEAR(kFit->resistance, kResistance, 0.01_Ohm);
  UNIMOC_EXPECT_NEAR(kFit->dead_time_voltage, kDeadTimeVoltage, 0.02_V);
}

ZTEST(resistance_regression, test_needs_both_signs_and_two_magnitudes) {
  ResistanceRegression one_sign;
  one_sign.Add(1.0_A, Demand(1.0_A));
  one_sign.Add(2.0_A, Demand(2.0_A));
  zexpect_false(one_sign.Solve().has_value());

  ResistanceRegression one_magnitude;
  one_magnitude.Add(2.0_A, Demand(2.0_A));
  one_magnitude.Add(-2.0_A, Demand(-2.0_A));
  zexpect_false(one_magnitude.Solve().has_value());

  ResistanceRegression zero_current;
  zero_current.Add(Current{}, 1.0_V);
  zexpect_equal(zero_current.Count(), 0U);
}

ZTEST(impedance_estimator, test_phasor_of_a_cosine_with_offset) {
  PhasorAccumulator<Current> phasor;
  constexpr uint32_t kSamples = 128U;
  constexpr uint32_t kStep = 0x1'0000'0000ULL / kSamples;
  for (uint32_t sample = 0; sample < 4U * kSamples; ++sample) {
    const auto kPhase = PortableSinCosProvider::Calculate(static_cast<int32_t>(sample * kStep));
    // 2 A cos(phi + 30 deg) = 2 A (cos(phi) cos 30 - sin(phi) sin 30), on a 5 A bias.
    const Current kValue = 5.0_A + 2.0_A * ((kPhase.cos * 0.8660254F) - (kPhase.sin * 0.5F));
    phasor.Add(kValue, kPhase);
  }

  UNIMOC_EXPECT_NEAR(phasor.Real(), 1.7320508_A, 1.0e-3_A);
  UNIMOC_EXPECT_NEAR(phasor.Imaginary(), 1.0_A, 1.0e-3_A);
  zexpect_equal(phasor.Count(), 4U * kSamples);
}

ZTEST(impedance_estimator, test_winding_impedance_from_the_phasors) {
  // i = I cos(phi); u = R i + L di/dt = I (R cos(phi) - w L sin(phi)).
  constexpr Inductance kInductance = 0.8_mH;
  constexpr AngularVelocity kOmega = 3141.5927_rad_per_s;  // 500 Hz
  const Resistance kReactance = (kOmega / 1.0_rad) * kInductance;
  constexpr Current kAmplitude = 3.0_A;

  PhasorAccumulator<Voltage> voltage;
  PhasorAccumulator<Current> current;
  constexpr uint32_t kSamples = 128U;
  constexpr uint32_t kStep = 0x1'0000'0000ULL / kSamples;
  for (uint32_t sample = 0; sample < 10U * kSamples; ++sample) {
    const auto kPhase = PortableSinCosProvider::Calculate(static_cast<int32_t>(sample * kStep));
    current.Add(kAmplitude * kPhase.cos, kPhase);
    voltage.Add(kAmplitude * ((kResistance * kPhase.cos) - (kReactance * kPhase.sin)), kPhase);
  }

  const auto kImpedance = EstimateImpedance(voltage, current, kOmega);

  zassert_true(kImpedance.has_value());
  UNIMOC_EXPECT_NEAR(kImpedance->resistance, kResistance, 2.0e-3_Ohm);
  UNIMOC_EXPECT_NEAR(kImpedance->inductance, kInductance, 2.0e-6_H);
}

ZTEST(impedance_estimator, test_no_impedance_without_current) {
  const PhasorAccumulator<Voltage> kVoltage;
  const PhasorAccumulator<Current> kCurrent;
  zexpect_false(EstimateImpedance(kVoltage, kCurrent, 100.0_rad_per_s).has_value());
}

ZTEST_SUITE(resistance_regression, nullptr, nullptr, nullptr, nullptr, nullptr);
ZTEST_SUITE(impedance_estimator, nullptr, nullptr, nullptr, nullptr, nullptr);
