/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file svm_test.cpp
 *    @brief ztest coverage of the space-vector modulator.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "svm.hpp"
#include <zephyr/ztest.h>
#include "unimoc_test.hpp"

using Stator = unimoc::system::Stator<unimoc::unit::Ratio>;
using Svm = unimoc::control::Svm;
using namespace unimoc::unit;

ZTEST(svm, test_zero_voltage_produces_centered_duties) {
  const Svm kSvm;

  const auto kDuties = kSvm.Calculate(Stator{0.0F, 0.0F}.ToThreePhase());

  UNIMOC_EXPECT_FLOAT_EQ(kDuties.a, 0.5_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(kDuties.b, 0.5_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(kDuties.c, 0.5_ratio);
}

ZTEST(svm, test_alpha_voltage_uses_centered_zero_sequence) {
  const Svm kSvm;

  const auto kDuties = kSvm.Calculate(Stator{0.2F, 0.0F}.ToThreePhase());

  UNIMOC_EXPECT_NEAR(kDuties.a, 0.65_ratio, 1.0e-6_ratio);
  UNIMOC_EXPECT_NEAR(kDuties.b, 0.35_ratio, 1.0e-6_ratio);
  UNIMOC_EXPECT_NEAR(kDuties.c, 0.35_ratio, 1.0e-6_ratio);
  UNIMOC_EXPECT_NEAR(kDuties.a + kDuties.c, 1.0_ratio, 1.0e-6_ratio);
}

ZTEST(svm, test_duties_are_clamped_to_configured_range) {
  unimoc::settings::NvmSettings settings;
  settings.svm_duty_min = 0.2_ratio;
  settings.svm_duty_max = 0.8_ratio;

  Svm svm;
  svm.Init(settings);

  const auto kDuties = svm.Calculate(Stator{1.0F, 0.0F}.ToThreePhase());

  UNIMOC_EXPECT_FLOAT_EQ(kDuties.a, 0.8_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(kDuties.b, 0.2_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(kDuties.c, 0.2_ratio);
}
ZTEST(svm, test_voltage_overload_normalises_by_dc_link_voltage) {
  const Svm kSvm;
  const unimoc::system::Stator<unimoc::unit::Voltage> kVoltage{10.0F, 0.0F};

  const auto kDuties = kSvm.CalculateWithDeadTimeCompensation(kVoltage, 50.0_V, Stator{0.0F, 0.0F});

  UNIMOC_EXPECT_NEAR(kDuties.a, 0.65_ratio, 1.0e-6_ratio);
  UNIMOC_EXPECT_NEAR(kDuties.b, 0.35_ratio, 1.0e-6_ratio);
  UNIMOC_EXPECT_NEAR(kDuties.c, 0.35_ratio, 1.0e-6_ratio);
}

ZTEST(svm, test_voltage_overload_adds_dead_time_compensation) {
  const Svm kSvm;
  const unimoc::system::Stator<unimoc::unit::Voltage> kVoltage{0.0F, 0.0F};

  const auto kDuties = kSvm.CalculateWithDeadTimeCompensation(kVoltage, 50.0_V, Stator{0.1F, 0.0F});

  UNIMOC_EXPECT_NEAR(kDuties.a, 0.575_ratio, 1.0e-6_ratio);
  UNIMOC_EXPECT_NEAR(kDuties.b, 0.425_ratio, 1.0e-6_ratio);
  UNIMOC_EXPECT_NEAR(kDuties.c, 0.425_ratio, 1.0e-6_ratio);
}

ZTEST(svm, test_voltage_overload_guards_against_zero_dc_link_voltage) {
  const Svm kSvm;
  const unimoc::system::Stator<unimoc::unit::Voltage> kVoltage{0.2F, 0.0F};

  const auto kDuties = kSvm.CalculateWithDeadTimeCompensation(kVoltage, 0.0_V, Stator{0.0F, 0.0F});

  UNIMOC_EXPECT_NEAR(kDuties.a, 0.65_ratio, 1.0e-6_ratio);
}

ZTEST(svm, test_precomputed_inverse_matches_voltage_overload) {
  const Svm kSvm;
  const unimoc::system::Stator<unimoc::unit::Voltage> kVoltage{7.0F, -3.0F};
  const Stator kDeadTime{0.01F, -0.02F};
  const auto kInverse = Svm::InverseDcLinkVoltage(48.0_V);

  const auto kDirect = kSvm.CalculateWithDeadTimeCompensation(kVoltage, 48.0_V, kDeadTime);
  const auto kPrecomputed = kSvm.CalculateWithInverseDcLinkVoltage(kVoltage, kInverse, kDeadTime);

  UNIMOC_EXPECT_FLOAT_EQ(kInverse, 1.0F / 48.0_V);
  UNIMOC_EXPECT_FLOAT_EQ(kPrecomputed.a, kDirect.a);
  UNIMOC_EXPECT_FLOAT_EQ(kPrecomputed.b, kDirect.b);
  UNIMOC_EXPECT_FLOAT_EQ(kPrecomputed.c, kDirect.c);
}

ZTEST(svm, test_inverse_dc_link_voltage_guards_against_zero) {
  UNIMOC_EXPECT_FLOAT_EQ(Svm::InverseDcLinkVoltage(0.0_V), 1.0F / 1.0_V);
}

ZTEST_SUITE(svm, nullptr, nullptr, nullptr, nullptr, nullptr);
