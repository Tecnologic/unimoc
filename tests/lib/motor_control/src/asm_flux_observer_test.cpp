/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file asm_flux_observer_test.cpp
 *    @brief ztest coverage of AsmFluxObserver::Estimate() against a standing, magnetized induction motor.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "asm_flux_observer.hpp"
#include <zephyr/ztest.h>
#include "unimoc_test.hpp"

using namespace unimoc::unit;
using unimoc::observer::AsmFluxObserver;
using unimoc::system::Stator;

ZTEST_SUITE(asm_flux_observer, nullptr, nullptr, nullptr, nullptr, nullptr);

ZTEST(asm_flux_observer, test_a_dc_current_magnetizes_the_estimate_along_alpha) {
  AsmFluxObserver<float> observer{};
  // 2 A along alpha at 1 V (R_s 0.5 Ohm): the rotor flux settles at L_m i = 94 mWb, angle 0.
  for (int step = 0; step < 100000; ++step) {
    observer.Estimate(Stator<Voltage>{1.0_V, Voltage{}},
                      Stator<Current>{2.0_A, Current{}},
                      AngularVelocity{},
                      31.25_us);
  }
  zassert_true(IsFinite(observer.flux_magnitude));
  const MagneticFlux kExpected = 47.0_mH * 2.0_A;
  UNIMOC_EXPECT_NEAR(observer.flux_magnitude, kExpected, 2e-3_Wb);
  UNIMOC_EXPECT_NEAR(observer.flux_angle, Angle{}, 1e-3_rad);
}
