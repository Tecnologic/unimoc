/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file stator_resistance_estimator_test.cpp
 *    @brief Tests for the online stator resistance estimate at low speed.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "stator_resistance_estimator.hpp"
#include <zephyr/ztest.h>
#include "unimoc_test.hpp"

namespace unimoc::control::test {

using namespace unit;

namespace {

constexpr Resistance kResistance = 0.15_Ohm;
constexpr Inductance kInductanceD = 0.3_mH;
constexpr Inductance kInductanceQ = 0.4_mH;
constexpr MagneticFlux kFlux = 23.0_mWb;
constexpr Time kCycle = 62.5_us;
constexpr uint32_t kWindow = 256U;

constexpr StatorResistanceEstimator::Parameters kParameters{.speed_max = 100.0_rad_per_s,
                                                            .current_min = 3.0_A,
                                                            .inductance_d = kInductanceD,
                                                            .inductance_q = kInductanceQ,
                                                            .cycle = kCycle,
                                                            .window_cycles = kWindow};

/// The voltage of the machine model for a current, its change over the last cycle and a speed.
StatorResistanceEstimator::Sample Machine(const system::Rotor<Current>& current,
                                          const system::Rotor<Current>& previous,
                                          const AngularVelocity speed) {
  const Voltage kInductiveD = (kInductanceD * (current.d - previous.d)) / kCycle;
  const Voltage kInductiveQ = (kInductanceQ * (current.q - previous.q)) / kCycle;
  return {.voltage = {(kResistance * current.d) - (speed * (kInductanceQ * current.q)) + kInductiveD,
                      (kResistance * current.q) + (speed * ((kInductanceD * current.d) + kFlux)) +
                          kInductiveQ},
          .current = current,
          .speed = speed,
          .flux = kFlux};
}

/// Feeds a constant operating point for one window; returns the result of its last cycle.
std::optional<Resistance> Window(StatorResistanceEstimator& estimator,
                                 const system::Rotor<Current>& current,
                                 const AngularVelocity speed) {
  std::optional<Resistance> result;
  for (uint32_t cycle = 0U; cycle < kWindow; ++cycle) {
    result = estimator.Add(Machine(current, current, speed));
    if (cycle + 1U < kWindow) {
      zexpect_false(result.has_value(), "no result before the window is full");
    }
  }
  return result;
}

}  // namespace

ZTEST(stator_resistance_estimator, test_standstill_gives_the_resistance) {
  StatorResistanceEstimator estimator{kParameters};
  const auto kResult = Window(estimator, {0.0_A, 10.0_A}, 0.0_rad_per_s);
  zassert_true(kResult.has_value());
  UNIMOC_EXPECT_NEAR(*kResult, kResistance, 1e-5_Ohm);
}

ZTEST(stator_resistance_estimator, test_speed_and_saliency_terms_are_removed) {
  StatorResistanceEstimator estimator{kParameters};
  // 80 rad/s: the back-EMF (1.84 V) is larger than the resistive voltage (1.5 V).
  const auto kForward = Window(estimator, {-4.0_A, 10.0_A}, 80.0_rad_per_s);
  zassert_true(kForward.has_value());
  UNIMOC_EXPECT_NEAR(*kForward, kResistance, 1e-4_Ohm);
  const auto kBraking = Window(estimator, {-4.0_A, -10.0_A}, -60.0_rad_per_s);
  zassert_true(kBraking.has_value());
  UNIMOC_EXPECT_NEAR(*kBraking, kResistance, 1e-4_Ohm);
}

ZTEST(stator_resistance_estimator, test_inductive_voltage_of_a_current_ramp_is_removed) {
  StatorResistanceEstimator estimator{kParameters};
  std::optional<Resistance> result;
  system::Rotor<Current> previous{0.0_A, 5.0_A};
  for (uint32_t cycle = 0U; cycle < kWindow; ++cycle) {
    // 5 A to 15 A over the window: L di/dt is about 0.25 V.
    const system::Rotor<Current> kCurrent{0.0_A, 5.0_A + (10.0_A * (static_cast<float>(cycle) / kWindow))};
    result = estimator.Add(Machine(kCurrent, previous, 20.0_rad_per_s));
    previous = kCurrent;
  }
  zassert_true(result.has_value());
  UNIMOC_EXPECT_NEAR(*result, kResistance, 2e-4_Ohm);
}

ZTEST(stator_resistance_estimator, test_high_speed_discards_the_window) {
  StatorResistanceEstimator estimator{kParameters};
  for (uint32_t cycle = 0U; cycle < kWindow - 1U; ++cycle) {
    zexpect_false(estimator.Add(Machine({0.0_A, 10.0_A}, {0.0_A, 10.0_A}, 0.0_rad_per_s)).has_value());
  }
  // One fast cycle: the nearly full window starts anew.
  zexpect_false(estimator.Add(Machine({0.0_A, 10.0_A}, {0.0_A, 10.0_A}, 150.0_rad_per_s)).has_value());
  const auto kResult = Window(estimator, {0.0_A, 10.0_A}, 0.0_rad_per_s);
  zexpect_true(kResult.has_value(), "a full window after the restart");
}

ZTEST(stator_resistance_estimator, test_small_current_gives_no_estimate) {
  StatorResistanceEstimator estimator{kParameters};
  zexpect_false(Window(estimator, {0.0_A, 2.0_A}, 0.0_rad_per_s).has_value());
  zexpect_false(Window(estimator, {1.5_A, -2.0_A}, 0.0_rad_per_s).has_value());
  // 3 A along d alone is enough.
  zexpect_true(Window(estimator, {3.0_A, 0.0_A}, 0.0_rad_per_s).has_value());
}

ZTEST(stator_resistance_estimator, test_an_inverter_offset_shows_as_a_small_error_at_high_current) {
  // A residual inverter error of 0.1 V along the current: at 30 A it adds
  // 3.3 mOhm (2.2 %, about 6 K), at 5 A 20 mOhm. The current threshold keeps it small.
  StatorResistanceEstimator estimator{kParameters};
  std::optional<Resistance> result;
  for (uint32_t cycle = 0U; cycle < kWindow; ++cycle) {
    StatorResistanceEstimator::Sample sample = Machine({0.0_A, 30.0_A}, {0.0_A, 30.0_A}, 0.0_rad_per_s);
    sample.voltage.q += 0.1_V;
    result = estimator.Add(sample);
  }
  zassert_true(result.has_value());
  UNIMOC_EXPECT_NEAR(*result, kResistance + (0.1_V / 30.0_A), 1e-5_Ohm);
}

ZTEST_SUITE(stator_resistance_estimator, nullptr, nullptr, nullptr, nullptr, nullptr);

}  // namespace unimoc::control::test
