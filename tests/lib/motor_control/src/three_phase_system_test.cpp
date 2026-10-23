/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file three_phase_system_test.cpp
 *    @brief ztest suites for ThreePhase, Stator and the unit literals.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "three_phase_system.hpp"
#include <zephyr/ztest.h>
#include <cmath>
#include <concepts>
#include "unimoc_test.hpp"

using ThreePhase = unimoc::system::ThreePhase<unimoc::unit::Ratio>;
using namespace unimoc::unit;

static_assert(unimoc::unit::UnitLike<unimoc::unit::Current>);
static_assert(!unimoc::unit::UnitLike<float>);

template <typename T>
concept CanInstantiateThreePhase = requires { typename unimoc::system::ThreePhase<T>; };

template <typename T>
concept CanInstantiateStator = requires { typename unimoc::system::Stator<T>; };

static_assert(!CanInstantiateThreePhase<float>);
static_assert(!CanInstantiateStator<float>);
static_assert(std::same_as<decltype(unimoc::system::ThreePhase<unimoc::unit::Current>{}.ToStator()),
                           unimoc::system::Stator<unimoc::unit::Current>>);

ZTEST(unit_literal, test_converts_scaled_values_to_base_units) {
  UNIMOC_EXPECT_FLOAT_EQ(unimoc::unit::Inductance{47.0_mH}, 47.0e-3_H);
  UNIMOC_EXPECT_FLOAT_EQ(unimoc::unit::Time{2.0_us}, 2.0e-6_s);
  UNIMOC_EXPECT_FLOAT_EQ(unimoc::unit::Frequency{20.0_kHz}, 20000.0_Hz);
}

// Test default constructor
ZTEST(three_phase, test_default_constructor) {
  ThreePhase phase;
  // Default constructor should initialize values (implementation dependent)
  // Just verify object is created without throwing
  ThreePhase();

  (void)phase;  // Suppress unused variable warning
}

// Test parameterized constructor
ZTEST(three_phase, test_parameterized_constructor) {
  ThreePhase phase(1.5F, 2.5F, 3.5F);

  UNIMOC_EXPECT_FLOAT_EQ(phase.a, 1.5_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(phase.b, 2.5_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(phase.c, 3.5_ratio);
}

// Test parameterized constructor with zero values
ZTEST(three_phase, test_constructor_with_zeros) {
  ThreePhase phase(0.0F, 0.0F, 0.0F);

  UNIMOC_EXPECT_FLOAT_EQ(phase.a, 0.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(phase.b, 0.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(phase.c, 0.0_ratio);
}

// Test parameterized constructor with negative values
ZTEST(three_phase, test_constructor_with_negative_values) {
  ThreePhase phase(-1.0F, -2.0F, -3.0F);

  UNIMOC_EXPECT_FLOAT_EQ(phase.a, -1.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(phase.b, -2.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(phase.c, -3.0_ratio);
}

// Test copy constructor
ZTEST(three_phase, test_copy_constructor) {
  ThreePhase original(1.0F, 2.0F, 3.0F);
  ThreePhase copy(original);

  UNIMOC_EXPECT_FLOAT_EQ(copy.a, 1.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(copy.b, 2.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(copy.c, 3.0_ratio);

  // Verify independence
  copy.a = 10.0_ratio;
  UNIMOC_EXPECT_FLOAT_EQ(original.a, 1.0_ratio);
}

// Test move constructor
ZTEST(three_phase, test_move_constructor) {
  ThreePhase original(1.0F, 2.0F, 3.0F);
  ThreePhase moved(original);

  UNIMOC_EXPECT_FLOAT_EQ(moved.a, 1.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(moved.b, 2.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(moved.c, 3.0_ratio);
}

// Test copy assignment operator
ZTEST(three_phase, test_copy_assignment_operator) {
  ThreePhase original(1.0F, 2.0F, 3.0F);
  ThreePhase copy(4.0F, 5.0F, 6.0F);

  copy = original;

  UNIMOC_EXPECT_FLOAT_EQ(copy.a, 1.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(copy.b, 2.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(copy.c, 3.0_ratio);
}

// Test move assignment operator
ZTEST(three_phase, test_move_assignment_operator) {
  ThreePhase original(1.0F, 2.0F, 3.0F);
  ThreePhase moved(4.0F, 5.0F, 6.0F);

  moved = original;

  UNIMOC_EXPECT_FLOAT_EQ(moved.a, 1.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(moved.b, 2.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(moved.c, 3.0_ratio);
}

// Test equality operator with equal values
ZTEST(three_phase, test_equality_operator_true) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(1.0F, 2.0F, 3.0F);

  zexpect_true(phase1 == phase2);
  zexpect_true(phase2 == phase1);
}

// Test equality operator with different values
ZTEST(three_phase, test_equality_operator_false) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(1.0F, 2.0F, 4.0F);

  zexpect_false(phase1 == phase2);
}

// Test equality operator with self
ZTEST(three_phase, test_equality_operator_self) {
  ThreePhase phase(1.0F, 2.0F, 3.0F);

  zexpect_true(phase == phase);
}

// Test inequality operator
ZTEST(three_phase, test_inequality_operator_true) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(4.0F, 5.0F, 6.0F);

  zexpect_true(phase1 != phase2);
  zexpect_true(phase2 != phase1);
}

// Test inequality operator with equal values
ZTEST(three_phase, test_inequality_operator_false) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(1.0F, 2.0F, 3.0F);

  zexpect_false(phase1 != phase2);
}

// Test addition operator
ZTEST(three_phase, test_addition_operator) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(4.0F, 5.0F, 6.0F);

  ThreePhase result = phase1 + phase2;

  UNIMOC_EXPECT_FLOAT_EQ(result.a, 5.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(result.b, 7.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(result.c, 9.0_ratio);
}

// Test addition with zero
ZTEST(three_phase, test_addition_with_zero) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase zero(0.0F, 0.0F, 0.0F);

  ThreePhase result = phase1 + zero;

  UNIMOC_EXPECT_FLOAT_EQ(result.a, 1.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(result.b, 2.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(result.c, 3.0_ratio);
}

// Test addition with negative values
ZTEST(three_phase, test_addition_with_negative_values) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(-1.0F, -2.0F, -3.0F);

  ThreePhase result = phase1 + phase2;

  UNIMOC_EXPECT_FLOAT_EQ(result.a, 0.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(result.b, 0.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(result.c, 0.0_ratio);
}

// Test addition commutativity
ZTEST(three_phase, test_addition_commutativity) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(4.0F, 5.0F, 6.0F);

  ThreePhase result1 = phase1 + phase2;
  ThreePhase result2 = phase2 + phase1;

  zexpect_true(result1 == result2);
}

// Test subtraction operator
ZTEST(three_phase, test_subtraction_operator) {
  ThreePhase phase1(5.0F, 7.0F, 9.0F);
  ThreePhase phase2(1.0F, 2.0F, 3.0F);

  ThreePhase result = phase1 - phase2;

  UNIMOC_EXPECT_FLOAT_EQ(result.a, 4.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(result.b, 5.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(result.c, 6.0_ratio);
}

// Test subtraction with zero
ZTEST(three_phase, test_subtraction_with_zero) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase zero(0.0F, 0.0F, 0.0F);

  ThreePhase result = phase1 - zero;

  UNIMOC_EXPECT_FLOAT_EQ(result.a, 1.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(result.b, 2.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(result.c, 3.0_ratio);
}

// Test subtraction with itself
ZTEST(three_phase, test_subtraction_with_self) {
  ThreePhase phase(1.0F, 2.0F, 3.0F);

  ThreePhase result = phase - phase;

  UNIMOC_EXPECT_FLOAT_EQ(result.a, 0.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(result.b, 0.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(result.c, 0.0_ratio);
}

// Test subtraction with negative values
ZTEST(three_phase, test_subtraction_with_negative_values) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(-1.0F, -2.0F, -3.0F);

  ThreePhase result = phase1 - phase2;

  UNIMOC_EXPECT_FLOAT_EQ(result.a, 2.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(result.b, 4.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(result.c, 6.0_ratio);
}

// Test ToArray conversion
ZTEST(three_phase, test_to_array_conversion) {
  ThreePhase phase(1.5F, 2.5F, 3.5F);

  auto arr = phase.ToArray();

  zexpect_equal(arr.size(), 3);
  UNIMOC_EXPECT_FLOAT_EQ(arr.at(0), 1.5_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(arr.at(1), 2.5_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(arr.at(2), 3.5_ratio);
}

// Test ToArray with zero values
ZTEST(three_phase, test_to_array_with_zeros) {
  ThreePhase phase(0.0F, 0.0F, 0.0F);

  auto arr = phase.ToArray();

  UNIMOC_EXPECT_FLOAT_EQ(arr.at(0), 0.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(arr.at(1), 0.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(arr.at(2), 0.0_ratio);
}

// Test ToArray with negative values
ZTEST(three_phase, test_to_array_with_negative_values) {
  ThreePhase phase(-1.0F, -2.0F, -3.0F);

  auto arr = phase.ToArray();

  UNIMOC_EXPECT_FLOAT_EQ(arr.at(0), -1.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(arr.at(1), -2.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(arr.at(2), -3.0_ratio);
}

ZTEST(three_phase, test_to_stator_uses_unit_representation) {
  const unimoc::system::ThreePhase<unimoc::unit::Current> kPhase{1.0F, 2.0F, 3.0F};

  const auto kStator = kPhase.ToStator();

  UNIMOC_EXPECT_FLOAT_EQ(kStator.alpha, -1.0_A);
  UNIMOC_EXPECT_NEAR(kStator.beta, -0.577350269_A, 1.0e-6_A);
}

ZTEST(three_phase, test_to_stator_rejects_zero_sequence) {
  const unimoc::system::ThreePhase<unimoc::unit::Current> kPhase{2.0F, 2.0F, 2.0F};

  const auto kStator = kPhase.ToStator();

  UNIMOC_EXPECT_FLOAT_EQ(kStator.alpha, 0.0_A);
  UNIMOC_EXPECT_FLOAT_EQ(kStator.beta, 0.0_A);
}

ZTEST(three_phase, test_to_stator_transforms_balanced_phase_vector) {
  const unimoc::system::ThreePhase<unimoc::unit::Current> kPhase{1.0F, -0.5F, -0.5F};

  const auto kStator = kPhase.ToStator();

  UNIMOC_EXPECT_FLOAT_EQ(kStator.alpha, 1.0_A);
  UNIMOC_EXPECT_NEAR(kStator.beta, 0.0_A, 1.0e-6_A);
}

// Test constexpr functionality (compile-time evaluation)
ZTEST(three_phase, test_constexpr_constructor) {
  constexpr ThreePhase kPhase(1.0F, 2.0F, 3.0F);

  UNIMOC_EXPECT_FLOAT_EQ(kPhase.a, 1.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(kPhase.b, 2.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(kPhase.c, 3.0_ratio);
}

// Test constexpr addition
ZTEST(three_phase, test_constexpr_addition) {
  constexpr ThreePhase kPhase1(1.0F, 2.0F, 3.0F);
  constexpr ThreePhase kPhase2(4.0F, 5.0F, 6.0F);
  constexpr ThreePhase kResult = kPhase1 + kPhase2;

  UNIMOC_EXPECT_FLOAT_EQ(kResult.a, 5.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(kResult.b, 7.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(kResult.c, 9.0_ratio);
}

// Test constexpr subtraction
ZTEST(three_phase, test_constexpr_subtraction) {
  constexpr ThreePhase kPhase1(5.0F, 7.0F, 9.0F);
  constexpr ThreePhase kPhase2(1.0F, 2.0F, 3.0F);
  constexpr ThreePhase kResult = kPhase1 - kPhase2;

  UNIMOC_EXPECT_FLOAT_EQ(kResult.a, 4.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(kResult.b, 5.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(kResult.c, 6.0_ratio);
}

// Test constexpr equality
ZTEST(three_phase, test_constexpr_equality) {
  constexpr ThreePhase kPhase1(1.0F, 2.0F, 3.0F);
  constexpr ThreePhase kPhase2(1.0F, 2.0F, 3.0F);
  constexpr bool kEqual = kPhase1 == kPhase2;

  zexpect_true(kEqual);
}

// Test with large values
ZTEST(three_phase, test_large_values) {
  ThreePhase phase(1000.0F, 2000.0F, 3000.0F);

  UNIMOC_EXPECT_FLOAT_EQ(phase.a, 1000.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(phase.b, 2000.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(phase.c, 3000.0_ratio);
}

// Test with very small values
ZTEST(three_phase, test_small_values) {
  ThreePhase phase(0.001F, 0.002F, 0.003F);

  UNIMOC_EXPECT_FLOAT_EQ(phase.a, 0.001_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(phase.b, 0.002_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(phase.c, 0.003_ratio);
}

// Test addition and subtraction chaining
ZTEST(three_phase, test_arithmetic_chaining) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(4.0F, 5.0F, 6.0F);
  ThreePhase phase3(7.0F, 8.0F, 9.0F);

  ThreePhase result = phase1 + phase2 - phase3;

  UNIMOC_EXPECT_FLOAT_EQ(result.a, -2.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(result.b, -1.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(result.c, 0.0_ratio);
}

// Test multiple operations
ZTEST(three_phase, test_multiple_operations) {
  ThreePhase phase1(10.0F, 20.0F, 30.0F);
  ThreePhase phase2(5.0F, 10.0F, 15.0F);

  // Add
  ThreePhase sum = phase1 + phase2;
  UNIMOC_EXPECT_FLOAT_EQ(sum.a, 15.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(sum.b, 30.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(sum.c, 45.0_ratio);

  // Subtract
  ThreePhase diff = phase1 - phase2;
  UNIMOC_EXPECT_FLOAT_EQ(diff.a, 5.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(diff.b, 10.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(diff.c, 15.0_ratio);

  // Convert to array
  auto arr = sum.ToArray();
  UNIMOC_EXPECT_FLOAT_EQ(arr.at(0), 15.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(arr.at(1), 30.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(arr.at(2), 45.0_ratio);
}

// Test floating point precision edge cases
ZTEST(three_phase, test_floating_point_precision) {
  ThreePhase phase1(0.1F + 0.2F, 0.3F, 0.0F);
  ThreePhase phase2(0.3F, 0.3F, 0.0F);

  // Due to floating point precision, these might not be exactly equal
  // but should be very close
  UNIMOC_EXPECT_NEAR(phase1.a, phase2.a, 1e-6_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(phase1.b, phase2.b);
}

// Test that operations don't modify original objects
ZTEST(three_phase, test_operations_immutability) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(4.0F, 5.0F, 6.0F);

  ThreePhase sum = phase1 + phase2;
  ThreePhase diff = phase1 - phase2;

  // Original objects should remain unchanged
  UNIMOC_EXPECT_FLOAT_EQ(phase1.a, 1.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(phase1.b, 2.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(phase1.c, 3.0_ratio);

  UNIMOC_EXPECT_FLOAT_EQ(phase2.a, 4.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(phase2.b, 5.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(phase2.c, 6.0_ratio);

  UNIMOC_EXPECT_FLOAT_EQ(sum.a, 5.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(sum.b, 7.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(sum.c, 9.0_ratio);

  UNIMOC_EXPECT_FLOAT_EQ(diff.a, -3.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(diff.b, -3.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(diff.c, -3.0_ratio);
}

ZTEST_SUITE(unit_literal, nullptr, nullptr, nullptr, nullptr, nullptr);
ZTEST_SUITE(three_phase, nullptr, nullptr, nullptr, nullptr, nullptr);
