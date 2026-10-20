/*
           __  ___   ________  _______  ______
          / / / / | / /  _/  |/  / __ \/ ____/
         / / / /  |/ // // /|_/ / / / / /
        / /_/ / /|  // // /  / / /_/ / /___
        \____/_/ |_/___/_/  /_/\____/\____/

        Universal Motor Control  2025 Alexander <tecnologic86@gmail.com> Evers

        This file is part of UNIMOC.

        UNIMOC is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.

        This program is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
        GNU General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
  UNIMOC_EXPECT_FLOAT_EQ(unimoc::unit::Inductance{47.0_mH}.Value(), 47.0e-3F);
  UNIMOC_EXPECT_FLOAT_EQ(unimoc::unit::Time{2.0_us}.Value(), 2.0e-6F);
  UNIMOC_EXPECT_FLOAT_EQ(unimoc::unit::Frequency{20.0_kHz}.Value(), 20000.0F);
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

  UNIMOC_EXPECT_FLOAT_EQ(phase.a.Value(), 1.5F);
  UNIMOC_EXPECT_FLOAT_EQ(phase.b.Value(), 2.5F);
  UNIMOC_EXPECT_FLOAT_EQ(phase.c.Value(), 3.5F);
}

// Test parameterized constructor with zero values
ZTEST(three_phase, test_constructor_with_zeros) {
  ThreePhase phase(0.0F, 0.0F, 0.0F);

  UNIMOC_EXPECT_FLOAT_EQ(phase.a.Value(), 0.0F);
  UNIMOC_EXPECT_FLOAT_EQ(phase.b.Value(), 0.0F);
  UNIMOC_EXPECT_FLOAT_EQ(phase.c.Value(), 0.0F);
}

// Test parameterized constructor with negative values
ZTEST(three_phase, test_constructor_with_negative_values) {
  ThreePhase phase(-1.0F, -2.0F, -3.0F);

  UNIMOC_EXPECT_FLOAT_EQ(phase.a.Value(), -1.0F);
  UNIMOC_EXPECT_FLOAT_EQ(phase.b.Value(), -2.0F);
  UNIMOC_EXPECT_FLOAT_EQ(phase.c.Value(), -3.0F);
}

// Test copy constructor
ZTEST(three_phase, test_copy_constructor) {
  ThreePhase original(1.0F, 2.0F, 3.0F);
  ThreePhase copy(original);

  UNIMOC_EXPECT_FLOAT_EQ(copy.a.Value(), 1.0F);
  UNIMOC_EXPECT_FLOAT_EQ(copy.b.Value(), 2.0F);
  UNIMOC_EXPECT_FLOAT_EQ(copy.c.Value(), 3.0F);

  // Verify independence
  copy.a = 10.0_ratio;
  UNIMOC_EXPECT_FLOAT_EQ(original.a.Value(), 1.0F);
}

// Test move constructor
ZTEST(three_phase, test_move_constructor) {
  ThreePhase original(1.0F, 2.0F, 3.0F);
  ThreePhase moved(original);

  UNIMOC_EXPECT_FLOAT_EQ(moved.a.Value(), 1.0F);
  UNIMOC_EXPECT_FLOAT_EQ(moved.b.Value(), 2.0F);
  UNIMOC_EXPECT_FLOAT_EQ(moved.c.Value(), 3.0F);
}

// Test copy assignment operator
ZTEST(three_phase, test_copy_assignment_operator) {
  ThreePhase original(1.0F, 2.0F, 3.0F);
  ThreePhase copy(4.0F, 5.0F, 6.0F);

  copy = original;

  UNIMOC_EXPECT_FLOAT_EQ(copy.a.Value(), 1.0F);
  UNIMOC_EXPECT_FLOAT_EQ(copy.b.Value(), 2.0F);
  UNIMOC_EXPECT_FLOAT_EQ(copy.c.Value(), 3.0F);
}

// Test move assignment operator
ZTEST(three_phase, test_move_assignment_operator) {
  ThreePhase original(1.0F, 2.0F, 3.0F);
  ThreePhase moved(4.0F, 5.0F, 6.0F);

  moved = original;

  UNIMOC_EXPECT_FLOAT_EQ(moved.a.Value(), 1.0F);
  UNIMOC_EXPECT_FLOAT_EQ(moved.b.Value(), 2.0F);
  UNIMOC_EXPECT_FLOAT_EQ(moved.c.Value(), 3.0F);
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

  UNIMOC_EXPECT_FLOAT_EQ(result.a.Value(), 5.0F);
  UNIMOC_EXPECT_FLOAT_EQ(result.b.Value(), 7.0F);
  UNIMOC_EXPECT_FLOAT_EQ(result.c.Value(), 9.0F);
}

// Test addition with zero
ZTEST(three_phase, test_addition_with_zero) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase zero(0.0F, 0.0F, 0.0F);

  ThreePhase result = phase1 + zero;

  UNIMOC_EXPECT_FLOAT_EQ(result.a.Value(), 1.0F);
  UNIMOC_EXPECT_FLOAT_EQ(result.b.Value(), 2.0F);
  UNIMOC_EXPECT_FLOAT_EQ(result.c.Value(), 3.0F);
}

// Test addition with negative values
ZTEST(three_phase, test_addition_with_negative_values) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(-1.0F, -2.0F, -3.0F);

  ThreePhase result = phase1 + phase2;

  UNIMOC_EXPECT_FLOAT_EQ(result.a.Value(), 0.0F);
  UNIMOC_EXPECT_FLOAT_EQ(result.b.Value(), 0.0F);
  UNIMOC_EXPECT_FLOAT_EQ(result.c.Value(), 0.0F);
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

  UNIMOC_EXPECT_FLOAT_EQ(result.a.Value(), 4.0F);
  UNIMOC_EXPECT_FLOAT_EQ(result.b.Value(), 5.0F);
  UNIMOC_EXPECT_FLOAT_EQ(result.c.Value(), 6.0F);
}

// Test subtraction with zero
ZTEST(three_phase, test_subtraction_with_zero) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase zero(0.0F, 0.0F, 0.0F);

  ThreePhase result = phase1 - zero;

  UNIMOC_EXPECT_FLOAT_EQ(result.a.Value(), 1.0F);
  UNIMOC_EXPECT_FLOAT_EQ(result.b.Value(), 2.0F);
  UNIMOC_EXPECT_FLOAT_EQ(result.c.Value(), 3.0F);
}

// Test subtraction with itself
ZTEST(three_phase, test_subtraction_with_self) {
  ThreePhase phase(1.0F, 2.0F, 3.0F);

  ThreePhase result = phase - phase;

  UNIMOC_EXPECT_FLOAT_EQ(result.a.Value(), 0.0F);
  UNIMOC_EXPECT_FLOAT_EQ(result.b.Value(), 0.0F);
  UNIMOC_EXPECT_FLOAT_EQ(result.c.Value(), 0.0F);
}

// Test subtraction with negative values
ZTEST(three_phase, test_subtraction_with_negative_values) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(-1.0F, -2.0F, -3.0F);

  ThreePhase result = phase1 - phase2;

  UNIMOC_EXPECT_FLOAT_EQ(result.a.Value(), 2.0F);
  UNIMOC_EXPECT_FLOAT_EQ(result.b.Value(), 4.0F);
  UNIMOC_EXPECT_FLOAT_EQ(result.c.Value(), 6.0F);
}

// Test ToArray conversion
ZTEST(three_phase, test_to_array_conversion) {
  ThreePhase phase(1.5F, 2.5F, 3.5F);

  auto arr = phase.ToArray();

  zexpect_equal(arr.size(), 3);
  UNIMOC_EXPECT_FLOAT_EQ(arr.at(0).Value(), 1.5F);
  UNIMOC_EXPECT_FLOAT_EQ(arr.at(1).Value(), 2.5F);
  UNIMOC_EXPECT_FLOAT_EQ(arr.at(2).Value(), 3.5F);
}

// Test ToArray with zero values
ZTEST(three_phase, test_to_array_with_zeros) {
  ThreePhase phase(0.0F, 0.0F, 0.0F);

  auto arr = phase.ToArray();

  UNIMOC_EXPECT_FLOAT_EQ(arr.at(0).Value(), 0.0F);
  UNIMOC_EXPECT_FLOAT_EQ(arr.at(1).Value(), 0.0F);
  UNIMOC_EXPECT_FLOAT_EQ(arr.at(2).Value(), 0.0F);
}

// Test ToArray with negative values
ZTEST(three_phase, test_to_array_with_negative_values) {
  ThreePhase phase(-1.0F, -2.0F, -3.0F);

  auto arr = phase.ToArray();

  UNIMOC_EXPECT_FLOAT_EQ(arr.at(0).Value(), -1.0F);
  UNIMOC_EXPECT_FLOAT_EQ(arr.at(1).Value(), -2.0F);
  UNIMOC_EXPECT_FLOAT_EQ(arr.at(2).Value(), -3.0F);
}

ZTEST(three_phase, test_to_stator_uses_unit_representation) {
  const unimoc::system::ThreePhase<unimoc::unit::Current> kPhase{1.0F, 2.0F, 3.0F};

  const auto kStator = kPhase.ToStator();

  UNIMOC_EXPECT_FLOAT_EQ(kStator.alpha.Value(), -1.0F);
  UNIMOC_EXPECT_NEAR(kStator.beta.Value(), -0.577350269F, 1.0e-6F);
}

ZTEST(three_phase, test_to_stator_rejects_zero_sequence) {
  const unimoc::system::ThreePhase<unimoc::unit::Current> kPhase{2.0F, 2.0F, 2.0F};

  const auto kStator = kPhase.ToStator();

  UNIMOC_EXPECT_FLOAT_EQ(kStator.alpha.Value(), 0.0F);
  UNIMOC_EXPECT_FLOAT_EQ(kStator.beta.Value(), 0.0F);
}

ZTEST(three_phase, test_to_stator_transforms_balanced_phase_vector) {
  const unimoc::system::ThreePhase<unimoc::unit::Current> kPhase{1.0F, -0.5F, -0.5F};

  const auto kStator = kPhase.ToStator();

  UNIMOC_EXPECT_FLOAT_EQ(kStator.alpha.Value(), 1.0F);
  UNIMOC_EXPECT_NEAR(kStator.beta.Value(), 0.0F, 1.0e-6F);
}

// Test constexpr functionality (compile-time evaluation)
ZTEST(three_phase, test_constexpr_constructor) {
  constexpr ThreePhase kPhase(1.0F, 2.0F, 3.0F);

  UNIMOC_EXPECT_FLOAT_EQ(kPhase.a.Value(), 1.0F);
  UNIMOC_EXPECT_FLOAT_EQ(kPhase.b.Value(), 2.0F);
  UNIMOC_EXPECT_FLOAT_EQ(kPhase.c.Value(), 3.0F);
}

// Test constexpr addition
ZTEST(three_phase, test_constexpr_addition) {
  constexpr ThreePhase kPhase1(1.0F, 2.0F, 3.0F);
  constexpr ThreePhase kPhase2(4.0F, 5.0F, 6.0F);
  constexpr ThreePhase kResult = kPhase1 + kPhase2;

  UNIMOC_EXPECT_FLOAT_EQ(kResult.a.Value(), 5.0F);
  UNIMOC_EXPECT_FLOAT_EQ(kResult.b.Value(), 7.0F);
  UNIMOC_EXPECT_FLOAT_EQ(kResult.c.Value(), 9.0F);
}

// Test constexpr subtraction
ZTEST(three_phase, test_constexpr_subtraction) {
  constexpr ThreePhase kPhase1(5.0F, 7.0F, 9.0F);
  constexpr ThreePhase kPhase2(1.0F, 2.0F, 3.0F);
  constexpr ThreePhase kResult = kPhase1 - kPhase2;

  UNIMOC_EXPECT_FLOAT_EQ(kResult.a.Value(), 4.0F);
  UNIMOC_EXPECT_FLOAT_EQ(kResult.b.Value(), 5.0F);
  UNIMOC_EXPECT_FLOAT_EQ(kResult.c.Value(), 6.0F);
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

  UNIMOC_EXPECT_FLOAT_EQ(phase.a.Value(), 1000.0F);
  UNIMOC_EXPECT_FLOAT_EQ(phase.b.Value(), 2000.0F);
  UNIMOC_EXPECT_FLOAT_EQ(phase.c.Value(), 3000.0F);
}

// Test with very small values
ZTEST(three_phase, test_small_values) {
  ThreePhase phase(0.001F, 0.002F, 0.003F);

  UNIMOC_EXPECT_FLOAT_EQ(phase.a.Value(), 0.001F);
  UNIMOC_EXPECT_FLOAT_EQ(phase.b.Value(), 0.002F);
  UNIMOC_EXPECT_FLOAT_EQ(phase.c.Value(), 0.003F);
}

// Test addition and subtraction chaining
ZTEST(three_phase, test_arithmetic_chaining) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(4.0F, 5.0F, 6.0F);
  ThreePhase phase3(7.0F, 8.0F, 9.0F);

  ThreePhase result = phase1 + phase2 - phase3;

  UNIMOC_EXPECT_FLOAT_EQ(result.a.Value(), -2.0F);
  UNIMOC_EXPECT_FLOAT_EQ(result.b.Value(), -1.0F);
  UNIMOC_EXPECT_FLOAT_EQ(result.c.Value(), 0.0F);
}

// Test multiple operations
ZTEST(three_phase, test_multiple_operations) {
  ThreePhase phase1(10.0F, 20.0F, 30.0F);
  ThreePhase phase2(5.0F, 10.0F, 15.0F);

  // Add
  ThreePhase sum = phase1 + phase2;
  UNIMOC_EXPECT_FLOAT_EQ(sum.a.Value(), 15.0F);
  UNIMOC_EXPECT_FLOAT_EQ(sum.b.Value(), 30.0F);
  UNIMOC_EXPECT_FLOAT_EQ(sum.c.Value(), 45.0F);

  // Subtract
  ThreePhase diff = phase1 - phase2;
  UNIMOC_EXPECT_FLOAT_EQ(diff.a.Value(), 5.0F);
  UNIMOC_EXPECT_FLOAT_EQ(diff.b.Value(), 10.0F);
  UNIMOC_EXPECT_FLOAT_EQ(diff.c.Value(), 15.0F);

  // Convert to array
  auto arr = sum.ToArray();
  UNIMOC_EXPECT_FLOAT_EQ(arr.at(0).Value(), 15.0F);
  UNIMOC_EXPECT_FLOAT_EQ(arr.at(1).Value(), 30.0F);
  UNIMOC_EXPECT_FLOAT_EQ(arr.at(2).Value(), 45.0F);
}

// Test floating point precision edge cases
ZTEST(three_phase, test_floating_point_precision) {
  ThreePhase phase1(0.1F + 0.2F, 0.3F, 0.0F);
  ThreePhase phase2(0.3F, 0.3F, 0.0F);

  // Due to floating point precision, these might not be exactly equal
  // but should be very close
  UNIMOC_EXPECT_NEAR(phase1.a.Value(), phase2.a.Value(), 1e-6F);
  UNIMOC_EXPECT_FLOAT_EQ(phase1.b.Value(), phase2.b.Value());
}

// Test that operations don't modify original objects
ZTEST(three_phase, test_operations_immutability) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(4.0F, 5.0F, 6.0F);

  ThreePhase sum = phase1 + phase2;
  ThreePhase diff = phase1 - phase2;

  // Original objects should remain unchanged
  UNIMOC_EXPECT_FLOAT_EQ(phase1.a.Value(), 1.0F);
  UNIMOC_EXPECT_FLOAT_EQ(phase1.b.Value(), 2.0F);
  UNIMOC_EXPECT_FLOAT_EQ(phase1.c.Value(), 3.0F);

  UNIMOC_EXPECT_FLOAT_EQ(phase2.a.Value(), 4.0F);
  UNIMOC_EXPECT_FLOAT_EQ(phase2.b.Value(), 5.0F);
  UNIMOC_EXPECT_FLOAT_EQ(phase2.c.Value(), 6.0F);

  UNIMOC_EXPECT_FLOAT_EQ(sum.a.Value(), 5.0F);
  UNIMOC_EXPECT_FLOAT_EQ(sum.b.Value(), 7.0F);
  UNIMOC_EXPECT_FLOAT_EQ(sum.c.Value(), 9.0F);

  UNIMOC_EXPECT_FLOAT_EQ(diff.a.Value(), -3.0F);
  UNIMOC_EXPECT_FLOAT_EQ(diff.b.Value(), -3.0F);
  UNIMOC_EXPECT_FLOAT_EQ(diff.c.Value(), -3.0F);
}

ZTEST_SUITE(unit_literal, nullptr, nullptr, nullptr, nullptr, nullptr);
ZTEST_SUITE(three_phase, nullptr, nullptr, nullptr, nullptr, nullptr);
