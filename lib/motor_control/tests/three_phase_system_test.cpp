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
#include <gtest/gtest.h>
#include <cmath>
#include <concepts>

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
static_assert(std::same_as<decltype(unimoc::system::ThreePhase<unimoc::unit::Current>{}.ToStator()), unimoc::system::Stator<unimoc::unit::Current>>);

TEST(UnitLiteralTest, ConvertsScaledValuesToBaseUnits) {
  EXPECT_FLOAT_EQ(unimoc::unit::Inductance{47.0_mH}.Value(), 47.0e-3F);
  EXPECT_FLOAT_EQ(unimoc::unit::Time{2.0_us}.Value(), 2.0e-6F);
  EXPECT_FLOAT_EQ(unimoc::unit::Frequency{20.0_kHz}.Value(), 20000.0F);
}

// Test fixture for ThreePhase tests
class ThreePhaseTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Setup code if needed
  }

  void TearDown() override {
    // Cleanup code if needed
  }
};

// Test default constructor
TEST_F(ThreePhaseTest, DefaultConstructor) {
  ThreePhase phase;
  // Default constructor should initialize values (implementation dependent)
  // Just verify object is created without throwing
  EXPECT_NO_THROW(ThreePhase());

  (void)phase;  // Suppress unused variable warning
}

// Test parameterized constructor
TEST_F(ThreePhaseTest, ParameterizedConstructor) {
  ThreePhase phase(1.5F, 2.5F, 3.5F);

  EXPECT_FLOAT_EQ(phase.a.Value(), 1.5F);
  EXPECT_FLOAT_EQ(phase.b.Value(), 2.5F);
  EXPECT_FLOAT_EQ(phase.c.Value(), 3.5F);
}

// Test parameterized constructor with zero values
TEST_F(ThreePhaseTest, ConstructorWithZeros) {
  ThreePhase phase(0.0F, 0.0F, 0.0F);

  EXPECT_FLOAT_EQ(phase.a.Value(), 0.0F);
  EXPECT_FLOAT_EQ(phase.b.Value(), 0.0F);
  EXPECT_FLOAT_EQ(phase.c.Value(), 0.0F);
}

// Test parameterized constructor with negative values
TEST_F(ThreePhaseTest, ConstructorWithNegativeValues) {
  ThreePhase phase(-1.0F, -2.0F, -3.0F);

  EXPECT_FLOAT_EQ(phase.a.Value(), -1.0F);
  EXPECT_FLOAT_EQ(phase.b.Value(), -2.0F);
  EXPECT_FLOAT_EQ(phase.c.Value(), -3.0F);
}

// Test copy constructor
TEST_F(ThreePhaseTest, CopyConstructor) {
  ThreePhase original(1.0F, 2.0F, 3.0F);
  ThreePhase copy(original);

  EXPECT_FLOAT_EQ(copy.a.Value(), 1.0F);
  EXPECT_FLOAT_EQ(copy.b.Value(), 2.0F);
  EXPECT_FLOAT_EQ(copy.c.Value(), 3.0F);

  // Verify independence
  copy.a = 10.0_ratio;
  EXPECT_FLOAT_EQ(original.a.Value(), 1.0F);
}

// Test move constructor
TEST_F(ThreePhaseTest, MoveConstructor) {
  ThreePhase original(1.0F, 2.0F, 3.0F);
  ThreePhase moved(original);

  EXPECT_FLOAT_EQ(moved.a.Value(), 1.0F);
  EXPECT_FLOAT_EQ(moved.b.Value(), 2.0F);
  EXPECT_FLOAT_EQ(moved.c.Value(), 3.0F);
}

// Test copy assignment operator
TEST_F(ThreePhaseTest, CopyAssignmentOperator) {
  ThreePhase original(1.0F, 2.0F, 3.0F);
  ThreePhase copy(4.0F, 5.0F, 6.0F);

  copy = original;

  EXPECT_FLOAT_EQ(copy.a.Value(), 1.0F);
  EXPECT_FLOAT_EQ(copy.b.Value(), 2.0F);
  EXPECT_FLOAT_EQ(copy.c.Value(), 3.0F);
}

// Test move assignment operator
TEST_F(ThreePhaseTest, MoveAssignmentOperator) {
  ThreePhase original(1.0F, 2.0F, 3.0F);
  ThreePhase moved(4.0F, 5.0F, 6.0F);

  moved = original;

  EXPECT_FLOAT_EQ(moved.a.Value(), 1.0F);
  EXPECT_FLOAT_EQ(moved.b.Value(), 2.0F);
  EXPECT_FLOAT_EQ(moved.c.Value(), 3.0F);
}

// Test equality operator with equal values
TEST_F(ThreePhaseTest, EqualityOperatorTrue) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(1.0F, 2.0F, 3.0F);

  EXPECT_TRUE(phase1 == phase2);
  EXPECT_TRUE(phase2 == phase1);
}

// Test equality operator with different values
TEST_F(ThreePhaseTest, EqualityOperatorFalse) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(1.0F, 2.0F, 4.0F);

  EXPECT_FALSE(phase1 == phase2);
}

// Test equality operator with self
TEST_F(ThreePhaseTest, EqualityOperatorSelf) {
  ThreePhase phase(1.0F, 2.0F, 3.0F);

  EXPECT_TRUE(phase == phase);
}

// Test inequality operator
TEST_F(ThreePhaseTest, InequalityOperatorTrue) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(4.0F, 5.0F, 6.0F);

  EXPECT_TRUE(phase1 != phase2);
  EXPECT_TRUE(phase2 != phase1);
}

// Test inequality operator with equal values
TEST_F(ThreePhaseTest, InequalityOperatorFalse) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(1.0F, 2.0F, 3.0F);

  EXPECT_FALSE(phase1 != phase2);
}

// Test addition operator
TEST_F(ThreePhaseTest, AdditionOperator) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(4.0F, 5.0F, 6.0F);

  ThreePhase result = phase1 + phase2;

  EXPECT_FLOAT_EQ(result.a.Value(), 5.0F);
  EXPECT_FLOAT_EQ(result.b.Value(), 7.0F);
  EXPECT_FLOAT_EQ(result.c.Value(), 9.0F);
}

// Test addition with zero
TEST_F(ThreePhaseTest, AdditionWithZero) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase zero(0.0F, 0.0F, 0.0F);

  ThreePhase result = phase1 + zero;

  EXPECT_FLOAT_EQ(result.a.Value(), 1.0F);
  EXPECT_FLOAT_EQ(result.b.Value(), 2.0F);
  EXPECT_FLOAT_EQ(result.c.Value(), 3.0F);
}

// Test addition with negative values
TEST_F(ThreePhaseTest, AdditionWithNegativeValues) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(-1.0F, -2.0F, -3.0F);

  ThreePhase result = phase1 + phase2;

  EXPECT_FLOAT_EQ(result.a.Value(), 0.0F);
  EXPECT_FLOAT_EQ(result.b.Value(), 0.0F);
  EXPECT_FLOAT_EQ(result.c.Value(), 0.0F);
}

// Test addition commutativity
TEST_F(ThreePhaseTest, AdditionCommutativity) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(4.0F, 5.0F, 6.0F);

  ThreePhase result1 = phase1 + phase2;
  ThreePhase result2 = phase2 + phase1;

  EXPECT_TRUE(result1 == result2);
}

// Test subtraction operator
TEST_F(ThreePhaseTest, SubtractionOperator) {
  ThreePhase phase1(5.0F, 7.0F, 9.0F);
  ThreePhase phase2(1.0F, 2.0F, 3.0F);

  ThreePhase result = phase1 - phase2;

  EXPECT_FLOAT_EQ(result.a.Value(), 4.0F);
  EXPECT_FLOAT_EQ(result.b.Value(), 5.0F);
  EXPECT_FLOAT_EQ(result.c.Value(), 6.0F);
}

// Test subtraction with zero
TEST_F(ThreePhaseTest, SubtractionWithZero) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase zero(0.0F, 0.0F, 0.0F);

  ThreePhase result = phase1 - zero;

  EXPECT_FLOAT_EQ(result.a.Value(), 1.0F);
  EXPECT_FLOAT_EQ(result.b.Value(), 2.0F);
  EXPECT_FLOAT_EQ(result.c.Value(), 3.0F);
}

// Test subtraction with itself
TEST_F(ThreePhaseTest, SubtractionWithSelf) {
  ThreePhase phase(1.0F, 2.0F, 3.0F);

  ThreePhase result = phase - phase;

  EXPECT_FLOAT_EQ(result.a.Value(), 0.0F);
  EXPECT_FLOAT_EQ(result.b.Value(), 0.0F);
  EXPECT_FLOAT_EQ(result.c.Value(), 0.0F);
}

// Test subtraction with negative values
TEST_F(ThreePhaseTest, SubtractionWithNegativeValues) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(-1.0F, -2.0F, -3.0F);

  ThreePhase result = phase1 - phase2;

  EXPECT_FLOAT_EQ(result.a.Value(), 2.0F);
  EXPECT_FLOAT_EQ(result.b.Value(), 4.0F);
  EXPECT_FLOAT_EQ(result.c.Value(), 6.0F);
}

// Test ToArray conversion
TEST_F(ThreePhaseTest, ToArrayConversion) {
  ThreePhase phase(1.5F, 2.5F, 3.5F);

  auto arr = phase.ToArray();

  EXPECT_EQ(arr.size(), 3);
  EXPECT_FLOAT_EQ(arr.at(0).Value(), 1.5F);
  EXPECT_FLOAT_EQ(arr.at(1).Value(), 2.5F);
  EXPECT_FLOAT_EQ(arr.at(2).Value(), 3.5F);
}

// Test ToArray with zero values
TEST_F(ThreePhaseTest, ToArrayWithZeros) {
  ThreePhase phase(0.0F, 0.0F, 0.0F);

  auto arr = phase.ToArray();

  EXPECT_FLOAT_EQ(arr.at(0).Value(), 0.0F);
  EXPECT_FLOAT_EQ(arr.at(1).Value(), 0.0F);
  EXPECT_FLOAT_EQ(arr.at(2).Value(), 0.0F);
}

// Test ToArray with negative values
TEST_F(ThreePhaseTest, ToArrayWithNegativeValues) {
  ThreePhase phase(-1.0F, -2.0F, -3.0F);

  auto arr = phase.ToArray();

  EXPECT_FLOAT_EQ(arr.at(0).Value(), -1.0F);
  EXPECT_FLOAT_EQ(arr.at(1).Value(), -2.0F);
  EXPECT_FLOAT_EQ(arr.at(2).Value(), -3.0F);
}

TEST_F(ThreePhaseTest, ToStatorUsesUnitRepresentation) {
  const unimoc::system::ThreePhase<unimoc::unit::Current> kPhase{1.0F, 2.0F, 3.0F};

  const auto kStator = kPhase.ToStator();

  EXPECT_FLOAT_EQ(kStator.alpha.Value(), -1.0F);
  EXPECT_NEAR(kStator.beta.Value(), -0.577350269F, 1.0e-6F);
}

TEST_F(ThreePhaseTest, ToStatorRejectsZeroSequence) {
  const unimoc::system::ThreePhase<unimoc::unit::Current> kPhase{2.0F, 2.0F, 2.0F};

  const auto kStator = kPhase.ToStator();

  EXPECT_FLOAT_EQ(kStator.alpha.Value(), 0.0F);
  EXPECT_FLOAT_EQ(kStator.beta.Value(), 0.0F);
}

TEST_F(ThreePhaseTest, ToStatorTransformsBalancedPhaseVector) {
  const unimoc::system::ThreePhase<unimoc::unit::Current> kPhase{1.0F, -0.5F, -0.5F};

  const auto kStator = kPhase.ToStator();

  EXPECT_FLOAT_EQ(kStator.alpha.Value(), 1.0F);
  EXPECT_NEAR(kStator.beta.Value(), 0.0F, 1.0e-6F);
}

// Test constexpr functionality (compile-time evaluation)
TEST_F(ThreePhaseTest, ConstexprConstructor) {
  constexpr ThreePhase kPhase(1.0F, 2.0F, 3.0F);

  EXPECT_FLOAT_EQ(kPhase.a.Value(), 1.0F);
  EXPECT_FLOAT_EQ(kPhase.b.Value(), 2.0F);
  EXPECT_FLOAT_EQ(kPhase.c.Value(), 3.0F);
}

// Test constexpr addition
TEST_F(ThreePhaseTest, ConstexprAddition) {
  constexpr ThreePhase kPhase1(1.0F, 2.0F, 3.0F);
  constexpr ThreePhase kPhase2(4.0F, 5.0F, 6.0F);
  constexpr ThreePhase kResult = kPhase1 + kPhase2;

  EXPECT_FLOAT_EQ(kResult.a.Value(), 5.0F);
  EXPECT_FLOAT_EQ(kResult.b.Value(), 7.0F);
  EXPECT_FLOAT_EQ(kResult.c.Value(), 9.0F);
}

// Test constexpr subtraction
TEST_F(ThreePhaseTest, ConstexprSubtraction) {
  constexpr ThreePhase kPhase1(5.0F, 7.0F, 9.0F);
  constexpr ThreePhase kPhase2(1.0F, 2.0F, 3.0F);
  constexpr ThreePhase kResult = kPhase1 - kPhase2;

  EXPECT_FLOAT_EQ(kResult.a.Value(), 4.0F);
  EXPECT_FLOAT_EQ(kResult.b.Value(), 5.0F);
  EXPECT_FLOAT_EQ(kResult.c.Value(), 6.0F);
}

// Test constexpr equality
TEST_F(ThreePhaseTest, ConstexprEquality) {
  constexpr ThreePhase kPhase1(1.0F, 2.0F, 3.0F);
  constexpr ThreePhase kPhase2(1.0F, 2.0F, 3.0F);
  constexpr bool kEqual = kPhase1 == kPhase2;

  EXPECT_TRUE(kEqual);
}

// Test with large values
TEST_F(ThreePhaseTest, LargeValues) {
  ThreePhase phase(1000.0F, 2000.0F, 3000.0F);

  EXPECT_FLOAT_EQ(phase.a.Value(), 1000.0F);
  EXPECT_FLOAT_EQ(phase.b.Value(), 2000.0F);
  EXPECT_FLOAT_EQ(phase.c.Value(), 3000.0F);
}

// Test with very small values
TEST_F(ThreePhaseTest, SmallValues) {
  ThreePhase phase(0.001F, 0.002F, 0.003F);

  EXPECT_FLOAT_EQ(phase.a.Value(), 0.001F);
  EXPECT_FLOAT_EQ(phase.b.Value(), 0.002F);
  EXPECT_FLOAT_EQ(phase.c.Value(), 0.003F);
}

// Test addition and subtraction chaining
TEST_F(ThreePhaseTest, ArithmeticChaining) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(4.0F, 5.0F, 6.0F);
  ThreePhase phase3(7.0F, 8.0F, 9.0F);

  ThreePhase result = phase1 + phase2 - phase3;

  EXPECT_FLOAT_EQ(result.a.Value(), -2.0F);
  EXPECT_FLOAT_EQ(result.b.Value(), -1.0F);
  EXPECT_FLOAT_EQ(result.c.Value(), 0.0F);
}

// Test multiple operations
TEST_F(ThreePhaseTest, MultipleOperations) {
  ThreePhase phase1(10.0F, 20.0F, 30.0F);
  ThreePhase phase2(5.0F, 10.0F, 15.0F);

  // Add
  ThreePhase sum = phase1 + phase2;
  EXPECT_FLOAT_EQ(sum.a.Value(), 15.0F);
  EXPECT_FLOAT_EQ(sum.b.Value(), 30.0F);
  EXPECT_FLOAT_EQ(sum.c.Value(), 45.0F);

  // Subtract
  ThreePhase diff = phase1 - phase2;
  EXPECT_FLOAT_EQ(diff.a.Value(), 5.0F);
  EXPECT_FLOAT_EQ(diff.b.Value(), 10.0F);
  EXPECT_FLOAT_EQ(diff.c.Value(), 15.0F);

  // Convert to array
  auto arr = sum.ToArray();
  EXPECT_FLOAT_EQ(arr.at(0).Value(), 15.0F);
  EXPECT_FLOAT_EQ(arr.at(1).Value(), 30.0F);
  EXPECT_FLOAT_EQ(arr.at(2).Value(), 45.0F);
}

// Test floating point precision edge cases
TEST_F(ThreePhaseTest, FloatingPointPrecision) {
  ThreePhase phase1(0.1F + 0.2F, 0.3F, 0.0F);
  ThreePhase phase2(0.3F, 0.3F, 0.0F);

  // Due to floating point precision, these might not be exactly equal
  // but should be very close
  EXPECT_NEAR(phase1.a.Value(), phase2.a.Value(), 1e-6F);
  EXPECT_FLOAT_EQ(phase1.b.Value(), phase2.b.Value());
}

// Test that operations don't modify original objects
TEST_F(ThreePhaseTest, OperationsImmutability) {
  ThreePhase phase1(1.0F, 2.0F, 3.0F);
  ThreePhase phase2(4.0F, 5.0F, 6.0F);

  ThreePhase sum = phase1 + phase2;
  ThreePhase diff = phase1 - phase2;

  // Original objects should remain unchanged
  EXPECT_FLOAT_EQ(phase1.a.Value(), 1.0F);
  EXPECT_FLOAT_EQ(phase1.b.Value(), 2.0F);
  EXPECT_FLOAT_EQ(phase1.c.Value(), 3.0F);

  EXPECT_FLOAT_EQ(phase2.a.Value(), 4.0F);
  EXPECT_FLOAT_EQ(phase2.b.Value(), 5.0F);
  EXPECT_FLOAT_EQ(phase2.c.Value(), 6.0F);

  EXPECT_FLOAT_EQ(sum.a.Value(), 5.0F);
  EXPECT_FLOAT_EQ(sum.b.Value(), 7.0F);
  EXPECT_FLOAT_EQ(sum.c.Value(), 9.0F);

  EXPECT_FLOAT_EQ(diff.a.Value(), -3.0F);
  EXPECT_FLOAT_EQ(diff.b.Value(), -3.0F);
  EXPECT_FLOAT_EQ(diff.c.Value(), -3.0F);
}
