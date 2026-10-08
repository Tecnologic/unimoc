#include "rotor_system.hpp"
#include <gtest/gtest.h>
#include <concepts>
#include "rotor_angle.hpp"

using Rotor = unimoc::system::Rotor<unimoc::unit::Ratio>;
using Stator = unimoc::system::Stator<unimoc::unit::Ratio>;
using RotorAngle = unimoc::system::RotorAngle;

class RotorSystemTest : public ::testing::Test {};

TEST_F(RotorSystemTest, ToStatorAtZeroAngle) {
  const Rotor kRotor{1.0F, 2.0F};
  const RotorAngle kAngle;

  const auto kStator = kRotor.ToStator(kAngle);

  EXPECT_NEAR(kStator.alpha.Value(), 1.0F, 1.0e-5F);
  EXPECT_NEAR(kStator.beta.Value(), 2.0F, 1.0e-5F);
}

TEST_F(RotorSystemTest, ToStatorAtQuarterTurn) {
  const Rotor kRotor{1.0F, 2.0F};
  const RotorAngle kAngle = RotorAngle::FromRaw(0x4000'0000);

  const auto kStator = kRotor.ToStator(kAngle);

  EXPECT_NEAR(kStator.alpha.Value(), -2.0F, 1.0e-5F);
  EXPECT_NEAR(kStator.beta.Value(), 1.0F, 1.0e-5F);
}

TEST_F(RotorSystemTest, LengthSquaredAvoidsSquareRoot) {
  constexpr Rotor kRotor{3.0F, 4.0F};

  static_assert(std::same_as<decltype(kRotor.LengthSquared()), float>);
  static_assert(kRotor.LengthSquared() == 25.0F);
  EXPECT_FLOAT_EQ(kRotor.LengthSquared(), 25.0F);
}