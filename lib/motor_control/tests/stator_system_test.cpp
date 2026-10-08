#include "stator_system.hpp"
#include <gtest/gtest.h>
#include <concepts>
#include "rotor_angle.hpp"
#include "rotor_system.hpp"
#include "three_phase_system.hpp"

using Stator = unimoc::system::Stator<unimoc::unit::Ratio>;
using Rotor = unimoc::system::Rotor<unimoc::unit::Ratio>;
using RotorAngle = unimoc::system::RotorAngle;
using namespace unimoc::unit;

class StatorSystemTest : public ::testing::Test {};

TEST_F(StatorSystemTest, ToRotorAtZeroAngle) {
  const Stator kStator{1.0F, 2.0F};
  const RotorAngle kAngle;

  const auto kRotor = kStator.ToRotor(kAngle);

  EXPECT_NEAR(kRotor.d.Value(), 1.0F, 1.0e-5F);
  EXPECT_NEAR(kRotor.q.Value(), 2.0F, 1.0e-5F);
}

TEST_F(StatorSystemTest, ToRotorAtQuarterTurn) {
  const Stator kStator{1.0F, 2.0F};
  const RotorAngle kAngle = RotorAngle::FromRaw(0x4000'0000);

  const auto kRotor = kStator.ToRotor(kAngle);

  EXPECT_NEAR(kRotor.d.Value(), 2.0F, 1.0e-5F);
  EXPECT_NEAR(kRotor.q.Value(), -1.0F, 1.0e-5F);
}

TEST_F(StatorSystemTest, ParkAndInverseParkRoundTrip) {
  const Stator kOriginal{1.25F, -0.75F};
  const RotorAngle kAngle = RotorAngle::FromAngle(0.6435011_rad);

  const Rotor kRotor = kOriginal.ToRotor(kAngle);
  const Stator kRestored = kRotor.ToStator(kAngle);

  EXPECT_NEAR(kRestored.alpha.Value(), kOriginal.alpha.Value(), 1.0e-5F);
  EXPECT_NEAR(kRestored.beta.Value(), kOriginal.beta.Value(), 1.0e-5F);
}

TEST_F(StatorSystemTest, ParkTransformIsConstexpr) {
  constexpr Stator kStator{1.0F, 2.0F};
  constexpr Rotor kRotor = kStator.ToRotor(RotorAngle{});

  static_assert(kRotor.d.Value() == 1.0F);
  static_assert(kRotor.q.Value() == 2.0F);
  EXPECT_FLOAT_EQ(kRotor.d.Value(), 1.0F);
}

TEST_F(StatorSystemTest, LengthSquaredAvoidsSquareRoot) {
  constexpr Stator kStator{3.0F, 4.0F};

  static_assert(std::same_as<decltype(kStator.LengthSquared()), float>);
  static_assert(kStator.LengthSquared() == 25.0F);
  EXPECT_FLOAT_EQ(kStator.LengthSquared(), 25.0F);
}

TEST_F(StatorSystemTest, ClarkeAndParkPreserveTheUnitType) {
  const unimoc::system::ThreePhase<unimoc::unit::Current> kPhase{1.0F, -0.5F, -0.5F};

  const unimoc::system::Stator<unimoc::unit::Current> kStator = kPhase.ToStator();
  const unimoc::system::Rotor<unimoc::unit::Current> kRotor = kStator.ToRotor(RotorAngle{});

  EXPECT_NEAR(kRotor.d.Value(), 1.0F, 1.0e-5F);
}

TEST_F(StatorSystemTest, InverseClarkePreservesTheUnitType) {
  const unimoc::system::Stator<unimoc::unit::Current> kStator{1.0F, 0.0F};

  const auto kPhase = kStator.ToThreePhase();

  EXPECT_NEAR(kPhase.a.Value(), 1.0F, 1.0e-5F);
  EXPECT_NEAR(kPhase.b.Value(), -0.5F, 1.0e-5F);
  EXPECT_NEAR(kPhase.c.Value(), -0.5F, 1.0e-5F);
}