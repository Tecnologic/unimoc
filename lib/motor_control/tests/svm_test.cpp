#include "svm.hpp"
#include <gtest/gtest.h>

using Stator = unimoc::system::Stator<unimoc::unit::DimensionlessRatio>;
using Svm = unimoc::control::Svm;
using namespace unimoc::unit;

class SvmTest : public ::testing::Test {};

TEST_F(SvmTest, ZeroVoltageProducesCenteredDuties) {
  const Svm svm;

  const auto duties = svm.Calculate(Stator{0.0F, 0.0F}.ToThreePhase());

  EXPECT_FLOAT_EQ(duties.a.Value(), 0.5F);
  EXPECT_FLOAT_EQ(duties.b.Value(), 0.5F);
  EXPECT_FLOAT_EQ(duties.c.Value(), 0.5F);
}

TEST_F(SvmTest, AlphaVoltageUsesCenteredZeroSequence) {
  const Svm svm;

  const auto duties = svm.Calculate(Stator{0.2F, 0.0F}.ToThreePhase());

  EXPECT_NEAR(duties.a.Value(), 0.65F, 1.0e-6F);
  EXPECT_NEAR(duties.b.Value(), 0.35F, 1.0e-6F);
  EXPECT_NEAR(duties.c.Value(), 0.35F, 1.0e-6F);
  EXPECT_NEAR(duties.a.Value() + duties.c.Value(), 1.0F, 1.0e-6F);
}

TEST_F(SvmTest, DutiesAreClampedToConfiguredRange) {
  unimoc::settings::NvmSettings settings;
  settings.svm_duty_min = 0.2_ratio;
  settings.svm_duty_max = 0.8_ratio;

  Svm svm;
  svm.Init(settings);

  const auto duties = svm.Calculate(Stator{1.0F, 0.0F}.ToThreePhase());

  EXPECT_FLOAT_EQ(duties.a.Value(), 0.8F);
  EXPECT_FLOAT_EQ(duties.b.Value(), 0.2F);
  EXPECT_FLOAT_EQ(duties.c.Value(), 0.2F);
}
TEST_F(SvmTest, VoltageOverloadNormalisesByDcLinkVoltage) {
  const Svm svm;
  const unimoc::system::Stator<unimoc::unit::Voltage> voltage{10.0F, 0.0F};

  const auto duties = svm.CalculateWithDeadTimeCompensation(voltage, 50.0_V, Stator{0.0F, 0.0F});

  EXPECT_NEAR(duties.a.Value(), 0.65F, 1.0e-6F);
  EXPECT_NEAR(duties.b.Value(), 0.35F, 1.0e-6F);
  EXPECT_NEAR(duties.c.Value(), 0.35F, 1.0e-6F);
}

TEST_F(SvmTest, VoltageOverloadAddsDeadTimeCompensation) {
  const Svm svm;
  const unimoc::system::Stator<unimoc::unit::Voltage> voltage{0.0F, 0.0F};

  const auto duties = svm.CalculateWithDeadTimeCompensation(voltage, 50.0_V, Stator{0.1F, 0.0F});

  EXPECT_NEAR(duties.a.Value(), 0.575F, 1.0e-6F);
  EXPECT_NEAR(duties.b.Value(), 0.425F, 1.0e-6F);
  EXPECT_NEAR(duties.c.Value(), 0.425F, 1.0e-6F);
}

TEST_F(SvmTest, VoltageOverloadGuardsAgainstZeroDcLinkVoltage) {
  const Svm svm;
  const unimoc::system::Stator<unimoc::unit::Voltage> voltage{0.2F, 0.0F};

  const auto duties = svm.CalculateWithDeadTimeCompensation(voltage, 0.0_V, Stator{0.0F, 0.0F});

  EXPECT_NEAR(duties.a.Value(), 0.65F, 1.0e-6F);
}
