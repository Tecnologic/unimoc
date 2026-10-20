#include "svm.hpp"
#include <gtest/gtest.h>

using Stator = unimoc::system::Stator<unimoc::unit::Ratio>;
using Svm = unimoc::control::Svm;
using namespace unimoc::unit;

class SvmTest : public ::testing::Test {};

TEST_F(SvmTest, ZeroVoltageProducesCenteredDuties) {
  const Svm kSvm;

  const auto kDuties = kSvm.Calculate(Stator{0.0F, 0.0F}.ToThreePhase());

  EXPECT_FLOAT_EQ(kDuties.a.Value(), 0.5F);
  EXPECT_FLOAT_EQ(kDuties.b.Value(), 0.5F);
  EXPECT_FLOAT_EQ(kDuties.c.Value(), 0.5F);
}

TEST_F(SvmTest, AlphaVoltageUsesCenteredZeroSequence) {
  const Svm kSvm;

  const auto kDuties = kSvm.Calculate(Stator{0.2F, 0.0F}.ToThreePhase());

  EXPECT_NEAR(kDuties.a.Value(), 0.65F, 1.0e-6F);
  EXPECT_NEAR(kDuties.b.Value(), 0.35F, 1.0e-6F);
  EXPECT_NEAR(kDuties.c.Value(), 0.35F, 1.0e-6F);
  EXPECT_NEAR(kDuties.a.Value() + kDuties.c.Value(), 1.0F, 1.0e-6F);
}

TEST_F(SvmTest, DutiesAreClampedToConfiguredRange) {
  unimoc::settings::NvmSettings settings;
  settings.svm_duty_min = 0.2_ratio;
  settings.svm_duty_max = 0.8_ratio;

  Svm svm;
  svm.Init(settings);

  const auto kDuties = svm.Calculate(Stator{1.0F, 0.0F}.ToThreePhase());

  EXPECT_FLOAT_EQ(kDuties.a.Value(), 0.8F);
  EXPECT_FLOAT_EQ(kDuties.b.Value(), 0.2F);
  EXPECT_FLOAT_EQ(kDuties.c.Value(), 0.2F);
}
TEST_F(SvmTest, VoltageOverloadNormalisesByDcLinkVoltage) {
  const Svm kSvm;
  const unimoc::system::Stator<unimoc::unit::Voltage> kVoltage{10.0F, 0.0F};

  const auto kDuties = kSvm.CalculateWithDeadTimeCompensation(kVoltage, 50.0_V, Stator{0.0F, 0.0F});

  EXPECT_NEAR(kDuties.a.Value(), 0.65F, 1.0e-6F);
  EXPECT_NEAR(kDuties.b.Value(), 0.35F, 1.0e-6F);
  EXPECT_NEAR(kDuties.c.Value(), 0.35F, 1.0e-6F);
}

TEST_F(SvmTest, VoltageOverloadAddsDeadTimeCompensation) {
  const Svm kSvm;
  const unimoc::system::Stator<unimoc::unit::Voltage> kVoltage{0.0F, 0.0F};

  const auto kDuties = kSvm.CalculateWithDeadTimeCompensation(kVoltage, 50.0_V, Stator{0.1F, 0.0F});

  EXPECT_NEAR(kDuties.a.Value(), 0.575F, 1.0e-6F);
  EXPECT_NEAR(kDuties.b.Value(), 0.425F, 1.0e-6F);
  EXPECT_NEAR(kDuties.c.Value(), 0.425F, 1.0e-6F);
}

TEST_F(SvmTest, VoltageOverloadGuardsAgainstZeroDcLinkVoltage) {
  const Svm kSvm;
  const unimoc::system::Stator<unimoc::unit::Voltage> kVoltage{0.2F, 0.0F};

  const auto kDuties = kSvm.CalculateWithDeadTimeCompensation(kVoltage, 0.0_V, Stator{0.0F, 0.0F});

  EXPECT_NEAR(kDuties.a.Value(), 0.65F, 1.0e-6F);
}

TEST_F(SvmTest, PrecomputedInverseMatchesVoltageOverload) {
  const Svm kSvm;
  const unimoc::system::Stator<unimoc::unit::Voltage> kVoltage{7.0F, -3.0F};
  const Stator kDeadTime{0.01F, -0.02F};
  const float kInverse = Svm::InverseDcLinkVoltage(48.0_V);

  const auto kDirect = kSvm.CalculateWithDeadTimeCompensation(kVoltage, 48.0_V, kDeadTime);
  const auto kPrecomputed = kSvm.CalculateWithInverseDcLinkVoltage(kVoltage, kInverse, kDeadTime);

  EXPECT_FLOAT_EQ(kInverse, 1.0F / 48.0F);
  EXPECT_FLOAT_EQ(kPrecomputed.a.Value(), kDirect.a.Value());
  EXPECT_FLOAT_EQ(kPrecomputed.b.Value(), kDirect.b.Value());
  EXPECT_FLOAT_EQ(kPrecomputed.c.Value(), kDirect.c.Value());
}

TEST_F(SvmTest, InverseDcLinkVoltageGuardsAgainstZero) {
  EXPECT_FLOAT_EQ(Svm::InverseDcLinkVoltage(0.0_V), 1.0F);
}
