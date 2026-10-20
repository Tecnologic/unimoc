#include "hfi.hpp"
#include <gtest/gtest.h>

using namespace unimoc::unit;

namespace {

unimoc::observer::Hfi<float> MakeHfi() {
  unimoc::observer::Hfi<float> hfi;
  hfi.v_inject = Voltage{2.0F};
  return hfi;
}

}  // namespace

TEST(HfiTest, ExplicitStepFollowsTheFourStepPattern) {
  const auto kHfi = MakeHfi();
  // Estimated angle 0: d axis = alpha, q axis = beta.
  const Ratio kSinTh{0.0F};
  const Ratio kCosTh{1.0F};

  EXPECT_FLOAT_EQ(kHfi.get_injection_voltage(0, kSinTh, kCosTh).alpha.Value(), 2.0F);
  EXPECT_FLOAT_EQ(kHfi.get_injection_voltage(1, kSinTh, kCosTh).alpha.Value(), -2.0F);
  EXPECT_FLOAT_EQ(kHfi.get_injection_voltage(2, kSinTh, kCosTh).beta.Value(), 2.0F);
  EXPECT_FLOAT_EQ(kHfi.get_injection_voltage(3, kSinTh, kCosTh).beta.Value(), -2.0F);
  EXPECT_FLOAT_EQ(kHfi.get_injection_voltage(4, kSinTh, kCosTh).alpha.Value(), 0.0F);
  EXPECT_FLOAT_EQ(kHfi.get_injection_voltage(4, kSinTh, kCosTh).beta.Value(), 0.0F);
}

TEST(HfiTest, ImplicitStepMatchesExplicitStep) {
  auto hfi = MakeHfi();
  const Ratio kSinTh{0.5F};
  const Ratio kCosTh{0.8660254F};
  for (int k = 0; k < 4; ++k) {
    hfi.step = k;
    const auto kImplicit = hfi.get_injection_voltage(kSinTh, kCosTh);
    const auto kExplicit = hfi.get_injection_voltage(k, kSinTh, kCosTh);
    EXPECT_FLOAT_EQ(kImplicit.alpha.Value(), kExplicit.alpha.Value());
    EXPECT_FLOAT_EQ(kImplicit.beta.Value(), kExplicit.beta.Value());
  }
}
