#include "sin_cos.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <numbers>

using unimoc::system::GenerateSinCos;
using unimoc::system::PortableSinCosProvider;
using unimoc::system::SinCosProvider;
using unimoc::unit::Angle;
using namespace unimoc::unit;

namespace {

constexpr float kPi = std::numbers::pi_v<float>;
constexpr float kTwoPi = 2.0F * kPi;

struct TestProvider {
  using Result = unimoc::system::SinCos<unimoc::unit::Ratio>;

  [[nodiscard]] static constexpr Result Calculate(std::int32_t kRaw) noexcept {
    static_cast<void>(kRaw);
    return Result{0.25_ratio, 0.75_ratio};
  }
};

static_assert(SinCosProvider<PortableSinCosProvider>);
static_assert(SinCosProvider<TestProvider>);

}  // namespace

TEST(PortableSinCosProviderTest, GeneratesZeroAngle) {
  constexpr auto kResult = PortableSinCosProvider::Calculate(0);
  static_assert(kResult.sin.Value() == 0.0F);
  static_assert(kResult.cos.Value() == 1.0F);
  EXPECT_FLOAT_EQ(kResult.sin.Value(), 0.0F);
  EXPECT_FLOAT_EQ(kResult.cos.Value(), 1.0F);
}

TEST(PortableSinCosProviderTest, MatchesStandardLibrary) {
  constexpr int kSamples = 4096;
  for (int sample_index = 0; sample_index < kSamples; ++sample_index) {
    const float kExpected = -kPi + ((kTwoPi * static_cast<float>(sample_index)) / static_cast<float>(kSamples));
    const auto kResult = PortableSinCosProvider::Calculate(Angle{kExpected});

    EXPECT_NEAR(kResult.sin.Value(), std::sin(kExpected), 2.0e-5F) << "sample=" << sample_index;
    EXPECT_NEAR(kResult.cos.Value(), std::cos(kExpected), 2.0e-5F) << "sample=" << sample_index;
  }
}

TEST(PortableSinCosProviderTest, AngleValueUsesPortableProvider) {
  const unimoc::system::SinCos<unimoc::unit::Ratio> kResult{0.5_rad};
  const auto kExpected = PortableSinCosProvider::Calculate(0.5_rad);

  EXPECT_FLOAT_EQ(kResult.sin.Value(), kExpected.sin.Value());
  EXPECT_FLOAT_EQ(kResult.cos.Value(), kExpected.cos.Value());
}

TEST(SinCosProviderTest, GenerateSinCosUsesProviderContract) {
  constexpr TestProvider kProvider;
  constexpr auto kResult = GenerateSinCos(kProvider, 123);

  static_assert(kResult.sin.Value() == 0.25F);
  static_assert(kResult.cos.Value() == 0.75F);
  EXPECT_FLOAT_EQ(kResult.sin.Value(), 0.25F);
  EXPECT_FLOAT_EQ(kResult.cos.Value(), 0.75F);
}