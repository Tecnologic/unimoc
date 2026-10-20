/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file sin_cos_test.cpp
 *    @brief ztest coverage of the sin/cos providers.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "sin_cos.hpp"
#include <zephyr/ztest.h>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <numbers>
#include "unimoc_test.hpp"

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

  [[nodiscard]] static constexpr Result Calculate(std::int32_t k_raw) noexcept {
    static_cast<void>(k_raw);
    return Result{0.25_ratio, 0.75_ratio};
  }
};

static_assert(SinCosProvider<PortableSinCosProvider>);
static_assert(SinCosProvider<TestProvider>);

}  // namespace

ZTEST(portable_sin_cos_provider, test_generates_zero_angle) {
  constexpr auto kResult = PortableSinCosProvider::Calculate(0);
  static_assert(kResult.sin.Value() == 0.0F);
  static_assert(kResult.cos.Value() == 1.0F);
  UNIMOC_EXPECT_FLOAT_EQ(kResult.sin.Value(), 0.0F);
  UNIMOC_EXPECT_FLOAT_EQ(kResult.cos.Value(), 1.0F);
}

ZTEST(portable_sin_cos_provider, test_matches_standard_library) {
  constexpr int kSamples = 4096;
  for (int sample_index = 0; sample_index < kSamples; ++sample_index) {
    const float kExpected =
        -kPi + ((kTwoPi * static_cast<float>(sample_index)) / static_cast<float>(kSamples));
    const auto kResult = PortableSinCosProvider::Calculate(Angle{kExpected});

    UNIMOC_EXPECT_NEAR(kResult.sin.Value(),
                       std::sin(kExpected),
                       2.0e-5F,
                       "sample=%d",
                       static_cast<int>(sample_index));
    UNIMOC_EXPECT_NEAR(kResult.cos.Value(),
                       std::cos(kExpected),
                       2.0e-5F,
                       "sample=%d",
                       static_cast<int>(sample_index));
  }
}

ZTEST(portable_sin_cos_provider, test_angle_value_uses_portable_provider) {
  const unimoc::system::SinCos<unimoc::unit::Ratio> kResult{0.5_rad};
  const auto kExpected = PortableSinCosProvider::Calculate(0.5_rad);

  UNIMOC_EXPECT_FLOAT_EQ(kResult.sin.Value(), kExpected.sin.Value());
  UNIMOC_EXPECT_FLOAT_EQ(kResult.cos.Value(), kExpected.cos.Value());
}

ZTEST(sin_cos_provider, test_generate_sin_cos_uses_provider_contract) {
  constexpr TestProvider kProvider;
  constexpr auto kResult = GenerateSinCos(kProvider, 123);

  static_assert(kResult.sin.Value() == 0.25F);
  static_assert(kResult.cos.Value() == 0.75F);
  UNIMOC_EXPECT_FLOAT_EQ(kResult.sin.Value(), 0.25F);
  UNIMOC_EXPECT_FLOAT_EQ(kResult.cos.Value(), 0.75F);
}

ZTEST_SUITE(portable_sin_cos_provider, nullptr, nullptr, nullptr, nullptr, nullptr);
ZTEST_SUITE(sin_cos_provider, nullptr, nullptr, nullptr, nullptr, nullptr);
