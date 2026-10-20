/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file hfi_test.cpp
 *    @brief ztest coverage of the four-step HFI injection voltages.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "hfi.hpp"
#include <zephyr/ztest.h>
#include "unimoc_test.hpp"

using namespace unimoc::unit;

namespace {

unimoc::observer::Hfi<float> MakeHfi() {
  unimoc::observer::Hfi<float> hfi;
  hfi.v_inject = Voltage{2.0F};
  return hfi;
}

}  // namespace

ZTEST(hfi, test_explicit_step_follows_the_four_step_pattern) {
  const auto kHfi = MakeHfi();
  // Estimated angle 0: d axis = alpha, q axis = beta.
  const Ratio kSinTh{0.0F};
  const Ratio kCosTh{1.0F};

  UNIMOC_EXPECT_FLOAT_EQ(kHfi.get_injection_voltage(0, kSinTh, kCosTh).alpha.Value(), 2.0F);
  UNIMOC_EXPECT_FLOAT_EQ(kHfi.get_injection_voltage(1, kSinTh, kCosTh).alpha.Value(), -2.0F);
  UNIMOC_EXPECT_FLOAT_EQ(kHfi.get_injection_voltage(2, kSinTh, kCosTh).beta.Value(), 2.0F);
  UNIMOC_EXPECT_FLOAT_EQ(kHfi.get_injection_voltage(3, kSinTh, kCosTh).beta.Value(), -2.0F);
  UNIMOC_EXPECT_FLOAT_EQ(kHfi.get_injection_voltage(4, kSinTh, kCosTh).alpha.Value(), 0.0F);
  UNIMOC_EXPECT_FLOAT_EQ(kHfi.get_injection_voltage(4, kSinTh, kCosTh).beta.Value(), 0.0F);
}

ZTEST(hfi, test_implicit_step_matches_explicit_step) {
  auto hfi = MakeHfi();
  const Ratio kSinTh{0.5F};
  const Ratio kCosTh{0.8660254F};
  for (int k = 0; k < 4; ++k) {
    hfi.step = k;
    const auto kImplicit = hfi.get_injection_voltage(kSinTh, kCosTh);
    const auto kExplicit = hfi.get_injection_voltage(k, kSinTh, kCosTh);
    UNIMOC_EXPECT_FLOAT_EQ(kImplicit.alpha.Value(), kExplicit.alpha.Value());
    UNIMOC_EXPECT_FLOAT_EQ(kImplicit.beta.Value(), kExplicit.beta.Value());
  }
}

ZTEST_SUITE(hfi, nullptr, nullptr, nullptr, nullptr, nullptr);
