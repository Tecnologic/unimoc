/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file unimoc_test.hpp
 *    @brief Floating-point checks for the ztest suites of UNIMOC.
 *
 *    ztest has no counterpart to a relative float comparison. These macros
 *    follow the ztest conventions: the UNIMOC_EXPECT_* forms mark the test as
 *    failed and continue (like zexpect_*), the UNIMOC_ASSERT_* forms return
 *    from the test (like zassert_*). Each operand is evaluated once; on a
 *    failure the values are printed before the ztest report.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <zephyr/sys/printk.h>
#include <zephyr/ztest.h>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace unimoc::test {

/// Units in the last place two floats may differ by, as in GoogleTest's float comparison.
inline constexpr int32_t kMaxUlps = 4;

/**
 * @brief Maps a float onto an integer line where adjacent floats differ by one.
 * @param value Finite or infinite float.
 * @return Biased integer representation.
 */
[[nodiscard]] inline int64_t OrderedBits(const float value) noexcept {
  uint32_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  const auto kMagnitude = static_cast<int64_t>(bits & 0x7FFFFFFFU);
  return (bits & 0x80000000U) != 0U ? -kMagnitude : kMagnitude;
}

/**
 * @brief Checks two floats for equality within kMaxUlps.
 * @param actual Value under test.
 * @param expected Expected value.
 * @return true when both are within kMaxUlps; prints both values otherwise.
 */
[[nodiscard]] inline bool FloatEq(const float actual, const float expected) noexcept {
  const int64_t kDistance = OrderedBits(actual) - OrderedBits(expected);
  const bool kEqual =
      !std::isnan(actual) && !std::isnan(expected) && kDistance <= kMaxUlps && -kDistance <= kMaxUlps;
  if (!kEqual) {
    printk("    actual %.9g, expected %.9g\n", static_cast<double>(actual), static_cast<double>(expected));
  }
  return kEqual;
}

/**
 * @brief Checks that two values differ by at most a tolerance.
 * @param actual Value under test.
 * @param expected Expected value.
 * @param tolerance Largest allowed absolute difference.
 * @return true when |actual - expected| <= tolerance; prints the values otherwise.
 */
[[nodiscard]] inline bool Near(const double actual, const double expected, const double tolerance) noexcept {
  const bool kNear = std::fabs(actual - expected) <= tolerance;
  if (!kNear) {
    printk("    actual %.9g, expected %.9g, tolerance %.3g\n", actual, expected, tolerance);
  }
  return kNear;
}

}  // namespace unimoc::test

// NOLINTBEGIN(cppcoreguidelines-macro-usage): assertion macros must expand at the call site,
// so that ztest reports the file and line of the check.

/// Float equality within 4 ULP; marks the test as failed and continues.
#define UNIMOC_EXPECT_FLOAT_EQ(actual, expected, ...)                                             \
  zexpect_true(::unimoc::test::FloatEq(static_cast<float>(actual), static_cast<float>(expected)), \
               ##__VA_ARGS__)

/// Absolute tolerance; marks the test as failed and continues.
#define UNIMOC_EXPECT_NEAR(actual, expected, tolerance, ...)         \
  zexpect_true(::unimoc::test::Near(static_cast<double>(actual),     \
                                    static_cast<double>(expected),   \
                                    static_cast<double>(tolerance)), \
               ##__VA_ARGS__)

/// Absolute tolerance; ends the test on failure.
#define UNIMOC_ASSERT_NEAR(actual, expected, tolerance, ...)         \
  zassert_true(::unimoc::test::Near(static_cast<double>(actual),     \
                                    static_cast<double>(expected),   \
                                    static_cast<double>(tolerance)), \
               ##__VA_ARGS__)

// NOLINTEND(cppcoreguidelines-macro-usage)
