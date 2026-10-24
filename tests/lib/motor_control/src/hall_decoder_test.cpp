/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file hall_decoder_test.cpp
 *    @brief ztest coverage of the Hall decoder, the Hall event queue and the angle wrap.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "hall_decoder.hpp"
#include <zephyr/ztest.h>
#include <array>
#include <cstdint>
#include <numbers>
#include "angle_wrap.hpp"
#include "hall_capture.hpp"
#include "unimoc_test.hpp"

namespace unimoc::observer::test {

using namespace unit;

namespace {

constexpr float kPi = std::numbers::pi_v<float>;
constexpr float kDegree = kPi / 180.0F;

// Forward sequence of the codes, sector 0 first.
constexpr std::array<uint8_t, 6> kForwardCodes = {5U, 1U, 3U, 2U, 6U, 4U};

}  // namespace

ZTEST(hall_decoder, test_wrap_angle_maps_to_one_revolution) {
  UNIMOC_EXPECT_NEAR(WrapAngle(Angle{3.0F * kPi}), kPi * 1.0_rad, 1e-5_rad);
  UNIMOC_EXPECT_NEAR(WrapAngle(Angle{-kPi}), kPi * 1.0_rad, 1e-6_rad);
  UNIMOC_EXPECT_NEAR(WrapAngle(Angle{-3.5F * kPi}), (0.5F * kPi) * 1.0_rad, 1e-5_rad);
  UNIMOC_EXPECT_FLOAT_EQ(WrapAngle(0.25_rad), 0.25_rad);
}

ZTEST(hall_decoder, test_wrap_angle_positive_maps_to_zero_to_two_pi) {
  UNIMOC_EXPECT_NEAR(WrapAnglePositive(Angle{-0.5F * kPi}), (1.5F * kPi) * 1.0_rad, 1e-5_rad);
  UNIMOC_EXPECT_NEAR(WrapAnglePositive(Angle{5.0F * kPi}), kPi * 1.0_rad, 1e-5_rad);
  UNIMOC_EXPECT_FLOAT_EQ(WrapAnglePositive(Angle{2.0F * kPi}), 0.0_rad);
  UNIMOC_EXPECT_FLOAT_EQ(WrapAnglePositive(0.25_rad), 0.25_rad);
  // Rounds to 2 pi when added; the result stays below 2 pi.
  zexpect_true(WrapAnglePositive(-1e-9_rad) < 2.0F * kPi * 1.0_rad);
  zexpect_true(WrapAnglePositive(-1e-9_rad) >= 0.0_rad);
}

ZTEST(hall_decoder, test_codes_map_to_sectors_and_invalid_codes_are_rejected) {
  for (uint8_t sector = 0U; sector < HallDecoder::kSectors; ++sector) {
    const auto kSector = HallDecoder::Sector(kForwardCodes[sector]);
    zassert_true(kSector.has_value());
    zexpect_equal(*kSector, sector);
  }
  zexpect_false(HallDecoder::Sector(0U).has_value());
  zexpect_false(HallDecoder::Sector(7U).has_value());
  zexpect_false(HallDecoder::Sector(8U).has_value());
}

ZTEST(hall_decoder, test_edges_lie_on_the_sector_boundary_in_both_directions) {
  const HallDecoder kDecoder{Angle{10.0F * kDegree}};
  for (uint8_t sector = 0U; sector < HallDecoder::kSectors; ++sector) {
    const uint8_t kFrom = kForwardCodes[sector];
    const uint8_t kTo = kForwardCodes[(sector + 1U) % HallDecoder::kSectors];
    const auto kForward = kDecoder.EdgeAngle(kFrom, kTo);
    const auto kBackward = kDecoder.EdgeAngle(kTo, kFrom);
    zassert_true(kForward.has_value() && kBackward.has_value());
    const Angle kExpected =
        WrapAngle(((10.0F + 60.0F * static_cast<float>(sector + 1U)) * kDegree) * 1.0_rad);
    UNIMOC_EXPECT_NEAR(*kForward, kExpected, 1e-5_rad);
    UNIMOC_EXPECT_NEAR(*kBackward, kExpected, 1e-5_rad);
  }
}

ZTEST(hall_decoder, test_skipped_sectors_and_invalid_codes_give_no_edge) {
  const HallDecoder kDecoder{};
  zexpect_false(kDecoder.EdgeAngle(5U, 3U).has_value());  // sector 0 to 2
  zexpect_false(kDecoder.EdgeAngle(5U, 2U).has_value());  // opposite sector
  zexpect_false(kDecoder.EdgeAngle(5U, 5U).has_value());  // no change
  zexpect_false(kDecoder.EdgeAngle(0U, 5U).has_value());
  zexpect_false(kDecoder.EdgeAngle(5U, 7U).has_value());
}

ZTEST(hall_decoder, test_sector_innovation_pulls_to_the_nearest_boundary) {
  const HallDecoder kDecoder{};  // sector 0 is [0, 60) degrees
  UNIMOC_EXPECT_NEAR(kDecoder.SectorInnovation(0U, Angle{70.0F * kDegree}),
                     (-10.0F * kDegree) * 1.0_rad,
                     1e-5_rad);
  UNIMOC_EXPECT_NEAR(kDecoder.SectorInnovation(0U, Angle{-20.0F * kDegree}),
                     (20.0F * kDegree) * 1.0_rad,
                     1e-5_rad);
  UNIMOC_EXPECT_FLOAT_EQ(kDecoder.SectorInnovation(0U, Angle{30.0F * kDegree}), 0.0_rad);
  // Across the +-pi seam: sector 3 is [180, 240) degrees.
  UNIMOC_EXPECT_NEAR(kDecoder.SectorInnovation(3U, Angle{170.0F * kDegree}),
                     (10.0F * kDegree) * 1.0_rad,
                     1e-5_rad);
  UNIMOC_EXPECT_NEAR(kDecoder.SectorInnovation(3U, Angle{-110.0F * kDegree}),
                     (-10.0F * kDegree) * 1.0_rad,
                     1e-5_rad);
}

ZTEST(hall_decoder, test_event_queue_keeps_order_and_counts_overflows) {
  HallEventQueue<4U> queue;
  HallEvent event{};
  zexpect_false(queue.Pop(event));
  for (uint8_t i = 1U; i <= 4U; ++i) {
    zexpect_true(queue.Push(HallEvent{i, 100U * i}));
  }
  zexpect_false(queue.Push(HallEvent{6U, 500U}));  // full: dropped and counted
  zexpect_equal(queue.Overflows(), 1U);
  zexpect_equal(queue.LatestCode(), 6U);
  for (uint8_t i = 1U; i <= 4U; ++i) {
    zassert_true(queue.Pop(event));
    zexpect_equal(event.code, i);
    zexpect_equal(event.timestamp, 100U * i);
  }
  zexpect_false(queue.Pop(event));
  zexpect_true(queue.Push(HallEvent{2U, 7U}));
  queue.Clear();
  zexpect_false(queue.Pop(event));
}

}  // namespace unimoc::observer::test

ZTEST_SUITE(hall_decoder, nullptr, nullptr, nullptr, nullptr, nullptr);
