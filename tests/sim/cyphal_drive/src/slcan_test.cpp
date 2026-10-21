/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file slcan_test.cpp
 *    @brief Line protocol of the native_sim slcan interface.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <zephyr/ztest.h>
#include <array>
#include <string_view>
#include "slcan_media.hpp"

namespace {

using unimoc::cyphal::CanFrame;
using unimoc::hardware::sim::SlcanMedia;

ZTEST(sim_slcan, test_open_close_and_other_commands_are_acknowledged) {
  auto result = SlcanMedia::ParseLine("O");
  zassert_true(result.open.has_value());
  zexpect_true(*result.open);
  zexpect_equal(result.reply, std::string_view{"\r"});
  result = SlcanMedia::ParseLine("C");
  zexpect_false(result.open.value_or(true));
  result = SlcanMedia::ParseLine("S8");
  zexpect_false(result.open.has_value());
  zexpect_false(result.frame.has_value());
  zexpect_equal(result.reply, std::string_view{"\r"});
}

ZTEST(sim_slcan, test_extended_frame_round_trip) {
  CanFrame frame{};
  frame.extended_can_id = 0x107D550AU;
  frame.size = 3U;
  frame.data[0] = std::byte{0x00};
  frame.data[1] = std::byte{0xAB};
  frame.data[2] = std::byte{0xE1};
  std::array<char, 32> text{};
  const std::size_t kLength = SlcanMedia::FormatFrame(frame, text);
  zassert_equal(std::string_view(text.data(), kLength), std::string_view{"T107D550A300ABE1\r"});

  const auto kResult = SlcanMedia::ParseLine(std::string_view(text.data(), kLength - 1U));
  zassert_true(kResult.frame.has_value());
  zexpect_equal(kResult.frame->extended_can_id, frame.extended_can_id);
  zexpect_equal(kResult.frame->size, 3U);
  zexpect_equal(kResult.frame->data[1], std::byte{0xAB});
  zexpect_true(kResult.reply.empty());
}

ZTEST(sim_slcan, test_fd_frames_are_accepted_and_bad_lines_rejected) {
  // 'D' with DLC 9 = 12 bytes.
  const auto kFd = SlcanMedia::ParseLine("D000000019000102030405060708090A0B");
  zassert_true(kFd.frame.has_value());
  zexpect_equal(kFd.frame->size, 12U);
  zexpect_equal(kFd.frame->data[11], std::byte{0x0B});

  zexpect_false(SlcanMedia::ParseLine("T1234").frame.has_value());           // too short
  zexpect_false(SlcanMedia::ParseLine("T00000001900").frame.has_value());    // DLC 9 in a classic frame
  zexpect_false(SlcanMedia::ParseLine("T000000012zz00").frame.has_value());  // not hex
  zexpect_equal(SlcanMedia::ParseLine("T000000012zz00").reply, std::string_view{"\a"});
  zexpect_false(SlcanMedia::ParseLine("t1231AA").frame.has_value());  // standard IDs are not Cyphal
}

}  // namespace

ZTEST_SUITE(sim_slcan, nullptr, nullptr, nullptr, nullptr, nullptr);
