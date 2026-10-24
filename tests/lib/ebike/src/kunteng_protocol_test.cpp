/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file kunteng_protocol_test.cpp
 *    @brief ztest coverage of the Kunteng display frame codec.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "kunteng_protocol.hpp"
#include <zephyr/ztest.h>
#include <array>
#include <cstdint>
#include <vector>
#include "unimoc_test.hpp"

namespace unimoc::ebike::kunteng::test {

using namespace unit;

namespace {

DisplayRequest SampleRequest() {
  DisplayRequest request;
  request.assist_level = 3U;
  request.light = true;
  request.wheel_size_code = 0x12U;
  request.max_speed = 25U;
  request.motor_characteristic = 4U;
  request.power_assist_mode = true;
  request.max_current_code = 9U;
  return request;
}

/// Pushes all bytes; returns the number of decoded frames, the last one in @p last.
int PushAll(DisplayDecoder& decoder, const std::vector<uint8_t>& bytes, DisplayRequest& last) {
  int frames = 0;
  for (const uint8_t kByte : bytes) {
    if (decoder.Push(kByte, last)) {
      ++frames;
    }
  }
  return frames;
}

std::vector<uint8_t> Bytes(const std::array<uint8_t, kDisplayFrameSize>& frame) {
  return {frame.begin(), frame.end()};
}

}  // namespace

ZTEST(kunteng_protocol, test_controller_frame_layout) {
  ControllerStatus status;
  status.battery_level = BatteryLevel::kThreeBars;
  status.nominal_voltage = 36U;
  status.wheel_period = 500.0_ms;
  status.brake = true;
  status.battery_current = 10.0_A;
  status.motor_temperature = 40.0_degC;
  const std::array<uint8_t, kControllerFrameSize>
      kExpected{0x41, 0x0C, 0x24, 0x01, 0xF4, 0x00, 0xCC, 0x20, 0x28, 0x19, 0x00, 0x00};
  zexpect_mem_equal(EncodeControllerFrame(status).data(), kExpected.data(), kControllerFrameSize);
}

ZTEST(kunteng_protocol, test_controller_frame_modes_and_limits) {
  ControllerStatus status;
  status.throttle = true;
  status.cruise = true;
  status.pas = true;
  status.wheel_period = 70000.0_ms;       // beyond 16 bit
  status.battery_current = 100.0_A;       // beyond 8 bit in quarter amps
  status.motor_temperature = -40.0_degC;  // below the offset
  const auto kFrame = EncodeControllerFrame(status);
  zexpect_equal(kFrame[7], 0x1AU);
  zexpect_equal(kFrame[3], 0xFFU);
  zexpect_equal(kFrame[4], 0xFFU);
  zexpect_equal(kFrame[8], 0xFFU);
  zexpect_equal(kFrame[9], 0x00U);
  uint8_t sum = 0U;
  for (std::size_t i = 1U; i < kFrame.size(); ++i) {
    sum = static_cast<uint8_t>(sum ^ kFrame[i]);
  }
  zexpect_equal(sum, 0U, "xor over bytes 1..11 including the checksum is zero");
}

ZTEST(kunteng_protocol, test_request_round_trip) {
  DisplayDecoder decoder;
  DisplayRequest decoded;
  zexpect_equal(PushAll(decoder, Bytes(EncodeDisplayRequest(SampleRequest())), decoded), 1);
  const DisplayRequest kWanted = SampleRequest();
  zexpect_equal(decoded.assist_level, kWanted.assist_level);
  zexpect_equal(decoded.light, kWanted.light);
  zexpect_equal(decoded.wheel_size_code, kWanted.wheel_size_code);
  zexpect_equal(decoded.max_speed, kWanted.max_speed);
  zexpect_equal(decoded.motor_characteristic, kWanted.motor_characteristic);
  zexpect_equal(decoded.power_assist_mode, kWanted.power_assist_mode);
  zexpect_equal(decoded.max_current_code, kWanted.max_current_code);
}

ZTEST(kunteng_protocol, test_max_speed_over_the_whole_range) {
  for (uint8_t speed = 10U; speed <= 63U; ++speed) {
    DisplayRequest request = SampleRequest();
    request.max_speed = speed;
    DisplayDecoder decoder;
    DisplayRequest decoded;
    zexpect_equal(PushAll(decoder, Bytes(EncodeDisplayRequest(request)), decoded), 1);
    zexpect_equal(decoded.max_speed, speed, "speed %u", static_cast<unsigned>(speed));
  }
}

ZTEST(kunteng_protocol, test_both_end_markers_are_accepted) {
  auto frame = EncodeDisplayRequest(SampleRequest());
  frame[11] = 0x32U;
  // The marker is part of the checksum: recompute it.
  uint8_t sum = 0U;
  for (std::size_t i = 0U; i < frame.size(); ++i) {
    sum = static_cast<uint8_t>(sum ^ (i == 5U ? 0U : frame[i]));
  }
  frame[5] = static_cast<uint8_t>(sum ^ 10U);
  DisplayDecoder decoder;
  DisplayRequest decoded;
  zexpect_equal(PushAll(decoder, Bytes(frame), decoded), 1);
}

ZTEST(kunteng_protocol, test_decoder_resynchronizes_after_noise_and_decodes_back_to_back_frames) {
  std::vector<uint8_t> stream = {0x00, 0x0E, 0x37, 0xFF, 0x12, 0x0E, 0x37};  // junk, including end-like bytes
  const auto kFrame = Bytes(EncodeDisplayRequest(SampleRequest()));
  stream.insert(stream.end(), kFrame.begin(), kFrame.end());
  stream.push_back(0x55);
  stream.insert(stream.end(), kFrame.begin(), kFrame.end());
  stream.insert(stream.end(), kFrame.begin(), kFrame.end());
  DisplayDecoder decoder;
  DisplayRequest decoded;
  zexpect_equal(PushAll(decoder, stream, decoded), 3);
}

ZTEST(kunteng_protocol, test_bad_checksum_is_rejected) {
  auto frame = EncodeDisplayRequest(SampleRequest());
  frame[1] ^= 0x01U;  // a bit flips on the line
  DisplayDecoder decoder;
  DisplayRequest decoded;
  zexpect_equal(PushAll(decoder, Bytes(frame), decoded), 0);
  zexpect_equal(decoder.ChecksumSeed(), 10U);
}

ZTEST(kunteng_protocol, test_checksum_constant_is_learned_after_two_matching_frames) {
  DisplayDecoder decoder;
  DisplayRequest decoded;
  const auto kOther = Bytes(EncodeDisplayRequest(SampleRequest(), 23U));
  zexpect_equal(PushAll(decoder, kOther, decoded), 0);
  zexpect_equal(decoder.ChecksumSeed(), 10U);
  // The second frame with the same constant is adopted and decoded.
  zexpect_equal(PushAll(decoder, kOther, decoded), 1);
  zexpect_equal(decoder.ChecksumSeed(), 23U);
  zexpect_equal(decoded.assist_level, 3U);
}

ZTEST(kunteng_protocol, test_one_odd_frame_does_not_change_the_constant) {
  DisplayDecoder decoder;
  DisplayRequest decoded;
  zexpect_equal(PushAll(decoder, Bytes(EncodeDisplayRequest(SampleRequest(), 23U)), decoded), 0);
  zexpect_equal(PushAll(decoder, Bytes(EncodeDisplayRequest(SampleRequest(), 10U)), decoded), 1);
  zexpect_equal(decoder.ChecksumSeed(), 10U);
  // A different odd constant after it starts the count again.
  zexpect_equal(PushAll(decoder, Bytes(EncodeDisplayRequest(SampleRequest(), 31U)), decoded), 0);
  zexpect_equal(PushAll(decoder, Bytes(EncodeDisplayRequest(SampleRequest(), 40U)), decoded), 0);
  zexpect_equal(decoder.ChecksumSeed(), 10U);
}

ZTEST(kunteng_protocol, test_undecoded_parameters_survive_in_the_raw_frame) {
  DisplayRequest request = SampleRequest();
  request.raw[0] = 0x2AU;
  request.raw[6] = 0x15U;
  request.raw[9] = 0x07U;
  DisplayDecoder decoder;
  DisplayRequest decoded;
  zexpect_equal(PushAll(decoder, Bytes(EncodeDisplayRequest(request)), decoded), 1);
  zexpect_equal(decoded.raw[0], 0x2AU);
  zexpect_equal(decoded.raw[6], 0x15U);
  zexpect_equal(decoded.raw[9], 0x07U);
}

}  // namespace unimoc::ebike::kunteng::test

ZTEST_SUITE(kunteng_protocol, nullptr, nullptr, nullptr, nullptr, nullptr);
