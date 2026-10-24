/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file cymon_test.cpp
 *    @brief ztest suite of cymon-lib in the Zephyr build: limits, one capture, DSDL.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include <cymon/GetBufferInfo_0_1.h>
#include <cymon/ReadSamples_0_1.h>
#include <zephyr/ztest.h>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include "cymon/device.hpp"
#include "unimoc_test.hpp"

namespace {

using cymon::CaptureState;
using cymon::Device;
using cymon::ErrorCode;
using cymon::TriggerMode;

/// Tick period of the control cycle at 32 kHz PWM [us].
constexpr float kTickUs = 62.5F;

/// The signals of the fake tick: a ramp and its negative.
struct Signals {
  float ramp{0.0F};
  float negative{0.0F};
};

Signals g_signals;

/// The device is large: keep it off the ztest stack.
Device& MakeDevice() {
  static std::optional<Device> storage;
  Device& device = storage.emplace();
  g_signals = Signals{};
  zassert_true(device.RegisterVariable("ramp", "A", &g_signals.ramp));
  zassert_true(device.RegisterVariable("negative", "V", &g_signals.negative));
  device.SetTickPeriod(kTickUs);
  return device;
}

/// One sampling period: the "control cycle" writes the signals, then samples them.
void FakeTick(Device& device, const float value) {
  g_signals.ramp = value;
  g_signals.negative = -value;
  (void)device.Tick();
}

}  // namespace

ZTEST(cymon_lib, test_limits_come_from_kconfig) {
  zexpect_equal(cymon::kMaxBufferBytes, static_cast<std::size_t>(CONFIG_UNIMOC_TRACE_BUFFER_BYTES));
  zexpect_equal(cymon::kMaxVars, static_cast<std::size_t>(CONFIG_UNIMOC_TRACE_VARS));
  zexpect_equal(cymon::kMaxChannels, static_cast<std::size_t>(CONFIG_UNIMOC_TRACE_CHANNELS));

  Device& device = MakeDevice();
  const Device::BufferInfo kInfo = device.HandleGetBufferInfo();
  zexpect_equal(kInfo.buffer_bytes, 256U);
  zexpect_equal(kInfo.max_channels, 2U);
  zexpect_equal(kInfo.max_frames, 64U);
  UNIMOC_EXPECT_FLOAT_EQ(kInfo.tick_period_us, kTickUs);
  zexpect_equal(device.GetVariable(1U).name, "negative");
  zexpect_true(device.GetVariable(2U).name.empty());

  // The registry holds CONFIG_UNIMOC_TRACE_VARS variables.
  zexpect_true(device.RegisterVariable("third", "", &g_signals.ramp));
  zexpect_true(device.RegisterVariable("fourth", "", &g_signals.ramp));
  zexpect_false(device.RegisterVariable("fifth", "", &g_signals.ramp));
}

ZTEST(cymon_lib, test_capture_with_a_rising_trigger) {
  Device& device = MakeDevice();
  Device::CaptureConfig config{};
  config.channel_ids = {0U, 1U};
  config.num_channels = 2U;
  config.sample_period_us = 2.0F * kTickUs;  // every second tick
  config.num_samples = 8U;
  config.pretrigger_samples = 2U;
  const Device::SetupCaptureResult kSetup = device.HandleSetupCapture(config);
  zassert_equal(kSetup.error, ErrorCode::kNone);
  UNIMOC_EXPECT_FLOAT_EQ(kSetup.actual_sample_period_us, 125.0F);

  // 64 frames of one channel are 32 of two: 40 do not fit.
  Device::CaptureConfig too_long = config;
  too_long.num_samples = 40U;
  zexpect_equal(device.HandleSetupCapture(too_long).error, ErrorCode::kBufferTooSmall);
  zassert_equal(device.HandleSetupCapture(config).error, ErrorCode::kNone);

  zassert_equal(device.HandleArmTrigger({.arm = true,
                                         .source_variable_id = 0U,
                                         .mode = TriggerMode::kRising,
                                         .level = 10.5F,
                                         .hysteresis_band = 0.0F}),
                ErrorCode::kNone);
  zexpect_equal(device.State(), CaptureState::kArmed);
  for (int tick = 0; tick < 40 && device.State() != CaptureState::kComplete; ++tick) {
    FakeTick(device, static_cast<float>(tick));
  }
  zassert_equal(device.State(), CaptureState::kComplete);

  Device::TriggerEvent event{};
  zassert_true(device.TakeTriggerEvent(&event));
  zexpect_false(device.TakeTriggerEvent(&event));
  zexpect_equal(event.source_variable_id, 0U);
  // Samples at odd ticks (every second): the first above 10.5 is 11.
  UNIMOC_EXPECT_FLOAT_EQ(event.trigger_value, 11.0F);

  const Device::ReadSamplesResult kRead = device.HandleReadSamples(0U, 8U);
  zexpect_equal(kRead.state, CaptureState::kComplete);
  zexpect_equal(kRead.num_samples, 8U);
  zexpect_true(kRead.end_of_data);
  zassert_equal(kRead.sample_count, 16U);
  // Frame 2 (pretrigger_samples) is the trigger sample; frames are 2 ticks apart.
  for (std::size_t frame = 0U; frame < 8U; ++frame) {
    const float kExpected = 11.0F + (2.0F * (static_cast<float>(frame) - 2.0F));
    UNIMOC_EXPECT_FLOAT_EQ(kRead.samples[2U * frame], kExpected);
    UNIMOC_EXPECT_FLOAT_EQ(kRead.samples[(2U * frame) + 1U], -kExpected);
  }
}

ZTEST(cymon_lib, test_requested_trigger_and_disarm) {
  Device& device = MakeDevice();
  Device::CaptureConfig config{};
  config.channel_ids = {1U, 0U};
  config.num_channels = 1U;
  config.sample_period_us = kTickUs;
  config.num_samples = 4U;
  config.pretrigger_samples = 1U;
  zassert_equal(device.HandleSetupCapture(config).error, ErrorCode::kNone);
  zassert_equal(device.HandleArmTrigger(
                    {.arm = true, .source_variable_id = 0U, .mode = TriggerMode::kFalling}),
                ErrorCode::kNone);
  // Setup is refused while armed.
  zexpect_equal(device.HandleSetupCapture(config).error, ErrorCode::kAlreadyCapturing);

  FakeTick(device, 1.0F);
  FakeTick(device, 2.0F);
  zexpect_equal(device.State(), CaptureState::kArmed);
  device.RequestTrigger();  // e.g. a cymon.TriggerEvent of another node
  for (int tick = 3; tick < 10; ++tick) {
    FakeTick(device, static_cast<float>(tick));
  }
  zassert_equal(device.State(), CaptureState::kComplete);
  const Device::ReadSamplesResult kRead = device.HandleReadSamples(0U, 4U);
  zassert_equal(kRead.sample_count, 4U);
  UNIMOC_EXPECT_FLOAT_EQ(kRead.samples[0], -2.0F);
  UNIMOC_EXPECT_FLOAT_EQ(kRead.samples[1], -3.0F);  // the trigger sample
  UNIMOC_EXPECT_FLOAT_EQ(kRead.samples[3], -5.0F);

  zexpect_equal(device.HandleArmTrigger({.arm = false}), ErrorCode::kNone);
  zexpect_equal(device.State(), CaptureState::kConfigured);
}

ZTEST(cymon_lib, test_dsdl_serialization_is_generated) {
  // A full ReadSamples page fits its extent; the buffer info round-trips.
  static cymon_ReadSamples_Response_0_1 page{};
  page.state.value = cymon_CaptureState_0_1_COMPLETE;
  page.samples.count = cymon_ReadSamples_Response_0_1_samples_ARRAY_CAPACITY_;
  zexpect_equal(page.samples.count, cymon::kMaxReadSamples);
  std::array<uint8_t, cymon_ReadSamples_Response_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_> buffer{};
  std::size_t size = buffer.size();
  zassert_equal(cymon_ReadSamples_Response_0_1_serialize_(&page, buffer.data(), &size), 0);
  zexpect_true(size <= cymon_ReadSamples_Response_0_1_EXTENT_BYTES_);

  cymon_GetBufferInfo_Response_0_1 info{};
  info.buffer_bytes = 8192U;
  info.max_channels = 8U;
  info.tick_period_us = kTickUs;
  std::array<uint8_t, cymon_GetBufferInfo_Response_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_> info_buffer{};
  size = info_buffer.size();
  zassert_equal(cymon_GetBufferInfo_Response_0_1_serialize_(&info, info_buffer.data(), &size), 0);
  cymon_GetBufferInfo_Response_0_1 decoded{};
  zassert_equal(cymon_GetBufferInfo_Response_0_1_deserialize_(&decoded, info_buffer.data(), &size), 0);
  zexpect_equal(decoded.buffer_bytes, 8192U);
  zexpect_equal(decoded.max_channels, 8U);
  UNIMOC_EXPECT_FLOAT_EQ(decoded.tick_period_us, kTickUs);
}

ZTEST_SUITE(cymon_lib, nullptr, nullptr, nullptr, nullptr, nullptr);
