/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file edge_uart_decoder_test.cpp
 *    @brief ztest coverage of the software UART receiver with generated edge streams.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "edge_uart_decoder.hpp"
#include <zephyr/ztest.h>
#include <cstdint>
#include <vector>
#include "kunteng_protocol.hpp"
#include "unimoc_test.hpp"

namespace unimoc::ebike::test {

namespace {

constexpr uint32_t kTicksPerBit = 17'708U;  // 170 MHz at 9600 baud

/// Edges of a serial line for @p bytes starting at @p start; bit times are scaled by @p baud_error.
std::vector<UartEdge> Line(const std::vector<uint8_t>& bytes,
                           const uint32_t start,
                           const float baud_error = 1.0F,
                           const uint32_t gap_bits = 0U) {
  std::vector<UartEdge> edges;
  const float kBit = static_cast<float>(kTicksPerBit) * baud_error;
  bool level = true;
  float now = static_cast<float>(start);
  const auto set_level = [&](const bool next) {
    if (next != level) {
      level = next;
      edges.push_back({static_cast<uint32_t>(now), level});
    }
  };
  for (const uint8_t byte : bytes) {
    set_level(false);  // start
    now += kBit;
    for (unsigned i = 0U; i < 8U; ++i) {
      set_level(((byte >> i) & 1U) != 0U);
      now += kBit;
    }
    set_level(true);  // stop
    now += kBit * static_cast<float>(1U + gap_bits);
  }
  return edges;
}

std::vector<uint8_t> Decode(EdgeUartDecoder& decoder,
                            const std::vector<UartEdge>& edges,
                            const uint32_t end) {
  for (const UartEdge edge : edges) {
    decoder.Push(edge);
  }
  decoder.Poll(end);
  std::vector<uint8_t> bytes;
  uint8_t byte = 0U;
  while (decoder.Pop(byte)) {
    bytes.push_back(byte);
  }
  return bytes;
}

}  // namespace

ZTEST(edge_uart_decoder, test_decodes_every_byte_value) {
  std::vector<uint8_t> all;
  for (unsigned value = 0U; value < 256U; ++value) {
    all.push_back(static_cast<uint8_t>(value));
  }
  EdgeUartDecoder decoder(kTicksPerBit);
  // The FIFO holds 32 bytes: decode in chunks.
  uint32_t start = 1000U;
  for (std::size_t offset = 0U; offset < all.size(); offset += 16U) {
    const std::vector<uint8_t> chunk(all.begin() + static_cast<long>(offset),
                                     all.begin() + static_cast<long>(offset + 16U));
    const uint32_t kEnd = start + (16U * 10U + 2U) * kTicksPerBit;
    zexpect_true(Decode(decoder, Line(chunk, start), kEnd) == chunk,
                 "chunk at %u",
                 static_cast<unsigned>(offset));
    start = kEnd;
  }
  zexpect_equal(decoder.FramingErrors(), 0U);
}

ZTEST(edge_uart_decoder, test_poll_completes_a_frame_that_ends_in_idle_level_bits) {
  // 0x01 ends in seven zero... the last data bits and the stop bit are high/low mixed; 0x00 ends low then
  // stop high: the stop bit is a final edge, but 0xFF has no edge after the start bit at all.
  EdgeUartDecoder decoder(kTicksPerBit);
  const std::vector<uint8_t> kBytes{0xFF};
  const auto kEdges = Line(kBytes, 5000U);
  for (const UartEdge edge : kEdges) {
    decoder.Push(edge);
  }
  uint8_t byte = 0U;
  zexpect_false(decoder.Pop(byte), "the frame is not complete before its bits have passed");
  decoder.Poll(5000U + 11U * kTicksPerBit);
  zexpect_true(decoder.Pop(byte));
  zexpect_equal(byte, 0xFFU);
}

ZTEST(edge_uart_decoder, test_back_to_back_bytes_and_the_counter_wrap) {
  const std::vector<uint8_t> kBytes{0x41, 0x0C, 0x00, 0xFF, 0x55, 0xAA};
  EdgeUartDecoder decoder(kTicksPerBit);
  const uint32_t kStart = 0xFFFFFFFFU - 3U * kTicksPerBit;  // the frame crosses the wrap
  zexpect_true(Decode(decoder, Line(kBytes, kStart), kStart + 70U * kTicksPerBit) == kBytes);
}

ZTEST(edge_uart_decoder, test_baud_rate_tolerance) {
  const std::vector<uint8_t> kBytes{0x41, 0x0C, 0x24, 0x01, 0xF4, 0x00, 0xCC, 0x20, 0x28, 0x19, 0x00, 0x00};
  for (const float error : {0.97F, 0.99F, 1.0F, 1.01F, 1.03F}) {
    EdgeUartDecoder decoder(kTicksPerBit);
    zexpect_true(Decode(decoder, Line(kBytes, 2000U, error), 2000U + 140U * kTicksPerBit * 2U) == kBytes,
                 "error %f",
                 static_cast<double>(error));
  }
}

ZTEST(edge_uart_decoder, test_edge_jitter) {
  const std::vector<uint8_t> kBytes{0x37, 0xC5, 0x0E, 0xA0};
  auto edges = Line(kBytes, 4000U);
  uint32_t seed = 7U;
  for (UartEdge& edge : edges) {
    seed = (seed * 1664525U) + 1013904223U;
    // +-3 % of a bit (a few microseconds of interrupt latency jitter).
    edge.ticks += (seed >> 16U) % (kTicksPerBit / 16U);
  }
  EdgeUartDecoder decoder(kTicksPerBit);
  zexpect_true(Decode(decoder, edges, 4000U + 60U * kTicksPerBit) == kBytes);
}

ZTEST(edge_uart_decoder, test_glitches_are_ignored) {
  EdgeUartDecoder decoder(kTicksPerBit);
  // A low pulse of a fifth of a bit on the idle line: no byte.
  decoder.Push({1000U, false});
  decoder.Push({1000U + kTicksPerBit / 5U, true});
  decoder.Poll(1000U + 12U * kTicksPerBit);
  uint8_t byte = 0U;
  zexpect_false(decoder.Pop(byte));
  zexpect_equal(decoder.FramingErrors(), 0U);
  // The next real byte is decoded.
  zexpect_true(Decode(decoder, Line({0x5A}, 40U * kTicksPerBit), 60U * kTicksPerBit) ==
               std::vector<uint8_t>{0x5A});
}

ZTEST(edge_uart_decoder, test_a_missing_stop_bit_is_a_framing_error) {
  EdgeUartDecoder decoder(kTicksPerBit);
  // Start, eight zero data bits, and the line stays low through the stop bit.
  decoder.Push({1000U, false});
  decoder.Poll(1000U + 12U * kTicksPerBit);
  uint8_t byte = 0U;
  zexpect_false(decoder.Pop(byte));
  zexpect_equal(decoder.FramingErrors(), 1U);
  // The line recovers with the next start.
  decoder.Push({1000U + 14U * kTicksPerBit, true});
  zexpect_true(Decode(decoder, Line({0x33}, 1000U + 20U * kTicksPerBit), 1000U + 40U * kTicksPerBit) ==
               std::vector<uint8_t>{0x33});
}

ZTEST(edge_uart_decoder, test_fifo_overrun_is_counted) {
  EdgeUartDecoder decoder(kTicksPerBit);
  const std::vector<uint8_t> kBytes(EdgeUartDecoder::kFifoSize + 3U, 0x5AU);
  const auto kEdges = Line(kBytes, 1000U);
  for (const UartEdge edge : kEdges) {
    decoder.Push(edge);
  }
  decoder.Poll(1000U + (kBytes.size() + 1U) * 10U * kTicksPerBit);
  zexpect_equal(decoder.Overruns(), 3U);
}

ZTEST(edge_uart_decoder, test_display_request_through_the_decoder) {
  // A whole display frame, as it comes off the PB15 line, ends up in the Kunteng decoder.
  kunteng::DisplayRequest request;
  request.assist_level = 4U;
  request.light = true;
  request.max_speed = 25U;
  const auto kFrame = kunteng::EncodeDisplayRequest(request);
  const std::vector<uint8_t> kBytes(kFrame.begin(), kFrame.end());
  EdgeUartDecoder uart(kTicksPerBit);
  kunteng::DisplayDecoder decoder;
  kunteng::DisplayRequest decoded;
  int frames = 0;
  for (const uint8_t byte : Decode(uart, Line(kBytes, 3000U, 1.0F, 1U), 3000U + 150U * kTicksPerBit)) {
    if (decoder.Push(byte, decoded)) {
      ++frames;
    }
  }
  zexpect_equal(frames, 1);
  zexpect_equal(decoded.assist_level, 4U);
  zexpect_equal(decoded.max_speed, 25U);
}

}  // namespace unimoc::ebike::test

ZTEST_SUITE(edge_uart_decoder, nullptr, nullptr, nullptr, nullptr, nullptr);
