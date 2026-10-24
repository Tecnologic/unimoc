/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file kunteng_protocol.hpp
 *    @brief Frame codec of the Kunteng display protocol (KT-LCD3/5/8): controller status and display request.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include "units.hpp"

/**
 * @namespace unimoc::ebike::kunteng Kunteng display protocol namespace
 */
namespace unimoc::ebike::kunteng {

using namespace unit;

/// Bytes of the status frame from the controller to the display.
inline constexpr std::size_t kControllerFrameSize = 12U;
/// Bytes of the request frame from the display to the controller.
inline constexpr std::size_t kDisplayFrameSize = 13U;

/// Battery level codes of the display (a bar is four counts; 3 blinks "empty", 1 is below empty).
enum class BatteryLevel : uint8_t {
  kBelowEmpty = 1,
  kEmpty = 3,
  kOneBar = 4,
  kTwoBars = 8,
  kThreeBars = 12,
  kFourBars = 16,
};

/// Content of the status frame (controller to display).
struct ControllerStatus {
  /// Battery bars of the display.
  BatteryLevel battery_level{BatteryLevel::kFourBars};
  /// Nominal battery voltage in volts as the display shows its scale (24, 36, 48); the display's own
  /// setting has to match.
  uint8_t nominal_voltage{36U};
  /// Time of one wheel revolution; the display derives the speed from it with its own wheel size.
  Time wheel_period;
  /// Error code of the display (0: none).
  uint8_t error_code{0U};
  bool throttle{false};
  bool cruise{false};
  bool pas{false};
  bool brake{false};
  /// Battery current; the display shows the power from it and the nominal voltage.
  Current battery_current;
  /// Motor temperature.
  Temperature motor_temperature;
};

/// Content of the request frame (display to controller).
struct DisplayRequest {
  /// Assist level selected on the display (0 to 7; the KT-LCD8 uses 0 to 5).
  uint8_t assist_level{0U};
  /// The light is switched on.
  bool light{false};
  /// Raw wheel size code of the display (5 bit); the mapping to a diameter is not decoded.
  uint8_t wheel_size_code{0U};
  /// Speed limit in km/h.
  uint8_t max_speed{0U};
  /// Motor characteristic parameter (byte 3: for example the number of magnets or a gear ratio).
  uint8_t motor_characteristic{0U};
  /// The display asks for power assist control mode.
  bool power_assist_mode{false};
  /// Maximum controller current code of the display (4 bit).
  uint8_t max_current_code{0U};
  /// The frame as received; the display parameters that are not decoded stay available (for storing).
  std::array<uint8_t, kDisplayFrameSize> raw{};
};

namespace detail {

inline constexpr uint8_t kControllerStart = 65U;
inline constexpr uint8_t kDisplayEnd = 0x0EU;
inline constexpr uint8_t kDisplayEndMarkerA = 0x32U;
inline constexpr uint8_t kDisplayEndMarkerB = 0x37U;
inline constexpr std::size_t kDisplayChecksumIndex = 5U;
inline constexpr uint8_t kDefaultChecksumSeed = 10U;
inline constexpr uint8_t kMaxChecksumSeed = 50U;
inline constexpr float kCurrentCountsPerAmp = 4.0F;
inline constexpr Temperature kTemperatureOffset = 15.0_degC;

inline constexpr uint8_t kThrottleBit = 1U << 1U;
inline constexpr uint8_t kCruiseBit = 1U << 3U;
inline constexpr uint8_t kPasBit = 1U << 4U;
inline constexpr uint8_t kBrakeBit = 1U << 5U;

template <typename Frame>
[[nodiscard]] constexpr uint8_t XorExcept(const Frame& frame, const std::size_t skipped) noexcept {
  uint8_t sum = 0U;
  for (std::size_t i = 0U; i < frame.size(); ++i) {
    if (i != skipped) {
      sum = static_cast<uint8_t>(sum ^ frame[i]);
    }
  }
  return sum;
}

[[nodiscard]] constexpr uint8_t ClampByte(const float value) noexcept {
  return static_cast<uint8_t>(std::clamp(value + 0.5F, 0.0F, 255.0F));
}

}  // namespace detail

/**
 * @brief Builds the status frame the display expects as the answer to each of its requests.
 *
 * Layout (byte 0 is 65): 1 battery level, 2 nominal voltage, 3..4 wheel period in ms (big endian), 5 error,
 * 6 checksum (xor of all other bytes except byte 0), 7 mode bits (throttle 2, cruise 8, PAS 16, brake 32),
 * 8 battery current in quarter amps, 9 motor temperature in degrees Celsius minus 15, 10..11 zero.
 *
 * @param status Values to report.
 * @return The 12 bytes to send.
 */
[[nodiscard]] inline std::array<uint8_t, kControllerFrameSize> EncodeControllerFrame(
    const ControllerStatus& status) noexcept {
  std::array<uint8_t, kControllerFrameSize> frame{};
  const Ratio kPeriodMs = status.wheel_period / 1.0_ms;  // converts the millisecond quotient to a plain ratio
  const auto kPeriod = static_cast<uint16_t>(std::clamp(kPeriodMs.Value(), 0.0F, 65535.0F));
  frame[0] = detail::kControllerStart;
  frame[1] = static_cast<uint8_t>(status.battery_level);
  frame[2] = status.nominal_voltage;
  frame[3] = static_cast<uint8_t>(kPeriod >> 8U);
  frame[4] = static_cast<uint8_t>(kPeriod & 0xFFU);
  frame[5] = status.error_code;
  uint8_t mode = 0U;
  mode = static_cast<uint8_t>(mode | (status.throttle ? detail::kThrottleBit : 0U));
  mode = static_cast<uint8_t>(mode | (status.cruise ? detail::kCruiseBit : 0U));
  mode = static_cast<uint8_t>(mode | (status.pas ? detail::kPasBit : 0U));
  mode = static_cast<uint8_t>(mode | (status.brake ? detail::kBrakeBit : 0U));
  frame[7] = mode;
  frame[8] = detail::ClampByte(((status.battery_current / 1.0_A) * detail::kCurrentCountsPerAmp).Value());
  frame[9] = detail::ClampByte(((status.motor_temperature - detail::kTemperatureOffset) / 1.0_degC).Value());
  frame[6] = detail::XorExcept(frame, 0U);  // byte 6 is still zero here
  return frame;
}

/**
 * @brief Builds a request frame, as a display would send it (for tests and a simulated display).
 * @param request Fields to encode; the raw bytes of the request are the starting point, so parameters that
 * are not decoded stay as they are.
 * @param checksum_seed Constant that the checksum is xored with (10 for the displays seen so far).
 * @return The 13 bytes of the frame.
 */
[[nodiscard]] inline std::array<uint8_t, kDisplayFrameSize> EncodeDisplayRequest(
    const DisplayRequest& request, const uint8_t checksum_seed = detail::kDefaultChecksumSeed) noexcept {
  std::array<uint8_t, kDisplayFrameSize> frame = request.raw;
  // The speed is (10 + field) | (bit 5 of byte 4): up to 41 km/h in the field alone, above it with bit 5.
  const uint8_t kSpeed = std::clamp<uint8_t>(request.max_speed, 10U, 63U);
  const bool kSpeedBit = kSpeed > 41U;
  const auto kSpeedField = static_cast<uint8_t>((kSpeedBit ? (kSpeed & 0x1FU) : kSpeed) - 10U);
  frame[1] = static_cast<uint8_t>((request.assist_level & 7U) | (request.light ? 0x80U : 0U));
  frame[2] = static_cast<uint8_t>(((request.wheel_size_code >> 2U) & 7U) | (kSpeedField << 3U));
  frame[3] = request.motor_characteristic;
  frame[4] = static_cast<uint8_t>((frame[4] & 0x17U) | ((request.wheel_size_code & 3U) << 6U) |
                                  (kSpeedBit ? 0x20U : 0U) | (request.power_assist_mode ? 0x08U : 0U));
  frame[7] = static_cast<uint8_t>((frame[7] & 0xF0U) | (request.max_current_code & 0x0FU));
  frame[11] = detail::kDisplayEndMarkerB;
  frame[12] = detail::kDisplayEnd;
  frame[detail::kDisplayChecksumIndex] =
      static_cast<uint8_t>(detail::XorExcept(frame, detail::kDisplayChecksumIndex) ^ checksum_seed);
  return frame;
}

/**
 * @brief Finds and decodes display request frames in the byte stream of the UART.
 *
 * The display polls the controller about every 100 ms with a 13-byte frame; the controller answers each valid
 * request with the status frame. The decoder needs no frame gaps or DMA tricks: it keeps the last 13 bytes
 * and accepts them as a frame when the end bytes (0x32 or 0x37, then 0x0E) and the checksum are right, which
 * also resynchronizes after noise or a start in the middle of a frame.
 *
 * The checksum is the xor of all bytes except byte 5, xored with a constant that differs between display
 * firmwares (10 for most). The decoder starts with 10 and adopts another constant (up to 50) only after two
 * frames in a row show the same one, so a single corrupt frame cannot change it.
 */
class DisplayDecoder {
 public:
  /**
   * @brief Feeds one received byte.
   * @param byte Byte from the UART.
   * @param request Receives the decoded frame when this call returns true.
   * @return true when @p byte completed a valid request frame.
   */
  bool Push(const uint8_t byte, DisplayRequest& request) noexcept {
    std::copy(window_.begin() + 1, window_.end(), window_.begin());
    window_.back() = byte;
    count_ = std::min(count_ + 1U, window_.size());
    if (count_ < window_.size() || !HasEndMarker()) {
      return false;
    }
    const auto kSeed = static_cast<uint8_t>(detail::XorExcept(window_, detail::kDisplayChecksumIndex) ^
                                            window_[detail::kDisplayChecksumIndex]);
    if (kSeed != seed_) {
      LearnSeed(kSeed);
      if (kSeed != seed_) {
        return false;
      }
    }
    candidate_count_ = 0U;
    request = Decode(window_);
    count_ = 0U;  // the next frame starts after this one
    return true;
  }

  /// Checksum constant in use.
  [[nodiscard]] uint8_t ChecksumSeed() const noexcept { return seed_; }

  /**
   * @brief Decodes a frame without checking it.
   * @param frame The 13 bytes.
   * @return The fields of the frame.
   */
  [[nodiscard]] static DisplayRequest Decode(const std::array<uint8_t, kDisplayFrameSize>& frame) noexcept {
    DisplayRequest request;
    request.assist_level = static_cast<uint8_t>(frame[1] & 7U);
    request.light = (frame[1] & 0x80U) != 0U;
    request.wheel_size_code = static_cast<uint8_t>(((frame[4] & 0xC0U) >> 6U) | ((frame[2] & 7U) << 2U));
    request.max_speed = static_cast<uint8_t>((10U + ((frame[2] & 0xF8U) >> 3U)) | (frame[4] & 0x20U));
    request.motor_characteristic = frame[3];
    request.power_assist_mode = (frame[4] & 0x08U) != 0U;
    request.max_current_code = static_cast<uint8_t>(frame[7] & 0x0FU);
    request.raw = frame;
    return request;
  }

 private:
  [[nodiscard]] bool HasEndMarker() const noexcept {
    const uint8_t kMarker = window_[window_.size() - 2U];
    return (kMarker == detail::kDisplayEndMarkerA || kMarker == detail::kDisplayEndMarkerB) &&
           window_.back() == detail::kDisplayEnd;
  }

  void LearnSeed(const uint8_t seed) noexcept {
    if (seed > detail::kMaxChecksumSeed) {
      candidate_count_ = 0U;
      return;
    }
    if (seed == candidate_ && candidate_count_ > 0U) {
      ++candidate_count_;
    } else {
      candidate_ = seed;
      candidate_count_ = 1U;
    }
    if (candidate_count_ >= kSeedFrames) {
      seed_ = seed;
      candidate_count_ = 0U;
    }
  }

  static constexpr unsigned kSeedFrames = 2U;

  std::array<uint8_t, kDisplayFrameSize> window_{};
  std::size_t count_{0U};
  uint8_t seed_{detail::kDefaultChecksumSeed};
  uint8_t candidate_{0U};
  unsigned candidate_count_{0U};
};

}  // namespace unimoc::ebike::kunteng
