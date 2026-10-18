/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file dead_time.hpp
 *    @brief DTG encoding of the STM32 advanced timer dead time.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include <optional>

namespace unimoc::hardware::foc_stm32g4 {

/// Longest dead time the DTG field can encode, in ticks of the dead time clock t_DTS.
inline constexpr uint32_t kMaxDeadTimeTicks = 63U * 16U;

/// Converts a dead time in ticks of t_DTS into the 8 bit DTG field of TIM_BDTR.
///
/// The field has four ranges with growing step size (RM0440, TIMx_BDTR):
///   0xx xxxx: DT = DTG[7:0] * t_DTS                  0 .. 127
///   10x xxxx: DT = (64 + DTG[5:0]) * 2 * t_DTS     128 .. 254
///   110 xxxx: DT = (32 + DTG[4:0]) * 8 * t_DTS     256 .. 504
///   111 xxxx: DT = (32 + DTG[4:0]) * 16 * t_DTS    512 .. 1008
/// The result never undercuts the requested time: it is rounded up to the next
/// step. Returns nullopt if the time is longer than the field can encode.
[[nodiscard]] constexpr std::optional<uint8_t> EncodeDeadTime(const uint32_t ticks) noexcept {
  if (ticks <= 127U) return static_cast<uint8_t>(ticks);
  if (ticks <= 254U) return static_cast<uint8_t>(0x80U | ((ticks + 1U) / 2U - 64U));
  if (ticks <= 504U) return static_cast<uint8_t>(0xC0U | ((ticks + 7U) / 8U - 32U));
  if (ticks <= 1008U) return static_cast<uint8_t>(0xE0U | ((ticks + 15U) / 16U - 32U));
  return std::nullopt;
}

/// Dead time in ticks of t_DTS that a DTG field value produces.
[[nodiscard]] constexpr uint32_t DecodeDeadTime(const uint8_t dtg) noexcept {
  if ((dtg & 0x80U) == 0U) return dtg;
  if ((dtg & 0x40U) == 0U) return (64U + (dtg & 0x3FU)) * 2U;
  if ((dtg & 0x20U) == 0U) return (32U + (dtg & 0x1FU)) * 8U;
  return (32U + (dtg & 0x1FU)) * 16U;
}

/// Dead time in ticks for a time in ns at the given dead time clock, rounded up.
[[nodiscard]] constexpr uint32_t DeadTimeTicks(const uint32_t nanoseconds, const uint32_t clock_hz) noexcept {
  return static_cast<uint32_t>((static_cast<uint64_t>(nanoseconds) * clock_hz + 999999999ULL) / 1000000000ULL);
}

static_assert(*EncodeDeadTime(0U) == 0x00U);
static_assert(*EncodeDeadTime(68U) == 68U);
static_assert(DecodeDeadTime(*EncodeDeadTime(127U)) == 127U);
static_assert(DecodeDeadTime(*EncodeDeadTime(128U)) == 128U);
static_assert(DecodeDeadTime(*EncodeDeadTime(129U)) == 130U);
static_assert(DecodeDeadTime(*EncodeDeadTime(254U)) == 254U);
static_assert(DecodeDeadTime(*EncodeDeadTime(255U)) == 256U);
static_assert(DecodeDeadTime(*EncodeDeadTime(504U)) == 504U);
static_assert(DecodeDeadTime(*EncodeDeadTime(505U)) == 512U);
static_assert(DecodeDeadTime(*EncodeDeadTime(1008U)) == 1008U);
static_assert(!EncodeDeadTime(1009U).has_value());
static_assert(DeadTimeTicks(400U, 170000000U) == 68U);

}  // namespace unimoc::hardware::foc_stm32g4
