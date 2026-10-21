/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file hall_decoder.hpp
 *    @brief Sectors and edge angles of three 120 degree Hall sensors.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cstdint>
#include <numbers>
#include <optional>
#include "angle_wrap.hpp"
#include "units.hpp"

/**
 * @namespace unimoc::observer observer algorithms namespace
 */
namespace unimoc::observer {

/**
 * @brief Geometry of three Hall sensors spaced 120 degrees electrical.
 *
 * Hall A is high for the electrical angle (relative to `offset`) in
 * [0, 180) degrees, Hall B in [120, 300) and Hall C in [240, 60), the
 * convention of the simulated sensors (lib/sim, `sim::HallSensor`). The six
 * sectors of 60 degrees start at `offset` + k * 60 degrees and carry the
 * codes 5, 1, 3, 2, 6, 4 (bit 0 = A, bit 1 = B, bit 2 = C); forward rotation
 * steps through them in this order. The codes 0 and 7 are invalid (a broken
 * or unpowered sensor).
 *
 * `offset` is the electrical angle at which Hall A rises: the start of
 * sector 0. The edge between two adjacent sectors lies on their common
 * boundary in both directions of rotation; a sensor hysteresis would need a
 * table per direction (docs/hotpath_plan.md, section 5).
 */
class HallDecoder {
 public:
  /// Number of sectors per electrical revolution.
  static constexpr uint8_t kSectors = 6U;

  /// Width of one sector.
  static constexpr unit::Angle kSectorWidth{std::numbers::pi_v<float> / 3.0F};

  /**
   * @brief Constructs a decoder for a mounting offset.
   * @param offset Electrical angle at which Hall A rises.
   */
  explicit constexpr HallDecoder(const unit::Angle offset = unit::Angle{}) noexcept : offset_{offset} {}

  /// Electrical angle at which Hall A rises.
  [[nodiscard]] constexpr unit::Angle Offset() const noexcept { return offset_; }

  /**
   * @brief Returns the sector of a Hall code.
   * @param code Hall code, bit 0 = A, bit 1 = B, bit 2 = C.
   * @return The sector 0..5, or nothing for the invalid codes 0 and 7.
   */
  [[nodiscard]] static constexpr std::optional<uint8_t> Sector(const uint8_t code) noexcept {
    constexpr std::array<int8_t, 8> kSectorOfCode = {-1, 1, 3, 2, 5, 0, 4, -1};
    if (code >= kSectorOfCode.size() || kSectorOfCode[code] < 0) {
      return std::nullopt;
    }
    return static_cast<uint8_t>(kSectorOfCode[code]);
  }

  /**
   * @brief Returns the electrical angle at the start of a sector.
   * @param sector Sector 0..5.
   * @return The start angle, wrapped to (-pi, pi].
   */
  [[nodiscard]] constexpr unit::Angle SectorStart(const uint8_t sector) const noexcept {
    return WrapAngle(offset_ + kSectorWidth * static_cast<float>(sector % kSectors));
  }

  /**
   * @brief Returns the electrical angle of the edge between two codes.
   * @param previous Code before the change.
   * @param code Code after the change.
   * @return The angle of the common sector boundary, or nothing when one
   *         code is invalid or the sectors are not adjacent (a missed edge
   *         or a disturbed signal).
   */
  [[nodiscard]] constexpr std::optional<unit::Angle> EdgeAngle(const uint8_t previous,
                                                              const uint8_t code) const noexcept {
    const std::optional<uint8_t> kFrom = Sector(previous);
    const std::optional<uint8_t> kTo = Sector(code);
    if (!kFrom || !kTo) {
      return std::nullopt;
    }
    if (*kTo == (*kFrom + 1U) % kSectors) {
      return SectorStart(*kTo);  // forward: the start of the new sector
    }
    if (*kFrom == (*kTo + 1U) % kSectors) {
      return SectorStart(*kFrom);  // backward: the start of the old sector
    }
    return std::nullopt;
  }

  /**
   * @brief Innovation of the sector measurement for an angle estimate.
   *
   * Without an edge only "the angle lies in the sector" is known. The
   * innovation is zero while the estimate lies in the sector and otherwise
   * the signed distance from the estimate to the nearest sector boundary, so
   * the sector center does not pull the estimate.
   *
   * @param sector Sector 0..5 of the current code.
   * @param estimate Electrical angle estimate.
   * @return Measured minus estimated angle, in (-pi, pi].
   */
  [[nodiscard]] constexpr unit::Angle SectorInnovation(const uint8_t sector,
                                                       const unit::Angle estimate) const noexcept {
    const unit::Angle kHalfWidth = kSectorWidth * 0.5F;
    const unit::Angle kCenter = SectorStart(sector) + kHalfWidth;
    const unit::Angle kDeviation = WrapAngle(estimate - kCenter);
    if (kDeviation > kHalfWidth) {
      return kHalfWidth - kDeviation;
    }
    if (kDeviation < -kHalfWidth) {
      return -kHalfWidth - kDeviation;
    }
    return unit::Angle{};
  }

 private:
  unit::Angle offset_;
};

}  // namespace unimoc::observer
