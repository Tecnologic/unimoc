/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file hall_sensor.hpp
 *	@brief Three digital Hall sensors spaced 120 degrees electrical.
 *
 *	SPDX-License-Identifier: GPL-3.0-or-later
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cstdint>
#include "rotor_angle.hpp"
#include "units.hpp"

namespace unimoc::sim {

/**
 * @brief Parameters of the Hall sensor set.
 */
struct HallParameters {
  /// Electrical angle at which Hall A rises. Models the mounting misalignment.
  unit::Angle offset;
};

/**
 * @brief Hall signals after a simulation step.
 */
struct HallSignals {
  /// Hall code: bit 0 = A, bit 1 = B, bit 2 = C. Valid codes are 1..6.
  std::uint8_t code{0U};
  /// `true` when at least one Hall signal changed during the last step.
  bool edge{false};
  /// Time from the last edge to the end of the step. Valid when `edge` is set.
  unit::Time edge_age;
};

/**
 * @brief Three Hall sensors spaced 120 degrees electrical (120 degree Hall encoder).
 *
 * Hall A is high for the electrical angle (relative to the offset) in [0, 180)
 * degrees, Hall B in [120, 300) and Hall C in [240, 60). The sectors starting
 * at 0, 60, 120, 180, 240 and 300 degrees therefore carry the codes
 * 5, 1, 3, 2, 6, 4: forward rotation steps through this sequence with an edge
 * every 60 degrees electrical, and the invalid codes 0 and 7 never occur.
 *
 * The angle is evaluated in integer Q1.31 counts, so the edges are
 * deterministic. The edge age is interpolated from the electrical speed, as a
 * timer-captured Hall input would report it.
 */
class HallSensor {
 public:
  /** @brief Constructs a sensor set with zero offset. */
  constexpr HallSensor() = default;

  /**
   * @brief Constructs a sensor set with the given parameters.
   * @param parameters Mounting offset of the sensors.
   */
  explicit constexpr HallSensor(const HallParameters& parameters) : parameters_(parameters) {}

  /**
   * @brief Returns the parameters.
   * @return The Hall parameters.
   */
  [[nodiscard]] constexpr const HallParameters& Parameters() const noexcept { return parameters_; }

  /**
   * @brief Returns the Hall code of an electrical angle.
   * @param electrical_angle Electrical rotor angle.
   * @return The code with bit 0 = A, bit 1 = B, bit 2 = C.
   */
  [[nodiscard]] constexpr std::uint8_t Code(const system::RotorAngle& electrical_angle) const noexcept {
    const std::uint32_t kPosition = Position(electrical_angle);
    std::uint8_t code = 0U;
    for (std::uint32_t sensor = 0U; sensor < kSensors; ++sensor) {
      // A sensor is high for the half revolution after its rising edge.
      if (kPosition - kRisingEdge.at(sensor) < kHalfRevolution) {
        code = static_cast<std::uint8_t>(code | (1U << sensor));
      }
    }
    return code;
  }

  /**
   * @brief Sets the signals to the code of an angle without reporting an edge.
   * @param electrical_angle Electrical rotor angle.
   */
  constexpr void Reset(const system::RotorAngle& electrical_angle) noexcept {
    signals_ = HallSignals{Code(electrical_angle), false, unit::Time{}};
  }

  /**
   * @brief Updates the signals at the end of a step.
   * @param electrical_angle Electrical angle at the end of the step.
   * @param electrical_speed Mean electrical speed of the step.
   * @param time_step Step size.
   */
  constexpr void Update(const system::RotorAngle& electrical_angle,
                        unit::AngularVelocity electrical_speed,
                        unit::Time time_step) noexcept {
    const std::uint8_t kCode = Code(electrical_angle);
    if (kCode == signals_.code) {
      signals_.edge = false;
      return;
    }
    signals_.edge = true;
    signals_.code = kCode;
    signals_.edge_age = EdgeAge(electrical_angle, electrical_speed, time_step);
  }

  /**
   * @brief Returns the signals of the last update.
   * @return The Hall code and the edge information.
   */
  [[nodiscard]] constexpr const HallSignals& Signals() const noexcept { return signals_; }

 private:
  /// Number of Hall sensors.
  static constexpr std::uint32_t kSensors = 3U;
  /// Half an electrical revolution in Q1.31 counts.
  static constexpr std::uint32_t kHalfRevolution = 0x8000'0000U;
  /// One sixth of an electrical revolution in Q1.31 counts.
  static constexpr std::uint32_t kSixthRevolution = 0x2AAA'AAABU;
  /// Rising edges of Hall A, B, C relative to the offset, in Q1.31 counts.
  static constexpr std::array<std::uint32_t, kSensors> kRisingEdge = {0U,
                                                                      2U * kSixthRevolution,
                                                                      4U * kSixthRevolution};

  /**
   * @brief Returns the angle relative to the offset as an unsigned position.
   * @param electrical_angle Electrical rotor angle.
   * @return The position in [0, 2^32) counts per revolution.
   */
  [[nodiscard]] constexpr std::uint32_t Position(const system::RotorAngle& electrical_angle) const noexcept {
    const system::RotorAngle kOffset = system::RotorAngle::FromAngle(parameters_.offset);
    return static_cast<std::uint32_t>(electrical_angle.Raw()) - static_cast<std::uint32_t>(kOffset.Raw());
  }

  /**
   * @brief Interpolates the time since the most recent edge passed in the step.
   * @param electrical_angle Electrical angle at the end of the step.
   * @param electrical_speed Mean electrical speed of the step.
   * @param time_step Step size.
   * @return The age of the edge, limited to [0, time_step].
   */
  [[nodiscard]] constexpr unit::Time EdgeAge(const system::RotorAngle& electrical_angle,
                                             unit::AngularVelocity electrical_speed,
                                             unit::Time time_step) const noexcept {
    if (electrical_speed == unit::AngularVelocity{}) {
      return {};
    }
    // Edges lie on the 60 degree grid relative to the offset. The most recent one is the
    // grid point just behind the angle in the direction of rotation.
    const std::uint32_t kPosition = Position(electrical_angle);
    const std::uint32_t kSinceForwardEdge = kPosition % kSixthRevolution;
    const std::uint32_t kDistance = (electrical_speed > unit::AngularVelocity{})
                                        ? kSinceForwardEdge
                                        : (kSixthRevolution - kSinceForwardEdge);
    const unit::Angle kTravel =
        system::RotorAngle::FromRaw(static_cast<std::int32_t>(kDistance)).AngleInRevolution();
    const unit::Angle kTravelMagnitude = (electrical_speed > unit::AngularVelocity{}) ? kTravel : -kTravel;
    const unit::Time kAge = kTravelMagnitude / electrical_speed;
    return kAge.Clamp(unit::Time{}, time_step);
  }

  /// Mounting offset of the sensors.
  HallParameters parameters_;
  /// Signals of the last update.
  HallSignals signals_;
};

}  // namespace unimoc::sim
