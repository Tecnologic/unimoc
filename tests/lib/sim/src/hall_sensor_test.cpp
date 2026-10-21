/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file hall_sensor_test.cpp
 *	@brief Tests of the 120 degree Hall sensor model.
 *
 *	SPDX-License-Identifier: GPL-3.0-or-later
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */

#include "hall_sensor.hpp"
#include <zephyr/ztest.h>
#include <array>
#include <cstdint>
#include <numbers>
#include "hall_decoder.hpp"
#include "unimoc_test.hpp"

using namespace unimoc::unit;
using unimoc::sim::HallParameters;
using unimoc::sim::HallSensor;
using unimoc::system::RotorAngle;

ZTEST(hall_sensor, test_sector_centers_follow_the_forward_sequence) {
  const HallSensor kSensor;
  constexpr std::array<std::uint8_t, 6> kExpected = {5U, 1U, 3U, 2U, 6U, 4U};
  for (std::size_t sector = 0U; sector < kExpected.size(); ++sector) {
    const Angle kCenter = (60.0_deg * static_cast<float>(sector)) + 30.0_deg;
    zexpect_equal(kSensor.Code(RotorAngle::FromAngle(kCenter)),
                  kExpected.at(sector),
                  "sector %d",
                  static_cast<int>(sector));
  }
}

ZTEST(hall_sensor, test_full_revolution_never_produces_invalid_codes) {
  const HallSensor kSensor;
  int changes = 0;
  std::uint8_t previous = kSensor.Code(RotorAngle{});
  RotorAngle angle;
  // Sweep slightly past one revolution so the edge at 360 degrees is crossed too.
  for (int step = 0; step < 3610; ++step) {
    angle += 0.1_deg;
    const std::uint8_t kCode = kSensor.Code(angle);
    zexpect_not_equal(kCode, 0U);
    zexpect_not_equal(kCode, 7U);
    changes += (kCode != previous) ? 1 : 0;
    previous = kCode;
  }
  zexpect_equal(changes, 6);
}

ZTEST(hall_sensor, test_offset_shifts_the_edges) {
  const HallSensor kSensor(HallParameters{20.0_deg});
  zexpect_equal(kSensor.Code(RotorAngle::FromAngle(19.0_deg)), 4U);
  zexpect_equal(kSensor.Code(RotorAngle::FromAngle(21.0_deg)), 5U);
  zexpect_equal(kSensor.Code(RotorAngle::FromAngle(-41.0_deg)), 6U);
}

ZTEST(hall_sensor, test_update_reports_edge_and_interpolated_age) {
  HallSensor sensor;
  sensor.Reset(RotorAngle::FromAngle(55.0_deg));
  zexpect_equal(sensor.Signals().code, 5U);
  zexpect_false(sensor.Signals().edge);

  // 10 deg per 100 us; the edge at 60 deg lies 5 deg = 50 us before the end of the step.
  const AngularVelocity kSpeed = 10.0_deg / Time{100.0e-6F};
  sensor.Update(RotorAngle::FromAngle(65.0_deg), kSpeed, Time{100.0e-6F});
  zexpect_equal(sensor.Signals().code, 1U);
  zexpect_true(sensor.Signals().edge);
  UNIMOC_EXPECT_NEAR(sensor.Signals().edge_age.Value(), 50.0e-6F, 1.0e-7F);

  sensor.Update(RotorAngle::FromAngle(75.0_deg), kSpeed, Time{100.0e-6F});
  zexpect_false(sensor.Signals().edge);
}

ZTEST(hall_sensor, test_reverse_rotation_reports_age_from_the_edge_behind) {
  HallSensor sensor;
  sensor.Reset(RotorAngle::FromAngle(62.0_deg));
  const AngularVelocity kSpeed = -10.0_deg / Time{100.0e-6F};
  sensor.Update(RotorAngle::FromAngle(52.0_deg), kSpeed, Time{100.0e-6F});
  zexpect_equal(sensor.Signals().code, 5U);
  zexpect_true(sensor.Signals().edge);
  UNIMOC_EXPECT_NEAR(sensor.Signals().edge_age.Value(), 80.0e-6F, 1.0e-7F);
}

ZTEST(hall_sensor, test_observer_decoder_matches_the_simulated_sensors) {
  // The decoder of the observer (lib/motor_control) and the simulated sensors
  // share the convention: the sector of the code contains the angle, for any offset.
  constexpr float kDegree = std::numbers::pi_v<float> / 180.0F;
  for (const float kOffsetDegrees : {0.0F, 17.0F, -100.0F}) {
    const Angle kOffset{kOffsetDegrees * kDegree};
    const unimoc::observer::HallDecoder kDecoder{kOffset};
    const HallSensor kSensor{HallParameters{kOffset}};
    for (int degrees = -180; degrees < 180; degrees += 7) {
      const Angle kAngle{static_cast<float>(degrees) * kDegree};
      const std::uint8_t kCode = kSensor.Code(RotorAngle::FromAngle(kAngle));
      const auto kSector = unimoc::observer::HallDecoder::Sector(kCode);
      zassert_true(kSector.has_value(), "code %u", kCode);
      // Zero inside the sector; on a boundary within the rounding of the two angle formats.
      UNIMOC_EXPECT_NEAR(kDecoder.SectorInnovation(*kSector, kAngle).Value(), 0.0F, 1e-5F);
    }
  }
}

ZTEST_SUITE(hall_sensor, nullptr, nullptr, nullptr, nullptr, nullptr);
