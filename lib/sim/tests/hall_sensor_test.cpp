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
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */

#include "hall_sensor.hpp"
#include <gtest/gtest.h>
#include <array>
#include <cstdint>

using namespace unimoc::unit;
using unimoc::sim::HallParameters;
using unimoc::sim::HallSensor;
using unimoc::system::RotorAngle;

TEST(HallSensorTest, SectorCentersFollowTheForwardSequence) {
  const HallSensor kSensor;
  constexpr std::array<std::uint8_t, 6> kExpected = {5U, 1U, 3U, 2U, 6U, 4U};
  for (std::size_t sector = 0U; sector < kExpected.size(); ++sector) {
    const Angle kCenter = (60.0_deg * static_cast<float>(sector)) + 30.0_deg;
    EXPECT_EQ(kSensor.Code(RotorAngle::FromAngle(kCenter)), kExpected.at(sector)) << "sector " << sector;
  }
}

TEST(HallSensorTest, FullRevolutionNeverProducesInvalidCodes) {
  const HallSensor kSensor;
  int changes = 0;
  std::uint8_t previous = kSensor.Code(RotorAngle{});
  RotorAngle angle;
  // Sweep slightly past one revolution so the edge at 360 degrees is crossed too.
  for (int step = 0; step < 3610; ++step) {
    angle += 0.1_deg;
    const std::uint8_t kCode = kSensor.Code(angle);
    EXPECT_NE(kCode, 0U);
    EXPECT_NE(kCode, 7U);
    changes += (kCode != previous) ? 1 : 0;
    previous = kCode;
  }
  EXPECT_EQ(changes, 6);
}

TEST(HallSensorTest, OffsetShiftsTheEdges) {
  const HallSensor kSensor(HallParameters{20.0_deg});
  EXPECT_EQ(kSensor.Code(RotorAngle::FromAngle(19.0_deg)), 4U);
  EXPECT_EQ(kSensor.Code(RotorAngle::FromAngle(21.0_deg)), 5U);
  EXPECT_EQ(kSensor.Code(RotorAngle::FromAngle(-41.0_deg)), 6U);
}

TEST(HallSensorTest, UpdateReportsEdgeAndInterpolatedAge) {
  HallSensor sensor;
  sensor.Reset(RotorAngle::FromAngle(55.0_deg));
  EXPECT_EQ(sensor.Signals().code, 5U);
  EXPECT_FALSE(sensor.Signals().edge);

  // 10 deg per 100 us; the edge at 60 deg lies 5 deg = 50 us before the end of the step.
  const AngularVelocity kSpeed = 10.0_deg / Time{100.0e-6F};
  sensor.Update(RotorAngle::FromAngle(65.0_deg), kSpeed, Time{100.0e-6F});
  EXPECT_EQ(sensor.Signals().code, 1U);
  EXPECT_TRUE(sensor.Signals().edge);
  EXPECT_NEAR(sensor.Signals().edge_age.Value(), 50.0e-6F, 1.0e-7F);

  sensor.Update(RotorAngle::FromAngle(75.0_deg), kSpeed, Time{100.0e-6F});
  EXPECT_FALSE(sensor.Signals().edge);
}

TEST(HallSensorTest, ReverseRotationReportsAgeFromTheEdgeBehind) {
  HallSensor sensor;
  sensor.Reset(RotorAngle::FromAngle(62.0_deg));
  const AngularVelocity kSpeed = -10.0_deg / Time{100.0e-6F};
  sensor.Update(RotorAngle::FromAngle(52.0_deg), kSpeed, Time{100.0e-6F});
  EXPECT_EQ(sensor.Signals().code, 5U);
  EXPECT_TRUE(sensor.Signals().edge);
  EXPECT_NEAR(sensor.Signals().edge_age.Value(), 80.0e-6F, 1.0e-7F);
}
