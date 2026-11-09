/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file drive_monitor_test.cpp
 *    @brief ztest coverage of the drive monitor: freshness, availability and the torque windows.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "drive_monitor.hpp"
#include <zephyr/ztest.h>
#include <limits>
#include "unimoc_test.hpp"

namespace unimoc::vehicle::test {

using namespace unit;

namespace {

constexpr uint64_t kTimeoutUs = 100'000U;

DriveReport Engaged(const AngularVelocity speed = 0.0_rad_per_s) {
  DriveReport report;
  report.wheel_speed = speed;
  report.estimated_torque = 1.0_Nm;
  report.drive_torque_limit = 20.0_Nm;
  report.brake_torque_limit = 8.0_Nm;
  report.engaged = true;
  report.power_stage_enabled = true;
  return report;
}

}  // namespace

ZTEST(drive_monitor, test_no_report_is_not_available) {
  const DriveMonitor kMonitor;
  for (const Wheel kWheel : kWheels) {
    zexpect_false(kMonitor.Fresh(kWheel, 0U));
    zexpect_false(kMonitor.Available(kWheel, 0U));
    UNIMOC_EXPECT_FLOAT_EQ(kMonitor.Window(kWheel, 0U).forward, 0.0_Nm);
  }
}

ZTEST(drive_monitor, test_report_is_fresh_until_the_timeout) {
  DriveMonitor monitor;
  monitor.SetTimeout(kTimeoutUs);
  monitor.OnReport(Wheel::kRearLeft, Engaged(), 1'000U);
  zexpect_true(monitor.Fresh(Wheel::kRearLeft, 1'000U + kTimeoutUs));
  zexpect_true(monitor.Available(Wheel::kRearLeft, 1'000U + kTimeoutUs));
  zexpect_false(monitor.Fresh(Wheel::kRearLeft, 1'001U + kTimeoutUs));
  zexpect_false(monitor.Available(Wheel::kRearLeft, 1'001U + kTimeoutUs));
  zexpect_false(monitor.Fresh(Wheel::kRearRight, 1'000U), "only the reported wheel");
  UNIMOC_EXPECT_FLOAT_EQ(monitor.Report(Wheel::kRearLeft).estimated_torque, 1.0_Nm);
}

ZTEST(drive_monitor, test_disengaged_tripped_or_limp_home_is_not_available) {
  DriveMonitor monitor;
  DriveReport report = Engaged();
  report.engaged = false;
  monitor.OnReport(Wheel::kFrontLeft, report, 0U);
  zexpect_true(monitor.Fresh(Wheel::kFrontLeft, 0U));
  zexpect_false(monitor.Available(Wheel::kFrontLeft, 0U), "disengaged");

  report = Engaged();
  report.power_stage_enabled = false;
  monitor.OnReport(Wheel::kFrontLeft, report, 0U);
  zexpect_false(monitor.Available(Wheel::kFrontLeft, 0U), "power stage off");

  report = Engaged();
  report.limp_home = true;
  monitor.OnReport(Wheel::kFrontLeft, report, 0U);
  zexpect_false(monitor.Available(Wheel::kFrontLeft, 0U), "limp-home");

  report = Engaged();
  report.tripped = true;
  monitor.OnReport(Wheel::kFrontLeft, report, 0U);
  zexpect_false(monitor.Available(Wheel::kFrontLeft, 0U), "tripped");
  UNIMOC_EXPECT_FLOAT_EQ(monitor.Window(Wheel::kFrontLeft, 0U).forward, 0.0_Nm);

  monitor.OnReport(Wheel::kFrontLeft, Engaged(), 0U);
  zexpect_true(monitor.Available(Wheel::kFrontLeft, 0U));
  monitor.Reset();
  zexpect_false(monitor.Fresh(Wheel::kFrontLeft, 0U), "reset forgets the reports");
}

ZTEST(drive_monitor, test_window_follows_the_direction_of_motion) {
  DriveMonitor monitor;
  monitor.OnReport(Wheel::kFrontLeft, Engaged(0.0_rad_per_s), 0U);
  monitor.OnReport(Wheel::kFrontRight, Engaged(5.0_rad_per_s), 0U);
  monitor.OnReport(Wheel::kRearLeft, Engaged(-5.0_rad_per_s), 0U);

  // Standstill and forward: the drive limit forward, the brake limit backward.
  for (const Wheel kWheel : {Wheel::kFrontLeft, Wheel::kFrontRight}) {
    const TorqueWindow kWindow = monitor.Window(kWheel, 0U);
    UNIMOC_EXPECT_FLOAT_EQ(kWindow.forward, 20.0_Nm);
    UNIMOC_EXPECT_FLOAT_EQ(kWindow.backward, 8.0_Nm);
  }
  // Backward: braking is forward torque.
  const TorqueWindow kBackward = monitor.Window(Wheel::kRearLeft, 0U);
  UNIMOC_EXPECT_FLOAT_EQ(kBackward.forward, 8.0_Nm);
  UNIMOC_EXPECT_FLOAT_EQ(kBackward.backward, 20.0_Nm);
}

ZTEST(drive_monitor, test_unknown_or_negative_limits_count_as_zero) {
  DriveMonitor monitor;
  DriveReport report = Engaged();
  report.wheel_speed = AngularVelocity{std::numeric_limits<float>::quiet_NaN()};
  report.drive_torque_limit = Torque{std::numeric_limits<float>::quiet_NaN()};
  report.brake_torque_limit = -3.0_Nm;
  monitor.OnReport(Wheel::kRearRight, report, 0U);
  const TorqueWindow kWindow = monitor.Window(Wheel::kRearRight, 0U);
  UNIMOC_EXPECT_FLOAT_EQ(kWindow.forward, 0.0_Nm);
  UNIMOC_EXPECT_FLOAT_EQ(kWindow.backward, 0.0_Nm);
}

ZTEST_SUITE(drive_monitor, nullptr, nullptr, nullptr, nullptr, nullptr);

}  // namespace unimoc::vehicle::test
