/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file torque_split_test.cpp
 *    @brief ztest coverage of the torque split: axle share, equal wheels of an axle, lost drives, limits.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "torque_split.hpp"
#include <zephyr/ztest.h>
#include <limits>
#include "unimoc_test.hpp"

namespace unimoc::vehicle::test {

using namespace unit;

namespace {

constexpr Torque kTolerance = 1.0e-4_Nm;

PerWheel<TorqueWindow> AllWheels(const Torque forward, const Torque backward) {
  PerWheel<TorqueWindow> windows{};
  windows.fill(TorqueWindow{forward, backward});
  return windows;
}

Torque Sum(const PerWheel<Torque>& torques) {
  Torque sum;
  for (const Torque kTorque : torques) {
    sum = sum + kTorque;
  }
  return sum;
}

}  // namespace

ZTEST(torque_split, test_equal_share_splits_evenly) {
  const PerWheel<Torque> kTorques = SplitTorque(40.0_Nm, 0.5_ratio, AllWheels(50.0_Nm, 50.0_Nm));
  for (const Torque kTorque : kTorques) {
    UNIMOC_EXPECT_NEAR(kTorque, 10.0_Nm, kTolerance);
  }
}

ZTEST(torque_split, test_front_share_moves_torque_between_the_axles) {
  const PerWheel<Torque> kTorques = SplitTorque(40.0_Nm, 0.25_ratio, AllWheels(50.0_Nm, 50.0_Nm));
  UNIMOC_EXPECT_NEAR(kTorques[Index(Wheel::kFrontLeft)], 5.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(kTorques[Index(Wheel::kFrontRight)], 5.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(kTorques[Index(Wheel::kRearLeft)], 15.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(kTorques[Index(Wheel::kRearRight)], 15.0_Nm, kTolerance);

  const PerWheel<Torque> kRearOnly = SplitTorque(40.0_Nm, 0.0_ratio, AllWheels(50.0_Nm, 50.0_Nm));
  UNIMOC_EXPECT_NEAR(kRearOnly[Index(Wheel::kFrontLeft)], 0.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(kRearOnly[Index(Wheel::kRearRight)], 20.0_Nm, kTolerance);
}

ZTEST(torque_split, test_braking_uses_the_backward_window) {
  const PerWheel<Torque> kTorques = SplitTorque(-40.0_Nm, 0.5_ratio, AllWheels(50.0_Nm, 4.0_Nm));
  for (const Torque kTorque : kTorques) {
    UNIMOC_EXPECT_NEAR(kTorque, -4.0_Nm, kTolerance);
  }
}

ZTEST(torque_split, test_a_lost_drive_takes_its_axle_out) {
  PerWheel<TorqueWindow> windows = AllWheels(50.0_Nm, 50.0_Nm);
  windows[Index(Wheel::kFrontRight)] = {};  // the drive is lost: no window
  const PerWheel<Torque> kTorques = SplitTorque(40.0_Nm, 0.5_ratio, windows);
  UNIMOC_EXPECT_NEAR(kTorques[Index(Wheel::kFrontLeft)], 0.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(kTorques[Index(Wheel::kFrontRight)], 0.0_Nm, kTolerance);
  // The rear axle takes the whole request: no yaw moment, the same total.
  UNIMOC_EXPECT_NEAR(kTorques[Index(Wheel::kRearLeft)], 20.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(kTorques[Index(Wheel::kRearRight)], 20.0_Nm, kTolerance);
}

ZTEST(torque_split, test_the_smaller_wheel_window_limits_both_wheels_of_an_axle) {
  PerWheel<TorqueWindow> windows = AllWheels(50.0_Nm, 50.0_Nm);
  windows[Index(Wheel::kRearLeft)].forward = 6.0_Nm;
  const PerWheel<Torque> kTorques = SplitTorque(40.0_Nm, 0.5_ratio, windows);
  UNIMOC_EXPECT_NEAR(kTorques[Index(Wheel::kRearLeft)], 6.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(kTorques[Index(Wheel::kRearRight)], 6.0_Nm, kTolerance);
  // The front axle takes what the rear axle cannot.
  UNIMOC_EXPECT_NEAR(kTorques[Index(Wheel::kFrontLeft)], 14.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(kTorques[Index(Wheel::kFrontRight)], 14.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(Sum(kTorques), 40.0_Nm, kTolerance);
}

ZTEST(torque_split, test_a_request_above_all_windows_is_limited) {
  const PerWheel<Torque> kTorques = SplitTorque(100.0_Nm, 0.5_ratio, AllWheels(10.0_Nm, 10.0_Nm));
  for (const Torque kTorque : kTorques) {
    UNIMOC_EXPECT_NEAR(kTorque, 10.0_Nm, kTolerance);
  }
}

ZTEST(torque_split, test_no_available_drive_gives_zero) {
  const PerWheel<Torque> kTorques = SplitTorque(40.0_Nm, 0.5_ratio, PerWheel<TorqueWindow>{});
  UNIMOC_EXPECT_NEAR(Sum(kTorques), 0.0_Nm, kTolerance);
}

ZTEST(torque_split, test_non_finite_inputs) {
  const PerWheel<Torque> kNanRequest =
      SplitTorque(Torque{std::numeric_limits<float>::quiet_NaN()}, 0.5_ratio, AllWheels(50.0_Nm, 50.0_Nm));
  UNIMOC_EXPECT_NEAR(Sum(kNanRequest), 0.0_Nm, kTolerance);

  // A share that is not finite splits evenly; a share outside 0 to 1 is clamped.
  const PerWheel<Torque> kNanShare =
      SplitTorque(40.0_Nm, Ratio{std::numeric_limits<float>::quiet_NaN()}, AllWheels(50.0_Nm, 50.0_Nm));
  UNIMOC_EXPECT_NEAR(kNanShare[Index(Wheel::kFrontLeft)], 10.0_Nm, kTolerance);
  const PerWheel<Torque> kAboveOne = SplitTorque(40.0_Nm, 2.0_ratio, AllWheels(50.0_Nm, 50.0_Nm));
  UNIMOC_EXPECT_NEAR(kAboveOne[Index(Wheel::kFrontLeft)], 20.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(kAboveOne[Index(Wheel::kRearLeft)], 0.0_Nm, kTolerance);
}

ZTEST(torque_split, test_wheel_helpers) {
  zexpect_equal(Index(Wheel::kRearRight), 3U);
  zexpect_true(WheelAt(2U) == Wheel::kRearLeft);
  zexpect_true(OppositeWheel(Wheel::kFrontLeft) == Wheel::kFrontRight);
  zexpect_true(OppositeWheel(Wheel::kRearRight) == Wheel::kRearLeft);
  zexpect_true(AxleOf(Wheel::kFrontRight) == Axle::kFront);
  zexpect_true(AxleOf(Wheel::kRearLeft) == Axle::kRear);
  zexpect_true(WheelsOf(Axle::kRear)[1] == Wheel::kRearRight);
}

ZTEST_SUITE(torque_split, nullptr, nullptr, nullptr, nullptr, nullptr);

}  // namespace unimoc::vehicle::test
