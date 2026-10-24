/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file speed_controller_test.cpp
 *    @brief ztest coverage of the PI speed controller.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "speed_controller.hpp"
#include <zephyr/ztest.h>
#include <cmath>
#include <limits>
#include "unimoc_test.hpp"

namespace unimoc::control::test {

using namespace unit;

namespace {

constexpr Time kDt{0.5e-3F};

SpeedController MakeController() {
  SpeedController controller;
  controller.kp = 0.02_Nm_s_per_rad;
  controller.ki = 10.0_per_s;
  controller.torque_limit = 1.0_Nm;
  return controller;
}

}  // namespace

ZTEST(speed_controller, test_proportional_and_integral_parts) {
  SpeedController controller = MakeController();
  // Error 10 rad/s: P = 0.2 N m, integral after one step 0.2 * 10 * 0.5 ms = 1 mN m.
  const Torque kFirst = controller.Update(10.0_rad_per_s, AngularVelocity{}, kDt);
  UNIMOC_EXPECT_NEAR(kFirst, 0.201_Nm, 1e-6_Nm);
  const Torque kSecond = controller.Update(10.0_rad_per_s, AngularVelocity{}, kDt);
  UNIMOC_EXPECT_NEAR(kSecond, 0.202_Nm, 1e-6_Nm);
}

ZTEST(speed_controller, test_integral_removes_a_constant_load) {
  // Plant: J dw/dt = T - T_load. The integral settles at the load torque.
  SpeedController controller = MakeController();
  constexpr Inertia kInertia = 1e-3_kg_m2;
  constexpr Torque kLoad = 0.3_Nm;
  AngularVelocity speed;
  for (int i = 0; i < 20000; ++i) {  // 10 s
    const Torque kTorque = controller.Update(50.0_rad_per_s, speed, kDt);
    speed += (kTorque - kLoad) / kInertia * kDt;
  }
  UNIMOC_EXPECT_NEAR(speed, 50.0_rad_per_s, 0.05_rad_per_s);
  UNIMOC_EXPECT_NEAR(controller.integral, kLoad, 0.01_Nm);
}

ZTEST(speed_controller, test_output_is_limited_and_the_integral_does_not_wind_up) {
  SpeedController controller = MakeController();
  for (int i = 0; i < 1000; ++i) {
    const Torque kTorque = controller.Update(1000.0_rad_per_s, AngularVelocity{}, kDt);
    zassert_true(kTorque <= 1.0_Nm + 1e-6_Nm);
  }
  UNIMOC_EXPECT_FLOAT_EQ(controller.output, 1.0_Nm);
  // P alone saturates (20 N m): the integral did not grow at all.
  UNIMOC_EXPECT_FLOAT_EQ(controller.integral, 0.0_Nm);
  // At the setpoint the output drops at once instead of unwinding a large integral.
  const Torque kAtSetpoint = controller.Update(1000.0_rad_per_s, 1000.0_rad_per_s, kDt);
  UNIMOC_EXPECT_FLOAT_EQ(kAtSetpoint, 0.0_Nm);
}

ZTEST(speed_controller, test_reference_ramps_with_the_acceleration_limit) {
  SpeedController controller = MakeController();
  controller.accel_limit = 100.0_rad_per_s2;
  for (int i = 0; i < 10; ++i) {
    (void)controller.Update(100.0_rad_per_s, AngularVelocity{}, kDt);
  }
  // 10 steps of 0.5 ms at 100 rad/s^2.
  UNIMOC_EXPECT_NEAR(controller.reference, 0.5_rad_per_s, 1e-5_rad_per_s);
  (void)controller.Track(-20.0_rad_per_s, AngularVelocity{}, kDt);
  UNIMOC_EXPECT_FLOAT_EQ(controller.reference, -20.0_rad_per_s);  // Track() does not ramp
  controller.Reset(7.0_rad_per_s);
  UNIMOC_EXPECT_FLOAT_EQ(controller.reference, 7.0_rad_per_s);
  UNIMOC_EXPECT_FLOAT_EQ(controller.integral, 0.0_Nm);
}

ZTEST(speed_controller, test_non_finite_inputs_give_zero_torque) {
  SpeedController controller = MakeController();
  (void)controller.Update(10.0_rad_per_s, AngularVelocity{}, kDt);
  const Torque kTorque =
      controller.Update(AngularVelocity{std::numeric_limits<float>::quiet_NaN()}, AngularVelocity{}, kDt);
  UNIMOC_EXPECT_FLOAT_EQ(kTorque, 0.0_Nm);
  zexpect_true(IsFinite(controller.reference));
}

ZTEST(speed_controller, test_init_loads_the_settings) {
  settings::NvmSettings settings{};
  settings.speed_kp = 0.5_Nm_s_per_rad;
  settings.speed_ki = 3.0_per_s;
  settings.speed_accel_limit = 42.0_rad_per_s2;
  SpeedController controller;
  controller.integral = 1.0_Nm;
  controller.Init(settings);
  UNIMOC_EXPECT_FLOAT_EQ(controller.kp, 0.5_Nm_s_per_rad);
  UNIMOC_EXPECT_FLOAT_EQ(controller.ki, 3.0_per_s);
  UNIMOC_EXPECT_FLOAT_EQ(controller.accel_limit, 42.0_rad_per_s2);
  UNIMOC_EXPECT_FLOAT_EQ(controller.integral, 0.0_Nm);
}

ZTEST(speed_controller, test_additional_bounds_limit_the_output_and_stop_the_integral) {
  SpeedController controller = MakeController();
  controller.torque_min = -0.2_Nm;  // e.g. the battery charge limit while braking
  for (int i = 0; i < 100; ++i) {
    (void)controller.Track(0.0_rad_per_s, 100.0_rad_per_s, kDt);
  }
  UNIMOC_EXPECT_FLOAT_EQ(controller.output, -0.2_Nm);
  zexpect_true(controller.integral >= -0.2_Nm,
               "integral %f",
               static_cast<double>(controller.integral.Value()));

  // The bound released and the error gone: no stored windup drives the output.
  controller.torque_min = Torque{-std::numeric_limits<float>::infinity()};
  const Torque kOutput = controller.Track(100.0_rad_per_s, 100.0_rad_per_s, kDt);
  zexpect_true(kOutput >= -0.2_Nm, "output %f", static_cast<double>(kOutput.Value()));
}

}  // namespace unimoc::control::test

ZTEST_SUITE(speed_controller, nullptr, nullptr, nullptr, nullptr, nullptr);
