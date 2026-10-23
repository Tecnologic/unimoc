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
  controller.kp = TorquePerAngularVelocity{0.02F};
  controller.ki = InverseTime{10.0F};
  controller.torque_limit = Torque{1.0F};
  return controller;
}

}  // namespace

ZTEST(speed_controller, test_proportional_and_integral_parts) {
  SpeedController controller = MakeController();
  // Error 10 rad/s: P = 0.2 N m, integral after one step 0.2 * 10 * 0.5 ms = 1 mN m.
  const Torque kFirst = controller.Update(AngularVelocity{10.0F}, AngularVelocity{}, kDt);
  UNIMOC_EXPECT_NEAR(kFirst.Value(), 0.201F, 1e-6F);
  const Torque kSecond = controller.Update(AngularVelocity{10.0F}, AngularVelocity{}, kDt);
  UNIMOC_EXPECT_NEAR(kSecond.Value(), 0.202F, 1e-6F);
}

ZTEST(speed_controller, test_integral_removes_a_constant_load) {
  // Plant: J dw/dt = T - T_load. The integral settles at the load torque.
  SpeedController controller = MakeController();
  constexpr float kInertia = 1e-3F;
  constexpr float kLoad = 0.3F;
  float speed = 0.0F;
  for (int i = 0; i < 20000; ++i) {  // 10 s
    const Torque kTorque = controller.Update(AngularVelocity{50.0F}, AngularVelocity{speed}, kDt);
    speed += (kTorque.Value() - kLoad) / kInertia * kDt.Value();
  }
  UNIMOC_EXPECT_NEAR(speed, 50.0F, 0.05F);
  UNIMOC_EXPECT_NEAR(controller.integral.Value(), kLoad, 0.01F);
}

ZTEST(speed_controller, test_output_is_limited_and_the_integral_does_not_wind_up) {
  SpeedController controller = MakeController();
  for (int i = 0; i < 1000; ++i) {
    const Torque kTorque = controller.Update(AngularVelocity{1000.0F}, AngularVelocity{}, kDt);
    zassert_true(kTorque.Value() <= 1.0F + 1e-6F);
  }
  UNIMOC_EXPECT_FLOAT_EQ(controller.output.Value(), 1.0F);
  // P alone saturates (20 N m): the integral did not grow at all.
  UNIMOC_EXPECT_FLOAT_EQ(controller.integral.Value(), 0.0F);
  // At the setpoint the output drops at once instead of unwinding a large integral.
  const Torque kAtSetpoint = controller.Update(AngularVelocity{1000.0F}, AngularVelocity{1000.0F}, kDt);
  UNIMOC_EXPECT_FLOAT_EQ(kAtSetpoint.Value(), 0.0F);
}

ZTEST(speed_controller, test_reference_ramps_with_the_acceleration_limit) {
  SpeedController controller = MakeController();
  controller.accel_limit = AngularAcceleration{100.0F};
  for (int i = 0; i < 10; ++i) {
    (void)controller.Update(AngularVelocity{100.0F}, AngularVelocity{}, kDt);
  }
  // 10 steps of 0.5 ms at 100 rad/s^2.
  UNIMOC_EXPECT_NEAR(controller.reference.Value(), 0.5F, 1e-5F);
  (void)controller.Track(AngularVelocity{-20.0F}, AngularVelocity{}, kDt);
  UNIMOC_EXPECT_FLOAT_EQ(controller.reference.Value(), -20.0F);  // Track() does not ramp
  controller.Reset(AngularVelocity{7.0F});
  UNIMOC_EXPECT_FLOAT_EQ(controller.reference.Value(), 7.0F);
  UNIMOC_EXPECT_FLOAT_EQ(controller.integral.Value(), 0.0F);
}

ZTEST(speed_controller, test_non_finite_inputs_give_zero_torque) {
  SpeedController controller = MakeController();
  (void)controller.Update(AngularVelocity{10.0F}, AngularVelocity{}, kDt);
  const Torque kTorque =
      controller.Update(AngularVelocity{std::numeric_limits<float>::quiet_NaN()}, AngularVelocity{}, kDt);
  UNIMOC_EXPECT_FLOAT_EQ(kTorque.Value(), 0.0F);
  zexpect_true(std::isfinite(controller.reference.Value()));
}

ZTEST(speed_controller, test_init_loads_the_settings) {
  settings::NvmSettings settings{};
  settings.speed_kp = TorquePerAngularVelocity{0.5F};
  settings.speed_ki = InverseTime{3.0F};
  settings.speed_accel_limit = AngularAcceleration{42.0F};
  SpeedController controller;
  controller.integral = Torque{1.0F};
  controller.Init(settings);
  UNIMOC_EXPECT_FLOAT_EQ(controller.kp.Value(), 0.5F);
  UNIMOC_EXPECT_FLOAT_EQ(controller.ki.Value(), 3.0F);
  UNIMOC_EXPECT_FLOAT_EQ(controller.accel_limit.Value(), 42.0F);
  UNIMOC_EXPECT_FLOAT_EQ(controller.integral.Value(), 0.0F);
}

ZTEST(speed_controller, test_additional_bounds_limit_the_output_and_stop_the_integral) {
  SpeedController controller = MakeController();
  controller.torque_min = Torque{-0.2F};  // e.g. the battery charge limit while braking
  for (int i = 0; i < 100; ++i) {
    (void)controller.Track(AngularVelocity{0.0F}, AngularVelocity{100.0F}, kDt);
  }
  UNIMOC_EXPECT_FLOAT_EQ(controller.output.Value(), -0.2F);
  zexpect_true(controller.integral.Value() >= -0.2F,
               "integral %f",
               static_cast<double>(controller.integral.Value()));

  // The bound released and the error gone: no stored windup drives the output.
  controller.torque_min = Torque{-std::numeric_limits<float>::infinity()};
  const Torque kOutput = controller.Track(AngularVelocity{100.0F}, AngularVelocity{100.0F}, kDt);
  zexpect_true(kOutput.Value() >= -0.2F, "output %f", static_cast<double>(kOutput.Value()));
}

}  // namespace unimoc::control::test

ZTEST_SUITE(speed_controller, nullptr, nullptr, nullptr, nullptr, nullptr);
