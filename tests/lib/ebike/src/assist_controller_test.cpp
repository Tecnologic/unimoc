/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file assist_controller_test.cpp
 *    @brief ztest coverage of the pedal assist law: torque and power regime, level, limits and ramps.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "assist_controller.hpp"
#include <zephyr/ztest.h>
#include <cmath>
#include "unimoc_test.hpp"

namespace unimoc::ebike::test {

using namespace unit;

namespace {

constexpr Time kStep = 10.0_ms;
constexpr AngularVelocity kCadence = 6.0_rad_per_s;  // about 57 rpm

AssistConfig PlainConfig() {
  AssistConfig config;
  config.power_scale = 1.0e9_W;  // no exponential: the ratio is the level gain
  config.torque_deadband = 0.0_Nm;
  return config;
}

AssistInput Pedaling(const Torque pedal_torque, const AngularVelocity motor_speed, const uint8_t level = 3U) {
  AssistInput input;
  input.pedaling = true;
  input.cadence = kCadence;
  input.pedal_torque = pedal_torque;
  input.motor_speed = motor_speed;
  input.speed_limit = 500.0_rad_per_s;
  input.torque_limit = 40.0_Nm;
  input.level = level;
  return input;
}

/// Runs the controller until the ramp has settled.
Torque Settle(AssistController& controller, const AssistInput& input) {
  Torque torque;
  for (int i = 0; i < 400; ++i) {
    torque = controller.Step(kStep, input);
  }
  return torque;
}

}  // namespace

ZTEST(assist_controller, test_low_speed_torque_follows_the_pedal_torque) {
  AssistController controller(PlainConfig());
  // Level 3 has ratio 1.0, torque ratio 0.5: 20 Nm of pedal gives 10 Nm at a standing motor.
  UNIMOC_EXPECT_NEAR(Settle(controller, Pedaling(20.0_Nm, 0.0_rad_per_s)), 10.0_Nm, 1e-3_Nm);
  AssistController doubled(PlainConfig());
  UNIMOC_EXPECT_NEAR(Settle(doubled, Pedaling(40.0_Nm, 0.0_rad_per_s)), 20.0_Nm, 1e-3_Nm);
}

ZTEST(assist_controller, test_high_speed_motor_power_follows_the_pedal_power) {
  AssistController controller(PlainConfig());
  const AngularVelocity kMotor = 100.0_rad_per_s;
  const Torque kTorque = Settle(controller, Pedaling(20.0_Nm, kMotor));
  // Level 3: ratio 1.0, so motor power = pedal power = 20 Nm * 6 rad/s = 120 W.
  UNIMOC_EXPECT_NEAR(kTorque * kMotor, 20.0_Nm * kCadence, 0.5_W);
  AssistController strong(PlainConfig());
  const Torque kStrong = Settle(strong, Pedaling(20.0_Nm, kMotor, 5U));
  // Level 5: ratio 2.0.
  UNIMOC_EXPECT_NEAR(kStrong * kMotor, 2.0F * 20.0_Nm * kCadence, 0.5_W);
}

ZTEST(assist_controller, test_regimes_join_continuously_at_the_crossover_speed) {
  const AssistConfig kConfig = PlainConfig();
  // The crossover is the motor speed at which the cadence is torque_ratio times the motor speed.
  const AngularVelocity kCrossover = kCadence / kConfig.torque_ratio;
  AssistController below(kConfig);
  AssistController above(kConfig);
  const Torque kBelow = Settle(below, Pedaling(20.0_Nm, kCrossover * 0.999F));
  const Torque kAbove = Settle(above, Pedaling(20.0_Nm, kCrossover * 1.001F));
  UNIMOC_EXPECT_NEAR(kBelow, kAbove, 0.05_Nm);
  // Going faster only lowers the torque.
  AssistController fast(kConfig);
  zexpect_true(Settle(fast, Pedaling(20.0_Nm, kCrossover * 3.0F)) < kAbove);
}

ZTEST(assist_controller, test_motor_direction_does_not_matter) {
  AssistController forward(PlainConfig());
  AssistController reverse(PlainConfig());
  UNIMOC_EXPECT_NEAR(Settle(forward, Pedaling(20.0_Nm, 100.0_rad_per_s)),
                     Settle(reverse, Pedaling(20.0_Nm, -100.0_rad_per_s)),
                     1e-4_Nm);
}

ZTEST(assist_controller, test_assist_rises_exponentially_with_the_pedal_power) {
  AssistConfig config = PlainConfig();
  config.power_scale = 100.0_W;
  config.max_ratio = 100.0_ratio;
  AssistController low(config);
  AssistController high(config);
  // 10 Nm * 6 rad/s = 60 W and 20 Nm * 6 rad/s = 120 W: the ratio grows by exp(60/100).
  Settle(low, Pedaling(10.0_Nm, 0.0_rad_per_s));
  Settle(high, Pedaling(20.0_Nm, 0.0_rad_per_s));
  UNIMOC_EXPECT_NEAR(low.AssistRatio(), 1.0_ratio * std::exp(0.6F), 1e-3_ratio);
  UNIMOC_EXPECT_NEAR(high.AssistRatio(), 1.0_ratio * std::exp(1.2F), 1e-3_ratio);
  UNIMOC_EXPECT_NEAR(high.AssistRatio() / low.AssistRatio(), 1.0_ratio * std::exp(0.6F), 1e-3_ratio);
}

ZTEST(assist_controller, test_ratio_is_limited) {
  AssistConfig config = PlainConfig();
  config.power_scale = 10.0_W;
  AssistController controller(config);
  Settle(controller, Pedaling(100.0_Nm, 0.0_rad_per_s, 5U));
  UNIMOC_EXPECT_NEAR(controller.AssistRatio(), config.max_ratio, 1e-6_ratio);
  // A huge power does not overflow the exponential.
  AssistController huge(config);
  const Torque kTorque = Settle(huge, Pedaling(1.0e6_Nm, 0.0_rad_per_s, 5U));
  zexpect_true(kTorque == kTorque && kTorque <= 40.0_Nm, "torque %f", static_cast<double>(kTorque.Value()));
}

ZTEST(assist_controller, test_no_assist_without_pedaling_or_level) {
  AssistController idle(PlainConfig());
  AssistInput input = Pedaling(30.0_Nm, 0.0_rad_per_s);
  input.pedaling = false;
  UNIMOC_EXPECT_NEAR(Settle(idle, input), 0.0_Nm, 1e-6_Nm);
  zexpect_false(idle.Active());

  AssistController off(PlainConfig());
  UNIMOC_EXPECT_NEAR(Settle(off, Pedaling(30.0_Nm, 0.0_rad_per_s, 0U)), 0.0_Nm, 1e-6_Nm);

  AssistController slow(PlainConfig());
  AssistInput slow_input = Pedaling(30.0_Nm, 0.0_rad_per_s);
  slow_input.cadence = 1.0_rad_per_s;  // below the minimum cadence
  UNIMOC_EXPECT_NEAR(Settle(slow, slow_input), 0.0_Nm, 1e-6_Nm);
}

ZTEST(assist_controller, test_deadband) {
  AssistConfig config = PlainConfig();
  config.torque_deadband = 3.0_Nm;
  AssistController below(config);
  UNIMOC_EXPECT_NEAR(Settle(below, Pedaling(2.5_Nm, 0.0_rad_per_s)), 0.0_Nm, 1e-6_Nm);
  AssistController above(config);
  // Only the torque above the deadband counts: (13 - 3) * 1.0 * 0.5.
  UNIMOC_EXPECT_NEAR(Settle(above, Pedaling(13.0_Nm, 0.0_rad_per_s)), 5.0_Nm, 1e-3_Nm);
}

ZTEST(assist_controller, test_speed_limit_fades_the_assist_out) {
  // The limit lies in the torque regime (motor speed below cadence / torque_ratio = 12 rad/s), where the full
  // torque does not depend on the speed.
  AssistConfig config = PlainConfig();
  config.speed_taper = 4.0_rad_per_s;
  AssistInput input = Pedaling(20.0_Nm, 0.0_rad_per_s);
  input.speed_limit = 12.0_rad_per_s;

  AssistController free_running(config);
  input.motor_speed = input.speed_limit - config.speed_taper * 2.0F;
  const Torque kFull = Settle(free_running, input);
  AssistController half(config);
  input.motor_speed = input.speed_limit - config.speed_taper * 0.5F;
  const Torque kHalf = Settle(half, input);
  AssistController at_limit(config);
  input.motor_speed = input.speed_limit;
  const Torque kNone = Settle(at_limit, input);
  AssistController beyond(config);
  input.motor_speed = input.speed_limit * 1.2F;
  const Torque kBeyond = Settle(beyond, input);
  UNIMOC_EXPECT_NEAR(kHalf, kFull * 0.5F, 0.05_Nm);
  UNIMOC_EXPECT_NEAR(kNone, 0.0_Nm, 1e-6_Nm);
  UNIMOC_EXPECT_NEAR(kBeyond, 0.0_Nm, 1e-6_Nm);

  AssistController no_limit(config);
  input.speed_limit = 0.0_rad_per_s;
  input.motor_speed = 0.0_rad_per_s;
  UNIMOC_EXPECT_NEAR(Settle(no_limit, input), 0.0_Nm, 1e-6_Nm);
}

ZTEST(assist_controller, test_torque_limit) {
  AssistController controller(PlainConfig());
  AssistInput input = Pedaling(100.0_Nm, 0.0_rad_per_s, 5U);
  input.torque_limit = 12.0_Nm;
  UNIMOC_EXPECT_NEAR(Settle(controller, input), 12.0_Nm, 1e-4_Nm);
  input.torque_limit = 0.0_Nm;
  UNIMOC_EXPECT_NEAR(Settle(controller, input), 0.0_Nm, 1e-6_Nm);
}

ZTEST(assist_controller, test_output_ramps_up_slowly_and_down_quickly) {
  const AssistConfig kConfig = PlainConfig();
  AssistController controller(kConfig);
  const AssistInput kInput = Pedaling(100.0_Nm, 0.0_rad_per_s, 5U);  // wants far more than the limit

  // From zero, the first step may not exceed the limit times dt / rise_time.
  const Torque kFirst = controller.Step(kStep, kInput);
  UNIMOC_EXPECT_NEAR(kFirst, kInput.torque_limit * (kStep / kConfig.rise_time), 1e-4_Nm);
  Time rise;
  while (controller.MotorTorque() < kInput.torque_limit && rise < 5.0_s) {
    controller.Step(kStep, kInput);
    rise += kStep;
  }
  UNIMOC_EXPECT_NEAR(rise, kConfig.rise_time, 3.0F * kStep);

  AssistInput stopped = kInput;
  stopped.pedaling = false;
  Time fall;
  while (controller.Active() && fall < 5.0_s) {
    controller.Step(kStep, stopped);
    fall += kStep;
  }
  UNIMOC_EXPECT_NEAR(fall, kConfig.fall_time, 3.0F * kStep);
  zexpect_true(fall < rise);
}

}  // namespace unimoc::ebike::test

ZTEST_SUITE(assist_controller, nullptr, nullptr, nullptr, nullptr, nullptr);
