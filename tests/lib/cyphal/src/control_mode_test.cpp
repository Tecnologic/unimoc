/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file control_mode_test.cpp
 *    @brief ztest coverage for Cyphal control-mode selection.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "cyphal/control_mode.hpp"
#include <zephyr/ztest.h>
#include <cmath>
#include <limits>
#include "unimoc_test.hpp"

namespace unimoc::cyphal::test {

ZTEST(control_mode, test_selects_position_from_finite_angular_position) {
  const auto kMode = select_control_mode_from_udral_servo_rotation(0.35F,
                                                                   400.0F,
                                                                   std::numeric_limits<float>::quiet_NaN(),
                                                                   50.0F);
  zassert_true(kMode.has_value());
  zexpect_equal(*kMode, ControlMode::kPosition);
}

ZTEST(control_mode, test_selects_speed_when_position_is_not_finite) {
  const auto kMode = select_control_mode_from_udral_servo_rotation(std::numeric_limits<float>::quiet_NaN(),
                                                                   -120.0F,
                                                                   std::numeric_limits<float>::quiet_NaN(),
                                                                   50.0F);
  zassert_true(kMode.has_value());
  zexpect_equal(*kMode, ControlMode::kSpeed);
}

ZTEST(control_mode, test_selects_torque_when_no_kinematic_field_is_finite) {
  const auto kMode = select_control_mode_from_udral_servo_rotation(std::numeric_limits<float>::quiet_NaN(),
                                                                   std::numeric_limits<float>::quiet_NaN(),
                                                                   std::numeric_limits<float>::quiet_NaN(),
                                                                   30.0F);
  zassert_true(kMode.has_value());
  zexpect_equal(*kMode, ControlMode::kTorque);
}

ZTEST(control_mode, test_ignores_unsupported_acceleration_only_setpoint) {
  const auto kMode = select_control_mode_from_udral_servo_rotation(std::numeric_limits<float>::quiet_NaN(),
                                                                   std::numeric_limits<float>::quiet_NaN(),
                                                                   25.0F,
                                                                   std::numeric_limits<float>::quiet_NaN());
  zexpect_false(kMode.has_value());
}

ZTEST(control_mode, test_ignores_empty_setpoint) {
  const auto kMode = select_control_mode_from_udral_servo_rotation(std::numeric_limits<float>::quiet_NaN(),
                                                                   std::numeric_limits<float>::quiet_NaN(),
                                                                   std::numeric_limits<float>::quiet_NaN(),
                                                                   std::numeric_limits<float>::quiet_NaN());
  zexpect_false(kMode.has_value());
}

}  // namespace unimoc::cyphal::test

ZTEST_SUITE(control_mode, nullptr, nullptr, nullptr, nullptr, nullptr);
