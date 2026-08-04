#pragma once

#ifndef UNIMOC_TEST_CONTROL_MODE_H_
#define UNIMOC_TEST_CONTROL_MODE_H_

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "ControlMode.hpp"

namespace unimoc
{
namespace system
{
namespace test
{

class ControlModeTest : public ::testing::Test {};

TEST_F(ControlModeTest, SelectsPositionFromFiniteAngularPosition)
{
    const auto mode = select_control_mode_from_udral_servo_rotation(
        0.35f,
        400.0f,
        std::numeric_limits<float>::quiet_NaN(),
        50.0f);
    ASSERT_TRUE(mode.has_value());
    EXPECT_EQ(*mode, ControlMode::POSITION);
}

TEST_F(ControlModeTest, SelectsSpeedWhenPositionIsNotFinite)
{
    const auto mode = select_control_mode_from_udral_servo_rotation(
        std::numeric_limits<float>::quiet_NaN(),
        -120.0f,
        std::numeric_limits<float>::quiet_NaN(),
        50.0f);
    ASSERT_TRUE(mode.has_value());
    EXPECT_EQ(*mode, ControlMode::SPEED);
}

TEST_F(ControlModeTest, SelectsTorqueWhenNoKinematicFieldIsFinite)
{
    const auto mode = select_control_mode_from_udral_servo_rotation(
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        30.0f);
    ASSERT_TRUE(mode.has_value());
    EXPECT_EQ(*mode, ControlMode::TORQUE);
}

TEST_F(ControlModeTest, IgnoresUnsupportedAccelerationOnlySetpoint)
{
    const auto mode = select_control_mode_from_udral_servo_rotation(
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        25.0f,
        std::numeric_limits<float>::quiet_NaN());
    EXPECT_FALSE(mode.has_value());
}

TEST_F(ControlModeTest, IgnoresEmptySetpoint)
{
    const auto mode = select_control_mode_from_udral_servo_rotation(
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN());
    EXPECT_FALSE(mode.has_value());
}

}  // namespace test
}  // namespace system
}  // namespace unimoc

#endif /* UNIMOC_TEST_CONTROL_MODE_H_ */
