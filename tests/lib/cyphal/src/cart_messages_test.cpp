/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file cart_messages_test.cpp
 *    @brief ztest coverage for the UNIMOC DSDL messages: one CAN FD frame each, cart message round trips.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

/**
 * @file cart_messages_test.cpp
 * @brief Frame size of every message of dsdl/unimoc and serialization round trips of the cart messages.
 */
#include <unimoc/cart/BatteryStatus_0_1.h>
#include <unimoc/cart/DriveStatus_0_1.h>
#include <unimoc/cart/PedalState_0_1.h>
#include <unimoc/cart/WheelCommand_0_1.h>
#include <unimoc/drive/CurrentSetpoint_0_1.h>
#include <unimoc/drive/Telemetry_0_1.h>
#include <unimoc/sim/Plant_0_1.h>
#include <zephyr/ztest.h>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include "unimoc_test.hpp"

namespace unimoc::cyphal::test {
namespace {

/// Payload of one CAN FD frame without the tail byte of Cyphal/CAN.
constexpr size_t kCanFdSingleFramePayload = 63U;

// Every message of the UNIMOC namespace fits in one CAN FD frame (docs/system_plan.md, section 4).
static_assert(unimoc_cart_WheelCommand_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_ <= kCanFdSingleFramePayload);
static_assert(unimoc_cart_DriveStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_ <= kCanFdSingleFramePayload);
static_assert(unimoc_cart_PedalState_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_ <= kCanFdSingleFramePayload);
static_assert(unimoc_cart_BatteryStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_ <= kCanFdSingleFramePayload);
static_assert(unimoc_drive_CurrentSetpoint_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_ <= kCanFdSingleFramePayload);
static_assert(unimoc_drive_Telemetry_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_ <= kCanFdSingleFramePayload);
static_assert(unimoc_sim_Plant_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_ <= kCanFdSingleFramePayload);

// The cart messages leave room to grow within the frame (@extent): a receiver accepts a longer minor version.
static_assert(unimoc_cart_WheelCommand_0_1_EXTENT_BYTES_ == kCanFdSingleFramePayload);
static_assert(unimoc_cart_DriveStatus_0_1_EXTENT_BYTES_ == kCanFdSingleFramePayload);
static_assert(unimoc_cart_PedalState_0_1_EXTENT_BYTES_ == kCanFdSingleFramePayload);
static_assert(unimoc_cart_BatteryStatus_0_1_EXTENT_BYTES_ == kCanFdSingleFramePayload);

/// Relative resolution of float16 (11 significant bits) with some margin.
constexpr float kFloat16Resolution = 1.0e-3F;

}  // namespace

ZTEST(cart_messages, test_wheel_command_round_trip) {
  unimoc_cart_WheelCommand_0_1 sent{};
  sent.wheel_torque[unimoc_cart_WheelCommand_0_1_WHEEL_FRONT_LEFT] = 12.5F;
  sent.wheel_torque[unimoc_cart_WheelCommand_0_1_WHEEL_FRONT_RIGHT] = 10.25F;
  sent.wheel_torque[unimoc_cart_WheelCommand_0_1_WHEEL_REAR_LEFT] = -30.0F;
  sent.wheel_torque[unimoc_cart_WheelCommand_0_1_WHEEL_REAR_RIGHT] = 0.0F;

  std::array<uint8_t, unimoc_cart_WheelCommand_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_> buffer{};
  size_t size = buffer.size();
  zassert_true(unimoc_cart_WheelCommand_0_1_serialize_(&sent, buffer.data(), &size) >= 0);
  zexpect_equal(size, 8U);

  unimoc_cart_WheelCommand_0_1 received{};
  zassert_true(unimoc_cart_WheelCommand_0_1_deserialize_(&received, buffer.data(), &size) >= 0);
  for (size_t wheel = 0U; wheel < 4U; ++wheel) {
    UNIMOC_EXPECT_NEAR(received.wheel_torque[wheel], sent.wheel_torque[wheel], 30.0F * kFloat16Resolution);
  }
}

ZTEST(cart_messages, test_drive_status_round_trip) {
  unimoc_cart_DriveStatus_0_1 sent{};
  sent.wheel_speed = 52.0F;
  sent.estimated_torque = -18.5F;
  sent.drive_torque_limit = 40.0F;
  sent.brake_torque_limit = 22.0F;
  sent.dc_link_voltage = 51.2F;
  sent.dc_link_current = -9.5F;
  sent.power_stage_temperature = 318.15F;
  sent.motor_temperature = NAN;
  sent.wheel = unimoc_cart_WheelCommand_0_1_WHEEL_REAR_RIGHT;
  sent.power_stage = unimoc_cart_DriveStatus_0_1_POWER_STAGE_ENABLED;
  sent.engaged = true;
  sent.regen_derated = true;
  sent.overvoltage_tripped = true;

  std::array<uint8_t, unimoc_cart_DriveStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_> buffer{};
  size_t size = buffer.size();
  zassert_true(unimoc_cart_DriveStatus_0_1_serialize_(&sent, buffer.data(), &size) >= 0);
  zexpect_equal(size, 18U);

  unimoc_cart_DriveStatus_0_1 received{};
  zassert_true(unimoc_cart_DriveStatus_0_1_deserialize_(&received, buffer.data(), &size) >= 0);
  UNIMOC_EXPECT_NEAR(received.wheel_speed, sent.wheel_speed, 52.0F * kFloat16Resolution);
  UNIMOC_EXPECT_NEAR(received.estimated_torque, sent.estimated_torque, 18.5F * kFloat16Resolution);
  UNIMOC_EXPECT_NEAR(received.drive_torque_limit, sent.drive_torque_limit, 40.0F * kFloat16Resolution);
  UNIMOC_EXPECT_NEAR(received.brake_torque_limit, sent.brake_torque_limit, 22.0F * kFloat16Resolution);
  UNIMOC_EXPECT_NEAR(received.dc_link_voltage, sent.dc_link_voltage, 51.2F * kFloat16Resolution);
  UNIMOC_EXPECT_NEAR(received.dc_link_current, sent.dc_link_current, 9.5F * kFloat16Resolution);
  UNIMOC_EXPECT_NEAR(received.power_stage_temperature,
                     sent.power_stage_temperature,
                     318.15F * kFloat16Resolution);
  zexpect_true(std::isnan(received.motor_temperature), "an unmeasured value stays NaN");
  zexpect_equal(received.wheel, unimoc_cart_WheelCommand_0_1_WHEEL_REAR_RIGHT);
  zexpect_equal(received.power_stage, unimoc_cart_DriveStatus_0_1_POWER_STAGE_ENABLED);
  zexpect_true(received.engaged);
  zexpect_false(received.limp_home);
  zexpect_true(received.regen_derated);
  zexpect_false(received.overcurrent_tripped);
  zexpect_true(received.overvoltage_tripped);
}

ZTEST(cart_messages, test_pedal_state_round_trip) {
  unimoc_cart_PedalState_0_1 sent{};
  sent.drive_pedal[0] = 0.25F;
  sent.drive_pedal[1] = 0.26F;
  sent.brake_pedal[0] = 1.0F;
  sent.brake_pedal[1] = NAN;
  sent.drive_pedal_plausible = true;
  sent.brake_pedal_plausible = false;

  std::array<uint8_t, unimoc_cart_PedalState_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_> buffer{};
  size_t size = buffer.size();
  zassert_true(unimoc_cart_PedalState_0_1_serialize_(&sent, buffer.data(), &size) >= 0);
  zexpect_equal(size, 9U);

  unimoc_cart_PedalState_0_1 received{};
  zassert_true(unimoc_cart_PedalState_0_1_deserialize_(&received, buffer.data(), &size) >= 0);
  UNIMOC_EXPECT_NEAR(received.drive_pedal[0], sent.drive_pedal[0], kFloat16Resolution);
  UNIMOC_EXPECT_NEAR(received.drive_pedal[1], sent.drive_pedal[1], kFloat16Resolution);
  UNIMOC_EXPECT_FLOAT_EQ(received.brake_pedal[0], 1.0F);
  zexpect_true(std::isnan(received.brake_pedal[1]), "a failed channel stays NaN");
  zexpect_true(received.drive_pedal_plausible);
  zexpect_false(received.brake_pedal_plausible);
}

ZTEST(cart_messages, test_battery_status_round_trip) {
  unimoc_cart_BatteryStatus_0_1 sent{};
  sent.voltage = 53.6F;
  sent.current = -20.0F;
  sent.state_of_charge = 0.95F;
  sent.discharge_current_limit = 50.0F;
  sent.charge_current_limit = 5.0F;
  sent.cell_temperature = 298.15F;
  sent.chopper_temperature = 350.0F;
  sent.contactor = unimoc_cart_BatteryStatus_0_1_CONTACTOR_OPENING;
  sent.chopper_active = true;

  std::array<uint8_t, unimoc_cart_BatteryStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_> buffer{};
  size_t size = buffer.size();
  zassert_true(unimoc_cart_BatteryStatus_0_1_serialize_(&sent, buffer.data(), &size) >= 0);
  zexpect_equal(size, 15U);

  unimoc_cart_BatteryStatus_0_1 received{};
  zassert_true(unimoc_cart_BatteryStatus_0_1_deserialize_(&received, buffer.data(), &size) >= 0);
  UNIMOC_EXPECT_NEAR(received.voltage, sent.voltage, 53.6F * kFloat16Resolution);
  UNIMOC_EXPECT_NEAR(received.current, sent.current, 20.0F * kFloat16Resolution);
  UNIMOC_EXPECT_NEAR(received.state_of_charge, sent.state_of_charge, kFloat16Resolution);
  UNIMOC_EXPECT_NEAR(received.discharge_current_limit,
                     sent.discharge_current_limit,
                     50.0F * kFloat16Resolution);
  UNIMOC_EXPECT_NEAR(received.charge_current_limit, sent.charge_current_limit, 5.0F * kFloat16Resolution);
  UNIMOC_EXPECT_NEAR(received.cell_temperature, sent.cell_temperature, 298.15F * kFloat16Resolution);
  UNIMOC_EXPECT_NEAR(received.chopper_temperature, sent.chopper_temperature, 350.0F * kFloat16Resolution);
  zexpect_equal(received.contactor, unimoc_cart_BatteryStatus_0_1_CONTACTOR_OPENING);
  zexpect_true(received.chopper_active);
}

ZTEST_SUITE(cart_messages, nullptr, nullptr, nullptr, nullptr, nullptr);

}  // namespace unimoc::cyphal::test
