/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file main.cpp
 *    @brief ztest coverage of the VDC node: readiness, wheel command, drive status and registers.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <reg/udral/service/common/Readiness_0_1.h>
#include <unimoc/cart/DriveStatus_0_1.h>
#include <unimoc/cart/WheelCommand_0_1.h>
#include <zephyr/ztest.h>
#include <array>
#include <cstddef>
#include <cstdint>
#include <new>
#include <optional>
#include <span>
#include <string_view>
#include "can_test_bus.hpp"
#include "cyphal/canard_transport.hpp"
#include "cyphal/cyphal_interface.hpp"
#include "cyphal/node.hpp"
#include "unimoc_test.hpp"
#include "vdc_node.hpp"

namespace unimoc::vdc::test {
namespace {

using namespace unit;
using unimoc::test::CanTestBus;
using unimoc::test::CanTestMedia;
using vehicle::Index;
using vehicle::kWheels;
using vehicle::Wheel;

constexpr uint8_t kVdcId = 1U;
/// The test node plays every drive: all wheels get its node-ID.
constexpr uint8_t kDrivesId = 20U;
constexpr uint64_t kStepUs = 1'000U;
constexpr Torque kTolerance = 1.0e-2_Nm;  // float16 on the bus

/// Records the readiness and the wheel commands the VDC publishes.
class DriveSide final : public cyphal::MessageListener {
 public:
  void OnMessage(const cyphal::Transfer& transfer, uint64_t /*now_us*/) noexcept override {
    const auto* const kBytes = reinterpret_cast<const uint8_t*>(transfer.payload.data());
    std::size_t size = transfer.payload.size();
    if (transfer.port_id == cyphal::kSubWheelCommand) {
      unimoc_cart_WheelCommand_0_1 command{};
      if (unimoc_cart_WheelCommand_0_1_deserialize_(&command, kBytes, &size) >= 0) {
        ++commands;
        for (const Wheel kWheel : kWheels) {
          torques[Index(kWheel)] = Torque{command.wheel_torque[Index(kWheel)]};
        }
      }
    } else {
      reg_udral_service_common_Readiness_0_1 readiness{};
      if (reg_udral_service_common_Readiness_0_1_deserialize_(&readiness, kBytes, &size) >= 0) {
        ++readiness_count;
        readiness_port = transfer.port_id;
        readiness_value = readiness.value;
      }
    }
  }

  std::size_t commands{0U};
  vehicle::PerWheel<Torque> torques{};
  std::size_t readiness_count{0U};
  uint16_t readiness_port{cyphal::kUnsetPortId};
  uint8_t readiness_value{0xFFU};
};

struct Rig {
  Rig()
      : vdc_media{bus, cyphal::kCanFdMtu},
        drives_media{bus, cyphal::kCanFdMtu},
        vdc_interfaces{&vdc_media},
        drives_interfaces{&drives_media},
        vdc_transport{vdc_arena.bytes, vdc_interfaces},
        drives_transport{drives_arena.bytes, drives_interfaces},
        node{vdc_transport},
        drives{drives_transport},
        vdc{node} {}

  /// Starts both nodes; the VDC takes the test node as the drive of every wheel.
  void Start() {
    zassert_true(vdc.Start(kVdcId, now));
    zassert_true(
        drives.Subscribe(cyphal::kSubWheelCommand, unimoc_cart_WheelCommand_0_1_EXTENT_BYTES_, side));
    zassert_true(drives.Subscribe(cyphal::EffectiveReadinessId(cyphal::kReadinessFollowsNodeId, kDrivesId),
                                  reg_udral_service_common_Readiness_0_1_EXTENT_BYTES_,
                                  side));
    zassert_true(drives.Start(kDrivesId, now));
    WriteNodeIds({kDrivesId, kDrivesId, kDrivesId, kDrivesId});
    Run(1U);  // the transports date their first frames by the clock of their first spin
  }

  /// Advances both nodes by @p steps of 1 ms.
  void Run(const unsigned steps) {
    for (unsigned i = 0U; i < steps; ++i) {
      now += kStepUs;
      vdc.Spin(now);
      drives.Spin(now);
    }
  }

  /// Publishes a drive status of @p wheel from the test node.
  void PublishStatus(const Wheel wheel, const bool engaged = true, const float drive_limit = 50.0F) {
    unimoc_cart_DriveStatus_0_1 status{};
    status.wheel = static_cast<uint8_t>(Index(wheel));
    status.wheel_speed = 2.0F;
    status.estimated_torque = 3.0F;
    status.drive_torque_limit = drive_limit;
    status.brake_torque_limit = 50.0F;
    status.engaged = engaged;
    status.power_stage = engaged ? unimoc_cart_DriveStatus_0_1_POWER_STAGE_ENABLED
                                 : unimoc_cart_DriveStatus_0_1_POWER_STAGE_DISABLED;
    std::array<std::byte, unimoc_cart_DriveStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_> buffer{};
    std::size_t size = buffer.size();
    zassert_true(unimoc_cart_DriveStatus_0_1_serialize_(&status,
                                                        reinterpret_cast<uint8_t*>(buffer.data()),
                                                        &size) >= 0);
    zassert_true(drives.Publish(status_publisher, std::span<const std::byte>{buffer.data(), size}));
  }

  /// All four drives report, then both nodes run for @p steps.
  void ReportAll(const unsigned steps = 5U, const bool engaged = true) {
    for (const Wheel kWheel : kWheels) {
      PublishStatus(kWheel, engaged);
    }
    Run(steps);
  }

  [[nodiscard]] std::optional<std::size_t> Find(const std::string_view name) const {
    for (std::size_t i = 0U; i < vdc.Count(); ++i) {
      if (vdc.Name(i) == name) {
        return i;
      }
    }
    return std::nullopt;
  }

  cyphal::RegisterWriteResult WriteReal(const std::string_view name, const float number) {
    cyphal::RegisterValue value{};
    cyphal::SetReal32(value, number);
    return vdc.Write(Find(name).value(), value);
  }

  cyphal::RegisterWriteResult WriteBit(const std::string_view name, const bool bit) {
    cyphal::RegisterValue value{};
    cyphal::SetBit(value, bit);
    return vdc.Write(Find(name).value(), value);
  }

  cyphal::RegisterWriteResult WriteNodeIds(const vehicle::PerWheel<uint8_t>& ids) {
    cyphal::RegisterValue value{};
    uavcan_register_Value_1_0_select_natural8_(&value);
    for (std::size_t i = 0U; i < ids.size(); ++i) {
      value.natural8.value.elements[i] = ids[i];
    }
    value.natural8.value.count = ids.size();
    return vdc.Write(Find(cyphal::reg::kVdcDriveNodeIds).value(), value);
  }

  cyphal::RegisterValue Read(const std::string_view name) const {
    cyphal::RegisterValue value{};
    (void)vdc.Read(Find(name).value(), value);
    return value;
  }

  cyphal::CanardArena<32768> vdc_arena;
  cyphal::CanardArena<32768> drives_arena;
  CanTestBus bus;
  CanTestMedia vdc_media;
  CanTestMedia drives_media;
  std::array<cyphal::CanMedia*, 1> vdc_interfaces;
  std::array<cyphal::CanMedia*, 1> drives_interfaces;
  cyphal::CanardTransport vdc_transport;
  cyphal::CanardTransport drives_transport;
  cyphal::CyphalNode node;
  cyphal::CyphalNode drives;
  VdcNode vdc;
  DriveSide side;
  cyphal::Publisher status_publisher{cyphal::kPubDriveStatus, cyphal::Priority::kFast, 0U};
  uint64_t now{1'000'000U};
};

Rig& MakeRig() {
  alignas(Rig) static std::array<std::byte, sizeof(Rig)> storage;
  static Rig* rig = nullptr;
  if (rig != nullptr) {
    rig->~Rig();
  }
  // NOLINTNEXTLINE(cppcoreguidelines-owning-memory): placement new into static storage.
  rig = new (storage.data()) Rig{};
  return *rig;
}

}  // namespace

ZTEST(vdc_node, test_defaults_and_port_registers) {
  Rig& rig = MakeRig();
  zassert_true(rig.vdc.Start(kVdcId, rig.now));
  // The default drives follow the simulated cluster: node-ID 10 + wheel.
  zexpect_equal(rig.vdc.ReadinessPort(Wheel::kFrontLeft), 1034U);
  zexpect_equal(rig.vdc.ReadinessPort(Wheel::kRearRight), 1037U);
  zexpect_equal(cyphal::GetInteger(rig.Read(cyphal::reg::kPubWheelCommandId)).value_or(-1), 110);
  zexpect_equal(cyphal::GetInteger(rig.Read(cyphal::reg::kSubDriveStatusId)).value_or(-1), 214);
  zexpect_equal(cyphal::GetInteger(rig.Read(cyphal::reg::kPubReadinessRearLeftId)).value_or(-1), 1036);
  zexpect_true(cyphal::GetString(rig.Read(cyphal::reg::kPubWheelCommandType)).value_or("") ==
               "unimoc.cart.WheelCommand.0.1");
  zexpect_true(cyphal::GetString(rig.Read(cyphal::reg::kPubReadinessFrontRightType)).value_or("") ==
               "reg.udral.service.common.Readiness.0.1");
  zexpect_equal(cyphal::GetInteger(rig.Read(cyphal::reg::kNodeId)).value_or(-1), kVdcId);
  // Every register name is unique and every port has its type register.
  for (std::size_t i = 0U; i < rig.vdc.Count(); ++i) {
    for (std::size_t j = i + 1U; j < rig.vdc.Count(); ++j) {
      zexpect_false(rig.vdc.Name(i) == rig.vdc.Name(j));
    }
  }
  zexpect_true(rig.vdc.Name(rig.vdc.Count()).empty());
}

ZTEST(vdc_node, test_standby_and_zero_torque_until_engaged) {
  Rig& rig = MakeRig();
  rig.Start();
  zexpect_equal(rig.WriteReal(cyphal::reg::kVdcTorqueRequest, 40.0F), cyphal::RegisterWriteResult::kAccepted);
  rig.ReportAll(200U, false);
  zexpect_true(rig.side.readiness_count > 0U);
  zexpect_equal(rig.side.readiness_port, 1024U + kDrivesId);
  zexpect_equal(rig.side.readiness_value, reg_udral_service_common_Readiness_0_1_STANDBY);
  // 250 Hz: about 50 commands in 200 ms, all zero.
  zexpect_true(rig.side.commands >= 45U && rig.side.commands <= 55U,
               "%u commands",
               static_cast<unsigned>(rig.side.commands));
  for (const Torque kTorque : rig.side.torques) {
    UNIMOC_EXPECT_NEAR(kTorque, 0.0_Nm, kTolerance);
  }
}

ZTEST(vdc_node, test_engaged_drives_get_the_split_torque) {
  Rig& rig = MakeRig();
  rig.Start();
  zexpect_equal(rig.WriteBit(cyphal::reg::kVdcEngage, true), cyphal::RegisterWriteResult::kAccepted);
  zexpect_equal(rig.WriteReal(cyphal::reg::kVdcTorqueRequest, 40.0F), cyphal::RegisterWriteResult::kAccepted);
  rig.Run(5U);
  zexpect_equal(rig.side.readiness_value, reg_udral_service_common_Readiness_0_1_ENGAGED);
  // Engaged, but no drive status yet: no wheel is available.
  for (const Torque kTorque : rig.side.torques) {
    UNIMOC_EXPECT_NEAR(kTorque, 0.0_Nm, kTolerance);
  }
  for (unsigned i = 0U; i < 4U; ++i) {
    rig.ReportAll(10U);
  }
  for (const Torque kTorque : rig.side.torques) {
    UNIMOC_EXPECT_NEAR(kTorque, 10.0_Nm, kTolerance);
  }
  for (const Wheel kWheel : kWheels) {
    zexpect_true(rig.vdc.Drives().Available(kWheel, rig.now));
    UNIMOC_EXPECT_NEAR(rig.vdc.WheelTorques()[Index(kWheel)], 10.0_Nm, kTolerance);
  }
  const cyphal::RegisterValue kAvailable = rig.Read(cyphal::reg::kVdcDriveAvailable);
  zexpect_equal(kAvailable.natural8.value.count, 4U);
  zexpect_equal(kAvailable.natural8.value.elements[3], 1U);
  const cyphal::RegisterValue kSpeed = rig.Read(cyphal::reg::kVdcWheelSpeed);
  zexpect_equal(kSpeed.real32.value.count, 4U);
  UNIMOC_EXPECT_NEAR(kSpeed.real32.value.elements[2], 2.0F, 1e-3F);

  // A front share of 0.75 moves torque to the front axle.
  zexpect_equal(rig.WriteReal(cyphal::reg::kVdcFrontShare, 0.75F), cyphal::RegisterWriteResult::kAccepted);
  rig.ReportAll(10U);
  UNIMOC_EXPECT_NEAR(rig.side.torques[Index(Wheel::kFrontLeft)], 15.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(rig.side.torques[Index(Wheel::kRearRight)], 5.0_Nm, kTolerance);

  // Disengaging zeroes the command and sends STANDBY.
  zexpect_equal(rig.WriteBit(cyphal::reg::kVdcEngage, false), cyphal::RegisterWriteResult::kAccepted);
  rig.ReportAll(10U);
  zexpect_equal(rig.side.readiness_value, reg_udral_service_common_Readiness_0_1_STANDBY);
  for (const Torque kTorque : rig.side.torques) {
    UNIMOC_EXPECT_NEAR(kTorque, 0.0_Nm, kTolerance);
  }
}

ZTEST(vdc_node, test_a_lost_drive_moves_the_torque_to_the_other_axle) {
  Rig& rig = MakeRig();
  rig.Start();
  (void)rig.WriteBit(cyphal::reg::kVdcEngage, true);
  (void)rig.WriteReal(cyphal::reg::kVdcTorqueRequest, 40.0F);
  rig.ReportAll(10U);
  // The rear right drive stops reporting; after the status timeout (0.1 s) its axle is out.
  for (unsigned i = 0U; i < 20U; ++i) {
    rig.PublishStatus(Wheel::kFrontLeft);
    rig.PublishStatus(Wheel::kFrontRight);
    rig.PublishStatus(Wheel::kRearLeft);
    rig.Run(10U);
  }
  zexpect_false(rig.vdc.Drives().Available(Wheel::kRearRight, rig.now));
  UNIMOC_EXPECT_NEAR(rig.side.torques[Index(Wheel::kRearLeft)], 0.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(rig.side.torques[Index(Wheel::kRearRight)], 0.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(rig.side.torques[Index(Wheel::kFrontLeft)], 20.0_Nm, kTolerance);
  UNIMOC_EXPECT_NEAR(rig.side.torques[Index(Wheel::kFrontRight)], 20.0_Nm, kTolerance);
}

ZTEST(vdc_node, test_status_of_an_unconfigured_node_is_ignored) {
  Rig& rig = MakeRig();
  rig.Start();
  // Only the rear left wheel has the test node as its drive.
  zexpect_equal(rig.WriteNodeIds({10U, 11U, kDrivesId, kNoDrive}), cyphal::RegisterWriteResult::kAccepted);
  zexpect_equal(rig.vdc.ReadinessPort(Wheel::kRearRight), cyphal::kUnsetPortId);
  rig.ReportAll(5U);
  zexpect_false(rig.vdc.Drives().Fresh(Wheel::kFrontLeft, rig.now));
  zexpect_true(rig.vdc.Drives().Fresh(Wheel::kRearLeft, rig.now));
  zexpect_false(rig.vdc.Drives().Fresh(Wheel::kRearRight, rig.now));
}

ZTEST(vdc_node, test_register_writes_are_checked) {
  Rig& rig = MakeRig();
  rig.Start();
  zexpect_equal(rig.WriteReal(cyphal::reg::kVdcFrontShare, 1.5F), cyphal::RegisterWriteResult::kOutOfRange);
  zexpect_equal(rig.WriteReal(cyphal::reg::kVdcCycleRate, 0.0F), cyphal::RegisterWriteResult::kOutOfRange);
  zexpect_equal(rig.WriteReal(cyphal::reg::kVdcStatusTimeout, 0.5F), cyphal::RegisterWriteResult::kAccepted);
  UNIMOC_EXPECT_NEAR(rig.vdc.Parameters().status_timeout, 0.5_s, 1e-6_s);
  zexpect_equal(rig.WriteNodeIds({200U, 11U, 12U, 13U}), cyphal::RegisterWriteResult::kOutOfRange);
  zexpect_equal(rig.WriteReal(cyphal::reg::kVdcWheelTorque, 1.0F), cyphal::RegisterWriteResult::kReadOnly);
  zexpect_equal(rig.WriteReal(cyphal::reg::kPubWheelCommandId, 300.0F),
                cyphal::RegisterWriteResult::kReadOnly);
}

ZTEST_SUITE(vdc_node, nullptr, nullptr, nullptr, nullptr, nullptr);

}  // namespace unimoc::vdc::test
