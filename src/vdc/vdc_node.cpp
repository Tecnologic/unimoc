/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file vdc_node.cpp
 *    @brief Cyphal application of the vehicle dynamics controller (VDC).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "vdc_node.hpp"
#include <reg/udral/service/common/Readiness_0_1.h>
#include <unimoc/cart/DriveStatus_0_1.h>
#include <unimoc/cart/WheelCommand_0_1.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <span>
#include "cyphal/cyphal_interface.hpp"
#include "torque_split.hpp"

namespace unimoc::vdc {

namespace {

using vehicle::Index;
using vehicle::kWheelCount;
using vehicle::kWheels;
using vehicle::Wheel;

enum class Register : uint8_t {
  kNodeId,
  kWheelCommandId,
  kWheelCommandType,
  kDriveStatusId,
  kDriveStatusType,
  kReadinessFrontLeftId,
  kReadinessFrontRightId,
  kReadinessRearLeftId,
  kReadinessRearRightId,
  kReadinessFrontLeftType,
  kReadinessFrontRightType,
  kReadinessRearLeftType,
  kReadinessRearRightType,
  kDriveNodeIds,
  kEngage,
  kTorqueRequest,
  kFrontShare,
  kCycleRate,
  kReadinessRate,
  kStatusTimeout,
  kDriveAvailable,
  kWheelSpeed,
  kEstimatedTorque,
  kWheelTorque,
  kCount,
};

constexpr std::array<std::string_view, static_cast<std::size_t>(Register::kCount)> kNames{
    cyphal::reg::kNodeId,
    cyphal::reg::kPubWheelCommandId,
    cyphal::reg::kPubWheelCommandType,
    cyphal::reg::kSubDriveStatusId,
    cyphal::reg::kSubDriveStatusType,
    cyphal::reg::kPubReadinessFrontLeftId,
    cyphal::reg::kPubReadinessFrontRightId,
    cyphal::reg::kPubReadinessRearLeftId,
    cyphal::reg::kPubReadinessRearRightId,
    cyphal::reg::kPubReadinessFrontLeftType,
    cyphal::reg::kPubReadinessFrontRightType,
    cyphal::reg::kPubReadinessRearLeftType,
    cyphal::reg::kPubReadinessRearRightType,
    cyphal::reg::kVdcDriveNodeIds,
    cyphal::reg::kVdcEngage,
    cyphal::reg::kVdcTorqueRequest,
    cyphal::reg::kVdcFrontShare,
    cyphal::reg::kVdcCycleRate,
    cyphal::reg::kVdcReadinessRate,
    cyphal::reg::kVdcStatusTimeout,
    cyphal::reg::kVdcDriveAvailable,
    cyphal::reg::kVdcWheelSpeed,
    cyphal::reg::kVdcEstimatedTorque,
    cyphal::reg::kVdcWheelTorque,
};

using cyphal::RegisterEffect;
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr std::array<cyphal::RegisterDescription, kNames.size()> kDescriptions{{
    {{}, 0.0F, 127.0F, RegisterEffect::kRestart, "Node-ID of the VDC"},
    {{}, 0.0F, 8191.0F, RegisterEffect::kRestart, "Wheel command subject"},
    {{}, -kInf, kInf, RegisterEffect::kImmediate, "Type of the wheel command"},
    {{}, 0.0F, 8191.0F, RegisterEffect::kRestart, "Drive status subject"},
    {{}, -kInf, kInf, RegisterEffect::kImmediate, "Type of the drive status"},
    {{}, 0.0F, 65535.0F, RegisterEffect::kImmediate, "Readiness subject of the front left drive"},
    {{}, 0.0F, 65535.0F, RegisterEffect::kImmediate, "Readiness subject of the front right drive"},
    {{}, 0.0F, 65535.0F, RegisterEffect::kImmediate, "Readiness subject of the rear left drive"},
    {{}, 0.0F, 65535.0F, RegisterEffect::kImmediate, "Readiness subject of the rear right drive"},
    {{}, -kInf, kInf, RegisterEffect::kImmediate, "Type of the readiness"},
    {{}, -kInf, kInf, RegisterEffect::kImmediate, "Type of the readiness"},
    {{}, -kInf, kInf, RegisterEffect::kImmediate, "Type of the readiness"},
    {{}, -kInf, kInf, RegisterEffect::kImmediate, "Type of the readiness"},
    {{}, 0.0F, 255.0F, RegisterEffect::kImmediate, "Node-ID of the drive of each wheel; 255: none"},
    {{}, 0.0F, 1.0F, RegisterEffect::kImmediate, "1: arm the drives (ENGAGED), 0: STANDBY"},
    {"N m", -kInf, kInf, RegisterEffect::kImmediate, "Total wheel torque, positive forward"},
    {{}, 0.0F, 1.0F, RegisterEffect::kImmediate, "Share of the front axle"},
    {"Hz", 1.0F, 1000.0F, RegisterEffect::kImmediate, "Wheel command rate"},
    {"Hz", 1.0F, 100.0F, RegisterEffect::kImmediate, "Readiness rate"},
    {"s", 0.001F, 10.0F, RegisterEffect::kImmediate, "Age after which a drive status counts as lost"},
    {{}, -kInf, kInf, RegisterEffect::kImmediate, "1 per wheel whose drive follows the VDC"},
    {"rad/s", -kInf, kInf, RegisterEffect::kImmediate, "Wheel speed of each drive status"},
    {"N m", -kInf, kInf, RegisterEffect::kImmediate, "Torque estimate of each drive status"},
    {"N m", -kInf, kInf, RegisterEffect::kImmediate, "Torque of each wheel in the last wheel command"},
}};

constexpr std::string_view kReadinessType = "reg.udral.service.common.Readiness.0.1";

[[nodiscard]] const uint8_t* InputBytes(const std::span<const std::byte> payload) noexcept {
  return reinterpret_cast<const uint8_t*>(payload.data());
}

/// Period of a rate [us]; 0 for a rate that is not positive.
[[nodiscard]] uint64_t PeriodMicroseconds(const unit::Frequency rate) noexcept {
  return (rate > unit::Frequency{}) ? static_cast<uint64_t>(1.0e6F / rate.Value()) : 0U;
}

/// Advances a deadline; true when it was due.
[[nodiscard]] bool Due(const uint64_t period_us, const uint64_t now_us, uint64_t& next_us) noexcept {
  if (period_us == 0U || now_us < next_us) {
    return false;
  }
  next_us += period_us;
  if (next_us <= now_us) {
    next_us = now_us + period_us;
  }
  return true;
}

void SetNatural8Array(cyphal::RegisterValue& value, const vehicle::PerWheel<uint8_t>& numbers) noexcept {
  uavcan_register_Value_1_0_select_natural8_(&value);
  std::copy(numbers.begin(), numbers.end(), std::begin(value.natural8.value.elements));
  value.natural8.value.count = numbers.size();
}

void SetReal32Array(cyphal::RegisterValue& value, const vehicle::PerWheel<float>& numbers) noexcept {
  uavcan_register_Value_1_0_select_real32_(&value);
  std::copy(numbers.begin(), numbers.end(), std::begin(value.real32.value.elements));
  value.real32.value.count = numbers.size();
}

/// A per-wheel value of the drive status as floats for a register.
template <typename Get>
[[nodiscard]] vehicle::PerWheel<float> PerWheelFloats(Get get) noexcept {
  vehicle::PerWheel<float> numbers{};
  for (const Wheel kWheel : kWheels) {
    numbers[Index(kWheel)] = get(kWheel);
  }
  return numbers;
}

/// The node-IDs of a natural8[4] (or any integer array of four) register value.
[[nodiscard]] std::optional<vehicle::PerWheel<uint8_t>> GetNodeIds(
    const cyphal::RegisterValue& value) noexcept {
  if (!uavcan_register_Value_1_0_is_natural8_(&value) || value.natural8.value.count != kWheelCount) {
    return std::nullopt;
  }
  vehicle::PerWheel<uint8_t> ids{};
  for (std::size_t i = 0U; i < kWheelCount; ++i) {
    const uint8_t kId = value.natural8.value.elements[i];
    if (kId > 127U && kId != kNoDrive) {
      return std::nullopt;
    }
    ids[i] = kId;
  }
  return ids;
}

/// A finite number of a register value within the limits of its description.
[[nodiscard]] std::optional<float> GetBounded(const cyphal::RegisterValue& value,
                                              const cyphal::RegisterDescription& description) noexcept {
  const std::optional<float> kNumber = cyphal::GetNumber(value);
  if (!kNumber.has_value() || !std::isfinite(*kNumber) || *kNumber < description.minimum ||
      *kNumber > description.maximum) {
    return std::nullopt;
  }
  return kNumber;
}

}  // namespace

uint16_t VdcNode::ReadinessPort(const Wheel wheel) const noexcept {
  const uint8_t kNodeId = parameters_.drive_node_ids[Index(wheel)];
  return (kNodeId == kNoDrive) ? cyphal::kUnsetPortId
                               : cyphal::EffectiveReadinessId(cyphal::kReadinessFollowsNodeId, kNodeId);
}

bool VdcNode::Start(const uint8_t node_id, const uint64_t now_us) noexcept {
  now_us_ = now_us;
  // The wheel command has the highest priority of the cart, the readiness after it (system plan 6.2).
  wheel_command_ = cyphal::Publisher{cyphal::kSubWheelCommand, cyphal::Priority::kImmediate, 0U};
  bool started = node_->AddRegisterProvider(*this);
  started =
      node_->Subscribe(cyphal::kPubDriveStatus, unimoc_cart_DriveStatus_0_1_EXTENT_BYTES_, *this) && started;
  started = node_->AddPublisher(wheel_command_) && started;
  for (const Wheel kWheel : kWheels) {
    started =
        node_->AddPublisher(cyphal::Publisher{ReadinessPort(kWheel), cyphal::Priority::kHigh, 0U}) && started;
  }
  started = node_->Start(node_id, now_us) && started;
  node_->SetMode(cyphal::Mode::kOperational);
  ApplyTiming();
  next_cycle_us_ = now_us;
  next_readiness_us_ = now_us;
  UpdateHealth(now_us);
  return started;
}

void VdcNode::Spin(const uint64_t now_us) noexcept {
  now_us_ = now_us;
  node_->Spin(now_us);  // dispatches the drive status and the register writes
  if (Due(readiness_period_us_, now_us, next_readiness_us_)) {
    PublishReadiness();
  }
  if (Due(cycle_period_us_, now_us, next_cycle_us_)) {
    PublishWheelCommand(now_us);
  }
  UpdateHealth(now_us);
}

void VdcNode::ApplyTiming() noexcept {
  cycle_period_us_ = PeriodMicroseconds(parameters_.cycle_rate);
  readiness_period_us_ = PeriodMicroseconds(parameters_.readiness_rate);
  drives_.SetTimeout(static_cast<uint64_t>(parameters_.status_timeout.Value() * 1.0e6F));
}

void VdcNode::OnMessage(const cyphal::Transfer& transfer, const uint64_t now_us) noexcept {
  if (transfer.port_id == cyphal::kPubDriveStatus) {
    OnDriveStatus(transfer, now_us);
  }
}

void VdcNode::OnDriveStatus(const cyphal::Transfer& transfer, const uint64_t now_us) noexcept {
  unimoc_cart_DriveStatus_0_1 status{};
  std::size_t size = transfer.payload.size();
  if (unimoc_cart_DriveStatus_0_1_deserialize_(&status, InputBytes(transfer.payload), &size) < 0 ||
      status.wheel >= kWheelCount) {
    return;
  }
  const Wheel kWheel = vehicle::WheelAt(status.wheel);
  // Only the drive configured for the wheel counts: a second drive with the same wheel index is ignored.
  if (transfer.source_node_id != parameters_.drive_node_ids[Index(kWheel)]) {
    return;
  }
  vehicle::DriveReport report;
  report.wheel_speed = unit::AngularVelocity{status.wheel_speed};
  report.estimated_torque = unit::Torque{status.estimated_torque};
  report.drive_torque_limit = unit::Torque{status.drive_torque_limit};
  report.brake_torque_limit = unit::Torque{status.brake_torque_limit};
  report.engaged = status.engaged;
  report.power_stage_enabled = status.power_stage == unimoc_cart_DriveStatus_0_1_POWER_STAGE_ENABLED;
  report.limp_home = status.limp_home;
  report.tripped = status.overcurrent_tripped || status.overvoltage_tripped;
  drives_.OnReport(kWheel, report, now_us);
}

void VdcNode::PublishWheelCommand(const uint64_t now_us) noexcept {
  // Torque only to drives the VDC engages; a drive engaged by someone else gets zero.
  const unit::Torque kRequest = parameters_.engage ? parameters_.torque_request : unit::Torque{};
  wheel_torques_ = vehicle::SplitTorque(kRequest, parameters_.front_share, drives_.Windows(now_us));
  unimoc_cart_WheelCommand_0_1 command{};
  for (const Wheel kWheel : kWheels) {
    command.wheel_torque[Index(kWheel)] = wheel_torques_[Index(kWheel)].Value();
  }
  std::array<std::byte, unimoc_cart_WheelCommand_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_> buffer{};
  std::size_t size = buffer.size();
  if (unimoc_cart_WheelCommand_0_1_serialize_(&command, reinterpret_cast<uint8_t*>(buffer.data()), &size) >=
      0) {
    (void)node_->Publish(wheel_command_, std::span<const std::byte>{buffer.data(), size});
  }
}

void VdcNode::PublishReadiness() noexcept {
  reg_udral_service_common_Readiness_0_1 readiness{};
  readiness.value = parameters_.engage ? reg_udral_service_common_Readiness_0_1_ENGAGED
                                       : reg_udral_service_common_Readiness_0_1_STANDBY;
  std::array<std::byte, reg_udral_service_common_Readiness_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_> buffer{};
  std::size_t size = buffer.size();
  if (reg_udral_service_common_Readiness_0_1_serialize_(&readiness,
                                                        reinterpret_cast<uint8_t*>(buffer.data()),
                                                        &size) < 0) {
    return;
  }
  for (const Wheel kWheel : kWheels) {
    cyphal::Publisher publisher{ReadinessPort(kWheel),
                                cyphal::Priority::kHigh,
                                readiness_transfer_ids_[Index(kWheel)]};
    if (publisher.port_id == cyphal::kUnsetPortId) {
      continue;
    }
    (void)node_->Publish(publisher, std::span<const std::byte>{buffer.data(), size});
    readiness_transfer_ids_[Index(kWheel)] = publisher.next_transfer_id;
  }
}

void VdcNode::UpdateHealth(const uint64_t now_us) noexcept {
  bool nominal = true;
  for (const Wheel kWheel : kWheels) {
    if (parameters_.drive_node_ids[Index(kWheel)] == kNoDrive) {
      continue;
    }
    nominal =
        nominal && (parameters_.engage ? drives_.Available(kWheel, now_us) : drives_.Fresh(kWheel, now_us));
  }
  node_->SetHealth(nominal ? cyphal::Health::kNominal : cyphal::Health::kAdvisory);
}

std::size_t VdcNode::Count() const noexcept { return kNames.size(); }

std::string_view VdcNode::Name(const std::size_t index) const noexcept {
  return (index < kNames.size()) ? kNames[index] : std::string_view{};
}

cyphal::RegisterDescription VdcNode::Describe(const std::size_t index) const noexcept {
  return (index < kDescriptions.size()) ? kDescriptions[index] : cyphal::RegisterDescription{};
}

cyphal::RegisterFlags VdcNode::Read(const std::size_t index, cyphal::RegisterValue& value) const noexcept {
  constexpr cyphal::RegisterFlags kReadOnly{false, false};
  constexpr cyphal::RegisterFlags kWritable{true, false};
  switch (static_cast<Register>(index)) {
    case Register::kNodeId:
      cyphal::SetNatural16(value, node_->NodeId());
      return kReadOnly;
    case Register::kWheelCommandId:
      cyphal::SetNatural16(value, wheel_command_.port_id);
      return kReadOnly;
    case Register::kWheelCommandType:
      cyphal::SetString(value, "unimoc.cart.WheelCommand.0.1");
      return kReadOnly;
    case Register::kDriveStatusId:
      cyphal::SetNatural16(value, cyphal::kPubDriveStatus);
      return kReadOnly;
    case Register::kDriveStatusType:
      cyphal::SetString(value, "unimoc.cart.DriveStatus.0.1");
      return kReadOnly;
    case Register::kReadinessFrontLeftId:
    case Register::kReadinessFrontRightId:
    case Register::kReadinessRearLeftId:
    case Register::kReadinessRearRightId:
      cyphal::SetNatural16(value,
                           ReadinessPort(vehicle::WheelAt(
                               index - static_cast<std::size_t>(Register::kReadinessFrontLeftId))));
      return kReadOnly;
    case Register::kReadinessFrontLeftType:
    case Register::kReadinessFrontRightType:
    case Register::kReadinessRearLeftType:
    case Register::kReadinessRearRightType:
      cyphal::SetString(value, kReadinessType);
      return kReadOnly;
    case Register::kDriveNodeIds:
      SetNatural8Array(value, parameters_.drive_node_ids);
      return kWritable;
    case Register::kEngage:
      cyphal::SetBit(value, parameters_.engage);
      return kWritable;
    case Register::kTorqueRequest:
      cyphal::SetReal32(value, parameters_.torque_request.Value());
      return kWritable;
    case Register::kFrontShare:
      cyphal::SetReal32(value, parameters_.front_share.Value());
      return kWritable;
    case Register::kCycleRate:
      cyphal::SetReal32(value, parameters_.cycle_rate.Value());
      return kWritable;
    case Register::kReadinessRate:
      cyphal::SetReal32(value, parameters_.readiness_rate.Value());
      return kWritable;
    case Register::kStatusTimeout:
      cyphal::SetReal32(value, parameters_.status_timeout.Value());
      return kWritable;
    case Register::kDriveAvailable: {
      vehicle::PerWheel<uint8_t> available{};
      for (const Wheel kWheel : kWheels) {
        available[Index(kWheel)] = drives_.Available(kWheel, now_us_) ? 1U : 0U;
      }
      SetNatural8Array(value, available);
      return kReadOnly;
    }
    case Register::kWheelSpeed:
      SetReal32Array(value, PerWheelFloats([this](const Wheel wheel) noexcept {
                       return drives_.Report(wheel).wheel_speed.Value();
                     }));
      return kReadOnly;
    case Register::kEstimatedTorque:
      SetReal32Array(value, PerWheelFloats([this](const Wheel wheel) noexcept {
                       return drives_.Report(wheel).estimated_torque.Value();
                     }));
      return kReadOnly;
    case Register::kWheelTorque:
      SetReal32Array(value, PerWheelFloats([this](const Wheel wheel) noexcept {
                       return wheel_torques_[Index(wheel)].Value();
                     }));
      return kReadOnly;
    case Register::kCount:
      break;
  }
  cyphal::SetEmpty(value);
  return kReadOnly;
}

cyphal::RegisterWriteResult VdcNode::Write(const std::size_t index,
                                           const cyphal::RegisterValue& value) noexcept {
  const cyphal::RegisterDescription kDescription = Describe(index);
  const std::optional<float> kNumber = GetBounded(value, kDescription);
  switch (static_cast<Register>(index)) {
    case Register::kDriveNodeIds: {
      const std::optional<vehicle::PerWheel<uint8_t>> kIds = GetNodeIds(value);
      if (!kIds.has_value()) {
        return cyphal::RegisterWriteResult::kOutOfRange;
      }
      parameters_.drive_node_ids = *kIds;
      drives_.Reset();  // the reports of the old drives do not count for the new ones
      return cyphal::RegisterWriteResult::kAccepted;
    }
    case Register::kEngage:
      if (!kNumber.has_value()) {
        return cyphal::RegisterWriteResult::kOutOfRange;
      }
      parameters_.engage = *kNumber != 0.0F;
      next_readiness_us_ = now_us_;  // the drives see the change at once
      return cyphal::RegisterWriteResult::kAccepted;
    case Register::kTorqueRequest:
      if (!kNumber.has_value()) {
        return cyphal::RegisterWriteResult::kOutOfRange;
      }
      parameters_.torque_request = unit::Torque{*kNumber};
      return cyphal::RegisterWriteResult::kAccepted;
    case Register::kFrontShare:
      if (!kNumber.has_value()) {
        return cyphal::RegisterWriteResult::kOutOfRange;
      }
      parameters_.front_share = unit::Ratio{*kNumber};
      return cyphal::RegisterWriteResult::kAccepted;
    case Register::kCycleRate:
    case Register::kReadinessRate:
    case Register::kStatusTimeout:
      if (!kNumber.has_value()) {
        return cyphal::RegisterWriteResult::kOutOfRange;
      }
      if (static_cast<Register>(index) == Register::kCycleRate) {
        parameters_.cycle_rate = unit::Frequency{*kNumber};
      } else if (static_cast<Register>(index) == Register::kReadinessRate) {
        parameters_.readiness_rate = unit::Frequency{*kNumber};
      } else {
        parameters_.status_timeout = unit::Time{*kNumber};
      }
      ApplyTiming();
      return cyphal::RegisterWriteResult::kAccepted;
    default:
      return cyphal::RegisterWriteResult::kReadOnly;
  }
}

}  // namespace unimoc::vdc
