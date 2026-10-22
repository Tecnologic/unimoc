/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file drive_node.cpp
 *    @brief Cyphal application of the drive: arming, setpoints, telemetry,
 *           settings registers and node commands.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "drive_node.hpp"
#include <reg/udral/physics/dynamics/rotation/Planar_0_1.h>
#include <reg/udral/service/common/Readiness_0_1.h>
#include <uavcan/si/unit/angle/Scalar_1_0.h>
#include <uavcan/si/unit/angular_velocity/Scalar_1_0.h>
#include <uavcan/si/unit/torque/Scalar_1_0.h>
#include <unimoc/drive/CurrentSetpoint_0_1.h>
#include <unimoc/drive/Telemetry_0_1.h>
#include <array>
#include <cmath>
#include <optional>
#include "angle_wrap.hpp"
#include "current_control_isr.hpp"
#include "current_control_runtime.hpp"
#include "cyphal/control_mode.hpp"
#include "power_stage_interface.hpp"
#include "sin_cos.hpp"

namespace unimoc::drive {

namespace {

using current_control::CurrentControlIsr;
using current_control::OuterLoop;
using current_control::OuterSetpoint;
using hardware::kPowerStage;
using hardware::PowerStageState;

struct InfoRegister {
  std::string_view name;
  std::string_view type;  ///< Empty: the unique-ID register.
};

constexpr std::array<InfoRegister, 15> kInfoRegisters{{
    {cyphal::reg::kSubReadinessType, "reg.udral.service.common.Readiness.0.1"},
    {cyphal::reg::kSubCurrentSetpointType, "unimoc.drive.CurrentSetpoint.0.1"},
    {cyphal::reg::kSubTorqueSetpointType, "uavcan.si.unit.torque.Scalar.1.0"},
    {cyphal::reg::kSubSpeedSetpointType, "uavcan.si.unit.angular_velocity.Scalar.1.0"},
    {cyphal::reg::kSubPositionSetpointType, "uavcan.si.unit.angle.Scalar.1.0"},
    {cyphal::reg::kSubServoSetpointType, "reg.udral.physics.dynamics.rotation.Planar.0.1"},
    {cyphal::reg::kPubTelemetryType, "unimoc.drive.Telemetry.0.1"},
    {cyphal::reg::kSrvCymonVariablesType, "cymon.GetVariableList.0.1"},
    {cyphal::reg::kSrvCymonBufferInfoType, "cymon.GetBufferInfo.0.1"},
    {cyphal::reg::kSrvCymonSetupType, "cymon.SetupCapture.0.1"},
    {cyphal::reg::kSrvCymonArmType, "cymon.ArmTrigger.0.1"},
    {cyphal::reg::kSrvCymonReadType, "cymon.ReadSamples.0.1"},
    {cyphal::reg::kPubCymonTriggerType, "cymon.TriggerEvent.0.1"},
    {cyphal::reg::kSubCymonTriggerType, "cymon.TriggerEvent.0.1"},
    {cyphal::reg::kNodeUniqueId, ""},
}};

[[nodiscard]] const uint8_t* InputBytes(std::span<const std::byte> payload) noexcept {
  return reinterpret_cast<const uint8_t*>(payload.data());
}

[[nodiscard]] uint64_t ToMicroseconds(const unit::Time time) noexcept {
  return static_cast<uint64_t>(time.Value() * 1.0e6F);
}

}  // namespace

// ----- DriveInfoRegisters ----------------------------------------------------

std::size_t DriveInfoRegisters::Count() const noexcept { return kInfoRegisters.size(); }

std::string_view DriveInfoRegisters::Name(const std::size_t index) const noexcept {
  return index < kInfoRegisters.size() ? kInfoRegisters[index].name : std::string_view{};
}

cyphal::RegisterFlags DriveInfoRegisters::Read(const std::size_t index,
                                               cyphal::RegisterValue& value) const noexcept {
  if (index >= kInfoRegisters.size()) {
    cyphal::SetEmpty(value);
  } else if (kInfoRegisters[index].type.empty()) {
    cyphal::SetUnstructured(value, unique_id_);
    return {false, true};
  } else {
    cyphal::SetString(value, kInfoRegisters[index].type);
  }
  return {false, false};
}

cyphal::RegisterWriteResult DriveInfoRegisters::Write(std::size_t /*index*/,
                                                      const cyphal::RegisterValue& /*value*/) noexcept {
  return cyphal::RegisterWriteResult::kReadOnly;
}

// ----- DriveNode ---------------------------------------------------------------

DriveNode::DriveNode(cyphal::CyphalNode& node,
                     settings::SettingsStore& store,
                     const hardware::HardwareInterface& motor,
                     const DriveHooks& hooks,
                     const std::span<const uint8_t> unique_id) noexcept
    : node_{&node},
      store_{&store},
      motor_{&motor},
      hooks_{hooks},
      settings_registers_{store},
      info_registers_{unique_id},
      startup_{CurrentControlIsr::Instance(), store.GetOperations()},
      startup_registers_{startup_, engaged_} {}

bool DriveNode::Start(const uint64_t now_us) noexcept {
  now_us_ = now_us;
  const settings::SettingsSnapshot kSnapshot = store_->GetSnapshot();
  const settings::NvmSettings& live = kSnapshot.Get();
  readiness_port_ = live.sub_readiness_id;
  current_setpoint_port_ = live.sub_current_setpoint_id;
  torque_setpoint_port_ = live.sub_torque_setpoint_id;
  speed_setpoint_port_ = live.sub_speed_setpoint_id;
  position_setpoint_port_ = live.sub_position_setpoint_id;
  servo_setpoint_port_ = live.sub_servo_setpoint_id;
  telemetry_ = cyphal::Publisher{live.pub_telemetry_id, cyphal::Priority::kHigh, 0U};
  // The runtime was started by main() with these settings.
  applied_writes_ = settings_registers_.WriteCount();
  LoadParameters(live);

  bool subscribed = node_->AddRegisterProvider(settings_registers_);
  subscribed = node_->AddRegisterProvider(info_registers_) && subscribed;
  subscribed = node_->AddRegisterProvider(startup_registers_) && subscribed;
  subscribed =
      node_->Subscribe(readiness_port_, reg_udral_service_common_Readiness_0_1_EXTENT_BYTES_, *this) &&
      subscribed;
  subscribed =
      node_->Subscribe(current_setpoint_port_, unimoc_drive_CurrentSetpoint_0_1_EXTENT_BYTES_, *this) &&
      subscribed;
  subscribed =
      node_->Subscribe(torque_setpoint_port_, uavcan_si_unit_torque_Scalar_1_0_EXTENT_BYTES_, *this) &&
      subscribed;
  subscribed = node_->Subscribe(speed_setpoint_port_,
                                uavcan_si_unit_angular_velocity_Scalar_1_0_EXTENT_BYTES_,
                                *this) &&
               subscribed;
  subscribed =
      node_->Subscribe(position_setpoint_port_, uavcan_si_unit_angle_Scalar_1_0_EXTENT_BYTES_, *this) &&
      subscribed;
  subscribed = node_->Subscribe(servo_setpoint_port_,
                                reg_udral_physics_dynamics_rotation_Planar_0_1_EXTENT_BYTES_,
                                *this) &&
               subscribed;
  subscribed = node_->AddPublisher(telemetry_) && subscribed;
  node_->SetCommandHandler(*this);
  node_->SetNodeIdListener(*this);

  // 0 is "not assigned" in the settings: request a node-ID by plug-and-play.
  const uint8_t kNodeId = (live.node_id == 0U) ? cyphal::kNodeIdUnset : live.node_id;
  subscribed = node_->Start(kNodeId, now_us) && subscribed;
  node_->SetMode(cyphal::Mode::kOperational);
  next_telemetry_us_ = now_us;
  UpdateStatus();
  return subscribed;
}

void DriveNode::Spin(const uint64_t now_us) noexcept {
  now_us_ = now_us;
  node_->Spin(now_us);  // dispatches the received messages and commands

  if (engage_requested_) {
    engage_requested_ = false;
    Engage();
  }
  SpinStartup();

  const auto& isr = CurrentControlIsr::Instance();
  if (engaged_) {
    if (now_us - last_readiness_us_ > command_timeout_us_) {
      Disengage("readiness timeout");
    } else if (kPowerStage.GetState() != PowerStageState::kEnabled || isr.state.overcurrent_tripped) {
      rearm_required_ = true;
      Disengage(isr.state.overcurrent_tripped ? "overcurrent trip" : "power stage fault");
    }
  }
  if (engaged_ && setpoint_active_ && now_us - last_setpoint_us_ > command_timeout_us_) {
    ClearSetpoint();
  }
  // The outer loop commands zero current while the drive is disengaged.
  OuterSetpoint setpoint = setpoint_;
  setpoint.active = engaged_;
  OuterLoop::Instance().SetSetpoint(setpoint);

  if (!engaged_ && settings_registers_.WriteCount() != applied_writes_) {
    settings_pending_ = true;
  }
  if (!engaged_ && settings_pending_) {
    (void)ApplySettings();
  }

  if (telemetry_period_us_ > 0U && now_us >= next_telemetry_us_) {
    PublishTelemetry();
    next_telemetry_us_ += telemetry_period_us_;
    if (next_telemetry_us_ <= now_us) {
      next_telemetry_us_ = now_us + telemetry_period_us_;
    }
  }

  if (restart_requested_ && now_us >= restart_at_us_ && hooks_.restart != nullptr) {
    restart_requested_ = false;
    hooks_.restart();
  }
  UpdateStatus();
}

void DriveNode::LoadParameters(const settings::NvmSettings& settings) noexcept {
  command_timeout_us_ = ToMicroseconds(settings.command_timeout);
  telemetry_period_us_ = (settings.telemetry_rate.Value() > 0.0F)
                             ? static_cast<uint64_t>(1.0e6F / settings.telemetry_rate.Value())
                             : 0U;
  current_limit_ = settings.motor_i_max.Value();
  // PMSM torque constant: T = 3/2 p psi_pm i_q.
  torque_constant_ = 1.5F * static_cast<float>(settings.pole_pairs) * settings.flux_pm.Value();
}

void DriveNode::SpinStartup() noexcept {
  if (startup_.IsActive()) {
    startup_active_ = true;
    startup_.RunOnce();  // one sample of the control cycle per spin
    return;
  }
  if (startup_active_) {
    // DONE or FAULT: the outputs are off again. DONE wrote the calibration
    // to the settings; the controller restarts with it. A FAULT leaves the
    // control cycle at forced neutral duties: back to the control loop.
    startup_active_ = false;
    CurrentControlIsr::Instance().ReleaseForceDuty();
    settings_pending_ = true;
  }
}

bool DriveNode::ApplySettings() noexcept {
  if (kPowerStage.GetState() == PowerStageState::kEnabled) {
    // Not while the outputs are released (engaged, startup aid): the next
    // engage or the end of the startup run applies them.
    settings_pending_ = true;
    return false;
  }
  applied_writes_ = settings_registers_.WriteCount();
  settings_pending_ = false;
  const settings::SettingsSnapshot kSnapshot = store_->GetSnapshot();
  const settings::NvmSettings& live = kSnapshot.Get();
  LoadParameters(live);

  if (!current_control::StartRuntime(live, *motor_, store_->GetProfile().capabilities.max_phase_current)) {
    (void)node_->Log(cyphal::Severity::kError, "settings rejected by the hardware (PWM frequency)");
    return false;
  }
  if (hooks_.settings_applied != nullptr) {
    hooks_.settings_applied(live);
  }
  return true;
}

void DriveNode::OnMessage(const cyphal::Transfer& transfer, const uint64_t now_us) noexcept {
  if (transfer.port_id == readiness_port_) {
    OnReadiness(transfer, now_us);
  } else if (transfer.port_id == current_setpoint_port_) {
    OnCurrentSetpoint(transfer, now_us);
  } else if (transfer.port_id == torque_setpoint_port_) {
    OnTorqueSetpoint(transfer, now_us);
  } else if (transfer.port_id == speed_setpoint_port_) {
    OnSpeedSetpoint(transfer, now_us);
  } else if (transfer.port_id == position_setpoint_port_) {
    OnPositionSetpoint(transfer, now_us);
  } else if (transfer.port_id == servo_setpoint_port_) {
    OnServoSetpoint(transfer, now_us);
  }
}

void DriveNode::OnReadiness(const cyphal::Transfer& transfer, const uint64_t now_us) noexcept {
  reg_udral_service_common_Readiness_0_1 readiness{};
  std::size_t size = transfer.payload.size();
  if (reg_udral_service_common_Readiness_0_1_deserialize_(&readiness, InputBytes(transfer.payload), &size) <
      0) {
    return;
  }
  last_readiness_us_ = now_us;
  if (readiness.value == reg_udral_service_common_Readiness_0_1_ENGAGED) {
    if (!engaged_ && !rearm_required_) {
      engage_requested_ = true;  // engaged in Spin(), outside the dispatch
    }
  } else {
    rearm_required_ = false;
    if (engaged_) {
      Disengage("readiness");
    } else if (readiness.value == reg_udral_service_common_Readiness_0_1_STANDBY) {
      // STANDBY acknowledges a latched trip or fault without releasing the
      // outputs, so a flash save is possible again (the save gate needs kDisabled).
      CurrentControlIsr::Instance().ClearOvercurrentTrip();
      kPowerStage.ClearFault();
    }
  }
}

void DriveNode::OnCurrentSetpoint(const cyphal::Transfer& transfer, const uint64_t now_us) noexcept {
  unimoc_drive_CurrentSetpoint_0_1 setpoint{};
  std::size_t size = transfer.payload.size();
  if (unimoc_drive_CurrentSetpoint_0_1_deserialize_(&setpoint, InputBytes(transfer.payload), &size) < 0) {
    return;
  }
  SetCurrentSetpoint(setpoint.i_d, setpoint.i_q, now_us);
}

void DriveNode::OnTorqueSetpoint(const cyphal::Transfer& transfer, const uint64_t now_us) noexcept {
  uavcan_si_unit_torque_Scalar_1_0 torque{};
  std::size_t size = transfer.payload.size();
  if (uavcan_si_unit_torque_Scalar_1_0_deserialize_(&torque, InputBytes(transfer.payload), &size) < 0 ||
      torque_constant_ <= 0.0F) {
    return;
  }
  SetCurrentSetpoint(0.0F, torque.newton_meter / torque_constant_, now_us);
}

void DriveNode::OnSpeedSetpoint(const cyphal::Transfer& transfer, const uint64_t now_us) noexcept {
  uavcan_si_unit_angular_velocity_Scalar_1_0 speed{};
  std::size_t size = transfer.payload.size();
  if (uavcan_si_unit_angular_velocity_Scalar_1_0_deserialize_(&speed, InputBytes(transfer.payload), &size) <
      0) {
    return;
  }
  SetSpeedSetpoint(speed.radian_per_second, now_us);
}

void DriveNode::OnPositionSetpoint(const cyphal::Transfer& transfer, const uint64_t now_us) noexcept {
  uavcan_si_unit_angle_Scalar_1_0 position{};
  std::size_t size = transfer.payload.size();
  if (uavcan_si_unit_angle_Scalar_1_0_deserialize_(&position, InputBytes(transfer.payload), &size) < 0) {
    return;
  }
  SetPositionSetpoint(position.radian, now_us);
}

void DriveNode::OnServoSetpoint(const cyphal::Transfer& transfer, const uint64_t now_us) noexcept {
  reg_udral_physics_dynamics_rotation_Planar_0_1 servo{};
  std::size_t size = transfer.payload.size();
  if (reg_udral_physics_dynamics_rotation_Planar_0_1_deserialize_(&servo,
                                                                  InputBytes(transfer.payload),
                                                                  &size) < 0) {
    return;
  }
  const float kPosition = servo.kinematics.angular_position.radian;
  const float kSpeed = servo.kinematics.angular_velocity.radian_per_second;
  const float kTorque = servo._torque.newton_meter;
  const std::optional<cyphal::ControlMode> kMode =
      cyphal::SelectControlModeFromUdralServoRotation(kPosition,
                                                      kSpeed,
                                                      servo.kinematics.angular_acceleration
                                                          .radian_per_second_per_second,
                                                      kTorque);
  if (!kMode.has_value()) {
    return;  // nothing the drive can control: the setpoint is ignored
  }
  switch (*kMode) {
    case cyphal::ControlMode::kPosition:
      SetPositionSetpoint(kPosition, now_us);
      break;
    case cyphal::ControlMode::kSpeed:
      SetSpeedSetpoint(kSpeed, now_us);
      break;
    case cyphal::ControlMode::kTorque:
      if (torque_constant_ > 0.0F) {
        SetCurrentSetpoint(0.0F, kTorque / torque_constant_, now_us);
      }
      break;
  }
}

void DriveNode::SetCurrentSetpoint(const float i_d, const float i_q, const uint64_t now_us) noexcept {
  if (!std::isfinite(i_d) || !std::isfinite(i_q)) {
    return;
  }
  // Limit the current vector to i_max, keeping its direction.
  const float kMagnitude = std::sqrt((i_d * i_d) + (i_q * i_q));
  const float kScale =
      (kMagnitude > current_limit_ && kMagnitude > 0.0F) ? current_limit_ / kMagnitude : 1.0F;
  setpoint_ = OuterSetpoint{};
  setpoint_.mode = cyphal::ControlMode::kTorque;
  setpoint_.current = {unit::Current{i_d * kScale}, unit::Current{i_q * kScale}};
  last_setpoint_us_ = now_us;
  setpoint_active_ = true;
}

void DriveNode::SetSpeedSetpoint(const float speed, const uint64_t now_us) noexcept {
  if (!std::isfinite(speed)) {
    return;
  }
  setpoint_ = OuterSetpoint{};
  setpoint_.mode = cyphal::ControlMode::kSpeed;
  setpoint_.speed = unit::AngularVelocity{speed};  // limited by the outer loop
  last_setpoint_us_ = now_us;
  setpoint_active_ = true;
}

void DriveNode::SetPositionSetpoint(const float position, const uint64_t now_us) noexcept {
  if (!std::isfinite(position)) {
    return;
  }
  setpoint_ = OuterSetpoint{};
  setpoint_.mode = cyphal::ControlMode::kPosition;
  setpoint_.position = unit::Angle{position};
  last_setpoint_us_ = now_us;
  setpoint_active_ = true;
}

void DriveNode::ClearSetpoint() noexcept {
  setpoint_active_ = false;
  setpoint_ = OuterSetpoint{};
}

void DriveNode::Engage() noexcept {
  if (engaged_ || emergency_stop_) {
    return;
  }
  if (startup_.IsActive()) {
    rearm_required_ = true;
    (void)node_->Log(cyphal::Severity::kWarning, "engage refused: the startup aid is running");
    return;
  }
  if (settings_pending_ || settings_registers_.WriteCount() != applied_writes_) {
    if (!ApplySettings()) {
      return;
    }
  }
  auto& isr = CurrentControlIsr::Instance();
  ClearSetpoint();
  OuterLoop::Instance().SetSetpoint(OuterSetpoint{});
  // ENGAGED is the explicit acknowledgement of a latched trip (as HwStartup::RequestRun()).
  isr.ClearOvercurrentTrip();
  kPowerStage.ClearFault();
  if (!kPowerStage.Enable()) {
    rearm_required_ = true;
    (void)node_->Log(cyphal::Severity::kWarning, "engage refused: external enable missing or fault latched");
    return;
  }
  engaged_ = true;
  (void)node_->Log(cyphal::Severity::kInfo, "engaged");
}

void DriveNode::Disengage(const std::string_view reason) noexcept {
  ClearSetpoint();
  OuterLoop::Instance().SetSetpoint(OuterSetpoint{});
  if (engaged_ || !startup_.IsActive()) {
    kPowerStage.Disable();  // the outputs of a startup run stay with the FSM
  }
  if (engaged_) {
    engaged_ = false;
    std::array<char, 64> text{};
    const std::string_view kPrefix{"disengaged: "};
    std::size_t length = 0U;
    for (const char kChar : kPrefix) {
      text[length++] = kChar;
    }
    for (std::size_t i = 0U; i < reason.size() && length < text.size(); ++i) {
      text[length++] = reason[i];
    }
    (void)node_->Log(cyphal::Severity::kNotice, std::string_view{text.data(), length});
  }
}

uint8_t DriveNode::OnCommand(const uint16_t command, std::span<const uint8_t> /*parameter*/) noexcept {
  auto operations = store_->GetOperations();
  switch (command) {
    case cyphal::cmd::kCommandRestart:
      startup_.RequestAbort();
      Disengage("restart");
      restart_requested_ = true;
      restart_at_us_ = now_us_ + kRestartDelayUs;
      return cyphal::cmd::kStatusSuccess;
    case cyphal::cmd::kCommandEmergencyStop:
      emergency_stop_ = true;
      startup_.RequestAbort();
      Disengage("emergency stop");
      return cyphal::cmd::kStatusSuccess;
    case cyphal::cmd::kCommandStorePersistentStates: {
      const settings::SettingsStatus kStatus = operations.SaveToFlash();
      if (kStatus == settings::SettingsStatus::kSuccess) {
        return cyphal::cmd::kStatusSuccess;
      }
      return (kStatus == settings::SettingsStatus::kPersistBlocked) ? cyphal::cmd::kStatusBadState
                                                                    : cyphal::cmd::kStatusFailure;
    }
    case cyphal::cmd::kCommandFactoryReset: {
      if (engaged_ || startup_.IsActive()) {
        return cyphal::cmd::kStatusBadState;
      }
      if (operations.ResetToFactoryDefaults() != settings::SettingsStatus::kSuccess) {
        return cyphal::cmd::kStatusFailure;
      }
      settings_pending_ = true;
      const settings::SettingsStatus kStatus = operations.SaveToFlash();
      if (kStatus == settings::SettingsStatus::kSuccess) {
        return cyphal::cmd::kStatusSuccess;
      }
      return (kStatus == settings::SettingsStatus::kPersistBlocked) ? cyphal::cmd::kStatusBadState
                                                                    : cyphal::cmd::kStatusFailure;
    }
    default:
      return cyphal::cmd::kStatusBadCommand;
  }
}

void DriveNode::OnNodeIdAllocated(const uint8_t node_id) noexcept {
  auto operations = store_->GetOperations();
  if (operations.SetNodeId(node_id) == settings::SettingsStatus::kSuccess &&
      operations.SaveToFlash() == settings::SettingsStatus::kSuccess) {
    (void)node_->Log(cyphal::Severity::kInfo, "node-ID allocated and saved");
  }
  applied_writes_ = settings_registers_.WriteCount();
}

void DriveNode::PublishTelemetry() noexcept {
  const auto& isr = CurrentControlIsr::Instance();
  const auto& state = isr.state;
  // A snapshot without locking the control cycle (see unimoc.drive.Telemetry).
  const unit::Angle kAngle = isr.mech_obs.theta;
  const system::SinCos<unit::Ratio> kSinCos(kAngle);
  const auto kCurrent = state.raw_phase_currents.ToStator().ToRotor(kSinCos);

  unimoc_drive_Telemetry_0_1 telemetry{};
  telemetry.control_cycles = state.control_cycles;
  telemetry.i_d = kCurrent.d.Value();
  telemetry.i_q = kCurrent.q.Value();
  telemetry.i_d_ref = state.i_ref.d.Value();
  telemetry.i_q_ref = state.i_ref.q.Value();
  telemetry.u_d = state.u_dq_last.d.Value();
  telemetry.u_q = state.u_dq_last.q.Value();
  telemetry.electrical_angle = observer::WrapAnglePositive(kAngle).Value();
  telemetry.electrical_speed = isr.mech_obs.omega.Value();
  telemetry.dc_link_voltage = state.raw_vdc.Value();
  telemetry.power_stage = static_cast<uint8_t>(kPowerStage.GetState());
  telemetry.overcurrent_tripped = state.overcurrent_tripped;
  telemetry.engaged = engaged_;

  std::array<std::byte, unimoc_drive_Telemetry_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_> buffer{};
  std::size_t size = buffer.size();
  if (unimoc_drive_Telemetry_0_1_serialize_(&telemetry, reinterpret_cast<uint8_t*>(buffer.data()), &size) >=
      0) {
    (void)node_->Publish(telemetry_, std::span<const std::byte>{buffer.data(), size});
  }
}

void DriveNode::UpdateStatus() noexcept {
  const PowerStageState kState = kPowerStage.GetState();
  cyphal::Health health = cyphal::Health::kNominal;
  if (emergency_stop_) {
    health = cyphal::Health::kCaution;
  } else if (kState == PowerStageState::kFault || CurrentControlIsr::Instance().state.overcurrent_tripped) {
    health = cyphal::Health::kAdvisory;
  }
  node_->SetHealth(health);
  // Vendor status: bits 0..2 power stage state, bit 3 engaged.
  node_->SetVendorStatus(static_cast<uint8_t>(static_cast<uint8_t>(kState) | (engaged_ ? 0x08U : 0x00U)));
}

}  // namespace unimoc::drive
