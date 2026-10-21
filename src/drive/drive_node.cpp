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
#include <reg/udral/service/common/Readiness_0_1.h>
#include <uavcan/si/unit/torque/Scalar_1_0.h>
#include <unimoc/drive/CurrentSetpoint_0_1.h>
#include <unimoc/drive/Telemetry_0_1.h>
#include <array>
#include <cmath>
#include <numbers>
#include "current_control_isr.hpp"
#include "current_control_runtime.hpp"
#include "power_stage_interface.hpp"
#include "sin_cos.hpp"

namespace unimoc::drive {

namespace {

using current_control::CurrentControlIsr;
using hardware::kPowerStage;
using hardware::PowerStageState;

struct InfoRegister {
  std::string_view name;
  std::string_view type;  ///< Empty: the unique-ID register.
};

constexpr std::array<InfoRegister, 5> kInfoRegisters{{
    {cyphal::reg::kSubReadinessType, "reg.udral.service.common.Readiness.0.1"},
    {cyphal::reg::kSubCurrentSetpointType, "unimoc.drive.CurrentSetpoint.0.1"},
    {cyphal::reg::kSubTorqueSetpointType, "uavcan.si.unit.torque.Scalar.1.0"},
    {cyphal::reg::kPubTelemetryType, "unimoc.drive.Telemetry.0.1"},
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
      info_registers_{unique_id} {}

bool DriveNode::Start(const uint64_t now_us) noexcept {
  now_us_ = now_us;
  const settings::SettingsSnapshot kSnapshot = store_->GetSnapshot();
  const settings::NvmSettings& live = kSnapshot.Get();
  readiness_port_ = live.sub_readiness_id;
  current_setpoint_port_ = live.sub_current_setpoint_id;
  torque_setpoint_port_ = live.sub_torque_setpoint_id;
  telemetry_ = cyphal::Publisher{live.pub_telemetry_id, cyphal::Priority::kHigh, 0U};
  // The runtime was started by main() with these settings.
  applied_writes_ = settings_registers_.WriteCount();
  LoadParameters(live);

  bool subscribed = node_->AddRegisterProvider(settings_registers_);
  subscribed = node_->AddRegisterProvider(info_registers_) && subscribed;
  subscribed =
      node_->Subscribe(readiness_port_, reg_udral_service_common_Readiness_0_1_EXTENT_BYTES_, *this) &&
      subscribed;
  subscribed =
      node_->Subscribe(current_setpoint_port_, unimoc_drive_CurrentSetpoint_0_1_EXTENT_BYTES_, *this) &&
      subscribed;
  subscribed =
      node_->Subscribe(torque_setpoint_port_, uavcan_si_unit_torque_Scalar_1_0_EXTENT_BYTES_, *this) &&
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

  auto& isr = CurrentControlIsr::Instance();
  if (engaged_) {
    if (now_us - last_readiness_us_ > command_timeout_us_) {
      Disengage("readiness timeout");
    } else if (kPowerStage.GetState() != PowerStageState::kEnabled || isr.state.overcurrent_tripped) {
      rearm_required_ = true;
      Disengage(isr.state.overcurrent_tripped ? "overcurrent trip" : "power stage fault");
    }
  }
  if (engaged_) {
    if (setpoint_active_ && now_us - last_setpoint_us_ > command_timeout_us_) {
      setpoint_active_ = false;
      setpoint_d_ = 0.0F;
      setpoint_q_ = 0.0F;
    }
    isr.SetCurrentRef({unit::Current{setpoint_d_}, unit::Current{setpoint_q_}});
  }

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

bool DriveNode::ApplySettings() noexcept {
  applied_writes_ = settings_registers_.WriteCount();
  settings_pending_ = false;
  const settings::SettingsSnapshot kSnapshot = store_->GetSnapshot();
  const settings::NvmSettings& live = kSnapshot.Get();
  LoadParameters(live);

  if (kPowerStage.GetState() == PowerStageState::kEnabled) {
    return false;  // not while the outputs are released; the next engage applies them
  }
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
    }
  }
}

void DriveNode::OnCurrentSetpoint(const cyphal::Transfer& transfer, const uint64_t now_us) noexcept {
  unimoc_drive_CurrentSetpoint_0_1 setpoint{};
  std::size_t size = transfer.payload.size();
  if (unimoc_drive_CurrentSetpoint_0_1_deserialize_(&setpoint, InputBytes(transfer.payload), &size) < 0) {
    return;
  }
  SetSetpoint(setpoint.i_d, setpoint.i_q, now_us);
}

void DriveNode::OnTorqueSetpoint(const cyphal::Transfer& transfer, const uint64_t now_us) noexcept {
  uavcan_si_unit_torque_Scalar_1_0 torque{};
  std::size_t size = transfer.payload.size();
  if (uavcan_si_unit_torque_Scalar_1_0_deserialize_(&torque, InputBytes(transfer.payload), &size) < 0 ||
      torque_constant_ <= 0.0F) {
    return;
  }
  SetSetpoint(0.0F, torque.newton_meter / torque_constant_, now_us);
}

void DriveNode::SetSetpoint(const float i_d, const float i_q, const uint64_t now_us) noexcept {
  if (!std::isfinite(i_d) || !std::isfinite(i_q)) {
    return;
  }
  // Limit the current vector to i_max, keeping its direction.
  const float kMagnitude = std::sqrt((i_d * i_d) + (i_q * i_q));
  const float kScale =
      (kMagnitude > current_limit_ && kMagnitude > 0.0F) ? current_limit_ / kMagnitude : 1.0F;
  setpoint_d_ = i_d * kScale;
  setpoint_q_ = i_q * kScale;
  last_setpoint_us_ = now_us;
  setpoint_active_ = true;
}

void DriveNode::Engage() noexcept {
  if (engaged_ || emergency_stop_) {
    return;
  }
  if (settings_pending_ || settings_registers_.WriteCount() != applied_writes_) {
    if (!ApplySettings()) {
      return;
    }
  }
  auto& isr = CurrentControlIsr::Instance();
  setpoint_active_ = false;
  setpoint_d_ = 0.0F;
  setpoint_q_ = 0.0F;
  isr.SetCurrentRef({unit::Current{}, unit::Current{}});
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
  kPowerStage.Disable();
  CurrentControlIsr::Instance().SetCurrentRef({unit::Current{}, unit::Current{}});
  setpoint_active_ = false;
  setpoint_d_ = 0.0F;
  setpoint_q_ = 0.0F;
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
      Disengage("restart");
      restart_requested_ = true;
      restart_at_us_ = now_us_ + kRestartDelayUs;
      return cyphal::cmd::kStatusSuccess;
    case cyphal::cmd::kCommandEmergencyStop:
      emergency_stop_ = true;
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
      if (engaged_) {
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
  constexpr float kTwoPi = 2.0F * std::numbers::pi_v<float>;
  float angle = std::fmod(kAngle.Value(), kTwoPi);
  angle = (angle < 0.0F) ? angle + kTwoPi : angle;

  unimoc_drive_Telemetry_0_1 telemetry{};
  telemetry.control_cycles = state.control_cycles;
  telemetry.i_d = kCurrent.d.Value();
  telemetry.i_q = kCurrent.q.Value();
  telemetry.i_d_ref = state.i_ref.d.Value();
  telemetry.i_q_ref = state.i_ref.q.Value();
  telemetry.u_d = state.u_dq_last.d.Value();
  telemetry.u_q = state.u_dq_last.q.Value();
  telemetry.electrical_angle = angle;
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
