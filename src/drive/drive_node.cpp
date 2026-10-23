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
#include <unimoc/cart/BatteryStatus_0_1.h>
#include <unimoc/cart/DriveStatus_0_1.h>
#include <unimoc/cart/PedalState_0_1.h>
#include <unimoc/cart/WheelCommand_0_1.h>
#include <unimoc/drive/CurrentSetpoint_0_1.h>
#include <unimoc/drive/Telemetry_0_1.h>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string_view>
#include <utility>
#include "angle_wrap.hpp"
#include "current_control_isr.hpp"
#include "current_control_runtime.hpp"
#include "cyphal/control_mode.hpp"
#include "power_stage_interface.hpp"
#include "settings_registers.hpp"
#include "settings_store.hpp"
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

constexpr std::array<InfoRegister, 19> kInfoRegisters{{
    {cyphal::reg::kSubReadinessType, "reg.udral.service.common.Readiness.0.1"},
    {cyphal::reg::kSubCurrentSetpointType, "unimoc.drive.CurrentSetpoint.0.1"},
    {cyphal::reg::kSubTorqueSetpointType, "uavcan.si.unit.torque.Scalar.1.0"},
    {cyphal::reg::kSubSpeedSetpointType, "uavcan.si.unit.angular_velocity.Scalar.1.0"},
    {cyphal::reg::kSubPositionSetpointType, "uavcan.si.unit.angle.Scalar.1.0"},
    {cyphal::reg::kSubServoSetpointType, "reg.udral.physics.dynamics.rotation.Planar.0.1"},
    {cyphal::reg::kPubTelemetryType, "unimoc.drive.Telemetry.0.1"},
    {cyphal::reg::kSubWheelCommandType, "unimoc.cart.WheelCommand.0.1"},
    {cyphal::reg::kSubPedalStateType, "unimoc.cart.PedalState.0.1"},
    {cyphal::reg::kSubBatteryStatusType, "unimoc.cart.BatteryStatus.0.1"},
    {cyphal::reg::kPubDriveStatusType, "unimoc.cart.DriveStatus.0.1"},
    {cyphal::reg::kSrvCymonVariablesType, "cymon.GetVariableList.0.1"},
    {cyphal::reg::kSrvCymonBufferInfoType, "cymon.GetBufferInfo.0.1"},
    {cyphal::reg::kSrvCymonSetupType, "cymon.SetupCapture.0.1"},
    {cyphal::reg::kSrvCymonArmType, "cymon.ArmTrigger.0.1"},
    {cyphal::reg::kSrvCymonReadType, "cymon.ReadSamples.0.1"},
    {cyphal::reg::kPubCymonTriggerType, "cymon.TriggerEvent.0.1"},
    {cyphal::reg::kSubCymonTriggerType, "cymon.TriggerEvent.0.1"},
    {cyphal::reg::kNodeUniqueId, ""},
}};

/**
 * @brief Copies the registers that may change while engaged (live tuning).
 *
 * The gains of the current, speed and position loops, the telemetry rate
 * and the command timeout: the running loops take them without a restart.
 */
void CopyTunable(const settings::NvmSettings& source, settings::NvmSettings& target) noexcept {
  target.current_kp_d = source.current_kp_d;
  target.current_ki_d = source.current_ki_d;
  target.current_kp_q = source.current_kp_q;
  target.current_ki_q = source.current_ki_q;
  target.speed_kp = source.speed_kp;
  target.speed_ki = source.speed_ki;
  target.speed_accel_limit = source.speed_accel_limit;
  target.pos_kp_pos = source.pos_kp_pos;
  target.pos_speed_limit = source.pos_speed_limit;
  target.pos_accel_limit = source.pos_accel_limit;
  target.pos_position_tolerance = source.pos_position_tolerance;
  target.pos_speed_tolerance = source.pos_speed_tolerance;
  target.telemetry_rate = source.telemetry_rate;
  target.command_timeout = source.command_timeout;
  target.cart_status_rate = source.cart_status_rate;
  target.cart_torque_ramp = source.cart_torque_ramp;
  target.cart_limp_home_torque = source.cart_limp_home_torque;
  target.cart_standstill_speed = source.cart_standstill_speed;
}

/// The first register in which @p live differs from @p applied outside the tunable ones; empty: none.
[[nodiscard]] std::string_view FirstUntunableChange(const settings::NvmSettings& applied,
                                                    const settings::NvmSettings& live) noexcept {
  settings::NvmSettings untuned = live;
  CopyTunable(applied, untuned);
  std::string_view changed{};
  settings::ForEachRegister(
      [&changed](const std::string_view name, const auto& first, const auto& second) {
        if (changed.empty() && !settings::settings_store_internal::SameEncoding(first, second)) {
          changed = name;
        }
      },
      applied,
      std::as_const(untuned));
  return changed;
}

/// @p prefix followed by @p detail in @p buffer, cut to its size.
template <std::size_t kSize>
[[nodiscard]] std::string_view Joined(const std::string_view prefix,
                                      const std::string_view detail,
                                      std::array<char, kSize>& buffer) noexcept {
  std::size_t length = 0U;
  for (const char kChar : prefix) {
    if (length < kSize) {
      buffer[length++] = kChar;
    }
  }
  for (const char kChar : detail) {
    if (length < kSize) {
      buffer[length++] = kChar;
    }
  }
  return std::string_view{buffer.data(), length};
}

[[nodiscard]] const uint8_t* InputBytes(std::span<const std::byte> payload) noexcept {
  return reinterpret_cast<const uint8_t*>(payload.data());
}

[[nodiscard]] uint64_t ToMicroseconds(const unit::Time time) noexcept {
  return static_cast<uint64_t>(time.Value() * 1.0e6F);
}

/// Publication period of a rate [us]; 0: not published.
[[nodiscard]] uint64_t PeriodMicroseconds(const unit::Frequency rate) noexcept {
  return (rate > unit::Frequency{}) ? static_cast<uint64_t>(1.0e6F / rate.Value()) : 0U;
}

/// Advances a publication deadline; true when it was due.
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

/// A temperature reading (degrees Celsius) in kelvin for the bus; NaN without one.
[[nodiscard]] float Kelvin(const std::optional<unit::Temperature>& temperature) noexcept {
  constexpr float kZeroCelsius = 273.15F;
  return temperature.has_value() ? temperature->Value() + kZeroCelsius
                                 : std::numeric_limits<float>::quiet_NaN();
}

/// Measured current in the rotor frame of the observer angle (a snapshot without locking the control cycle).
[[nodiscard]] system::Rotor<unit::Current> MeasuredCurrent(const CurrentControlIsr& isr) noexcept {
  const system::SinCos<unit::Ratio> kSinCos(isr.mech_obs.theta);
  return isr.state.raw_phase_currents.ToStator().ToRotor(kSinCos);
}

}  // namespace

// ----- DriveInfoRegisters ----------------------------------------------------

// The info registers, then `unimoc.settings.pending` and `unimoc.can.bus_off_count`.
std::size_t DriveInfoRegisters::Count() const noexcept { return kInfoRegisters.size() + 2U; }

std::string_view DriveInfoRegisters::Name(const std::size_t index) const noexcept {
  if (index == kInfoRegisters.size()) {
    return cyphal::reg::kSettingsPending;
  }
  if (index == kInfoRegisters.size() + 1U) {
    return cyphal::reg::kCanBusOffCount;
  }
  return index < kInfoRegisters.size() ? kInfoRegisters[index].name : std::string_view{};
}

cyphal::RegisterFlags DriveInfoRegisters::Read(const std::size_t index,
                                               cyphal::RegisterValue& value) const noexcept {
  if (index == kInfoRegisters.size()) {
    cyphal::SetBit(value, *settings_pending_);
  } else if (index == kInfoRegisters.size() + 1U) {
    cyphal::SetNatural32(value, *bus_off_count_);
  } else if (index >= kInfoRegisters.size()) {
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
      info_registers_{unique_id, settings_waiting_, bus_off_count_},
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
  wheel_command_port_ = live.sub_wheel_command_id;
  pedal_state_port_ = live.sub_pedal_state_id;
  battery_status_port_ = live.sub_battery_status_id;
  telemetry_ = cyphal::Publisher{live.pub_telemetry_id, cyphal::Priority::kHigh, 0U};
  // After the wheel command (VDC) and with the pedal state (system plan section 6.2).
  drive_status_ = cyphal::Publisher{live.pub_drive_status_id, cyphal::Priority::kFast, 0U};
  // The runtime was started by main() with these settings.
  applied_writes_ = settings_registers_.WriteCount();
  checked_writes_ = applied_writes_;
  applied_settings_ = live;
  LoadParameters(live);

  bool subscribed = node_->AddRegisterProvider(settings_registers_);
  subscribed = node_->AddRegisterProvider(info_registers_) && subscribed;
  subscribed = node_->AddRegisterProvider(startup_registers_) && subscribed;
  subscribed = node_->AddRegisterProvider(timing_registers_) && subscribed;
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
  subscribed =
      node_->Subscribe(wheel_command_port_, unimoc_cart_WheelCommand_0_1_EXTENT_BYTES_, *this) && subscribed;
  subscribed =
      node_->Subscribe(pedal_state_port_, unimoc_cart_PedalState_0_1_EXTENT_BYTES_, *this) && subscribed;
  subscribed = node_->Subscribe(battery_status_port_, unimoc_cart_BatteryStatus_0_1_EXTENT_BYTES_, *this) &&
               subscribed;
  subscribed = node_->AddPublisher(telemetry_) && subscribed;
  subscribed = node_->AddPublisher(drive_status_) && subscribed;
  node_->SetCommandHandler(*this);
  node_->SetNodeIdListener(*this);

  // 0 is "not assigned" in the settings: request a node-ID by plug-and-play.
  const uint8_t kNodeId = (live.node_id == 0U) ? cyphal::kNodeIdUnset : live.node_id;
  subscribed = node_->Start(kNodeId, now_us) && subscribed;
  node_->SetMode(cyphal::Mode::kOperational);
  next_telemetry_us_ = now_us;
  next_drive_status_us_ = now_us;
  UpdateStatus();
  return subscribed;
}

void DriveNode::Spin(const uint64_t now_us) noexcept {
  now_us_ = now_us;
  node_->Spin(now_us);  // dispatches the received messages and commands
  WatchBus();
  SpinLocalControl(now_us);

  if (engage_requested_) {
    engage_requested_ = false;
    Engage();
  }
  SpinStartup();
  SpinEngagement(now_us);
  // The outer loop commands zero current while the drive is disengaged.
  OuterSetpoint setpoint = setpoint_;
  setpoint.active = engaged_;
  OuterLoop::Instance().SetSetpoint(setpoint);

  if (engaged_ && settings_registers_.WriteCount() != checked_writes_) {
    TuneLive();
  }
  if (!engaged_ && settings_registers_.WriteCount() != applied_writes_) {
    settings_pending_ = true;
  }
  if (!engaged_ && settings_pending_) {
    (void)ApplySettings();
  }

  if (Due(telemetry_period_us_, now_us, next_telemetry_us_)) {
    PublishTelemetry();
  }
  if (Due(drive_status_period_us_, now_us, next_drive_status_us_)) {
    PublishDriveStatus();
  }

  if (restart_requested_ && now_us >= restart_at_us_ && hooks_.restart != nullptr) {
    restart_requested_ = false;
    hooks_.restart();
  }
  settings_waiting_ = settings_pending_ || settings_registers_.WriteCount() != applied_writes_;
  UpdateStatus();
}

void DriveNode::SpinLocalControl(const uint64_t now_us) noexcept {
  if (ebike_ == nullptr) {
    return;
  }
  const auto& isr = CurrentControlIsr::Instance();
  const current_control::OuterStatus kOuter = OuterLoop::Instance().Status();
  const unit::Torque kTorque = torque_constant_ * MeasuredCurrent(isr).q;
  ebike::EbikeVehicle vehicle;
  vehicle.motor_speed = kOuter.speed;
  vehicle.torque_min = kOuter.torque_min;
  vehicle.torque_max = kOuter.torque_max;
  vehicle.dc_link = isr.state.raw_vdc;
  // Battery current from the mechanical power, as in the drive status.
  vehicle.battery_current =
      (isr.state.raw_vdc > unit::Voltage{}) ? (kTorque * kOuter.speed) / isr.state.raw_vdc : unit::Current{};
  vehicle.motor_temperature = motor_->GetTemperatureMeasurements().motor.value_or(unit::Temperature{});
  vehicle.engaged = engaged_;
  vehicle.faulted = rearm_required_;

  const ebike::EbikeOutput kOutput = ebike_->Update(now_us, vehicle);
  ebike_engage_ = kOutput.engage;
  ebike_torque_ = kOutput.torque;
  if (kOutput.engage) {
    // The e-bike's readiness: the readiness timeout stays fresh while the rider needs the drive.
    last_readiness_us_ = now_us;
    if (!engaged_ && !rearm_required_) {
      engage_requested_ = true;
    }
  } else {
    rearm_required_ = false;  // as a readiness other than ENGAGED: the next request may engage again
    if (engaged_) {
      Disengage("e-bike");
    }
  }
}

void DriveNode::SpinEngagement(const uint64_t now_us) noexcept {
  // The cart torque replaces the setpoints once a wheel command arrived; in
  // limp-home and while it ramps down, the readiness timeout waits.
  const CartOutput kCart = cart_.Update(now_us, OuterLoop::Instance().Status().speed, engaged_);
  cart_mode_ = kCart.mode;
  const auto& isr = CurrentControlIsr::Instance();
  if (engaged_) {
    if (now_us - last_readiness_us_ > command_timeout_us_ && !kCart.holds_engagement) {
      Disengage("readiness timeout");
    } else if (kPowerStage.GetState() != PowerStageState::kEnabled || isr.Tripped()) {
      rearm_required_ = true;
      const char* const kTrip = isr.TripName();
      Disengage(kTrip != nullptr ? kTrip : "power stage fault");
    }
  }
  if (engaged_ && kCart.mode != CartMode::kOff) {
    const unit::Current kCurrent =
        (torque_constant_ > unit::MagneticFlux{}) ? kCart.torque / torque_constant_ : unit::Current{};
    SetCurrentSetpoint(unit::Current{}, kCurrent, now_us);
  }
  if (engaged_ && ebike_ != nullptr && ebike_engage_) {
    const unit::Current kCurrent =
        (torque_constant_ > unit::MagneticFlux{}) ? ebike_torque_ / torque_constant_ : unit::Current{};
    SetCurrentSetpoint(unit::Current{}, kCurrent, now_us);
  }
  if (engaged_ && setpoint_active_ && now_us - last_setpoint_us_ > command_timeout_us_) {
    ClearSetpoint();
  }
  OuterLoop::Instance().SetBatteryLimits(cart_.Battery(now_us));
}

void DriveNode::LoadParameters(const settings::NvmSettings& settings) noexcept {
  command_timeout_us_ = ToMicroseconds(settings.command_timeout);
  telemetry_period_us_ = PeriodMicroseconds(settings.telemetry_rate);
  drive_status_period_us_ = PeriodMicroseconds(settings.cart_status_rate);
  cart_wheel_ = settings.cart_wheel;
  cart_.Configure(CartParameters::From(settings));
  current_limit_ = settings.motor_i_max;
  // PMSM torque constant: T = 3/2 p psi_pm i_q.
  torque_constant_ = settings.flux_pm * (1.5F * static_cast<float>(settings.pole_pairs));
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
  checked_writes_ = applied_writes_;
  settings_pending_ = false;
  const settings::SettingsSnapshot kSnapshot = store_->GetSnapshot();
  const settings::NvmSettings& live = kSnapshot.Get();
  applied_settings_ = live;
  LoadParameters(live);

  if (!current_control::StartRuntime(live, *motor_, store_->GetProfile().capabilities.max_phase_current)) {
    (void)node_->Log(cyphal::Severity::kError, "settings rejected by the hardware (PWM frequency)");
    return false;
  }
  if (hooks_.settings_applied != nullptr) {
    hooks_.settings_applied(live);
  }
  if (waiting_announced_) {
    waiting_announced_ = false;
    (void)node_->Log(cyphal::Severity::kInfo, "waiting settings applied");
  }
  return true;
}

void DriveNode::TuneLive() noexcept {
  checked_writes_ = settings_registers_.WriteCount();
  const settings::SettingsSnapshot kSnapshot = store_->GetSnapshot();
  const settings::NvmSettings& live = kSnapshot.Get();
  const std::string_view kUntunable = FirstUntunableChange(applied_settings_, live);
  if (!kUntunable.empty()) {
    settings_pending_ = true;  // a restart with all of them after the disengage
    std::array<char, 112> text{};
    (void)node_->Log(cyphal::Severity::kWarning,
                     Joined("settings wait for the disengage: ", kUntunable, text));
    waiting_announced_ = true;
    return;
  }
  CurrentControlIsr::Instance().SetCurrentGains(current_control::CurrentGains{.kp_d = live.current_kp_d,
                                                                              .ki_d = live.current_ki_d,
                                                                              .kp_q = live.current_kp_q,
                                                                              .ki_q = live.current_ki_q});
  OuterLoop::Instance().SetGains(OuterLoop::GainsFrom(live));
  LoadParameters(live);
  applied_settings_ = live;
  applied_writes_ = checked_writes_;
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
  } else if (transfer.port_id == wheel_command_port_) {
    OnWheelCommand(transfer, now_us);
  } else if (transfer.port_id == pedal_state_port_) {
    OnPedalState(transfer, now_us);
  } else if (transfer.port_id == battery_status_port_) {
    OnBatteryStatus(transfer, now_us);
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
    if (ebike_ != nullptr) {
      return;  // the e-bike control engages the drive
    }
    if (!engaged_ && !rearm_required_) {
      engage_requested_ = true;  // engaged in Spin(), outside the dispatch
    }
  } else {
    rearm_required_ = false;
    if (ebike_ != nullptr) {
      ebike_->Inhibit();  // the bus takes the drive off the e-bike until the pedals stood still
    }
    if (engaged_) {
      Disengage("readiness");
    } else if (readiness.value == reg_udral_service_common_Readiness_0_1_STANDBY) {
      // STANDBY acknowledges a latched trip or fault without releasing the
      // outputs, so a flash save is possible again (the save gate needs kDisabled).
      CurrentControlIsr::Instance().ClearTrips();
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
  SetCurrentSetpoint(unit::Current{setpoint.i_d}, unit::Current{setpoint.i_q}, now_us);
}

void DriveNode::OnTorqueSetpoint(const cyphal::Transfer& transfer, const uint64_t now_us) noexcept {
  uavcan_si_unit_torque_Scalar_1_0 torque{};
  std::size_t size = transfer.payload.size();
  if (uavcan_si_unit_torque_Scalar_1_0_deserialize_(&torque, InputBytes(transfer.payload), &size) < 0 ||
      torque_constant_ <= unit::MagneticFlux{}) {
    return;
  }
  SetCurrentSetpoint(unit::Current{}, unit::Torque{torque.newton_meter} / torque_constant_, now_us);
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
      if (torque_constant_ > unit::MagneticFlux{}) {
        SetCurrentSetpoint(unit::Current{}, unit::Torque{kTorque} / torque_constant_, now_us);
      }
      break;
  }
}

void DriveNode::OnWheelCommand(const cyphal::Transfer& transfer, const uint64_t now_us) noexcept {
  unimoc_cart_WheelCommand_0_1 command{};
  std::size_t size = transfer.payload.size();
  if (unimoc_cart_WheelCommand_0_1_deserialize_(&command, InputBytes(transfer.payload), &size) < 0 ||
      cart_wheel_ >= std::size(command.wheel_torque)) {
    return;
  }
  cart_.OnWheelCommand(unit::Torque{command.wheel_torque[cart_wheel_]}, now_us);
}

void DriveNode::OnPedalState(const cyphal::Transfer& transfer, const uint64_t now_us) noexcept {
  unimoc_cart_PedalState_0_1 state{};
  std::size_t size = transfer.payload.size();
  if (unimoc_cart_PedalState_0_1_deserialize_(&state, InputBytes(transfer.payload), &size) < 0) {
    return;
  }
  cart_.OnPedalState(CartPedals{.drive = {unit::Ratio{state.drive_pedal[0]},
                                          unit::Ratio{state.drive_pedal[1]}},
                                .brake = {unit::Ratio{state.brake_pedal[0]},
                                          unit::Ratio{state.brake_pedal[1]}},
                                .drive_plausible = state.drive_pedal_plausible,
                                .brake_plausible = state.brake_pedal_plausible},
                     now_us);
}

void DriveNode::OnBatteryStatus(const cyphal::Transfer& transfer, const uint64_t now_us) noexcept {
  unimoc_cart_BatteryStatus_0_1 status{};
  std::size_t size = transfer.payload.size();
  if (unimoc_cart_BatteryStatus_0_1_deserialize_(&status, InputBytes(transfer.payload), &size) < 0) {
    return;
  }
  cart_.OnBatteryStatus(CartBattery{.discharge_limit = unit::Current{status.discharge_current_limit},
                                    .charge_limit = unit::Current{status.charge_current_limit},
                                    .contactor = static_cast<Contactor>(status.contactor)},
                        now_us);
}

void DriveNode::WatchBus() noexcept {
  const cyphal::CanBusStatus kBus = node_->BusStatus();
  const bool kWentOff = kBus.state == cyphal::CanBusState::kBusOff || kBus.bus_off_count != bus_off_count_;
  bus_off_count_ = kBus.bus_off_count;
  if (kWentOff && !bus_lost_) {
    bus_lost_ = true;
    cart_.OnBusLost();  // nothing that arrived before counts any more
  }
  if (bus_lost_ && kBus.state != cyphal::CanBusState::kBusOff) {
    bus_lost_ = false;
    (void)node_->Log(cyphal::Severity::kWarning, "CAN bus-off, back on the bus");
  }
}

void DriveNode::SetCurrentSetpoint(const unit::Current i_d,
                                   const unit::Current i_q,
                                   const uint64_t now_us) noexcept {
  if (!unit::IsFinite(i_d) || !unit::IsFinite(i_q)) {
    return;
  }
  // Limit the current vector to i_max, keeping its direction.
  const unit::Current kMagnitude = unit::Hypot(i_d, i_q);
  const unit::Ratio kScale = (kMagnitude > current_limit_ && kMagnitude > unit::Current{})
                                 ? current_limit_ / kMagnitude
                                 : unit::Ratio{1.0F};
  setpoint_ = OuterSetpoint{};
  setpoint_.mode = cyphal::ControlMode::kTorque;
  setpoint_.current = {i_d * kScale, i_q * kScale};
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
  cart_.Reset();
  OuterLoop::Instance().SetSetpoint(OuterSetpoint{});
  // ENGAGED is the explicit acknowledgement of a latched trip (as HwStartup::RequestRun()).
  isr.ClearTrips();
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
  cart_.Reset();
  OuterLoop::Instance().SetSetpoint(OuterSetpoint{});
  if (engaged_ || !startup_.IsActive()) {
    kPowerStage.Disable();  // the outputs of a startup run stay with the FSM
  }
  if (engaged_) {
    engaged_ = false;
    std::array<char, 64> text{};
    (void)node_->Log(cyphal::Severity::kNotice, Joined("disengaged: ", reason, text));
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
  const auto kCurrent = MeasuredCurrent(isr);

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

void DriveNode::PublishDriveStatus() noexcept {
  const auto& isr = CurrentControlIsr::Instance();
  const auto& state = isr.state;
  const current_control::OuterStatus kOuter = OuterLoop::Instance().Status();
  const unit::Torque kTorque = torque_constant_ * MeasuredCurrent(isr).q;

  // The torque window in the cart frame, then relative to the motion
  // (standstill counts as forward).
  const unit::Torque kCartMin = unit::Min(cart_.ToCart(kOuter.torque_min), cart_.ToCart(kOuter.torque_max));
  const unit::Torque kCartMax = unit::Max(cart_.ToCart(kOuter.torque_min), cart_.ToCart(kOuter.torque_max));
  const unit::AngularVelocity kSpeed = cart_.ToCart(kOuter.speed);
  const bool kForward = !(kSpeed < unit::AngularVelocity{});
  const unit::Torque kDriveLimit = unit::Max(kForward ? kCartMax : -kCartMin, unit::Torque{});
  const unit::Torque kBrakeLimit = unit::Max(kForward ? -kCartMin : kCartMax, unit::Torque{});
  const hardware::TemperatureMeasurements kTemperatures = motor_->GetTemperatureMeasurements();

  unimoc_cart_DriveStatus_0_1 status{};
  status.wheel_speed = kSpeed.Value();
  status.estimated_torque = cart_.ToCart(kTorque).Value();
  status.drive_torque_limit = kDriveLimit.Value();
  status.brake_torque_limit = kBrakeLimit.Value();
  status.dc_link_voltage = state.raw_vdc.Value();
  // Battery current from the mechanical power (the estimate of control::DcLinkLimit).
  status.dc_link_current = (state.raw_vdc > unit::Voltage{})
                               ? ((kTorque * kOuter.speed) / state.raw_vdc).Value()
                               : std::numeric_limits<float>::quiet_NaN();
  status.power_stage_temperature = Kelvin(kTemperatures.bridge);
  status.motor_temperature = Kelvin(kTemperatures.motor);
  status.wheel = cart_wheel_;
  status.power_stage = static_cast<uint8_t>(kPowerStage.GetState());
  status.engaged = engaged_;
  status.limp_home = cart_mode_ == CartMode::kLimpHome;
  status.regen_derated = kOuter.regen_derated;
  status.overcurrent_tripped = state.overcurrent_tripped;
  status.overvoltage_tripped = state.overvoltage_tripped;

  std::array<std::byte, unimoc_cart_DriveStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_> buffer{};
  std::size_t size = buffer.size();
  if (unimoc_cart_DriveStatus_0_1_serialize_(&status, reinterpret_cast<uint8_t*>(buffer.data()), &size) >=
      0) {
    (void)node_->Publish(drive_status_, std::span<const std::byte>{buffer.data(), size});
  }
}

void DriveNode::UpdateStatus() noexcept {
  const PowerStageState kState = kPowerStage.GetState();
  cyphal::Health health = cyphal::Health::kNominal;
  if (emergency_stop_) {
    health = cyphal::Health::kCaution;
  } else if (kState == PowerStageState::kFault || CurrentControlIsr::Instance().Tripped()) {
    health = cyphal::Health::kAdvisory;
  }
  node_->SetHealth(health);
  // Vendor status: bits 0..2 power stage state, bit 3 engaged.
  node_->SetVendorStatus(static_cast<uint8_t>(static_cast<uint8_t>(kState) | (engaged_ ? 0x08U : 0x00U)));
}

}  // namespace unimoc::drive
