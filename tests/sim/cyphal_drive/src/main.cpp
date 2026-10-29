/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file main.cpp
 *    @brief The Cyphal application of the drive against the simulated plant:
 *           arming, current, torque, speed, position and servo setpoints,
 *           limits, timeouts, telemetry, registers, the startup aid and
 *           node commands, driven by a second libcanard node.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <reg/udral/physics/dynamics/rotation/Planar_0_1.h>
#include <reg/udral/service/common/Readiness_0_1.h>
#include <uavcan/_register/Access_1_0.h>
#include <uavcan/node/ExecuteCommand_1_3.h>
#include <uavcan/si/unit/angle/Scalar_1_0.h>
#include <uavcan/si/unit/angular_velocity/Scalar_1_0.h>
#include <uavcan/si/unit/torque/Scalar_1_0.h>
#include <unimoc/cart/BatteryStatus_0_1.h>
#include <unimoc/cart/DriveStatus_0_1.h>
#include <unimoc/cart/PedalState_0_1.h>
#include <unimoc/cart/WheelCommand_0_1.h>
#include <unimoc/drive/CommissioningStatus_0_1.h>
#include <unimoc/drive/CurrentSetpoint_0_1.h>
#include <unimoc/drive/Telemetry_0_1.h>
#include <zephyr/ztest.h>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <new>
#include <numbers>
#include <string>
#include <string_view>
#include "angle_wrap.hpp"
#include "can_test_bus.hpp"
#include "current_control_isr.hpp"
#include "current_control_runtime.hpp"
#include "cyphal/canard_transport.hpp"
#include "cyphal/node.hpp"
#include "drive_node.hpp"
#include "ebike_control.hpp"
#include "ebike_interface.hpp"
#include "hardware_interface.hpp"
#include "hot_path_interface.hpp"
#include "outer_loop.hpp"
#include "power_stage_interface.hpp"
#include "settings_registers.hpp"
#include "settings_save_gate.hpp"
#include "settings_store.hpp"
#include "sim_control.hpp"
#include "sim_ebike.hpp"
#include "sim_test_motor.hpp"
#include "sin_cos.hpp"
#include "slow_update.hpp"
#include "unimoc_test.hpp"
#include "zephyr_settings_storage.hpp"

// The platform calls these directly (hot_path_interface.hpp). The control
// thread runs synchronously here, every OuterLoop::kCyclesPerStep cycles.
void unimoc::hardware::ControlCycleHandler() noexcept {
  unimoc::current_control::CurrentControlIsr::Instance().OnControlCycle();
}
void unimoc::hardware::ObserverHandler() noexcept {
  unimoc::current_control::SlowUpdate::Instance().RunOnce();
  if (unimoc::current_control::OuterLoop::Instance().OnObserverCycle()) {
    unimoc::current_control::OuterLoop::Instance().Step();
  }
}

namespace {

using namespace unimoc::unit;
using unimoc::current_control::CurrentControlIsr;
using unimoc::current_control::OuterLoop;
using unimoc::cyphal::CanardArena;
using unimoc::cyphal::CanardTransport;
using unimoc::cyphal::CanMedia;
using unimoc::cyphal::CyphalNode;
using unimoc::cyphal::Priority;
using unimoc::cyphal::ReceiveResult;
using unimoc::cyphal::Transfer;
using unimoc::cyphal::TransferKind;
using unimoc::hardware::kPowerStage;
using unimoc::hardware::PowerStageState;
using unimoc::hardware::ZephyrSettingsStorage;
using unimoc::settings::NvmSettings;
using unimoc::settings::SettingsStatus;
using unimoc::settings::SettingsStore;
using unimoc::test::CanTestBus;
using unimoc::test::CanTestMedia;
namespace cmd = unimoc::cyphal::cmd;
namespace port = unimoc::cyphal::port;
namespace reg = unimoc::cyphal::reg;
namespace sim = unimoc::hardware::sim;

constexpr uint8_t kDriveId = 10U;  // the sim profile's factory node-ID
// The factory readiness subject follows the node-ID.
constexpr uint16_t kReadinessPort =
    unimoc::cyphal::EffectiveReadinessId(unimoc::cyphal::kReadinessFollowsNodeId, kDriveId);
constexpr uint8_t kClientId = 100U;
constexpr unsigned kCyclesPerMs = 16U;  // 32 kHz PWM, 16 kHz control cycle

const uint8_t* In(std::span<const std::byte> buffer) {
  return reinterpret_cast<const uint8_t*>(buffer.data());
}
uint8_t* Out(std::span<std::byte> buffer) { return reinterpret_cast<uint8_t*>(buffer.data()); }

/// The platform profile with the test motor this suite was written for.
const unimoc::settings::SettingsProfile& TestProfile() {
  return unimoc::test::SimTestMotorProfile(unimoc::hardware::kSettings.GetSettingsProfile());
}

SettingsStore MakeStore() {
  return SettingsStore{TestProfile(),
                       ZephyrSettingsStorage::Adapter(),
                       unimoc::settings::PowerStageSaveGate(kPowerStage),
                       unimoc::hardware::kSettings.GetSettingsLock()};
}

void EraseAllRegisters() {
  const auto kStorage = ZephyrSettingsStorage::Adapter();
  const NvmSettings kSettings{};
  unimoc::settings::ForEachRegister([&](std::string_view name,
                                        const auto& /*field*/) { (void)kStorage.Erase(name); },
                                    kSettings);
}

int g_restarts = 0;
void CountRestart() noexcept { ++g_restarts; }
int g_settings_applied = 0;  // controller restarts with new settings
void CountSettingsApplied(const unimoc::settings::NvmSettings& /*unused*/) noexcept { ++g_settings_applied; }

/// The drive (store, node, DriveNode) and a client on one CAN FD bus.
// Members in the order of their construction dependencies, not by size.
// NOLINTNEXTLINE(clang-analyzer-optin.performance.Padding): see above.
struct Rig {
  Rig()
      : store{MakeStore()},
        drive_media{bus, unimoc::cyphal::kCanFdMtu},
        client_media{bus, unimoc::cyphal::kCanFdMtu},
        drive_interfaces{&drive_media},
        client_interfaces{&client_media},
        drive_transport{drive_arena.bytes, drive_interfaces},
        client{client_arena.bytes, client_interfaces},
        node{drive_transport},
        drive{node,
              store,
              unimoc::hardware::kMotors[0],
              unimoc::drive::DriveHooks{CountRestart, CountSettingsApplied},
              unique_id} {
    zassert_equal(store.Load(), SettingsStatus::kFactoryDefaults);
    const auto kSnapshot = store.GetSnapshot();
    zassert_true(unimoc::current_control::StartRuntime(kSnapshot.Get(),
                                                       unimoc::hardware::kMotors[0],
                                                       store.GetProfile().capabilities.max_phase_current));
    sim::SetFreeRunning(false);
    client.SetLocalNodeId(kClientId);
    client.Process(now);
    zassert_true(client.Subscribe(TransferKind::kMessage, unimoc::cyphal::kPubTelemetry, 64U));
    zassert_true(client.Subscribe(TransferKind::kMessage, unimoc::cyphal::kPubDriveStatus, 64U));
    zassert_true(client.Subscribe(TransferKind::kMessage, unimoc::cyphal::kPubCommissioning, 64U));
    zassert_true(client.Subscribe(TransferKind::kServiceResponse, port::kRegisterAccess, 267U));
    zassert_true(client.Subscribe(TransferKind::kServiceResponse, port::kExecuteCommand, 48U));
    zassert_true(drive.Start(now));
  }

  /// Advances the drive, the plant and the client by @p milliseconds.
  void Run(const unsigned milliseconds) {
    for (unsigned i = 0U; i < milliseconds; ++i) {
      now += 1000U;
      sim::SetTime(now);  // the simulated rider and display follow the virtual time
      drive.Spin(now);
      sim::AdvanceCycles(kCyclesPerMs);
      client.Process(now);
    }
  }

  void Publish(const uint16_t subject, std::span<const std::byte> payload) {
    const Transfer kTransfer{.port_id = subject,
                             .source_node_id = kClientId,
                             .destination_node_id = unimoc::cyphal::kNodeIdUnset,
                             .transfer_id = next_transfer_id++,
                             .kind = TransferKind::kMessage,
                             .priority = Priority::kHigh,
                             .payload = payload};
    zassert_true(client.Send(kTransfer));
  }

  void Readiness(const uint8_t value) {
    reg_udral_service_common_Readiness_0_1 readiness{value};
    std::size_t size = tx.size();
    zassert_equal(reg_udral_service_common_Readiness_0_1_serialize_(&readiness, Out(tx), &size), 0);
    Publish(kReadinessPort, std::span<const std::byte>{tx.data(), size});
  }

  void Current(const float i_d, const float i_q) {
    unimoc_drive_CurrentSetpoint_0_1 setpoint{i_d, i_q};
    std::size_t size = tx.size();
    zassert_equal(unimoc_drive_CurrentSetpoint_0_1_serialize_(&setpoint, Out(tx), &size), 0);
    Publish(unimoc::cyphal::kSubCurrentSetpoint, std::span<const std::byte>{tx.data(), size});
  }

  void Speed(const float speed) {
    uavcan_si_unit_angular_velocity_Scalar_1_0 setpoint{speed};
    std::size_t size = tx.size();
    zassert_equal(uavcan_si_unit_angular_velocity_Scalar_1_0_serialize_(&setpoint, Out(tx), &size), 0);
    Publish(unimoc::cyphal::kSubSpeedSetpoint, std::span<const std::byte>{tx.data(), size});
  }

  void Position(const float position) {
    uavcan_si_unit_angle_Scalar_1_0 setpoint{position};
    std::size_t size = tx.size();
    zassert_equal(uavcan_si_unit_angle_Scalar_1_0_serialize_(&setpoint, Out(tx), &size), 0);
    Publish(unimoc::cyphal::kSubPositionSetpoint, std::span<const std::byte>{tx.data(), size});
  }

  /// UDRAL servo setpoint; NaN marks a field as not commanded.
  void Servo(const float position, const float speed, const float torque) {
    reg_udral_physics_dynamics_rotation_Planar_0_1 setpoint{};
    setpoint.kinematics.angular_position.radian = position;
    setpoint.kinematics.angular_velocity.radian_per_second = speed;
    setpoint.kinematics.angular_acceleration.radian_per_second_per_second =
        std::numeric_limits<float>::quiet_NaN();
    setpoint._torque.newton_meter = torque;
    std::size_t size = tx.size();
    zassert_equal(reg_udral_physics_dynamics_rotation_Planar_0_1_serialize_(&setpoint, Out(tx), &size), 0);
    Publish(unimoc::cyphal::kSubServoSetpoint, std::span<const std::byte>{tx.data(), size});
  }

  /// Wheel command with @p torque [N m] for the front left wheel (the drive's default wheel).
  void WheelCommand(const float torque) {
    unimoc_cart_WheelCommand_0_1 command{};
    command.wheel_torque[unimoc_cart_WheelCommand_0_1_WHEEL_FRONT_LEFT] = torque;
    command.wheel_torque[unimoc_cart_WheelCommand_0_1_WHEEL_REAR_RIGHT] = -torque;  // another wheel
    std::size_t size = tx.size();
    zassert_equal(unimoc_cart_WheelCommand_0_1_serialize_(&command, Out(tx), &size), 0);
    Publish(unimoc::cyphal::kSubWheelCommand, std::span<const std::byte>{tx.data(), size});
  }

  /// Plausible pedal state, both channels equal.
  void Pedals(const float drive, const float brake) {
    unimoc_cart_PedalState_0_1 state{};
    state.drive_pedal[0] = drive;
    state.drive_pedal[1] = drive;
    state.brake_pedal[0] = brake;
    state.brake_pedal[1] = brake;
    state.drive_pedal_plausible = true;
    state.brake_pedal_plausible = true;
    std::size_t size = tx.size();
    zassert_equal(unimoc_cart_PedalState_0_1_serialize_(&state, Out(tx), &size), 0);
    Publish(unimoc::cyphal::kSubPedalState, std::span<const std::byte>{tx.data(), size});
  }

  /// Battery status of a BMS with the contactor closed.
  void Battery(const float discharge_limit, const float charge_limit) {
    unimoc_cart_BatteryStatus_0_1 status{};
    status.voltage = 48.0F;
    status.discharge_current_limit = discharge_limit;
    status.charge_current_limit = charge_limit;
    status.contactor = unimoc_cart_BatteryStatus_0_1_CONTACTOR_CLOSED;
    std::size_t size = tx.size();
    zassert_equal(unimoc_cart_BatteryStatus_0_1_serialize_(&status, Out(tx), &size), 0);
    Publish(unimoc::cyphal::kSubBatteryStatus, std::span<const std::byte>{tx.data(), size});
  }

  /// Waits for a drive status published from now on.
  bool DriveStatus(unimoc_cart_DriveStatus_0_1& status) {
    Transfer transfer{};
    while (client.Receive(rx, transfer) == ReceiveResult::kReceived) {
    }
    for (unsigned i = 0U; i < 100U; ++i) {
      Run(1U);
      while (client.Receive(rx, transfer) == ReceiveResult::kReceived) {
        if (transfer.port_id == unimoc::cyphal::kPubDriveStatus) {
          std::size_t size = transfer.payload.size();
          return unimoc_cart_DriveStatus_0_1_deserialize_(&status, In(transfer.payload), &size) >= 0;
        }
      }
    }
    return false;
  }

  /// Engaged, current setpoint refreshed every 10 ms for @p milliseconds.
  void Hold(const float i_d, const float i_q, const unsigned milliseconds) {
    HoldWith([&] { Current(i_d, i_q); }, milliseconds);
  }

  /// Engaged, @p setpoint published every 10 ms for @p milliseconds.
  template <typename Setpoint>
  void HoldWith(const Setpoint& setpoint, const unsigned milliseconds) {
    for (unsigned elapsed = 0U; elapsed < milliseconds; elapsed += 10U) {
      Readiness(reg_udral_service_common_Readiness_0_1_ENGAGED);
      setpoint();
      Run(10U);
    }
  }

  bool Request(const uint16_t service, std::span<const std::byte> payload, Transfer& response) {
    const Transfer kTransfer{.port_id = service,
                             .source_node_id = kClientId,
                             .destination_node_id = kDriveId,
                             .transfer_id = next_transfer_id++,
                             .kind = TransferKind::kServiceRequest,
                             .priority = Priority::kNominal,
                             .payload = payload};
    if (!client.Send(kTransfer)) {
      return false;
    }
    for (unsigned i = 0U; i < 20U; ++i) {
      Run(1U);
      while (client.Receive(rx, response) == ReceiveResult::kReceived) {
        if (response.port_id == service) {
          return true;
        }
      }
    }
    return false;
  }

  /// ExecuteCommand with a text parameter; the output text of the answer goes to last_output.
  uint8_t Command(const uint16_t command, const std::string_view parameter = {}) {
    uavcan_node_ExecuteCommand_Request_1_3 request{};
    request.command = command;
    std::memcpy(std::begin(request.parameter.elements), parameter.data(), parameter.size());
    request.parameter.count = parameter.size();
    std::size_t size = tx.size();
    zassert_equal(uavcan_node_ExecuteCommand_Request_1_3_serialize_(&request, Out(tx), &size), 0);
    Transfer response{};
    zassert_true(Request(port::kExecuteCommand, std::span<const std::byte>{tx.data(), size}, response));
    uavcan_node_ExecuteCommand_Response_1_3 result{};
    size = response.payload.size();
    zassert_equal(uavcan_node_ExecuteCommand_Response_1_3_deserialize_(&result, In(response.payload), &size),
                  0);
    last_output.assign(reinterpret_cast<const char*>(std::begin(result.output.elements)),
                       result.output.count);
    return result.status;
  }

  /// Waits for the next commissioning status message (published on change and at least every 2 s).
  bool Commissioning(unimoc_drive_CommissioningStatus_0_1& status) {
    Transfer transfer{};
    while (client.Receive(rx, transfer) == ReceiveResult::kReceived) {
    }
    for (unsigned i = 0U; i < 2100U; ++i) {
      Run(1U);
      while (client.Receive(rx, transfer) == ReceiveResult::kReceived) {
        if (transfer.port_id == unimoc::cyphal::kPubCommissioning) {
          std::size_t size = transfer.payload.size();
          return unimoc_drive_CommissioningStatus_0_1_deserialize_(&status, In(transfer.payload), &size) >= 0;
        }
      }
    }
    return false;
  }

  /// Writes (or reads, with an empty value) a register; returns the value of the response.
  uavcan_register_Value_1_0 Access(std::string_view name, const uavcan_register_Value_1_0& value) {
    uavcan_register_Access_Request_1_0 request{};
    std::memcpy(std::begin(request.name.name.elements), name.data(), name.size());
    request.name.name.count = name.size();
    request.value = value;
    std::size_t size = tx.size();
    zassert_equal(uavcan_register_Access_Request_1_0_serialize_(&request, Out(tx), &size), 0);
    Transfer response{};
    zassert_true(Request(port::kRegisterAccess, std::span<const std::byte>{tx.data(), size}, response));
    uavcan_register_Access_Response_1_0 result{};
    size = response.payload.size();
    zassert_equal(uavcan_register_Access_Response_1_0_deserialize_(&result, In(response.payload), &size), 0);
    return result.value;
  }

  /// Waits for a telemetry message published from now on.
  bool Telemetry(unimoc_drive_Telemetry_0_1& telemetry) {
    Transfer transfer{};
    while (client.Receive(rx, transfer) == ReceiveResult::kReceived) {
    }
    for (unsigned i = 0U; i < 100U; ++i) {
      Run(1U);
      while (client.Receive(rx, transfer) == ReceiveResult::kReceived) {
        if (transfer.port_id == unimoc::cyphal::kPubTelemetry) {
          std::size_t size = transfer.payload.size();
          return unimoc_drive_Telemetry_0_1_deserialize_(&telemetry, In(transfer.payload), &size) >= 0;
        }
      }
    }
    return false;
  }

  SettingsStore store;
  CanTestBus bus;
  CanTestMedia drive_media;
  CanTestMedia client_media;
  std::array<CanMedia*, 1> drive_interfaces;
  std::array<CanMedia*, 1> client_interfaces;
  CanardArena<32768> drive_arena;
  CanardArena<32768> client_arena;
  CanardTransport drive_transport;
  CanardTransport client;
  CyphalNode node;
  std::array<uint8_t, 16> unique_id{1U, 2U, 3U};
  unimoc::drive::DriveNode drive;
  /// The e-bike control of the e-bike tests (attached with drive.SetLocalControl()).
  unimoc::ebike::EbikeControl ebike{unimoc::hardware::GetEbikeIo()};
  std::array<std::byte, 600> rx{};
  std::array<std::byte, 600> tx{};
  std::string last_output;  ///< Output text of the last ExecuteCommand answer.
  uint64_t now{1'000'000U};
  uint8_t next_transfer_id{0U};
};

alignas(Rig) std::array<std::byte, sizeof(Rig)> g_storage;
Rig* g_rig = nullptr;

void Before(void* /*unused*/) {
  kPowerStage.Disable();
  // The test motor (no permanent magnet) unless a test configures another.
  zassert_equal(sim::Plant().Configure(TestProfile().factory_settings), unimoc::sim::Status::kOk);
  sim::Plant().SetInput(unimoc::sim::SimulationInput{});
  sim::Plant().SetDcLinkVoltage(48.0_V);
  sim::ClearHallCodeOverride();
  sim::SetFreeRunning(false);
  sim::SetExternalEnable(true);
  sim::ClearPhaseCurrentOverride();
  kPowerStage.ClearFault();
  CurrentControlIsr::Instance().ClearTrips();
  EraseAllRegisters();
  // Locked rotor: the current loop works on a standing plant.
  sim::Plant().SetShaftMode(unimoc::sim::ShaftMode::kImposedSpeed, AngularVelocity{});
  g_restarts = 0;
  g_settings_applied = 0;
  sim::UseManualTime(true);
  sim::SetPedal(AngularVelocity{}, Torque{});
  sim::SilenceDisplay();
  // NOLINTNEXTLINE(cppcoreguidelines-owning-memory): placement new into static storage, destroyed in After().
  g_rig = new (g_storage.data()) Rig{};
}

void After(void* /*unused*/) {
  kPowerStage.Disable();
  g_rig->~Rig();
  g_rig = nullptr;
}

/// Plant current in the frame the controller works in (its estimated angle).
unimoc::system::Rotor<Current> PlantCurrent() {
  const auto kAlphaBeta = sim::Plant().Output().phase_currents.ToStator();
  const unimoc::system::SinCos<Ratio> kSinCos(CurrentControlIsr::Instance().mech_obs.theta);
  return kAlphaBeta.ToRotor(kSinCos);
}

uavcan_register_Value_1_0 Empty() { return uavcan_register_Value_1_0{}; }

uavcan_register_Value_1_0 Real(const float value) {
  uavcan_register_Value_1_0 result{};
  unimoc::cyphal::SetReal32(result, value);
  return result;
}

uavcan_register_Value_1_0 Natural8(const uint8_t value) {
  uavcan_register_Value_1_0 result{};
  unimoc::cyphal::SetNatural8(result, value);
  return result;
}

int64_t Integer(const uavcan_register_Value_1_0& value) {
  return unimoc::cyphal::GetInteger(value).value_or(-1);
}

/// A motor with a permanent magnet, Hall sensors and a speed loop, written over
/// the bus; the plant takes the same motor on a free shaft.
void ConfigureSpeedControl(Rig& rig) {
  (void)rig.Access(reg::kMotorPolePairs, Natural8(7U));
  (void)rig.Access(reg::kPmsmFluxPm, Real(5e-3F));
  (void)rig.Access(reg::kHallEnable, Natural8(1U));
  (void)rig.Access(reg::kPmsmFluxObsRSpeed, Real(1.0F));
  (void)rig.Access(reg::kSpeedKp, Real(0.01F));
  (void)rig.Access(reg::kSpeedKi, Real(40.0F));
  rig.Run(2U);  // applied while disengaged
  zassert_equal(sim::Plant().Configure(rig.store.GetSnapshot().Get()), unimoc::sim::Status::kOk);
  sim::Plant().SetShaftMode(unimoc::sim::ShaftMode::kFree);
}

AngularVelocity PlantSpeed() { return sim::Plant().Output().mechanical_speed; }

/// Torque constant of the drive's settings: 3/2 p psi_pm.
MagneticFlux TorqueConstant(const Rig& rig) {
  const auto kSnapshot = rig.store.GetSnapshot();
  return 1.5F * static_cast<float>(kSnapshot.Get().pole_pairs) * kSnapshot.Get().flux_pm;
}

/// A permanent magnet for the test motor, so a torque makes a q current.
MagneticFlux GiveTheMotorAMagnet(Rig& rig) {
  (void)rig.Access(reg::kPmsmFluxPm, Real(5e-3F));
  rig.Run(2U);  // applied while disengaged
  const MagneticFlux kTorqueConstant = TorqueConstant(rig);
  zassert_true(kTorqueConstant > MagneticFlux{});
  return kTorqueConstant;
}
Angle PlantAngle() { return sim::Plant().Output().mechanical_angle.AbsoluteAngle(); }

/// Pole pairs of ConfigureSpeedControl().
constexpr float kSpeedControlPolePairs = 7.0F;

/// True shaft angle of the drive's home (position zero). The drive counts the
/// position from the estimate at its start, which the Hall sector places only
/// within half a sector (30 degrees electrical) of the shaft; the edges
/// correct the angle estimate later, but not the home. Valid at standstill.
Angle TrueHome() {
  const Angle kEstimateError =
      unimoc::observer::WrapAngle(sim::Plant().Output().electrical_angle.AngleInRevolution() -
                                  CurrentControlIsr::Instance().mech_obs.theta);
  return PlantAngle() - OuterLoop::Instance().Status().position - (kEstimateError / kSpeedControlPolePairs);
}

}  // namespace

ZTEST(sim_cyphal_drive, test_engage_and_current_setpoint_reach_the_plant) {
  Rig& rig = *g_rig;
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kDisabled);
  rig.Hold(0.0F, 10.0F, 30U);
  zassert_true(rig.drive.IsEngaged());
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kEnabled);
  zexpect_within(CurrentControlIsr::Instance().state.i_ref.q, 10.0_A, 1e-3_A);
  zexpect_within(PlantCurrent().q, 10.0_A, 0.5_A, "i_q = %f", static_cast<double>(PlantCurrent().q.Value()));

  unimoc_drive_Telemetry_0_1 telemetry{};
  zassert_true(rig.Telemetry(telemetry));
  zexpect_true(telemetry.engaged);
  zexpect_equal(telemetry.power_stage, unimoc_drive_Telemetry_0_1_POWER_STAGE_ENABLED);
  zexpect_within(telemetry.i_q_ref, 10.0F, 1e-3F);
  zexpect_within(telemetry.i_q, 10.0F, 0.5F);
  zexpect_within(telemetry.dc_link_voltage, 48.0F, 0.5F);
  zexpect_true(telemetry.control_cycles > 0U);
}

ZTEST(sim_cyphal_drive, test_readiness_and_setpoint_timeouts) {
  Rig& rig = *g_rig;
  rig.Hold(0.0F, 5.0F, 20U);
  zassert_true(rig.drive.IsEngaged());

  // Readiness refreshed, setpoint not: back to zero current, still engaged.
  for (unsigned elapsed = 0U; elapsed < 700U; elapsed += 100U) {
    rig.Readiness(reg_udral_service_common_Readiness_0_1_ENGAGED);
    rig.Run(100U);
  }
  zexpect_true(rig.drive.IsEngaged());
  zexpect_equal(CurrentControlIsr::Instance().state.i_ref.q, 0.0_A);

  // No readiness for longer than the command timeout (0.5 s): disengaged.
  rig.Run(600U);
  zexpect_false(rig.drive.IsEngaged());
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kDisabled);
}

ZTEST(sim_cyphal_drive, test_sleep_disengages_and_setpoints_are_limited) {
  Rig& rig = *g_rig;
  rig.Hold(30.0F, 40.0F, 20U);  // |i| = 50 A, at the test motor limit of 50 A
  zexpect_within(CurrentControlIsr::Instance().state.i_ref.d, 30.0_A, 1e-3_A);
  rig.Hold(300.0F, 400.0F, 10U);  // scaled to 50 A, direction kept
  zexpect_within(CurrentControlIsr::Instance().state.i_ref.d, 30.0_A, 1e-3_A);
  zexpect_within(CurrentControlIsr::Instance().state.i_ref.q, 40.0_A, 1e-3_A);

  rig.Readiness(reg_udral_service_common_Readiness_0_1_SLEEP);
  rig.Run(5U);
  zexpect_false(rig.drive.IsEngaged());
  zexpect_equal(CurrentControlIsr::Instance().state.i_ref.q, 0.0_A);
}

ZTEST(sim_cyphal_drive, test_torque_setpoint_uses_the_torque_constant) {
  Rig& rig = *g_rig;
  (void)rig.Access(reg::kPmsmFluxPm, Real(5e-3F));  // the test motor has no flux
  const auto kSnapshot = rig.store.GetSnapshot();
  const MagneticFlux kTorqueConstant =
      1.5F * static_cast<float>(kSnapshot.Get().pole_pairs) * kSnapshot.Get().flux_pm;
  zassert_true(kTorqueConstant > MagneticFlux{});
  uavcan_si_unit_torque_Scalar_1_0 torque{(2.0_A * kTorqueConstant).Value()};  // DSDL payload
  for (unsigned elapsed = 0U; elapsed < 20U; elapsed += 10U) {
    rig.Readiness(reg_udral_service_common_Readiness_0_1_ENGAGED);
    std::size_t size = rig.tx.size();
    zassert_equal(uavcan_si_unit_torque_Scalar_1_0_serialize_(&torque, Out(rig.tx), &size), 0);
    rig.Publish(unimoc::cyphal::kSubTorqueSetpoint, std::span<const std::byte>{rig.tx.data(), size});
    rig.Run(10U);
  }
  zexpect_within(CurrentControlIsr::Instance().state.i_ref.q, 2.0_A, 1e-3_A);
}

ZTEST(sim_cyphal_drive, test_trip_disengages_and_needs_a_rearm) {
  Rig& rig = *g_rig;
  rig.Hold(0.0F, 5.0F, 20U);
  zassert_true(rig.drive.IsEngaged());
  sim::SetPhaseCurrentOverride(80.0F, -40.0F, -40.0F);  // above the 60 A trip limit
  rig.Hold(0.0F, 5.0F, 20U);
  sim::ClearPhaseCurrentOverride();
  zexpect_false(rig.drive.IsEngaged());
  rig.Hold(0.0F, 5.0F, 50U);  // still ENGAGED: no re-arm without a pause
  zexpect_false(rig.drive.IsEngaged());

  rig.Readiness(reg_udral_service_common_Readiness_0_1_STANDBY);
  rig.Run(5U);
  rig.Hold(0.0F, 5.0F, 20U);
  zexpect_true(rig.drive.IsEngaged());
  zexpect_false(CurrentControlIsr::Instance().state.overcurrent_tripped);
}

ZTEST(sim_cyphal_drive, test_missing_external_enable_refuses_engage) {
  Rig& rig = *g_rig;
  sim::SetExternalEnable(false);
  rig.Hold(0.0F, 5.0F, 20U);
  zexpect_false(rig.drive.IsEngaged());
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kBlocked);

  // Back: the break stays latched until STANDBY acknowledges it, without engaging.
  (void)rig.Access(reg::kMotorPolePairs, Real(5.0F));  // something to save
  sim::SetExternalEnable(true);
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kFault);
  rig.Readiness(reg_udral_service_common_Readiness_0_1_SLEEP);
  rig.Run(2U);
  zexpect_equal(rig.Command(cmd::kCommandStorePersistentStates), cmd::kStatusBadState);
  rig.Readiness(reg_udral_service_common_Readiness_0_1_STANDBY);
  rig.Run(2U);
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kDisabled);
  zexpect_false(rig.drive.IsEngaged());
  zexpect_equal(rig.Command(cmd::kCommandStorePersistentStates), cmd::kStatusSuccess);
}

ZTEST(sim_cyphal_drive, test_registers_change_live_settings_within_limits) {
  Rig& rig = *g_rig;
  auto value = rig.Access(reg::kMotorIMax, Real(20.0F));
  zexpect_equal(unimoc::cyphal::GetNumber(value).value_or(0.0F), 20.0F);
  value = rig.Access(reg::kMotorIMax, Real(500.0F));  // above the hardware capability
  zexpect_equal(unimoc::cyphal::GetNumber(value).value_or(0.0F), 20.0F);

  // The new limit applies to the next setpoint.
  rig.Hold(0.0F, 40.0F, 20U);
  zexpect_within(CurrentControlIsr::Instance().state.i_ref.q, 20.0_A, 1e-3_A);

  value = rig.Access(reg::kNodeId, Empty());
  zexpect_true(uavcan_register_Value_1_0_is_natural16_(&value));
  zexpect_equal(unimoc::cyphal::GetInteger(value).value_or(0), kDriveId);
  value = rig.Access(reg::kSubReadinessType, Empty());
  zexpect_equal(unimoc::cyphal::GetString(value).value_or(""),
                std::string_view{"reg.udral.service.common.Readiness.0.1"});
  value = rig.Access(reg::kNodeUniqueId, Empty());
  zexpect_true(uavcan_register_Value_1_0_is_unstructured_(&value));
  zexpect_equal(value.unstructured.value.elements[2], 3U);
}

ZTEST(sim_cyphal_drive, test_controller_restarts_with_new_settings_while_disengaged) {
  Rig& rig = *g_rig;
  auto& isr = CurrentControlIsr::Instance();
  const VoltagePerCurrent kKp = isr.cc.kp_q;
  (void)rig.Access(reg::kCurrentKpQ, Real((kKp * 2.0F).Value()));  // register payload: plain number
  rig.Run(2U);
  zexpect_within(isr.cc.kp_q, kKp * 2.0F, 1e-6_V_per_A);
}

ZTEST(sim_cyphal_drive, test_store_is_refused_while_engaged_and_persists_otherwise) {
  Rig& rig = *g_rig;
  (void)rig.Access(reg::kMotorPolePairs, Real(7.0F));
  rig.Hold(0.0F, 1.0F, 20U);
  zassert_true(rig.drive.IsEngaged());
  zexpect_equal(rig.Command(cmd::kCommandStorePersistentStates), cmd::kStatusBadState);
  zexpect_equal(rig.Command(cmd::kCommandFactoryReset), cmd::kStatusBadState);

  rig.Readiness(reg_udral_service_common_Readiness_0_1_SLEEP);
  rig.Run(5U);
  zexpect_equal(rig.Command(cmd::kCommandStorePersistentStates), cmd::kStatusSuccess);
  auto reloaded = MakeStore();
  zassert_equal(reloaded.Load(), SettingsStatus::kSuccess);
  zexpect_equal(reloaded.GetSnapshot().Get().pole_pairs, 7U);

  zexpect_equal(rig.Command(cmd::kCommandFactoryReset), cmd::kStatusSuccess);
  zexpect_equal(rig.store.GetSnapshot().Get().pole_pairs, TestProfile().factory_settings.pole_pairs);
  auto erased = MakeStore();
  zexpect_equal(erased.Load(), SettingsStatus::kFactoryDefaults);
}

ZTEST(sim_cyphal_drive, test_emergency_stop_restart_and_unknown_commands) {
  Rig& rig = *g_rig;
  rig.Hold(0.0F, 1.0F, 20U);
  zassert_true(rig.drive.IsEngaged());
  zexpect_equal(rig.Command(cmd::kCommandEmergencyStop), cmd::kStatusSuccess);
  rig.Run(2U);
  zexpect_false(rig.drive.IsEngaged());
  rig.Readiness(reg_udral_service_common_Readiness_0_1_STANDBY);
  rig.Run(2U);
  rig.Hold(0.0F, 1.0F, 20U);
  zexpect_false(rig.drive.IsEngaged());  // until the restart

  zexpect_equal(rig.Command(cmd::kCommandPowerOff), cmd::kStatusBadCommand);
  zexpect_equal(rig.Command(cmd::kCommandBeginSoftwareUpdate), cmd::kStatusBadCommand);
  zexpect_equal(rig.Command(cmd::kCommandRestart), cmd::kStatusSuccess);
  zexpect_equal(g_restarts, 0);  // after the response is out
  rig.Run(250U);
  zexpect_equal(g_restarts, 1);
}

ZTEST(sim_cyphal_drive, test_speed_setpoint_runs_the_speed_loop) {
  Rig& rig = *g_rig;
  ConfigureSpeedControl(rig);
  rig.HoldWith([&] { rig.Speed(30.0F); }, 500U);
  zassert_true(rig.drive.IsEngaged());
  zexpect_within(PlantSpeed(),
                 30.0_rad_per_s,
                 1.0_rad_per_s,
                 "speed %f",
                 static_cast<double>(PlantSpeed().Value()));
  const auto kStatus = OuterLoop::Instance().Status();
  zexpect_equal(kStatus.mode, unimoc::cyphal::ControlMode::kSpeed);
  zexpect_within(kStatus.speed, 30.0_rad_per_s, 1.0_rad_per_s);

  unimoc_drive_Telemetry_0_1 telemetry{};
  zassert_true(rig.Telemetry(telemetry));
  zexpect_within(telemetry.electrical_speed, 7.0F * 30.0F, 7.0F);

  // Without new setpoints the drive coasts (zero current), still engaged.
  for (unsigned elapsed = 0U; elapsed < 700U; elapsed += 100U) {
    rig.Readiness(reg_udral_service_common_Readiness_0_1_ENGAGED);
    rig.Run(100U);
  }
  zexpect_true(rig.drive.IsEngaged());
  zexpect_equal(CurrentControlIsr::Instance().state.i_ref.q, 0.0_A);
  zexpect_equal(OuterLoop::Instance().Status().mode, unimoc::cyphal::ControlMode::kTorque);
}

ZTEST(sim_cyphal_drive, test_position_setpoint_moves_the_shaft) {
  Rig& rig = *g_rig;
  ConfigureSpeedControl(rig);
  const Angle kStart = PlantAngle();
  rig.HoldWith([&] { rig.Position(3.0F); }, 1500U);
  const Angle kHome = TrueHome();
  constexpr Angle kHalfSector = (std::numbers::pi_v<float> / 6.0F / kSpeedControlPolePairs) * 1.0_rad;
  zexpect_within(kHome,
                 kStart,
                 kHalfSector + 0.005_rad,
                 "home %f rad from the start",
                 static_cast<double>((kHome - kStart).Value()));
  zexpect_within(PlantAngle() - kHome,
                 3.0_rad,
                 0.05_rad,
                 "moved %f rad",
                 static_cast<double>((PlantAngle() - kHome).Value()));
  const auto kStatus = OuterLoop::Instance().Status();
  zexpect_true(kStatus.in_position,
               "estimate %f rad, %f rad/s",
               static_cast<double>(kStatus.position.Value()),
               static_cast<double>(kStatus.speed.Value()));
}

ZTEST(sim_cyphal_drive, test_servo_setpoint_selects_the_mode) {
  Rig& rig = *g_rig;
  ConfigureSpeedControl(rig);
  constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
  // Velocity first (the torque after it is a profile limit and ignored).
  rig.HoldWith([&] { rig.Servo(kNan, -20.0F, 0.5F); }, 500U);
  zexpect_equal(OuterLoop::Instance().Status().mode, unimoc::cyphal::ControlMode::kSpeed);
  zexpect_within(PlantSpeed(),
                 -20.0_rad_per_s,
                 1.0_rad_per_s,
                 "speed %f",
                 static_cast<double>(PlantSpeed().Value()));

  // Position wins over velocity.
  rig.HoldWith([&] { rig.Servo(0.0F, 5.0F, kNan); }, 100U);
  zexpect_equal(OuterLoop::Instance().Status().mode, unimoc::cyphal::ControlMode::kPosition);

  // Torque only: torque mode through the torque constant (3/2 * 7 * 5 mWb).
  rig.HoldWith([&] { rig.Servo(kNan, kNan, 0.0525F); }, 50U);
  zexpect_equal(OuterLoop::Instance().Status().mode, unimoc::cyphal::ControlMode::kTorque);
  zexpect_within(CurrentControlIsr::Instance().state.i_ref.q, 1.0_A, 1e-3_A);

  // Nothing finite: ignored, the last setpoint stays.
  rig.HoldWith([&] { rig.Servo(kNan, kNan, kNan); }, 50U);
  zexpect_within(CurrentControlIsr::Instance().state.i_ref.q, 1.0_A, 1e-3_A);
}

ZTEST(sim_cyphal_drive, test_regenerative_braking_is_limited_by_the_charge_current_and_the_dc_link) {
  Rig& rig = *g_rig;
  ConfigureSpeedControl(rig);  // torque constant 3/2 * 7 * 5 mWb = 0.0525 N m/A
  (void)rig.Access(reg::kBatteryChargeIMax, Real(0.5F));
  zassert_equal(rig.store.GetSnapshot().Get().battery_charge_current_max, 0.5_A);
  rig.Run(2U);  // applied while disengaged
  // A dyno: the shaft turns at 50 rad/s whatever the torque.
  sim::Plant().SetShaftMode(unimoc::sim::ShaftMode::kImposedSpeed, 50.0_rad_per_s);
  constexpr MagneticFlux kTorqueConstant = 1.5F * 7.0F * 5e-3_Wb;

  // Motoring: 20 A make 52.5 W, far below the drive limit.
  rig.Hold(0.0F, 20.0F, 500U);
  zassert_true(rig.drive.IsEngaged());
  zexpect_within(OuterLoop::Instance().Status().current_reference.q, 20.0_A, 1e-3_A);

  // Braking: at most 0.5 A * 48 V = 24 W into the battery. The observer
  // expects the braking torque to slow the shaft, which the dyno does not:
  // its speed estimate (and with it the limit) settles within some 300 ms.
  rig.Hold(0.0F, -20.0F, 500U);
  auto status = OuterLoop::Instance().Status();
  zexpect_within(status.speed,
                 50.0_rad_per_s,
                 1.0_rad_per_s,
                 "estimated speed %f",
                 static_cast<double>(status.speed.Value()));
  zexpect_within(status.current_reference.q,
                 -24.0_W / (kTorqueConstant * status.speed),
                 1e-3_A,
                 "i_q %f",
                 static_cast<double>(status.current_reference.q.Value()));
  const Power kChargePower = (-kTorqueConstant * PlantCurrent().q) * PlantSpeed();
  zexpect_within(kChargePower,
                 24.0_W,
                 2.0_W,
                 "mechanical braking power %f W",
                 static_cast<double>(kChargePower.Value()));

  // At 56.5 V (derating from 55 V to the 58 V trip) half of the 0.5 A remains.
  sim::Plant().SetDcLinkVoltage(56.5_V);
  rig.Hold(0.0F, -20.0F, 500U);
  status = OuterLoop::Instance().Status();
  zexpect_within(status.current_reference.q,
                 -(0.25_A * 56.5_V) / (kTorqueConstant * status.speed),
                 1e-3_A,
                 "i_q %f",
                 static_cast<double>(status.current_reference.q.Value()));
  zexpect_true(rig.drive.IsEngaged());

  // From the trip level the control cycle trips the power stage and the drive disengages.
  sim::Plant().SetDcLinkVoltage(58.5_V);
  rig.Hold(0.0F, -20.0F, 20U);
  zexpect_false(rig.drive.IsEngaged());
  zexpect_true(CurrentControlIsr::Instance().state.overvoltage_tripped);
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kFault);
}

ZTEST(sim_cyphal_drive, test_gains_are_tuned_live_while_engaged) {
  Rig& rig = *g_rig;
  rig.Hold(0.0F, 2.0F, 50U);
  zassert_true(rig.drive.IsEngaged());
  const int kApplied = g_settings_applied;

  // Gains reach the running loops at once, without a restart.
  (void)rig.Access(reg::kCurrentKpQ, Real(0.7F));
  (void)rig.Access(reg::kSpeedKp, Real(0.03F));
  rig.Hold(0.0F, 2.0F, 5U);
  zexpect_true(rig.drive.IsEngaged());
  zexpect_within(CurrentControlIsr::Instance().cc.kp_q, 0.7_V_per_A, 1e-6_V_per_A);
  zexpect_equal(g_settings_applied, kApplied, "restarted for a gain");
  zexpect_equal(Integer(rig.Access(reg::kSettingsPending, Empty())), 0);

  // A motor parameter needs a restart: it waits for the disengage, the gain stays.
  (void)rig.Access(reg::kMotorPolePairs, Natural8(5U));
  rig.Hold(0.0F, 2.0F, 5U);
  zexpect_true(rig.drive.IsEngaged());
  zexpect_equal(g_settings_applied, kApplied);
  zexpect_equal(Integer(rig.Access(reg::kSettingsPending, Empty())), 1);
  zexpect_equal(Integer(rig.Access(reg::kSettingsPending, Natural8(0U))), 1, "read only");
  rig.Readiness(reg_udral_service_common_Readiness_0_1_SLEEP);
  rig.Run(2U);
  zexpect_equal(g_settings_applied, kApplied + 1);
  zexpect_equal(Integer(rig.Access(reg::kSettingsPending, Empty())), 0);
  zexpect_within(CurrentControlIsr::Instance().cc.kp_q, 0.7_V_per_A, 1e-6_V_per_A);
}

ZTEST(sim_cyphal_drive, test_speed_setpoint_is_limited_to_the_motor_speed) {
  Rig& rig = *g_rig;
  ConfigureSpeedControl(rig);
  (void)rig.Access(reg::kMotorOmegaMax, Real(7.0F * 20.0F));  // electrical: 20 rad/s mechanical
  rig.Run(2U);
  rig.HoldWith([&] { rig.Speed(50.0F); }, 600U);
  zexpect_within(PlantSpeed(),
                 20.0_rad_per_s,
                 1.0_rad_per_s,
                 "speed %f",
                 static_cast<double>(PlantSpeed().Value()));
}

namespace startup_aid {

using unimoc::startup::FsmState;
using unimoc::startup::StartupNeeds;
constexpr uint16_t kStart = unimoc_drive_CommissioningStatus_0_1_COMMAND_START_STARTUP_AID;
constexpr uint16_t kContinue = unimoc_drive_CommissioningStatus_0_1_COMMAND_CONTINUE;
constexpr uint16_t kReading = unimoc_drive_CommissioningStatus_0_1_COMMAND_SET_READING;
constexpr uint16_t kAbort = unimoc_drive_CommissioningStatus_0_1_COMMAND_ABORT;

FsmState Step(const Rig& rig) { return rig.drive.Startup().State(); }

/// Runs the drive until the step has measured, needs a reading or the run ended (one sample per millisecond).
void Measure(Rig& rig, const unsigned limit_ms = 7000U) {
  rig.Run(2U);  // the FSM takes a reading or a continue at its next run
  for (unsigned i = 0U; i < limit_ms; i += 10U) {
    const auto kStatus = rig.drive.Startup().Status();
    if (!kStatus.active || kStatus.measured || kStatus.needs == StartupNeeds::kDcLinkVoltage ||
        kStatus.needs == StartupNeeds::kPhaseCurrent) {
      return;
    }
    rig.Run(10U);
  }
}

/// Waits until the step has measured, continues (with a confirmation) and lets the FSM take the step.
void Next(Rig& rig, const std::string_view confirmation = {}) {
  Measure(rig);
  zexpect_equal(rig.Command(kContinue, confirmation), cmd::kStatusSuccess, "%s", rig.last_output.c_str());
  rig.Run(2U);
}

}  // namespace startup_aid

ZTEST(sim_cyphal_drive, test_startup_aid_commands_status_and_abort) {
  using namespace startup_aid;
  Rig& rig = *g_rig;
  unimoc_drive_CommissioningStatus_0_1 status{};
  zassert_true(rig.Commissioning(status));
  zexpect_equal(status.procedure, unimoc_drive_CommissioningStatus_0_1_PROCEDURE_NONE);
  zexpect_equal(status.state, unimoc_drive_CommissioningStatus_0_1_STATE_IDLE);
  zexpect_equal(rig.Command(kContinue), cmd::kStatusBadState);  // nothing runs

  zexpect_equal(rig.Command(kStart), cmd::kStatusSuccess);
  zexpect_equal(Step(rig), FsmState::kPwmDisable);
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kEnabled);
  zexpect_equal(rig.Command(kStart), cmd::kStatusBadState, "runs already");

  // The FSM owns the outputs: no engage, no settings restart, no factory reset.
  rig.Hold(0.0F, 5.0F, 30U);
  zexpect_false(rig.drive.IsEngaged());
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kEnabled);
  zexpect_equal(rig.Command(cmd::kCommandFactoryReset), cmd::kStatusBadState);
  rig.Readiness(reg_udral_service_common_Readiness_0_1_SLEEP);  // clears the re-arm
  rig.Run(2U);

  Next(rig);
  zexpect_equal(Step(rig), FsmState::kAdcOffsetCal);
  // A continue before the step has measured is refused: it would continue on its own later.
  zassert_true(rig.Commissioning(status));
  zexpect_equal(status.procedure, unimoc_drive_CommissioningStatus_0_1_PROCEDURE_STARTUP_AID);
  zexpect_equal(status.step, 2U);
  zexpect_equal(status.state, unimoc_drive_CommissioningStatus_0_1_STATE_MEASURING);
  zexpect_true(status.progress < 100U);
  zexpect_equal(rig.Command(kContinue), cmd::kStatusBadState);
  zexpect_true(rig.last_output == "the step is still measuring", "%s", rig.last_output.c_str());
  Measure(rig);
  zassert_true(rig.Commissioning(status));
  zexpect_equal(status.state, unimoc_drive_CommissioningStatus_0_1_STATE_WAITING);
  zexpect_equal(status.progress, 100U);
  zexpect_equal(rig.Command(kContinue, "motor-connected"), cmd::kStatusBadParameter, "not this step");
  zexpect_equal(rig.Command(kContinue, "anything"), cmd::kStatusBadParameter);
  for (unsigned i = 0U; i < 5U; ++i) {  // ADC_OFFSET_CAL .. DUTY_FORCE_HIGH
    Next(rig);
  }
  zexpect_equal(Step(rig), FsmState::kDcLinkVoltageCheck);

  // The step measures, then needs the multimeter reading.
  Measure(rig);
  zassert_true(rig.Commissioning(status));
  zexpect_equal(status.state, unimoc_drive_CommissioningStatus_0_1_STATE_NEEDS_READING);
  zexpect_equal(status.needs, unimoc_drive_CommissioningStatus_0_1_NEEDS_DC_LINK_VOLTAGE);
  zexpect_equal(rig.Command(kContinue), cmd::kStatusBadState, "needs the reading");
  zexpect_equal(rig.Command(kReading, "vdc=abc"), cmd::kStatusBadParameter);
  zexpect_equal(rig.Command(kReading, "vdc=-48"), cmd::kStatusBadParameter);
  zexpect_equal(rig.Command(kReading, "power=48"), cmd::kStatusBadParameter);
  zexpect_equal(rig.Command(kReading, "vdc=48.0"), cmd::kStatusSuccess);
  Measure(rig);
  zexpect_equal(status.passed & 0x6U, 0x6U, "PWM_DISABLE and ADC_OFFSET_CAL passed");
  zassert_true(rig.Commissioning(status));
  zexpect_equal(status.state, unimoc_drive_CommissioningStatus_0_1_STATE_WAITING);
  zexpect_true((status.passed & (1U << 7U)) != 0U, "DC_LINK_VOLTAGE_CHECK passed");

  auto results = rig.Access(reg::kStartupResults, Empty());
  zassert_true(uavcan_register_Value_1_0_is_natural8_(&results));
  zexpect_equal(results.natural8.value.count, 14U);
  zexpect_equal(results.natural8.value.elements[2], 1U);  // ADC_OFFSET_CAL passed (no offset in the sim)

  // Abort: FAULT, outputs off; a new run starts again from the beginning.
  zexpect_equal(rig.Command(kAbort), cmd::kStatusSuccess);
  zexpect_equal(Step(rig), FsmState::kFault);
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kDisabled);
  zassert_true(rig.Commissioning(status));
  zexpect_equal(status.state, unimoc_drive_CommissioningStatus_0_1_STATE_ABORTED);
  zexpect_equal(status.fault, unimoc_drive_CommissioningStatus_0_1_FAULT_ABORTED);
  zexpect_equal(rig.Command(kReading, "vdc=48"), cmd::kStatusBadState, "not running");
  zexpect_equal(rig.Command(kStart), cmd::kStatusSuccess);
  zexpect_equal(Step(rig), FsmState::kPwmDisable);
  zexpect_equal(rig.Command(kAbort), cmd::kStatusSuccess);

  // After the run the drive engages again.
  rig.Readiness(reg_udral_service_common_Readiness_0_1_STANDBY);
  rig.Run(2U);
  rig.Hold(0.0F, 5.0F, 30U);
  zexpect_true(rig.drive.IsEngaged());
}

ZTEST(sim_cyphal_drive, test_startup_aid_is_refused_while_engaged) {
  using namespace startup_aid;
  Rig& rig = *g_rig;
  rig.Hold(0.0F, 2.0F, 20U);
  zassert_true(rig.drive.IsEngaged());
  zexpect_equal(rig.Command(kStart), cmd::kStatusBadState);
  zexpect_true(rig.last_output == "engaged: disengage first", "%s", rig.last_output.c_str());
  zexpect_equal(Step(rig), FsmState::kIdle);
  zexpect_true(rig.drive.IsEngaged());
}

ZTEST(sim_cyphal_drive, test_emergency_stop_aborts_a_startup_run) {
  using namespace startup_aid;
  Rig& rig = *g_rig;
  zexpect_equal(rig.Command(kStart), cmd::kStatusSuccess);
  zexpect_equal(rig.Command(cmd::kCommandEmergencyStop), cmd::kStatusSuccess);
  zexpect_equal(Step(rig), FsmState::kFault);
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kDisabled);
  unimoc_drive_CommissioningStatus_0_1 status{};
  zassert_true(rig.Commissioning(status));
  zexpect_equal(status.state, unimoc_drive_CommissioningStatus_0_1_STATE_ABORTED);
  zexpect_equal(status.fault, unimoc_drive_CommissioningStatus_0_1_FAULT_EMERGENCY_STOP);
  zexpect_equal(rig.Command(kStart), cmd::kStatusBadState, "the latched emergency stop holds a new run off");
}

ZTEST(sim_cyphal_drive, test_a_second_abort_leaves_the_control_to_the_drive) {
  using namespace startup_aid;
  Rig& rig = *g_rig;
  zexpect_equal(rig.Command(kStart), cmd::kStatusSuccess);
  zexpect_equal(rig.Command(kAbort), cmd::kStatusSuccess);
  zexpect_equal(Step(rig), FsmState::kFault);
  rig.Run(2U);
  // A tool that aborts every drive on a stop sends the abort again: nothing changes.
  zexpect_equal(rig.Command(kAbort), cmd::kStatusSuccess);
  zexpect_equal(Step(rig), FsmState::kFault);
  rig.Run(2U);
  rig.Hold(0.0F, 10.0F, 30U);
  zassert_true(rig.drive.IsEngaged());
  UNIMOC_EXPECT_NEAR(PlantCurrent().q, 10.0_A, 0.5_A);  // the current loop runs, no forced duties
}

ZTEST(sim_cyphal_drive, test_startup_run_to_done_commits_the_calibration) {
  using namespace startup_aid;
  Rig& rig = *g_rig;
  const auto kNumber = [&rig](const char* name) {
    return unimoc::cyphal::GetNumber(rig.Access(name, Empty())).value_or(-1.0F);
  };
  // Constant readings: zero offsets 0.2 A and -0.1 A, the plant behind them unused.
  sim::SetPhaseCurrentOverride(0.2F, -0.1F, -0.1F);
  zexpect_equal(rig.Command(kStart), cmd::kStatusSuccess);
  for (unsigned i = 0U; i < 6U; ++i) {  // PWM_DISABLE .. DUTY_FORCE_HIGH
    Next(rig);
  }
  zexpect_equal(Step(rig), FsmState::kDcLinkVoltageCheck);
  zexpect_equal(rig.Command(kReading, "vdc=48"), cmd::kStatusSuccess);
  Next(rig);
  Next(rig);  // GATE_DRIVER_ENABLE_CHECK (fails without a gate response, continues)
  Next(rig, "motor-connected");
  zexpect_equal(Step(rig), FsmState::kPhaseAdcAlignment);
  Measure(rig);
  // The calibration current before the next step collects: 0.8 A above the offset.
  sim::SetPhaseCurrentOverride(1.0F, -0.5F, -0.5F);
  Next(rig);
  zexpect_equal(Step(rig), FsmState::kCurrentSenseCalibration);
  // The reading may come after the measurement: the step waits for it.
  Measure(rig);
  unimoc_drive_CommissioningStatus_0_1 status{};
  zassert_true(rig.Commissioning(status));
  zexpect_equal(status.needs, unimoc_drive_CommissioningStatus_0_1_NEEDS_PHASE_CURRENT);
  zexpect_equal(rig.Command(kReading, "current=1.0"), cmd::kStatusSuccess);
  Next(rig);

  // DONE: outputs off, the calibration in the settings registers.
  zexpect_equal(Step(rig), FsmState::kDone);
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kDisabled);
  zassert_true(rig.Commissioning(status));
  zexpect_equal(status.state, unimoc_drive_CommissioningStatus_0_1_STATE_DONE);
  zexpect_true((status.failed & (1U << 8U)) != 0U, "the gate check failed (no gate response in the sim)");
  zexpect_within(kNumber(reg::kStartupAdcOffsetA), 0.2F, 1e-5F);
  zexpect_within(kNumber(reg::kStartupAdcOffsetB), -0.1F, 1e-5F);
  zexpect_within(kNumber(reg::kStartupGainA), 1.25F, 1e-3F);  // 1 / (0.8 A / 1 A)
  zexpect_within(kNumber(reg::kStartupGainB), 1.25F, 1e-3F);
  zexpect_within(kNumber(reg::kStartupAdcOffsetC), -0.1F, 1e-5F);
  zexpect_within(kNumber(reg::kStartupGainC), 1.25F, 1e-3F);
  zexpect_within(kNumber(reg::kStartupGainVdc), 1.0F, 1e-3F);

  // The drive takes the calibration with the next engage.
  sim::ClearPhaseCurrentOverride();
  rig.Readiness(reg_udral_service_common_Readiness_0_1_STANDBY);
  rig.Run(2U);
  rig.Hold(0.0F, 2.0F, 30U);
  zexpect_true(rig.drive.IsEngaged());

  // From then on every sample is calibrated; the raw sample stays for the startup aid.
  rig.Readiness(reg_udral_service_common_Readiness_0_1_SLEEP);
  rig.Run(2U);
  zassert_false(rig.drive.IsEngaged());
  sim::SetPhaseCurrentOverride(1.0F, -0.5F, -0.5F);
  rig.Run(2U);
  const auto& state = CurrentControlIsr::Instance().state;
  UNIMOC_EXPECT_NEAR(state.raw_phase_currents.a, 1.0_A, 1e-6_A);
  UNIMOC_EXPECT_NEAR(state.phase_currents.a, 1.0_A, 2e-3_A);   // (1.0 - 0.2) * 1.25
  UNIMOC_EXPECT_NEAR(state.phase_currents.b, -0.5_A, 2e-3_A);  // (-0.5 + 0.1) * 1.25
  UNIMOC_EXPECT_NEAR(state.phase_currents.c, -0.5_A, 2e-3_A);
  UNIMOC_EXPECT_NEAR(state.dc_link, state.raw_vdc, 0.05_V);  // the DC-link gain is about 1
  sim::ClearPhaseCurrentOverride();
}

ZTEST(sim_cyphal_drive, test_startup_phase_2_runs_on_the_motor) {
  using namespace startup_aid;
  Rig& rig = *g_rig;
  // Phase 1 without the motor: no current (the plant has it connected all the time).
  sim::SetPhaseCurrentOverride(0.0F, 0.0F, 0.0F);
  zexpect_equal(rig.Command(kStart), cmd::kStatusSuccess);
  for (unsigned i = 0U; i < 6U; ++i) {
    Next(rig);
  }
  zexpect_equal(rig.Command(kReading, "vdc=48"), cmd::kStatusSuccess);
  Next(rig);
  Next(rig);
  zexpect_equal(Step(rig), FsmState::kConnectMotor);

  // The barrier holds the outputs off while the motor is connected; the
  // external enable may be pulled meanwhile. Continue needs the confirmation.
  Measure(rig);
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kDisabled);
  unimoc_drive_CommissioningStatus_0_1 status{};
  zassert_true(rig.Commissioning(status));
  zexpect_equal(status.state, unimoc_drive_CommissioningStatus_0_1_STATE_WAITING);
  zexpect_equal(status.needs, unimoc_drive_CommissioningStatus_0_1_NEEDS_MOTOR_CONNECTED);
  sim::SetExternalEnable(false);
  rig.Run(100U);
  zexpect_equal(Step(rig), FsmState::kConnectMotor, "the barrier waits");
  sim::SetExternalEnable(true);
  zexpect_equal(rig.Command(kContinue), cmd::kStatusBadParameter);
  zexpect_true(rig.last_output == "confirm motor-connected", "%s", rig.last_output.c_str());

  // Phase 2 on the motor of the plant: the alignment sweep and the
  // calibration stay far below the overcurrent limit of the run (6 A).
  sim::ClearPhaseCurrentOverride();
  Next(rig, "motor-connected");
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kEnabled);
  Next(rig);
  zexpect_equal(Step(rig), FsmState::kCurrentSenseCalibration, "the sweep tripped");
  rig.Run(200U);  // the current settles (L/R = 10 ms)
  const Current kPhaseA = sim::Plant().Output().phase_currents.a;
  UNIMOC_EXPECT_NEAR(kPhaseA, 0.96_A, 0.01_A);  // V / R in steady state: 2/3 of 0.3 % of 48 V over 0.1 Ohm
  zexpect_equal(rig.Command(kReading, "current=0.96"), cmd::kStatusSuccess);  // the clamp meter
  Next(rig);

  zexpect_equal(Step(rig), FsmState::kDone);
  auto results = rig.Access(reg::kStartupResults, Empty());
  zexpect_equal(results.natural8.value.elements[11], 1U, "the gains of the plant are 1");
  const auto kGain = [&rig](const char* name) {
    return unimoc::cyphal::GetNumber(rig.Access(name, Empty())).value_or(-1.0F);
  };
  zexpect_within(kGain(reg::kStartupGainA), 1.0F, 0.02F);
  zexpect_within(kGain(reg::kStartupGainB), 1.0F, 0.02F);
  zexpect_within(kGain(reg::kStartupGainC), 1.0F, 0.02F);
}

ZTEST(sim_cyphal_drive, test_a_motor_in_phase_1_faults_the_startup_aid) {
  using namespace startup_aid;
  Rig& rig = *g_rig;
  zexpect_equal(rig.Command(kStart), cmd::kStatusSuccess);
  for (unsigned i = 0U; i < 9U && Step(rig) != FsmState::kFault; ++i) {
    Measure(rig);
    (void)rig.Command(kReading, "vdc=48");
    Measure(rig);
    (void)rig.Command(kContinue);
    rig.Run(2U);
  }
  // GATE_DRIVER_ENABLE_CHECK drives a current into the connected motor: FAULT, outputs off.
  zexpect_equal(Step(rig), FsmState::kFault);
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kDisabled);
  unimoc_drive_CommissioningStatus_0_1 status{};
  zassert_true(rig.Commissioning(status));
  zexpect_equal(status.state, unimoc_drive_CommissioningStatus_0_1_STATE_FAILED);
  zexpect_equal(status.fault, unimoc_drive_CommissioningStatus_0_1_FAULT_OVERCURRENT);
}

ZTEST(sim_cyphal_drive, test_timing_registers_count_the_interrupts_and_reset) {
  Rig& rig = *g_rig;
  rig.Run(2U);
  const auto kStats = [&rig](const char* name) {
    auto value = rig.Access(name, Empty());
    zassert_true(uavcan_register_Value_1_0_is_natural32_(&value));
    zassert_equal(value.natural32.value.count, 4U);
    return value.natural32.value;
  };
  const auto kBefore = kStats(reg::kTimingControlCycle);
  zexpect_true(kBefore.elements[3] > 0U, "records");
  zexpect_true(kBefore.elements[0] <= kBefore.elements[1] && kBefore.elements[1] <= kBefore.elements[2]);
  zexpect_true(kStats(reg::kTimingObserver).elements[3] > 0U);
  zexpect_true(kStats(reg::kTimingSampleIsr).elements[3] > 0U);
  zexpect_equal(Integer(rig.Access(reg::kTimingClockHz, Empty())), 1'000'000'000);

  auto overruns = rig.Access(reg::kTimingOverruns, Empty());
  zassert_true(uavcan_register_Value_1_0_is_natural32_(&overruns));
  zexpect_equal(overruns.natural32.value.count, 2U);
  zexpect_equal(overruns.natural32.value.elements[0], 0U);  // the simulation applies every plan

  // Read only; the reset starts new statistics at the next record.
  zexpect_equal(Integer(rig.Access(reg::kTimingControlCycle, Natural8(1U))), 0);
  zexpect_equal(Integer(rig.Access(reg::kTimingReset, Natural8(2U))), 0);  // only 1
  const uint32_t kBeforeReset = kStats(reg::kTimingControlCycle).elements[3];
  zexpect_equal(Integer(rig.Access(reg::kTimingReset, Natural8(1U))), 0);
  rig.Run(1U);
  const uint32_t kAfter = kStats(reg::kTimingControlCycle).elements[3];
  // Each access advances the simulation by a few cycles.
  zexpect_true(kAfter >= kCyclesPerMs && kAfter < kBeforeReset,
               "records %u, before the reset %u",
               kAfter,
               kBeforeReset);
}

ZTEST(sim_cyphal_drive, test_every_settings_register_is_reachable) {
  Rig& rig = *g_rig;
  // The last registers of the settings (the provider once held fewer than there are).
  const auto kTimeout = unimoc::cyphal::GetNumber(rig.Access(reg::kCommandTimeout, Empty()));
  zassert_true(kTimeout.has_value());
  zexpect_within(*kTimeout, 0.5F, 1e-6F);
  zexpect_equal(Integer(rig.Access(reg::kCartWheel, Empty())), 0);
  const auto kShare = unimoc::cyphal::GetNumber(rig.Access(reg::kCartBatteryShare, Empty()));
  zassert_true(kShare.has_value());
  zexpect_within(*kShare, 1.0F, 1e-6F);
  zexpect_equal(Integer(rig.Access(reg::kCanBusOffCount, Empty())), 0);
}

ZTEST(sim_cyphal_drive, test_wheel_command_drives_the_plant_and_the_drive_status_reports_it) {
  Rig& rig = *g_rig;
  const MagneticFlux kTorqueConstant = GiveTheMotorAMagnet(rig);
  const Torque kTorque = 2.0_A * kTorqueConstant;
  rig.HoldWith([&rig, kTorque] { rig.WheelCommand(kTorque.Value()); }, 30U);  // DSDL payload
  zassert_true(rig.drive.IsEngaged());
  zexpect_within(CurrentControlIsr::Instance().state.i_ref.q, 2.0_A, 1e-3_A);

  // The cart torque replaces the other setpoints until the disengage.
  rig.HoldWith(
      [&rig, kTorque] {
        rig.WheelCommand(kTorque.Value());
        rig.Current(0.0F, 10.0F);
      },
      30U);
  zexpect_within(CurrentControlIsr::Instance().state.i_ref.q, 2.0_A, 1e-3_A);

  unimoc_cart_DriveStatus_0_1 status{};
  zassert_true(rig.DriveStatus(status));
  zexpect_true(status.engaged);
  zexpect_false(status.limp_home);
  zexpect_equal(status.wheel, unimoc_cart_WheelCommand_0_1_WHEEL_FRONT_LEFT);
  zexpect_equal(status.power_stage, unimoc_cart_DriveStatus_0_1_POWER_STAGE_ENABLED);
  zexpect_within(status.estimated_torque,
                 kTorque.Value(),
                 (0.5_A * kTorqueConstant).Value(),
                 "torque %f",
                 static_cast<double>(status.estimated_torque));
  zexpect_within(status.dc_link_voltage, 48.0F, 0.5F);
  zexpect_true(status.drive_torque_limit > 0.0F);
  zexpect_true(status.brake_torque_limit > 0.0F);
  zexpect_true(std::isnan(status.motor_temperature), "the simulation measures no temperature");
}

ZTEST(sim_cyphal_drive, test_limp_home_on_the_pedals_then_stop_without_them) {
  Rig& rig = *g_rig;
  const MagneticFlux kTorqueConstant = GiveTheMotorAMagnet(rig);
  (void)rig.Access(reg::kCartLimpHomeTorque, Real((2.0_A * kTorqueConstant).Value()));
  rig.Run(2U);
  rig.HoldWith(
      [&rig] {
        rig.WheelCommand(0.0F);
        rig.Pedals(0.0F, 0.0F);
      },
      30U);
  zassert_true(rig.drive.IsEngaged());

  // The VDC is gone (no readiness, no wheel command); the pedal unit stays,
  // longer than the readiness timeout.
  for (unsigned elapsed = 0U; elapsed < 700U; elapsed += 10U) {
    rig.Pedals(1.0F, 0.0F);
    rig.Run(10U);
  }
  zassert_true(rig.drive.IsEngaged(), "limp-home keeps the drive engaged without the readiness");
  zexpect_within(CurrentControlIsr::Instance().state.i_ref.q, 2.0_A, 1e-3_A);
  unimoc_cart_DriveStatus_0_1 status{};
  rig.Pedals(1.0F, 0.0F);
  zassert_true(rig.DriveStatus(status));
  zexpect_true(status.limp_home);

  // The pedal unit is gone too: the torque ramps to zero, then the readiness timeout disengages.
  rig.Run(200U);
  zexpect_false(rig.drive.IsEngaged());
  zexpect_within(CurrentControlIsr::Instance().state.i_ref.q, 0.0_A, 1e-3_A);
}

ZTEST(sim_cyphal_drive, test_battery_status_of_the_bms_replaces_the_battery_limits) {
  Rig& rig = *g_rig;
  ConfigureSpeedControl(rig);
  sim::Plant().SetShaftMode(unimoc::sim::ShaftMode::kImposedSpeed, 50.0_rad_per_s);
  // The BMS allows 0.5 A charge: 24 W at 48 V, 0.48 N m against 50 rad/s.
  rig.HoldWith(
      [&rig] {
        rig.WheelCommand(0.0F);
        rig.Battery(30.0F, 0.5F);
      },
      500U);
  zassert_true(rig.drive.IsEngaged());
  unimoc_cart_DriveStatus_0_1 status{};
  rig.Battery(30.0F, 0.5F);
  zassert_true(rig.DriveStatus(status));
  zexpect_within(status.brake_torque_limit,
                 0.48F,
                 0.03F,
                 "brake limit %f",
                 static_cast<double>(status.brake_torque_limit));
  zexpect_true(status.regen_derated);
  zexpect_true(status.drive_torque_limit > 1.0F);

  // Without the BMS the limits of the settings apply again (20 A charge).
  rig.HoldWith([&rig] { rig.WheelCommand(0.0F); }, 200U);
  zassert_true(rig.DriveStatus(status));
  zexpect_true(status.brake_torque_limit > 1.0F,
               "brake limit %f",
               static_cast<double>(status.brake_torque_limit));
  zexpect_false(status.regen_derated);
}

ZTEST(sim_cyphal_drive, test_bus_off_stops_the_cart_torque_and_is_counted) {
  Rig& rig = *g_rig;
  const MagneticFlux kTorqueConstant = GiveTheMotorAMagnet(rig);
  const Torque kTorque = 2.0_A * kTorqueConstant;
  rig.HoldWith([&rig, kTorque] { rig.WheelCommand(kTorque.Value()); }, 30U);
  zassert_true(rig.drive.IsEngaged());

  rig.drive_media.bus_status =
      unimoc::cyphal::CanBusStatus{.state = unimoc::cyphal::CanBusState::kBusOff, .bus_off_count = 1U};
  rig.Run(20U);
  zexpect_within(CurrentControlIsr::Instance().state.i_ref.q, 0.0_A, 1e-3_A, "torque ramped to zero at once");
  rig.Run(600U);
  zexpect_false(rig.drive.IsEngaged(), "the readiness timed out");

  // Back on the bus.
  rig.drive_media.bus_status =
      unimoc::cyphal::CanBusStatus{.state = unimoc::cyphal::CanBusState::kErrorActive, .bus_off_count = 1U};
  rig.Run(5U);
  zexpect_equal(Integer(rig.Access(reg::kCanBusOffCount, Empty())), 1);
}

namespace {

/// The e-bike on the factory motor with a magnet: level 3 gives 2 A of i_q for 20 Nm of pedal torque.
void AttachEbike(Rig& rig, const MagneticFlux torque_constant) {
  unimoc::ebike::EbikeParameters parameters;
  parameters.assist.power_scale = 1.0e9_W;  // no exponential
  parameters.assist.torque_deadband = 0.0_Nm;
  parameters.assist.torque_ratio = (2.0_A * torque_constant) / 20.0_Nm;
  parameters.assist.rise_time = 100.0_ms;
  parameters.pedal.zero_offset = 0.0_Nm;
  parameters.engage_hold = 500.0_ms;
  parameters.rearm_pedal_stop = 500.0_ms;
  rig.ebike.Configure(parameters);
  rig.drive.SetLocalControl(&rig.ebike);
}

constexpr AngularVelocity kCadence = 6.0_rad_per_s;

}  // namespace

ZTEST(sim_cyphal_drive, test_ebike_engages_on_demand_and_the_plant_follows_the_pedals) {
  Rig& rig = *g_rig;
  const MagneticFlux kTorqueConstant = GiveTheMotorAMagnet(rig);
  AttachEbike(rig, kTorqueConstant);

  // A display at level 3 alone does not engage the drive; the pedals do.
  sim::SetDisplay(3U, 25U);
  rig.Run(300U);
  zexpect_false(rig.drive.IsEngaged());
  zexpect_true(sim::DisplayStatusCount() > 0U, "the display is answered");
  zexpect_equal(sim::LastDisplayStatus()[0], 65U);

  sim::SetPedal(kCadence, 20.0_Nm);
  rig.Run(2500U);
  zassert_true(rig.drive.IsEngaged());
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kEnabled);
  zexpect_within(CurrentControlIsr::Instance().state.i_ref.q, 2.0_A, 0.1_A);
  zexpect_within(PlantCurrent().q, 2.0_A, 0.5_A);
  zexpect_equal(sim::LastDisplayStatus()[5], 0U, "no error");

  // The pedals stop: the torque is gone at once, the drive is released after the engage hold.
  sim::SetPedal(AngularVelocity{}, Torque{});
  rig.Run(1500U);
  zexpect_within(CurrentControlIsr::Instance().state.i_ref.q, 0.0_A, 1e-3_A);
  rig.Run(2000U);
  zexpect_false(rig.drive.IsEngaged());
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kDisabled);
}

ZTEST(sim_cyphal_drive, test_ebike_ignores_the_readiness_of_the_bus_and_obeys_standby) {
  Rig& rig = *g_rig;
  const MagneticFlux kTorqueConstant = GiveTheMotorAMagnet(rig);
  AttachEbike(rig, kTorqueConstant);
  rig.Hold(0.0F, 5.0F, 50U);  // ENGAGED on the bus, no pedals
  zexpect_false(rig.drive.IsEngaged());

  sim::SetDisplay(3U, 25U);
  sim::SetPedal(kCadence, 20.0_Nm);
  rig.Run(2500U);
  zassert_true(rig.drive.IsEngaged());
  // A service tool's STANDBY releases the drive, and it stays released while the rider keeps pedaling.
  rig.Readiness(reg_udral_service_common_Readiness_0_1_STANDBY);
  rig.Run(5U);
  zexpect_false(rig.drive.IsEngaged());
  rig.Run(2000U);
  zexpect_false(rig.drive.IsEngaged());
}

ZTEST(sim_cyphal_drive, test_ebike_without_a_display_gives_no_assist) {
  Rig& rig = *g_rig;
  const MagneticFlux kTorqueConstant = GiveTheMotorAMagnet(rig);
  AttachEbike(rig, kTorqueConstant);
  sim::SetPedal(kCadence, 20.0_Nm);
  rig.Run(3000U);
  zexpect_false(rig.drive.IsEngaged());
  zexpect_equal(sim::DisplayStatusCount(), 0U);
}

ZTEST(sim_cyphal_drive, test_ebike_trip_latches_until_the_pedals_stood_still) {
  Rig& rig = *g_rig;
  const MagneticFlux kTorqueConstant = GiveTheMotorAMagnet(rig);
  AttachEbike(rig, kTorqueConstant);
  sim::SetDisplay(3U, 25U);
  sim::SetPedal(kCadence, 20.0_Nm);
  rig.Run(2500U);
  zassert_true(rig.drive.IsEngaged());

  sim::SetPhaseCurrentOverride(80.0F, -40.0F, -40.0F);  // above the 60 A trip limit
  rig.Run(20U);
  sim::ClearPhaseCurrentOverride();
  zexpect_false(rig.drive.IsEngaged());
  // The rider keeps pedaling: the drive stays released, it does not engage in a loop.
  rig.Run(3000U);
  zexpect_false(rig.drive.IsEngaged());

  // The pedals stand still for the re-arm time, then the rider starts again.
  sim::SetPedal(AngularVelocity{}, Torque{});
  rig.Run(1500U);
  sim::SetPedal(kCadence, 20.0_Nm);
  rig.Run(2500U);
  zexpect_true(rig.drive.IsEngaged());
  zexpect_false(CurrentControlIsr::Instance().state.overcurrent_tripped);
}

ZTEST_SUITE(sim_cyphal_drive, nullptr, nullptr, Before, After, nullptr);
