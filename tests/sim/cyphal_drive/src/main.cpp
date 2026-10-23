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
#include <unimoc/drive/CurrentSetpoint_0_1.h>
#include <unimoc/drive/Telemetry_0_1.h>
#include <zephyr/ztest.h>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <new>
#include <numbers>
#include <string_view>
#include "angle_wrap.hpp"
#include "can_test_bus.hpp"
#include "current_control_isr.hpp"
#include "current_control_runtime.hpp"
#include "cyphal/canard_transport.hpp"
#include "cyphal/node.hpp"
#include "drive_node.hpp"
#include "hardware_interface.hpp"
#include "hot_path_interface.hpp"
#include "outer_loop.hpp"
#include "power_stage_interface.hpp"
#include "settings_registers.hpp"
#include "settings_save_gate.hpp"
#include "settings_store.hpp"
#include "sim_control.hpp"
#include "sin_cos.hpp"
#include "slow_update.hpp"
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
constexpr uint8_t kClientId = 100U;
constexpr unsigned kCyclesPerMs = 16U;  // 32 kHz PWM, 16 kHz control cycle

const uint8_t* In(std::span<const std::byte> buffer) {
  return reinterpret_cast<const uint8_t*>(buffer.data());
}
uint8_t* Out(std::span<std::byte> buffer) { return reinterpret_cast<uint8_t*>(buffer.data()); }

SettingsStore MakeStore() {
  return SettingsStore{unimoc::hardware::kSettings.GetSettingsProfile(),
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
    zassert_true(client.Subscribe(TransferKind::kServiceResponse, port::kRegisterAccess, 267U));
    zassert_true(client.Subscribe(TransferKind::kServiceResponse, port::kExecuteCommand, 48U));
    zassert_true(drive.Start(now));
  }

  /// Advances the drive, the plant and the client by @p milliseconds.
  void Run(const unsigned milliseconds) {
    for (unsigned i = 0U; i < milliseconds; ++i) {
      now += 1000U;
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
    Publish(unimoc::cyphal::kSubReadiness, std::span<const std::byte>{tx.data(), size});
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

  uint8_t Command(const uint16_t command) {
    uavcan_node_ExecuteCommand_Request_1_3 request{};
    request.command = command;
    std::size_t size = tx.size();
    zassert_equal(uavcan_node_ExecuteCommand_Request_1_3_serialize_(&request, Out(tx), &size), 0);
    Transfer response{};
    zassert_true(Request(port::kExecuteCommand, std::span<const std::byte>{tx.data(), size}, response));
    uavcan_node_ExecuteCommand_Response_1_3 result{};
    size = response.payload.size();
    zassert_equal(uavcan_node_ExecuteCommand_Response_1_3_deserialize_(&result, In(response.payload), &size),
                  0);
    return result.status;
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
  std::array<std::byte, 600> rx{};
  std::array<std::byte, 600> tx{};
  uint64_t now{1'000'000U};
  uint8_t next_transfer_id{0U};
};

alignas(Rig) std::array<std::byte, sizeof(Rig)> g_storage;
Rig* g_rig = nullptr;

void Before(void* /*unused*/) {
  kPowerStage.Disable();
  // The factory motor (no permanent magnet) unless a test configures another.
  zassert_equal(sim::Plant().Configure(unimoc::hardware::kSettings.GetSettingsProfile().factory_settings),
                unimoc::sim::Status::kOk);
  sim::Plant().SetInput(unimoc::sim::SimulationInput{});
  sim::Plant().SetDcLinkVoltage(Voltage{48.0F});
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

float PlantSpeed() { return sim::Plant().Output().mechanical_speed.Value(); }
float PlantAngle() { return sim::Plant().Output().mechanical_angle.AbsoluteAngle().Value(); }

/// Pole pairs of ConfigureSpeedControl().
constexpr float kSpeedControlPolePairs = 7.0F;

/// True shaft angle of the drive's home (position zero). The drive counts the
/// position from the estimate at its start, which the Hall sector places only
/// within half a sector (30 degrees electrical) of the shaft; the edges
/// correct the angle estimate later, but not the home. Valid at standstill.
float TrueHome() {
  const float kEstimateError =
      unimoc::observer::WrapAngle(sim::Plant().Output().electrical_angle.AngleInRevolution() -
                                  CurrentControlIsr::Instance().mech_obs.theta)
          .Value();
  return PlantAngle() - OuterLoop::Instance().Status().position.Value() -
         (kEstimateError / kSpeedControlPolePairs);
}

}  // namespace

ZTEST(sim_cyphal_drive, test_engage_and_current_setpoint_reach_the_plant) {
  Rig& rig = *g_rig;
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kDisabled);
  rig.Hold(0.0F, 10.0F, 30U);
  zassert_true(rig.drive.IsEngaged());
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kEnabled);
  zexpect_within(CurrentControlIsr::Instance().state.i_ref.q.Value(), 10.0F, 1e-3F);
  zexpect_within(PlantCurrent().q.Value(),
                 10.0F,
                 0.5F,
                 "i_q = %f",
                 static_cast<double>(PlantCurrent().q.Value()));

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
  zexpect_equal(CurrentControlIsr::Instance().state.i_ref.q.Value(), 0.0F);

  // No readiness for longer than the command timeout (0.5 s): disengaged.
  rig.Run(600U);
  zexpect_false(rig.drive.IsEngaged());
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kDisabled);
}

ZTEST(sim_cyphal_drive, test_sleep_disengages_and_setpoints_are_limited) {
  Rig& rig = *g_rig;
  rig.Hold(30.0F, 40.0F, 20U);  // |i| = 50 A, at the factory limit of 50 A
  zexpect_within(CurrentControlIsr::Instance().state.i_ref.d.Value(), 30.0F, 1e-3F);
  rig.Hold(300.0F, 400.0F, 10U);  // scaled to 50 A, direction kept
  zexpect_within(CurrentControlIsr::Instance().state.i_ref.d.Value(), 30.0F, 1e-3F);
  zexpect_within(CurrentControlIsr::Instance().state.i_ref.q.Value(), 40.0F, 1e-3F);

  rig.Readiness(reg_udral_service_common_Readiness_0_1_SLEEP);
  rig.Run(5U);
  zexpect_false(rig.drive.IsEngaged());
  zexpect_equal(CurrentControlIsr::Instance().state.i_ref.q.Value(), 0.0F);
}

ZTEST(sim_cyphal_drive, test_torque_setpoint_uses_the_torque_constant) {
  Rig& rig = *g_rig;
  (void)rig.Access(reg::kPmsmFluxPm, Real(5e-3F));  // the factory profile has no flux
  const auto kSnapshot = rig.store.GetSnapshot();
  const float kTorqueConstant =
      1.5F * static_cast<float>(kSnapshot.Get().pole_pairs) * kSnapshot.Get().flux_pm.Value();
  zassert_true(kTorqueConstant > 0.0F);
  uavcan_si_unit_torque_Scalar_1_0 torque{2.0F * kTorqueConstant};
  for (unsigned elapsed = 0U; elapsed < 20U; elapsed += 10U) {
    rig.Readiness(reg_udral_service_common_Readiness_0_1_ENGAGED);
    std::size_t size = rig.tx.size();
    zassert_equal(uavcan_si_unit_torque_Scalar_1_0_serialize_(&torque, Out(rig.tx), &size), 0);
    rig.Publish(unimoc::cyphal::kSubTorqueSetpoint, std::span<const std::byte>{rig.tx.data(), size});
    rig.Run(10U);
  }
  zexpect_within(CurrentControlIsr::Instance().state.i_ref.q.Value(), 2.0F, 1e-3F);
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
  zexpect_within(CurrentControlIsr::Instance().state.i_ref.q.Value(), 20.0F, 1e-3F);

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
  const float kKp = isr.cc.kp_q.Value();
  (void)rig.Access(reg::kCurrentKpQ, Real(kKp * 2.0F));
  rig.Run(2U);
  zexpect_within(isr.cc.kp_q.Value(), kKp * 2.0F, 1e-6F);
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
  zexpect_equal(rig.store.GetSnapshot().Get().pole_pairs,
                unimoc::hardware::kSettings.GetSettingsProfile().factory_settings.pole_pairs);
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
  zexpect_within(PlantSpeed(), 30.0F, 1.0F, "speed %f", static_cast<double>(PlantSpeed()));
  const auto kStatus = OuterLoop::Instance().Status();
  zexpect_equal(kStatus.mode, unimoc::cyphal::ControlMode::kSpeed);
  zexpect_within(kStatus.speed.Value(), 30.0F, 1.0F);

  unimoc_drive_Telemetry_0_1 telemetry{};
  zassert_true(rig.Telemetry(telemetry));
  zexpect_within(telemetry.electrical_speed, 7.0F * 30.0F, 7.0F);

  // Without new setpoints the drive coasts (zero current), still engaged.
  for (unsigned elapsed = 0U; elapsed < 700U; elapsed += 100U) {
    rig.Readiness(reg_udral_service_common_Readiness_0_1_ENGAGED);
    rig.Run(100U);
  }
  zexpect_true(rig.drive.IsEngaged());
  zexpect_equal(CurrentControlIsr::Instance().state.i_ref.q.Value(), 0.0F);
  zexpect_equal(OuterLoop::Instance().Status().mode, unimoc::cyphal::ControlMode::kTorque);
}

ZTEST(sim_cyphal_drive, test_position_setpoint_moves_the_shaft) {
  Rig& rig = *g_rig;
  ConfigureSpeedControl(rig);
  const float kStart = PlantAngle();
  rig.HoldWith([&] { rig.Position(3.0F); }, 1500U);
  const float kHome = TrueHome();
  constexpr float kHalfSector = std::numbers::pi_v<float> / 6.0F / kSpeedControlPolePairs;
  zexpect_within(kHome,
                 kStart,
                 kHalfSector + 0.005F,
                 "home %f rad from the start",
                 static_cast<double>(kHome - kStart));
  zexpect_within(PlantAngle() - kHome,
                 3.0F,
                 0.05F,
                 "moved %f rad",
                 static_cast<double>(PlantAngle() - kHome));
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
  zexpect_within(PlantSpeed(), -20.0F, 1.0F, "speed %f", static_cast<double>(PlantSpeed()));

  // Position wins over velocity.
  rig.HoldWith([&] { rig.Servo(0.0F, 5.0F, kNan); }, 100U);
  zexpect_equal(OuterLoop::Instance().Status().mode, unimoc::cyphal::ControlMode::kPosition);

  // Torque only: torque mode through the torque constant (3/2 * 7 * 5 mWb).
  rig.HoldWith([&] { rig.Servo(kNan, kNan, 0.0525F); }, 50U);
  zexpect_equal(OuterLoop::Instance().Status().mode, unimoc::cyphal::ControlMode::kTorque);
  zexpect_within(CurrentControlIsr::Instance().state.i_ref.q.Value(), 1.0F, 1e-3F);

  // Nothing finite: ignored, the last setpoint stays.
  rig.HoldWith([&] { rig.Servo(kNan, kNan, kNan); }, 50U);
  zexpect_within(CurrentControlIsr::Instance().state.i_ref.q.Value(), 1.0F, 1e-3F);
}

ZTEST(sim_cyphal_drive, test_regenerative_braking_is_limited_by_the_charge_current_and_the_dc_link) {
  Rig& rig = *g_rig;
  ConfigureSpeedControl(rig);  // torque constant 3/2 * 7 * 5 mWb = 0.0525 N m/A
  (void)rig.Access(reg::kBatteryChargeIMax, Real(0.5F));
  zassert_equal(rig.store.GetSnapshot().Get().battery_charge_current_max.Value(), 0.5F);
  rig.Run(2U);  // applied while disengaged
  // A dyno: the shaft turns at 50 rad/s whatever the torque.
  sim::Plant().SetShaftMode(unimoc::sim::ShaftMode::kImposedSpeed, AngularVelocity{50.0F});
  constexpr float kTorqueConstant = 1.5F * 7.0F * 5e-3F;

  // Motoring: 20 A make 52.5 W, far below the drive limit.
  rig.Hold(0.0F, 20.0F, 500U);
  zassert_true(rig.drive.IsEngaged());
  zexpect_within(OuterLoop::Instance().Status().current_reference.q.Value(), 20.0F, 1e-3F);

  // Braking: at most 0.5 A * 48 V = 24 W into the battery. The observer
  // expects the braking torque to slow the shaft, which the dyno does not:
  // its speed estimate (and with it the limit) settles within some 300 ms.
  rig.Hold(0.0F, -20.0F, 500U);
  auto status = OuterLoop::Instance().Status();
  zexpect_within(status.speed.Value(),
                 50.0F,
                 1.0F,
                 "estimated speed %f",
                 static_cast<double>(status.speed.Value()));
  zexpect_within(status.current_reference.q.Value(),
                 -24.0F / (kTorqueConstant * status.speed.Value()),
                 1e-3F,
                 "i_q %f",
                 static_cast<double>(status.current_reference.q.Value()));
  const float kChargePower = -kTorqueConstant * PlantCurrent().q.Value() * PlantSpeed();
  zexpect_within(kChargePower,
                 24.0F,
                 2.0F,
                 "mechanical braking power %f W",
                 static_cast<double>(kChargePower));

  // At 56.5 V (derating from 55 V to the 58 V trip) half of the 0.5 A remains.
  sim::Plant().SetDcLinkVoltage(Voltage{56.5F});
  rig.Hold(0.0F, -20.0F, 500U);
  status = OuterLoop::Instance().Status();
  zexpect_within(status.current_reference.q.Value(),
                 -(0.25F * 56.5F) / (kTorqueConstant * status.speed.Value()),
                 1e-3F,
                 "i_q %f",
                 static_cast<double>(status.current_reference.q.Value()));
  zexpect_true(rig.drive.IsEngaged());

  // From the trip level the control cycle trips the power stage and the drive disengages.
  sim::Plant().SetDcLinkVoltage(Voltage{58.5F});
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
  zexpect_within(CurrentControlIsr::Instance().cc.kp_q.Value(), 0.7F, 1e-6F);
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
  zexpect_within(CurrentControlIsr::Instance().cc.kp_q.Value(), 0.7F, 1e-6F);
}

ZTEST(sim_cyphal_drive, test_speed_setpoint_is_limited_to_the_motor_speed) {
  Rig& rig = *g_rig;
  ConfigureSpeedControl(rig);
  (void)rig.Access(reg::kMotorOmegaMax, Real(7.0F * 20.0F));  // electrical: 20 rad/s mechanical
  rig.Run(2U);
  rig.HoldWith([&] { rig.Speed(50.0F); }, 600U);
  zexpect_within(PlantSpeed(), 20.0F, 1.0F, "speed %f", static_cast<double>(PlantSpeed()));
}

ZTEST(sim_cyphal_drive, test_startup_registers_run_step_and_abort) {
  Rig& rig = *g_rig;
  zexpect_equal(Integer(rig.Access(reg::kStartupStep, Empty())), 0);  // IDLE
  zexpect_equal(Integer(rig.Access(reg::kStartupRun, Natural8(1U))), 1);
  zexpect_equal(Integer(rig.Access(reg::kStartupStep, Empty())), 1);  // PWM_DISABLE
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kEnabled);

  // The FSM owns the outputs: no engage, no settings restart, no factory reset.
  rig.Hold(0.0F, 5.0F, 30U);
  zexpect_false(rig.drive.IsEngaged());
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kEnabled);
  zexpect_equal(rig.Command(cmd::kCommandFactoryReset), cmd::kStatusBadState);
  rig.Readiness(reg_udral_service_common_Readiness_0_1_SLEEP);  // clears the re-arm
  rig.Run(2U);

  // Step through phase 1 up to the DC-link check: one sample per millisecond.
  // A step write takes effect at the next FSM run, after the response.
  zexpect_equal(Integer(rig.Access(reg::kStartupStep, Natural8(1U))), 1);
  rig.Run(2U);
  zexpect_equal(Integer(rig.Access(reg::kStartupStep, Empty())), 2);  // ADC_OFFSET_CAL
  for (unsigned i = 0U; i < 5U; ++i) {                                // ADC_OFFSET_CAL .. DUTY_FORCE_HIGH
    rig.Run(1100U);                                                   // 1024 or 1000 samples per step
    (void)rig.Access(reg::kStartupStep, Natural8(1U));
  }
  rig.Run(2U);
  zexpect_equal(Integer(rig.Access(reg::kStartupStep, Empty())), 7);  // DC_LINK_VOLTAGE_CHECK

  auto results = rig.Access(reg::kStartupResults, Empty());
  zassert_true(uavcan_register_Value_1_0_is_natural8_(&results));
  zexpect_equal(results.natural8.value.count, 14U);
  zexpect_equal(results.natural8.value.elements[1], 1U);  // PWM_DISABLE passed
  zexpect_equal(results.natural8.value.elements[2], 1U);  // ADC_OFFSET_CAL passed (no offset in the sim)

  zexpect_within(unimoc::cyphal::GetNumber(rig.Access(reg::kStartupExtVdcV, Real(48.0F))).value_or(0.0F),
                 48.0F,
                 1e-6F);
  zexpect_equal(Integer(rig.Access(reg::kStartupStep, Natural8(2U))), 7);  // only 1 advances

  // Abort: FAULT, outputs off; a new run starts again from the beginning.
  zexpect_equal(Integer(rig.Access(reg::kStartupRun, Natural8(0U))), 0);
  zexpect_equal(Integer(rig.Access(reg::kStartupStep, Empty())), 13);  // FAULT
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kDisabled);
  zexpect_equal(Integer(rig.Access(reg::kStartupRun, Natural8(1U))), 1);
  zexpect_equal(Integer(rig.Access(reg::kStartupStep, Empty())), 1);
  zexpect_equal(Integer(rig.Access(reg::kStartupRun, Natural8(0U))), 0);

  // After the run the drive engages again.
  rig.Readiness(reg_udral_service_common_Readiness_0_1_STANDBY);
  rig.Run(2U);
  rig.Hold(0.0F, 5.0F, 30U);
  zexpect_true(rig.drive.IsEngaged());
}

ZTEST(sim_cyphal_drive, test_startup_run_is_refused_while_engaged) {
  Rig& rig = *g_rig;
  rig.Hold(0.0F, 2.0F, 20U);
  zassert_true(rig.drive.IsEngaged());
  zexpect_equal(Integer(rig.Access(reg::kStartupRun, Natural8(1U))), 0);
  zexpect_equal(Integer(rig.Access(reg::kStartupStep, Empty())), 0);  // still IDLE
  zexpect_true(rig.drive.IsEngaged());
}

ZTEST(sim_cyphal_drive, test_emergency_stop_aborts_a_startup_run) {
  Rig& rig = *g_rig;
  zexpect_equal(Integer(rig.Access(reg::kStartupRun, Natural8(1U))), 1);
  zexpect_equal(rig.Command(cmd::kCommandEmergencyStop), cmd::kStatusSuccess);
  zexpect_equal(Integer(rig.Access(reg::kStartupStep, Empty())), 13);  // FAULT
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kDisabled);
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

ZTEST_SUITE(sim_cyphal_drive, nullptr, nullptr, Before, After, nullptr);
