/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file main.cpp
 *    @brief The Cyphal application of the drive against the simulated plant:
 *           arming, setpoints, limits, timeouts, telemetry, registers and
 *           node commands, driven by a second libcanard node.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <reg/udral/service/common/Readiness_0_1.h>
#include <uavcan/_register/Access_1_0.h>
#include <uavcan/node/ExecuteCommand_1_3.h>
#include <uavcan/si/unit/torque/Scalar_1_0.h>
#include <unimoc/drive/CurrentSetpoint_0_1.h>
#include <unimoc/drive/Telemetry_0_1.h>
#include <zephyr/ztest.h>
#include <array>
#include <cstring>
#include <new>
#include <string_view>
#include "can_test_bus.hpp"
#include "current_control_isr.hpp"
#include "current_control_runtime.hpp"
#include "cyphal/canard_transport.hpp"
#include "cyphal/node.hpp"
#include "drive_node.hpp"
#include "hardware_interface.hpp"
#include "hot_path_interface.hpp"
#include "power_stage_interface.hpp"
#include "settings_registers.hpp"
#include "settings_save_gate.hpp"
#include "settings_store.hpp"
#include "sim_control.hpp"
#include "sin_cos.hpp"
#include "slow_update.hpp"
#include "zephyr_settings_storage.hpp"

// The platform calls these directly (hot_path_interface.hpp).
void unimoc::hardware::ControlCycleHandler() noexcept {
  unimoc::current_control::CurrentControlIsr::Instance().OnControlCycle();
}
void unimoc::hardware::ObserverHandler() noexcept {
  unimoc::current_control::SlowUpdate::Instance().RunOnce();
}

namespace {

using namespace unimoc::unit;
using unimoc::current_control::CurrentControlIsr;
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

/// The drive (store, node, DriveNode) and a client on one CAN FD bus.
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
              unimoc::drive::DriveHooks{CountRestart, nullptr},
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

  /// Engaged, current setpoint refreshed every 10 ms for @p milliseconds.
  void Hold(const float i_d, const float i_q, const unsigned milliseconds) {
    for (unsigned elapsed = 0U; elapsed < milliseconds; elapsed += 10U) {
      Readiness(reg_udral_service_common_Readiness_0_1_ENGAGED);
      Current(i_d, i_q);
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
  sim::SetFreeRunning(false);
  sim::SetExternalEnable(true);
  sim::ClearPhaseCurrentOverride();
  kPowerStage.ClearFault();
  CurrentControlIsr::Instance().ClearOvercurrentTrip();
  EraseAllRegisters();
  // Locked rotor: the current loop works on a standing plant.
  sim::Plant().SetShaftMode(unimoc::sim::ShaftMode::kImposedSpeed, AngularVelocity{});
  g_restarts = 0;
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

ZTEST_SUITE(sim_cyphal_drive, nullptr, nullptr, Before, After, nullptr);
