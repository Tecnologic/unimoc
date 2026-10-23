/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file settings_store_test.cpp
 *    @brief Tests for settings storage, validation, and mutation boundaries.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "settings_store.hpp"
#include <zephyr/ztest.h>
#include <algorithm>
#include <array>
#include <cstddef>
#include <string_view>
#include <type_traits>
#include <utility>
#include "ram_settings_storage.hpp"
#include "unimoc_test.hpp"

namespace unimoc::settings::test {

using namespace unimoc::unit;
using unimoc::test::RamSettingsStorage;
namespace reg = unimoc::cyphal::reg;

namespace {

SettingsProfile MakeProfile() {
  SettingsProfile profile{};
  profile.factory_settings.motor_i_max = 30.0_A;
  profile.factory_settings.battery_drive_current_max = 12.0_A;
  profile.factory_settings.battery_charge_current_max = 4.0_A;
  profile.capabilities.max_phase_current = 50.0_A;
  profile.capabilities.max_motor_current = 30.0_A;
  profile.capabilities.max_battery_drive_current = 12.0_A;
  profile.capabilities.max_battery_charge_current = 4.0_A;
  return profile;
}

bool g_save_allowed = true;

SettingsSaveGate MakeGate() {
  return SettingsSaveGate{nullptr, [](void* /*unused*/) { return g_save_allowed; }};
}

struct Fixture {
  RamSettingsStorage backend{};
  SettingsStore store{MakeProfile(), backend.Adapter(), MakeGate()};

  Fixture() { g_save_allowed = true; }
};

template <typename T>
std::array<std::byte, kMaxRegisterValueSize> Bytes(const T& value, std::size_t& size) {
  std::array<std::byte, kMaxRegisterValueSize> bytes{};
  size = EncodeRegisterValue(value, bytes);
  return bytes;
}

}  // namespace

ZTEST(settings_store, test_snapshot_exposes_only_const_settings) {
  static_assert(std::is_same_v<decltype(std::declval<const SettingsSnapshot&>().Get()), const NvmSettings&>);
}

ZTEST(settings_store, test_empty_storage_uses_factory_defaults_and_writes_nothing) {
  Fixture ctx;
  zexpect_equal(ctx.store.Load(), SettingsStatus::kFactoryDefaults);
  zexpect_equal(ctx.store.GetSnapshot().Get().motor_i_max, 30.0_A);
  zexpect_equal(ctx.backend.Writes(), 0U);
  zexpect_equal(ctx.backend.Erases(), 0U);
  zexpect_equal(ctx.backend.Count(), 0U);
}

ZTEST(settings_store, test_live_update_changes_ram_only) {
  Fixture ctx;
  zassert_equal(ctx.store.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = ctx.store.GetOperations();

  zexpect_equal(operations.SetMotorCurrentLimit(20.0_A), SettingsStatus::kSuccess);
  zexpect_equal(ctx.store.GetSnapshot().Get().motor_i_max, 20.0_A);
  zexpect_equal(operations.UnsavedCount(), 1U);
  zexpect_equal(ctx.backend.Writes(), 0U);
  zexpect_equal(ctx.backend.Count(), 0U);
}

ZTEST(settings_store, test_save_writes_only_changed_registers) {
  Fixture ctx;
  zassert_equal(ctx.store.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = ctx.store.GetOperations();
  zassert_equal(operations.SetMotorCurrentLimit(20.0_A), SettingsStatus::kSuccess);
  zassert_equal(operations.SetNodeId(7U), SettingsStatus::kSuccess);

  zexpect_equal(operations.SaveToFlash(), SettingsStatus::kSuccess);
  zexpect_equal(ctx.backend.Writes(), 2U);
  zexpect_true(ctx.backend.Contains(reg::kMotorIMax));
  zexpect_true(ctx.backend.Contains(reg::kNodeId));
  zexpect_equal(ctx.backend.Count(), 2U);
  zexpect_equal(operations.UnsavedCount(), 0U);

  // Nothing changed since: nothing is written.
  zexpect_equal(operations.SaveToFlash(), SettingsStatus::kSuccess);
  zexpect_equal(ctx.backend.Writes(), 2U);

  // One more change: exactly one more write.
  zassert_equal(operations.SetMotorCurrentLimit(25.0_A), SettingsStatus::kSuccess);
  zexpect_equal(operations.SaveToFlash(), SettingsStatus::kSuccess);
  zexpect_equal(ctx.backend.Writes(), 3U);
}

ZTEST(settings_store, test_saved_registers_load_over_the_factory_profile) {
  Fixture ctx;
  zassert_equal(ctx.store.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = ctx.store.GetOperations();
  zassert_equal(operations.Update([](NvmSettings& settings) {
    settings.motor_i_max = 20.0_A;
    settings.stator_r = 0.23_Ohm;
    settings.pwm_frequency = 32.0_kHz;
    settings.motor_type = system::MotorType::kAsm;
    settings.pole_pairs = 7U;
  }),
                SettingsStatus::kSuccess);
  zassert_equal(operations.SaveToFlash(), SettingsStatus::kSuccess);

  SettingsStore reloaded{MakeProfile(), ctx.backend.Adapter(), MakeGate()};
  zexpect_equal(reloaded.Load(), SettingsStatus::kSuccess);
  const auto kSnapshot = reloaded.GetSnapshot();
  const NvmSettings& loaded = kSnapshot.Get();
  zexpect_equal(loaded.motor_i_max, 20.0_A);
  zexpect_equal(loaded.stator_r, 0.23_Ohm);
  zexpect_equal(loaded.pwm_frequency, 32000.0_Hz);
  zexpect_equal(loaded.motor_type, system::MotorType::kAsm);
  zexpect_equal(loaded.pole_pairs, 7U);
  zexpect_equal(loaded.battery_drive_current_max, 12.0_A);  // not stored: factory
  zexpect_equal(reloaded.GetOperations().UnsavedCount(), 0U);
}

ZTEST(settings_store, test_register_back_at_factory_value_is_erased) {
  Fixture ctx;
  zassert_equal(ctx.store.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = ctx.store.GetOperations();
  zassert_equal(operations.SetMotorCurrentLimit(20.0_A), SettingsStatus::kSuccess);
  zassert_equal(operations.SaveToFlash(), SettingsStatus::kSuccess);
  zassert_true(ctx.backend.Contains(reg::kMotorIMax));

  zassert_equal(operations.SetMotorCurrentLimit(30.0_A), SettingsStatus::kSuccess);
  zexpect_equal(operations.SaveToFlash(), SettingsStatus::kSuccess);
  zexpect_false(ctx.backend.Contains(reg::kMotorIMax));
  zexpect_equal(ctx.backend.Erases(), 1U);
}

ZTEST(settings_store, test_reset_to_factory_is_live_and_erases_on_save) {
  Fixture ctx;
  zassert_equal(ctx.store.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = ctx.store.GetOperations();
  zassert_equal(operations.SetMotorCurrentLimit(20.0_A), SettingsStatus::kSuccess);
  zassert_equal(operations.SaveToFlash(), SettingsStatus::kSuccess);

  zexpect_equal(operations.ResetToFactoryDefaults(), SettingsStatus::kSuccess);
  zexpect_equal(ctx.store.GetSnapshot().Get().motor_i_max, 30.0_A);
  zexpect_true(ctx.backend.Contains(reg::kMotorIMax));  // flash is untouched until saved
  zexpect_equal(operations.UnsavedCount(), 1U);

  zexpect_equal(operations.SaveToFlash(), SettingsStatus::kSuccess);
  zexpect_equal(ctx.backend.Count(), 0U);
}

ZTEST(settings_store, test_save_is_refused_while_the_gate_is_closed) {
  Fixture ctx;
  zassert_equal(ctx.store.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = ctx.store.GetOperations();
  zassert_equal(operations.SetMotorCurrentLimit(20.0_A), SettingsStatus::kSuccess);

  g_save_allowed = false;
  zexpect_equal(operations.SaveToFlash(), SettingsStatus::kPersistBlocked);
  zexpect_equal(ctx.backend.Writes(), 0U);
  zexpect_equal(operations.UnsavedCount(), 1U);
  zexpect_equal(ctx.store.GetSnapshot().Get().motor_i_max, 20.0_A);  // live tuning still works

  g_save_allowed = true;
  zexpect_equal(operations.SaveToFlash(), SettingsStatus::kSuccess);
  zexpect_equal(ctx.backend.Writes(), 1U);
}

ZTEST(settings_store, test_default_gate_never_allows_saving) {
  RamSettingsStorage backend{};
  SettingsStore store{MakeProfile(), backend.Adapter(), SettingsSaveGate{}};
  zassert_equal(store.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = store.GetOperations();
  zassert_equal(operations.SetNodeId(3U), SettingsStatus::kSuccess);
  zexpect_equal(operations.SaveToFlash(), SettingsStatus::kPersistBlocked);
  zexpect_equal(backend.Writes(), 0U);
}

ZTEST(settings_store, test_capability_violation_leaves_settings_unchanged) {
  Fixture ctx;
  zassert_equal(ctx.store.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = ctx.store.GetOperations();

  zexpect_equal(operations.SetMotorCurrentLimit(31.0_A), SettingsStatus::kOutOfRange);
  zexpect_equal(ctx.store.GetSnapshot().Get().motor_i_max, 30.0_A);
  zexpect_equal(operations.UnsavedCount(), 0U);
}

ZTEST(settings_store, test_dc_link_voltages_are_ordered_and_within_the_capability) {
  Fixture ctx;
  zassert_equal(ctx.store.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = ctx.store.GetOperations();
  const auto kSet = [&operations](const Voltage derate_start, const Voltage trip) {
    return operations.Update([derate_start, trip](NvmSettings& settings) {
      settings.dc_link_regen_derate_start = derate_start;
      settings.dc_link_trip = trip;
    });
  };
  zexpect_equal(kSet(50.0_V, 56.0_V), SettingsStatus::kSuccess);
  zexpect_equal(kSet(56.0_V, 56.0_V), SettingsStatus::kOutOfRange);  // no derating range
  zexpect_equal(kSet(50.0_V, 61.0_V), SettingsStatus::kOutOfRange);  // above the 60 V capability
  zexpect_equal(kSet(0.0_V, 56.0_V), SettingsStatus::kOutOfRange);
  zexpect_equal(ctx.store.GetSnapshot().Get().dc_link_trip, 56.0_V);
}

ZTEST(settings_store, test_two_ports_on_one_port_id_are_rejected) {
  Fixture ctx;
  zassert_equal(ctx.store.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = ctx.store.GetOperations();
  const auto kSet = [&operations](uint16_t NvmSettings::* port, const uint16_t port_id) {
    return operations.Update([port, port_id](NvmSettings& settings) { settings.*port = port_id; });
  };
  const NvmSettings& factory = MakeProfile().factory_settings;

  // Two subscriptions, or a publication on a subscribed subject.
  zexpect_equal(kSet(&NvmSettings::sub_speed_setpoint_id, factory.sub_torque_setpoint_id),
                SettingsStatus::kInvalidSettings);
  zexpect_equal(kSet(&NvmSettings::pub_telemetry_id, factory.sub_readiness_id),
                SettingsStatus::kInvalidSettings);
  zexpect_equal(kSet(&NvmSettings::sub_cymon_trigger_id, factory.sub_current_setpoint_id),
                SettingsStatus::kInvalidSettings);
  zexpect_equal(kSet(&NvmSettings::pub_cymon_trigger_id, factory.pub_telemetry_id),
                SettingsStatus::kInvalidSettings);
  // Two services on one service-ID.
  zexpect_equal(kSet(&NvmSettings::srv_cymon_read_id, factory.srv_cymon_arm_id),
                SettingsStatus::kInvalidSettings);
  zexpect_equal(operations.UnsavedCount(), 0U);

  // A subject-ID may equal a service-ID, and disabled ports never collide.
  zexpect_equal(kSet(&NvmSettings::srv_cymon_setup_id, factory.sub_torque_setpoint_id),
                SettingsStatus::kSuccess);
  zexpect_equal(kSet(&NvmSettings::sub_speed_setpoint_id, cyphal::kUnsetPortId), SettingsStatus::kSuccess);
  zexpect_equal(kSet(&NvmSettings::sub_position_setpoint_id, cyphal::kUnsetPortId), SettingsStatus::kSuccess);
  // The cymon trigger may publish and subscribe one shared subject.
  zexpect_equal(kSet(&NvmSettings::pub_cymon_trigger_id, 300U), SettingsStatus::kSuccess);
  zexpect_equal(kSet(&NvmSettings::sub_cymon_trigger_id, 300U), SettingsStatus::kSuccess);
}

ZTEST(settings_store, test_stored_port_id_collision_falls_back_to_factory) {
  Fixture ctx;
  std::size_t size = 0U;
  const auto kBytes = Bytes(uint16_t{cyphal::kSubTorqueSetpoint}, size);
  ctx.backend.Put(reg::kSubSpeedSetpointId, std::span<const std::byte>{kBytes.data(), size});

  zexpect_equal(ctx.store.Load(), SettingsStatus::kInvalidSettings);
  zexpect_equal(ctx.store.GetSnapshot().Get().sub_speed_setpoint_id, cyphal::kSubSpeedSetpoint);
}

ZTEST(settings_store, test_write_failure_keeps_the_register_unsaved_and_a_retry_saves_it) {
  Fixture ctx;
  zassert_equal(ctx.store.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = ctx.store.GetOperations();
  zassert_equal(operations.SetMotorCurrentLimit(20.0_A), SettingsStatus::kSuccess);

  ctx.backend.FailWrites(true);
  zexpect_equal(operations.SaveToFlash(), SettingsStatus::kStorageError);
  zexpect_equal(operations.UnsavedCount(), 1U);
  zexpect_equal(ctx.store.GetSnapshot().Get().motor_i_max, 20.0_A);

  ctx.backend.FailWrites(false);
  zexpect_equal(operations.SaveToFlash(), SettingsStatus::kSuccess);
  zexpect_equal(operations.UnsavedCount(), 0U);
}

ZTEST(settings_store, test_partial_save_keeps_what_was_written) {
  Fixture ctx;
  zassert_equal(ctx.store.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = ctx.store.GetOperations();
  zassert_equal(operations.Update([](NvmSettings& settings) {
    settings.node_id = 9U;
    settings.pole_pairs = 5U;
    settings.motor_i_max = 20.0_A;
  }),
                SettingsStatus::kSuccess);

  ctx.backend.FailWritesAfter(1);
  zexpect_equal(operations.SaveToFlash(), SettingsStatus::kStorageError);
  zexpect_equal(ctx.backend.Count(), 1U);
  zexpect_equal(operations.UnsavedCount(), 2U);

  ctx.backend.FailWritesAfter(-1);
  zexpect_equal(operations.SaveToFlash(), SettingsStatus::kSuccess);
  zexpect_equal(ctx.backend.Count(), 3U);
  zexpect_equal(ctx.backend.Writes(), 4U);  // 1 ok + 1 failed + 2 retried
}

ZTEST(settings_store, test_undecodable_entry_falls_back_to_factory_and_a_save_removes_it) {
  Fixture ctx;
  const std::array<std::byte, 3> kBadSize{};
  ctx.backend.Put(reg::kMotorIMax, kBadSize);

  zexpect_equal(ctx.store.Load(), SettingsStatus::kInvalidImage);
  zexpect_equal(ctx.store.GetSnapshot().Get().motor_i_max, 30.0_A);

  zexpect_equal(ctx.store.GetOperations().SaveToFlash(), SettingsStatus::kSuccess);
  zexpect_false(ctx.backend.Contains(reg::kMotorIMax));
  zexpect_equal(ctx.store.Load(), SettingsStatus::kFactoryDefaults);
}

ZTEST(settings_store, test_stored_value_beyond_the_capability_is_rejected) {
  Fixture ctx;
  std::size_t size = 0U;
  const auto kBytes = Bytes(40.0_A, size);
  ctx.backend.Put(reg::kMotorIMax, std::span<const std::byte>{kBytes.data(), size});

  zexpect_equal(ctx.store.Load(), SettingsStatus::kOutOfRange);
  zexpect_equal(ctx.store.GetSnapshot().Get().motor_i_max, 30.0_A);
}

ZTEST(settings_store, test_unknown_keys_are_ignored) {
  Fixture ctx;
  const std::array<std::byte, 1> kValue{std::byte{1}};
  ctx.backend.Put("unimoc.removed.in.a.later.release", kValue);
  zexpect_equal(ctx.store.Load(), SettingsStatus::kFactoryDefaults);
}

ZTEST(settings_store, test_unavailable_storage_is_reported) {
  Fixture ctx;
  ctx.backend.SetUnavailable(true);
  zexpect_equal(ctx.store.Load(), SettingsStatus::kStorageUnavailable);
  zexpect_equal(ctx.store.GetSnapshot().Get().motor_i_max, 30.0_A);
}

ZTEST(settings_store, test_empty_node_name_is_stored_not_deleted) {
  Fixture ctx;
  zassert_equal(ctx.store.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = ctx.store.GetOperations();
  zassert_equal(operations.SetNodeName(""), SettingsStatus::kSuccess);
  zassert_equal(operations.SaveToFlash(), SettingsStatus::kSuccess);
  zexpect_equal(ctx.backend.Value(reg::kNodeDescription).size(), 1U);  // the terminator

  SettingsStore reloaded{MakeProfile(), ctx.backend.Adapter(), MakeGate()};
  zexpect_equal(reloaded.Load(), SettingsStatus::kSuccess);
  zexpect_equal(reloaded.GetSnapshot().Get().identity.GetName().size(), 0U);
}

// Records how the store uses the gate and the lock.
struct Probe {
  int lock_depth{0};
  int max_lock_depth{0};
  int locks{0};
  int gate_begins{0};
  int gate_ends{0};
  bool gate_held{false};
  bool gate_taken_under_lock{true};
  bool gate_allowed{true};
};

Probe g_probe{};

SettingsSaveGate MakeProbeGate() {
  return SettingsSaveGate{nullptr,
                          [](void* /*unused*/) {
                            ++g_probe.gate_begins;
                            g_probe.gate_taken_under_lock =
                                g_probe.gate_taken_under_lock && g_probe.lock_depth == 1;
                            g_probe.gate_held = g_probe.gate_allowed;
                            return g_probe.gate_allowed;
                          },
                          [](void* /*unused*/) {
                            ++g_probe.gate_ends;
                            g_probe.gate_held = false;
                          }};
}

SettingsLock MakeProbeLock() {
  return SettingsLock{nullptr,
                      [](void* /*unused*/) {
                        ++g_probe.locks;
                        ++g_probe.lock_depth;
                        g_probe.max_lock_depth = std::max(g_probe.max_lock_depth, g_probe.lock_depth);
                      },
                      [](void* /*unused*/) { --g_probe.lock_depth; }};
}

ZTEST(settings_store, test_gate_is_held_once_for_the_whole_save) {
  g_probe = Probe{};
  RamSettingsStorage backend{};
  SettingsStore store{MakeProfile(), backend.Adapter(), MakeProbeGate(), MakeProbeLock()};
  zassert_equal(store.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = store.GetOperations();

  // Nothing to write: the gate is not even asked.
  zexpect_equal(operations.SaveToFlash(), SettingsStatus::kSuccess);
  zexpect_equal(g_probe.gate_begins, 0);

  zassert_equal(operations.SetNodeId(9U), SettingsStatus::kSuccess);
  zassert_equal(operations.SetMotorCurrentLimit(20.0_A), SettingsStatus::kSuccess);
  zassert_equal(operations.SetNodeName("probe"), SettingsStatus::kSuccess);
  zexpect_equal(operations.SaveToFlash(), SettingsStatus::kSuccess);
  zexpect_equal(backend.Writes(), 3U);
  zexpect_equal(g_probe.gate_begins, 1);
  zexpect_equal(g_probe.gate_ends, 1);
  zexpect_false(g_probe.gate_held);
  zexpect_true(g_probe.gate_taken_under_lock);
}

ZTEST(settings_store, test_refused_gate_is_not_released) {
  g_probe = Probe{};
  RamSettingsStorage backend{};
  SettingsStore store{MakeProfile(), backend.Adapter(), MakeProbeGate(), MakeProbeLock()};
  zassert_equal(store.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = store.GetOperations();
  zassert_equal(operations.SetNodeId(9U), SettingsStatus::kSuccess);

  g_probe.gate_allowed = false;
  zexpect_equal(operations.SaveToFlash(), SettingsStatus::kPersistBlocked);
  zexpect_equal(g_probe.gate_begins, 1);
  zexpect_equal(g_probe.gate_ends, 0);
  zexpect_equal(backend.Writes(), 0U);
}

ZTEST(settings_store, test_every_operation_runs_under_the_lock) {
  g_probe = Probe{};
  RamSettingsStorage backend{};
  SettingsStore store{MakeProfile(), backend.Adapter(), MakeProbeGate(), MakeProbeLock()};
  auto operations = store.GetOperations();

  int expected = 0;
  const auto kCheck = [&expected]() {
    ++expected;
    zexpect_equal(g_probe.locks, expected);
    zexpect_equal(g_probe.lock_depth, 0);
  };
  zassert_equal(store.Load(), SettingsStatus::kFactoryDefaults);
  kCheck();
  (void)store.GetSnapshot();
  kCheck();
  zassert_equal(operations.Update([](NvmSettings& settings) {
    // The operation itself runs under the lock.
    zexpect_equal(g_probe.lock_depth, 1);
    settings.node_id = 4U;
  }),
                SettingsStatus::kSuccess);
  kCheck();
  zassert_equal(operations.SetNodeId(5U), SettingsStatus::kSuccess);
  kCheck();
  zexpect_equal(operations.UnsavedCount(), 1U);
  kCheck();
  zexpect_equal(operations.SaveToFlash(), SettingsStatus::kSuccess);
  kCheck();
  zassert_equal(operations.ResetToFactoryDefaults(), SettingsStatus::kSuccess);
  kCheck();
  zexpect_equal(g_probe.max_lock_depth, 1);  // never taken twice (the lock need not be recursive)
}

}  // namespace unimoc::settings::test

ZTEST_SUITE(settings_store, nullptr, nullptr, nullptr, nullptr, nullptr);
