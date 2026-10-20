/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file main.cpp
 *    @brief SettingsStore on the Zephyr settings subsystem and ZMS (flash simulator) with the save gate.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <zephyr/ztest.h>
#include <array>
#include <cstddef>
#include <span>
#include <string_view>
#include "hardware_interface.hpp"
#include "hot_path_interface.hpp"
#include "power_stage_interface.hpp"
#include "settings_registers.hpp"
#include "settings_save_gate.hpp"
#include "settings_store.hpp"
#include "sim_control.hpp"
#include "zephyr_settings_storage.hpp"

namespace {

using namespace unimoc::unit;
using unimoc::hardware::kPowerStage;
using unimoc::hardware::PowerStageState;
using unimoc::hardware::ZephyrSettingsStorage;
using unimoc::settings::NvmSettings;
using unimoc::settings::SettingsStatus;
using unimoc::settings::SettingsStore;
namespace reg = unimoc::cyphal::reg;
namespace sim = unimoc::hardware::sim;

SettingsStore MakeStore() {
  return SettingsStore{unimoc::hardware::kSettings.GetSettingsProfile(),
                       ZephyrSettingsStorage::Adapter(),
                       unimoc::settings::PowerStageSaveGate(kPowerStage),
                       unimoc::hardware::kSettings.GetSettingsLock()};
}

// Removes every register from the storage: each test starts from an empty flash.
void EraseAllRegisters() {
  const auto kStorage = ZephyrSettingsStorage::Adapter();
  const NvmSettings kSettings{};
  unimoc::settings::ForEachRegister(
      [&](std::string_view name, const auto& /*field*/) {
        zassert_equal(kStorage.Erase(name), unimoc::settings::SettingsStorageStatus::kSuccess);
      },
      kSettings);
}

std::size_t StoredEntries() {
  std::size_t count = 0U;
  zassert_equal(ZephyrSettingsStorage::Adapter().LoadAll(
                    [&](std::string_view /*key*/, std::span<const std::byte> /*value*/) { ++count; }),
                unimoc::settings::SettingsStorageStatus::kSuccess);
  return count;
}

void Before(void* /*unused*/) {
  sim::SetFreeRunning(false);
  sim::SetExternalEnable(true);
  kPowerStage.Disable();
  kPowerStage.ClearFault();
  EraseAllRegisters();
}

}  // namespace

// The platform calls these directly (hot_path_interface.hpp); the cycle never runs here.
void unimoc::hardware::ControlCycleHandler() noexcept {}
void unimoc::hardware::ObserverHandler() noexcept {}

ZTEST(sim_settings_persistence, test_empty_flash_loads_the_factory_profile) {
  auto store = MakeStore();
  zexpect_equal(store.Load(), SettingsStatus::kFactoryDefaults);
  zexpect_equal(StoredEntries(), 0U);
}

ZTEST(sim_settings_persistence, test_saved_registers_survive_a_new_store) {
  auto first = MakeStore();
  zassert_equal(first.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = first.GetOperations();
  zassert_equal(operations.Update([](NvmSettings& settings) {
    settings.pole_pairs = 7U;
    settings.stator_r = 0.07_Ohm;
    settings.node_id = 42U;
  }),
                SettingsStatus::kSuccess);
  zassert_equal(operations.SetNodeName("zms.node"), SettingsStatus::kSuccess);
  zexpect_equal(StoredEntries(), 0U);  // live tuning: nothing written yet

  zexpect_equal(operations.SaveToFlash(), SettingsStatus::kSuccess);
  zexpect_equal(StoredEntries(), 4U);

  auto second = MakeStore();
  zexpect_equal(second.Load(), SettingsStatus::kSuccess);
  const auto kSnapshot = second.GetSnapshot();
  const NvmSettings& loaded = kSnapshot.Get();
  zexpect_equal(loaded.pole_pairs, 7U);
  zexpect_equal(loaded.node_id, 42U);
  zexpect_equal(loaded.stator_r.Value(), 0.07F);
  zexpect_equal(loaded.identity.GetName(), std::string_view{"zms.node"});
}

ZTEST(sim_settings_persistence, test_saving_needs_the_power_stage_disabled) {
  auto store = MakeStore();
  zassert_equal(store.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = store.GetOperations();
  zassert_equal(operations.SetNodeId(5U), SettingsStatus::kSuccess);

  zassert_true(kPowerStage.Enable());
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kEnabled);
  zexpect_equal(operations.SaveToFlash(), SettingsStatus::kPersistBlocked);
  zexpect_equal(StoredEntries(), 0U);
  zexpect_equal(operations.UnsavedCount(), 1U);

  kPowerStage.Disable();
  zexpect_equal(operations.SaveToFlash(), SettingsStatus::kSuccess);
  zexpect_equal(StoredEntries(), 1U);

  // A latched fault is not "disabled" either.
  zassert_equal(operations.SetNodeId(6U), SettingsStatus::kSuccess);
  kPowerStage.Trip();
  zexpect_equal(operations.SaveToFlash(), SettingsStatus::kPersistBlocked);
  kPowerStage.ClearFault();
  zexpect_equal(operations.SaveToFlash(), SettingsStatus::kSuccess);
}

ZTEST(sim_settings_persistence, test_enable_is_held_off_while_a_save_runs) {
  const auto kGate = unimoc::settings::PowerStageSaveGate(kPowerStage);
  zassert_equal(kPowerStage.GetState(), PowerStageState::kDisabled);

  zassert_true(kGate.TryBegin());
  zexpect_false(kGate.TryBegin());  // one save at a time
  zexpect_false(kPowerStage.Enable());
  zexpect_equal(kPowerStage.GetState(), PowerStageState::kDisabled);
  kGate.End();

  zassert_true(kPowerStage.Enable());
  zexpect_false(kGate.TryBegin());  // enabled: refused, and Enable() is not held off
  kPowerStage.Disable();
  zassert_true(kPowerStage.Enable());
  kPowerStage.Disable();
}

ZTEST(sim_settings_persistence, test_register_back_at_factory_value_leaves_the_flash) {
  auto store = MakeStore();
  zassert_equal(store.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = store.GetOperations();
  zassert_equal(operations.SetNodeId(5U), SettingsStatus::kSuccess);
  zassert_equal(operations.SaveToFlash(), SettingsStatus::kSuccess);
  zassert_equal(StoredEntries(), 1U);

  zassert_equal(operations.SetNodeId(
                    unimoc::hardware::kSettings.GetSettingsProfile().factory_settings.node_id),
                SettingsStatus::kSuccess);
  zexpect_equal(operations.SaveToFlash(), SettingsStatus::kSuccess);
  zexpect_equal(StoredEntries(), 0U);
}

ZTEST(sim_settings_persistence, test_empty_node_name_survives) {
  auto first = MakeStore();
  zassert_equal(first.Load(), SettingsStatus::kFactoryDefaults);
  zassert_equal(first.GetOperations().SetNodeName(""), SettingsStatus::kSuccess);
  zassert_equal(first.GetOperations().SaveToFlash(), SettingsStatus::kSuccess);

  auto second = MakeStore();
  zexpect_equal(second.Load(), SettingsStatus::kSuccess);
  zexpect_equal(second.GetSnapshot().Get().identity.GetName().size(), 0U);
}

ZTEST(sim_settings_persistence, test_undecodable_entry_is_reported_and_healed) {
  const std::array<std::byte, 3> kBadSize{};
  zassert_equal(ZephyrSettingsStorage::Adapter().Write(reg::kMotorIMax, kBadSize),
                unimoc::settings::SettingsStorageStatus::kSuccess);

  auto store = MakeStore();
  zexpect_equal(store.Load(), SettingsStatus::kInvalidImage);
  zexpect_equal(store.GetSnapshot().Get().motor_i_max.Value(),
                unimoc::hardware::kSettings.GetSettingsProfile().factory_settings.motor_i_max.Value());

  zexpect_equal(store.GetOperations().SaveToFlash(), SettingsStatus::kSuccess);
  zexpect_equal(StoredEntries(), 0U);
  auto reloaded = MakeStore();
  zexpect_equal(reloaded.Load(), SettingsStatus::kFactoryDefaults);
}

ZTEST(sim_settings_persistence, test_all_registers_fit_and_survive_rewrites) {
  const auto kStorage = ZephyrSettingsStorage::Adapter();
  const NvmSettings kSettings{};
  // Every register stored, rewritten many times: exercises the garbage collection of the partition.
  for (unsigned round = 0U; round < 40U; ++round) {
    unimoc::settings::ForEachRegister(
        [&](std::string_view name, const auto& field) {
          std::array<std::byte, unimoc::settings::kMaxRegisterValueSize> bytes{};
          const std::size_t kSize = unimoc::settings::EncodeRegisterValue(field, bytes);
          bytes[0] = static_cast<std::byte>(round);  // changes the value every round
          zassert_equal(kStorage.Write(name, std::span<const std::byte>{bytes.data(), kSize}),
                        unimoc::settings::SettingsStorageStatus::kSuccess,
                        "%s round %u",
                        name.data(),
                        round);
        },
        kSettings);
  }
  zexpect_equal(StoredEntries(), 71U);
}

ZTEST_SUITE(sim_settings_persistence, nullptr, nullptr, Before, nullptr, nullptr);
