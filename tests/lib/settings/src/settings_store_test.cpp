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
#include <cstring>
#include <ranges>
#include <type_traits>
#include <utility>
#include "settings_codec.hpp"
#include "unimoc_test.hpp"

namespace unimoc::settings::test {

using namespace unimoc::unit;

namespace {

struct MemoryBackend {
  std::array<std::byte, kSettingsImageSize> image{};
  bool has_image{false};
  bool fail_save{false};
  uint32_t save_count{0U};
};

SettingsStorageStatus LoadImage(void* context, std::span<std::byte> image) noexcept {
  auto& backend = *static_cast<MemoryBackend*>(context);
  if (!backend.has_image) {
    return SettingsStorageStatus::kNotFound;
  }
  std::ranges::copy(backend.image, image.begin());
  return SettingsStorageStatus::kSuccess;
}

SettingsStorageStatus SaveImage(void* context, std::span<const std::byte> image) noexcept {
  auto& backend = *static_cast<MemoryBackend*>(context);
  ++backend.save_count;
  if (backend.fail_save) {
    return SettingsStorageStatus::kIoError;
  }
  std::ranges::copy(image, backend.image.begin());
  backend.has_image = true;
  return SettingsStorageStatus::kSuccess;
}

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

SettingsStorage MakeStorage(MemoryBackend& backend) {
  return SettingsStorage{&backend, LoadImage, SaveImage};
}

}  // namespace

ZTEST(settings_store, test_snapshot_exposes_only_const_settings) {
  static_assert(std::is_same_v<decltype(std::declval<const SettingsSnapshot&>().Get()), const NvmSettings&>);
}

ZTEST(settings_store, test_missing_image_uses_and_persists_factory_defaults) {
  MemoryBackend backend{};
  SettingsStore store{MakeProfile(), MakeStorage(backend)};

  zexpect_equal(store.Load(), SettingsStatus::kFactoryDefaults);
  zexpect_true(backend.has_image);
  zexpect_equal(store.GetSnapshot().Get().motor_i_max.Value(), 30.0F);
}

ZTEST(settings_store, test_codec_round_trip_preserves_typed_values) {
  NvmSettings original{};
  original.stator_r = 0.23_Ohm;
  original.l_d = 1.2_mH;
  original.motor_j = 0.00042_kg_m2;
  original.pwm_frequency = 32.0_kHz;

  std::array<std::byte, kSettingsImageSize> image{};
  NvmSettings decoded{};

  zassert_true(SettingsCodec::Encode(original, image));
  zassert_true(SettingsCodec::Decode(image, decoded));
  zexpect_equal(decoded.stator_r.Value(), original.stator_r.Value());
  zexpect_equal(decoded.l_d.Value(), original.l_d.Value());
  zexpect_equal(decoded.motor_j.Value(), original.motor_j.Value());
  zexpect_equal(decoded.pwm_frequency, original.pwm_frequency);
}

ZTEST(settings_store, test_authorized_operation_updates_and_persists) {
  MemoryBackend backend{};
  SettingsStore store{MakeProfile(), MakeStorage(backend)};
  zassert_equal(store.Load(), SettingsStatus::kFactoryDefaults);

  auto operations = store.GetOperations();
  zexpect_equal(operations.SetMotorCurrentLimit(20.0_A), SettingsStatus::kSuccess);
  zexpect_equal(store.GetSnapshot().Get().motor_i_max.Value(), 20.0F);
  zexpect_equal(backend.save_count, 2U);
}

ZTEST(settings_store, test_capability_violation_leaves_snapshot_unchanged) {
  MemoryBackend backend{};
  SettingsStore store{MakeProfile(), MakeStorage(backend)};
  zassert_equal(store.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = store.GetOperations();

  zexpect_equal(operations.SetMotorCurrentLimit(31.0_A), SettingsStatus::kOutOfRange);
  zexpect_equal(store.GetSnapshot().Get().motor_i_max.Value(), 30.0F);
  zexpect_equal(backend.save_count, 1U);
}

ZTEST(settings_store, test_save_failure_leaves_snapshot_unchanged) {
  MemoryBackend backend{};
  SettingsStore store{MakeProfile(), MakeStorage(backend)};
  zassert_equal(store.Load(), SettingsStatus::kFactoryDefaults);
  backend.fail_save = true;

  auto operations = store.GetOperations();
  zexpect_equal(operations.SetMotorCurrentLimit(20.0_A), SettingsStatus::kStorageError);
  zexpect_equal(store.GetSnapshot().Get().motor_i_max.Value(), 30.0F);
}

ZTEST(settings_store, test_reset_restores_factory_profile) {
  MemoryBackend backend{};
  SettingsStore store{MakeProfile(), MakeStorage(backend)};
  zassert_equal(store.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = store.GetOperations();
  zassert_equal(operations.SetMotorCurrentLimit(20.0_A), SettingsStatus::kSuccess);

  zexpect_equal(operations.ResetToFactoryDefaults(), SettingsStatus::kSuccess);
  zexpect_equal(store.GetSnapshot().Get().motor_i_max.Value(), 30.0F);
}

}  // namespace unimoc::settings::test

ZTEST_SUITE(settings_store, nullptr, nullptr, nullptr, nullptr, nullptr);
