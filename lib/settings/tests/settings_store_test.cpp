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
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "settings_store.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstring>
#include <ranges>
#include <type_traits>
#include <utility>
#include "settings_codec.hpp"

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

TEST(SettingsStoreTest, SnapshotExposesOnlyConstSettings) {
  static_assert(std::is_same_v<decltype(std::declval<const SettingsSnapshot&>().Get()), const NvmSettings&>);
}

TEST(SettingsStoreTest, MissingImageUsesAndPersistsFactoryDefaults) {
  MemoryBackend backend{};
  SettingsStore store{MakeProfile(), MakeStorage(backend)};

  EXPECT_EQ(store.Load(), SettingsStatus::kFactoryDefaults);
  EXPECT_TRUE(backend.has_image);
  EXPECT_EQ(store.GetSnapshot().Get().motor_i_max.Value(), 30.0F);
}

TEST(SettingsStoreTest, CodecRoundTripPreservesTypedValues) {
  NvmSettings original{};
  original.stator_r = 0.23_Ohm;
  original.l_d = 1.2_mH;
  original.motor_j = 0.00042_kg_m2;
  original.pwm_frequency = 32.0_kHz;

  std::array<std::byte, kSettingsImageSize> image{};
  NvmSettings decoded{};

  ASSERT_TRUE(SettingsCodec::Encode(original, image));
  ASSERT_TRUE(SettingsCodec::Decode(image, decoded));
  EXPECT_EQ(decoded.stator_r.Value(), original.stator_r.Value());
  EXPECT_EQ(decoded.l_d.Value(), original.l_d.Value());
  EXPECT_EQ(decoded.motor_j.Value(), original.motor_j.Value());
  EXPECT_EQ(decoded.pwm_frequency, original.pwm_frequency);
}

TEST(SettingsStoreTest, AuthorizedOperationUpdatesAndPersists) {
  MemoryBackend backend{};
  SettingsStore store{MakeProfile(), MakeStorage(backend)};
  ASSERT_EQ(store.Load(), SettingsStatus::kFactoryDefaults);

  auto operations = store.GetOperations();
  EXPECT_EQ(operations.SetMotorCurrentLimit(20.0_A), SettingsStatus::kSuccess);
  EXPECT_EQ(store.GetSnapshot().Get().motor_i_max.Value(), 20.0F);
  EXPECT_EQ(backend.save_count, 2U);
}

TEST(SettingsStoreTest, CapabilityViolationLeavesSnapshotUnchanged) {
  MemoryBackend backend{};
  SettingsStore store{MakeProfile(), MakeStorage(backend)};
  ASSERT_EQ(store.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = store.GetOperations();

  EXPECT_EQ(operations.SetMotorCurrentLimit(31.0_A), SettingsStatus::kOutOfRange);
  EXPECT_EQ(store.GetSnapshot().Get().motor_i_max.Value(), 30.0F);
  EXPECT_EQ(backend.save_count, 1U);
}

TEST(SettingsStoreTest, SaveFailureLeavesSnapshotUnchanged) {
  MemoryBackend backend{};
  SettingsStore store{MakeProfile(), MakeStorage(backend)};
  ASSERT_EQ(store.Load(), SettingsStatus::kFactoryDefaults);
  backend.fail_save = true;

  auto operations = store.GetOperations();
  EXPECT_EQ(operations.SetMotorCurrentLimit(20.0_A), SettingsStatus::kStorageError);
  EXPECT_EQ(store.GetSnapshot().Get().motor_i_max.Value(), 30.0F);
}

TEST(SettingsStoreTest, ResetRestoresFactoryProfile) {
  MemoryBackend backend{};
  SettingsStore store{MakeProfile(), MakeStorage(backend)};
  ASSERT_EQ(store.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = store.GetOperations();
  ASSERT_EQ(operations.SetMotorCurrentLimit(20.0_A), SettingsStatus::kSuccess);

  EXPECT_EQ(operations.ResetToFactoryDefaults(), SettingsStatus::kSuccess);
  EXPECT_EQ(store.GetSnapshot().Get().motor_i_max.Value(), 30.0F);
}

}  // namespace unimoc::settings::test
