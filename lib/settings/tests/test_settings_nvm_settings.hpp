/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file test_settings_nvm_settings.hpp
 *    @brief GoogleTest cases for typed persistent settings.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <gtest/gtest.h>
#include "nvm_settings.hpp"

namespace unimoc {
namespace settings {
namespace test {

using namespace unit;

class NvmSettingsTest : public ::testing::Test {};

// --- Default construction is valid
TEST_F(NvmSettingsTest, DefaultIsValid) {
  NvmSettings settings;
  EXPECT_TRUE(settings.IsValid());
}

TEST_F(NvmSettingsTest, DefaultMagicAndVersion) {
  NvmSettings settings;
  EXPECT_EQ(settings.magic, kNvmMagic);
  EXPECT_EQ(settings.version, kNvmVersion);
}

// --- Default node_id == 0 triggers PnP
TEST_F(NvmSettingsTest, DefaultNodeIdZero) {
  NvmSettings settings;
  EXPECT_EQ(settings.node_id, 0U);
}

// --- Default motor type is PMSM
TEST_F(NvmSettingsTest, DefaultMotorType) {
  NvmSettings settings;
  EXPECT_EQ(settings.motor_type, system::MotorType::kPmsm);
}

// --- Default control mode is TORQUE
TEST_F(NvmSettingsTest, DefaultControlMode) {
  NvmSettings settings;
  EXPECT_EQ(settings.control_mode, cyphal::ControlMode::kTorque);
}

// --- Corrupt magic invalidates the block
TEST_F(NvmSettingsTest, CorruptMagicInvalid) {
  NvmSettings settings;
  settings.magic = 0xDEADBEEFU;
  EXPECT_FALSE(settings.IsValid());
}

// --- Corrupt version invalidates the block
TEST_F(NvmSettingsTest, CorruptVersionInvalid) {
  NvmSettings settings;
  settings.version = 0xFFFFU;
  EXPECT_FALSE(settings.IsValid());
}

// --- reset_to_defaults restores valid state
TEST_F(NvmSettingsTest, ResetToDefaultsIsValid) {
  NvmSettings settings;
  settings.magic = 0U;
  settings.version = 0U;
  settings.ResetToDefaults();
  EXPECT_TRUE(settings.IsValid());
}

// --- Identity defaults are preserved
TEST_F(NvmSettingsTest, IdentityDefaultName) {
  NvmSettings settings;
  EXPECT_EQ(settings.identity.get_name(), std::string_view("unimoc"));
}

// --- Node ID can be updated to valid range
TEST_F(NvmSettingsTest, NodeIdCanBeSet) {
  NvmSettings settings;
  settings.node_id = 42U;
  EXPECT_EQ(settings.node_id, 42U);
  EXPECT_TRUE(settings.IsValid());
}

// --- Parameter spot-checks (verify fields exist + have sensible defaults)
TEST_F(NvmSettingsTest, DefaultStatorR) {
  NvmSettings settings;
  EXPECT_FLOAT_EQ(settings.stator_r.Value(), 0.1F);
}

TEST_F(NvmSettingsTest, DefaultFwVMax) {
  NvmSettings settings;
  EXPECT_FLOAT_EQ(settings.fw_v_max.Value(), 0.9F);
}

TEST_F(NvmSettingsTest, DefaultSvmDutyRange) {
  NvmSettings settings;
  EXPECT_LT(settings.svm_duty_min, settings.svm_duty_max);
}

TEST_F(NvmSettingsTest, DefaultPosSpeedLimit) {
  NvmSettings settings;
  EXPECT_GT(settings.pos_speed_limit.Value(), 0.0F);
}

TEST_F(NvmSettingsTest, DefaultExcitationMode) {
  NvmSettings settings;
  EXPECT_EQ(settings.excitation_mode, 0U);  // CurrentMode
}

// --- Phase current balance defaults ---

TEST_F(NvmSettingsTest, DefaultPhaseBalanceA) {
  NvmSettings settings;
  EXPECT_FLOAT_EQ(settings.phase_balance_a.Value(), 1.0F);
}

TEST_F(NvmSettingsTest, DefaultPhaseBalanceB) {
  NvmSettings settings;
  EXPECT_FLOAT_EQ(settings.phase_balance_b.Value(), 1.0F);
}

TEST_F(NvmSettingsTest, DefaultPhaseBalanceC) {
  NvmSettings settings;
  EXPECT_FLOAT_EQ(settings.phase_balance_c.Value(), 1.0F);
}

TEST_F(NvmSettingsTest, PhaseBalanceCanBeModified) {
  NvmSettings settings;
  settings.phase_balance_a = 1.02_ratio;
  settings.phase_balance_b = 0.98_ratio;
  settings.phase_balance_c = 1.00_ratio;
  EXPECT_FLOAT_EQ(settings.phase_balance_a.Value(), 1.02F);
  EXPECT_FLOAT_EQ(settings.phase_balance_b.Value(), 0.98F);
  EXPECT_FLOAT_EQ(settings.phase_balance_c.Value(), 1.00F);
  EXPECT_TRUE(settings.IsValid());
}

TEST_F(NvmSettingsTest, ResetRestoresPhaseBalanceDefaults) {
  NvmSettings settings;
  settings.phase_balance_a = 1.05_ratio;
  settings.phase_balance_b = 0.95_ratio;
  settings.phase_balance_c = 0.99_ratio;
  settings.ResetToDefaults();
  EXPECT_FLOAT_EQ(settings.phase_balance_a.Value(), 1.0F);
  EXPECT_FLOAT_EQ(settings.phase_balance_b.Value(), 1.0F);
  EXPECT_FLOAT_EQ(settings.phase_balance_c.Value(), 1.0F);
}

// --- NVM version is 2 after layout change ---

TEST_F(NvmSettingsTest, NvmVersionIsTwo) {
  EXPECT_EQ(kNvmVersion, 3U);
  NvmSettings settings;
  EXPECT_EQ(settings.version, 3U);
}

}  // namespace test
}  // namespace settings
}  // namespace unimoc
